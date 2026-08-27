"""KiCad 10 SWIG ActionPlugin entry point for selection-seeded track gloss."""

from __future__ import annotations

import logging
import os
import threading
import time
import traceback

import pcbnew
import wx

from .configuration import get_session_config
from .engine import (find_pad_terminal_targets, find_track_terminal_vertices,
                     generate_conservative_candidate,
                     generate_converged_plan, plan_identity,
                     plan_net_gain, plan_net_ids, subset_plan_by_nets,
                     summarize_plan)
from .engine.model import segment_key
from .kicad import BoardAdapter
from .kicad.diagnostics import append_plan_statistics, append_search_statistics
from .kicad.report_dialog import (show_diagnostic_report as _show_diagnostic_report,
                                  show_report as _show_report,
                                  warning_bell as _warning_bell)
from .kicad.settings_dialog import show_session_settings
from .kicad.types import is_arc, is_straight_track, is_via
from .version import __version__


PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
LOG = logging.getLogger("KiCadTrackGloss")

# One-click policy: no preview or success/no-op popup. Session settings are
# intentionally reachable only by invoking either action with no track seed.
# Native KiCad DRC validation is intentionally retained as a safety gate and
# can dominate response time even for a single selected connection.
ALLOW_EQUAL_LENGTH_SIMPLIFICATION = True
# Interactive work must remain responsive. Independent net/layer groups are
# converged inside parallel workers; these passes are only global reconciliation
# rounds. A safe partial result is applied if this guard is reached.
BUSY_CURSOR_DELAY_SECONDS = 3.0
BUSY_CURSOR_POLL_SECONDS = 0.05


class NoTrackSelection(ValueError):
    """Normal user condition which opens process-local session settings."""


def _show_session_settings():
    """Indirection kept small so the no-selection path is easy to test."""
    return show_session_settings()


def _busy_cursor_controller(
        operation_started, delay_seconds=BUSY_CURSOR_DELAY_SECONDS):
    """Return polling/cleanup callbacks for one delayed non-modal cursor."""
    busy_started = False

    def wait_callback():
        nonlocal busy_started
        if (not busy_started and
                time.monotonic() - operation_started >= delay_seconds):
            try:
                wx.BeginBusyCursor()
                busy_started = True
            except Exception:
                LOG.exception("Could not display the Track Gloss busy cursor")
        try:
            if hasattr(wx, "YieldIfNeeded"):
                wx.YieldIfNeeded()
        except Exception:
            LOG.exception("Could not refresh the Track Gloss busy cursor")

    def close():
        if busy_started:
            try:
                wx.EndBusyCursor()
            except Exception:
                LOG.exception("Could not restore the Track Gloss cursor")

    return wait_callback, close


def _run_with_delayed_busy_cursor(
        function, operation_started=None,
        delay_seconds=BUSY_CURSOR_DELAY_SECONDS):
    """Run API-neutral work off-thread and show only a delayed wait cursor."""
    operation_started = (time.monotonic() if operation_started is None else
                         operation_started)
    completed = threading.Event()
    outcome = {}
    wait_callback, close = _busy_cursor_controller(
        operation_started, delay_seconds)

    def worker():
        try:
            outcome["result"] = function()
        except BaseException as error:
            outcome["error"] = error
            outcome["traceback"] = error.__traceback__
        finally:
            completed.set()

    thread = threading.Thread(
        target=worker, name="KiCadTrackGlossPlanner", daemon=True)
    thread.start()
    try:
        while not completed.wait(BUSY_CURSOR_POLL_SECONDS):
            wait_callback()
        if "error" in outcome:
            raise outcome["error"].with_traceback(outcome["traceback"])
        return outcome["result"]
    finally:
        close()


def _selection_counts(board):
    counts = {"segments": 0, "arcs": 0, "vias": 0, "other": 0}
    for item in board.GetTracks():
        try:
            if not item.IsSelected():
                continue
        except Exception:
            continue
        if is_straight_track(pcbnew, item):
            counts["segments"] += 1
        elif is_arc(pcbnew, item):
            counts["arcs"] += 1
        elif is_via(pcbnew, item):
            counts["vias"] += 1
        else:
            counts["other"] += 1

    other_collections = []
    for alternatives in (("GetFootprints",), ("GetDrawings",),
                         ("Zones", "GetZones")):
        for accessor in alternatives:
            try:
                other_collections.append(getattr(board, accessor)())
                break
            except Exception:
                continue
    try:
        other_collections.extend(footprint.Pads()
                                 for footprint in board.GetFootprints())
    except Exception:
        pass
    for values in other_collections:
        for item in values:
            try:
                if item.IsSelected():
                    counts["other"] += 1
            except Exception:
                continue
    return counts


def _eligible_net_names(model, eligible_keys):
    eligible = set(eligible_keys)
    labels = {segment.net_name or "net {}".format(segment.net_id)
              for segment in model.segments
              if segment_key(segment) in eligible}
    return sorted(labels)


def _append_performance_timings(report, timings, operation_started):
    timings["total"] = (time.monotonic() - operation_started) * 1000.0
    report.extend(["", "Performance timings:"] + [
        "  {}: {:.3f} ms".format(
            key.replace("_", " ").title(), value)
        for key, value in timings.items()
    ])


def _validate_with_delayed_busy_cursor(
        adapter, board, plan, *, force_native, skip_native, timeout_seconds,
        operation_started, delay_seconds=BUSY_CURSOR_DELAY_SECONDS):
    wait_callback, close = _busy_cursor_controller(
        operation_started, delay_seconds)

    try:
        # pcbnew SWIG snapshots must remain on KiCad's main thread. The native
        # validator runs expensive work in hidden subprocesses and invokes the
        # callback while waiting. This preserves UI responsiveness without
        # moving any board object to a secondary Python thread.
        return adapter.validate_plan(
            board, plan, force_native=force_native,
            skip_native=skip_native, timeout_seconds=timeout_seconds,
            wait_callback=wait_callback)
    finally:
        close()


def _validate_ladder_with_delayed_busy_cursor(
        adapter, board, plans, *, force_native, skip_native, timeout_seconds,
        operation_started, delay_seconds=BUSY_CURSOR_DELAY_SECONDS):
    wait_callback, close = _busy_cursor_controller(
        operation_started, delay_seconds)

    try:
        return adapter.validate_plan_ladder(
            board, plans, force_native=force_native,
            skip_native=skip_native, timeout_seconds=timeout_seconds,
            wait_callback=wait_callback)
    finally:
        close()


def _maximize_safe_native_subset(
        adapter, board, model, eligible_keys, source_plan, *, force_native,
        skip_native, operation_started, operation_deadline):
    """Greedily retain the largest gain-ordered DRC-safe net subset.

    Rejected chunks are bisected. Every accepted candidate becomes the new
    base immediately, so expiration returns useful work already approved by
    KiCad instead of reverting to a global no-op.
    """
    ranked_nets = tuple(sorted(
        plan_net_ids(model, source_plan),
        key=lambda net_id: (-round(
            plan_net_gain(model, source_plan, net_id), 9), net_id)))
    if len(ranked_nets) < 2:
        return None, None, 0, False

    base_nets = frozenset()
    best_plan = None
    best_native = None
    attempts = 0
    deadline_reached = False
    pending = [ranked_nets]
    # The caller has already validated and rejected the complete plan.
    rejected_sets = {frozenset(ranked_nets)}

    while pending:
        if time.monotonic() >= operation_deadline:
            deadline_reached = True
            break
        chunk = pending.pop(0)
        candidate_nets = base_nets.union(chunk)
        rejected = candidate_nets in rejected_sets
        infrastructure_error = False
        if not rejected:
            try:
                candidate = subset_plan_by_nets(
                    model, eligible_keys, source_plan, candidate_nets)
            except ValueError:
                rejected = True
            else:
                remaining = operation_deadline - time.monotonic()
                if remaining <= 0.0:
                    deadline_reached = True
                    break
                candidate_native = _validate_with_delayed_busy_cursor(
                    adapter, board, candidate, force_native=force_native,
                    skip_native=skip_native, timeout_seconds=remaining,
                    operation_started=operation_started)
                attempts += 1
                if candidate_native.allowed:
                    base_nets = candidate_nets
                    best_plan = candidate
                    best_native = candidate_native
                    continue
                rejected_sets.add(candidate_nets)
                rejected = True
                if candidate_native.error:
                    infrastructure_error = (
                        candidate_native.validation_mode != "native_timeout")
                    deadline_reached = not infrastructure_error
        if infrastructure_error or deadline_reached:
            break
        if rejected and len(chunk) > 1:
            midpoint = len(chunk) // 2
            # Gain-ranked half first. The second half remains available and
            # can be added later to whatever safe base was already retained.
            pending[0:0] = [chunk[:midpoint], chunk[midpoint:]]

    return best_plan, best_native, attempts, deadline_reached


class KiCadTrackGlossPlugin(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "KiCad Track Gloss"
        self.category = "Routing"
        self.description = ("Gloss one or more selected track segments, "
                            "connections, or complete nets")
        self.show_toolbar_button = True
        self.icon_file_name = os.path.join(PLUGIN_DIR, "icon_24.png")
        dark = os.path.join(PLUGIN_DIR, "icon_24_dark.png")
        if os.path.exists(dark):
            self.dark_icon_file_name = dark

    def Run(self):
        try:
            changed = self._run([])
        except NoTrackSelection:
            try:
                _show_session_settings()
            except Exception:
                LOG.exception("Could not display Track Gloss session settings")
        except Exception:
            _warning_bell()
            LOG.exception("Track gloss failed; the board was left unchanged")
            try:
                _show_report("KiCad Track Gloss — Error", [
                    "UNEXPECTED ERROR",
                    "Plugin version: " + __version__,
                    "The operation was aborted; in-memory rollback was requested.",
                    "",
                    traceback.format_exc(),
                ])
            except Exception:
                LOG.exception("Could not display the Track Gloss error report")
        else:
            if changed is False:
                _warning_bell()

    def _run(self, report, diagnostic=False):
        config = get_session_config()
        operation_started = time.monotonic()
        operation_deadline = (
            operation_started +
            config.timing.interactive_total_time_budget_seconds)
        timings = {}
        report.append("Plugin version: " + __version__)
        board = pcbnew.GetBoard()
        if board is None:
            report.append("Result: no active PCB board.")
            return False
        try:
            report.append("KiCad version: " + str(pcbnew.Version()))
        except Exception:
            pass
        report.append(
            "Optimization coordinates: exact copper geometry; active KiCad grid not used.")
        report.append(
            "Session policy: minimum saving {:.6f} mm; interactive passes {} "
            "(group {}); single-track KiCad DRC {}; time budget {:.1f} s "
            "(planning {:.1f} s).".format(
                config.gloss.minimum_saved_length_mm,
                config.convergence.interactive_max_passes,
                config.convergence.interactive_group_max_passes,
                "enabled" if config.safety.kicad_drc_for_single_track
                else "disabled",
                config.timing.interactive_total_time_budget_seconds,
                config.timing.interactive_planning_time_budget_seconds))
        stage_started = time.monotonic()
        counts = _selection_counts(board)
        timings["selection_scan"] = (
            time.monotonic() - stage_started) * 1000.0
        report.append(
            "Selected objects: {segments} straight segment(s), {arcs} arc(s), "
            "{vias} via(s), {other} other.".format(**counts))
        if counts["segments"] == 0:
            raise NoTrackSelection(
                "Select at least one straight track segment before running Track Gloss.")
        adapter = BoardAdapter(pcbnew)
        try:
            stage_started = time.monotonic()
            snapshot = adapter.snapshot(board)
            timings["snapshot"] = (
                time.monotonic() - stage_started) * 1000.0
        except ValueError as error:
            report.append("Result: selection rejected.")
            report.append("Reason: " + str(error))
            if diagnostic:
                _append_performance_timings(
                    report, timings, operation_started)
            return False
        report.append("Eligible straight segments: " + str(len(snapshot.eligible_keys)))
        net_names = _eligible_net_names(snapshot.model, snapshot.eligible_keys)
        report.append("Eligible net(s) ({}): {}".format(
            len(net_names), ", ".join(net_names) if net_names else "none"))
        report.append("Automatic connection expansion: {} seed(s) + {} segment(s).".format(
            snapshot.selection_seed_count, snapshot.auto_expanded_count))
        report.append("Protected tuned segments: " +
                      str(snapshot.tuned_protected_count))
        for warning in snapshot.warnings:
            report.append("Protection: " + warning)
        stage_started = time.monotonic()
        track_terminals = find_track_terminal_vertices(
            snapshot.model, snapshot.eligible_keys)
        pad_terminals = find_pad_terminal_targets(
            snapshot.model, snapshot.eligible_keys)
        timings["terminal_analysis"] = (
            time.monotonic() - stage_started) * 1000.0
        report.append("Sliding track-intersection terminations: " +
                      str(len(track_terminals)))
        report.append("Sliding pad-area terminations: " +
                      str(len(pad_terminals)))
        if (len(snapshot.eligible_keys) < 2 and not track_terminals and
                not pad_terminals):
            report.append("Result: no modification.")
            report.append(
                "Reason: automatic connection expansion did not find a second eligible "
                "straight segment or a sliding track/pad termination.")
            if diagnostic:
                _append_performance_timings(
                    report, timings, operation_started)
            return False

        stage_started = time.monotonic()
        planning_deadline = min(
            operation_deadline,
            stage_started +
            config.timing.interactive_planning_time_budget_seconds)
        conservative_ladder = []
        best = _run_with_delayed_busy_cursor(
            lambda: generate_converged_plan(
                snapshot.model, snapshot.eligible_keys,
                max_passes=config.convergence.interactive_max_passes,
                return_partial_on_limit=True,
                group_max_passes=(
                    config.convergence.interactive_group_max_passes),
                min_gain=config.gloss.minimum_saved_length_mm,
                allow_equal_length_simpler=(
                    ALLOW_EQUAL_LENGTH_SIMPLIFICATION),
                clearance=snapshot.minimum_clearance,
                collect_statistics=diagnostic,
                parallel=True,
                deadline=planning_deadline,
                conservative_ladder=conservative_ladder,
                cancellation_grace_seconds=(
                    config.timing.interactive_cancellation_grace_seconds)),
            operation_started=operation_started)
        timings["planning"] = (
            time.monotonic() - stage_started) * 1000.0
        report.append("Convergence passes: " + str(best.convergence_passes))
        report.append("Fixed point reached: " +
                      ("yes" if best.fixed_point else "no"))
        report.append("Connected chains considered: " +
                      str(best.chains_considered))
        aggressive_plan = best
        for warning in best.warnings:
            if "time budget" in warning.lower():
                report.append("Planning limit: " + warning)
        if not best.changed:
            if diagnostic:
                append_search_statistics(
                    report, best.search_counts, best.blocking_nets)
                _append_performance_timings(
                    report, timings, operation_started)
            if not best.fixed_point:
                report.append("Result: interactive planning time budget reached.")
                report.append(
                    "No fully composed improvement was available before the deadline; "
                    "the current board was left unchanged.")
            else:
                report.append("Result: no safe improvement found.")
                report.append(
                    "Possible reasons: disconnected selection, fixed junction, locked/tuned "
                    "track, insufficient length gain, clearance, pad, via, keepout, or board edge.")
            return False
        stage_started = time.monotonic()
        single_track_selection = snapshot.selection_seed_count == 1
        drc_budget = operation_deadline - time.monotonic()
        if drc_budget <= 0.0:
            report.append("Result: interactive total time budget reached.")
            report.append(
                "The candidate was not sent to KiCad DRC and the current board "
                "was left unchanged.")
            return False
        force_native = (
            single_track_selection and
            config.safety.kicad_drc_for_single_track)
        skip_native = (
            single_track_selection and
            not config.safety.kicad_drc_for_single_track)
        validation_plans = [best]
        if conservative_ladder:
            conservative = conservative_ladder[0]
            if (conservative.changed and
                    plan_identity(conservative) != plan_identity(best)):
                validation_plans.append(conservative)
        if len(validation_plans) > 1:
            native_results = _validate_ladder_with_delayed_busy_cursor(
                adapter, board, validation_plans, force_native=force_native,
                skip_native=skip_native, timeout_seconds=drc_budget,
                operation_started=operation_started)
            native = native_results[0]
        else:
            native_results = None
            native = _validate_with_delayed_busy_cursor(
                adapter, board, best, force_native=force_native,
                skip_native=skip_native, timeout_seconds=drc_budget,
                operation_started=operation_started)

        # Native DRC is the final authority. If the most aggressive refinement
        # is rejected, retain quality progressively: retry a one-pass,
        # unrefined candidate which often preserves a terminal that the deeper
        # local search moved too far. The fallback still receives the complete
        # native DRC gate and remains inside the same interactive time budget.
        fallback_used = False
        partial_subset_used = False
        partial_subset_attempts = 0
        partial_subset_deadline = False
        primary_native = native
        if native_results is not None and not native.allowed:
            fallback_native = native_results[1]
            if fallback_native.allowed:
                best = validation_plans[1]
                native = fallback_native
                fallback_used = True
            elif native.error:
                pass
            elif fallback_native.error:
                native = fallback_native
        if (native_results is None and not native.allowed and
                native.validation_mode != "native_timeout" and
                time.monotonic() < planning_deadline):
            try:
                conservative = _run_with_delayed_busy_cursor(
                    lambda:
                    generate_conservative_candidate(
                        snapshot.model, snapshot.eligible_keys,
                        min_gain=config.gloss.minimum_saved_length_mm,
                        clearance=snapshot.minimum_clearance,
                        collect_statistics=diagnostic,
                        deadline=planning_deadline,
                        cancellation_grace_seconds=(
                            config.timing.interactive_cancellation_grace_seconds)),
                    operation_started=operation_started)
            except Exception:
                LOG.exception("Could not build the conservative DRC fallback")
                conservative = None
            if (conservative is not None and conservative.changed and
                    plan_identity(conservative) != plan_identity(best)):
                remaining = operation_deadline - time.monotonic()
                if remaining > 0.0:
                    fallback_native = _validate_with_delayed_busy_cursor(
                        adapter, board, conservative,
                        force_native=force_native, skip_native=skip_native,
                        timeout_seconds=remaining,
                        operation_started=operation_started)
                    if fallback_native.allowed:
                        best = conservative
                        native = fallback_native
                        fallback_used = True
                    elif fallback_native.error:
                        native = fallback_native
        if (not native.allowed and not native.error and
                native.validation_mode != "native_timeout" and
                time.monotonic() < operation_deadline and
                len(plan_net_ids(snapshot.model, aggressive_plan)) > 1):
            (partial_plan, partial_native, partial_subset_attempts,
             partial_subset_deadline) = _maximize_safe_native_subset(
                adapter, board, snapshot.model, snapshot.eligible_keys,
                aggressive_plan, force_native=force_native,
                skip_native=skip_native,
                operation_started=operation_started,
                operation_deadline=operation_deadline)
            if (partial_plan is not None and partial_native is not None and
                    partial_native.allowed):
                best = partial_plan
                native = partial_native
                partial_subset_used = True
        timings["native_drc_gate"] = (
            time.monotonic() - stage_started) * 1000.0
        if fallback_used or partial_subset_used:
            for key, value in primary_native.timings_ms.items():
                timings["native_primary_" + key] = value
        for key, value in native.timings_ms.items():
            timings["native_" + key] = value
        if not native.allowed:
            if native.validation_mode == "native_timeout":
                report.append("Native KiCad DRC gate: interactive time budget reached.")
            elif native.error:
                report.append("Native KiCad DRC gate: validation infrastructure failed.")
            else:
                report.append("Native KiCad DRC gate: plan rejected.")
            if native.increases:
                report.append("New native DRC findings: " + ", ".join(
                    "{} +{}".format(key, value)
                    for key, value in native.increases.items()))
            if native.error:
                report.append("Native DRC error: " + native.error)
            if diagnostic:
                _append_performance_timings(
                    report, timings, operation_started)
            report.append("Result: no safe improvement found.")
            return False
        if fallback_used:
            report.append(
                "Candidate ladder: aggressive plan rejected; conservative "
                "candidate accepted by native KiCad DRC.")
        if partial_subset_used:
            retained_nets = plan_net_ids(snapshot.model, best)
            total_nets = plan_net_ids(snapshot.model, aggressive_plan)
            report.append(
                "Native DRC salvage: retained {} of {} modified net(s) after "
                "{} subset validation(s).".format(
                    len(retained_nets), len(total_nets),
                    partial_subset_attempts))
            omitted_ids = set(total_nets) - set(retained_nets)
            omitted_names = sorted({
                segment.net_name or "net {}".format(segment.net_id)
                for segment in snapshot.model.segments
                if segment.net_id in omitted_ids})
            if omitted_names:
                report.append(
                    "Not retained (rejected or unvalidated before the time "
                    "budget): " + ", ".join(omitted_names))
            if partial_subset_deadline:
                report.append(
                    "Native DRC salvage stopped at the interactive time "
                    "budget; the best already validated subset was retained.")
        if native.validation_mode == "geometric_removal_fast_path":
            report.append(
                "Safety gate: proven removal-only geometry; native DRC not required.")
        elif native.validation_mode == "single_track_drc_disabled":
            report.append(
                "Safety gate: single-track KiCad DRC disabled by internal policy.")
        else:
            report.append("Native KiCad DRC gate: no category increase.")
        report.append("Chosen plan: remove {} segment(s), add {} segment(s).".format(
            len(best.remove_keys), len(best.additions)))
        report.append("Copper length saved: {:.3f} mm.".format(best.saved_mm))
        report.append("Non-octolinear segments corrected: {}.".format(
            best.angle_corrections))
        stage_started = time.monotonic()
        adapter.apply(board, best, rollback_on_error=True)
        timings["apply"] = (time.monotonic() - stage_started) * 1000.0
        try:
            board.SetModified()
        except Exception:
            pass
        pcbnew.Refresh()
        timings["total"] = (
            time.monotonic() - operation_started) * 1000.0
        if diagnostic:
            summary = summarize_plan(
                snapshot.model, snapshot.eligible_keys, best)
            summary["timings_ms"] = timings
            summary["native_baseline_cached"] = native.baseline_cached
            summary["validation_mode"] = native.validation_mode
            append_plan_statistics(report, summary)
        return True


class KiCadTrackGlossDiagnosticPlugin(KiCadTrackGlossPlugin):
    def defaults(self):
        self.name = "KiCad Track Gloss — Diagnostic"
        self.category = "Routing"
        self.description = ("Run Track Gloss and display a detailed diagnostic "
                            "report, including no-op reasons")
        self.show_toolbar_button = False
        self.icon_file_name = os.path.join(PLUGIN_DIR, "icon_24.png")
        dark = os.path.join(PLUGIN_DIR, "icon_24_dark.png")
        if os.path.exists(dark):
            self.dark_icon_file_name = dark

    def Run(self):
        report = ["KiCad Track Gloss diagnostic", ""]
        try:
            changed = self._run(report, diagnostic=True)
        except NoTrackSelection:
            try:
                _show_session_settings()
            except Exception:
                LOG.exception("Could not display Track Gloss session settings")
            return
        except Exception:
            _warning_bell()
            LOG.exception("Track gloss diagnostic run failed")
            report.extend([
                "",
                "UNEXPECTED ERROR",
                "The operation was aborted; in-memory rollback was requested.",
                "",
                traceback.format_exc(),
            ])
        else:
            if changed is False:
                _warning_bell()
        try:
            _show_diagnostic_report("KiCad Track Gloss — Diagnostic", report)
        except Exception:
            LOG.exception("Could not display the Track Gloss diagnostic report")
