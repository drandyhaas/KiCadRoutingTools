#!/usr/bin/env python3
"""Score a complete KiCad route by its converged all-track gloss result.

The input file is never modified. ``SCORE_JSON=`` follows the score-instrument
convention used by KiCadRoutingTools, while the final ``SCORE=<float>`` stdout
line follows ``place_route_loop --accept-cmd``: lower is better.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import time
import types


ROOT = Path(__file__).resolve().parents[1]


def _internal_cli_max_passes():
    """Read the CLI default without importing the GUI plugin package."""
    path = ROOT / "kicad_track_gloss" / "internal_config.json"
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        value = document["convergence"]["cli_max_passes"]
    except (OSError, json.JSONDecodeError, KeyError, TypeError) as error:
        raise RuntimeError(
            "cannot read Track Gloss internal CLI policy: {}".format(error))
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError("convergence.cli_max_passes must be at least one")
    return value


def _internal_cli_time_budget():
    """Return the optional offline limit; null deliberately means unlimited."""
    path = ROOT / "kicad_track_gloss" / "internal_config.json"
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        value = document["timing"]["cli_total_time_budget_seconds"]
    except (OSError, json.JSONDecodeError, KeyError, TypeError) as error:
        raise RuntimeError(
            "cannot read Track Gloss internal CLI timing policy: {}".format(error))
    if value is not None and (
            isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or value <= 0.0):
        raise ValueError(
            "timing.cli_total_time_budget_seconds must be positive or null")
    return None if value is None else float(value)


def _internal_minimum_saved_length():
    """Read the shared plugin/CLI default without importing the GUI package."""
    path = ROOT / "kicad_track_gloss" / "internal_config.json"
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        value = document["gloss"]["minimum_saved_length_mm"]
    except (OSError, json.JSONDecodeError, KeyError, TypeError) as error:
        raise RuntimeError(
            "cannot read Track Gloss minimum saved length: {}".format(error))
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or value < 0.0):
        raise ValueError(
            "gloss.minimum_saved_length_mm must be finite and non-negative")
    return float(value)


def _parser():
    parser = argparse.ArgumentParser(
        description=(
            "Gloss every eligible straight track to a fixed point in memory "
            "and print a place_route_loop-compatible SCORE=<float>."))
    parser.add_argument(
        "paths", nargs="+", metavar="PATH",
        help=("direct mode: BOARD.kicad_pcb or PROJECT.kicad_pro; "
              "place-route-loop mode: PLACED.kicad_pcb ROUTED.kicad_pcb "
              "ROUTE.json"))
    parser.add_argument(
        "--project", metavar="PROJECT.kicad_pro",
        help=("project whose sibling .kicad_dru and routing rules must grade "
              "the board; defaults to the board's same-stem sibling"))
    parser.add_argument(
        "--place-route-loop", action="store_true",
        help="consume the three positional paths appended by --accept-cmd")
    parser.add_argument(
        "--no-parallel", action="store_true",
        help="disable independent net/layer worker processes")
    parser.add_argument(
        "--output", metavar="OUTPUT.kicad_pcb",
        help=("write the converged gloss result to a new board; omitted for "
              "read-only scoring and forbidden with --place-route-loop"))
    parser.add_argument(
        "--json-out", metavar="RESULT.json",
        help=("write the canonical SCORE_JSON payload to a JSON file; this is "
              "independent from the optional glossed PCB --output"))
    parser.add_argument(
        "--force", action="store_true",
        help="allow --output to replace an existing file (never the input)")
    parser.add_argument(
        "--scope", action="append", metavar="SCOPE",
        help=("authorized seed scope; repeat ALL, net:<exact-name>, or "
              "segment:<uuid>; defaults to ALL"))
    parser.add_argument(
        "--scope-file", metavar="SCOPE.json",
        help='JSON manifest containing {"scopes":["net:VCC", ...]}')
    parser.add_argument(
        "--max-passes", type=int, default=_internal_cli_max_passes(), metavar="N",
        help="maximum changed convergence passes; default from internal policy")
    parser.add_argument(
        "--time-budget", type=float, default=_internal_cli_time_budget(),
        metavar="SECONDS",
        help=("optional total planning and DRC time budget; the internal "
              "default is unlimited"))
    parser.add_argument(
        "--minimum-saved-length-mm", type=float,
        default=_internal_minimum_saved_length(), metavar="MM",
        help=("minimum saving required for each length-only transformation; "
              "default from the shared plugin/CLI policy"))
    parser.add_argument(
        "--trace-passes", action="store_true",
        help=("print one GLOSS_PASS_JSON record per convergence state to "
              "stderr, including the terminal fixed-point or limit record"))
    return parser


def resolve_inputs(paths, place_route_loop=False):
    """Return ``(board, placed, route_json)`` from the two CLI contracts."""
    if place_route_loop:
        if len(paths) != 3:
            raise ValueError(
                "--place-route-loop expects PLACED ROUTED ROUTE_JSON")
        return Path(paths[1]), Path(paths[0]), Path(paths[2])
    if len(paths) != 1:
        raise ValueError(
            "direct mode expects one BOARD.kicad_pcb or PROJECT.kicad_pro")
    return Path(paths[0]), None, None


def resolve_board_project(board_or_project, explicit_project=None):
    """Resolve the direct ``.kicad_pcb``/``.kicad_pro`` user interface."""
    source = Path(board_or_project)
    project = Path(explicit_project) if explicit_project else None
    if source.suffix.lower() == ".kicad_pro":
        if project is not None:
            raise ValueError(
                "do not combine a positional .kicad_pro with --project")
        return source.with_suffix(".kicad_pcb"), source
    if source.suffix.lower() != ".kicad_pcb":
        raise ValueError("expected a .kicad_pcb or .kicad_pro path")
    return source, project


def score_stdout(payload):
    """Stable machine output; the final line is the accept-cmd contract.

    ``GLOSS_SCORE_JSON`` remains as a compatibility alias for consumers of
    releases before 0.3.39. Both prefixes carry the exact same document.
    """
    document = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return ("SCORE_JSON=" + document + "\nGLOSS_SCORE_JSON=" + document +
            "\nSCORE={:.9f}".format(payload["score"]))


def write_json_output(payload, output_path):
    """Write the canonical score document requested by ``--json-out``."""
    if output_path is None:
        return None
    output = Path(output_path).resolve()
    if not output.parent.is_dir():
        raise ValueError(
            "JSON output directory does not exist: " + str(output.parent))
    output.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    return output


def resolve_scopes(values=None, scope_file=None):
    """Normalize CLI and manifest scopes without requiring pcbnew."""
    scopes = list(values or [])
    if scope_file:
        manifest_path = Path(scope_file)
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError("cannot read scope manifest: " + str(error))
        if not isinstance(manifest, dict) or not isinstance(
                manifest.get("scopes"), list):
            raise ValueError('scope manifest must contain a "scopes" array')
        scopes.extend(manifest["scopes"])
    if not scopes:
        scopes = ["ALL"]
    normalized = []
    for raw in scopes:
        if not isinstance(raw, str) or not raw.strip():
            raise ValueError("every scope must be a non-empty string")
        value = raw.strip()
        if value.upper() == "ALL":
            normalized.append("ALL")
            continue
        prefix, separator, target = value.partition(":")
        if not separator or prefix.lower() not in ("net", "segment") or not target:
            raise ValueError(
                "scope must be ALL, net:<exact-name>, or segment:<uuid>: " +
                value)
        normalized.append(prefix.lower() + ":" + target)
    normalized = list(dict.fromkeys(normalized))
    if "ALL" in normalized and len(normalized) != 1:
        raise ValueError("ALL cannot be combined with narrower scopes")
    return normalized


def seed_keys_for_scopes(records, scopes, protected_keys):
    """Resolve scope strings to the same admissible seeds as the plugin."""
    matched = set()
    if scopes == ["ALL"]:
        matched.update(records)
    else:
        for scope in scopes:
            kind, target = scope.split(":", 1)
            if kind == "segment":
                if target not in records:
                    raise ValueError("scope segment was not found: " + target)
                matched.add(target)
            else:
                net_keys = {
                    key for key, (_item, segment) in records.items()
                    if segment.net_name == target}
                if not net_keys:
                    raise ValueError("scope net was not found: " + target)
                matched.update(net_keys)
    seeds = set(matched) - set(protected_keys)
    if not seeds:
        raise ValueError(
            "scope contains no track outside KiCad native protection")
    return seeds


@contextmanager
def prepared_board(board_path, project_path=None):
    """Give pcbnew a same-stem board/project/rule set without touching input."""
    board_path = Path(board_path).resolve()
    if board_path.suffix.lower() != ".kicad_pcb" or not board_path.is_file():
        raise ValueError("board not found or not .kicad_pcb: " + str(board_path))
    if project_path is None:
        sibling = board_path.with_suffix(".kicad_pro")
        project_path = sibling if sibling.is_file() else None
    if project_path is None:
        yield board_path, None
        return

    project_path = Path(project_path).resolve()
    if project_path.suffix.lower() != ".kicad_pro" or not project_path.is_file():
        raise ValueError("project not found or not .kicad_pro: " + str(project_path))
    with tempfile.TemporaryDirectory(prefix="track-gloss-score-") as tmp_name:
        tmp = Path(tmp_name)
        staged_board = tmp / "score_target.kicad_pcb"
        staged_project = tmp / "score_target.kicad_pro"
        shutil.copy2(board_path, staged_board)
        shutil.copy2(project_path, staged_project)
        design_rules = project_path.with_suffix(".kicad_dru")
        if design_rules.is_file():
            shutil.copy2(design_rules, tmp / "score_target.kicad_dru")
        yield staged_board, project_path


def _bootstrap_engine():
    try:
        import wx
        wx.Log.SetActiveTarget(wx.LogStderr())
    except Exception:
        pass
    import pcbnew

    # Import engine modules without registering the GUI ActionPlugins in a
    # headless KiCad Python process.
    package = types.ModuleType("kicad_track_gloss")
    package.__path__ = [str(ROOT / "kicad_track_gloss")]
    sys.modules["kicad_track_gloss"] = package
    from kicad_track_gloss.engine import (
        generate_converged_plan)
    from kicad_track_gloss.configuration import CONFIG
    from kicad_track_gloss.engine.geometry import length
    from kicad_track_gloss.engine.model import segment_key
    from kicad_track_gloss.kicad import BoardAdapter
    from kicad_track_gloss.kicad.authority import protected_track_keys
    from kicad_track_gloss.kicad.types import is_straight_track
    from kicad_track_gloss.version import __version__

    return (pcbnew, BoardAdapter, generate_converged_plan,
            length, segment_key,
            protected_track_keys, is_straight_track, __version__, CONFIG)


def _save_output(pcbnew, board, input_path, output_path, force=False):
    if output_path is None:
        return None
    output = Path(output_path).resolve()
    input_path = Path(input_path).resolve()
    if output.suffix.lower() != ".kicad_pcb":
        raise ValueError("--output must end in .kicad_pcb")
    if output == input_path:
        raise ValueError("--output cannot overwrite the input board")
    if output.exists() and not force:
        raise ValueError("output already exists; use --force: " + str(output))
    if not output.parent.is_dir():
        raise ValueError("output directory does not exist: " + str(output.parent))
    if not pcbnew.SaveBoard(str(output), board):
        raise RuntimeError("KiCad could not save output board: " + str(output))
    return output


def evaluate(board_path, project_path=None, parallel=True, output_path=None,
             force=False, max_passes=None, scopes=None, pass_observer=None,
             time_budget_seconds=None, minimum_saved_length_mm=None):
    (pcbnew, BoardAdapter, generate_converged_plan, length, segment_key,
     protected_track_keys, is_straight_track,
     version, config) = _bootstrap_engine()
    from kicad_track_gloss.engine import (
        compose_compatible_connection_plans, generate_connection_candidates,
        rank_candidate_plans)
    from kicad_track_gloss.engine.model import GlossResult
    from kicad_track_gloss.kicad.native_salvage import (
        maximize_safe_native_candidates)
    if max_passes is None:
        max_passes = config.convergence.cli_max_passes
    if time_budget_seconds is None:
        time_budget_seconds = config.timing.cli_total_time_budget_seconds
    if minimum_saved_length_mm is None:
        minimum_saved_length_mm = config.gloss.minimum_saved_length_mm
    if (time_budget_seconds is not None and
            (not math.isfinite(time_budget_seconds) or
             time_budget_seconds <= 0.0)):
        raise ValueError("time budget must be positive")
    if (isinstance(minimum_saved_length_mm, bool) or
            not isinstance(minimum_saved_length_mm, (int, float)) or
            not math.isfinite(minimum_saved_length_mm) or
            minimum_saved_length_mm < 0.0):
        raise ValueError("minimum saved length must be finite and non-negative")
    minimum_saved_length_mm = float(minimum_saved_length_mm)
    operation_started = time.monotonic()
    timings_ms = {}
    operation_deadline = (
        None if time_budget_seconds is None else
        operation_started + float(time_budget_seconds))
    planning_deadline = (
        None if time_budget_seconds is None else
        operation_started + float(time_budget_seconds) * 0.5)
    with prepared_board(board_path, project_path) as (load_path, used_project):
        stage_started = time.monotonic()
        board = pcbnew.LoadBoard(str(load_path))
        timings_ms["load"] = (time.monotonic() - stage_started) * 1000.0
        adapter = BoardAdapter(pcbnew)
        stage_started = time.monotonic()
        initial = adapter.snapshot(board, require_selection=False)
        timings_ms["snapshot"] = (time.monotonic() - stage_started) * 1000.0
        before_mm = sum(length(
            (segment.start_x, segment.start_y),
            (segment.end_x, segment.end_y))
            for segment in initial.model.segments if not segment.arc)
        before_segments = sum(
            not segment.arc for segment in initial.model.segments)
        stage_started = time.monotonic()
        records = {}
        for item in board.GetTracks():
            if not is_straight_track(pcbnew, item):
                continue
            segment = adapter.segment_from_item(item)
            records[segment_key(segment)] = (item, segment)
        scopes = resolve_scopes(scopes)
        protected = protected_track_keys(pcbnew, board, records)
        seeds = seed_keys_for_scopes(records, scopes, protected)
        eligible, expanded, protected_expanded, connection_scopes = \
            adapter.expand_eligible_scopes(
            board, records, seeds, [])
        timings_ms["scope"] = (time.monotonic() - stage_started) * 1000.0
        if not eligible:
            raise ValueError("scope contains no eligible track after protection")
        conservative_ladder = []
        connection_plans = []
        connection_plan = None
        # The first half of a bounded run produces both the broad plan and
        # independently validatable connections.  The second half belongs to
        # native DRC and anytime salvage, which can always return its incumbent.
        connection_planning_deadline = planning_deadline
        if (len(connection_scopes) > 1 and
                (connection_planning_deadline is None or
                 time.monotonic() < connection_planning_deadline)):
            stage_started = time.monotonic()
            (connection_plans, _connection_rejections,
             _connection_deadline) = generate_connection_candidates(
                initial.model, eligible, connection_scopes, GlossResult(),
                min_gain=minimum_saved_length_mm,
                allow_equal_length_simpler=True,
                clearance=initial.minimum_clearance,
                max_passes=max_passes,
                group_max_passes=max_passes,
                collect_statistics=False,
                planning_deadline=connection_planning_deadline,
                cancellation_grace_seconds=(
                    config.timing.interactive_cancellation_grace_seconds))
            (connection_plan, _compatible_plans,
             _composition_rejections) = compose_compatible_connection_plans(
                initial.model, eligible, connection_plans)
            timings_ms["planning_connections"] = (
                time.monotonic() - stage_started) * 1000.0
        global_plan = GlossResult()
        if (planning_deadline is None or
                time.monotonic() < planning_deadline):
            stage_started = time.monotonic()
            global_plan = generate_converged_plan(
                initial.model, eligible, max_passes=max_passes,
                min_gain=minimum_saved_length_mm,
                allow_equal_length_simpler=True,
                clearance=initial.minimum_clearance, parallel=parallel,
                pass_observer=pass_observer, deadline=planning_deadline,
                conservative_ladder=conservative_ladder,
                cancellation_grace_seconds=(
                    config.timing.interactive_cancellation_grace_seconds))
            timings_ms["planning_primary"] = (
                time.monotonic() - stage_started) * 1000.0
        planning_candidates = [global_plan]
        if connection_plan is not None and connection_plan.changed:
            planning_candidates.append(connection_plan)
        if conservative_ladder and conservative_ladder[0].changed:
            planning_candidates.append(conservative_ladder[0])
        planning_candidates = list(rank_candidate_plans(planning_candidates))
        best = planning_candidates[0]
        native = None
        applied = False
        fallback_used = False
        fallback_plan_cached = bool(
            conservative_ladder and conservative_ladder[0].changed)
        connection_salvage_used = False
        connection_salvage_attempts = 0
        connections_retained = 0
        connections_planned = 0
        if best.changed:
            stage_started = time.monotonic()
            conservative = (
                conservative_ladder[0]
                if conservative_ladder and conservative_ladder[0].changed
                else None)
            decision = maximize_safe_native_candidates(
                adapter, board, initial.model, eligible,
                planning_candidates, conservative_plan=conservative,
                connection_plans=connection_plans,
                force_native=False, skip_native=False,
                operation_deadline=operation_deadline,
                wait_callback=None)
            timings_ms["native_candidate_search"] = (
                time.monotonic() - stage_started) * 1000.0
            if decision.initial_portfolio:
                timings_ms["native_portfolio"] = decision.initial_validation_ms
            else:
                timings_ms["native_primary"] = decision.initial_validation_ms
            if decision.followup_validations:
                timings_ms["native_candidate_followups"] = (
                    decision.followup_validation_ms)
            if decision.salvage_attempts:
                timings_ms["native_connection_salvage"] = decision.salvage_ms
            best = decision.plan or best
            native = decision.native
            fallback_used = decision.fallback_used
            connection_salvage_used = decision.salvage_used
            connection_salvage_attempts = decision.salvage_attempts
            connections_retained = decision.connections_retained
            connections_planned = decision.connections_planned
        if (best.changed and native is not None and
                not native.allowed and native.error):
            raise RuntimeError(
                "native DRC validation failed: {}".format(native.error))
        if best.changed and native.allowed:
            adapter.apply(board, best, rollback_on_error=True)
            applied = True

        planned_saved_mm = best.saved_mm
        planned_removed = len(best.remove_keys)
        planned_added = len(best.additions)
        planned_angle_corrections = best.angle_corrections
        stage_started = time.monotonic()
        final = adapter.snapshot(board, require_selection=False)
        timings_ms["final_snapshot"] = (
            time.monotonic() - stage_started) * 1000.0
        after_mm = sum(length(
            (segment.start_x, segment.start_y),
            (segment.end_x, segment.end_y))
            for segment in final.model.segments if not segment.arc)
        after_segments = sum(
            not segment.arc for segment in final.model.segments)
        saved_mm = max(0.0, before_mm - after_mm)
        stage_started = time.monotonic()
        output = _save_output(
            pcbnew, board, board_path, output_path, force=force)
        timings_ms["save"] = (time.monotonic() - stage_started) * 1000.0
        timings_ms["total"] = (
            time.monotonic() - operation_started) * 1000.0
        return {
            "schema": 1,
            "kind": "track-gloss-score",
            "plugin_version": version,
            "board": str(Path(board_path).resolve()),
            "project": str(used_project) if used_project else None,
            "score": after_mm,
            "score_meaning": (
                "virtual post-gloss straight-track copper length in mm; lower "
                "is better"),
            "straight_tracks": before_segments,
            "scopes": scopes,
            "selected_seeds": len(seeds),
            "expanded_tracks": len(expanded),
            "eligible_tracks": len(eligible),
            "native_protected_tracks": len(protected_expanded),
            "convergence_passes": best.convergence_passes,
            "max_passes": max_passes,
            "time_budget_seconds": time_budget_seconds,
            "minimum_saved_length_mm": minimum_saved_length_mm,
            "fixed_point": best.fixed_point,
            "planned_saved_mm": planned_saved_mm,
            "planned_removed": planned_removed,
            "planned_added": planned_added,
            "planned_angle_corrections": planned_angle_corrections,
            "before_mm": before_mm,
            "potential_saved_mm": saved_mm,
            "potential_saved_percent": (
                100.0 * saved_mm / before_mm if before_mm else 0.0),
            "after_mm": after_mm,
            "segments_after": after_segments,
            "segments_saved": before_segments - after_segments,
            "changed": applied,
            "candidate_ladder_fallback": fallback_used,
            "candidate_ladder_cached": fallback_plan_cached,
            "connection_salvage_used": connection_salvage_used,
            "connection_salvage_attempts": connection_salvage_attempts,
            "connections_retained": connections_retained,
            "connections_planned": connections_planned,
            "native_drc_gate": (
                "passed" if native and native.allowed else
                "rejected" if native else "not_needed"),
            "native_drc_increases": (
                native.increases if native is not None else {}),
            "native_drc_before": (
                native.before if native is not None else {}),
            "native_drc_after": (
                native.after if native is not None else {}),
            "native_drc_error": (
                native.error if native is not None else ""),
            "native_drc_timings_ms": (
                native.timings_ms if native is not None else {}),
            "native_drc_validation_mode": (
                native.validation_mode if native is not None else "not_needed"),
            "native_drc_baseline_cached": (
                native.baseline_cached if native is not None else False),
            "timings_ms": timings_ms,
            "output": str(output) if output else None,
        }


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        if args.max_passes < 1:
            raise ValueError("--max-passes must be at least one")
        if (args.time_budget is not None and
                (not math.isfinite(args.time_budget) or
                 args.time_budget <= 0.0)):
            raise ValueError("--time-budget must be positive")
        if (not math.isfinite(args.minimum_saved_length_mm) or
                args.minimum_saved_length_mm < 0.0):
            raise ValueError(
                "--minimum-saved-length-mm must be finite and non-negative")
        if args.place_route_loop and args.output:
            raise ValueError("--output is forbidden with --place-route-loop")
        if args.force and not args.output:
            raise ValueError("--force requires --output")
        board, placed, route_json = resolve_inputs(
            args.paths, args.place_route_loop)
        board, project = resolve_board_project(board, args.project)
        scopes = resolve_scopes(args.scope, args.scope_file)
        pass_observer = None
        if args.trace_passes:
            def emit_pass(state):
                print("GLOSS_PASS_JSON=" + json.dumps(
                    state, sort_keys=True, separators=(",", ":")),
                    file=sys.stderr, flush=True)
            pass_observer = emit_pass
        payload = evaluate(
            board, project, parallel=not args.no_parallel,
            output_path=args.output, force=args.force, scopes=scopes,
            max_passes=args.max_passes, pass_observer=pass_observer,
            time_budget_seconds=args.time_budget,
            minimum_saved_length_mm=args.minimum_saved_length_mm)
        if placed is not None:
            payload["placed_board"] = str(placed.resolve())
            payload["route_json"] = str(route_json.resolve())
        json_output = write_json_output(payload, args.json_out)
        if json_output is not None:
            payload["json_output"] = str(json_output)
            # Re-write so the file and stdout carry the same final payload.
            write_json_output(payload, json_output)
        print(score_stdout(payload))
        return 0
    except Exception as error:
        print("track-gloss-score: {}: {}".format(
            type(error).__name__, error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
