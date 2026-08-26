"""One-shot KiCad-native DRC gate for a composed Track Gloss plan."""

from __future__ import annotations

from collections import Counter, OrderedDict
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import time

if __package__:
    from .drc_report import (drc_increases as _drc_increases,
                             json_report_counts as _json_report_counts,
                             json_report_summary as _json_report_summary)
else:
    # The candidate-board helper deliberately executes this file directly in
    # KiCad's Python process.  Direct scripts have no package parent, but their
    # own directory is importable.  Keep that supported entry point explicit.
    from drc_report import (drc_increases as _drc_increases,
                            json_report_counts as _json_report_counts,
                            json_report_summary as _json_report_summary)


@dataclass
class NativeDrcResult:
    allowed: bool
    before: dict = field(default_factory=dict)
    after: dict = field(default_factory=dict)
    increases: dict = field(default_factory=dict)
    error: str = ""
    timings_ms: dict = field(default_factory=dict)
    baseline_cached: bool = False
    validation_mode: str = "native_parallel"


_CACHE_LIMIT = 8
_baseline_cache = OrderedDict()
_validation_cache = OrderedDict()
_cache_lock = threading.Lock()


def _cache_get(cache, key):
    with _cache_lock:
        value = cache.get(key)
        if value is not None:
            cache.move_to_end(key)
        return value


def _cache_put(cache, key, value):
    with _cache_lock:
        cache[key] = value
        cache.move_to_end(key)
        while len(cache) > _CACHE_LIMIT:
            cache.popitem(last=False)


def _state_digest(adapter, board_path):
    """Hash the exact snapshot, local rules and KiCad CLI executable state."""
    digest = hashlib.sha256()
    for path in (board_path, board_path.with_suffix(".kicad_pro"),
                 board_path.with_suffix(".kicad_dru")):
        digest.update(path.suffix.encode("ascii"))
        if path.is_file():
            digest.update(path.read_bytes())
    cli = _kicad_cli(adapter)
    stat = cli.stat()
    digest.update(str(cli.resolve()).encode("utf-8", errors="replace"))
    digest.update("{}:{}".format(stat.st_size, stat.st_mtime_ns).encode("ascii"))
    return digest.digest()


def _copy_summary(summary):
    counts, fingerprints = summary
    return Counter(counts), Counter(fingerprints)


def _hidden_process_kwargs():
    """Keep helper and kicad-cli processes out of the Windows taskbar."""
    if os.name != "nt":
        return {}
    return {"creationflags": getattr(subprocess, "CREATE_NO_WINDOW", 0)}


def _has_zones(board):
    for accessor in ("Zones", "GetZones"):
        try:
            return bool(list(getattr(board, accessor)()))
        except Exception:
            continue
    # Failure to inspect zones must retain the complete native gate.
    return True


def _point_line_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    squared = dx * dx + dy * dy
    if squared <= 1e-18:
        return ((point[0] - a[0]) ** 2 + (point[1] - a[1]) ** 2) ** 0.5
    cross = abs((point[0] - a[0]) * dy - (point[1] - a[1]) * dx)
    return cross / squared ** 0.5


def _addition_is_existing_copper(addition, removed):
    """Prove that the whole addition is covered by removed collinear copper."""
    a, b = addition.start, addition.end
    dx, dy = b[0] - a[0], b[1] - a[1]
    squared = dx * dx + dy * dy
    if squared <= 1e-18:
        return False
    intervals = []
    for segment in removed:
        if (segment.net_id != addition.net_id or
                segment.layer != addition.layer or
                abs(segment.width - addition.width) > 1e-6):
            continue
        start = (segment.start_x, segment.start_y)
        end = (segment.end_x, segment.end_y)
        if (_point_line_distance(start, a, b) > 1e-6 or
                _point_line_distance(end, a, b) > 1e-6):
            continue
        first = ((start[0] - a[0]) * dx + (start[1] - a[1]) * dy) / squared
        second = ((end[0] - a[0]) * dx + (end[1] - a[1]) * dy) / squared
        low, high = sorted((first, second))
        low, high = max(0.0, low), min(1.0, high)
        if high >= low - 1e-9:
            intervals.append((low, high))
    covered = 0.0
    for low, high in sorted(intervals):
        if low > covered + 1e-7:
            return False
        covered = max(covered, high)
        if covered >= 1.0 - 1e-7:
            return True
    return False


def _is_strict_removal_only_plan(adapter, board, plan):
    """Return true only when the plan cannot add copper to new geometry.

    With no copper zones and after the engine's exact connectivity/clearance
    checks, deleting copper cannot introduce a new collision. This deliberately
    excludes normal corner cutting, endpoint sliding and every zoned board.
    """
    if _has_zones(board) or not plan.remove_keys or not plan.additions:
        return False
    wanted = set(plan.remove_keys)
    removed = []
    for item in board.GetTracks():
        if _item_uuid(item) not in wanted:
            continue
        try:
            removed.append(adapter.segment_from_item(item))
        except Exception:
            return False
    if len(removed) != len(wanted):
        return False
    return all(_addition_is_existing_copper(addition, removed)
               for addition in plan.additions)


def _copy_project_files(source_board, target_board):
    source = Path(str(source_board.GetFileName()))
    for suffix in (".kicad_pro", ".kicad_dru"):
        candidate = source.with_suffix(suffix)
        if candidate.is_file():
            shutil.copy2(candidate, target_board.with_suffix(suffix))


def _kicad_cli(adapter):
    suffix = ".exe" if os.name == "nt" else ""
    candidates = []
    module_path = getattr(adapter.pcbnew, "__file__", "")
    if module_path:
        module = Path(module_path).resolve()
        candidates.extend(parent / ("kicad-cli" + suffix)
                          for parent in list(module.parents)[:5])
    resolved = shutil.which("kicad-cli")
    if resolved:
        candidates.append(Path(resolved))
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise RuntimeError("kicad-cli was not found beside pcbnew or on PATH")


def _kicad_python(adapter):
    suffix = ".exe" if os.name == "nt" else ""
    module_path = getattr(adapter.pcbnew, "__file__", "")
    candidates = []
    if module_path:
        module = Path(module_path).resolve()
        for directory in list(module.parents)[:5]:
            candidates.extend((directory / ("python" + suffix),
                               directory / ("python3" + suffix)))
    candidates.append(Path(sys.executable))
    for candidate in candidates:
        if candidate.is_file() and candidate.name.lower() not in (
                "kicad.exe", "pcbnew.exe"):
            return candidate
    raise RuntimeError("KiCad's Python interpreter was not found")


def _write_plan(path, plan):
    if any(str(key).startswith("geom:") for key in plan.remove_keys):
        raise RuntimeError("native DRC requires KiCad UUID track identities")
    path.write_text(json.dumps({
        "remove_keys": list(plan.remove_keys),
        "additions": [{
            "start": list(item.start), "end": list(item.end),
            "width": item.width, "layer": item.layer,
            "net_id": item.net_id,
        } for item in plan.additions],
    }), encoding="utf-8")


def _apply_plan_process(adapter, baseline_path, candidate_path, plan_path,
                        timeout_seconds=None):
    timeout = (180.0 if timeout_seconds is None else
               max(0.1, min(180.0, float(timeout_seconds))))
    process = subprocess.run([
        str(_kicad_python(adapter)), str(Path(__file__).resolve()),
        "--apply-plan", str(baseline_path), str(candidate_path),
        str(plan_path),
    ], capture_output=True, text=True, timeout=timeout,
        **_hidden_process_kwargs())
    if process.returncode != 0 or not candidate_path.is_file():
        detail = (process.stderr or process.stdout).strip()[:500]
        raise RuntimeError(
            "candidate snapshot failed (exit {}): {}".format(
                process.returncode, detail))


def _run_drc(adapter, board_path, report_path, timeout_seconds=None):
    timeout = (300.0 if timeout_seconds is None else
               max(0.1, min(300.0, float(timeout_seconds))))
    command = [
        str(_kicad_cli(adapter)), "pcb", "drc", "--format", "json",
        "--severity-all", "--units", "mm", "--refill-zones",
        "--output", str(report_path), str(board_path),
    ]
    process = subprocess.run(
        command, capture_output=True, text=True, timeout=timeout,
        **_hidden_process_kwargs())
    if process.returncode != 0 or not report_path.is_file():
        detail = (process.stderr or process.stdout).strip()[:500]
        raise RuntimeError(
            "native DRC failed (exit {}): {}".format(
                process.returncode, detail))
    return _json_report_summary(report_path.read_text(
        encoding="utf-8", errors="replace"))


def validate_native_plan(adapter, board, plan, *, force_native=False,
                         skip_native=False, timeout_seconds=None):
    """Reject a plan if KiCad reports any increased DRC category.

    Temporary boards and reports are private implementation artifacts and are
    removed before returning.  The current PCB and its zone fills are never
    mutated by this validation.
    """
    started = time.monotonic()
    deadline = (None if timeout_seconds is None else
                started + max(0.0, float(timeout_seconds)))
    timings = {}

    def remaining():
        if deadline is None:
            return None
        value = deadline - time.monotonic()
        if value <= 0.0:
            raise TimeoutError("KiCad DRC time budget reached")
        return value
    if force_native and skip_native:
        raise ValueError("native DRC cannot be both forced and skipped")
    if skip_native:
        timings["total"] = (time.monotonic() - started) * 1000.0
        return NativeDrcResult(
            True, timings_ms=timings,
            validation_mode="single_track_drc_disabled")
    if (not force_native and
            _is_strict_removal_only_plan(adapter, board, plan)):
        timings["total"] = (time.monotonic() - started) * 1000.0
        return NativeDrcResult(
            True, timings_ms=timings,
            validation_mode="geometric_removal_fast_path")
    try:
        with tempfile.TemporaryDirectory(prefix="kicad-track-gloss-drc-") as name:
            root = Path(name)
            baseline_path = root / "baseline.kicad_pcb"
            candidate_path = root / "candidate.kicad_pcb"
            plan_path = root / "plan.json"
            stage = time.monotonic()
            if not adapter.pcbnew.SaveBoard(str(baseline_path), board):
                raise RuntimeError("KiCad could not snapshot the current board")
            _copy_project_files(board, baseline_path)
            _copy_project_files(board, candidate_path)
            _write_plan(plan_path, plan)
            remaining()
            timings["snapshot"] = (time.monotonic() - stage) * 1000.0
            baseline_key = _state_digest(adapter, baseline_path)
            plan_key = hashlib.sha256(plan_path.read_bytes()).digest()
            cached_validation = _cache_get(
                _validation_cache, (baseline_key, plan_key))
            if cached_validation is not None:
                elapsed = (time.monotonic() - started) * 1000.0
                result = NativeDrcResult(
                    allowed=cached_validation.allowed,
                    before=dict(cached_validation.before),
                    after=dict(cached_validation.after),
                    increases=dict(cached_validation.increases),
                    error=cached_validation.error,
                    timings_ms={
                        "snapshot": timings["snapshot"],
                        "cache_lookup": elapsed,
                        "total": elapsed,
                    },
                    baseline_cached=True,
                    validation_mode="native_validation_cache")
                return result

            cached_before = _cache_get(_baseline_cache, baseline_key)
            baseline_cached = cached_before is not None

            def timed_drc(path, report, label):
                drc_started = time.monotonic()
                value = _run_drc(
                    adapter, path, report, timeout_seconds=remaining())
                return value, (time.monotonic() - drc_started) * 1000.0, label

            # Baseline DRC is independent from candidate construction. Start it
            # immediately and overlap it with both the helper process and the
            # candidate DRC. kicad-cli runs in separate processes, so this also
            # uses two CPU cores without exposing pcbnew SWIG objects to threads.
            with ThreadPoolExecutor(max_workers=2,
                                    thread_name_prefix="track-gloss-drc") as pool:
                before_future = None
                if not baseline_cached:
                    before_future = pool.submit(
                        timed_drc, baseline_path, root / "before.rpt", "before")
                stage = time.monotonic()
                _apply_plan_process(
                    adapter, baseline_path, candidate_path, plan_path,
                    timeout_seconds=remaining())
                timings["candidate_snapshot"] = (
                    time.monotonic() - stage) * 1000.0
                after_future = pool.submit(
                    timed_drc, candidate_path, root / "after.rpt", "after")
                if baseline_cached:
                    before, before_fingerprints = _copy_summary(cached_before)
                    timings["before_drc"] = 0.0
                else:
                    before_value, before_ms, _label = before_future.result(
                        timeout=remaining())
                    before, before_fingerprints = before_value
                    timings["before_drc"] = before_ms
                    _cache_put(_baseline_cache, baseline_key,
                               _copy_summary(before_value))
                after_value, after_ms, _label = after_future.result(
                    timeout=remaining())
                after, after_fingerprints = after_value
                timings["after_drc"] = after_ms
            increases = _drc_increases(
                before, after, before_fingerprints, after_fingerprints)
            timings["total"] = (time.monotonic() - started) * 1000.0
            result = NativeDrcResult(
                allowed=not increases,
                before=dict(before), after=dict(after),
                increases=dict(increases),
                timings_ms=timings, baseline_cached=baseline_cached)
            _cache_put(_validation_cache, (baseline_key, plan_key), result)
            return result
    except (TimeoutError, subprocess.TimeoutExpired) as error:
        timings["total"] = (time.monotonic() - started) * 1000.0
        return NativeDrcResult(
            False, error="KiCad DRC time budget reached: {}".format(error),
            timings_ms=timings, validation_mode="native_timeout")
    except Exception as error:
        timings["total"] = (time.monotonic() - started) * 1000.0
        return NativeDrcResult(False, error=str(error), timings_ms=timings)


def _item_uuid(item):
    try:
        return item.m_Uuid.AsString()
    except Exception:
        return item.GetUuid().AsString()


def _headless_apply(baseline_path, candidate_path, plan_path):
    try:
        import wx
        wx.Log.SetActiveTarget(wx.LogStderr())
    except Exception:
        pass
    import pcbnew

    def from_mm(value):
        try:
            scale = float(pcbnew.PCB_IU_PER_MM)
        except (AttributeError, TypeError, ValueError):
            return int(pcbnew.FromMM(float(value)))
        return int(round(float(value) * scale))

    def is_straight_track(item):
        try:
            return int(item.Type()) == int(pcbnew.PCB_TRACE_T)
        except (AttributeError, TypeError, ValueError):
            return (isinstance(item, pcbnew.PCB_TRACK) and
                    not isinstance(item, pcbnew.PCB_VIA) and
                    not isinstance(item, pcbnew.PCB_ARC))

    board = pcbnew.LoadBoard(str(baseline_path))
    plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
    tracks = {
        _item_uuid(item): item for item in board.GetTracks()
        if is_straight_track(item)
    }
    missing = [key for key in plan["remove_keys"] if key not in tracks]
    if missing:
        raise RuntimeError("candidate snapshot is missing planned tracks")
    for key in plan["remove_keys"]:
        board.RemoveNative(tracks[key])
    for addition in plan["additions"]:
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(pcbnew.VECTOR2I(
            from_mm(addition["start"][0]), from_mm(addition["start"][1])))
        track.SetEnd(pcbnew.VECTOR2I(
            from_mm(addition["end"][0]), from_mm(addition["end"][1])))
        track.SetWidth(from_mm(addition["width"]))
        track.SetLayer(int(addition["layer"]))
        track.SetNetCode(int(addition["net_id"]))
        board.Add(track)
    if not pcbnew.SaveBoard(str(candidate_path), board):
        raise RuntimeError("KiCad could not save the candidate snapshot")


if __name__ == "__main__":
    if len(sys.argv) == 5 and sys.argv[1] == "--apply-plan":
        _headless_apply(sys.argv[2], sys.argv[3], sys.argv[4])
    else:
        raise SystemExit("expected --apply-plan BASELINE CANDIDATE PLAN")
