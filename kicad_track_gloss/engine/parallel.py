"""Deterministic subprocess workers for independent fallback planning."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import logging
import os
from pathlib import Path
import pickle
import subprocess
import sys
import tempfile
import time
import types


LOG = logging.getLogger(__name__)


if not __package__:
    root = Path(__file__).resolve().parents[1]
    package = types.ModuleType("kicad_track_gloss")
    package.__path__ = [str(root)]
    sys.modules["kicad_track_gloss"] = package
    from kicad_track_gloss.engine.context import PlannerContext
    from kicad_track_gloss.engine.model import (
        AddedSegment, BoardModel, BoardOutline, CircleObstacle, GlossResult,
        PadRegion, PolygonKeepout, Segment, Transformation)
    from kicad_track_gloss.engine.planner import (generate_converged_plan,
                                                   smooth_selected_chains)
else:
    from .context import PlannerContext
    from .model import (AddedSegment, BoardModel, BoardOutline, CircleObstacle,
                        GlossResult, PadRegion, PolygonKeepout, Segment,
                        Transformation)
    from .planner import generate_converged_plan, smooth_selected_chains


def _encode_model(model):
    return {
        "segments": [asdict(item) for item in model.segments],
        "obstacles": [asdict(item) for item in model.obstacles],
        "keepouts": [asdict(item) for item in model.keepouts],
        "pad_regions": [asdict(item) for item in model.pad_regions],
        "net_clearances": dict(model.net_clearances),
        "minimum_clearance": model.minimum_clearance,
        "copper_edge_clearance": model.copper_edge_clearance,
        "board_bounds": model.board_bounds,
        "board_outline": (asdict(model.board_outline)
                          if model.board_outline is not None else None),
    }


def _decode_model(data):
    keepouts = []
    for item in data["keepouts"]:
        item["points"] = tuple(tuple(point) for point in item["points"])
        item["layers"] = tuple(item["layers"])
        keepouts.append(PolygonKeepout(**item))
    obstacles = []
    for item in data["obstacles"]:
        item["layers"] = tuple(item["layers"])
        obstacles.append(CircleObstacle(**item))
    pads = []
    for item in data["pad_regions"]:
        item["layers"] = tuple(item["layers"])
        item["polygons"] = tuple(
            (tuple(tuple(point) for point in outer),
             tuple(tuple(tuple(point) for point in hole) for hole in holes))
            for outer, holes in item.get("polygons", ()))
        pads.append(PadRegion(**item))
    outline = data.get("board_outline")
    if outline is not None:
        outline = BoardOutline(
            tuple(tuple(tuple(point) for point in polygon)
                  for polygon in outline.get("outlines", ())),
            tuple(tuple(tuple(point) for point in polygon)
                  for polygon in outline.get("holes", ())))
    return BoardModel(
        segments=[Segment(**item) for item in data["segments"]],
        obstacles=obstacles,
        keepouts=keepouts,
        net_clearances=dict(data["net_clearances"]),
        minimum_clearance=data["minimum_clearance"],
        copper_edge_clearance=data["copper_edge_clearance"],
        board_bounds=data["board_bounds"],
        pad_regions=pads,
        board_outline=outline)


def _stop_processes(processes, grace_seconds=2.0):
    """Reap every worker before Windows temporary files are removed."""
    deadline = time.monotonic() + max(0.0, float(grace_seconds))
    for process in processes:
        if process.poll() is None:
            try:
                process.terminate()
            except Exception:
                LOG.exception("Could not terminate Track Gloss worker")
    for process in processes:
        if process.poll() is not None:
            continue
        try:
            process.wait(timeout=max(0.0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            try:
                process.kill()
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except Exception:
                LOG.exception("Could not kill Track Gloss worker")


def _encode_plan(plan):
    return {
        "remove_keys": list(plan.remove_keys),
        "additions": [asdict(item) for item in plan.additions],
        "saved_mm": plan.saved_mm,
        "chains_considered": plan.chains_considered,
        "chains_changed": plan.chains_changed,
        "warnings": list(plan.warnings),
        "transformations": [asdict(item) for item in plan.transformations],
        "search_counts": dict(plan.search_counts),
        "blocking_nets": dict(plan.blocking_nets),
        "angle_corrections": plan.angle_corrections,
        "convergence_passes": plan.convergence_passes,
        "fixed_point": plan.fixed_point,
    }


def _decode_plan(data):
    return GlossResult(
        remove_keys=list(data["remove_keys"]),
        additions=[AddedSegment(**item) for item in data["additions"]],
        saved_mm=data["saved_mm"],
        chains_considered=data["chains_considered"],
        chains_changed=data["chains_changed"],
        warnings=list(data["warnings"]),
        transformations=[Transformation(**item)
                         for item in data["transformations"]],
        search_counts=dict(data["search_counts"]),
        blocking_nets=dict(data["blocking_nets"]),
        angle_corrections=data["angle_corrections"],
        convergence_passes=data.get("convergence_passes", 0),
        fixed_point=data.get("fixed_point", False))


def _worker(input_path, output_path):
    with open(input_path, "rb") as stream:
        payload = pickle.load(stream)
    model = _decode_model(payload["model"])
    context = PlannerContext(model)
    rows = []

    def publish():
        temporary = Path(str(output_path) + ".tmp-{}".format(os.getpid()))
        with open(temporary, "wb") as stream:
            pickle.dump(rows, stream, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(temporary, output_path)

    for group_key, eligible in payload["groups"]:
        try:
            if payload.get("converge"):
                worker_kwargs = dict(payload["kwargs"])
                worker_kwargs.pop("planner_context", None)
                plan = generate_converged_plan(
                    model, set(eligible),
                    max_passes=payload.get("max_passes", 6),
                    return_partial_on_limit=True, parallel=False,
                    **worker_kwargs)
            else:
                plan = smooth_selected_chains(
                    model, set(eligible), span_strategy="global",
                    path_preference=0, planner_context=context,
                    **payload["kwargs"])
            rows.append((group_key, _encode_plan(plan), ""))
        except Exception as error:
            rows.append((group_key, None,
                         type(error).__name__ + ": " + str(error)))
        # Publish after every deterministic group. The parent can retain work
        # completed before an interactive deadline instead of discarding the
        # whole worker batch and restarting sequentially.
        publish()


def _python_executable():
    configured = os.environ.get("KICAD_TRACK_GLOSS_PYTHON")
    candidates = [Path(configured)] if configured else []
    executable = Path(sys.executable)
    if executable.stem.lower().startswith("python"):
        candidates.append(executable)
    if os.name == "nt":
        candidates.append(executable.with_name("python.exe"))
    else:
        candidates.extend((executable.with_name("python3"),
                           executable.with_name("python")))
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


class ParallelPlanJob:
    def __init__(self, temporary, processes, outputs):
        self.temporary = temporary
        self.processes = processes
        self.outputs = outputs

    def collect(self, timeout_seconds=None, cancellation_grace_seconds=1.0):
        deadline = (None if timeout_seconds is None else
                    time.monotonic() + max(0.0, float(timeout_seconds)))
        timed_out = False
        try:
            while any(process.poll() is None for process in self.processes):
                if deadline is not None and time.monotonic() >= deadline:
                    timed_out = True
                    break
                time.sleep(0.02)
            if timed_out:
                _stop_processes(
                    self.processes, cancellation_grace_seconds)
            rows = []
            infrastructure_failed = False
            for process, output_path in zip(self.processes, self.outputs):
                if not timed_out and process.returncode:
                    _stdout, stderr = process.communicate()
                    LOG.error("Parallel Track Gloss worker failed: %s",
                              stderr.decode("utf-8", "replace"))
                    infrastructure_failed = True
                    continue
                if not output_path.exists():
                    if not timed_out:
                        infrastructure_failed = True
                    continue
                with open(output_path, "rb") as stream:
                    rows.extend(pickle.load(stream))
            if infrastructure_failed:
                # ``None`` explicitly asks the planner to use its in-process
                # path.  An empty list would look like a valid no-op and could
                # suppress every candidate search for a large selection.
                return None
            decoded = [(tuple(group_key),
                        _decode_plan(plan) if plan is not None else None, error)
                       for group_key, plan, error in rows]
            return sorted(decoded, key=lambda row: row[0]), timed_out
        except Exception:
            LOG.exception("Parallel Track Gloss planning failed; retaining no partial work")
            _stop_processes(self.processes)
            return [], timed_out
        finally:
            try:
                self.temporary.cleanup()
            except Exception:
                LOG.exception("Could not remove Track Gloss worker files")


def start_parallel_group_plans(model, group_items, kwargs, max_workers=0,
                               converge=False, max_passes=6):
    """Start deterministic workers, or return ``None`` for safe fallback."""
    executable = _python_executable()
    items = list(group_items)
    if executable is None or len(items) < 2:
        return None
    workers = max_workers or min(4, max(2, (os.cpu_count() or 2) // 2))
    workers = min(workers, len(items))
    chunks = [[] for _ in range(workers)]
    loads = [0] * workers
    # Longest-processing-time scheduling keeps dense nets from accumulating in
    # one worker while preserving deterministic output through final sorting.
    for item in sorted(items, key=lambda row: (-len(row[1]), row[0])):
        index = min(range(workers), key=lambda value: (loads[value], value))
        chunks[index].append(item)
        # Candidate spans grow faster than linearly with chain size. Squared
        # weights avoid placing several expensive long connections in the
        # same static worker batch.
        loads[index] += max(1, len(item[1])) ** 2
    primitive_kwargs = {key: value for key, value in kwargs.items()
                        if key != "planner_context"}
    creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    temporary = None
    processes = []
    try:
        temporary = tempfile.TemporaryDirectory(prefix="track-gloss-workers-")
        directory = Path(temporary.name)
        outputs = []
        encoded_model = _encode_model(model)
        plugin_root = str(Path(__file__).resolve().parents[1])
        worker_bootstrap = (
            "import pathlib,runpy,sys,types; "
            "root=pathlib.Path(sys.argv.pop(1)).resolve(); "
            "engine=(root/'engine').resolve(); "
            "sys.path[:]=[p for p in sys.path "
            "if pathlib.Path(p or '.').resolve()!=engine]; "
            "package=types.ModuleType('kicad_track_gloss'); "
            "package.__path__=[str(root)]; "
            "sys.modules['kicad_track_gloss']=package; "
            "runpy.run_module('kicad_track_gloss.engine.parallel', "
            "run_name='__main__')")
        for index, chunk in enumerate(chunks):
            input_path = directory / ("input-{}.pickle".format(index))
            output_path = directory / ("output-{}.pickle".format(index))
            with open(input_path, "wb") as stream:
                pickle.dump({"model": encoded_model, "groups": chunk,
                             "kwargs": primitive_kwargs,
                             "converge": bool(converge),
                             "max_passes": max_passes}, stream,
                            protocol=pickle.HIGHEST_PROTOCOL)
            process = subprocess.Popen(
                [str(executable), "-c", worker_bootstrap, plugin_root,
                 "--worker", str(input_path), str(output_path)],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                creationflags=creationflags)
            processes.append(process)
            outputs.append(output_path)
        return ParallelPlanJob(temporary, processes, outputs)
    except Exception:
        LOG.exception("Could not start parallel Track Gloss planning")
        _stop_processes(processes)
        if temporary is not None:
            try:
                temporary.cleanup()
            except Exception:
                LOG.exception("Could not remove Track Gloss worker files")
        return None


def run_parallel_group_plans(model, group_items, kwargs, max_workers=0,
                             converge=False, max_passes=6,
                             timeout_seconds=None,
                             cancellation_grace_seconds=1.0):
    job = start_parallel_group_plans(
        model, group_items, kwargs, max_workers, converge, max_passes)
    return (job.collect(timeout_seconds, cancellation_grace_seconds)
            if job is not None else None)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("input")
    parser.add_argument("output")
    arguments = parser.parse_args()
    if not arguments.worker:
        raise SystemExit("worker mode required")
    _worker(arguments.input, arguments.output)
