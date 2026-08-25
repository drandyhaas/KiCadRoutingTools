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
import types


LOG = logging.getLogger(__name__)


if not __package__:
    root = Path(__file__).resolve().parents[1]
    package = types.ModuleType("kicad_track_gloss")
    package.__path__ = [str(root)]
    sys.modules["kicad_track_gloss"] = package
    from kicad_track_gloss.engine.context import PlannerContext
    from kicad_track_gloss.engine.model import (
        AddedSegment, BoardModel, CircleObstacle, GlossResult, PadRegion,
        PolygonKeepout, Segment, Transformation)
    from kicad_track_gloss.engine.planner import (generate_converged_plan,
                                                   smooth_selected_chains)
else:
    from .context import PlannerContext
    from .model import (AddedSegment, BoardModel, CircleObstacle, GlossResult,
                        PadRegion, PolygonKeepout, Segment, Transformation)
    from .planner import generate_converged_plan


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
        pads.append(PadRegion(**item))
    return BoardModel(
        segments=[Segment(**item) for item in data["segments"]],
        obstacles=obstacles,
        keepouts=keepouts,
        net_clearances=dict(data["net_clearances"]),
        minimum_clearance=data["minimum_clearance"],
        copper_edge_clearance=data["copper_edge_clearance"],
        board_bounds=data["board_bounds"],
        pad_regions=pads)


def _stop_processes(processes):
    """Reap every worker before Windows temporary files are removed."""
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
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
                process.wait(timeout=2)
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
    with open(output_path, "wb") as stream:
        pickle.dump(rows, stream, protocol=pickle.HIGHEST_PROTOCOL)


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

    def collect(self):
        try:
            rows = []
            for process, output_path in zip(self.processes, self.outputs):
                _stdout, stderr = process.communicate(timeout=60)
                if process.returncode or not output_path.exists():
                    raise RuntimeError(stderr.decode("utf-8", "replace"))
                with open(output_path, "rb") as stream:
                    rows.extend(pickle.load(stream))
            decoded = [(tuple(group_key),
                        _decode_plan(plan) if plan is not None else None, error)
                       for group_key, plan, error in rows]
            return sorted(decoded, key=lambda row: row[0])
        except Exception:
            LOG.exception("Parallel Track Gloss planning failed; using sequential fallback")
            _stop_processes(self.processes)
            return None
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
        loads[index] += len(item[1])
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
        package_parent = str(Path(__file__).resolve().parents[2])
        worker_bootstrap = (
            "import pathlib,runpy,sys,types; "
            "root=pathlib.Path(sys.argv.pop(1)); "
            "package=types.ModuleType('kicad_track_gloss'); "
            "package.__path__=[str(root/'kicad_track_gloss')]; "
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
                [str(executable), "-c", worker_bootstrap, package_parent,
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
                             converge=False, max_passes=6):
    job = start_parallel_group_plans(
        model, group_items, kwargs, max_workers, converge, max_passes)
    return job.collect() if job is not None else None


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("input")
    parser.add_argument("output")
    arguments = parser.parse_args()
    if not arguments.worker:
        raise SystemExit("worker mode required")
    _worker(arguments.input, arguments.output)
