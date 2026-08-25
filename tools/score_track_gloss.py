#!/usr/bin/env python3
"""Score a complete KiCad route by its converged all-track gloss result.

The input file is never modified. The final ``SCORE=<float>`` stdout line
follows ``place_route_loop --accept-cmd``: lower is better.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import shutil
import sys
import tempfile
import types


ROOT = Path(__file__).resolve().parents[1]


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
        "--force", action="store_true",
        help="allow --output to replace an existing file (never the input)")
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
    """Stable machine output; the final line is the accept-cmd contract."""
    document = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return "GLOSS_SCORE_JSON=" + document + "\nSCORE={:.9f}".format(
        payload["score"])


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
    import pcbnew

    # Import engine modules without registering the GUI ActionPlugins in a
    # headless KiCad Python process.
    package = types.ModuleType("kicad_track_gloss")
    package.__path__ = [str(ROOT / "kicad_track_gloss")]
    sys.modules["kicad_track_gloss"] = package
    from kicad_track_gloss.engine import generate_candidate_plans
    from kicad_track_gloss.engine.geometry import length
    from kicad_track_gloss.engine.model import segment_key
    from kicad_track_gloss.kicad import BoardAdapter
    from kicad_track_gloss.kicad.selection import is_probable_diff_pair
    from kicad_track_gloss.version import __version__

    return (pcbnew, BoardAdapter, generate_candidate_plans, length,
            segment_key, is_probable_diff_pair, __version__)


def _geometry_signature(model):
    return tuple(sorted(
        (round(segment.start_x, 9), round(segment.start_y, 9),
         round(segment.end_x, 9), round(segment.end_y, 9),
         round(segment.width, 9), segment.layer, segment.net_id,
         bool(segment.locked), bool(segment.arc))
        for segment in model.segments))


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
             force=False, max_passes=16):
    (pcbnew, BoardAdapter, generate_candidate_plans, length,
     segment_key, is_probable_diff_pair, version) = _bootstrap_engine()
    with prepared_board(board_path, project_path) as (load_path, used_project):
        board = pcbnew.LoadBoard(str(load_path))
        adapter = BoardAdapter(pcbnew)
        initial = adapter.snapshot(board, require_selection=False)
        before_mm = sum(length(
            (segment.start_x, segment.start_y),
            (segment.end_x, segment.end_y))
            for segment in initial.model.segments if not segment.arc)
        before_segments = sum(
            not segment.arc for segment in initial.model.segments)
        first_counts = None
        seen = set()
        changed_passes = 0

        for _pass_index in range(max_passes):
            snapshot = adapter.snapshot(board, require_selection=False)
            signature = _geometry_signature(snapshot.model)
            if signature in seen:
                raise RuntimeError("gloss convergence entered a geometry cycle")
            seen.add(signature)
            records = {}
            for item in board.GetTracks():
                if str(item.GetClass()) != "PCB_TRACK":
                    continue
                segment = adapter.segment_from_item(item)
                records[segment_key(segment)] = (item, segment)
            seeds = {
                key for key, (_item, segment) in records.items()
                if (not segment.locked and
                    not is_probable_diff_pair(segment.net_name))
            }
            eligible, expanded, meanders = adapter.expand_eligible_keys(
                board, records, seeds, [])
            if first_counts is None:
                first_counts = (len(seeds), len(expanded), len(eligible),
                                len(meanders))
            plans = generate_candidate_plans(
                snapshot.model, eligible, min_gain=0.01,
                allow_equal_length_simpler=True,
                clearance=snapshot.minimum_clearance, parallel=parallel)
            best = next((plan for plan in plans if plan.changed), plans[0])
            if not best.changed:
                break
            adapter.apply(board, best, rollback_on_error=True)
            changed_passes += 1
        else:
            raise RuntimeError(
                "gloss did not reach a fixed point in {} passes".format(
                    max_passes))

        final = adapter.snapshot(board, require_selection=False)
        after_mm = sum(length(
            (segment.start_x, segment.start_y),
            (segment.end_x, segment.end_y))
            for segment in final.model.segments if not segment.arc)
        after_segments = sum(
            not segment.arc for segment in final.model.segments)
        saved_mm = max(0.0, before_mm - after_mm)
        selected_seeds, expanded_tracks, eligible_tracks, protected = first_counts
        output = _save_output(
            pcbnew, board, board_path, output_path, force=force)
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
            "selected_seeds": selected_seeds,
            "expanded_tracks": expanded_tracks,
            "eligible_tracks": eligible_tracks,
            "protected_tuned_tracks": protected,
            "convergence_passes": changed_passes,
            "fixed_point": True,
            "before_mm": before_mm,
            "potential_saved_mm": saved_mm,
            "potential_saved_percent": (
                100.0 * saved_mm / before_mm if before_mm else 0.0),
            "after_mm": after_mm,
            "segments_after": after_segments,
            "segments_saved": before_segments - after_segments,
            "changed": changed_passes > 0,
            "output": str(output) if output else None,
        }


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        if args.place_route_loop and args.output:
            raise ValueError("--output is forbidden with --place-route-loop")
        if args.force and not args.output:
            raise ValueError("--force requires --output")
        board, placed, route_json = resolve_inputs(
            args.paths, args.place_route_loop)
        board, project = resolve_board_project(board, args.project)
        payload = evaluate(
            board, project, parallel=not args.no_parallel,
            output_path=args.output, force=args.force)
        if placed is not None:
            payload["placed_board"] = str(placed.resolve())
            payload["route_json"] = str(route_json.resolve())
        print(score_stdout(payload))
        return 0
    except Exception as error:
        print("track-gloss-score: {}: {}".format(
            type(error).__name__, error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
