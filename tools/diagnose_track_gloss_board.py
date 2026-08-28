#!/usr/bin/env python3
"""Read-only Track Gloss diagnosis against a real KiCad board."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import statistics
import sys
import time
import types

import pcbnew


ROOT = Path(__file__).resolve().parents[1]
package = types.ModuleType("kicad_track_gloss")
package.__path__ = [str(ROOT / "kicad_track_gloss")]
sys.modules["kicad_track_gloss"] = package

from kicad_track_gloss.engine import (  # noqa: E402
    find_pad_terminal_targets,
    find_track_terminal_targets,
    generate_converged_plan,
)
from kicad_track_gloss.configuration import CONFIG  # noqa: E402
from kicad_track_gloss.engine.model import segment_key  # noqa: E402
from kicad_track_gloss.kicad import BoardAdapter  # noqa: E402
from kicad_track_gloss.kicad.authority import protected_track_keys  # noqa: E402
from kicad_track_gloss.kicad.types import is_straight_track  # noqa: E402
from kicad_track_gloss.version import __version__  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("board")
    parser.add_argument("uuid", nargs="*")
    parser.add_argument("--apply-in-memory", action="store_true")
    parser.add_argument("--sweep", action="store_true")
    parser.add_argument("--max-scopes", type=int, default=0)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(args.board)
    print("plugin version:", __version__)
    adapter = BoardAdapter(pcbnew)
    snapshot = adapter.snapshot(board, require_selection=False)
    records = {}
    for item in board.GetTracks():
        if not is_straight_track(pcbnew, item):
            continue
        segment = adapter.segment_from_item(item)
        records[segment_key(segment)] = (item, segment)
    if args.sweep:
        sweep(board, adapter, snapshot, records, args.board, args.apply_in_memory,
              args.max_scopes)
        return
    missing = [uuid for uuid in args.uuid if uuid not in records]
    if not args.uuid or missing:
        raise SystemExit("Track UUID not found: " + str(missing or args.uuid))

    warnings = []
    eligible, expanded, protected = adapter.expand_eligible_keys(
        board, records, set(args.uuid), warnings)
    targets = find_track_terminal_targets(snapshot.model, eligible)
    pad_targets = find_pad_terminal_targets(snapshot.model, eligible)
    plan = generate_converged_plan(
        snapshot.model, eligible,
        min_gain=CONFIG.gloss.minimum_saved_length_mm,
        allow_equal_length_simpler=True,
        clearance=snapshot.minimum_clearance)

    print("board:", args.board)
    print("seeds:", args.uuid)
    print("expanded:", len(expanded), sorted(expanded))
    for key in sorted(expanded):
        segment = records[key][1]
        print("  ", key, (segment.start_x, segment.start_y), "->",
              (segment.end_x, segment.end_y))
    print("native-protected:", len(protected), sorted(protected))
    print("eligible:", len(eligible), sorted(eligible))
    print("warnings:", warnings)
    print("sliding terminals:", len(targets))
    for terminal, tracks in sorted(targets.items()):
        print(" ", terminal, "->", [segment_key(track) for track in tracks])
    print("sliding pad areas:", len(pad_targets))
    for terminal, regions in sorted(pad_targets.items()):
        print(" ", terminal, "->", [
            (region.shape, region.x, region.y, region.width, region.height)
            for region in regions])
    print("converged plan: changed=", plan.changed,
          "fixed_point=", plan.fixed_point,
          "passes=", plan.convergence_passes,
          "saved=", round(plan.saved_mm, 6),
          "remove=", len(plan.remove_keys), "add=", len(plan.additions),
          "chains=", plan.chains_considered)
    for addition in plan.additions:
        print("   ", addition.start, "->", addition.end)
    if args.apply_in_memory:
        if not plan.changed:
            raise SystemExit("No changed plan to apply")
        before = len(list(board.GetTracks()))
        created = adapter.apply(board, plan, rollback_on_error=True)
        after = len(list(board.GetTracks()))
        print("in-memory apply: passed; tracks", before, "->", after,
              "created", len(created), "(board was not saved)")


def sweep(board, adapter, snapshot, records, board_path, verify_apply, max_scopes):
    scopes = {}
    assigned = {}
    native_protected = protected_track_keys(adapter, board, records)
    for seed_key, (_item, seed) in sorted(records.items()):
        if seed_key in native_protected:
            continue
        if seed_key in assigned:
            continue
        warnings = []
        eligible, _expanded, protected = adapter.expand_eligible_keys(
            board, records, {seed_key}, warnings)
        eligible = frozenset(eligible)
        signature = tuple(sorted(eligible))
        if signature and signature not in scopes:
            scopes[signature] = (seed_key, seed.net_name, warnings, len(protected))
            for member in eligible:
                assigned[member] = signature

    rows = []
    scope_items = list(scopes.items())
    if max_scopes:
        scope_items = scope_items[:max_scopes]
    print("inventory: tracks", len(records), "unique scopes", len(scopes),
          "evaluating", len(scope_items), flush=True)
    for scope_index, (signature, scope_data) in enumerate(scope_items, 1):
        seed_key, net_name, warnings, meander_count = scope_data
        eligible = set(signature)
        planning_started = time.monotonic()
        try:
            best = generate_converged_plan(
                snapshot.model, eligible,
                min_gain=CONFIG.gloss.minimum_saved_length_mm,
                allow_equal_length_simpler=True,
                clearance=snapshot.minimum_clearance)
            if not best.changed:
                best = None
            error = ""
        except Exception as exception:
            best = None
            error = type(exception).__name__ + ": " + str(exception)
        planning_ms = (time.monotonic() - planning_started) * 1000.0
        apply_error = ""
        if verify_apply and best is not None:
            try:
                fresh = pcbnew.LoadBoard(board_path)
                BoardAdapter(pcbnew).apply(fresh, best, rollback_on_error=True)
            except Exception as exception:
                apply_error = type(exception).__name__ + ": " + str(exception)
        rows.append({
            "seed": seed_key,
            "net": net_name,
            "eligible": len(eligible),
            "terminals": len(find_track_terminal_targets(snapshot.model, eligible)),
            "pad_terminals": len(find_pad_terminal_targets(
                snapshot.model, eligible)),
            "changed": best is not None,
            "saved": round(best.saved_mm, 6) if best else 0.0,
            "remove": len(best.remove_keys) if best else 0,
            "add": len(best.additions) if best else 0,
            "meander": meander_count,
            "warnings": warnings,
            "error": error,
            "apply_error": apply_error,
            "planning_ms": planning_ms,
        })
        if scope_index % 10 == 0:
            print("evaluated", scope_index, "/", len(scope_items), flush=True)

    errors = [row for row in rows if row["error"]]
    apply_errors = [row for row in rows if row["apply_error"]]
    changed = sorted((row for row in rows if row["changed"]),
                     key=lambda row: (-row["saved"], row["seed"]))
    print("SWEEP", board_path)
    print("straight tracks:", len(records), "protected seeds:", len(native_protected))
    print("unique eligible connections:", len(rows))
    print("changed:", len(changed), "no-op:", len(rows) - len(changed) - len(errors))
    print("generation errors:", len(errors), "apply errors:", len(apply_errors))
    durations = sorted(row["planning_ms"] for row in rows)
    if durations:
        p90 = durations[min(len(durations) - 1,
                            math.ceil(0.9 * len(durations)) - 1)]
        print("planning ms: median {:.3f}, p90 {:.3f}, max {:.3f}".format(
            statistics.median(durations), p90, durations[-1]))
    print("total potential saving mm:", round(sum(row["saved"] for row in changed), 6))
    for label, selected in (("ERRORS", errors), ("APPLY ERRORS", apply_errors),
                            ("TOP CHANGES", changed[:25])):
        print(label)
        for row in selected:
            print(row)


if __name__ == "__main__":
    main()
