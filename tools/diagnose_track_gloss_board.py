#!/usr/bin/env python3
"""Read-only Track Gloss diagnosis against a real KiCad board."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import types

import pcbnew


ROOT = Path(__file__).resolve().parents[1]
package = types.ModuleType("kicad_track_gloss")
package.__path__ = [str(ROOT / "kicad_track_gloss")]
sys.modules["kicad_track_gloss"] = package

from kicad_track_gloss.board_adapter import BoardAdapter, _meander_keys  # noqa: E402
from kicad_track_gloss.gloss_engine import (  # noqa: E402
    find_track_terminal_targets,
    generate_candidate_plans,
)
from kicad_track_gloss.model import segment_key  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("board")
    parser.add_argument("uuid")
    parser.add_argument("--apply-in-memory", action="store_true")
    args = parser.parse_args()

    board = pcbnew.LoadBoard(args.board)
    adapter = BoardAdapter(pcbnew)
    snapshot = adapter.snapshot(board, require_selection=False)
    records = {}
    for item in board.GetTracks():
        if str(item.GetClass()) != "PCB_TRACK":
            continue
        segment = adapter._segment_from_item(item)
        records[segment_key(segment)] = (item, segment)
    if args.uuid not in records:
        raise SystemExit("Track UUID not found: " + args.uuid)

    warnings = []
    expanded = adapter._expand_seed_keys(
        board, records, {args.uuid}, warnings)
    meanders = _meander_keys(
        [segment for _item, segment in records.values()
         if segment_key(segment) in expanded])
    eligible = set(expanded)
    eligible.difference_update(meanders)
    targets = find_track_terminal_targets(snapshot.model, eligible)
    plans = generate_candidate_plans(
        snapshot.model, eligible, min_gain=0.01,
        allow_equal_length_simpler=True,
        clearance=snapshot.minimum_clearance)

    print("board:", args.board)
    print("seed:", args.uuid)
    print("expanded:", len(expanded), sorted(expanded))
    for key in sorted(expanded):
        segment = records[key][1]
        print("  ", key, (segment.start_x, segment.start_y), "->",
              (segment.end_x, segment.end_y))
    print("meander-protected:", len(meanders), sorted(meanders))
    print("eligible:", len(eligible), sorted(eligible))
    print("warnings:", warnings)
    print("sliding terminals:", len(targets))
    for terminal, tracks in sorted(targets.items()):
        print(" ", terminal, "->", [segment_key(track) for track in tracks])
    print("plans:", len(plans))
    for index, plan in enumerate(plans):
        print(index, "changed=", plan.changed, "saved=", round(plan.saved_mm, 6),
              "remove=", len(plan.remove_keys), "add=", len(plan.additions),
              "chains=", plan.chains_considered)
        for addition in plan.additions:
            print("   ", addition.start, "->", addition.end)
    if args.apply_in_memory:
        changed = [plan for plan in plans if plan.changed]
        if not changed:
            raise SystemExit("No changed plan to apply")
        before = len(list(board.GetTracks()))
        created = adapter.apply(board, changed[0], rollback_on_error=True)
        after = len(list(board.GetTracks()))
        print("in-memory apply: passed; tracks", before, "->", after,
              "created", len(created), "(board was not saved)")


if __name__ == "__main__":
    main()
