#!/usr/bin/env python3
"""Replay Track Gloss against the frozen real-board regression pattern."""

from __future__ import annotations

from pathlib import Path
import sys
import types

import pcbnew
import wx


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = (ROOT / "tests" / "fixtures" / "track_gloss" /
           "dispenser_labels" / "dispenser_labels.kicad_pcb")

# Avoid ActionPlugin registration when this integration test imports engine
# modules under KiCad's headless Python interpreter.
package = types.ModuleType("kicad_track_gloss")
package.__path__ = [str(ROOT / "kicad_track_gloss")]
sys.modules["kicad_track_gloss"] = package

# Repeated in-memory LoadBoard calls can otherwise emit one harmless wx image
# handler warning per fixture reload.
WX_LOG_SILENCER = wx.LogNull()

from kicad_track_gloss.board_adapter import (  # noqa: E402
    BoardAdapter,
    _is_probable_diff_pair,
)
from kicad_track_gloss.gloss_engine import generate_candidate_plans  # noqa: E402
from kicad_track_gloss.model import segment_key  # noqa: E402


EXPECTED_TRACKS = 706
EXPECTED_SCOPES = 335
EXPECTED_CHANGES = 61
EXPECTED_SAVED_MM = 9.198662


def _records(adapter, board):
    result = {}
    for item in board.GetTracks():
        if str(item.GetClass()) == "PCB_TRACK":
            segment = adapter._segment_from_item(item)
            result[segment_key(segment)] = (item, segment)
    return result


def _scopes(board, adapter, records):
    scopes = {}
    assigned = set()
    for seed_key, (_item, seed) in sorted(records.items()):
        if seed.locked or _is_probable_diff_pair(seed.net_name) or seed_key in assigned:
            continue
        eligible, _expanded, _meanders = adapter.expand_eligible_keys(
            board, records, {seed_key}, [])
        signature = tuple(sorted(eligible))
        if signature and signature not in scopes:
            scopes[signature] = seed_key
            assigned.update(eligible)
    return scopes


def main():
    board = pcbnew.LoadBoard(str(FIXTURE))
    adapter = BoardAdapter(pcbnew)
    snapshot = adapter.snapshot(board, require_selection=False)
    records = _records(adapter, board)
    scopes = _scopes(board, adapter, records)

    assert len(records) == EXPECTED_TRACKS, (len(records), EXPECTED_TRACKS)
    assert len(scopes) == EXPECTED_SCOPES, (len(scopes), EXPECTED_SCOPES)

    changed = []
    for index, (eligible, seed) in enumerate(scopes.items(), 1):
        plans = generate_candidate_plans(
            snapshot.model, set(eligible), min_gain=0.01,
            allow_equal_length_simpler=True,
            clearance=snapshot.minimum_clearance)
        best = next((plan for plan in plans if plan.changed), None)
        if best is not None:
            changed.append((seed, best))
        if index % 50 == 0:
            print(f"generated {index}/{len(scopes)} scopes", flush=True)

    total_saved = round(sum(plan.saved_mm for _seed, plan in changed), 6)
    assert len(changed) == EXPECTED_CHANGES, (len(changed), EXPECTED_CHANGES)
    assert total_saved == EXPECTED_SAVED_MM, (total_saved, EXPECTED_SAVED_MM)

    # Every accepted pattern is applied to a fresh in-memory board. This tests
    # UUID lookup and pcbnew Add/RemoveNative without altering the fixture.
    for index, (_seed, plan) in enumerate(changed, 1):
        fresh = pcbnew.LoadBoard(str(FIXTURE))
        BoardAdapter(pcbnew).apply(fresh, plan, rollback_on_error=True)
        if index % 20 == 0:
            print(f"applied {index}/{len(changed)} plans in memory", flush=True)

    print(
        "PASS:", len(records), "tracks,", len(scopes), "scopes,",
        len(changed), "changes,", total_saved, "mm saved,",
        len(changed), "fresh in-memory applications")


if __name__ == "__main__":
    main()
