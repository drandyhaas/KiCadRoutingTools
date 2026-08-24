#!/usr/bin/env python3
"""Replay Track Gloss against the frozen real-board regression pattern."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import random
import sys
import types

import pcbnew
import wx


ROOT = Path(__file__).resolve().parents[2]
FIXTURE = (ROOT / "tests" / "track_gloss" / "patterns" /
           "dispenser_labels" / "dispenser_labels.kicad_pcb")

# Avoid ActionPlugin registration when this integration test imports engine
# modules under KiCad's headless Python interpreter.
package = types.ModuleType("kicad_track_gloss")
package.__path__ = [str(ROOT / "kicad_track_gloss")]
sys.modules["kicad_track_gloss"] = package

# Repeated in-memory LoadBoard calls can otherwise emit one harmless wx image
# handler warning per fixture reload.
WX_LOG_SILENCER = wx.LogNull()

from kicad_track_gloss.engine import generate_candidate_plans  # noqa: E402
from kicad_track_gloss.engine.model import segment_key  # noqa: E402
from kicad_track_gloss.kicad import BoardAdapter  # noqa: E402
from kicad_track_gloss.kicad.selection import (  # noqa: E402
    is_probable_diff_pair as _is_probable_diff_pair,
)


EXPECTED_TRACKS = 706
EXPECTED_SCOPES = 334
EXPECTED_CHANGES = 61
EXPECTED_SAVED_MM = 9.198662
EXPECTED_ALL_SELECTED_SAVED_MM = 4.341542
EXPECTED_ALL_SELECTED_REMOVED = 100
EXPECTED_ALL_SELECTED_ADDED = 62
MICRO_JOG_SEED = "58ebb541-fac6-4d02-8a68-65aca50766b5"


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


def _plan_signature(plan):
    return (
        tuple(sorted(plan.remove_keys)),
        tuple(sorted((addition.start, addition.end, addition.width,
                      addition.layer, addition.net_id)
                     for addition in plan.additions)),
        round(plan.saved_mm, 9),
    )


def _all_selected_regression(board, adapter, snapshot, records):
    seeds = {key for key, (_item, segment) in records.items()
             if not segment.locked and not _is_probable_diff_pair(segment.net_name)}
    eligible, expanded, meanders = adapter.expand_eligible_keys(
        board, records, seeds, [])

    original = list(snapshot.model.segments)
    variants = {
        "board order": original,
        "reverse order": list(reversed(original)),
        "nets ascending": sorted(
            original, key=lambda segment: (segment.net_id, segment.layer,
                                           segment.width, segment_key(segment))),
        "nets descending": sorted(
            original, key=lambda segment: (-segment.net_id, -segment.layer,
                                           -segment.width, segment_key(segment))),
    }
    for seed in range(3):
        shuffled = list(original)
        random.Random(seed).shuffle(shuffled)
        variants[f"shuffle {seed}"] = shuffled

    signatures = {}
    reference_plans = None
    for label, ordered_segments in variants.items():
        print(f"all-selected order: {label}", flush=True)
        model = replace(snapshot.model, segments=ordered_segments)
        plans = generate_candidate_plans(
            model, eligible, min_gain=0.01,
            allow_equal_length_simpler=True,
            clearance=snapshot.minimum_clearance)
        signatures[label] = tuple(_plan_signature(plan) for plan in plans)
        if reference_plans is None:
            reference_plans = plans
    assert len(set(signatures.values())) == 1, {
        label: len(signature) for label, signature in signatures.items()
    }

    changed = [plan for plan in reference_plans if plan.changed]
    assert changed
    best = changed[0]
    assert round(best.saved_mm, 6) == EXPECTED_ALL_SELECTED_SAVED_MM
    assert len(best.remove_keys) == EXPECTED_ALL_SELECTED_REMOVED
    assert len(best.additions) == EXPECTED_ALL_SELECTED_ADDED
    fresh = pcbnew.LoadBoard(str(FIXTURE))
    BoardAdapter(pcbnew).apply(fresh, best, rollback_on_error=True)
    saved_segments = len(best.remove_keys) - len(best.additions)
    print(
        "ALL SELECTED PASS:", len(seeds), "seeds,", len(expanded), "expanded,",
        len(meanders), "meander-protected,", len(eligible), "eligible,",
        round(best.saved_mm, 6), "mm saved,", saved_segments,
        "segments saved (", len(best.remove_keys), "removed /",
        len(best.additions), "added ),", len(variants), "orders identical")
    return best


def _dense_micro_jog_regression(board, adapter, records):
    warnings = []
    eligible, expanded, protected = adapter.expand_eligible_keys(
        board, records, {MICRO_JOG_SEED}, warnings)
    assert len(expanded) == 111, len(expanded)
    assert protected == expanded
    assert not eligible
    assert any("dense micro-jog" in warning for warning in warnings)
    print("DENSE MICRO-JOG PASS: 111 tuned segments protected, 0 planned")


def main():
    board = pcbnew.LoadBoard(str(FIXTURE))
    adapter = BoardAdapter(pcbnew)
    snapshot = adapter.snapshot(board, require_selection=False)
    records = _records(adapter, board)
    scopes = _scopes(board, adapter, records)

    assert len(records) == EXPECTED_TRACKS, (len(records), EXPECTED_TRACKS)
    _dense_micro_jog_regression(board, adapter, records)
    assert len(scopes) == EXPECTED_SCOPES, (len(scopes), EXPECTED_SCOPES)

    _all_selected_regression(board, adapter, snapshot, records)

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
