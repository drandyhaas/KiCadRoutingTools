#!/usr/bin/env python3
"""Replay Track Gloss against the frozen real-board regression pattern."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import random
import shutil
import sys
import tempfile
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

from kicad_track_gloss.engine import generate_converged_plan  # noqa: E402
from kicad_track_gloss.engine.model import segment_key  # noqa: E402
from kicad_track_gloss.kicad import BoardAdapter  # noqa: E402
from kicad_track_gloss.kicad.selection import (  # noqa: E402
    is_probable_diff_pair as _is_probable_diff_pair,
)


EXPECTED_TRACKS = 706
EXPECTED_SCOPES = 334
EXPECTED_ALL_SELECTED_SAVED_MM = 64.029515
EXPECTED_ALL_SELECTED_REMOVED = 227
EXPECTED_ALL_SELECTED_ADDED = 192
MICRO_JOG_SEED = "58ebb541-fac6-4d02-8a68-65aca50766b5"
SHORT_VCC_SEED = "cc798608-5e9b-4c2a-9856-dde85f9d85f0"
PAD_SLIDING_SEED = "54640123-2d45-4136-984c-783155178230"
PASTE_PAD_SEED = "e149801e-8263-4ee7-8861-6e960836dada"
DESCENDING_GND_SEED = "4fd6ed29-9fec-4147-a9e1-484055bf19bc"
MULTI_WIDTH_GND_SEEDS = {
    "912e3a2c-243d-40fe-9ff5-205898090e6e",
    "eaa5f084-9cb0-4b63-b93a-cf41c344d3ac",
}


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


def _all_selected_regression(board, adapter, snapshot, records,
                             check_all_orders=False):
    seeds = {key for key, (_item, segment) in records.items()
             if not segment.locked and not _is_probable_diff_pair(segment.net_name)}
    eligible, expanded, meanders = adapter.expand_eligible_keys(
        board, records, seeds, [])
    with tempfile.TemporaryDirectory(prefix="track-gloss-selection-") as name:
        temporary = Path(name)
        selected_path = temporary / "selected.kicad_pcb"
        shutil.copy2(FIXTURE, selected_path)
        for suffix in (".kicad_pro", ".kicad_dru"):
            sibling = FIXTURE.with_suffix(suffix)
            if sibling.is_file():
                shutil.copy2(sibling, selected_path.with_suffix(suffix))
        selected_board = pcbnew.LoadBoard(str(selected_path))
        selected_adapter = BoardAdapter(pcbnew)
        selected_records = _records(selected_adapter, selected_board)
        for item, _segment in selected_records.values():
            item.SetSelected()
        plugin_snapshot = selected_adapter.snapshot(selected_board)
        assert plugin_snapshot.eligible_keys == eligible
        assert plugin_snapshot.selection_seed_count == len(seeds)

    original = list(snapshot.model.segments)
    variants = {"board order": original}
    if check_all_orders:
        variants.update({
            "reverse order": list(reversed(original)),
            "nets ascending": sorted(
                original,
                key=lambda segment: (segment.net_id, segment.layer,
                                     segment.width, segment_key(segment))),
            "nets descending": sorted(
                original,
                key=lambda segment: (-segment.net_id, -segment.layer,
                                     -segment.width, segment_key(segment))),
        })
        for seed in range(3):
            shuffled = list(original)
            random.Random(seed).shuffle(shuffled)
            variants[f"shuffle {seed}"] = shuffled

    signatures = {}
    reference_plan = None
    for label, ordered_segments in variants.items():
        print(f"all-selected order: {label}", flush=True)
        model = replace(snapshot.model, segments=ordered_segments)
        plan = generate_converged_plan(
            model, eligible, min_gain=0.01,
            allow_equal_length_simpler=True,
            clearance=snapshot.minimum_clearance, parallel=True)
        signatures[label] = _plan_signature(plan)
        if reference_plan is None:
            reference_plan = plan
    if check_all_orders:
        assert len(set(signatures.values())) == 1, {
            label: len(signature) for label, signature in signatures.items()
        }

    best = reference_plan
    print("all-selected result:", round(best.saved_mm, 6), "mm,",
          len(best.remove_keys), "removed,", len(best.additions), "added",
          flush=True)
    assert best.changed and best.fixed_point
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
        len(best.additions), "added ), plugin/CLI scopes identical,",
        len(variants), "orders identical")
    if not check_all_orders:
        print("ORDER INDEPENDENCE CHECK SUSPENDED: use --all-orders to replay "
              "all 7 input orders", flush=True)
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


def _short_vcc_regression(board, adapter, snapshot, records):
    eligible, expanded, protected = adapter.expand_eligible_keys(
        board, records, {SHORT_VCC_SEED}, [])
    best = generate_converged_plan(
        snapshot.model, eligible, min_gain=0.01,
        allow_equal_length_simpler=True,
        clearance=snapshot.minimum_clearance, parallel=True)
    assert len(expanded) == 9
    assert not protected
    assert round(best.saved_mm, 6) == 1.003620
    assert len(best.remove_keys) == 8
    assert len(best.additions) == 6
    fresh = pcbnew.LoadBoard(str(FIXTURE))
    BoardAdapter(pcbnew).apply(fresh, best, rollback_on_error=True)
    print("SHORT VCC PASS: 1.003620 mm saved with exact pad-area sliding")


def _pad_sliding_regression(board, adapter, snapshot, records):
    eligible, expanded, protected = adapter.expand_eligible_keys(
        board, records, {PAD_SLIDING_SEED}, [])
    best = generate_converged_plan(
        snapshot.model, eligible, min_gain=0.01,
        allow_equal_length_simpler=True,
        clearance=snapshot.minimum_clearance, parallel=True)
    assert len(expanded) == 1
    assert not protected
    assert round(best.saved_mm, 6) == 0.596798
    assert len(best.remove_keys) == 1
    assert len(best.additions) == 1
    assert [item.mechanism for item in best.transformations] == ["pad_slide"]
    assert [item.geometry for item in best.transformations] == [
        "corner_relocation"]
    fresh = pcbnew.LoadBoard(str(FIXTURE))
    BoardAdapter(pcbnew).apply(fresh, best, rollback_on_error=True)
    print("PAD SLIDING PASS: 0.596798 mm saved between two pad areas")


def _reported_clearance_regressions(board, adapter, snapshot, records):
    # F.Paste-only apertures around A1 must never enter the copper model.
    assert not any(
        obstacle.net_id == 0 and
        abs(obstacle.x - 165.062) < 1e-6 and
        abs(obstacle.y - 96.774) < 1e-6
        for obstacle in snapshot.model.obstacles)

    cases = (
        ({PASTE_PAD_SEED}, 1.453743, 7, 3),
        # The mixed-width engine now moves the 0.127/0.25 transition instead
        # of retaining it as a fixed anchor, so all four originals are
        # replaced by three exact-width octolinear segments.
        ({DESCENDING_GND_SEED}, 0.714108, 4, 3),
        (MULTI_WIDTH_GND_SEEDS, 2.856996, 9, 3),
    )
    results = []
    for seeds, expected_saved, expected_removed, expected_added in cases:
        eligible, _expanded, protected = adapter.expand_eligible_keys(
            board, records, set(seeds), [])
        assert not protected
        plan = generate_converged_plan(
            snapshot.model, eligible, min_gain=0.01,
            allow_equal_length_simpler=True,
            clearance=snapshot.minimum_clearance, parallel=True)
        assert round(plan.saved_mm, 6) == expected_saved
        assert len(plan.remove_keys) == expected_removed
        assert len(plan.additions) == expected_added
        assert set(seeds) <= set(plan.remove_keys)
        fresh = pcbnew.LoadBoard(str(FIXTURE))
        BoardAdapter(pcbnew).apply(fresh, plan, rollback_on_error=True)
        results.append(plan)

    descending = results[1]
    assert any(abs(addition.start[1] - 108.3) < 1e-6 and
               abs(addition.end[1] - 108.3) < 1e-6
               for addition in descending.additions)
    multi_width = results[2]
    assert any({addition.start, addition.end} == {
        (208.086179, 120.125), (208.5, 120.125)}
        for addition in multi_width.additions)
    assert not any(abs(point[0] - 208.6) < 1e-6 and
                   abs(point[1] - 120.125) < 1e-6
                   for addition in multi_width.additions
                   for point in (addition.start, addition.end))
    print("REPORTED CASES PASS: paste pads ignored, exact pad corridors, "
          "multi-width T refinement")


def main():
    parser = argparse.ArgumentParser(
        description="Replay Track Gloss against the real-board pattern.")
    parser.add_argument(
        "--all-orders", action="store_true",
        help="also replay all 7 segment orders (slow, disabled by default)")
    parser.add_argument(
        "--full-sweep", action="store_true",
        help="also generate every scope and apply every changed plan "
             "(slow, disabled by default)")
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(FIXTURE))
    adapter = BoardAdapter(pcbnew)
    snapshot = adapter.snapshot(board, require_selection=False)
    records = _records(adapter, board)

    assert len(records) == EXPECTED_TRACKS, (len(records), EXPECTED_TRACKS)
    _dense_micro_jog_regression(board, adapter, records)
    _short_vcc_regression(board, adapter, snapshot, records)
    _pad_sliding_regression(board, adapter, snapshot, records)
    _reported_clearance_regressions(board, adapter, snapshot, records)

    _all_selected_regression(
        board, adapter, snapshot, records, check_all_orders=args.all_orders)

    if not args.full_sweep:
        print("FULL SCOPE SWEEP SUSPENDED: use --full-sweep to generate every "
              "scope and apply every changed plan", flush=True)
        print("PASS: routine real-board regressions")
        return

    scopes = _scopes(board, adapter, records)
    assert len(scopes) == EXPECTED_SCOPES, (len(scopes), EXPECTED_SCOPES)
    changed = []
    for index, (eligible, seed) in enumerate(scopes.items(), 1):
        best = generate_converged_plan(
            snapshot.model, set(eligible), min_gain=0.01,
            allow_equal_length_simpler=True,
            clearance=snapshot.minimum_clearance, parallel=True)
        if best.changed:
            changed.append((seed, best))
        if index % 50 == 0:
            print(f"generated {index}/{len(scopes)} scopes", flush=True)

    total_saved = round(sum(plan.saved_mm for _seed, plan in changed), 6)

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
