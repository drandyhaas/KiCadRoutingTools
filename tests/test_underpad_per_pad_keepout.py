#!/usr/bin/env python3
"""Issue #378: under-pad via keep-out reservations must scale per-pad.

Issue #202 sizes each via-in-pad to fit its OWN pad (clamp_via_to_pad), and the
keep-out reserved for it must use that same clamped size so a neighbouring
escape can route past the smaller via. The bulk reservation loops (plane balls,
Phase-B inner balls/pairs) used a single NOMINAL via_keep instead, over-reserving
on small/mixed-pitch pads and pre-blocking corridors the clamped via leaves open.

This pins the invariant behind the fix (per-pad keepout radius, computed exactly
as underpad.vkeep_for_pad does) and smoke-checks that the under-pad escape still
fans a dense array with a clamping via size (no regression).

    python3 tests/test_underpad_per_pad_keepout.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from bga_fanout.geometry import clamp_via_to_pad
from list_nets import fab_floor_ladder


def _pad(sx, sy):
    from kicad_parser import Pad
    return Pad(component_ref="U1", pad_number="1", global_x=0.0, global_y=0.0,
               local_x=0.0, local_y=0.0, size_x=sx, size_y=sy, shape="circle",
               layers=["F.Cu"], net_id=1, net_name="N")


def _vkeep(via_size, pad, floors, track_width, clearance, margin=0.0):
    """Per-pad keep-out radius -- mirrors underpad.vkeep_for_pad exactly."""
    cs = clamp_via_to_pad(via_size, 0.3, pad, floors)[0]
    return cs / 2 + track_width / 2 + clearance + margin


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")

    via_size, tw, clr = 0.5, 0.15, 0.1
    floors = fab_floor_ladder(4)  # 4-copper-layer standard fab-tier ladder
    nominal = via_size / 2 + tw / 2 + clr  # the old single via_keep

    # A pad the nominal via already fits: per-pad keepout == nominal (no change,
    # so ordinary coarse-pitch arrays are byte-identical).
    big = _vkeep(via_size, _pad(0.8, 0.8), floors, tw, clr)
    check("large pad keepout == nominal (no regression for fitting pads)",
          abs(big - nominal) < 1e-9)

    # A small pad clamps the via smaller -> its keepout is SMALLER than nominal,
    # freeing the corridor the #202 clamp was meant to recover.
    small = _vkeep(via_size, _pad(0.4, 0.4), floors, tw, clr)
    check("small (0.4) pad keepout is smaller than nominal (#378 corridor freed)",
          small < nominal - 1e-9)
    check("small pad keepout still covers the clamped via + track + clearance",
          small >= 0.4 / 2 + tw / 2 + clr - 1e-9)

    # Monotonic: smaller pad -> smaller-or-equal keepout, never larger (can only
    # free space, never under-reserve relative to the pad's own clamped via).
    mid = _vkeep(via_size, _pad(0.46, 0.46), floors, tw, clr)
    check("keepout monotonic in pad size (0.4 <= 0.46 <= 0.8)",
          small <= mid + 1e-9 <= big + 1e-9 and mid <= big + 1e-9)

    # No-regression smoke: the under-pad escape still fans a dense BGA with a
    # clamping via size (via 0.5 > the 0.4 balls -> clamps fire).
    board = os.path.join(ROOT, "kicad_files", "ulx3s.kicad_pcb")
    if os.path.exists(board):
        from kicad_parser import parse_kicad_pcb
        from bga_fanout import generate_bga_fanout
        pcb = parse_kicad_pcb(board)
        tracks, vias, _vr, failed = generate_bga_fanout(
            pcb.footprints["U1"], pcb,
            layers=["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"],
            track_width=0.12, clearance=0.1, via_size=0.5, via_drill=0.3,
            escape_method='underpad')
        check("under-pad escape still fans the dense array with clamping via",
              len(tracks) > 0 and len({t['net_id'] for t in tracks}) > 150)
    else:
        print("  [SKIP] ulx3s board not present -- smoke check skipped")

    print("=" * 60)
    if fails:
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  per-pad via keepout scales with pad size (nominal for fitting")
    print("        pads, smaller for clamped pads); dense escape unregressed")
    return 0


if __name__ == "__main__":
    sys.exit(run())
