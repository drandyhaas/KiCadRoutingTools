#!/usr/bin/env python3
"""Slot (oval) drills must be modelled as capsules, not long-axis circles.

thunderscope's 30.2x1mm milled slots -- (drill oval 30.2 1) NPTH pads -- were
modelled as 30.2mm-diameter round holes, producing 817 phantom copper-to-hole
DRC violations (KiCad's own DRC: zero) and a 15mm-radius routing keep-out.
The capsule model keeps the keep-out at the slot's real 0.5mm half-width
along its axis.

    python3 tests/test_slot_drill_capsule.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb, pad_drill_capsule, pad_drill_circles
from check_drc import run_drc


def _board(tracks):
    return f'''(kicad_pcb (version 20241229) (generator test)
\t(layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
\t(net 0 "")
\t(net 1 "SIG")
\t(gr_line (start 0 0) (end 40 0) (layer "Edge.Cuts") (width 0.1))
\t(gr_line (start 40 0) (end 40 40) (layer "Edge.Cuts") (width 0.1))
\t(gr_line (start 40 40) (end 0 40) (layer "Edge.Cuts") (width 0.1))
\t(gr_line (start 0 40) (end 0 0) (layer "Edge.Cuts") (width 0.1))
\t(footprint "test:slot" (layer "F.Cu")
\t\t(at 20 20)
\t\t(property "Reference" "H1" (at 0 -2) (layer "F.SilkS"))
\t\t(pad "" np_thru_hole oval (at 0 0) (size 10 1) (drill oval 10 1) (layers "*.Cu" "*.Mask"))
\t)
{tracks}
)
'''


def _drc(tracks):
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(_board(tracks))
        path = f.name
    try:
        v = run_drc(path, clearance=0.2, quiet=True)
        return [x for x in v if x['type'] == 'track-hole']
    finally:
        os.unlink(path)


def run():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            fails.append(name)

    # --- geometry helpers ---
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(_board(''))
        path = f.name
    try:
        pcb = parse_kicad_pcb(path)
        pad = pcb.footprints['H1'].pads[0]
        check("slot dims parsed", pad.drill == 10 and pad.drill_w == 10 and pad.drill_h == 1)
        (x1, y1), (x2, y2), r = pad_drill_capsule(pad)
        check("capsule radius = short/2", abs(r - 0.5) < 1e-9)
        check("capsule spans slot axis",
              abs(x1 - 15.5) < 1e-6 and abs(x2 - 24.5) < 1e-6 and
              abs(y1 - 20) < 1e-6 and abs(y2 - 20) < 1e-6)
        circles = pad_drill_circles(pad)
        check("circles use short dimension", all(abs(d - 1.0) < 1e-9 for _x, _y, d in circles))
        xs = [c[0] for c in circles]
        check("circles cover the slot span",
              abs(min(xs) - 15.5) < 1e-6 and abs(max(xs) - 24.5) < 1e-6)
    finally:
        os.unlink(path)

    # --- DRC: parallel track 2mm off-axis (inside the old phantom 5mm disc) ---
    near_parallel = '\t(segment (start 15 22) (end 25 22) (width 0.2) (layer "F.Cu") (net 1) (uuid "t1"))'
    check("parallel track 2mm from slot axis is clean", _drc(near_parallel) == [])

    # --- DRC: track crossing the slot -> real violation ---
    crossing = '\t(segment (start 20 15) (end 20 25) (width 0.2) (layer "F.Cu") (net 1) (uuid "t2"))'
    check("track crossing the slot is flagged", len(_drc(crossing)) >= 1)

    # --- DRC: track grazing the slot END cap (beyond old short-radius, inside capsule) ---
    endcap = '\t(segment (start 24.9 19.0) (end 24.9 21.0) (width 0.2) (layer "F.Cu") (net 1) (uuid "t3"))'
    check("track inside the end-cap clearance is flagged", len(_drc(endcap)) >= 1)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
