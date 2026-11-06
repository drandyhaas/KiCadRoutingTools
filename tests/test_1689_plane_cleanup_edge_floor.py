#!/usr/bin/env python3
"""#1689: the plane-copper cleanup honours the copper-to-edge floor the board is
graded at.

The route step's finalize runs the shared cleanup over the plane nets with a bare
config that carried no edge clearance, so its passes fell back to the track
clearance. Its gap snap then laid a connector from a free stub end to a same-net
pad 0.187 mm from the outline, on a board whose rule is 0.2: inside check_drc's
and KiCad's edge clearance, and absent from the route trace because it is cleanup
copper.

  1. a connector that keeps the track clearance but not the edge floor is refused;
  2. control: the same connector is laid where the floor allows it;
  3. the floor reaches the file front too.

    python3 tests/test_1689_plane_cleanup_edge_floor.py
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb               # noqa: E402
from pcb_modification import (clean_plane_copper,      # noqa: E402
                              compute_plane_copper_cleanup)

FAILS = []
W = 0.25
# The connector's copper ends 0.187 mm from the outline at x = 39.312: inside a
# 0.2 floor, outside the 0.0889 track clearance.
EDGE_X = 39.312


def check(name, cond, detail=''):
    print(("  ok   " if cond else "  FAIL ") + name
          + (f"  {detail}" if detail else ''))
    if not cond:
        FAILS.append(name)


def board(path):
    # A free GND stub whose end stops 0.3 mm short of a same-net pad, on a net
    # whose other copper hangs off a via so the stub is not a dead end to sweep.
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (gr_rect (start 10 0) (end {EDGE_X} 20) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
  (footprint "T:P" (layer "F.Cu") (at 39.2 10)
    (property "Reference" "J1" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu") (net "GND")))
  (footprint "T:P" (layer "F.Cu") (at 30 10)
    (property "Reference" "J2" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu") (net "GND")))
  (segment (start 30 10) (end 38.7 10) (width {W}) (layer "F.Cu") (net "GND"))
)
''')


def connectors(path, edge):
    pcb = parse_kicad_pcb(path)
    with contextlib.redirect_stdout(io.StringIO()):
        delta = compute_plane_copper_cleanup(pcb, ['GND'], 0.0889, 0.05, edge)
    return [(c.start_x, c.start_y, c.end_x, c.end_y) for c in delta.connectors]


def main():
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, 'in.kicad_pcb')
        board(src)

        print("1. a floor the connector would break")
        got = connectors(src, 0.2)
        check("no connector is laid", not got, f"{got}")

        print("2. control: a floor it keeps")
        got = connectors(src, 0.1)
        check("the connector is laid", len(got) == 1, f"{got}")

        print("3. the file front")
        with contextlib.redirect_stdout(io.StringIO()):
            snapped, _ = clean_plane_copper(src, ['GND'], 0.0889, 0.05, 0.2)
        check("no stub gap closed at the floor", snapped == 0, f"{snapped}")

    if FAILS:
        print(f"FAILED: {len(FAILS)}")
        return 1
    print("PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
