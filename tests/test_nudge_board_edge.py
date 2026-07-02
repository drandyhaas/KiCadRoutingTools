#!/usr/bin/env python3
"""nudge_grazing_octolinear must not re-bend a jog off-board / across an
Edge.Cuts cutout (issue #256).

The octolinear re-bend candidates were only verified against foreign COPPER
(pads/tracks/vias), so a bend could be pushed through a board cutout the
original A* route legally skirted -- lily58 Net-(LED10-DIN) was re-bent 1mm
INTO a switch cutout ("off the board" DRC). The candidate list tries
diag-then-horizontal first: this board is built so that first candidate cuts
straight through a cutout while the second (horizontal-then-diag) is fully
legal. The fixed pass must pick the legal bend; the old code committed the
cutout-crossing one.

    python3 tests/test_nudge_board_edge.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb
from pcb_modification import nudge_grazing_octolinear
import check_drc as cd

BOARD = '''(kicad_pcb
 (version 20221018)
 (net 0 "")
 (net 1 "/A")
 (net 2 "/B")
 (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type solid)) (fill no)
  (layer "Edge.Cuts") (uuid "e1"))
 (gr_rect (start 6 11) (end 8 13) (stroke (width 0.1) (type solid)) (fill no)
  (layer "Edge.Cuts") (uuid "e2"))
 (footprint "L:A" (layer "F.Cu") (at 5 10)
   (property "Reference" "R1")
   (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu") (net 1 "/A")))
 (footprint "L:B" (layer "F.Cu") (at 15 12)
   (property "Reference" "R2")
   (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu") (net 1 "/A")))
 (footprint "L:C" (layer "F.Cu") (at 10 11.05)
   (property "Reference" "C1")
   (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu") (net 2 "/B")))
 (segment (start 5 10) (end 15 12) (width 0.2) (layer "F.Cu") (net 1) (uuid "s1"))
)'''


def run():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            fails.append(name)

    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(BOARD)
        path = f.name
    try:
        pcb = parse_kicad_pcb(path)
    finally:
        os.unlink(path)

    rings, outer, cuts = cd.board_edge_geometry(pcb.board_info)
    check("test board parses outline + 1 cutout",
          outer is not None and len(cuts) == 1)

    n_changed, nets_changed, _removed, _added = nudge_grazing_octolinear(
        [], pcb, None, clearance=0.3)

    segs = [s for s in pcb.segments if s.net_id == 1]
    off = [p for s in segs for p in ((s.start_x, s.start_y), (s.end_x, s.end_y))
           if not cd._point_on_board(p[0], p[1], outer, cuts)]
    check("re-bent net has NO off-board / in-cutout vertices", not off)

    # The graze had a legal alternative bend (horizontal-then-diag at y=10),
    # so the pass should still fix it -- rejecting the cutout bend must not
    # disable the nudge entirely.
    check("the jog was still re-bent via the legal candidate", nets_changed == 1)

    ok = all(cd._segment_to_rings_distance(s.start_x, s.start_y, s.end_x, s.end_y,
                                           rings) >= 0.3 + s.width / 2 - 1e-4
             for s in segs)
    check("all re-bent segments respect edge clearance", ok)

    print("=" * 60)
    if fails:
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("\n4/4 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
