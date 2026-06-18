#!/usr/bin/env python3
"""extract_segments must linearize (arc ...) track elements into straight
Segments so connectivity/clearance see arc copper. KiCad routes rounded corners
as arcs; dropping them fragmented hand-routed boards and produced false
disconnections vs KiCad (the boards/ reference set).

    python3 tests/test_arc_track_parse.py
"""
import os
import sys
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import extract_segments


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # A quarter-circle arc (1,0)->(0,1) centered at origin (mid on the curve),
    # plus a straight segment feeding its start. v10 named-net format.
    snippet = '''
    (segment (start 2 0) (end 1 0) (width 0.2) (layer "F.Cu") (net "SIG") (uuid "s"))
    (arc (start 1 0) (mid 0.70710678 0.70710678) (end 0 1) (width 0.2) (layer "F.Cu") (net "SIG") (uuid "a"))
    '''
    segs = extract_segments(snippet, name_to_id={'SIG': 7})

    arc_pieces = [s for s in segs if s.uuid == 'a']
    check("arc linearized into multiple straight segments", len(arc_pieces) >= 8)
    check("all pieces on the arc's net", all(s.net_id == 7 for s in segs))
    check("all pieces on the arc's layer/width",
          all(s.layer == 'F.Cu' and abs(s.width - 0.2) < 1e-9 for s in arc_pieces))

    # Endpoints preserved exactly so the arc chains to its neighbours.
    starts = [(round(s.start_x, 6), round(s.start_y, 6)) for s in arc_pieces]
    ends = [(round(s.end_x, 6), round(s.end_y, 6)) for s in arc_pieces]
    check("arc starts exactly at (1,0)", (1.0, 0.0) in starts)
    check("arc ends exactly at (0,1)", (0.0, 1.0) in ends)

    # Pieces chain end-to-start (a connected polyline).
    chained = all(abs(arc_pieces[i].end_x - arc_pieces[i + 1].start_x) < 1e-9 and
                  abs(arc_pieces[i].end_y - arc_pieces[i + 1].start_y) < 1e-9
                  for i in range(len(arc_pieces) - 1))
    check("arc pieces chain end-to-start", chained)

    # Midpoints lie on the unit circle (radius 1 from origin) -> real curve, not a chord.
    mids_on_circle = all(abs(math.hypot(s.start_x, s.start_y) - 1.0) < 0.02 for s in arc_pieces)
    check("arc pieces follow the circle (not a straight chord)", mids_on_circle)

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  arc tracks linearized into a connected, on-circle polyline")
    print("        on the correct net/layer/width, endpoints preserved")
    print(f"\n6/6 checks passed ({len(arc_pieces)} arc pieces)")
    return 0


if __name__ == '__main__':
    sys.exit(run())
