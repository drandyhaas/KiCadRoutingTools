#!/usr/bin/env python3
"""Multi-outline boards (issue #304).

A single .kicad_pcb can carry several DISJOINT outer outlines (split keyboards
drawn as left+right halves, panelized boards). The old contour rule -- largest
ring is THE outline, everything else a cutout -- modelled the second half as a
hole: every track in it graded as a board-edge violation (crkbd's pristine
human original: 3479 phantoms) and routing/off-board logic treated it as
outside the board. Contours are now classified by CONTAINMENT, and Edge.Cuts
collection also handles gr_curve beziers, gr_circle rings, and footprint-
embedded fp_* edge shapes (the pieces without which crkbd's outline never
chained closed).

    python3 tests/test_multi_outline_board.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb
from check_drc import run_drc, make_off_board_test


def _rect_edge(x1, y1, x2, y2):
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    return "\n".join(
        f'\t(gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
        f'(stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))'
        for a, b in zip(pts, pts[1:] + pts[:1]))


BOARD = f'''(kicad_pcb (version 20241229) (generator test)
\t(layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
\t(net 0 "")
\t(net 1 "L")
\t(net 2 "R")
{_rect_edge(0, 0, 30, 30)}
{_rect_edge(50, 0, 80, 30)}
{_rect_edge(10, 10, 20, 20)}
\t(footprint "test:conn" (layer "F.Cu") (at 5 5)
\t\t(property "Reference" "J1" (at 0 -2) (layer "F.SilkS"))
\t\t(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "L"))
\t)
\t(footprint "test:conn" (layer "F.Cu") (at 55 5)
\t\t(property "Reference" "J2" (at 0 -2) (layer "F.SilkS"))
\t\t(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 2 "R"))
\t)
\t(segment (start 52 5) (end 78 5) (width 0.2) (layer "F.Cu") (net 2) (uuid "r1"))
\t(segment (start 2 25) (end 28 25) (width 0.2) (layer "F.Cu") (net 1) (uuid "l1"))
)
'''


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
        bi = pcb.board_info

        # containment classification: two disjoint outers + one true cutout
        check("two outer rings", len(bi.board_outlines) == 2)
        check("one cutout (contained ring)", len(bi.board_cutouts) == 1)

        # off-board test: on-board in BOTH halves, off-board between them
        off = make_off_board_test(bi)
        check("point in left half on-board", not off(15, 25))
        check("point in right half on-board", not off(65, 5))
        check("point between halves off-board", off(40, 15))
        check("point in cutout off-board", off(15, 15))

        # DRC: tracks in the second outline are NOT board-edge violations
        v = run_drc(path, clearance=0.2, quiet=True)
        edge = [x for x in v if 'board-edge' in x['type']]
        check("no phantom board-edge violations", edge == [])
    finally:
        os.unlink(path)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
