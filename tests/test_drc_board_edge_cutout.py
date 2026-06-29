#!/usr/bin/env python3
"""check_drc's board-edge clearance check must measure against the real
Edge.Cuts outline (outer ring + interior cutouts), not just the rectangular
bounding box (issue #236). Copper routed INTO an internal cutout/slot sits well
inside the bounding box, so the old bbox-only check never flagged it; KiCad
reports it as copper_edge_clearance.

    python3 tests/test_drc_board_edge_cutout.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from check_drc import run_drc


def _rect_edge(x1, y1, x2, y2):
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    out = []
    for i in range(4):
        a = pts[i]
        b = pts[(i + 1) % 4]
        out.append(f' (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
                   f'(layer "Edge.Cuts") (width 0.1))')
    return "\n".join(out)


def _board(items, cutout=True):
    body = _rect_edge(0, 0, 30, 30)
    if cutout:
        body += "\n" + _rect_edge(10, 10, 20, 20)  # interior cutout 10..20
    return f'(kicad_pcb\n (version 20221018)\n (net 0 "")\n (net 1 "SIG")\n{body}\n{items}\n)'


def _run(items, cutout=True):
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(_board(items, cutout))
        path = f.name
    try:
        v = run_drc(path, clearance=0.2, quiet=True, board_edge_clearance=0.2)
        return [x for x in v if 'board-edge' in x['type']]
    finally:
        os.unlink(path)


SEG_IN_CUTOUT = ' (segment (start 14 14) (end 16 16) (width 0.2) (layer "F.Cu") (net 1) (uuid "a"))'
SEG_INTERIOR = ' (segment (start 3 3) (end 5 5) (width 0.2) (layer "F.Cu") (net 1) (uuid "b"))'
SEG_NEAR_EDGE = ' (segment (start 0.1 5) (end 0.1 8) (width 0.2) (layer "F.Cu") (net 1) (uuid "c"))'
VIA_IN_CUTOUT = ' (via (at 15 15) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1) (uuid "d"))'


def run():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            fails.append(name)

    # A track inside the interior cutout IS flagged when the cutout exists...
    in_cut = _run(SEG_IN_CUTOUT, cutout=True)
    check("track in cutout -> flagged (off-board)",
          len(in_cut) == 1 and in_cut[0]['edge'] == 'off-board')

    # ...but the SAME track with no cutout in the outline is NOT flagged -- this
    # is the bounding-box blind spot the fix closes.
    check("same track, no cutout -> not flagged (was the bbox blind spot)",
          len(_run(SEG_IN_CUTOUT, cutout=False)) == 0)

    # A track well inside the board, clear of edges and cutout -> clean.
    check("clean interior track -> 0 violations", len(_run(SEG_INTERIOR)) == 0)

    # A track hugging the outer edge -> flagged (works in both modes).
    near = _run(SEG_NEAR_EDGE)
    check("track near outer edge -> flagged", len(near) == 1)

    # A via inside the cutout -> flagged off-board.
    via = _run(VIA_IN_CUTOUT)
    check("via in cutout -> flagged (off-board)",
          len(via) == 1 and via[0]['type'] == 'via-board-edge' and via[0]['edge'] == 'off-board')

    print("=" * 60)
    if fails:
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("\n5/5 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
