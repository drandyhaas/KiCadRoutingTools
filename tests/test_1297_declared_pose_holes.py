#!/usr/bin/env python3
"""#1297: a declared pose is judged on its holes, not their bounding squares,
and a castellated half-hole on the edge or a hole co-located with another
part's is not a refusal.

`_fixed_pose_check`'s overhang branch counted every drilled pad's bounding
square past the outline with no castellated exemption, so rp2350's Teensy
U8 was refused for "33 of 66 drill hole(s) past the outline" -- all 33 its
half-holes -- and every seed was then illegal. And `_drill_conflict` refused
CM5's Module301/302 (one module drawn as two footprints) under a courtyard
waiver for holes at one centre, which KiCad grades `holes_co_located`, a
warning.

Checks, on a board built here (a 20 x 10 mm outline, one footprint with a
castellated half-hole and a plain hole on its top edge):
  1. `_holes_outside`: both holes counted, only the plain one outside; both
     inside once the part is moved onto the board.
  2. `_drill_conflict`: two parts with holes at one centre do not conflict;
     0.3 mm apart they do.

    python3 tests/test_1297_declared_pose_holes.py
"""
import os
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement.legality import BoardOutlineGate                # noqa: E402
from placement.seeder import _drill_conflict, _holes_outside   # noqa: E402

EDGE = ''.join(
    f'  (gr_line (start {a}) (end {b}) (stroke (width 0.1) (type default)) '
    f'(layer "Edge.Cuts"))\n'
    for a, b in (('0 0', '20 0'), ('20 0', '20 10'), ('20 10', '0 10'),
                 ('0 10', '0 0')))

BOARD = f"""(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "")
{EDGE}  (footprint "M:Module" (layer "F.Cu") (at 10 0)
    (property "Reference" "M1" (at 0 -3) (layer "F.SilkS"))
    (pad "1" thru_hole circle (at -2 0) (size 1.6 1.6) (drill 1)
      (layers "*.Cu" "*.Mask") (property pad_prop_castellated) (net 0 ""))
    (pad "2" thru_hole circle (at 2 0) (size 1.6 1.6) (drill 1)
      (layers "*.Cu" "*.Mask") (net 0 ""))
  )
)
"""


def _board():
    d = tempfile.mkdtemp(prefix='t1297_')
    path = os.path.join(d, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    return parse_kicad_pcb(path), path


def test_castellated_half_hole_is_exempt():
    pcb, _path = _board()
    state = SimpleNamespace(pcb_data=pcb)
    gate = BoardOutlineGate(pcb.board_info, 0.0)
    assert _holes_outside(state, gate, 'M1', 10, 0, 0) == (2, 1), (
        _holes_outside(state, gate, 'M1', 10, 0, 0))
    assert _holes_outside(state, gate, 'M1', 10, 5, 0) == (2, 0)
    print("  PASS: the castellated half-hole is exempt, the plain hole is not")


def test_co_located_holes_are_not_a_conflict():
    pcb, path = _board()
    fp = pcb.footprints['M1']
    pcb.footprints['M2'] = fp
    state = SimpleNamespace(pcb_data=pcb, pcb_file=path)
    assert _drill_conflict(state, 'M1', (10, 5, 0), 'M2', (10, 5, 0)) is None
    assert _drill_conflict(state, 'M1', (10, 5, 0), 'M2', (10.3, 5, 0)), (
        "holes 0.3 mm apart overlap and must still be refused")
    print("  PASS: holes at one centre pass, overlapping ones do not")


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        fn()
    print(f"{len(fns)} test(s) passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
