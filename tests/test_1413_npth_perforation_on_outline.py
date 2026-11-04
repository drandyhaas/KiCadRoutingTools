#!/usr/bin/env python3
"""#1413: a copper-less NPTH drill centred on the outline is not "past" it.

`seeder._holes_outside` counted a drill straddling the outline as past it
(a castellated half-hole was the only exemption), so a declared mouse-bite
perforation on the outer edge was refused at its declared pose in every seed,
while the same hole on a slot's contour (a milled ring, on-board inside) was
accepted. A copper-less NPTH drill that still touches the board is exempt now,
as a half-hole is; a hole wholly off the board, and a plated hole of a copper
pad straddling the edge, still count.

    python3 tests/test_1413_npth_perforation_on_outline.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
import pose_score                                              # noqa: E402
from placement import seeder                                   # noqa: E402
from placement.legality import BoardOutlineGate                # noqa: E402
from run_utils import evidence                                 # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def part(ref, kind, x, y):
    if kind == 'npth':
        pad = '    (pad "" np_thru_hole circle (at 0 0) (size 0.5 0.5) (drill 0.5) (layers "*.Cu" "*.Mask"))\n'
    else:
        pad = '    (pad "1" thru_hole circle (at 0 0) (size 1.2 1.2) (drill 0.5) (layers "*.Cu" "*.Mask"))\n'
    return (f'  (footprint "t:{kind}" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0 0) (layer "F.SilkS"))\n' + pad + '  )\n')


def main():
    tmp = tempfile.mkdtemp(prefix='t1413_')
    try:
        path = os.path.join(tmp, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write('(kicad_pcb (version 20241229) (generator "t1413")\n  (net 0 "")\n'
                     '  (gr_rect (start 0 0) (end 40 30) (layer "Edge.Cuts"))\n'
                     + part('H1', 'npth', 20, 20) + part('P1', 'pth', 20, 20) + ')\n')
        evidence(path, 'synthetic board')
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            pcb = parse_kicad_pcb(path)
            state = pose_score.make_state(pcb, path, clearance=0.25, board_edge_clearance=0.55)
        zero = BoardOutlineGate(pcb.board_info, 0.0)
        out = lambda ref, x, y: seeder._holes_outside(state, zero, ref, x, y, 0.0)   # noqa: E731
        check('an NPTH drill inside is not outside', out('H1', 20, 15) == (1, 0), str(out('H1', 20, 15)))
        check('an NPTH drill centred ON the outer edge is exempt (mouse-bite)',
              out('H1', 0, 15) == (1, 0), str(out('H1', 0, 15)))
        check('an NPTH drill wholly off the board still counts',
              out('H1', -5, 15) == (1, 1), str(out('H1', -5, 15)))
        check('a plated hole of a copper pad on the edge still counts',
              out('P1', 0, 15) == (1, 1), str(out('P1', 0, 15)))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
