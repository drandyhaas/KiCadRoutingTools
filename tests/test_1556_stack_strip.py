#!/usr/bin/env python3
"""A strip removes the member of a same-span stack it lists (#1556).

A fanout stub ends in a short grid-snap connector from its off-grid tip P to
the grid point G of P's cell, and a route from the stub could lay a second
P->G connector of its own width on top of it (a stacked twin). A strip list
(the writer's, the output reconcile's, the GUI's) matched a segment by span,
layer and net only and took the FIRST block of the span. When a pass removed
the twin listed second, the strip took the keeper instead; the board-vs-file
output reconcile then removed the twin as file-only copper, and the joint
shipped open (a 0.023 mm SEGMENT-ENDPOINT-GAP). The GUI's set match removed
every member of the span outright. Strips are now counted and
width-preferring (`kicad_writer.match_stack_members`).

Checks:
  C. remove_segments_from_content removes the listed width of a same-span
     stack, falls back to the first block when no width matches, and refuses
     that fallback under match_width
  D. route.py on a board already carrying the stacked twin ships the joint
     closed: one connector on the span, no SEGMENT-ENDPOINT-GAP

    python3 tests/test_1556_stack_strip.py
"""
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence, tool                    # noqa: E402
from synth import RoutingBoard                          # noqa: E402
from kicad_parser import Segment, parse_kicad_pcb       # noqa: E402

FAILS = []
P = (10.0792, 5.0092)       # the stub's off-grid tip
G = (10.1, 5.0)             # the grid point of P's cell (0.1 mm grid)


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def near(a, b, tol=1e-6):
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol


def board(path, connector_reversed=False, twin=False, third_pad=False):
    b = RoutingBoard(w=30, h=10)
    b.pad('J1', 5, 5, 'N')
    b.pad('J2', 20, 5, 'N')
    if third_pad:
        b.pad('J3', 25, 8, 'N')
    b.seg(5, 5, P[0], P[1], 'N', w=0.1)
    a, z = (G, P) if connector_reversed else (P, G)
    b.seg(a[0], a[1], z[0], z[1], 'N', w=0.0762, uid='keep-connector')
    if twin:
        b.seg(P[0], P[1], G[0], G[1], 'N', w=0.1, uid='twin-connector')
        b.seg(G[0], G[1], 20, 5, 'N', w=0.1)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(b.text())
    return path


def _stack_text():
    return ('(kicad_pcb (version 20241229)\n  (net 0 "")\n  (net 1 "N")\n'
            '  (segment (start 10.0792 5.0092) (end 10.1 5.0) (width 0.0762) '
            '(layer "F.Cu") (net 1) (uuid "keep"))\n'
            '  (segment (start 10.0792 5.0092) (end 10.1 5.0) (width 0.1) '
            '(layer "F.Cu") (net 1) (uuid "twin"))\n)\n')


def check_strip():
    print('C. a strip removes the stack member it lists')
    from kicad_writer import remove_segments_from_content
    twin = Segment(P[0], P[1], G[0], G[1], 0.1, 'F.Cu', 1)
    out, n = remove_segments_from_content(_stack_text(), [twin])
    check('listed 0.1 member: removed, the 0.0762 keeper stays',
          n == 1 and '"keep"' in out and '"twin"' not in out, f'n={n}')
    other = Segment(G[0], G[1], P[0], P[1], 0.2, 'F.Cu', 1)
    out, n = remove_segments_from_content(_stack_text(), [other])
    check('a width on no member: the first block of the span goes (counted)',
          n == 1 and '"keep"' not in out and '"twin"' in out, f'n={n}')
    left = []
    out, n = remove_segments_from_content(_stack_text(), [other],
                                          unmatched_out=left, match_width=True)
    check('match_width: no member of that width, nothing removed, reported',
          n == 0 and '"keep"' in out and '"twin"' in out and left == [other],
          f'n={n}, unmatched={len(left)}')
    out, n = remove_segments_from_content(_stack_text(), [twin, twin])
    check('two listings of one width: both members go', n == 2, f'n={n}')


def check_cli(tmp):
    print('D. route.py ships the joint closed over a stacked twin')
    src = board(os.path.join(tmp, 'twin.kicad_pcb'), twin=True, third_pad=True)
    out = os.path.join(tmp, 'twin_out.kicad_pcb')
    r = subprocess.run([sys.executable, tool('route.py'), src, out, '--nets', 'N',
                        '--grid-step', '0.1', '--track-width', '0.15',
                        '--clearance', '0.1', '--layers', 'F.Cu', 'B.Cu'],
                       capture_output=True, text=True, timeout=600)
    check('route.py exits 0', r.returncode == 0, r.stderr[-400:])
    pcb = parse_kicad_pcb(evidence(out))
    on_span = [s for s in pcb.segments
               if {tuple(round(v, 4) for v in (s.start_x, s.start_y)),
                   tuple(round(v, 4) for v in (s.end_x, s.end_y))} == {P, G}]
    check('exactly one connector left on the P-G span', len(on_span) == 1,
          f'{len(on_span)}')
    d = subprocess.run([sys.executable, tool('check_drc.py'), out],
                       capture_output=True, text=True, timeout=600)
    check('check_drc ran', 'DRC' in d.stdout, d.stderr[-400:])
    check('no SEGMENT-ENDPOINT-GAP', 'SEGMENT-ENDPOINT-GAP' not in d.stdout)


def main():
    with tempfile.TemporaryDirectory() as tmp:
        check_strip()
        check_cli(tmp)
    if FAILS:
        print(f'FAIL: {len(FAILS)} check(s): {FAILS}')
        sys.exit(1)
    print('PASS: #1556 stack strip')


if __name__ == '__main__':
    main()
