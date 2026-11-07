#!/usr/bin/env python3
"""#1700: a legal arc outline is not refused because the bounds read an arc more
coarsely than the rings do.

`board_bounds` reads an Edge.Cuts arc with a 5 um chord sag and the rings with
0.5 um (#1404, where copper is cleared against them). Where the coarse chords
miss the apex, a ring stands up to 5 um past the bounds, and #550's check, at
its 1 um tolerance, refused the board: `UntrustworthyOutline` from the emit.

The bounds stay where they are, because copper and poses are laid out from them,
and the check asks the same scan again at the rings' sag. On #1603's fixture with
its east edge redrawn as an arc (none of this asks KiCad):

  1. over ten sags, some of whose apexes the coarse chords miss, every outline
     is trusted and the emit runs;
  2. `board_bounds` is still the coarse reading, vertex for vertex;
  3. the same on an outline a rotated footprint draws;
  4. #550's check still refuses what it exists for: a scan that really missed a
     shape, and a ring past the bounds by more than its 1 um at the rings' sag.

    python3 tests/test_1700_arc_bounds_outline.py
"""
import contextlib
import io
import math
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(REPO, _p))
sys.path.insert(0, TESTS)

import run_utils  # noqa: E402
from test_1603_body_in_board import FIXTURE, quiet  # noqa: E402
from test_1692_bent_edge_body_floor import WEST, arc, drawn, line  # noqa: E402

RUN_ALL_TIMEOUT = 300
#: the apex of each arc stands this far past its 20 mm chord
SAGS = (0.7, 1.0, 1.3, 1.6, 2.0, 2.3, 2.6, 3.1, 3.7, 4.3)
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def past(bounds, rings):
    return max(max(bounds[0] - x, x - bounds[2], bounds[1] - y, y - bounds[3])
               for r in rings for x, y in r)


def minimal(tmp, name, shapes):
    """A bare board whose Edge.Cuts are `shapes`, spelled as the file spells them."""
    path = os.path.join(tmp, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                '  (net 0 "")\n' + shapes + ')\n')
    return path


def fp_shape(kind, *pts):
    words = ' '.join(f'({w} {p[0]} {p[1]})' for w, p in zip(
        ('start', 'mid', 'end') if kind == 'fp_arc' else ('start', 'end'), pts))
    return (f'    ({kind} {words} (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))\n')


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    import kicad_parser as kp
    from placement.floorplan import UntrustworthyOutline, emit_intent, outline_state
    tmp = tempfile.mkdtemp(prefix='t1700_')
    try:
        print('over ten sags:')
        missed = 0
        for sag in SAGS:
            a, m, b = (31 - sag, 5), (31, 15), (31 - sag, 25)
            path = drawn(tmp, f'sag {sag}', [WEST, line((5, 25), (31 - sag, 25)),
                                             line((31 - sag, 5), (5, 5)), arc(a, m, b)])
            pcb = quiet(kp.parse_kicad_pcb, path)
            bi = pcb.board_info
            over = past(bi.board_bounds, bi.board_outlines)
            missed += over > 1e-3
            st = outline_state(pcb, path)
            check(f'sag {sag}: the ring stands {over * 1e3:.2f} um past the bounds, and it is trusted',
                  st['trustworthy'], str(st['problems']))
            try:
                with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    emit_intent(pcb, path)
                said = ''
            except UntrustworthyOutline as exc:
                said = str(exc)
            check('...and the emit runs', not said, said)
            xs = [p[0] for seg in kp._arc_to_segments(a, m, b) for p in seg]
            check('...with board_bounds the coarse reading, vertex for vertex',
                  bi.board_bounds[2] == max(xs), f'{bi.board_bounds[2]} against {max(xs)}')
        check('the sags reach the defect: the coarse chords miss some apexes and not all',
              0 < missed < len(SAGS), f'{missed} of {len(SAGS)}')

        print('an outline a rotated footprint draws:')
        shapes = ('  (footprint "outline" (layer "F.Cu") (at 40 40 90)\n'
                  '    (property "Reference" "BRD" (at 0 0 0) (layer "F.SilkS"))\n'
                  + fp_shape('fp_line', (-15, -10), (15, -10))
                  + fp_shape('fp_line', (-15, 10), (15, 10))
                  + fp_shape('fp_line', (-15, -10), (-15, 10))
                  + fp_shape('fp_arc', (15, -10), (17, 0), (15, 10))
                  + '  )\n')
        path = minimal(tmp, 'footprint', shapes)
        pcb = quiet(kp.parse_kicad_pcb, path)
        rings = pcb.board_info.board_outlines
        check('it chains into a ring', bool(rings), str(len(rings)))
        check('it reaches past the coarse bounds, and is trusted',
              past(pcb.board_info.board_bounds, rings) > 1e-3 and outline_state(pcb, path)['trustworthy'],
              f"{past(pcb.board_info.board_bounds, rings) * 1e3:.2f} um "
              f"{outline_state(pcb, path)['problems']}")

        print("#550's check still refuses what it is for:")
        real = kp._footprint_edge_points
        kp._footprint_edge_points = lambda *_a, **_k: []
        try:
            small = (shapes + '  (gr_line (start 39 39) (end 41 41) (stroke (width 0.1) '
                     '(type default)) (layer "Edge.Cuts"))\n')
            path_short = minimal(tmp, 'short', small)
            kp._footprint_edge_points_by_ref_cached.cache_clear()
            pcb_short = quiet(kp.parse_kicad_pcb, path_short)
            st = outline_state(pcb_short, path_short)
        finally:
            kp._footprint_edge_points = real
        check('a scan that really missed a shape', not st['trustworthy']
              and any('outside board_bounds' in p for p in st['problems']), str(st['problems']))

        sag = 2.0
        path = drawn(tmp, 'poked', [WEST, line((5, 25), (31 - sag, 25)),
                                    line((31 - sag, 5), (5, 5)),
                                    arc((31 - sag, 5), (31, 15), (31 - sag, 25))])

        def poked(by):
            pcb = quiet(kp.parse_kicad_pcb, path)
            ring = pcb.board_info.board_outlines[0]
            i = max(range(len(ring)), key=lambda k: ring[k][0])
            ring[i] = (ring[i][0] + by, ring[i][1])
            pcb.board_info.board_outline = ring
            return outline_state(pcb, path)
        fine = kp.extract_board_bounds(open(path, encoding='utf-8').read())
        check('the fixture for it is an arc board', math.isclose(fine[2], 31.0, abs_tol=0.01), str(fine))
        st = poked(0.0015)
        check('a ring 1.5 um past the scan at its own sag', not st['trustworthy'], str(st['problems']))
        st = poked(0.0005)
        check('...and 0.5 um past it stands inside the 1 um tolerance', st['trustworthy'], str(st['problems']))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
