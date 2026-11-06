"""The placer seats a part on a courtyard that is not a rectangle box by box.

A courtyard drawn as an L or a U -- one stepped outline, or several shapes --
was read by the placer as its bounding box, so nothing could stand in its
notch although check_assembly (polygon exact) grades a part there clean.
Now `parser.extract_courtyard_boxes` gives such a courtyard as disjoint
boxes, `quench._ShapedPart.rect` answers them as a `legality.FarSide` (its
own value the bounding box), and the anneal's overlap
(`Annealer._shape_correction`), the halo, the seat gate and the violation
measure read the boxes. A rectangle, and a single shape that is not
axis-aligned, are the box they always were.

    python3 tests/test_placer_courtyard_boxes.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _rect(x0, y0, x1, y1, layer='F.CrtYd'):
    return (f'    (fp_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.05)'
            f' (type default)) (layer "{layer}"))\n')


def _lines(pts, layer='F.CrtYd'):
    """A closed chain of fp_lines through `pts`."""
    return ''.join(f'    (fp_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (stroke'
                   f' (width 0.05) (type default)) (layer "{layer}"))\n'
                   for a, b in zip(pts, pts[1:] + pts[:1]))


#: a U drawn as three rectangles: two arms and the bottom between them; its
#: notch is 4 mm wide and 7.5 mm deep, open at the top
U_SHAPES = [(-9.5, -6.5, -2, 6.5), (2, -6.5, 9.5, 6.5), (-2, 1, 2, 6.5)]


def _block(ref, x, y, courtyard, locked=True):
    return (f'  (footprint "t:BLOCK" (layer "F.Cu") (at {x} {y})\n'
            + ('    (locked yes)\n' if locked else '')
            + f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            + courtyard
            + '    (pad "1" smd rect (at -5 -4) (size 1 1) (layers "F.Cu") (net 1 "N1"))\n'
            '    (pad "2" smd rect (at 5 -4) (size 1 1) (layers "F.Cu") (net 2 "N2")))\n')


def _chip(ref, x, y, courtyard=None):
    return (f'  (footprint "t:R_0603" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            + (courtyard or _rect(-1.48, -0.73, 1.48, 0.73))
            + '    (pad "1" smd rect (at -0.775 0) (size 0.9 0.95) (layers "F.Cu") (net 1 "N1"))\n'
            '    (pad "2" smd rect (at 0.775 0) (size 0.9 0.95) (layers "F.Cu") (net 2 "N2")))\n')


def _board(path, parts, size=(20, 14)):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n  (net 1 "N1")\n  (net 2 "N2")\n'
                 f'  (gr_rect (start 0 0) (end {size[0]} {size[1]}) (stroke (width 0.1)'
                 ' (type default)) (layer "Edge.Cuts"))\n' + ''.join(parts) + ')\n')
    with open(path[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        fh.write('{"board": {"design_settings": {}}}')
    return path


#: the same U as one closed outline, as a library footprint draws it
U_OUTLINE = [(-9.5, -6.5), (-2, -6.5), (-2, 1), (2, 1), (2, -6.5), (9.5, -6.5),
             (9.5, 6.5), (-9.5, 6.5)]


def _u_board(path, shaped, locked=True, r1=(10, 11)):
    """U1 fills the 20 x 14 board less its 0.5 mm margin; the only room left
    for R1 is U1's notch, when U1 is drawn as the U: three rectangles
    (`shaped` True) or one outline (`shaped` 'outline')."""
    crt = (_lines(U_OUTLINE) if shaped == 'outline' else
           ''.join(_rect(*b) for b in U_SHAPES) if shaped else _rect(-9.5, -6.5, 9.5, 6.5))
    return _board(path, [_block('U1', 10, 7, crt, locked), _chip('R1', *r1)])


def _state(path):
    import pose_score
    return pose_score.make_state(parse_kicad_pcb(path), path, clearance=0.25,
                                 board_edge_clearance=0.5,
                                 estimate_courtyards=True)


def _r(boxes, k=9):
    return sorted(tuple(round(v, k) + 0.0 for v in b) for b in boxes)


def test_parser(td):
    """Which drawings are several shapes, and their boxes."""
    from placement.parser import extract_courtyard_boxes
    L_RECTS = _rect(0, 0, 6, 2) + _rect(0, 0, 2, 6)
    parts = [
        _chip('U', 0, 0, ''.join(_rect(*b) for b in U_SHAPES)),
        _chip('L', 0, 0, L_RECTS),
        # the same L drawn as two overlapping closed chains of lines
        _chip('LL', 0, 0, _lines([(0, 0), (6, 0), (6, 2), (0, 2)])
              + _lines([(0, 0), (2, 0), (2, 6), (0, 6)])),
        # two halves of one rectangle: one box
        _chip('H', 0, 0, _rect(-2, -1, 0, 1) + _rect(0, -1, 2, 1)),
        # the L as ONE closed chain of lines, and as one fp_poly
        _chip('P', 0, 0, _lines([(0, 0), (6, 0), (6, 2), (2, 2), (2, 6), (0, 6)])),
        _chip('Q', 0, 0, '    (fp_poly (pts (xy 0 0) (xy 6 0) (xy 6 2) (xy 2 2)'
              ' (xy 2 6) (xy 0 6)) (stroke (width 0.05) (type default)) (fill no)'
              ' (layer "F.CrtYd"))\n'),
        # a rectangle drawn as eight half-edges: one box
        _chip('E', 0, 0, _lines([(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2),
                                 (0, 2), (0, 1)])),
        # one chamfered outline (an electrolytic's): its bounding box
        _chip('C', 0, 0, _lines([(-2, -1), (2, -1), (3, 0), (2, 1), (-2, 1)])),
        # a ring (two concentric circles): its hole is no notch
        _chip('O', 0, 0, '    (fp_circle (center 0 0) (end 3 0) (stroke (width 0.05)'
              ' (type default)) (layer "F.CrtYd"))\n'
              '    (fp_circle (center 0 0) (end 1 0) (stroke (width 0.05)'
              ' (type default)) (layer "F.CrtYd"))\n'),
        # a disc beside a square: the disc covered outward, slab by slab
        _chip('D', 0, 0, _rect(-4, -1, -2, 1) + '    (fp_circle (center 2 0) (end 4 0)'
              ' (stroke (width 0.05) (type default)) (layer "F.CrtYd"))\n'),
    ]
    path = _board(os.path.join(td, 'parse.kicad_pcb'), parts)
    got = extract_courtyard_boxes(path)
    ok('a U of three rectangles is its three boxes',
       _r(got.get('U', {}).get('F', ())) == _r(U_SHAPES), got.get('U'))
    want_l = [(0, 0, 2, 6), (2, 0, 6, 2)]
    ok('an L of two overlapping rectangles: two DISJOINT boxes',
       _r(got.get('L', {}).get('F', ())) == _r(want_l), got.get('L'))
    ok('... and the same drawn as closed chains of lines',
       _r(got.get('LL', {}).get('F', ())) == _r(want_l), got.get('LL'))
    ok('an L drawn as one closed chain of lines: the same two boxes',
       _r(got.get('P', {}).get('F', ())) == _r(want_l), got.get('P'))
    ok('... and as one fp_poly',
       _r(got.get('Q', {}).get('F', ())) == _r(want_l), got.get('Q'))
    ok('two halves of a rectangle, a rectangle of eight edges, a chamfered '
       'outline and a ring are absent (the bounding box, as before)',
       not any(r in got for r in ('H', 'E', 'C', 'O')), sorted(got))
    from shapely.geometry import box
    from shapely.ops import unary_union
    from placement.parser import extract_courtyard_shapes
    d = got.get('D', {}).get('F', ())
    cover = unary_union([box(*b) for b in d])
    drawn = extract_courtyard_shapes(path)['D']['F'][0]     # what the grade reads
    ok('a disc beside a square is covered, never cut: the outline the grade '
       'reads lies inside the boxes',
       len(d) >= 3 and cover.buffer(1e-9).contains(drawn), d)
    ok('... and the cover is tighter than the disc\'s bounding box',
       cover.area < 16.0 + 4.0 - 0.5, round(cover.area, 3))


#: The L's boxes, (0,0,2,6) and (2,0,6,2), at (10, 20) at each quarter turn,
#: worked by hand with KiCad's turn (x, y) -> (y, -x) at 90 degrees; the
#: probes are local (3,3,4,4) in the notch 1 mm from each arm, and local
#: (1,3,3,4) 1 mm2 into the upright arm, turned the same way.
TURNS = {
    0.0: ([(10, 20, 12, 26), (12, 20, 16, 22)], (13, 23, 14, 24), (11, 23, 13, 24)),
    90.0: ([(10, 18, 16, 20), (10, 14, 12, 18)], (13, 16, 14, 17), (13, 17, 14, 19)),
    180.0: ([(8, 14, 10, 20), (4, 18, 8, 20)], (6, 16, 7, 17), (7, 16, 9, 17)),
    270.0: ([(4, 20, 10, 22), (8, 22, 10, 26)], (6, 23, 7, 24), (6, 21, 7, 23)),
}


def test_turns(td):
    from placement import legality
    path = _board(os.path.join(td, 'turns.kicad_pcb'),
                  [_block('L1', 10, 20, _rect(0, 0, 6, 2) + _rect(0, 0, 2, 6), False)],
                  size=(40, 40))
    part = _state(path).parts['L1']
    for rot, (boxes, notch, into) in TURNS.items():
        part.ensure_rotation(rot)
        r = part.rect(10, 20, rot)
        ok(f'{rot:g} deg: the boxes', _r(legality.far_boxes(r)) == _r(boxes),
           _r(legality.far_boxes(r)))
        ok(f'{rot:g} deg: its own value is the bounding box',
           tuple(round(v, 9) for v in r) == tuple(map(float, (
               min(b[0] for b in boxes), min(b[1] for b in boxes),
               max(b[2] for b in boxes), max(b[3] for b in boxes)))), tuple(r))
        g = legality.far_gap(r, notch)
        ok(f'{rot:g} deg: a box in the notch is 1 mm clear (the bounding box '
           'says 3 mm inside)', abs(g - 1.0) < 1e-9
           and legality.rect_gap(tuple(r), notch) < -2.9, g)
        a = legality.far_overlap_area(r, into)
        ok(f'{rot:g} deg: a box 1 mm into the upright overlaps 1 mm2 (the '
           'bounding box: 2)', abs(a - 1.0) < 1e-9
           and abs(legality.rect_overlap_area(tuple(r), into) - 2.0) < 1e-9, a)
        ok(f'{rot:g} deg: the grade keeps the bounding box',
           type(part.grade_rect(10, 20, rot)) is tuple
           and _r([part.grade_rect(10, 20, rot)]) == _r([r]),
           (part.grade_rect(10, 20, rot), tuple(r)))


def test_gates(td):
    """R1 in the notch: the seat gate admits it and the violation measure
    reads 0 on the U; on its bounding rectangle both refuse it. Shifted 1 mm
    into the left arm it is short by the arm's 0.48 mm overlap plus the
    0.25 mm clearance (the bounding box: its 3.23 mm inside plus 0.25)."""
    from placement.seeder import pose_ok
    u = _state(_u_board(os.path.join(td, 'gu.kicad_pcb'), True))
    r = _state(_u_board(os.path.join(td, 'gr.kicad_pcb'), False))
    ok('U: R1 seats in the notch', pose_ok(u, 'R1', 10.0, 3.0, 0.0, set()))
    ok('U: ... and its violation there is (0, 0)',
       u.violation_parts('R1', 10.0, 3.0, 0.0) == (0.0, 0.0),
       u.violation_parts('R1', 10.0, 3.0, 0.0))
    ok('rectangle: refused there', not pose_ok(r, 'R1', 10.0, 3.0, 0.0, set()))
    v_u = u.violation_parts('R1', 9.0, 3.0, 0.0)[1]
    v_r = r.violation_parts('R1', 9.0, 3.0, 0.0)[1]
    ok('1 mm into the arm: short 0.73 mm on the U', abs(v_u - 0.73) < 1e-9, v_u)
    ok('... 3.48 mm on the rectangle', abs(v_r - 3.48) < 1e-9, v_r)
    ok('... and refused on the U', not pose_ok(u, 'R1', 9.0, 3.0, 0.0, set()))
    # R1's courtyard 0.2 mm, then 0.3 mm, from the left arm (x = 8)
    ok('0.2 mm from the arm: refused, 0.05 mm short',
       not pose_ok(u, 'R1', 9.68, 3.0, 0.0, set())
       and abs(u.violation_parts('R1', 9.68, 3.0, 0.0)[1] - 0.05) < 1e-9,
       u.violation_parts('R1', 9.68, 3.0, 0.0))
    ok('0.3 mm from the arm: admitted', pose_ok(u, 'R1', 9.78, 3.0, 0.0, set()))
    ok('turned a quarter (1.46 x 2.96 in the 4 x 7.5 notch): admitted',
       pose_ok(u, 'R1', 10.0, 3.0, 90.0, set()))


def _annealer(state):
    """An Annealer over every part, on a stand-in for `simple.Model` that
    carries what `Annealer.overlap` reads: the parts' committed rects and
    their index."""
    import types
    import numpy as np
    from placement import simple
    from placement.simple_anneal import Annealer
    names = sorted(state.parts)
    ix = simple.SpatialIndex()
    for n in names:
        ix.insert(n, state.parts[n].rect())
    model = types.SimpleNamespace(
        state=state, groups=[], _ixp=ix, _pid=names.index,
        _CR=np.asarray([state.parts[n].rect() for n in names], dtype=float))
    return Annealer(model, names, seed=0)


def test_anneal_overlap(td):
    """The anneal's overlap of R1 (grown by half the 0.25 mm clearance)
    against U1: 0 in the notch; 1 mm into the left arm the grown R1
    (7.395..10.605 x 2.145..3.855) and the grown arm (..8.125) share
    0.73 x 1.71 = 1.2483 mm2, where the bounding box swallows all of R1's
    3.21 x 1.71 = 5.4891. The incremental sum (Rust, then the correction)
    and the full-board sum agree."""
    from placement.simple_anneal import _place
    for shaped, notch, into in ((True, 0.0, 1.2483), (False, 3.21 * 1.71, 5.4891)):
        st = _state(_u_board(os.path.join(td, f'a{int(shaped)}.kicad_pcb'), shaped,
                             r1=(10, 3)))
        an = _annealer(st)
        tag = 'U' if shaped else 'rectangle'
        ok(f'{tag}: U1 is {"" if shaped else "not "}priced box by box',
           an.shaped == (frozenset({'U1'}) if shaped else frozenset()), an.shaped)
        got = an.overlap(['R1'])
        ok(f'{tag}: R1 in the notch overlaps {notch:.4f} mm2', abs(got - notch) < 1e-9, got)
        hits = [b for b, _B in an._hits('R1', an._grown('R1'), {})]
        ok(f'{tag}: the repair\'s shove {"finds nothing to push" if shaped else "pushes U1"}',
           hits == ([] if shaped else ['U1']), hits)
        _place(st, 'R1', 9.0, 3.0, 0.0)
        inc, full = an.overlap(['R1']), an.overlap()
        ok(f'{tag}: 1 mm into the arm, {into} mm2', abs(inc - into) < 1e-9, inc)
        ok(f'{tag}: ... the full-board sum the same', abs(full - inc) < 1e-9, (full, inc))
        if shaped:
            # across the seam of the left arm and the bottom (x = 8): each
            # grown box takes 1.73 x 1.71 of R1's grown 3.21 x 1.71, and the
            # seam band is counted once -- 5.4891, not the boxes' 5.9166
            _place(st, 'R1', 8.0, 10.0, 0.0)
            got = an.overlap(['R1'])
            ok('U: across a seam, R1\'s whole grown box once (5.4891 mm2)',
               abs(got - 5.4891) < 1e-9, got)
    st = _state(_u_board(os.path.join(td, 'a2.kicad_pcb'), True, r1=(10, 3)))
    an = _annealer(st)
    for rot, x, y in ((90.0, 10.0, 7.0), (180.0, 10.0, 7.0), (270.0, 10.0, 7.0)):
        # U1 turned about its origin; R1 placed in the notch where it now opens
        _place(st, 'U1', x, y, rot)
        st.parts['U1'].ensure_rotation(rot)
        r1 = {90.0: (6.0, 7.0, 90.0), 180.0: (10.0, 11.0, 0.0), 270.0: (14.0, 7.0, 90.0)}[rot]
        st.parts['R1'].ensure_rotation(r1[2])
        _place(st, 'R1', *r1)
        boxes = sorted(st.parts['U1'].rect().boxes)
        inc = an.overlap(['R1', 'U1']) - an._outside('U1')
        ok(f'U turned {rot:g}: R1 in the turned notch overlaps nothing',
           abs(inc) < 1e-9 and abs(an.overlap() - an._outside('U1')) < 1e-9,
           (inc, boxes))


def test_halo(td):
    """`Model._halo_raw` with R1 given a 0.5 mm front on every side (U1 has
    none), worked by hand. In the notch at (10, 3) each box of U1 leaves R1
    clear on the axis that separates them best (0.52 mm to each arm against
    the 0.5 mm front): 0. The bounding box contains R1, so its y axis is
    short 0.5 + 3.23 mm: 3.73**2 = 13.9129. 0.2 mm from the left arm
    (x = 9.68) the arm's x axis is short 0.5 - 0.2: 0.3**2 = 0.09."""
    import types
    from placement import simple
    front = (0.5, -10.0, 10.0)
    for shaped, cases in ((True, ((10.0, 0.0), (9.68, 0.09))),
                          (False, ((10.0, 3.73 ** 2),))):
        st = _state(_u_board(os.path.join(td, f'h{int(shaped)}.kicad_pcb'), shaped))
        m = types.SimpleNamespace(state=st, halo_weight=1.0)
        m.fronts = lambda r: (front,) * 4 if r == 'R1' else (None,) * 4
        m._halo_rects = lambda *a: simple.Model._halo_rects(m, *a)
        for x, want in cases:
            st.parts['R1'].x, st.parts['R1'].y = x, 3.0
            got = simple.Model._halo_raw(m, 'R1', 'U1')
            ok(f'{"U" if shaped else "rectangle"}: halo with R1 at x {x:g} is {want:g}',
               abs(got - want) < 1e-9, got)


def _place_simple(td, board, seed='0', moves=None, illegal=None):
    """place_simple on `board`; `illegal`: the part it must report left
    illegal (exit 4), else it must succeed."""
    intent = board[:-len('.kicad_pcb')] + '.intent.json'
    check([sys.executable, '-X', 'utf8', tool('check_floorplan.py'), board,
           '--emit-intent', intent, '--allow-unplaced'], accept=True)
    evidence(intent, 'the emitted intent')
    groups = os.path.join(td, 'groups.json')
    with open(groups, 'w', encoding='utf-8') as fh:
        json.dump({'schema': 1, 'groups': [{'name': 'g', 'refs': ['U1', 'R1']}]}, fh)
    out = board[:-len('.kicad_pcb')] + '.placed.kicad_pcb'
    argv = [sys.executable, '-X', 'utf8', tool('place_simple.py'), board, out,
            '--groups', groups, '--intent', intent, '--seed', seed]
    if moves:
        argv += ['--moves', str(moves)]
    r = (check(argv, accept=True) if illegal is None else
         check(argv, code=4, refuse=f'{illegal} is left illegal'))
    evidence(out, 'the placed board')
    summary = json.loads(r.stdout.split('JSON_SUMMARY:', 1)[1].splitlines()[0])
    return out, summary


def test_place_simple_notch(td):
    """End to end, seed 0: R1 seated in U1's notch, graded clean; on the
    bounding rectangle the same board leaves R1 illegal."""
    for shaped, locked in ((True, True), (True, False), ('outline', True),
                           (False, True)):
        tag = ({True: 'U', 'outline': 'U outline'}.get(shaped, 'rectangle')
               + (', locked' if locked else ', movable'))
        stem = f'p{shaped}{int(locked)}'
        board = _u_board(os.path.join(td, stem + '.kicad_pcb'), shaped, locked)
        out, summary = _place_simple(td, board, illegal=None if shaped else 'R1')
        pcb = parse_kicad_pcb(out)
        u, r = pcb.footprints['U1'], pcb.footprints['R1']
        js = os.path.join(td, f'asm{stem}.json')
        argv = [sys.executable, '-X', 'utf8', tool('check_assembly.py'), out,
                '--baseline', board, '--json', js]
        if shaped:
            check(argv, accept=True)
        else:
            check(argv, code=4, refuse='NOT BUILDABLE')
        doc = json.load(open(js))
        if shaped:
            # R1's centre in the notch, U1's local (-2..2, -6.5..1)
            import math
            t = math.radians(u.rotation)
            dx, dy = r.x - u.x, r.y - u.y
            lx, ly = dx * math.cos(t) - dy * math.sin(t), dx * math.sin(t) + dy * math.cos(t)
            ok(f'{tag}: R1 seated in the notch',
               summary['illegal'] == [] and -2 < lx < 2 and -6.5 < ly < 1,
               (summary['illegal'], round(lx, 3), round(ly, 3), u.rotation))
            ok(f'{tag}: check_assembly grades it buildable, no courtyard pair',
               doc.get('buildable') is True and doc.get('courtyard_blocking') == 0,
               (doc.get('buildable'), doc.get('courtyard_blocking')))
        else:
            ok(f'{tag}: R1 has nowhere legal to go', summary['illegal'] == ['R1']
               and 'courtyard vs U1' in summary['illegal_why'].get('R1', ''),
               summary['illegal_why'])
            ok(f'{tag}: ... and is graded NOT buildable', doc.get('buildable') is False,
               doc.get('buildable'))


def test_one_box_unchanged(td):
    """A courtyard drawn as two halves of its rectangle is one box: the same
    board with it drawn as one rectangle places to the same poses and cost."""
    def parts(halves):
        crt = (_rect(-9.5, -6.5, 0, 6.5) + _rect(0, -6.5, 9.5, 6.5)) if halves else \
            _rect(-9.5, -6.5, 9.5, 6.5)
        return [_block('U1', 8, 10, crt, False), _chip('R1', 30, 5),
                _chip('R2', 5, 25, _rect(-1, -2, 0, 2) + _rect(0, -2, 1, 2))]
    got = []
    for halves in (False, True):
        sub = os.path.join(td, f'h{int(halves)}')
        os.makedirs(sub)
        board = _board(os.path.join(sub, 'b.kicad_pcb'), parts(halves), size=(36, 30))
        out, summary = _place_simple(sub, board, seed='3', moves=1500)
        pcb = parse_kicad_pcb(out)
        got.append(([(r, f.x, f.y, f.rotation) for r, f in sorted(pcb.footprints.items())],
                    summary['final']['cost']))
    ok('two halves place exactly as one rectangle (poses, cost)', got[0] == got[1], got)


#: esp_prog piled (make_unplaced), its intent emitted, four groups of four,
#: placed with `--seed 0 --moves 3000`: recorded at 95e6dc1d3, before
#: courtyards were read as boxes. No part of esp_prog draws a courtyard that
#: is not a rectangle, so nothing may change.
ESP_GROUPS = [['C1', 'C2', 'C3', 'C4'], ['CON1', 'CON2', 'Q1', 'Q2'],
              ['R1', 'R2', 'R3', 'R4'], ['U1', 'U2', 'USB1', 'Y1']]
ESP_GOLDEN = [
    ["#00000000-0000-0000-0000-00005a3b5201", 115.25, 95.75, 90.0],
    ["#00000000-0000-0000-0000-00005d8c51dd", 119.2, 100.9, 0.0],
    ["#00000000-0000-0000-0000-00005e7dd057", 130.0, 99.0, 180.0],
    ["C1", 137.1088, 95.7717, 180.0], ["C2", 119.386, 95.5547, 0.0],
    ["C3", 134.9458, 96.0473, 90.0], ["C4", 121.2726, 93.9987, 90.0],
    ["CON1", 115.5474, 96.1767, 270.0], ["CON2", 130.6921, 92.9169, 0.0],
    ["Q1", 128.9671, 95.9681, 0.0], ["Q2", 128.1094, 102.1348, 180.0],
    ["R1", 128.1937, 100.1085, 180.0], ["R2", 126.8974, 99.0602, 90.0],
    ["R3", 126.2568, 96.9044, 270.0], ["R4", 125.1968, 95.6202, 180.0],
    ["Ref*", 130.6135, 99.0469, 0.0], ["Ref*~2", 132.9093, 98.1712, 0.0],
    ["U1", 121.9208, 100.775, 180.0], ["U2", 133.1643, 102.3094, 0.0],
    ["USB1", 140.0393, 100.868, 0.0], ["Y1", 119.3341, 93.0908, 90.0]]
ESP_GOLDEN_COST = 400.99956625781584


def test_existing_board_unchanged(td):
    pile = os.path.join(td, 'pile.kicad_pcb')
    check([sys.executable, '-X', 'utf8', os.path.join(
        ROOT, '.claude', 'skills', 'plan-pcb-placement', 'scripts', 'make_unplaced.py'),
        os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'), pile], accept=True)
    evidence(pile, 'the piled board')
    intent = os.path.join(td, 'intent.json')
    check([sys.executable, '-X', 'utf8', tool('check_floorplan.py'), pile,
           '--allow-unplaced', '--emit-intent', intent], accept=True)
    evidence(intent, 'the emitted intent')
    groups = os.path.join(td, 'groups.json')
    with open(groups, 'w', encoding='utf-8') as fh:
        json.dump({'schema': 1, 'groups': [{'name': f'g{4 * i}', 'refs': g}
                                           for i, g in enumerate(ESP_GROUPS)]}, fh)
    out = os.path.join(td, 's0.kicad_pcb')
    r = check([sys.executable, '-X', 'utf8', tool('place_simple.py'), pile, out,
               '--groups', groups, '--intent', intent, '--seed', '0', '--moves', '3000',
               '--jobs', '1'], accept=True)
    evidence(out, 'the placed board')
    pcb = parse_kicad_pcb(out)
    got = sorted([r_, round(f.x, 4), round(f.y, 4), round(f.rotation % 360, 4)]
                 for r_, f in pcb.footprints.items())
    ok('esp_prog seed 0: every pose as recorded before', got == ESP_GOLDEN,
       [g for g in got if g not in ESP_GOLDEN])
    cost = json.loads(r.stdout.split('JSON_SUMMARY:', 1)[1].splitlines()[0])['final']['cost']
    ok('... and the same cost, to the bit', cost == ESP_GOLDEN_COST, cost)


def main():
    with tempfile.TemporaryDirectory(prefix='tboxes_') as td:
        for name in ('parser', 'turns', 'gates', 'anneal_overlap', 'halo', 'place_simple_notch',
                     'one_box_unchanged', 'existing_board_unchanged'):
            sub = os.path.join(td, name)
            os.makedirs(sub)
            print(name)
            globals()['test_' + name](sub)
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
