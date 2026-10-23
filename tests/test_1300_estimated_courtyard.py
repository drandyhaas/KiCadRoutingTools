"""#1300: a part that draws no courtyard gets an ESTIMATED one.

On multichannel_mixer every place_simple seed graded NOT BUILDABLE on four
courtyard-blocking pairs (C31<->RV12, C2<->RV10, ...): the radial
electrolytics there draw a 5 mm .Fab body and no courtyard, so the placer
seated them on their 4.0 x 1.5 mm PAD box and packed caps into each other's
bodies, while check_assembly gates the body.

`body.estimated_courtyards` derives the courtyard such a part would have had:
its occupancy -- what check_assembly grades it on (the .Fab body, else silk
with the pads, else the pads, each united with the pads) -- with no library
courtyard margin round it, since the board declares none. It is a placement
INPUT and never a grade: the graders keep reading what is drawn.

    python3 tests/test_1300_estimated_courtyard.py
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import body, legality                         # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})"
                                                    if detail and not ok
                                                    else ''))
    if not ok:
        FAILS.append(name)


def _board(td, body_text, size=(44, 24)):
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
            f'  (gr_rect (start 0 0) (end {size[0]} {size[1]}) (stroke'
            ' (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
            + body_text + ')\n')
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def _radial(ref, x, y):
    """multichannel_mixer's CAPPRB254-500X840: two drilled pads 2.54 mm
    apart (a 4.0 x 1.5 mm pad box) inside a 5 mm .Fab circle, no courtyard."""
    return (f'  (footprint "t:CAPPRB254-500X840" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_circle (center 0 0) (end 2.5 0) (stroke (width 0.1)'
            ' (type default)) (layer "F.Fab"))\n'
            '    (pad "1" thru_hole rect (at -1.27 0) (size 1.5 1.5)'
            ' (drill 0.8) (layers "*.Cu" "*.Mask") (net 1 "N1"))\n'
            '    (pad "2" thru_hole circle (at 1.27 0) (size 1.5 1.5)'
            ' (drill 0.8) (layers "*.Cu" "*.Mask") (net 2 "N2")))\n')


def _smd(ref, name, x, y, pads, fab, court=None):
    """An SMD part: `pads` [(x, y, w, h)], a .Fab rect `fab` (x0, y0, x1,
    y1), and a courtyard rect when `court` is given."""
    out = (f'  (footprint "t:{name}" (layer "F.Cu") (at {x} {y})\n'
           f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
           f'    (fp_rect (start {fab[0]} {fab[1]}) (end {fab[2]} {fab[3]})'
           ' (stroke (width 0.1) (type default)) (layer "F.Fab"))\n')
    if court is not None:
        out += (f'    (fp_rect (start {court[0]} {court[1]}) (end {court[2]}'
                f' {court[3]}) (stroke (width 0.05) (type default))'
                ' (layer "F.CrtYd"))\n')
    for i, (px, py, w, h) in enumerate(pads, 1):
        out += (f'    (pad "{i}" smd rect (at {px} {py}) (size {w} {h})'
                f' (layers "F.Cu") (net {1 + i % 2} "N{1 + i % 2}"))\n')
    return out + '  )\n'


def _frame(ref, x, y):
    """A pin frame (a socketed module): a 40 x 20 mm .Fab outline with its
    four drilled pins at the corners and nothing inside -- a container,
    which other parts sit under by design and no consumer spaces as a body."""
    pins = ''.join(
        f'    (pad "{i}" thru_hole circle (at {px} {py}) (size 1.7 1.7)'
        f' (drill 1.0) (layers "*.Cu" "*.Mask") (net 1 "N1"))\n'
        for i, (px, py) in enumerate([(-19, -9), (19, -9), (-19, 9),
                                      (19, 9)], 1))
    return (f'  (footprint "t:MODULE" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -20 -10) (end 20 10) (stroke (width 0.1)'
            ' (type default)) (layer "F.Fab"))\n' + pins + '  )\n')


def _logo(ref, x, y):
    return (f'  (footprint "t:LOGO" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -2 -2) (end 2 2) (stroke (width 0.1)'
            ' (type default)) (layer "F.Fab")))\n')


R0402 = [(-0.48, 0, 0.56, 0.62), (0.48, 0, 0.56, 0.62)]
HEADER = [(-1.27, 0, 1.7, 1.7), (1.27, 0, 1.7, 1.7)]
BGA = [(-1.2 + 0.8 * i, -1.2 + 0.8 * j, 0.4, 0.4)
       for i in range(4) for j in range(4)]


def _fixture(td):
    return _board(td, ''.join([
        _radial('C1', 8, 6),
        _radial('C3', 16, 6),
        _smd('R1', 'R_0402', 8, 14, R0402, (-0.5, -0.25, 0.5, 0.25)),
        _smd('J1', 'PinHeader_1x02', 16, 14, HEADER, (-2.54, -1.27, 2.54,
                                                       1.27)),
        _smd('U1', 'BGA-16_3x3mm', 24, 6, BGA, (-1.5, -1.5, 1.5, 1.5)),
        _smd('C2', 'C_0603', 24, 14, [(-0.775, 0, 0.9, 0.95),
                                      (0.775, 0, 0.9, 0.95)],
             (-0.8, -0.4, 0.8, 0.4), court=(-1.48, -0.73, 1.48, 0.73)),
        _logo('G1', 32, 6),
    ]))


def test_who_gets_one_and_how_big(td):
    path = _fixture(td)
    pcb = parse_kicad_pcb(path)
    est = body.estimated_courtyards(pcb, path)
    check('every pad-bearing part that draws no courtyard is estimated, '
          'and only those', sorted(est) == ['C1', 'C3', 'J1', 'R1', 'U1'],
          sorted(est))
    lbs = legality.part_local_bounds(pcb, path)
    for ref in ('C1', 'C3', 'R1', 'J1', 'U1'):
        e = est.get(ref)
        if e is None:
            continue
        occ = tuple(round(v, 6) for v in lbs[ref].local)
        check(f'{ref}: the estimate is the graded occupancy, no margin',
              tuple(round(v, 6) for v in e.local) == occ, (e.local, occ))
        check(f'{ref}: on its own side, from the rung the grader reads',
              e.side == 'F' and e.basis == lbs[ref].source,
              (e.side, e.basis, lbs[ref].source))
    c1 = est['C1'].local
    check('C1, the mixer cap: its 5 mm body, not its 4.0 x 1.5 mm pad box',
          abs((c1[2] - c1[0]) - 5.0) < 1e-6 and abs((c1[3] - c1[1]) - 5.0)
          < 1e-6, c1)


def test_never_a_grade(td):
    """The graders keep reading what is drawn: part_local_bounds (check_
    assembly's courtyard channel, the floorplan grade) is unchanged."""
    path = _fixture(td)
    pcb = parse_kicad_pcb(path)
    lbs = legality.part_local_bounds(pcb, path)
    check('C1 is graded on its .Fab body, 5 x 5 mm, not on the estimate',
          lbs['C1'].source == 'fab'
          and tuple(round(v, 6) for v in lbs['C1'].local)
          == (-2.5, -2.5, 2.5, 2.5), (lbs['C1'].source, lbs['C1'].local))
    # Two caps whose BODIES are 0.3 mm apart: the grade -- on the bodies --
    # has no pair, and their estimates, being the bodies, leave them apart
    # too: an estimate adds no spacing the board does not declare.
    path2 = _board(td, _radial('C1', 8, 6) + _radial('C3', 13.3, 6))
    pcb2 = parse_kicad_pcb(path2)
    g = legality.grade_body_overlap(pcb2, 0.2, pcb_file=path2)
    pairs = [(q.a, q.b, q.kind) for q in g['pairs']]
    check('bodies 0.3 mm apart: no courtyard pair',
          not any(k == 'courtyard' for _a, _b, k in pairs), pairs)
    est = body.estimated_courtyards(pcb2, path2)
    a, b = est['C1'].local, est['C3'].local
    check('... and their estimates do not overlap (8 + 2.5 < 13.3 - 2.5)',
          8 + a[2] < 13.3 + b[0], (a, b))


def test_the_map_consumers_read(td):
    path = _fixture(td)
    pcb = parse_kicad_pcb(path)
    from placement.parser import extract_courtyard_sides
    drawn = extract_courtyard_sides(path)
    sides, est = body.courtyard_sides_with_estimates(pcb, path)
    check('a drawn courtyard is kept as drawn',
          sides.get('C2') == drawn.get('C2'), (sides.get('C2'),
                                               drawn.get('C2')))
    check('an estimate enters as its part\'s own-side courtyard',
          sides.get('C1') == {'F': est['C1'].local}, sides.get('C1'))
    check('the parser\'s own map is not mutated', 'C1' not in drawn)
    again, _ = body.courtyard_sides_with_estimates(pcb, path, drawn=drawn)
    check('handing the parser map in gives the same answer', again == sides)
    # place_simple's fallback for a part the estimates leave no legal pose
    skipped, est2 = body.courtyard_sides_with_estimates(pcb, path, skip={'C1'})
    check('a skipped part keeps what the board draws, its estimate unused',
          'C1' not in skipped and 'C1' not in est2, (skipped.get('C1'), sorted(est2)))
    check('...and only that part: the others keep their estimates',
          {r: v for r, v in skipped.items() if r != 'C1'}
          == {r: v for r, v in sides.items() if r != 'C1'})
    note = body.format_estimates(est, 'placement') or ''
    check('the NOTE names the count and each rung',
          '5 footprint(s)' in note and 'fab: C1, C3' in note
          and 'pad_bbox' not in note, note)
    check('no estimates, no note', body.format_estimates({}) is None)


def test_containers_and_pins(td):
    path = _board(td, _frame('A1', 22, 12) + _radial('C1', 22, 12))
    pcb = parse_kicad_pcb(path)
    est = body.estimated_courtyards(pcb, path)
    check('a pin frame (container) gets no estimate; the cap inside it does',
          sorted(est) == ['C1'], sorted(est))


def test_tracked_board():
    """esp_prog (OLIMEX): 0 of 21 footprints draw a courtyard."""
    path = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
    if not os.path.isfile(path):
        check('kicad_files/esp_prog.kicad_pcb present', False, path)
        return
    pcb = parse_kicad_pcb(path)
    est = body.estimated_courtyards(pcb, path)
    check('esp_prog: 18 pad-bearing parts estimated', len(est) == 18,
          len(est))
    lbs = legality.part_local_bounds(pcb, path)
    off = sorted(r for r, e in est.items()
                 if tuple(round(v, 6) for v in e.local)
                 != tuple(round(v, 6) for v in lbs[r].local))
    check('esp_prog: every estimate is its part\'s graded occupancy', not off,
          off)


def main():
    with tempfile.TemporaryDirectory(prefix='t1300_') as td:
        test_who_gets_one_and_how_big(td)
        test_never_a_grade(td)
        test_the_map_consumers_read(td)
        test_containers_and_pins(td)
    test_tracked_board()
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
