"""#1300: place_simple seats a part that draws no courtyard on an ESTIMATED one.

multichannel_mixer's radial electrolytics draw a 5 mm .Fab body round a
4.0 x 1.5 mm pad box and no courtyard. The placer's part model seated them on
the pad box, while check_assembly grades the body, so every seed packed caps
into each other's bodies and graded NOT BUILDABLE (4 courtyard-blocking
pairs on each of seeds 0-3).

`QuenchState(estimate_courtyards=True)` -- which place_simple, fit_template
and render_placement build -- reads `body.courtyard_sides_with_estimates`, so
such a part's `rect` and `grade_rect` are its estimated courtyard (the graded
body and pads, no margin) wherever a drawn courtyard
would be read: the anneal's overlap, the seat gate (`seeder.pose_ok`), the
final legality. Two declared or locked poses are still judged against each
other on what the board draws (`_Part.drawn_rect`), so no declared
arrangement is refused on an estimate. A state that grades (the default)
is unchanged.

    python3 tests/test_1300_place_simple_estimated_courtyards.py
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


def _radial(ref, x, y, n1, n2, locked=False):
    """multichannel_mixer's CAPPRB254-500X840: two drilled pads 2.54 mm
    apart (a 4.04 x 1.5 mm pad box) in a 5 mm .Fab circle, no courtyard."""
    return (f'  (footprint "t:CAPPRB254-500X840" (layer "F.Cu") (at {x} {y})\n'
            + ('    (locked yes)\n' if locked else '')
            + f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_circle (center 0 0) (end 2.5 0) (stroke (width 0.1)'
            ' (type default)) (layer "F.Fab"))\n'
            '    (pad "1" thru_hole rect (at -1.27 0) (size 1.5 1.5)'
            f' (drill 0.8) (layers "*.Cu" "*.Mask") (net {n1} "N{n1}"))\n'
            '    (pad "2" thru_hole circle (at 1.27 0) (size 1.5 1.5)'
            f' (drill 0.8) (layers "*.Cu" "*.Mask") (net {n2} "N{n2}")))\n')


def _chip(ref, x, y):
    """A part that draws its courtyard: nothing is estimated for it."""
    return (f'  (footprint "t:C_0603" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -1.48 -0.73) (end 1.48 0.73) (stroke'
            ' (width 0.05) (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -0.775 0) (size 0.9 0.95)'
            ' (layers "F.Cu") (net 1 "N1"))\n'
            '    (pad "2" smd rect (at 0.775 0) (size 0.9 0.95)'
            ' (layers "F.Cu") (net 2 "N2")))\n')


def _board(path, parts, size=(40, 30)):
    nets = ''.join(f'  (net {i} "N{i}")\n' for i in range(1, 7))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
                 ' (44 "Edge.Cuts" user))\n  (net 0 "")\n' + nets
                 + f'  (gr_rect (start 0 0) (end {size[0]} {size[1]})'
                 ' (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
                 + ''.join(parts) + ')\n')
    return path


def _state(path, estimate, locked=None):
    import pose_score
    pcb = parse_kicad_pcb(path)
    return pose_score.make_state(pcb, path, clearance=0.25,
                                 board_edge_clearance=0.5,
                                 extra_locked_refs=locked,
                                 estimate_courtyards=estimate)


def test_the_part_model(td):
    path = _board(os.path.join(td, 'model.kicad_pcb'),
                  [_radial('C1', 10, 10, 1, 2), _radial('C3', 30, 10, 2, 3),
                   _chip('C2', 20, 22)])
    plain, est = _state(path, False), _state(path, True)

    def wh(r):
        return (round(r[2] - r[0], 4), round(r[3] - r[1], 4))
    ok('a grading state (the default) seats C1 on its 4.04 x 1.5 pad box',
       wh(plain.parts['C1'].rect()) == (4.04, 1.5),
       wh(plain.parts['C1'].rect()))
    ok('place_simple\'s state seats it on the 5.0 mm estimate (its body), '
       'rect and grade rect alike', wh(est.parts['C1'].rect()) == (5.0, 5.0)
       and wh(est.parts['C1'].grade_rect()) == (5.0, 5.0),
       (wh(est.parts['C1'].rect()), wh(est.parts['C1'].grade_rect())))
    ok('... and keeps the pad box for a declared-against-declared judgement',
       wh(est.parts['C1'].drawn_rect()) == (4.04, 1.5),
       wh(est.parts['C1'].drawn_rect()))
    ok('a drawn courtyard is untouched',
       est.parts['C2'].rect() == plain.parts['C2'].rect()
       and est.parts['C2'].drawn_rect() == est.parts['C2'].grade_rect())
    ok('the state names its estimates',
       sorted(est.courtyard_estimates) == ['C1', 'C3']
       and plain.courtyard_estimates == {},
       sorted(est.courtyard_estimates))


def test_the_seat_gate(td):
    """C3 1.5 mm below C1's pads: the pad boxes clear the 0.25 mm
    clearance, the 5 mm bodies overlap by 2 mm. The seat gate refuses it
    only on the estimate."""
    from placement.seeder import pose_ok
    path = _board(os.path.join(td, 'seat.kicad_pcb'),
                  [_radial('C1', 10, 10, 1, 2), _radial('C3', 30, 10, 2, 3)])
    plain, est = _state(path, False), _state(path, True)
    ok('pad-box model: C3 at (10, 13) seats, its body 2 mm into C1\'s',
       pose_ok(plain, 'C3', 10.0, 13.0, 0.0, set()))
    ok('estimated courtyards: refused there', not pose_ok(est, 'C3', 10.0,
                                                          13.0, 0.0, set()))
    ok('... and accepted where the estimates clear by the clearance',
       pose_ok(est, 'C3', 10.0, 10.0 + 5.0 + 0.26, 0.0, set()))


def test_declared_against_declared(td):
    """`_courtyard_overlap` between two LOCKED parts reads what the board
    draws; a searched pose against them reads the estimate."""
    from placement.seeder import _courtyard_overlap
    path = _board(os.path.join(td, 'decl.kicad_pcb'),
                  [_radial('C1', 10, 10, 1, 2), _radial('C3', 10, 13, 2, 3)])
    both = _state(path, True, locked={'C1', 'C3'})
    one = _state(path, True, locked={'C1'})
    a_both = _courtyard_overlap(both, 'C1', (10, 10, 0), 'C3', (10, 13, 0))[0]
    a_one = _courtyard_overlap(one, 'C1', (10, 10, 0), 'C3', (10, 13, 0))[0]
    ok('two declared poses: judged on the drawn ladder, as before (0)',
       a_both == 0.0, a_both)
    ok('a searched pose: screened on the estimate, the bodies\' overlap '
       'found (10 mm2)', abs(a_one - 10.0) < 0.05, a_one)


def test_place_simple(td):
    """End to end: four radial caps in a ring of nets on a board just big
    enough. place_simple names the estimates, and its seed grades buildable
    with no body pair."""
    parts = [_radial(f'C{i + 1}', 5 + 8 * i, 24, i + 1, (i + 1) % 4 + 1)
             for i in range(4)]
    board = _board(os.path.join(td, 'ring.kicad_pcb'), parts, size=(16, 16))
    groups = os.path.join(td, 'groups.json')
    with open(groups, 'w', encoding='utf-8') as fh:
        json.dump({'schema': 1, 'board': 'ring.kicad_pcb', 'groups': [
            {'name': 'caps', 'role': 'signal',
             'refs': ['C1', 'C2', 'C3', 'C4'], 'why': 'one ring'}]}, fh)
    intent = os.path.join(td, 'intent.json')
    check([sys.executable, '-X', 'utf8', tool('check_floorplan.py'), board,
           '--emit-intent', intent, '--allow-unplaced'], accept=True)
    evidence(intent, 'the emitted intent')
    out = os.path.join(td, 'placed.kicad_pcb')
    r = check([sys.executable, '-X', 'utf8', tool('place_simple.py'), board,
               out, '--groups', groups, '--intent', intent, '--seed', '0',
               '--moves', '3000'], accept=True)
    ok('place_simple names the estimated courtyards',
       'seated on an ESTIMATED one' in r.stdout and 'C1, C2, C3, C4'
       in r.stdout, r.stdout[-800:])
    evidence(out, 'the placed board')
    js = os.path.join(td, 'assembly.json')
    check([sys.executable, '-X', 'utf8', tool('check_assembly.py'), out,
           '--baseline', board, '--json', js], accept=True)
    doc = json.load(open(js))
    ok('the seed is buildable, no courtyard pair gating',
       doc.get('buildable') is True and doc.get('courtyard_blocking') == 0,
       (doc.get('buildable'), doc.get('courtyard_blocking')))
    fab = [p for p in doc.get('pairs') or () if p.get('kind') == 'fab']
    ok('no two drawn bodies overlap', fab == [], fab)


def main():
    with tempfile.TemporaryDirectory(prefix='t1300ps_') as td:
        test_the_part_model(td)
        test_the_seat_gate(td)
        test_declared_against_declared(td)
        test_place_simple(td)
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
