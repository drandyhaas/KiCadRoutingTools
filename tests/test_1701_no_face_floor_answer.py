#!/usr/bin/env python3
"""#1701: the body floor's `OWED:` line on a part with no mating face goes when
its entry gives what the line asks for.

The line gives the overhang for each side and asks for an `edge`, a
`mating_face` and that `overhang_mm.min`. The row was never read back, so the
line stood whatever the entry said. On #1603's fixture piled, with a synthetic
body at board depth under a capacitor whose footprint is symmetric about its
pads (none of this asks KiCad):

  1. with no entry, the line gives every side;
  2. an entry with the edge, the side and that side's floor answers it, on each
     side;
  3. an entry short of the floor keeps a line, naming the side and what the
     entry declares; so does one with no edge, and one with no side. On the
     fixture's edge jack read as having no face, which has a seat, the entry that
     answers it keeps the body off the board and the one that does not leaves it
     through;
  4. a `why` still answers it, and a side whose floor is 0 owes nothing;
  5. a declared face outranks a weak reading, in the floor as in the seat, and
     a strong reading outranks the declaration;
  6. `pad_floor_owed`, beside it, already read the declared face;
  7. the skill says what answers the line.

    python3 tests/test_1701_no_face_floor_answer.py
"""
import contextlib
import io
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
from placement import heights  # noqa: E402
from test_1603_body_in_board import FIXTURE, T, box, low, quiet, synthetic  # noqa: E402

RUN_ALL_TIMEOUT = 600
SKILL = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-placement', 'SKILL.md')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def lines(fn, pcb, path, h, rows, ref):
    from placement import floorplan
    intent = floorplan.intent_from_dict(
        {'schema': 1, 'kind': 'floorplan-intent', 'edge_connectors': rows}, 'test')
    return [ln for ln in fn(intent, pcb, path, h) if ln.startswith(ref + ' ')]


def entry(ref, face=None, low_mm=None, edge='east', why=None):
    row = {'ref': ref}
    if edge:
        row['edge'] = edge
    if face:
        row['mating_face'] = face
    if low_mm is not None:
        row['overhang_mm'] = {'min': low_mm, 'max': low_mm + 0.6}
    if why:
        row['why'] = why
    return row


def no_face(*_a, **_k):
    """A reading that abstains, whatever the footprint."""
    from placement.mating import LocalFace
    return LocalFace(None, 0.0, '', 'symmetric about its pads')


def body_from(x0):
    """The fixture's edge jack with its body at board depth from `x0` along its
    own +x, past its pads, so a seat exists."""
    def make(pcb):
        return heights._load({'top': {'J3': 4.34, 'J1': 8.54, 'J2': 8.54, 'C1': 0.5, 'C2': 0.5},
                              'covers': {}, 'bodies': {},
                              'depth': {'J3': heights._depth_raster(
                                  low(box(x0, 15.24, -4.76, 4.76, -3.58, 4.34)), T)},
                              'frame_fits': 1}, pcb, FIXTURE, {})
    return make


def seated_body_gates(pcb, path, h, row):
    """Seat `row`'s part on its edge and read check_assembly's gate at both ends
    and the middle of each stretch of its line. None when it gets no seat."""
    from placement import edge_seat, floorplan
    doc = {'schema': 1, 'kind': 'floorplan-intent', 'edge_connectors': [row]}
    intent = floorplan.intent_from_dict(doc, 'test')
    clearance, edge = floorplan.resolved_floors(path)
    real = heights.for_board
    heights.for_board = lambda *_a, **_k: h
    try:
        st = quiet(edge_seat.seat_state, pcb, path, intent, {}, clearance, edge)
        with contextlib.redirect_stdout(io.StringIO()):
            got, _notes, _why = edge_seat.seats(st, pcb, path, intent, {row['ref']})
    finally:
        heights.for_board = real
    seat = got.get(row['ref'])
    if seat is None:
        return None
    gated = []
    for rot, ln in seat.lines.items():
        for f0, f1 in ln.free:
            for a in (f0, (f0 + f1) / 2.0, f1):
                pose = (a, ln.coord, rot) if seat.axis == 0 else (ln.coord, a, rot)
                gated += heights.body_in_board(h, pcb, {row['ref']: pose})['gating_refs']
    return gated


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    run_utils.evidence(SKILL, 'the skill')
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    from placement.body import board_bodies
    from placement.mating import LocalFace, mating_face_local
    tmp = tempfile.mkdtemp(prefix='t1701_')
    try:
        pile = os.path.join(tmp, 'pile.kicad_pcb')
        run_utils.check([sys.executable, '-X', 'utf8', '-B',
                         os.path.join(REPO, '.claude/skills/plan-pcb-placement/scripts/make_unplaced.py'),
                         FIXTURE, pile], accept=True)
        run_utils.evidence(pile, 'the pile')
        pcb = quiet(parse_kicad_pcb, pile)
        ref = 'C1'
        check('the part reads no mating face',
              mating_face_local(pcb.footprints[ref], board_bodies(pcb, pile).get(ref)).face is None)
        lip = heights._depth_raster(low(box(-3.0, 3.0, 1.0, 1.5, -1.2, 0.5)), T)
        h = synthetic(pcb, extra={ref: lip})
        floors = floorplan.body_clearing_overhang(h, pcb, pile, ref)['by_face']
        owed = floorplan.body_floor_owed

        def body(rows):
            return lines(owed, pcb, pile, h, rows, ref)

        print('no entry:')
        one = (body([]) or [''])[0]
        check('the line gives every side and asks for the three keys',
              all(f in one for f in floors) and '`edge`, `mating_face`' in one, one)

        print('an entry giving what it asks for:')
        for face, v in floors.items():
            check(f'{face}, at its floor of {v}: answered',
                  body([entry(ref, face, v)]) == [], str(body([entry(ref, face, v)])))
        most = max(floors, key=floors.get)

        print('an entry not reaching it:')
        one = (body([entry(ref, most, floors[most] - 0.5)]) or [''])[0]
        check('a band under the side\'s floor keeps a line, with the side and the entry\'s minimum',
              f'at least {floors[most]:.3f}mm' in one and f'({most})' in one
              and f'declares a minimum of {floors[most] - 0.5:.3f}mm on the east edge' in one, one)
        one = (body([entry(ref, most, floors[most], edge=None)]) or [''])[0]
        check('no edge keeps a line asking for it', f'({most})' in one and '`edge`' in one, one)
        one = (body([entry(ref, None, max(floors.values()))]) or [''])[0]
        check('no side keeps the line giving every side',
              all(f in one for f in floors) and '`mating_face`' in one, one)

        print('what else answers it:')
        check('a `why`', body([entry(ref, why='over a slot the outline will carry')]) == [])
        flat = next(f for f, v in floors.items() if v <= 0.0)
        check(f'a side with no floor ({flat}) owes nothing, with or without a band',
              body([entry(ref, flat)]) == [] and body([entry(ref, flat, edge=None)]) == [])

        from placement import mating
        real = mating.mating_face_local
        try:
            print('the same answer, on a part that can seat:')
            mating.mating_face_local = no_face
            hj = body_from(3.54)(pcb)
            jack = floorplan.body_clearing_overhang(hj, pcb, pile, 'J3')['by_face']
            check('the edge jack, reading no face, owes a line',
                  bool(lines(owed, pcb, pile, hj, [], 'J3')), str(jack))
            row = entry('J3', '+x', jack['+x'])
            check('...its edge, side and floor answer it', lines(owed, pcb, pile, hj, [row], 'J3') == [])
            gated = seated_body_gates(pcb, pile, hj, row)
            check('...and the seat then keeps that body off the board', gated == [], str(gated))
            short = dict(entry('J3', '+x'), overhang_mm={'min': jack['+x'] - 0.5,
                                                         'max': jack['+x'] - 0.45})
            gated = seated_body_gates(pcb, pile, hj, short)
            check('...where a band short of the floor, which keeps the line, leaves it through the board',
                  bool(lines(owed, pcb, pile, hj, [short], 'J3')) and gated == ['J3'] * len(gated) and gated,
                  str(gated))
            check('...and its pads, on the side declared, owe no line either',
                  lines(floorplan.pad_floor_owed, pcb, pile, hj, [row], 'J3') == [])
            print('a declared face against a reading:')
            weak = LocalFace('+y', 0.5, 'courtyard', '', (), False, None)
            strong = LocalFace('+y', 2.0, 'courtyard', '', (), True, None)
            mating.mating_face_local = lambda *_a, **_k: weak
            one = (body([entry(ref, most, floors['+y'])]) or [''])[0]
            check('it outranks a weak reading: the floor asked is the declared side\'s',
                  floors[most] > floors['+y'] and f'({most})' in one
                  and f'at least {floors[most]:.3f}mm' in one, one)
            check('...and that side\'s floor answers it', body([entry(ref, most, floors[most])]) == [])
            mating.mating_face_local = lambda *_a, **_k: strong
            check('a strong reading outranks it, as in the seat: that side\'s floor is the one read',
                  floors['+y'] <= 0.0 and body([entry(ref, most, 0.0)]) == [], str(floors))
        finally:
            mating.mating_face_local = real

        print('pad_floor_owed, beside it:')
        wide = heights._depth_raster(low(box(-6.0, 6.0, -6.0, 6.0, -1.2, 0.5)), T)
        hw = synthetic(pcb, extra={ref: wide})
        pad = floorplan.pad_floor_owed
        one = (lines(pad, pcb, pile, hw, [], ref) or [''])[0]
        check('with no side it says none can seat', 'no side of it can seat' in one, one)
        one = (lines(pad, pcb, pile, hw, [entry(ref, '+x')], ref) or [''])[0]
        check('with a declared side it reads that side', '(+x)' in one
              and 'no side of it can seat' not in one, one)

        print('the skill:')
        with open(SKILL, encoding='utf-8') as f:
            flat_text = ' '.join(f.read().split())
        check('it says what answers the line',
              'the entry answers it with its `edge`, the side that hangs as `mating_face`'
              in flat_text)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
