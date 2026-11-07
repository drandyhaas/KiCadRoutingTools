#!/usr/bin/env python3
"""#1694: the emit and --plan-only say, with no pose, that a part's body floor
needs its pads nearer the edge than the clearance binding them.

Until now a run learned that from place_simple, after its seeds. On #1603's
fixture, whose J3 reads a floor of 11.83 mm with its pads then 0.10 mm from the
edge (none of this asks KiCad; tests/test_1603_body_in_board_kicad.py reads the
real model):

  1. the `OWED:` line names the part, the floor, the clearance and its source,
     and the rule that frees it, on a pile and a placed board alike and at any
     staging pose;
  2. it agrees with the seat itself: over project floors and rule values, it
     stands exactly when `edge_seat.seats` has no seat at that band;
  3. a rule too large to free the pads is named as their source, and a rule on
     only some of them leaves the others named;
  4. a control whose pads stand well inside its body floor gets no line, and it
     seats;
  5. a pad on or past the outline at the floor is not offered a rule;
  6. a `why`, a declared pose, a lock or no model read answers it;
  7. a part with no mating face is asked only when no side of it can seat, and a
     declared face is the one read;
  8. the roster prints it through check_floorplan's own reader, and the skill
     says to act on it at the connector step.

    python3 tests/test_1694_pad_floor_owed.py
"""
import contextlib
import io
import json
import os
import re
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
BAND = {'min': 11.83, 'max': 12.43}
RULE = ('(version 1)\n(rule "edge_launch"\n  (condition "A.Type == \'Pad\' && '
        'A.memberOfFootprint(\'J3\'){}")\n  (constraint edge_clearance (min {}mm)))\n')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def piled(tmp, name, pro=None, dru=None, source=FIXTURE):
    d = os.path.join(tmp, name)
    os.makedirs(d)
    path = os.path.join(d, 'pile.kicad_pcb')
    run_utils.check([sys.executable, '-X', 'utf8', '-B',
                     os.path.join(REPO, '.claude/skills/plan-pcb-placement/scripts/make_unplaced.py'),
                     source, path], accept=True)
    run_utils.evidence(path, name)
    if pro is not None:
        with open(os.path.join(d, 'pile.kicad_pro'), 'w', encoding='utf-8') as f:
            json.dump({'board': {'design_settings': {'rules': {'min_copper_edge_clearance': pro}}}}, f)
    if dru is not None:
        with open(os.path.join(d, 'pile.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write(dru)
    return path


def restaged(pile, tmp, x, y, rot):
    """The pile with J3 staged elsewhere, by the placement's own writer: a turn
    has to carry the pads' stored angles with it."""
    from placement.writer import write_placed_output
    path = os.path.join(tmp, f'staged_{x}_{y}_{rot}.kicad_pcb')
    quiet(write_placed_output, pile, path,
          [{'reference': 'J3', 'new_x': x, 'new_y': y, 'new_rotation': rot}])
    return path


def body_at(pcb, x0=2.54, ref='J3', y=(-4.76, 4.76)):
    """The fixture's heights, with `ref`'s body at board depth starting `x0` along
    its own +x. 2.54 is where J3's pads end."""
    return heights._load({'top': {'J3': 4.34, 'J1': 8.54, 'J2': 8.54, 'C1': 0.5, 'C2': 0.5},
                          'covers': {}, 'bodies': {},
                          'depth': {ref: heights._depth_raster(
                              low(box(x0, 15.24, y[0], y[1], -3.58, 4.34)), T)},
                          'frame_fits': 1}, pcb, FIXTURE, {})


def owed(path, doc=None, make=synthetic, **kw):
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    pcb = quiet(parse_kicad_pcb, path)
    intent = floorplan.intent_from_dict(doc or {'schema': 1, 'kind': 'floorplan-intent', 'edge_connectors': []}, 'test')
    return floorplan.pad_floor_owed(intent, pcb, path, make(pcb), **kw)


def has_seat(path, band=BAND):
    """Does `edge_seat.seats` seat J3 on the east edge at `band` on this board?"""
    from kicad_parser import parse_kicad_pcb
    from placement import edge_seat, floorplan
    pcb = quiet(parse_kicad_pcb, path)
    real = heights.for_board
    heights.for_board = lambda *_a, **_k: heights.Heights(note='no heights')
    try:
        doc = quiet(floorplan.emit_intent, pcb, path)
        doc['edge_connectors'] = [c for c in doc.get('edge_connectors') or ()
                                  if c.get('ref') != 'J3']
        doc['edge_connectors'].append({'ref': 'J3', 'edge': 'east', 'overhang_mm': band})
        intent = floorplan.intent_from_dict(doc, 'test')
        clearance, edge = floorplan.resolved_floors(path)
        st = quiet(edge_seat.seat_state, pcb, path, intent, {}, clearance, edge)
        with contextlib.redirect_stdout(io.StringIO()):
            got, _notes, _why = edge_seat.seats(st, pcb, path, intent, {'J3'})
    finally:
        heights.for_board = real
    return 'J3' in got


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    run_utils.evidence(SKILL, 'the skill')
    tmp = tempfile.mkdtemp(prefix='t1694_')
    try:
        print('the line:')
        pile = piled(tmp, 'plain')
        got = owed(pile)
        line = got[0] if got else ''
        check('one line, naming the part', len(got) == 1 and line.startswith('J3 ('), str(got))
        check('...its floor, and where that leaves its pads',
              'overhang of 11.830mm' in line and 'its pads would stand 0.100mm from the edge' in line,
              line)
        check('...the binding clearance and its source, in the seat\'s words',
              'is 0.20mm, from the fab edge floor: the board declares no min_copper_edge_clearance'
              in line, line)
        check('...and the rule that frees it',
              "A.memberOfFootprint('J3')" in line and '(constraint edge_clearance (min 0mm))' in line
              and 'Lowering the band will not help' in line and "the vendor's drawing" in line,
              line)
        print('the same on a pile and a placed board:')
        placed = owed(FIXTURE)
        check('the placed board reads the same line', placed == got, str(placed))
        for x, y, rot in ((120.0, -40.0, 0), (18.0, 15.0, 90), (60.0, 60.0, 180)):
            other = owed(restaged(pile, tmp, x, y, rot))
            check(f'staged at {x}, {y}, turned {rot}: the same line', other == got, str(other))

        print('it agrees with the seat:')
        cases = {'no project': {}, 'a 0.4 mm floor': {'pro': 0.4},
                 'a 0.05 mm floor, under the fab floor': {'pro': 0.05},
                 'a rule at 0': {'dru': RULE.format('', 0)},
                 'a rule at 0.09': {'dru': RULE.format('', 0.09)},
                 'a rule at 0.095': {'dru': RULE.format('', 0.095)},
                 'a rule at 0.1': {'dru': RULE.format('', 0.1)},
                 'a rule at 0.15': {'dru': RULE.format('', 0.15)}}
        for i, (name, kw) in enumerate(cases.items()):
            p = piled(tmp, f'case{i}', **kw)
            line_there, seat = bool(owed(p)), has_seat(p)
            check(f'{name}: line {line_there}, seat {seat}', line_there != seat)

        print('a rule that does not free them:')
        p = piled(tmp, 'rule015', dru=RULE.format('', 0.15))
        text = (owed(p) or [''])[0]
        check("a rule too large is named as the clearance's source",
              "is 0.15mm, from the .kicad_dru rule 'edge_launch'" in text, text)
        p = piled(tmp, 'rule_pad', dru=RULE.format(" && A.Pad_Number == '1'", 0))
        text = (owed(p) or [''])[0]
        check('a rule on one pad leaves the others named',
              ' pad 2' in text and 'its pads' not in text.split('binding')[0], text)

        print('a control whose pads stand well inside its floor:')
        ctl = lambda pcb: body_at(pcb, 3.54)    # noqa: E731
        check('no line', owed(pile, make=ctl) == [], str(owed(pile, make=ctl)))
        from kicad_parser import parse_kicad_pcb
        from placement import floorplan
        pcb = quiet(parse_kicad_pcb, pile)
        floor = floorplan.body_clearing_overhang(ctl(pcb), pcb, pile, 'J3')
        check('...its floor is 1 mm less',
              abs(floor['min'] - (BAND['min'] - 1.0)) < 0.002, str(floor))
        check('...and it seats, at that band',
              has_seat(pile, {'min': floor['min'], 'max': floor['min'] + 0.6}))

        print('a pad on or past the outline:')
        into = lambda pcb: body_at(pcb, 1.0)    # noqa: E731
        text = (owed(pile, make=into) or [''])[0]
        check('is told no rule frees it, and not offered one',
              'would leave the board by' in text and 'No edge_clearance rule frees' in text
              and 'constraint edge_clearance' not in text, text)

        print('what answers it:')
        check('a `why`', owed(pile, {'schema': 1, 'kind': 'floorplan-intent', 'edge_connectors': [
            {'ref': 'J3', 'why': 'over a declared slot'}]}) == [])
        check('a declared pose', owed(pile, {'schema': 1, 'kind': 'floorplan-intent', 'edge_connectors': [], 'fixed_poses': [
            {'ref': 'J3', 'x': 10.0, 'y': 10.0, 'rot': 0.0, 'basis': 'declared'}]}) == [])
        p = piled(tmp, 'rule0', dru=RULE.format('', 0))
        check('a rule that frees the pads', owed(p) == [])
        check('no model read', owed(pile, make=lambda _p: heights.Heights(note='no heights')) == [])
        check("the run's own edge clearance replaces the board's",
              owed(p) == [] and 'the edge clearance given to this run' in (
                  owed(pile, edge_clearance=0.3) or [''])[0], str(owed(pile, edge_clearance=0.3)))
        with open(FIXTURE, encoding='utf-8', newline='') as f:
            t = f.read()
        m = re.search(r'\(footprint "Connector_Coaxial:SMA_Amphenol[^\n]*\n', t)
        locked = os.path.join(tmp, 'locked.kicad_pcb')
        with open(locked, 'w', encoding='utf-8', newline='') as f:
            f.write(t[:m.end()] + '\t\t(locked yes)\n' + t[m.end():])
        check('a lock', owed(locked) == [], str(owed(locked)))

        print('a part with no mating face:')
        pcb = quiet(parse_kicad_pcb, pile)
        from placement.body import board_bodies
        from placement.mating import mating_face_local
        none = next((r for r in ('C1', 'C2') if not mating_face_local(
            pcb.footprints[r], board_bodies(pcb, pile).get(r)).face), None)
        check('the fixture has one', none is not None)
        wide = lambda pcb: body_at(pcb, -6.0, none, (-6.0, 6.0))    # noqa: E731
        text = (owed(pile, make=wide) or [''])[0]
        check('no side can seat: named, and it says so',
              text.startswith(none) and 'no side of it can seat' in text, text)
        check('a declared face is the one read',
              '(+x)' in (owed(pile, {'schema': 1, 'kind': 'floorplan-intent', 'edge_connectors': [
                  {'ref': none, 'mating_face': '+x'}]}, make=wide) or [''])[0])

        print('the reader and the skill:')
        import check_floorplan
        real = heights.for_board
        heights.for_board = lambda *_a, **_k: synthetic(pcb)
        try:
            lines = check_floorplan._model_owed(
                floorplan.intent_from_dict({'schema': 1, 'kind': 'floorplan-intent', 'edge_connectors': []}, 't'), pcb, pile)
        finally:
            heights.for_board = real
        printed = floorplan.format_roster([], (), lines)
        check('the roster prints it as an OWED: line, beside the body floor\'s',
              sum(ln.startswith('    OWED: J3 (') for ln in printed) == 2
              and any('its pads would stand' in ln for ln in printed)
              and any('its own 3D model has body at board depth' in ln for ln in printed),
              str(printed))
        with open(SKILL, encoding='utf-8') as f:
            flat = ' '.join(f.read().split())
        check('the skill says the roster says so, and to settle it there',
              'The roster of `--emit-intent` and `--plan-only` says so here' in flat
              and 'settle it now' in flat)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
