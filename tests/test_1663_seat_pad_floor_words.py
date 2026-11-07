#!/usr/bin/env python3
"""#1663: where a part's pad floor holds it short of its band's minimum, the
seat says so, and the skill says what to do.

An end-launch jack needs its pads nearly on the edge; the seat holds them the
board's edge clearance inside it, so every position it tests is short of the
band. `_Probe.why` asks the band first, so the message blamed the band -- the one
input that must not come down, since lowered it leaves the body in the laminate.
Near a corner it was worse: the band read included the overhang past the OTHER
edge, so a courtyard hanging off the corner met the minimum, and the part seated
where the grade then failed it. On #1603's fixture, whose J3 needs 11.83 mm
(none of this asks KiCad):

  1. at a project floor of 0.4 mm: no seat, and the reason names the pad floor,
     its value, its source, what the minimum would take, and the way out;
  2. with no project: the fab floor, named as that;
  3. a position near the corner no longer meets a minimum on this edge;
  4. a `.kicad_dru` `edge_clearance` rule binding its pads seats it, at the
     band; a rule too large to reach it is named as the source;
  5. a band not held by the pad floor keeps its plain words;
  6. the skill names this direction and the rule.

    python3 tests/test_1663_seat_pad_floor_words.py
"""
import contextlib
import io
import json
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
from test_1603_body_in_board import FIXTURE, quiet  # noqa: E402

RUN_ALL_TIMEOUT = 600
BAND = {'min': 11.83, 'max': 12.43}
SKILL = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-placement', 'SKILL.md')
RULE = ('(version 1)\n(rule "edge_launch"\n  (condition "A.Type == \'Pad\' && '
        'A.memberOfFootprint(\'J3\')")\n  (constraint edge_clearance (min {}mm)))\n')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def board(tmp, name, pro=None, dru=None):
    d = os.path.join(tmp, name)
    os.makedirs(d)
    path = os.path.join(d, 'pile.kicad_pcb')
    run_utils.check([sys.executable, '-X', 'utf8', '-B',
                     os.path.join(REPO, '.claude/skills/plan-pcb-placement/scripts/make_unplaced.py'),
                     FIXTURE, path], accept=True)
    run_utils.evidence(path, name)
    if pro is not None:
        with open(os.path.join(d, 'pile.kicad_pro'), 'w', encoding='utf-8') as f:
            json.dump({'board': {'design_settings': {'rules': {'min_copper_edge_clearance': pro}}}}, f)
    if dru is not None:
        with open(os.path.join(d, 'pile.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write(dru)
    return path


def seat(path, band=BAND):
    """edge_seat.seats on the board, J3 declared on its east edge; what it
    seated, why not, and the probe's state."""
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
            got, _notes, unseated = edge_seat.seats(st, pcb, path, intent, {'J3'})
    finally:
        heights.for_board = real
    return got, unseated, st, pcb


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    run_utils.evidence(SKILL, 'the skill')
    tmp = tempfile.mkdtemp(prefix='t1663_')
    try:
        print('the pad floor names itself:')
        _s, why, _st, _pcb = seat(board(tmp, 'floor04', pro=0.4))
        text = why.get('J3', '')
        check('a project floor of 0.4 mm: no seat', 'J3' in why, text)
        check('...the band is said, and the pad floor as what stopped it',
              'declared minimum 11.83mm' in text and "pad copper's edge clearance stops it" in text,
              text)
        check('...its value and where it comes from',
              "0.40mm from the board's min_copper_edge_clearance" in text, text)
        check('...what the minimum would take', 'its pads would leave the board' not in text
              and 'at that minimum its pads would stand 0.10mm from the edge' in text, text)
        check('...that lowering the band will not help, and the way out',
              'Lowering the band will not help a part whose body needs it' in text
              and "the vendor's drawing" in text and '.kicad_dru edge_clearance rule' in text, text)

        _s, why, _st, _pcb = seat(board(tmp, 'nofloor'))
        text = why.get('J3', '')
        check('no project: the fab floor, as that', '0.20mm from the fab edge floor' in text
              and 'declares no min_copper_edge_clearance' in text, text)

        print('a minimum is met on this edge alone:')
        from placement import edge_seat, floorplan
        path = board(tmp, 'corner')
        _s, _w, st, pcb = seat(path)
        from placement.body import board_bodies
        probe = edge_seat._Probe(st, pcb, path, board_bodies(pcb, path), floorplan.outline_state(pcb, path))
        entry = {'ref': 'J3', 'edge': 'east', 'overhang_mm': BAND}
        probe.faces['J3'] = '+x'
        # the courtyard's north side past the north edge, the copper limit's x
        pose = (28.255, 10.2, 0.0)
        past, others, _basis = probe.band_parts('J3', 'east', pose)
        kind, read = probe.why(entry, 'east', pose, {})
        check('hanging off the corner, the sum reaches the minimum and the seat refuses it',
              past < BAND['min'] and past + others >= BAND['min'] and kind == 'band_min'
              and abs(read - past) < 1e-9, f'past {past}, others {others}: {kind} {read}')

        print('a rule binding the pads:')
        got, why, _st, _pcb = seat(board(tmp, 'rule0', dru=RULE.format(0)))
        check('min 0 seats it', 'J3' in got and 'J3' not in why, str(why))
        if 'J3' in got:
            coord = next(iter(got['J3'].lines.values())).coord
            check('...at the band', abs(coord - (28.46 + 11.835 - 11.93)) < 0.01, str(coord))
        got, why, _st, _pcb = seat(board(tmp, 'rule015', dru=RULE.format(0.15)))
        text = why.get('J3', '')
        check("a rule too large to reach it is named as the floor's source",
              "0.15mm from the .kicad_dru rule 'edge_launch'" in text, text)

        print('a band nothing holds:')
        part_probe = probe
        words = edge_seat._unseated_reason(part_probe, entry, 'east', 1, 11.83, 0.205, {}, {},
                                           {'band_min': [3, [11.7]]})
        check('keeps its plain words', 'under the declared minimum 11.83mm' in words
              and 'Lowering' not in words, words)

        print('the skill:')
        with open(SKILL, encoding='utf-8') as f:
            skill = f.read()
        flat = ' '.join(skill.split())
        check('its list of what stopped a seat names this direction',
              "its pad copper's edge clearance holding it short of its band's minimum" in flat)
        check('...and it says what to do',
              "Check the vendor's drawing" in flat and "`.kicad_dru` `edge_clearance` rule" in flat
              and "the board owner's agreement" in flat)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
