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
  6. the skill names this direction and the rule;
  7. (#1773) a pad-number rule that frees only the pad that stops the seat:
     the words name THAT pad, its own clearance and rule, where it would stand
     at the minimum, and #1694's way out when that is past the outline, at
     every site the seat words a pad floor, read through the one per-pad
     reading `pad_floor_owed` uses.

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


def launch_board(tmp):
    """#1773's J1, an edge-launch part on a 40 x 30 board: pad 1 (the launch)
    ends 4.0 mm east of its origin, its two pads "2" (posts) 1.25 mm, its
    F.Fab body 4.5 mm. The project floor is 0.4 mm; a rule frees pad 1 alone."""
    d = os.path.join(tmp, 'launch')
    os.makedirs(d)

    def rect(x0, y0, x1, y1, layer):
        return (f'    (fp_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.05) '
                f'(type default)) (layer "{layer}"))\n')

    def pad(n, x, y, w, h):
        return (f'    (pad "{n}" smd rect (at {x} {y}) (size {w} {h}) '
                f'(layers "F.Cu" "F.Mask"))\n')
    fp = ('  (footprint "t:EdgeLaunch" (layer "F.Cu") (at 20 15)\n'
          '    (property "Reference" "J1" (at 0 -5) (layer "F.SilkS"))\n'
          + rect(-1.5, -3, 4.5, 3, 'F.Fab') + rect(-2, -3.5, 5, 3.5, 'F.CrtYd')
          + pad('1', 3, 0, 2, 1) + pad('2', 0.5, -2.2, 1.5, 1) + pad('2', 0.5, 2.2, 1.5, 1)
          + '  )\n')
    cuts = ''.join(f'  (gr_line (start {a} {b}) (end {c} {e}) (stroke (width 0.05) '
                   f'(type default)) (layer "Edge.Cuts"))\n'
                   for a, b, c, e in ((0, 0, 40, 0), (40, 0, 40, 30), (40, 30, 0, 30), (0, 30, 0, 0)))
    path = os.path.join(d, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20241229) (generator "t1773")\n  (net 0 "")\n'
                + cuts + fp + ')\n')
    with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {'rules': {'min_copper_edge_clearance': 0.4}}}}, f)
    with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as f:
        f.write("(version 1)\n(rule \"launch_pin\"\n  (condition \"A.Type == 'Pad' && "
                "A.Pad_Number == '1' && A.memberOfFootprint('J1')\")\n"
                "  (constraint edge_clearance (min 0mm)))\n")
    run_utils.evidence(path, 'the #1773 board')
    return path


def launch_seat(path, low):
    """edge_seat.seats on the #1773 board, J1 declared on its east edge
    with its band at `low`..2.0: (seated, unseated, state, pcb, entry)."""
    from kicad_parser import parse_kicad_pcb
    from placement import edge_seat, floorplan
    pcb = quiet(parse_kicad_pcb, path)
    row = {'ref': 'J1', 'edge': 'east', 'mating_face': '+x', 'class': 'edge_receptacle',
           'overhang_mm': {'min': low, 'max': 2.0}}
    it = floorplan.intent_from_dict({'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
                                     'units': 'mm', 'edge_connectors': [row]}, source_path='t')
    st = quiet(edge_seat.seat_state, pcb, path, it, {}, *floorplan.resolved_floors(path))
    got, _notes, why = quiet(edge_seat.seats, st, pcb, path, it, {'J1'})
    return got, why, st, pcb, row


def binding_pad(tmp):
    """7. (#1773)."""
    from design_rules import PadEdgeRules
    from placement import edge_seat, floorplan
    from placement.body import board_bodies
    from placement.connector_geometry import ConnectorGeometry
    print('a rule that frees only the pad that stops it (#1773):')
    path = launch_board(tmp)
    got, why, st, pcb, row = launch_seat(path, 0.495)
    check('min 0.495 seats it, pad 1 at its 0 mm rule', 'J1' in got, str(why))
    probe = edge_seat._Probe(st, pcb, path, board_bodies(pcb, path),
                             floorplan.outline_state(pcb, path))
    probe.faces['J1'] = '+x'
    if 'J1' in got:
        x = next(iter(got['J1'].lines.values())).coord
        b = probe.binding('J1', (x, 15.0, 0.0))
        check('...the pad that binds there is pad 1, at its rule',
              b.names == 'pad 1' and abs(b.clearance) < 1e-9 and 'launch_pin' in b.source
              and abs(b.inside - 0.005) < 1e-4, f'{b}')
    _g, why, _st, _pcb, _row = launch_seat(path, 0.8)
    text = why.get('J1', '')
    check('min 0.8: no seat, the reason names pad 1, its clearance and its rule',
          "the seat holds pad 1 0.005mm inside the edge, 0.00mm from the .kicad_dru rule "
          "'launch_pin'" in text, text)
    check('...where pad 1 would stand at that minimum',
          'at that minimum pad 1 would leave the board by 0.300mm' in text, text)
    check("...#1694's way out, not a rule for pads already at 0 mm",
          edge_seat.PAD_ON_OUTLINE in text and 'edge_clearance rule for those pads' not in text,
          text)
    check("...and not the posts' board floor", "0.40mm from the board's" not in text
          and 'would stand 0.10' not in text, text)

    print('...the other sites read the same pads:')
    pose = (x, 15.0, 0.0) if 'J1' in got else (34.5, 15.0, 0.0)
    out = (pose[0] + 0.2, pose[1], pose[2])     # pad 1 0.195 mm past its 0.005
    words = edge_seat._unseated_reason(probe, row, 'east', 1, 0.8, {}, {},
                                       {'copper': [3, [out]]})
    check('the copper tally names pad 1 and its rule',
          "3 with pad copper inside its edge clearance: pad 1, 0.00mm from the .kicad_dru "
          "rule 'launch_pin'" in words, words)
    words = edge_seat._unseated_reason(probe, row, 'east', 1, 0.8, {}, {0.0: 50.0}, {})
    check('a span too long names the largest clearance, of the posts, as what both ends '
          'are held to', "inside it by 0.40mm, the edge clearance of pad 2 (the board's "
          "min_copper_edge_clearance)" in words, words)
    entry = dict(row, max_setback_mm=0.1)
    words = edge_seat._setback_held_text(probe, entry, 'east', [(0.6, 'body:F.Fab', pose)])
    check('the setback clause names pad 1, and where it would stand at the limit',
          "holds it there: pad 1, 0.00mm from the .kicad_dru rule 'launch_pin'" in words
          and 'at its setback limit pad 1 would leave the board by 0.495mm' in words
          and edge_seat.PAD_ON_OUTLINE in words, words)

    print("...through the reading #1694's roster uses:")
    reads = {r.index: r[1:] for r in probe.pad_reads('J1', pose)}
    geometry = ConnectorGeometry(pcb, path)
    rules = PadEdgeRules(pcb, path)
    owed = floorplan._pads_short_of_floor(geometry, rules, pcb, 'J1', 0.4, 0.8, 4.5, '+x')
    check('the roster and the seat give each pad the same number, clearance and rule',
          owed and all(reads[r.index][0] == r.number and reads[r.index][2:] == r[3:]
                       for r in owed), f'{reads} vs {owed}')


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
              and 'at that minimum its pads would stand 0.100mm from the edge' in text, text)
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
        words = edge_seat._unseated_reason(part_probe, entry, 'east', 1, 11.83, {}, {},
                                           {'band_min': [3, [11.7]]})
        check('keeps its plain words', 'under the declared minimum 11.83mm' in words
              and 'Lowering' not in words, words)

        binding_pad(tmp)

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
