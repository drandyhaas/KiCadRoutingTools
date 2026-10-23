#!/usr/bin/env python3
"""check_floorplan reports an edge whose declared connectors cannot all seat
together, BEFORE placing, from the measure place_simple seats them by.

place_simple has reported such an edge since its seats were packed per edge
(`edge_seat._crowding`, `oversubscribed` in judge.json); a check of the plan
said nothing. CM5_MINIMA's intent declares J501 (16.90 mm) and J802 (17.10
mm) on its south edge where one fits: check_floorplan on its pile graded no
such finding, and only the placement run said so.

`floorplan.edge_oversubscribed_findings` now raises `edge_oversubscribed`,
WARN unless the intent's severity map promotes it, from
`edge_seat.oversubscribed` -- which calls `edge_seat.seats` on the board
place_simple seats on (declared faces and poses applied to a scratch copy,
the same state) -- in `grade(edge_seats=True)` (check_floorplan asks for it)
and in `plan_check` (`--plan-only`).

Checks, on small boards built here (a 16 mm gap between fixed parts on the
south edge, as tests/test_edge_oversubscribed.py builds them):
  1. two 14 mm parts declared on it: check_floorplan WARNs once, naming the
     edge, both parts and the fix; the record is the one `edge_seat.seats`
     makes;
  2. the intent's severity map promotes it to an error;
  3. `--plan-only` reports it too;
  4. a seed place_simple placed from that board reads the same record (the
     seat lines do not depend on where the movable parts stand);
  5. the second part declared on the B face, the face place_simple flips it
     to: they fit, and nothing is reported -- the declared face is applied
     before the measure;
  6. NEGATIVE CONTROLS: two 7 mm parts that fit report nothing; and with
     `edge_seat._crowding` finding no row short, the grade reports nothing
     either -- it reads the seat measure, not a copy of it.

    python3 tests/test_edge_oversubscribed_grade.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
from unittest import mock

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
import pose_score                                              # noqa: E402
from placement import edge_seat, floorplan                     # noqa: E402
from run_utils import evidence                                 # noqa: E402

failures = []
CLEARANCE, EDGE = 0.25, 0.55


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def footprint(ref, at, half, side='F', locked=False):
    """A part with a 2*half x 6 mm courtyard (a wall: 8 mm deep) and two pads."""
    deep = 4 if locked else 3
    lo, hi = (-half[0], half[1]) if isinstance(half, tuple) else (-half, half)
    pads = (f'    (pad "1" smd rect (at 0 0) (size 1 1) (layers "{side}.Cu" "{side}.Mask"))\n'
            if locked else
            ''.join(f'    (pad "{i + 1}" smd rect (at {x} 0) (size 1 1) '
                    f'(layers "{side}.Cu" "{side}.Mask"))\n'
                    for i, x in enumerate((lo + 2, hi - 2))))
    return (f'  (footprint "Test:Part"{" (locked yes)" if locked else ""} '
            f'(layer "{side}.Cu") (at {at})\n'
            f'    (property "Reference" "{ref}" (at 0 0 0) (layer "{side}.SilkS")'
            f' (effects (font (size 1 1) (thickness 0.15))))\n'
            f'    (fp_rect (start {lo} {-deep}) (end {hi} {deep}) (layer "{side}.CrtYd"))\n'
            + pads + '  )\n')


def walls(side='F', tag=''):
    """Two locked parts on the south edge, leaving x 20-36 (16 mm) free."""
    return (footprint(f'W1{tag}', '10 35', (9.4, 10.0), side, locked=True)
            + footprint(f'W2{tag}', '48 35', (12.0, 11.4), side, locked=True))


def board(path, parts):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "t_edge_grade")\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n'
                 '  (gr_rect (start 0 0) (end 60 40) (layer "Edge.Cuts"))\n'
                 + ''.join(parts) + ')\n')
    with open(os.path.splitext(path)[0] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'net_settings': {'classes': [
            {'name': 'Default', 'clearance': CLEARANCE, 'track_width': 0.2}]},
            'board': {'design_settings': {'rules': {
                'min_copper_edge_clearance': EDGE}}}}, fh)
    return evidence(path, 'synthetic board')


def row(ref, face='+y', side=None):
    out = {'ref': ref, 'edge': 'south', 'class': 'edge_receptacle', 'mating_face': face}
    if side:
        out['side'] = side
    return out


def intent_file(path, rows, severity=None):
    doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
           'edge_connectors': rows}
    if severity:
        doc['severity'] = severity
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    return path


def check_floorplan(pcb, intent, *extra):
    """`(exit, [edge_oversubscribed findings], every finding)` of check_floorplan."""
    out = pcb + '.json'
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), pcb,
                        '--intent', intent, '--allow-unplaced', '--json', out, *extra],
                       capture_output=True, text=True, encoding='utf-8', cwd=ROOT)
    if not os.path.isfile(out):
        return r.returncode, None, (r.stdout + r.stderr)[-1500:]
    doc = json.load(open(out, encoding='utf-8'))
    found = doc.get('violations') or doc.get('plan_findings') or []
    return r.returncode, [v for v in found if v['rule'] == 'edge_oversubscribed'], found


def seats_record(path, rows):
    """The record `edge_seat.seats` itself makes on the board."""
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        state = pose_score.make_state(pcb, path, clearance=CLEARANCE,
                                      board_edge_clearance=EDGE)
        movable = {r for r, p in state.parts.items() if not p.locked}
        crowded = []
        edge_seat.seats(state, pcb, path,
                        floorplan.intent_from_dict({'schema': floorplan.SCHEMA_VERSION,
                                                    'kind': floorplan.KIND, 'units': 'mm',
                                                    'edge_connectors': rows}),
                        movable, crowded=crowded)
    return crowded


def same(found, rec):
    m = found['measured']
    return (m['edge'], m['side'], m['parts'], m['free'], m['fit'], m['seated'],
            m['unseated']) == (rec['edge'], rec['side'], [p['ref'] for p in rec['parts']],
                               rec['free'], rec['fit'], rec['seated'], rec['unseated'])


def main():
    tmp = tempfile.mkdtemp(prefix='t_edge_grade_')
    try:
        two = board(os.path.join(tmp, 'two.kicad_pcb'),
                    [walls(), walls('B', 'b'), footprint('J1', '15 10 0', 7),
                     footprint('J2', '45 10 0', 7)])
        rows = [row('J1'), row('J2')]
        intent = intent_file(os.path.join(tmp, 'intent.json'), rows)

        # 1. the WARN, the words, and the seat measure's own record
        code, found, every = check_floorplan(two, intent)
        rec = seats_record(two, rows)
        f = found[0] if found and len(found) == 1 else None
        check('1. one edge_oversubscribed finding, a WARN', f is not None
              and f['severity'] == 'warn', f'{found if found is not None else every}')
        check('1. it names the edge, both parts and what to change', f is not None and all(
            w in f['message'] for w in ('south edge', 'J1', 'J2', 'cannot seat',
                                        'another edge', 'drop a declaration'))
              and f['measured']['unseated'] == ['J2'], f['message'] if f else '')
        check("1. the record is edge_seat.seats' own", f is not None and len(rec) == 1
              and same(f, rec[0]), json.dumps([f and f['measured'], rec]))

        # 2. promoted
        hard = intent_file(os.path.join(tmp, 'hard.json'), rows,
                           severity={'edge_oversubscribed': 'error'})
        code2, found2, _e = check_floorplan(two, hard)
        check('2. the severity map promotes it to an error, exit 4',
              found2 and [v['severity'] for v in found2] == ['error'] and code2 == 4,
              f'exit {code2}, {found2}')

        # 3. the plan check before placing
        code3, found3, every3 = check_floorplan(two, intent, '--plan-only')
        check('3. --plan-only reports it, a WARN', found3 is not None and len(found3) == 1
              and found3[0]['severity'] == 'warn' and same(found3[0], rec[0]),
              f'{found3 if found3 is not None else every3}')

        # 4. a placed seed reads the same record
        run = os.path.join(tmp, 'run')
        os.makedirs(run)
        groups = os.path.join(tmp, 'groups.json')
        with open(groups, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'groups': [{'name': 'io', 'refs': ['J1', 'J2']}]}, fh)
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_placer', 'place_simple.py'), two,
                            os.path.join(run, 's{seed}.kicad_pcb'), '--groups', groups,
                            '--intent', intent, '--seeds', '0', '--jobs', '1'],
                           capture_output=True, text=True, encoding='utf-8', cwd=ROOT)
        placed = os.path.join(run, 's0.kicad_pcb')
        if not os.path.isfile(placed):
            check('4. place_simple wrote the seed', False,
                  f'exit {r.returncode}: {(r.stdout + r.stderr)[-1500:]}')
        else:
            _c4, found4, every4 = check_floorplan(placed, intent)
            judge = json.load(open(os.path.join(run, 'judge.json'), encoding='utf-8'))
            check("4. the placed seed reads place_simple's own record",
                  found4 is not None and len(found4) == 1
                  and len(judge.get('oversubscribed', ())) == 1
                  and same(found4[0], judge['oversubscribed'][0]),
                  f"{found4 if found4 is not None else every4} {judge.get('oversubscribed')}")

        # 5. J2 declared on the B face: place_simple flips it, and they fit
        sided = intent_file(os.path.join(tmp, 'sided.json'), [row('J1'), row('J2', None, 'B')])
        _c5, found5, every5 = check_floorplan(two, sided)
        check('5. declared on two faces, nothing is reported', found5 == [],
              f'{found5 if found5 is not None else every5}')

        # 6. negative controls
        fits = board(os.path.join(tmp, 'fits.kicad_pcb'),
                     [walls(), footprint('J3', '35 10 0', 3.5), footprint('J4', '50 10 0', 3.5)])
        small = intent_file(os.path.join(tmp, 'small.json'), [row('J3'), row('J4')])
        _c6, found6, every6 = check_floorplan(fits, small)
        check('6. two parts that fit: nothing reported', found6 == [],
              f'{found6 if found6 is not None else every6}')
        with mock.patch.object(edge_seat, '_crowding', lambda *a, **k: []), \
                contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            res = floorplan.grade(floorplan.load_intent(intent), parse_kicad_pcb(two), two,
                                  clearance=CLEARANCE, board_edge_clearance=EDGE,
                                  edge_seats=True)
        check("6. NEGATIVE CONTROL: with no row short in the seat measure, the grade says nothing",
              not [v for v in res.violations if v.rule == 'edge_oversubscribed'])
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            res = floorplan.grade(floorplan.load_intent(intent), parse_kicad_pcb(two), two,
                                  clearance=CLEARANCE, board_edge_clearance=EDGE,
                                  edge_seats=True)
        check('6. and unmocked, the same grade reports it',
              len([v for v in res.violations if v.rule == 'edge_oversubscribed']) == 1)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
