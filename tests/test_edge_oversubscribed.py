#!/usr/bin/env python3
"""Edge connectors that cannot all fit on their declared edge are the
intent's to fix, reported once, and the seeds stay judgeable.

CM5_MINIMA's intent declares J501 (HDMI, 16.90 mm) and J802 (camera FFC,
17.10 mm) both on its south edge, and between the fixed parts they can stand
only at x 89.43-94.10 and 89.59-103.40: one at a time. Each had a seat of
its own, so both were seated; every seed then annealed three times (11 s to
33 s on seed 7), left both illegal, and judge.json named no seed, the cause
buried in each seed's `illegal` list.

`edge_seat.seats` now packs each edge's seated parts along it, one row per
edge and face, exactly over every subset (`_crowding`). When they do not all
fit, the most that fit together keep their seats, the earlier declared on a
tie, and the rest are placed as any other part; a record names the edge, the
parts, each one's length and where it can stand, and the free span.
place_simple prints it once, carries it as `oversubscribed` in each seed's
summary and in judge.json, beside `refused`, and exits 4; the seeds are
judged.

Checks, on small boards built here (a 16 mm gap between fixed parts on the
south edge):
  1. two 14 mm parts: the first declared keeps its seat, the other is back
     where it stood and its seat note is gone; the record names both, their
     lengths, ranges and the free span;
  2. declared the other way round, the other one is seated;
  3. the most that fit outrank the declared order: a 14 mm part declared
     before two 7 mm parts that fit together is the one placed freely;
  4. two 7 mm parts fit: no record, both seated (the seat is unchanged);
  5. two 14 mm parts on opposite faces, the gap walled on both: no record
     (courtyards on two faces never meet);
  6. `crowding_text` says what to change;
  7. end to end, place_simple over two seeds: exit 4, the warning printed
     once by the run, `oversubscribed` in judge.json and in each seed's
     summary, no seed illegal, and both seeds named for judging.

    python3 tests/test_edge_oversubscribed.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

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
            f'    (property "Reference" "{ref}" (at 0 0 0) (layer "{side}.SilkS"))\n'
            f'    (fp_rect (start {lo} {-deep}) (end {hi} {deep}) (layer "{side}.CrtYd"))\n'
            + pads + '  )\n')


def walls(side='F', tag=''):
    """Two locked parts on the south edge, leaving x 20-36 (16 mm) free."""
    return (footprint(f'W1{tag}', '10 35', (9.4, 10.0), side, locked=True)
            + footprint(f'W2{tag}', '48 35', (12.0, 11.4), side, locked=True))


def board(path, parts):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "t_edge_full")\n  (net 0 "")\n'
                 '  (gr_rect (start 0 0) (end 60 40) (layer "Edge.Cuts"))\n'
                 + ''.join(parts) + ')\n')
    return evidence(path, 'synthetic board')


def row(ref, face='+y'):
    out = {'ref': ref, 'edge': 'south', 'class': 'edge_receptacle'}
    if face:
        out['mating_face'] = face
    return out


def intent_doc(rows):
    return {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
            'units': 'mm', 'edge_connectors': rows}


def seat(path, rows):
    """`(state, seats, notes, crowded, poses before)`."""
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        state = pose_score.make_state(pcb, path, clearance=CLEARANCE,
                                      board_edge_clearance=EDGE)
        before = {r: (p.x, p.y, p.rot) for r, p in state.parts.items()}
        movable = {r for r, p in state.parts.items() if not p.locked}
        crowded = []
        out, notes, _unseated = edge_seat.seats(
            state, pcb, path, floorplan.intent_from_dict(intent_doc(rows)), movable,
            crowded=crowded)
    return state, out, notes, crowded, before


def main():
    tmp = tempfile.mkdtemp(prefix='t_edge_full_')
    try:
        two = board(os.path.join(tmp, 'two.kicad_pcb'),
                    [walls(), footprint('J1', '15 10 0', 7), footprint('J2', '45 10 0', 7)])

        # 1. one of two fits: the first declared keeps its seat
        state, out, notes, crowded, before = seat(two, [row('J1'), row('J2')])
        rec = crowded[0] if len(crowded) == 1 else {}
        check('1. the first declared keeps its seat, the other has none',
              'J1' in out and 'J2' not in out, f'{sorted(out)}')
        check('1. one record, for the south edge, F side, 1 of 2 fit',
              rec.get('edge') == 'south' and rec.get('side') == 'F'
              and rec.get('fit') == 1 and rec.get('seated') == ['J1']
              and rec.get('unseated') == ['J2'], json.dumps(crowded))
        parts = {p['ref']: p for p in rec.get('parts', ())}
        j1 = (parts.get('J1') or {}).get('turns') or [{}]
        check('1. each part: its length along the edge, where it can stand, its span',
              set(parts) == {'J1', 'J2'} and abs(j1[0].get('length_mm', 0) - 14.0) < 0.01
              and len(j1[0].get('at', ())) == 1
              and 26.9 < j1[0]['at'][0][0] < j1[0]['at'][0][1] < 29.1
              and 19.9 < j1[0]['span'][0][0] and j1[0]['span'][0][1] < 36.1,
              json.dumps(parts.get('J1')))
        check('1. the free span is the 16 mm gap',
              len(rec.get('free', ())) == 1
              and abs(rec['free'][0][0] - 20.0) < 0.05 and abs(rec['free'][0][1] - 36.0) < 0.05,
              str(rec.get('free')))
        p = state.parts['J2']
        check('1. the other is back where it stood, its seat note gone',
              (p.x, p.y, p.rot) == before['J2']
              and not any('J2' in n for n in notes) and any('J1' in n for n in notes),
              f'{(p.x, p.y, p.rot)} {notes}')

        # 2. declared the other way round
        _s, out, _n, crowded, _b = seat(two, [row('J2'), row('J1')])
        check('2. declared first, J2 keeps the seat',
              'J2' in out and 'J1' not in out
              and [c.get('seated') for c in crowded] == [['J2']], json.dumps(crowded))

        # 3. the most that fit outrank the declared order
        three = board(os.path.join(tmp, 'three.kicad_pcb'),
                      [walls(), footprint('J1', '15 10 0', 7),
                       footprint('J3', '35 10 0', 3.5), footprint('J4', '50 10 0', 3.5)])
        _s, out, _n, crowded, _b = seat(three, [row('J1'), row('J3'), row('J4')])
        check('3. the two that fit together beat the one declared first',
              sorted(out) == ['J3', 'J4']
              and [(c.get('fit'), c.get('unseated')) for c in crowded] == [(2, ['J1'])],
              f'{sorted(out)} {json.dumps(crowded)}')

        # 4. they fit: nothing to report, the seat unchanged
        _s, out, _n, crowded, _b = seat(three, [row('J3'), row('J4')])
        check('4. two that fit are both seated, no record',
              sorted(out) == ['J3', 'J4'] and crowded == [], json.dumps(crowded))

        # 5. opposite faces, each walled: abreast, not in a row
        faces = board(os.path.join(tmp, 'faces.kicad_pcb'),
                      [walls(), walls('B', 'b'), footprint('J1', '15 10 0', 7),
                       footprint('J2', '45 10 0', 7, side='B')])
        state, out, _n, crowded, _b = seat(faces, [row('J1'), row('J2', face=None)])
        check('5. two faces: both seated, no record',
              sorted(out) == ['J1', 'J2'] and crowded == []
              and state.parts['J2'].side == 'B', f'{sorted(out)} {json.dumps(crowded)}')

        # 6. the words
        _s, _o, _n, crowded, _b = seat(two, [row('J1'), row('J2')])
        text = edge_seat.crowding_text(crowded[0]) if crowded else ''
        check('6. the text names the edge, the parts, the lengths, the span and the fix',
              all(w in text for w in ('south edge', 'J1', 'J2', '14.00 mm long',
                                      'free span x 20.0', 'Seated: J1',
                                      'placed without its edge claim: J2',
                                      'another edge', 'drop a declaration', 'widen a band')),
              text)

        # 7. end to end: two seeds, the warning once, both judged
        run = os.path.join(tmp, 'run')
        os.makedirs(run)
        intent = os.path.join(tmp, 'intent.json')
        groups = os.path.join(tmp, 'groups.json')
        with open(intent, 'w', encoding='utf-8') as fh:
            json.dump(intent_doc([row('J1'), row('J2')]), fh)
        with open(groups, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'groups': [{'name': 'io', 'refs': ['J1', 'J2']}]}, fh)
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_placer', 'place_simple.py'), two,
                            os.path.join(run, 's{seed}.kicad_pcb'), '--groups', groups,
                            '--intent', intent, '--seeds', '0-1', '--jobs', '1'],
                           capture_output=True, text=True, encoding='utf-8', cwd=ROOT)
        judge_path = os.path.join(run, 'judge.json')
        if not os.path.isfile(judge_path):
            check('7. place_simple wrote judge.json', False,
                  f'exit {r.returncode}: {(r.stdout + r.stderr)[-2000:]}')
        else:
            judge = json.load(open(judge_path, encoding='utf-8'))
            warned = [ln for ln in r.stdout.splitlines()
                      if 'WARNING' in ln and 'cannot seat J1 and J2 together' in ln]
            check('7. exit 4, the warning once in the run', r.returncode == 4
                  and len(warned) == 1, f'exit {r.returncode}, {len(warned)} warning(s)')
            check('7. judge.json carries the record and names both seeds',
                  [c.get('unseated') for c in judge.get('oversubscribed', ())] == [['J2']]
                  and sorted(judge.get('judge', ())) == [0, 1] and judge.get('illegal') == [],
                  json.dumps({k: judge.get(k) for k in ('judge', 'illegal', 'oversubscribed')}))
            summaries = []
            for s in (0, 1):
                with open(os.path.join(run, f's{s}.log'), encoding='utf-8') as fh:
                    summaries += [json.loads(ln[len('JSON_SUMMARY: '):]) for ln in fh
                                  if ln.startswith('JSON_SUMMARY: ')]
            check("7. each seed's summary carries it, no part illegal",
                  len(summaries) == 2 and all(
                      s.get('oversubscribed') == judge.get('oversubscribed')
                      and s.get('illegal') == [] for s in summaries),
                  json.dumps([{k: s.get(k) for k in ('illegal', 'oversubscribed')}
                              for s in summaries]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
