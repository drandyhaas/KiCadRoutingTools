"""#1507: a cover whose standoff cannot move fixes its connector.

A `covers[]` row's standoffs are a template anchored on the connector, and a
template's cost only PULLS an anchor toward a member that cannot move: with
the standoff at a declared pose, place_simple left the connector millimetres
off and printed "H3 seated as drawn and carried with it". Now each seed seats
the connector at a pose that puts every fixed standoff at its length -- the
next legal one per seed, one per quarter turn -- and holds it; with none
legal it refuses before placing. The NOTE says a member that cannot move is
held, not carried.

The issue's own reproduction: tigard piled, a synthetic cover on J6 built
from tigard's J6/H3 geometry, the four holes declared at a pose.

What is pinned is the cover, not one anneal's legality (#1644): a seed may
exit 4 for a pair its anneal left illegal -- where that happens depends on
the part heights, which kicad-cli reads from 3D models and a host without it
guesses from IPC names -- so a seed passes when it ran to the end and
reported its reason, J6 sits at the seat it announced, and H3 is on the line.

    python3 tests/test_1507_cover_fixed_standoff.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 900
PY = [sys.executable, '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def brief(h3=(33.0, 73.0), standoffs=({'ref': 'H3', 'length_mm': 15.95},)):
    holes = {'H1': (33.0, 33.0), 'H2': (73.0, 33.0), 'H3': h3, 'H4': (73.0, 73.0)}
    return {'schema': 1, 'kind': 'design-brief', 'units': 'mm',
            'fixed': [{'ref': r, 'why': 'enclosure boss',
                       'pose': {'x': x, 'y': y, 'rot': 0, 'side': 'F'}}
                      for r, (x, y) in holes.items()],
            'covers': [{'connector': 'J6', 'side': 'F', 'axis_deg': 90, 'width_mm': 4,
                        'underside_mm': 6, 'length_mm': 16, 'offset_mm': -2.49,
                        'standoffs': list(standoffs), 'removable': True,
                        'source': "synthetic: tigard's own J6/H3 geometry"}]}


def run(argv, **kw):
    return subprocess.run(PY + argv, capture_output=True, text=True, cwd=REPO, **kw)


def setup(td, name, doc):
    d = os.path.join(td, name)
    os.makedirs(d)
    pile = os.path.join(d, 'pile.kicad_pcb')
    run([os.path.join('.claude', 'skills', 'plan-pcb-placement', 'scripts', 'make_unplaced.py'),
         os.path.join('kicad_files', 'tigard.kicad_pcb'), pile])
    run_utils.evidence(pile)
    with open(os.path.join(d, 'pile.design-brief.json'), 'w') as fh:
        json.dump(doc, fh)
    intent = os.path.join(d, 'intent.json')
    run([os.path.join('py_tools', 'check_floorplan.py'), pile, '--emit-intent', intent,
         '--allow-unplaced'])
    run_utils.evidence(intent)
    return d, pile, intent


def place(d, pile, intent, seed):
    out = os.path.join(d, f's{seed}.kicad_pcb')
    r = run([os.path.join('py_placer', 'place_simple.py'), pile, out, '--groups',
             os.path.join('docs', 'examples', 'tigard.groups.json'), '--intent', intent,
             '--seed', str(seed)])
    return r, out


#: place_simple's seat line: `J6 seated where its cover puts H3, which cannot
#: move: at (X, Y) ROT, legal pose K of N`
SEAT = re.compile(r'J6 seated where its cover puts H3, which cannot move: '
                  r'at \(([-\d.]+), ([-\d.]+)\) ([-\d.]+), legal pose (\d+) of (\d+)')


def placed(label, r, out):
    """place_simple ran to the end: exit 0, or 4 for parts its anneal left
    illegal, which it names (its reason is on STDOUT). Returns the seat it
    announced, (x, y, rot, k, n), or None."""
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', r.stdout, re.M)
    summary = json.loads(m.group(1)) if m else {}
    illegal_only = (r.returncode == 4 and summary.get('illegal')
                    and not summary.get('refused') and not summary.get('oversubscribed'))
    reasons = '; '.join(l.strip() for l in r.stdout.splitlines() if 'is left illegal' in l)
    check(f'{label}: place_simple ran to the end (exit 0, or 4 naming what it left illegal)',
          r.returncode == 0 or bool(illegal_only),
          f'rc {r.returncode}: {reasons or r.stdout[-400:]} {r.stderr[-300:]}')
    if r.returncode == 4 and illegal_only:
        print(f'      (exit 4, the anneal\'s, not the cover\'s: {reasons})')
    run_utils.evidence(out, 'the placed board')
    s = SEAT.search(r.stdout)
    check(f'{label}: J6 is seated where its cover puts H3', bool(s), r.stdout[-600:])
    if not s:
        return None
    seat = tuple(float(v) for v in s.groups()[:3]) + (int(s.group(4)), int(s.group(5)))
    x, y, rot = j6_pose(out)
    check(f'{label}: ...and held there', (abs(x - seat[0]) < 0.01 and abs(y - seat[1]) < 0.01
                                           and abs((rot - seat[2]) % 360) < 0.1),
          f'seated {seat[:3]}, placed {(x, y, rot)}')
    return seat


#: the grade's line for H3 at its length on the line (a zero offset may print as -0.00)
ON_LINE = re.compile(r'H3 15\.95mm along \(declared 15\.95\), [+-]0\.00mm off the line')


def graded(out, intent):
    r = run([os.path.join('py_tools', 'check_floorplan.py'), out, '--intent', intent,
             '--no-brief'])
    return r.stdout


def j6_pose(path):
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    from kicad_parser import parse_kicad_pcb
    f = parse_kicad_pcb(path).footprints['J6']
    return round(f.x, 2), round(f.y, 2), round((f.rotation or 0.0) % 360, 1)


def main():
    td = tempfile.mkdtemp(prefix='krt1507_')
    try:
        print('1. a standoff in the corner: one turn is legal')
        d, pile, intent = setup(td, 'corner', brief())
        r, out = place(d, pile, intent, 0)
        seat = placed('seed 0', r, out)
        check('...at the one turn that reaches H3', seat == (48.95, 70.51, 270.0, 1, 1),
              f'{seat}')
        check('...and that H3 is held, not carried',
              'template J6: H3 cannot move: the anchor is held where they are' in r.stdout
              and 'H3 seated as drawn' not in r.stdout)
        g = graded(out, intent)
        check('the grade finds H3 at its length, on the line',
              bool(ON_LINE.search(g)), g[-600:])

        print('2. a standoff in the middle: the seeds take the turns in turn')
        d, pile, intent = setup(td, 'middle', brief(h3=(53.0, 53.0)))
        seats = []
        for s in (0, 1):
            r, out = place(d, pile, intent, s)
            seats.append(placed(f'seed {s}', r, out))
            g = graded(out, intent)
            check(f'seed {s}: H3 at its length, on the line',
                  bool(ON_LINE.search(g)), g[-600:])
        check('the two seeds seat J6 at two different turns, the next legal one each',
              None not in seats and seats[0][2] != seats[1][2] and seats[0][4] > 1
              and [t[3] for t in seats] == [1, 2], f'{seats}')

        print('3. two fixed standoffs the cover cannot both reach: refused')
        d, pile, intent = setup(td, 'disagree', brief(standoffs=(
            {'ref': 'H3', 'length_mm': 15.95}, {'ref': 'H4', 'length_mm': 15.95})))
        r, _out = place(d, pile, intent, 0)
        check('place_simple refuses before placing, naming the cover',
              r.returncode == 2 and 'the cover on J6 holds it to H3, H4' in r.stderr
              and 'its standoffs disagree at every quarter turn' in r.stderr,
              f'rc {r.returncode}: {r.stderr[-400:]}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
