"""#1508: cover_facts withholds pose-derived facts on a pile.

The placement skill runs cover_facts on `fixed.kicad_pcb`, which is a pile in
a from-scratch run, and on a pile it listed staging neighbours as standoff
candidates at sub-mm distances and counted staged parts "inside the raised
extent now", with no notice. It now applies #1353's rule, through the same
pile predicate board_context uses: a standoff distance or an under-extent
part is withheld unless both parts are KiCad-locked, the PILE notice comes
first, and the JSON carries `pile` with what was withheld. A placed board is
unchanged.

    python3 tests/test_1508_cover_facts_pile.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def run(argv):
    return subprocess.run(PY + argv, capture_output=True, text=True, cwd=REPO)


def facts(board, out):
    r = run([os.path.join('py_tools', 'cover_facts.py'), board, '--refs', 'J6', '--json', out])
    run_utils.evidence(out)
    with open(out) as fh:
        return r, json.load(fh)


def main():
    td = tempfile.mkdtemp(prefix='krt1508_')
    try:
        placed = os.path.join(td, 'placed.kicad_pcb')
        staged = os.path.join(td, 'staged.kicad_pcb')
        run([os.path.join('py_router', 'copy_board.py'),
             os.path.join('kicad_files', 'tigard.kicad_pcb'), placed])
        run([os.path.join('.claude', 'skills', 'plan-pcb-placement', 'scripts', 'make_unplaced.py'),
             os.path.join('kicad_files', 'tigard.kicad_pcb'), staged])
        run_utils.evidence(placed)
        run_utils.evidence(staged)

        print('1. the placed board: every fact, no pile')
        r, d = facts(placed, os.path.join(td, 'p.json'))
        c = d['connectors'][0]
        got = {(s['ref'], s['axis_deg'], s['along_mm']) for s in c['standoff_candidates']}
        check('exit 0', r.returncode == 0, r.stderr[-300:])
        check('no pile key, no notice', d.get('pile') is None and 'PILE' not in r.stdout)
        check("H3 and H4 are J6's standoff candidates at 15.95 and 24.05 mm",
              {('H3', 90, 15.95), ('H4', 270, 24.05)} <= got, f'{got}')

        print('2. the same board piled: the pose facts withheld, and said so first')
        r, d = facts(staged, os.path.join(td, 's.json'))
        c = d['connectors'][0]
        check('exit 0', r.returncode == 0, r.stderr[-300:])
        out = r.stdout.splitlines()
        check('the PILE notice comes before any connector',
              len(out) > 1 and out[1].startswith('  This board is a PILE'), '\n'.join(out[:3]))
        check('the JSON says pile, with what was withheld',
              bool(d.get('pile')) and d['pile']['withheld']['standoff_candidates'] >= 4,
              json.dumps(d.get('pile')))
        check('no staging neighbour is listed as a standoff', c['standoff_candidates'] == [],
              json.dumps(c['standoff_candidates']))
        check('no staged part is counted inside the raised extent',
              not c.get('inside_raised')
              and (('inside_raised' not in c) or c.get('inside_raised_withheld', 0) > 0),
              json.dumps({k: c.get(k) for k in ('inside_raised', 'inside_raised_withheld')}))
        check('the pose-free facts stay: footprint, height and models',
              c['footprint'] and 'height_mm' in c and 'models' in c)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
