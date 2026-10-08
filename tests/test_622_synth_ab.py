#!/usr/bin/env python3
"""Two runs of the generated cases compared case by case (awx/synth_ab.py).

  python3 tests/test_622_synth_ab.py

On two hand-made synth_handoff outputs (handoff.tsv, and each case's run.log with its WHOLE lines) and two
synth_layers tables:

1. a case whose vias moved is named with both values, one only in one run is named as such, and an equal case is not;
2. a handoff case's copper is its run.log's LAST WHOLE line (the best round's), and the totals sum it;
3. a synth_layers table keys a case by tag AND layer count, takes its own copper column, and counts ROUTED as routed
   (synth_layers' OPTIMAL, BETTER and ROUTED; FAIL is the one failure here);
4. two tables of one run joined with a comma read as one run.
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOL = os.path.join(ROOT, 'awx', 'synth_ab.py')
BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


HANDOFF = 'tag\tk\tverdict\trings\tround\tlanes\tvias\tconnected\tdrc\topen\tgap\tturn\thold\tstatic\tsecs\targs\n'


def handoff(d, rows, copper):
    os.makedirs(d)
    with open(os.path.join(d, 'handoff.tsv'), 'w') as f:
        f.write(HANDOFF + ''.join('\t'.join(r) + '\n' for r in rows))
    for tag, mms in copper.items():
        os.makedirs(os.path.join(d, tag))
        with open(os.path.join(d, tag, 'run.log'), 'w') as f:
            for i, mm in enumerate(mms):
                f.write(f'WHOLE K=12 round={i + 1} lanes=12/12 vias=0 copper={mm}mm connected=1 drc=1 secs=9 open=0\n')


LAYERS = 'tag\tlayers\tk\tverdict\tvias\tround\tcopper\tconnected\tdrc\topen\tsecs\n'


def layers(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(LAYERS + ''.join('\t'.join(r) + '\n' for r in rows))


def run(a, b):
    r = subprocess.run([sys.executable, TOOL, a, b], capture_output=True, text=True)
    if r.returncode != 0 or 'cases in both' not in r.stdout:
        print(r.stdout[-2000:], r.stderr[-2000:])
        raise SystemExit('BROKEN TEST: synth_ab printed no totals')
    return r.stdout


with tempfile.TemporaryDirectory() as td:
    row = lambda tag, vias: [tag, '12', 'PASS', "{'S':4}", '1', '12/12', vias, '1', '1', '0', '0.0', '30', '0', '0',  # noqa
                             '40', '--x']
    A, B = os.path.join(td, 'A'), os.path.join(td, 'B')
    handoff(A, [row('same', '2'), row('moved', '2'), row('gone', '0')], {'same': [170], 'moved': [180, 171]})
    handoff(B, [row('same', '2'), row('moved', '4'), row('new', '0')], {'same': [170], 'moved': [175]})
    out = run(A, B)
    print(out)
    # 1.
    check('moved' in out and 'vias 2 -> 4' in out, 'a case whose vias moved is named with both values')
    check('only in A: gone' in out and 'only in B: new' in out, 'a case in one run only is named as such')
    check(not any(ln.strip().startswith('same ') for ln in out.splitlines()), 'an equal case is not named')
    # 2. moved's best round is its last WHOLE line: 171 in A, 175 in B
    check('copper 171 -> 175' in out and 'copper 341 -> 345 mm' in out,
          'a handoff case\'s copper is its last WHOLE line, and the totals sum it')
    # 3. + 4.
    LA, LB = os.path.join(td, 'LA'), os.path.join(td, 'LB')
    layers(os.path.join(LA, 'l3', 'layers.tsv'), [['c', '3', '9', 'OPTIMAL', '12', '1', '129mm', '1', '1', '0', '40']])
    layers(os.path.join(LA, 'l4', 'layers.tsv'), [['c', '4', '9', 'ROUTED', '10', '1', '120mm', '1', '1', '0', '40']])
    layers(os.path.join(LB, 'l3', 'layers.tsv'), [['c', '3', '9', 'OPTIMAL', '12', '1', '129mm', '1', '1', '0', '40']])
    layers(os.path.join(LB, 'l4', 'layers.tsv'), [['c', '4', '9', 'FAIL', '10', '1', '122mm', '1', '1', '0', '40']])
    out = run(f'{LA}/l3,{LA}/l4', f'{LB}/l3,{LB}/l4')
    print(out)
    check('2 cases in both, 1 differ' in out and 'L4' in out and 'verdict ROUTED -> FAIL' in out,
          'a layers table keys a case by tag and layer count; two tables joined with a comma are one run')
    check('copper 249 -> 251 mm' in out, 'a layers row takes its own copper column')
    check('failed 0 -> 1' in out, 'ROUTED counts as routed, FAIL as failed')

print(f'\n{"PASS" if not BAD else "FAIL"}: {len(BAD)} failure(s)' + (': ' + '; '.join(BAD) if BAD else ''))
sys.exit(1 if BAD else 0)
