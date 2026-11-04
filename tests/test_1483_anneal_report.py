"""#1483: place_simple says how many anneals it ran, and how long one takes.

A seed can anneal many times -- restarts until one is legal, the #1300
courtyard fallback, the face pass -- and the log line and the JSON used to
report the kept run alone. Now the log names the worst case up front, each
anneal prints an estimate after its first tenth of moves, and the final line
and the JSON (`anneals`, `moves_total`) count every anneal that ran.

The case: a front-only keep-out over the middle of a small board leaves the
front too small, so every pass runs.
"""

import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
GROUPS = {'schema': 1, 'groups': [
    {'name': 'power', 'refs': ['U2', 'C1', 'C3'], 'anchor': 'U2', 'flippable': ['C1', 'C3']},
    {'name': 'core', 'refs': ['U1', 'Y1', 'C2', 'C4', 'R3', 'R4'], 'anchor': 'U1',
     'flippable': ['C2', 'C4', 'R3', 'R4']},
    {'name': 'autoreset', 'refs': ['Q1', 'Q2', 'R1', 'R2'], 'anchor': 'Q1',
     'flippable': ['Q1', 'Q2', 'R1', 'R2']}]}
FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def run(*args):
    return subprocess.run([sys.executable, '-X', 'utf8'] + list(args), capture_output=True,
                          text=True, cwd=ROOT)


def main():
    with tempfile.TemporaryDirectory() as td:
        fixed = os.path.join(td, 'fixed.kicad_pcb')
        run(os.path.join(ROOT, 'py_placer', 'place_pose.py'), BOARD, fixed,
            'lock', 'USB1', 'CON1', 'CON2', 'Ref*', 'Ref*~2')
        intent = os.path.join(td, 'intent.json')
        run(os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), fixed, '--emit-intent', intent)
        d = json.load(open(intent))
        d['assembly']['sides'] = 'both'
        d['keepouts'] = [{'name': 'front_full', 'rect': [121.5, 94.5, 135.0, 105.5],
                          'sides': ['F']}]
        json.dump(d, open(intent, 'w'))
        groups = os.path.join(td, 'groups.json')
        json.dump(GROUPS, open(groups, 'w'))
        js = os.path.join(td, 's.json')
        r = run(os.path.join(ROOT, 'py_placer', 'place_simple.py'), fixed,
                os.path.join(td, 's.kicad_pcb'), '--groups', groups, '--intent', intent,
                '--seed', '3', '--json', js)
        ok('place_simple ran', os.path.exists(js), r.stderr[-300:])
        if not os.path.exists(js):
            return 1
        j = json.load(open(js))
        n = len(j['anneals'])
        labels = [p['pass'] for p in j['anneals']]
        ok('more than one anneal ran, every pass named', n > 1
           and set(labels) <= {'front', 'fallback', 'face'} and 'front' in labels, labels)
        ok('moves_total is every anneal\'s moves', j['moves_total']
           == sum(p['moves'] for p in j['anneals']) > j['moves'], (j['moves_total'], j['moves']))
        up = re.search(r'up to (\d+) anneal\(s\) in all', r.stdout)
        ok('the worst case is named up front, and holds', up and int(up.group(1)) >= n,
           r.stdout[:300])
        line = re.search(r'annealed \d+ moves in [\d.]+ s \((\d+) anneal\(s\), (\d+) moves in all',
                         r.stdout)
        ok('the final line counts them all', line and int(line.group(1)) == n
           and int(line.group(2)) == j['moves_total'], line and line.group(0))
        ok('each anneal prints an estimate',
           len(re.findall(r'estimate: about \d+ s for this anneal', r.stdout)) == n)
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
