"""#1481: place_simple --preview, where each group would go, non-binding.

The anchored start's own solve cannot be shown on its own: on a pile with
nothing fixed it is translation-invariant and puts every group on the board's
centre. So the preview places ONE real seed (`--seed`, from the anchored
start) in a scratch directory and reports where each group landed -- a
table, a PNG of that board with the boxes as draft zones, and with
`--draft-zones` the boxes as draft `blocks[]` -- writing no board.
"""

import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PILE = os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
GROUPS = {'schema': 1, 'groups': [
    {'name': 'usb_in', 'refs': ['USB1'], 'anchor': 'USB1', 'near': ['power']},
    {'name': 'power', 'refs': ['U2', 'C1', 'C3'], 'anchor': 'U2', 'near': ['usb_in', 'core']},
    {'name': 'core', 'refs': ['U1', 'Y1', 'C2', 'C4', 'R3', 'R4'], 'anchor': 'U1'},
    {'name': 'autoreset', 'refs': ['Q1', 'Q2', 'R1', 'R2'], 'anchor': 'Q1', 'near': ['prog']},
    {'name': 'prog', 'refs': ['CON1', 'CON2', 'Ref*', 'Ref*~2'], 'anchor': 'CON1'}]}
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
        intent = os.path.join(td, 'intent.json')
        run(os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), PILE, '--emit-intent', intent,
            '--allow-unplaced')
        groups = os.path.join(td, 'groups.json')
        json.dump(GROUPS, open(groups, 'w'))
        png, draft = os.path.join(td, 'preview.png'), os.path.join(td, 'draft.json')
        ps = os.path.join(ROOT, 'py_placer', 'place_simple.py')
        r = run(ps, PILE, png, '--groups', groups, '--intent', intent, '--preview',
                '--draft-zones', draft)
        ok('the preview runs', r.returncode == 0, r.stderr[-400:] or r.stdout[-400:])
        ok('it draws the PNG', os.path.exists(png))
        ok('and writes no board', not [f for f in os.listdir(td) if f.endswith('.kicad_pcb')],
           os.listdir(td))
        blocks = json.load(open(draft))['blocks'] if os.path.exists(draft) else []
        ok('one draft zone per group, its members its refs',
           [b['refs'] for b in blocks] == [g['refs'] for g in GROUPS['groups']], blocks)
        centres = {((b['zone'][0] + b['zone'][2]) / 2, (b['zone'][1] + b['zone'][3]) / 2)
                   for b in blocks}
        ok('the groups land apart, not on one centre', len(centres) == len(blocks), centres)
        table = [line for line in r.stdout.splitlines() if 'part(s)  centre' in line]
        ok('the table names every group', len(table) == len(blocks), r.stdout[-300:])
        doc = json.load(open(intent))
        doc['blocks'] = blocks
        pi = os.path.join(td, 'pi.json')
        json.dump(doc, open(pi, 'w'))
        r2 = run(os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), PILE, '--intent', pi,
                 '--plan-only', '--allow-unplaced')
        ok('the draft is an intent the plan check reads (no ERROR)', r2.returncode == 0,
           r2.stdout[-300:])
        r3 = run(ps, PILE, png, '--groups', groups, '--intent', intent, '--preview',
                 '--seeds', '0-1')
        ok('--preview with --seeds is refused', r3.returncode == 2, r3.stderr[-200:])
        r4 = run(ps, PILE, os.path.join(td, 'x.kicad_pcb'), '--groups', groups, '--intent',
                 intent, '--seed', '0', '--draft-zones', draft)
        ok('--draft-zones without --preview is refused', r4.returncode == 2, r4.stderr[-200:])
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
