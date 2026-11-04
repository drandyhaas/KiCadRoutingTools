"""#1484: on a pile, --emit-intent claims no block side read off a face.

make_unplaced keeps each part's face, so on a from-scratch pile every block
whose members all sit on the front got `side: "F"` -- which place_simple
reads as a declared face, so a flippable part in it could never flip. A
pile's faces are staging, like its poses (#1103): the side is withheld and
named in `pose_claims_withheld`, while a placed board keeps it.
"""

import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
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
        bare, pile = os.path.join(td, 'bare.kicad_pcb'), os.path.join(td, 'pile.kicad_pcb')
        run(os.path.join(ROOT, 'tests', 'stress', 'strip_copper_only.py'), BOARD, bare)
        run(os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'scripts',
                         'make_unplaced.py'), bare, pile)
        placed_i, pile_i = os.path.join(td, 'placed.json'), os.path.join(td, 'pile.json')
        cf = os.path.join(ROOT, 'py_tools', 'check_floorplan.py')
        run(cf, bare, '--emit-intent', placed_i)
        run(cf, pile, '--allow-unplaced', '--emit-intent', pile_i)
        placed, piled = json.load(open(placed_i)), json.load(open(pile_i))
        ok('a placed board keeps the side its blocks sit on',
           any(b.get('side') for b in placed['blocks']), placed['blocks'])
        ok('a pile claims none', piled['blocks'] and not any(b.get('side') for b in piled['blocks']),
           [(b['name'], b.get('side')) for b in piled['blocks']])
        ok('and says it withheld them', 'blocks[].side read off a face'
           in piled['context']['pose_claims_withheld']['withheld'])
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
