"""#1386: place_simple prices a part's drawn courtyard, but check_assembly gates
courtyard PLUS pad copper, so a neighbour can be parked over a pad that reaches
past its owner's courtyard and the seed grades NOT BUILDABLE.

The pre-flight now names each part whose pad copper reaches further than
`PAD_REACH_WARN_MM` past its drawn courtyard (locked ones marked), before the
anneal, and the report carries them (`pad_overhang`). An ordinary part, whose
pads sit inside its courtyard, is not named.

    python3 tests/test_1386_pad_overhang_preflight.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check, evidence, tool                   # noqa: E402

FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _part(ref, x, y, pad2_x, locked=False):
    """A 3 x 2 mm courtyard with pad 1 inside it and pad 2 at `pad2_x`."""
    lk = '(locked yes) ' if locked else ''
    return (f'  (footprint "t:P" {lk}(layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -1.5 -1) (end 1.5 1) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -0.8 0) (size 0.8 0.8) (layers "F.Cu")'
            ' (net 1 "N1"))\n'
            f'    (pad "2" smd rect (at {pad2_x} 0) (size 1.0 1.0) (layers "F.Cu")'
            ' (net 2 "N2")))\n')


def main():
    with tempfile.TemporaryDirectory(prefix='t1386_') as td:
        board = os.path.join(td, 'b.kicad_pcb')
        with open(board, 'w', encoding='utf-8') as fh:
            fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                     '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                     '  (net 0 "")\n  (net 1 "N1")\n  (net 2 "N2")\n'
                     '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1)'
                     ' (type default)) (layer "Edge.Cuts"))\n'
                     + _part('J1', 10, 10, 2.5, locked=True)    # a pad 1.5 mm past
                     + _part('J2', 30, 20, 2.5)                  # likewise, movable
                     + _part('R1', 20, 5, 0.8) + ')\n')
        groups = os.path.join(td, 'groups.json')
        json.dump({'schema': 1, 'groups': [{'name': 'all', 'refs': ['J1', 'J2', 'R1']}]},
                  open(groups, 'w'))
        intent = os.path.join(td, 'intent.json')
        json.dump({'kind': 'floorplan-intent', 'schema': 1}, open(intent, 'w'))
        js = os.path.join(td, 'r.json')
        r = check([sys.executable, '-X', 'utf8', tool('place_simple.py'), board,
                   os.path.join(td, 'out.kicad_pcb'), '--groups', groups, '--intent', intent,
                   '--seed', '0', '--moves', '300', '--json', js], accept=True)
        evidence(js, 'the place_simple report')
        rep = json.load(open(js))
        ok('the parts whose pads reach past the courtyard are named, locked one marked',
           'WARNING: pad copper reaches past the drawn courtyard: J1 1.50 mm (locked), '
           'J2 1.50 mm;' in r.stdout, r.stdout[-900:])
        ok('... an ordinary part is not, and the report carries the two',
           sorted(rep['pad_overhang']) == ['J1', 'J2'], rep['pad_overhang'])
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
