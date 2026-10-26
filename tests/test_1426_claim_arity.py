"""#1426: a `proximity` claim with a limit and no `pads` is priced over every
shared-net pad, ground included, while check_floorplan grades ONE pair (the
closest of the two parts' pads). The objective then calls the claim broken
(`claim_broken`) while the grade passes it, and nothing said so.

place_simple now WARNS, names the claim (parts, limit, leg count, nets, the
rail nets among them) and carries it in its JSON report (`claim_arity`) and in
`judge.json`. A claim that names its pads, or has one leg, is not flagged.

    python3 tests/test_1426_claim_arity.py
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


def _conn(ref, x, y):
    """A 3-pad part (signal, ground, signal) with a drawn courtyard."""
    return (f'  (footprint "t:H3" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -2.2 -1.2) (end 2.2 1.2) (stroke'
            ' (width 0.05) (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -1.5 0) (size 0.8 0.8) (layers "F.Cu")'
            ' (net 1 "SIG_A"))\n'
            '    (pad "2" smd rect (at 0 0) (size 0.8 0.8) (layers "F.Cu")'
            ' (net 2 "GND"))\n'
            '    (pad "3" smd rect (at 1.5 0) (size 0.8 0.8) (layers "F.Cu")'
            ' (net 3 "SIG_B")))\n')


def _board(path):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
                 ' (44 "Edge.Cuts" user))\n  (net 0 "")\n'
                 '  (net 1 "SIG_A")\n  (net 2 "GND")\n  (net 3 "SIG_B")\n'
                 '  (gr_rect (start 0 0) (end 40 30)'
                 ' (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
                 + _conn('J1', 8, 8) + _conn('J2', 30, 22) + ')\n')
    return path


def _run(td, name, claim):
    board = _board(os.path.join(td, f'{name}.kicad_pcb'))
    groups = os.path.join(td, f'{name}_groups.json')
    with open(groups, 'w', encoding='utf-8') as fh:
        json.dump({'schema': 1, 'groups': [{'name': 'all', 'refs': ['J1', 'J2']}]}, fh)
    intent = os.path.join(td, f'{name}_intent.json')
    with open(intent, 'w', encoding='utf-8') as fh:
        json.dump({'kind': 'floorplan-intent', 'schema': 1, 'proximity': [claim]}, fh)
    out = os.path.join(td, f'{name}_out.kicad_pcb')
    js = os.path.join(td, f'{name}.json')
    r = check([sys.executable, '-X', 'utf8', tool('place_simple.py'), board, out,
               '--groups', groups, '--intent', intent, '--seed', '0', '--moves', '600',
               '--json', js], accept=True)
    evidence(js, 'the place_simple report')
    return r.stdout, json.load(open(js))


def main():
    with tempfile.TemporaryDirectory(prefix='t1426_') as td:
        out, rep = _run(td, 'bare', {'ref': 'J1', 'near': 'J2', 'max_mm': 2})
        rows = rep.get('claim_arity')
        ok('a pad-less claim with a limit is carried in the report',
           isinstance(rows, list) and len(rows) == 1
           and rows[0]['legs'] == 3 and rows[0]['rails'] == ['GND']
           and rows[0]['nets'] == ['GND', 'SIG_A', 'SIG_B'], rows)
        ok('... and WARNED, naming the claim, the legs and the rail',
           'WARNING: proximity J1 near J2 (max 2 mm) names no pads' in out
           and '3 shared-net pad legs' in out and 'rail nets: GND' in out, out[-1500:])

        out, rep = _run(td, 'named', {'ref': 'J1', 'near': 'J2', 'max_mm': 2,
                                      'pads': {'J1': ['1'], 'J2': ['1']}})
        ok('a claim that names its pads is not flagged',
           rep.get('claim_arity') == [] and 'names no pads' not in out,
           rep.get('claim_arity'))
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
