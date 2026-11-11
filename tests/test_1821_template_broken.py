"""#1821: a template member far off its template pose is a floorplan ERROR
(`template_broken`), so the seed judge ranks that seed below one that kept
the stage together.

Drift had one severity: a member 0.5 mm off and one 17.6 mm off were the
same `template` WARN, and `place_simple.judge_order` ranked neither seed
lower. A member whose worst pad is past `TEMPLATE_BROKEN_FACTOR` (5) x the
template's `tolerance_mm` is now `template_broken`, ERROR by default; between
1 x and 5 x it stays the WARN `template`.

On #1387's fixture (U1 locked, its template putting L1 at (24, 15),
tolerance 0.3 mm):
  B1. L1 at 4 x the tolerance (1.2 mm off): `template` WARN, no
      `template_broken`, check_floorplan exits 0.
  B2. L1 at 6 x (1.8 mm off): `template_broken` ERROR, exit 4.
  B3. `severity.template_broken: warn` demotes it: exit 0, the rule a WARN.
  B4. The judge: the seed at 6 x, given the LOWER cost, ranks after the seed
      at 4 x (`judge_order` over each board's floorplan as place_simple
      grades it, `_floorplan`).
  B5. Control: L1 on its template grades no template finding.

    python3 tests/test_1821_template_broken.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan as fp                         # noqa: E402
import place_simple                                           # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

FAILS = []
TOL = 0.3


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _part(ref, x, y, locked=False, hw=1.5, hh=1.0):
    lk = '(locked yes) ' if locked else ''
    return (f'  (footprint "t:P" {lk}(layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            f'    (fp_rect (start {-hw} {-hh}) (end {hw} {hh}) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -0.8 0) (size 0.8 0.8) (layers "F.Cu")'
            ' (net 1 "N1"))\n'
            '    (pad "2" smd rect (at 0.8 0) (size 0.8 0.8) (layers "F.Cu")'
            ' (net 2 "N2")))\n')


def _board(path, l1_x):
    parts = _part('U1', 20, 15, locked=True) + _part('L1', l1_x, 15) + _part('C1', 32, 24)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n  (net 1 "N1")\n  (net 2 "N2")\n'
                 '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1)'
                 ' (type default)) (layer "Edge.Cuts"))\n' + parts + ')\n')
    return evidence(path, 'the fixture board')


def _intent(td, board, name, severity=None):
    doc = fp.emit_intent(parse_kicad_pcb(board), board)
    doc['templates'] = [{'anchor': 'U1', 'tolerance_mm': TOL,
                         'members': [{'ref': 'L1', 'at': [4.0, 0.0], 'rot': 0.0}]}]
    doc['min_reader'] = max(int(doc.get('min_reader') or 0), fp.TEMPLATES_READER)
    if severity:
        doc['severity'] = dict(doc.get('severity') or {}, **severity)
    path = os.path.join(td, name)
    json.dump(doc, open(path, 'w'))
    return evidence(path, 'the intent')


def _grade(td, board, intent, tag, **kw):
    out = os.path.join(td, f'{tag}.json')
    r = check([sys.executable, '-X', 'utf8', tool('check_floorplan.py'), board,
               '--intent', intent, '--json', out], **kw)
    rows = json.load(open(evidence(out, 'the check_floorplan report')))['violations']
    return r, {(v['rule'], v['severity']) for v in rows if v['rule'].startswith('template')}


def main():
    with tempfile.TemporaryDirectory(prefix='t1821b_') as td:
        on = _board(os.path.join(td, 'on.kicad_pcb'), 24.0)
        x4 = _board(os.path.join(td, 'x4.kicad_pcb'), 24.0 + 4 * TOL)
        x6 = _board(os.path.join(td, 'x6.kicad_pcb'), 24.0 + 6 * TOL)
        intent = _intent(td, on, 'intent.json')

        _r, got = _grade(td, x4, intent, 'x4', accept=True)
        ok('B1 4 x the tolerance: the WARN `template`, no `template_broken`',
           got == {('template', 'warn')}, got)
        r, got = _grade(td, x6, intent, 'x6', refuse='template_broken', code=4)
        ok('B2 6 x the tolerance: the ERROR `template_broken`',
           got == {('template_broken', 'error')} and '1.80mm' in r.stdout + r.stderr, got)
        demoted = _intent(td, on, 'demoted.json', {'template_broken': 'warn'})
        _r, got = _grade(td, x6, demoted, 'x6_demoted', accept=True)
        ok('B3 severity.template_broken warn demotes it',
           got == {('template_broken', 'warn')}, got)
        _r, got = _grade(td, on, intent, 'on', accept=True)
        ok('B5 control: on its template, no template finding', got == set(), got)

        fp4, fp6 = place_simple._floorplan(x4, intent), place_simple._floorplan(x6, intent)
        reports = {4: {'final': {'cost': 900.0}, 'illegal': [], 'floorplan': fp4},
                   6: {'final': {'cost': 100.0}, 'illegal': [], 'floorplan': fp6}}
        order = place_simple.judge_order(reports)
        ok('B4 the judge ranks the broken seed (lower cost) after the 4 x one',
           fp4.get('errors') == 0 and fp6.get('errors') == 1
           and 'template_broken' in fp6.get('rules', []) and order == [4, 6],
           (fp4, fp6, order))
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
