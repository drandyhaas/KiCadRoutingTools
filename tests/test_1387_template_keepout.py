"""#1387: a keep-out that binds a datasheet-template member but not the
template's chip forbids the template pose wherever the chip stands under it,
and nothing said so (the member drifted, a WARN the judge does not rank on).

  P. PLAN   `check_floorplan --plan-only` raises `plan_template_keepout` (WARN)
            -- FORCED where the chip's pose is known (locked in the board), so
            the member's template pose is tested against the region; otherwise
            "possible", where the region admits the chip but binds the member.
  S. PLACE  `place_simple` WARNS in the same words for a member whose template
            pose for the placed chip lies inside the keep-out, WARNS for a
            member left past its template's tolerance, and carries both in its
            report (`template_keepout`, `template_off`).
  N. A keep-out that allows the member too is not flagged.
  H. A height-limited keep-out (#1443, a shield can's lid) binds a member by
     its height: one under the limit is not flagged, by the plan or by
     place_simple, and one over it still is -- and the chip by its own
     height, so a chip that fits under the lid is admitted there.

    python3 tests/test_1387_template_keepout.py
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
from run_utils import check, evidence, tool                   # noqa: E402

FAILS = []


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


def _board(path, locked_chip):
    parts = (_part('U1', 20, 15, locked=locked_chip) + _part('L1', 8, 6)
             + _part('C1', 32, 24))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n  (net 1 "N1")\n  (net 2 "N2")\n'
                 '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1)'
                 ' (type default)) (layer "Edge.Cuts"))\n' + parts + ')\n')
    return path


def _intent(td, board, allow, name, lid_l1=None):
    doc = fp.emit_intent(parse_kicad_pcb(board), board)
    doc['templates'] = [{'anchor': 'U1', 'tolerance_mm': 0.3,
                         'members': [{'ref': 'L1', 'at': [4.0, 0.0], 'rot': 0.0}]}]
    doc['keepouts'] = [{'name': 'low-parts-only', 'rect': [21.5, 11.0, 28.0, 19.0],
                        'sides': ['F'], 'allow': allow, 'why': 'a region for chips only'}]
    doc['min_reader'] = max(int(doc.get('min_reader') or 0), fp.TEMPLATES_READER)
    if lid_l1 is not None:
        # a 2.0 mm lid over the region, binding no one by name: L1 is lid_l1 tall
        doc['keepouts'][0].update({'max_height_mm': 2.0, 'allow': []})
        doc['heights'] = [{'ref': 'U1', 'height_mm': 1.0, 'source': 'test'},
                          {'ref': 'L1', 'height_mm': lid_l1, 'source': 'test'}]
        doc['min_reader'] = max(doc['min_reader'], fp.COVERS_READER)
    path = os.path.join(td, name)
    json.dump(doc, open(path, 'w'))
    return path


def _plan(board, intent):
    r = check([sys.executable, '-X', 'utf8', tool('check_floorplan.py'), board,
               '--intent', intent, '--plan-only', '--allow-unplaced'], accept=True)
    return r.stdout


def main():
    with tempfile.TemporaryDirectory(prefix='t1387_') as td:
        locked = _board(os.path.join(td, 'locked.kicad_pcb'), True)
        free = _board(os.path.join(td, 'free.kicad_pcb'), False)

        out = _plan(locked, _intent(td, locked, ['U*'], 'i_chip.json'))
        ok('P1 a locked chip: the member\'s template pose lies in the region -> FORCED',
           'plan_template_keepout' in out and "binds L1, a member of U1's template" in out
           and 'puts L1 at (24' in out, out[-700:])
        out = _plan(free, _intent(td, free, ['U*'], 'i_chip2.json'))
        ok('P2 a chip with no known pose: the region admits it but binds the member -> possible',
           "admits U1 but binds L1" in out, out[-700:])
        out = _plan(locked, _intent(td, locked, ['U*', 'L*'], 'i_both.json'))
        ok('N a keep-out that allows the member too is not flagged',
           'plan_template_keepout' not in out, out[-500:])
        out = _plan(locked, _intent(td, locked, [], 'i_lid_fits.json', lid_l1=1.0))
        ok('H1 a lid L1 fits under (1.0 mm, limit 2.0) is not flagged',
           'plan_template_keepout' not in out and 'plan_errors": 0' in out, out[-500:])
        out = _plan(locked, _intent(td, locked, [], 'i_lid_tall.json', lid_l1=2.5))
        ok('H2 a lid L1 does not fit under (2.5 mm) still is -> FORCED',
           "keep-out 'low-parts-only' binds L1, a member of U1's template" in out,
           out[-700:])
        out = _plan(free, _intent(td, free, [], 'i_lid_free.json', lid_l1=2.5))
        ok('H4 a chip with no known pose that fits under the lid (1.0 mm), its member '
           'too tall -> possible', "admits U1 but binds L1" in out, out[-700:])

        groups = os.path.join(td, 'groups.json')
        json.dump({'schema': 1, 'groups': [{'name': 'all', 'refs': ['U1', 'L1', 'C1']}]},
                  open(groups, 'w'))
        intent = os.path.join(td, 'i_chip.json')
        res = os.path.join(td, 'res.json')
        r = check([sys.executable, '-X', 'utf8', tool('place_simple.py'), locked,
                   os.path.join(td, 'out.kicad_pcb'), '--groups', groups, '--intent', intent,
                   '--seed', '0', '--moves', '800', '--json', res], accept=True)
        evidence(res, 'the place_simple report')
        rep = json.load(open(res))
        ok('S1 the placed member\'s template pose is forbidden: WARNED, and in the report',
           "WARNING: keep-out 'low-parts-only' binds L1" in r.stdout
           and [f['ref'] for f in rep['template_keepout']] == ['L1'], r.stdout[-900:])
        ok('S2 the member left past the tolerance is WARNED and in the report',
           'WARNING: off its datasheet template past the tolerance: L1 (U1)' in r.stdout
           and [d['ref'] for d in rep['template_off']] == ['L1']
           and rep['template_off'][0]['tolerance_mm'] == 0.3, rep['template_off'])
        lid = os.path.join(td, 'i_lid_fits.json')
        res2 = os.path.join(td, 'res2.json')
        r = check([sys.executable, '-X', 'utf8', tool('place_simple.py'), locked,
                   os.path.join(td, 'out2.kicad_pcb'), '--groups', groups, '--intent', lid,
                   '--seed', '0', '--moves', '800', '--json', res2], accept=True)
        evidence(res2, 'the place_simple report')
        rep = json.load(open(res2))
        ok('H3 place_simple: a member that fits under the lid is not warned',
           'binds L1' not in r.stdout and rep['template_keepout'] == [],
           r.stdout[-900:])
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
