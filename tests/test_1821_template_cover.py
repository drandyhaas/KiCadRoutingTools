"""#1821: a declared cover (`covers[]`) too low for a datasheet-template
member at its template pose is named, as a keep-out of the same room is
(#1387).

The keep-out form of a low ceiling over a member's template pose was named
twice (`plan_template_keepout` and place_simple's WARNING); the same room
declared as a cover's body was named by nothing -- `template_keepout_findings`
read `intent.keepouts` only -- while the seat's height pair moved the member
off its template, which the run reported only as drift.

  C1. PLAN, the chip and the cover's connector locked, the member too tall
      for the body over its template pose: `plan_template_keepout` names the
      cover and the member (FORCED, at the template pose).
  C2. PLAN control: the member low enough to fit under the body: no finding.
  C3. PLAN, the chip free: the body admits the chip but not the member ->
      the "possible" form.
  C4. PLACE, as C1: place_simple WARNS naming the cover, and its report's
      `template_keepout` carries the cover's finding.
  C5. PLACE control, as C2: no WARNING, no finding, the member on its
      template (no `template_off`).

    python3 tests/test_1821_template_cover.py
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
    """The #1387 fixture (chip U1, member L1, one more part) plus a locked
    connector J1 whose cover reaches over the member's template pose."""
    parts = (_part('U1', 20, 15, locked=locked_chip) + _part('L1', 8, 6)
             + _part('C1', 32, 24) + _part('J1', 24, 6, locked=True))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n  (net 1 "N1")\n  (net 2 "N2")\n'
                 '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1)'
                 ' (type default)) (layer "Edge.Cuts"))\n' + parts + ')\n')
    return path


def _intent(td, board, l1_height, name):
    doc = fp.emit_intent(parse_kicad_pcb(board), board)
    # the template puts L1 at (24, 15) for U1 where the board has it
    doc['templates'] = [{'anchor': 'U1', 'tolerance_mm': 0.3,
                         'members': [{'ref': 'L1', 'at': [4.0, 0.0], 'rot': 0.0}]}]
    # a body on J1 reaching +y over x 20..28, y 6..20, 2.0 mm over the face:
    # 1.5 mm of room once the 0.5 mm margin is kept
    doc['covers'] = [{'connector': 'J1', 'side': 'F', 'axis_deg': 90, 'width_mm': 8,
                      'offset_mm': 0, 'length_mm': 14, 'underside_mm': 2.0,
                      'removable': True, 'source': 'test: a board over J1'}]
    doc['heights'] = [{'ref': 'U1', 'height_mm': 1.0, 'source': 'test'},
                      {'ref': 'L1', 'height_mm': l1_height, 'source': 'test'},
                      {'ref': 'J1', 'height_mm': 1.0, 'source': 'test'}]
    doc['min_reader'] = max(int(doc.get('min_reader') or 0), fp.TEMPLATES_READER,
                            fp.COVERS_READER)
    path = os.path.join(td, name)
    json.dump(doc, open(path, 'w'))
    return evidence(path, 'the intent')


def _plan(board, intent):
    r = check([sys.executable, '-X', 'utf8', tool('check_floorplan.py'), board,
               '--intent', intent, '--plan-only', '--allow-unplaced'], accept=True)
    return r.stdout


def _place(td, board, intent, tag, groups):
    res = os.path.join(td, f'res_{tag}.json')
    r = check([sys.executable, '-X', 'utf8', tool('place_simple.py'), board,
               os.path.join(td, f'out_{tag}.kicad_pcb'), '--groups', groups,
               '--intent', intent, '--seed', '0', '--moves', '800', '--json', res],
              accept=True)
    return r.stdout, json.load(open(evidence(res, 'the place_simple report')))


def main():
    with tempfile.TemporaryDirectory(prefix='t1821_') as td:
        locked = _board(os.path.join(td, 'locked.kicad_pcb'), True)
        free = _board(os.path.join(td, 'free.kicad_pcb'), False)
        tall = _intent(td, locked, 2.5, 'i_tall.json')
        low = _intent(td, locked, 1.0, 'i_low.json')

        out = _plan(locked, tall)
        ok('C1 plan: the cover on J1 over the tall member\'s template pose -> FORCED',
           'plan_template_keepout' in out
           and "the cover on J1 stands over L1, a member of U1's template" in out
           and 'puts L1 at (24, 15, 0)' in out, out[-900:])
        out = _plan(locked, low)
        ok('C2 plan control: a member that fits under the body is not flagged',
           'plan_template_keepout' not in out, out[-600:])
        out = _plan(free, _intent(td, free, 2.5, 'i_free.json'))
        ok('C3 plan, the chip free: the body admits U1 but not L1 -> possible',
           'the cover on J1 admits U1 under its body but not L1' in out, out[-900:])

        groups = os.path.join(td, 'groups.json')
        json.dump({'schema': 1, 'groups': [{'name': 'all',
                                            'refs': ['U1', 'L1', 'C1', 'J1']}]},
                  open(groups, 'w'))
        evidence(groups, 'the groups file')
        out, rep = _place(td, locked, tall, 'tall', groups)
        ok('C4 place: WARNED naming the cover, and in the report',
           'WARNING: the cover on J1 stands over L1' in out
           and [(f['ref'], f['cover']) for f in rep['template_keepout']] == [('L1', 'J1')],
           out[-900:])
        out, rep = _place(td, locked, low, 'low', groups)
        ok('C5 place control: a member that fits is not warned and stays on its template',
           'the cover on J1' not in out and rep['template_keepout'] == []
           and rep['template_off'] == [], (rep['template_keepout'], rep['template_off'],
                                           out[-600:]))
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
