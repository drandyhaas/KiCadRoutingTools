"""#1468: the zone plan is checked before any seed, and says more than it did.

`floorplan.plan_check` is the pre-placement fit check. Once the seeder that
called it was retired, nothing in the flow ran it: place_simple placed a plan
`check_floorplan --plan-only` refuses and said so only after placing. And
two zone cases went unremarked:

  * a member larger than its zone turns the zone into a centre coordinate
    and was dropped from the zone's area sum without a word -- it is now
    named (`plan_zone_member_oversized`) and charged the area its held
    centre still makes it cover;
  * zones well short of full failed to seat every member (77% and 60%
    courtyard fill, every seed) with no finding at all -- an advisory
    `plan_zone_utilisation` now fires at 50%, a WARN that can never be an
    error, because area is necessary, not sufficient.

The fixture is the tracked board the issue measured, with its zones.

    python3 tests/test_1468_preseed_zone_fit.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan                               # noqa: E402
from run_utils import check as run_check, evidence, tool      # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
FAILS = []
ZONE_A = (['R%d' % i for i in list(range(1, 13)) + [14]]
          + ['D%d' % i for i in range(1, 7)] + ['C9', 'C10', 'C11', 'C12'])
GROUPS = {'schema': 1, 'groups': [
    {'name': 'zone_a', 'refs': ZONE_A},
    {'name': 'drivers', 'refs': ['U%d' % i for i in range(1, 11)]
     + ['C%d' % i for i in range(1, 9)]},
    {'name': 'conn', 'refs': ['J3'] + ['J%d' % i for i in range(5, 21)]},
    {'name': 'holes', 'refs': ['H%d' % i for i in range(1, 8)]}]}


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _intent(blocks, budget=None, severity=None):
    d = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
         'blocks': blocks}
    if budget:
        d['legality_budget'] = budget
    if severity:
        d['severity'] = severity
    return d


def _plan(doc):
    it = floorplan.intent_from_dict(doc, '')
    with contextlib.redirect_stdout(io.StringIO()):
        return floorplan.plan_check(it, parse_kicad_pcb(BOARD), BOARD)


def za(zone):
    return {'name': 'zone_a', 'refs': ZONE_A, 'zone': zone, 'tolerance_mm': 0}


ZB = {'name': 'zone_b', 'refs': ['U1', 'C1', 'C2', 'C4', 'C5', 'C6', 'C7'],
      'zone': [60, 38, 65, 43], 'tolerance_mm': 0}


def test_an_oversized_member_is_named_and_charged():
    found, measured = _plan(_intent([ZB]))
    over = [v for v in found if v.rule == 'plan_zone_member_oversized']
    check('the member larger than its zone is named',
          [v.ref for v in over] == ['U1'], [v.message for v in over])
    check('...as a WARN', over and over[0].severity == floorplan.WARN)
    row = measured['zones'][0]
    # 7.4 x 10.4 mm in a 5 x 5 mm zone: its centre held inside, it covers at
    # least min(5, 3.7) * min(5, 5.2) of it
    check('the zone is charged the area its held centre covers',
          row['anchor_members'] == ['U1']
          and abs(over[0].measured['charged_mm2'] - 18.5) < 1e-6
          and row['members_area_mm2'] > 25.93 + 18.49, row)


def test_the_utilisation_heuristic():
    for zone, pct in (([100, 36, 111.4, 47.3], 77),
                      ([100, 36, 112.87, 48.87], 60)):
        found, _m = _plan(_intent([za(zone)]))
        u = [v for v in found if v.rule == 'plan_zone_utilisation']
        check(f'a zone at {pct}% courtyard fill gets the advisory',
              len(u) == 1 and u[0].severity == floorplan.WARN
              and 'heuristic' in u[0].message, [v.rule for v in found])
    found, _m = _plan(_intent([za([100, 36, 115, 50])]))
    check('a zone at 47% does not',
          not [v for v in found if v.rule == 'plan_zone_utilisation'],
          [v.message for v in found])
    try:
        floorplan.intent_from_dict(_intent(
            [za([100, 36, 111.4, 47.3])],
            severity={'plan_zone_utilisation': 'error'}), '')
        refused = False
    except floorplan.IntentError as ex:
        refused = 'heuristic' in str(ex)
    check('the heuristic cannot be raised to an error', refused)


def test_place_simple_refuses_a_plan_error_before_any_seed(td):
    board = os.path.join(td, 'b.kicad_pcb')
    shutil.copy(BOARD, board)
    for ext in ('.kicad_pro', '.kicad_prl'):
        src = os.path.splitext(BOARD)[0] + ext
        if os.path.isfile(src):
            shutil.copy(src, os.path.splitext(board)[0] + ext)
    intent = os.path.join(td, 'budget.json')
    with open(intent, 'w', encoding='utf-8') as fh:
        json.dump(_intent([za([100, 36, 111.4, 47.3]), ZB],
                          budget={'overlap_area': 0.0, 'oob_count': 0}), fh)
    groups = os.path.join(td, 'groups.json')
    with open(groups, 'w', encoding='utf-8') as fh:
        json.dump(GROUPS, fh)
    evidence(intent, 'the intent')
    evidence(groups, 'the groups file')
    seeds = os.path.join(td, 'seeds')
    r = run_check([sys.executable, '-X', 'utf8', tool('place_simple.py'), board,
                   os.path.join(seeds, 's{seed}.kicad_pcb'), '--groups', groups,
                   '--intent', intent, '--seeds', '0-3', '--jobs', '1'],
                  refuse='refused before any seed is placed', code=2)
    out = r.stdout + r.stderr
    check('the refusal names the plan ERROR', 'plan_zone_overfull' in out, out)
    check('...prints the WARNs once',
          out.count('PLAN [warn] plan_zone_utilisation') == 1, out)
    check('...and no seed was started', not os.path.exists(seeds))


def main():
    test_an_oversized_member_is_named_and_charged()
    test_the_utilisation_heuristic()
    with tempfile.TemporaryDirectory(prefix='t1468_') as td:
        test_place_simple_refuses_a_plan_error_before_any_seed(td)
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
