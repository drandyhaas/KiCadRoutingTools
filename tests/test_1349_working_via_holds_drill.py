#!/usr/bin/env python3
"""#1349: list_nets --design-rules never suggests a via its own drill does not fit.

`effective_floors` floored the working via's diameter (min_via_diameter vs the
fab via) and its drill (min_through_hole_diameter vs the fab drill)
separately. One-Air-Max declares min_via_diameter 0.3 and
min_through_hole_diameter 0.508, so it printed "via 0.45/0.508" and
SUGGESTED `--via-size 0.45 --via-drill 0.508`: a drill wider than the via.
The same split suggested 0.45/0.3 on boards whose min_via_annular_width is
0.1, a via KiCad flags on its own rule.

The diameter now holds the drill plus a ring each side: the board's
min_via_annular_width, and never less than the ring of the smallest via the
active fab tier can make.

Checks:
  1. One-Air-Max's constraints: 0.608/0.508 on the auto tier, 0.758/0.508 on
     a hard standard tier, and the result says the drill set it.
  2. A declared 0.1 annular ring over 0.45/0.3 gives 0.5/0.3.
  3. Unchanged where nothing conflicted: KiCad's factory rules (0.5/0.3, ring
     0.1) and a board declaring nothing keep their old working via.
  4. Over a grid of declarations, diameter >= drill + 2 x ring, always.
  5. `list_nets --design-rules` prints the note and suggests
     `--via-size 0.608 --via-drill 0.508`.

    python3 tests/test_1349_working_via_holds_drill.py
"""
import itertools
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from copy_board import copy_board                              # noqa: E402
from run_utils import evidence                                 # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'flat_hierarchy.kicad_pcb')
ONE_AIR_MAX = {'min_via_diameter': 0.3, 'min_through_hole_diameter': 0.508,
               'min_via_annular_width': 0.0}
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def via(eff):
    return (eff['working_via_diameter'], eff['working_via_drill'])


def main():
    from fab_tiers import get_default_fab_tier, set_default_fab_tier
    from list_nets import effective_floors
    prev = get_default_fab_tier()
    try:
        set_default_fab_tier('auto')
        print('1. One-Air-Max (min_via_diameter 0.3, min_through_hole_diameter 0.508):')
        eff = effective_floors(ONE_AIR_MAX, 4)
        check('auto tier: 0.608/0.508', via(eff) == (0.608, 0.508), str(via(eff)))
        check('the drill set the diameter', eff.get('working_via_from_drill') is True)
        set_default_fab_tier('standard')
        eff = effective_floors(ONE_AIR_MAX, 4)
        check('hard standard tier: 0.758/0.508 (its smallest via has a 0.125 ring)',
              via(eff) == (0.758, 0.508), str(via(eff)))
        set_default_fab_tier('auto')

        print('2. a declared annular ring is held:')
        eff = effective_floors({'min_via_diameter': 0.45,
                                'min_through_hole_diameter': 0.3,
                                'min_via_annular_width': 0.1}, 2)
        check('0.45/0.3 with a 0.1 ring rule -> 0.5/0.3', via(eff) == (0.5, 0.3),
              str(via(eff)))

        print('3. unchanged where nothing conflicted:')
        eff = effective_floors({'min_via_diameter': 0.5,
                                'min_through_hole_diameter': 0.3,
                                'min_via_annular_width': 0.1}, 2)
        check("KiCad's factory rules keep 0.5/0.3", via(eff) == (0.5, 0.3)
              and not eff.get('working_via_from_drill'), str(via(eff)))
        for n in (2, 4):
            eff = effective_floors({}, n)
            check(f'{n}-layer board declaring nothing keeps the fab via 0.45/0.2',
                  via(eff) == (0.45, 0.2), str(via(eff)))

        print('4. diameter >= drill + 2 x ring over a grid:')
        bad = []
        for dia, drill, ann, n, tier in itertools.product(
                (0, 0.25, 0.3, 0.45, 0.6), (0, 0.15, 0.3, 0.508, 0.8),
                (0, 0.05, 0.1, 0.2), (2, 4), ('auto', 'standard', 'advanced')):
            set_default_fab_tier(tier)
            eff = effective_floors({'min_via_diameter': dia,
                                    'min_through_hole_diameter': drill,
                                    'min_via_annular_width': ann}, n)
            d, h = via(eff)
            if d + 1e-9 < h + 2 * max(ann, eff.get('working_via_ring', 0.0)) - 1e-4 \
                    or eff.get('working_via_ring', 0.0) < ann - 1e-9 or d <= h:
                bad.append((dia, drill, ann, n, tier, d, h))
        set_default_fab_tier('auto')
        check('every combination holds its drill', not bad, str(bad[:5]))
    finally:
        set_default_fab_tier(*prev)

    print('5. list_nets --design-rules:')
    evidence(BOARD, 'the flat_hierarchy fixture')
    with tempfile.TemporaryDirectory() as tmp:
        pcb = os.path.join(tmp, 'b.kicad_pcb')
        copy_board(BOARD, pcb)
        pro = os.path.join(tmp, 'b.kicad_pro')
        with open(pro) as f:
            doc = json.load(f)
        doc['board']['design_settings']['rules'].update(ONE_AIR_MAX)
        with open(pro, 'w') as f:
            json.dump(doc, f, indent=2)
        r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/list_nets.py',
                            pcb, '--design-rules'], cwd=ROOT, capture_output=True,
                           text=True, encoding='utf-8', errors='replace')
        check('exit 0', r.returncode == 0, r.stderr[-300:])
        check('suggests --via-size 0.608 --via-drill 0.508',
              '--via-size 0.608 --via-drill 0.508' in r.stdout, r.stdout[-600:])
        check('says the drill set the diameter',
              'working via 0.608/0.508: the drill 0.508' in r.stdout)
        check('never suggests 0.45/0.508', '0.45/0.508' not in r.stdout
              and '--via-size 0.45 --via-drill 0.508' not in r.stdout)

    if failures:
        print(f"\nFAILED ({len(failures)}): {failures}")
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
