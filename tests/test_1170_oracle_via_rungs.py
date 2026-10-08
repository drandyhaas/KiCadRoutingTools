#!/usr/bin/env python3
"""The oracle reconnect's via descent walks the escalation policy (#1170).

Both oracle via ladders (the exact-fill tier and the main link path) used a
literal `(0.45, 0.2)` rung, outside `fab_tiers.escalation_rungs` -- the #857
chokepoint every other descent site walks. complex_hierarchy under
`--escalation off` (board minimums 0.889/0.508, class via 1.651/0.6) shipped
two GND vias at 0.45/0.2 with no `via_diameter` row in design_rules, and the
writeback then lowered the project to match.

Rows: the rungs `oracle_via_rungs` offers under off / board / fab, with the
fab rung the literal it replaces; neither ladder carries a literal any more;
the emission records a smaller via in the narrowing ledger.

    python3 tests/test_1170_oracle_via_rungs.py
"""
import ast
import os
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import fab_tiers                                  # noqa: E402
from kicad_oracle import oracle_via_rungs          # noqa: E402
from routing_config import GridRouteConfig         # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


pcb = NS(board_info=NS(copper_layers=['F.Cu', 'B.Cu']))
cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.4, clearance=0.3,
                      via_size=1.651, via_drill=0.6)
prev = fab_tiers.get_escalation_policy()
try:
    fab_tiers.set_escalation_policy('off')
    r = oracle_via_rungs(cfg, pcb, 1)
    check('off: the link keeps its own via and has no smaller rung',
          r == [(1.651, 0.6)], str(r))
    fab_tiers.set_escalation_policy('board', {'via_diameter': 0.889, 'via_drill': 0.508})
    r = oracle_via_rungs(cfg, pcb, 1)
    check("board: the descent stops at the board's own 0.889/0.508",
          r == [(1.651, 0.6), (0.889, 0.508)], str(r))
    fab_tiers.set_escalation_policy('fab')
    r = oracle_via_rungs(cfg, pcb, 1)
    check('fab: the tier floor, exactly the 0.45/0.2 the literal was',
          r == [(1.651, 0.6), (0.45, 0.2)], str(r))
    fine = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.2, clearance=0.2,
                           via_size=0.5, via_drill=0.15)
    r = oracle_via_rungs(fine, pcb, 1)
    check('fab: the rung is the tier floor as declared, drill included (the old 0.45/0.2)',
          r == [(0.5, 0.15), (0.45, 0.2)], str(r))
    small = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.2, clearance=0.2,
                            via_size=0.4, via_drill=0.2)
    r = oracle_via_rungs(small, pcb, 1)
    check('fab: a via already below the floor rung is not "descended" upward',
          r == [(0.4, 0.2)], str(r))
finally:
    fab_tiers.set_escalation_policy(*prev)

src = open(os.path.join(ROOT, 'py_router', 'kicad_oracle.py'), encoding='utf-8').read()
tree = ast.parse(src)
lits = [n.lineno for n in ast.walk(tree) if isinstance(n, ast.Tuple)
        and [getattr(e, 'value', None) for e in n.elts] == [0.45, 0.2]]
check('no literal (0.45, 0.2) rung is left in kicad_oracle', lits == [], str(lits))
ladders = [n.lineno for n in ast.walk(tree) if isinstance(n, ast.For)
           and isinstance(n.iter, ast.Call)
           and getattr(n.iter.func, 'id', '') == 'oracle_via_rungs']
check('both via ladders walk oracle_via_rungs', len(ladders) == 2, str(ladders))
notes = [n.lineno for n in ast.walk(tree) if isinstance(n, ast.Call)
         and getattr(n.func, 'id', '') == 'note_narrowing'
         and len(n.args) > 1 and getattr(n.args[1], 'value', None) == 'via_diameter']
check('the emission records a via descent (design_rules via_diameter)',
      len(notes) == 1, str(notes))

print()
if FAILS:
    print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
    sys.exit(1)
print('ALL PASS')
