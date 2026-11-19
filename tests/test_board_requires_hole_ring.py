#!/usr/bin/env python3
"""Under ``--escalation board`` the board's own hole clearance is a rule the
run may not go below, like its via size and drill minimums (#1676): every via
the router places holds the copper-to-hole ring, drill + 2*(hole_clearance -
clearance) (#1914).

Was: the #1914 ladder only PREFERRED the ring under every policy -- each rung
raised to the ring, then the rung itself -- so ``board`` shipped ring-short
vias wherever the raised rung did not fit, and KiCad DRC graded them as
hole_clearance errors against the board's own rules. The layer-swap shrink
loop did the same (its last resort keeps the nominal drill and shrinks the
copper below the ring).

A ring-short via is legal only where its raised twin is: other-net copper held
at the clearance off the raised via's copper is exactly the hole clearance off
the hole. So requiring the ring under ``board`` loses no site a DRC-clean via
could take; ``fab`` keeps completion first (unchanged, test_1914).

Unit checks, at clearance 0.1 / hole clearance 0.25 (ring 0.15):
  1. board, standard tier: the ladder is 0.5/0.2 only (fab: 0.5/0.2, 0.45/0.2);
  2. board, tier auto: every rung holds the ring (fab keeps the short twins);
  3. board, no pair installed: the tier's ladder, unchanged;
  4. board, swap loop: a site where (0.5, 0.2) fits ships it, as under fab;
  5. board, swap loop: a site no ring-holding via fits leaves the via
     unfitted (fab ships the ring-short (0.45, 0.3) last resort);
  6. board, swap loop: every pair it can ship holds the ring.

    python3 tests/test_board_requires_hole_ring.py
"""
import os
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import fab_tiers  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from layer_swap_optimization import _swap_vias_fit_or_shrink  # noqa: E402

fails = []
RING = 0.15


def ok(name, cond, detail=''):
    print(('PASS: ' if cond else 'FAIL: ') + name + (f'  {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def _pairs(rungs):
    return [(r['via_diameter'], r['via_drill']) for r in rungs]


def _holds(d, h):
    return (d - h) / 2.0 >= RING - 1e-9


def _board(tmp):
    """A foreign 0.6/0.3 via (net 2) at (20, 20) on a 40 x 40 two-layer board."""
    edge = "\n".join(
        f' (gr_line (start {a} {b}) (end {c} {d}) (layer "Edge.Cuts") (width 0.1))'
        for a, b, c, d in [(0, 0, 40, 0), (40, 0, 40, 40), (40, 40, 0, 40), (0, 40, 0, 0)])
    text = f'''(kicad_pcb
 (version 20221018)
 (net 0 "")
 (net 1 "A")
 (net 2 "B")
{edge}
 (via (at 20 20) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 2) (tstamp 00000000-0000-0000-0000-000000000001))
)'''
    pcb = os.path.join(tmp, 'b.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(text)
    return pcb


def _config():
    return SimpleNamespace(
        clearance=0.1, hole_to_hole_clearance=0.2,
        layers=['F.Cu', 'B.Cu'], via_size=0.6, via_drill=0.3,
        max_pair_clearance=lambda: 0.1,
        pair_clearance_inert=lambda: True,
        pair_clearance=lambda a, b, kind=None: 0.1,
        pad_pair_clearance_before_override=lambda pad, net: 0.1,
        rule_floors=lambda net_id: {})


def ladder():
    saved_tier, saved_esc = fab_tiers._DEFAULT_TIER, fab_tiers._ESCALATION
    try:
        fab_tiers.set_default_fab_tier('standard')
        fab_tiers.set_escalation_policy('fab', {})
        fab_tiers.set_via_hole_clearance(0.25, 0.1)
        fab_std = _pairs(fab_tiers.escalation_rungs(4))
        fab_tiers.set_escalation_policy('board', {})
        fab_tiers.set_via_hole_clearance(0.25, 0.1)
        std = _pairs(fab_tiers.escalation_rungs(4))
        ok('1. board, standard: only the ring-holding 0.5/0.2 (fab keeps 0.45/0.2)',
           std == [(0.5, 0.2)] and fab_std == [(0.5, 0.2), (0.45, 0.2)],
           f'board {std} fab {fab_std}')
        fab_tiers.set_default_fab_tier('auto')
        auto = _pairs(fab_tiers.escalation_rungs(4))
        ok('2. board, tier auto: every rung holds the ring',
           auto and all(_holds(d, h) for d, h in auto), auto)
        fab_tiers.set_default_fab_tier('standard')
        fab_tiers.set_escalation_policy('board', {})
        ok('3. board, no pair installed: the tier\'s ladder',
           _pairs(fab_tiers.escalation_rungs(4)) == [(0.45, 0.2)],
           _pairs(fab_tiers.escalation_rungs(4)))
    finally:
        fab_tiers.set_via_hole_clearance(None, None)
        fab_tiers.set_escalation_policy(saved_esc, {})
        fab_tiers.set_default_fab_tier(saved_tier)


def swap(tmp):
    saved_tier, saved_esc = fab_tiers._DEFAULT_TIER, fab_tiers._ESCALATION
    data = parse_kicad_pcb(_board(tmp))
    cfg = _config()
    try:
        fab_tiers.set_default_fab_tier('standard')
        fab_tiers.set_escalation_policy('board', {})
        fab_tiers.set_via_hole_clearance(0.25, 0.1)
        v = SimpleNamespace(x=20.66, y=20.0, size=0.6, drill=0.3, net_id=1)
        fitted = _swap_vias_fit_or_shrink(data, [v], cfg)
        ok('4. board, swap: (0.5, 0.2) fits and ships',
           fitted and (v.size, v.drill) == (0.5, 0.2), (fitted, v.size, v.drill))
        v = SimpleNamespace(x=20.63, y=20.0, size=0.6, drill=0.3, net_id=1)
        fitted = _swap_vias_fit_or_shrink(data, [v], cfg)
        ok('5. board, swap: no ring-holding via fits -> not fitted, nothing short ships',
           not fitted and _holds(v.size, v.drill), (fitted, v.size, v.drill))
        shipped = []
        for i in range(40):
            v = SimpleNamespace(x=20.55 + i * 0.01, y=20.0, size=0.6, drill=0.3, net_id=1)
            if _swap_vias_fit_or_shrink(data, [v], cfg):
                shipped.append((round(v.size, 3), round(v.drill, 3)))
        ok('6. board, swap: every pair shipped across 0.55..0.94 holds the ring',
           shipped and all(_holds(d, h) for d, h in shipped), sorted(set(shipped)))
    finally:
        fab_tiers.set_via_hole_clearance(None, None)
        fab_tiers.set_escalation_policy(saved_esc, {})
        fab_tiers.set_default_fab_tier(saved_tier)


def main():
    with tempfile.TemporaryDirectory() as tmp:
        ladder()
        swap(tmp)
    if fails:
        print(f'{len(fails)} check(s) FAILED')
        sys.exit(1)
    print('all checks passed')


if __name__ == '__main__':
    main()
