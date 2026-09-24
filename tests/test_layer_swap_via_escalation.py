#!/usr/bin/env python3
"""
The layer-swap via shrink must walk the escalation policy, not the physical
fab floor.

`_swap_vias_fit_or_shrink` (layer_swap_optimization) shrinks a swap's two new
pad vias when they collide at a tight pad pitch (#277). Every other descent
site walks `escalation_rungs` (fab_tiers), which is empty under
`--escalation off`, bounded by a hard tier and raised to the board's declared
minimums under `board`; this one read `fab_floor_for_param` (the PHYSICAL
floor) directly, so it shrank below the requested via under `off`, below a
board's declared minimum under `board`, and below the hard `standard` tier's
floor -- and recorded none of it in the narrowing ledger, so even the
permitted default shrink was missing from `design_rules`.

Fixture: two 0.45/0.20 vias at a 0.5 mm pitch (P/N via-via collision) on a
4-layer config. They fit at 0.40.

  default (fab / auto)   shrinks to fit, and the ledger records it
  off                    False, vias unchanged (the caller reverts the swap)
  board, min via 0.45    False, vias unchanged (0.40 is below the board floor)
  fab, hard standard     False, vias unchanged (0.40 is below the tier floor)

Run:
    python3 tests/test_layer_swap_via_escalation.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))

from kicad_parser import PCBData, BoardInfo, Via
from routing_config import GridRouteConfig
import fab_tiers
import layer_swap_optimization as lso

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
REQUESTED = [(0.45, 0.20), (0.45, 0.20)]


def _cfg():
    c = GridRouteConfig()
    c.layers = LAYERS
    c.via_size = 0.45
    c.via_drill = 0.20
    c.clearance = 0.1
    c.hole_to_hole_clearance = 0.2
    c.track_width = 0.1
    return c


def _case():
    pcb = PCBData(board_info=BoardInfo(layers={}, copper_layers=LAYERS),
                  nets={}, footprints={}, vias=[], segments=[], pads_by_net={})
    vias = [Via(x=0.0, y=0.0, size=0.45, drill=0.20, layers=LAYERS, net_id=1),
            Via(x=0.5, y=0.0, size=0.45, drill=0.20, layers=LAYERS, net_id=2)]
    return pcb, vias


def _run(policy, floors=None, tier=None):
    fab_tiers.set_default_fab_tier(tier or fab_tiers.DEFAULT_TIER)
    fab_tiers.set_escalation_policy(policy, floors)
    pcb, vias = _case()
    ok = lso._swap_vias_fit_or_shrink(pcb, vias, _cfg())
    return ok, [(v.size, v.drill) for v in vias], fab_tiers.escalation_summary()


def main():
    prev_policy = fab_tiers.get_escalation_policy()
    prev_tier = fab_tiers.get_default_fab_tier()
    fails = []
    try:
        pcb, vias = _case()
        if lso._bare_pad_pair_vias_fit(pcb, vias, _cfg())[0]:
            print("  SKIP: fixture vias already fit; shrink path not exercised")
            return 0

        ok, sizes, ledger = _run('fab')
        print(f"  fab:            fit={ok} vias={sizes} ledger={ledger.get('count')}")
        if not ok or not all(s < 0.45 for s, _ in sizes):
            fails.append(f"fab policy should shrink to fit, got fit={ok} {sizes}")
        if not ledger.get('count'):
            fails.append("fab policy shrank the swap vias without recording it "
                         "in the narrowing ledger")

        for label, policy, floors, tier in (
                ('off', 'off', None, None),
                ('board min 0.45', 'board', {'via_diameter': 0.45, 'via_drill': 0.20}, None),
                ('hard standard', 'fab', None, 'standard')):
            ok, sizes, ledger = _run(policy, floors, tier)
            print(f"  {label + ':':15s} fit={ok} vias={sizes} ledger={ledger.get('count')}")
            if ok:
                fails.append(f"{label}: shrank the swap vias to {sizes}")
            if sizes != REQUESTED:
                fails.append(f"{label}: left vias mutated: {sizes}")
    finally:
        fab_tiers.set_default_fab_tier(*prev_tier)
        fab_tiers.set_escalation_policy(*prev_policy)

    for f in fails:
        print(f"  FAIL: {f}")
    print("PASS" if not fails else f"FAILED ({len(fails)})")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
