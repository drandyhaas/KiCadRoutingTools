#!/usr/bin/env python3
"""
`--escalation off` must freeze the layer-swap via shrink.

`_swap_vias_fit_or_shrink` (layer_swap_optimization) shrinks a swap's two new
pad vias toward the fab floor when they collide at a tight pad pitch (#277).
Every other descent site consults the escalation policy (`may_narrow` /
`escalation_rungs`, fab_tiers) so `--escalation off` keeps the requested
geometry; this one read `fab_floor_for_param` directly and still shrank the
vias below the requested size under `off`.

Fixture: two 0.45/0.20 vias at a 0.5 mm pitch (P/N via-via collision). Under
the default policy they shrink until they fit (True, size < 0.45). Under
`off` the call must report "does not fit" (False) and leave both vias at the
requested 0.45/0.20 so the caller reverts the swap.

Run:
    python3 tests/test_layer_swap_escalation_off.py
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


def main():
    prev = fab_tiers.get_escalation_policy()
    fails = []
    try:
        cfg = _cfg()
        pcb, vias = _case()
        if lso._bare_pad_pair_vias_fit(pcb, vias, cfg)[0]:
            print("  SKIP: fixture vias already fit; shrink path not exercised")
            return 0

        fab_tiers.set_escalation_policy('fab')
        ok = lso._swap_vias_fit_or_shrink(pcb, vias, cfg)
        sizes = [(v.size, v.drill) for v in vias]
        print(f"  fab: fit={ok} vias={sizes}")
        if not ok or not all(s < 0.45 for s, _ in sizes):
            fails.append(f"fab policy should shrink to fit, got fit={ok} {sizes}")

        fab_tiers.set_escalation_policy('off')
        pcb, vias = _case()
        ok = lso._swap_vias_fit_or_shrink(pcb, vias, cfg)
        sizes = [(v.size, v.drill) for v in vias]
        print(f"  off: fit={ok} vias={sizes}")
        if ok:
            fails.append(f"--escalation off shrank the swap vias to {sizes}")
        if sizes != [(0.45, 0.20), (0.45, 0.20)]:
            fails.append(f"--escalation off left vias mutated: {sizes}")
    finally:
        fab_tiers.set_escalation_policy(*prev)

    for f in fails:
        print(f"  FAIL: {f}")
    print("PASS" if not fails else f"FAILED ({len(fails)})")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
