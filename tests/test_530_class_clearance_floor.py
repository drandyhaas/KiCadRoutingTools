#!/usr/bin/env python3
"""#530 (corpus A/B, core1106_cam): a net in a non-Default class is graded by
KiCad at that class's clearance whatever a rescue narrowed to, and the
writeback lowers only the Default class (decision 2). So the per-net floors
every descent site walks (GridRouteConfig.rule_floors -> escalation_rungs)
carry the net's own class clearance, under every policy; and the plane-tap
clearance ladder floors at it too.

Pure config-level test, no routing:
  - a net in the class map: rule_floors carries 'clearance' = its class value,
    and escalation_rungs(..., extra_floors=...) never yields a rung below it
    (board policy AND fab policy -- it is a grading floor, not a fab one)
  - a Default-only net (absent from the map): no clearance floor
  - fine_tap_configs for a pad of the classed net never yields a clearance
    below the class value
"""
import os
import sys
from dataclasses import dataclass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from routing_config import GridRouteConfig  # noqa: E402
import fab_tiers  # noqa: E402
from fab_tiers import escalation_rungs, set_escalation_policy, set_default_fab_tier  # noqa: E402


@dataclass
class _Pad:
    net_id: int
    size_x: float = 0.4
    size_y: float = 0.4


@dataclass
class _BI:
    copper_layers: tuple = ('F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu')


@dataclass
class _PCB:
    board_info: _BI


def main():
    fails = []
    cfg = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.5, via_drill=0.3,
                          layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'])
    cfg.set_net_clearances({7: 0.15, 9: 0.3}, [7, 9, 21])
    set_default_fab_tier('auto')            # the deepest ladder there is
    for policy in ('board', 'fab'):
        set_escalation_policy(policy, {})
        fl = cfg.rule_floors(7)
        if abs(fl.get('clearance', 0.0) - 0.15) > 1e-9:
            fails.append(f"[{policy}] rule_floors(7) lacks the 0.15 class clearance: {fl}")
        rungs = escalation_rungs(4, extra_floors=cfg.rule_floors(7))
        low = min(r['clearance'] for r in rungs)
        if low < 0.15 - 1e-9:
            fails.append(f"[{policy}] a rung for net 7 sits at clearance {low} < its class 0.15")
        rungs9 = escalation_rungs(4, extra_floors=cfg.rule_floors(9))
        if min(r['clearance'] for r in rungs9) < 0.3 - 1e-9:
            fails.append(f"[{policy}] a rung for net 9 sits below its class 0.3")
        if 'clearance' in cfg.rule_floors(21):
            fails.append(f"[{policy}] a Default-only net got a class clearance floor: {cfg.rule_floors(21)}")
        r21 = escalation_rungs(4, extra_floors=cfg.rule_floors(21))
        if min(r['clearance'] for r in r21) > 0.1 + 1e-9:
            fails.append(f"[{policy}] the Default-only net's ladder no longer reaches the fab floor: {r21}")
    set_escalation_policy('board', {})
    from plane_pad_tap import fine_tap_configs
    pcb = _PCB(_BI())
    clrs = [c.clearance for c in fine_tap_configs(cfg, _Pad(7), pcb)]
    if not clrs or min(clrs) < 0.15 - 1e-9:
        fails.append(f"fine_tap_configs for a class-0.15 net stepped below it: {clrs}")
    clrs21 = [c.clearance for c in fine_tap_configs(cfg, _Pad(21), pcb)]
    if not clrs21 or min(clrs21) > 0.1 + 1e-9:
        fails.append(f"fine_tap_configs for a Default-only net no longer reaches the fab floor: {clrs21}")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print("PASS: a net's own class clearance floors every automatic clearance descent "
          "(rungs and the tap ladder) under board and fab policy; Default-only nets are untouched")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
