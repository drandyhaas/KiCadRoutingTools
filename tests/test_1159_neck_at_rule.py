#!/usr/bin/env python3
"""A terminal whose edge gap EQUALS the clearance is not a graze (#1159).

`_neck_terminal_grazes` used to neck when `d - own_l - 1e-4 < width/2`: the
1e-4 margin sat in the TRIGGER, so a terminal exactly at the rule counted as
grazing. complex_hierarchy (Default 0.3 / 0.4 mm, 0.1 mm grid) shipped nine
terminal segments at 0.3998 under `--escalation fab`, and under `--escalation
off` the same terminals took the #842 refusal and printed "would OVERLAP a
foreign track/via (edge dist 0.500mm)" -- 0.5 mm is not an overlap.

Rows, each a 0.4 mm terminal beside another net's 0.4 mm track at 0.3 mm
clearance, against both policies:

  at the rule (pitch 0.7)     fab: untouched      off: no refusal
  10 um inside (pitch 0.69)   fab: necked         off: refused as a GRAZE
  overlapping copper          fab: hard (short)   off: hard (short)

    python3 tests/test_1159_neck_at_rule.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import fab_tiers                                              # noqa: E402
from kicad_parser import Net, PCBData, BoardInfo, Segment     # noqa: E402
from routing_config import GridRouteConfig                    # noqa: E402
from single_ended_routing import (_neck_terminal_grazes,      # noqa: E402
                                  _hard_terminal_why)

FAILS = []
W, CLR = 0.4, 0.3


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def run(pitch, policy):
    """Neck a terminal at y=0 beside a foreign track at y=pitch."""
    foreign = Segment(start_x=-1.0, start_y=pitch, end_x=3.0, end_y=pitch,
                      width=W, layer='F.Cu', net_id=2)
    pcb = PCBData(footprints={}, nets={1: Net(1, '/A'), 2: Net(2, '/B')},
                  segments=[foreign], vias=[],
                  board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu']),
                  pads_by_net={})
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=W, clearance=CLR)
    seg = Segment(start_x=0.0, start_y=0.0, end_x=2.0, end_y=0.0, width=W,
                  layer='F.Cu', net_id=1)
    prev = fab_tiers.get_escalation_policy()
    fab_tiers.set_escalation_policy(policy)
    try:
        necked, hard = _neck_terminal_grazes([seg], [(0.0, 0.0)], pcb, 1, cfg,
                                             floor=0.1)
    finally:
        fab_tiers.set_escalation_policy(*prev)
    return necked, hard, seg.width


# At the rule: edge gap 0.7 - 0.2 - 0.2 = 0.3 exactly.
n, h, w = run(0.7, 'fab')
check('fab: a terminal exactly at the rule keeps its width',
      n == 0 and not h and w == W, f'necked={n} hard={len(h)} width={w}')
n, h, w = run(0.7, 'off')
check('off: a terminal exactly at the rule is not refused',
      n == 0 and not h and w == W, f'necked={n} hard={len(h)} width={w}')

# 10 um inside the rule: a real graze, so the control rows bite.
n, h, w = run(0.69, 'fab')
check('fab: a terminal 10 um inside the rule is necked',
      n == 1 and not h and w < W - 1e-9, f'necked={n} hard={len(h)} width={w}')
check('fab: ...to just inside the rule (the 1e-4 margin is on the TARGET)',
      abs((0.69 - W / 2.0 - w / 2.0) - (CLR + 1e-4)) < 1e-4,
      f'edge gap {0.69 - W / 2.0 - w / 2.0:.5f}')
n, h, w = run(0.69, 'off')
check('off: the same graze is refused, not necked',
      n == 0 and len(h) == 1 and w == W, f'necked={n} hard={len(h)} width={w}')
if h:
    why, ships = _hard_terminal_why(h[0])
    check('off: the refusal names a clearance graze, not a short',
          'OVERLAP' not in why and 'clearance' in why and ships == 'a clearance violation',
          why)
    check('off: ...and says by how much', '10.0um' in why, why)

# Overlapping copper is still a short under either policy.
for policy in ('fab', 'off'):
    n, h, w = run(0.2, policy)
    check(f'{policy}: overlapping copper is still a hard short',
          len(h) == 1 and h[0][2] is None and 'OVERLAP' in _hard_terminal_why(h[0])[0],
          f'hard={[(e[1], e[2]) for e in h]}')

print()
if FAILS:
    print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
    sys.exit(1)
print('ALL PASS')
