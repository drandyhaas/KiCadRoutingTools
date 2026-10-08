#!/usr/bin/env python3
"""Oracle links ship at the net's own width where it fits, narrow only at the
pinch (#1169).

The oracle's width ladder re-routed the WHOLE link at each width and stopped at
the first that did not fit, and the exact-fill tier had no ladder at all, so
one pinch anywhere shipped the whole strap at the class width. complex_hierarchy
with --power-nets-widths 0.6: GND 74.0 of 92.9 mm under 0.6, all of it from two
oracle straps, where 58.6 mm of it fits at 0.6 with check_drc still clean.

Rows: `widen_link_legs` on a straight link past one foreign pinch ships wide /
narrow / wide, with every wide piece clear by the same exact check
(`wide_route_clear`) the oracle trusts; a link with no pinch ships whole at the
net width; a net that asked for nothing wider is unchanged; and the emission
writes the legs, from either tier.

    python3 tests/test_1169_oracle_link_widths.py
"""
import ast
import os
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import Net, PCBData, BoardInfo, Segment   # noqa: E402
from routing_config import GridRouteConfig                   # noqa: E402
from kicad_oracle import widen_link_legs                     # noqa: E402
from plane_region_connector import wide_route_clear          # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def board(foreign):
    return PCBData(footprints={}, nets={1: Net(1, 'GND'), 2: Net(2, '/SIG')},
                   segments=list(foreign), vias=[],
                   board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=(-20, -20, 40, 20)),
                   pads_by_net={})


cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.4, clearance=0.3,
                      power_net_widths={1: 0.6})
LINK = [(0.0, 0.0, 'F.Cu'), (10.0, 0.0, 'F.Cu')]
# A short foreign stub at y = 0.65 over x in [4.8, 5.2]: its edge is 0.55 from
# the link centreline. 0.4 needs 0.2 + 0.3 = 0.5 (clears), 0.6 needs 0.6
# (does not) -- a pinch only there.
pinch = Segment(start_x=4.8, start_y=0.65, end_x=5.2, end_y=0.65, width=0.2,
                layer='F.Cu', net_id=2)
pcb = board([pinch])
check('fixture: 0.4 clears the whole link',
      wide_route_clear(LINK, 0.4, pcb, 1, cfg, board_edge_clearance=0.0))
check('fixture: 0.6 does not clear the whole link',
      not wide_route_clear(LINK, 0.6, pcb, 1, cfg, board_edge_clearance=0.0))

legs = widen_link_legs(LINK, 0.4, pcb, 1, cfg)
widths = [round(l[5], 4) for l in legs]
check('wide / narrow / wide', widths == [0.6, 0.4, 0.6], str(widths))
wide_mm = sum(abs(l[2] - l[0]) for l in legs if l[5] > 0.5)
check('...most of the 10 mm is at 0.6', wide_mm > 8.0, f'{wide_mm:.2f} mm')
check('...the legs tile the link end to end',
      abs(legs[0][0]) < 1e-9 and abs(legs[-1][2] - 10.0) < 1e-9
      and all(abs(legs[i][2] - legs[i + 1][0]) < 1e-9 for i in range(len(legs) - 1)))
check('...the narrow leg covers the pinch',
      any(l[5] < 0.5 and l[0] <= 5.2 and l[2] >= 4.8 for l in legs))
check('...every wide leg is clear on its own, end caps included',
      all(wide_route_clear([(l[0], l[1], l[4]), (l[2], l[3], l[4])], l[5], pcb, 1,
                           cfg, board_edge_clearance=0.0) for l in legs if l[5] > 0.5))

legs = widen_link_legs(LINK, 0.4, board([]), 1, cfg)
check('no pinch: one leg, whole, at the net width',
      [(round(l[0], 3), round(l[2], 3), l[5]) for l in legs] == [(0.0, 10.0, 0.6)], str(legs))
plain = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.4, clearance=0.3)
legs = widen_link_legs(LINK, 0.4, pcb, 1, plain)
check('a net that asked for nothing wider is unchanged',
      [l[5] for l in legs] == [0.4], str(legs))

src = open(os.path.join(ROOT, 'py_router', 'kicad_oracle.py'), encoding='utf-8').read()
tree = ast.parse(src)
calls = [n.lineno for n in ast.walk(tree) if isinstance(n, ast.Call)
         and getattr(n.func, 'id', '') == 'widen_link_legs']
check('the shared emission path (both tiers) writes the widened legs',
      len(calls) == 1, str(calls))

print()
if FAILS:
    print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
    sys.exit(1)
print('ALL PASS')
