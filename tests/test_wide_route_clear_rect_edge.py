#!/usr/bin/env python3
"""The wide-copper gate measures the board edge on a plain rectangle.

A rectangular Edge.Cuts parses with no outline ring (only its bounds), and
`wide_route_clear` -- the gate every plane-join width upgrade and every
oracle strap's piecewise widening (#1169) asks -- measured the edge only
against rings. On such a board it measured no edge at all: an oracle strap
routed at the stamp width along the edge band was widened to the net's 0.3 mm
and shipped 0.07 mm inside a 0.5 mm edge rule. `board_edge_geometry(...,
rect_ring=True)` gives the rectangle as the outer ring. Rows:

  - board_edge_geometry: no ring by default, the rectangle with rect_ring,
    the REAL board's rectangle on a local window (parent_board_bounds);
  - wide_route_clear refuses a 0.3 mm track whose copper is inside the edge
    band, accepts it once it clears, and accepts the narrow track that does;
  - on a local window, the window's own side is no edge.

    python3 tests/test_wide_route_clear_rect_edge.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import BoardInfo, Net, PCBData             # noqa: E402
from routing_config import GridRouteConfig                    # noqa: E402
from check_drc import board_edge_geometry                     # noqa: E402
from plane_region_connector import wide_route_clear           # noqa: E402

FAILS = []
EDGE = 0.5
BOUNDS = (94.1, 61.42, 188.08, 112.22)


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def board(bounds=BOUNDS, parent=None):
    bi = BoardInfo(layers={}, copper_layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
                   board_bounds=bounds)
    if parent is not None:
        bi.parent_board_bounds = parent
    return PCBData(board_info=bi, nets={1: Net(1, '+3V3')}, footprints={},
                   vias=[], segments=[], pads_by_net={})


CFG = GridRouteConfig(layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], track_width=0.15,
                      clearance=0.09, grid_step=0.1, board_edge_clearance=EDGE)


def clear(pcb, y, w):
    return wide_route_clear([(140.0, y, 'In2.Cu'), (150.0, y, 'In2.Cu')], w,
                            pcb, 1, CFG, board_edge_clearance=EDGE)


def main():
    print('board_edge_geometry')
    pcb = board()
    rings, outer, _ = board_edge_geometry(pcb.board_info)
    check('a plain rectangle parses with no ring', not rings and not outer)
    rings, outer, cuts = board_edge_geometry(pcb.board_info, rect_ring=True)
    xs = sorted({p[0] for r in rings for p in r})
    ys = sorted({p[1] for r in rings for p in r})
    check('rect_ring returns the rectangle as the outer ring',
          len(rings) == 1 and outer == rings and xs == [94.1, 188.08]
          and ys == [61.42, 112.22], f'{rings}')
    win = board(bounds=(130.0, 61.42, 160.0, 70.0), parent=BOUNDS)
    rings, _, _ = board_edge_geometry(win.board_info, rect_ring=True)
    xs = sorted({p[0] for r in rings for p in r})
    check('on a window the ring is the real board, not the window',
          xs == [94.1, 188.08], f'{xs}')

    print('wide_route_clear (edge 0.5 mm at y = 61.42)')
    # centre 0.58 from the edge: a 0.3 track's copper is at 0.43, inside 0.5
    check('refuses 0.3 mm copper 0.43 mm from the edge', not clear(pcb, 62.0, 0.3))
    check('refuses 0.15 mm copper 0.005 mm from the edge', not clear(pcb, 61.5, 0.15))
    check('accepts 0.15 mm copper 0.505 mm from the edge', clear(pcb, 62.0, 0.15))
    check('accepts 0.3 mm copper 0.53 mm from the edge', clear(pcb, 62.1, 0.3))
    check('refuses 0.3 mm copper 0.47 mm from the bottom edge',
          not wide_route_clear([(170.0, 111.6, 'In2.Cu'), (172.0, 111.6, 'In2.Cu')],
                               0.3, pcb, 1, CFG, board_edge_clearance=EDGE))
    # a window whose own top side is the real edge and whose bottom (70.0) is not
    check('window: the real edge still refuses', not clear(win, 62.0, 0.3))
    check("window: its own side is no edge", clear(win, 69.9, 0.3))

    print('FAIL' if FAILS else 'PASS', f'({len(FAILS)} failed)' if FAILS else '')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
