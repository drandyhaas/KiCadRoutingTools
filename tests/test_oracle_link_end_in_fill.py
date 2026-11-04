#!/usr/bin/env python3
"""Copper the oracle lays at the end of a route keeps its clearance.

Two paths laid copper no later pass looks at:

  1. A forced oracle link (#572) routes between KiCad's own nearest-approach
     anchors. An anchor that is not on the net's pad, via or track sits on
     its island's EDGE, which stands at clearance from the foreign copper
     that carved it, so a track started there reaches half its width into
     that clearance -- from a grid cell the obstacle map blocks.
     `step_link_end_into_fill` moves such an end into its own island (away
     from the other end) to the first point on fill whose cell is free, and
     leaves an end on own copper, or with no such point in reach, as it was.
  2. The #648 sliver weld checked foreign tracks, vias, pads and holes but not
     the board edge or a milled slot; on a plain rectangular board the edge
     reaches no outline rings, so the bounds are the edge there.

    python3 tests/test_oracle_link_end_in_fill.py
"""
import ast
import os
import sys
import tempfile
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for p in (ROOT, TESTS_DIR, os.path.join(ROOT, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import RoutingBoard, make_pcb, make_via            # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402
from routing_config import GridRouteConfig, GridCoord          # noqa: E402
from run_utils import evidence                                 # noqa: E402
import plane_fill_model                                        # noqa: E402
import single_ended_routing as ser                             # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail else ''))


class Fill:
    """The net's fill on F.Cu: everything left of x = 10."""
    def query_component(self, x, y, size=0.0):
        return 1 if x < 10.0 else 0


class Map:
    """Foreign copper right of the fill carves a band: cells at x >= 9.85
    are blocked (or every cell, with `all_blocked`)."""
    def __init__(self, step, all_blocked=False):
        self.step, self.all_blocked = step, all_blocked

    def is_blocked(self, gx, gy, layer):
        return self.all_blocked or gx * self.step >= 9.85 - 1e-9


def step_cases():
    print('1. a forced link end on its fill edge starts inside the island')
    if not hasattr(ser, 'step_link_end_into_fill'):
        check('single_ended_routing.step_link_end_into_fill exists', False)
        return
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.2,
                          clearance=0.1, grid_step=0.05)
    coord = GridCoord(cfg.grid_step)
    real = plane_fill_model.get_fill_models
    plane_fill_model.get_fill_models = lambda pcb, nid: {'F.Cu': [Fill()]}
    try:
        bi = SimpleNamespace(copper_layers=['F.Cu', 'B.Cu'])
        pcb = make_pcb(board_info=bi)
        end, other = (10.0, 5.0), (11.0, 5.0)
        got = ser.step_link_end_into_fill(pcb, 1, end, other, 'F.Cu', 0, cfg,
                                          Map(cfg.grid_step))
        gx, gy = coord.to_grid(*got)
        check('the end moves away from the other end, along their line',
              got[0] < end[0] and abs(got[1] - end[1]) < 1e-9, str(got))
        check('it lands on the fill, on a cell the map leaves free',
              Fill().query_component(*got) > 0
              and not Map(cfg.grid_step).is_blocked(gx, gy, 0), str(got))
        check('it moves no further than the first such point',
              end[0] - got[0] <= 0.2 + cfg.grid_step, f'{end[0] - got[0]:.3f} mm')

        on_via = make_pcb(board_info=bi, vias=[make_via(10.0, 5.0, size=0.5)])
        check('an end on the net\'s own via stays where it is',
              ser.step_link_end_into_fill(on_via, 1, end, other, 'F.Cu', 0,
                                          cfg, Map(cfg.grid_step)) == end)
        check('with no free cell on fill in reach the end stays where it is',
              ser.step_link_end_into_fill(pcb, 1, end, other, 'F.Cu', 0, cfg,
                                          Map(cfg.grid_step, True)) == end)
    finally:
        plane_fill_model.get_fill_models = real

    src = open(os.path.join(ROOT, 'py_router', 'single_ended_routing.py')).read()
    fn = next(n for n in ast.walk(ast.parse(src))
              if isinstance(n, ast.FunctionDef) and n.name == 'route_oracle_links')
    calls = [c for c in ast.walk(fn) if isinstance(c, ast.Call)
             and getattr(c.func, 'id', None) == 'step_link_end_into_fill']
    check('route_oracle_links steps both ends of every link', len(calls) == 2,
          f'{len(calls)} call(s)')


def weld_cases():
    print('2. the sliver weld keeps the board edge clearance')
    from kicad_oracle import _direct_sliver_weld
    with tempfile.TemporaryDirectory() as td:
        b = RoutingBoard(w=40, h=30)
        b.pad('R1', 20, 20, 'A')
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w') as fh:
            fh.write(b.text())
        evidence(path)
        pcb = parse_kicad_pcb(path)
        nid = next(i for i, n in pcb.nets.items() if n.name == 'A')

        def weld(x, **kw):
            c = GridRouteConfig(clearance=0.2, track_width=0.2,
                                layers=['F.Cu', 'B.Cu'])
            for k, v in kw.items():
                setattr(c, k, v)
            # a vertical weld at x: its centreline is x from the left edge
            return _direct_sliver_weld(pcb, nid, x, 10.0, x, 10.6, 'F.Cu', c)
        check('a weld clear of the edge is laid', weld(0.35) is not None)
        check('a weld inside the edge band (clearance + half width) is refused',
              weld(0.25) is None)
        check("the board's own edge clearance prices the band",
              weld(0.35, board_edge_clearance=0.5) is None)
        check('...and a weld clear of that wider band is laid',
              weld(0.65, board_edge_clearance=0.5) is not None)


def main():
    step_cases()
    weld_cases()
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
