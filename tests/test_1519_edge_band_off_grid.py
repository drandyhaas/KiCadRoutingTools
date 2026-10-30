#!/usr/bin/env python3
"""The board-edge keep-out holds the edge clearance beside a rectangular
outline whose edge lies between grid lines (#1519).

A plain-rectangle outline used to be stamped from its bounding box ROUNDED to
the grid: a right edge at x = 40.08 on a 0.05 grid became 40.10, and a 0.0963
track (reach 0.2 + 0.04815 = 0.24815) was let onto x = 39.85, 0.1819 from the
real edge. The polygon path measures the exact edge and was right. Rows:

  - GridCoord.edge_band_limits against the band's own definition, and against
    the old formula on an on-grid edge (where nothing may change);
  - the route map's track and via bands, at the issue's widths;
  - the plane builder's track and via bands, same board;
  - the terminal neck: a route landing on a fanout stub end inside its own
    edge band is narrowed to hold the clearance; one landing on its own pad in
    the band is not (#441);
  - route.py end to end on the issue's one-net board at width 0.0963.

    python3 tests/test_1519_edge_band_off_grid.py
"""
import math
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))

import numpy as np                                            # noqa: E402
from kicad_parser import (BoardInfo, Net, Pad, PCBData,       # noqa: E402
                          Segment, parse_kicad_pcb)
from routing_config import GridCoord, GridRouteConfig         # noqa: E402
from obstacle_map import add_board_edge_obstacles             # noqa: E402
import plane_obstacle_builder as pob                          # noqa: E402
from single_ended_routing import _neck_terminal_grazes        # noqa: E402
from run_utils import evidence, tool, tool_env                # noqa: E402

FAILS = []
STEP, EDGE_CLR = 0.05, 0.2
BOUNDS = (0.0, 0.0, 40.08, 40.0)       # right edge off the grid, others on it
ROW = 100                               # y = 5.0, far from top and bottom


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


class Recorder:
    """Stands in for GridObstacleMap: records every cell and via stamped."""
    def __init__(self):
        self.cells, self.vias = set(), set()

    def _cells(self, arr):
        for gx, gy, li in np.asarray(arr).tolist():
            self.cells.add((gx, gy, li))

    def _vias(self, arr):
        for gx, gy in np.asarray(arr).tolist():
            self.vias.add((gx, gy))

    add_static_blocked_cells_batch = add_blocked_cells_batch = _cells
    add_static_blocked_vias_batch = add_blocked_vias_batch = _vias


def board(bounds=BOUNDS, pads=()):
    nets = {1: Net(1, '/A'), 2: Net(2, '/B')}
    return PCBData(board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=bounds),
                   nets=nets, footprints={}, vias=[], segments=[],
                   pads_by_net={1: list(pads)} if pads else {})


def cfg(width):
    return GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=width,
                           clearance=0.09, grid_step=STEP, via_size=0.3,
                           board_edge_clearance=EDGE_CLR)


def right_band(cells_on_row, reach, strict):
    """(blocked, should-be-blocked) x indices along ROW near the right edge."""
    lo, hi = int(39.0 / STEP), int(40.6 / STEP)
    got, want = [], []
    for gx in range(lo, hi + 1):
        d = BOUNDS[2] - gx * STEP
        if abs(d - reach) < 1e-9:
            continue                    # a tie: either rule is legal
        if gx in cells_on_row:
            got.append(gx)
        if (d < reach) if strict else (d <= reach):
            want.append(gx)
    return got, want


# -- the route map (obstacle_map.add_board_edge_obstacles) ------------------
for w in (0.0963, 0.09, 0.1, 0.12):
    rec = Recorder()
    add_board_edge_obstacles(rec, board(), cfg(w))
    reach = EDGE_CLR + w / 2
    row = {gx for gx, gy, li in rec.cells if gy == ROW and li == 0}
    got, want = right_band(row, reach, strict=False)
    check(f'route map, width {w}: the track band is every cell within '
          f'{reach:.5f} of the EXACT edge', got == want,
          f'first blocked x={min(got) * STEP:.2f}, want {min(want) * STEP:.2f}')
    check(f'route map, width {w}: x=39.85 (0.23 from the edge) is blocked',
          (round(39.85 / STEP), ROW, 0) in rec.cells)
    vrow = {gx for gx, gy in rec.vias if gy == ROW}
    vreach = EDGE_CLR + 0.3 / 2
    got, want = right_band(vrow, vreach, strict=True)
    check(f'route map, width {w}: the via band too', got == want,
          f'first blocked x={min(got) * STEP:.2f}, want {min(want) * STEP:.2f}')

# -- the plane builder's rectangular bands ----------------------------------
rec = Recorder()
pob._add_board_edge_track_obstacles(rec, board(), cfg(0.0963), 0, track_width=0.0963)
got, want = right_band({gx for gx, gy, li in rec.cells if gy == ROW},
                       EDGE_CLR + 0.0963 / 2, strict=True)
check('plane builder: the track band is measured from the exact edge',
      got == want, f'first blocked x={min(got) * STEP:.2f}, want {min(want) * STEP:.2f}')
rec = Recorder()
pob._add_board_edge_via_obstacles(rec, board(), cfg(0.0963))
got, want = right_band({gx for gx, gy in rec.vias if gy == ROW},
                       EDGE_CLR + 0.3 / 2, strict=True)
check('plane builder: the via band is measured from the exact edge',
      got == want, f'first blocked x={min(got) * STEP:.2f}, want {min(want) * STEP:.2f}')


# -- the terminal neck -------------------------------------------------------
def neck(term, pads=()):
    """A 0.2 terminal leg along y=5 ending at x=`term`."""
    seg = Segment(start_x=39.0, start_y=5.0, end_x=term, end_y=5.0, width=0.2,
                  layer='F.Cu', net_id=1)
    n, hard = _neck_terminal_grazes([seg], [(term, 5.0)], board(pads=pads), 1,
                                    cfg(0.2), floor=0.1)
    return n, hard, seg


# 39.80 is 0.28 from the edge: a 0.2 track needs 0.30, a 0.16 one fits.
n, hard, seg = neck(39.80)
gap = BOUNDS[2] - 39.80 - seg.width / 2
check('neck: a terminal on a stub end inside the edge band is narrowed',
      n == 1 and not hard and seg.width < 0.2, f'width={seg.width}')
check('neck: ...to hold the edge clearance', gap >= EDGE_CLR - 1e-9,
      f'copper-to-edge {gap:.4f}')
n, hard, seg = neck(39.5)
check('neck: a terminal clear of the edge band keeps its width',
      n == 0 and seg.width == 0.2, f'width={seg.width}')
pad = Pad(component_ref='J1', pad_number='1', global_x=39.80, global_y=5.0,
          local_x=0.0, local_y=0.0, size_x=0.3, size_y=0.3, shape='rect',
          layers=['F.Cu'], net_id=1, net_name='/A')
n, hard, seg = neck(39.80, pads=[pad])
check('neck: a terminal on its own pad in the band is left alone (#441)',
      n == 0 and seg.width == 0.2, f'width={seg.width}')


# -- GridCoord.edge_band_limits ---------------------------------------------
coord = GridCoord(STEP)
if not hasattr(coord, 'edge_band_limits'):
    check('GridCoord.edge_band_limits exists', False)
else:
    bad = []
    for lo_mm, hi_mm in ((0.0, 40.08), (-3.217, 12.333), (1.0, 20.0)):
        for reach in (0.24815, 0.25, 0.3, 0.375, 0.1):
            for strict in (False, True):
                lo, hi = coord.edge_band_limits(lo_mm, hi_mm, reach, strict)
                for g in range(math.floor(lo_mm / STEP) - 5, math.ceil(lo_mm / STEP) + 15):
                    d = g * STEP - lo_mm
                    if abs(d - reach) > 1e-9 and \
                            (g <= lo) != ((d < reach) if strict else (d <= reach)):
                        bad.append((lo_mm, reach, strict, 'lo', g))
                for g in range(math.floor(hi_mm / STEP) - 15, math.ceil(hi_mm / STEP) + 5):
                    d = hi_mm - g * STEP
                    if abs(d - reach) > 1e-9 and \
                            (g >= hi) != ((d < reach) if strict else (d <= reach)):
                        bad.append((hi_mm, reach, strict, 'hi', g))
    check('edge_band_limits: the band is every cell within the reach of the exact edge',
          not bad, f'{bad[:3]}')
    same = True
    for lo_mm, hi_mm in ((0.0, 40.0), (-12.35, 7.7), (101.6, 152.4)):
        for reach in (0.24815, 0.25, 0.3, 0.57, 0.375):
            g0, _ = coord.to_grid(lo_mm, 0)
            g1, _ = coord.to_grid(hi_mm, 0)
            t, k = coord.to_grid_dist(reach), coord.to_grid_dist_safe(reach)
            same &= coord.edge_band_limits(lo_mm, hi_mm, reach, False) == (g0 + t, g1 - t)
            same &= coord.edge_band_limits(lo_mm, hi_mm, reach, True) == (g0 + k - 1, g1 - k + 1)
    check('edge_band_limits: an on-grid edge gets exactly the band it had before', same)


# -- route.py end to end -----------------------------------------------------
from constraint_agreement import write_board                  # noqa: E402

with tempfile.TemporaryDirectory() as tmp:
    src, out = os.path.join(tmp, 'e1.kicad_pcb'), os.path.join(tmp, 'o.kicad_pcb')
    fab = os.path.join(tmp, 'fab.txt')
    with open(fab, 'w') as f:
        f.write('track_width = 0.09\nclearance = 0.09\n')
    write_board(src, edge=BOUNDS, footprints=[
        {'ref': 'U1', 'x': 39.85, 'y': 5, 'net_id': 1, 'net_name': 'A',
         'layer': 'F.Cu', 'size': 0.05},
        {'ref': 'U2', 'x': 39.85, 'y': 35, 'net_id': 1, 'net_name': 'A',
         'layer': 'F.Cu', 'size': 0.05}])
    evidence(src, 'fixture board')
    r = subprocess.run([sys.executable, '-X', 'utf8', tool('route.py'), src, out,
                        '--nets', 'A', '--track-width', '0.0963', '--clearance', '0.09',
                        '--board-edge-clearance', str(EDGE_CLR), '--grid-step', str(STEP),
                        '--layers', 'F.Cu', 'B.Cu', '--fab-overrides', fab],
                       capture_output=True, text=True, env=tool_env(), cwd=ROOT,
                       timeout=300)
    check('route.py ran', r.returncode == 0, (r.stdout + r.stderr)[-400:])
    if r.returncode == 0:
        evidence(out, 'routed board')
        segs = parse_kicad_pcb(out).segments
        gaps = [BOUNDS[2] - max(s.start_x, s.end_x) - s.width / 2 for s in segs]
        check('route.py: the net is routed', bool(segs))
        check('route.py: every track holds the edge clearance from the off-grid edge',
              segs and min(gaps) >= EDGE_CLR - 1e-6,
              f'min copper-to-edge {min(gaps):.4f}' if gaps else '')

print()
if FAILS:
    print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
    sys.exit(1)
print('ALL PASS')
