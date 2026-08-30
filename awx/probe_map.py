#!/usr/bin/env python3
"""ASCII map of what walls one lane: route the corridor's earlier lanes
as the braid does (attempt 0), then build the router's obstacle map for
this lane exactly as connect() does and print, for a board-coordinate
box, each cell on each layer:  '#' blocked by copper (no band applied),
'b' free of copper but outside the band, '.' free.  Vias of the routed
lanes are marked 'V', the lane's own two ends 'A' and 'Z'.

usage: probe_map.py FANOUT_BOARD K NET --box x0,y0,x1,y1 [--no-virtual]
       [--step 0.05] [--dest REF]
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import braid as te  # noqa: E402
import connect as cn  # noqa: E402
from schedule import Schedule  # noqa: E402
from kicad_parser import Segment  # noqa: E402
from routing_config import GridCoord  # noqa: E402
from routing_utils import build_layer_map  # noqa: E402
from plane_pad_tap import make_local_window  # noqa: E402
from obstacle_map import (build_base_obstacle_map,  # noqa: E402
                          add_same_net_via_clearance,
                          add_same_net_pad_drill_via_clearance,
                          same_net_pad_via_keepout_cells)
from routing_context import _add_free_via_positions  # noqa: E402
from net_rescue import _fence_window  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('net')
    ap.add_argument('--box', required=True)
    ap.add_argument('--dest', default='DU1')
    ap.add_argument('--no-virtual', action='store_true')
    ap.add_argument('--drop', default='',
                    help='NET[,NET]: leave these lanes\' copper out of the map')
    ap.add_argument('--drop-what', default='all', choices=('all', 'segs', 'vias'))
    ap.add_argument('--step', type=float, default=0.05,
                    help='print every n-th grid cell so the map fits')
    a = ap.parse_args()
    nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    quiet = lambda *x: None  # noqa: E731
    ctx, groups = te.setup(a.board, names, a.dest, quiet)
    grp = next(g for g in groups if a.net in g)
    c = te.Corridor(0, grp, ctx, quiet)
    c.build_spine()
    c.classify()
    c.offsets(0.35)
    c.reserve_intervals()
    sched = Schedule(c.launch, c.target, ctx.tooth_layer)
    cols, _gate = c.plan_columns(sched, {d: 1 for d in sched.divers},
                                 {d: 0 for d in sched.divers})
    c.lay_lanes(cols)
    order = list(sched.priority) + [x for x in c.target if x not in sched.divers]
    routed = set()
    for om in order:
        if om == a.net:
            break
        virt = c.virtual_of([x for x in grp if x != om and x not in routed])
        res = c.route_lane(om, virt)
        if res is None:
            print(f'(earlier lane {om} refused)')
            continue
        routed.add(om)
    nm = a.net
    nid = ctx.byname[nm][0]
    virt = None if a.no_virtual else c.virtual_of(
        [x for x in grp if x != nm and x not in routed])
    band = c.band_of(nm)
    cfg = ctx.cfg
    pcb = ctx.pcb
    aa, bb = c.teeth[nm], c.stubs[nm]
    # --- connect()'s window and obstacle map, verbatim
    coord = GridCoord(cfg.grid_step)
    layer_map = build_layer_map(cfg.layers)
    pts = [aa, bb] + list(c.lane_xy[nm])
    bx0, bx1 = min(p[0] for p in pts), max(p[0] for p in pts)
    by0, by1 = min(p[1] for p in pts), max(p[1] for p in pts)
    cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
    half = max(bx1 - bx0, by1 - by0) / 2 + 0.6
    window = make_local_window(pcb, cx, cy, half)
    if a.drop:
        drop_ids = {ctx.byname[x][0] for x in a.drop.split(',') if x}
        n0 = len(window.segments) + len(window.vias)
        if a.drop_what in ('all', 'segs'):
            window.segments = [s for s in window.segments if s.net_id not in drop_ids]
        if a.drop_what in ('all', 'vias'):
            window.vias = [v for v in window.vias if v.net_id not in drop_ids]
        print(f'dropped {n0 - len(window.segments) - len(window.vias)} '
              f'{a.drop_what} of {a.drop}')
    if virt:
        window.segments = list(window.segments) + [
            Segment(p[0], p[1], q[0], q[1], cfg.track_width, layer, cn.VIRTUAL_NET)
            for (p, q, layer) in virt if layer in layer_map]
    obstacles = build_base_obstacle_map(window, cfg, [nid])
    _fence_window(obstacles, window, cfg)
    _add_free_via_positions(obstacles, window, [nid], cfg)
    add_same_net_via_clearance(obstacles, window, nid, cfg)
    add_same_net_pad_drill_via_clearance(obstacles, window, nid, cfg)
    keep = same_net_pad_via_keepout_cells(pcb, nid, cfg)
    if len(keep):
        obstacles.add_blocked_vias_batch(keep)
    cells = cn._band_cells(coord, window, band, list(cfg.layers), 0.0)
    banded = {(int(x), int(y), int(L)) for x, y, L in cells}
    x0, y0, x1, y1 = [float(v) for v in a.box.split(',')]
    every = max(1, int(round(a.step / cfg.grid_step)))
    gx0, gy0 = coord.to_grid(x0, y0)
    gx1, gy1 = coord.to_grid(x1, y1)
    marks = {}
    for v in pcb.vias:
        if x0 <= v.x <= x1 and y0 <= v.y <= y1:
            gx, gy = coord.to_grid(v.x, v.y)
            marks[(gx // every, gy // every)] = 'V'
    for p, ch in ((aa, 'A'), (bb, 'Z')):
        gx, gy = coord.to_grid(*p)
        marks[(gx // every, gy // every)] = ch
    for L, lname in enumerate(cfg.layers):
        print(f'\n{lname}  x {x0}..{x1} (cols), y {y0}..{y1} (rows), '
              f'one char = {every} cell(s) of {cfg.grid_step} mm')
        header = ''
        for gx in range(gx0, gx1 + 1, every):
            x = coord.to_float(gx, 0)[0]
            header += str(int(x) % 10) if abs(x - round(x)) < cfg.grid_step * every / 2 else ' '
        print('        ' + header)
        for gy in range(gy0, gy1 + 1, every):
            y = coord.to_float(0, gy)[1]
            row = ''
            for gx in range(gx0, gx1 + 1, every):
                m = marks.get((gx // every, gy // every))
                if m:
                    row += m
                    continue
                blocked = obstacles.is_blocked(gx, gy, L) if hasattr(obstacles, 'is_blocked') \
                    else obstacles.blocked[gx, gy, L]
                if blocked:
                    row += '#'
                elif (gx, gy, L) in banded:
                    row += 'b'
                else:
                    row += '.'
            print(f'{y:7.2f} ' + row)


if __name__ == '__main__':
    main()
