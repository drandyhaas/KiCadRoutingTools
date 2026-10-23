#!/usr/bin/env python3
"""A claimed #189/#535 unblock cell is drilled at the site the unblock checked.

_register_unblock_via registers a FREE VIA on the cell of every unblock via
it places, and the claim outlives a via the route drops. That is the #212
mechanism: a later search -- an orphan fallback into the same boxed pad --
changes layer on the cell and reaches the pad through the shrunk via an
earlier, failed retry found legal. Route conversion used to drill that via
at the CELL CENTRE, though the unblock checks its via on a 0.05 mm fine
pass, so the centre can sit up to half a cell from the checked site.
Measured on a GND tap edge into U1.4: the #535 escape via was placed
legally at (147.65, 111.35) and dropped with its edge; the orphan fallback
drilled its cell at (147.6, 111.4), 8.5 um inside U1.5's clearance. Taking
the claim away instead fixed that board and cost two others the route
through a legal shrunk via-in-pad (one GND net ended in two parts and was
rescued at 0.127 mm). So the claim stays, and conversion drills at the site.

Rows:
  1. hand-built pocket (the pad's pocket sealed on F.Cu and via-blocked):
     the unblock places its via and the retry fails; the claim is left on
     the cell, and a later search out of the pocket changes layer there;
  2. conversion: a path that changes layer on a cell with a registered
     site drills the via at the site, and the refit judges the site; a cell
     with no site is drilled at its centre (control);
  3. source: _register_unblock_via adds the free via, the unblock records a
     site with every size it registers, and both emission sites drill
     through _emit_via_site.

    python3 tests/test_unblock_via_exact_site.py
"""
import ast
import contextlib
import io
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

import single_ended_routing as ser                        # noqa: E402
from grid_router import GridObstacleMap, GridRouter       # noqa: E402
from kicad_parser import BoardInfo, Via                   # noqa: E402
from routing_config import GridCoord, GridRouteConfig     # noqa: E402
from synth import make_net, make_pcb                      # noqa: E402

FAILS = []
STEP = 0.05
T = (200, 200)          # the boxed pad's cell, (10.0, 10.0) mm
S = (100, 200)          # source, 5 mm west
WALL = 3
CFG = dict(layers=['F.Cu', 'B.Cu'], grid_step=STEP, clearance=0.15,
           track_width=0.2, via_size=0.45, via_drill=0.3,
           hole_to_hole_clearance=0.25)
UNBLOCK = Via(x=10.0, y=10.0, size=0.25, drill=0.15,
              layers=['F.Cu', 'B.Cu'], net_id=1)


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def board():
    return make_pcb(nets={1: make_net(1, 'N')},
                    board_info=BoardInfo(layers={0: 'F.Cu', 2: 'B.Cu'},
                                         copper_layers=['F.Cu', 'B.Cu']))


def pocket_map():
    """T in a pocket: F.Cu wall 3 cells out, every pocket cell via-blocked,
    B.Cu open only inside the pocket -- so no retry from S can reach it."""
    obs = GridObstacleMap(2)
    for dx in range(-WALL, WALL + 1):
        for dy in range(-WALL, WALL + 1):
            obs.add_blocked_via(T[0] + dx, T[1] + dy)
            if max(abs(dx), abs(dy)) == WALL:
                obs.add_blocked_cell(T[0] + dx, T[1] + dy, 0)
                obs.add_blocked_cell(T[0] + dx, T[1] + dy, 1)
    for gx in range(S[0] - 6, T[0] + 31):
        for gy in range(T[1] - 31, T[1] + 31):
            if max(abs(gx - T[0]), abs(gy - T[1])) > WALL:
                obs.add_blocked_cell(gx, gy, 1)
            if gx in (S[0] - 6, T[0] + 30) or gy in (T[1] - 31, T[1] + 30):
                obs.add_blocked_cell(gx, gy, 0)
    return obs


def run_unblock(obs):
    cfg = GridRouteConfig(**CFG)
    pcb = board()
    pcb._via_unblock_failed = set()   # the real placement creates it
    pad = SimpleNamespace(component_ref='U1', pad_number='1', global_x=10.0,
                          global_y=10.0, size_x=0.3, size_y=0.3, drill=0,
                          layers=['F.Cu'])
    placed = []
    saved = {n: getattr(ser, n) for n in
             ('_net_pad_near', '_net_pads_near', '_place_shrunk_via_in_pad')}
    ser._net_pad_near = lambda *a, **k: pad
    ser._net_pads_near = lambda *a, **k: []
    ser._place_shrunk_via_in_pad = lambda *a, **k: (
        placed.append(1) or (UNBLOCK, T, 0, []))
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            res = ser._route_with_via_unblock(
                GridRouter(via_cost=50, h_weight=1.5, turn_cost=100), obs, cfg,
                [(S[0], S[1], 0)], [(T[0], T[1], 0)], 0, pcb, 1)
    finally:
        for n, f in saved.items():
            setattr(ser, n, f)
    return res, len(placed)


def rows_pocket():
    print('1. hand-built pocket: the dropped via leaves its claim')
    obs = pocket_map()
    res, n_placed = run_unblock(obs)
    check('precondition: the unblock placed its via and the retry failed '
          '(the via is dropped)', n_placed == 1 and res[0] is None,
          f'placed {n_placed}, path {res[0] is not None}')
    check('the claim is left on the dropped via cell', obs.is_free_via(*T))
    r = GridRouter(via_cost=50, h_weight=1.5, turn_cost=100)
    path, _it, _bl = r.route_with_frontier(obs, [(T[0], T[1], 0)],
                                           [(T[0] + 1, T[1], 1)], 5000)
    changes = [p[:2] for a, p in zip(path or [], (path or [])[1:])
               if a[:2] == p[:2] and a[2] != p[2]]
    check('a later search out of the pocket changes layer on the claimed cell',
          path is not None and list(T) in [list(c) for c in changes],
          f'path {path is not None}, layer changes {changes}')


def convert(pcb, cell):
    cfg = GridRouteConfig(**CFG)
    coord = GridCoord(STEP)
    x, y = coord.to_float(*cell)
    path = [(cell[0] - 4, cell[1], 0), (cell[0], cell[1], 0),
            (cell[0], cell[1], 1), (cell[0] + 4, cell[1], 1)]
    seen = []
    real = ser._unblock_via_refit

    def spy(pcb_data, net_id, vx, vy, rec, config, *a, **k):
        seen.append((round(vx, 4), round(vy, 4)))
        return real(pcb_data, net_id, vx, vy, rec, config, *a, **k)
    ser._unblock_via_refit = spy
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            _segs, vias = ser._path_to_segments_vias(
                path, coord, ['F.Cu', 'B.Cu'], 1, cfg,
                (x - 4 * STEP, y, 'F.Cu'), (x + 4 * STEP, y, 'B.Cu'),
                pcb_data=pcb)
    finally:
        ser._unblock_via_refit = real
    return vias, seen, (x, y)


def rows_conversion():
    print('2. conversion drills a claimed cell at its checked site')
    site = (10.025, 9.975)
    pcb = board()
    pcb._unblock_via_sizes = {T: (0.25, 0.15)}
    pcb._unblock_via_sites = {T: site}
    vias, seen, centre = convert(pcb, T)
    got = [(round(v.x, 4), round(v.y, 4), v.size) for v in vias]
    check('the via is drilled at the registered site, at its registered size',
          got == [(site[0], site[1], 0.25)], f'vias {got}, cell centre {centre}')
    check('the refit judged the site, not the cell centre',
          seen == [site], f'refit at {seen}')
    pcb = board()
    vias, seen, centre = convert(pcb, T)
    got = [(round(v.x, 4), round(v.y, 4)) for v in vias]
    check('control: a cell with no site is drilled at its centre',
          got == [(round(centre[0], 4), round(centre[1], 4))], f'vias {got}')


def row_source():
    print('3. source')
    tree = ast.parse(open(os.path.join(ROOT, 'py_router', 'single_ended_routing.py'),
                          encoding='utf-8').read())
    fns = {n.name: n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}

    def calls(fn, name):
        return [c for c in ast.walk(fns[fn]) if isinstance(c, ast.Call)
                and getattr(c.func, 'attr', getattr(c.func, 'id', '')) == name]
    check('_register_unblock_via adds the free via',
          len(calls('_register_unblock_via', 'add_free_via')) == 1)
    src = ast.unparse(fns['_place_shrunk_via_in_pad_impl'])
    check('the unblock records a site with the size it registers',
          "sizes[vgx, vgy] = (v['size'], v['drill'])" in src
          and "sites[vgx, vgy] = (v['x'], v['y'])" in src)
    for fn in ('_route_net_with_obstacles_impl', '_path_to_segments_vias'):
        check(f'{fn} drills through _emit_via_site',
              len(calls(fn, '_emit_via_site')) == 1)


def main():
    rows_pocket()
    rows_conversion()
    row_source()
    print('-' * 60)
    if FAILS:
        print(f'{len(FAILS)} FAILED: ' + '; '.join(FAILS))
        sys.exit(1)
    print('ALL PASS')


if __name__ == '__main__':
    main()
