#!/usr/bin/env python3
"""The #189 unblock retry keeps its OWN vias hole-to-hole off the unblock via.

_route_with_via_unblock drops a fab-floor via at a boxed pad and retries the
A* with that via's cell as an extra endpoint. The via is in neither pcb_data
nor any via map until the route commits, so nothing kept the retry's own
layer-change vias away from its drill. Measured: SJ1.1 is walled in
on F.Cu by its own tied bridge, the in-pad unblock drops a 0.25/0.15 via at
the pad corner, and the retry came along F.Cu, dropped to B.Cu with a
0.45/0.3 via 0.18 mm from it and stepped onto the via cell -- drills
overlapping against a 0.25 hole-to-hole rule. (192c98f2 covered only the
other direction: an unblock placement held off the net's uncommitted vias.)
_register_unblock_via now stamps the ring a tap edge's in-progress via gets
for the length of the retries.

Fixture: a hand-built two-layer map at grid 0.05. The target pad cell T sits
in a pocket sealed on F.Cu by a one-cell wall three cells out and via-blocked
inside the wall; B.Cu is open only within 0.85 mm of T, so the retry has to
come along F.Cu and change layer near the pad -- as close as the map lets it.
Pad lookup and via placement are stubbed (what they decide is not the
question); the search, the unblock wrapper and the map are the real ones.

Rows:
  1. precondition: the first search sticks on the target side, the unblock
     fires and the retry ends on the via cell on B.Cu;
  2. every via the retry drops is >= hole-to-hole off the unblock drill;
  3. negative control: with the ring disabled the retry drills inside it
     (the defect) -- so row 2 is not passing by accident of the geometry;
  4. the rings come off again: the via maps return to their counts on
     entry, on a retry that routes and on one that fails;
  5. a populated rung carries the ring while it is up, and loses it after;
  6. every _register_unblock_via call site hands over its via and a ring
     list, and _route_with_via_unblock releases them in a finally.

    python3 tests/test_unblock_via_retry_h2h.py
"""
import ast
import contextlib
import io
import math
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

import single_ended_routing as ser                       # noqa: E402
from grid_router import GridObstacleMap, GridRouter      # noqa: E402
from kicad_parser import BoardInfo, Via                  # noqa: E402
from routing_config import GridCoord, GridRouteConfig    # noqa: E402
from synth import make_net, make_pcb                     # noqa: E402

FAILS = []
PLACED = []                       # placements the stub made, per unblock() call
STEP = 0.05
T = (200, 200)                    # target pad cell: (10.0, 10.0) mm
S = (100, 200)                    # source cell, 5 mm west
WALL = 3                          # F.Cu wall, Chebyshev distance from T
B_OPEN = 17                       # B.Cu open within this many cells of T
H2H = 0.25
CFG = dict(layers=['F.Cu', 'B.Cu'], grid_step=STEP, clearance=0.15,
           track_width=0.2, via_size=0.45, via_drill=0.3,
           hole_to_hole_clearance=H2H)
UNBLOCK = Via(x=10.0, y=10.0, size=0.25, drill=0.15,
              layers=['F.Cu', 'B.Cu'], net_id=1)
NEED = (UNBLOCK.drill + CFG['via_drill']) / 2 + H2H     # 0.475 mm


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def build_map(b_open=B_OPEN, rung=False):
    obs = GridObstacleMap(2)
    for dx in range(-WALL, WALL + 1):
        for dy in range(-WALL, WALL + 1):
            obs.add_blocked_via(T[0] + dx, T[1] + dy)      # the pocket
            if max(abs(dx), abs(dy)) == WALL:
                obs.add_blocked_cell(T[0] + dx, T[1] + dy, 0)
    # B.Cu: closed outside a square around T (cells only; vias stay legal).
    for gx in range(S[0] - 5, T[0] + 30):
        for gy in range(T[1] - 30, T[1] + 30):
            if max(abs(gx - T[0]), abs(gy - T[1])) > b_open:
                obs.add_blocked_cell(gx, gy, 1)
    # ...and a fence on F.Cu so the search stays in the box.
    for gx in range(S[0] - 6, T[0] + 31):
        for gy in (T[1] - 31, T[1] + 30):
            obs.add_blocked_cell(gx, gy, 0)
            obs.add_blocked_cell(gx, gy, 1)
    for gy in range(T[1] - 31, T[1] + 31):
        for gx in (S[0] - 6, T[0] + 30):
            obs.add_blocked_cell(gx, gy, 0)
            obs.add_blocked_cell(gx, gy, 1)
    if rung:
        obs.add_blocked_via_rung(1, 9999, 9999)   # populate rung 1, far away
    return obs


def unblock(obs, ring=True):
    """One _route_with_via_unblock call on `obs`; returns (result, log)."""
    cfg = GridRouteConfig(**CFG)
    pcb = make_pcb(nets={1: make_net(1, 'N')},
                   board_info=BoardInfo(layers={0: 'F.Cu', 2: 'B.Cu'},
                                        copper_layers=['F.Cu', 'B.Cu']))
    pcb._via_unblock_failed = set()   # the real placement creates it
    pad = SimpleNamespace(component_ref='U1', pad_number='1', global_x=10.0,
                          global_y=10.0, size_x=0.3, size_y=0.3, drill=0,
                          layers=['F.Cu'])
    router = GridRouter(via_cost=50, h_weight=1.5, turn_cost=100)
    saved = {n: getattr(ser, n) for n in
             ('_net_pad_near', '_net_pads_near', '_place_shrunk_via_in_pad',
              'inprogress_via_ring_cells')}
    ser._net_pad_near = lambda *a, **k: pad
    ser._net_pads_near = lambda *a, **k: []
    PLACED.clear()
    ser._place_shrunk_via_in_pad = lambda *a, **k: (
        PLACED.append(1) or (UNBLOCK, T, 0, []))
    if not ring:
        ser.inprogress_via_ring_cells = lambda *a, **k: []
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            res = ser._route_with_via_unblock(
                router, obs, cfg, [(S[0], S[1], 0)], [(T[0], T[1], 0)], 0,
                pcb, 1)
    finally:
        for n, f in saved.items():
            setattr(ser, n, f)
    return res, out.getvalue()


def path_vias(path):
    return [p[:2] for a, p in zip(path, path[1:])
            if a[:2] == p[:2] and a[2] != p[2]]


def gap_mm(cell):
    return math.hypot(cell[0] * STEP - UNBLOCK.x, cell[1] * STEP - UNBLOCK.y)


def maps(obs):
    st = obs.get_stats()
    return (st[1], st[7])          # blocked_vias, all via rungs


def main():
    print('1-2. the retry, with the ring')
    obs = build_map()
    before = maps(obs)
    res, log = unblock(obs)
    path, used = res[0], res[9]
    ends = {tuple(path[0]), tuple(path[-1])} if path else set()
    check('precondition: the unblock fired and the retry ended on the via '
          'cell on B.Cu', used == [UNBLOCK] and (T[0], T[1], 1) in ends,
          f'ends {sorted(ends)}, used {len(used)}')
    vias = path_vias(path or [])
    check('the retry changes layer at least once (it has to reach B.Cu)',
          len(vias) >= 1, str(vias))
    worst = min((gap_mm(c) for c in vias), default=float('inf'))
    check(f'every retry via is >= {NEED:.3f} mm off the unblock via',
          worst >= NEED - 1e-9, f'nearest {worst:.3f} mm')
    check('row 4: the via maps are back to their entry counts (routed)',
          maps(obs) == before, f'{before} -> {maps(obs)}')

    print('3. negative control: no ring')
    obs = build_map()
    res, _ = unblock(obs, ring=False)
    vias = path_vias(res[0] or [])
    worst = min((gap_mm(c) for c in vias), default=float('inf'))
    check('without the ring the retry drills inside hole-to-hole (the defect)',
          res[0] is not None and worst < NEED - 1e-9, f'nearest {worst:.3f} mm')

    print('4. a retry that fails still takes its rings off')
    obs = build_map(b_open=WALL + 2)     # no legal via site left near T
    before = maps(obs)
    res, log = unblock(obs)
    check('precondition: the unblock placed its via and the retry failed',
          res[0] is None and len(PLACED) == 1, f'placed {len(PLACED)}')
    check('the via maps are back to their entry counts (failed)',
          maps(obs) == before, f'{before} -> {maps(obs)}')

    print('5. a populated rung carries the ring while it is up')
    if not hasattr(ser, '_release_unblock_via_rings'):
        check('the unblock ring API exists', False, 'no _release_unblock_via_rings')
        return report()
    obs = build_map(rung=True)
    cfg = GridRouteConfig(**CFG)
    coord = GridCoord(STEP)
    probe = (T[0] - 6, T[1])          # 0.30 mm west: inside the ring
    pre = (obs.is_via_blocked_rung(*probe, 1), obs.rung_len(1))
    rings = []
    ser._register_unblock_via(obs, T[0], T[1], cfg.layers, UNBLOCK, 1, cfg,
                              coord, rings)
    up = (obs.is_via_blocked(*probe), obs.is_via_blocked_rung(*probe, 1))
    ser._release_unblock_via_rings(obs, rings)
    post = (obs.is_via_blocked_rung(*probe, 1), obs.rung_len(1))
    check('precondition: rung 1 is populated and open at the probe',
          pre == (False, 1), str(pre))
    check('the ring blocks the probe on rung 0 and rung 1', up == (True, True), str(up))
    check('release restores rung 1 exactly', post == pre and rings == [],
          f'{pre} -> {post}')

    print('6. call sites')
    tree = ast.parse(open(os.path.join(ROOT, 'py_router', 'single_ended_routing.py'),
                          encoding='utf-8').read())
    sites = [c for c in ast.walk(tree) if isinstance(c, ast.Call)
             and isinstance(c.func, ast.Name) and c.func.id == '_register_unblock_via']
    check('every _register_unblock_via call hands over its via and a ring list',
          len(sites) >= 4 and all(len(c.args) >= 9 for c in sites),
          f'{len(sites)} site(s), arg counts {[len(c.args) for c in sites]}')
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
              and n.name == '_route_with_via_unblock')
    fin = [t for t in ast.walk(fn) if isinstance(t, ast.Try) and t.finalbody
           and '_release_unblock_via_rings' in ast.unparse(t.finalbody)]
    check('_route_with_via_unblock releases its rings in a finally', bool(fin))
    report()


def report():
    print('-' * 60)
    if FAILS:
        print(f'{len(FAILS)} FAILED: ' + '; '.join(FAILS))
        sys.exit(1)
    print('ALL PASS')


if __name__ == '__main__':
    main()
