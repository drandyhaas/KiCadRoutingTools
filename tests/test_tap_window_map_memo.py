#!/usr/bin/env python3
"""The tap window-map memo (plane_pad_tap._memo_window_build) returns a map
equal to a fresh build, hands out a fresh clone on every hit, and misses when
any input it keys on changes.

try_tap_pad rebuilds the via and routing maps for the same window with the
same inputs many times over a plane run (escalation ladder, retries, rounds
over pending pads); the memo serves the repeats. Checks, on esp_prog:

  1. via map from the memo (miss, then hit) equals a direct build, cell by cell;
  2. routing map likewise;
  3. a hit returns a new object each time (callers mutate their map);
  4. moving a window segment in place changes the key (a miss), and the map
     built then equals a direct build of the moved copper;
  5. KRT_TAP_MAP_CACHE=0 style off switch (wkey None) builds directly.

    python3 tests/test_tap_window_map_memo.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import plane_pad_tap as ppt  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from plane_obstacle_builder import (  # noqa: E402
    build_via_obstacle_map, build_routing_obstacle_map)
from routing_config import GridRouteConfig, GridCoord  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(("  ok   " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        FAILS.append(name)


def cells(cx, cy, half, step):
    coord = GridCoord(step)
    g0x, g0y = coord.to_grid(cx - half, cy - half)
    g1x, g1y = coord.to_grid(cx + half, cy + half)
    for gx in range(g0x, g1x + 1):
        for gy in range(g0y, g1y + 1):
            yield gx, gy


def via_sig(m, cx, cy, half, step):
    return [(m.is_via_blocked(gx, gy), m.get_stub_proximity_cost(gx, gy))
            for gx, gy in cells(cx, cy, half, step)]


def route_sig(m, cx, cy, half, step):
    return [(m.is_blocked(gx, gy, 0), m.get_layer_proximity_cost(gx, gy, 0))
            for gx, gy in cells(cx, cy, half, step)]


def main():
    pcb = parse_kicad_pcb(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'))
    config = GridRouteConfig(grid_step=0.05)
    pad = next(p for ps in pcb.pads_by_net.values() for p in ps
               if p.net_id and 'Cu' in ''.join(p.layers or ()))
    net_id = pad.net_id
    layer = next(l for l in pad.layers if l.endswith('.Cu') and '*' not in l)
    half = 3.0
    local = ppt.make_local_window(pcb, pad.global_x, pad.global_y, half)
    check('window has foreign copper', any(s.net_id != net_id for s in local.segments)
          or any(ps for n, ps in local.pads_by_net.items() if n != net_id))
    wkey = ppt._tap_window_key(local, pcb)

    def vmemo(loc, wk):
        return ppt._memo_window_build(pcb, build_via_obstacle_map, wk, loc, config,
                                      net_id, verbose=False, same_net_pad_clearance=-1.0)

    def rmemo(loc, wk):
        return ppt._memo_window_build(pcb, build_routing_obstacle_map, wk, loc, config,
                                      net_id, layer, skip_pad_blocking=False, verbose=False)

    direct_v = build_via_obstacle_map(local, config, net_id, verbose=False,
                                      same_net_pad_clearance=-1.0)
    ref_v = via_sig(direct_v, pad.global_x, pad.global_y, half, 0.05)
    m1 = vmemo(local, wkey)
    m2 = vmemo(local, wkey)
    check('via map: miss equals direct', via_sig(m1, pad.global_x, pad.global_y, half, 0.05) == ref_v)
    check('via map: hit equals direct', via_sig(m2, pad.global_x, pad.global_y, half, 0.05) == ref_v)
    check('one cached via entry', len(pcb._tap_map_cache) == 1, str(len(pcb._tap_map_cache)))
    check('hit is a fresh object', m1 is not m2 and m2 is not vmemo(local, wkey))
    # A caller mutating its map must not reach the cache.
    gx, gy = next(c for c in cells(pad.global_x, pad.global_y, half, 0.05)
                  if not m2.is_via_blocked(*c))
    m2.add_blocked_via(gx, gy)
    check('mutating a hit leaves the cache intact',
          not vmemo(local, wkey).is_via_blocked(gx, gy))

    direct_r = build_routing_obstacle_map(local, config, net_id, layer,
                                          skip_pad_blocking=False, verbose=False)
    ref_r = route_sig(direct_r, pad.global_x, pad.global_y, half, 0.05)
    r1 = rmemo(local, wkey)
    r2 = rmemo(local, wkey)
    check('routing map: miss equals direct', route_sig(r1, pad.global_x, pad.global_y, half, 0.05) == ref_r)
    check('routing map: hit equals direct', route_sig(r2, pad.global_x, pad.global_y, half, 0.05) == ref_r)
    check('two cached entries', len(pcb._tap_map_cache) == 2, str(len(pcb._tap_map_cache)))

    # In-place move of a foreign segment in the window: the key must change.
    seg = next((s for s in local.segments if s.net_id != net_id), None)
    if seg is not None:
        seg.start_x += 0.4
        seg.end_x += 0.4
        wkey2 = ppt._tap_window_key(local, pcb)
        check('moved segment changes the key', wkey2[0] != wkey[0])
        moved = vmemo(local, wkey2)
        fresh = build_via_obstacle_map(local, config, net_id, verbose=False,
                                       same_net_pad_clearance=-1.0)
        check('moved copper: memo equals direct',
              via_sig(moved, pad.global_x, pad.global_y, half, 0.05)
              == via_sig(fresh, pad.global_x, pad.global_y, half, 0.05))
        seg.start_x -= 0.4
        seg.end_x -= 0.4
    else:
        print('  skip moved-segment check: no foreign segment in window')

    n = len(pcb._tap_map_cache)
    vmemo(local, None)
    check('wkey None builds directly', len(pcb._tap_map_cache) == n)
    w2 = ppt.make_local_window(pcb, pad.global_x, pad.global_y, half)
    check('child window drops the memo', '_tap_map_cache' not in vars(w2))
    check('same window again: same key', ppt._tap_window_key(w2, pcb)[0] == wkey[0])

    print('FAIL' if FAILS else 'PASS', f'({len(FAILS)} failed)')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
