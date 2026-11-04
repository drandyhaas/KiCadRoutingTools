#!/usr/bin/env python3
"""#1226: copper stamped outside the per-net cache reaches every via rung, at
that rung's OWN via.

A net with its own via size (#530, config.net_via_sizes) searches at its own
via-legality rung, and `is_via_blocked_rung` reads that rung alone -- never
rung 0's dynamic map. Before #1226:

  * the per-net adds behind routing_context.build_single_ended_obstacles
    stamped rung 0 only, so routed copper was via-LEGAL at the net's rung
    (anyradio_cm5 / berkeley_obsidian: vias on foreign tracks after the
    casualty reconcile);
  * the list adds (phase-3 tap copper) copied RUN-size cells into the rungs,
    leaving a larger per-net via (rung - run)/2 short of every edge;
  * the board-edge bands reached a rung only at the run's via;
  * the per-net adds never reached the #568 small rung, which the cache
    entries populate.

The rows below build real maps on a synthetic 4-layer board. The A* row
routes a per-net net whose only cheap via sites sit on a foreign In1.Cu
copper band, and runs it twice: with the rung stamping, and with it switched
off -- the second must ship the short, or this test cannot tell the two
apart.

Needs grid_router 0.22.0+ (SKIPs otherwise).
"""
import math
import os
import sys
import tempfile
from dataclasses import replace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))

FAILS = []
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
RUN_VIA = (0.3, 0.15)
NET_VIA = (0.6, 0.3)        # net B's own via: LARGER than the run's
CLR = 0.15
BAND = (14.0, 26.0)         # net A's In1.Cu copper band (y range)


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def _board(td):
    import constraint_agreement as ca
    path = os.path.join(td, 'r.kicad_pcb')
    # net A: a solid In1.Cu band of tracks (0.2 wide, 0.5 pitch) -- no via of
    # either size fits between them. net B: F.Cu pad -> B.Cu pad across it.
    segs = [(1.0, y / 10, 39.0, y / 10, 0.2, 'In1.Cu', 1)
            for y in range(int(BAND[0] * 10), int(BAND[1] * 10) + 1, 5)]
    # net D is never routed, so its pads stay in the BASE -- as on any real
    # board, whose base copper populates every per-net rung. (A board whose
    # every net is routed leaves the rungs empty, and an empty rung falls
    # back to rung 0: the defect then reads as a smaller one.)
    ca.NETS[4] = 'D'
    try:
        ca.write_board(path, segments=segs, footprints=[
        {'ref': 'U1', 'x': 2, 'y': 2, 'net_id': 1, 'net_name': 'A', 'layer': 'F.Cu'},
        {'ref': 'U2', 'x': 38, 'y': 2, 'net_id': 1, 'net_name': 'A', 'layer': 'F.Cu'},
        {'ref': 'U3', 'x': 5, 'y': 20, 'net_id': 2, 'net_name': 'B', 'layer': 'F.Cu'},
        {'ref': 'U4', 'x': 35, 'y': 20, 'net_id': 2, 'net_name': 'B', 'layer': 'B.Cu'},
        {'ref': 'U5', 'x': 20, 'y': 36, 'net_id': 3, 'net_name': 'C', 'layer': 'F.Cu'},
        {'ref': 'U6', 'x': 30, 'y': 36, 'net_id': 3, 'net_name': 'C', 'layer': 'F.Cu'},
        {'ref': 'U7', 'x': 20, 'y': 4, 'net_id': 4, 'net_name': 'D', 'layer': 'F.Cu'},
        {'ref': 'U8', 'x': 30, 'y': 4, 'net_id': 4, 'net_name': 'D', 'layer': 'F.Cu'}])
    finally:
        del ca.NETS[4]
    text = open(path, encoding='utf-8').read().replace(
        '(layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))',
        '(layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) '
        '(31 "B.Cu" signal) (44 "Edge.Cuts" user))')
    open(path, 'w', encoding='utf-8').write(text)
    return path


def _cfg(net_via_sizes=None, via=RUN_VIA):
    from routing_config import GridRouteConfig
    return GridRouteConfig(layers=LAYERS, track_width=0.12, clearance=CLR, grid_step=0.1,
                           via_size=via[0], via_drill=via[1],
                           net_via_sizes=dict(net_via_sizes or {}))


def _single_ended_map(pcb, cfg, cache=None):
    """The map build_single_ended_obstacles hands net B's search: A routed,
    C still unrouted (from its cache entry when given)."""
    from obstacle_map import build_base_obstacle_map, build_layer_map
    from routing_context import build_single_ended_obstacles
    base = build_base_obstacle_map(pcb, cfg, [1, 2, 3], static_base=True)
    obs, _ = build_single_ended_obstacles(
        base, pcb, cfg, routed_net_ids=[1], remaining_net_ids=[2, 3],
        all_unrouted_net_ids=[2, 3], net_id=2, gnd_net_id=None,
        track_proximity_cache={}, layer_map=build_layer_map(cfg.layers),
        net_obstacles_cache=cache)
    return base, obs


def _cells(pcb, step=0.1):
    x0, y0, x1, y1 = pcb.board_info.board_bounds
    for gx in range(int(x0 / step), int(x1 / step) + 1):
        for gy in range(int(y0 / step), int(y1 / step) + 1):
            yield gx, gy


def row_map_matches_reference(pcb):
    """Rung r of the rebuilt map == rung 0 of the SAME builder at rung r's via
    (same nets, same copper, only the via differs), cell for cell over the
    board -- edge band included. Before #1226 the routed band was missing."""
    from obstacle_cache import rung_for_net, via_rungs
    cfg = _cfg({2: NET_VIA})
    r = rung_for_net(cfg, pcb, 2)
    check('net B searches at a per-net rung', r >= 1 and via_rungs(cfg, pcb)[r - 1] == NET_VIA,
          f'rung {r}, rungs {via_rungs(cfg, pcb)}')
    _, obs = _single_ended_map(pcb, cfg)
    _, ref = _single_ended_map(pcb, _cfg(via=NET_VIA))
    under = over = 0
    for gx, gy in _cells(pcb):
        m, t = obs.is_via_blocked_rung(gx, gy, r), ref.is_via_blocked(gx, gy)
        under += t and not m
        over += m and not t
    check('rung map == reference at the net\'s own via (no under-block)', under == 0, f'{under} cells')
    check('rung map == reference at the net\'s own via (no over-block)', over == 0, f'{over} cells')
    # the issue's own probe: a via ON net A's copper
    from routing_config import GridCoord
    gx, gy = GridCoord(0.1).to_grid(20.0, 20.0)
    check('a via on net A\'s In1.Cu track is blocked at net B\'s rung',
          obs.is_via_blocked_rung(gx, gy, r))


def row_small_rung(pcb):
    """With the #568 small rung armed and POPULATED (C's cache entry stamps
    it), every raw via cell rung 0 blocks is blocked there too."""
    from obstacle_cache import rung_small_armed, precompute_net_obstacles, _small_via_pair
    cfg = _cfg({2: NET_VIA})
    if not (rung_small_armed() and _small_via_pair(cfg, pcb)):
        print('  (small rung not armed on this board: row skipped)')
        return
    cache = {3: precompute_net_obstacles(pcb, 3, cfg)}
    _, obs = _single_ended_map(pcb, cfg, cache=cache)
    check('small rung is populated by the cache entry', obs.rung_len(1) > 0)
    miss = sum(1 for gx, gy in _cells(pcb)
               if obs.is_via_blocked(gx, gy) and not obs.is_via_blocked_rung(gx, gy, 1))
    check('small rung blocks every via cell rung 0 blocks', miss == 0, f'{miss} cells')


def row_list_twins(pcb):
    """add/remove_*_list_as_obstacles: rung r at its OWN via while added, and
    every rung back to its count after the remove."""
    from obstacle_map import (build_base_obstacle_map, add_segments_list_as_obstacles,
                              remove_segments_list_from_obstacles, add_vias_list_as_obstacles,
                              remove_vias_list_from_obstacles)
    from obstacle_cache import rung_for_net
    from kicad_parser import Segment, Via
    from routing_config import GridCoord
    cfg = _cfg({2: NET_VIA})
    r = rung_for_net(cfg, pcb, 2)
    w = build_base_obstacle_map(pcb, cfg, [1, 2, 3], static_base=True).clone_fresh()
    before = ([w.rung_len(i) for i in range(w.rung_count())], tuple(w.get_stats()))
    seg = Segment(start_x=5, start_y=32, end_x=35, end_y=32, width=0.12, layer='B.Cu', net_id=3)
    via = Via(x=10, y=8, size=0.4, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=3)
    add_segments_list_as_obstacles(w, [seg], cfg)
    add_vias_list_as_obstacles(w, [via], cfg, diagonal_margin=0.25)
    c = GridCoord(0.1)
    # a 0.6 via needs 0.3 + 0.06 + 0.15 = 0.51 mm from the track centre; a
    # 0.3 via only 0.36 -- the old run-size mirror left 0.4 legal
    gx, gy = c.to_grid(20.0, 32.4)
    check('list add: the rung blocks a cell only its own via reaches', w.is_via_blocked_rung(gx, gy, r)
          and not w.is_via_blocked(gx, gy))
    # via-via: 0.2 + 0.3 + 0.15 = 0.65 from the via centre at the rung's via
    gx, gy = c.to_grid(10.0, 8.6)
    check('list add: via-via disc at the rung\'s via', w.is_via_blocked_rung(gx, gy, r)
          and not w.is_via_blocked(gx, gy))
    remove_segments_list_from_obstacles(w, [seg], cfg)
    remove_vias_list_from_obstacles(w, [via], cfg, diagonal_margin=0.25)
    after = ([w.rung_len(i) for i in range(w.rung_count())], tuple(w.get_stats()))
    check('list remove: every rung and every count back to before', after == before,
          f'{before} -> {after}' if after != before else '')


def row_layout():
    """_per_net_rung_geometry recovers the rung layout from (config, map)."""
    import grid_router
    from obstacle_map import _per_net_rung_geometry

    def mk(n):
        m = grid_router.GridObstacleMap(2)
        if n:
            m.add_blocked_via_rung(n, 0, 0)
        return m
    cfg = _cfg({7: (0.45, 0.2), 8: (0.6, 0.3)})
    check('layout A: no small slot, per-net rungs 1..n largest first',
          _per_net_rung_geometry(mk(2), cfg) == [(1, 0.6, 0.3), (2, 0.45, 0.2)],
          str(_per_net_rung_geometry(mk(2), cfg)))
    check('layout B: small slot at 1, per-net rungs 2..n',
          _per_net_rung_geometry(mk(3), cfg) == [(2, 0.6, 0.3), (3, 0.45, 0.2)],
          str(_per_net_rung_geometry(mk(3), cfg)))
    big = _cfg({7: (1.0, 0.5), 8: (0.45, 0.2)}, via=(0.8, 0.4))
    check('layout ambiguous (a per-net via below the run may BE the small pair): '
          'each rung at the largest it could hold',
          _per_net_rung_geometry(mk(2), big) == [(1, 1.0, 0.5), (2, 1.0, 0.5)],
          str(_per_net_rung_geometry(mk(2), big)))
    check('a count that fits no layout: every rung at the largest',
          _per_net_rung_geometry(mk(5), cfg) == [(r, 0.6, 0.3) for r in range(1, 6)],
          str(_per_net_rung_geometry(mk(5), cfg)))
    check('a single-rung map has no per-net rungs', _per_net_rung_geometry(mk(0), cfg) == [])


def row_rung_retry(pcb):
    """The #568 rung-1 retry from a net's own-via clone ships the pair rung 1
    was STAMPED at (the run's small fab via), and only when it is smaller
    than the net's own; the path vias it registers are judged at the net's
    own rung."""
    import grid_router
    from obstacle_cache import _small_via_pair, rung_small_armed
    from single_ended_routing import (_per_net_via_config, _rung_search_pair,
                                      _register_rung_path_vias)
    run = _cfg({2: NET_VIA}, via=(0.4, 0.2))
    small = _small_via_pair(run, pcb)
    if not (rung_small_armed() and small):
        print('  (small rung not armed on this board: retry rows skipped)')
    else:
        own = _per_net_via_config(run, pcb, 2)
        check('the per-net clone records the run via', own.run_via == (0.4, 0.2), str(own.run_via))
        check('retry pair = the run\'s small pair, not one below the net\'s via',
              _rung_search_pair(own, pcb) == small, f'{_rung_search_pair(own, pcb)} vs {small}')
        tiny = _per_net_via_config(_cfg({2: (small[0], small[1])}, via=(0.4, 0.2)), pcb, 2)
        check('no retry when the small pair is not smaller than the net\'s via',
              _rung_search_pair(tiny, pcb) is None)
        check('a run-via net keeps the old rule',
              _rung_search_pair(run, pcb) == small)
    m = grid_router.GridObstacleMap(2)
    m.add_blocked_via_rung(2, 5, 5)            # blocked for the net's own via only
    path = [(5, 5, 0), (5, 5, 1), (6, 5, 1)]

    class _P:
        pass
    p0, p2 = _P(), _P()
    _register_rung_path_vias(p0, m, path, 0.25, 0.15, net_id=2)
    _register_rung_path_vias(p2, m, path, 0.25, 0.15, own_rung=2, net_id=2)
    check('registration at rung 0 misses a cell only the net\'s own via is blocked on',
          (2, 5, 5) not in p0._unblock_via_sizes)
    check('registration at the net\'s own rung ships the small via there',
          p2._unblock_via_sizes.get((2, 5, 5)) == (0.25, 0.15))


def _band_shorts(result):
    """B's new vias closer to net A's In1.Cu band than via/2 + 0.1 + clearance."""
    bad = []
    for v in (result or {}).get('new_vias', []):
        ys = [y / 10 for y in range(int(BAND[0] * 10), int(BAND[1] * 10) + 1, 5)]
        d = min(abs(v.y - y) for y in ys) if 1.0 <= v.x <= 39.0 else 99
        if d < v.size / 2 + 0.1 + CLR - 1e-6:
            bad.append((round(v.x, 2), round(v.y, 2), v.size, round(d, 3)))
    return bad


def row_astar(pcb):
    """The A* itself: net B (own via 0.6) must change layers between pads that
    sit inside net A's In1.Cu band. Correct maps force the via out of the band;
    with the rung stamping switched off the via lands ON net A's copper."""
    import obstacle_map
    from single_ended_routing import route_net_with_obstacles
    cfg = _cfg({2: NET_VIA})
    _, obs = _single_ended_map(pcb, cfg)
    res = route_net_with_obstacles(pcb, 2, cfg, obs)
    vias = (res or {}).get('new_vias', [])
    check('fixed: net B routes with a via', bool(vias))
    check('fixed: the via is net B\'s own size', all(abs(v.size - NET_VIA[0]) < 1e-9 for v in vias),
          str([v.size for v in vias]))
    check('fixed: no via within clearance of net A\'s In1.Cu band', not _band_shorts(res),
          str(_band_shorts(res)))
    saved = obstacle_map._stamp_rungs_at_own_via
    obstacle_map._stamp_rungs_at_own_via = lambda *a, **k: None
    try:
        _, obs0 = _single_ended_map(pcb, cfg)
        res0 = route_net_with_obstacles(pcb, 2, cfg, obs0)
    finally:
        obstacle_map._stamp_rungs_at_own_via = saved
    check('negative control (rung stamping off): the via lands on net A\'s copper',
          bool(_band_shorts(res0)), str(_band_shorts(res0)))


def main():
    try:
        import grid_router
        if not hasattr(grid_router.GridObstacleMap, 'add_blocked_vias_rung_batch'):
            print('SKIP: grid_router without the via-rung API (needs 0.22.0+)')
            return 0
    except ImportError:
        print('SKIP: grid_router not importable')
        return 0
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(_board(td))
        check('fixture is 4-layer', list(pcb.board_info.copper_layers) == LAYERS,
              str(pcb.board_info.copper_layers))
        print('-- the rebuilt map vs the reference at the net\'s own via')
        row_map_matches_reference(pcb)
        print('-- the #568 small rung')
        row_small_rung(pcb)
        print('-- list add/remove twins')
        row_list_twins(pcb)
        print('-- rung layout')
        row_layout()
        print('-- rung-1 retry')
        row_rung_retry(pcb)
        print('-- the A* on the rebuilt map')
        row_astar(pcb)
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
