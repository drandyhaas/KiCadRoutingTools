#!/usr/bin/env python3
"""#980: what `oracle_reconnect` DOES with the net-class map it is handed --
with no KiCad at all.

tests/test_980_oracle_class_map.py pins that every caller passes the map;
this pins what the oracle does with it, on a real (tiny) board file. The
link source and the refills are stubbed: `kicad_exact_fill.exact_unconnected`
reports one missing link on net A, then none; `kicad_unconnected` and the
island refill report nothing. A spy on `plane_region_connector.
build_base_obstacles` -- the obstacle map every weld link routes against --
records the config it is handed. What each case pins:

* the map is RE-KEYED to the parsed board's own ids, by name (the ids the
  caller had are deliberately different);
* the link's routing floor is its own net's class, so `obstacle_clearance`
  of every foreign net is the KiCad pair value;
* the caller's config is not mutated (a private copy);
* with no map the oracle is unchanged: no copy, no floor.

    python3 tests/test_980_oracle_behaviour.py
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', os.path.join('tests', 'oracle'), 'rust_router'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from constraint_agreement import write_board  # noqa: E402


def _run(by_name):
    import kicad_oracle
    import kicad_exact_fill
    import plane_region_connector
    from routing_config import GridRouteConfig
    seen = []
    calls = {'n': 0}

    def fake_exact(board_file, net_names=None, pcb_data=None,
                   verbose=False, project_from=None):
        calls['n'] += 1
        if calls['n'] > 1:
            return []
        return [('A', (10.0, 10.0, 'F.Cu', 'track'),
                 (13.0, 10.0, 'F.Cu', 'track'))]

    real_bbo = plane_region_connector.build_base_obstacles

    def spy(*a, **k):
        cfg = k.get('config')
        seen.append({'cfg': cfg, 'nc': dict(cfg.net_clearances or {}),
                     'floor': cfg.net_clearance_floor,
                     'b_obstacle': None})
        return real_bbo(*a, **k)

    saved = (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
             kicad_exact_fill.refill_islands,
             plane_region_connector.build_base_obstacles)
    kicad_exact_fill.exact_unconnected = fake_exact
    kicad_oracle.kicad_unconnected = lambda *a, **k: None
    kicad_exact_fill.refill_islands = lambda *a, **k: None
    plane_region_connector.build_base_obstacles = spy
    try:
        with tempfile.TemporaryDirectory() as td:
            board = os.path.join(td, 'b.kicad_pcb')
            # net A broken in two on F.Cu, net B running alongside
            write_board(board, segments=[
                (5, 10, 10, 10, 0.2, 'F.Cu', 1),
                (13, 10, 18, 10, 0.2, 'F.Cu', 1),
                (5, 12, 18, 12, 0.2, 'F.Cu', 2)])
            # board_edge_clearance at the board's own 0.2: the oracle's
            # edge-rule `replace(config)` does not fire, so the only copy is
            # the one #980 makes
            caller = GridRouteConfig(clearance=0.2, track_width=0.2,
                                     via_size=0.6, via_drill=0.3,
                                     layers=['F.Cu', 'B.Cu'],
                                     board_edge_clearance=0.2)
            # the caller's ids are NOT the board's: 41/42 vs 1/2
            caller.net_clearances = {41: 0.3, 42: 0.35}
            kicad_oracle.oracle_reconnect(
                board, ['A'], caller, track_via_clearance=0.2,
                hole_to_hole_clearance=0.2, max_rounds=1,
                net_clearances_by_name=by_name)
            from kicad_parser import parse_kicad_pcb
            ids = {n.name: nid for nid, n in parse_kicad_pcb(board).nets.items()}
    finally:
        (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
         kicad_exact_fill.refill_islands,
         plane_region_connector.build_base_obstacles) = saved
    return seen, caller, ids


def test_the_map_is_rekeyed_and_the_floor_is_the_links_own_class():
    seen, caller, ids = _run({'A': 0.3, 'B': 0.35})
    assert seen, 'the oracle never built an obstacle map for the link'
    first = seen[0]
    assert first['nc'] == {ids['A']: 0.3, ids['B']: 0.35}, (first['nc'], ids)
    assert abs(first['floor'] - 0.3) < 1e-12, first['floor']
    cfg = first['cfg']
    assert abs(cfg.obstacle_clearance(ids['B']) - 0.35) < 1e-12
    assert abs(cfg.obstacle_clearance(ids['C']) - 0.3) < 1e-12
    # the caller's config is untouched
    assert cfg is not caller
    assert caller.net_clearances == {41: 0.3, 42: 0.35}
    assert caller.net_clearance_floor is None
    print(f"  PASS: the link's map is {first['nc']} (re-keyed by name), its "
          f"floor A's 0.3; the caller's config is unchanged")


def test_no_map_changes_nothing():
    seen, caller, _ids = _run(None)
    assert seen
    assert seen[0]['cfg'] is caller
    assert seen[0]['nc'] == {41: 0.3, 42: 0.35}
    assert seen[0]['floor'] is None
    print("  PASS: with no map the oracle routes on the caller's config, "
          "untouched")


TESTS = [test_the_map_is_rekeyed_and_the_floor_is_the_links_own_class,
         test_no_map_changes_nothing]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print('ALL PASS')
