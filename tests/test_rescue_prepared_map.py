"""The single-ended rescue rungs route on a map prepared for the net.

The stub-swap rescue (the last rung of the failure ladder, on by default) and
the tap-relocation rescue routed on the working map as the previous net's
restore left it: every soft cost cleared, no same-net hole-to-hole rings (a
stub swap's new pad via included), no own-pad lift, no free vias -- and the
net's OWN cached copper back on the map as an obstacle. Rescues that should
have succeeded failed on their own copper. They now route through
single_ended_loop._route_on_prepared_map, the main pass's prepare/restore
bracket.

Row: on a fanned board, ten two-pad nets all route through the helper, while
the same nets on the restored map (the old rescue path) lose some -- measured
6 of 10 when this was written.

    python3 tests/test_rescue_prepared_map.py
"""

import contextlib
import io
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from run_utils import evidence  # noqa: E402

# Gitignored, built from a tracked root on a fresh clone (test_457).
BOARD_NAME = 'fanout_output1.kicad_pcb'


def test_rescue_routes_on_the_prepared_map():
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    from obstacle_map import build_base_obstacle_map
    from obstacle_cache import precompute_all_net_obstacles, build_working_obstacle_map
    from single_ended_routing import route_net_with_obstacles
    from net_queries import get_all_unrouted_net_ids
    import single_ended_loop as sel
    log = io.StringIO()
    with contextlib.redirect_stdout(log):
        from fixture_boards import ensure
        pcb = parse_kicad_pcb(evidence(ensure(BOARD_NAME)))
    layers = pcb.board_info.copper_layers
    cfg = GridRouteConfig(layers=layers, track_width=0.1, clearance=0.1,
                          via_size=0.3, via_drill=0.2, grid_step=0.1)
    unrouted = sorted(set(get_all_unrouted_net_ids(pcb)))
    nets = [n for n in unrouted if len(pcb.pads_by_net.get(n, [])) == 2][:10]
    assert len(nets) == 10, len(nets)
    with contextlib.redirect_stdout(log):
        base = build_base_obstacle_map(pcb, cfg, nets)
        cache = precompute_all_net_obstacles(pcb, nets, cfg)
        work = build_working_obstacle_map(base, cache)
    state = types.SimpleNamespace(
        working_obstacles=work, all_unrouted_net_ids=unrouted,
        net_obstacles_cache=cache, ripped_route_layer_costs={},
        ripped_route_via_positions={})
    layer_map = {name: i for i, name in enumerate(layers)}

    def ok(r):
        return bool(r) and not r.get('failed')
    prepared = restored = 0
    for nid in nets:
        with contextlib.redirect_stdout(log):
            prepared += ok(sel._route_on_prepared_map(
                pcb, nid, cfg, state, [], {}, layer_map))
            restored += ok(route_net_with_obstacles(pcb, nid, cfg, work))
            for clear in ('clear_free_vias', 'clear_source_target_cells',
                          'clear_endpoint_exempt', 'clear_allowed_cells'):
                getattr(work, clear)()
    print(f"    prepared map {prepared}/10, restored map {restored}/10")
    assert prepared == 10, prepared
    assert restored < prepared, (restored, "the control no longer discriminates")


TESTS = [test_rescue_routes_on_the_prepared_map]


if __name__ == '__main__':
    fails = 0
    for t in TESTS:
        try:
            t()
            print(f"  PASS {t.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"  FAIL {t.__name__}: {e}")
    print('ALL PASS' if not fails else f'{fails} FAILED')
    sys.exit(1 if fails else 0)
