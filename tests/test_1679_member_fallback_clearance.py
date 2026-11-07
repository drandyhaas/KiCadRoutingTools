#!/usr/bin/env python3
"""#1679: a pair's single-ended member fallback keeps its clearance to pads.

The diff engine's maps bake each layer's impedance width into their track and
via stamps and give a wide member no margin; pads stamp the call's nominal
width. A member routed on such a map at a layer wider than the call's width
comes (layer width - track width) / 2 inside a neighbouring pad's clearance,
and nothing warned. The fallback now routes in the single-ended engine, where
the margin prices every obstacle class and a route can neck down.

Rows:
  - routed: the fallback itself, on the maps the diff engine builds, for a
    pair whose run along a foreign pad is on a layer about twice the call's
    width. No pad is inside its clearance and both members connect;
  - the engine: a wide layer gets the nominal reserve, a real track margin and
    its own map, and the run's lift records come back after the pass;
  - no layer wider than the call's: the run's own config and map, untouched.

    python3 tests/test_1679_member_fallback_clearance.py
"""
import contextlib
import io
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from check_drc import check_pad_segment_overlap  # noqa: E402
from diff_pair_custody import (_member_connected, _single_ended_engine,  # noqa: E402
                               run_single_ended_member_fallback)
from kicad_parser import BoardInfo  # noqa: E402
from obstacle_map import build_base_obstacle_map  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from routing_utils import build_layer_map  # noqa: E402
from synth import make_net, make_pad, make_pcb  # noqa: E402

FAILS = []
P, N, GND = 1, 2, 3
CLEARANCE, TRACK, WIDE = 0.1, 0.15, 0.34


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def _board():
    """On one layer: a GND pad as a wall across P's straight line, so P bends
    over it, and the run along the wall's edge touches neither terminal and no
    terminal neck reaches it. N passes clear underneath."""
    bi = BoardInfo(layers={0: 'F.Cu'}, copper_layers=['F.Cu'],
                   board_bounds=(0.0, 0.0, 12.0, 8.0))
    pads = {
        P: [make_pad(P, 2.0, 4.0, ref='U1', num='1', net_name='/D_P', size_x=0.3, size_y=0.3),
            make_pad(P, 10.0, 4.0, ref='J1', num='1', net_name='/D_P', size_x=0.3, size_y=0.3)],
        N: [make_pad(N, 2.0, 6.4, ref='U1', num='2', net_name='/D_N', size_x=0.3, size_y=0.3),
            make_pad(N, 10.0, 6.4, ref='J1', num='2', net_name='/D_N', size_x=0.3, size_y=0.3)],
        GND: [make_pad(GND, 6.0, 3.8, ref='R1', num='1', net_name='GND',
                       size_x=2.0, size_y=2.6)],
    }
    return make_pcb(nets={P: make_net(P, '/D_P'), N: make_net(N, '/D_N'),
                          GND: make_net(GND, 'GND')},
                    segments=[], pads_by_net=pads, board_info=bi)


def test_routed_member_keeps_clear_of_the_pad():
    cfg = GridRouteConfig(layers=['F.Cu'], clearance=CLEARANCE, track_width=TRACK,
                          grid_step=0.05, via_size=0.5, via_drill=0.3,
                          layer_widths={'F.Cu': WIDE}, reserve_layer_widths=True)
    pcb = _board()
    log = io.StringIO()
    with contextlib.redirect_stdout(log):
        state = SimpleNamespace(
            pcb_data=pcb, config=cfg, routed_results={}, results=[], routed_net_ids=[],
            remaining_net_ids=[P, N], all_unrouted_net_ids=[P, N], gnd_net_id=None,
            track_proximity_cache={}, layer_map=build_layer_map(cfg.layers),
            ripped_route_layer_costs={}, ripped_route_via_positions={},
            routed_net_paths={}, pair_diagnostics={},
            base_obstacles=build_base_obstacle_map(pcb, cfg, [P, N]))
        pair = SimpleNamespace(p_net_id=P, n_net_id=N, p_net_name='/D_P', n_net_name='/D_N')
        summary = run_single_ended_member_fallback(state, [('/D', pair)], [P, N], None)
    check('routed: both members went through the fallback',
          sorted(summary['routed']) == ['/D_N', '/D_P'], f"{summary} {log.getvalue()[-300:]}")
    check('routed: both members connect',
          _member_connected(pcb, P) and _member_connected(pcb, N))
    segs = [s for r in state.results for s in (r.get('new_segments') or [])
            if s.net_id in (P, N)]
    check('routed: the members have copper', bool(segs))
    gnd = pcb.pads_by_net[GND][0]
    grazes = [(round(s.width, 3), s.layer, round(s.start_y, 2)) for s in segs
              if check_pad_segment_overlap(gnd, s, CLEARANCE, ['F.Cu'], 0.0)[0]]
    check('routed: no member track inside the GND pad\'s clearance', not grazes, str(grazes))
    check('routed: the run still lays the layer width where it fits',
          any(abs(s.width - WIDE) < 1e-6 for s in segs),
          str(sorted({round(s.width, 3) for s in segs})))


def _state(layer_widths):
    cfg = GridRouteConfig(layers=['F.Cu'], clearance=CLEARANCE, track_width=TRACK,
                          grid_step=0.05, layer_widths=layer_widths,
                          reserve_layer_widths=True)
    pcb = _board()
    return SimpleNamespace(config=cfg, pcb_data=pcb, base_obstacles=object())


def test_wide_layer_takes_the_single_ended_engine():
    state = _state({'F.Cu': WIDE})
    state.pcb_data._graphic_own_pad_lift = 'the run\'s record'
    with contextlib.redirect_stdout(io.StringIO()):
        se_cfg, base, records = _single_ended_engine(state, [P, N], None)
    check('engine: a wide layer does not keep the baked reserve',
          se_cfg.route_reserve_width('F.Cu') == TRACK,
          str(se_cfg.route_reserve_width('F.Cu')))
    check('engine: the width rides a real margin there',
          se_cfg.track_margins_for_net(P)[0] >= (WIDE - TRACK) / 2 / 0.05,
          str(se_cfg.track_margins_for_net(P)))
    check('engine: on the baked config the same net has none',
          state.config.track_margins_for_net(P)[0] == 0.0)
    check('engine: its own map', base is not state.base_obstacles)
    check('engine: the run\'s config is left as it was',
          state.config.reserve_layer_widths is True)
    check('engine: the run\'s lift record is handed back to restore',
          (records or {}).get('_graphic_own_pad_lift') == 'the run\'s record')


def test_no_wide_layer_keeps_the_run_map():
    for label, widths in (('no layer widths', None),
                          ('a narrower layer', {'F.Cu': 0.12})):
        state = _state(widths)
        cfg, base, records = _single_ended_engine(state, [P, N], None)
        check(f'{label}: the run\'s config and map, no rebuild',
              cfg is state.config and base is state.base_obstacles and records is None)


def main():
    test_routed_member_keeps_clear_of_the_pad()
    test_wide_layer_takes_the_single_ended_engine()
    test_no_wide_layer_keeps_the_run_map()
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
