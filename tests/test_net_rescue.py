"""Tests for the per-net fine-parameter rescue pass (issues #331/#371).

Geometry used throughout: a 2-layer board with two full-width net-2 "wall"
tracks on BOTH layers at y = +/-W, leaving a straight channel along y=0 that
net 1 must cross. With wall half-width 0.1 and walls at +/-0.32 the channel
admits a centerline track only when (0.1 + clearance + track/2) <= 0.32:

  nominal  (clearance 0.15, track 0.15):  0.325 > 0.32  -> main run FAILS
  rescue   (clearance <=0.1565, track 0.127 fab floor): fits -> rescue routes

so the rescue must succeed via its neck-down rungs, not rung 0. Walls at
+/-0.24 shrink the channel below even the fab-floor need (0.29) - nothing can
route, and the board must come back untouched (the no-rip-up guarantee).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

import env_knobs

from types import SimpleNamespace

from kicad_parser import BoardInfo
from routing_config import GridRouteConfig
from synth import make_net, make_pad, make_pcb, make_seg

import net_rescue
from net_rescue import (_attempt_edge, _choose_grid, _net_component_info,
                        rescue_failed_nets)

VICTIM, WALL = 1, 2


def _board(wall_y=0.32):
    """Two victim pads separated by net-2 walls on both layers."""
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(-1.0, -1.0, 4.0, 1.0))
    pads = [make_pad(VICTIM, -0.4, 0.0, ref='U1', num='1', net_name='VICTIM',
                     size_x=0.3, size_y=0.3),
            make_pad(VICTIM, 3.4, 0.0, ref='U2', num='1', net_name='VICTIM',
                     size_x=0.3, size_y=0.3)]
    walls = [make_seg(-1.0, y, 4.0, y, layer=lay, net_id=WALL, width=0.2)
             for y in (wall_y, -wall_y) for lay in ('F.Cu', 'B.Cu')]
    return make_pcb(
        nets={VICTIM: make_net(VICTIM, 'VICTIM'), WALL: make_net(WALL, 'WALL')},
        segments=walls, pads_by_net={VICTIM: pads, WALL: []},
        board_info=bi)


def _cfg():
    c = GridRouteConfig()
    c.layers = ['F.Cu', 'B.Cu']
    c.grid_step = 0.05
    c.clearance = 0.15
    c.track_width = 0.15
    c.via_size = 0.5
    c.via_drill = 0.3
    return c


def _gap(pcb):
    p1, p2 = pcb.pads_by_net[VICTIM]
    import math
    d = math.hypot(p2.global_x - p1.global_x, p2.global_y - p1.global_y)
    return (d, p1.global_x, p1.global_y, p2.global_x, p2.global_y)


def test_component_info_counts_pad_components():
    pcb = _board()
    num, comp_points, comp_pads = _net_component_info(pcb, VICTIM)
    assert num == 2, f"two isolated pads must be two components, got {num}"
    # Attach a stub to pad 1: still two components, stub tip a join point.
    pcb.segments.append(make_seg(-0.4, 0.0, 0.4, 0.0, net_id=VICTIM, width=0.15))
    num, comp_points, comp_pads = _net_component_info(pcb, VICTIM)
    assert num == 2
    stub_comp = [cid for cid, pads in comp_pads.items()
                 if any(p.global_x < 0 for p in pads)][0]
    assert (0.4, 0.0) in comp_points[stub_comp], \
        "stub endpoint must be a candidate join point"


def test_choose_grid_respects_cell_budget():
    cfg = _cfg()
    import routing_defaults as d
    assert _choose_grid(cfg, 6.0) == d.RESCUE_GRID_STEP
    # #516 (529152c) deleted the 40mm gap cap: the CELL BUDGET is the real
    # bound, so a huge window may come out COARSER than the run's own
    # grid_step -- the grid doubles until the window fits the budget. (The
    # old assertion `big <= cfg.grid_step` tested the deleted cap.)
    big = _choose_grid(cfg, 1000.0)
    assert (2 * 1000.0 / big) ** 2 <= d.RESCUE_MAX_WINDOW_CELLS
    assert big >= min(cfg.grid_step, d.RESCUE_GRID_STEP)


def test_rescue_necks_down_through_the_pinch():
    pcb = _board(wall_y=0.32)
    cfg = _cfg()
    result, used = _attempt_edge(pcb, VICTIM, _gap(pcb), cfg, None)
    assert result is not None, "rescue must route the 0.32 channel"
    assert used.clearance < cfg.clearance, \
        "the 0.32 channel is only legal below nominal clearance"
    assert used.track_width < cfg.track_width
    assert result['new_segments'], "no copper returned"
    assert all(abs(s.start_y) < 0.25 and abs(s.end_y) < 0.25
               for s in result['new_segments']), "route must stay in the channel"
    # _attempt_edge itself must not commit anything.
    assert all(s.net_id != VICTIM for s in pcb.segments)


def test_impossible_gap_leaves_board_untouched():
    pcb = _board(wall_y=0.24)  # channel below even the fab-floor need
    segs_before = list(pcb.segments)
    vias_before = list(pcb.vias)
    result, used = _attempt_edge(pcb, VICTIM, _gap(pcb), _cfg(), None)
    assert result is None and used is None
    assert pcb.segments == segs_before and pcb.vias == vias_before, \
        "a failed rescue must leave the board untouched"


def _state(pcb, cfg):
    return SimpleNamespace(pcb_data=pcb, config=cfg, routed_results={},
                           results=[], remaining_net_ids=[VICTIM],
                           routed_net_ids=[], net_history={}, route_index=0)


def test_rescue_failed_nets_end_to_end():
    pcb = _board(wall_y=0.32)
    cfg = _cfg()
    state = _state(pcb, cfg)
    summary = rescue_failed_nets(state, [('VICTIM', VICTIM)])
    assert summary is not None and summary['recovered'] == ['VICTIM']
    assert VICTIM in state.routed_results
    assert state.routed_results[VICTIM].get('is_rescue')
    assert not state.routed_results[VICTIM].get('failed_pads_info')
    assert state.results and state.results[0]['new_segments']
    assert any(s.net_id == VICTIM for s in pcb.segments), \
        "rescued copper must be committed to the board"
    assert VICTIM not in state.remaining_net_ids
    num, _, _ = _net_component_info(pcb, VICTIM)
    assert num == 1, "net must grade fully connected after the rescue"
    # The below-nominal clearance must reach the ledger for DRC grading.
    assert getattr(pcb.board_info, 'min_clearance_used', None) is not None
    assert pcb.board_info.min_clearance_used < cfg.clearance


def test_rescue_env_kill_switch():
    pcb = _board(wall_y=0.32)
    state = _state(pcb, _cfg())
    os.environ['KICAD_NET_RESCUE'] = '0'
    env_knobs.refresh()
    try:
        assert rescue_failed_nets(state, [('VICTIM', VICTIM)]) is None
        assert not state.results and VICTIM not in state.routed_results
        assert all(s.net_id != VICTIM for s in pcb.segments)
    finally:
        os.environ.pop('KICAD_NET_RESCUE', None)
        env_knobs.refresh()


def test_rescue_skips_connected_and_reports_unchanged():
    # Impossible channel: candidate attempted, nothing changes, honest report.
    pcb = _board(wall_y=0.24)
    state = _state(pcb, _cfg())
    summary = rescue_failed_nets(state, [('VICTIM', VICTIM)])
    assert summary is not None and summary['unchanged'] == ['VICTIM']
    assert VICTIM not in state.routed_results
    assert all(s.net_id != VICTIM for s in pcb.segments)


def _pinch_board():
    """#1033 part 3b: VICTIM is a POWER net (0.4 requested) whose only way
    across is a 1 mm long pinch at x 1..2 -- solid net-2 copper above and
    below it on both layers -- with free space on either side. Only a rung
    below the power width fits the pinch."""
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(-1.5, -1.5, 4.5, 1.5))
    pads = [make_pad(VICTIM, -0.8, 0.0, ref='U1', num='1', net_name='VICTIM',
                     size_x=0.6, size_y=0.6),
            make_pad(VICTIM, 3.8, 0.0, ref='U2', num='1', net_name='VICTIM',
                     size_x=0.6, size_y=0.6)]
    walls = []
    for lay in ('F.Cu', 'B.Cu'):
        for sgn in (1, -1):
            y = 0.32
            while y < 1.5:
                walls.append(make_seg(1.0, sgn * y, 2.0, sgn * y, layer=lay,
                                      net_id=WALL, width=0.2))
                y += 0.18
    return make_pcb(
        nets={VICTIM: make_net(VICTIM, 'VICTIM'), WALL: make_net(WALL, 'WALL')},
        segments=walls, pads_by_net={VICTIM: pads, WALL: []},
        board_info=bi)


def test_rescued_power_net_is_widened_where_it_fits():
    import math
    from power_widen import ExactWideCheck
    pcb = _pinch_board()
    cfg = _cfg()
    cfg.power_net_widths = {VICTIM: 0.4}
    state = _state(pcb, cfg)
    summary = rescue_failed_nets(state, [('VICTIM', VICTIM)])
    assert summary is not None and summary['recovered'] == ['VICTIM'], summary
    # The rescue itself lays its rung width -- no widening inside the
    # routing loop (completion first, #1033 part 3 moved it out).
    assert max(s.width for s in pcb.segments if s.net_id == VICTIM) < 0.2,         "the rescue must not widen in the loop any more"
    # The SHARED post-route cleanup pipeline (both fronts) widens it, judged
    # against the finished board at the run's clearance.
    from cleanup_pipeline import run_post_route_cleanup
    out = run_post_route_cleanup(state.results, pcb, {VICTIM}, cfg,
                                 snap=False, phantom=False, graze=False,
                                 octolinear=False, via_nudge=False,
                                 cycles=False, neck=False, smooth=False)
    assert out.counts.get('power_widened_nets') == 1, out.counts
    segs = [s for s in pcb.segments if s.net_id == VICTIM]

    def length(pred_x, pred_w):
        # sampled along each segment, so a long unsplit segment spanning the
        # pinch still counts its free-space part
        tot = 0.0
        for s in segs:
            L = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
            for k in range(40):
                x = s.start_x + (s.end_x - s.start_x) * (k + 0.5) / 40
                if pred_x(x) and pred_w(s.width):
                    tot += L / 40
        return tot

    def _free(x):
        return (-0.4 < x < 0.85) or (2.15 < x < 3.4)
    outside = length(_free, lambda w: True)
    outside_wide = length(_free, lambda w: w > 0.2)
    pinch = [s.width for s in segs
             if min(s.start_x, s.end_x) < 1.9 and max(s.start_x, s.end_x) > 1.1]
    assert outside > 0.5 and outside_wide >= 0.5 * outside, \
        f"free space either side of the pinch must be widened: " \
        f"{outside_wide:.2f} of {outside:.2f} mm wide"
    assert pinch and max(pinch) < 0.2, f"the pinch must stay narrow: {pinch}"
    # every widened piece clears at the ORIGINAL clearance, exactly
    chk = ExactWideCheck(pcb, cfg, VICTIM)
    bad = [s for s in segs if s.width > 0.2 and not chk.clears(
        s.start_x, s.start_y, s.end_x, s.end_y, s.layer, s.width)]
    assert not bad, f"widened copper must clear at 0.15: {bad}"
    # still one connected net
    num, _cp, _cpads = _net_component_info(pcb, VICTIM)
    assert num == 1, f"widening must not break connectivity ({num} parts)"
    # the state's result carries the widened copper (it is what ships)
    rs = state.routed_results[VICTIM]['new_segments']
    assert any(s.width > 0.2 for s in rs)


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        print(f"  {fn.__name__} ...", end=" ")
        fn()
        print("OK")
    print(f"{len(fns)} test(s) passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
