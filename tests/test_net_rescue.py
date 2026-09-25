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
    """#1033: VICTIM is a POWER net (0.4 requested) whose only way across is
    a 0.5 mm long pinch at x 3.0..3.5 -- solid net-2 copper above and below it
    on both layers -- with free space either side. Its pads are 10 mm apart,
    so beyond the 2.5 mm neck zone at each pad there is ~3 mm of free middle
    where the full width fits; only a width below the power width fits the
    pinch."""
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(-5.0, -2.0, 7.0, 2.0))
    pads = [make_pad(VICTIM, -4.0, 0.0, ref='U1', num='1', net_name='VICTIM',
                     size_x=0.6, size_y=0.6),
            make_pad(VICTIM, 6.0, 0.0, ref='U2', num='1', net_name='VICTIM',
                     size_x=0.6, size_y=0.6)]
    walls = []
    for lay in ('F.Cu', 'B.Cu'):
        for sgn in (1, -1):
            y = 0.32
            while y < 2.0:
                walls.append(make_seg(3.0, sgn * y, 3.5, sgn * y, layer=lay,
                                      net_id=WALL, width=0.2))
                y += 0.18
    return make_pcb(
        nets={VICTIM: make_net(VICTIM, 'VICTIM'), WALL: make_net(WALL, 'WALL')},
        segments=walls, pads_by_net={VICTIM: pads, WALL: []},
        board_info=bi)


def _seg_dist(a, b):
    """Centreline distance between two segments (no crossings here)."""
    import math

    def pt(px, py, s):
        dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - s.start_x) * dx
                                                 + (py - s.start_y) * dy) / L2))
        return math.hypot(px - (s.start_x + u * dx), py - (s.start_y + u * dy))
    return min(pt(a.start_x, a.start_y, b), pt(a.end_x, a.end_y, b),
               pt(b.start_x, b.start_y, a), pt(b.end_x, b.end_y, a))


def _rescue_pinch(power_tap_neckdown=True):
    pcb = _pinch_board()
    cfg = _cfg()
    cfg.power_net_widths = {VICTIM: 0.4}
    cfg.power_tap_neckdown = power_tap_neckdown
    state = _state(pcb, cfg)
    summary = rescue_failed_nets(state, [('VICTIM', VICTIM)])
    assert summary is not None and summary['recovered'] == ['VICTIM'], summary
    num, _cp, _cpads = _net_component_info(pcb, VICTIM)
    assert num == 1, f"the rescued net must be connected ({num} parts)"
    return pcb, [s for s in pcb.segments if s.net_id == VICTIM]


def test_rescued_power_net_keeps_its_width_where_it_fits():
    """#1033: a rescued POWER net is laid at its requested width where that
    fits and narrower only where it must -- the main router's own neck-down,
    not the whole gap at the floor width."""
    import math
    pcb, segs = _rescue_pinch()
    wide = [s for s in segs if abs(s.width - 0.4) < 1e-9]
    wide_len = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                   for s in wide)
    assert wide_len >= 2.0, f"the free middle must carry the full 0.4: {wide_len:.2f} mm"
    assert all(-1.6 < min(s.start_x, s.end_x) and max(s.start_x, s.end_x) < 2.6
               for s in wide), "full width only in the free middle"
    pinch = [s.width for s in segs
             if min(s.start_x, s.end_x) < 3.4 and max(s.start_x, s.end_x) > 3.1]
    assert pinch and max(pinch) < 0.2, f"the pinch must stay narrow: {pinch}"
    # every piece clears the walls at the clearance the rescue routed at
    clr = pcb.board_info.min_clearance_used
    walls = [s for s in pcb.segments if s.net_id == WALL]
    bad = [(s.width, round(min(_seg_dist(s, w) for w in walls
                               if w.layer == s.layer), 4))
           for s in segs
           if min(_seg_dist(s, w) - (s.width + w.width) / 2.0
                  for w in walls if w.layer == s.layer) < clr - 1e-4]
    assert not bad, f"rescued copper inside {clr} of the walls: {bad}"


def test_rescued_power_net_without_neckdown_routes_at_the_floor():
    """Change detector: with neck-down off a blocked wide attempt has no
    fallback, so the rescue routes the net at the floor width, as before."""
    _pcb, segs = _rescue_pinch(power_tap_neckdown=False)
    assert segs and max(s.width for s in segs) < 0.2, [s.width for s in segs]


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
