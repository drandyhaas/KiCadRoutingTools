"""The rescue's fine grid must not turn a wide net into a giant A* margin
(found reviewing #1282).

A rescue rung routes a power net as a track MARGIN over the width its map is
stamped at, and the router's swept-capsule test scans (2*ceil(r)+1)^2 cells
per neighbour for a margin of r cells. The margin scales with 1/grid, so at
the 0.025 mm rescue grid splitflap's +12V (2.5 mm over a 0.3 mm stamp) had a
63-cell margin: one rung's search ran 149k steps in 340 s and a run under
--escalation off went 8 hours without finishing. The same gap with the map
stamped at 2.5 mm (margin 0) took 40k steps in 0.3 s.

_rescue_rungs now coarsens a rung's grid until the net's margin fits
routing_defaults.RESCUE_MAX_MARGIN_CELLS, never past the run's own grid.

  * a 2.5 mm power net: every rung's margin within the bound, on a grid no
    coarser than the run's, under both --escalation fab and off (FAILS on
    bb5943a0: rung 0 at 0.025 mm with a 63-cell margin);
  * controls -- a plain net and a 1.0 mm power net (watchy's rescue, 20-25
    cells): every rung on the 0.025 mm grid exactly as before;
  * a rung whose margin is 0 (the net's own width stamped, as rung 1 is under
    off) keeps the fine grid even when its sibling rung 0 is coarsened.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import BoardInfo  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from synth import make_net, make_pad, make_pcb  # noqa: E402

import fab_tiers  # noqa: E402
import routing_defaults as defaults  # noqa: E402
from net_rescue import _rescue_rungs  # noqa: E402

NET = 1
FINE = 0.025
RUN_GRID = 0.1
# The bound the fix introduces; bb5943a0 has no such constant, and the test
# must fail there on the MARGIN, not on a missing name.
BOUND = getattr(defaults, 'RESCUE_MAX_MARGIN_CELLS', 30)


def _pcb():
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(-5.0, -5.0, 15.0, 5.0))
    pads = [make_pad(NET, 0.0, 0.0, ref='U1', num='1', net_name='P',
                     size_x=0.6, size_y=0.6),
            make_pad(NET, 10.0, 0.0, ref='U2', num='1', net_name='P',
                     size_x=0.6, size_y=0.6)]
    return make_pcb(nets={NET: make_net(NET, 'P')}, segments=[],
                    pads_by_net={NET: pads}, board_info=bi)


def _cfg(power_w=None):
    c = GridRouteConfig()
    c.layers = ['F.Cu', 'B.Cu']
    c.grid_step = RUN_GRID
    c.clearance = 0.25
    c.track_width = 0.3
    if power_w is not None:
        c.power_net_widths = {NET: power_w}
    return c


def _rungs(power_w, policy):
    prev = fab_tiers.get_escalation_policy()
    fab_tiers.set_escalation_policy(policy)
    try:
        return _rescue_rungs(_cfg(power_w), FINE, _pcb(), NET)
    finally:
        fab_tiers.set_escalation_policy(*prev)


def _margin(cfg):
    return max(cfg.track_margins_for_net(NET))


def test_wide_power_net_rungs_are_bounded():
    for policy in ('fab', 'off'):
        rungs = _rungs(2.5, policy)
        assert rungs, f"{policy}: no rungs built"
        for i, r in enumerate(rungs):
            m = _margin(r)
            assert m <= BOUND + 1e-9, (
                f"--escalation {policy}: rung {i} searches a 2.5 mm net on a "
                f"{r.grid_step:g} mm grid with a {m:.1f}-cell track margin "
                f"(> {BOUND}): {(2 * int(-(-m // 1)) + 1) ** 2} cells per "
                f"neighbour test")
            assert r.grid_step <= RUN_GRID + 1e-9, (
                f"--escalation {policy}: rung {i} coarsened past the run's "
                f"own grid ({r.grid_step:g} > {RUN_GRID:g})")


def test_margin_free_rung_keeps_the_fine_grid():
    # Under off, rung 1 is stamped at the net's own width (track_width =
    # 2.5): margin 0, so it stays on the fine grid while rung 0 coarsens.
    rungs = _rungs(2.5, 'off')
    free = [r for r in rungs if _margin(r) <= 1e-9]
    assert free, [(_margin(r), r.grid_step) for r in rungs]
    assert all(abs(r.grid_step - FINE) < 1e-12 for r in free), \
        [(_margin(r), r.grid_step) for r in free]


def test_controls_keep_the_fine_grid():
    # A plain net, and a 1.0 mm power net (watchy VBUS's default-policy rescue,
    # 20-25 cells at 0.025): nothing under the bound moves.
    for power_w in (None, 1.0):
        for policy in ('fab', 'off'):
            rungs = _rungs(power_w, policy)
            assert rungs
            bad = [(i, r.grid_step, round(_margin(r), 2))
                   for i, r in enumerate(rungs)
                   if abs(r.grid_step - FINE) > 1e-12]
            assert not bad, (f"power width {power_w}, --escalation {policy}: "
                             f"rungs left the {FINE} grid: {bad}")


TESTS = [test_wide_power_net_rungs_are_bounded,
         test_margin_free_rung_keeps_the_fine_grid,
         test_controls_keep_the_fine_grid]

if __name__ == '__main__':
    for t in TESTS:
        t()
        print(f'ok {t.__name__}')
    print('ALL PASS')
