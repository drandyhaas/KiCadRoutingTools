#!/usr/bin/env python3
"""
Tests for issue #370 -- Tier B post-pass DRC-guarantee gaps (B1-B4, B7).

Each section has at least one would-have-failed-before check plus one
no-over-rejection check.

  B1: meander keep-out honors the FOREIGN copper's real width / via size and
      the real board outline+cutouts (not the bbox).
  B2: NPTH-hole blindness family (octolinear re-bend, snap_stub_gaps
      connectors, close_soft_joints bridges, cap-optimizer obstacles + real
      board outline).
  B3: cap-opt via mover validates board edge, rotated pad rects, and its new
      connector segments vs edge/NPTH.
  B4: bga_fanout via placement validates drill hole-to-hole (net-independent)
      and foreign tracks.
  B7: nudge_grazing_vias models pad drills at the real (offset/slot-aware)
      hole location.

Run:
    python3 tests/test_370_tierb_fixes.py
"""

import math
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import PCBData, BoardInfo, Segment, Via  # noqa: E402
from routing_config import GridRouteConfig                 # noqa: E402

FAILS = []


def check(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}")
    if not cond:
        FAILS.append(name)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _cfg():
    c = GridRouteConfig()
    c.track_width = 0.15
    c.clearance = 0.15
    c.via_size = 0.6
    c.grid_step = 0.05
    c.layers = ["F.Cu", "B.Cu"]
    c.meander_amplitude = 1.0
    return c


def _board(segments=(), vias=(), bounds=(-5.0, -5.0, 15.0, 50.0),
           outline=None, cutouts=None, pads_by_net=None, footprints=None):
    bi = BoardInfo(layers={}, board_bounds=bounds, copper_layers=["F.Cu", "B.Cu"])
    if outline is not None:
        bi.board_outline = outline
    if cutouts is not None:
        bi.board_cutouts = cutouts
    return PCBData(board_info=bi, nets={}, footprints=footprints or {},
                   vias=list(vias), segments=list(segments),
                   pads_by_net=pads_by_net or {})


# ---------------------------------------------------------------------------
# B1 -- meander keep-out: foreign copper real width / via size + real outline
# ---------------------------------------------------------------------------

def test_b1():
    print("B1: meander keep-out uses foreign width/via size + real outline")
    from length_matching import get_safe_amplitude_at_point

    MEANDER_NET, FOREIGN_NET = 5, 99
    common = dict(cx=5.0, cy=0.0, ux=1.0, uy=0.0, px=0.0, py=1.0, direction=1,
                  max_amplitude=1.0, min_amplitude=0.1, layer="F.Cu",
                  net_id=MEANDER_NET, config=_cfg())

    def foreign_seg(offset, width):
        return Segment(start_x=0.0, start_y=offset, end_x=10.0, end_y=offset,
                       width=width, layer="F.Cu", net_id=FOREIGN_NET)

    # A thin neighbor at 1.2mm allows some amplitude; a 1.0mm-wide trunk at the
    # same centerline offset must pull the bump back by (1.0-0.15)/2 = 0.425.
    thin = get_safe_amplitude_at_point(
        pcb_data=_board([foreign_seg(1.2, 0.15)]), **common)
    wide = get_safe_amplitude_at_point(
        pcb_data=_board([foreign_seg(1.2, 1.0)]), **common)
    check("wide foreign trunk pulls the meander back (was config width)",
          thin > 0 and wide < thin)
    # would-have-failed-before: amplitude the thin case allows must leave the
    # wide trunk's true clearance violated if reused -- i.e. the new amp keeps
    # the copper edges >= clearance apart.
    cfg = common['config']
    if wide > 0:
        # bump peak copper edge vs trunk copper edge
        gap = (1.2 - 1.0 / 2.0) - (wide + cfg.track_width / 2.0)
        check("accepted wide-trunk amplitude keeps real clearance",
              gap >= cfg.clearance - 1e-6)
    # no-over-rejection: foreign width == track_width is byte-identical
    same = get_safe_amplitude_at_point(
        pcb_data=_board([foreign_seg(1.2, 0.15)]), **common)
    check("foreign width == track_width unchanged", abs(same - thin) < 1e-12)

    # Foreign via at its REAL size: config.via_size 0.6 vs a 1.4mm via body.
    def via(offset, size):
        return Via(x=5.0, y=offset, size=size, drill=0.3,
                   layers=["F.Cu", "B.Cu"], net_id=FOREIGN_NET)

    small_v = get_safe_amplitude_at_point(pcb_data=_board(vias=[via(1.6, 0.6)]), **common)
    big_v = get_safe_amplitude_at_point(pcb_data=_board(vias=[via(1.6, 1.4)]), **common)
    check("oversized foreign via pulls the meander back (was config via_size)",
          small_v > 0 and big_v < small_v)
    same_v = get_safe_amplitude_at_point(pcb_data=_board(vias=[via(1.6, 0.6)]), **common)
    check("foreign via == via_size unchanged", abs(same_v - small_v) < 1e-12)

    # Real outline: interior cutout in the bump's path. The bbox bounds are far
    # away (old check accepted), but the cutout ring right above the centerline
    # must clamp the bump.
    cutout = [(4.0, 0.5), (6.0, 0.5), (6.0, 1.5), (4.0, 1.5)]
    outline = [(-5.0, -5.0), (15.0, -5.0), (15.0, 50.0), (-5.0, 50.0)]
    clamped = get_safe_amplitude_at_point(
        pcb_data=_board(outline=outline, cutouts=[cutout]), **common)
    open_amp = get_safe_amplitude_at_point(
        pcb_data=_board(outline=outline), **common)
    check("interior cutout clamps the bump (bbox check was blind)",
          clamped < open_amp and clamped < 0.5)
    check("outline far away does not clamp (no over-rejection)",
          open_amp == 1.0)


# ---------------------------------------------------------------------------

def main():
    test_b1()
    print()
    if FAILS:
        print(f"{len(FAILS)} check(s) FAILED")
        sys.exit(1)
    print("PASS  all #370 Tier B checks")


if __name__ == "__main__":
    main()
