#!/usr/bin/env python3
"""
Regression test for issue #175: length-matching meander clearance must be sized
from the net's ACTUAL routed width (impedance-controlled --impedance layer widths
or a power_net_widths override), not the base config.track_width.

A net routed wider than track_width that then gets length-/time-matched had its
meander arms under-reserved by ~(net_width - track_width)/2, so the meander could
graze neighbors at the routed clearance.

Covers:
  * Single-ended keep-out (get_safe_amplitude_at_point): a wide power net pulls its
    meander back relative to a base-width net, by exactly (net_w - track_width)/2.
  * The common case (net_w == track_width) is byte-identical (no behavior change).
  * Diff-pair keep-out (get_safe_amplitude_for_diff_pair): an impedance-widened
    pair pulls its centerline meander back too.
  * Board-edge keep-out: a meander bump near the board outline is clamped so its
    copper stays edge_clearance inside the bounds (the meander pass used to ignore
    the outline, unlike the A* router, so bumps printed off the edge).

Run with:  python3 tests/test_meander_net_width.py
"""
import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, "rust_router"))

from kicad_parser import PCBData, Segment, BoardInfo          # noqa: E402
from routing_config import GridRouteConfig                    # noqa: E402
from length_matching import (                                 # noqa: E402
    get_safe_amplitude_at_point,
    get_safe_amplitude_for_diff_pair,
)

MEANDER_NET = 5
FOREIGN_NET = 99


def _cfg(power_w=None, layer_w=None):
    c = GridRouteConfig()
    c.track_width = 0.15
    c.clearance = 0.15
    c.via_size = 0.6
    c.grid_step = 0.05
    c.layers = ["F.Cu", "B.Cu"]
    c.meander_amplitude = 1.0
    if power_w is not None:
        c.power_net_widths = {MEANDER_NET: power_w}
    if layer_w is not None:
        c.layer_widths = {"F.Cu": layer_w}
    return c


def _pcb_with_neighbor(offset, net_id=FOREIGN_NET):
    """A single foreign track parallel to the meander centerline, offset in +y."""
    seg = Segment(start_x=0.0, start_y=offset, end_x=10.0, end_y=offset,
                  width=0.15, layer="F.Cu", net_id=net_id)
    bi = BoardInfo(layers={}, board_bounds=None, copper_layers=["F.Cu", "B.Cu"])
    return PCBData(board_info=bi, nets={}, footprints={}, vias=[],
                   segments=[seg], pads_by_net={})


# Meander centerline sits at (5, 0) running +x; perpendicular +y aims at the neighbor.
_COMMON = dict(cx=5.0, cy=0.0, ux=1.0, uy=0.0, px=0.0, py=1.0, direction=1,
               max_amplitude=1.0, min_amplitude=0.1, layer="F.Cu")


def _pcb_no_obstacles(bounds=None):
    """Empty board (no copper) with optional board_bounds for edge tests."""
    bi = BoardInfo(layers={}, board_bounds=bounds, copper_layers=["F.Cu", "B.Cu"])
    return PCBData(board_info=bi, nets={}, footprints={}, vias=[],
                   segments=[], pads_by_net={})


def _fail(msg):
    print("FAIL  " + msg)
    sys.exit(1)


def test_board_edge_keepout():
    # No copper anywhere, but the board outline is close on the +y side: the meander
    # must not print past it. Centerline at (5,0), bump goes +y; a 0.4mm-wide net's
    # copper edge (centerline + net_half 0.2) must stay edge_clearance inside max_y.
    # edge_clearance falls back to config.clearance (0.15). So the bump peak must be
    # <= max_y - (0.15 + 0.2). With max_y=0.7 -> peak <= 0.35.
    max_y = 0.7
    limit = max_y - (0.15 + 0.4 / 2)   # clearance + net_half
    near = get_safe_amplitude_at_point(pcb_data=_pcb_no_obstacles(bounds=(-5.0, -5.0, 15.0, max_y)),
                                       net_id=MEANDER_NET, config=_cfg(power_w=0.4), **_COMMON)
    if not (0 < near <= limit + 1e-9):
        _fail(f"board-edge: amplitude {near} should be clamped to <={limit:.3f} (edge at y={max_y})")
    # With no board_bounds the edge check is skipped -> full amplitude available.
    unbounded = get_safe_amplitude_at_point(pcb_data=_pcb_no_obstacles(bounds=None),
                                            net_id=MEANDER_NET, config=_cfg(power_w=0.4), **_COMMON)
    if unbounded <= near:
        _fail(f"board-edge: unbounded amp {unbounded} should exceed edge-limited {near}")
    # A far edge must not constrain a modest bump at all.
    far = get_safe_amplitude_at_point(pcb_data=_pcb_no_obstacles(bounds=(-5.0, -5.0, 15.0, 50.0)),
                                      net_id=MEANDER_NET, config=_cfg(power_w=0.4), **_COMMON)
    if abs(far - unbounded) > 1e-9:
        _fail(f"board-edge: far edge should not clamp ({far} vs {unbounded})")


def test_single_ended_widths():
    offset = 1.2  # loose enough that both widths can meander, so amps are comparable
    base = get_safe_amplitude_at_point(pcb_data=_pcb_with_neighbor(offset),
                                       net_id=MEANDER_NET, config=_cfg(), **_COMMON)
    wide_power = get_safe_amplitude_at_point(pcb_data=_pcb_with_neighbor(offset),
                                             net_id=MEANDER_NET,
                                             config=_cfg(power_w=0.5), **_COMMON)
    wide_imp = get_safe_amplitude_at_point(pcb_data=_pcb_with_neighbor(offset),
                                           net_id=MEANDER_NET,
                                           config=_cfg(layer_w=0.5), **_COMMON)

    if not (base > 0 and wide_power > 0):
        _fail(f"expected both to meander at offset {offset} (base={base}, wide={wide_power})")
    # The wide net must pull its arm back: its own half-width contribution grew from
    # track_width/2 to net_w/2 (plus the corner-bloat that scales with it). The exact
    # value is quantized by the binary search's geometric amplitude ladder, so assert
    # the ordering, not a precise delta.
    if not (wide_power < base):
        _fail(f"wide power net should pull meander back (base={base}, wide={wide_power})")
    if abs(wide_imp - wide_power) > 1e-9:
        _fail(f"impedance layer width should match power width keep-out "
              f"({wide_imp:.4f} vs {wide_power:.4f})")

    # A wide net must also REFUSE a gap that only a base-width arm fits.
    tight = 0.8
    base_tight = get_safe_amplitude_at_point(pcb_data=_pcb_with_neighbor(tight),
                                             net_id=MEANDER_NET, config=_cfg(), **_COMMON)
    wide_tight = get_safe_amplitude_at_point(pcb_data=_pcb_with_neighbor(tight),
                                             net_id=MEANDER_NET,
                                             config=_cfg(power_w=0.5), **_COMMON)
    if not (base_tight > 0 and wide_tight == 0):
        _fail(f"tight gap: base should fit ({base_tight}) and wide should not ({wide_tight})")


def test_own_width_override():
    # A segment widened beyond its netclass (no impedance/power config) must still
    # pull its meander back, driven purely by own_width (mirrors obstacle_cache's
    # max(net_w, seg.width)). This is the width the meander arms actually carry.
    offset = 1.2
    base = get_safe_amplitude_at_point(pcb_data=_pcb_with_neighbor(offset),
                                       net_id=MEANDER_NET, config=_cfg(),
                                       own_width=0.15, **_COMMON)
    wide = get_safe_amplitude_at_point(pcb_data=_pcb_with_neighbor(offset),
                                       net_id=MEANDER_NET, config=_cfg(),
                                       own_width=0.5, **_COMMON)
    if not (base > 0 and wide > 0):
        _fail(f"own_width: expected both to meander (base={base}, wide={wide})")
    if not (wide < base):
        _fail(f"own_width=0.5 should pull meander back vs 0.15 (base={base}, wide={wide})")
    # own_width == track_width must be identical to not passing it at all.
    default = get_safe_amplitude_at_point(pcb_data=_pcb_with_neighbor(offset),
                                          net_id=MEANDER_NET, config=_cfg(), **_COMMON)
    if abs(base - default) > 1e-9:
        _fail(f"own_width==track_width not identical to default ({base} vs {default})")


def test_common_case_unchanged():
    # net_w == track_width everywhere -> keep-out must be identical to a config with
    # no impedance/power overrides at all. This guards the "byte-identical" promise.
    for offset in (0.6, 0.8, 1.0, 1.2, 1.5):
        plain = get_safe_amplitude_at_point(pcb_data=_pcb_with_neighbor(offset),
                                            net_id=MEANDER_NET, config=_cfg(), **_COMMON)
        # power width equal to track_width, and layer width equal to track_width:
        eq_power = get_safe_amplitude_at_point(pcb_data=_pcb_with_neighbor(offset),
                                               net_id=MEANDER_NET,
                                               config=_cfg(power_w=0.15), **_COMMON)
        eq_layer = get_safe_amplitude_at_point(pcb_data=_pcb_with_neighbor(offset),
                                               net_id=MEANDER_NET,
                                               config=_cfg(layer_w=0.15), **_COMMON)
        if abs(plain - eq_power) > 1e-9 or abs(plain - eq_layer) > 1e-9:
            _fail(f"common case not byte-identical at offset {offset}: "
                  f"plain={plain}, eq_power={eq_power}, eq_layer={eq_layer}")


def test_diff_pair_widths():
    # Diff-pair centerline meander: get_safe_amplitude_for_diff_pair takes an int
    # layer index and P/N net ids. A wider pair must pull its meander back too.
    p_net, n_net = 5, 6
    spacing = 0.2  # P/N offset from centerline
    offset = 1.2   # close enough that the neighbor constrains the meander amplitude

    def cfg_pair(width):
        c = _cfg()
        if width != c.track_width:
            c.power_net_widths = {p_net: width, n_net: width}
        return c

    common = dict(cx=5.0, cy=0.0, ux=1.0, uy=0.0, px=0.0, py=1.0, direction=1,
                  max_amplitude=1.0, min_amplitude=0.1, layer=0,
                  p_net_id=p_net, n_net_id=n_net, spacing_mm=spacing)

    base = get_safe_amplitude_for_diff_pair(pcb_data=_pcb_with_neighbor(offset),
                                            config=cfg_pair(0.15), **common)
    wide = get_safe_amplitude_for_diff_pair(pcb_data=_pcb_with_neighbor(offset),
                                            config=cfg_pair(0.5), **common)
    if not (base > 0 and wide > 0):
        _fail(f"diff pair: expected both to meander (base={base}, wide={wide})")
    if not (wide < base):
        _fail(f"diff-pair wide net should pull meander back (base={base}, wide={wide})")


if __name__ == "__main__":
    test_single_ended_widths()
    test_own_width_override()
    test_common_case_unchanged()
    test_diff_pair_widths()
    test_board_edge_keepout()
    print("PASS  meander keep-out sized from actual net width (#175)")
