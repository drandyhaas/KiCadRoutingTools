#!/usr/bin/env python3
"""Custom-shape pads must be modelled by their REAL polygon copper, not their
symmetric bounding box, in the fanout obstacle paths (#232 phantom class).

A custom pad's pad.size_x/size_y is the bbox taken symmetrically about the pad
ORIGIN. For an off-centre outline (a meander antenna whose feed origin sits well
below its copper) that bbox balloons over empty board and phantom-blocks via
placement metres from any copper -- on mikoto the AE1 antenna's Ø20mm bbox disk
reached U1 8mm away and forced its whole inner ring into long grazing stubs
(should have been clean via-in-pad drops). The main obstacle map (_add_pad_obstacle)
and check_drc already model the polygon; these are the fanout-side gaps.

Fixture: a 2x2mm square of copper at (10,20) on a pad whose ORIGIN is (10,10),
so pad.size_y ~ 24mm (symmetric bbox) covers the empty origin region. A via at
the origin is 9mm from any real copper -- it must be treated as clear.
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _custom_pad():
    # Real copper: a 2x2 square centred at (10, 20). Origin at (10, 10).
    poly = [(9.0, 19.0), (11.0, 19.0), (11.0, 21.0), (9.0, 21.0)]
    return SimpleNamespace(
        global_x=10.0, global_y=10.0,
        size_x=4.0, size_y=24.0,          # symmetric bbox about the origin -> phantom
        shape='custom', polygons=[poly],
        net_id=7, layers=['F.Cu'], drill=0.0,
        pad_type='smd', rect_rotation=0.0, component_ref='AE1',
    )


ORIGIN = (10.0, 10.0)      # on the pad origin, 9mm from real copper (was phantom-blocked)
ON_COPPER = (10.0, 20.0)   # inside the real polygon


def test_seg_hits_pad_uses_polygon():
    """qfn_fanout via_clears / stub-graze path (bga_fanout.reroute._seg_hits_pad)."""
    from bga_fanout.reroute import _seg_hits_pad
    pad = _custom_pad()
    # A via (zero-length segment) at the origin is 9mm from copper: NOT a hit,
    # even with a fat 0.2mm clearance margin. The bbox model would falsely hit.
    assert not _seg_hits_pad(*ORIGIN, *ORIGIN, pad, margin=0.2), \
        "origin 9mm from real copper must not hit the custom pad (bbox phantom)"
    # On the real copper it must still hit.
    assert _seg_hits_pad(*ON_COPPER, *ON_COPPER, pad, margin=0.0), \
        "a point inside the real polygon must hit"
    print("PASS _seg_hits_pad polygon-aware for custom pads")


def test_pad_extent_bounding_circle():
    """BGA underpad disk stamps use the real-polygon bounding circle."""
    from bga_fanout.underpad import _pad_extent
    cx, cy, r = _pad_extent(_custom_pad())
    assert abs(cx - 10.0) < 1e-6 and abs(cy - 20.0) < 1e-6, (cx, cy)
    assert abs(r - 1.4142) < 0.01, r                 # half-diagonal of the 2x2 square
    assert (cy - r) > 18.0, "the keep disk must not reach down toward the origin"
    print("PASS _pad_extent uses the real polygon bounding circle")


def test_occ_block_poly_and_via_clears():
    """BGA occ stamp + exact via-site test both polygon-aware."""
    from bga_fanout.underpad import _Occ, _custom_polys
    from bga_fanout.geometry import via_clears_pad_rects
    pad = _custom_pad()
    occ = _Occ((0.0, 0.0, 20.0, 30.0), 0.05, ['F.Cu', 'B.Cu'])
    occ.block_poly(_custom_polys(pad), 0.15, None)

    def blocked(x, y):
        ix, iy = occ.cell(x, y)
        return bool(occ.blocked(0, ix, iy))

    assert not blocked(*ORIGIN), "origin cell must be free (was bbox-phantom-blocked)"
    assert blocked(*ON_COPPER), "real-copper cell must be blocked"
    # Exact via-site test.
    assert via_clears_pad_rects(*ORIGIN, 0.125, 0.1, [pad]), \
        "a via at the origin clears the (far) real copper"
    assert not via_clears_pad_rects(*ON_COPPER, 0.125, 0.1, [pad]), \
        "a via on the real copper does not clear"
    print("PASS _Occ.block_poly + via_clears_pad_rects polygon-aware")


def test_normal_pad_unchanged():
    """A rect pad (no polygons) keeps the exact bbox disk behaviour."""
    from bga_fanout.underpad import _pad_extent
    rect = SimpleNamespace(global_x=3.0, global_y=4.0, size_x=1.0, size_y=2.0,
                           shape='rect', polygons=None)
    cx, cy, r = _pad_extent(rect)
    assert (cx, cy, r) == (3.0, 4.0, 1.0), (cx, cy, r)
    print("PASS non-custom pad extent unchanged")


if __name__ == '__main__':
    test_seg_hits_pad_uses_polygon()
    test_pad_extent_bounding_circle()
    test_occ_block_poly_and_via_clears()
    test_normal_pad_unchanged()
    print("\nAll custom-pad phantom tests passed.")
