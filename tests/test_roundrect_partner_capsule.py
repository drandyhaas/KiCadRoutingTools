"""Partner-pad capsule of a hybrid leg encloses the pad's corner copper exactly (0.5 mm pitch QFN pads)."""
import math
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))
import diff_pair_routing as dpr  # noqa: E402


def pad(shape, sx, sy, rratio=0.0, polygons=None):
    return SimpleNamespace(global_x=0.0, global_y=0.0, size_x=sx, size_y=sy, shape=shape,
                           roundrect_rratio=rratio, layers=['F.Cu'], drill=0, net_id=1,
                           polygons=polygons)


def width(p):
    (seg,) = dpr._pad_obstacle_segments(p, ['F.Cu', 'B.Cu'])
    return seg.width


def test_sharp_rect_keeps_sqrt2():
    assert math.isclose(width(pad('rect', 0.85, 0.3)), 0.3 * math.sqrt(2.0))


def test_oval_and_circle_are_exact():
    assert math.isclose(width(pad('oval', 0.85, 0.3)), 0.3)
    assert math.isclose(width(pad('circle', 0.5, 0.5)), 0.5)


def test_roundrect_encloses_its_corner_arc_only():
    # 0.5 mm pitch QFN pad: 0.30 short, rratio 0.25 -> corner radius 0.075
    w = width(pad('roundrect', 0.85, 0.3, 0.25))
    r = 0.075
    assert math.isclose(w / 2.0, math.sqrt(2.0) * (0.15 - r) + r)
    assert 0.3 < w < 0.3 * math.sqrt(2.0)
    # the farthest corner copper point lies on the capsule boundary, none outside
    half = 0.85 / 2 - 0.15
    cx, cy = 0.85 / 2 - r, 0.15 - r
    far = math.hypot(cx - half, cy) + r
    assert far <= w / 2.0 + 1e-12


def test_full_rratio_roundrect_is_a_stadium():
    assert math.isclose(width(pad('roundrect', 0.85, 0.3, 0.5)), 0.3)


def _spine_reach(w, poly, sx, sy):
    half = max(sx, sy) / 2 - min(sx, sy) / 2
    return max(dpr._pt_seg_dist(x, y, -half, 0.0, half, 0.0) for x, y in poly)


def test_shallow_chamfer_is_enclosed():
    # A chamfered rect stays shape 'roundrect' (its rratio on the uncut corners,
    # its outline in pad.polygons, #1450). A shallow chamfer (ratio 0.1) reaches
    # past the 0.25-rratio corner arc, so the capsule must come off the outline.
    from kicad_parser import chamfered_rect_polygon
    sx, sy, rr = 0.85, 0.3, 0.25
    poly = chamfered_rect_polygon(0.0, 0.0, sx, sy, 0.0, ['top_right'], 0.1, rr * sy)
    w = width(pad('roundrect', sx, sy, rr, [poly]))
    reach = _spine_reach(w, poly, sx, sy)
    assert reach > math.sqrt(2.0) * (0.15 - rr * sy) + rr * sy   # the formula would miss it
    assert reach <= w / 2.0 and w / 2.0 <= reach + 0.0011


def test_deep_chamfer_is_tighter_than_sharp():
    from kicad_parser import chamfered_rect_polygon
    sx, sy = 0.85, 0.3
    poly = chamfered_rect_polygon(0.0, 0.0, sx, sy, 0.0,
                                  ['top_left', 'top_right', 'bottom_right', 'bottom_left'],
                                  0.25, 0.0)
    w = width(pad('roundrect', sx, sy, 0.0, [poly]))
    assert _spine_reach(w, poly, sx, sy) <= w / 2.0 < 0.3 * math.sqrt(2.0) / 2.0


if __name__ == '__main__':
    test_shallow_chamfer_is_enclosed()
    test_deep_chamfer_is_tighter_than_sharp()
    test_sharp_rect_keeps_sqrt2()
    test_oval_and_circle_are_exact()
    test_roundrect_encloses_its_corner_arc_only()
    test_full_rratio_roundrect_is_a_stadium()
    print('PASS')
