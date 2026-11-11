"""Partner-pad capsule of a hybrid leg encloses the pad's corner copper exactly (0.5 mm pitch QFN pads)."""
import math
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))
import diff_pair_routing as dpr  # noqa: E402


def pad(shape, sx, sy, rratio=0.0):
    return SimpleNamespace(global_x=0.0, global_y=0.0, size_x=sx, size_y=sy, shape=shape,
                           roundrect_rratio=rratio, layers=['F.Cu'], drill=0, net_id=1)


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


if __name__ == '__main__':
    test_sharp_rect_keeps_sqrt2()
    test_oval_and_circle_are_exact()
    test_roundrect_encloses_its_corner_arc_only()
    test_full_rratio_roundrect_is_a_stadium()
    print('PASS')
