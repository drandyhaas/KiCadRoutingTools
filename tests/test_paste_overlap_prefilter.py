#!/usr/bin/env python3
"""`paste_apertures._pad_overlaps` tests a sample's exact distance only where
the sample can matter, and answers as it did before.

A point can be in a paste opening only inside the opening's bounds, and within
reach of a pad only inside the pad's bounds grown by that reach; both are
supersets, so testing them first changes no answer. Without them a mounting
hole's paste-only pad -- a custom polygon of ~1000 edges -- cost 38 s per
footprint in the via-in-paste index (32.7 M point-segment distances), once
for every route step's board load.

  1. the verdict equals the unfiltered test's on overlapping, touching,
     disjoint, inside-only and stroked cases;
  2. a pad far from a 1000-edge opening (but inside the footprint) makes no
     exact-distance call against it.

    python3 tests/test_paste_overlap_prefilter.py
"""
import math
import os
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path[:0] = [os.path.join(REPO, 'py_router'), TESTS]

from synth import make_pad  # noqa: E402
import paste_apertures as pa  # noqa: E402
from check_drc import _pad_perimeter_points, point_to_pad_distance  # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def unfiltered(ap, pad):
    """The test as it was before the bounds checks."""
    if pa.aperture_distance(pad.global_x, pad.global_y, ap) <= 1e-9:
        return True
    n = max(8, int(max(pad.size_x, pad.size_y) / 0.05) + 1)
    for (px, py) in _pad_perimeter_points(pad, n):
        if pa.aperture_distance(px, py, ap) <= 1e-9:
            return True
    reach = ap.width / 2.0 if ap.shape_pad is None else 0.0
    for (vx, vy) in pa._boundary_samples(ap):
        if point_to_pad_distance(vx, vy, pad) <= reach + 1e-9:
            return True
    return False


def disc(cx, cy, r, n):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n))
            for k in range(n)]


def poly_aperture(poly):
    sp = make_pad(net_id=0, x=0.0, y=0.0, shape='custom', size_x=3.6, size_y=3.6,
                  layers=('F.Paste',), polygons=[poly])
    return pa.PasteAperture(owner_ref='H1', layer='F.Paste', source='paste_only_pad',
                            shape_pad=sp, bounds=pa._pad_bounds(sp))


def ring_aperture(pts, width):
    return pa.PasteAperture(owner_ref='U1', layer='F.Paste', source='graphic', rings=[pts],
                            closed=True, filled=False, width=width,
                            bounds=pa._ring_bounds([pts], None, width))


def main():
    big = poly_aperture(disc(0.0, 0.0, 1.6, 1000))
    cases = {
        'a pad across the opening': (big, make_pad(1, 1.5, 0.0, size_x=0.6, size_y=0.6)),
        'a pad touching its edge': (big, make_pad(1, 1.9, 0.0, size_x=0.6, size_y=0.6)),
        'a pad clear of it': (big, make_pad(1, 2.5, 0.0, size_x=0.6, size_y=0.6)),
        'an opening inside a big pad': (big, make_pad(1, 0.0, 0.0, size_x=6.0, size_y=6.0)),
        'a stroked ring over a pad': (ring_aperture(disc(5.0, 5.0, 1.0, 64), 0.3),
                                      make_pad(1, 6.1, 5.0, size_x=0.4, size_y=0.4)),
        'a stroked ring clear of a pad': (ring_aperture(disc(5.0, 5.0, 1.0, 64), 0.3),
                                          make_pad(1, 6.6, 5.0, size_x=0.4, size_y=0.4)),
    }
    print('1. the verdict is unchanged')
    for label, (ap, pad) in cases.items():
        a, b = pa._pad_overlaps(ap, pad), unfiltered(ap, pad)
        check(f'{label}: {a}', a == b, f'filtered {a}, unfiltered {b}')

    print('2. no exact distance where a sample cannot matter')
    calls = {'n': 0}
    real = pa.aperture_distance

    def counted(x, y, ap):
        calls['n'] += 1
        return real(x, y, ap)
    pa.aperture_distance = counted
    try:
        far = make_pad(1, 0.0, 6.0, size_x=1.1, size_y=1.1, shape='circle')
        hit = pa._pad_overlaps(big, far)
    finally:
        pa.aperture_distance = real
    check('a pad 6 mm off a 1000-edge opening: no overlap, no exact distance to it',
          hit is False and calls['n'] == 0, f'overlap {hit}, {calls["n"]} call(s)')

    print('FAILED: ' + ', '.join(failures) if failures else 'ALL PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
