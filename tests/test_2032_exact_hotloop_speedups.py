"""The hot-loop rewrites in check_connected's SpatialIndex/SegmentIndex,
geometry_utils.point_to_segment_distance and rip_up_reroute's collider
prefilter answer exactly what the loops they replace answered: the same items,
in the same order, and the same floats."""
import math
import os
import random
import sys
from collections import defaultdict
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

from check_connected import SegmentIndex, SpatialIndex  # noqa: E402
from geometry_utils import point_to_segment_distance  # noqa: E402
import rip_up_reroute  # noqa: E402


# --- the replaced implementations, verbatim -------------------------------

def _ref_ptseg(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-10:
        return math.sqrt((px - x1)**2 + (py - y1)**2)
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)


class _RefSpatial:
    def __init__(self, cell_size=1.0):
        self.cell_size = cell_size
        self.grid = defaultdict(list)

    def _cell(self, x, y):
        return (int(x // self.cell_size), int(y // self.cell_size))

    def add(self, x, y, layer, point_id, size):
        gx, gy = self._cell(x, y)
        self.grid[(gx, gy, layer)].append((x, y, point_id, size))

    def query_nearby(self, x, y, layer, radius):
        gx, gy = self._cell(x, y)
        n = int(radius / self.cell_size) + 1
        out = []
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                for pt in self.grid.get((gx + dx, gy + dy, layer), []):
                    out.append(pt)
        return out


class _RefSegIdx:
    def __init__(self, cell_size=1.0):
        self.cell_size = cell_size
        self.grid = defaultdict(list)

    def add(self, seg, sid):
        x1, y1, x2, y2 = seg.start_x, seg.start_y, seg.end_x, seg.end_y
        gx1, gy1 = int(min(x1, x2) // self.cell_size), int(min(y1, y2) // self.cell_size)
        gx2, gy2 = int(max(x1, x2) // self.cell_size), int(max(y1, y2) // self.cell_size)
        for gx in range(gx1, gx2 + 1):
            for gy in range(gy1, gy2 + 1):
                self.grid[(gx, gy, seg.layer)].append((seg, sid))

    def query_near(self, x, y, layer, radius=None):
        gx, gy = int(x // self.cell_size), int(y // self.cell_size)
        r = 1
        if radius is not None and radius > self.cell_size:
            r = int(radius // self.cell_size) + 1
        seen, out = set(), []
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for item in self.grid.get((gx + dx, gy + dy, layer), ()):
                    if id(item[0]) not in seen:
                        seen.add(id(item[0]))
                        out.append(item)
        return out


def _coord(r):
    # Grid-aligned, negative, signed-zero and plain values all occur on boards.
    return r.choice([r.uniform(-12, 12), float(r.randint(-4, 4)), 0.0, -0.0,
                     r.randint(-40, 40) * 0.25])


def _segs(r, n):
    return [SimpleNamespace(start_x=_coord(r), start_y=_coord(r), end_x=_coord(r),
                            end_y=_coord(r), layer=r.choice(['F.Cu', 'B.Cu', 'In1.Cu']),
                            width=r.choice([0.1, 0.2, 0.5, 2.0]),
                            net_id=r.randint(0, 6)) for _ in range(n)]


def test_point_to_segment_distance_matches_clamp():
    r = random.Random(7)
    for _ in range(100000):
        a = [_coord(r) for _ in range(6)]
        assert point_to_segment_distance(*a) == _ref_ptseg(*a)
    # The clamp's edges: exactly 0, exactly 1, NaN-free degenerate point.
    assert point_to_segment_distance(0, 0, 0, 0, 1, 0) == _ref_ptseg(0, 0, 0, 0, 1, 0)
    assert point_to_segment_distance(1, 0, 0, 0, 1, 0) == _ref_ptseg(1, 0, 0, 0, 1, 0)
    assert point_to_segment_distance(3, 4, 0, 0, 0, 0) == 5.0


def test_spatial_index_same_points_same_order():
    r = random.Random(11)
    for cs in (0.5, 1.0, 2.5):
        new, ref = SpatialIndex(cs), _RefSpatial(cs)
        for i in range(400):
            x, y, layer = _coord(r), _coord(r), r.choice(['F.Cu', 'B.Cu'])
            new.add(x, y, layer, i, 0.3)
            ref.add(x, y, layer, i, 0.3)
        assert dict(new.grid) == dict(ref.grid)
        for _ in range(500):
            q = (_coord(r), _coord(r), r.choice(['F.Cu', 'B.Cu']), r.choice([0.02, 0.6, 1.7, 4.0]))
            assert new.query_nearby(*q) == ref.query_nearby(*q)


def test_segment_index_same_cells_same_order():
    r = random.Random(13)
    for cs in (0.5, 1.0, 2.5):
        segs = _segs(r, 300)
        new, ref = SegmentIndex(cs), _RefSegIdx(cs)
        for i, s in enumerate(segs):
            new.add(s, i)
            ref.add(s, i)
        assert dict(new.grid) == dict(ref.grid)
        for _ in range(500):
            x, y, layer = _coord(r), _coord(r), r.choice(['F.Cu', 'B.Cu', 'In1.Cu'])
            rad = r.choice([None, 0.3, 1.0, 2.6, 5.0])
            assert new.query_near(x, y, layer, rad) == ref.query_near(x, y, layer, rad)
            assert new.query_at(x, y, layer) == ref.grid.get(
                (int(x // cs), int(y // cs), layer), [])


def _ref_colliders(saved, pcb, own, clearance):
    """The full-board pairwise test the prefilter must not change."""
    segs = saved['new_segments']
    out = []
    for o in pcb.segments:
        if o.net_id == 0 or o.net_id in own:
            continue
        for s in segs:
            if s.layer != o.layer:
                continue
            thr = s.width / 2 + o.width / 2 + clearance
            if rip_up_reroute._seg_seg_dist_sq(s.start_x, s.start_y, s.end_x, s.end_y,
                                               o.start_x, o.start_y, o.end_x, o.end_y) < thr * thr:
                out.append(id(o))
                break
    return sorted(out)


def test_saved_route_colliders_prefilter_keeps_every_collider():
    r = random.Random(17)
    for _ in range(60):
        board = _segs(r, 250)
        pcb = SimpleNamespace(segments=board, vias=[], pads_by_net={})
        own = {1}
        saved = {'new_segments': [s for s in _segs(r, 4)], 'new_vias': []}
        for s in saved['new_segments']:
            s.net_id = 1
        hits = rip_up_reroute._saved_route_colliders(saved, pcb, own, 0.2)
        got = sorted(id(o) for kind, o in hits if kind == 'segment')
        assert got == _ref_colliders(saved, pcb, own, 0.2)


def main():
    tests = [test_point_to_segment_distance_matches_clamp,
             test_spatial_index_same_points_same_order,
             test_segment_index_same_cells_same_order,
             test_saved_route_colliders_prefilter_keeps_every_collider]
    for t in tests:
        t()
        print('PASS', t.__name__)
    return 0


if __name__ == '__main__':
    sys.exit(main())
