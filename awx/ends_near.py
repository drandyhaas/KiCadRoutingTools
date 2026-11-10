"""ends_near: is a point AT a run's ends -- ONE test, the loop's feedback's (whole_feedback: a finding at the teeth
or the berths is the fanout's to change) and its ends' repair's (whole_route.repair_ends: the rows of that finding it
repairs). Two tests disagreed: the feedback named a lane's tooth crowded by a pad beside the array's side face, within
the width the face's lanes stack to, and the repair, which looked only as far as the front of a face, found "no row of
it there" and moved nothing -- the loop ran again on the same ends and stopped on the same finding.

The ends are a run's plan sidecar's (`ends` {net: (tooth, berth)}, `dest_of` {net: its part} where the run ends at
several parts); every point in the board's frame."""
import math


def box(pts):
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def boxes(ends, dest_of=None):
    """{0: [the teeth's box], 1: [the berths' box -- one a part where the run ends at several: one box round them all
    would span the gap between the parts]}"""
    groups = {}
    for n, e in ends.items():
        groups.setdefault((dest_of or {}).get(n, ''), []).append(e[1])
    return {0: [box([e[0] for e in ends.values()])],
            1: [box(v) for _k, v in sorted(groups.items())] if len(groups) > 1 else [box([e[1] for e in ends.values()])]}


def gap(b, x, y):
    return math.hypot(max(b[0] - x, 0.0, x - b[2]), max(b[1] - y, 0.0, y - b[3]))


def side_reach(ends, k, face, b, lane_min):
    """beside a side face ('N' or 'S') of box b of the ends k: the width its ends' lanes stack to, and two pitches"""
    y0 = b[1] if face == 'N' else b[3]
    n = sum(1 for e in ends.values() if abs(e[k][1] - y0) < 1e-3)
    return (n + 2) * lane_min


def near(ends, bxs, k, x, y, front, lane_min):
    """is (x, y) at the ends k (0 the teeth, 1 the berths): beside a side face within its stack, else in front within
    `front` (of the box of the ends nearest it, `bxs` boxes(ends, dest_of))"""
    b = min(bxs[k], key=lambda b_: gap(b_, x, y))
    dx = max(b[0] - x, 0.0, x - b[2])
    dy = max(b[1] - y, 0.0, y - b[3])
    if dx == 0.0 and dy > 0.0:
        return dy <= side_reach(ends, k, 'N' if y < b[1] else 'S', b, lane_min)
    return math.sqrt(dx * dx + dy * dy) <= front
