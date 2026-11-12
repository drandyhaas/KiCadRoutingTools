"""_seg_foreign_seg_dist's column prune returns the sampled minimum of the full sweep: random boards
of foreign tracks and vias, against a brute-force sweep of every sample and every foreign item."""
import math
import os
import random
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

import single_ended_routing as ser  # noqa: E402


def brute(segs, vias, net_id, x1, y1, x2, y2, layer, excess_of=None):
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.02) + 1)
    items = [(s.start_x, s.start_y, s.end_x, s.end_y, s.width / 2, s.net_id) for s in segs
             if s.layer == layer and s.net_id != net_id]
    items += [(v.x, v.y, v.x, v.y, v.size / 2, v.net_id) for v in vias if v.net_id != net_id]
    R = ser._FOREIGN_PAD_WINDOW
    best = math.inf
    for ax, ay, bx, by, hw, nid in items:
        if (max(ax, bx) + hw < min(x1, x2) - R or min(ax, bx) - hw > max(x1, x2) + R or
                max(ay, by) + hw < min(y1, y2) - R or min(ay, by) - hw > max(y1, y2) + R):
            continue
        ex = excess_of(nid) if excess_of else 0.0
        for i in range(n + 1):
            t = i / n
            px, py = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            u = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
            best = min(best, math.hypot(px - (ax + u * dx), py - (ay + u * dy)) - hw - ex)
    return best if best < math.inf else 1e9


def test_prune_matches_full_sweep():
    rnd = random.Random(5)
    for trial in range(60):
        segs = []
        for _ in range(rnd.randint(5, 60)):
            x, y = rnd.uniform(0, 10), rnd.uniform(0, 10)
            a = rnd.choice([0, 45, 90, 135]) * math.pi / 180
            ln = rnd.uniform(0, 4)
            segs.append(NS(start_x=x, start_y=y, end_x=x + ln * math.cos(a), end_y=y + ln * math.sin(a),
                           layer=rnd.choice(['F.Cu', 'B.Cu']), width=rnd.choice([0.1, 0.13, 0.25]),
                           net_id=rnd.randint(1, 6)))
        vias = [NS(x=rnd.uniform(0, 10), y=rnd.uniform(0, 10), size=0.45, drill=0.2, net_id=rnd.randint(1, 6),
                   layers=['F.Cu', 'B.Cu']) for _ in range(rnd.randint(0, 10))]
        pcb = NS(segments=segs, vias=vias, footprints=None)
        nc = {k: rnd.choice([0.1, 0.15, 0.2]) for k in range(1, 7)} if trial % 2 else None
        for _ in range(10):
            x1, y1 = rnd.uniform(1, 9), rnd.uniform(1, 9)
            x2, y2 = x1 + rnd.uniform(-3, 3), y1 + rnd.uniform(-3, 3)
            net = rnd.randint(1, 6)
            got = ser._seg_foreign_seg_dist(pcb, net, x1, y1, x2, y2, 'F.Cu', net_clearances=nc,
                                            base_clearance=0.1)
            ex = (lambda f: max(0.0, nc.get(f, 0.1) - 0.1)) if nc else None
            want = brute(segs, vias, net, x1, y1, x2, y2, 'F.Cu', ex)
            assert abs(got - want) < 1e-9, (trial, got, want)


if __name__ == '__main__':
    test_prune_matches_full_sweep()
    print('PASS')
