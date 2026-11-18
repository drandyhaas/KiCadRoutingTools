"""_saved_route_colliders' cell index changes no verdict.

The restore test looks up foreign copper through `_BoxIndex`. Swapping in an
index that returns every item gives the full all-pairs scan; on random boards
(mixed widths, via sizes, per-pair clearances, hole-to-hole rules) both must
return the same colliders in the same order, for first_only on and off.
"""
import os
import random
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'py_router'))
import rip_up_reroute as rur  # noqa: E402

LAYERS = ['F.Cu', 'In1.Cu', 'B.Cu']


class _FullScan:
    """Stand-in for _BoxIndex: every item is a candidate (the old scan)."""

    def __init__(self, reach):
        self.n = 0

    def add(self, x0, y0, x1, y1):
        self.n += 1

    def near(self, x0, y0, x1, y1):
        return range(self.n)


class _Cfg:
    def __init__(self, rng):
        self.maxc = 0.4
        self.hole_to_hole_clearance = rng.choice([0.0, 0.25, 0.5])
        self.hole_rules = None
        self.vals = {}

    def pair_clearance_inert(self):
        return False

    def max_pair_clearance(self, base=None):
        return max(self.maxc, base or 0)

    def pair_clearance(self, a, b, layer, kind='layer', base=None):
        k = (a, b, layer, kind)
        if k not in self.vals:
            self.vals[k] = random.Random(repr(k)).uniform(base, self.maxc)
        return self.vals[k]


def _seg(rng, net, span):
    x, y = rng.uniform(0, span), rng.uniform(0, span)
    L = rng.choice([0.0, 0.3, 2, 10, 40])
    return NS(start_x=x, start_y=y, end_x=x + rng.uniform(-L, L), end_y=y + rng.uniform(-L, L),
              width=rng.choice([0.1, 0.2, 0.5, 2.0]), layer=rng.choice(LAYERS), net_id=net)


def _via(rng, net, span):
    s = rng.choice([0.4, 0.6, 1.0])
    return NS(x=rng.uniform(0, span), y=rng.uniform(0, span), size=s,
              drill=s * rng.choice([0.5, 0.6]), net_id=net)


def _run(saved, pcb, clr, first_only, cfg, index_cls):
    keep = rur._BoxIndex
    rur._BoxIndex = index_cls
    try:
        return [(k, id(o)) for k, o in
                rur._saved_route_colliders(saved, pcb, [7], clr, first_only, cfg)]
    finally:
        rur._BoxIndex = keep


def main():
    cases = hits = bad = 0
    for trial in range(600):
        rng = random.Random(trial)
        span = rng.choice([3, 10, 50])
        pcb = NS(segments=[_seg(rng, rng.randint(0, 20), span) for _ in range(rng.randint(0, 200))],
                 vias=[_via(rng, rng.randint(0, 20), span) for _ in range(rng.randint(0, 60))])
        saved = {'new_segments': [_seg(rng, 7, span) for _ in range(rng.randint(0, 20))],
                 'new_vias': [_via(rng, 7, span) for _ in range(rng.randint(0, 5))]}
        clr = rng.choice([0.1, 0.2, 0.3])
        for first_only in (False, True):
            for use_cfg in (False, True):
                full = _run(saved, pcb, clr, first_only,
                            _Cfg(random.Random(trial)) if use_cfg else None, _FullScan)
                idx = _run(saved, pcb, clr, first_only,
                           _Cfg(random.Random(trial)) if use_cfg else None, rur._BoxIndex)
                cases += 1
                hits += bool(full)
                if full != idx:
                    bad += 1
                    print(f"FAIL trial {trial} first_only={first_only} cfg={use_cfg}: "
                          f"full {len(full)} vs index {len(idx)}")
    print(f"{cases} cases, {hits} with colliders, {bad} differ")
    if hits < cases // 4:
        print("FAIL: too few colliding cases to mean anything")
        return 1
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
