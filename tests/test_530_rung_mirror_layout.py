#!/usr/bin/env python3
"""#530 per-net via rungs: layout and raw-copper mirrors, at the map level.

Rung 0 is the run's via. When the #568 small map is armed it holds rung 1
(obstacle_cache.via_rungs keeps that slot for it) and its own mirror stamps
it, so per-net rungs start at 2; otherwise they start at 1. The raw-copper
mirrors (_mirror_rungs_add/_remove) must stamp every PER-NET rung and never
touch the small map (its stamps are computed at the small via size by the
#568 path); before this test the mirrors ran only when the small map was
NOT armed, so per-net rungs missed every raw copper add on a run with the
small map on.

Needs grid_router 0.22.0+ (SKIPs otherwise).
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))


def main():
    try:
        import grid_router
        if not hasattr(grid_router.GridObstacleMap, 'add_blocked_vias_rung_batch'):
            print("SKIP: grid_router without the via-rung API (needs 0.22.0+)")
            return 0
    except ImportError:
        print("SKIP: grid_router not importable")
        return 0
    import numpy as np
    import obstacle_map as om
    fails = []
    saved = om._rung_small_armed
    try:
        for armed in (False, True):
            om._rung_small_armed = lambda armed=armed: armed
            m = grid_router.GridObstacleMap(2)
            m.add_blocked_via_small(1, 1)            # rung 1 exists (the #568 map)
            m.add_blocked_via_rung(2, 1, 1)          # one per-net rung
            m.add_blocked_via_rung(3, 1, 1)          # and another
            want = list(range(2 if armed else 1, 4))
            got = list(om._per_net_rungs(m))
            if got != want:
                fails.append(f"armed={armed}: _per_net_rungs {got} != {want}")
            before = [m.rung_len(r) for r in (1, 2, 3)]
            cells = np.array([(5, 5), (6, 5)], dtype=np.int32)
            om._mirror_rungs_add(m, cells)
            after = [m.rung_len(r) for r in (1, 2, 3)]
            exp = [before[0] + (0 if armed else 2), before[1] + 2, before[2] + 2]
            if after != exp:
                fails.append(f"armed={armed}: rung lens after add {after} != {exp}")
            om._mirror_rungs_remove(m, cells)
            back = [m.rung_len(r) for r in (1, 2, 3)]
            if back != before:
                fails.append(f"armed={armed}: rung lens after remove {back} != {before}")
            if m.is_via_blocked_rung(5, 5, 2):
                fails.append(f"armed={armed}: cell still blocked at rung 2 after the symmetric remove")
        # a single-rung map (no per-net rungs) mirrors nothing
        om._rung_small_armed = lambda: False
        m1 = grid_router.GridObstacleMap(2)
        if list(om._per_net_rungs(m1)) != []:
            fails.append(f"single-rung map reports per-net rungs {list(om._per_net_rungs(m1))}")
        om._mirror_rungs_add(m1, np.array([(1, 1)], dtype=np.int32))
        if m1.rung_count() != 1:
            fails.append("mirroring into a single-rung map allocated a rung")
    finally:
        om._rung_small_armed = saved
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print("PASS: per-net rungs sit above the small map when it is armed (else from 1), "
          "raw-copper mirrors stamp every per-net rung and never the small map, and remove is symmetric")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
