#!/usr/bin/env python3
"""#622 `awx/plan_audit.py` check_dives: two vias of ONE lane keep the hole-to-hole rule.

A lane's vias are one net's copper, with no clearance to keep from each other, but they are two drills. The audit
measured a via against every OTHER lane's vias and never against its own lane's: a lane that came up to its berth by
a via at its stub's end, after a dive a little before it, was planned with the two 0.18 mm apart, laid so, and put
back as it came for the violation. What this asserts, on a lane with two via sites along one line:

1. the two nearer than a drill and the hole-to-hole clearance: both sites are found, each "via d/bar (its own)";
2. the two that far apart or more: none;
3. ANOTHER lane's via at the nearer distance is found as it always was, by the via-to-via rule, and is not counted
   twice;
4. a lane with one via site: none.
"""
import contextlib
import io
import os
import sys
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'awx'))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'py_router'))
import plan_audit as pa  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402

FAIL = []


def check(cond, what):
    if not cond:
        FAIL.append(what)


VIA, DRILL, CL, TW, G, H2H = 0.3, 0.15, 0.09, 0.09, 0.025, 0.2
BAR = DRILL + H2H                      # two drills of one net: 0.35


def dives(sites, lines):
    """check_dives' rows for lanes {name: [its via sites]} with pieces {name: [(p, q, layer)]}, every site on the grid"""
    names = list(sites)
    ctx = SimpleNamespace(
        cfg=GridRouteConfig(via_size=VIA, via_drill=DRILL, clearance=CL, track_width=TW, grid_step=G,
                            hole_to_hole_clearance=H2H, layers=['F.Cu', 'B.Cu']),
        pairs={}, base_segments=[], base_vias=[],
        pcb=SimpleNamespace(footprints={}, nets={i: SimpleNamespace(name=n) for i, n in enumerate(names)},
                            board_info=SimpleNamespace(board_bounds=(-20.0, -20.0, 20.0, 20.0))),
        byname={n: (i,) for i, n in enumerate(names)}, net_key=lambda s: s)
    c = SimpleNamespace(idx=0, members=names,
                        virtual_vias_of=lambda ns: [s for n in ns for s in sites[n]],
                        virtual_of=lambda ns: [p for n in ns for p in lines[n]])
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        pa.check_dives(ctx, [c])
    return [ln for ln in buf.getvalue().splitlines() if ln.startswith('DIVE ') and '(' in ln.split('failing')[0]
            and 'planned via sites' not in ln]


def lane(y, xs):
    """a lane along y through its via sites at xs: F.Cu to the first, B.Cu between, F.Cu after"""
    pts = [(-3.0, y)] + [(x, y) for x in xs] + [(6.0, y)]
    return [(a, b, 'FB'[i % 2] + '.Cu') for i, (a, b) in enumerate(zip(pts, pts[1:]))]


# 1. its own two vias 0.2 apart: both found, by the hole rule
r = dives({'A': [(0.0, 0.0), (0.2, 0.0)]}, {'A': lane(0.0, [0.0, 0.2])})
check(len(r) == 2 and all('(its own)' in ln and f'0.200/{BAR:.3f}' in ln for ln in r), f'two vias of one lane 0.2 apart: {r}')
# 2. at the bar, and well past it: none
for d in (BAR, 0.6):
    r = dives({'A': [(0.0, 0.0), (d, 0.0)]}, {'A': lane(0.0, [0.0, d])})
    check(r == [], f'two vias of one lane {d} apart: {r}')
# 3. another lane's via 0.2 from it: the via-to-via rule's own row, once a site, and no "its own"
r = dives({'A': [(0.0, 0.0)], 'B': [(0.0, 0.2)]}, {'A': lane(0.0, [0.0]), 'B': lane(0.2, [0.0])})
check(len(r) == 2 and not any('(its own)' in ln for ln in r) and all(' via 0.200/' in ln for ln in r),
      f'two lanes\' vias 0.2 apart: {r}')
# 4. one via alone
r = dives({'A': [(0.0, 0.0)]}, {'A': lane(0.0, [0.0])})
check(r == [], f'a lane with one via: {r}')

if FAIL:
    print('\n'.join('FAIL ' + f for f in FAIL))
    sys.exit(1)
print('PASS test_622_own_via_holes: a lane\'s own two vias held a drill and the hole-to-hole clearance apart, another '
      'lane\'s via by the via-to-via rule as before, a single via alone')
