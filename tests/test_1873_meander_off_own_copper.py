#!/usr/bin/env python3
"""#1873: a length-matching bump is not laid back over its own net's copper.

`get_safe_amplitude_at_point` skips every piece of the meandered net's own
copper: the bump joins the route, so the run it replaces must not block it.
It skipped the REST of the net too, so on a run leaving an acute corner of
its own route the first bump climbed back over the copper that came into
the corner. Measured in route_diff's intra-pair matching: two matched members
whose risers ran 0.054 and 0.066 mm off their own track (one strip of copper
laid twice, counted twice in the member's length) and crossed it; the
step shipped one of them with a dangling end.

`apply_meanders_to_route` now hands the generator the net's own copper
outside the run (`_own_board_copper`, `_own_copper_beside_run`), and a bump
keeps from it the pitch it keeps between its own arms. Checked:

  1. the precondition: with the corner's other leg absent, the first bump
     stands where that leg would be (else this test exercises nothing);
  2. with that leg in the route, and with it on the board only: no bump
     crosses it or comes within the arm pitch of it, and the run is still
     meandered;
  3. a run whose corner turns away from the bumps is meandered exactly as
     without its other leg (the check binds only where the copper is).

    python3 tests/test_1873_meander_off_own_copper.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import BoardInfo, Net, PCBData, Segment       # noqa: E402
from routing_config import GridRouteConfig                     # noqa: E402
from geometry_utils import segment_to_segment_distance, segments_intersect  # noqa: E402
import length_matching as lm                                    # noqa: E402

NET = 7
W = 0.1
EXTRA = 2.0
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def seg(a, b, net=NET):
    return Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=W,
                   layer='F.Cu', net_id=net)


def board(segs):
    bi = BoardInfo(layers={}, board_bounds=(90.0, 90.0, 120.0, 115.0),
                   copper_layers=['F.Cu', 'B.Cu'])
    return PCBData(board_info=bi, footprints={}, vias=[], pads_by_net={},
                   nets={NET: Net(NET, '/SIG')}, segments=list(segs))


def cfg():
    c = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=W, clearance=0.1)
    c.meander_amplitude = 1.0
    return c


# The run heads west from the corner at (110, 103); with u = -x the first
# bump goes to +p = -y. The corner's other leg comes in from the north-west
# at 45 degrees (a 135-degree turn), on that side and ahead of the run's start.
RUN = seg((110.0, 103.0), (100.0, 103.0))
ACUTE = seg((107.0, 100.0), (110.0, 103.0))
# A corner that turns away: the other leg comes in from the east.
AWAY = seg((113.0, 101.5), (110.0, 103.0))


def new_copper(out, run):
    """The meander's copper: what replaced `run` in the route."""
    return [s for s in out if s is not run and s.net_id == NET
            and s not in (ACUTE, AWAY)]


def bumps_only(copper):
    """The meander's copper off the run's centre line (y 103): the bumps,
    without the lead-in and lead-out that lie on the route as it was."""
    return [s for s in copper if max(abs(s.start_y - 103.0), abs(s.end_y - 103.0)) > 1e-6]


def nearest(copper, leg):
    return min(segment_to_segment_distance(s.start_x, s.start_y, s.end_x, s.end_y,
                                           leg.start_x, leg.start_y, leg.end_x, leg.end_y)
               for s in copper)


def length(segs):
    return math.fsum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs)


def main():
    c = cfg()
    pitch = 2 * lm.resolve_meander_chamfer(c, NET, 'F.Cu', W)

    print("1. the precondition: without the corner's other leg")
    out, bumps = lm.apply_meanders_to_route([RUN], EXTRA, c, pcb_data=board([RUN]), net_id=NET)
    ok(bumps > 0, f"the run is meandered ({bumps} bumps)")
    ok(nearest(bumps_only(new_copper(out, RUN)), ACUTE) < W,
       f"a bump stands on the copper the other leg would lay "
       f"({nearest(bumps_only(new_copper(out, RUN)), ACUTE):.3f} mm off its centre line)")

    for where, route, on_board in (("in the route", [ACUTE, RUN], [ACUTE, RUN]),
                                   ("on the board only", [RUN], [ACUTE, RUN])):
        print(f"2. the acute corner's other leg {where}")
        out, bumps = lm.apply_meanders_to_route(route, EXTRA, c, pcb_data=board(on_board),
                                                net_id=NET)
        mine = new_copper(out, RUN)
        gap = nearest(bumps_only(mine), ACUTE)
        ok(bumps > 0 and gap >= pitch - 1e-6,
           f"no bump within the arm pitch {pitch:.2f} mm of it ({gap:.3f} mm, {bumps} bumps)")
        crossed = [s for s in bumps_only(mine) if segments_intersect(
            s.start_x, s.start_y, s.end_x, s.end_y,
            ACUTE.start_x, ACUTE.start_y, ACUTE.end_x, ACUTE.end_y)]
        ok(not crossed, f"no bump crosses it ({len(crossed)} segments do)")
        added = length(mine) - length([RUN])
        ok(added > 1.0, f"the run is still lengthened ({added:.3f} mm)")

    print("3. a corner that turns away from the bumps")
    alone, n0 = lm.apply_meanders_to_route([RUN], EXTRA, c, pcb_data=board([RUN]), net_id=NET)
    beside, n1 = lm.apply_meanders_to_route([AWAY, RUN], EXTRA, c,
                                            pcb_data=board([AWAY, RUN]), net_id=NET)

    def pts(segs):
        return [(round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4), round(s.end_y, 4))
                for s in segs]
    ok(n0 == n1 and pts(new_copper(alone, RUN)) == pts(new_copper(beside, RUN)),
       f"meandered exactly as without the other leg ({n0} / {n1} bumps)")

    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
