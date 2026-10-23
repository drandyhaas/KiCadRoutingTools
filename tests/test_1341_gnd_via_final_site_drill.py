#!/usr/bin/env python3
"""#1341: a companion GND via's FINAL site is checked against every drill.

route_diff's router validates each GND return via on the grid (the floored
offset); `_create_gnd_vias` places it at the float offset and
`_settle_gnd_via` may move it further, and neither re-check looked at a
drill: the settle's `via_barrel_clear_of_foreign_copper` skips the via's own
net, so another pair's GND via was never seen. On CM5_MINIMA_3 (gp14 M27)
DPHY0_P.C's GND via shipped 0.21 mm hole-to-hole from DPHY0_P.D2's against
0.25 (check_drc VIA-DRILL-HOLE, KiCad hole_to_hole). The final site is now
checked against every via drill (any net), every pad hole and the GND vias
the call already placed; a site that fails is not placed and is reported.

Checks, on `_create_gnd_vias` itself (the engine both fronts call):
  1. Control: with an empty board both GND vias of a layer change are placed.
  2. Another pair's GND via 0.20 mm hole-to-hole from the P-side site: that
     via is not placed and the reason names the GND via; the N side is.
  3. A plated pad hole 0.10 mm from the N-side site: that one is not placed.
  4. A via the call REPLACES (a meandered pair's own, still on the board) is
     not an obstacle: passed as ignore_vias, both are placed again.
  5. Clear of everything by more than hole-to-hole: both placed.

    python3 tests/test_1341_gnd_via_final_site_drill.py
"""
import contextlib
import io
import math
import os
import sys
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from diff_pair_routing import _create_gnd_vias                  # noqa: E402
from kicad_parser import Net, Via                               # noqa: E402
from routing_config import GridCoord, GridRouteConfig           # noqa: E402
from synth import make_pad                                      # noqa: E402

GND, P, N = 1, 2, 3
H2H, DRILL, VIA = 0.25, 0.15, 0.3
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def board(vias=(), pads=()):
    by_net = {}
    for p in pads:
        by_net.setdefault(p.net_id, []).append(p)
    return SimpleNamespace(vias=list(vias), segments=[], pads_by_net=by_net,
                           nets={GND: Net(net_id=GND, name='GND'),
                                 P: Net(net_id=P, name='/D_P'),
                                 N: Net(net_id=N, name='/D_N'),
                                 9: Net(net_id=9, name='/X')})


def run(pcb, ignore=()):
    cfg = GridRouteConfig(clearance=0.13, track_width=0.1, via_size=VIA,
                          via_drill=DRILL, hole_to_hole_clearance=H2H,
                          diff_pair_gap=0.13)
    # A straight centreline along +x with one layer change at (10, 10).
    path = [(90, 100, 0), (100, 100, 0), (100, 100, 1), (110, 100, 1)]
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        vias = _create_gnd_vias(path, GridCoord(0.1), cfg, ['F.Cu', 'B.Cu'],
                                0.115, GND, [1], pair_net_ids=(P, N),
                                pcb_data=pcb, ignore_vias=ignore)
    return [(round(v.x, 4), round(v.y, 4)) for v in vias], out.getvalue()


def other_gnd_via(x, y, gap):
    """A GND via whose drill edge sits `gap` from a GND via drill at (x, y)."""
    d = gap + DRILL
    return Via(x=x + d, y=y, size=VIA, drill=DRILL, layers=['F.Cu', 'B.Cu'],
               net_id=GND)


def main():
    # 1. Control.
    sites, _ = run(board())
    check('control: both GND vias of the layer change are placed',
          len(sites) == 2, str(sites))
    if len(sites) != 2:
        print('FAILED: no fixture'); return 1
    (px, py), (nx, ny) = sites

    # 2. Another pair's GND via under hole-to-hole of the P site.
    blocker = other_gnd_via(px, py, 0.20)
    got, log = run(board(vias=[blocker]))
    check('a GND via 0.20mm hole-to-hole from the P site: P not placed, N placed',
          got == [(nx, ny)], str(got))
    check('...and the run says why, naming the GND via',
          'not placed' in log and 'GND via drill' in log and '(#1341)' in log,
          log.strip()[:200])

    # 3. A plated pad hole by the N site.
    pad = make_pad(9, nx + 0.10 + DRILL / 2 + 0.4, ny, ref='J9', num='1',
                   net_name='/X', size_x=1.2, size_y=1.2, shape='circle',
                   layers=('*.Cu',), drill=0.8, pad_type='thru_hole')
    got, log = run(board(pads=[pad]))
    check('a plated pad hole 0.10mm from the N site: N not placed',
          got == [(px, py)] and "pad J9.1's hole" in log, f'{got}; {log.strip()[:160]}')

    # 4. Vias being replaced are not obstacles.
    own = [Via(x=x, y=y, size=VIA, drill=DRILL, layers=['F.Cu', 'B.Cu'], net_id=GND)
           for x, y in sites]
    got, _ = run(board(vias=own), ignore=own)
    check('the pair\'s own vias, passed as replaced, do not block their re-creation',
          got == sites, str(got))
    got, _ = run(board(vias=own))
    check('witness: without ignore_vias those same vias do block it',
          got == [], str(got))

    # 5. Clear by more than hole-to-hole.
    got, _ = run(board(vias=[other_gnd_via(px, py, 0.30)]))
    check('a GND via 0.30mm hole-to-hole away: both placed', got == sites,
          str(got))
    gap = math.hypot(blocker.x - px, blocker.y - py) - DRILL
    check('fixture: the step-2 blocker really is under hole-to-hole',
          gap < H2H, f'{gap:.3f} < {H2H}')

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
