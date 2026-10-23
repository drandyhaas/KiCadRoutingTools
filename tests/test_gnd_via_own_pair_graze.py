#!/usr/bin/env python3
"""A companion GND via never ships inside its own pair's clearance.

route_diff's `_create_gnd_vias` places a GND return via beside each pair via
at the float offsets (0.44 mm out, 0.49 mm along with a 0.4 via, a
0.1/0.1 pair and 0.09 clearance). The router checked the site at the FLOORED
grid offset (4 cells along) and lets the pair turn 45 degrees 4+2 cells after
the via, so a turn toward the GND via passes its inner track 0.289 mm from
the via centre against 0.340. `_settle_gnd_via` was to move such a via clear,
but it looked at the position only BEFORE each of its 4 moves -- the 0.051 mm
shortfall needs 4 moves and a 5th look -- and on any failure it returned the
planned site, which was placed: one board shipped 5 grazing GND vias (9
check_drc / KiCad clearance items); where the pair wraps round the site (a
U-turn) no outward move clears at all.

Checks, on `_create_gnd_vias` itself (the engine both fronts call), on a
centreline along +x with one layer change at (10, 10) and the pair's P/N
signal vias there (they cap the U-turn's pocket, as measured):
  1. Control: no pair copper -> both vias at their planned sites.
  2. The 45-degree turn: precondition, the planned site grazes the turn by
     ~0.051 mm. The via is placed, moved straight outward (< 0.25 mm), and
     clears every pair track.
  3. The U-turn pocket: precondition, no outward move within 0.25 mm clears.
     The via is placed at the site mirrored across the signal via and clears.
  4. Both sites blocked -- the mirror by the pair's own STUB already on the
     board, and no site within 0.25 mm of either clears: the via is NOT
     placed, the log names why, and `not_placed` carries one 'pair-copper'
     record at the planned site.
  5. The ring: the mirror blocked by a stub beside it, one site 0.15 mm off
     the mirror clears -- the via is placed there, clear of everything.
  Negative control: the test's own clearance measure flags the planned
  sites of 2 and 3 (a measure that cannot see a graze would pass anything).

    python3 tests/test_gnd_via_own_pair_graze.py
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

import diff_pair_routing as dpr                                 # noqa: E402
from geometry_utils import point_to_segment_distance_seg        # noqa: E402
from kicad_parser import Net, Segment, Via                      # noqa: E402
from routing_config import GridCoord, GridRouteConfig           # noqa: E402

GND, P, N = 1, 2, 3
VIA, TRACK, GAP, CLR = 0.4, 0.1, 0.1, 0.09
NEED = VIA / 2 + TRACK / 2 + CLR            # GND via centre to track centre
NEED_VIA = VIA + CLR                        # GND via centre to pair via centre
CFG = GridRouteConfig(clearance=CLR, track_width=TRACK, via_size=VIA,
                      via_drill=0.2, hole_to_hole_clearance=0.25,
                      diff_pair_gap=GAP)
PN_OFF = dpr._pair_via_offset(CFG, (TRACK + GAP) / 2, P, N)
# The pair's signal vias at the layer change: N on the +y side.
PAIR_VIAS = [Via(x=10.0, y=10.0 + PN_OFF, size=VIA, drill=0.2,
                 layers=['F.Cu', 'B.Cu'], net_id=N),
             Via(x=10.0, y=10.0 - PN_OFF, size=VIA, drill=0.2,
                 layers=['F.Cu', 'B.Cu'], net_id=P)]
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def seg(x1, y1, x2, y2, net, layer='B.Cu'):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=TRACK,
                   layer=layer, net_id=net)


def board(segments=()):
    return SimpleNamespace(vias=[], segments=list(segments), pads_by_net={},
                           nets={GND: Net(net_id=GND, name='GND'),
                                 P: Net(net_id=P, name='/D_P'),
                                 N: Net(net_id=N, name='/D_N')})


def gap(x, y, segs, vias=PAIR_VIAS):
    """Smallest GND-via clearance shortfall to the pair (< 0 = inside)."""
    return min([point_to_segment_distance_seg(x, y, s) - NEED for s in segs]
               + [math.hypot(v.x - x, v.y - y) - NEED_VIA for v in vias])


def run(pair_segs=(), pcb=None, pair_vias=()):
    path = [(90, 100, 0), (100, 100, 0), (100, 100, 1), (110, 100, 1)]
    out, np_ = io.StringIO(), []
    with contextlib.redirect_stdout(out):
        vias = dpr._create_gnd_vias(path, GridCoord(0.1), CFG, ['F.Cu', 'B.Cu'],
                                    (TRACK + GAP) / 2, GND, [1], pair_net_ids=(P, N),
                                    pair_segs=list(pair_segs), pcb_data=pcb or board(),
                                    pair_vias=list(pair_vias), not_placed=np_)
    return [(round(v.x, 4), round(v.y, 4)) for v in vias], out.getvalue(), np_


# The pair as drawn on B.Cu after the via (heading +x; +y is the side the
# turn goes toward), plus its incoming F.Cu run. Inner track at y = 10.1.
INCOMING = [seg(8.0, 10.1, 10.0, 10.1, N, 'F.Cu'), seg(8.0, 9.9, 10.0, 9.9, P, 'F.Cu')]
OUTER_STRAIGHT = [seg(10.0, 9.9, 12.0, 9.9, P)]
# 2: the centreline turns 45 degrees toward +y 0.6 mm after the via; the
# inner track's corner sits 0.1 * tan(22.5) before that.
_c = 0.6 - 0.1 * math.tan(math.radians(22.5))
TURN = [seg(10.0, 10.1, 10.0 + _c, 10.1, N), seg(10.0 + _c, 10.1, 12.0, 10.1 + 2.0 - _c, N)]
# 3: the pair wraps back round the site (a measured SYZYGY2.TX2_N's shape):
# turn toward +y, run straight +y, then head back -x past the via.
UTURN = [seg(10.0, 10.1, 10.5586, 10.1, N), seg(10.5586, 10.1, 10.8, 10.3414, N),
         seg(10.8, 10.3414, 10.8, 10.5586, N), seg(10.8, 10.5586, 9.1, 12.2586, N)]


def main():
    # 1. Control.
    sites, _, _ = run()
    check('control: both GND vias of the layer change are placed', len(sites) == 2,
          str(sites))
    if len(sites) != 2:
        print('FAILED: no fixture')
        return 1
    plus = max(sites, key=lambda s: s[1])          # the +y (turn) side
    minus = min(sites, key=lambda s: s[1])
    check('fixture: the planned sites clear the pair\'s signal vias',
          gap(*plus, []) >= -1e-6 and gap(*minus, []) >= -1e-6,
          f'{gap(*plus, []):.4f} / {gap(*minus, []):.4f}')

    # 2. The 45-degree turn.
    segs = INCOMING + OUTER_STRAIGHT + TURN
    g0 = gap(*plus, segs)
    check('precondition: the planned site grazes the turn by ~0.051mm',
          -0.06 < g0 < -0.04, f'{g0:.4f}')
    got, log, np_ = run(segs, pair_vias=PAIR_VIAS)
    moved = [s for s in got if s != minus]
    check('the turn: both vias placed', len(got) == 2 and minus in got, str(got))
    if moved:
        (mx, my), = moved
        check('...moved straight outward, under 0.25mm',
              abs(mx - plus[0]) < 1e-6 and 0 < my - plus[1] < 0.25,
              f'({mx},{my}) from {plus}')
        check('...and it clears every pair track and via', gap(mx, my, segs) >= -1e-6,
              f'{gap(mx, my, segs):.4f}')
    check('...with nothing reported not placed', not np_ and 'not placed' not in log,
          log.strip()[:200])

    # 3. The U-turn pocket: no outward move clears.
    segs = INCOMING + OUTER_STRAIGHT + UTURN
    check('precondition: the planned site grazes the U-turn',
          gap(*plus, segs) < -0.02, f'{gap(*plus, segs):.4f}')
    best = max(gap(plus[0], plus[1] + k * 0.005, segs) for k in range(51))
    check('precondition: no outward move up to 0.25mm clears it', best < -1e-6,
          f'best {best:.4f}')
    got, log, np_ = run(segs, pair_vias=PAIR_VIAS)
    mirror = (round(2 * 10.0 - plus[0], 4), plus[1])
    check('the U-turn: placed at the site mirrored across the signal via',
          sorted(got) == sorted([mirror, minus]), f'{got}; mirror {mirror}')
    check('...which clears every pair track and via', gap(*mirror, segs) >= -1e-6,
          f'{gap(*mirror, segs):.4f}')

    # 4. Both sites blocked: the mirror by the pair's own stub on the board.
    stub = seg(mirror[0] - 0.5, mirror[1] + 0.2, mirror[0] + 0.3, mirror[1] + 0.2, N)
    check('precondition: the board stub is inside the mirror site\'s clearance',
          gap(*mirror, [stub], []) < 0, f'{gap(*mirror, [stub], []):.4f}')
    ring = [(c[0] + r * math.cos(a), c[1] + r * math.sin(a))
            for c in (plus, mirror) for r in (0.05, 0.1, 0.15, 0.2, 0.25)
            for a in (2 * math.pi * j / 64 for j in range(64))]
    check('precondition: no site within 0.25mm of either clears',
          max(gap(x, y, segs + [stub]) for x, y in ring) < -1e-6,
          f'best {max(gap(x, y, segs + [stub]) for x, y in ring):.4f}')
    got, log, np_ = run(segs, board([stub]), pair_vias=PAIR_VIAS)
    check('both sites blocked: that via is NOT placed, the other is', got == [minus],
          str(got))
    check('...the run says why', "not placed: no site clears the pair's own copper"
          in log and '/D_N track on B.Cu' in log, log.strip()[:240])
    check('...and not_placed carries one pair-copper record at the planned site',
          len(np_) == 1 and np_[0]['reason'] == 'pair-copper'
          and (np_[0]['x'], np_[0]['y']) == plus and np_[0]['pair'] == '/D',
          str(np_))
    got, _, _ = run(segs, board([seg(5.0, 5.0, 5.5, 5.0, N)]), pair_vias=PAIR_VIAS)
    check('witness: a stub far away does not block the mirror',
          sorted(got) == sorted([mirror, minus]), str(got))

    # 5. The ring: a stub beside the mirror leaves one site 0.15 mm off it.
    stub = seg(mirror[0] + 0.2, mirror[1] - 0.14, mirror[0] + 0.2, mirror[1] + 0.6, N)
    check('precondition: the stub blocks the mirror and its outward move',
          max(gap(mirror[0], mirror[1] + k * 0.005, segs + [stub])
              for k in range(51)) < -1e-6)
    got, log, np_ = run(segs, board([stub]), pair_vias=PAIR_VIAS)
    placed = [s for s in got if s != minus]
    check('the ring: both vias placed, nothing reported', len(got) == 2 and not np_,
          f'{got}; {np_}')
    if placed:
        (rx, ry), = placed
        check('...within 0.25mm of the mirror, clear of the pair and the stub',
              math.hypot(rx - mirror[0], ry - mirror[1]) <= 0.25 + 1e-6
              and gap(rx, ry, segs + [stub]) >= -1e-6,
              f'({rx},{ry}) gap {gap(rx, ry, segs + [stub]):.4f}')

    # Negative control: the measure sees a graze (2 and 3 planned sites).
    check('negative control: the clearance measure flags the planned sites',
          gap(*plus, INCOMING + TURN) < 0 and gap(*plus, INCOMING + UTURN) < 0)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
