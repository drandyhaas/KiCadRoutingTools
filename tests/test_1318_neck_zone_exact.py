#!/usr/bin/env python3
"""#1318: a pad-entry zone piece the routing grid refuses ships at the pad's
width where it clears in exact geometry.

The neck zone's fit test runs on the routing grid: cells round, and a piece
whose end is off the grid (the pad centre it lands on) is tested with
_OFFGRID_FIT_GUARD (0.7 cell) of extra margin. A 0.3 mm rail into a 0.3 mm pin
at 0.5 mm pitch that clears its neighbours by 10 um was therefore refused and
shipped at the signal width. _neck_zone_exact_fits now gives such a piece an
exact verdict: foreign pads by check_drc.segment_pad_gap at the pair
clearance, tracks and vias by the exact segment distance. Where the routing
map holds something those measures do not model (the board edge, a bare
drill hole, a keep-out ...), it stands aside and the grid's verdict holds.

Board: own pin (net 1, 0.3 x 1.55, centre off the 0.05 grid) between two
foreign pins at 0.5 mm pitch; clearance 0.19; the rail is 0.3 wide, routed
at 0.15 (the uniform-width path), running out of the pin along its axis.

  1. Precondition: the grid refuses the entry piece and the exact check
     admits it -- otherwise this test tests nothing.
  2. The entry ships at the pad's 0.3 alongside the neighbours, and every
     shipped piece clears them by check_drc's own measure.
  3. Negative control: the neighbours 0.01 mm closer (0.48 pitch, 10 um
     short of the rule) -- the entry necks and nothing ships grazing.
  4. Negative control: an NPTH hole within reach of a piece whose copper
     clears it -- the exact check stands aside (the grid keeps the verdict);
     without the hole it admits the same piece.
  5. Negative control: the board edge within reach -- likewise.

    python3 tests/test_1318_neck_zone_exact.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'rust_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import BoardInfo, Net                            # noqa: E402
from routing_config import GridRouteConfig, GridCoord              # noqa: E402
from obstacle_map import build_base_obstacle_map                   # noqa: E402
from check_drc import segment_pad_gap                              # noqa: E402
from synth import make_pad, make_pcb, make_seg                     # noqa: E402
import single_ended_routing as ser                                 # noqa: E402

RUN_ALL_TIMEOUT = 120

CLR = 0.19
NET_W = 0.3
NARROW = 0.15
X0, Y0 = 10.0, 10.025          # own pin centre: on the grid in x, off it in y
PIN = dict(size_x=0.3, size_y=1.55, layers=('F.Cu',))

fails = []


def check(name, ok, detail=''):
    print(('PASS' if ok else 'FAIL') + f': {name}' + (f' -- {detail}' if detail else ''))
    if not ok:
        fails.append(name)


def board(pitch=0.5, extra_pads=(), bounds=(0.0, 0.0, 20.0, 20.0)):
    pads = {1: [make_pad(1, X0, Y0, num='1', net_name='+1V8', **PIN)],
            2: [make_pad(2, X0 - pitch, Y0, num='2', net_name='/A', **PIN)],
            3: [make_pad(3, X0 + pitch, Y0, num='3', net_name='/B', **PIN)]}
    for p in extra_pads:
        pads.setdefault(p.net_id, []).append(p)
    return make_pcb(
        pads_by_net=pads,
        nets={1: Net(1, '+1V8'), 2: Net(2, '/A'), 3: Net(3, '/B'), 4: Net(4, '/H')},
        board_info=BoardInfo(layers={0: 'F.Cu'}, board_bounds=bounds,
                             copper_layers=['F.Cu']))


cfg = GridRouteConfig(layers=['F.Cu'], track_width=NARROW, clearance=CLR, grid_step=0.05)
cfg.power_net_widths = {1: NET_W}
coord = GridCoord(cfg.grid_step)
MARGIN = cfg.track_margins_for_net(1)


def rail():
    # away from the pin (free) -> into the pin centre; the pad is at the END
    return [make_seg(X0, 7.0, X0, 9.0, width=NET_W, net_id=1),
            make_seg(X0, 9.0, X0, Y0, width=NET_W, net_id=1)]


def run(pcb):
    obs = build_base_obstacle_map(pcb, cfg, nets_to_route=[1])
    return ser._assign_wide_route_widths(
        rail(), cfg, 1, obs, coord, ['F.Cu'], MARGIN, False, NARROW,
        neck_start=False, pcb_data=pcb), obs


def width_at(out, y):
    for s in out:
        lo, hi = sorted((s.start_y, s.end_y))
        if lo - 1e-9 <= y <= hi + 1e-9:
            return round(s.width, 4)
    return None


def worst_pad_slack(out, pcb):
    """min over shipped pieces and foreign pads of gap - w/2 - clearance."""
    worst = math.inf
    for s in out:
        for nid, plist in pcb.pads_by_net.items():
            if nid == 1:
                continue
            for p in plist:
                if p.pad_type == 'np_thru_hole':
                    continue
                g = segment_pad_gap(s.start_x, s.start_y, s.end_x, s.end_y, p)
                worst = min(worst, g - s.width / 2 - CLR)
    return worst


# The entry piece alongside the neighbours, at the pad's width.
ENTRY = make_seg(X0, 9.5, X0, Y0, width=NET_W, net_id=1)
Y_NEXT = 9.8                   # a point on the run beside both neighbours

print("1. precondition")
pcb = board()
obs = build_base_obstacle_map(pcb, cfg, nets_to_route=[1])
grid_ok = ser._segment_fits_wide(ENTRY, obs, coord, 0, MARGIN[0] + ser._OFFGRID_FIT_GUARD)
check("the grid refuses the entry piece", not grid_ok)
check("the exact check admits it (10 um of slack)",
      ser._neck_zone_exact_fits(ENTRY, pcb, 1, cfg))

print("2. the entry ships at the pad's width")
out, _ = run(pcb)
check("beside the neighbours the rail ships at 0.3", width_at(out, Y_NEXT) == NET_W,
      [(round(s.start_y, 3), round(s.end_y, 3), s.width) for s in out])
sl = worst_pad_slack(out, pcb)
check("every shipped piece clears the neighbours (check_drc measure)", sl >= -1e-6,
      f"worst slack {sl * 1000:.1f} um")
check("length conserved",
      abs(sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in out)
          - (Y0 - 7.0)) < 1e-6)

print("3. negative control: neighbours 10 um short of the rule")
pcb3 = board(pitch=0.48)
check("the exact check refuses the entry", not ser._neck_zone_exact_fits(ENTRY, pcb3, 1, cfg))
out3, _ = run(pcb3)
check("beside the neighbours the rail necks", width_at(out3, Y_NEXT) == NARROW,
      [(round(s.start_y, 3), round(s.end_y, 3), s.width) for s in out3])
sl3 = worst_pad_slack(out3, pcb3)
check("nothing ships grazing", sl3 >= -1e-6, f"worst slack {sl3 * 1000:.1f} um")

print("4. negative control: an NPTH hole within reach")
# A piece below the pin row, off the neighbours. Reach is the half-width +
# the widest floor (the 0.20 NPTH-to-track one here) + 2 cells = 0.45 mm.
FREE = make_seg(X0, 8.0, X0, 9.0, width=NET_W, net_id=1)
hole = make_pad(4, X0 + 0.8, 8.5, num='MH', size_x=0.8, size_y=0.8, shape='circle',
                layers=('*.Cu', '*.Mask'), drill=0.8, pad_type='np_thru_hole')
check("the hole is in reach, its wall 0.25 mm off the copper (floor 0.20)",
      abs((0.8 - 0.4) - NET_W / 2 - 0.25) < 1e-9)
check("without the hole the exact check admits the piece",
      ser._neck_zone_exact_fits(FREE, board(), 1, cfg))
check("with it the exact check stands aside",
      not ser._neck_zone_exact_fits(FREE, board(extra_pads=[hole]), 1, cfg))

print("5. negative control: the board edge within reach")
# The edge 0.39 mm off the centreline: the copper clears it by 0.24 (0.19 rule).
check("the exact check stands aside",
      not ser._neck_zone_exact_fits(FREE, board(bounds=(0.0, 0.0, X0 + 0.39, 20.0)),
                                    1, cfg))

if fails:
    print(f"\nFAILED: {fails}")
    sys.exit(1)
print("\nALL PASSED")
