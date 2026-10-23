#!/usr/bin/env python3
"""bga_fanout and qfn_fanout's under-pad escape measure pads exactly
(bga_fanout.reroute.PadFreeMap), as the surface fan does (found replaying
#1328's chains).

Both build their obstacle map with extra_clearance = track_width / 2 on top
of stamps that already carry the track's half-width -- the guard
check_line_clearance's nearest-cell sampling needs -- so every pad was held a
further half track width (plus cell rounding) away. On the recorded chains
bga_fanout refused end-jogs beside R89.1 / C95.1 (ottercast_audio) and a
pad-crossing repair beside U1.K1 (icepi_zero), and the under-pad escape
refused stub candidates beside C2.2 (mikoto_nrf52840), all of which clear
those pads exactly. PadFreeMap is the map without pad copper plus the pads it
would have stamped -- every pad of a net the map does not route, the BGA's own
balls included -- measured with check_drc.segment_pad_gap.

Checks (grid 0.1, track 0.15, clearance 0.09, as ottercast's bga_fanout):
  1. Precondition: a segment 20 um outside a foreign pad's rule is refused
     by the stamped map; PadFreeMap admits it.
  2. Negative control: 20 um inside the rule, PadFreeMap refuses it.
  3. A pad of a net the map routes (a fanned net) is not measured, as it
     was never stamped; one of any other net is -- a non-fanned ball too.
  4. The NPTH hole keep-out stays on the pad-free grid.
  5. segments_clear_of_pads routes a PadFreeMap through the exact check.

    python3 tests/test_1328_fanout_padfree_map.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'rust_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import BoardInfo, Net                              # noqa: E402
from routing_config import GridRouteConfig                           # noqa: E402
from obstacle_map import (build_base_obstacle_map, build_layer_map,  # noqa: E402
                          check_line_clearance)
from bga_fanout.reroute import PadFreeMap, segments_clear_of_pads    # noqa: E402
from synth import make_pad, make_pcb                                 # noqa: E402

RUN_ALL_TIMEOUT = 60
TW, CLR = 0.15, 0.09
NEED = CLR + TW / 2
FANNED, FOREIGN, BALL = 1, 2, 3
fails = []


def check(name, ok, detail=''):
    print(('PASS' if ok else 'FAIL') + f': {name}' + (f' -- {detail}' if detail else ''))
    if not ok:
        fails.append(name)


def board(pads):
    by = {}
    for p in pads:
        by.setdefault(p.net_id, []).append(p)
    return make_pcb(pads_by_net=by,
                    nets={0: Net(0, ''), FANNED: Net(FANNED, '/D0'),
                          FOREIGN: Net(FOREIGN, '/X'), BALL: Net(BALL, 'GND')},
                    board_info=BoardInfo(layers={0: 'F.Cu'}, board_bounds=(0, 0, 20, 20),
                                         copper_layers=['F.Cu']))


cfg = GridRouteConfig(layers=['F.Cu'], track_width=TW, clearance=CLR, grid_step=0.1)
LM = build_layer_map(cfg.layers)


def maps(pcb):
    old = build_base_obstacle_map(pcb, cfg, nets_to_route=[FANNED], extra_clearance=TW / 2)
    new = PadFreeMap(build_base_obstacle_map(pcb, cfg, nets_to_route=[FANNED],
                                             extra_clearance=TW / 2, stamp_pads=False),
                     pcb, {FANNED}, cfg)
    return old, new


def seg_beside(pad, slack):
    """A vertical segment beside `pad`'s right edge, NEED + slack from it."""
    x = pad.global_x + pad.size_x / 2 + NEED + slack
    return (x, pad.global_y - 1.0, x, pad.global_y + 1.0)


pad = make_pad(FOREIGN, 10.0, 10.0, ref='R89', num='1', size_x=0.8, size_y=0.8)

print("1. 20 um outside the rule")
pcb = board([pad])
old, new = maps(pcb)
s = seg_beside(pad, +0.02)
check("precondition: the stamped map refuses it", not check_line_clearance(old, *s, 0, cfg))
check("PadFreeMap admits it", new.line_clear(*s, 'F.Cu', cfg))

print("2. negative control: 20 um inside the rule")
s = seg_beside(pad, -0.02)
check("PadFreeMap refuses it", not new.line_clear(*s, 'F.Cu', cfg))

print("3. which pads are measured")
fan = make_pad(FANNED, 10.0, 10.0, ref='U1', num='A1', size_x=0.8, size_y=0.8)
ball = make_pad(BALL, 10.0, 10.0, ref='U1', num='A2', size_x=0.8, size_y=0.8,
                shape='circle')
_o, n_fan = maps(board([fan]))
_o, n_ball = maps(board([ball]))
s_in = seg_beside(fan, -0.05)
check("a fanned net's pad is not measured (it was never stamped)",
      n_fan.line_clear(*s_in, 'F.Cu', cfg))
check("a non-fanned ball is measured, round",
      not n_ball.line_clear(*seg_beside(ball, -0.05), 'F.Cu', cfg)
      and n_ball.line_clear(*seg_beside(ball, +0.02), 'F.Cu', cfg))

print("4. the NPTH hole keep-out stays on the grid")
hole = make_pad(0, 10.0, 10.0, ref='H1', num='', size_x=1.0, size_y=1.0, shape='circle',
                layers=('*.Cu', '*.Mask'), drill=1.0, pad_type='np_thru_hole')
_o, n_hole = maps(board([hole]))
check("a segment through the hole is refused",
      not n_hole.line_clear(10.0, 8.0, 10.0, 12.0, 'F.Cu', cfg))

print("5. segments_clear_of_pads")
segs_ok = [{'start': seg_beside(pad, +0.02)[:2], 'end': seg_beside(pad, +0.02)[2:]}]
segs_bad = [{'start': seg_beside(pad, -0.02)[:2], 'end': seg_beside(pad, -0.02)[2:]}]
check("it takes the exact verdict through a PadFreeMap",
      segments_clear_of_pads(segs_ok, 'F.Cu', new, cfg, LM)
      and not segments_clear_of_pads(segs_bad, 'F.Cu', new, cfg, LM))
check("...and the grid's on a plain map, as before",
      not segments_clear_of_pads(segs_ok, 'F.Cu', old, cfg, LM))

if fails:
    print(f"\nFAILED: {fails}")
    sys.exit(1)
print("\nALL PASSED")
