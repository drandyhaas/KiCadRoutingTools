#!/usr/bin/env python3
"""#1318: qfn_fanout lays a power net's stub at the net's width where it fits.

route.py keeps the copper a fanout laid, so a power pin escaped at the
call's --width shipped at it: on steppenprobe 7.6 mm of +1V8 and 6.9 mm of
+3V3 were qfn_fanout's 0.15 mm stubs under a 0.3 mm request, untouched by
the route step's pad-entry zone. qfn_fanout now takes route.py's
--power-nets / --power-nets-widths: each surface stub of such a net, on a
pad at least that wide, is laid leg by leg at the net's width wherever that
clears everything the stub test checks and every other net's stub as laid,
and at --width where it does not.

Checks:
  1. Unit (_widen_power_stub_legs), 0.4 mm pitch, a power pin between two
     signal pins: the straight escape widens; the 45-degree fan, 0.28 mm
     from its neighbour's, stays at --width; a leg the stub test refuses at
     the wide width stays narrow; a pad narrower than the rail widens
     nothing (negative control: the stub would ship under width anyway).
  2. End to end on tigard U3 (QFN-64, 0.5 mm, 0.25 mm pins), +3V3 and VREG
     at 0.25 mm: power legs widen, nothing else changes width or geometry,
     and every track clears every foreign pad and every other net's track
     by check_drc's measure. Without --power-nets the output is unchanged.

    python3 tests/test_1318_qfn_power_stub_width.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'rust_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import parse_kicad_pcb                          # noqa: E402
from check_drc import segment_pad_gap, _pad_has_no_copper         # noqa: E402
from geometry_utils import segment_to_segment_distance            # noqa: E402
from qfn_fanout import generate_qfn_fanout, _widen_power_stub_legs  # noqa: E402
from qfn_fanout.types import FanoutStub                           # noqa: E402
from synth import make_pad                                        # noqa: E402

RUN_ALL_TIMEOUT = 300
BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
fails = []


def check(name, ok, detail=''):
    print(('PASS' if ok else 'FAIL') + f': {name}' + (f' -- {detail}' if detail else ''))
    if not ok:
        fails.append(name)


# ---- 1. unit ----------------------------------------------------------------
print("1. _widen_power_stub_legs")
PITCH, TW, CLR, PW = 0.4, 0.1, 0.15, 0.25


def stub(net, x, pad_w=0.25, fan=-1):
    pad = make_pad(net, x, 0.0, num=str(net), size_x=pad_w, size_y=0.8)
    # straight escape 0.8 mm down, then a 45-degree fan away from the side's
    # centre (`fan` -1 left, +1 right), as qfn_fanout lays a row
    return FanoutStub(pad=pad, pad_pos=(x, 0.0), corner_pos=(x, 0.8),
                      stub_end=(x + fan * 1.0, 1.8), side='bottom')


def run(stubs, power, refuse=None):
    def grazes(a, b, nid, width=TW):
        return bool(refuse and refuse(a, b, nid, width))

    def reserve(a, b, nid, width=TW):
        return False
    return _widen_power_stub_legs(stubs, power, TW, CLR, grazes, reserve)


row = [stub(2, -PITCH), stub(1, 0.0), stub(3, PITCH, fan=+1)]
w = run(row, {1: PW})
check("the power pin's straight escape widens (0.225 mm to its neighbours' at 0.4 pitch)",
      w[id(row[1])][0] == PW, str(w[id(row[1])]))
check("its fan, parallel to its left neighbour's 0.283 mm away, stays at --width (0.108 < 0.15)",
      w[id(row[1])][1] == TW, str(w[id(row[1])]))
check("the signal stubs are untouched",
      w[id(row[0])] == (TW, TW, TW) and w[id(row[2])] == (TW, TW, TW))
row = [stub(2, -PITCH), stub(1, 0.0), stub(3, PITCH, fan=+1)]
w = run(row, {1: PW}, refuse=lambda a, b, nid, width: width > TW and a == (0.0, 0.0))
check("a leg the stub test refuses at the wide width stays at --width",
      w[id(row[1])][0] == TW, str(w[id(row[1])]))
row = [stub(2, -PITCH), stub(1, 0.0, pad_w=0.2), stub(3, PITCH, fan=+1)]
w = run(row, {1: PW})
check("negative control: a 0.2 mm pad under a 0.25 mm rail widens nothing",
      w[id(row[1])] == (TW, TW, TW), str(w[id(row[1])]))

# ---- 2. tigard U3 -------------------------------------------------------------
print("2. tigard U3, +3V3 and VREG at 0.25 mm")
if not os.path.exists(BOARD):
    print(f"FAIL: board missing ({BOARD})")
    sys.exit(1)
TW2, CLR2 = 0.1, 0.1


def fan(power=None):
    pcb = parse_kicad_pcb(BOARD)
    u3 = pcb.footprints['U3']
    nets = sorted({p.net_name for p in u3.pads if p.net_name and 'GND' not in p.net_name})
    tracks, _v, dropped = generate_qfn_fanout(
        u3, pcb, net_filter=nets, layer='F.Cu', track_width=TW2, clearance=CLR2,
        grid_step=0.05, escape_method='stub',
        power_nets=list(power) if power else None,
        power_nets_widths=[0.25] * len(power) if power else None)
    return pcb, tracks, dropped


def key(t):
    return (round(t['start'][0], 6), round(t['start'][1], 6),
            round(t['end'][0], 6), round(t['end'][1], 6), t['net_id'])


pcb0, base, d0 = fan()
pcb1, wide, d1 = fan(('+3V3', 'VREG'))
power_ids = {i for i, n in pcb1.nets.items() if n.name in ('+3V3', 'VREG')}
check("no --power-nets: every stub at --width", all(t['width'] == TW2 for t in base))
check("the same legs in both runs (only widths may change)",
      sorted(map(key, base)) == sorted(map(key, wide)) and d0 == d1)
widened = [t for t in wide if t['width'] > TW2 + 1e-9]
check("power legs widened to 0.25", bool(widened)
      and all(t['net_id'] in power_ids and abs(t['width'] - 0.25) < 1e-9 for t in widened),
      f"{len(widened)} of {sum(1 for t in wide if t['net_id'] in power_ids)} power legs")
worst = math.inf
for t in wide:
    for nid, plist in pcb1.pads_by_net.items():
        if nid == t['net_id']:
            continue
        for p in plist:
            if _pad_has_no_copper(p) or abs(p.global_x - t['start'][0]) > 3 \
                    or abs(p.global_y - t['start'][1]) > 3:
                continue
            worst = min(worst, segment_pad_gap(t['start'][0], t['start'][1],
                                               t['end'][0], t['end'][1], p) - t['width'] / 2)
    for o in wide:
        if o['net_id'] == t['net_id']:
            continue
        d = segment_to_segment_distance(t['start'][0], t['start'][1], t['end'][0],
                                        t['end'][1], o['start'][0], o['start'][1],
                                        o['end'][0], o['end'][1])
        worst = min(worst, d - t['width'] / 2 - o['width'] / 2)
check("every track clears every foreign pad and other net's track",
      worst >= CLR2 - 1e-9, f"worst gap {worst:.4f} mm (clearance {CLR2})")

if fails:
    print(f"\nFAILED: {fails}")
    sys.exit(1)
print("\nALL PASSED")
