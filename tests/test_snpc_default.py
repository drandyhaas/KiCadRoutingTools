#!/usr/bin/env python3
"""Vias stay off same-net SMD pads by default; exposed thermal pads excepted.

An open via in an SMD pad wicks solder away from the joint during reflow, so a
board without a recorded --same-net-pad-clearance gets
SAME_NET_PAD_CLEARANCE_DEFAULT. A recorded -1 (filled vias) or 0 is honoured.
An exposed thermal pad (route_planes.is_thermal_pad) stays open to vias either
way: its via array (#487) is how the part sheds heat.
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'py_router'))

import routing_defaults  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from obstacle_map import snpc_exempt_pad, same_net_pad_via_keepout_cells  # noqa: E402
from protected_nets import (persist_same_net_pad_clearance,  # noqa: E402
                            read_same_net_pad_clearance)
from routing_config import GridCoord, GridRouteConfig  # noqa: E402

QFN = ['(kicad_pcb (version 20240108) (generator pcbnew)',
       '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))',
       '  (net 0 "") (net 1 "GND") (net 2 "SIG")',
       '  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))',
       '  (footprint "t:QFN-16-1EP_3x3mm" (layer "F.Cu") (at 10 10)',
       '    (property "Reference" "U1" (at 0 -3) (layer "F.SilkS"))']
for i in range(16):
    side, k = divmod(i, 4)
    off = -0.75 + 0.5 * k
    x, y = [(-1.45, off), (off, 1.45), (1.45, -off), (-off, -1.45)][side]
    net = '(net 1 "GND")' if i == 0 else '(net 2 "SIG")'
    QFN.append(f'    (pad "{i + 1}" smd rect (at {x} {y}) (size 0.3 0.3) (layers "F.Cu" "F.Paste" "F.Mask") {net})')
QFN += ['    (pad "17" smd rect (at 0 0) (size 1.7 1.7) (layers "F.Cu" "F.Mask") (net 1 "GND")))', ')']


def _board(td):
    p = os.path.join(td, 'b.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(QFN))
    return parse_kicad_pcb(p), p


def test_default_and_recorded_values():
    with tempfile.TemporaryDirectory() as td:
        def fresh(name):                 # one file per value: the reader caches by file stamp
            pro = os.path.join(td, name + '.kicad_pro')
            with open(pro, 'w', encoding='utf-8') as fh:
                json.dump({}, fh)
            return pro
        assert read_same_net_pad_clearance(fresh('a')) == routing_defaults.SAME_NET_PAD_CLEARANCE_DEFAULT > 0
        for name, value in (('allow', -1.0), ('active', 0.2)):   # -1: filled vias, allowed
            pro = fresh(name)
            assert persist_same_net_pad_clearance(pro, value, verbose=False)
            assert read_same_net_pad_clearance(pro) == value


def test_exposed_pad_exempt_signal_pad_not():
    with tempfile.TemporaryDirectory() as td:
        pcb, _ = _board(td)
        pads = {p.pad_number: p for p in pcb.footprints['U1'].pads}
        assert snpc_exempt_pad(pads['17'], pcb)                 # the exposed pad
        assert not snpc_exempt_pad(pads['1'], pcb)              # a GND signal pad


def test_keepout_covers_pins_not_the_exposed_pad():
    with tempfile.TemporaryDirectory() as td:
        pcb, _ = _board(td)
        cfg = GridRouteConfig(track_width=0.15, clearance=0.15, via_size=0.4, via_drill=0.2,
                              grid_step=0.05, layers=['F.Cu', 'B.Cu'])
        cfg.same_net_pad_clearance = routing_defaults.SAME_NET_PAD_CLEARANCE_DEFAULT
        cells = {tuple(c) for c in same_net_pad_via_keepout_cells(pcb, 1, cfg)}
        coord = GridCoord(cfg.grid_step)
        pads = {p.pad_number: p for p in pcb.footprints['U1'].pads}
        assert coord.to_grid(pads['1'].global_x, pads['1'].global_y) in cells
        assert coord.to_grid(pads['17'].global_x, pads['17'].global_y) not in cells


def test_ep_named_footprint_leads_not_exempt():
    # SOIC-8-1EP: the name says exposed pad and the 0.6 x 1.5 mm leads pass the
    # 0.5 mm floor, but only the largest pad (the EP) is exempt
    lines = ['(kicad_pcb (version 20240108) (generator pcbnew)',
             '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))',
             '  (net 0 "") (net 1 "GND")',
             '  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))',
             '  (footprint "t:SOIC-8-1EP_3.9x4.9mm" (layer "F.Cu") (at 10 10)',
             '    (property "Reference" "U2" (at 0 -4) (layer "F.SilkS"))']
    for i in range(8):
        x, y = (-2.7, -1.905 + 1.27 * i) if i < 4 else (2.7, 1.905 - 1.27 * (i - 4))
        lines.append(f'    (pad "{i + 1}" smd rect (at {x} {y}) (size 1.5 0.6) (layers "F.Cu") (net 1 "GND"))')
    lines += ['    (pad "9" smd rect (at 0 0) (size 2.4 3.1) (layers "F.Cu") (net 1 "GND")))', ')']
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'soic.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write('\n'.join(lines))
        pcb = parse_kicad_pcb(path)
        pads = {p.pad_number: p for p in pcb.footprints['U2'].pads}
        assert snpc_exempt_pad(pads['9'], pcb)
        assert not any(snpc_exempt_pad(pads[str(n)], pcb) for n in range(1, 9))


TESTS = [test_default_and_recorded_values, test_exposed_pad_exempt_signal_pad_not,
         test_ep_named_footprint_leads_not_exempt,
         test_keepout_covers_pins_not_the_exposed_pad]

if __name__ == '__main__':
    for t in TESTS:
        t()
        print(f"PASS {t.__name__}")
