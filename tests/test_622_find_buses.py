#!/usr/bin/env python3
"""The bus step's buses found on the board (route_bus.find_buses, is_array, half_pairs, route_buses).

  python3 tests/test_622_find_buses.py

The step takes every pair of ball arrays sharing MIN_BUS_NETS nets or more the whole route admits, the larger array the
source, the largest bus first; a pair with a row part at one end is named and left to the router, as is every net on
both parts the whole route does not admit. On tracked boards this pins:

1. ball arrays told from row parts: orangecrab_ext_pll's ECP5 (U3) and DDR3 (U4) are arrays, its QFN (U9) is not;
   qfn_interior_pads' QFN with pads inside its ring is not;
2. orangecrab_ext_pll: the DDR3 bus U3 -> U4 taken, its address and command nets left with the resistor packs they
   pass through, the connector J1 named and not taken;
3. ulx3s: U1 -> U2 (an SDRAM in a TSOP) not taken -- no bus taken;
4. a pair is laid whole or not at all: a leg whose partner is on a third part as well is refused, both legs of a
   point-to-point pair kept, and a base name that is a net of its own is no pair; a net whose name's last part is
   another net's too is refused (the whole route names a net by it);
5. a board with no bus: route_buses hands the board on as it came, exit 1, the summary saying none was found.
"""
import contextlib
import json
import os
import sys
import tempfile
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import route_bus as rb  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

BOARDS = os.path.join(ROOT, 'kicad_files')


def board(name):
    path = os.path.join(BOARDS, name)
    if not os.path.isfile(path):
        raise SystemExit(f'BROKEN TEST: no board {path}')
    with contextlib.redirect_stdout(sys.stderr):
        return parse_kicad_pcb(path)


def net(name, *refs):
    return NS(name=name, pads=[NS(component_ref=r) for r in refs])


def main():
    print('=' * 60)
    print('the bus step\'s buses found on the board')
    fails = []
    # 1
    oc = board('orangecrab_ext_pll.kicad_pcb')
    got = {r: rb.is_array(oc.footprints[r]) for r in ('U3', 'U4', 'U9')}
    if got != {'U3': True, 'U4': True, 'U9': False}:
        fails.append(f'orangecrab_ext_pll arrays {got}, want U3 and U4 ball arrays and the QFN U9 not')
    qfn = board('qfn_interior_pads.kicad_pcb')
    if rb.is_array(qfn.footprints['U1']):
        fails.append('qfn_interior_pads U1, a QFN with pads inside its ring, read as a ball array')
    # 2
    found = rb.find_buses(oc)
    taken = [(d['src'], d['dest'], len(d['nets'])) for d in found if d['taken']]
    if taken != [('U3', 'U4', 23)]:
        fails.append(f'orangecrab_ext_pll buses taken {taken}, want the DDR3 bus U3 -> U4, 23 nets')
    else:
        left = found[0]['left']
        if left.get('RAM_A0') != 'also on RN5' or left.get('RAM_CK+') != 'also on RN6':
            fails.append(f'orangecrab_ext_pll: RAM_A0 left {left.get("RAM_A0")!r}, RAM_CK+ {left.get("RAM_CK+")!r}, '
                         f'want each with the resistor pack it passes through')
    j1 = [d for d in found if d['dest'] == 'J1']
    if not j1 or j1[0]['taken'] or 'no ball array' not in (j1[0]['why'] or ''):
        fails.append(f'orangecrab_ext_pll U3 -> J1: {j1}, want it named and not taken (J1 no ball array)')
    # 3
    ux = rb.find_buses(board('ulx3s.kicad_pcb'))
    if any(d['taken'] for d in ux) or not any(d['src'] == 'U1' and d['dest'] == 'U2' for d in ux):
        fails.append(f'ulx3s: {[(d["src"], d["dest"], d["taken"]) for d in ux]}, want U1 -> U2 named, nothing taken')
    # 4
    pcb = NS(nets={i: n for i, n in enumerate([
        net('/A_P', 'U1', 'U5'), net('/A_N', 'U1', 'U5', 'R9'), net('/B_P', 'U1', 'U5'), net('/B_N', 'U1', 'U5'),
        net('/C', 'U1', 'U5'), net('/DP', 'U1', 'U5'), net('/D', 'U1', 'U5'), net('/DN', 'U1', 'U9')])})
    got = rb.half_pairs(pcb, ['/A_P', '/B_P', '/B_N', '/C', '/DP'])
    if sorted(got) != ['A_P'] or 'A_N' not in got['A_P'] or 'R9' not in got['A_P']:
        fails.append(f'half pairs {got}, want A_P alone (its A_N on R9 as well); B whole, C no pair, DP no pair '
                     f'(D is a net of its own)')
    # ...and a net whose name's last part is another's too is refused (the zynq's ENABLE and TEST/ENABLE)
    pcb = NS(nets={i: n for i, n in enumerate([
        net('ENABLE', 'U1', 'U5'), net('TEST/ENABLE', 'U1', 'U5', 'RX5'), net('/X', 'U1', 'U5'), net('/s/Y', 'U1')])})
    got = rb.name_clashes(pcb, ['ENABLE', '/X'])
    if sorted(got) != ['ENABLE'] or 'TEST/ENABLE' not in got['ENABLE']:
        fails.append(f'name clashes {got}, want ENABLE alone, naming TEST/ENABLE')
    # 5
    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(BOARDS, 'qfn_interior_pads.kicad_pcb')
        out = os.path.join(td, 'out.kicad_pcb')
        code, grades = rb.route_buses(src, out, log=lambda *a: None)
        summ = os.path.join(td, 'out.bus', 'summary.json')
        if code != 1 or grades or not os.path.isfile(out) or not os.path.isfile(summ):
            fails.append(f'no bus: exit {code}, grades {grades}, OUT written {os.path.isfile(out)}, want exit 1 and '
                         f'the board handed on as it came with its summary')
        else:
            s = json.load(open(summ))
            if s['buses'] or s['found'] != [] or s['exit'] != 1:
                fails.append(f'no bus: summary {s}, want no bus run, none found, exit 1')
            if os.path.getsize(out) != os.path.getsize(src):
                fails.append('no bus: OUT is not the board as it came')
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: ball arrays told from row parts; the DDR3 bus taken, its resistor-pack nets and the connector left '
          'named; a TSOP at one end not taken; a pair whole or not at all; a clashing name refused; no bus, the board as '
          'it came')
    return 0


if __name__ == '__main__':
    sys.exit(main())
