#!/usr/bin/env python3
"""The net rescue's bare-ball escape stays out of keep-out rule areas (#1778).

The #666 fanout-rescue escape (generate_bga_fanout, net-scoped) places on maps
of its own, and the escape was judged against the NET-FILTERED keep-outs only
(net_keepouts.audit): a board rule area with `(vias not_allowed)` or
`(tracks not_allowed)` and no net filter was never asked, so the escape could
ship a via KiCad reports as items_not_allowed. It now asks
`net_rescue.escape_keepout_refusal`, which is `check_drc.copper_keepout_gate`
(both kinds of area, copper grown by half its width / the via's radius).

  1. escape_keepout_refusal on the issue's board (rule area KEEPOUT1 over
     U1, F.Cu and B.Cu, tracks and vias forbidden, no net filter): a via
     inside and a track inside are refused, a via whose copper only reaches
     in is refused, copper clear of the area passes; with vias ALLOWED in the
     area the via passes and the track is still refused;
  2. a net-filtered keep-out still refuses its bound net only;
  3. wiring: the bare-ball escape in rescue_failed_nets asks it before the
     escape's copper is added to the board.

    python3 tests/test_1778_rescue_escape_keepout.py
"""
import ast
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb, Segment, Via            # noqa: E402
from net_rescue import escape_keepout_refusal                     # noqa: E402
import net_keepouts                                               # noqa: E402

BOARD = os.path.join(TESTS_DIR, 'fixtures', '1778', 'via-ko.kicad_pcb')
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def via(x, y, nid, size=0.6):
    return Via(x=x, y=y, size=size, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=nid)


def seg(x1, y1, x2, y2, nid, layer='F.Cu', w=0.2):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w, layer=layer,
                   net_id=nid)


def rule_area():
    print('1. a board rule area (KEEPOUT1: 11..13.2 x 11.3..13.2, F.Cu+B.Cu)')
    pcb = parse(BOARD)
    gnd = next(i for i, n in pcb.nets.items() if n.name == 'GND')
    ok('the fixture carries the area', any(k.get('name') == 'KEEPOUT1'
                                           for k in pcb.board_info.keepouts))
    why = escape_keepout_refusal(pcb, [], [via(12.0, 12.0, gnd)])
    ok('escape via inside refused', 'KEEPOUT1' in why and 'vias not allowed' in why, why)
    ok('escape via reaching in refused (centre 0.2 outside, radius 0.3)',
       bool(escape_keepout_refusal(pcb, [], [via(12.0, 13.4, gnd)])))
    ok('escape via clear passes', escape_keepout_refusal(pcb, [], [via(12.0, 13.6, gnd)]) == '')
    why = escape_keepout_refusal(pcb, [seg(12.0, 12.0, 12.0, 14.0, gnd)], [])
    ok('escape track inside refused', 'tracks not allowed' in why, why)
    ok('escape track clear passes',
       escape_keepout_refusal(pcb, [seg(14.0, 12.0, 15.0, 12.0, gnd)], []) == '')
    tmp = tempfile.mkdtemp(prefix='t1778r_')
    try:
        t = open(BOARD, encoding='utf-8', newline='').read()
        allowed = os.path.join(tmp, 'v.kicad_pcb')
        t2 = t.replace('(vias not_allowed)', '(vias allowed)')
        assert t2 != t, 'BROKEN TEST: the vias-allowed edit changed nothing'
        open(allowed, 'w', encoding='utf-8', newline='').write(t2)
        pv = parse(allowed)
        ok('vias allowed: the via passes',
           escape_keepout_refusal(pv, [], [via(12.0, 12.0, gnd)]) == '')
        ok('vias allowed: the track is still refused',
           bool(escape_keepout_refusal(pv, [seg(12.0, 12.0, 12.0, 14.0, gnd)], [])))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return pcb, gnd


def net_filtered(pcb, gnd):
    print('2. a net-filtered keep-out binds its net only')
    s1 = next(i for i, n in pcb.nets.items() if n.name == 'S1')
    net_keepouts.attach(pcb, [{
        'name': 'NK', 'nets': frozenset({'S1'}), 'layers': ['F.Cu', 'B.Cu'],
        'polygon': [(30, 5), (34, 5), (34, 9), (30, 9)], 'holes': [],
        'tracks_allowed': False, 'vias_allowed': False}])
    why = escape_keepout_refusal(pcb, [], [via(32, 7, s1)])
    ok('bound net refused', 'NK' in why, why)
    ok('other net passes', escape_keepout_refusal(pcb, [], [via(32, 7, gnd)]) == '')
    net_keepouts.attach(pcb, None)


def wiring():
    print('3. the bare-ball escape asks it before shipping')
    src = open(os.path.join(ROOT, 'py_router', 'net_rescue.py'), encoding='utf-8').read()
    fn = next(n for n in ast.walk(ast.parse(src))
              if isinstance(n, ast.FunctionDef) and n.name == 'rescue_failed_nets')
    body = ast.get_source_segment(src, fn)
    ask = body.find('escape_keepout_refusal(pcb_data, _gsegs, _gvias)')
    ship = body.find('pcb_data.vias.extend(_gvias)')
    ok('the escape calls escape_keepout_refusal', ask >= 0)
    ok('before its copper is added to the board', 0 <= ask < ship, f'{ask} vs {ship}')


def main():
    pcb, gnd = rule_area()
    net_filtered(pcb, gnd)
    wiring()
    if failures:
        print(f'FAILED: {len(failures)}: {failures}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
