#!/usr/bin/env python3
"""#1313: check_drc grades every `.kicad_dru` length window.

`(constraint length (min ..) (max ..))` bounds each matching net's routed
length, and KiCad's DRC reports `length_out_of_range` outside it. Nothing in
this tool read it: on imx6ulz (gp14 H21) KiCad reported 8 DRAM_LDQ nets out
of their 11.5-14.5 mm window while check_drc and board_score read clean.
check_drc now reads the windows through `kicad_dru.read_board_length_rules`,
measures each net with `net_match_length` (mode 'total', KiCad's length DRC
measure, as the installed kicad-cli's release measures it since #1432) and
reports a net outside its window as `length-out-of-range`; board_score counts
the rows in its `drc` component.

Fixture: routed_output.kicad_pcb copied with a `.kicad_dru` and a net class
BUS (pattern `Net-(U2A-BE_*`) added to its project. The windows are set from
the nets' own measured lengths, in file order:

  unmodelled  an `&&` condition: not modelled. Every net no later rule binds
              reaches it first, so it is NOT GRADED and named.
  bus class   A.NetClass == 'BUS', window around BE_1/BE_2: BE_0 is over.
  under       DATA_0 below its window.
  shadowed    DATA_1 at max 1 mm ...
  wins        ... and the LATER rule (max 1000 mm) is the one KiCad applies.
  wild        A.NetName == 'NET-(U2A-DATA_1?)', max 5 mm: KiCad's wildcard,
              case-insensitive -- every routed DATA_1x is over.
  ignored     (severity ignore): skipped with a note.
  unrouted    a net whose pads are not joined: reported not measured.

Checks: the violation set is exactly the expected nets with the right bound;
the console names the unmeasured net, the rule it could not model and the
ignored one; `--nets` scopes the grade; a board with no dru grades nothing new;
and board_score counts the rows in `drc`.

    python3 tests/test_1313_dru_length_window.py
"""
import contextlib
import io
import json
import os
import re
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from check_drc import run_drc                                   # noqa: E402
from copy_board import copy_board                               # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
import net_queries                                              # noqa: E402
from net_queries import net_match_length                        # noqa: E402
from run_utils import evidence                                  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
UNROUTED = '/fpga_adc/VA11'
# KiCad's own measure of these routed_output nets: the `actual` of its
# length_out_of_range reports (`pcb drc --severity-all`) on this fixture, per
# release -- kicad-cli 10.0.0 (2026-10-21) and 10.0.7 (2026-10-25, #1432: 11 of
# the 13 differ). The measure is pinned to each with the release forced. Most
# carry vias-in-pad whose tracks leave on an inner layer of the 10-layer,
# 1.59 mm stack; charging those vias the whole stack read BE_1 18.50 mm and
# DATA_13 17.77 mm.
KICAD_MM = {
    '10.0.0': {
        'Net-(U2A-BE_0)': 18.8847, 'Net-(U2A-BE_1)': 15.6614,
        'Net-(U2A-BE_2)': 15.2877, 'Net-(U2A-DATA_0)': 24.4315,
        'Net-(U2A-DATA_11)': 18.0903, 'Net-(U2A-DATA_12)': 24.1856,
        'Net-(U2A-DATA_13)': 15.1494, 'Net-(U2A-DATA_14)': 21.3243,
        'Net-(U2A-DATA_15)': 17.4839, 'Net-(U2A-DATA_16)': 15.6106,
        'Net-(U2A-DATA_17)': 16.8717, 'Net-(U2A-DATA_18)': 15.6413,
        'Net-(U2A-DATA_19)': 15.1546,
    },
    '10.0.7': {
        'Net-(U2A-BE_0)': 18.8453, 'Net-(U2A-BE_1)': 15.6462,
        'Net-(U2A-BE_2)': 15.2725, 'Net-(U2A-DATA_0)': 24.4163,
        'Net-(U2A-DATA_11)': 18.0903, 'Net-(U2A-DATA_12)': 24.1704,
        'Net-(U2A-DATA_13)': 15.3782, 'Net-(U2A-DATA_14)': 21.3091,
        'Net-(U2A-DATA_15)': 17.4687, 'Net-(U2A-DATA_16)': 15.6106,
        'Net-(U2A-DATA_17)': 16.8565, 'Net-(U2A-DATA_18)': 15.6261,
        'Net-(U2A-DATA_19)': 15.1394,
    },
}
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    out = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        return fn(*a, **k), out.getvalue()


def lengths(pcb):
    out = {}
    for nid, pads in pcb.pads_by_net.items():
        if nid and len(pads) >= 2:
            out[pcb.nets[nid].name] = net_match_length(pcb, nid, 'total')
    return out


def via_span_unit():
    """kicad_via_length: the barrel over the layers the via joins, as 10.0.0
    and as 10.0.7 charge it."""
    from kicad_parser import (BoardInfo, Net, Pad, PCBData, Segment,
                              StackupLayer, Via)
    from net_queries import kicad_via_length
    layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
    stack = []
    for i, n in enumerate(layers):
        stack.append(StackupLayer(name=n, layer_type='copper', thickness=0.035))
        if i < 3:
            stack.append(StackupLayer(name=f'd{i}', layer_type='core', thickness=0.5))
    info = BoardInfo(layers=dict(enumerate(layers)), copper_layers=list(layers))
    info.stackup = stack
    pcb = PCBData(board_info=info, nets={1: Net(net_id=1, name='N')}, footprints={},
                  segments=[], vias=[], pads_by_net={})
    via = Via(x=0.0, y=0.0, size=0.3, drill=0.15, layers=['F.Cu', 'B.Cu'], net_id=1)
    pad = Pad(pad_number='1', net_id=1, net_name='N', shape='rect', global_x=0.0,
              global_y=0.0, local_x=0.0, local_y=0.0, size_x=0.5, size_y=0.5,
              layers=['F.Cu'], drill=0.0, pad_type='smd', component_ref='U1')

    def seg(layer, x0):
        return Segment(start_x=x0, start_y=0.0, end_x=5.0, end_y=0.0, width=0.1,
                       layer=layer, net_id=1)
    # 10.0.0 charges the stackup between the joined layers, both copper layers
    # whole, and joins only track ends exactly at the centre; 10.0.7 counts
    # half of an INNER copper layer that ends the span (10.0.2) and joins any
    # end inside the via pad (10.0.4).
    fb = 4 * 0.035 + 3 * 0.5
    f1_00, f1_07 = 0.035 + 0.5 + 0.035, 0.035 + 0.5 + 0.035 / 2
    f2_07 = 0.035 + 0.5 + 0.035 + 0.5 + 0.035 / 2
    i12_00, i12_07 = 0.035 + 0.5 + 0.035, 0.035 / 2 + 0.5 + 0.035 / 2
    cases = [   # label, segments, pads, (10.0.0, 10.0.7)
        ('via-in-pad, track on In1.Cu: F.Cu to In1.Cu (10.0.7: half of In1.Cu)',
         [seg('In1.Cu', 0.0)], [pad], (f1_00, f1_07)),
        ('tracks on F.Cu and B.Cu: the whole stack',
         [seg('F.Cu', 0.0), seg('B.Cu', 0.0)], [], (fb, fb)),
        ('tracks on In1.Cu and In2.Cu (10.0.7: half of each)',
         [seg('In1.Cu', 0.0), seg('In2.Cu', 0.0)], [], (i12_00, i12_07)),
        ('a track ending 0.04 mm off the centre adds nothing in 10.0.0, its '
         'layer in 10.0.7 (inside the via pad)',
         [seg('In1.Cu', 0.0), seg('In2.Cu', 0.04)], [pad], (f1_00, f2_07)),
        ('a track ending 0.2 mm off the centre, outside the via pad, adds nothing',
         [seg('In1.Cu', 0.0), seg('In2.Cu', 0.2)], [pad], (f1_00, f1_07)),
        ('one joined layer charges nothing', [seg('In1.Cu', 0.0)], [], (0.0, 0.0)),
    ]
    for label, segs, pads, wants in cases:
        for release, want in zip(('10.0.0', '10.0.7'), wants):
            got = kicad_via_length(via, segs, pads, pcb, release=release)
            check(f'via span {release}: {label}', abs(got - want) < 1e-9,
                  f'{got:.4f} vs {want:.4f}')


def main():
    via_span_unit()
    src, _ = quiet(parse_kicad_pcb, evidence(BOARD, 'routed_output board'))
    for release, pins in KICAD_MM.items():
        net_queries.use_length_measure_release(release)
        try:
            L = lengths(src)
        finally:
            net_queries.use_length_measure_release(None)
        off = {n: (L.get(n), k) for n, k in pins.items()
               if L.get(n) is None or f'{L[n]:.4f}' != f'{k:.4f}'}
        check(f"net_match_length measures these nets as KiCad {release}'s "
              f"length DRC does, at its 4 decimals", not off, str(off))
    # The windows below are set from the measure this machine grades with.
    L = lengths(src)
    be1, be2 = L['Net-(U2A-BE_1)'], L['Net-(U2A-BE_2)']
    lo, hi = min(be1, be2) - 0.5, max(be1, be2) + 0.5
    wild = sorted(n for n, v in L.items() if v is not None
                  and re.fullmatch(r'net-\(u2a-data_1.\)', n, re.I))
    check('fixture: the nets the windows are set from are routed',
          all(L.get(n) is not None for n in ('Net-(U2A-BE_0)', 'Net-(U2A-BE_1)',
                                             'Net-(U2A-BE_2)', 'Net-(U2A-DATA_0)',
                                             'Net-(U2A-DATA_1)'))
          and L['Net-(U2A-BE_0)'] > hi and L['Net-(U2A-DATA_0)'] < 30
          and len(wild) >= 3 and all(L[n] > 5 for n in wild),
          f'BE window {lo:.2f}..{hi:.2f}, BE_0 {L["Net-(U2A-BE_0)"]:.2f}, '
          f'{len(wild)} routed DATA_1x')
    check('fixture: the unrouted net has pads not joined by track copper',
          UNROUTED in L and L[UNROUTED] is None)

    dru = f"""(version 1)
(rule "unmodelled"
  (condition "A.NetName == 'Net-(U2A-CLK)' && A.Type == 'track'")
  (constraint length (max 1mm)))
(rule "bus class"
  (condition "A.NetClass == 'BUS'")
  (constraint length (min {lo:.4f}mm) (opt {(lo + hi) / 2:.4f}mm) (max {hi:.4f}mm)))
(rule "under"
  (condition "A.NetName == 'Net-(U2A-DATA_0)'")
  (constraint length (min 30mm) (max 40mm)))
(rule "shadowed"
  (condition "A.NetName == 'Net-(U2A-DATA_1)'")
  (constraint length (max 1mm)))
(rule "wins"
  (condition "A.NetName == 'Net-(U2A-DATA_1)'")
  (constraint length (max 1000mm)))
(rule "wild"
  (condition "A.NetName == 'NET-(U2A-DATA_1?)'")
  (constraint length (max 5mm)))
(rule "ignored"
  (severity ignore)
  (condition "A.NetName == 'Net-(U2A-DATA_2*'")
  (constraint length (max 1mm)))
(rule "unrouted"
  (condition "A.NetName == '{UNROUTED}'")
  (constraint length (max 1mm)))
"""
    with tempfile.TemporaryDirectory() as tmp:
        board = os.path.join(tmp, 'ro.kicad_pcb')
        quiet(copy_board, BOARD, board)
        with open(os.path.join(tmp, 'ro.kicad_dru'), 'w') as fh:
            fh.write(dru)
        pro = os.path.join(tmp, 'ro.kicad_pro')
        doc = json.load(open(evidence(pro, 'copied project')))
        ns = doc['net_settings']
        bus = dict(ns['classes'][0], name='BUS')
        ns['classes'].append(bus)
        ns['netclass_patterns'] = [{'netclass': 'BUS', 'pattern': 'Net-(U2A-BE_*'}]
        with open(pro, 'w') as fh:
            json.dump(doc, fh, indent=2)

        rows, log = quiet(run_drc, board, clearance=0.1, max_print=0,
                          check_sizes=False)
        got = {v['net1']: v['bound'] for v in rows
               if v.get('type') == 'length-out-of-range'}
        want = {'Net-(U2A-BE_0)': 'max', 'Net-(U2A-DATA_0)': 'min'}
        want.update({n: 'max' for n in wild})
        check('the out-of-window nets, and only they, are flagged, with their bound',
              got == want, f'got {sorted(got.items())}')
        check('the later of two matching rules is the one applied (DATA_1)',
              'Net-(U2A-DATA_1)' not in got)
        check('a class rule binds its members and spares the in-window ones',
              'Net-(U2A-BE_1)' not in got and 'Net-(U2A-BE_2)' not in got
              and got.get('Net-(U2A-BE_0)') == 'max')
        check('the row carries the length, the rule and KiCad\'s name',
              all(v['rule'] and v['length_mm'] > 0 for v in rows
                  if v.get('type') == 'length-out-of-range')
              and 'length_out_of_range' in log, '')
        check('the unrouted net is reported not measured, not graded',
              UNROUTED not in got and 'not measured' in log and UNROUTED in log)
        check("the rule it cannot model is named NOT GRADED",
              re.search(r"NOT GRADED: rule 'unmodelled' \(.+\) -- \d+ net", log)
              is not None)
        check('the severity-ignore rule is skipped with a note',
              "rule 'ignored': length window, severity ignore" in log)

        rows, _ = quiet(run_drc, board, clearance=0.1, max_print=0,
                        check_sizes=False, net_patterns=['Net-(U2A-BE_0)'])
        scoped = [v['net1'] for v in rows if v.get('type') == 'length-out-of-range']
        check('--nets scopes the grade', scoped == ['Net-(U2A-BE_0)'], str(scoped))

        bare = os.path.join(tmp, 'bare.kicad_pcb')
        quiet(copy_board, BOARD, bare)
        rows, log = quiet(run_drc, bare, clearance=0.1, max_print=0,
                          check_sizes=False)
        check('a board with no .kicad_dru grades no length window',
              not any(v.get('type') == 'length-out-of-range' for v in rows)
              and 'Length rules' not in log)

        s = subprocess.run([sys.executable, '-X', 'utf8', 'py_tools/board_score.py',
                            board, '--json', os.path.join(tmp, 's.json')],
                           cwd=ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        check('board_score ran to a verdict',
              'Traceback' not in s.stdout + s.stderr and 'SCORE_JSON' in s.stdout,
              (s.stdout + s.stderr)[-300:])
        sd = json.load(open(evidence(os.path.join(tmp, 's.json'), 'score json')))
        n = sd['components']['drc']['by_type'].get('length-out-of-range')
        check('board_score counts the rows in its drc component', n == len(want),
              f'{n} vs {len(want)}')

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
