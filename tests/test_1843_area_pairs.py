#!/usr/bin/env python3
"""#1843: a coupled pair keeps a tightened rule area between P and N.

KiCad lets a custom clearance rule outrank a pair's implicit gap rule, so a
rule area whose clearance is above a pair's gap grades P against N at the
area's value wherever the pair runs in it (kicad-cli 10.0.6: 13 P<->N
violations on the probe below before this). A coupled pair is laid at one
gap, so `area_clearance.pair_gap_plan` decides per pair:

  * the area lies on the span of the pair's pads: the pair is built at the
    area's value for the run, disclosed on the console and in
    JSON_SUMMARY['area_pair_gaps'] (its impedance changes);
  * otherwise the pair is kept out of the area (a net-filtered keep-out),
    at its own gap.

The board is the bundled lvds_converter_dualclk (pairs /CLK and /DATA from
J1 to IC4, R3 and R4), with a rule area ROOM on F.Cu and B.Cu and a rule
`clearance 0.4mm` in it, against the run's 0.2 mm (gap 0.2):

  1. the plan, on synthetic geometry: 'widened' across a pair's span,
     'around' off it, nothing without a tightened area;
  2. ROOM across the pairs' span (x 85.5-90.5, y 55-70): both pairs are
     built at 0.4 and routed, and after route.py finishes the board (the
     chain) check_drc grades it clean at KiCad's tolerance;
  3. ROOM off their span (x 100-108, y 52-58): both are kept out of it at
     their own gap, and route; no pair copper lies in the area.
"""
import json
import os
import re
import shutil
import sys
import tempfile
import uuid
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def plan_unit():
    print("1. the plan")
    from shapely.geometry import box
    from area_clearance import AreaClearance, Region, pair_gap_plan
    area = AreaClearance([Region('r', 'ROOM', 0.4, box(4.0, 0.0, 6.0, 10.0),
                                 frozenset(['F.Cu']), 1.0)], ['F.Cu'], split_reach=0.2)

    def pad(x, y):
        return SimpleNamespace(global_x=x, global_y=y, size_x=0.5, size_y=0.5,
                               rect_rotation=0.0, layers=['F.Cu'])
    pcb = SimpleNamespace(pads_by_net={1: [pad(1, 5), pad(9, 5)], 2: [pad(1, 6), pad(9, 6)],
                                       3: [pad(1, 12), pad(9, 12)], 4: [pad(1, 13), pad(9, 13)]})
    pairs = [('A', SimpleNamespace(p_net_id=1, n_net_id=2)),
             ('B', SimpleNamespace(p_net_id=3, n_net_id=4))]
    rows = {r['pair']: r['how'] for r in pair_gap_plan(area, pcb, pairs, lambda p: 0.2)}
    ok(rows == {'A': 'widened', 'B': 'around'},
       f"widened across a pair's span, around off it: {rows}")
    ok(pair_gap_plan(area, pcb, pairs, lambda p: 0.4) == [],
       "nothing for a pair already at the area's value")
    ok(pair_gap_plan(None, pcb, pairs, lambda p: 0.2) == [], "nothing without an area")


def stage(td, poly):
    board = os.path.join(td, 'in.kicad_pcb')
    check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'),
                evidence(BOARD, 'the bundled board'), board], accept=True)
    with open(board, encoding='utf-8') as fh:
        text = fh.read()
    pts = ' '.join(f'(xy {x} {y})' for x, y in poly)
    zone = (f'\t(zone (layers "F.Cu" "B.Cu") (uuid "{uuid.uuid4()}") (name "ROOM") '
            f'(hatch edge 0.5) (connect_pads (clearance 0)) (min_thickness 0.25)\n'
            f'\t\t(keepout (tracks allowed) (vias allowed) (pads allowed) '
            f'(copperpour allowed) (footprints allowed))\n'
            f'\t\t(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
            f'\t\t(polygon (pts {pts})))\n')
    i = text.rstrip().rfind(')')
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write(text[:i] + zone + text[i:])
    with open(os.path.join(td, 'in.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write("(version 1)\n(rule \"room\"\n  (constraint clearance (min 0.4mm))\n"
                 "  (condition \"A.insideArea('ROOM')\"))\n")
    return board


def route_diff(td, board):
    out = os.path.join(td, 'diff.kicad_pcb')
    r = check(PY + [os.path.join(REPO, 'py_router', 'route_diff.py'), board, out,
                    '--nets', '/CLK*', '/DATA*', '--clearance', '0.2'],
              accept=True, timeout=900)
    log = (r.stdout or '') + (r.stderr or '')
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', log, re.M)
    if m is None:
        raise AssertionError("BROKEN TEST: route_diff printed no JSON_SUMMARY\n" + log[-1500:])
    return evidence(out, 'the diff-pair board'), json.loads(m.group(1)), log


def drc_clean(board):
    r = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'),
                    evidence(board, 'the routed board'), '--clearance', '0.2',
                    '--clearance-margin-mm', '0.0005'], accept=True)
    return 'NO DRC VIOLATIONS FOUND' in (r.stdout or ''), r.stdout or ''


def main():
    plan_unit()
    with tempfile.TemporaryDirectory() as td:
        print("2. a room across the pairs' span")
        sub = os.path.join(td, 'across')
        os.mkdir(sub)
        board = stage(sub, [(85.5, 55), (90.5, 55), (90.5, 70), (85.5, 70)])
        out, doc, log = route_diff(sub, board)
        rows = {r['pair']: (r['how'], r['built_gap'], r['routed'])
                for r in doc.get('area_pair_gaps') or []}
        ok(rows == {'/CLK': ('widened', 0.4, True), '/DATA': ('widened', 0.4, True)},
           f"both pairs are built at the room's value and route: {rows}")
        ok('its impedance changes' in log, "and the run says so")
        chain = os.path.join(sub, 'chain.kicad_pcb')
        check(PY + [os.path.join(REPO, 'py_router', 'route.py'), out, chain,
                    '--clearance-ceiling', '0.2'], accept=True, timeout=900)
        clean, text = drc_clean(chain)
        ok(clean, "the chain's board is clean at KiCad's tolerance"
           + ('' if clean else '\n' + text[-1200:]))

        print("3. a room off their span")
        sub = os.path.join(td, 'off')
        os.mkdir(sub)
        board = stage(sub, [(100, 52), (108, 52), (108, 58), (100, 58)])
        out, doc, log = route_diff(sub, board)
        rows = {r['pair']: (r['how'], r['built_gap'], r['routed'])
                for r in doc.get('area_pair_gaps') or []}
        ok(rows == {'/CLK': ('around', 0.2, True), '/DATA': ('around', 0.2, True)},
           f"both pairs keep their own gap, kept out of the room, and route: {rows}")
        from shapely.geometry import LineString, box
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(out)
        room = box(100, 52, 108, 58)
        inside = [pcb.nets[s.net_id].name for s in pcb.segments
                  if pcb.nets[s.net_id].name.startswith(('/CLK', '/DATA'))
                  and LineString([(s.start_x, s.start_y), (s.end_x, s.end_y)])
                  .buffer(s.width / 2.0).intersects(room)]
        ok(not inside, f"no pair copper is in the room: {inside}")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
