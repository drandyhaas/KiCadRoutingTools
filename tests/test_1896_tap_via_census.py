#!/usr/bin/env python3
"""#1896: how many plane-net pads each tap via serves, counted on the board a
step ships.

A pad reaches its plane through a via of its own or through a neighbour's,
joined to it by a surface track. The plane repair logged `reused via at ...`
per pad, and nothing counted how many pads sit on one via. A count kept by
the repair's tap loop would know only the pads that loop tapped: a fan-out
dogbone, a strap to another pad's copper or the guaranteed-join gate's track
would be missed, and a strap's "reused via" position is a pad centre, not a
via. So the count is read off the copper (`tap_via_census`): every via a pad
reaches through the copper on its own layer, whoever laid it.

Rows:
  1. the census on a board built here: a dogbone, two pads on one via, a via
     in a pad with a strap from a second pad, a through-hole pad, a pad with
     no copper, a via no pad reaches, a blind via that does not reach the
     pour, and another net's via under a pad;
  2. repair_planes.py on tests/fixtures/1842/plane_under (8 VDD pads on F.Cu,
     a VDD pour on In1.Cu, no through-via site under the BGA): the repair taps
     J1.16 and J1.21 and the join gate runs a surface track from the four
     balls to J1.21's via, so VDD is complete with 2 vias. JSON_SUMMARY.tap_vias
     counts exactly the board's 2 VDD vias, with the four balls on J1.21's;
  3. route.py's finalize on the same board: `--json-out` keeps the record (its
     reconciliation sub-run does not clear it) and so does the log
     (`merge_route_summaries`); it counts the vias the board has (J1.22's
     strap to J1.21's pad is logged as a "reused via" at a pad centre where
     no via stands); and the GUI path (`return_results`) reads the same
     record off the board it will apply.
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence, tool  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1842', 'plane_under')
FAILS = []

BOARD = """(kicad_pcb (version 20240108) (generator pcbnew)
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal)
    (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "") (net 1 "VDD") (net 2 "SIG")
  (gr_rect (start 0 0) (end 40 20) (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))
  (footprint "t:U" (layer "F.Cu") (at 10 10)
    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at -8 0) (size 1 1) (layers "F.Cu") (net 1 "VDD"))
    (pad "2" smd rect (at -4 0) (size 1 1) (layers "F.Cu") (net 1 "VDD"))
    (pad "3" smd rect (at -4 4) (size 1 1) (layers "F.Cu") (net 1 "VDD"))
    (pad "4" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "VDD"))
    (pad "5" smd rect (at 0 4) (size 1 1) (layers "F.Cu") (net 1 "VDD"))
    (pad "6" smd rect (at 4 0) (size 1 1) (layers "F.Cu") (net 1 "VDD"))
    (pad "7" thru_hole circle (at 8 0) (size 1.6 1.6) (drill 0.8) (layers "*.Cu") (net 1 "VDD"))
    (pad "8" smd rect (at 12 0) (size 1 1) (layers "F.Cu") (net 1 "VDD"))
    (pad "9" smd rect (at 16 0) (size 1 1) (layers "F.Cu") (net 1 "VDD")))
  (segment (start 2 10) (end 2 12) (width 0.2) (layer "F.Cu") (net 1))
  (segment (start 6 10) (end 8 12) (width 0.2) (layer "F.Cu") (net 1))
  (segment (start 6 14) (end 8 12) (width 0.2) (layer "F.Cu") (net 1))
  (segment (start 10 14) (end 10 10) (width 0.2) (layer "F.Cu") (net 1))
  (segment (start 14 10) (end 14 12) (width 0.2) (layer "F.Cu") (net 1))
  (segment (start 18 10) (end 18 12) (width 0.2) (layer "F.Cu") (net 1))
  (segment (start 22 10) (end 22 12) (width 0.2) (layer "F.Cu") (net 1))
  (via (at 2 12) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))
  (via (at 8 12) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))
  (via (at 10 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))
  (via (at 14 12) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))
  (via (at 18 12) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))
  (via (at 30 18) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))
  (via blind (at 22 12) (size 0.6) (drill 0.3) (layers "F.Cu" "In1.Cu") (net 1))
  (via (at 26 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 2))
  (zone (net 1) (net_name "VDD") (layer "In2.Cu") (hatch edge 0.5)
    (connect_pads (clearance 0.2)) (min_thickness 0.2)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 0 0) (xy 40 0) (xy 40 20) (xy 0 20))))
)
"""


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, tag):
    sub = os.path.join(td, tag)
    os.makedirs(sub)
    for ext in ('.kicad_pcb', '.kicad_pro'):
        shutil.copy(evidence(FIX + ext, 'the fixture'),
                    os.path.join(sub, 'plane_under' + ext))
    return os.path.join(sub, 'plane_under.kicad_pcb'), os.path.join(sub, 'out.kicad_pcb')


def vdd_vias(path):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(evidence(path, 'the written board'))
    vdd = next(n.net_id for n in pcb.nets.values() if n.name == 'VDD')
    return sorted([round(v.x, 4), round(v.y, 4)] for v in pcb.vias if v.net_id == vdd)


def census_rows(td):
    print("1. the census on a board built here")
    from kicad_parser import parse_kicad_pcb
    from tap_via_census import tap_via_census, tap_via_line
    p = os.path.join(td, 'census.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(BOARD)
    pcb = parse_kicad_pcb(p)
    got = tap_via_census(pcb, ['VDD'])
    print(f"    {got}")
    shared = {tuple(r['at']): r['pads'] for r in got['shared']}
    ok(shared == {(8.0, 12.0): ['U1.2', 'U1.3'], (10.0, 10.0): ['U1.4', 'U1.5']},
       "two pads on one dogbone via, and a pad strapped to a via in another "
       f"pad, are the shared vias: {shared}")
    ok(got['vias'] == 4 and got['pads'] == 6,
       "U1.1's and U1.6's dogbones are tap vias of one pad each; the through-"
       "hole pad, the pad with no copper, the via no pad reaches, the blind "
       "via that stops short of the pour and the other net's via are not "
       f"counted: vias {got['vias']}, pads {got['pads']}")
    ok(got['shared_vias'] == 2 and got['max_pads_per_via'] == 2
       and got['pads_on_shared_vias'] == 4, f"the tallies: {got}")
    line = tap_via_line(got) or ''
    ok(line.startswith('Tap vias: 4 tap via(s) serve 6 plane-net pad(s); 2 serve '
                       'more than one pad') and 'U1.2+U1.3 at (8.00, 12.00)' in line,
       f"the console line: {line}")
    ok(tap_via_census(pcb, ['SIG'])['vias'] == 0,
       "a net with no pour has no tap vias")


def repair_row(td):
    print("2. repair_planes.py: the board it writes")
    board, out = stage(td, 'repair')
    r = check(PY + [tool('repair_planes.py'), board, out, '--nets', 'VDD',
                    '--plane-layers', 'In1.Cu', '--clearance', '0.1'],
              accept=True, timeout=1800)
    log = (r.stdout or '') + (r.stderr or '')
    doc = {}
    for l in log.splitlines():
        if l.startswith('JSON_SUMMARY: '):
            doc = json.loads(l[len('JSON_SUMMARY: '):])
    tv = doc.get('tap_vias') or {}
    vias = vdd_vias(out)
    ok(len(vias) == 2 and tv.get('vias') == len(vias),
       f"the record counts the board's VDD vias: {tv.get('vias')} vs {vias}")
    balls = {'U1.E5', 'U1.E6', 'U1.F5', 'U1.F6'}
    on = [r_ for r_ in tv.get('shared') or [] if balls <= set(r_['pads'])]
    ok(tv.get('pads') == 8 and on and on[0]['pads'][:2] == ['J1.21', 'J1.22'],
       "the four balls the join gate's track joins to J1.21's via are counted on "
       f"that via, with J1.22's tap: {tv.get('shared')}")
    ok(not doc.get('unconnected_pads'),
       f"and VDD ships complete: {doc.get('unconnected_pads')}")
    ok('Tap vias: 2 tap via(s) serve 8 plane-net pad(s)' in log,
       "the console says so")


GUI_DRIVER = """import json, os, sys
sys.path.insert(0, os.path.join(sys.argv[1], 'py_router'))
from route import batch_route
r = batch_route(sys.argv[2], '', ['VDD'], clearance=0.1, skip_routing=True,
                return_results=True)
print('GUI_TAP_VIAS: ' + json.dumps(r[3].get('tap_vias')))
"""


def route_rows(td):
    print("3. route.py's plane finalize: the board it ships")
    env = dict(os.environ, KICAD_FINALIZE_ONLY='1')
    board, out = stage(td, 'route')
    js = os.path.join(td, 'route.json')
    r = check(PY + [tool('route.py'), board, out, '--nets', 'VDD', '--clearance', '0.1',
                    '--skip-routing', '--json-out', js],
              accept=True, timeout=1800, env=env)
    with open(evidence(js, 'the --json-out summary'), encoding='utf-8') as fh:
        tv = json.load(fh).get('tap_vias') or {}
    vias = vdd_vias(out)
    ok(tv.get('vias') == len(vias) >= 1 and tv.get('pads') == 8,
       f"--json-out keeps the record, and it counts the board's vias: {tv} vs {vias}")
    from route_summary import merge_route_summaries
    ok((merge_route_summaries(r.stdout or '') or {}).get('tap_vias') == tv,
       "and a reader of the log alone gets the same record (JSON_SHIPPED_AUDIT)")
    print("   the GUI path (return_results)")
    driver = os.path.join(td, 'gui.py')
    with open(driver, 'w', encoding='utf-8') as fh:
        fh.write(GUI_DRIVER)
    board2, _o = stage(td, 'gui')
    g = check(PY + [driver, REPO, board2], accept=True, timeout=1800, env=env)
    gtv = None
    for l in (g.stdout or '').splitlines():
        if l.startswith('GUI_TAP_VIAS: '):
            gtv = json.loads(l[len('GUI_TAP_VIAS: '):])
    ok(gtv == tv, f"results_data carries the same record: {gtv}")


def main():
    with tempfile.TemporaryDirectory() as td:
        census_rows(td)
        repair_row(td)
        route_rows(td)
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
