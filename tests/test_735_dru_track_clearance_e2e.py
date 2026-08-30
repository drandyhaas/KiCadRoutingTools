#!/usr/bin/env python3
"""End-to-end (#735): a .kicad_dru TRACK-scoped clearance rule is honored by the
router and graded by check_drc, with no CLI flag and no GUI wiring.

Board: splitflap_driver, class 'CRIT' = /LED_* (netclass_patterns in a staged
.kicad_pro), track rule 0.45 mm, routed at clearance 0.2.

  1. WITH the dru: engine announces the rule; every routed CRIT segment keeps
     >= ~0.45 mm edge-to-edge from every foreign segment; the board grades
     clean under check_drc's auto-read; all nets still route (the raise must
     not strand pads).
  2. WITHOUT the dru (control): the same nets routed plain come closer than
     0.45 somewhere -- proving the rule actually changed the copper (the
     c99ffb4 inert-wiring lesson).
"""
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

# #522 reorg + skill merge: engine -> py_router/, placer -> py_placer/,
# board_score.py -> the placement-and-routing skill. Without these roots the
# imports below raise and the test never runs (it reports as a failure while
# asserting nothing).
_R522 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p522 in ('py_router', 'py_placer',
              os.path.join('.claude', 'skills',
                           'plan-pcb-placement-and-routing', 'scripts')):
    _d522 = os.path.join(_R522, _p522)
    if _d522 not in sys.path:
        sys.path.insert(0, _d522)


BOARD = os.path.join(REPO, "kicad_files", "splitflap_driver.kicad_pcb")
RULE = 0.45
DRU = ('(version 1)\n(rule crit_space (condition "A.Type==\'track\' && '
       'B.Type==\'track\' && A.NetClass==\'CRIT\'") '
       f'(constraint clearance (min {RULE}mm)))\n')
PRO = {"net_settings": {
    "classes": [{"name": "Default", "clearance": 0.2},
                {"name": "CRIT", "clearance": 0.2}],
    "netclass_patterns": [{"pattern": "/LED_*", "netclass": "CRIT"}]}}
# Foreign nets whose pads sit 1.3-1.7 mm from the LED pads (measured on the
# fixture) -- close enough that a plain 0.2-clearance route comes well under
# the 0.45 rule, so the control actually proves the rule moved copper.
NETS = ["/LED_*", "/LOOPBACK_*", "/MOTOR_C_*", "Net-(D?-Pad2)"]

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


def route(workdir, with_dru, out_name):
    src = os.path.join(workdir, ("in_dru" if with_dru else "in_plain") + ".kicad_pcb")
    from copy_board import copy_board
    copy_board(BOARD, src)
    open(os.path.splitext(src)[0] + ".kicad_pro", "w").write(json.dumps(PRO))
    if with_dru:
        open(os.path.splitext(src)[0] + ".kicad_dru", "w").write(DRU)
    out = os.path.join(workdir, out_name)
    js = out + ".json"
    r = subprocess.run(
        [sys.executable, os.path.join(REPO, 'py_router', 'route.py'), src, out,
         "--nets", *NETS, "--layers", "F.Cu", "B.Cu",
         "--clearance", "0.2", "--track-width", "0.2", "--json-out", js],
        capture_output=True, text=True, timeout=1200)
    summary = json.load(open(js)) if os.path.isfile(js) else {}
    return out, r, summary


def min_crit_edge_gap(board):
    """Min edge-to-edge distance between a CRIT (/LED_*) segment and any
    foreign segment on the same layer."""
    from kicad_parser import parse_kicad_pcb
    from check_drc import _seg_seg_dist_coords
    pcb = parse_kicad_pcb(board)
    crit = {nid for nid, n in pcb.nets.items()
            if n.name and n.name.startswith('/LED_')}
    best = float('inf')
    for a in pcb.segments:
        if a.net_id not in crit:
            continue
        for b in pcb.segments:
            if b.net_id in crit or b.net_id == 0 or b.layer != a.layer:
                continue
            d = _seg_seg_dist_coords(a.start_x, a.start_y, a.end_x, a.end_y,
                                     b.start_x, b.start_y, b.end_x, b.end_y)
            gap = d - a.width / 2 - b.width / 2
            if gap < best:
                best = gap
    return best


with tempfile.TemporaryDirectory() as d:
    out_dru, r1, s1 = route(d, True, "out_dru.kicad_pcb")
    check("route (dru) completed", r1.returncode == 0 and os.path.isfile(out_dru),
          f"rc={r1.returncode}\n{r1.stdout[-800:]}")
    check("engine announced the track rule",
          "Track-to-track clearance rule" in r1.stdout and "CRIT" in r1.stdout)
    check("all nets still route under the raise",
          s1.get('failed') == 0, f"failed={s1.get('failed')}")

    gap1 = min_crit_edge_gap(out_dru)
    check(f"CRIT copper keeps >= ~{RULE} to foreign tracks",
          gap1 >= RULE * 0.95, f"min gap {gap1:.3f}")

    dr = subprocess.run(
        [sys.executable, os.path.join(REPO, 'py_router', 'check_drc.py'), out_dru,
         "--no-size-checks"],
        capture_output=True, text=True, timeout=600)
    check("check_drc grades the ruled board clean", dr.returncode == 0,
          (dr.stdout + dr.stderr)[-600:])
    check("checker announced the track rule",
          "Track-to-track clearance rules" in dr.stdout)

    out_plain, r2, s2 = route(d, False, "out_plain.kicad_pcb")
    check("route (plain) completed", r2.returncode == 0)
    gap2 = min_crit_edge_gap(out_plain)
    check(f"control run comes closer than {RULE} (the rule changed copper)",
          gap2 < RULE, f"min gap {gap2:.3f}")

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
