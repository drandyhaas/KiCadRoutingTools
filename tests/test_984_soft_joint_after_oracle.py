#!/usr/bin/env python3
"""#984: soft joints the oracle legs lay are bridged, and a pair of ends whose
segments already meet is not a soft joint.

Two halves:

A. The detectors' "ONLY". A soft joint is a dangling end that reaches the rest
   of the net ONLY by cap-overlapping another (check_drc's own comment);
   check_drc and check_weird paired ANY two dangling ends within cap distance.
   sonde_xilinx GND shipped a "1.100 mm gap" between two stubs fanning out of
   ONE vertex -- an oracle strap re-tracing a region join from the join's own
   vertex -- which nothing hangs on. Both detectors now skip a pair whose
   segments share a strict root (connectivity.strict_joint_roots).

B. Ordering. close_soft_joints runs in the route step's cleanup, before the
   plane finalize; the finalize's oracle leg (and the #666 cap re-weld, the
   opt-in #678 weld and #589 re-audit) lays copper after it, so smartknob_base
   shipped a GND tap 0.111 mm short of its trunk. route.py's
   _late_soft_joint_bridge984 runs the same pass over the nets those legs
   touched, on the board the run ships: the file (CLI) or the write model
   (GUI).

    python3 tests/test_984_soft_joint_after_oracle.py
"""
import os
import re
import shutil
import sys
import tempfile
from types import SimpleNamespace as NS

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb, Segment       # noqa: E402
from kicad_writer import generate_segment_sexpr          # noqa: E402
from check_drc import run_drc                            # noqa: E402
from check_weird import _check_soft_joints               # noqa: E402
from routing_config import GridRouteConfig               # noqa: E402
from route import _late_soft_joint_bridge984             # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
NET = 'DPA_P'
failures = []


def check(label, got, want):
    ok = got == want
    print(f"  [{'ok' if ok else 'FAIL'}] {label}: got {got!r}, want {want!r}")
    if not ok:
        failures.append(label)


def board_with(segs, tmp):
    """cap_chain with `segs` [(x1, y1, x2, y2, w), ...] on DPA_P, B.Cu."""
    path = os.path.join(tmp, 'b.kicad_pcb')
    shutil.copy(BOARD, path)
    nid = next(n.net_id for n in parse_kicad_pcb(path).nets.values()
               if n.name == NET)
    with open(path, encoding='utf-8') as f:
        c = f.read()
    sx = [generate_segment_sexpr((a, b), (cc, d), w, 'B.Cu', nid, NET)
          for a, b, cc, d, w in segs]
    lp = c.rfind(')')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(c[:lp] + '\n'.join(sx) + '\n' + c[lp:])
    return path


def soft_joints_drc(path):
    v = run_drc(path, clearance=0.1, quiet=True, print_summary=False)
    return [x for x in v if x.get('type') == 'segment-endpoint-gap']


def soft_joints_weird(path):
    pcb = parse_kicad_pcb(path)
    nid = next(n.net_id for n in pcb.nets.values() if n.name == NET)
    found = []
    _check_soft_joints(nid, NET, [s for s in pcb.segments if s.net_id == nid],
                       [v for v in pcb.vias if v.net_id == nid],
                       pcb.pads_by_net.get(nid, []), found,
                       pcb.board_info.copper_layers)
    return found


# sonde_xilinx's shape, moved onto cap_chain: two stubs out of ONE vertex
# (12.5, 22.0), a 1.6 join stub and a 0.635 strap over it, whose free ends
# are 1.1 mm apart with caps overlapping 0.0175 mm; a third segment leaves
# the vertex so it is a real junction.
FAN = [(12.5, 22.0, 12.0, 22.0, 1.6), (12.5, 22.0, 10.9, 22.0, 0.635),
       (12.5, 22.0, 14.6, 20.0, 0.635)]
# smartknob's shape: a 0.8 trunk and a 0.2 tap whose ends are 0.111 apart,
# the two pieces joined by nothing else.
TAP = [(16.5, 21.7, 16.0, 22.2, 0.8), (16.6, 21.652, 16.048, 21.1, 0.2)]

tmp = tempfile.mkdtemp(prefix='t984_')
try:
    print("A. a pair whose segments already meet is not a soft joint")
    p = board_with(FAN, tmp)
    check("check_drc, stubs out of one vertex", len(soft_joints_drc(p)), 0)
    check("check_weird, stubs out of one vertex", len(soft_joints_weird(p)), 0)
    p = board_with(TAP, tmp)
    check("check_drc, tap short of its trunk", len(soft_joints_drc(p)), 1)
    check("check_weird, tap short of its trunk", len(soft_joints_weird(p)), 1)

    print("B. the post-oracle pass bridges it, on both fronts")
    cfg = GridRouteConfig(clearance=0.1, track_width=0.2, via_size=0.6,
                          via_drill=0.3, layers=['F.Cu', 'B.Cu'], grid_step=0.05)
    p = board_with(TAP, tmp)
    n_before = len(parse_kicad_pcb(p).segments)
    n = _late_soft_joint_bridge984(None, p, False, None, None, {NET}, cfg)
    check("CLI: connectors added", n, 1)
    check("CLI: written into the file",
          len(parse_kicad_pcb(p).segments), n_before + 1)
    check("CLI: check_drc now reports no soft joint", len(soft_joints_drc(p)), 0)
    check("CLI: a net outside the scope is left alone",
          _late_soft_joint_bridge984(None, board_with(TAP, tmp), False, None,
                                     None, {'DPB_P'}, cfg), 0)

    pcb = parse_kicad_pcb(board_with(TAP, tmp))
    nid = next(x.net_id for x in pcb.nets.values() if x.name == NET)

    def write_model(rd):
        segs, vias = {}, {}
        for s in pcb.segments:
            segs.setdefault(s.net_id, []).append(s)
        for v in pcb.vias:
            vias.setdefault(v.net_id, []).append(v)
        return segs, vias

    rd = {'results': []}
    n = _late_soft_joint_bridge984(pcb, None, True, rd, write_model, {NET}, cfg)
    added = [s for r in rd['results'] for s in r.get('new_segments') or []]
    check("GUI: connectors added", n, 1)
    check("GUI: on the applier's channel, tagged",
          [(r.get('cleanup'), len(r['new_segments'])) for r in rd['results']],
          [('soft_joint_bridge', 1)])
    check("GUI: mirrored into pcb_data",
          sum(1 for s in pcb.segments if s is added[0]), 1)
    check("GUI: on the net", added[0].net_id, nid)

    print("C. every oracle leg in route.py records the nets it laid copper on")
    src = open(os.path.join(ROOT, 'py_router', 'route.py'), encoding='utf-8').read()
    calls = re.findall(r'(_orc_cap|_orc|_orc678\(|_orc10) = (?:oracle_reconnect|_orc_fn10)\(|'
                       r'^\s*(_orc678)\(', src, flags=re.M)
    check("oracle_reconnect call sites in route.py", len(calls), 4)
    check("sites that record their nets",
          len(re.findall(r'_oracle_nets984\.update\(', src)), 4)
finally:
    shutil.rmtree(tmp, ignore_errors=True)

print()
if failures:
    print(f"FAIL: {len(failures)} check(s): {failures}")
    sys.exit(1)
print("PASS: test_984_soft_joint_after_oracle")
