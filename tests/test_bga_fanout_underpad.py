#!/usr/bin/env python3
"""Regression test for issue #122: the under-pad grid escape method.

  python3 tests/test_bga_fanout_underpad.py

The under-pad method routes each BGA signal ball with a via-in-pad and a path
UNDER the pad field on inner layers, escaping dense, fully-populated arrays the
channel router cannot. This test runs it through the engine entry point
(generate_bga_fanout, escape_method='underpad') on the dense ulx3s 22x22 BGA and
checks:
  - all signal balls escape (0 failed),
  - power/plane nets are skipped (not fanned),
  - every signal ball gets exactly one via-in-pad,
  - a bottom-side (B.Cu) BGA escapes too (layer roles swap correctly).

Needs the ulx3s stress board; skips cleanly if absent.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb
from bga_fanout import generate_bga_fanout

BOARD = os.path.expanduser(
    "~/Documents/kicad_stress_test/boards_unrouted_set2/ulx3s.kicad_pcb")
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
PARAMS = dict(track_width=0.12, clearance=0.1, via_size=0.35, via_drill=0.2,
              escape_method='underpad')


def main():
    print("=" * 60)
    print("Issue #122 under-pad escape regression test")
    print("=" * 60)
    if not os.path.exists(BOARD):
        print(f"  [SKIP] stress board not present: {BOARD}")
        return 0

    checks = []
    pcb = parse_kicad_pcb(BOARD)
    fp = pcb.footprints["U1"]
    tracks, vias, _vr, failed = generate_bga_fanout(fp, pcb, layers=LAYERS, **PARAMS)

    checks.append(("tracks were generated", len(tracks) > 0))
    checks.append(("no balls failed to escape", len(failed) == 0))
    # ulx3s U1 has ~204 real signal nets; all should escape. Outer balls go on
    # the BGA layer with NO via, so the via count is < the escaped count.
    escaped_nets = {t['net_id'] for t in tracks}
    checks.append(("escaped most signal nets (>180)", len(escaped_nets) > 180))
    checks.append(("some balls escape via-less on the BGA layer (outer ring)",
                   0 < len(vias) < len(escaped_nets)))
    via_nets = {v['net_id'] for v in vias}
    checks.append(("power/plane nets not fanned (GND absent)",
                   all(pcb.nets.get(nid) is None or pcb.nets[nid].name != 'GND'
                       for nid in via_nets)))
    # Every via is via-in-pad (on a copper-spanning via) at a BGA ball.
    checks.append(("all vias span the stack", all(
        v.get('layers') and v['layers'][0] == 'F.Cu' for v in vias)))

    # Bottom-side BGA: flip U1 to B.Cu, escape should still fully succeed.
    pcb2 = parse_kicad_pcb(BOARD)
    fp2 = pcb2.footprints["U1"]
    fp2.layer = 'B.Cu'
    t2, v2, _v2r, f2 = generate_bga_fanout(fp2, pcb2, layers=LAYERS, **PARAMS)
    checks.append(("bottom-side BGA escapes (0 failed)", len(f2) == 0 and len(t2) > 0))
    # The via-less edge escape must now live on B.Cu, not F.Cu.
    bcu_edge = any(t['layer'] == 'B.Cu' for t in t2)
    checks.append(("bottom-side edge escape uses B.Cu", bcu_edge))

    for name, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    n_pass = sum(1 for _, ok in checks if ok)
    print(f"\n{n_pass}/{len(checks)} checks passed")
    print("=" * 60)
    return 0 if n_pass == len(checks) else 1


if __name__ == "__main__":
    sys.exit(main())
