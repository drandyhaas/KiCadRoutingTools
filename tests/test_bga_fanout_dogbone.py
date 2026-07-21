#!/usr/bin/env python3
"""Regression test for issue #128: the dog-bone escape method.

  python3 tests/test_bga_fanout_dogbone.py

Dog-bone = the under-pad engine with every inner ball's escape via placed in
the diagonal inter-ball gap (ball -> 45 stub -> via) instead of in the pad,
staggered toward the escape side. This test runs it through the engine entry
point (generate_bga_fanout, escape_method='dogbone') on the dense ulx3s 22x22
BGA and checks:
  - escape parity: dog-bone escapes at least as many nets as under-pad,
  - every via sits in a GAP (off every ball pad, the #267 graze-kill),
  - every gap via has a same-net stub connecting it to its ball,
  - vias are mutually consistent (drill hole-to-hole, ring clearance),
  - via rings clear every foreign ball pad by the clearance.

Uses kicad_files/ulx3s.kicad_pcb; skips cleanly if absent.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb
from bga_fanout import generate_bga_fanout

BOARD = os.path.join(ROOT, "kicad_files", "ulx3s.kicad_pcb")
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
PARAMS = dict(track_width=0.12, clearance=0.1, via_size=0.35, via_drill=0.2)
H2H = 0.2


def run(pcb, method):
    fp = pcb.footprints["U1"]
    tracks, vias, _vr, failed = generate_bga_fanout(
        fp, pcb, layers=LAYERS, escape_method=method, **PARAMS)
    return tracks, vias, failed


def main():
    print("=" * 60)
    print("Issue #128 dog-bone escape regression test")
    print("=" * 60)
    if not os.path.exists(BOARD):
        print(f"  [SKIP] board not present: {BOARD}")
        return 0

    checks = []
    pcb = parse_kicad_pcb(BOARD)
    balls = list(pcb.footprints["U1"].pads)
    ball_r = max(max(p.size_x, p.size_y) for p in balls) / 2.0

    _t, u_vias, u_failed = run(pcb, 'underpad')
    u_nets = {t['net_id'] for t in _t}
    pcb = parse_kicad_pcb(BOARD)   # fresh: the first run mutates nothing, but
    balls = list(pcb.footprints["U1"].pads)   # keep the objects consistent
    tracks, vias, failed = run(pcb, 'dogbone')
    d_nets = {t['net_id'] for t in tracks}

    checks.append(("tracks were generated", len(tracks) > 0))
    checks.append(("escape parity with under-pad (nets)",
                   len(d_nets) >= len(u_nets)))
    checks.append(("no more failures than under-pad",
                   len(failed) <= len(u_failed)))

    # Every via sits in a gap: its CENTER is off every ball pad (#267 kill --
    # a via-in-pad would be ~0 from its own ball's centre).
    min_center = min(
        (min(math.hypot(v['x'] - p.global_x, v['y'] - p.global_y)
             for p in balls) for v in vias),
        default=1e9)
    checks.append(("every via centre is off the ball grid "
                   f"(min ball-centre dist {min_center:.3f} > ball r {ball_r:.3f})",
                   min_center > ball_r))

    # Every via ring clears every FOREIGN ball pad by the clearance.
    viol = 0
    for v in vias:
        for p in balls:
            if p.net_id == v['net_id']:
                continue
            pr = max(p.size_x, p.size_y) / 2.0
            if (math.hypot(v['x'] - p.global_x, v['y'] - p.global_y)
                    < v['size'] / 2.0 + pr + PARAMS['clearance'] - 1e-6):
                viol += 1
    checks.append((f"via rings clear foreign ball pads ({viol} violations)",
                   viol == 0))

    # Vias are mutually consistent: drill hole-to-hole (net-independent) and
    # ring clearance between foreign nets.
    vv = 0
    for i, a in enumerate(vias):
        for b in vias[i + 1:]:
            d = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
            if d < a['drill'] / 2.0 + b['drill'] / 2.0 + H2H - 1e-6:
                vv += 1
            elif (a['net_id'] != b['net_id']
                  and d < a['size'] / 2.0 + b['size'] / 2.0
                  + PARAMS['clearance'] - 1e-6):
                vv += 1
    checks.append((f"vias mutually consistent ({vv} violations)", vv == 0))

    # Every gap via connects to its ball: a same-net track endpoint at the via.
    ends = {}
    for t in tracks:
        for e in (t['start'], t['end']):
            ends.setdefault(t['net_id'], set()).add(
                (round(e[0], 3), round(e[1], 3)))
    orphans = [v for v in vias
               if (round(v['x'], 3), round(v['y'], 3))
               not in ends.get(v['net_id'], set())]
    checks.append((f"every via touches same-net fanout copper "
                   f"({len(orphans)} orphans)", len(orphans) == 0))

    print()
    ok = True
    for label, passed in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}")
        ok = ok and passed
    print()
    print(f"{sum(1 for _, p in checks if p)}/{len(checks)} checks passed")
    print("ALL PASS" if ok else "FAILURES")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
