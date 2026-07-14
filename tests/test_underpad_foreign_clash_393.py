#!/usr/bin/env python3
"""Issue #393: under-pad escape must not ship sibling via/track clashes.

On orangecrab U3 (0.5 mm pitch, 0.23 mm balls, 0.3/0.15 via) the under-pad
escape shipped a short: astar dropped the EXT_PLL+/- vias OFF-CENTRE (allowed
anywhere in the ball's own home disk) gated only by the via's single centre
cell on the destination layer, and the home-disk exemption let the later
IO_MOSI/IO_1 escapes route straight through those vias (adjacent balls' home
disks overlap, so copper committed in that lens was invisible). 9 kicad-
confirmed DRC items, including an EXT_PLL- <-> IO_1 short.

This pins the fixed invariant on the real trigger board: every via the
under-pad escape returns clears every FOREIGN returned track, via (ring and
drill hole-to-hole), and BGA ball pad at the routed clearance. Also guards
the escape rate so the fix rejects clashing sites rather than tanking the
escape (the EXT_PLL pair now fails honestly -- route_diff picks it up later
in the chain, which is how the pre-regression waves routed it).

    python3 tests/test_underpad_foreign_clash_393.py

Uses the stress-corpus board (~/Documents/kicad_stress_test); skips if absent.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

BOARD = os.path.expanduser(
    "~/Documents/kicad_stress_test/boards_unrouted_set3/orangecrab.kicad_pcb")
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu", "B.Cu"]
TRACK, CLEARANCE = 0.1, 0.1
VIA, DRILL = 0.3, 0.15
# Violations #393 shipped were 20-200 um deep; grid quantization is ~8 um.
TOL = 0.02


def _pt_seg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    if l2 <= 0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def main():
    print("=" * 60)
    print("Issue #393 under-pad sibling via/track clash regression test")
    print("=" * 60)
    if not os.path.exists(BOARD):
        print(f"  [SKIP] board not present: {BOARD}")
        return 0

    from kicad_parser import parse_kicad_pcb
    from bga_fanout import generate_bga_fanout
    from routing_defaults import HOLE_TO_HOLE_CLEARANCE

    pcb = parse_kicad_pcb(BOARD)
    fp = pcb.footprints["U3"]
    tracks, vias, _removed, failed = generate_bga_fanout(
        fp, pcb, net_filter=["*", "!GND", "!P3.3V"], layers=LAYERS,
        track_width=TRACK, clearance=CLEARANCE, via_size=VIA, via_drill=DRILL,
        escape_method="underpad")

    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")

    clashes = []
    for v in vias:
        vr, vdr = v["size"] / 2.0, v["drill"] / 2.0
        for t in tracks:
            if t["net_id"] == v["net_id"]:
                continue
            d = _pt_seg(v["x"], v["y"], t["start"][0], t["start"][1],
                        t["end"][0], t["end"][1])
            if d < vr + t["width"] / 2.0 + CLEARANCE - TOL:
                clashes.append(f"VIA-SEG net{v['net_id']} vs net{t['net_id']} "
                               f"d={d:.4f} at ({v['x']:.2f},{v['y']:.2f})")
        for w in vias:
            if w is v:
                continue
            d = math.hypot(v["x"] - w["x"], v["y"] - w["y"])
            if v["net_id"] != w["net_id"] and d < vr + w["size"] / 2.0 \
                    + CLEARANCE - TOL:
                clashes.append(f"VIA-VIA net{v['net_id']} vs net{w['net_id']} "
                               f"d={d:.4f}")
            if 0 < d < vdr + w["drill"] / 2.0 + HOLE_TO_HOLE_CLEARANCE - TOL:
                clashes.append(f"HOLE-HOLE net{v['net_id']} vs "
                               f"net{w['net_id']} d={d:.4f}")
        for p in fp.pads:
            if p.net_id == v["net_id"] or p.pad_type == "np_thru_hole":
                continue
            d = math.hypot(v["x"] - p.global_x, v["y"] - p.global_y)
            if d < vr + max(p.size_x, p.size_y) / 2.0 + CLEARANCE - TOL:
                clashes.append(f"VIA-PAD net{v['net_id']} vs pad "
                               f"net{p.net_id} d={d:.4f}")
    for c in clashes[:12]:
        print(f"    {c}")
    check("no foreign via/track/pad clash in the returned fanout (#393)",
          not clashes)

    escaped_nets = {t["net_id"] for t in tracks}
    print(f"    escaped nets: {len(escaped_nets)}, failed: {len(failed)}")
    check("escape rate held (>= 90 nets escaped, was 100/110 post-fix)",
          len(escaped_nets) >= 90)

    print("=" * 60)
    if fails:
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  under-pad fanout copper is mutually clearance-consistent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
