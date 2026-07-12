#!/usr/bin/env python3
"""Regression test for issue #218: under-pad escape must fan dense power rails
that the caller did NOT exclude from the fanout.

  python3 tests/test_bga_fanout_underpad_dense_rail.py

The under-pad engine used to treat *any* net with >= plane_min_pads pads as a
plane and skip it, assuming it would tap a copper zone later. On real boards the
secondary power rails (cparti's +1V0/+1V8, daisho's V2P5/VREF) are ordinary,
high-pin-count nets with no plane - so their inner-core balls were silently left
unescaped and the rail could not connect downstream. The fix only skips a
high-pin net when the caller ALSO excluded it via the --nets filter (that
exclusion is what marks GND/+3V3/... as real planes).

Fixture: ulx3s U1 (dense 22x22 BGA). Its +2V5 rail has 8 pads (>= plane_min_pads
= 6) but is NOT one of the excluded plane nets, so every +2V5 ball must escape.
Skips cleanly if the board is absent.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb
from bga_fanout import generate_bga_fanout

BOARD = os.path.join(ROOT, "kicad_files", "ulx3s.kicad_pcb")
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
# Exclude the real planes (GND + the two densest rails) but keep +2V5 in the
# fanout - it is the dense rail whose inner balls issue #218 dropped.
NET_FILTER = ["*", "!GND", "!+3V3", "!+1V1"]
RAIL = "+2V5"


def main():
    print("=" * 60)
    print("Issue #218 under-pad dense-rail escape regression test")
    print("=" * 60)
    if not os.path.exists(BOARD):
        print(f"  [SKIP] board not present: {BOARD}")
        return 0

    pcb = parse_kicad_pcb(BOARD)
    fp = pcb.footprints["U1"]
    rail_pads = [p for p in fp.pads if p.net_name == RAIL and p.net_id]
    rail_nid = rail_pads[0].net_id if rail_pads else None
    print(f"  {RAIL}: {len(rail_pads)} pads on U1 (net_id={rail_nid})")

    tracks, vias, _vr, failed = generate_bga_fanout(
        fp, pcb, layers=LAYERS,
        track_width=0.12, clearance=0.1, via_size=0.35, via_drill=0.2,
        escape_method="underpad", net_filter=NET_FILTER,
    )

    checks = []
    checks.append((f"{RAIL} is a dense rail (>= 6 pads)", len(rail_pads) >= 6))
    checks.append(("no balls failed to escape", len(failed) == 0))

    escaped_nets = {t["net_id"] for t in tracks} | {v["net_id"] for v in vias}
    checks.append((f"{RAIL} rail is fanned (not skipped as a phantom plane)",
                   rail_nid in escaped_nets))
    # Every +2V5 ball must carry rail copper it can actually reach: an escape
    # via in the pad, or (escape priority, #129) a same-net strap on the
    # ball's own layer. The #218 regression this guards against was balls
    # silently left with NOTHING (phantom-plane skip).
    via_pts = [(v["x"], v["y"]) for v in vias if v["net_id"] == rail_nid]

    def _ball_connected(p):
        if any(abs(vx - p.global_x) < 0.3 and abs(vy - p.global_y) < 0.3
               for vx, vy in via_pts):
            return True
        pl = next((l for l in (p.layers or [])
                   if l.endswith(".Cu") and not l.startswith("*")), None)
        return any(
            t["net_id"] == rail_nid and (pl is None or t["layer"] == pl) and (
                (abs(t["start"][0] - p.global_x) < 0.3
                 and abs(t["start"][1] - p.global_y) < 0.3)
                or (abs(t["end"][0] - p.global_x) < 0.3
                    and abs(t["end"][1] - p.global_y) < 0.3))
            for t in tracks)

    escaped_balls = sum(_ball_connected(p) for p in rail_pads)
    checks.append((f"all {len(rail_pads)} {RAIL} balls carry reachable rail "
                   f"copper (escape via or #129 strap)",
                   escaped_balls == len(rail_pads)))
    # The excluded planes must still be skipped (kept as obstacles, not fanned).
    gnd_nid = next((p.net_id for p in fp.pads if p.net_name == "GND"), None)
    checks.append(("excluded GND plane is NOT fanned",
                   gnd_nid not in {v["net_id"] for v in vias}))

    print()
    for name, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    npass = sum(ok for _, ok in checks)
    print(f"\n{npass}/{len(checks)} checks passed")
    print("=" * 60)
    return 0 if npass == len(checks) else 1


if __name__ == "__main__":
    sys.exit(main())
