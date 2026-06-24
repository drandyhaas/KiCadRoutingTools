#!/usr/bin/env python3
"""Issue #164: qfn_fanout --escape-method underpad drops a clearing via per pad.

Escapes a fine-pitch QFN diff pair (tigard U3, QFN-64 0.5mm, /USB_DP+/USB_DN)
by dropping a staggered through-via just past each pad instead of the surface
45-degree fan. Asserts both halves escape and the two different-net vias clear.

Uses kicad_files/tigard.kicad_pcb; skips cleanly if absent.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from kicad_parser import parse_kicad_pcb
from qfn_fanout import generate_qfn_fanout

BOARD = os.path.join(ROOT, "kicad_files", "tigard.kicad_pcb")

VIA_SIZE, CLEARANCE = 0.45, 0.1


def main():
    if not os.path.exists(BOARD):
        print(f"SKIP: corpus board not found ({BOARD})")
        return 0

    pcb = parse_kicad_pcb(BOARD)
    u3 = pcb.footprints["U3"]
    tracks, vias, dropped = generate_qfn_fanout(
        u3, pcb, net_filter=["/USB_DP", "/USB_DN"], layer="F.Cu",
        track_width=0.1, clearance=CLEARANCE, grid_step=0.05,
        escape_method="underpad", via_size=VIA_SIZE, via_drill=0.25)

    fails = []
    if dropped:
        fails.append(f"dropped (should escape): {dropped}")
    if len(vias) != 2:
        fails.append(f"expected 2 escape vias, got {len(vias)}")
    # every via is a through-hole on the net, with a stub feeding it
    net_ids = {v["net_id"] for v in vias}
    for v in vias:
        if v["layers"] != ["F.Cu", "B.Cu"]:
            fails.append(f"via not through-hole: {v['layers']}")
    if len(net_ids) != 2:
        fails.append(f"expected both pair halves to escape, got nets {net_ids}")
    if not any(t["net_id"] in net_ids for t in tracks):
        fails.append("no stub feeding the escape vias")
    # the two different-net vias must clear (centre-to-centre >= via + clearance)
    for i in range(len(vias)):
        for j in range(i + 1, len(vias)):
            if vias[i]["net_id"] == vias[j]["net_id"]:
                continue
            d = math.hypot(vias[i]["x"] - vias[j]["x"], vias[i]["y"] - vias[j]["y"])
            if d < VIA_SIZE + CLEARANCE - 1e-6:
                fails.append(f"vias too close: {d:.3f} < {VIA_SIZE + CLEARANCE}")

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print(f"PASS: pair escaped as {len(vias)} staggered through-vias, "
          f"min different-net spacing OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
