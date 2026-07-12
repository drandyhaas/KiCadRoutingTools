#!/usr/bin/env python3
"""Issue #356: qfn_fanout escapes must clear the fanned part's OWN pads.

The escape graze scan excluded every pad of the footprint being fanned
(`component_ref != footprint.reference`), and the obstacle map excludes all
fanned nets -- so a stub threading a dense pad field (hex_gateway U1, an
AQFN-73 inner grid) shipped 0.006mm from a NEIGHBOUR pad of the same part
with no check at all. The scan now includes own-part pads; the per-stub
same-net skip still exempts the stub's own pad(s).

Two checks on kicad_files/tigard.kicad_pcb (QFN-64, 0.5mm):
  1. Behavior preserved: the full stub fanout still escapes every net with no
     own-pad graze (perimeter escapes run perpendicular, away from neighbours).
  2. Regression: a synthetic same-footprint different-net pad injected into one
     pad's escape path must not be grazed by any emitted track -- pre-fix the
     scan couldn't see it and the escape shipped straight through it.

Skips cleanly if the corpus board is absent.
"""
import copy
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from kicad_parser import parse_kicad_pcb
from qfn_fanout import generate_qfn_fanout
from bga_fanout.reroute import _seg_hits_pad

BOARD = os.path.join(ROOT, "kicad_files", "tigard.kicad_pcb")
TRACK_W, CLEARANCE = 0.1, 0.1
MARGIN = CLEARANCE + TRACK_W / 2


def _own_pad_grazes(tracks, pads, layer):
    hits = []
    for t in tracks:
        for p in pads:
            if p.net_id == t["net_id"]:
                continue
            if p.drill <= 0 and layer not in (p.layers or []):
                continue
            if _seg_hits_pad(t["start"][0], t["start"][1],
                             t["end"][0], t["end"][1], p, margin=MARGIN):
                hits.append((t, p.pad_number, p.net_name))
    return hits


def main():
    if not os.path.exists(BOARD):
        print(f"SKIP: corpus board not found ({BOARD})")
        return 0
    fails = []

    # --- 1. full fanout: everything escapes, nothing grazes an own pad ---
    pcb = parse_kicad_pcb(BOARD)
    u3 = pcb.footprints["U3"]
    nets = sorted({p.net_name for p in u3.pads if p.net_name and "GND" not in p.net_name})
    tracks, _vias, dropped = generate_qfn_fanout(
        u3, pcb, net_filter=nets, layer="F.Cu",
        track_width=TRACK_W, clearance=CLEARANCE, grid_step=0.05,
        escape_method="stub")
    if dropped:
        fails.append(f"clean fanout dropped nets: {dropped}")
    grazes = _own_pad_grazes(tracks, u3.pads, "F.Cu")
    if grazes:
        fails.append(f"{len(grazes)} own-pad graze(s) in clean fanout: {grazes[:3]}")

    # --- 2. injected own pad in the escape path is respected ---
    # U3 pad 2 (Net-(C23-Pad1)) sits at (44.55, 57.15) on the left side, so its
    # stub escapes in -x. Park a same-footprint GND pad 0.95mm out on that path.
    pcb2 = parse_kicad_pcb(BOARD)
    u3b = pcb2.footprints["U3"]
    victim_net = "Net-(C23-Pad1)"
    donor = next(p for p in u3b.pads if p.net_name == victim_net)
    gnd_id = next(p.net_id for p in u3b.pads if p.net_name == "GND")
    blocker = copy.copy(donor)
    blocker.pad_number = "X356"
    blocker.net_id = gnd_id
    blocker.net_name = "GND"
    blocker.shape = "circle"
    blocker.size_x = blocker.size_y = 0.3
    blocker.global_x = donor.global_x - 0.95
    blocker.global_y = donor.global_y
    # Register only in pads_by_net (the graze scan's source); keeping it out of
    # footprint.pads leaves the QFN layout analysis untouched.
    pcb2.pads_by_net.setdefault(gnd_id, []).append(blocker)

    tracks2, _v2, _d2 = generate_qfn_fanout(
        u3b, pcb2, net_filter=[victim_net], layer="F.Cu",
        track_width=TRACK_W, clearance=CLEARANCE, grid_step=0.05,
        escape_method="stub")
    grazes2 = _own_pad_grazes(tracks2, [blocker], "F.Cu")
    if grazes2:
        fails.append(f"escape shipped through injected own pad: {grazes2}")

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print(f"PASS: {len(nets)} nets escape with no own-pad graze; "
          f"injected same-footprint pad is respected (dropped/shortened)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
