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

Grazes are graded by check_drc's segment_pad_gap on the pad's true copper.
The square-cornered `_seg_hits_pad` box this used to grade with flagged a fan
that clears the injected ROUND pad by 0.1062 mm at a 0.1 mm rule, once the
fanout measured pads exactly (found in #1328's replays).

Skips cleanly if the corpus board is absent.
"""
import copy
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522
from kicad_parser import parse_kicad_pcb
from qfn_fanout import generate_qfn_fanout
from check_drc import segment_pad_gap

BOARD = os.path.join(ROOT, "kicad_files", "tigard.kicad_pcb")
TRACK_W, CLEARANCE = 0.1, 0.1


def _own_pad_grazes(tracks, pads, layer):
    hits = []
    for t in tracks:
        for p in pads:
            if p.net_id == t["net_id"]:
                continue
            if p.drill <= 0 and layer not in (p.layers or []):
                continue
            if segment_pad_gap(t["start"][0], t["start"][1], t["end"][0],
                               t["end"][1], p) - t["width"] / 2 < CLEARANCE - 1e-9:
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
    # stub escapes in -x and fans out. Park a same-footprint GND pad on the
    # middle of that fan, so a scan blind to own pads runs straight through it.
    victim_net = "Net-(C23-Pad1)"

    def victim_fanout(blocker=None):
        pcb_v = parse_kicad_pcb(BOARD)
        u3v = pcb_v.footprints["U3"]
        if blocker is not None:
            # Register only in pads_by_net (the graze scan's source); keeping
            # it out of footprint.pads leaves the QFN layout analysis untouched.
            pcb_v.pads_by_net.setdefault(blocker.net_id, []).append(blocker)
        return generate_qfn_fanout(
            u3v, pcb_v, net_filter=[victim_net], layer="F.Cu",
            track_width=TRACK_W, clearance=CLEARANCE, grid_step=0.05,
            escape_method="stub")[0], u3v

    free, u3b = victim_fanout()
    ends = {tuple(t["end"]) for t in free}
    fan = [t for t in free if tuple(t["start"]) in ends]
    if len(fan) != 1:
        print(f"FAIL: expected one fan leg for {victim_net}, got {free}")
        return 1
    fan = fan[0]
    donor = next(p for p in u3b.pads if p.net_name == victim_net)
    gnd_id = next(p.net_id for p in u3b.pads if p.net_name == "GND")
    blocker = copy.copy(donor)
    blocker.pad_number = "X356"
    blocker.net_id = gnd_id
    blocker.net_name = "GND"
    blocker.shape = "circle"
    blocker.size_x = blocker.size_y = 0.3
    blocker.global_x = (fan["start"][0] + fan["end"][0]) / 2
    blocker.global_y = (fan["start"][1] + fan["end"][1]) / 2
    # The grader's own control: the fan laid blind to the pad runs through it.
    if not _own_pad_grazes(free, [blocker], "F.Cu"):
        fails.append("control: a fan laid blind to the injected pad is not "
                     "graded as grazing it")

    tracks2, _u3 = victim_fanout(blocker)
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
