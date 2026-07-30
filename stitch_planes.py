#!/usr/bin/env python3
"""Plane stitching + pour-pocket tie vias (gh-tracked: kad#70).

Two jobs on plane-backed boards (GND/PWR pours on outer layers + inner
reference planes):

1. --stitch-pads: every plane-net SMD pad without a nearby same-net via gets
   one (via-in-pad when legal, clearance-checked against foreign copper on
   every layer, per-instance via sizes, drill hole-to-hole floor).

2. --tie-pockets: KiCad deletes pour islands with no connection ("empty
   islands" -- fenced-off pockets show as bare voids). This pass fills the
   zones with island removal disabled, finds every would-be-removed island
   big enough to matter (--min-pocket-mm2), and drops a clearance-checked
   through-via inside it so the island ties to the reference plane and the
   pour KEEPS the copper: better return paths, EMI shielding, copper
   balance.

Island geometry requires zone filling, which only pcbnew (KiCad's python)
can do; pass --pcbnew-python (defaults to the macOS KiCad app bundle).
The clearance model is KRT's own (kicad_parser + exact pad geometry).

Usage:
    python3 stitch_planes.py board.kicad_pcb --output out.kicad_pcb \
        --nets GND [--stitch-pads] [--tie-pockets] [--min-pocket-mm2 4]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "rust_router"))

from kicad_parser import parse_kicad_pcb, Via
from check_drc import point_to_pad_distance
from geometry_utils import point_to_segment_distance

DEFAULT_PCBNEW_PY = ("/Applications/KiCad/KiCad.app/Contents/Frameworks/"
                     "Python.framework/Versions/Current/bin/python3")

# runs inside KiCad's bundled python: fill zones with island removal
# disabled, report each F/B island of the target nets that has no via and no
# through-hole pad inside (i.e. would be REMOVED), as bbox + a probe grid of
# interior points.
_PCBNEW_ISLANDS = r"""
import json, sys
import pcbnew
board_path, nets_json = sys.argv[1], sys.argv[2]
target_nets = set(json.loads(nets_json))
b = pcbnew.LoadBoard(board_path)
for z in b.Zones():
    if not z.GetIsRuleArea():
        try:
            z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_NEVER)
        except AttributeError:
            pass
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
net_codes = {}
for name, net in b.GetNetsByName().items():
    if str(name) in target_nets:
        net_codes[net.GetNetCode()] = str(name)
vias = [(t.GetPosition().x / 1e6, t.GetPosition().y / 1e6, t.GetNetCode())
        for t in b.GetTracks() if t.GetClass() == "PCB_VIA"]
tht = [(p.GetPosition().x / 1e6, p.GetPosition().y / 1e6, p.GetNetCode())
       for f in b.GetFootprints() for p in f.Pads()
       if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
out = []
for z in b.Zones():
    if z.GetIsRuleArea() or z.GetNetCode() not in net_codes:
        continue
    for lid in z.GetLayerSet().CuStack():
        layer = b.GetLayerName(lid)
        if layer not in ("F.Cu", "B.Cu"):
            continue
        polys = z.GetFilledPolysList(lid)
        for oi in range(polys.OutlineCount()):
            o = polys.Outline(oi)

            def inside(x, y):
                return o.PointInside(pcbnew.VECTOR2I(int(x * 1e6),
                                                     int(y * 1e6)))
            anchored = any(inside(x, y) for x, y, nc in vias
                           if nc == z.GetNetCode())
            anchored = anchored or any(inside(x, y) for x, y, nc in tht
                                       if nc == z.GetNetCode())
            if anchored:
                continue
            bb = o.BBox()
            x0, y0 = bb.GetLeft() / 1e6, bb.GetTop() / 1e6
            x1, y1 = bb.GetRight() / 1e6, bb.GetBottom() / 1e6
            probes = []
            step = 0.15
            yy = y0 + step
            while yy < y1:
                xx = x0 + step
                while xx < x1:
                    ring = all(inside(xx + dx, yy + dy)
                               for dx, dy in ((0, 0), (0.18, 0), (-0.18, 0),
                                              (0, 0.18), (0, -0.18)))
                    if ring:
                        probes.append((round(xx, 3), round(yy, 3)))
                    xx += step
                yy += step
            out.append({"net": net_codes[z.GetNetCode()], "layer": layer,
                        "bbox": [x0, y0, x1, y1], "probes": probes})
print("ISLANDS_JSON:" + json.dumps(out))
"""


class ViaOracle:
    """Through-via legality at (x, y): clears foreign copper on every copper
    layer, foreign drills at hole-to-hole floor, exact pad geometry."""

    def __init__(self, pcb, net_id: int, clearance_map: Dict[int, float],
                 base: float, via_size: float, via_drill: float,
                 hole_to_hole: float):
        self.net_id = net_id
        self.clr = clearance_map
        self.base = base
        self.vs, self.vd, self.h2h = via_size, via_drill, hole_to_hole
        self.own = clearance_map.get(net_id, base)
        self.segs = [s for s in pcb.segments if s.net_id != net_id]
        self.vias = list(pcb.vias)
        self.pads = [p for pads in pcb.pads_by_net.values() for p in pads]

    def _pair(self, other: int) -> float:
        return max(self.own, self.clr.get(other, self.base))

    def ok(self, x: float, y: float) -> bool:
        r = self.vs / 2
        for s in self.segs:
            if point_to_segment_distance(x, y, s.start_x, s.start_y, s.end_x,
                                         s.end_y) < r + self._pair(s.net_id) \
                    + s.width / 2:
                return False
        for v in self.vias:
            d = math.hypot(x - v.x, y - v.y)
            if d < 1e-6:
                continue
            if d < self.vd / 2 + self.h2h + v.drill / 2:
                return False
            if v.net_id != self.net_id and d < r + self._pair(v.net_id) \
                    + v.size / 2:
                return False
        for p in self.pads:
            if p.net_id == self.net_id:
                continue
            if point_to_pad_distance(x, y, p) < r + self._pair(p.net_id):
                return False
            drill = max(p.drill or 0.0, getattr(p, "drill_w", 0.0) or 0.0)
            if drill > 0 and math.hypot(x - p.global_x, y - p.global_y) \
                    < self.vd / 2 + self.h2h + drill / 2:
                return False
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("board")
    ap.add_argument("--output", required=True)
    ap.add_argument("--nets", nargs="+", required=True)
    ap.add_argument("--stitch-pads", action="store_true")
    ap.add_argument("--tie-pockets", action="store_true")
    ap.add_argument("--min-pocket-mm2", type=float, default=4.0)
    ap.add_argument("--via-size", type=float, default=0.25)
    ap.add_argument("--via-drill", type=float, default=0.15)
    ap.add_argument("--clearance", type=float, default=0.1)
    ap.add_argument("--hole-to-hole", type=float, default=0.2)
    ap.add_argument("--near-via-mm", type=float, default=0.8,
                    help="pad counts as stitched if a same-net via is "
                         "within this distance")
    ap.add_argument("--pcbnew-python", default=DEFAULT_PCBNEW_PY)
    args = ap.parse_args()

    pcb = parse_kicad_pcb(args.board)
    content = open(args.board, encoding="utf-8").read()
    clearance_map: Dict[int, float] = {}
    try:
        from list_nets import net_clearance_map_by_id
        clearance_map = net_clearance_map_by_id(
            args.board, {nid: n.name for nid, n in pcb.nets.items()}) or {}
    except Exception:
        pass

    name_to_id = {n.name: nid for nid, n in pcb.nets.items()}
    add_vias: List[Tuple[float, float, int]] = []

    if args.stitch_pads:
        for net_name in args.nets:
            nid = name_to_id.get(net_name)
            if nid is None:
                continue
            oracle = ViaOracle(pcb, nid, clearance_map, args.clearance,
                               args.via_size, args.via_drill,
                               args.hole_to_hole)
            own_vias = [(v.x, v.y) for v in pcb.vias if v.net_id == nid]
            own_vias += [(x, y) for x, y, n2 in add_vias if n2 == nid]
            placed = blocked = 0
            for p in pcb.pads_by_net.get(nid, []):
                if getattr(p, "pad_type", "smd") != "smd":
                    continue
                px, py = p.global_x, p.global_y
                if any(math.hypot(px - a, py - b) <= args.near_via_mm
                       for a, b in own_vias):
                    continue
                if min(p.size_x, p.size_y) >= args.via_size + 0.02 \
                        and oracle.ok(px, py):
                    add_vias.append((px, py, nid))
                    own_vias.append((px, py))
                    oracle.vias.append(Via(
                        x=px, y=py, size=args.via_size, drill=args.via_drill,
                        layers=["F.Cu", "B.Cu"], net_id=nid, uuid=""))
                    placed += 1
                else:
                    blocked += 1
            print(f"stitch {net_name}: {placed} via(s), {blocked} blocked")

    if args.tie_pockets:
        r = subprocess.run(
            [args.pcbnew_python, "-c", _PCBNEW_ISLANDS, args.board,
             json.dumps(args.nets)],
            capture_output=True, text=True)
        line = next((l for l in r.stdout.splitlines()
                     if l.startswith("ISLANDS_JSON:")), None)
        if line is None:
            print("pcbnew island probe failed:", r.stderr[-400:])
            sys.exit(2)
        islands = json.loads(line[len("ISLANDS_JSON:"):])
        tied = skipped = 0
        for isl in islands:
            x0, y0, x1, y1 = isl["bbox"]
            if (x1 - x0) * (y1 - y0) < args.min_pocket_mm2:
                skipped += 1
                continue
            nid = name_to_id.get(isl["net"])
            if nid is None:
                continue
            oracle = ViaOracle(pcb, nid, clearance_map, args.clearance,
                               args.via_size, args.via_drill,
                               args.hole_to_hole)
            done = False
            for (px, py) in isl["probes"]:
                if oracle.ok(px, py):
                    add_vias.append((px, py, nid))
                    pcb.vias.append(Via(
                        x=px, y=py, size=args.via_size, drill=args.via_drill,
                        layers=["F.Cu", "B.Cu"], net_id=nid, uuid=""))
                    tied += 1
                    done = True
                    break
            if not done:
                print(f"  pocket {isl['layer']} bbox={isl['bbox']}: no spot")
        print(f"tie-pockets: {tied} tied, {skipped} below "
              f"{args.min_pocket_mm2}mm2")

    add_txt = "".join(
        '\n\t(via (at %.4f %.4f) (size %.3g) (drill %.3g) '
        '(layers "F.Cu" "B.Cu") (net %d))' % (x, y, args.via_size,
                                              args.via_drill, nid)
        for (x, y, nid) in add_vias)
    i = content.rstrip().rfind(")")
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(content[:i] + add_txt + "\n" + content[i:])
    print(f"wrote {args.output} (+{len(add_vias)} via(s))")


if __name__ == "__main__":
    main()
