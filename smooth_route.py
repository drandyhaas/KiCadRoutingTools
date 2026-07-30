#!/usr/bin/env python3
"""Post-route geometry smoothing (gh#74).

Grid A* emits every avoidance wiggle as grid-step staircase jogs, and they
survive to the output: DRC-clean but maze-router-raw copper. This pass
rewrites each net's polyline chains with clearance-checked any-angle
shortcuts (greedy farthest-visible-vertex), collapsing staircases and
redundant meanders while provably preserving clearances:

  * a candidate shortcut segment is accepted only if its swept capsule
    (width/2 + pairwise netclass clearance) clears ALL foreign copper --
    segments, via barrels at per-instance size, pads at exact geometry --
    plus drill holes and the board edge;
  * chain endpoints, via positions, and pad junctions are immovable;
  * chains are split at junctions (>=3 incident segments), so T-joints and
    stitch taps are preserved.

Also trims dangling stubs (--trim-dangles): a chain end that lands on no
same-net pad, via, or junction is dead copper (e.g. a fanout stub whose via
was later deleted) and is removed segment-by-segment back to the last
anchored point. Pour-net stubs are trimmed too -- the solid pour provides
the connectivity, the stub is noise.

Usage:
    python3 smooth_route.py in.kicad_pcb --output out.kicad_pcb \
        [--nets NET ...] [--trim-dangles] [--edge-clearance 0.45]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "rust_router"))

from kicad_parser import parse_kicad_pcb
from check_drc import point_to_pad_distance
from geometry_utils import (point_to_segment_distance,
                            segment_to_segment_distance)

Q = 4  # coordinate quantization decimals for junction keying


def _key(x: float, y: float) -> Tuple[float, float]:
    return (round(x, Q), round(y, Q))


class Obstacles:
    """Exact-geometry clearance oracle for one net (foreign copper only)."""

    def __init__(self, pcb, net_id: int, clearance_map: Dict[int, float],
                 base_clearance: float, edge_clearance: float,
                 outline_circle: Optional[Tuple[float, float, float]]):
        self.own_clr = clearance_map.get(net_id, base_clearance)
        self.clr = clearance_map
        self.base = base_clearance
        self.edge_clr = edge_clearance
        self.outline = outline_circle
        self.segs_by_layer: Dict[str, List] = defaultdict(list)
        for s in pcb.segments:
            if s.net_id != net_id:
                self.segs_by_layer[s.layer].append(s)
        self.vias = [v for v in pcb.vias if v.net_id != net_id]
        self.pads_by_layer: Dict[str, List] = defaultdict(list)
        copper_layers = {l for l in self.segs_by_layer}
        copper_layers.update({"F.Cu", "B.Cu"})
        for pads in pcb.pads_by_net.values():
            for p in pads:
                if p.net_id == net_id:
                    continue
                for l in (p.layers or []):
                    if l == "*.Cu":
                        for cl in ("F.Cu", "In1.Cu", "In2.Cu", "In3.Cu",
                                   "In4.Cu", "B.Cu"):
                            self.pads_by_layer[cl].append(p)
                        break
                    if l.endswith(".Cu"):
                        self.pads_by_layer[l].append(p)

    def _pair(self, other_net: int) -> float:
        return max(self.own_clr, self.clr.get(other_net, self.base))

    def capsule_clear(self, x1, y1, x2, y2, width, layer) -> bool:
        hw = width / 2
        for s in self.segs_by_layer.get(layer, ()):
            need = hw + self._pair(s.net_id) + s.width / 2
            if segment_to_segment_distance(x1, y1, x2, y2, s.start_x,
                                           s.start_y, s.end_x,
                                           s.end_y) < need:
                return False
        for v in self.vias:
            need = hw + self._pair(v.net_id) + v.size / 2
            if point_to_segment_distance(v.x, v.y, x1, y1, x2, y2) < need:
                return False
        for p in self.pads_by_layer.get(layer, ()):
            need = hw + self._pair(p.net_id)
            n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.02))
            hit = False
            for k in range(n + 1):
                t = k / n
                px, py = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
                if point_to_pad_distance(px, py, p) < need:
                    hit = True
                    break
            if hit:
                return False
        if self.outline is not None:
            cx, cy, r = self.outline
            lim = r - self.edge_clr - hw
            if (math.hypot(x1 - cx, y1 - cy) > lim
                    or math.hypot(x2 - cx, y2 - cy) > lim):
                return False
        return True


def build_chains(segs: Sequence, hard_points: Optional[set] = None) -> List[List]:
    """Split a net's segments (single layer) into polyline chains, broken at
    junction points (degree >= 3) AND at hard points (same-net via taps /
    pad touches -- removing such a vertex severs the tap). Returns lists of
    Segment objects in path order (orientation not normalized)."""
    adj: Dict[Tuple[float, float], List] = defaultdict(list)
    for s in segs:
        adj[_key(s.start_x, s.start_y)].append(s)
        adj[_key(s.end_x, s.end_y)].append(s)
    junction = {pt for pt, lst in adj.items() if len(lst) >= 3}
    if hard_points:
        junction |= {pt for pt in adj if pt in hard_points}
    seen = set()
    chains = []
    for start in segs:
        if id(start) in seen:
            continue
        chain = [start]
        seen.add(id(start))
        for endpoint, forward in ((_key(start.end_x, start.end_y), True),
                                  (_key(start.start_x, start.start_y),
                                   False)):
            pt = endpoint
            while pt not in junction:
                nxt = [s for s in adj[pt] if id(s) not in seen]
                if len(nxt) != 1:
                    break
                s = nxt[0]
                seen.add(id(s))
                if forward:
                    chain.append(s)
                else:
                    chain.insert(0, s)
                pt = (_key(s.end_x, s.end_y)
                      if _key(s.start_x, s.start_y) == pt
                      else _key(s.start_x, s.start_y))
        chains.append(chain)
    return chains


def chain_points(chain: Sequence) -> Optional[List[Tuple[float, float]]]:
    """Ordered vertex list of a chain, or None if it isn't a simple path."""
    if not chain:
        return None
    if len(chain) == 1:
        s = chain[0]
        return [(s.start_x, s.start_y), (s.end_x, s.end_y)]
    pts: List[Tuple[float, float]] = []
    first, second = chain[0], chain[1]
    ends0 = {_key(first.start_x, first.start_y): (first.end_x, first.end_y),
             _key(first.end_x, first.end_y): (first.start_x, first.start_y)}
    shared = None
    for cand in (_key(second.start_x, second.start_y),
                 _key(second.end_x, second.end_y)):
        if cand in ends0:
            shared = cand
    if shared is None:
        return None
    pts.append(ends0[shared])
    pts.append((shared[0], shared[1]))
    cur = shared
    for s in chain[1:]:
        a, b = _key(s.start_x, s.start_y), _key(s.end_x, s.end_y)
        if a == cur:
            nxt = b
        elif b == cur:
            nxt = a
        else:
            return None
        pts.append((nxt[0], nxt[1]))
        cur = nxt
    return pts


def shortcut(points: List[Tuple[float, float]], width: float, layer: str,
             obs: Obstacles) -> List[Tuple[float, float]]:
    """Greedy farthest-visible-vertex simplification."""
    out = [points[0]]
    i = 0
    n = len(points)
    while i < n - 1:
        j = n - 1
        while j > i + 1:
            a, b = points[i], points[j]
            if obs.capsule_clear(a[0], a[1], b[0], b[1], width, layer):
                break
            j -= 1
        out.append(points[j])
        i = j
    return out


def polyline_len(pts: Sequence[Tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(pts, pts[1:]))


def find_outline_circle(content: str):
    import re
    m = re.search(r'\(gr_circle\s+\(center ([0-9.-]+) ([0-9.-]+)\)\s+'
                  r'\(end ([0-9.-]+) ([0-9.-]+)\)[^)]*?\(layer "Edge\.Cuts"\)',
                  content, re.DOTALL)
    if not m:
        return None
    cx, cy = float(m.group(1)), float(m.group(2))
    ex, ey = float(m.group(3)), float(m.group(4))
    return (cx, cy, math.hypot(ex - cx, ey - cy))


def trim_dangles(pcb, net_ids) -> List:
    """Iteratively remove segments whose free end anchors on nothing:
    no same-net pad copper, no same-net via, no other same-net segment."""
    removed = []
    seg_pool = {id(s): s for s in pcb.segments if s.net_id in net_ids}
    # dedupe exact twins first (same net/layer/span/width) -- duplicated
    # copper anchors itself and defeats the dangle test below.
    seen_sig = set()
    for s in list(seg_pool.values()):
        a, b = _key(s.start_x, s.start_y), _key(s.end_x, s.end_y)
        sig = (s.net_id, s.layer, round(s.width, 4)) + tuple(sorted((a, b)))
        if sig in seen_sig:
            removed.append(s)
            del seg_pool[id(s)]
        else:
            seen_sig.add(sig)
    pads_by_net = {nid: pcb.pads_by_net.get(nid, []) for nid in net_ids}
    vias_by_net: Dict[int, List] = defaultdict(list)
    for v in pcb.vias:
        if v.net_id in net_ids:
            vias_by_net[v.net_id].append(v)
    changed = True
    while changed:
        changed = False
        by_pt: Dict[Tuple[int, str, Tuple[float, float]], List] = defaultdict(list)
        for s in seg_pool.values():
            by_pt[(s.net_id, s.layer, _key(s.start_x, s.start_y))].append(s)
            by_pt[(s.net_id, s.layer, _key(s.end_x, s.end_y))].append(s)
        for s in list(seg_pool.values()):
            for ex, ey in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
                mates = by_pt[(s.net_id, s.layer, _key(ex, ey))]
                if len(mates) > 1:
                    continue
                anchored = False
                for v in vias_by_net[s.net_id]:
                    if math.hypot(ex - v.x, ey - v.y) <= v.size / 2 + 0.01:
                        anchored = True
                        break
                if not anchored:
                    for p in pads_by_net[s.net_id]:
                        # layer-aware: an F.Cu SMD pad does not anchor a
                        # B.Cu track end sitting under it.
                        p_layers = p.layers or []
                        on_layer = ("*.Cu" in p_layers or s.layer in p_layers
                                    or (p.drill or 0.0) > 0)
                        if on_layer and point_to_pad_distance(ex, ey, p) <= 0.01:
                            anchored = True
                            break
                if not anchored:
                    # soft joint: free end ON another same-net segment's
                    # centerline anchors it. A copper GRAZE (overlapping
                    # copper, endpoint off the centerline) is load-bearing
                    # connectivity but grades track_dangling in KiCad --
                    # SNAP the endpoint onto the neighbour's centerline.
                    for o in seg_pool.values():
                        if o is s or o.net_id != s.net_id or o.layer != s.layer:
                            continue
                        d_o = point_to_segment_distance(
                            ex, ey, o.start_x, o.start_y, o.end_x, o.end_y)
                        if d_o <= 0.02:
                            anchored = True
                            break
                        if d_o <= o.width / 2 + s.width / 2:
                            dx, dy = o.end_x - o.start_x, o.end_y - o.start_y
                            L = dx * dx + dy * dy
                            t = 0.0 if L <= 0 else max(0.0, min(1.0, (
                                (ex - o.start_x) * dx + (ey - o.start_y) * dy) / L))
                            px2, py2 = o.start_x + t * dx, o.start_y + t * dy
                            if (ex, ey) == (s.start_x, s.start_y):
                                s.start_x, s.start_y = px2, py2
                            else:
                                s.end_x, s.end_y = px2, py2
                            if math.hypot(s.end_x - s.start_x,
                                          s.end_y - s.start_y) < 0.01:
                                # snap collapsed it -- pure redundancy
                                removed.append(s)
                                del seg_pool[id(s)]
                                changed = True
                            anchored = True
                            break
                if not anchored:
                    removed.append(s)
                    del seg_pool[id(s)]
                    changed = True
                    break
    return removed


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("board")
    ap.add_argument("--output", required=True)
    ap.add_argument("--nets", nargs="*", default=None,
                    help="restrict to these nets (default: all routed nets)")
    ap.add_argument("--clearance", type=float, default=0.1)
    ap.add_argument("--edge-clearance", type=float, default=0.45)
    ap.add_argument("--trim-dangles", action="store_true")
    ap.add_argument("--trim-vias", action="store_true",
                    help="only remove dangling vias (<=1-layer copper touch, "
                         "no zone credit) + their orphaned feeder stubs; no "
                         "general segment trimming")
    ap.add_argument("--exclude-nets", nargs="*", default=[],
                    help="nets to leave untouched")
    args = ap.parse_args()

    pcb = parse_kicad_pcb(args.board)
    content = open(args.board, encoding="utf-8").read()
    outline = find_outline_circle(content)

    clearance_map: Dict[int, float] = {}
    try:
        from list_nets import net_clearance_map_by_id
        clearance_map = net_clearance_map_by_id(
            args.board, {nid: n.name for nid, n in pcb.nets.items()}) or {}
    except Exception:
        pass

    routed_net_ids = sorted({s.net_id for s in pcb.segments if s.net_id})
    if args.nets:
        wanted = set(args.nets)
        routed_net_ids = [nid for nid in routed_net_ids
                          if pcb.nets.get(nid) and pcb.nets[nid].name in wanted]
    excl = set(args.exclude_nets)
    routed_net_ids = [nid for nid in routed_net_ids
                      if not (pcb.nets.get(nid) and pcb.nets[nid].name in excl)]

    from kicad_parser import Segment
    saved_mm = 0.0
    chains_touched = 0

    for nid in routed_net_ids:
        # SEQUENTIAL: rebuild the oracle per net over the CURRENT board so a
        # net already smoothed this run is an obstacle at its new geometry
        # (two nets shortcutting into the same freed corridor collide).
        obs = Obstacles(pcb, nid, clearance_map, args.clearance,
                        args.edge_clearance, outline)
        # group by (layer, width): a chain must be uniform-width -- emitting
        # at max(width) re-widens power-net neckdown taps into violations.
        segs_by_layer: Dict[Tuple[str, float], List] = defaultdict(list)
        for s in pcb.segments:
            if s.net_id == nid:
                segs_by_layer[(s.layer, round(s.width, 4))].append(s)
        own_vias = [v for v in pcb.vias if v.net_id == nid]
        own_pads = pcb.pads_by_net.get(nid, [])
        # hard points: polyline vertices on a same-net via or same-net pad
        # copper are taps; removing them severs the connection.
        vertex_pts = set()
        for segs in segs_by_layer.values():
            for s in segs:
                vertex_pts.add(_key(s.start_x, s.start_y))
                vertex_pts.add(_key(s.end_x, s.end_y))
        hard = set()
        for pt in vertex_pts:
            px, py = pt
            if any(math.hypot(px - v.x, py - v.y) <= v.size / 2 + 0.01
                   for v in own_vias):
                hard.add(pt)
                continue
            if any(point_to_pad_distance(px, py, p) <= 0.05
                   for p in own_pads):
                hard.add(pt)

        net_changed = False
        for (layer, _w), segs in segs_by_layer.items():
            # soft joints: a same-net segment END touching another segment's
            # INTERIOR. Collected across ALL widths on this layer -- a wide
            # power chain tapping a thin chain mid-span is still a joint
            # (width-grouped collection missed it, VBAT_SENSE regression).
            endpoints = []
            for (l2, _w2), segs2 in segs_by_layer.items():
                if l2 != layer:
                    continue
                for s in segs2:
                    endpoints.append((s.start_x, s.start_y, s))
                    endpoints.append((s.end_x, s.end_y, s))
            # same-net vias tapping a segment MID-SPAN are joints too: the
            # via center must survive as a pinned vertex or the layer
            # transition is severed (AFE_SAI_FS regression).
            for v in own_vias:
                endpoints.append((v.x, v.y, None))
            for chain in build_chains(segs, hard_points=hard):
                if len(chain) < 2:
                    continue
                pts = chain_points(chain)
                if pts is None or len(pts) < 3:
                    continue
                width = max(s.width for s in chain)
                chain_ids = {id(s) for s in chain}
                # insert + pin soft-joint touch points
                pinned_idx = {0, len(pts) - 1}
                touch_pts = []
                for (ex, ey, es) in endpoints:
                    if es is not None and id(es) in chain_ids:
                        continue
                    for a, b in zip(pts, pts[1:]):
                        if point_to_segment_distance(
                                ex, ey, a[0], a[1], b[0], b[1]) <= width / 2 + 0.01:
                            touch_pts.append((ex, ey))
                            break
                for (tx, ty) in touch_pts:
                    best_i, best_d = None, 1e9
                    for i2, (a, b) in enumerate(zip(pts, pts[1:])):
                        dd = point_to_segment_distance(tx, ty, a[0], a[1],
                                                       b[0], b[1])
                        if dd < best_d:
                            best_d, best_i = dd, i2
                    near_v = None
                    for i2, p2 in enumerate(pts):
                        if math.hypot(p2[0] - tx, p2[1] - ty) <= 0.02:
                            near_v = i2
                            break
                    if near_v is not None:
                        pinned_idx.add(near_v)
                    elif best_i is not None:
                        pts.insert(best_i + 1, (tx, ty))
                        pinned_idx = {(i2 if i2 <= best_i else i2 + 1)
                                      for i2 in pinned_idx}
                        pinned_idx.add(best_i + 1)
                # shortcut each pinned-to-pinned stretch independently
                order = sorted(pinned_idx)
                simp: List[Tuple[float, float]] = [pts[order[0]]]
                for a_i, b_i in zip(order, order[1:]):
                    stretch = pts[a_i:b_i + 1]
                    simp.extend(shortcut(stretch, width, layer, obs)[1:])
                if len(simp) >= len(pts):
                    continue
                old_len = polyline_len(pts)
                new_len = polyline_len(simp)
                chains_touched += 1
                saved_mm += old_len - new_len
                # apply immediately (sequential model)
                pcb.segments = [s for s in pcb.segments
                                if id(s) not in chain_ids]
                for a, b in zip(simp, simp[1:]):
                    pcb.segments.append(Segment(
                        start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                        width=width, layer=layer, net_id=nid, uuid=""))
                net_changed = True
        del net_changed

    dangles: List = []
    dangle_vias: List = []
    if args.trim_vias and not args.trim_dangles:
        zone_nets = {z.net_id for z in pcb.zones}
        for v in list(pcb.vias):
            if v.net_id not in set(routed_net_ids) or v.net_id in zone_nets:
                continue
            touch_layers = set()
            for s2 in pcb.segments:
                if s2.net_id != v.net_id:
                    continue
                if point_to_segment_distance(
                        v.x, v.y, s2.start_x, s2.start_y, s2.end_x,
                        s2.end_y) <= v.size / 2 + s2.width / 2:
                    touch_layers.add(s2.layer)
            for pd in pcb.pads_by_net.get(v.net_id, []):
                if point_to_pad_distance(v.x, v.y, pd) <= v.size / 2:
                    if (pd.drill or 0.0) > 0:
                        touch_layers.update(("F.Cu", "B.Cu"))
                    else:
                        for pl in (pd.layers or []):
                            if pl.endswith(".Cu") and not pl.startswith("*"):
                                touch_layers.add(pl)
            # stack-aware: co-located same-net vias share the barrel -- the
            # STACK's touch union decides; surplus stack members always go.
            stack = [o for o in pcb.vias if o is not v
                     and o.net_id == v.net_id
                     and math.hypot(o.x - v.x, o.y - v.y) < 0.05]
            if stack:
                for o in stack:
                    for s2 in pcb.segments:
                        if s2.net_id != v.net_id:
                            continue
                        if point_to_segment_distance(
                                o.x, o.y, s2.start_x, s2.start_y, s2.end_x,
                                s2.end_y) <= o.size / 2 + s2.width / 2:
                            touch_layers.add(s2.layer)
                    dangle_vias.append(o)
                    pcb.vias.remove(o)
            if len(touch_layers) <= 1:
                # collect the same-layer contact endpoints INSIDE the barrel:
                # two tracks can meet only through the via copper -- removal
                # must bridge them or the net severs (VBAT_SENSE case).
                contacts = []
                for s2 in pcb.segments:
                    if s2.net_id != v.net_id:
                        continue
                    for ex, ey in ((s2.start_x, s2.start_y),
                                   (s2.end_x, s2.end_y)):
                        if math.hypot(ex - v.x, ey - v.y)                                 <= v.size / 2 + s2.width / 2:
                            contacts.append((ex, ey, s2.layer, s2.width))
                dangle_vias.append(v)
                pcb.vias.remove(v)
                by_l: Dict[str, List] = defaultdict(list)
                for (ex, ey, l2, w2) in contacts:
                    by_l[l2].append((ex, ey, w2))
                for l2, pts2 in by_l.items():
                    uniq = []
                    for (ex, ey, w2) in pts2:
                        if all(math.hypot(ex - ux, ey - uy) > 1e-6
                               for ux, uy, _ in uniq):
                            uniq.append((ex, ey, w2))
                    for (a2, b2) in zip(uniq, uniq[1:]):
                        from kicad_parser import Segment as _Seg
                        pcb.segments.append(_Seg(
                            start_x=a2[0], start_y=a2[1], end_x=b2[0],
                            end_y=b2[1], width=max(a2[2], b2[2]), layer=l2,
                            net_id=v.net_id, uuid=""))
        # orphaned feeders: segments whose ONLY anchor was a removed via
        if dangle_vias:
            removed_pts = [(v.x, v.y, v.size / 2) for v in dangle_vias]
            live_vias = list(pcb.vias)
            for s2 in list(pcb.segments):
                if s2.net_id not in {v.net_id for v in dangle_vias}:
                    continue
                for ex, ey in ((s2.start_x, s2.start_y), (s2.end_x, s2.end_y)):
                    on_removed = any(math.hypot(ex - x, ey - y) <= r + 0.01
                                     for x, y, r in removed_pts)
                    if not on_removed:
                        continue
                    anchored = any(
                        v2.net_id == s2.net_id
                        and math.hypot(ex - v2.x, ey - v2.y) <= v2.size / 2
                        for v2 in live_vias)
                    if not anchored:
                        anchored = any(
                            point_to_pad_distance(ex, ey, pd2) <= 0.01
                            for pd2 in pcb.pads_by_net.get(s2.net_id, []))
                    mates = [o for o in pcb.segments
                             if o is not s2 and o.net_id == s2.net_id
                             and o.layer == s2.layer
                             and (_key(o.start_x, o.start_y) == _key(ex, ey)
                                  or _key(o.end_x, o.end_y) == _key(ex, ey))]
                    if not anchored and not mates:
                        dangles.append(s2)
                        pcb.segments.remove(s2)
                        break
    if args.trim_dangles:
        dangles = trim_dangles(pcb, set(routed_net_ids))
        d_ids = {id(s) for s in dangles}
        pcb.segments = [s for s in pcb.segments if id(s) not in d_ids]
        # dangling VIAS: same-net copper touches on <=1 layer and the net has
        # no zone (a pour/plane credits the via everywhere it spans).
        zone_nets = {z.net_id for z in pcb.zones}
        for v in list(pcb.vias):
            if v.net_id not in set(routed_net_ids) or v.net_id in zone_nets:
                continue
            touch_layers = set()
            for s2 in pcb.segments:
                if s2.net_id != v.net_id:
                    continue
                if point_to_segment_distance(
                        v.x, v.y, s2.start_x, s2.start_y, s2.end_x,
                        s2.end_y) <= v.size / 2 + s2.width / 2:
                    touch_layers.add(s2.layer)
            for pd in pcb.pads_by_net.get(v.net_id, []):
                if point_to_pad_distance(v.x, v.y, pd) <= v.size / 2:
                    if (pd.drill or 0.0) > 0:
                        touch_layers.update(("F.Cu", "B.Cu"))
                    else:
                        for pl in (pd.layers or []):
                            if pl.endswith(".Cu") and not pl.startswith("*"):
                                touch_layers.add(pl)
            if len(touch_layers) <= 1:
                dangle_vias.append(v)
                pcb.vias.remove(v)
        if dangle_vias:
            # removing a via can orphan its feeder stub -- re-trim
            more = trim_dangles(pcb, set(routed_net_ids))
            if more:
                m_ids = {id(s) for s in more}
                pcb.segments = [s for s in pcb.segments if id(s) not in m_ids]
                dangles = list(dangles) + more

    print(f"smoothed {chains_touched} chain(s), -{saved_mm:.1f}mm copper; "
          f"trimmed {len(dangles)} dangling segment(s), "
          f"{len(dangle_vias)} dangling via(s)")

    # rewrite the file: strip every original segment of the touched nets,
    # then append the (possibly smoothed) current pcb_data segments of those
    # nets. This keeps untouched nets byte-identical.
    import re
    touched = set(routed_net_ids)
    out_parts = []
    i = 0
    s = content
    while i < len(s):
        j = s.find("(segment", i)
        if j < 0:
            out_parts.append(s[i:])
            break
        if s[j + 8] not in " \n\t(":
            out_parts.append(s[i:j + 8])
            i = j + 8
            continue
        depth = 0
        k = j
        while k < len(s):
            if s[k] == "(":
                depth += 1
            elif s[k] == ")":
                depth -= 1
                if depth == 0:
                    break
            k += 1
        block = s[j:k + 1]
        out_parts.append(s[i:j])
        mn = re.search(r"\(net (\d+)\)", block)
        if not (mn and int(mn.group(1)) in touched):
            out_parts.append(block)
        i = k + 1
    # drop removed dangling vias from the verbatim copy: numeric block-walk
    # (text formats vary: "81.4" vs "81.400000" -- regex-by-string misses).
    if dangle_vias:
        import re as _re
        gone = [(v.x, v.y, v.net_id) for v in dangle_vias]
        src = "".join(out_parts)
        parts2 = []
        i2 = 0
        while i2 < len(src):
            j2 = src.find("(via", i2)
            if j2 < 0:
                parts2.append(src[i2:])
                break
            if src[j2 + 4] not in " \n\t(":
                parts2.append(src[i2:j2 + 4])
                i2 = j2 + 4
                continue
            depth = 0
            k2 = j2
            while k2 < len(src):
                if src[k2] == "(":
                    depth += 1
                elif src[k2] == ")":
                    depth -= 1
                    if depth == 0:
                        break
                k2 += 1
            block = src[j2:k2 + 1]
            parts2.append(src[i2:j2])
            ma = _re.search(r"\(at ([0-9.-]+) ([0-9.-]+)\)", block)
            mn = _re.search(r"\(net (\d+)\)", block)
            kill = False
            if ma and mn:
                bx, by, bn = float(ma.group(1)), float(ma.group(2)), int(mn.group(1))
                for (gx, gy, gn) in gone:
                    if gn == bn and abs(bx - gx) < 1e-3 and abs(by - gy) < 1e-3:
                        kill = True
                        gone.remove((gx, gy, gn))
                        break
            if not kill:
                parts2.append(block)
            i2 = k2 + 1
        out_parts = parts2
    body = "".join(out_parts)
    add_txt = "".join(
        '\n\t(segment (start %.4f %.4f) (end %.4f %.4f) (width %.4g) '
        '(layer "%s") (net %d))'
        % (sg.start_x, sg.start_y, sg.end_x, sg.end_y, sg.width, sg.layer,
           sg.net_id)
        for sg in pcb.segments if sg.net_id in touched)
    ins = body.rstrip().rfind(")")
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(body[:ins] + add_txt + "\n" + body[ins:])
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
