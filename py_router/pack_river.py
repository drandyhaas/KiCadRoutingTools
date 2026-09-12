#!/usr/bin/env python3
"""#658 river packing: post-route, OFF-GRID lane compaction for bus routes.

The measured 100%-gap root: the human packs bus lanes at ~0.28mm pitch
(just above the legal minimum) while grid routing quantizes pitch to 0.3+
(a 0.1 grid cannot express 0.28) -- ~25% pitch waste = ~2 lanes/layer of
lost corridor head-room. This pass slides each bus member's axis-aligned
runs laterally toward their nearest same-layer sibling run until the edge
gap is clearance + 4um, stretching/shrinking the two perpendicular
neighbor segments (classic channel compaction). Every move is
exact-geometry checked against ALL other copper before applying; output
is a pure coordinate rewrite of existing segments (no adds/removes).

Usage: pack_river.py IN.kicad_pcb OUT.kicad_pcb [--clearance C] [--min-run L]
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'actor'}

import argparse
import math
import re
import sys
from collections import defaultdict

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from kicad_parser import parse_kicad_pcb
from geometry_utils import (point_to_segment_distance as p2s,
                            segment_to_segment_distance as s2s)

EPS = 1e-6
MARGIN = 0.004     # packed edge gap beyond clearance
SLACK = 0.008      # only move when we gain more than this


def bus_groups(pcb):
    groups = defaultdict(list)
    for n in pcb.nets.values():
        if len(n.pads) == 2:
            refs = tuple(sorted(p.component_ref or '?' for p in n.pads))
            groups[refs].append(n.net_id)
    return {k: v for k, v in groups.items() if len(v) >= 4}


def axis_of(s):
    dx, dy = abs(s.end_x - s.start_x), abs(s.end_y - s.start_y)
    if dx < EPS and dy > EPS:
        return 'V'
    if dy < EPS and dx > EPS:
        return 'H'
    return None


def clear_move(pcb, moved, exempt_ids, clearance):
    """moved: list of (x1,y1,x2,y2,w,layer,net_id) candidate geometry."""
    for (x1, y1, x2, y2, w, lay, nid) in moved:
        for s in pcb.segments:
            if id(s) in exempt_ids or s.layer != lay or s.net_id == nid:
                continue
            if s2s(x1, y1, x2, y2, s.start_x, s.start_y, s.end_x, s.end_y) \
                    < w / 2 + s.width / 2 + clearance - 1e-4:
                return False
        for v in pcb.vias:
            if v.net_id == nid:
                continue
            if p2s(v.x, v.y, x1, y1, x2, y2) < w / 2 + v.size / 2 + clearance - 1e-4:
                return False
        for fp in pcb.footprints.values():
            for p in fp.pads:
                if p.net_id == nid and p.net_id != 0:
                    continue
                on = (p.drill and p.drill > 0) or (lay in (p.layers or ()))
                if not on or p.pad_type == 'np_thru_hole':
                    continue
                if p2s(p.global_x, p.global_y, x1, y1, x2, y2) \
                        < w / 2 + max(p.size_x, p.size_y) / 2 + clearance - 1e-4:
                    return False
    return True


def pack(pcb, clearance, min_run, verbose=True):
    groups = bus_groups(pcb)
    segs_by_net = defaultdict(list)
    for s in pcb.segments:
        segs_by_net[s.net_id].append(s)
    endpoint_index = defaultdict(list)
    for s in pcb.segments:
        endpoint_index[(round(s.start_x, 6), round(s.start_y, 6), s.net_id)].append(s)
        endpoint_index[(round(s.end_x, 6), round(s.end_y, 6), s.net_id)].append(s)

    def neighbors_at(s, x, y):
        return [o for o in endpoint_index[(round(x, 6), round(y, 6), s.net_id)]
                if o is not s]

    moves = []           # (seg, new_coords...) applied in-place
    rewrites = {}        # id(seg) -> None (marker that coords changed)
    n_moved = 0
    for refs, members in groups.items():
        mset = set(members)
        for nid in members:
            for s in list(segs_by_net[nid]):
                ax = axis_of(s)
                if ax is None:
                    continue
                run = abs((s.end_x - s.start_x) if ax == 'H' else (s.end_y - s.start_y))
                if run < min_run:
                    continue
                # nearest parallel same-layer sibling run with overlap
                lat = s.start_y if ax == 'H' else s.start_x
                lo = min(s.start_x, s.end_x) if ax == 'H' else min(s.start_y, s.end_y)
                hi = max(s.start_x, s.end_x) if ax == 'H' else max(s.start_y, s.end_y)
                best = None
                for onid in mset:
                    if onid == nid:
                        continue
                    for o in segs_by_net[onid]:
                        if o.layer != s.layer or axis_of(o) != ax:
                            continue
                        olo = min(o.start_x, o.end_x) if ax == 'H' else min(o.start_y, o.end_y)
                        ohi = max(o.start_x, o.end_x) if ax == 'H' else max(o.start_y, o.end_y)
                        if min(hi, ohi) - max(lo, olo) < 0.6:
                            continue
                        olat = o.start_y if ax == 'H' else o.start_x
                        d = abs(olat - lat)
                        if 0.05 < d < 0.8 and (best is None or d < best[0]):
                            best = (d, o, olat)
                if best is None:
                    continue
                d, o, olat = best
                target = (s.width + o.width) / 2 + clearance + MARGIN
                if d <= target + SLACK:
                    continue
                new_lat = olat + math.copysign(target, lat - olat)
                delta = new_lat - lat
                # both endpoints must join exactly one perpendicular neighbor
                ends = [(s.start_x, s.start_y), (s.end_x, s.end_y)]
                nbrs = []
                ok = True
                for (x, y) in ends:
                    nb = neighbors_at(s, x, y)
                    # exactly one continuing segment (any angle -- the
                    # moved joint may leave 45, which KiCad permits), and
                    # no via anchoring this endpoint.
                    if len(nb) != 1:
                        ok = False
                        break
                    if any(v.net_id == nid and math.hypot(v.x - x, v.y - y) < 1e-3
                           for v in pcb.vias):
                        ok = False
                        break
                    nbrs.append(nb[0])
                if not ok:
                    continue
                # candidate geometry: slid run + adjusted neighbors
                if ax == 'H':
                    cand = [(s.start_x, new_lat, s.end_x, new_lat, s.width, s.layer, nid)]
                else:
                    cand = [(new_lat, s.start_y, new_lat, s.end_y, s.width, s.layer, nid)]
                nb_new = []
                for (x, y), nb in zip(ends, nbrs):
                    fx, fy = ((nb.end_x, nb.end_y)
                              if (abs(nb.start_x - x) < EPS and abs(nb.start_y - y) < EPS)
                              else (nb.start_x, nb.start_y))
                    nx, ny = (x, new_lat) if ax == 'H' else (new_lat, y)
                    cand.append((fx, fy, nx, ny, nb.width, nb.layer, nid))
                    nb_new.append((nb, fx, fy, nx, ny))
                exempt = {id(s), id(nbrs[0]), id(nbrs[1])}
                if not clear_move(pcb, cand, exempt, clearance):
                    continue
                # apply in place
                if ax == 'H':
                    s.start_y = s.end_y = new_lat
                else:
                    s.start_x = s.end_x = new_lat
                for (nb, fx, fy, nx, ny) in nb_new:
                    nb.start_x, nb.start_y, nb.end_x, nb.end_y = fx, fy, nx, ny
                for t in (s, nbrs[0], nbrs[1]):
                    rewrites[id(t)] = t
                n_moved += 1
                # rebuild endpoint index entries for the three segments
                endpoint_index.clear()
                for t2 in pcb.segments:
                    endpoint_index[(round(t2.start_x, 6), round(t2.start_y, 6), t2.net_id)].append(t2)
                    endpoint_index[(round(t2.end_x, 6), round(t2.end_y, 6), t2.net_id)].append(t2)
    if verbose:
        print(f"pack_river: {n_moved} run(s) packed to clearance+{MARGIN*1000:.0f}um")
    return rewrites


def pack_result_segments(pcb_data, segments, vias, net_id, sibling_ids,
                         clearance, net_clearances=None, min_run=0.8):
    """#658 IN-RUN per-track packing: slide a FRESH route's axis-aligned
    runs laterally toward their nearest committed same-layer sibling run
    (edge gap clearance + MARGIN), mutating the not-yet-committed Segment
    objects in place -- called from the copper choke point BEFORE the
    route becomes an obstacle, so the vacated lane is free for every later
    net in the same pass. Same channel-compaction move as pack(): the two
    endpoint neighbors stretch; a run whose endpoint carries a via, or
    joins other than exactly one continuing segment, is left alone.

    Exact-checked against ALL committed copper at the PAIR-MAX class
    clearance (net_clearances is the #326 net_id->clearance map; classes
    only widen). The fresh route's own copper is same-net and exempt.
    Returns the number of runs moved."""
    ncl = net_clearances or {}
    own_clr = max(clearance, ncl.get(net_id, 0.0))

    own = [s for s in segments if s.net_id == net_id]
    epi = defaultdict(list)
    for s in own:
        epi[(round(s.start_x, 6), round(s.start_y, 6))].append(s)
        epi[(round(s.end_x, 6), round(s.end_y, 6))].append(s)
    sib_runs = [s for s in pcb_data.segments
                if s.net_id in sibling_ids and axis_of(s) is not None]
    if not sib_runs:
        return 0
    all_vias = list(vias or []) + pcb_data.vias

    def _blocked(cand):
        for (x1, y1, x2, y2, w, lay, nid) in cand:
            for s2 in pcb_data.segments:
                if s2.layer != lay or s2.net_id == nid:
                    continue
                c2 = max(own_clr, ncl.get(s2.net_id, 0.0))
                if s2s(x1, y1, x2, y2, s2.start_x, s2.start_y,
                       s2.end_x, s2.end_y) < w / 2 + s2.width / 2 + c2 - 1e-4:
                    return True
            for v in pcb_data.vias:
                if v.net_id == nid:
                    continue
                c2 = max(own_clr, ncl.get(v.net_id, 0.0))
                if p2s(v.x, v.y, x1, y1, x2, y2) \
                        < w / 2 + v.size / 2 + c2 - 1e-4:
                    return True
            for fp in pcb_data.footprints.values():
                for p in fp.pads:
                    if p.net_id == nid and p.net_id != 0:
                        continue
                    on = (p.drill and p.drill > 0) or (lay in (p.layers or ()))
                    if not on or p.pad_type == 'np_thru_hole':
                        continue
                    c2 = max(own_clr, ncl.get(p.net_id, 0.0),
                             getattr(p, 'local_clearance', 0.0) or 0.0)
                    if p2s(p.global_x, p.global_y, x1, y1, x2, y2) \
                            < w / 2 + max(p.size_x, p.size_y) / 2 + c2 - 1e-4:
                        return True
        return False

    n_moved = 0
    for s in own:
        ax = axis_of(s)
        if ax is None:
            continue
        run = abs((s.end_x - s.start_x) if ax == 'H' else (s.end_y - s.start_y))
        if run < min_run:
            continue
        lat = s.start_y if ax == 'H' else s.start_x
        lo = min(s.start_x, s.end_x) if ax == 'H' else min(s.start_y, s.end_y)
        hi = max(s.start_x, s.end_x) if ax == 'H' else max(s.start_y, s.end_y)
        best = None
        for o in sib_runs:
            if o.layer != s.layer or axis_of(o) != ax:
                continue
            olo = min(o.start_x, o.end_x) if ax == 'H' else min(o.start_y, o.end_y)
            ohi = max(o.start_x, o.end_x) if ax == 'H' else max(o.start_y, o.end_y)
            if min(hi, ohi) - max(lo, olo) < 0.6:
                continue
            olat = o.start_y if ax == 'H' else o.start_x
            d = abs(olat - lat)
            if 0.05 < d < 0.8 and (best is None or d < best[0]):
                best = (d, o, olat)
        if best is None:
            continue
        d, o, olat = best
        pair_clr = max(own_clr, ncl.get(o.net_id, 0.0))
        target = (s.width + o.width) / 2 + pair_clr + MARGIN
        if d <= target + SLACK:
            continue
        new_lat = olat + math.copysign(target, lat - olat)
        ends = [(s.start_x, s.start_y), (s.end_x, s.end_y)]
        nbrs = []
        ok = True
        for (x, y) in ends:
            nb = [o2 for o2 in epi[(round(x, 6), round(y, 6))] if o2 is not s]
            if len(nb) != 1:
                ok = False
                break
            if any(v.net_id == net_id
                   and math.hypot(v.x - x, v.y - y) < 1e-3
                   for v in all_vias):
                ok = False
                break
            nbrs.append(nb[0])
        if not ok:
            continue
        if ax == 'H':
            cand = [(s.start_x, new_lat, s.end_x, new_lat,
                     s.width, s.layer, net_id)]
        else:
            cand = [(new_lat, s.start_y, new_lat, s.end_y,
                     s.width, s.layer, net_id)]
        nb_new = []
        for (x, y), nb in zip(ends, nbrs):
            fx, fy = ((nb.end_x, nb.end_y)
                      if (abs(nb.start_x - x) < EPS
                          and abs(nb.start_y - y) < EPS)
                      else (nb.start_x, nb.start_y))
            nx, ny = (x, new_lat) if ax == 'H' else (new_lat, y)
            cand.append((fx, fy, nx, ny, nb.width, nb.layer, net_id))
            nb_new.append((nb, fx, fy, nx, ny))
        if _blocked(cand):
            continue
        if ax == 'H':
            s.start_y = s.end_y = new_lat
        else:
            s.start_x = s.end_x = new_lat
        for (nb, fx, fy, nx, ny) in nb_new:
            nb.start_x, nb.start_y, nb.end_x, nb.end_y = fx, fy, nx, ny
        n_moved += 1
    return n_moved


NUM = r"-?\d+(?:\.\d+)?"


def rewrite_file(inp, out, pcb_before, pcb_after):
    """Rewrite ONLY segment blocks, line-based (KiCad writes them multiline),
    keyed by original start/end/layer/net."""
    before = parse_kicad_pcb(inp)
    b, a = before.segments, pcb_after.segments
    assert len(b) == len(a)
    remap = {}
    for sb, sa in zip(b, a):
        if (abs(sb.start_x - sa.start_x) > EPS or abs(sb.start_y - sa.start_y) > EPS
                or abs(sb.end_x - sa.end_x) > EPS or abs(sb.end_y - sa.end_y) > EPS):
            key = (round(sb.start_x, 6), round(sb.start_y, 6),
                   round(sb.end_x, 6), round(sb.end_y, 6), sb.layer)
            remap.setdefault(key, []).append(
                (sa.start_x, sa.start_y, sa.end_x, sa.end_y))
    lines = open(inp, encoding='utf-8', errors='replace').read().split('\n')
    n_hit = 0
    i = 0
    num = re.compile(rf"\(start ({NUM}) ({NUM})\)")
    nume = re.compile(rf"\(end ({NUM}) ({NUM})\)")
    numl = re.compile(r'\(layer "([^"]+)"\)')
    while i < len(lines):
        if lines[i].lstrip().startswith('(segment'):
            block = []
            j = i
            depth = 0
            while j < len(lines):
                depth += lines[j].count('(') - lines[j].count(')')
                block.append(j)
                if depth <= 0:
                    break
                j += 1
            text_block = '\n'.join(lines[k] for k in block)
            ms, me = num.search(text_block), nume.search(text_block)
            ml = numl.search(text_block)
            if ms and me and ml:
                key = (round(float(ms.group(1)), 6), round(float(ms.group(2)), 6),
                       round(float(me.group(1)), 6), round(float(me.group(2)), 6),
                       ml.group(1))
                if remap.get(key):
                    nx1, ny1, nx2, ny2 = remap[key].pop(0)
                    if not remap[key]:
                        del remap[key]
                    for k in block:
                        lines[k] = num.sub(f"(start {nx1:.6g} {ny1:.6g})", lines[k])
                        lines[k] = nume.sub(f"(end {nx2:.6g} {ny2:.6g})", lines[k])
                    n_hit += 1
            i = j + 1
        else:
            i += 1
    open(out, 'w', encoding='utf-8').write('\n'.join(lines))
    print(f"pack_river: rewrote {n_hit} segment(s); unmatched: {len(remap)}")
    return len(remap) == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input'); ap.add_argument('output')
    ap.add_argument('--clearance', type=float, default=0.0889)
    ap.add_argument('--min-run', type=float, default=0.8)
    a = ap.parse_args()
    pcb = parse_kicad_pcb(a.input)
    for _ in range(3):
        rw = pack(pcb, a.clearance, a.min_run)
        if not rw:
            break
    ok = rewrite_file(a.input, a.output, None, pcb)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
