"""#658 river packing: OFF-GRID lane compaction for bus routes.

The measured 100%-gap root: the human packs bus lanes at ~0.28mm pitch
(just above the legal minimum) while grid routing quantizes pitch to 0.3+
(a 0.1 grid cannot express 0.28) -- ~25% pitch waste = ~2 lanes/layer of
lost corridor head-room. ``pack_result_segments`` slides a bus member's
axis-aligned runs laterally toward their nearest same-layer sibling run
until the edge gap is clearance + 4um, stretching/shrinking the two
perpendicular neighbor segments (classic channel compaction). Every move
is exact-geometry checked against ALL other copper before applying. It runs
in-run at the copper choke point (KICAD_PACK_INLINE); ``bus_groups`` finds
the groups it packs within.
"""

import math
from collections import defaultdict

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


def pack_result_segments(pcb_data, segments, vias, net_id, sibling_ids,
                         clearance, net_clearances=None, min_run=0.8):
    """#658 IN-RUN per-track packing: slide a FRESH route's axis-aligned
    runs laterally toward their nearest committed same-layer sibling run
    (edge gap clearance + MARGIN), mutating the not-yet-committed Segment
    objects in place -- called from the copper choke point BEFORE the
    route becomes an obstacle, so the vacated lane is free for every later
    net in the same pass. The two endpoint neighbors stretch; a run whose endpoint carries a via, or
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
