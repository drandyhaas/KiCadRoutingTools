"""Exact-geometry widening of power-net copper (#1033 part 3).

Two places laid a power net at its neck / rescue width even where the full
``--power-nets-widths`` width fits:

* the PAD-NECK ZONE (#72): ``_neck_pass`` forced every wide route to the neck
  width for ``neckdown_length`` mm from each pad, fit or not. Measured on the
  run-32 K3C board (+3V3 force-reroute): ~60 mm of the net's 0.127/0.15 copper;
* the RESCUE ladder: a rescued gap is routed at ``rescue_track`` with the power
  width popped, and nothing widened it afterwards (~33 mm on the same repro).

The obstacle map cannot judge either: its endpoint regions are obstacle-EXEMPT
near the pads being connected (so a net can reach its own pad), and a rescue's
map is built at the rung's stepped-down clearance. So the widening is decided
here on EXACT geometry at the net's own pairwise clearance -- the same
measurements check_drc grades (#1029: "width neck judges a pad graze on exact
pad copper") -- against every foreign pad (exact pad copper), track, via,
NPTH hole (declared hole floor), the board edge and rule-area keep-outs.

The entry into a pad of the net ITSELF is capped at the pad's narrow side: a
trace wider than the pad it lands on overhangs the pad outline, which is where
KiCad's own copper-sliver / clearance checks start flagging a neighbour.

Everything narrows back on failure: a piece that fits no candidate width keeps
the width the caller gave it (the neck / rescue width), so the result is never
worse than before -- only wider where the geometry allows it.
"""
from __future__ import annotations

import math
from typing import List, Optional

from kicad_parser import Segment

# Piece length for the neck-zone / rescue widen-back. Finer than the trunk's
# 0.5 mm because the neck zone is short (2.5 mm) and pad fields are dense.
PIECE_MM = 0.25


def width_ladder(target: float, floor: float) -> List[float]:
    """Candidate widths, widest first: the target, then target/2, /4 ...
    strictly above `floor` (the width the copper already has)."""
    out = [target]
    w = target / 2.0
    while w > floor + 1e-9:
        out.append(w)
        w /= 2.0
    return [round(x, 4) for x in out if x > floor + 1e-9]


class ExactWideCheck:
    """Does a piece of net `net_id` copper clear everything foreign at width w?

    Built once per call site (per route / per rescued gap). Every term is the
    exact measurement check_drc grades, at the pair clearance the router uses
    (config.obstacle_clearance, #498 layer rule applied)."""

    def __init__(self, pcb_data, config, net_id):
        from check_drc import (board_edge_geometry, npth_slot_capsules)
        from obstacle_map import resolve_hole_clearance
        import routing_defaults as defaults
        self.pcb = pcb_data
        self.cfg = config
        self.net_id = net_id
        self.nc = getattr(config, 'net_clearances', None) or None
        self.own = (config.obstacle_clearance(net_id)
                    if hasattr(config, 'obstacle_clearance') else config.clearance)
        self.npth = max(config.clearance, defaults.NPTH_TO_TRACK_CLEARANCE,
                        resolve_hole_clearance(pcb_data, config))
        self.edge_rings, self.edge_outer, self.edge_cutouts = \
            board_edge_geometry(pcb_data.board_info)
        self.bounds = pcb_data.board_info.board_bounds
        self.edge_clr = max(config.clearance,
                            getattr(config, 'board_edge_clearance', 0.0) or 0.0)
        self.slots = npth_slot_capsules(pcb_data)
        self.layers = list(getattr(config, 'layers', None) or [])
        # own pads per layer (for the entry cap)
        from net_queries import expand_pad_layers
        self.own_pads = {}
        for p in (pcb_data.pads_by_net.get(net_id) or []):
            for lay in expand_pad_layers(p.layers, self.layers or ['F.Cu', 'B.Cu']):
                self.own_pads.setdefault(lay, []).append(p)
        # foreign pads per layer, for the exact check_drc confirm
        self.foreign_pads = {}
        for nid, pads in (pcb_data.pads_by_net or {}).items():
            if nid == net_id:
                continue
            for p in pads:
                for lay in expand_pad_layers(p.layers, self.layers or ['F.Cu', 'B.Cu']):
                    self.foreign_pads.setdefault(lay, []).append(p)
        self.keepouts = []
        for ko in (getattr(pcb_data.board_info, 'keepouts', None) or []):
            if ko.get('tracks_allowed', True):
                continue
            poly = ko.get('polygon') or []
            if len(poly) >= 3:
                self.keepouts.append(([poly] + [h for h in (ko.get('holes') or [])
                                                if len(h) >= 3],
                                      set(ko.get('layers') or ()) or None))

    def _base(self, layer):
        if hasattr(self.cfg, 'layer_clearance'):
            return self.cfg.layer_clearance(layer, self.own)
        return self.own

    def pad_cap(self, x, y, layer) -> Optional[float]:
        """Narrow side of an OWN pad whose copper contains (x, y) on `layer`,
        else None."""
        cap = None
        for p in self.own_pads.get(layer, ()):
            hx, hy = p.size_x / 2.0, p.size_y / 2.0
            if abs(x - p.global_x) <= hx + 1e-6 and abs(y - p.global_y) <= hy + 1e-6:
                c = min(p.size_x, p.size_y)
                cap = c if cap is None else min(cap, c)
        return cap

    def clears(self, x1, y1, x2, y2, layer, w) -> bool:
        from single_ended_routing import (_seg_foreign_pad_dist,
                                          _seg_foreign_seg_dist,
                                          _seg_foreign_via_dist,
                                          _seg_foreign_hole_dist)
        eff = self._base(layer)
        need = eff + w / 2.0 - 1e-4
        nid = self.net_id
        if _seg_foreign_pad_dist(self.pcb, nid, x1, y1, x2, y2, layer,
                                 base_clearance=eff, net_clearances=self.nc) < need:
            return False
        if _seg_foreign_seg_dist(self.pcb, nid, x1, y1, x2, y2, layer,
                                 net_clearances=self.nc, base_clearance=eff,
                                 track_clearances=getattr(self.cfg, 'track_clearances', None) or None) < need:
            return False
        if _seg_foreign_via_dist(self.pcb, nid, x1, y1, x2, y2, layer,
                                 net_clearances=self.nc, base_clearance=eff) < need:
            return False
        if _seg_foreign_hole_dist(self.pcb, nid, x1, y1, x2, y2,
                                  base_clearance=self.npth) < self.npth + w / 2.0 - 1e-4:
            return False
        if not self._edge_ok(x1, y1, x2, y2, w):
            return False
        if not self._keepout_ok(x1, y1, x2, y2, layer, w):
            return False
        # #1029: the grader's EXACT pad copper as the final word on pads
        # (rect / roundrect / oval / custom polygon, rotation, overrides).
        from check_drc import check_pad_segment_overlap
        seg = Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                      layer=layer, net_id=nid)
        reach = w / 2.0 + eff + 1.0
        for p in self.foreign_pads.get(layer, ()):
            ext = math.hypot(p.size_x, p.size_y) / 2.0 + reach
            if (p.global_x < min(x1, x2) - ext or p.global_x > max(x1, x2) + ext
                    or p.global_y < min(y1, y2) - ext
                    or p.global_y > max(y1, y2) + ext):
                continue
            clr = eff
            if hasattr(self.cfg, 'pad_override_clearance'):
                clr = self.cfg.pad_override_clearance(eff, p)
            if check_pad_segment_overlap(p, seg, clr, self.layers or [layer],
                                         0.0)[0]:
                return False
        return True

    def _edge_ok(self, x1, y1, x2, y2, w):
        from check_drc import (_point_on_board, _segment_to_rings_distance,
                               segment_to_npth_slots_distance)
        required = self.edge_clr + w / 2.0 - 1e-4
        if segment_to_npth_slots_distance(self.slots, x1, y1, x2, y2) < required:
            return False
        if self.edge_rings:
            if not _point_on_board(x1, y1, self.edge_outer, self.edge_cutouts) or \
               not _point_on_board(x2, y2, self.edge_outer, self.edge_cutouts):
                return False
            return _segment_to_rings_distance(x1, y1, x2, y2,
                                              self.edge_rings) >= required
        if self.bounds:
            mnx, mny, mxx, mxy = self.bounds
            return all(min(x - mnx, mxx - x, y - mny, mxy - y) >= required
                       for x, y in ((x1, y1), (x2, y2)))
        return True

    def _keepout_ok(self, x1, y1, x2, y2, layer, w):
        if not self.keepouts:
            return True
        from obstacle_map import point_in_polygon, point_to_polygon_edge_distance
        margin = self.cfg.clearance + w / 2.0
        for rings, kls in self.keepouts:
            if kls is not None and not (layer in kls or '*.Cu' in kls):
                continue
            n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.1) + 1)
            for q in range(n + 1):
                t = q / n
                px, py = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
                inside = False
                for ring in rings:
                    if point_in_polygon(px, py, ring):
                        inside = not inside
                if inside or any(point_to_polygon_edge_distance(px, py, r) < margin
                                 for r in rings):
                    return False
        return True


def widen_segment(seg, target_w, check: ExactWideCheck,
                  piece_mm: float = PIECE_MM):
    """`seg` (at its current, narrower width) as collinear pieces, each at the
    widest of width_ladder(target_w, seg.width) that `check` clears -- and no
    wider than an own pad the piece's ends land in -- else at seg.width.
    Consecutive pieces of equal width merge back. Returns a list of Segments
    (the original object when nothing widens)."""
    floor_w = seg.width
    cands = width_ladder(target_w, floor_w)
    if not cands:
        return [seg]
    L = math.hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y)
    if L <= 1e-9:
        return [seg]
    n = max(1, int(math.ceil(L / piece_mm)))
    pts = [(seg.start_x + (seg.end_x - seg.start_x) * i / n,
            seg.start_y + (seg.end_y - seg.start_y) * i / n) for i in range(n)]
    pts.append((seg.end_x, seg.end_y))
    widths = []
    for i in range(n):
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        cap = None
        for (px, py) in ((ax, ay), (bx, by)):
            c = check.pad_cap(px, py, seg.layer)
            if c is not None:
                cap = c if cap is None else min(cap, c)
        chosen = floor_w
        _c = list(cands)
        if cap is not None and floor_w + 1e-9 < cap < target_w - 1e-9:
            # a pad narrower than the target: try exactly its narrow side
            _c = sorted(set(_c) | {round(cap, 4)}, reverse=True)
        for w in _c:
            if cap is not None and w > cap + 1e-9:
                continue
            if check.clears(ax, ay, bx, by, seg.layer, w):
                chosen = w
                break
        widths.append(chosen)
    if all(abs(w - floor_w) < 1e-12 for w in widths):
        return [seg]
    out = []
    i = 0
    while i < n:
        j = i
        while j < n and abs(widths[j] - widths[i]) < 1e-12:
            j += 1
        out.append(Segment(start_x=pts[i][0], start_y=pts[i][1],
                           end_x=pts[j][0], end_y=pts[j][1], width=widths[i],
                           layer=seg.layer, net_id=seg.net_id))
        i = j
    return out


def widen_rescued_copper(result, pcb_data, net_id, config) -> float:
    """#1033 part 3b: after a rescue routed a POWER net's gap at its rung
    width (the rescue pops the power width to find ANY path), widen the
    rescued copper piecewise wherever the net's own width -- or a step of its
    ladder -- clears, judged on exact geometry at the ORIGINAL config's
    clearance (not the rung's stepped-down one). The rescue search itself is
    unchanged; this only re-widths the copper it found. Pieces are collinear
    with the originals, so connectivity is untouched.

    Mutates result['new_segments'] and pcb_data.segments in place (the route
    is already on the board). Returns the mm of copper widened."""
    if net_id not in (getattr(config, 'power_net_widths', None) or {}):
        return 0.0
    segs = list(result.get('new_segments') or [])
    if not segs:
        return 0.0
    check = ExactWideCheck(pcb_data, config, net_id)
    new_list = []
    repl = {}
    widened = 0.0
    # Decide EVERYTHING first, then mutate the board once: the check reads
    # pcb_data.segments, so editing it mid-loop would feed the next query a
    # half-rewritten board (a first cut did, and crashed on a hole it left).
    for s in segs:
        target = config.get_net_track_width(net_id, s.layer)
        if s.width >= target - 1e-9:
            new_list.append(s)
            continue
        pieces = widen_segment(s, target, check)
        if len(pieces) == 1 and pieces[0] is s:
            new_list.append(s)
            continue
        for q in pieces:
            if q.width > s.width + 1e-9:
                widened += math.hypot(q.end_x - q.start_x, q.end_y - q.start_y)
        new_list.extend(pieces)
        repl[id(s)] = pieces
    if repl:
        board = []
        for x in pcb_data.segments:
            board.extend(repl.get(id(x), (x,)))
        pcb_data.segments[:] = board          # same list object, new contents
        result['new_segments'] = new_list
        pcb_data._copper_epoch = getattr(pcb_data, '_copper_epoch', 0) + 1
    return widened
