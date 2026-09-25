"""Post-route widening of power-net copper (#1033).

Routing lays a power net narrower than its ``--power-nets-widths`` in several
places, each for a reason that holds while routing and not afterwards: the
pad-neck zone (#72: ``neckdown_length`` mm from each pad, forced narrow),
neck-downs and short-edge ladders where the grid map refused the full width,
and rescue rungs (routed with the power width popped). Measured on run 32's
K3C board, +3V3 force-reroute: ~200 of ~530 mm shipped under 0.3.

``widen_power_copper`` takes that width back ONCE, after routing, inside the
shared cleanup pipeline (``cleanup_pipeline.run_post_route_cleanup``, so the
CLI and the GUI both get it). It runs after every net is routed, so it never
changes what a later net sees while routing -- completion comes first, and
widening only uses space every other net has already left over.

Each piece (~0.25 mm) takes the widest of the net's width and its ladder
(width/2, /4, ...; plus an own pad's narrow side) that ``ExactWideCheck``
clears, else keeps the width it shipped with, so nothing ever gets narrower.
The check is exact geometry at the net's pair clearance (#498 layer rule
applied) -- the measurements check_drc grades -- against foreign pads (the
sampled distance over an expanded-layer view, then check_drc's exact pad
copper at the pair clearance check_drc grades: both nets' classes, the layer
rule, the pad override, #1029), tracks (incl. .kicad_dru track rules), vias,
NPTH holes at the declared hole floor, the board edge and NPTH slots, rule-area
keep-outs (``*.Cu`` / ``F&B.Cu`` resolved), ``--keepout`` zones when enabled,
and footprint graphic copper (foreign to every net here: no #908 own-pad lift).
The grid map is not consulted: its endpoint regions are obstacle-exempt.

Limits: foreign pours are not modelled (the refill decides), copper laid
after the cleanup (the in-run plane finalize, the oracle reconnect) is not
widened, and protected / matched / impedance nets are skipped like the
smoother skips them. A check that raises refuses the piece (fail closed) and
is counted in ``ERRORS``; a constructor that raises is announced and skips
that net.
"""
from __future__ import annotations

import math
from typing import List, Optional

from kicad_parser import Segment

# Failures of the check itself, per run: a constructor that raised (the
# widening for that route is OFF, loudly) and a clears() call that raised
# (that piece is REFUSED -- fail closed). route.py puts both in JSON_SUMMARY.
ERRORS = {'check_errors': 0, 'ctor_errors': 0, 'last': ''}


def reset_errors():
    """Start of a run (batch_route, outermost call): the counters are
    process-wide, so a GUI session or an in-process caller would otherwise
    carry one run's failures into the next run's summary."""
    ERRORS['check_errors'] = 0
    ERRORS['ctor_errors'] = 0
    ERRORS['last'] = ''


def note_ctor_error(exc):
    """Loud, one line: a raised constructor must not silently turn the
    widening off for a net."""
    ERRORS['ctor_errors'] += 1
    ERRORS['last'] = f"{type(exc).__name__}: {exc}"
    print(f"WARNING: power-width widen check could not be built "
          f"({type(exc).__name__}: {exc}) -- this net keeps its routed "
          f"widths (#1033)")


# Piece length for the post-route widen. Finer than _neck_pass's 0.5 mm
# pieces because the pad-neck zone is short (neckdown_length) and pad fields
# are dense.
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

    Built once per net by widen_power_copper. Every term is the
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
        # The SAMPLED pad term reads single_ended_routing._foreign_pad_arrays,
        # which matches a layer only by `layer in pad.layers or '*.Cu'` -- so
        # an `F&B.Cu` through-hole pad is invisible to it (verifier: 418 of
        # 800 probe cases cleared a graze without the exact confirm). Feed it
        # a VIEW whose pads carry the expanded copper layers, the same set the
        # exact confirm uses, so the two terms agree on WHICH pads exist. The
        # router's own terminal checks keep the raw spelling (out of scope).
        import copy as _copy
        from types import SimpleNamespace
        _view = {}
        for nid, pads in (pcb_data.pads_by_net or {}).items():
            lst = []
            for p in pads:
                q = _copy.copy(p)
                q.layers = sorted(set(expand_pad_layers(
                    p.layers, self.layers or ['F.Cu', 'B.Cu'])))
                lst.append(q)
            _view[nid] = lst
        self.pad_view = SimpleNamespace(pads_by_net=_view)
        # #908 lifts a footprint's own GRAPHIC copper for the net of the pad
        # it touches (the router's reading). KiCad grades that copper as
        # net-less, so for OPTIONAL widening it is foreign to everyone -- the
        # conservative reading costs only width (watchy AE1: GND pieces on
        # the antenna polygon read clear through the lift).
        self.graphics = [g for g in (pcb_data.segments or [])
                         if getattr(g, 'graphic', False)]
        self.keepouts = []
        _all_cu = set(self.layers or ['F.Cu', 'B.Cu'])
        for ko in (getattr(pcb_data.board_info, 'keepouts', None) or []):
            if ko.get('tracks_allowed', True):
                continue
            poly = ko.get('polygon') or []
            if len(poly) >= 3:
                # composite tokens resolved like obstacle_map's rule-area
                # stamp (#369 A5): '*.Cu' = every copper layer, 'F&B.Cu' =
                # front and back. Empty = every layer.
                kls = set(ko.get('layers') or ())
                res = set()
                for ln in kls:
                    if ln == '*.Cu':
                        res |= _all_cu
                    elif ln in ('F&B.Cu', 'F&B'):
                        res |= {'F.Cu', 'B.Cu'}
                    else:
                        res.add(ln)
                self.keepouts.append(([poly] + [h for h in (ko.get('holes') or [])
                                                if len(h) >= 3],
                                      res or None))
        # #27 user keep-outs (--keepout), active when config.keepout_enabled,
        # on every layer -- the same block the #536 smoother honours.
        if getattr(config, 'keepout_enabled', False):
            for kz in (getattr(pcb_data, 'keepout_zones', None) or []):
                if len(kz.points) >= 3:
                    self.keepouts.append(([list(kz.points)], None))

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
        """Fail CLOSED: a check that raises refuses the piece (it keeps its
        narrower width) and is counted in ERRORS, never crashes the route."""
        try:
            return self._clears(x1, y1, x2, y2, layer, w)
        except Exception as exc:                                # noqa: BLE001
            ERRORS['check_errors'] += 1
            ERRORS['last'] = f"{type(exc).__name__}: {exc}"
            return False

    def _clears(self, x1, y1, x2, y2, layer, w) -> bool:
        from single_ended_routing import (_seg_foreign_pad_dist,
                                          _seg_foreign_seg_dist,
                                          _seg_foreign_via_dist,
                                          _seg_foreign_hole_dist)
        eff = self._base(layer)
        need = eff + w / 2.0 - 1e-4
        nid = self.net_id
        if _seg_foreign_pad_dist(self.pad_view, nid, x1, y1, x2, y2, layer,
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
        if not self._graphics_ok(x1, y1, x2, y2, layer, w, eff):
            return False
        # #1029: the grader's EXACT pad copper as the final word on pads
        # (rect / roundrect / oval / custom polygon, rotation), each priced at
        # the PAIR clearance check_drc grades it at (pad_pair_clearance).
        from check_drc import check_pad_segment_overlap
        seg = Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                      layer=layer, net_id=nid)
        for p in self.foreign_pads.get(layer, ()):
            clr = self.pad_pair_clearance(p, layer)
            ext = math.hypot(p.size_x, p.size_y) / 2.0 + w / 2.0 + clr + 1.0
            if (p.global_x < min(x1, x2) - ext or p.global_x > max(x1, x2) + ext
                    or p.global_y < min(y1, y2) - ext
                    or p.global_y > max(y1, y2) + ext):
                continue
            if check_pad_segment_overlap(p, seg, clr, self.layers or [layer],
                                         0.0)[0]:
                return False
        return True

    def pad_pair_clearance(self, pad, layer) -> float:
        """The clearance check_drc grades this net's track against foreign
        `pad` on `layer` at (its `_pad_pair_cl`), resolved with the router's
        own helpers rather than re-derived: the pair's class clearance -- the
        larger of both nets' ``config.obstacle_clearance``, so the FOREIGN
        pad's net class counts -- then the .kicad_dru layer rule REPLACING it
        (``config.layer_clearance``, #498), then a pad / footprint override
        REPLACING that, floored at rules.min_clearance
        (``config.pad_override_clearance``, #326)."""
        cfg = self.cfg
        clr = self.own
        if hasattr(cfg, 'obstacle_clearance'):
            clr = max(clr, cfg.obstacle_clearance(getattr(pad, 'net_id', 0)))
        if hasattr(cfg, 'layer_clearance'):
            clr = cfg.layer_clearance(layer, clr)
        if hasattr(cfg, 'pad_override_clearance'):
            clr = cfg.pad_override_clearance(clr, pad)
        return clr

    def _graphics_ok(self, x1, y1, x2, y2, layer, w, eff):
        """Footprint graphic copper on `layer`, foreign to EVERY net here
        (no #908 own-pad lift for optional widening)."""
        if not self.graphics:
            return True
        from geometry_utils import segment_to_segment_closest_points
        reach = eff + w / 2.0
        for g in self.graphics:
            if g.layer != layer:
                continue
            gh = (g.width or 0.0) / 2.0
            m = reach + gh
            if (max(g.start_x, g.end_x) < min(x1, x2) - m
                    or min(g.start_x, g.end_x) > max(x1, x2) + m
                    or max(g.start_y, g.end_y) < min(y1, y2) - m
                    or min(g.start_y, g.end_y) > max(y1, y2) + m):
                continue
            d = segment_to_segment_closest_points(
                Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                        layer=layer, net_id=self.net_id), g)[0]
            if d - gh < reach - 1e-4:
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
            if kls is not None and layer not in kls:
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


def widen_power_copper(results, pcb_data, config, scope_net_ids=None):
    """#1033, AFTER routing: widen each power net's copper where its
    requested width -- or a step of its ladder -- clears on exact geometry
    against the FINISHED board.

    One pass over every result's power-net copper, run by the shared cleanup
    pipeline (so the CLI and the GUI both get it). Completion comes first:
    the pass runs after routing, so it only takes space that every other net
    has left over.

    It covers whatever the run laid narrower than the net's width: the pad
    neck zone, rescue rungs, short-edge ladders and trunk pieces the grid fit
    refused. The exact check replaces the map's quantised fit, so a piece the
    map refused may widen now. Collinear pieces only; connectivity unchanged.

    Per net: decide every piece first, then rewrite that net's result lists
    and the board ONCE, so every piece of a net is judged against the same
    board and the next net's check sees the widened copper. Returns
    {'nets': n, 'widened_mm': mm}."""
    pw = getattr(config, 'power_net_widths', None) or {}
    stats = {'nets': 0, 'widened_mm': 0.0}
    if not pw or not results:
        return stats
    # Same skip set as the #536 smoother: protected nets (matched groups,
    # coupled pairs, locked copper) and impedance-declared nets -- their
    # geometry is the spec, so no width change either.
    try:
        from cleanup_pipeline import _smooth_skip_net_ids
        skip = _smooth_skip_net_ids(pcb_data)
    except Exception:                                           # noqa: BLE001
        skip = set()
    by_net = {}
    for r in results:
        for sg in (r.get('new_segments') or []):
            nid = getattr(sg, 'net_id', None)
            if (nid in pw and nid not in skip
                    and not getattr(sg, 'graphic', False)):
                if scope_net_ids is not None and nid not in scope_net_ids:
                    continue
                by_net.setdefault(nid, []).append(sg)
    for nid in sorted(by_net):
        segs = [sg for sg in by_net[nid]
                if sg.width < config.get_net_track_width(nid, sg.layer) - 1e-9]
        if not segs:
            continue
        try:
            check = ExactWideCheck(pcb_data, config, nid)
        except Exception as exc:                                # noqa: BLE001
            note_ctor_error(exc)
            continue
        repl = {}
        widened = 0.0
        for sg in segs:
            pieces = widen_segment(sg, config.get_net_track_width(nid, sg.layer),
                                   check)
            if len(pieces) == 1 and pieces[0] is sg:
                continue
            for q in pieces:
                if q.width > sg.width + 1e-9:
                    widened += math.hypot(q.end_x - q.start_x, q.end_y - q.start_y)
            repl[id(sg)] = pieces
        if not repl:
            continue
        for r in results:
            ns = r.get('new_segments')
            if not ns:
                continue
            out = []
            hit = False
            for x in ns:
                rp = repl.get(id(x))
                if rp is None:
                    out.append(x)
                else:
                    out.extend(rp)
                    hit = True
            if hit:
                r['new_segments'] = out
        board = []
        for x in pcb_data.segments:
            board.extend(repl.get(id(x), (x,)))
        pcb_data.segments[:] = board
        pcb_data._copper_epoch = getattr(pcb_data, '_copper_epoch', 0) + 1
        stats['nets'] += 1
        stats['widened_mm'] += widened
    stats['widened_mm'] = round(stats['widened_mm'], 3)
    return stats
