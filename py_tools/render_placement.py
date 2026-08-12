#!/usr/bin/env python3
"""Headless PNG stills of placement status (#431).

A routed board is self-evidently inspectable: open it in KiCad, the tracks are
there. A placement repair is a set of position deltas whose merit is invisible
without the context that motivated it -- which airwires were failing, which nets
walled them off, what moved and how far. `docs/placement-optimization.md` names
the acceptance criterion ("output that is visibly 'your placement, nudged' can
be reviewed in minutes") but nothing produced an artifact to review.

This composes; it does not re-render. Every pixel goes through
`route_render.BoardRenderer` (board outline, cutouts, zones, pads, drills, the
caption HUD) via its `overlays=` hook, and every world->pixel conversion through
`renderer.tf`. There is no coordinate arithmetic in this file -- a test asserts
the rect handed to the drawing layer is `state.parts[ref].rect()` identically,
which fails the day someone inlines a courtyard transform.

    python3 render_placement.py BOARD.kicad_pcb [-o OUT.png|OUTDIR] [options]
    python3 render_placement.py BOARD --before SEED --arrows -o delta.png
    python3 render_placement.py BOARD --zoom-group sheet:1a2b --per-side -o dir/

Known limits, stated because the artifact should not oversell (the issue's own
list): a dense board does not fit one readable frame, which is why delta-first
and focused crops are defaults rather than polish; a flat image projects away
the layer dimension; and geometry is not causality -- arrows show WHAT moved,
never why it helped. The caption strip carries the verdict, and a render without
it invites exactly the wrong review heuristic.
"""
from __future__ import annotations
import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import json
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

from PIL import ImageDraw

import routing_defaults as defaults
from kicad_parser import parse_kicad_pcb
from route_render import BoardRenderer, load_font

# --- palette (OmniLayout's categories: outline / THT / top SMD / back SMD) ----
C_COURT_F = (150, 152, 168)     # front courtyard
C_COURT_B = (108, 132, 160)     # back courtyard (cooler, like B.Cu)
C_COURT_DIM = (58, 60, 70)      # context part in delta-first mode
C_LOCKED = (92, 88, 74)         # locked: dimmed, and hatched
C_GHOST = (76, 76, 92)          # seed position
C_ARROW = (236, 214, 110)       # displacement
C_AIR = (86, 96, 112)           # ordinary airwire
C_AIR_FAIL = (232, 72, 72)      # failed net
C_AIR_BLOCK = (236, 158, 60)    # blocker net
C_AIR_PICK = (96, 214, 170)     # net named by --ratsnest-nets
C_LABEL = (226, 228, 238)
C_PAD_THT = (196, 150, 74)      # through-hole
C_PAD_F = (198, 172, 96)        # front SMD
C_PAD_B = (104, 150, 196)       # back SMD


# ---------------------------------------------------------------------------
# Model: what to draw, computed from the board (no PIL below this line)
# ---------------------------------------------------------------------------
class PlacementModel:
    """Everything drawable, derived once from a board.

    Uses a real `QuenchState` so the rects, airwires and metrics are the ones
    the OPTIMIZER acted on -- including the sorted-net_refs MST tie-break (#457)
    -- rather than a parallel implementation that drifts. Measured cost:
    interf_u 0.15s, ulx3s 0.70s, glasgow_revC 0.81s.
    """

    def __init__(self, pcb, pcb_file: str, *, exact: bool = True,
                 quench_kwargs: Optional[Dict] = None):
        self.pcb = pcb
        self.pcb_file = pcb_file
        self.state = None
        self.no_outline = False
        self.metrics: Dict[str, object] = {}
        # Which floors this model was built at, and where each came from.
        # Always present, so a caller never has to guess whether it was set.
        self.floor_knobs: Dict[str, Dict] = {}
        if exact:
            self.state = self._build_state(quench_kwargs or {})
        if self.state is not None:
            self.metrics = self._metrics()

    def _build_state(self, kw):
        from placement.quench import QuenchState
        from list_nets import board_floor_knobs
        # BOARD-FIRST floors, resolved HERE rather than in main() so the CLI and
        # every library caller get the same answer -- and so this instrument
        # agrees with place_optimize, which has resolved them this way since
        # run-7 S1 (place_optimize.py:144). Until now it did not: the renderer
        # hardcoded 0.25/0.55 while the optimizer read the board, so on any
        # board declaring its own floor the two disagreed about the geometry
        # they were supposedly sharing. On the measured board that is 0.2 and
        # 0.5, not 0.25 and 0.55.
        #
        # An explicit --clearance still wins: board_floor_knobs takes a non-None
        # value as 'cli' and returns it unchanged.
        clr, edge, self.floor_knobs = board_floor_knobs(
            self.pcb_file, clearance=kw.get('clearance'),
            board_edge_clearance=kw.get('board_edge_clearance'),
            clearance_default=defaults.CLEARANCE, edge_default=0.55)
        args = dict(clearance=clr, board_edge_clearance=edge,
                    crossing_penalty=10.0, halo_base=0.5, halo_coef=0.25,
                    halo_weight=2.0, edge_halo=2.0, edge_weight=2.0,
                    grid_step=defaults.GRID_STEP, length_weight=1.0)
        args.update({k: v for k, v in kw.items() if v is not None})
        try:
            return QuenchState(self.pcb, self.pcb_file, **args)
        except ValueError as e:
            # "No board boundary (Edge.Cuts) found" -- the optimizer genuinely
            # cannot gate containment without an outline, and relaxing that
            # guard to suit a VIEWER would be backwards. Fall back on an
            # in-memory bounds so everything that does NOT depend on the outline
            # (courtyards, sides, airwires, crossings, HPWL, overlap) stays
            # exact, and report oob as unavailable.
            if 'Edge.Cuts' not in str(e):
                raise
            from route_render import _geometry_bounds
            self.no_outline = True
            self.pcb.board_info.board_bounds = _geometry_bounds(self.pcb)
            return QuenchState(self.pcb, self.pcb_file, **args)

    def _metrics(self) -> Dict[str, object]:
        m = dict(self.state.total_cost())
        leg = self.state.legality_metrics()
        m.update(leg)
        if self.no_outline:
            # oob is meaningless without a real outline; say so rather than
            # printing a zero that reads as "clean".
            for k in ('oob_count', 'oob_amount', 'oob_area'):
                m[k] = None
        return m

    # -- geometry -----------------------------------------------------------
    def parts(self):
        return self.state.parts if self.state is not None else {}

    def rect(self, ref):
        """The part's courtyard rect, from the optimizer's own model."""
        p = self.state.parts.get(ref)
        return None if p is None else p.rect()

    def side(self, ref) -> str:
        p = self.state.parts.get(ref)
        return 'F' if p is None else p.side

    def sides(self, ref):
        p = self.state.parts.get(ref)
        return {'F'} if p is None else set(p.sides)

    def far_rect(self, ref):
        """Far-side footprint of a through-hole part: its DRILLED-PAD box, not
        its courtyard. `legality.rect_on` gates on exactly this, so drawing the
        courtyard on both sides would disagree with the grader."""
        p = self.state.parts.get(ref)
        if p is None or not getattr(p, 'has_tht', False):
            return None
        try:
            return p.tht_rect()
        except Exception:
            return None

    def airwires(self, net_ids=None):
        """(x1,y1,x2,y2,net_id) tuples, from the state's prebuilt cache."""
        out = []
        for nid, aws in sorted((self.state.net_airwires or {}).items()):
            if net_ids is not None and nid not in net_ids:
                continue
            out.extend(aws)
        return out

    def net_ids_for(self, names) -> set:
        want = set(names or ())
        return {nid for nid, n in self.pcb.nets.items()
                if nid > 0 and n.name in want}

    def net_ids_matching(self, patterns) -> set:
        """Net ids matching glob patterns, via the SHARED filter.

        `net_queries.matches_net_filter` is what `route.py --nets` and the plan
        executor use, so `'*USB*' '!*_N'` means here exactly what it means
        there -- including that a leading `!` is an exclusion, with the
        active-low caveat that helper already documents. A second matcher in a
        viewer would be a second set of surprises.
        """
        if not patterns:
            return set()
        from net_queries import matches_net_filter
        return {nid for nid, n in self.pcb.nets.items()
                if nid > 0 and n.name and matches_net_filter(n.name, list(patterns))}

    def refs_of_nets(self, net_ids) -> set:
        refs = set()
        for nid in net_ids:
            refs |= set(self.state.net_refs.get(nid, ()))
        return refs

    def net_points(self, net_ids) -> List[Tuple[float, float]]:
        pts = []
        for nid in sorted(net_ids):
            for ref in self.state.net_refs.get(nid, ()):
                p = self.state.parts.get(ref)
                if p is not None:
                    pts.extend((x, y) for x, y, _ in p.pad_globals())
        return pts


def legality_findings(model) -> Dict[str, object]:
    """Named legality findings, computed ONCE per model (run-4 G).

    Two run-3 problems, one cause: the metrics were COUNTS with no ref lists
    (the operator hand-computed WHICH parts were off-board to answer the
    mandate-8 checklist), and `draw_legality` re-derived the same O(n^2)
    pair sweep per panel (8 focus panels = 8 full sweeps). The findings are
    computed here once, cached on the model, consumed by both the overlay
    and the JSON.

    Channels are labelled because run 3 lost time to an UNLABELLED
    two-channel disagreement: `oob_refs_pad_copper` measures each part's PAD
    extent against the board OUTLINE (what the dashed-red overlay draws);
    `oob_refs_courtyard` measures the courtyard rect against the same outline
    gate (what `oob_count` in the metrics counts). They legitimately differ
    -- an NPTH mounting hole has no pad copper, a courtyard overhang may
    carry no copper -- but they differ only in WHICH RECT they measure. Both
    fall back to the bounding box only when the model carries no outline.
    """
    cached = getattr(model, '_legality_findings', None)
    if cached is not None:
        return cached
    out = {'oob_refs_pad_copper': [], 'oob_refs_courtyard': [],
           'pad_conflict_pairs_refs': [], 'hole_conflict_pairs_refs': [],
           'body_overlap_pairs_refs': [],
           'locked_refs': sorted(r for r, p in model.parts().items()
                                 if getattr(p, 'locked', False))}
    state = getattr(model, 'state', None)
    ctx = getattr(state, 'legality_ctx', None) if state is not None else None
    if ctx is not None:
        b = state.board
        # Measure the pad extent against the REAL outline, not the bounding
        # box. The board is not its bounding box: on a board with an inner
        # cutout (the run-11 smartknob has 1 ring + 5 cutouts) a part sitting
        # dead-centre in the hole scored oob = 0, so this channel -- which the
        # placement skill calls the top-priority placement gate, because
        # off-outline pad copper converts one-for-one into unrouted nets --
        # was structurally blind exactly where it mattered. The courtyard
        # channel 25 lines below has always used the outline gate; the two are
        # meant to differ in WHICH RECT they measure (pad extent vs courtyard),
        # not in which board. Bounding box stays as the fallback for a model
        # with no outline.
        # ZERO margin, deliberately. state.edge_gate carries the board-edge
        # CLEARANCE margin, so reusing it would report every part merely inside
        # the edge band as off-outline -- on run 11's board that turned 8 real
        # breaches into 21 findings, and this channel gates the placement->
        # routing hand-off. The question here is only "is pad copper outside
        # the outline", which is the margin-0 form. The ring geometry is
        # already computed, so this is a shallow copy, not a re-parse.
        _pad_gate = None
        if not getattr(model, 'no_outline', False):
            _src = getattr(state, 'edge_gate', None)
            _pad_gate = getattr(state, '_pad_edge_gate_m0', None)
            if _pad_gate is None and _src is not None:
                import copy as _copy
                _pad_gate = _copy.copy(_src)
                _pad_gate.margin = 0.0
                if getattr(_pad_gate, 'bounds', None) is not None:
                    _pad_gate.usable = tuple(_pad_gate.bounds)
                _pad_gate._near = {}
                try:
                    state._pad_edge_gate_m0 = _pad_gate
                except Exception:
                    pass
        for ref in sorted(ctx.parts):
            p = state.parts.get(ref)
            if p is None:
                continue
            ext = ctx.parts[ref].extent(p.x, p.y, p.rot)
            if ext is None:
                continue
            oob = None
            if _pad_gate is not None:
                try:
                    # Per PAD rect, not the part's whole axis-aligned extent.
                    # That extent is an AABB over every pad, so a part rotated
                    # off-axis near an edge (run 11's J1 sits at 45 deg) has
                    # AABB corners well outside its real copper and reported a
                    # breach no pad actually makes. This channel gates the
                    # hand-off to routing, so a false positive here costs a
                    # refusal on a healthy board.
                    _rects = ctx.parts[ref].pad_rects(p.x, p.y, p.rot)
                    oob = max((_pad_gate.rect_outside_amount(r[:4])
                               for r in _rects), default=0.0)
                except Exception:
                    oob = None
            if oob is None:
                oob = (max(0.0, b[0] - ext[0]) + max(0.0, ext[2] - b[2])
                       + max(0.0, b[1] - ext[1]) + max(0.0, ext[3] - b[3]))
            if oob > 1e-6:
                out['oob_refs_pad_copper'].append([ref, round(oob, 4)])
        refs = sorted(ctx.parts)
        for i, a in enumerate(refs):
            pa = state.parts.get(a)
            if pa is None:
                continue
            for bb in refs[i + 1:]:
                pb = state.parts.get(bb)
                if pb is None:
                    continue
                sf = ctx.pair_shortfall(a, bb)
                if sf.pad > 1e-6:
                    out['pad_conflict_pairs_refs'].append(
                        [a, bb, round(sf.pad, 4)])
                if sf.hole > 1e-6:
                    out['hole_conflict_pairs_refs'].append(
                        [a, bb, round(sf.hole, 4)])
                # run-6: any-net cross-footprint pad STACKS (the assembly
                # channel; the shipped C14-on-R14 class) -- what the
                # checklist's b_body_overlap_pairs key reports
                if sf.stack:
                    out['body_overlap_pairs_refs'].append([a, bb])
    if state is not None and not getattr(model, 'no_outline', False):
        gate = getattr(state, 'edge_gate', None)
        if gate is not None:
            for ref, p in sorted(model.parts().items()):
                try:
                    amt = gate.rect_outside_amount(p.rect())
                except Exception:
                    continue
                if amt > 1e-6:
                    out['oob_refs_courtyard'].append([ref, round(amt, 4)])
    model._legality_findings = out
    return out


def moved_parts(before_pcb, after_pcb, tol: float = 1e-4) -> List[Dict]:
    """Refs whose position changed, with both poses. Sorted (#457)."""
    out = []
    for ref, a in sorted((before_pcb.footprints or {}).items()):
        b = (after_pcb.footprints or {}).get(ref)
        if b is None:
            continue
        if abs(a.x - b.x) > tol or abs(a.y - b.y) > tol \
                or abs((a.rotation or 0) - (b.rotation or 0)) > 1e-3:
            out.append({'reference': ref,
                        'from': (a.x, a.y, a.rotation or 0.0),
                        'to': (b.x, b.y, b.rotation or 0.0),
                        'dist': math.hypot(b.x - a.x, b.y - a.y)})
    return out


# ---------------------------------------------------------------------------
# Views
# ---------------------------------------------------------------------------
def union_view(rects, pad_mm: float = 2.0, min_size: float = 8.0):
    """Bounding rect over `rects`, padded and floored.

    Generalizes animate_fanout_clearance._view_bounds' idea (frame to what will
    be DRAWN, not to the board). The min_size floor is why one nudged 0402 does
    not fill the screen.
    """
    xs, ys = [], []
    for r in rects:
        if not r:
            continue
        xs += [r[0], r[2]]
        ys += [r[1], r[3]]
    if not xs:
        return None
    x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
    x0, y0, x1, y1 = x0 - pad_mm, y0 - pad_mm, x1 + pad_mm, y1 + pad_mm
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    w, h = max(x1 - x0, min_size), max(y1 - y0, min_size)
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def cluster_points(points, gap: float) -> List[List[Tuple[float, float]]]:
    """Single-link clusters of pad coordinates, deterministic in input order.

    Used for --focus. The issue assumed failure COORDINATES were in the #409
    JSON; they are not -- `blocking_info_to_dict` carries net names and cell
    counts only -- so the geometry comes from the board.
    """
    pts = sorted(set((round(x, 4), round(y, 4)) for x, y in points))
    clusters: List[List[Tuple[float, float]]] = []
    for p in pts:
        hit = [c for c in clusters
               if any(math.hypot(p[0] - q[0], p[1] - q[1]) <= gap for q in c)]
        if not hit:
            clusters.append([p])
        else:
            merged = [p]
            for c in hit:
                merged += c
                clusters.remove(c)
            clusters.append(merged)
    clusters.sort(key=lambda c: (-len(c), min(q[0] for q in c),
                                 min(q[1] for q in c)))
    return clusters


# ---------------------------------------------------------------------------
# Overlays -- all drawing goes through renderer.tf, never raw coordinates
# ---------------------------------------------------------------------------
def _w(r, mm: float, floor: int = 1) -> int:
    """A width in mm -> device px, honouring supersampling. A hardcoded pixel
    width silently halves at supersample=2."""
    return max(floor, int(round(r.tf.length(mm))))


def _rect_pts(r, rect):
    x0, y0 = r.tf.pt(rect[0], rect[1])
    x1, y1 = r.tf.pt(rect[2], rect[3])
    return [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]


def _dash(d, p0, p1, on_px, off_px, fill, width):
    """PIL has no dashed line."""
    x0, y0 = p0
    x1, y1 = p1
    total = math.hypot(x1 - x0, y1 - y0)
    if total < 1e-6:
        return
    ux, uy = (x1 - x0) / total, (y1 - y0) / total
    t = 0.0
    while t < total:
        e = min(t + on_px, total)
        d.line([x0 + ux * t, y0 + uy * t, x0 + ux * e, y0 + uy * e],
               fill=fill, width=width)
        t = e + off_px


def draw_courtyards(d, r, model, refs, *, side=None, color=None, dim=False,
                    locked=(), width_mm=0.12):
    for ref in refs:
        rect = model.rect(ref)
        if rect is None:
            continue
        own = model.side(ref)
        if side is not None and side not in model.sides(ref):
            continue
        if side is not None and own != side:
            # Far side of a through-hole part: its DRILLED-PAD box, which is
            # what legality.rect_on gates on.
            rect = model.far_rect(ref)
            if rect is None:
                continue
        col = color or (C_COURT_DIM if dim else
                        (C_COURT_B if own == 'B' else C_COURT_F))
        if ref in locked:
            col = C_LOCKED
        box = _rect_pts(r, rect)
        d.rectangle(box, outline=col, width=_w(r, width_mm))
        if ref in locked:      # hatch so "locked" reads without a legend
            d.line([box[0], box[1], box[2], box[3]], fill=col, width=_w(r, 0.06))


C_CONFLICT = (255, 64, 64)      # pad/hole legality conflicts
C_HOLE = (255, 160, 64)         # NPTH keepout circles


def draw_legality(d, r, model, *, side=None):
    """The mess, DRAWN (the run-2 imaging finding: off-board parts and pad
    conflicts were numbers in a caption, invisible in the picture the reads
    were mandated on). Red rings + connecting line per conflicting pad pair,
    orange circles at NPTH keepouts, dashed red extent for parts whose pad
    copper leaves the board bbox."""
    state = getattr(model, 'state', None)
    if state is None or getattr(state, 'legality_ctx', None) is None:
        return
    ctx = state.legality_ctx
    # NPTH keepout circles
    for ref in sorted(ctx.parts):
        p = state.parts.get(ref)
        if p is None:
            continue
        for (cx, cy, rad) in ctx.parts[ref].hole_circles(p.x, p.y, p.rot):
            x0, y0 = r.tf.pt(cx - rad, cy - rad)
            x1, y1 = r.tf.pt(cx + rad, cy + rad)
            d.ellipse([min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)],
                      outline=C_HOLE, width=_w(r, 0.08))
    # Findings are computed ONCE per model and shared with the JSON (run-4 G):
    # this used to re-run the O(n^2) pair sweep per panel.
    fnd = legality_findings(model)
    # off-board pad extents (dashed red)
    for ref, _amt in fnd['oob_refs_pad_copper']:
        p = state.parts.get(ref)
        if p is None:
            continue
        ext = ctx.parts[ref].extent(p.x, p.y, p.rot)
        if ext is None:
            continue
        box = _rect_pts(r, ext)
        w = _w(r, 0.14)
        on = off = max(4, w * 3)
        _dash(d, (box[0], box[1]), (box[2], box[1]), on, off, C_CONFLICT, w)
        _dash(d, (box[2], box[1]), (box[2], box[3]), on, off, C_CONFLICT, w)
        _dash(d, (box[2], box[3]), (box[0], box[3]), on, off, C_CONFLICT, w)
        _dash(d, (box[0], box[3]), (box[0], box[1]), on, off, C_CONFLICT, w)
    # conflicting pairs: ring at the pair midpoint + connecting line
    for a, bb, _mm in (fnd['pad_conflict_pairs_refs']
                       + fnd['hole_conflict_pairs_refs']):
        pa = state.parts.get(a)
        pb = state.parts.get(bb)
        if pa is None or pb is None:
            continue
        ax, ay = r.tf.pt(pa.x, pa.y)
        bx, by = r.tf.pt(pb.x, pb.y)
        d.line([ax, ay, bx, by], fill=C_CONFLICT, width=_w(r, 0.1))
        mx, my = (ax + bx) / 2, (ay + by) / 2
        rad = _w(r, 0.8, floor=6)
        d.ellipse([mx - rad, my - rad, mx + rad, my + rad],
                  outline=C_CONFLICT, width=_w(r, 0.12))
    # run-6: BODY STACKS (any-net pad intersection -- the assembly channel).
    # Both courtyards in fail-red, FILLED intersection of the pad extents:
    # run 5's C14-on-R14 was drawn as two ordinary dim rectangles and read
    # as clean; a stack must be unmissable in the picture the mandate-8
    # reads are performed on.
    for a, bb in fnd['body_overlap_pairs_refs']:
        pa = state.parts.get(a)
        pb = state.parts.get(bb)
        if pa is None or pb is None:
            continue
        for ref in (a, bb):
            rect = model.rect(ref)
            if rect is not None:
                d.rectangle(_rect_pts(r, rect), outline=C_CONFLICT,
                            width=_w(r, 0.16))
        ea = ctx.parts[a].extent(pa.x, pa.y, pa.rot)
        eb = ctx.parts[bb].extent(pb.x, pb.y, pb.rot)
        if ea is not None and eb is not None:
            ix = (max(ea[0], eb[0]), max(ea[1], eb[1]),
                  min(ea[2], eb[2]), min(ea[3], eb[3]))
            if ix[2] > ix[0] and ix[3] > ix[1]:
                d.rectangle(_rect_pts(r, ix), fill=C_CONFLICT)


def draw_ghosts(d, r, model, moves, *, width_mm=0.1):
    """Dashed rect at each part's seed pose -- the placement diff, drawn."""
    for m in moves:
        rect = model.rect(m['reference'])
        if rect is None:
            continue
        dx = m['from'][0] - m['to'][0]
        dy = m['from'][1] - m['to'][1]
        box = _rect_pts(r, (rect[0] + dx, rect[1] + dy, rect[2] + dx, rect[3] + dy))
        w = _w(r, width_mm)
        on, off = max(3, w * 3), max(3, w * 3)
        _dash(d, (box[0], box[1]), (box[2], box[1]), on, off, C_GHOST, w)
        _dash(d, (box[2], box[1]), (box[2], box[3]), on, off, C_GHOST, w)
        _dash(d, (box[2], box[3]), (box[0], box[3]), on, off, C_GHOST, w)
        _dash(d, (box[0], box[3]), (box[0], box[1]), on, off, C_GHOST, w)


def draw_arrows(d, r, moves, *, head_mm=0.7, width_mm=0.14, min_px=5):
    for m in moves:
        sx, sy = r.tf.pt(m['from'][0], m['from'][1])
        ex, ey = r.tf.pt(m['to'][0], m['to'][1])
        if math.hypot(ex - sx, ey - sy) < min_px:
            continue                      # too short to read; the ghost says it
        w = _w(r, width_mm)
        d.line([sx, sy, ex, ey], fill=C_ARROW, width=w)
        ang = math.atan2(ey - sy, ex - sx)
        h = max(4.0, r.tf.length(head_mm))
        for s in (+1, -1):
            a = ang + s * math.radians(150)
            d.line([ex, ey, ex + h * math.cos(a), ey + h * math.sin(a)],
                   fill=C_ARROW, width=w)


def draw_airwires(d, r, airwires, *, color=C_AIR, width_mm=0.05):
    w = _w(r, width_mm)
    for (x1, y1, x2, y2, _nid) in airwires:
        d.line([*r.tf.pt(x1, y1), *r.tf.pt(x2, y2)], fill=color, width=w)


def draw_ref_labels(d, r, model, refs, *, min_px=14, lo=11, hi=34):
    """Reference designators, sized to the PART rather than to the image.

    Scaling the font by image height alone gives an 8px label on a 400px render
    whatever the zoom -- drawn, technically present, and unreadable. A label is
    only useful if you can read it, so it is sized from the part's own footprint
    on screen and skipped entirely when the part is too small to carry one.
    """
    for ref in refs:
        rect = model.rect(ref)
        if rect is None:
            continue
        box = _rect_pts(r, rect)
        w, h = box[2] - box[0], box[3] - box[1]
        if max(w, h) < min_px:
            continue                      # too small to letter; the arrow says it
        # fit to the shorter side, but let a long ref use the longer one
        size = int(max(lo, min(hi, min(max(w, h) * 0.45,
                                       (w * 1.6) / max(1, len(ref))))))
        d.text(((box[0] + box[2]) / 2, (box[1] + box[3]) / 2), ref,
               fill=C_LABEL, font=load_font(size), anchor='mm',
               stroke_width=max(1, size // 9), stroke_fill=(12, 14, 16))


def _dim(c, k=0.32):
    return (int(c[0] * k), int(c[1] * k), int(c[2] * k))


def pad_fill_for(model, side=None):
    """OmniLayout's encoding: through-hole / front SMD / back SMD read
    differently. A colour and a filter over pads BoardRenderer already knows how
    to rasterize -- custom polygons, capsules, roundrect, rotated rects.

    On a PER-SIDE panel the far side's SMD pads are dimmed rather than dropped:
    they are still context you need (a back-side part is why a front trace has
    to go around), but they must not compete with the side you asked to see.
    Through-hole pads stay full brightness on both panels, because they are
    physically on both -- a hole cannot move on one side only.
    """
    def fill(p):
        if getattr(p, 'drill', 0):
            return C_PAD_THT               # on both sides, always full
        ref = getattr(p, 'component_ref', None)
        own = model.side(ref) if ref else 'F'
        base = C_PAD_B if own == 'B' else C_PAD_F
        return base if (side is None or own == side) else _dim(base)
    return fill


# ---------------------------------------------------------------------------
# Panels
# ---------------------------------------------------------------------------
class PanelSpec:
    """One image: a view, a side filter, what is prominent, and a caption."""

    def __init__(self, model, *, view=None, side=None, prominent=(), moves=(),
                 hot_nets=(), blocker_nets=(), pick_nets=(), label='',
                 opts=None):
        self.model = model
        self.view = view
        self.side = side
        self.prominent = set(prominent)
        self.moves = list(moves)
        self.hot_nets = set(hot_nets)
        self.blocker_nets = set(blocker_nets)
        self.pick_nets = set(pick_nets)     # ids, from --ratsnest-nets
        self.label = label
        self.o = opts or {}


def overlay_for(spec: PanelSpec):
    """Compose the primitives into one `fn(draw, renderer)` for frame()."""
    m, o = spec.model, spec.o
    all_refs = sorted(m.parts())
    prom = sorted(spec.prominent) or all_refs
    context = [r for r in all_refs if r not in spec.prominent] \
        if (spec.prominent and o.get('delta_first', True)) else []
    locked = {r for r in all_refs if getattr(m.parts()[r], 'locked', False)}

    hot = m.net_ids_for(spec.hot_nets)
    blocked = m.net_ids_for(spec.blocker_nets) - hot
    if spec.pick_nets:
        # Named nets win over both defaults: between "the nets that moved" and
        # "every net on the board" is the case that actually comes up -- the
        # handful you are chasing. Draw only those, in their own colour.
        quiet_ids = set(spec.pick_nets)
    elif o.get('ratsnest_all') or not spec.prominent:
        # Every net. Either the hairball switch, or there is no delta to be
        # "first" about -- delta-first with nothing prominent would draw NO
        # airwires at all, which is not a summary, it is a blank.
        quiet_ids = None
    else:
        # Delta-first: only the moved parts' nets plus the attributed ones. The
        # issue's limit 1 -- a full ratsnest on a dense board is exactly the
        # useless hairball KiCad already shows.
        quiet_ids = set()
        for ref in spec.prominent:
            p = m.parts().get(ref)
            if p is not None:
                quiet_ids |= set(p.nets)
    def _draw(d, r):
        if o.get('borders', True):
            if context:
                draw_courtyards(d, r, m, context, side=spec.side, dim=True)
            draw_courtyards(d, r, m, prom, side=spec.side, locked=locked)
        if o.get('pads', True):
            r.draw_pads(d, fill_for=pad_fill_for(m, spec.side))
        if o.get('ratsnest', True):
            ids = quiet_ids if quiet_ids is not None else None
            quiet = [a for a in m.airwires(ids)
                     if a[4] not in hot and a[4] not in blocked
                     and a[4] not in spec.pick_nets]
            draw_airwires(d, r, quiet, color=C_AIR)
            if spec.pick_nets:
                draw_airwires(d, r, m.airwires(spec.pick_nets),
                              color=C_AIR_PICK, width_mm=0.08)
            draw_airwires(d, r, m.airwires(blocked), color=C_AIR_BLOCK,
                          width_mm=0.07)
            draw_airwires(d, r, m.airwires(hot), color=C_AIR_FAIL, width_mm=0.09)
        if o.get('legality', True):
            draw_legality(d, r, m, side=spec.side)
        if spec.moves and o.get('ghosts', True):
            draw_ghosts(d, r, m, spec.moves)
        if spec.moves and o.get('arrows', True):
            draw_arrows(d, r, spec.moves)
        if o.get('labels', True):
            draw_ref_labels(d, r, m, prom)
        if o.get('legend', True):
            draw_legend(d, r, spec)
    return _draw


def draw_legend(d, r, spec) -> None:
    """A colour key, bottom-left.

    The encoding lived only in source comments, so reading a panel correctly
    depended on having read render_placement.py -- and the two colours most
    likely to be misread are the two that matter: solid red is a BODY STACK
    (parts physically overlapping, unfixable by any router) while a red ring is
    a clearance shortfall (often fixable). A reader who conflates them draws
    the wrong conclusion from the picture, which is worse than not looking.

    Only the keys this panel can actually show are drawn -- a legend listing
    arrows on a panel with no --before is itself misinformation.
    """
    rows = [(C_CONFLICT, 'dashed', 'pad copper off-board'),
            (C_CONFLICT, 'ring', 'pad/hole clearance short'),
            (C_CONFLICT, 'solid', 'BODY STACK - parts overlap'),
            (C_HOLE, 'ring', 'NPTH keepout'),
            (C_LOCKED, 'hatch', 'KiCad-locked (never moved)')]
    if spec.moves:
        rows.append((C_ARROW, 'arrow', 'moved since --before'))
    if spec.hot_nets:
        rows.append((C_AIR_FAIL, 'line', 'failed net'))
    try:
        # Overlays draw on the SUPERSAMPLED canvas, which is `ss` times the
        # output size -- so `r.H` (the output height) put the legend at 1/ss of
        # the way up the image, on top of the board, instead of at the bottom.
        # The caption escapes this because _label draws on the final image.
        ss = max(1, int(getattr(r, 'ss', 1)))
        W, H = r.W * ss, r.H * ss
        font = load_font(max(10, H // 78))
        pad, sw = 6 * ss, max(10, H // 90)
        lh = sw + 5
        h = lh * len(rows) + 8
        w = max(int(d.textlength(t, font=font)) for _, _, t in rows) + sw + 20
        y0 = H - h - pad
        d.rectangle([pad, y0, pad + w, y0 + h], fill=(0, 0, 0))
        for i, (col, kind, text) in enumerate(rows):
            yy = y0 + 4 + i * lh
            box = [pad + 6, yy, pad + 6 + sw, yy + sw]
            if kind == 'solid':
                d.rectangle(box, fill=col)
            elif kind == 'ring':
                d.ellipse(box, outline=col, width=2)
            elif kind == 'dashed':
                for k in range(0, sw, 4):
                    d.line([box[0] + k, box[1], box[0] + k + 2, box[1]], fill=col)
                    d.line([box[0] + k, box[3], box[0] + k + 2, box[3]], fill=col)
            elif kind == 'hatch':
                d.rectangle(box, outline=col, width=1)
                for k in range(0, sw, 3):
                    d.line([box[0] + k, box[3], box[0] + sw, box[1] + k], fill=col)
            elif kind == 'arrow':
                d.line([box[0], box[3], box[2], box[1]], fill=col, width=2)
            else:
                d.line([box[0], (box[1] + box[3]) // 2,
                        box[2], (box[1] + box[3]) // 2], fill=col, width=2)
            d.text((pad + 6 + sw + 6, yy - 1), text, fill=(225, 225, 225),
                   font=font)
    except Exception:                                          # noqa: BLE001
        pass          # a legend is never worth failing a render over


def crop_findings(model, view) -> Dict[str, int]:
    """How many legality findings lie INSIDE `view`.

    A finding counts when any part it names overlaps the rect -- for a pair,
    either member. That is deliberately inclusive: a stack half in frame is
    still the thing the crop is showing, and undercounting it would recreate
    the whole-board-tally problem in the other direction.
    """
    x0, y0, x1, y1 = view

    def _hit(ref):
        r = model.rect(ref)
        return bool(r) and not (r[2] < x0 or r[0] > x1 or r[3] < y0 or r[1] > y1)

    f = legality_findings(model)
    out = {}
    out['body stacks'] = sum(1 for p in f['body_overlap_pairs_refs']
                             if _hit(p[0]) or _hit(p[1]))
    out['pad pairs'] = sum(1 for p in f['pad_conflict_pairs_refs']
                           if _hit(p[0]) or _hit(p[1]))
    out['hole conflicts'] = sum(1 for p in f['hole_conflict_pairs_refs']
                                if _hit(p[0]) or _hit(p[1]))
    out['off-board'] = sum(1 for r, _ in (f['oob_refs_pad_copper']
                                          + f['oob_refs_courtyard']) if _hit(r))
    return out


def _ref_centres(model, refs):
    """(x, y) centre of each ref's courtyard rect, skipping refs with none."""
    out = {}
    for r in refs:
        rc = model.rect(r)
        if rc:
            out[r] = ((rc[0] + rc[2]) / 2.0, (rc[1] + rc[3]) / 2.0)
    return out


def _pocket_views(model, refs, gap: float, cap: int):
    """Spatially cluster `refs` and return [(view_rect, [ref, ...]), ...].

    Reuses the same `cluster_points` / `union_view` the route-failure `--focus`
    path uses -- the question "do these findings share one pocket or scatter"
    is identical whether the findings are failed nets or overlapping parts, and
    it is the question that decides local-fix vs systemic.
    """
    centres = _ref_centres(model, refs)
    if not centres:
        return []
    inv = {}
    for r, xy in centres.items():
        inv.setdefault((round(xy[0], 4), round(xy[1], 4)), []).append(r)
    out = []
    for cl in cluster_points(list(centres.values()), gap)[:cap]:
        v = union_view([(x, y, x, y) for x, y in cl], gap / 2)
        members = sorted({r for x, y in cl
                          for r in inv.get((round(x, 4), round(y, 4)), [])})
        out.append((v, members))
    out.sort(key=lambda t: -len(t[1]))
    return out


def _pair_key(entry):
    """A finding's identity across two boards: the refs, order-independent."""
    if len(entry) >= 2 and isinstance(entry[1], str):
        return tuple(sorted(entry[:2]))
    return (entry[0],)


def describe_pair(before_model, after_model, args):
    """What the move FIXED, what it BROKE, and what it left alone.

    Two panels side by side answer "does it look different". They do not answer
    the question a move is judged on, which is whether the specific findings
    moved -- and a reader comparing two counts (46 stacks then 46 stacks) will
    conclude nothing changed when in fact eleven were resolved and eleven new
    ones appeared somewhere else. Only the by-NAME diff shows that, and it is
    the same reason the ledger records failing nets by name rather than by
    count.
    """
    b, a = legality_findings(before_model), legality_findings(after_model)
    L, J = [], {}
    L.append("")
    L.append("WHAT THE MOVE DID  (findings by NAME, not by count -- a count "
             "that stays level hides a swap)")
    CATS = (('body stacks', 'body_overlap_pairs_refs'),
            ('pad-clearance pairs', 'pad_conflict_pairs_refs'),
            ('hole conflicts', 'hole_conflict_pairs_refs'),
            ('off-board (pad copper)', 'oob_refs_pad_copper'),
            ('off-board (courtyard)', 'oob_refs_courtyard'))
    for label, key in CATS:
        bs = {_pair_key(e) for e in b[key]}
        as_ = {_pair_key(e) for e in a[key]}
        fixed, new, kept = sorted(bs - as_), sorted(as_ - bs), sorted(bs & as_)
        J[key] = {'fixed': ['<->'.join(k) for k in fixed],
                  'new': ['<->'.join(k) for k in new],
                  'kept': len(kept), 'before': len(bs), 'after': len(as_)}
        if not (bs or as_):
            continue
        verdict = ('no change' if not fixed and not new
                   else f"{len(fixed)} fixed, {len(new)} NEW, {len(kept)} kept")
        L.append(f"  {label}: {len(bs)} -> {len(as_)}   [{verdict}]")
        if fixed:
            L.append("    fixed: " + ", ".join('<->'.join(k) for k in fixed[:8])
                     + ("..." if len(fixed) > 8 else ""))
        if new:
            L.append("    NEW:   " + ", ".join('<->'.join(k) for k in new[:8])
                     + ("..." if len(new) > 8 else ""))
    net_new = sum(len(v['new']) for v in J.values())
    net_fixed = sum(len(v['fixed']) for v in J.values())
    J['net_fixed'], J['net_new'] = net_fixed, net_new
    L.append("")
    if net_new and not net_fixed:
        L.append(f"  VERDICT: this move introduced {net_new} finding(s) and "
                 f"resolved none.")
    elif net_new:
        L.append(f"  VERDICT: {net_fixed} resolved, {net_new} introduced. A "
                 f"level count is NOT no-change -- read the names above.")
    elif net_fixed:
        L.append(f"  VERDICT: {net_fixed} resolved, none introduced.")
    else:
        L.append("  VERDICT: no legality finding changed identity.")
    for mk, mlabel in (('crossings', 'crossings'), ('hpwl', 'hpwl mm'),
                       ('overlap_area', 'overlap mm2')):
        bm, am = before_model.metrics.get(mk), after_model.metrics.get(mk)
        if bm is not None and am is not None:
            arrow = '->' if abs(am - bm) > 1e-9 else '=='
            J[mk] = {'before': round(bm, 4), 'after': round(am, 4)}
            L.append(f"  {mlabel}: {bm:.2f} {arrow} {am:.2f}"
                     + ("   (worse)" if am > bm + 1e-9 else
                        "   (better)" if am < bm - 1e-9 else ""))
    return "\n".join(L), J


def describe(model, fnd, moves, args, panel_paths):
    """Say what is IN the picture, then say what to run to see it better.

    A render was being produced and not read -- across a whole campaign, once --
    and the honest reason is that a picture answers nothing until somebody
    forms a sentence about it. The JSON already carried the refs; nobody was
    going to assemble them into "these 46 stacks sit in 3 pockets, the biggest
    is around U1" by hand, every time.

    So the tool does it: the findings, grouped where they physically are, with
    the crop command for each group and the flags that clear the clutter. The
    numbers still come from the checklist -- this narrates them, it does not
    re-derive them.
    """
    L, J = [], {}
    n_stack = len(fnd['body_overlap_pairs_refs'])
    n_pad = len(fnd['pad_conflict_pairs_refs'])
    n_hole = len(fnd['hole_conflict_pairs_refs'])
    oob_pc = fnd['oob_refs_pad_copper']
    oob_cy = fnd['oob_refs_courtyard']

    L.append("WHAT THIS PANEL SHOWS")
    if not (n_stack or n_pad or n_hole or oob_pc or oob_cy):
        L.append("  no legality findings: nothing off the outline, no part on "
                 "another part, no hole conflict.")
    if oob_pc:
        L.append(f"  {len(oob_pc)} part(s) with pad COPPER off the outline "
                 f"(dashed red): "
                 + ", ".join(f"{r} by {a:g}mm" for r, a in oob_pc[:8]))
        L.append("    -> their nets cannot be routed at all; this is "
                 "placement-shaped, not routing-shaped.")
    if oob_cy:
        L.append(f"  {len(oob_cy)} part(s) with COURTYARD off the outline: "
                 + ", ".join(f"{r} by {a:g}mm" for r, a in oob_cy[:8]))
    if n_stack:
        L.append(f"  {n_stack} BODY STACK(S) -- pads of two parts physically "
                 f"overlapping (solid red). Not a clearance graze; no router "
                 f"can fix one.")
    if n_pad:
        L.append(f"  {n_pad} pad-clearance pair(s) (red rings).")
    if n_hole:
        L.append(f"  {n_hole} hole conflict(s) (orange) -- a fab blocker that "
                 f"is not a body overlap, so `blocking` does not count it.")
    if fnd['locked_refs']:
        L.append(f"  {len(fnd['locked_refs'])} KiCad-LOCKED part(s) (hatched): "
                 + ", ".join(fnd['locked_refs'][:10]))
        L.append("    -> a search may not settle a conflict by moving the "
                 "locked side; the OTHER part must move.")
    if moves:
        big = sorted(moves, key=lambda m: -m.get('dist', 0.0))[:6]
        L.append(f"  {len(moves)} part(s) moved vs --before (yellow arrows); "
                 f"largest: "
                 + ", ".join(f"{m['reference']} {m.get('dist', 0.0):.2f}mm"
                             for m in big))

    # --- the WORST findings, each with a crop that frames exactly it ---------
    #
    # Clustering everything implicated does NOT work here and the failure is
    # instructive: on a 50x49mm board with 54 implicated parts, single-linkage
    # at the default 10mm gap merges them into one 42mm "pocket" -- i.e. the
    # whole board, which is the panel you already have. A crop is only useful
    # when it frames ONE finding, so rank by severity and frame the pair.
    worst = []
    for a_, b_, sf in fnd['pad_conflict_pairs_refs']:
        worst.append((sf, 'pad-clearance', (a_, b_), f'{sf:g}mm short'))
    for a_, b_, sf in fnd['hole_conflict_pairs_refs']:
        worst.append((sf + 1e6, 'hole', (a_, b_), f'{sf:g}mm short'))
    for r, amt in oob_pc:
        worst.append((amt + 1e9, 'off-board pad copper', (r,), f'{amt:g}mm out'))
    for r, amt in oob_cy:
        worst.append((amt, 'off-board courtyard', (r,), f'{amt:g}mm out'))
    _stackset = {tuple(sorted(p[:2])) for p in fnd['body_overlap_pairs_refs']}
    worst.sort(key=lambda t: -t[0])

    board = args.board
    same = []
    # The EFFECTIVE clearance, always -- this line's whole job is "run this and
    # get the same numbers", and a run that omitted --clearance reproduces only
    # if the board's project has not moved since. Naming it pins the render to
    # a value instead of to a file that can change underneath it.
    _clr = (model.floor_knobs.get('clearance') or {}).get('value')
    if _clr is not None:
        same.append(f"--clearance {_clr:g}")
    if args.ignore_nets:
        same.append("--ignore-nets " + " ".join(args.ignore_nets))
    same_s = (" " + " ".join(same)) if same else ""

    shots = []
    for _sev, kind, refs, note in worst[:args.max_focus]:
        rects = [model.rect(r) for r in refs if model.rect(r)]
        if not rects:
            continue
        v = union_view(rects, 1.5, min_size=6.0)
        stacked = tuple(sorted(refs)) in _stackset
        shots.append({'kind': kind, 'refs': list(refs), 'note': note,
                      'body_stack': stacked, 'view': [round(c, 3) for c in v]})
    J['worst'] = shots
    if shots:
        L.append("")
        L.append(f"THE WORST {len(shots)}, framed one per crop")
        for i, s in enumerate(shots, 1):
            L.append(f"  {i}. {' <-> '.join(s['refs'])}  {s['kind']}, "
                     f"{s['note']}"
                     + ("  [also a BODY STACK]" if s['body_stack'] else ""))
            v = s['view']
            L.append(f"     python3 -X utf8 py_tools/render_placement.py {board}"
                     f" --view {v[0]},{v[1]},{v[2]},{v[3]}{same_s}"
                     f" --no-ratsnest -o wk/worst{i}.png")
    L.append("")
    L.append("SEE THE WHOLE PICTURE AGAIN, uncluttered")
    L.append(f"  python3 -X utf8 py_tools/render_placement.py {board}{same_s} "
             f"--no-ratsnest --no-labels -o wk/clean.png")
    L.append(f"  python3 -X utf8 py_tools/render_placement.py {board}{same_s} "
             f"--focus -o wk/focus/    # a panel per legality pocket")
    L.append("")
    L.append("DECLUTTER  (add these when the panel is too busy to read)")
    L.append("  --no-ratsnest    drop the airwires -- the biggest source of "
             "visual noise on a dense board")
    L.append("  --no-labels      drop ref designators (keep them on crops, "
             "drop them board-wide)")
    L.append("  --no-ghosts --no-arrows   drop the --before overlay when you "
             "want the CURRENT state alone")
    L.append("  --ratsnest-nets '<glob>'  show ONE bus instead of all of them")
    L.append("  --per-side / --flat       split or merge the two sides")
    if not args.ignore_nets:
        L.append("  --ignore-nets <plane nets>  a plane-routed rail's airwire "
                 "is a fiction; excluding it is what makes crossings/hpwl "
                 "reproduce the optimizer's own numbers")
    # Deliberately NOT storing the narrative text in the JSON. It is ~2kB of
    # prose that duplicates what was just printed, and an oversized payload is
    # the exact failure `converge record --score-file` had to be added for.
    # The structured `worst` entries carry everything a later reader needs.
    return "\n".join(L), J


def caption(spec: PanelSpec, extra: Optional[Dict] = None) -> str:
    """The metrics strip. Without it a reader adopts the wrong heuristic --
    "lots moved, looks broken" and "barely moved, looks safe" are both wrong,
    and only these numbers carry the verdict (#431 limit 3)."""
    m = spec.model.metrics
    bits = [spec.label] if spec.label else []
    if spec.side:
        bits.append(f"side {spec.side}")
    if spec.view:
        # COUNT THE FINDINGS THAT ARE ACTUALLY IN THIS CROP.
        #
        # `spec.model.metrics` is whole-board, always -- it comes from the
        # optimizer's model, which has no notion of a view. Labelling that
        # "WHOLE-BOARD" was honest but not much use: a reader looking at a 19mm
        # crop still had no idea how many of the 46 stacks were in front of
        # them. So report both, local first, and keep the whole-board scope
        # label on the rest.
        v = spec.view
        loc = crop_findings(spec.model, v)
        local = ", ".join(f"{n} {k}" for k, n in loc.items() if n)
        bits.append(f"CROP {v[0]:.1f},{v[1]:.1f}-{v[2]:.1f},{v[3]:.1f}mm")
        bits.append("IN CROP: " + (local if local else "no findings"))
        bits.append("board totals below")
    for k, fmt in (('crossings', '{:.0f}'), ('hpwl', '{:.1f}mm')):
        if m.get(k) is not None:
            bits.append(f"{k} " + fmt.format(m[k]))
    if m.get('overlap_area') is not None:
        bits.append(f"overlap {m['overlap_area']:.2f}mm2")
    if m.get('pad_intersection_pairs'):
        # run-6: a stack is never cosmetic -- name it in the caption (the
        # run-5 caption printed the aggregate scalar next to a zero-pair
        # checklist and the stack read as noise)
        bits.append(f"BODY-STACKS {m['pad_intersection_pairs']:.0f}")
    if m.get('pad_conflict_pairs') is not None:
        bits.append(f"pad-conflicts {m['pad_conflict_pairs']:.0f}")
    if m.get('hole_shortfall'):
        bits.append(f"hole-conflict {m['hole_shortfall']:.2f}mm")
    bits.append("oob n/a (no Edge.Cuts)" if spec.model.no_outline
                else (f"oob {m['oob_count']:.0f}" if m.get('oob_count') is not None
                      else ''))
    for k in ('failures', 'iterations', 'vias'):
        if extra and extra.get(k) is not None:
            v = extra[k]
            bits.append(f"{k} {v:,}" if isinstance(v, int) else f"{k} {v}")
    if extra and extra.get('verdict'):
        bits.append(extra['verdict'])
    if spec.moves:
        bits.append(f"{len(spec.moves)} moved")
    return "  |  ".join(b for b in bits if b)


def render_panel(spec: PanelSpec, *, size=1600, supersample=2, extra=None):
    r = BoardRenderer(spec.model.pcb, size=size, supersample=supersample,
                      show_pads=False, view=spec.view,
                      layers=([spec.side + '.Cu']
                              if spec.side in ('F', 'B')
                              and (spec.side + '.Cu') in
                              spec.model.pcb.board_info.copper_layers else None))
    return r.frame(overlays=[overlay_for(spec)], label=caption(spec, extra))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _bool_pair(parser, name, default, help_on):
    dest = name.replace('-', '_')
    g = parser.add_mutually_exclusive_group()
    g.add_argument(f'--{name}', dest=dest, action='store_true',
                   default=default, help=help_on)
    g.add_argument(f'--no-{name}', dest=dest, action='store_false')


def build_parser():
    p = argparse.ArgumentParser(
        description="Headless PNG stills of placement status (#431).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  render_placement.py board.kicad_pcb -o state.png
  render_placement.py placed.kicad_pcb --before seed.kicad_pcb -o delta.png
  render_placement.py board.kicad_pcb --list-groups --group-by sheet
  render_placement.py board.kicad_pcb --zoom-group sheet:58d913ec --per-side -o out/
""")
    p.add_argument('board', nargs='?', help='the board to render ("after")')
    p.add_argument('--before', metavar='BOARD',
                   help='seed board; enables ghost rects and displacement arrows')
    p.add_argument('-o', '--output', help='PNG path (one panel) or directory')
    p.add_argument('--summary-json', metavar='FILE',
                   help='route.py JSON_SUMMARY or a loop round log, naming the '
                        'failed and blocker nets to highlight')
    p.add_argument('--zoom-group', metavar='BLOCK',
                   help='frame one placement block (same names as route.py --group)')
    p.add_argument('--view', metavar='X0,Y0,X1,Y1',
                   help='frame an explicit world rect in board mm (the '
                        'question-scoped crop: an intrusion, an edge row, a '
                        'DRC cluster). Mutually exclusive with --zoom-group.')
    p.add_argument('--group-by', default='auto', metavar='SOURCES',
                   help='how blocks are derived (default: auto = kicad,sheet)')
    p.add_argument('--list-groups', action='store_true',
                   help='list the blocks --group-by would derive, and exit')
    p.add_argument('--focus', action='store_true',
                   help='also emit one cropped panel per failed-net cluster')
    p.add_argument('--focus-gap', type=float, default=10.0, metavar='MM')
    p.add_argument('--max-focus', type=int, default=6)
    p.add_argument('--zoom-pad', type=float, default=2.0, metavar='MM')
    p.add_argument('--per-side', action='store_true',
                   help='force an F and a B panel (run-6: boards with '
                        'back-side parts get per-side panels by DEFAULT; '
                        'this flag only matters with --flat semantics)')
    p.add_argument('--flat', action='store_true',
                   help='one flattened panel even when the board has '
                        'back-side parts (the pre-run-6 default; a B part '
                        'draws over an F part with no distinction)')
    _bool_pair(p, 'borders', True, 'draw component courtyards (default: on)')
    _bool_pair(p, 'labels', True, 'draw component references (default: on)')
    _bool_pair(p, 'legend', True,
               'draw the colour key bottom-left (default: on). The encoding '
               'used to live only in source comments, and the two colours most '
               'easily confused are the two that matter: solid red is a body '
               'stack no router can fix, a red ring is a clearance shortfall.')
    _bool_pair(p, 'ratsnest', True, 'draw airwires (default: on)')
    _bool_pair(p, 'pads', True, 'draw pads (default: on)')
    _bool_pair(p, 'ghosts', True, 'draw seed rects when --before is given')
    _bool_pair(p, 'legality', True,
               'draw the legality overlay: pad/hole conflict rings, NPTH '
               'keepouts, off-board pad extents (default: on)')
    _bool_pair(p, 'arrows', True, 'draw displacement arrows when --before is given')
    _bool_pair(p, 'delta-first', True,
               'moved parts prominent, everything else faint (default: on)')
    p.add_argument('--ratsnest-nets', nargs='+', metavar='PATTERN',
                   help='draw airwires for JUST these nets, in their own '
                        'colour, and label the parts that own them. Same glob '
                        'syntax as route.py --nets, exclusions included '
                        "(e.g. '*USB*' '/CLK*' '!*_N'). Between the nets that "
                        'moved and every net on the board, this is the case '
                        'that actually comes up: the handful you are chasing')
    p.add_argument('--ratsnest-all', action='store_true',
                   help='draw EVERY net, not just the moved/attributed ones. The '
                        'hairball switch: on a dense board this reproduces exactly '
                        'the unreadable ratsnest KiCad already shows')
    p.add_argument('--clearance', type=float, default=None,
                   help="pad clearance in mm for the legality/halo metrics. "
                        "DEFAULT: the board's own Default net-class clearance, "
                        "else its min_clearance constraint, else "
                        f"{defaults.CLEARANCE}. The effective value and its "
                        "source are printed, and land in the JSON as "
                        "instrument.floors -- this flag documented no default "
                        "at all, and four renders in one run were graded at a "
                        "clearance nothing recorded")
    # Same spelling and semantics as place_optimize's. Without it this tool
    # cannot reproduce a run's crossings/hpwl whenever the optimizer was given
    # --ignore-nets -- which is the normal case, since the plane nets have to be
    # excluded from airwire scoring. The gap turned the "re-measure the written
    # board" check into a false-alarm generator: GND alone took placedA from
    # 53 crossings to 116, which reads as a corrupted write and is not one.
    p.add_argument('--ignore-nets', nargs='+', default=None, metavar='NET',
                   help='net-name globs to exclude from airwire scoring; pass '
                        'the same set given to place_optimize --ignore-nets, '
                        'or crossings/hpwl will not match its JSON_SUMMARY')
    p.add_argument('--size', type=int, default=1600)
    p.add_argument('--supersample', type=int, default=2)
    p.add_argument('--json', action='store_true',
                   help='print a JSON_SUMMARY line with per-panel views + metrics')
    p.add_argument('--json-out', metavar='PATH', default=None,
                   help='ALSO write the summary document to a file (implies '
                        '--json). A separate flag rather than an optional '
                        'argument on --json, so `--json BOARD` can never '
                        'swallow the positional (run-4 G1)')
    p.add_argument('--pair', action='store_true',
                   help='render the --before board AND this one as separate '
                        'panels with byte-identical instrument settings, and '
                        'diff their legality findings BY NAME. `--before` '
                        'alone overlays ghosts and arrows on one panel, which '
                        'shows what MOVED; --pair shows what the move FIXED '
                        'and BROKE, which is what it should be judged on.')
    p.add_argument('--no-describe', action='store_true',
                   help='suppress the WHAT THIS PANEL SHOWS / LOOK CLOSER / '
                        'DECLUTTER narrative. On by default: a render that '
                        'does not say what is in it gets produced and not '
                        'read, which is measured, not hypothetical.')
    p.add_argument('--gate', action='store_true',
                   help='Exit 4 when the checklist finds anything: a part off '
                        'the outline, a pad-clearance or body-overlap pair, a '
                        'hole conflict, or a move count that disagrees with '
                        '--expect-moved. Default stays exit 0 -- seeing a '
                        'broken board is this tool\'s job -- but a caller who '
                        'wants a verdict should not have to re-read the JSON.')
    p.add_argument('--expect-moved', type=int, default=None, metavar='N',
                   help='the number of parts the step claims it moved; the '
                        "JSON checklist then carries d={moved, expected, "
                        "match} -- mandate 8's question (d), quotable instead "
                        'of recalled (run-4 G5)')
    p.add_argument('--quiet', action='store_true')
    return p


def net_pattern_report(pcb, patterns, flag: str) -> dict:
    """Did the net list this render was given actually MATCH anything?

    There was no field in the `instrument` block where "61 requested, 51
    matched" could ever appear, and that absence is exactly why a mangled net
    list went undetected: `board_score` published `--ignore-nets` candidates
    double-escaped, 10 of 61 matched nothing, and the render came back hpwl
    +45.6% / crossings +129.5% on a board that had not changed. Both numbers
    were internally consistent; the render simply scored a different net
    population than the one asked for, and said nothing.

    A pattern that matches nothing is ALWAYS worth reporting, even when it is
    intentional (a glob written for a family of boards). The cost of the note
    is a line; the cost of its absence was measured above.

    Returns per-pattern truth, not just a total, because the total cannot name
    the offender: `matched` is how many of the given patterns hit >= 1 net.
    """
    import fnmatch
    names = [n.name for n in pcb.nets.values() if n.name]
    pats = list(patterns or ())
    hit = {p: sum(1 for nm in names if fnmatch.fnmatch(nm, p)) for p in pats}
    unmatched = sorted(p for p, c in hit.items() if not c)
    return {'flag': flag,
            'requested': len(pats),
            'matched': len(pats) - len(unmatched),
            'unmatched': unmatched,
            'nets_matched': sum(1 for nm in names
                                if any(fnmatch.fnmatch(nm, p) for p in pats))}


def warn_unmatched(report: dict) -> None:
    """Say it on stderr, where a non-zero result cannot be scrolled past."""
    if not report.get('unmatched'):
        return
    shown = ', '.join(repr(p) for p in report['unmatched'][:8])
    more = '' if len(report['unmatched']) <= 8 else \
        f" (+{len(report['unmatched']) - 8} more)"
    print(f"WARNING: {report['flag']} -- {report['requested']} requested, "
          f"{report['matched']} matched. These matched NO net on this board: "
          f"{shown}{more}. The metrics below were computed over a DIFFERENT "
          f"net population than the one you asked for. A name that is merely "
          f"misspelled or mis-escaped is indistinguishable here from a net "
          f"that does not exist -- check it against the board.", file=sys.stderr)


def _load_summary(path):
    """Failed + blocker net names from a JSON_SUMMARY file or a route log.

    Uses place_route_loop's OWN arithmetic (metrics_from_summary, split out for
    exactly this) rather than re-deriving failure counts here.
    """
    if not path or not os.path.isfile(path):
        return [], [], {}
    txt = open(path, encoding='utf-8', errors='replace').read()
    summary = None
    try:
        summary = json.loads(txt)
    except ValueError:
        from place_route_loop import merge_route_summaries
        summary = merge_route_summaries(txt)
    if not summary:
        return [], [], {}
    from place_route_loop import metrics_from_summary
    m = metrics_from_summary(summary, txt)
    return m['failed_nets'], m['blockers'], m


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.board:
        parser.error('a board is required')

    pcb = parse_kicad_pcb(args.board)

    if args.list_groups:
        from placement.groups import derive_groups, parse_sources, short_name
        blocks = derive_groups(pcb, parse_sources(args.group_by))
        if not blocks:
            print(f"No placement blocks from sources {args.group_by!r}.")
            return 0
        print(f"{len(blocks)} placement block(s) from {args.group_by!r}:")
        for n, refs in sorted(blocks.items(), key=lambda kv: (-len(kv[1]), kv[0])):
            back = sum(1 for r in refs
                       if (pcb.footprints[r].layer or '').startswith('B'))
            print(f"  {short_name(n):34s} parts={len(refs):3d}  "
                  f"front={len(refs) - back:3d} back={back:3d}")
        return 0

    # The renderer WARNS where the placement CLIs refuse: being able to SEE an
    # unplaced board is the whole point of having a renderer for one.
    from placement.placement_state import gate_or_exit
    state = gate_or_exit(pcb, args.board, 'render_placement.py', warn_only=True)

    # DECLARED vs MATCHED, for every net list this run was handed. Computed even
    # when the list is empty so the fields exist unconditionally -- a key that
    # appears only on failure is a key no reader knows to look for.
    net_lists = {'ignore_nets': net_pattern_report(pcb, args.ignore_nets,
                                                   '--ignore-nets'),
                 'ratsnest_nets': net_pattern_report(pcb, args.ratsnest_nets,
                                                     '--ratsnest-nets')}
    for _rep in net_lists.values():
        warn_unmatched(_rep)

    ignore_ids = None
    if args.ignore_nets:
        import fnmatch
        ignore_ids = {nid for nid, net in pcb.nets.items()
                      if any(fnmatch.fnmatch(net.name, pat)
                             for pat in args.ignore_nets)}
        if not args.quiet:
            _r = net_lists['ignore_nets']
            print(f"Ignoring {len(ignore_ids)} nets for airwire scoring "
                  f"({_r['matched']}/{_r['requested']} patterns matched)")

    # exact=True is not a default here, it is a requirement: the render path
    # below reads `model.state` unconditionally (`state.parts`), so a stateless
    # model raises AttributeError before anything is drawn. There used to be a
    # `--metrics {exact,none}` flag suggesting otherwise; it was never read, and
    # wiring it up produced exactly that crash. Removed rather than left
    # lying -- a flag that accepts a value and changes nothing reads as a knob
    # somebody already thought about. Making `none` real means teaching the
    # render path to draw without a state, which is a different change.
    model = PlacementModel(pcb, args.board, exact=True,
                           quench_kwargs={'clearance': args.clearance,
                                          'ignore_net_ids': ignore_ids})

    # WHICH FLOOR, and WHERE FROM -- the same disclosure board_score makes with
    # floors.source. Four renders in the measured run omitted --clearance and
    # nothing in their output said what was used instead.
    if not args.quiet:
        _k = model.floor_knobs
        print("floors     " + ", ".join(
            f"{n.replace('_', ' ')} {d['value']}mm [{d['source']}]"
            for n, d in _k.items()))

    moves = moved_parts(parse_kicad_pcb(args.before), pcb) if args.before else []

    # --pair: build a SECOND model on the before board, with byte-identical
    # instrument settings. Same clearance, same ignored nets -- otherwise the
    # two sets of metrics are not comparable and the delta below is fiction.
    before_model = None
    if args.pair:
        if not args.before:
            print("render_placement: --pair needs --before <the board this one "
                  "came from>", file=sys.stderr)
            return 2
        _bpcb = parse_kicad_pcb(args.before)
        _bignore = set()
        if args.ignore_nets:
            _bignore = {nid for nid, net in _bpcb.nets.items()
                        if net.name and any(fnmatch.fnmatch(net.name, pat)
                                            for pat in args.ignore_nets)}
        # Pass the AFTER board's RESOLVED floors explicitly, rather than the
        # unresolved --clearance. Board-first resolution reads each board's own
        # project sibling, and the two boards need not agree: a routing chain's
        # DRC writeback lowers the project's clearance to whatever was routed
        # (CLAUDE.md, the fab-floor ratchet), so `before` and `after` can carry
        # different floors and the pair would then be graded at two different
        # clearances while claiming to be one instrument. Byte-identical
        # settings is the whole contract of --pair.
        _pair_floors = {n: d['value'] for n, d in model.floor_knobs.items()}
        before_model = PlacementModel(
            _bpcb, args.before, exact=True,
            quench_kwargs={'ignore_net_ids': _bignore, **_pair_floors})
    failed, blockers, route_metrics = _load_summary(args.summary_json)
    prominent = {m['reference'] for m in moves} | \
        model.refs_of_nets(model.net_ids_for(failed))

    pick = model.net_ids_matching(args.ratsnest_nets)
    if args.ratsnest_nets and not pick:
        print(f"  no nets match {' '.join(args.ratsnest_nets)}", file=sys.stderr)
    if pick:
        # The parts owning a named net become prominent, so "show me /CLK" also
        # labels and un-dims the parts it runs between -- a bare set of wires
        # with nothing identified is not much of an answer.
        prominent |= model.refs_of_nets(pick)

    opts = {'borders': args.borders, 'labels': args.labels,
            'ratsnest': args.ratsnest, 'pads': args.pads,
            'ghosts': args.ghosts, 'arrows': args.arrows,
            'legality': args.legality,
            'delta_first': args.delta_first, 'ratsnest_all': args.ratsnest_all}

    view = None
    tag = os.path.splitext(os.path.basename(args.board))[0]
    if args.view and args.zoom_group:
        print("ERROR: --view and --zoom-group both frame the panel; pick one",
              file=sys.stderr)
        return 2
    if args.view:
        from route_render import parse_view
        try:
            view = parse_view(args.view)
        except ValueError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return 2
        tag += '_view'
    elif args.zoom_group:
        from group_routing import GroupRoutingError, block_refs, resolved_name
        from placement.groups import parse_sources, short_name
        srcs = parse_sources(args.group_by)
        try:
            refs = block_refs(pcb, args.zoom_group, srcs)
        except GroupRoutingError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return 2
        full = resolved_name(pcb, args.zoom_group, srcs)
        if full and full != args.zoom_group and not args.quiet:
            print(f"  --zoom-group {args.zoom_group!r} resolved to block {full!r}")
        view = union_view([model.rect(r) for r in refs], args.zoom_pad)
        prominent |= set(refs)
        tag = short_name(full or args.zoom_group).replace(':', '_')
    elif state.unplaced:
        # A pile renders as a dot in the corner of an empty board unless the
        # frame follows the PARTS rather than the outline.
        view = union_view([model.rect(r) for r in model.parts()], 5.0)

    # Run-6: per-side panels are the DEFAULT whenever the board actually
    # carries back-side footprints. The flattened single panel draws a
    # B-side part on top of an F-side part with no distinction, and that is
    # exactly how run 5's JP1(B)-under-SW1(F) -- correct, identical on the
    # human board -- read as an overlap to the human eye. --per-side keeps
    # forcing panels; --flat restores the old single view.
    back = sum(1 for fp in pcb.footprints.values()
               if (fp.layer or '').startswith('B'))
    want_sides = args.per_side or (back > 0 and not args.flat)
    sides = ('F', 'B') if want_sides else (None,)
    if args.per_side and back == 0 \
            and 'B.Cu' not in pcb.board_info.copper_layers:
        sides = (None,)
        if not args.quiet:
            print("  (no back-side footprints and no B.Cu: skipping the B panel)")

    panels = []
    if before_model is not None:
        # BEFORE first, so a reader scanning the panel list sees them in the
        # order the change happened. No ghosts/arrows on it -- it IS the ghost.
        _bopts = dict(opts) if isinstance(opts, dict) else opts
        for side in sides:
            panels.append(PanelSpec(before_model, view=view, side=side,
                                    prominent=prominent, moves=(),
                                    hot_nets=failed, blocker_nets=blockers,
                                    pick_nets=pick,
                                    label=f"BEFORE {os.path.basename(args.before)}",
                                    opts=_bopts))
    for side in sides:
        panels.append(PanelSpec(model, view=view, side=side, prominent=prominent,
                                moves=moves, hot_nets=failed,
                                blocker_nets=blockers, pick_nets=pick,
                                label=(f"AFTER {tag}" if before_model is not None
                                       else tag),
                                opts=opts))
    _leg_pockets = []
    if args.focus and not args.summary_json:
        # Run-4 G7 made --focus warn here, because its clusters came from the
        # route summary's failed nets and without one it emitted a single panel
        # and said nothing. But the same question -- one pocket or scattered --
        # is worth asking of the LEGALITY findings, which need no route at all,
        # and on a copper-free board that is the only version of the question
        # available. So --focus now clusters those instead of refusing.
        _f = legality_findings(model)
        _hot = sorted({r for p in _f['body_overlap_pairs_refs'] for r in p[:2]}
                      | {r for p in _f['pad_conflict_pairs_refs'] for r in p[:2]}
                      | {r for p in _f['hole_conflict_pairs_refs'] for r in p[:2]}
                      | {r for r, _ in _f['oob_refs_pad_copper']}
                      | {r for r, _ in _f['oob_refs_courtyard']})
        _leg_pockets = _pocket_views(model, _hot, args.focus_gap, args.max_focus)
        if not _leg_pockets:
            print("  NOTE: --focus found nothing to focus on -- no route "
                  "summary was given and this board has no legality findings.",
                  file=sys.stderr)
        for i, (fview, members) in enumerate(_leg_pockets):
            panels.append(PanelSpec(model, view=fview, side=None,
                                    prominent=prominent, moves=moves,
                                    hot_nets=failed, blocker_nets=blockers,
                                    pick_nets=pick,
                                    label=f"{tag} pocket {i + 1} "
                                          f"({', '.join(members[:3])})",
                                    opts=opts))
    if args.focus and failed:
        pts = model.net_points(model.net_ids_for(failed))
        for i, cl in enumerate(cluster_points(pts, args.focus_gap)[:args.max_focus]):
            fview = union_view([(x, y, x, y) for x, y in cl], args.focus_gap / 2)
            panels.append(PanelSpec(model, view=fview, side=None,
                                    prominent=prominent, moves=moves,
                                    hot_nets=failed, blocker_nets=blockers,
                                    pick_nets=pick,
                                    label=f"{tag} focus {i + 1}", opts=opts))

    extra = None
    if route_metrics:
        extra = {'failures': route_metrics.get('failures'),
                 'iterations': route_metrics.get('iterations'),
                 'vias': route_metrics.get('vias')}

    out = args.output or (os.path.splitext(args.board)[0] + '_placement.png')
    # `-o` semantics (run-4 G4). A directory target is one that LOOKS like a
    # directory: trailing separator, an existing directory, or no extension.
    # `-o wk/x.png --per-side` used to os.makedirs('wk/x.png') -- a DIRECTORY
    # literally named x.png -- which make_film then globbed as a card and
    # died on (PermissionError reading a directory), and which no reader
    # expects. Multi-panel runs against a `.png` target now write stem-
    # suffixed SIBLING FILES (x_F.png, x_B.png, x_focus1.png), named from
    # the -o stem rather than the board stem.
    looks_like_dir = bool(args.output) and (
        args.output.endswith(('/', os.sep))
        or os.path.isdir(args.output)
        or not os.path.splitext(args.output)[1])
    as_dir = looks_like_dir
    multi_file = len(panels) > 1 and not as_dir
    if as_dir:
        os.makedirs(out, exist_ok=True)
    written = []
    written_paths = set()
    stem, ext = os.path.splitext(out)
    for i, spec in enumerate(panels):
        img = render_panel(spec, size=args.size, supersample=args.supersample,
                           extra=extra)
        # The suffix must DISTINGUISH the panel, and it only knew about `side`
        # and the literal word "focus". So --pair with --flat gave both panels
        # an empty suffix and the AFTER silently overwrote the BEFORE (one file
        # where the JSON promised two), and the legality-pocket panels collided
        # the same way because their label says "pocket". Derive it from the
        # label, and guarantee uniqueness rather than trusting the derivation.
        suffix_bits = []
        if spec.side:
            suffix_bits.append(spec.side)
        _lab = (spec.label or '').lower()
        if 'before' in _lab:
            suffix_bits.append('before')
        elif 'after' in _lab:
            suffix_bits.append('after')
        if 'focus' in _lab:
            suffix_bits.append(f"focus{i}")
        elif 'pocket' in _lab:
            suffix_bits.append(f"pocket{i}")
        if as_dir:
            path = os.path.join(out, "_".join([tag] + suffix_bits) + ".png")
        elif multi_file:
            path = stem + "".join("_" + s for s in suffix_bits) + (ext or '.png')
        else:
            path = out
        if multi_file or as_dir:
            _base, _e = os.path.splitext(path)
            _n = 2
            while path in written_paths:
                path = f"{_base}_{_n}{_e}"
                _n += 1
        written_paths.add(path)
        img.save(path)
        written.append(path)
        if not args.quiet:
            print(f"  wrote {path}  ({img.size[0]}x{img.size[1]})")

    if args.json or args.json_out:
        fnd = legality_findings(model)
        doc = {
            'panels': [{'label': s.label, 'side': s.side, 'view': s.view,
                        'path': w} for s, w in zip(panels, written)],
            'moved': len(moves),
            # moved_refs makes a wrong --before self-evident, and it is what
            # audits the two pixel-invisible move classes (sub-5px arrows are
            # dropped; a rotation-only ghost draws exactly atop the part).
            'moved_refs': [{'reference': m['reference'],
                            'dist': round(m['dist'], 4)} for m in moves],
            'failed_nets': sorted(failed), 'blocker_nets': sorted(blockers),
            'metrics': dict(model.metrics),
            # The instrument block (run-4 G2): two renders of the SAME board
            # differing only in --ignore-nets read 632 vs 412 crossings in
            # run 3, and neither JSON said which was which. A before/after
            # series is provably same-instrument only if the instrument
            # settings ride in the document.
            'instrument': {
                'board': os.path.abspath(args.board),
                'before': os.path.abspath(args.before) if args.before else None,
                'summary_json': (os.path.abspath(args.summary_json)
                                 if args.summary_json else None),
                # `clearance` used to record args.clearance, i.e. None on
                # every run that did not pass the flag -- so the document
                # named no clearance at all for the runs most likely to be
                # graded at the wrong one. It is now the EFFECTIVE value,
                # with what was requested and where it came from beside it.
                'clearance': model.floor_knobs.get(
                    'clearance', {}).get('value'),
                'clearance_requested': args.clearance,
                'floors': model.floor_knobs,
                'ignore_nets': sorted(args.ignore_nets or []),
                'ratsnest_nets': sorted(args.ratsnest_nets or []),
                # DECLARED vs MATCHED. Without these there was no field in
                # this document where "61 requested, 51 matched" could
                # appear, so a net list that silently missed had to be
                # checked by hand, outside the tool -- and was not.
                'net_lists': net_lists,
                'size': args.size, 'supersample': args.supersample,
            },
            # Mandate 8's four questions, quotable (run-4 G5). Channels are
            # labelled; see legality_findings.
            'checklist': {
                'a_off_outline': {
                    'pad_copper': fnd['oob_refs_pad_copper'],
                    'courtyard': fnd['oob_refs_courtyard']},
                # run-6 key honesty: the old 'b_overlap_pairs' NAME carried
                # the PAD-CLEARANCE channel, and a reader auditing overlap
                # with b_overlap_pairs=[] concluded there was none while two
                # parts sat stacked (the shipped C14-on-R14). The clearance
                # channel now lives under its true name; b_body_overlap_pairs
                # reports what the old name promised.
                'b_pad_clearance_pairs': fnd['pad_conflict_pairs_refs'],
                'b_body_overlap_pairs': fnd['body_overlap_pairs_refs'],
                'c_hole_conflicts': fnd['hole_conflict_pairs_refs'],
                'c_locked_refs': fnd['locked_refs'],
                'd_moved': {'moved': len(moves),
                            'expected': args.expect_moved,
                            'match': (None if args.expect_moved is None
                                      else len(moves) == args.expect_moved)},
            },
            'unplaced': state.unplaced, 'no_outline': model.no_outline,
        }
        if not args.no_describe:
            _txt, _dj = describe(model, legality_findings(model), moves, args,
                                 [p['path'] for p in doc['panels']])
            doc['describe'] = _dj
            print()
            print(_txt)
            if before_model is not None:
                _ptxt, _pj = describe_pair(before_model, model, args)
                doc['pair'] = _pj
                print(_ptxt)
        if args.json_out:
            with open(args.json_out, 'w', encoding='utf-8') as f:
                json.dump(doc, f, indent=2, sort_keys=True, default=str)
        print("JSON_SUMMARY: " + json.dumps(doc, sort_keys=True, default=str))
        if args.gate:
            # Opt-in verdict. The default stays 0 on purpose -- SEEING an
            # unplaced or broken board is this tool's job, and a renderer that
            # refuses to render one is useless. But the checklist could report
            # off-board parts, body stacks and hole conflicts while the tool
            # exited 0, so a caller who wanted a verdict had to re-implement the
            # reading. --gate makes the picture's own findings decide.
            _fail = {
                'a_off_outline.pad_copper':
                    len(doc['checklist']['a_off_outline']['pad_copper']),
                'a_off_outline.courtyard':
                    len(doc['checklist']['a_off_outline']['courtyard']),
                'b_pad_clearance_pairs':
                    len(doc['checklist']['b_pad_clearance_pairs']),
                'b_body_overlap_pairs':
                    len(doc['checklist']['b_body_overlap_pairs']),
                'c_hole_conflicts':
                    len(doc['checklist']['c_hole_conflicts']),
            }
            _hit = {k: v for k, v in _fail.items() if v}
            _moved = doc['checklist']['d_moved']
            if _moved.get('match') is False:
                _hit['d_moved'] = (f"moved {_moved.get('moved')} != expected "
                                   f"{_moved.get('expected')}")
            if _hit:
                print("GATE: FAIL -- " + '; '.join(f'{k}={v}'
                                                   for k, v in _hit.items()),
                      file=sys.stderr)
                return 4
            print("GATE: PASS -- checklist a/b/c clear"
                  + ("" if _moved.get('match') is None
                     else f", moved {_moved.get('moved')} as expected"),
                  file=sys.stderr)
    return 0


if __name__ == '__main__':
    import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
    sys.exit(main())
