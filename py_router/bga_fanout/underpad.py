"""
Under-pad grid escape for dense BGAs (issue #122).

The channel-template fanout confines every layer to the gaps BETWEEN ball rows,
so on a fully-populated array a handful of channels over-subscribe (conflict
cliques far exceeding the layer count) and the deepest balls cannot escape - the
ulx3s 22x22 floors at ~11 dropped balls no matter how channels/layers are
balanced.

The fix exploits a fact the channel model ignores: SMD pads block only F.Cu, so
inner-layer copper can run STRAIGHT UNDER the pad field. Each signal ball drops
a via in its own pad and escapes on an inner layer, mostly as a straight radial
run beneath the pads, jogging into a between-ball channel only where a via or an
already-placed track is in the way. Plane balls (nets the caller excluded from
the fanout) are NOT fanned - they tap their plane through a via (an obstacle here,
not an escape); dense power rails the caller still wants fanned ARE escaped like
any other signal (issue #218). Routing the deepest
balls first (inside-out) lets the interior claim the scarce central space before
the outer balls, which have the open rim, consume it.

A prototype of this model escapes all 210 ulx3s signals on its 4 layers with
plain via-in-pad (no dog-bone). This module is the production router: a small
per-ball grid search (straight-first) over a layered occupancy grid built from
the board's real pads, vias and copper.
"""
from __future__ import annotations

import heapq
import math
import os
from collections import Counter
from typing import Dict, List, Optional, Set, Tuple

from kicad_parser import Footprint, PCBData
from bga_fanout.types import BGAGrid
from bga_fanout.geometry import clamp_via_to_pad
from list_nets import fab_floors, fab_floor_ladder, fab_floor_min, warn_fab_escalation
from routing_defaults import HOLE_TO_HOLE_CLEARANCE


def _custom_polys(pad):
    """The pad's REAL polygon copper (board mm) if it is a custom-shape pad,
    else None. For a custom pad pad.size_x/size_y is only its symmetric bbox,
    which for an off-centre outline (a meander antenna, a comb/finger pad) is a
    gross over-estimate -- the #232 phantom class."""
    if getattr(pad, 'shape', None) == 'custom':
        return getattr(pad, 'polygons', None) or None
    return None


def _pad_extent(pad):
    """(centre_x, centre_y, radius) of the pad's copper. For a custom pad this
    is the bounding circle of the REAL polygons (contains all copper, never
    under-blocks) instead of the bbox disk centred on the pad origin, which for
    an off-centre outline phantom-blocks vias far from any copper (mikoto's AE1
    antenna: a Ø20mm bbox disk reaching a QFN 8mm away)."""
    polys = _custom_polys(pad)
    if polys:
        pts = [pt for poly in polys for pt in poly]
        if pts:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            cx = (min(xs) + max(xs)) / 2.0
            cy = (min(ys) + max(ys)) / 2.0
            r = max(math.hypot(px - cx, py - cy) for px, py in pts)
            return cx, cy, r
    return pad.global_x, pad.global_y, max(pad.size_x, pad.size_y) / 2.0


# Cardinal + diagonal steps; straight moves are cheaper so routes stay straight.
_STEPS = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


class _Occ:
    """Layered occupancy grid over the BGA escape region.

    One bytearray per copper layer. A set cell is blocked for new track
    centerlines on that layer. Obstacles are inflated by their own half-size
    plus the track half-width plus clearance, so a clear cell guarantees the
    track edge clears the obstacle edge by `clearance`.
    """

    def __init__(self, bounds, res, layers):
        self.x0, self.y0, self.x1, self.y1 = bounds
        self.res = res
        self.layers = layers
        self.nl = len(layers)
        self.nx = int((self.x1 - self.x0) / res) + 2
        self.ny = int((self.y1 - self.y0) / res) + 2
        self.grid = [bytearray(self.nx * self.ny) for _ in range(self.nl)]
        # Soft keep-outs (movable passives' pads, #278): running here is legal
        # but leaves a PAD-SEGMENT graze the cap-placement step may be unable
        # to move away from (an 0805 doesn't fit between two escape legs), so
        # routes prefer to avoid these cells and only pay when boxed in.
        # soft_owner records the pad's net per cell (-1 once two nets share a
        # cell): a route crossing its OWN cap's pad is no graze and pays
        # nothing.
        self.soft = [bytearray(self.nx * self.ny) for _ in range(self.nl)]
        self.soft_owner = {}
        self.has_soft = False
        # One shared run of set bytes for the block_* slice blits (sliced to
        # length per span); ny is the longest span any column can need.
        self._ones = b'\x01' * self.ny

    def cell(self, x, y):
        return (int((x - self.x0) / self.res), int((y - self.y0) / self.res))

    def xy(self, ix, iy):
        return (self.x0 + ix * self.res, self.y0 + iy * self.res)

    def inside(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def _disk_spans(self, x, y, r):
        """Column spans [(a, b_lo, b_hi)] of the _disk membership -- EXACTLY
        the cells the per-cell predicate admits (span edges are re-tested
        with the original arithmetic so floating-point rounding can never
        flip a boundary cell), but computed with one sqrt per column instead
        of float math per cell. The 28.6M-yield _disk generator was 42% of a
        BGA fanout profile; the block_* writers blit these spans instead."""
        fx = (x - self.x0) / self.res
        fy = (y - self.y0) / self.res
        cx, cy = int(fx), int(fy)
        rc = int(r / self.res) + 2
        thr = (r / self.res) ** 2
        a0 = cx - rc
        if a0 < 0:
            a0 = 0
        a1 = cx + rc
        if a1 > self.nx - 1:
            a1 = self.nx - 1
        blo_lim = cy - rc
        if blo_lim < 0:
            blo_lim = 0
        bhi_lim = cy + rc
        if bhi_lim > self.ny - 1:
            bhi_lim = self.ny - 1
        spans = []
        for a in range(a0, a1 + 1):
            adx2 = (a - fx) ** 2
            rem = thr - adx2
            if rem < 0:
                continue
            half = rem ** 0.5
            b0 = int(fy - half) - 1
            b1 = int(fy + half) + 1
            if b0 < blo_lim:
                b0 = blo_lim
            if b1 > bhi_lim:
                b1 = bhi_lim
            # Edge-correct with the ORIGINAL predicate (identical expression
            # and evaluation order) so membership is bit-identical.
            while b0 <= b1 and (a - fx) ** 2 + (b0 - fy) ** 2 > thr:
                b0 += 1
            while b1 >= b0 and (a - fx) ** 2 + (b1 - fy) ** 2 > thr:
                b1 -= 1
            if b0 > b1:
                continue
            while b0 - 1 >= blo_lim and \
                    (a - fx) ** 2 + (b0 - 1 - fy) ** 2 <= thr:
                b0 -= 1
            while b1 + 1 <= bhi_lim and \
                    (a - fx) ** 2 + (b1 + 1 - fy) ** 2 <= thr:
                b1 += 1
            spans.append((a, b0, b1))
        return spans

    def _disk(self, x, y, r):
        """Cells whose lattice point lies within r of the REAL point (x, y).

        Membership is tested against the true coordinate, not the truncated
        cell index: a via at y=70.999999 truncates a full cell low, shifting
        its keepout disk by up to res and letting a route one cell past the
        shifted disk graze the real via by up to res (issue #278).
        """
        for a, b0, b1 in self._disk_spans(x, y, r):
            for b in range(b0, b1 + 1):
                yield a, b

    def block_layer(self, li, x, y, r):
        g = self.grid[li]
        ny = self.ny
        ones = self._ones
        for a, b0, b1 in self._disk_spans(x, y, r):
            s = a * ny
            g[s + b0:s + b1 + 1] = ones[:b1 - b0 + 1]

    def block_all(self, x, y, r):
        ny = self.ny
        ones = self._ones
        for a, b0, b1 in self._disk_spans(x, y, r):
            s = a * ny
            run = ones[:b1 - b0 + 1]
            for li in range(self.nl):
                self.grid[li][s + b0:s + b1 + 1] = run

    def block_segment(self, li, p, q, r):
        (x0, y0), (x1, y1) = p, q
        n = int(max(abs(x1 - x0), abs(y1 - y0)) / self.res) + 1
        # Union of the sample disks, merged into per-column intervals and
        # blitted once -- the same membership as stamping each sample disk
        # (integer-adjacent intervals merge losslessly), ~10x less work.
        cols = {}
        dx, dy = x1 - x0, y1 - y0
        for i in range(n + 1):
            t = i / n
            for a, b0, b1 in self._disk_spans(x0 + dx * t, y0 + dy * t, r):
                iv = cols.get(a)
                if iv is None:
                    cols[a] = [(b0, b1)]
                else:
                    iv.append((b0, b1))
        g = self.grid[li]
        ny = self.ny
        ones = self._ones
        for a, ivs in cols.items():
            ivs.sort()
            s = a * ny
            cb0, cb1 = ivs[0]
            for b0, b1 in ivs[1:]:
                if b0 <= cb1 + 1:
                    if b1 > cb1:
                        cb1 = b1
                else:
                    g[s + cb0:s + cb1 + 1] = ones[:cb1 - cb0 + 1]
                    cb0, cb1 = b0, b1
            g[s + cb0:s + cb1 + 1] = ones[:cb1 - cb0 + 1]

    def block_poly(self, polys, r, li=None):
        """Keep-out for a custom pad's REAL polygon copper: block every cell
        whose lattice point lies within `r` mm of any polygon (li=None => all
        copper layers). The accurate analogue of block_all/block_layer's bbox
        disk -- mirrors the main obstacle map's polygon rasteriser so a
        meander-antenna / comb pad stops phantom-blocking escapes far from any
        real copper (#232 class, BGA side)."""
        from check_drc import _point_in_poly
        from geometry_utils import point_to_segment_distance
        pts = [pt for poly in polys for pt in poly]
        if not pts:
            return
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ia, ja = self.cell(min(xs) - r, min(ys) - r)
        ib, jb = self.cell(max(xs) + r, max(ys) + r)
        thr = r + 1e-9
        # #561: threshold DECISION, not exact distance -- early-exit on the
        # first edge within thr, with a per-edge bbox reject (identical
        # accept/reject outcome to _point_to_polys_distance(...) <= thr;
        # profiled 55 of 65 s at 72M exact distance calls).
        edges = []
        for poly in polys:
            n = len(poly)
            e = []
            for i in range(n):
                x1, y1 = poly[i]
                x2, y2 = poly[(i + 1) % n]
                e.append((min(x1, x2) - thr, max(x1, x2) + thr,
                          min(y1, y2) - thr, max(y1, y2) + thr,
                          x1, y1, x2, y2))
            edges.append((poly, e))

        def _within(x, y):
            for poly, e in edges:
                if _point_in_poly(x, y, poly):
                    return True
                for (bx0, bx1e, by0, by1e, x1, y1, x2, y2) in e:
                    if x < bx0 or x > bx1e or y < by0 or y > by1e:
                        continue
                    if point_to_segment_distance(x, y, x1, y1, x2, y2) <= thr:
                        return True
            return False
        for a in range(max(0, ia), min(self.nx, ib + 1)):
            for b in range(max(0, ja), min(self.ny, jb + 1)):
                x, y = self.xy(a, b)
                if _within(x, y):
                    if li is None:
                        for k in range(self.nl):
                            self.grid[k][a * self.ny + b] = 1
                    else:
                        self.grid[li][a * self.ny + b] = 1

    def blocked(self, li, ix, iy):
        return self.grid[li][ix * self.ny + iy]

    def mark_soft_layer(self, li, x, y, r, net_id):
        g = self.soft[li]
        ny = self.ny
        for a, b in self._disk(x, y, r):
            idx = a * ny + b
            g[idx] = 1
            key = (li, idx)
            owner = self.soft_owner.get(key)
            if owner is None:
                self.soft_owner[key] = net_id
            elif owner != net_id:
                self.soft_owner[key] = -1
        self.has_soft = True

    def soft_foreign(self, li, ix, iy, net_id):
        """True if (ix, iy) on layer li lies in a soft keep-out owned by a
        DIFFERENT net (crossing one's own cap pad is not a graze)."""
        idx = ix * self.ny + iy
        if not self.soft[li][idx]:
            return False
        return self.soft_owner.get((li, idx)) != net_id

    def seg_soft_hits(self, li, p, q, net_id, exempt=None):
        """Number of sampled cells on the centerline p->q that lie in a
        foreign-net soft keep-out on layer li (0 = the run leaves every
        movable passive's pad fully clear). Same sampling as seg_clear."""
        (x0, y0), (x1, y1) = p, q
        n = int(max(abs(x1 - x0), abs(y1 - y0)) / (self.res * 0.5)) + 1
        hits = 0
        for i in range(n + 1):
            t = i / n
            ix, iy = self.cell(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            if not self.inside(ix, iy):
                continue
            if exempt is not None and (ix, iy) in exempt:
                continue
            if self.soft_foreign(li, ix, iy, net_id):
                hits += 1
        return hits

    def seg_clear(self, li, p, q, exempt=None):
        """True if the track centerline p->q is clear on layer li.

        Tests the centerline cells (the grid is already inflated by obstacle
        half-size + track/2 + clearance, so a clear centerline cell guarantees
        the track edge clears the obstacle edge - same contract astar relies on).
        Cells in `exempt` (a set of (ix, iy), e.g. the ball's own home keepout)
        are ignored. Samples finely so no blocked cell between endpoints is
        missed.
        """
        (x0, y0), (x1, y1) = p, q
        n = int(max(abs(x1 - x0), abs(y1 - y0)) / (self.res * 0.5)) + 1
        for i in range(n + 1):
            t = i / n
            ix, iy = self.cell(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            if not self.inside(ix, iy):
                continue
            if exempt is not None and (ix, iy) in exempt:
                continue
            if self.blocked(li, ix, iy):
                return False
        return True


def _segments_from_cells(occ, cells):
    """Compress a same-layer cell path into (start_xy, end_xy) polyline points."""
    pts = [occ.xy(ix, iy) for (ix, iy) in cells]
    if len(pts) < 2:
        return []
    out = [pts[0]]
    pdir = None
    for a, b in zip(pts, pts[1:]):
        d = (round(b[0] - a[0], 6), round(b[1] - a[1], 6))
        nd = (0 if d[0] == 0 else (1 if d[0] > 0 else -1),
              0 if d[1] == 0 else (1 if d[1] > 0 else -1))
        if pdir is not None and nd != pdir:
            out.append(a)
        pdir = nd
    out.append(pts[-1])
    return out


def generate_underpad_escape(footprint: Footprint,
                             pcb_data: PCBData,
                             grid: BGAGrid,
                             layers: List[str],
                             track_width: float,
                             clearance: float,
                             via_size: float,
                             via_drill: float,
                             exit_margin: float,
                             plane_min_pads: int = 6,
                             net_filter_fn=None,
                             diff_pairs=None,
                             diff_pair_gap: float = 0.1,
                             res: float = None,
                             via_cost: float = 14.0,
                             outer_rings: float = math.inf,
                             keepout_margin: float = 0.0,
                             verbose: bool = True,
                             grid_step: float = 0.0,
                             only_pad_keys: Optional[Set[Tuple[float, float]]] = None,
                             dogbone: bool = False,
                             plane_drop_nets: Optional[Set[int]] = None,
                             plane_drop_report: Optional[Dict] = None,
                             plane_net_layers: Optional[Dict[str, List[str]]] = None,
                             # #581: forbid ANY via overlapping its own pad --
                             # dog-bone/off-pad sites only; balls with no legal
                             # off-pad site fail honestly.
                             no_via_in_pad: bool = False,
                             # Per-pad escape direction from a caller's plan,
                             # keyed (round(x,3), round(y,3)). The dog-bone
                             # site is normally chosen toward the ball's
                             # NEAREST BOUNDARY, which is a per-ball guess made
                             # without seeing the corridor the net has to join;
                             # a hint replaces that guess for the balls it
                             # names. None/{} leaves every site exactly where
                             # it is today.
                             escape_dir_hints: Optional[Dict[
                                 Tuple[float, float], str]] = None,
                             # Per-pad exit LINE, in mm along the exit
                             # edge (y for a left/right escape, x for
                             # up/down). A side alone does not pin the
                             # permutation -- which row gap a ball takes
                             # is what fixes its place in the
                             # launch->exit order, and the order IS the
                             # via floor. Measured on the bench: hints
                             # carrying only the side left 14 of 15
                             # stubs off the line the plan chose and
                             # raised the inversion count from 38 to 53,
                             # handing the next stage a permutation
                             # nobody had chosen.
                             escape_line_hints: Optional[Dict[
                                 Tuple[float, float], float]] = None,
                             # Per-pad escape LAYER from a caller's plan
                             # (#622), same key. Tried FIRST at every site
                             # that picks a layer (phase gating, the dogbone
                             # inner-run ladder, Phase D's layer set); never
                             # costs a ball its escape -- the unrestricted
                             # search stays as fallback and a miss is
                             # recorded, not swallowed.
                             escape_layer_hints: Optional[Dict[
                                 Tuple[float, float], str]] = None,
                             # Per-pad exit SLOT (#622): (side, lo, hi)
                             # -- an INTERVAL along the side's axis (y
                             # for left/right, x for up/down) the stub
                             # must exit within. The interval is the
                             # abstraction level measured to survive:
                             # a bare side under-specifies the order,
                             # an exact line over-specifies (LINES=1
                             # broken; K15 one-gap). Rides the existing
                             # astar line acceptance as (centre,
                             # half-width). Slot-first over
                             # escape_line_hints; misses recorded.
                             # DEBT (#622): this side-keyed shape is a
                             # QUANTIZATION of the real contract (a
                             # cyclic arc on the boundary loop) -- a
                             # corner-spanning slot is inexpressible
                             # and gets pinned to one side. Fix shape:
                             # accept one-or-two side-windows OR'd in
                             # the astar acceptance (an L for a corner
                             # arc).
                             escape_slot_hints: Optional[Dict[
                                 Tuple[float, float],
                                 Tuple[str, float, float]]] = None,
                             # progress_callback(current, total, label); the
                             # per-ball A* is where a large array's minutes go,
                             # so each escape loop reports its count through
                             # here (GUI status line). None = silent (CLI).
                             progress_callback=None,
                             cancel_check=None
                             ) -> Tuple[List[Dict], List[Dict], List[str]]:
    """Route BGA signal balls to the boundary under the pad field.

    Returns (tracks, vias_to_add, failed_net_names). Plane balls - high-pin-count
    nets the caller excluded from the fanout (or, if no net filter is given, any
    net with >= plane_min_pads pads) - are skipped: they tap their plane and act
    only as via obstacles. Dense power rails that pass the net filter are fanned
    like any signal, so their inner-core balls are not left unescaped (issue #218).

    Differential pairs (issue #182): when `diff_pairs` is given (a dict
    base_name -> DiffPairPads, e.g. from find_differential_pairs), each pair
    whose two balls are routable and ~one pitch apart is escaped *coupled* -
    both balls drop a via on one inner layer, converge to the diff spacing
    (track_width + diff_pair_gap) and run parallel out past the boundary. That
    leaves a clean same-layer, parallel pair of free ends route_diff can pick
    up; without it the two halves escape single-ended and route_diff cannot
    re-pair them. A pair that won't fit a coupled corridor falls back to the
    single-ended escape (still connected, just not coupled).

    Dog-bone mode (issue #128): with `dogbone=True`, inner balls do not drop
    their via IN the pad -- each ball gets a via in the diagonal inter-ball
    gap (a "dog-bone": ball -> short stub -> via), staggered toward the ball's
    escape side, and the inner-layer run starts at that via. This keeps the
    ball-grid positions free of barrels on the inner layers, opening the
    routing streets a via-in-pad grid closes (the human ulx3s escape is 100%
    dog-bone), and eliminates the via-vs-neighbour-ball graze class (#267).
    A ball whose four gap sites are all taken falls back to the classic
    via-in-pad escape, so dog-bone can never escape fewer balls than underpad.
    Coupled diff-pair escapes use each ball's dog-bone site the same way.

    Plane-ball drops (#424 D2): with `plane_drop_nets` (a set of net ids), every
    SMD ball on those nets that the fanout would otherwise skip gets a via NOW
    -- a dog-bone stub + gap via where a gap site is free, else an exact-checked
    via-in-pad tap -- and is NOT routed anywhere: the plane poured later picks
    the via up at fill. This claims the tap while the under-package space is
    still open, instead of leaving the plane step to push a via through the
    finished ball field (the #360 tap-behind-the-ball-wall class). Per-net
    counts are written into `plane_drop_report` when a dict is passed. Drop
    failures are reported there, never in the returned failed list (plane nets
    are not part of the requested/escaped ledger; the plane step's tap search
    remains the fallback).

    Cancellation (#621): `cancel_check` is a zero-arg predicate honoured at the
    head of the four escape loops (top-layer coupled pairs, the outside-in
    surface phase, the inner coupled pairs, the inner single balls) -- the same
    contract `batch_route` / `create_plane` already take, so the GUI's Cancel
    button reaches this and nothing new is needed. It BREAKS the loop; it never
    raises (an exception dies in this package's `except Exception` swallowers,
    which is the silent failure the cooperative contract exists to remove).
    Balls already escaped keep their copper; balls the cancel never got
    to are simply absent from BOTH the emitted copper and the returned failed
    list -- an unfinished search has measured nothing, so reporting it as a
    routing failure is exactly the misreading to avoid. The
    caller names that complement (`bga_fanout.LAST_CANCEL_SKIPPED`).
    """
    ref = footprint.reference
    fp_pads = [p for p in footprint.pads if p.net_id]

    # Net classification within this footprint.
    fp_net_counts = Counter(p.net_name for p in fp_pads)
    board_net_counts = Counter()
    for fp in pcb_data.footprints.values():
        for p in fp.pads:
            if p.net_id:
                board_net_counts[p.net_id] += 1

    # #565 (landed at #562 graduation): net names that own a copper pour.
    # Zone ownership is the honest plane signal in a pours-first chain --
    # every plane net owns a zone by fanout time.
    _zone_net_names = {pcb_data.nets[z.net_id].name
                       for z in (getattr(pcb_data, 'zones', None) or [])
                       if z.net_id in pcb_data.nets}

    def is_plane(p):
        # A high-pin-count net is only treated as a plane (skip its fanout, keep
        # its via as an all-layer obstacle) when the caller ALSO excluded it from
        # the fanout - that exclusion (e.g. --nets '!GND' '!+3V3') is what marks a
        # net as a real plane at fanout time, before any zone exists. A dense power
        # rail the caller still wants fanned (e.g. +1V0/+1V8) must have its
        # inner-core balls escaped, not silently skipped as a phantom plane
        # (issue #218). With no net filter, fall back to the pad-count heuristic.
        #
        # #565 additive OR: an EXCLUDED net that owns a copper pour gets
        # plane semantics regardless of ball count -- the plane-drop side
        # has had this OR since #424; this escape-skip side didn't, so a
        # small (<plane_min_pads) excluded rail WITH a pour was escaped as
        # signals. Flips nothing existing: filter-included nets and
        # no-filter runs keep the pad-count rule (see #565 for why the
        # pad-count leg must NOT be demoted -- mixed-order chains and
        # unfiltered runs depend on it).
        if (net_filter_fn is not None and not net_filter_fn(p.net_name)
                and p.net_name in _zone_net_names):
            return True
        if fp_net_counts[p.net_name] < plane_min_pads:
            return False
        if net_filter_fn is None:
            return True
        return not net_filter_fn(p.net_name)

    def is_nc(p):
        return board_net_counts[p.net_id] < 2

    # Nets already fanned out (a board segment touches one of THIS BGA's pads) are
    # left alone: not re-routed, and their copper is kept as an obstacle (see the
    # segment loop below). This lets the under-pad pass run AFTER diff pairs (or
    # anything else) were fanned first - it fills in only the remaining balls and
    # routes clear of the existing fanout.
    TOL = 0.05
    bga_pad_pts = {}
    for p in fp_pads:
        bga_pad_pts.setdefault(p.net_id, []).append((p.global_x, p.global_y))
    fanned_net_ids = set()
    for s in pcb_data.segments:
        for (px, py) in bga_pad_pts.get(s.net_id, ()):
            if ((abs(s.start_x - px) < TOL and abs(s.start_y - py) < TOL)
                    or (abs(s.end_x - px) < TOL and abs(s.end_y - py) < TOL)):
                fanned_net_ids.add(s.net_id)
                break

    plane_pads = [p for p in fp_pads if is_plane(p)]
    # only_pad_keys (issue #129 escape priority): when given, membership is
    # authoritative -- fan exactly these balls, even if their net already has
    # fanout copper (the priority pass fanned the net's primary ball; this
    # pass adds escapes for the remaining balls).
    signal_pads = [p for p in fp_pads
                   if not is_plane(p) and not is_nc(p)
                   and (net_filter_fn is None or net_filter_fn(p.net_name))
                   and ((p.global_x, p.global_y) in only_pad_keys
                        if only_pad_keys is not None
                        else p.net_id not in fanned_net_ids)]

    _ref = getattr(footprint, 'reference', '?')

    def _prog(cur, tot, what):
        if progress_callback:
            progress_callback(cur, tot, f"Under-pad escape {_ref}: {what}")

    # The occupancy grid + keep-out carving below is itself seconds on a
    # dense array; say so before going quiet.
    _prog(0, 0, f"gridding {len(signal_pads)} ball(s)...")

    # Region: BGA bbox plus margin so routes can exit past the boundary.
    # Grid resolution defaults to ~1/32 of the pitch - fine enough to place the
    # single-track channels and F.Cu corridors cleanly (coarser leaves a few
    # sub-clearance corridor clips), scaled to the array so big/fine boards stay
    # fast.
    if grid is None:
        # Run-6 defensive entry guard: a None grid (perimeter package) used
        # to crash on grid.pitch_x below; fail the escape cleanly instead.
        print("  ERROR: underpad escape aborted - no BGA grid (perimeter "
              "package? use qfn_fanout.py)")
        return [], [], [p.net_name for p in signal_pads]
    if res is None:
        res = min(grid.pitch_x, grid.pitch_y) / 32.0
    if res <= 0:
        # Backstop for a corrupt grid analysis (issue #283): a zero pitch would
        # divide-by-zero building the occupancy grid. Fail the escape cleanly.
        print(f"  ERROR: underpad escape aborted - grid resolution {res} is not "
              f"positive (pitch {grid.pitch_x:.4f} x {grid.pitch_y:.4f}mm "
              f"mis-detected?)")
        return [], [], [p.net_name for p in signal_pads]
    pad = max(exit_margin + 0.5, 1.0)
    bounds = (grid.min_x - pad, grid.min_y - pad, grid.max_x + pad, grid.max_y + pad)
    occ = _Occ(bounds, res, layers)

    # Pour-preserving escape costs (#563, measured-and-declined, kept for A/B;
    # KICAD_FANOUT_POUR_PRESERVE=<mm-equiv> arms, 0/unset = off). For every zone
    # whose layer is an escape layer, rasterize the PRE-escape fill inside
    # this occupancy window and charge escapes a fragility-style ramp: full
    # cost at fill boundaries/straits, zero deeper than ~1mm into the fill.
    # Under the BGA the flood is all straits, so escapes soft-prefer other
    # layers there and the flood survives for pour-direct; mid-flood
    # crossings outside stay ~free.
    occ.pour_soft = None
    try:
        _pp = float(os.environ.get('KICAD_FANOUT_POUR_PRESERVE', '0') or 0)
    except ValueError:
        _pp = 0.0
    if _pp > 0 and getattr(pcb_data, 'zones', None):
        try:
            from plane_fill_model import get_zone_model
            from plane_fragility import _erode_depth
            import numpy as _np
            _ramp_mm = 1.0
            _depth = max(1, int(round(_ramp_mm / occ.res)))
            _fields = {}
            for _z in pcb_data.zones:
                if _z.layer not in layers:
                    continue
                _li = layers.index(_z.layer)
                _m = get_zone_model(pcb_data, _z)
                if _m is None:
                    continue
                # sample the model over THIS occupancy window
                _mask = _np.zeros((occ.nx, occ.ny), dtype=bool)
                for _a in range(occ.nx):
                    _x = bounds[0] + (_a + 0.5) * occ.res
                    for _b in range(occ.ny):
                        _y = bounds[1] + (_b + 0.5) * occ.res
                        _mask[_a, _b] = (_m.query_component(_x, _y) or 0) > 0
                if not _mask.any():
                    continue
                _dist = _erode_depth(_mask, _depth)
                _fld = _fields.setdefault(_li, _np.zeros((occ.nx, occ.ny)))
                # Straits/boundaries cost full; interior fill is floored at
                # 0.4, not ~0 -- an inner plane is SOLID under the BGA (SMD
                # balls don't carve it), and with a free interior the escapes
                # dice it to sub-min slivers (measured In1 82%->42% from
                # tracks alone). Every fill cell consumed is copper lost.
                _frac = _np.where(_mask,
                                  _np.maximum(
                                      1.0 - (_np.maximum(_dist, 1) - 1) / _depth,
                                      0.4),
                                  0.0)
                _np.maximum(_fld, _pp * _frac, out=_fld)
            if _fields:
                occ.pour_soft = {li: f.ravel().tolist() for li, f in _fields.items()}
                print(f"  Pour-preserve (#563): fragility field on "
                      f"{len(_fields)} escape layer(s) at {_pp}mm-equiv")
        except Exception as _e:
            print(f"  Pour-preserve unavailable ({_e})")

    # The BGA's own layer is the "top" for escape purposes - its SMD pads block
    # only this layer, the via-less edge escape runs on it, and inner routing
    # goes UNDER the pads on the other layers. For a bottom-side BGA this is
    # B.Cu, not F.Cu, so resolve it from the footprint rather than assuming 0.
    top_idx = layers.index(footprint.layer) if footprint.layer in layers else 0

    # Via and track width are taken as given (the caller sets them small enough
    # for the pitch). A useful sanity note: the escape needs one clean track to
    # thread between two via-in-pads a pitch apart, i.e. roughly
    # via <= pitch - 2*track - 2*clearance; beyond that balls will start failing.
    pitch = min(grid.pitch_x, grid.pitch_y)
    if verbose and via_size > pitch - 2 * track_width - 2 * clearance + 1e-9:
        print(f"  Under-pad: WARNING via {via_size:.2f}mm is large for {pitch:.2f}mm "
              f"pitch (track {track_width:.2f}, clearance {clearance:.2f}); "
              f"expect drops. Try via <= {pitch - 2*track_width - 2*clearance:.2f}mm.")

    # An optional safety margin on top of the exact keepout. The no-corner-cutting
    # rule (in astar) already removes the diagonal clipping that used to need it,
    # so it defaults to 0; raise it only to trade routability for extra slack.
    margin = keepout_margin or 0.0
    pad_r = max((max(p.size_x, p.size_y) for p in fp_pads), default=0.4) / 2
    via_r = via_size / 2
    pad_keep = pad_r + track_width / 2 + clearance + margin
    via_keep = via_r + track_width / 2 + clearance + margin
    trk_keep = track_width + clearance + margin  # other centerlines stay this far

    # Issue #202: size each via-in-pad to fit its OWN pad up front, so the via
    # never bulges past the pad edge AND the keep-out reserved below uses the
    # real (smaller) via -- letting neighbouring escapes route past it. Done
    # per-pad, so on a mixed-pad-size array only the small pads get smaller vias.
    _copper = len(getattr(pcb_data.board_info, 'copper_layers', None) or []) or 4
    floors = fab_floor_ladder(_copper)
    clamp_stats = {'clamped': 0, 'floor': 0, 'escalated': 0}

    # DRILL FLOOR, board-first and raise-only. `_via_site_conflict` below spaced
    # every drill it places at the flat routing_defaults.HOLE_TO_HOLE_CLEARANCE
    # and this module never read the board at all -- grepping it for
    # board_floor / board_constraint / min_hole_to_hole / resolve_hole_clearance
    # returned nothing. So a board declaring min_hole_to_hole 0.3 got its BGA
    # underpad drills spaced at 0.2: the D9/D11 substitute-a-constant class,
    # in the direction that IGNORES a board asking for MORE.
    #
    # RAISE-ONLY IN THE CODE, not only in this comment (the Q4 lesson):
    # board_floor is board-authoritative and will happily return a declared
    # 0.10, so the fab floor is wrapped in explicitly. check_join.py:118-122 is
    # the same shape and was the correct model. qfn_fanout does this too; this
    # is its sibling engine and was the one still ignoring the board.
    from list_nets import board_floor as _board_floor
    _h2h_decl, _h2h_src = _board_floor(
        getattr(pcb_data, 'source_path', "") or "", 'hole_to_hole',
        None, HOLE_TO_HOLE_CLEARANCE)
    _h2h_fab = fab_floor_min(_copper).get('hole_to_hole', 0.0)
    _h2h = max(_h2h_decl, _h2h_fab)
    if _h2h_src == 'board constraint' and _h2h_decl > HOLE_TO_HOLE_CLEARANCE:
        print(f"  Hole-to-hole {_h2h:g}mm (from the board's own "
              f"min_hole_to_hole)")
    elif _h2h_src == 'board constraint' and _h2h_decl < _h2h_fab:
        print(f"  Board min_hole_to_hole {_h2h_decl:g}mm is below the "
              f"{_h2h_fab:g}mm fab hole-to-hole floor; using {_h2h:g}mm.")

    def via_for_pad(p):
        """Clamped (size, drill, keepout_radius) for a via dropped in pad p."""
        cs, cd, status, rung = clamp_via_to_pad(via_size, via_drill, p, floors)
        if status == 'clamped':
            clamp_stats['clamped'] += 1
        elif status == 'floor':
            clamp_stats['floor'] += 1
        if rung > 0:
            clamp_stats['escalated'] += 1
        return cs, cd, cs / 2 + track_width / 2 + clearance + margin

    def vkeep_for_pad(p):
        """Keep-out radius for the (clamped, #202) via that will land in pad p.
        Pure -- no clamp_stats side effect -- so the bulk reservation loops can
        reserve each pad's REAL (clamped) via keepout instead of the nominal
        via_keep, which over-reserves on small/mixed-pitch pads and pre-blocks
        corridors the smaller via leaves open (issue #378). Never larger than
        via_keep (the clamp only shrinks), so it can only free space, never
        under-reserve: any via that lands in a pad too small for the nominal via
        is itself bounded by that pad's clamped size."""
        cs = clamp_via_to_pad(via_size, via_drill, p, floors)[0]
        return cs / 2 + track_width / 2 + clearance + margin

    # Exact foreign-copper registry (issue #393) -------------------------------
    # The occupancy grid cannot say WHOSE stamp blocked a cell, and two of its
    # relaxations leak real conflicts between siblings of the SAME run:
    #   * astar may drop a ball's via OFF-CENTRE anywhere in its own home disk,
    #     gated only by the via's single centre cell on the destination layer --
    #     the ring/drill extent vs foreign copper is never tested;
    #   * the home exemption ignores blocked cells across the whole home disk,
    #     and adjacent balls' home disks overlap (home radius > half pitch), so
    #     copper committed in that lens by an earlier ball is invisible to a
    #     later ball's route (orangecrab U3: EXT_PLL+/- off-centre vias vs the
    #     IO_MOSI/IO_1 escapes -- a shipped short).
    # Both are answered with exact, net-aware geometry: off-centre via sites
    # are validated by _via_site_conflict, and home cells carrying a foreign
    # stamp lose their exemption via _carve_foreign. The lists below mirror
    # exactly what gets stamped into occ (plus per-item radii and net ids).
    home_r = max(pad_keep, via_keep)
    _EMPTY = frozenset()
    layer_idx = {name: i for i, name in enumerate(layers)}
    exact_vias: List[Tuple] = []      # (x, y, ring_r, drill_r, net_id) - real vias
    exact_segs: List[Tuple] = []      # (x1, y1, x2, y2, half_w, net_id, layer) - real tracks
    reserved_sites: List[Tuple] = []  # (x, y, ring_r, drill_r, net_id) - pending via-in-pads
    carve_disks: List[Tuple] = []     # (x, y, keep_r, layer_idx_or_None, net_id) - stamps
    exact_pads = [(p2.global_x, p2.global_y, max(p2.size_x, p2.size_y) / 2.0,
                   (p2.drill or 0.0) / 2.0, p2.net_id,
                   p2.pad_type != 'np_thru_hole')
                  for p2 in footprint.pads]

    # Static obstacles ---------------------------------------------------------
    # EVERY footprint pad blocks copper - including unconnected (net 0) balls,
    # which still occupy F.Cu. SMD blocks only F.Cu; through-hole blocks all
    # layers. (Use footprint.pads, not the netted subset, or NC balls get clipped.)
    for p in footprint.pads:
        if p.drill and p.drill > 0:
            occ.block_all(p.global_x, p.global_y, pad_keep)
            carve_disks.append((p.global_x, p.global_y, pad_keep, None, p.net_id))
        else:
            occ.block_layer(top_idx, p.global_x, p.global_y, pad_keep)
            carve_disks.append((p.global_x, p.global_y, pad_keep, top_idx, p.net_id))
    # Plane balls tap their plane through a via -> block all layers. Reserve each
    # ball's own clamped-via keepout (#378), not the nominal via_keep.
    for p in plane_pads:
        occ.block_all(p.global_x, p.global_y, vkeep_for_pad(p))
        cs, cd = clamp_via_to_pad(via_size, via_drill, p, floors)[:2]
        reserved_sites.append((p.global_x, p.global_y, cs / 2.0, cd / 2.0, p.net_id))
        carve_disks.append((p.global_x, p.global_y, vkeep_for_pad(p), None, p.net_id))
    # Existing vias / copper on the board within the region.
    for v in pcb_data.vias:
        if bounds[0] <= v.x <= bounds[2] and bounds[1] <= v.y <= bounds[3]:
            vkeep_v = v.size / 2 + track_width / 2 + clearance
            occ.block_all(v.x, v.y, vkeep_v)
            exact_vias.append((v.x, v.y, v.size / 2.0,
                               (getattr(v, 'drill', 0) or 0.0) / 2.0, v.net_id))
            carve_disks.append((v.x, v.y, vkeep_v, None, v.net_id))
    layer_set = set(layers)
    route_net_ids = {p.net_id for p in signal_pads}
    for s in pcb_data.segments:
        if s.layer not in layer_set:
            continue
        if not (bounds[0] <= s.start_x <= bounds[2] and bounds[1] <= s.start_y <= bounds[3]):
            continue
        if s.net_id in route_net_ids and only_pad_keys is None:
            # our own future nets - don't preblock (fanned nets are NOT in this
            # set, so their existing copper IS kept as obstacle). In an escape-
            # priority extras pass (#129, only_pad_keys set) the set nets'
            # existing copper is their PASS-1 primary escape -- real committed
            # copper every extra (own net included) must route clear of.
            continue
        li = layers.index(s.layer)
        occ.block_segment(li, (s.start_x, s.start_y), (s.end_x, s.end_y),
                          s.width / 2 + track_width / 2 + clearance)
        exact_segs.append((s.start_x, s.start_y, s.end_x, s.end_y,
                           (s.width or 0.0) / 2.0, s.net_id, li))

    # IMMOVABLE foreign components' pads in the region (locked parts,
    # connectors, test points -- anything place_fanout_clearance will not
    # relocate; #253/#254). Movable caps are ignored on purpose: the next
    # pipeline step moves them away from the fanout vias. On lpddr4_testbed a
    # locked back-side cap (C7) sat under a ball and the via-in-pad's B.Cu
    # ring landed 0.1mm INTO its pad; cap-opt correctly refused to move it.
    from bga_fanout.geometry import immovable_foreign_pads, via_clears_pad_rects
    # Pads slightly OUTSIDE the escape region still matter: a via placed just
    # inside the boundary can reach copper up to pad-half + via + clearance
    # beyond it (orangecrab TP20 sat past the region edge and was missed).
    _pad_margin = 1.0
    locked_smd_pads = []   # copper SMD pads a THROUGH via must also clear
    for p in immovable_foreign_pads(pcb_data, footprint.reference):
        # Custom-shape pads (meander antenna, comb) stamp their REAL polygon
        # copper, not the bbox disk that phantom-blocks vias far from any copper
        # (#232 class); cx/cy/p_half is the polygon bounding circle for the
        # coarse carve stamps (a plain disk is fine there -- it only un-exempts a
        # neighbouring ball's home cells, and never under-covers).
        cpolys = _custom_polys(p)
        cx, cy, p_half = _pad_extent(p)
        # Window by the pad's COPPER EXTENT, not its centre (#451 root cause):
        # zynq7020_som's U11 GND thermal paddle (3.4x4.3) centres 1.3mm outside
        # the escape region, so the centre-only test dropped it -- and the K5
        # escape stub was drawn 0.15mm INTO the paddle copper the region never
        # modeled. A big pad reaches p_half beyond its centre.
        _reach = _pad_margin + p_half
        if not (bounds[0] - _reach <= p.global_x <= bounds[2] + _reach and
                bounds[1] - _reach <= p.global_y <= bounds[3] + _reach):
            continue
        p_keep = p_half + track_width / 2 + clearance + margin
        if p.drill and p.drill > 0:
            occ.block_all(p.global_x, p.global_y, p_keep)
            carve_disks.append((p.global_x, p.global_y, p_keep, None, p.net_id))
            # TH/NPTH copper + drill for the exact via-site check (#393); the
            # locked-SMD copper stays with via_site_ok's exact rect test.
            exact_pads.append((p.global_x, p.global_y, p_half, p.drill / 2.0,
                               p.net_id, p.pad_type != 'np_thru_hole'))
            continue
        # Non-custom pads stamp their REAL RECT outline (rect_rotation-aware),
        # not a disk of radius max(size)/2: that disk under-covers a rect's
        # CORNERS by up to hypot(hx,hy)-max(hx,hy) -- 0.59mm on zynq7020_som's
        # 3.4x4.3 U11 GND thermal paddle, whose corner region the K5 escape
        # stub was then drawn straight into (#451 root cause). The rect poly
        # is exact for rect/roundrect and a conservative superset for
        # circle/oval. Stamp radius shrinks to the track/clearance term since
        # the copper extent is now in the polygon itself.
        _rect_keep = track_width / 2 + clearance + margin
        _rect_poly = None
        if not cpolys:
            hx, hy = p.size_x / 2.0, p.size_y / 2.0
            _corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
            rr = getattr(p, 'rect_rotation', 0.0) or 0.0
            if rr:
                _rad = math.radians(rr)
                _c, _s = math.cos(_rad), math.sin(_rad)
                _corners = [(lx * _c - ly * _s, lx * _s + ly * _c)
                            for lx, ly in _corners]
            _rect_poly = [[(p.global_x + lx, p.global_y + ly)
                           for lx, ly in _corners]]
        pad_layer_names = p.layers or []
        if '*.Cu' in pad_layer_names:
            occ.block_poly(cpolys or _rect_poly, p_keep if cpolys else _rect_keep, None)
            carve_disks.append((cx, cy, p_keep, None, p.net_id))
        else:
            for lname in pad_layer_names:
                if lname in layer_set:
                    li = layers.index(lname)
                    occ.block_poly(cpolys or _rect_poly,
                                   p_keep if cpolys else _rect_keep, li)
                    carve_disks.append((cx, cy, p_keep, li, p.net_id))
        locked_smd_pads.append(p)

    def via_site_ok(x, y, v_half):
        """A through via at (x, y) spans EVERY copper layer, so its ring must
        clear each immovable SMD pad by `clearance` even when the pad's layer
        is not one of the two runs the layer-blocking test sees."""
        return via_clears_pad_rects(x, y, v_half, clearance, locked_smd_pads)

    # MOVABLE passives' pads (unlocked 2-pad C/R/FB) become SOFT keep-outs
    # (#278). place_fanout_clearance moves these parts off fanout copper
    # afterwards, so they must not hard-block escapes -- but a wide cap between
    # two escape legs cannot be moved anywhere (ulx3s C19: 1.4mm pad vs a
    # 1.6mm comb gap), so routes should graze them only when boxed in.
    from bga_fanout.geometry import MOVABLE_PASSIVE_PREFIXES
    for fp in pcb_data.footprints.values():
        if fp.reference == footprint.reference:
            continue
        copper_pads = [p for p in fp.pads
                       if any(str(l).endswith('.Cu') for l in p.layers)]
        if not copper_pads:
            continue
        if (getattr(fp, 'locked', False)
                or not fp.reference.startswith(MOVABLE_PASSIVE_PREFIXES)
                or len(copper_pads) > 2):
            continue  # immovable: already a HARD obstacle above
        for p in copper_pads:
            if not (bounds[0] - _pad_margin <= p.global_x <= bounds[2] + _pad_margin
                    and bounds[1] - _pad_margin <= p.global_y <= bounds[3] + _pad_margin):
                continue
            p_keep = max(p.size_x, p.size_y) / 2.0 + track_width / 2 + clearance
            if p.drill and p.drill > 0:
                for li in range(len(layers)):
                    occ.mark_soft_layer(li, p.global_x, p.global_y, p_keep,
                                        p.net_id)
            else:
                for lname in (p.layers or []):
                    if lname in layer_set:
                        occ.mark_soft_layer(layers.index(lname),
                                            p.global_x, p.global_y, p_keep,
                                            p.net_id)

    # Issue #393 helpers -------------------------------------------------------
    def _carve_foreign(anchors, home, net_ids):
        """Home cells whose occupancy stamp came from FOREIGN copper.

        The home exemption exists so a route can break out of its OWN pad/via
        keep-out ring; it must not also hide a neighbour's committed via/track
        or pad stamp that reaches into the home disk (adjacent balls' home
        disks overlap). Returns (all_layers_cells, {layer_idx: cells}); cells
        listed stay blocked despite being in `home`. Same-net stamps are NOT
        carved (crossing own-net copper is not a DRC conflict), so an escape-
        priority extras pass can still exit over its net's pass-1 copper.
        """
        c_all: set = set()
        c_lay: dict = {}
        if not home:
            return c_all, c_lay

        def near_pt(x, y, reach):
            return any((x - ax) ** 2 + (y - ay) ** 2 < reach * reach
                       for ax, ay in anchors)

        def add(cells, li):
            cells &= home
            if not cells:
                return
            if li is None:
                c_all.update(cells)
            else:
                c_lay.setdefault(li, set()).update(cells)

        for (x, y, keep, li, nid) in carve_disks:
            if nid in net_ids or not near_pt(x, y, home_r + keep):
                continue
            add(set(occ._disk(x, y, keep)), li)
        for v in vias_to_add:
            keep = v['size'] / 2.0 + track_width / 2 + clearance + margin
            if v['net_id'] in net_ids or not near_pt(v['x'], v['y'], home_r + keep):
                continue
            add(set(occ._disk(v['x'], v['y'], keep)), None)

        def add_seg(x1, y1, x2, y2, keep, li):
            dx, dy = x2 - x1, y2 - y1
            slen = math.hypot(dx, dy)
            n = int(slen / res) + 1
            reach = (home_r + keep + res) / max(slen, 1e-9)
            idxs = set()
            for ax, ay in anchors:
                t = (((ax - x1) * dx + (ay - y1) * dy) / (slen * slen)
                     if slen > 0 else 0.0)
                k0 = max(0, int((t - reach) * n))
                k1 = min(n, int((t + reach) * n) + 1)
                idxs.update(range(k0, k1 + 1))
            cells = set()
            for i in idxs:
                t = i / n
                cells |= set(occ._disk(x1 + dx * t, y1 + dy * t, keep))
            add(cells, li)

        def seg_near(x1, y1, x2, y2, reach):
            dx, dy = x2 - x1, y2 - y1
            L2 = dx * dx + dy * dy
            for ax, ay in anchors:
                if L2 > 0:
                    t = max(0.0, min(1.0, ((ax - x1) * dx + (ay - y1) * dy) / L2))
                    d2 = (ax - (x1 + t * dx)) ** 2 + (ay - (y1 + t * dy)) ** 2
                else:
                    d2 = (ax - x1) ** 2 + (ay - y1) ** 2
                if d2 < reach * reach:
                    return True
            return False

        for (x1, y1, x2, y2, half_w, nid, li) in exact_segs:
            keep = half_w + track_width / 2 + clearance
            if nid in net_ids or not seg_near(x1, y1, x2, y2, home_r + keep):
                continue
            add_seg(x1, y1, x2, y2, keep, li)
        for tr in tracks:
            if tr['net_id'] in net_ids:
                continue
            (x1, y1), (x2, y2) = tr['start'], tr['end']
            if not seg_near(x1, y1, x2, y2, home_r + trk_keep):
                continue
            add_seg(x1, y1, x2, y2, trk_keep, layer_idx[tr['layer']])
        return c_all, c_lay

    def _eff_exempt(home, carve, li):
        """The home exemption set effective on layer li (home minus carve)."""
        if carve is None:
            return home
        c_all, c_lay = carve
        if not c_all and not c_lay:
            return home
        return home - c_all - c_lay.get(li, _EMPTY)

    def _via_ctx(net_id, cx, cy, extra=0.0):
        """Gather the geometry near (cx, cy) that an off-centre via must clear
        (one scan per ball; the per-cell check then touches only these).

        `extra` widens the window past the base reach -- the lane-walk site
        chooser (#652) probes sites up to elbow + lane_max*pitch from the
        ball, which lands EXACTLY at the old window edge at 0.8mm pitch
        (0.4 + 3*0.8 = 2.8 vs W ~2.79): a committed via sitting on such a
        site was invisible and mez_rx stacked a V1P2 plane-drop via dead on
        R4's dog-bone via. Coarser pitches leave even k=2 sites unchecked."""
        W = home_r + via_size + 2.0 + extra

        def close(x, y):
            return abs(x - cx) <= W and abs(y - cy) <= W

        vias = [t for t in exact_vias if close(t[0], t[1])]
        vias += [(v['x'], v['y'], v['size'] / 2.0, (v['drill'] or 0.0) / 2.0,
                  v['net_id']) for v in vias_to_add if close(v['x'], v['y'])]
        # SAME-net reserved sites are included: drill hole-to-hole is net-
        # independent (#282), and two same-net dog-bone balls must not pick
        # the same gap site. The ring-clearance test stays foreign-only.
        resv = [t for t in reserved_sites if close(t[0], t[1])]
        pads = [t for t in exact_pads if close(t[0], t[1])]
        segs = [t[:6] for t in exact_segs
                if t[5] != net_id
                and min(t[0], t[2]) - W <= cx <= max(t[0], t[2]) + W
                and min(t[1], t[3]) - W <= cy <= max(t[1], t[3]) + W]
        for tr in tracks:
            if tr['net_id'] == net_id:
                continue
            (x1, y1), (x2, y2) = tr['start'], tr['end']
            if (min(x1, x2) - W <= cx <= max(x1, x2) + W
                    and min(y1, y2) - W <= cy <= max(y1, y2) + W):
                segs.append((x1, y1, x2, y2, tr['width'] / 2.0, tr['net_id']))
        return {'vias': vias, 'resv': resv, 'pads': pads, 'segs': segs}

    def _via_site_conflict(x, y, net_id, ctx, vr=None, vdr=None,
                           skip_resv=False):
        """Reason a full-size OFF-CENTRE via cannot sit at (x, y), or None.

        Mirrors the channel engine's via_in_pad_conflict (#370 B4) but also
        against THIS run's committed tracks/vias and the reserved (pending)
        via-in-pad sites: an off-centre via escapes the mutual centre-
        reservation contract, so its ring and drill must prove themselves
        against exact geometry (#393). Drill hole-to-hole is net-INDEPENDENT
        (same-net drill overlap is still a fab violation, #282); copper checks
        skip same-net items. `vr`/`vdr` override the nominal via ring/drill
        radii (the plane-drop centre fallback checks its pad-CLAMPED size --
        the nominal size would falsely reject small pads, #202).
        """
        if vr is None:
            vr = via_size / 2.0
        if vdr is None:
            vdr = (via_drill or 0.0) / 2.0
        for (ox, oy, orr, odr, onid) in ctx['vias']:
            d = math.hypot(ox - x, oy - y)
            if d < vdr + odr + _h2h - 1e-6:
                return "drill hole-to-hole vs a via"
            if onid != net_id and d < vr + orr + clearance - 1e-6:
                return "via ring vs a foreign via"
        for (ox, oy, orr, odr, onid) in ([] if skip_resv else ctx['resv']):
            d = math.hypot(ox - x, oy - y)
            if d < vdr + odr + _h2h - 1e-6:
                return "drill hole-to-hole vs a reserved via site"
            if onid != net_id and d < vr + orr + clearance - 1e-6:
                return "via ring vs a reserved via site"
        for (px, py, half, pdr, pnid, has_cu) in ctx['pads']:
            d = math.hypot(px - x, py - y)
            if pdr and d < vdr + pdr + _h2h - 1e-6:
                return "drill hole-to-hole vs a pad drill"
            if has_cu and pnid != net_id and d < vr + half + clearance - 1e-6:
                return "via ring vs a foreign pad"
        for (x1, y1, x2, y2, half_w, snid) in ctx['segs']:
            dx, dy = x2 - x1, y2 - y1
            L2 = dx * dx + dy * dy
            if L2 > 0:
                t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
                d = math.hypot(x - (x1 + t * dx), y - (y1 + t * dy))
            else:
                d = math.hypot(x - x1, y - y1)
            if d < vr + half_w + clearance - 1e-6:
                return "via ring vs a foreign track"
        return None

    # Geometry helpers ---------------------------------------------------------
    bx0, by0 = occ.cell(grid.min_x, grid.min_y)
    bx1, by1 = occ.cell(grid.max_x, grid.max_y)

    def in_bbox(ix, iy):
        return bx0 <= ix <= bx1 and by0 <= iy <= by1

    def heur(ix, iy):
        return max(0, min(ix - bx0, bx1 - ix, iy - by0, by1 - iy))

    # Distance to ONE named side, for a search that has been told which
    # way the ball must leave. `heur` measures the nearest boundary,
    # which is the right guide for a ball considered alone -- and the
    # wrong one for a ball joining a bundle that leaves on a particular
    # side, since the search then races to the near edge and the plan is
    # lost with no error anywhere.
    _SIDE_H = {
        'left': lambda ix, iy: max(0, ix - bx0),
        'right': lambda ix, iy: max(0, bx1 - ix),
        'up': lambda ix, iy: max(0, iy - by0),
        'down': lambda ix, iy: max(0, by1 - iy),
    }

    def _line_of(p, side):
        """(cell coordinate, tolerance in cells) for this pad's exit line
        hint, or None. Half a pitch of slack: the plan names a row GAP,
        and landing anywhere in that gap preserves the order, while
        demanding an exact cell would just fail the search.

        A SLOT hint (#622) outranks a line hint: the slot's interval
        maps directly onto the same (centre, tol) acceptance -- the
        tolerance IS the slot half-width instead of the fixed 0.3
        pitch."""
        if not side:
            return None
        sl = (escape_slot_hints or {}).get((round(p.global_x, 3),
                                            round(p.global_y, 3)))
        if sl is not None and sl[0] == side:
            _c = (sl[1] + sl[2]) / 2.0
            ix, iy = occ.cell(_c, _c)
            tol = max(1.0, ((sl[2] - sl[1]) / 2.0) / occ.res)
            return ((iy if side in ('left', 'right') else ix), tol)
        if not escape_line_hints:
            return None
        v = escape_line_hints.get((round(p.global_x, 3),
                                   round(p.global_y, 3)))
        if v is None:
            return None
        ix, iy = occ.cell(v, v)
        pitch = grid.pitch_y if side in ('left', 'right') else grid.pitch_x
        # 0.3 of a pitch, not 0.5: at half a pitch the neighbouring
        # PAD ROW is exactly on the tolerance edge and gets accepted,
        # which is what let 13 of 15 stubs land off the plan's gap
        # while the search reported success.
        tol = max(1.0, (pitch * 0.3) / occ.res)
        return ((iy if side in ('left', 'right') else ix), tol)

    def _left_on(side, cx, cy):
        """Did the path leave the array on `side`?"""
        if side == 'left':
            return cx < bx0
        if side == 'right':
            return cx > bx1
        if side == 'up':
            return cy < by0
        return cy > by1

    def depth(p):
        return min(p.global_x - grid.min_x, grid.max_x - p.global_x,
                   p.global_y - grid.min_y, grid.max_y - p.global_y)

    def astar(sx, sy, home, route_layers, allow_via, via_ok=None, net_id=0,
              carve=None, start_layer=None, cost_out=None, side=None,
              line=None):
        """Route from the pad to any boundary cell.

        `route_layers` = the set of layer indices the track may run on. The pad
        sits on the BGA's own layer (top_idx); if that layer is not in
        route_layers the route must drop a via in its own pad (the ONLY via
        allowed - mid-field vias don't fit in a dense array) and run on another
        layer. Straight moves are cheapest so escapes stay radial.

        `carve` = optional (all_layers_cells, {layer: cells}) from
        _carve_foreign: home cells that carry a FOREIGN stamp and therefore
        keep blocking despite the home exemption (#393).
        """
        cv_all, cv_lay = carve if carve is not None else (_EMPTY, None)

        def exempt(L, c):
            if c not in home or c in cv_all:
                return False
            if cv_lay:
                s = cv_lay.get(L)
                if s and c in s:
                    return False
            return True

        # `side` restricts which boundary counts as an exit, and steers
        # the heuristic at it. None = leave on whichever edge is nearest,
        # which is what every caller without a plan does.
        _h = _SIDE_H.get(side, heur) if side else heur
        # `line` pins WHERE along that side, in cells. The exit
        # coordinate is y for a left/right escape and x for up/down.
        _lax = 1 if side in ('left', 'right') else 0
        if side and line is not None:
            _lc, _ltol = line

            def _h(ix, iy, _b=_SIDE_H[side], _c=_lc):
                # still admissible: the true remaining cost is at least
                # the distance to the edge AND at least the distance
                # along it, so the max of the two underestimates.
                return max(_b(ix, iy), abs((iy if _lax else ix) - _c))
        start = (sx, sy, top_idx if start_layer is None else start_layer)
        g = {start: 0.0}
        pq = [(_h(sx, sy), start)]
        came = {}
        # #561 hot-loop: byte-equivalent mechanical speedup -- localize the
        # per-iteration lookups and index the occupancy bytearrays directly
        # (identical arithmetic, identical heap tuples, identical step order,
        # so the search expands and ties exactly as before).
        _heappop = heapq.heappop
        _heappush = heapq.heappush
        _g_get = g.get
        _ony = occ.ny
        _onx = occ.nx
        _ogrid = occ.grid
        _has_soft = occ.has_soft
        _soft_foreign = occ.soft_foreign
        _steps = _STEPS
        _home = home
        _pour_soft = getattr(occ, 'pour_soft', None)
        _vcache = {}
        while pq:
            _, cur = _heappop(pq)
            cx, cy, L = cur
            if not (bx0 <= cx <= bx1 and by0 <= cy <= by1) \
                    and (side is None or _left_on(side, cx, cy)) \
                    and (line is None or side is None
                         or abs((cy if _lax else cx) - line[0]) <= line[1]):
                if cost_out is not None:
                    # outside the window the tie-break heuristic is 0, so the
                    # popped priority IS the path's g-cost (#563 layer compare)
                    cost_out.append(_)
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                path.reverse()
                return path
            cg = g[cur]
            if L in route_layers:
                _gL = _ogrid[L]
                for dx, dy in _steps:
                    nx, ny = cx + dx, cy + dy
                    if not (0 <= nx < _onx and 0 <= ny < _ony):
                        continue
                    # Outside the array bbox nothing blocks (see the test
                    # below: it is guarded on being INSIDE). That is safe for
                    # an unrestricted search, which stops the moment it leaves
                    # -- but a side-restricted one does not stop on the wrong
                    # edge, so without this it escapes into the unchecked
                    # margin and walks around the outside of the array to the
                    # side it was told to use, through everyone else's exit
                    # stubs. Measured: 42 segment-segment CONTACTS, one net
                    # running 6.3 mm along x=133.50 past the west edge across
                    # every other net's exit. So a side-restricted search may
                    # only leave the bbox by crossing the edge it was asked
                    # for -- which is the goal, and terminates it.
                    if side is not None \
                            and not (bx0 <= nx <= bx1 and by0 <= ny <= by1) \
                            and not _left_on(side, nx, ny):
                        continue
                    # The LINE-restricted sibling of the fix above
                    # (#622, root-caused off plan_full2): an outside
                    # cell on the ACCEPTED side is a goal for a
                    # side-only search (which therefore stops at its
                    # first outside step) -- but under a line window a
                    # wrong-line outside cell is NOT a goal, expansion
                    # still allowed it, and outside the bbox the
                    # occupancy test below never runs. So the search
                    # exited anywhere on its side and rode the
                    # UNCHECKED outside margin along the edge to the
                    # window, through every other net's exit stubs
                    # (SA8: an 8.4 mm rail at x=127.03, in-contact
                    # with 3 neighbours -- the long-standing "LINES=1
                    # broken at the fanout" disease). A line-restricted
                    # search may leave the bbox only THROUGH its
                    # window, where it immediately terminates as goal.
                    if side is not None and line is not None \
                            and not (bx0 <= nx <= bx1 and by0 <= ny <= by1) \
                            and abs((ny if _lax else nx) - line[0]) > line[1]:
                        continue
                    if (bx0 <= nx <= bx1 and by0 <= ny <= by1) \
                            and _gL[nx * _ony + ny] \
                            and not exempt(L, (nx, ny)):
                        continue
                    # No corner-cutting: a diagonal step past a blocked orthogonal
                    # neighbour clips that obstacle's clearance (the diagonal line
                    # passes nearer the via/pad than either cell). Forbid it.
                    if dx != 0 and dy != 0:
                        if (_gL[(cx + dx) * _ony + cy] and not exempt(L, (cx + dx, cy))) or \
                           (_gL[cx * _ony + cy + dy] and not exempt(L, (cx, cy + dy))):
                            continue
                    step = 1.0 if (dx == 0 or dy == 0) else 1.6   # discourage zig-zag
                    # Soft keep-out (#278): stepping through a movable
                    # passive's pad zone is legal but leaves a graze the
                    # cap-placement step may be unable to clear -- pay a
                    # steep per-cell premium so routes detour when any
                    # detour exists, and only graze when boxed in.
                    if _has_soft and (nx, ny) not in _home and \
                            _soft_foreign(L, nx, ny, net_id):
                        step += 4.0
                    if _pour_soft is not None:
                        _ps = _pour_soft.get(L)
                        if _ps is not None:
                            step += _ps[nx * _ony + ny]
                    nxt = (nx, ny, L)
                    ng = cg + step
                    if ng < _g_get(nxt, 1e18):
                        g[nxt] = ng
                        came[nxt] = cur
                        # The inlined heuristic is the #561 hot-loop copy of
                        # `heur`. With a side named it has to follow that side
                        # instead -- min-over-all-edges is still admissible, so
                        # the search would stay CORRECT, but it would race at
                        # the nearest edge and expand far more to reach the one
                        # asked for. The no-side branch is untouched, so a run
                        # without hints expands exactly as before.
                        _heappush(pq, (ng + (_h(nx, ny) if side else
                                             max(0, min(nx - bx0, bx1 - nx,
                                                        ny - by0, by1 - ny))),
                                       nxt))
            # The single via, only in the ball's own pad. A through via spans
            # all layers, so the site must also clear immovable foreign copper
            # on layers the run-blocking test never looks at (via_ok, #253/
            # #254). commit() emits the pad-CLAMPED size only at the exact pad
            # centre cell (sx, sy); any other home cell gets the full via_size
            # -- the gate must use the size that will actually be emitted.
            _at_center = (cx, cy) == (sx, sy)
            # #561: memoize via_ok per (cell, at_center) WITHIN this search --
            # the answer is pure until commit() mutates copper (after astar
            # returns), and the same home cells are re-queried on every heap
            # visit (profiled: 1.6M calls, 503 of 582 s). Byte-equivalent.
            if allow_via and (cx, cy) in home and via_ok is not None:
                _vk = (cx, cy, _at_center)
                _vr = _vcache.get(_vk)
                if _vr is None:
                    _vr = bool(via_ok(*occ.xy(cx, cy), _at_center))
                    _vcache[_vk] = _vr
            else:
                _vr = True
            if allow_via and (cx, cy) in home and \
                    (via_ok is None or _vr):
                for L2 in route_layers:
                    if L2 == L:
                        continue
                    # An OFF-centre via sits inside the home exemption disk,
                    # which hides real foreign obstacles (a neighbour ball's
                    # via/track) from the blocking test -- require the
                    # destination-layer cell to be genuinely clear there.
                    # (The centre cell keeps the original semantics: its via
                    # is clamped inside the pad copper.)
                    if not _at_center and occ.blocked(L2, cx, cy):
                        continue
                    nxt = (cx, cy, L2)
                    ng = cg + via_cost
                    if ng < g.get(nxt, 1e18):
                        g[nxt] = ng
                        came[nxt] = cur
                        heapq.heappush(pq, (ng + _h(cx, cy), nxt))
        return None

    # Layer policy: near-edge balls escape on the BGA's own layer (no via),
    # keeping the rim clear of vias so the deeper balls can always run UNDER them
    # on the other layers. Deeper balls drop a via in their pad and route there.
    nl = occ.nl
    inner_layers = set(i for i in range(nl) if i != top_idx) or {top_idx}
    pitch = (grid.pitch_x + grid.pitch_y) / 2
    # KICAD_UNDERPAD_OUTER_RINGS caps how many rings get a via-less
    # top-layer escape attempt before falling to via-in-pad. Default is
    # UNLIMITED (#424): Phase A runs outside-in, so trying every ball is
    # safe and fewer under-package barrels measured as better completion.
    import env_knobs
    if env_knobs.UNDERPAD_OUTER_RINGS:
        try:
            outer_rings = float(env_knobs.UNDERPAD_OUTER_RINGS)
        except ValueError:
            pass
    if dogbone and math.isinf(outer_rings):
        # Dog-bone (#128) WANTS its vias in the gaps -- an unlimited surface
        # phase claims the diagonal gap sites and forces via-in-pad fallbacks
        # (the #267 graze class it exists to kill). Keep its classic rim-only
        # top phase; an explicit KICAD_UNDERPAD_OUTER_RINGS still overrides.
        outer_rings = 2.0
    outer_depth = outer_rings * pitch

    signal_pads.sort(key=depth, reverse=True)

    tracks: List[Dict] = []
    vias_to_add: List[Dict] = []
    failed: List[str] = []
    # Balls whose planned side had no route, so they escaped some other
    # way. Reported rather than swallowed: a plan silently half-applied
    # is worse than one that says which half did not fit.
    _hint_missed: List[str] = []
    _line_missed: List[str] = []
    _slot_missed: List[str] = []
    # #622 layer hints: hinted layer index per pad key, tried FIRST at
    # every layer-choosing site below; delivered layers recorded here
    # and published to bga_fanout.LAST_LAYER_HINT_REPORT at the end.
    _got_layer: Dict = {}

    def _lhint_idx(p):
        h = (escape_layer_hints or {}).get((round(p.global_x, 3),
                                            round(p.global_y, 3)))
        if h is None or h not in layers:
            return None
        return layers.index(h)

    def _lh_order(p, Ls):
        _o = sorted(Ls)
        _h = _lhint_idx(p)
        if _h in Ls:
            _o = [_h] + [L for L in _o if L != _h]
        return _o

    def _rec_got(p, L_idx):
        _got_layer[(round(p.global_x, 3),
                    round(p.global_y, 3))] = (layers[L_idx], p.net_name)

    nvia = 0
    n_fcu = 0

    # Dog-bone escape sites (issue #128): ball id -> (vx, vy) via site in the
    # diagonal inter-ball gap. Filled by the dog-bone site chooser before
    # Phase B; consulted by home_of, the coupled templates and Phase D.
    db_site: Dict[int, Tuple[float, float]] = {}
    # Full stub polyline per ball (ball -> [elbow ->] site) for lane-walked
    # dog-bones (#652): db_site keeps the VIA spot for the site consumers;
    # everything stub-shaped (emission, corridor exemption, reservation)
    # walks db_path.
    db_path: Dict[int, List[Tuple[float, float]]] = {}

    def _stub_cells_seg(x1, y1, x2, y2):
        """Occupancy cells of one dog-bone stub segment (own-copper
        exemption, sampled at sub-resolution along the segment)."""
        cells = set()
        dx, dy = x2 - x1, y2 - y1
        n = max(2, int(math.hypot(dx, dy) / max(res, 1e-9)) + 1)
        r = track_width / 2 + clearance + margin
        for i in range(n + 1):
            t = i / n
            cells |= set(occ._disk(x1 + dx * t, y1 + dy * t, r))
        return cells

    def home_of(p):
        # Exempt the ball's OWN pad+via keepout so the track can break out of its
        # own pad (the keepout ring is wider than the pad and would otherwise
        # trap a route). max(pad,via)_keep < pitch, so it never reaches a
        # neighbouring pad. Same real-coordinate disk the blocking stamped, so
        # the exemption covers it exactly.
        cells = set(occ._disk(p.global_x, p.global_y, max(pad_keep, via_keep)))
        # A dog-bone ball's OWN copper extends to its gap via + stub (#128);
        # foreign stamps inside this lens are re-blocked by _carve_foreign
        # (callers pass the site as a second carve anchor).
        s = db_site.get(id(p))
        if s is not None:
            cells |= set(occ._disk(s[0], s[1], via_keep))
            pts = db_path.get(id(p)) or [(p.global_x, p.global_y), s]
            for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                cells |= _stub_cells_seg(ax, ay, bx, by)
        return cells

    def snap_exit_end(pts, li, exempt=None, net_id_for_snap=0):
        """Move an escape's outer free end onto the routing grid (issue #302).

        The occupancy lattice has an arbitrary origin, so A* exit points land
        off the router's --grid-step grid; the channel engine snaps its stub
        ends (#149) but the under-pad engine never did -- and since 'auto'
        fallback (#288) its results appear on boards that never asked for
        underpad. Round the final vertex to the grid: OUTWARD on the dominant
        travel axis (the free end must stay past the boundary), nearest on the
        other. The re-aimed final segment is clearance-checked; on a conflict
        the original off-grid end is kept (route.py still copes, just slower).
        """
        if grid_step <= 0 or len(pts) < 2:
            return
        (ax, ay), (bx, by) = pts[-2], pts[-1]
        dx, dy = bx - ax, by - ay

        def snap_out(v, d):
            k = v / grid_step
            if d > 1e-9:
                k = math.ceil(k - 1e-9)
            elif d < -1e-9:
                k = math.floor(k + 1e-9)
            else:
                k = round(k)
            return k * grid_step

        if abs(dx) >= abs(dy):
            gx, gy = snap_out(bx, dx), snap_out(by, 0.0)
        else:
            gx, gy = snap_out(bx, 0.0), snap_out(by, dy)
        if abs(gx - bx) < 1e-9 and abs(gy - by) < 1e-9:
            return
        # Append a short jog from the found end to the grid point rather than
        # re-aiming the whole final segment: the corridor behind the end is
        # often exactly one lattice cell wide (neighbouring escapes block the
        # rest), so a lateral re-aim rarely clears -- but the sub-grid-step
        # jog lives just past the boundary where only the fanned-out exits
        # run, comfortably spaced.
        def _ok(p, q):
            # same clearance contract as every A* move: the occupancy grid is
            # inflated by obstacle half-size + track/2 + clearance. Also refuse
            # movable-cap soft keep-outs (#278) -- a jog must not create a new
            # graze for cap-opt to clean up.
            return (occ.seg_clear(li, p, q, exempt=exempt) and
                    not (occ.has_soft and occ.seg_soft_hits(li, p, q, net_id_for_snap,
                                                            exempt=exempt)))
        if _ok((bx, by), (gx, gy)):
            pts.append((gx, gy))
        elif _ok((ax, ay), (gx, gy)):
            pts[-1] = (gx, gy)

    def commit(p, path, carve=None):
        """Emit tracks + the via-in-pad for a found path and mark occupancy."""
        nonlocal nvia
        sx, sy = occ.cell(p.global_x, p.global_y)
        home = home_of(p)
        net_id = p.net_id
        runs, via_cells = [], []
        cur_layer = path[0][2]
        cur_cells = [(path[0][0], path[0][1])]
        for (ix, iy, L) in path[1:]:
            if L != cur_layer:
                runs.append((cur_layer, cur_cells))
                via_cells.append((ix, iy))
                cur_layer = L
                cur_cells = [(ix, iy)]
            else:
                cur_cells.append((ix, iy))
        runs.append((cur_layer, cur_cells))
        # Compute (and grid-snap, #302) every run's polyline BEFORE blocking
        # occupancy: blocking stamps a track+clearance disk around each own
        # cell, which would veto the snap jog against the route's own copper.
        runs_pts = []
        for ri, (li, cells) in enumerate(runs):
            pts = _segments_from_cells(occ, cells)
            if ri == 0 and pts:
                pts[0] = (p.global_x, p.global_y)  # start exactly at the pad
            if ri == len(runs) - 1:
                snap_exit_end(pts, li, exempt=_eff_exempt(home, carve, li),
                              net_id_for_snap=net_id)
            runs_pts.append(pts)
        for ri, (li, _cells) in enumerate(runs):
            pts = runs_pts[ri]
            # Stamp the EMITTED polyline, and stamp it INSIDE `home` too (#505).
            # Two leaks this closes, both of which shipped real DRC on orangecrab
            # U3 (56 grazes at 0.166mm centre-to-centre against a 0.2mm
            # requirement; 0 after this change, and one MORE ball escapes):
            #   * the old stamp skipped cells in the ball's own home disk, so its
            #     near-pad escape copper was invisible to EVERY later ball. The
            #     home exemption is a ROUTE-TIME affordance (astar, so a route can
            #     break out of its own keep-out ring) -- it must not also erase the
            #     copper from the map once committed. _carve_foreign cannot cover
            #     this: it only re-blocks foreign stamps inside the QUERYING ball's
            #     home, so a later escape whose conflicting cell lies outside its
            #     own home never consults the carve, and the grid holds no stamp.
            #     Adjacent home disks overlap (home_r > half pitch), so a passing
            #     ball-to-boundary escape runs straight through that blind lens.
            #   * cell centres are not the copper: commit() rewrites pts[0] to the
            #     exact pad centre and snap_exit_end may move the tail, so stamping
            #     A* cells left the real first/last segments unstamped.
            # Same-net traversal is preserved: a later ball of the SAME net lifts
            # this via its own home exemption, and _carve_foreign does not re-add
            # same-net stamps.
            for a, b in zip(pts, pts[1:]):
                occ.block_segment(li, a, b, trk_keep)
                tracks.append({'start': a, 'end': b, 'width': track_width,
                               'layer': layers[li], 'net_id': net_id})
        for (ix, iy) in via_cells:
            # snap to the pad centre only if the via is still in the pad cell
            in_pad = (ix, iy) == (sx, sy)
            vx, vy = (p.global_x, p.global_y) if in_pad else occ.xy(ix, iy)
            # via-in-pad: clamp to the pad (#202); off-pad transition vias keep size
            v_size, v_drill, vkeep = via_for_pad(p) if in_pad else (via_size, via_drill, via_keep)
            occ.block_all(vx, vy, vkeep)
            vias_to_add.append({'x': vx, 'y': vy, 'size': v_size,
                                'drill': v_drill,
                                'layers': [layers[0], layers[-1]], 'net_id': net_id})
            nvia += 1

    # Differential pairs (issue #182): collect the pairs we can try to escape
    # coupled. A pair qualifies only if BOTH balls are in our routable signal
    # set (not plane / NC / already-fanned) and they sit ~one pitch apart and
    # axis-aligned (the array is axis-aligned in this frame). Anything else
    # routes single-ended like before.
    spacing = track_width + diff_pair_gap
    half_sp = spacing / 2.0
    signal_by_net = {p.net_id: p for p in signal_pads}
    coupled_targets = []          # (base_name, p_pad, n_pad)
    paired_net_ids = set()
    if diff_pairs:
        for base, dp in diff_pairs.items():
            if not dp.is_complete:
                continue
            pp = signal_by_net.get(dp.p_pad.net_id)
            nn = signal_by_net.get(dp.n_pad.net_id)
            if pp is None or nn is None:
                continue
            dx = abs(pp.global_x - nn.global_x)
            dy = abs(pp.global_y - nn.global_y)
            if math.hypot(dx, dy) > 1.5 * pitch or min(dx, dy) > 0.25 * pitch:
                continue          # not an adjacent, axis-aligned pair
            coupled_targets.append((base, pp, nn))
            paired_net_ids.add(pp.net_id)
            paired_net_ids.add(nn.net_id)

    single_pads = [p for p in signal_pads if p.net_id not in paired_net_ids]

    def boundary_dist(mx, my, e):
        """Distance from (mx,my) to the array boundary in cardinal dir e, plus
        the exit margin (so the coupled run ends past the boundary)."""
        ex, ey = e
        if ex > 0:
            return (grid.max_x - mx) + exit_margin
        if ex < 0:
            return (mx - grid.min_x) + exit_margin
        if ey > 0:
            return (grid.max_y - my) + exit_margin
        return (my - grid.min_y) + exit_margin

    def _via_spot(pad, use_via):
        """Where this ball's escape via will sit: its dog-bone gap site when
        one was assigned (#128), else the pad centre (via-in-pad)."""
        if use_via and dogbone:
            s = db_site.get(id(pad))
            if s is not None:
                return s
        return (pad.global_x, pad.global_y)

    def _via_gate_ok(pad):
        """locked-SMD gate for this ball's escape via at its real spot."""
        vx, vy = _via_spot(pad, True)
        if (vx, vy) == (pad.global_x, pad.global_y):
            half = clamp_via_to_pad(via_size, via_drill, pad, floors)[0] / 2.0
        else:
            half = via_size / 2.0
        return via_site_ok(vx, vy, half)

    def _drop_escape_via(pad):
        """Emit this ball's escape via -- at its dog-bone site (plus the
        pad->via stub, whose corridor was stamped at reservation) when
        assigned, else clamped in-pad."""
        nonlocal nvia
        s = db_site.get(id(pad)) if dogbone else None
        if s is None:
            v_size, v_drill, vkeep = via_for_pad(pad)
            vx, vy = pad.global_x, pad.global_y
        else:
            v_size, v_drill, vkeep = via_size, via_drill, via_keep
            vx, vy = s
            pts = db_path.get(id(pad)) or [(pad.global_x, pad.global_y), s]
            for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                tracks.append({'start': (ax, ay), 'end': (bx, by),
                               'width': track_width,
                               'layer': layers[top_idx],
                               'net_id': pad.net_id})
        occ.block_all(vx, vy, vkeep)
        vias_to_add.append({'x': vx, 'y': vy, 'size': v_size, 'drill': v_drill,
                            'layers': [layers[0], layers[-1]],
                            'net_id': pad.net_id})
        nvia += 1

    def _partner_via_clear(this_segs, other):
        """Dog-bone coupled escape (#128): a leg must clear its PARTNER's via
        spot at full clearance -- the template's home exemption covers both
        balls' lenses, so seg_clear cannot see the partner's gap via."""
        vx, vy = _via_spot(other, True)
        need = via_size / 2.0 + track_width / 2.0 + clearance - 1e-6
        return all(_pt_seg_d(vx, vy, a[0], a[1], b[0], b[1]) >= need
                   for a, b in this_segs)

    def try_coupled(pp, nn, candidates):
        """Escape one differential pair coupled, converging (<=45 deg) to +-half_sp
        about the pair centerline and running parallel in the escape direction
        (perpendicular to the P-N axis, toward the nearer boundary) out past the
        boundary.

        `candidates` is a list of (layer_index, use_via): a top-layer escape
        (use_via=False) keeps both pads via-free - the key to letting deeper
        pairs run UNDER an outer pair on inner layers without grazing a through-
        via (which blocks every layer). An inner escape (use_via=True) drops a
        via-in-pad in each ball. The first clear (direction, candidate) wins.
        Returns True and commits, else False (caller defers the pair)."""
        nonlocal nvia
        mx, my = (pp.global_x + nn.global_x) / 2.0, (pp.global_y + nn.global_y) / 2.0
        ax, ay = pp.global_x - nn.global_x, pp.global_y - nn.global_y   # axis P<-N
        al = math.hypot(ax, ay)
        if al < 1e-6:
            return False
        ax, ay = ax / al, ay / al
        half_axis = al / 2.0
        if half_axis <= half_sp:        # closer than the diff spacing - no room to converge
            return False
        Lc = half_axis - half_sp        # 45-degree converge length
        # Two escape directions perpendicular to the axis; nearer boundary first.
        cand_e = sorted([(-ay, ax), (ay, -ax)], key=lambda e: boundary_dist(mx, my, e))
        homes = home_of(pp) | home_of(nn)
        # Foreign copper in the pair's home lens keeps blocking (#393). The
        # partner's OWN pad/reservation is not carved (both pair nets exempt):
        # partner-vs-partner spacing is the coupled template's own contract.
        carve = _carve_foreign([(pp.global_x, pp.global_y),
                                (nn.global_x, nn.global_y)],
                               homes, {pp.net_id, nn.net_id})
        eff_cache = {}

        def eff(L):
            s = eff_cache.get(L)
            if s is None:
                s = eff_cache[L] = _eff_exempt(homes, carve, L)
            return s
        # Two passes (#278): first demand corridors whose runs stay clear of
        # every foreign movable-cap pad (soft keep-out); only if no such
        # corridor exists anywhere accept a grazing one.
        for strict in ((True, False) if occ.has_soft else (False,)):
          for e in cand_e:
            ex, ey = e
            De = boundary_dist(mx, my, e)
            if De <= Lc + 0.01:
                continue
            for L, use_via in candidates:
                if use_via and locked_smd_pads and not all(
                        _via_gate_ok(q) for q in (pp, nn)):
                    continue  # a through via would hit locked copper
                segs = []
                ok = True
                for pad, side in ((pp, 1.0), (nn, -1.0)):
                    p0 = _via_spot(pad, use_via)
                    pc = (mx + Lc * ex + side * half_sp * ax,
                          my + Lc * ey + side * half_sp * ay)
                    pe = (mx + De * ex + side * half_sp * ax,
                          my + De * ey + side * half_sp * ay)
                    if not (occ.seg_clear(L, p0, pc, exempt=eff(L))
                            and occ.seg_clear(L, pc, pe, exempt=eff(L))):
                        ok = False
                        break
                    if strict and (
                            occ.seg_soft_hits(L, p0, pc, pad.net_id, exempt=eff(L))
                            or occ.seg_soft_hits(L, pc, pe, pad.net_id, exempt=eff(L))):
                        ok = False
                        break
                    segs.append((pad, p0, pc, pe))
                if ok and use_via and dogbone:
                    for pad, p0, pc, pe in segs:
                        if not _partner_via_clear([(p0, pc), (pc, pe)],
                                                  nn if pad is pp else pp):
                            ok = False
                            break
                if not ok:
                    continue
                for pad, p0, pc, pe in segs:
                    occ.block_segment(L, p0, pc, trk_keep)
                    occ.block_segment(L, pc, pe, trk_keep)
                    for a, b in ((p0, pc), (pc, pe)):
                        if math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-6:
                            continue
                        tracks.append({'start': a, 'end': b, 'width': track_width,
                                       'layer': layers[L], 'net_id': pad.net_id})
                    if use_via:
                        _drop_escape_via(pad)
                return True
        return False

    escaped = set()      # id(pad) of balls already committed (top-layer escapes)

    def try_coupled_endon(pp, nn, candidates):
        """Escape a 'stacked' pair (the two balls in line toward the nearest edge)
        coupled, converging to the diff spacing and running parallel past the
        boundary.

        The lead ball (nearer the edge) runs straight out at -half_sp; the trail
        ball bulges out through the ~1-pitch gap beside the lead (clearing the
        lead pad/via), then converges to +half_sp. On the top layer
        (use_via=False) this is via-free, so deeper balls run UNDER the pair; on
        an inner layer (use_via=True) each ball drops a via-in-pad and the pair
        runs under the via-free edge rows to reach the boundary. The clean
        converged exit is what route_diff needs to couple them (two independently
        routed stubs do not pick up). `candidates` = [(layer, use_via)]; the first
        that fits wins. All-or-nothing."""
        nonlocal nvia
        mx, my = (pp.global_x + nn.global_x) / 2.0, (pp.global_y + nn.global_y) / 2.0
        ax, ay = pp.global_x - nn.global_x, pp.global_y - nn.global_y
        al = math.hypot(ax, ay)
        if al < 1e-6:
            return False
        ax, ay = ax / al, ay / al
        half_axis = al / 2.0
        run = max(0.3, 2.0 * half_sp)            # length of the clean parallel tail
        # Foreign copper in either ball's home lens keeps blocking (#393);
        # the partner's own pad/reservation stays exempt (template contract).
        _nets = {pp.net_id, nn.net_id}
        hp, hn = home_of(pp), home_of(nn)
        cvp = _carve_foreign([(pp.global_x, pp.global_y)], hp, _nets)
        cvn = _carve_foreign([(nn.global_x, nn.global_y)], hn, _nets)
        # Same soft keep-out preference as try_coupled (#278): soft-free first.
        for strict in ((True, False) if occ.has_soft else (False,)):
          for sgn in (1.0, -1.0):                # escape direction along the axis
            ex, ey = sgn * ax, sgn * ay
            De = boundary_dist(mx, my, (ex, ey))
            if De <= half_axis + run + 0.05:
                continue                          # edge must be ahead of the lead ball
            px, py = -ey, ex                      # perp unit (cross axis)
            # lead = the ball in the +e (toward-edge) direction. Each ball's
            # segments are clearance-checked against the OTHER ball's pad/via
            # (only the ball's own home is exempt), so the trail must really clear
            # the lead's via.
            if ex * (pp.global_x - mx) + ey * (pp.global_y - my) > 0:
                lead, trail, lh, th, lcv, tcv = pp, nn, hp, hn, cvp, cvn
            else:
                lead, trail, lh, th, lcv, tcv = nn, pp, hn, hp, cvn, cvp

            def pt(along, off):
                return (mx + along * ex + off * px, my + along * ey + off * py)

            for L, use_via in candidates:
                if use_via and locked_smd_pads and not all(
                        _via_gate_ok(q) for q in (pp, nn)):
                    continue  # a through via would hit locked copper
                for tside in (1.0, -1.0):         # which gap the trail bulges through
                    lside = -tside
                    lead_segs = [(_via_spot(lead, use_via), pt(De - run, lside * half_sp)),
                                 (pt(De - run, lside * half_sp), pt(De, lside * half_sp))]
                    W = pt(half_axis, tside * half_axis)     # gap beside the lead pad
                    trail_segs = [(_via_spot(trail, use_via), W),
                                  (W, pt(De - run, tside * half_sp)),
                                  (pt(De - run, tside * half_sp), pt(De, tside * half_sp))]
                    exl = _eff_exempt(lh, lcv, L)
                    ext = _eff_exempt(th, tcv, L)
                    if not (all(occ.seg_clear(L, a, b, exempt=exl) for a, b in lead_segs)
                            and all(occ.seg_clear(L, a, b, exempt=ext) for a, b in trail_segs)):
                        continue
                    if dogbone and use_via and not (
                            _partner_via_clear(lead_segs, trail)
                            and _partner_via_clear(trail_segs, lead)):
                        continue
                    if strict and (
                            any(occ.seg_soft_hits(L, a, b, lead.net_id, exempt=exl)
                                for a, b in lead_segs)
                            or any(occ.seg_soft_hits(L, a, b, trail.net_id, exempt=ext)
                                   for a, b in trail_segs)):
                        continue
                    for segs, pad in ((lead_segs, lead), (trail_segs, trail)):
                        for a, b in segs:
                            if math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-6:
                                continue
                            occ.block_segment(L, a, b, trk_keep)
                            tracks.append({'start': a, 'end': b, 'width': track_width,
                                           'layer': layers[L], 'net_id': pad.net_id})
                        if use_via:
                            _drop_escape_via(pad)
                    escaped.update((id(pp), id(nn)))
                    return True
        return False

    n_coupled = 0
    coupled_fail = []

    # Phase C1: escape diff pairs on the TOP (pad) layer with NO vias - edge
    # pairs first so they claim the rim. A broadside pair (axis parallel to the
    # edge) runs straight off the boundary; a stacked pair (axis pointing at the
    # edge) escapes END-ON, the trail ball bulging through the pad gap. Keeping
    # these pairs via-free is what lets the deeper pairs/balls run UNDER them on
    # inner layers without grazing a through-via - the "outer edges on top, inner
    # pads under them" strategy.
    coupled_targets.sort(key=lambda t: depth(t[1]) + depth(t[2]))
    remaining_pairs = []
    import env_knobs as _ek
    for _ci, (base, pp, nn) in enumerate(coupled_targets):
        if cancel_check and cancel_check():    # #621
            break
        _prog(_ci + 1, len(coupled_targets), f"coupled pair {base}")
        # #652 pair dive-first (human survey): diff pairs DIVE at the first
        # gap row (adjacent dog-bone vias, coupled inner run) instead of
        # running surface legs across the remaining rows -- measured on the
        # human orangecrab, even last-row pairs dive. Long surface pair legs
        # are exactly what walls the single-ended balls' corridors
        # (USER_BUTTON/RAM_LDM). KICAD_FANOUT_PAIR_DIVE=0 restores the
        # surface-first behaviour.
        if dogbone and _ek.FANOUT_PAIR_DIVE:
            remaining_pairs.append((base, pp, nn))
            continue
        _lhp = _lhint_idx(pp)
        _lhn = _lhint_idx(nn)
        if ((_lhp is not None and _lhp != top_idx)
                or (_lhn is not None and _lhn != top_idx)):
            remaining_pairs.append((base, pp, nn))
            continue
        if (try_coupled(pp, nn, [(top_idx, False)])
                or try_coupled_endon(pp, nn, [(top_idx, False)])):
            n_coupled += 1
            escaped.update((id(pp), id(nn)))
        else:
            remaining_pairs.append((base, pp, nn))  # try an inner coupled escape

    # Phase A: via-less escapes on the BGA layer, OUTSIDE-IN (#424): peel the
    # onion -- ring 1 escapes straight out claiming minimal rim, ring 2
    # threads between those stubs, and so on as deep as the surface allows.
    # Outside-in ordering means a deep ball can never strand a shallow one,
    # which is what makes trying EVERY ball (outer_rings default now
    # unlimited) safe; each failure simply falls through to the inner
    # (via-in-pad) phase, which keeps its deepest-first order. Fewer
    # under-package barrels measured directly as completion on ottercast
    # (via-in-pad 14 -> 11 final issues with only ~20 barrels).
    # (Paired balls are handled coupled below.)
    def _pt_seg_d(px_, py_, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((px_ - x1) * dx + (py_ - y1) * dy) / L2))
        return math.hypot(px_ - (x1 + t * dx), py_ - (y1 + t * dy))

    def _seg_conflict_item(net_id, gx, gy, vx, vy, ctx):
        """The first registered item one stub segment on the BGA's own
        layer would violate, or None (the occupancy exemption around the
        pad/site would hide foreign copper there, #393 -- so the stub
        proves itself against the registries)."""
        hw = track_width / 2.0
        for (x1, y1, x2, y2, half_w, snid) in ctx['segs']:
            if snid == net_id:
                continue
            # conservative: stub endpoint-to-seg / seg-to-stub sampling
            if (_pt_seg_d(x1, y1, gx, gy, vx, vy) < hw + half_w + clearance - 1e-6
                    or _pt_seg_d(x2, y2, gx, gy, vx, vy) < hw + half_w + clearance - 1e-6
                    or _pt_seg_d(gx, gy, x1, y1, x2, y2) < hw + half_w + clearance - 1e-6
                    or _pt_seg_d(vx, vy, x1, y1, x2, y2) < hw + half_w + clearance - 1e-6):
                return ('seg', snid, (x1, y1, x2, y2, half_w))
        for (ox, oy, orr, odr, onid) in ctx['vias'] + ctx['resv']:
            if onid == net_id:
                continue
            if _pt_seg_d(ox, oy, gx, gy, vx, vy) < hw + orr + clearance - 1e-6:
                return ('via', onid, (ox, oy, orr))
        for (px_, py_, half, pdr, pnid, has_cu) in ctx['pads']:
            if not has_cu or pnid == net_id:
                continue
            if _pt_seg_d(px_, py_, gx, gy, vx, vy) < hw + half + clearance - 1e-6:
                return ('pad', pnid, (px_, py_, half))
        return None

    def _seg_conflict(net_id, gx, gy, vx, vy, ctx):
        return _seg_conflict_item(net_id, gx, gy, vx, vy, ctx) is not None

    def _lane_pad_exempt(x1, y1, x2, y2):
        # The inter-row lane clears the flanking ball pads by only a few um
        # past the floor (0.25 - pad_half - track/2 vs clearance) -- legal by
        # the EXACT check, but the raster's inflated pad stamps can eat the
        # whole lane cell. Exempt the flanking OWN-FOOTPRINT pads' disks;
        # _seg_conflict has them exactly (registry-complete).
        cells = set()
        for p2 in footprint.pads:
            if _pt_seg_d(p2.global_x, p2.global_y, x1, y1, x2, y2) \
                    < max(grid.pitch_x, grid.pitch_y) * 0.55:
                cells |= set(occ._disk(p2.global_x, p2.global_y, pad_keep))
        return cells

    _lane_dirs = {'right': (1.0, 0.0), 'left': (-1.0, 0.0),
                  'down': (0.0, 1.0), 'up': (0.0, -1.0)}

    def _lane_run_escape(p, only_side=None, line=None):
        """Exact-checked surface LANE-RUN escape (#622, ported from
        bus622-take2): pad, 45-degree jog into an adjacent inter-row /
        inter-column gap lane, straight down the lane's midline and out
        the boundary. The human's dominant escape for near-edge balls
        that the raster A* cannot serve: its inflated pad stamps close a
        fine-pitch lane the exact check proves legal (take2's evidence:
        allwinner U1.R19/SDQ4 shipped an 8 mm wrong-way snake where the
        human exits due east through exactly this lane). Exact machinery
        only -- the same registry checks as the #652 lane walk; the raster
        is consulted with the flanking-pad exemption. Sides nearest-
        boundary first, a planned side (escape_dir_hints) promoted to the
        front -- or the ONLY side tried, with `only_side` -- capped at 4
        rings (deeper balls are dog-bone territory). The run ends at the
        array boundary like every other escape here. Returns the polyline
        or None."""
        gx, gy = p.global_x, p.global_y
        hx, hy = grid.pitch_x / 2.0, grid.pitch_y / 2.0
        d_e, d_w = grid.max_x - gx, gx - grid.min_x
        d_s, d_n = grid.max_y - gy, gy - grid.min_y
        sides = sorted(((d_e, (1.0, 0.0)), (d_w, (-1.0, 0.0)),
                        (d_s, (0.0, 1.0)), (d_n, (0.0, -1.0))),
                       key=lambda t: t[0])
        if only_side is not None:
            sides = [t for t in sides if t[1] == _lane_dirs.get(only_side)]
        else:
            _pd = (escape_dir_hints or {}).get((round(gx, 3), round(gy, 3)))
            if _pd in _lane_dirs:
                sides.sort(key=lambda t: t[1] != _lane_dirs[_pd])
        # Ring-parity stagger like _choose_dogbone_site: successive rings
        # prefer opposite lanes, spreading same-direction runs across both.
        alt = 1.0 if int(round(depth(p) / max(pitch, 1e-9))) % 2 == 0 else -1.0
        pad_ex = set(occ._disk(gx, gy, max(pad_keep, via_keep)))
        _dbg = os.environ.get('KICAD_FANOUT_LANE_DEBUG', '')
        _dbg = bool(_dbg) and any((p.net_name or '').endswith(s)
                                  for s in _dbg.split(',') if s)

        def _why(msg):
            if _dbg:
                print(f"    lane-run {p.net_name}: {msg}")
        for dist, (sx_, sy_) in sides:
            if dist > 4.0 * pitch:
                _why(f"side ({sx_:+.0f},{sy_:+.0f}) at {dist:.2f} mm is past 4 rings")
                continue
            ctx = _via_ctx(p.net_id, gx, gy, extra=dist + pitch)
            if sx_:
                end_x = grid.max_x if sx_ > 0 else grid.min_x
                cands = [((gx + sx_ * hx, gy + s * hy), (end_x, gy + s * hy))
                         for s in (alt, -alt)]
            else:
                end_y = grid.max_y if sy_ > 0 else grid.min_y
                cands = [((gx + s * hx, gy + sy_ * hy), (gx + s * hx, end_y))
                         for s in (alt, -alt)]
            if line is not None and only_side is not None:
                # the plan named the GAP (escape_line_hints): on the
                # planned side only that lane is the plan, so the other
                # is not tried here -- a lane the plan gave to a
                # neighbour is what the fallback searches may still
                # take, but not this one
                _tol = 0.3 * (grid.pitch_y if sx_ else grid.pitch_x)
                cands = [c for c in cands
                         if abs((c[0][1] if sx_ else c[0][0]) - line) <= _tol]
            for (ex_, ey_), (tx_, ty_) in cands:
                tag = (f"side ({sx_:+.0f},{sy_:+.0f}) jog ({ex_:.2f},{ey_:.2f}) "
                       f"run to ({tx_:.2f},{ty_:.2f})")
                hit = _seg_conflict_item(p.net_id, gx, gy, ex_, ey_, ctx)
                if hit is not None:
                    _why(tag + f": jog hits registered {hit}")
                    continue
                hit = _seg_conflict_item(p.net_id, ex_, ey_, tx_, ty_, ctx)
                if hit is not None:
                    _why(tag + f": run hits registered {hit}")
                    continue
                if not occ.seg_clear(top_idx, (gx, gy), (ex_, ey_),
                                     exempt=pad_ex):
                    _why(tag + ": jog blocked on the raster")
                    continue
                lane_ex = _lane_pad_exempt(ex_, ey_, tx_, ty_)
                if not occ.seg_clear(top_idx, (ex_, ey_), (tx_, ty_),
                                     exempt=pad_ex | lane_ex):
                    _why(tag + ": run blocked on the raster")
                    continue
                _why(tag + ": TAKEN")
                return [(gx, gy), (ex_, ey_), (tx_, ty_)]
        return None

    def _commit_lane_run(p, pts):
        # Stamp + register like commit()'s track half: the run must be
        # visible to the raster AND (via `tracks` -> _via_ctx) to every
        # later exact check, or the pairs' dive sites land on it.
        for a, b in zip(pts, pts[1:]):
            occ.block_segment(top_idx, a, b, trk_keep)
            tracks.append({'start': a, 'end': b, 'width': track_width,
                           'layer': layers[top_idx], 'net_id': p.net_id})

    _lane_on = env_knobs.FANOUT_LANE_ESCAPE
    n_lane_run = 0
    inner_pads = list(single_pads)
    if nl > 1:
        inner_pads = []
        _order = sorted(single_pads, key=depth)
        for _si, p in enumerate(_order):
            if cancel_check and cancel_check():    # #621
                # Untried balls are NOT pushed into inner_pads: they were never
                # attempted, so they must not reach the failure ledger either.
                break
            _lhA = _lhint_idx(p)
            if _lhA is not None and _lhA != top_idx:
                # #622: hinted off the top layer -- the surface phase
                # can only deliver top, so this ball goes straight to
                # the inner machinery (whose ladder ends at
                # all-layers: the hint cannot cost the escape)
                inner_pads.append(p)
                continue
            if depth(p) > outer_depth:
                # deeper than the rim phase tries -- but a straight lane
                # run may still reach the boundary (the human's escape for
                # such balls), and it keeps the gap sites free for vias
                pts = _lane_run_escape(p) if _lane_on else None
                if pts is not None:
                    _commit_lane_run(p, pts)
                    n_lane_run += 1
                    _rec_got(p, top_idx)
                else:
                    inner_pads.append(p)
                continue
            _prog(_si + 1, len(_order), f"top-layer escape {p.net_name}")
            sx, sy = occ.cell(p.global_x, p.global_y)
            home = home_of(p)
            carve = _carve_foreign([(p.global_x, p.global_y)], home, {p.net_id})
            # This is the phase that escapes the SHALLOW balls on the top
            # layer with no via, and on a shallow array it is where most of
            # the balls go -- so a plan that reaches only the dog-bone and
            # inner-ball phases reaches almost nothing. (Measured: hints wired
            # into those two alone left the bench's 21 balls bit-identical,
            # 10/21 obeyed either way, because every one of them escaped
            # here.) Side first, unrestricted retry after, so a hint can never
            # cost a ball its escape.
            _side = (escape_dir_hints or {}).get(
                (round(p.global_x, 3), round(p.global_y, 3)))
            path = None
            _line = _line_of(p, _side)
            if _side and _line:
                path = astar(sx, sy, home, {top_idx}, allow_via=False,
                             net_id=p.net_id, carve=carve, side=_side,
                             line=_line)
                if path is None:
                    ((_slot_missed if (escape_slot_hints or {}).get(
                        (round(p.global_x, 3), round(p.global_y, 3)))
                      else _line_missed).append(p.net_name))
            # The lane run on the PLANNED side comes before the side-
            # restricted raster A*, not after it: that A* accepts ANY
            # point of the planned face, so when the raster's inflated
            # pad stamps close the straight lane (which the exact check
            # proves legal) it "succeeds" with a snake down a column gap
            # to the far corner of the face -- K21 free plan: SDQ15 4.4 mm
            # and SDQ10 6 mm of snake to exit west at the bottom, eating
            # both column gaps beside column C, after which the deeper
            # balls there (SDQ14, SDQM1) had no lane at all and left by
            # another face. Straight and short first; the raster search
            # is the fallback.
            lane = None
            if path is None and _side and _lane_on:
                _sl = (escape_slot_hints or {}).get(
                    (round(p.global_x, 3), round(p.global_y, 3)))
                _lv = ((_sl[1] + _sl[2]) / 2.0
                       if _sl is not None and _sl[0] == _side else
                       (escape_line_hints or {}).get(
                           (round(p.global_x, 3), round(p.global_y, 3))))
                lane = _lane_run_escape(p, only_side=_side, line=_lv)
            if path is None and lane is None and _side:
                path = astar(sx, sy, home, {top_idx}, allow_via=False,
                             net_id=p.net_id, carve=carve, side=_side)
            if path is None and lane is None and _side:
                _hint_missed.append(p.net_name)
            if path is None and lane is None:
                path = astar(sx, sy, home, {top_idx}, allow_via=False,
                             net_id=p.net_id, carve=carve)
            if path is None and lane is None and _lane_on:
                lane = _lane_run_escape(p)
            if lane is not None:
                _commit_lane_run(p, lane)
                n_lane_run += 1
                _rec_got(p, top_idx)
            elif path is not None:
                commit(p, path, carve)
                n_fcu += 1
                _rec_got(p, path[-1][2])
            else:
                inner_pads.append(p)
        if n_lane_run:
            print(f"  Surface: {n_lane_run} lane-run escape(s) -- pad, 45-degree "
                  f"jog, straight down a gap lane to the boundary "
                  f"(KICAD_FANOUT_LANE_ESCAPE)")

    # Phase B: reserve EVERY via-in-pad keepout BEFORE routing any inner track -
    # the inner single balls AND both balls of every pair that still needs an
    # inner (via) escape. Otherwise a track running under pad P (placed before
    # P's via) and P's later via collide (via-segment short). Pairs that already
    # escaped via-less on top are NOT reserved, so deeper inner runs stay open
    # beneath them.
    def _reserve_via_site(p):
        vk = vkeep_for_pad(p)
        occ.block_all(p.global_x, p.global_y, vk)
        cs, cd = clamp_via_to_pad(via_size, via_drill, p, floors)[:2]
        reserved_sites.append((p.global_x, p.global_y, cs / 2.0, cd / 2.0,
                               p.net_id))
        carve_disks.append((p.global_x, p.global_y, vk, None, p.net_id))

    # Dog-bone site selection (#128) ------------------------------------------
    # (_pt_seg_d and _seg_conflict are defined above the surface phase, which
    # the lane-run escape shares them with.)
    def _stub_conflict(p, vx, vy, ctx):
        return _seg_conflict(p.net_id, p.global_x, p.global_y, vx, vy, ctx)

    def _choose_dogbone_site(p):
        """A free diagonal inter-ball gap site for p's dog-bone via, or None.

        Candidates are the four half-pitch diagonal gap centres, preferred
        toward the ball's nearest boundary (the escape side) and staggered by
        ring parity (successive rings alternate the perpendicular side) so
        the vias leave open inner-layer streets between via rows (#128)."""
        gx, gy = p.global_x, p.global_y
        hx, hy = grid.pitch_x / 2.0, grid.pitch_y / 2.0
        d_e, d_w = grid.max_x - gx, gx - grid.min_x
        d_s, d_n = grid.max_y - gy, gy - grid.min_y
        ex = 1.0 if d_e <= d_w else -1.0
        ey = 1.0 if d_s <= d_n else -1.0
        primary_x = min(d_e, d_w) <= min(d_s, d_n)
        # A caller's plan overrides "toward the nearest boundary". The
        # nearest boundary is the right guess for a ball considered
        # alone and the wrong one for a ball that has to join a bundle
        # leaving on a particular side -- which is the whole reason the
        # plan exists. Only the escape SIDE is taken from the hint; the
        # ring-parity stagger and the candidate ordering are untouched.
        _h = (escape_dir_hints or {}).get((round(gx, 3), round(gy, 3)))
        if _h == 'left':
            ex, primary_x = -1.0, True
        elif _h == 'right':
            ex, primary_x = 1.0, True
        elif _h == 'up':
            ey, primary_x = -1.0, False
        elif _h == 'down':
            ey, primary_x = 1.0, False
        alt = 1.0 if int(round(depth(p) / max(pitch, 1e-9))) % 2 == 0 else -1.0
        if primary_x:
            cands = [(ex * hx, alt * ey * hy), (ex * hx, -alt * ey * hy),
                     (-ex * hx, alt * ey * hy), (-ex * hx, -alt * ey * hy)]
        else:
            cands = [(alt * ex * hx, ey * hy), (-alt * ex * hx, ey * hy),
                     (alt * ex * hx, -ey * hy), (-alt * ex * hx, -ey * hy)]
        import env_knobs as _ek
        lane_max = _ek.FANOUT_LANE_WALK_MAX if _ek.FANOUT_LANE_WALK else 0
        # Window the exact-geometry scan out to the farthest lane-walked site
        # (elbow + lane_max pitches), or far sites are checked against a
        # context that cannot see their neighbours (the mez_rx via stack).
        ctx = _via_ctx(p.net_id, gx, gy,
                       extra=max(hx, hy) + lane_max * max(grid.pitch_x,
                                                          grid.pitch_y))
        pad_ex = set(occ._disk(gx, gy, max(pad_keep, via_keep)))

        def _lane_pad_exempt(x1, y1, x2, y2):
            # The inter-row lane clears the flanking ball pads by only a few
            # um past the floor (0.25 - pad_half - track/2 vs clearance) --
            # legal by the EXACT check, but the raster's inflated pad stamps
            # can eat the whole lane cell. Exempt the flanking OWN-FOOTPRINT
            # pads' disks; _seg_conflict has them exactly (registry-complete).
            cells = set()
            for p2 in footprint.pads:
                if _pt_seg_d(p2.global_x, p2.global_y, x1, y1, x2, y2)                         < max(grid.pitch_x, grid.pitch_y) * 0.55:
                    cells |= set(occ._disk(p2.global_x, p2.global_y, pad_keep))
            return cells

        def _site_ok(vx, vy, avoid_soft):
            if not (grid.min_x - 1e-6 <= vx <= grid.max_x + 1e-6
                    and grid.min_y - 1e-6 <= vy <= grid.max_y + 1e-6):
                return False
            if avoid_soft:
                six, siy = occ.cell(vx, vy)
                if any(occ.soft_foreign(L, six, siy, p.net_id)
                       for L in range(occ.nl)):
                    return False
            if locked_smd_pads and not via_site_ok(vx, vy, via_size / 2.0):
                return False
            return _via_site_conflict(vx, vy, p.net_id, ctx) is None

        # Movable-cap soft keep-outs (#278): a via under a movable cap is legal
        # (cap-opt relocates the cap, and its via gate avoids vias, #445) but a
        # clean gap is better -- prefer soft-free sites, fall back if boxed in.
        # Lane-walk (#652, human survey): when a diagonal gap is taken, the
        # stub RIDES the inter-row lane outward to the nearest FREE gap (the
        # human's dominant escape at fine pitch: gap sites are a POOL reached
        # by lane runs, tracks and vias alternating along the lane -- never a
        # track squeezed beside a via at half-pitch, which is what busts the
        # channel budget). k = gaps walked; k=0 reproduces the classic sites.
        for avoid_soft in ((True, False) if occ.has_soft else (False,)):
            for k in range(0, lane_max + 1):
                for dx, dy in cands:
                    ex_, ey_ = gx + dx, gy + dy   # the diagonal elbow (k=0 site)
                    if k == 0:
                        vx, vy = ex_, ey_
                        if not _site_ok(vx, vy, avoid_soft):
                            continue
                        if _stub_conflict(p, vx, vy, ctx):
                            continue
                        ex_cells = pad_ex | set(occ._disk(vx, vy, via_keep))
                        if not occ.seg_clear(top_idx, (gx, gy), (vx, vy),
                                             exempt=ex_cells):
                            continue
                        return (vx, vy), [(gx, gy), (vx, vy)]
                    # k >= 1: walk the horizontal lane, then the vertical one.
                    walks = [(ex_ + math.copysign(k * grid.pitch_x, dx), ey_),
                             (ex_, ey_ + math.copysign(k * grid.pitch_y, dy))]
                    for vx, vy in walks:
                        if not _site_ok(vx, vy, avoid_soft):
                            continue
                        if _seg_conflict(p.net_id, gx, gy, ex_, ey_, ctx):
                            continue
                        if _seg_conflict(p.net_id, ex_, ey_, vx, vy, ctx):
                            continue
                        via_ex = set(occ._disk(vx, vy, via_keep))
                        if not occ.seg_clear(top_idx, (gx, gy), (ex_, ey_),
                                             exempt=pad_ex | via_ex):
                            continue
                        lane_ex = _lane_pad_exempt(ex_, ey_, vx, vy)
                        if not occ.seg_clear(top_idx, (ex_, ey_), (vx, vy),
                                             exempt=pad_ex | via_ex | lane_ex):
                            continue
                        return (vx, vy), [(gx, gy), (ex_, ey_), (vx, vy)]
        return None

    def _reserve_dogbone(p, site, path=None):
        vx, vy = site
        occ.block_all(vx, vy, via_keep)
        reserved_sites.append((vx, vy, via_size / 2.0,
                               (via_drill or 0.0) / 2.0, p.net_id))
        carve_disks.append((vx, vy, via_keep, None, p.net_id))
        # Reserve the pad->via stub corridor too: it is emitted only when the
        # ball finally escapes, but a route committed in between must not
        # claim it (the cells sit outside every other ball's home disk, so
        # blocking is authoritative; the owner is exempt via home_of).
        pts = path or [(p.global_x, p.global_y), (vx, vy)]
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            occ.block_segment(top_idx, (ax, ay), (bx, by), trk_keep)
            exact_segs.append((ax, ay, bx, by,
                               track_width / 2.0, p.net_id, top_idx))
        db_site[id(p)] = site
        db_path[id(p)] = pts

    if dogbone:
        # Deepest-first so interior balls claim their toward-exit gaps before
        # shallower neighbours take them (adjacent balls share gap sites).
        db_all = list(inner_pads)
        for _base, pp, nn in remaining_pairs:
            db_all.extend((pp, nn))
        db_all.sort(key=depth, reverse=True)
        n_db = 0
        for p in db_all:
            picked = _choose_dogbone_site(p)
            if picked is not None:
                site, _dbp = picked
                _reserve_dogbone(p, site, _dbp)
                n_db += 1
            elif not no_via_in_pad:
                _reserve_via_site(p)   # classic via-in-pad fallback
            # #581 (no_via_in_pad): no centre reservation -- the classic
            # escape below may still place an OFF-pad via (gated by _via_ok),
            # else the ball fails honestly.
        if verbose:
            print(f"  Dog-bone: {n_db}/{len(db_all)} inner balls got a gap "
                  f"via site ({len(db_all) - n_db} "
                  f"{'off-pad-or-fail (#581)' if no_via_in_pad else 'via-in-pad'}"
                  f" fallback)")
    else:
        for p in inner_pads:
            _reserve_via_site(p)
        for _base, pp, nn in remaining_pairs:
            _reserve_via_site(pp)
            _reserve_via_site(nn)

    # Phase C2: escape the remaining pairs coupled on an inner layer (via-in-pad),
    # deepest-first so the interior claims the scarce central space. A pair that
    # still won't fit a coupled corridor falls back to single-ended (its vias are
    # already reserved, so it just joins the inner single balls).
    remaining_pairs.sort(key=lambda t: depth(t[1]) + depth(t[2]), reverse=True)
    inner_cands = [(L, True) for L in range(nl) if L != top_idx]
    for _ci, (base, pp, nn) in enumerate(remaining_pairs):
        if cancel_check and cancel_check():    # #621
            break
        _prog(_ci + 1, len(remaining_pairs), f"inner coupled pair {base}")
        _lhp = _lhint_idx(pp)
        if _lhp is None:
            _lhp = _lhint_idx(nn)
        _cands = inner_cands
        if _lhp is not None and _lhp != top_idx:
            _cands = ([(_lhp, True)]
                      + [c for c in inner_cands if c[0] != _lhp])
        if (try_coupled(pp, nn, _cands)
                or try_coupled_endon(pp, nn, _cands)):
            n_coupled += 1
        else:
            inner_pads.extend((pp, nn))
            coupled_fail.append(base)

    def commit_dogbone(p, site, path, carve=None):
        """Emit the dog-bone stub + gap via + single-layer run (#128). The
        stub corridor was stamped at reservation time; here its copper and
        the via are actually emitted."""
        nonlocal nvia, n_db_esc
        vx, vy = site
        home = home_of(p)
        li = path[0][2]
        cells = [(ix, iy) for (ix, iy, _L) in path]
        pts = _segments_from_cells(occ, cells)
        if pts:
            pts[0] = (vx, vy)          # start exactly at the via
        snap_exit_end(pts, li, exempt=_eff_exempt(home, carve, li),
                      net_id_for_snap=p.net_id)
        # Stamp the emitted polyline, home cells included (#505) -- see commit().
        for a, b in zip(pts, pts[1:]):
            occ.block_segment(li, a, b, trk_keep)
            tracks.append({'start': a, 'end': b, 'width': track_width,
                           'layer': layers[li], 'net_id': p.net_id})
        # Emit the pad->via stub along the RESERVED polyline (#652): a lane-
        # walked site is up to lane_max gaps away, and the straight hypotenuse
        # would cut across the ball rows the elbow+lane path was chosen to
        # clear (mez_rx: 23 stubs grazing every pad in between).
        stub_pts = db_path.get(id(p)) or [(p.global_x, p.global_y), (vx, vy)]
        for a, b in zip(stub_pts, stub_pts[1:]):
            tracks.append({'start': a, 'end': b, 'width': track_width,
                           'layer': layers[top_idx], 'net_id': p.net_id})
        occ.block_all(vx, vy, via_keep)
        vias_to_add.append({'x': vx, 'y': vy, 'size': via_size,
                            'drill': via_drill,
                            'layers': [layers[0], layers[-1]],
                            'net_id': p.net_id})
        nvia += 1
        n_db_esc += 1

    # Phase D: route the inner balls single-ended (deepest-first - the interior
    # claims the scarce central space before the shallower balls).
    n_db_esc = 0
    inner_pads.sort(key=depth, reverse=True)
    for _ii, p in enumerate(inner_pads):
        if cancel_check and cancel_check():    # #621
            break
        _prog(_ii + 1, len(inner_pads), f"inner escape {p.net_name}")
        # Dog-bone route (#128): the via already has a reserved gap site; the
        # inner run starts AT the via, so the ball-grid position stays free.
        # Any failure falls through to the classic via-in-pad A* (the classic
        # path may still place its via off-centre in the home disk).
        _site = db_site.get(id(p)) if dogbone else None
        if _site is not None:
            home = home_of(p)
            carve = _carve_foreign(db_path.get(id(p))
                                   or [(p.global_x, p.global_y), _site],
                                   home, {p.net_id})
            vsx, vsy = occ.cell(*_site)
            path = None
            if getattr(occ, 'pour_soft', None) is not None:
                # #563: layer choice was first-path-wins, so per-cell pour
                # costs could never move an escape off a flooded layer. With
                # the field active, route EVERY candidate layer and keep the
                # cheapest total (pour charge included); ties keep the lowest
                # layer index (the old deterministic order).
                best = None
                for _L in _lh_order(p, inner_layers):
                    _co = []
                    _pth = astar(vsx, vsy, home, {_L}, allow_via=False,
                                 net_id=p.net_id, carve=carve, start_layer=_L,
                                 cost_out=_co)
                    if _pth is not None and (best is None or _co[0] < best[0]):
                        best = (_co[0], _pth)
                if best is not None:
                    path = best[1]
            else:
                for _L in _lh_order(p, inner_layers):
                    path = astar(vsx, vsy, home, {_L}, allow_via=False,
                                 net_id=p.net_id, carve=carve, start_layer=_L)
                    if path is not None:
                        break
            if path is not None:
                commit_dogbone(p, _site, path, carve)
                _rec_got(p, path[-1][2])
                continue
        sx, sy = occ.cell(p.global_x, p.global_y)
        home = home_of(p)
        _anchors = [(p.global_x, p.global_y)] + ([_site] if _site else [])
        carve = _carve_foreign(_anchors, home, {p.net_id})
        _cs, _cd = clamp_via_to_pad(via_size, via_drill, p, floors)[:2]
        _ctx = _via_ctx(p.net_id, p.global_x, p.global_y)
        _memo = {}

        def _via_ok(x, y, at_center, _cs=_cs, _cd=_cd, _nid=p.net_id,
                    _c=_ctx, _m=_memo, _p=p):
            if no_via_in_pad:
                # #581: no via may overlap its own pad -- centre is out, and an
                # off-centre via must clear the pad copper (circumscribed
                # radius; exact for circular balls).
                if at_center:
                    return False
                _pr = math.hypot(_p.size_x, _p.size_y) / 2.0
                if math.hypot(x - _p.global_x, y - _p.global_y) \
                        < _pr + via_size / 2.0:
                    return False
            if locked_smd_pads and not via_site_ok(
                    x, y, (_cs if at_center else via_size) / 2.0):
                return False
            if at_center:
                # The centre's mutual-reservation contract (Phase B) only
                # covers RESERVATIONS -- #567: a neighbour's COMMITTED escape
                # run can lawfully pass within (clamped ring + clearance) of
                # this pad centre (an earlier pass or an equal-depth sort tie
                # routes it first; the SMD pad blocks only its own layer), and
                # the old blanket True dropped the via straight through that
                # copper. Check the pad-CLAMPED ring/drill against REAL copper
                # (board incl. materialized earlier-pass results, this run's
                # tracks/vias, pads) and skip only the resv list -- the
                # contract's actual domain. Vetoed centres fall to the checked
                # off-centre search or fail the ball honestly to the router.
                key = ('c', round(x, 6), round(y, 6))
                ok = _m.get(key)
                if ok is None:
                    ok = _via_site_conflict(x, y, _nid, _c, vr=_cs / 2.0,
                                            vdr=(_cd or 0.0) / 2.0,
                                            skip_resv=True) is None
                    _m[key] = ok
                return ok
            key = (round(x, 6), round(y, 6))
            ok = _m.get(key)
            if ok is None:
                ok = _via_site_conflict(x, y, _nid, _c) is None
                _m[key] = ok
            return ok

        # A plan's side is tried FIRST and is never allowed to cost a
        # ball: if no route leaves on it, the same searches run again
        # unrestricted. So hints can only change WHICH way a ball
        # escapes, never WHETHER it does.
        _side = (escape_dir_hints or {}).get(
            (round(p.global_x, 3), round(p.global_y, 3)))
        path = None
        # #622: the plan's LAYER is the contract-critical hint -- try
        # it first, jointly with the side when one is named, then
        # alone; the existing line/side/free ladder stays as the
        # fallback so the hint can never cost the ball its escape.
        _lhD = _lhint_idx(p)
        _line = _line_of(p, _side)
        # most-constrained FIRST: (layer+side+slot/line) leads, or a
        # local (layer+side) exit satisfies the ladder before the
        # slot is ever tried (measured: SA8 committed at y69.1 with
        # its slot at y59.4 -- the whole wild-escape class)
        if _lhD is not None and _side and _line:
            path = astar(sx, sy, home, {_lhD}, allow_via=True,
                         via_ok=_via_ok, net_id=p.net_id, carve=carve,
                         side=_side, line=_line)
        if _lhD is not None and path is None and _side:
            path = astar(sx, sy, home, {_lhD}, allow_via=True,
                         via_ok=_via_ok, net_id=p.net_id, carve=carve,
                         side=_side)
        if _lhD is not None and path is None:
            path = astar(sx, sy, home, {_lhD}, allow_via=True,
                         via_ok=_via_ok, net_id=p.net_id, carve=carve)
        if path is None and _side and _line:
            path = astar(sx, sy, home, inner_layers, allow_via=True,
                         via_ok=_via_ok, net_id=p.net_id, carve=carve,
                         side=_side, line=_line)
            if path is None and nl > 1:
                path = astar(sx, sy, home, set(range(nl)), allow_via=True,
                             via_ok=_via_ok, net_id=p.net_id, carve=carve,
                             side=_side, line=_line)
            if path is None:
                ((_slot_missed if (escape_slot_hints or {}).get(
                    (round(p.global_x, 3), round(p.global_y, 3)))
                  else _line_missed).append(p.net_name))
        if _side and path is None:
            path = astar(sx, sy, home, inner_layers, allow_via=True,
                         via_ok=_via_ok, net_id=p.net_id, carve=carve,
                         side=_side)
            if path is None and nl > 1:
                path = astar(sx, sy, home, set(range(nl)), allow_via=True,
                             via_ok=_via_ok, net_id=p.net_id, carve=carve,
                             side=_side)
            if path is None:
                _hint_missed.append(p.net_name)
        if path is None:
            path = astar(sx, sy, home, inner_layers, allow_via=True,
                         via_ok=_via_ok, net_id=p.net_id, carve=carve)
        if path is None and nl > 1:
            path = astar(sx, sy, home, set(range(nl)), allow_via=True,
                         via_ok=_via_ok, net_id=p.net_id, carve=carve)
        if path is None:
            failed.append(p.net_name)
            continue
        commit(p, path, carve)
        _rec_got(p, path[-1][2])

    # Plane-ball drops (#424 D2): stub + via for the skipped plane-net balls,
    # placed while the under-package space is still claimable. Runs after every
    # signal phase so signals keep first claim (the measured F-plan ordering).
    if plane_drop_nets:
        _sig_ids = {id(p) for p in signal_pads}
        drop_pads = [p for p in fp_pads
                     if p.net_id in plane_drop_nets
                     and not (p.drill and p.drill > 0)  # TH barrels already span all layers
                     and id(p) not in _sig_ids
                     and board_net_counts[p.net_id] >= 2]
        drop_pads.sort(key=depth, reverse=True)
        # Existing same-net connections (re-runs, prior taps): a via whose ring
        # overlaps the pad copper, or a track endpoint on the pad, means this
        # ball is already served -- skip it, keeping the pass idempotent.
        _net_vias: Dict[int, List[Tuple[float, float, float]]] = {}
        for v in pcb_data.vias:
            if v.net_id in plane_drop_nets:
                _net_vias.setdefault(v.net_id, []).append((v.x, v.y, v.size / 2.0))
        _net_seg_ends: Dict[int, List[Tuple[float, float]]] = {}
        for s in pcb_data.segments:
            if s.net_id in plane_drop_nets:
                _net_seg_ends.setdefault(s.net_id, []).extend(
                    [(s.start_x, s.start_y), (s.end_x, s.end_y)])
        # Surface-pour direct connect: a same-net zone on the ball's OWN copper
        # layer whose computed fill reaches the pad (and belongs to the zone's
        # main component, not an islet) connects the ball at fill time with no
        # via at all -- the dominant human pattern for rail balls under outer-
        # layer floods (corpus survey: up to 86% of a part's balls served this
        # way). The fill models must see THIS call's fresh copper (materialized
        # into pcb_data by the drop pass), so drop any stale per-board cache.
        _pour_models: Dict[Tuple[int, str], list] = {}
        # DECLARED future pours (--plane-net-layers): for a
        # declared (net, layer) with the layer on this BALL side, build a
        # SYNTHETIC board-rect zone and fill-model it against the copper that
        # exists NOW -- a predictor of whether the future pour will reach each
        # ball. Balls the predicted fill reaches skip their drop via entirely
        # (the fanout-time form of pour-direct; plane repair backstops misses).
        if plane_net_layers:
            try:
                from kicad_parser import Zone as _Zone
                from plane_fill_model import get_zone_model as _gzm
                _bb = pcb_data.board_info.board_bounds
                _name2id = {n.name: n.net_id for n in pcb_data.nets.values()}
                for _nname, _lays in plane_net_layers.items():
                    _nid = _name2id.get(_nname)
                    if _nid is None or not _bb:
                        continue
                    for _lay in _lays:
                        _z = _Zone(net_id=_nid, net_name=_nname, layer=_lay,
                                   polygon=[(_bb[0], _bb[1]), (_bb[2], _bb[1]),
                                            (_bb[2], _bb[3]), (_bb[0], _bb[3])])
                        _m = _gzm(pcb_data, _z)
                        if _m is not None:
                            _pour_models.setdefault((_nid, _lay), []).append(_m)
                if verbose and _pour_models:
                    print(f"  Declared-pour prediction: {len(_pour_models)} "
                          f"(net,layer) synthetic fill model(s)")
            except Exception as _e:
                print(f"  Declared-pour prediction unavailable ({_e})")
        import env_knobs
        if env_knobs.FANOUT_POUR_DIRECT and \
                any((z.net_id in plane_drop_nets) for z in (pcb_data.zones or [])):
            try:
                delattr(pcb_data, '_plane_fill_models')
            except AttributeError:
                pass
            from plane_fill_model import get_fill_models
            for nid in {p.net_id for p in drop_pads}:
                for lay, models in get_fill_models(pcb_data, nid).items():
                    _pour_models[(nid, lay)] = models

        def _pour_covers(p):
            lay = next((l for l in (p.layers or []) if l.endswith('.Cu')), None)
            if lay is None:
                return False
            for m in _pour_models.get((p.net_id, lay), ()):
                comp = m.query_component(p.global_x, p.global_y,
                                         size=min(p.size_x, p.size_y))
                if comp and comp == m.largest_component():
                    return True
            return False

        _rep_nets: Dict[str, Dict[str, int]] = {}
        n_gap = n_ctr = n_exist = n_fail = n_pour = 0
        n_trk = [0]
        n_ptrk = [0]
        for p in drop_pads:
            r = _rep_nets.setdefault(p.net_name,
                                     {'gap': 0, 'in_pad': 0, 'existing': 0,
                                      'pour': 0, 'failed': 0})
            gx, gy = p.global_x, p.global_y
            pad_half = max(p.size_x, p.size_y) / 2.0
            if any(math.hypot(vx - gx, vy - gy) < pad_half + vr_ + 1e-6
                   for (vx, vy, vr_) in _net_vias.get(p.net_id, ())) or \
               any(abs(ex - gx) < 0.05 and abs(ey - gy) < 0.05
                   for (ex, ey) in _net_seg_ends.get(p.net_id, ())):
                n_exist += 1
                r['existing'] += 1
                continue
            if _pour_covers(p):
                # Served by the surface pour on its own layer -- no via needed.
                # (The pad-centre tap reservation stays: if a later step's
                # copper carves the pour away from this pad, the plane repair
                # can still tap here.)
                n_pour += 1
                r['pour'] += 1
                continue
            # POUR-TRACK near-miss (#652): the pour's MAIN component stops
            # just short of this ball (fill eroded around a neighbouring
            # obstacle). A short track on the ball's own layer into the fill
            # connects it with NO via -- the other half of the human dog-bone
            # economy. KICAD_FANOUT_POUR_TRACK=0 reverts.
            if env_knobs.FANOUT_POUR_TRACK and _pour_models:
                _lay = next((l for l in (p.layers or [])
                             if l.endswith('.Cu')), None)
                _li = layers.index(_lay) if _lay in layers else None
                _hit = None
                if _li is not None:
                    for _m in _pour_models.get((p.net_id, _lay), ()):
                        _c = _m.largest_component()
                        if not _c:
                            continue
                        _pt = _m.nearest_component_point(
                            gx, gy, _c, env_knobs.FANOUT_POUR_TRACK_R)
                        if _pt is not None and (_hit is None
                                                or _pt[2] < _hit[2]):
                            _hit = _pt
                if _hit is not None:
                    _tx, _ty, _d = _hit
                    # Overshoot one track-width INTO the fill so the contact
                    # is solid copper, not a cell-edge graze.
                    if _d > 1e-6:
                        _ux, _uy = (_tx - gx) / _d, (_ty - gy) / _d
                        _tx += _ux * track_width
                        _ty += _uy * track_width
                    _n = max(2, int((_d + track_width) / max(occ.res, 1e-3)))
                    _free = True
                    for _t in range(_n + 1):
                        _sx = gx + (_tx - gx) * _t / _n
                        _sy = gy + (_ty - gy) * _t / _n
                        if math.hypot(_sx - gx, _sy - gy) < pad_half:
                            continue
                        _ix, _iy = occ.cell(_sx, _sy)
                        if not occ.inside(_ix, _iy) or \
                                occ.blocked(_li, _ix, _iy):
                            _free = False
                            break
                    if _free:
                        tracks.append({'start': (gx, gy), 'end': (_tx, _ty),
                                       'width': track_width, 'layer': _lay,
                                       'net_id': p.net_id})
                        occ.block_segment(_li, (gx, gy), (_tx, _ty),
                                          track_width / 2.0 + clearance)
                        _net_seg_ends.setdefault(p.net_id, []).extend(
                            [(gx, gy), (_tx, _ty)])
                        n_ptrk[0] += 1
                        r['pour_track'] = r.get('pour_track', 0) + 1
                        continue
            # TRACK-CONNECT (env-gated, KICAD_FANOUT_TRACK_CONNECT=1 arms): a through-barrel here
            # perforates EVERY foreign pour whose fill covers this (x,y) --
            # the mechanism that shreds middle-layer rail planes. When it
            # would, and an already-placed same-net drop via sits within
            # reach on this ball's own layer, connect by a short surface
            # TRACK to that via instead (humans: ~0.44 vias/ball -- balls
            # chain to shared barrels). KICAD_FANOUT_TRACK_CONNECT=1 arms.
            import os as _os
            if _os.environ.get('KICAD_FANOUT_TRACK_CONNECT', '') == '1':
                from plane_fill_model import get_fill_models as _gfm
                _gx, _gy = p.global_x, p.global_y
                _pierced = 0
                for _z in (pcb_data.zones or []):
                    if _z.net_id == p.net_id:
                        continue
                    _ms = _gfm(pcb_data, _z.net_id).get(_z.layer, [])
                    if any((_m.query_component(_gx, _gy) or 0) > 0 for _m in _ms):
                        _pierced += 1
                if _pierced >= 1:
                    _lay_i = top_idx
                    _r_mm = float(_os.environ.get('KICAD_FANOUT_TRACK_CONNECT_R', '2.0') or 2.0)
                    _tw2 = track_width / 2.0 + clearance
                    _best = None
                    for (_vx, _vy, _vr) in _net_vias.get(p.net_id, ()):
                        _d = math.hypot(_vx - _gx, _vy - _gy)
                        if _d <= _r_mm and (_best is None or _d < _best[2]):
                            _best = (_vx, _vy, _d)
                    if _best is not None:
                        _vx, _vy, _d = _best
                        _n = max(2, int(_d / max(occ.res, 1e-3)))
                        _free = True
                        for _t in range(_n + 1):
                            _sx = _gx + (_vx - _gx) * _t / _n
                            _sy = _gy + (_vy - _gy) * _t / _n
                            if math.hypot(_sx - _gx, _sy - _gy) < max(p.size_x, p.size_y) / 2.0:
                                continue
                            if math.hypot(_sx - _vx, _sy - _vy) < _vr:
                                continue
                            _ix, _iy = occ.cell(_sx, _sy)
                            if not occ.inside(_ix, _iy) or occ.blocked(_lay_i, _ix, _iy):
                                _free = False
                                break
                        if _free:
                            tracks.append({'start': (_gx, _gy), 'end': (_vx, _vy),
                                           'width': track_width,
                                           'layer': layers[top_idx],
                                           'net_id': p.net_id})
                            occ.block_segment(_lay_i, (_gx, _gy), (_vx, _vy), _tw2)
                            _net_seg_ends.setdefault(p.net_id, []).extend(
                                [(_gx, _gy), (_vx, _vy)])
                            r['track'] = r.get('track', 0) + 1
                            n_trk[0] += 1
                            continue
            # This ball's centre was reserved up front as its future plane-tap
            # site; the drop replaces that tap, so the reservation must not
            # veto its own via (drill-vs-drill at distance 0) or the gap site
            # at fine pitch. Void it; restore only if the drop fails outright.
            _own = [(rx, ry, rr, rd, rn) for (rx, ry, rr, rd, rn) in reserved_sites
                    if rn == p.net_id and abs(rx - gx) < 1e-6 and abs(ry - gy) < 1e-6]
            for t in _own:
                reserved_sites.remove(t)
            picked = _choose_dogbone_site(p)
            if picked is not None:
                site, _dbp = picked
                _reserve_dogbone(p, site, _dbp)
                vx, vy = site
                for (_ax, _ay), (_bx, _by) in zip(_dbp, _dbp[1:]):
                    tracks.append({'start': (_ax, _ay), 'end': (_bx, _by),
                                   'width': track_width,
                                   'layer': layers[top_idx],
                                   'net_id': p.net_id})
                vias_to_add.append({'x': vx, 'y': vy, 'size': via_size,
                                    'drill': via_drill,
                                    'layers': [layers[0], layers[-1]],
                                    'net_id': p.net_id})
                _net_vias.setdefault(p.net_id, []).append((vx, vy, via_size / 2.0))
                n_gap += 1
                r['gap'] += 1
                continue
            # Centre fallback: via-in-pad tap at the ball, pad-clamped (#202).
            # Exact-checked at the CLAMPED size: after a channel-engine signal
            # pass, inner-layer runs may legally cross this centre, so the
            # underpad centre-reservation contract cannot be assumed here.
            cs, cd, ckeep = via_for_pad(p)
            _ctx = _via_ctx(p.net_id, gx, gy)
            # #581 (no_via_in_pad): the centre fallback IS a via-in-pad --
            # fail the drop instead (the ball is left for the plane tap,
            # which honors the same constraint through its via map).
            ok = not no_via_in_pad
            ok = ok and not (locked_smd_pads and not via_site_ok(gx, gy, cs / 2.0))
            if ok and _via_site_conflict(gx, gy, p.net_id, _ctx,
                                         vr=cs / 2.0, vdr=cd / 2.0) is not None:
                ok = False
            if not ok:
                reserved_sites.extend(_own)   # plane step may still tap here
                n_fail += 1
                r['failed'] += 1
                continue
            occ.block_all(gx, gy, ckeep)
            exact_vias.append((gx, gy, cs / 2.0, cd / 2.0, p.net_id))
            vias_to_add.append({'x': gx, 'y': gy, 'size': cs, 'drill': cd,
                                'layers': [layers[0], layers[-1]],
                                'net_id': p.net_id})
            _net_vias.setdefault(p.net_id, []).append((gx, gy, cs / 2.0))
            n_ctr += 1
            r['in_pad'] += 1
        if plane_drop_report is not None:
            plane_drop_report.update(
                {'nets': _rep_nets, 'gap_vias': n_gap, 'pad_vias': n_ctr,
                 'skipped_existing': n_exist, 'skipped_pour': n_pour,
                 'track_connects': n_trk[0],
                 'pour_tracks': n_ptrk[0], 'failed': n_fail})
        if verbose and drop_pads:
            per = ", ".join(f"{n} {c['gap']}+{c['in_pad']}"
                            for n, c in sorted(_rep_nets.items()))
            extra = f", {n_exist} already connected" if n_exist else ""
            extra += (f", {n_pour} pour-covered (no via needed)"
                      if n_pour else "")
            extra += (f", {n_trk[0]} track-connected (shared barrel)"
                      if n_trk[0] else "")
            extra += (f", {n_ptrk[0]} pour-tracked (near-miss, no via)"
                      if n_ptrk[0] else "")
            extra += f", {n_fail} FAILED (left for the plane tap)" if n_fail else ""
            print(f"  Plane drops (#424): {n_gap} gap + {n_ctr} in-pad vias "
                  f"[{per}]{extra}")

    if verbose:
        already = f", {len(fanned_net_ids)} already-fanned skipped" if fanned_net_ids else ""
        pairs_msg = ""
        if coupled_targets or coupled_fail:
            total_pairs = n_coupled + len(coupled_fail)
            pairs_msg = f", {n_coupled}/{total_pairs} diff pairs coupled"
            if coupled_fail:
                pairs_msg += f" ({len(coupled_fail)} fell back single-ended)"
        engine = "Dog-bone" if dogbone else "Under-pad"
        print(f"  {engine} escape: {len(signal_pads)-len(failed)}/{len(signal_pads)} "
              f"signals escaped, {nvia} vias, {len(plane_pads)} plane balls skipped"
              f"{already}{pairs_msg}, {len(failed)} failed")
        if dogbone:
            print(f"  Dog-bone: {n_db_esc} single-ended run(s) started at a gap "
                  f"via ({len(db_site)} gap sites assigned)")
        if clamp_stats['clamped']:
            print(f"  Under-pad: clamped {clamp_stats['clamped']} via-in-pad(s) to "
                  f"fit their pad edge (#202)")
        if clamp_stats['escalated']:
            warn_fab_escalation(f"under-pad {clamp_stats['escalated']} via-in-pad(s) "
                                f"(sub-0.45mm pads)")
        if clamp_stats['floor']:
            print(f"  Under-pad: WARNING {clamp_stats['floor']} pad(s) smaller than "
                  f"the fab via floor ({fab_floor_min(_copper)['via_diameter']:.2f}mm "
                  f"dia); via held at the floor and still bulges past the pad edge")
    # The FAB requirement under-pad escape creates (#489 §8): via-in-pad needs
    # IPC-4761 Type VII. Emitted from the shared engine so both fronts report it.
    from fab_notes import print_via_in_pad_note
    print_via_in_pad_note(vias_to_add, pcb_data.pads_by_net,
                          context="BGA under-pad escape")
    # #622: publish per-pad layer-hint obedience. Entries only for
    # audited pads; a hinted pad with no _got_layer entry either
    # failed (recorded as such) or escaped through an unaudited path
    # (coupled pairs) and is left for the wrapper to count as
    # unaudited rather than guessed.
    if escape_layer_hints:
        import bga_fanout as _bf
        _eng = 'dogbone' if dogbone else 'underpad'
        _fs = set(failed)
        for _k, (_got, _nm) in _got_layer.items():
            _h = escape_layer_hints.get(_k)
            if _h is None:
                continue
            _bf.LAST_LAYER_HINT_REPORT[_k] = {
                'net': _nm, 'hint': _h, 'got': _got,
                'obeyed': _got == _h, 'engine': _eng, 'failed': False}
        _key_net = {(round(_p.global_x, 3), round(_p.global_y, 3)):
                    _p.net_name for _p in footprint.pads}
        for _k, _h in escape_layer_hints.items():
            _nm = _key_net.get(_k)
            if _nm in _fs and _k not in _got_layer:
                _bf.LAST_LAYER_HINT_REPORT[_k] = {
                    'net': _nm, 'hint': _h, 'got': None,
                    'obeyed': False, 'engine': _eng, 'failed': True}

    if _slot_missed and verbose:
        print(f"  Under-pad: {len(_slot_missed)} ball(s) could not exit "
              f"inside the SLOT asked for and took another gap "
              f"[{', '.join(sorted(_slot_missed)[:6])}"
              + (f", +{len(_slot_missed) - 6} more"
                 if len(_slot_missed) > 6 else "") + "]")
    if _line_missed and verbose:
        print(f"  Under-pad: {len(_line_missed)} ball(s) could not reach "
              f"the exit LINE asked for and took another gap on the same "
              f"side [{', '.join(sorted(_line_missed)[:6])}"
              + (f", +{len(_line_missed) - 6} more"
                 if len(_line_missed) > 6 else "") + "]")
    if _hint_missed and verbose:
        print(f"  Under-pad: {len(_hint_missed)} ball(s) had no route out "
              f"the side the caller asked for and escaped elsewhere "
              f"[{', '.join(sorted(_hint_missed)[:6])}"
              + (f", +{len(_hint_missed) - 6} more" if len(_hint_missed) > 6
                 else "") + "]")
    return tracks, vias_to_add, failed
