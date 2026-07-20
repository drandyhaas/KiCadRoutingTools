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

    def cell(self, x, y):
        return (int((x - self.x0) / self.res), int((y - self.y0) / self.res))

    def xy(self, ix, iy):
        return (self.x0 + ix * self.res, self.y0 + iy * self.res)

    def inside(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def _disk(self, x, y, r):
        """Cells whose lattice point lies within r of the REAL point (x, y).

        Membership is tested against the true coordinate, not the truncated
        cell index: a via at y=70.999999 truncates a full cell low, shifting
        its keepout disk by up to res and letting a route one cell past the
        shifted disk graze the real via by up to res (issue #278).
        """
        fx = (x - self.x0) / self.res
        fy = (y - self.y0) / self.res
        cx, cy = int(fx), int(fy)
        rc = int(r / self.res) + 2
        thr = (r / self.res) ** 2
        for dx in range(-rc, rc + 1):
            for dy in range(-rc, rc + 1):
                a, b = cx + dx, cy + dy
                if (a - fx) ** 2 + (b - fy) ** 2 <= thr:
                    if 0 <= a < self.nx and 0 <= b < self.ny:
                        yield a, b

    def block_layer(self, li, x, y, r):
        g = self.grid[li]
        ny = self.ny
        for a, b in self._disk(x, y, r):
            g[a * ny + b] = 1

    def block_all(self, x, y, r):
        for a, b in self._disk(x, y, r):
            for li in range(self.nl):
                self.grid[li][a * self.ny + b] = 1

    def block_segment(self, li, p, q, r):
        (x0, y0), (x1, y1) = p, q
        n = int(max(abs(x1 - x0), abs(y1 - y0)) / self.res) + 1
        for i in range(n + 1):
            t = i / n
            self.block_layer(li, x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, r)

    def block_poly(self, polys, r, li=None):
        """Keep-out for a custom pad's REAL polygon copper: block every cell
        whose lattice point lies within `r` mm of any polygon (li=None => all
        copper layers). The accurate analogue of block_all/block_layer's bbox
        disk -- mirrors the main obstacle map's polygon rasteriser so a
        meander-antenna / comb pad stops phantom-blocking escapes far from any
        real copper (#232 class, BGA side)."""
        from check_drc import _point_to_polys_distance
        pts = [pt for poly in polys for pt in poly]
        if not pts:
            return
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ia, ja = self.cell(min(xs) - r, min(ys) - r)
        ib, jb = self.cell(max(xs) + r, max(ys) + r)
        thr = r + 1e-9
        for a in range(max(0, ia), min(self.nx, ib + 1)):
            for b in range(max(0, ja), min(self.ny, jb + 1)):
                x, y = self.xy(a, b)
                if _point_to_polys_distance(x, y, polys) <= thr:
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
                             outer_rings: float = 2.0,
                             keepout_margin: float = 0.0,
                             verbose: bool = True,
                             grid_step: float = 0.0,
                             only_pad_keys: Optional[Set[Tuple[float, float]]] = None
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

    def is_plane(p):
        # A high-pin-count net is only treated as a plane (skip its fanout, keep
        # its via as an all-layer obstacle) when the caller ALSO excluded it from
        # the fanout - that exclusion (e.g. --nets '!GND' '!+3V3') is what marks a
        # net as a real plane at fanout time, before any zone exists. A dense power
        # rail the caller still wants fanned (e.g. +1V0/+1V8) must have its
        # inner-core balls escaped, not silently skipped as a phantom plane
        # (issue #218). With no net filter, fall back to the pad-count heuristic.
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

    # Region: BGA bbox plus margin so routes can exit past the boundary.
    # Grid resolution defaults to ~1/32 of the pitch - fine enough to place the
    # single-track channels and F.Cu corridors cleanly (coarser leaves a few
    # sub-clearance corridor clips), scaled to the array so big/fine boards stay
    # fast.
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

    def _via_ctx(net_id, cx, cy):
        """Gather the geometry near (cx, cy) that an off-centre via must clear
        (one scan per ball; the per-cell check then touches only these)."""
        W = home_r + via_size + 2.0

        def close(x, y):
            return abs(x - cx) <= W and abs(y - cy) <= W

        vias = [t for t in exact_vias if close(t[0], t[1])]
        vias += [(v['x'], v['y'], v['size'] / 2.0, (v['drill'] or 0.0) / 2.0,
                  v['net_id']) for v in vias_to_add if close(v['x'], v['y'])]
        resv = [t for t in reserved_sites if t[4] != net_id and close(t[0], t[1])]
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

    def _via_site_conflict(x, y, net_id, ctx):
        """Reason a full-size OFF-CENTRE via cannot sit at (x, y), or None.

        Mirrors the channel engine's via_in_pad_conflict (#370 B4) but also
        against THIS run's committed tracks/vias and the reserved (pending)
        via-in-pad sites: an off-centre via escapes the mutual centre-
        reservation contract, so its ring and drill must prove themselves
        against exact geometry (#393). Drill hole-to-hole is net-INDEPENDENT
        (same-net drill overlap is still a fab violation, #282); copper checks
        skip same-net items.
        """
        vr = via_size / 2.0
        vdr = (via_drill or 0.0) / 2.0
        for (ox, oy, orr, odr, onid) in ctx['vias']:
            d = math.hypot(ox - x, oy - y)
            if d < vdr + odr + HOLE_TO_HOLE_CLEARANCE - 1e-6:
                return "drill hole-to-hole vs a via"
            if onid != net_id and d < vr + orr + clearance - 1e-6:
                return "via ring vs a foreign via"
        for (ox, oy, orr, odr, onid) in ctx['resv']:   # foreign-only by build
            d = math.hypot(ox - x, oy - y)
            if d < vdr + odr + HOLE_TO_HOLE_CLEARANCE - 1e-6:
                return "drill hole-to-hole vs a reserved via-in-pad"
            if d < vr + orr + clearance - 1e-6:
                return "via ring vs a reserved via-in-pad"
        for (px, py, half, pdr, pnid, has_cu) in ctx['pads']:
            d = math.hypot(px - x, py - y)
            if pdr and d < vdr + pdr + HOLE_TO_HOLE_CLEARANCE - 1e-6:
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

    def depth(p):
        return min(p.global_x - grid.min_x, grid.max_x - p.global_x,
                   p.global_y - grid.min_y, grid.max_y - p.global_y)

    def astar(sx, sy, home, route_layers, allow_via, via_ok=None, net_id=0,
              carve=None):
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

        start = (sx, sy, top_idx)
        g = {start: 0.0}
        pq = [(heur(sx, sy), start)]
        came = {}
        while pq:
            _, cur = heapq.heappop(pq)
            cx, cy, L = cur
            if not in_bbox(cx, cy):
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                path.reverse()
                return path
            cg = g[cur]
            if L in route_layers:
                for dx, dy in _STEPS:
                    nx, ny = cx + dx, cy + dy
                    if not occ.inside(nx, ny):
                        continue
                    if in_bbox(nx, ny) and occ.blocked(L, nx, ny) \
                            and not exempt(L, (nx, ny)):
                        continue
                    # No corner-cutting: a diagonal step past a blocked orthogonal
                    # neighbour clips that obstacle's clearance (the diagonal line
                    # passes nearer the via/pad than either cell). Forbid it.
                    if dx != 0 and dy != 0:
                        if (occ.blocked(L, cx + dx, cy) and not exempt(L, (cx + dx, cy))) or \
                           (occ.blocked(L, cx, cy + dy) and not exempt(L, (cx, cy + dy))):
                            continue
                    step = 1.0 if (dx == 0 or dy == 0) else 1.6   # discourage zig-zag
                    # Soft keep-out (#278): stepping through a movable
                    # passive's pad zone is legal but leaves a graze the
                    # cap-placement step may be unable to clear -- pay a
                    # steep per-cell premium so routes detour when any
                    # detour exists, and only graze when boxed in.
                    if occ.has_soft and (nx, ny) not in home and \
                            occ.soft_foreign(L, nx, ny, net_id):
                        step += 4.0
                    nxt = (nx, ny, L)
                    ng = cg + step
                    if ng < g.get(nxt, 1e18):
                        g[nxt] = ng
                        came[nxt] = cur
                        heapq.heappush(pq, (ng + heur(nx, ny), nxt))
            # The single via, only in the ball's own pad. A through via spans
            # all layers, so the site must also clear immovable foreign copper
            # on layers the run-blocking test never looks at (via_ok, #253/
            # #254). commit() emits the pad-CLAMPED size only at the exact pad
            # centre cell (sx, sy); any other home cell gets the full via_size
            # -- the gate must use the size that will actually be emitted.
            _at_center = (cx, cy) == (sx, sy)
            if allow_via and (cx, cy) in home and \
                    (via_ok is None or via_ok(*occ.xy(cx, cy), _at_center)):
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
                        heapq.heappush(pq, (ng + heur(cx, cy), nxt))
        return None

    # Layer policy: near-edge balls escape on the BGA's own layer (no via),
    # keeping the rim clear of vias so the deeper balls can always run UNDER them
    # on the other layers. Deeper balls drop a via in their pad and route there.
    nl = occ.nl
    inner_layers = set(i for i in range(nl) if i != top_idx) or {top_idx}
    pitch = (grid.pitch_x + grid.pitch_y) / 2
    outer_depth = outer_rings * pitch

    signal_pads.sort(key=depth, reverse=True)

    tracks: List[Dict] = []
    vias_to_add: List[Dict] = []
    failed: List[str] = []
    nvia = 0
    n_fcu = 0

    def home_of(p):
        # Exempt the ball's OWN pad+via keepout so the track can break out of its
        # own pad (the keepout ring is wider than the pad and would otherwise
        # trap a route). max(pad,via)_keep < pitch, so it never reaches a
        # neighbouring pad. Same real-coordinate disk the blocking stamped, so
        # the exemption covers it exactly.
        return set(occ._disk(p.global_x, p.global_y, max(pad_keep, via_keep)))

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
        for ri, (li, cells) in enumerate(runs):
            for (ix, iy) in cells:
                if (ix, iy) not in home:
                    occ.block_layer(li, *occ.xy(ix, iy), trk_keep)
            pts = runs_pts[ri]
            if ri == len(runs) - 1 and len(pts) >= 2:
                # the snapped tail may extend past the A* cells -- stamp it
                occ.block_segment(li, pts[-2], pts[-1], trk_keep)
            for a, b in zip(pts, pts[1:]):
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
                        via_site_ok(q.global_x, q.global_y,
                                    clamp_via_to_pad(via_size, via_drill, q, floors)[0] / 2.0)
                        for q in (pp, nn)):
                    continue  # a through via-in-pad would hit locked copper
                segs = []
                ok = True
                for pad, side in ((pp, 1.0), (nn, -1.0)):
                    p0 = (pad.global_x, pad.global_y)
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
                        v_size, v_drill, vkeep = via_for_pad(pad)
                        occ.block_all(pad.global_x, pad.global_y, vkeep)
                        vias_to_add.append({'x': pad.global_x, 'y': pad.global_y,
                                            'size': v_size, 'drill': v_drill,
                                            'layers': [layers[0], layers[-1]],
                                            'net_id': pad.net_id})
                        nvia += 1
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
                        via_site_ok(q.global_x, q.global_y,
                                    clamp_via_to_pad(via_size, via_drill, q, floors)[0] / 2.0)
                        for q in (pp, nn)):
                    continue  # a through via-in-pad would hit locked copper
                for tside in (1.0, -1.0):         # which gap the trail bulges through
                    lside = -tside
                    lead_segs = [((lead.global_x, lead.global_y), pt(De - run, lside * half_sp)),
                                 (pt(De - run, lside * half_sp), pt(De, lside * half_sp))]
                    W = pt(half_axis, tside * half_axis)     # gap beside the lead pad
                    trail_segs = [((trail.global_x, trail.global_y), W),
                                  (W, pt(De - run, tside * half_sp)),
                                  (pt(De - run, tside * half_sp), pt(De, tside * half_sp))]
                    exl = _eff_exempt(lh, lcv, L)
                    ext = _eff_exempt(th, tcv, L)
                    if not (all(occ.seg_clear(L, a, b, exempt=exl) for a, b in lead_segs)
                            and all(occ.seg_clear(L, a, b, exempt=ext) for a, b in trail_segs)):
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
                            v_size, v_drill, vkeep = via_for_pad(pad)
                            occ.block_all(pad.global_x, pad.global_y, vkeep)
                            vias_to_add.append({'x': pad.global_x, 'y': pad.global_y,
                                                'size': v_size, 'drill': v_drill,
                                                'layers': [layers[0], layers[-1]],
                                                'net_id': pad.net_id})
                            nvia += 1
                    escaped.update((id(pp), id(nn)))
                    return True
        return False

    # Phase A: route the near-edge SINGLE balls on the BGA layer (no via).
    # Whatever can't take a clean top-layer escape falls through to the inner
    # phase. (Paired balls are handled coupled below.)
    inner_pads = list(single_pads)
    if nl > 1:
        inner_pads = []
        for p in single_pads:
            if depth(p) > outer_depth:
                inner_pads.append(p)
                continue
            sx, sy = occ.cell(p.global_x, p.global_y)
            home = home_of(p)
            carve = _carve_foreign([(p.global_x, p.global_y)], home, {p.net_id})
            path = astar(sx, sy, home, {top_idx}, allow_via=False,
                         net_id=p.net_id, carve=carve)
            if path is not None:
                commit(p, path, carve)
                n_fcu += 1
            else:
                inner_pads.append(p)

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
    for base, pp, nn in coupled_targets:
        if (try_coupled(pp, nn, [(top_idx, False)])
                or try_coupled_endon(pp, nn, [(top_idx, False)])):
            n_coupled += 1
            escaped.update((id(pp), id(nn)))
        else:
            remaining_pairs.append((base, pp, nn))  # try an inner coupled escape

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
    for base, pp, nn in remaining_pairs:
        if (try_coupled(pp, nn, inner_cands)
                or try_coupled_endon(pp, nn, inner_cands)):
            n_coupled += 1
        else:
            inner_pads.extend((pp, nn))
            coupled_fail.append(base)

    # Phase D: route the inner balls single-ended (deepest-first - the interior
    # claims the scarce central space before the shallower balls).
    inner_pads.sort(key=depth, reverse=True)
    for p in inner_pads:
        sx, sy = occ.cell(p.global_x, p.global_y)
        home = home_of(p)
        carve = _carve_foreign([(p.global_x, p.global_y)], home, {p.net_id})
        _cs = clamp_via_to_pad(via_size, via_drill, p, floors)[0]
        _ctx = _via_ctx(p.net_id, p.global_x, p.global_y)
        _memo = {}

        def _via_ok(x, y, at_center, _cs=_cs, _nid=p.net_id, _c=_ctx, _m=_memo):
            if locked_smd_pads and not via_site_ok(
                    x, y, (_cs if at_center else via_size) / 2.0):
                return False
            if at_center:
                # centre via-in-pads keep their mutual-reservation contract
                # (Phase B keep-outs) -- clamped inside the pad by commit().
                return True
            key = (round(x, 6), round(y, 6))
            ok = _m.get(key)
            if ok is None:
                ok = _via_site_conflict(x, y, _nid, _c) is None
                _m[key] = ok
            return ok

        path = astar(sx, sy, home, inner_layers, allow_via=True,
                     via_ok=_via_ok, net_id=p.net_id, carve=carve)
        if path is None and nl > 1:
            path = astar(sx, sy, home, set(range(nl)), allow_via=True,
                         via_ok=_via_ok, net_id=p.net_id, carve=carve)
        if path is None:
            failed.append(p.net_name)
            continue
        commit(p, path, carve)

    if verbose:
        already = f", {len(fanned_net_ids)} already-fanned skipped" if fanned_net_ids else ""
        pairs_msg = ""
        if coupled_targets or coupled_fail:
            total_pairs = n_coupled + len(coupled_fail)
            pairs_msg = f", {n_coupled}/{total_pairs} diff pairs coupled"
            if coupled_fail:
                pairs_msg += f" ({len(coupled_fail)} fell back single-ended)"
        print(f"  Under-pad escape: {len(signal_pads)-len(failed)}/{len(signal_pads)} "
              f"signals escaped, {nvia} vias, {len(plane_pads)} plane balls skipped"
              f"{already}{pairs_msg}, {len(failed)} failed")
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
    return tracks, vias_to_add, failed
