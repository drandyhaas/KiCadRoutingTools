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
already-placed track is in the way. Power/plane balls are NOT fanned - they tap
their plane through a via (an obstacle here, not an escape). Routing the deepest
balls first (inside-out) lets the interior claim the scarce central space before
the outer balls, which have the open rim, consume it.

A prototype of this model escapes all 210 ulx3s signals on its 4 layers with
plain via-in-pad (no dog-bone). This module is the production router: a small
per-ball grid search (straight-first) over a layered occupancy grid built from
the board's real pads, vias and copper.
"""

import heapq
import math
from collections import Counter
from typing import Dict, List, Optional, Tuple

from kicad_parser import Footprint, PCBData
from bga_fanout.types import BGAGrid


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

    def cell(self, x, y):
        return (int((x - self.x0) / self.res), int((y - self.y0) / self.res))

    def xy(self, ix, iy):
        return (self.x0 + ix * self.res, self.y0 + iy * self.res)

    def inside(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def _disk(self, cx, cy, r):
        rc = int(r / self.res) + 1
        thr = (r / self.res) ** 2
        for dx in range(-rc, rc + 1):
            for dy in range(-rc, rc + 1):
                if dx * dx + dy * dy <= thr:
                    a, b = cx + dx, cy + dy
                    if 0 <= a < self.nx and 0 <= b < self.ny:
                        yield a, b

    def block_layer(self, li, x, y, r):
        cx, cy = self.cell(x, y)
        g = self.grid[li]
        ny = self.ny
        for a, b in self._disk(cx, cy, r):
            g[a * ny + b] = 1

    def block_all(self, x, y, r):
        cx, cy = self.cell(x, y)
        for a, b in self._disk(cx, cy, r):
            for li in range(self.nl):
                self.grid[li][a * self.ny + b] = 1

    def block_segment(self, li, p, q, r):
        (x0, y0), (x1, y1) = p, q
        n = int(max(abs(x1 - x0), abs(y1 - y0)) / self.res) + 1
        for i in range(n + 1):
            t = i / n
            self.block_layer(li, x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, r)

    def blocked(self, li, ix, iy):
        return self.grid[li][ix * self.ny + iy]


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
                             res: float = None,
                             via_cost: float = 14.0,
                             outer_rings: float = 2.0,
                             keepout_margin: float = 0.0,
                             verbose: bool = True
                             ) -> Tuple[List[Dict], List[Dict], List[str]]:
    """Route BGA signal balls to the boundary under the pad field.

    Returns (tracks, vias_to_add, failed_net_names). Power/plane balls (>=
    plane_min_pads in the footprint) are skipped - they tap their plane and act
    only as via obstacles.
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
        return fp_net_counts[p.net_name] >= plane_min_pads

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
    signal_pads = [p for p in fp_pads
                   if not is_plane(p) and not is_nc(p)
                   and p.net_id not in fanned_net_ids
                   and (net_filter_fn is None or net_filter_fn(p.net_name))]

    # Region: BGA bbox plus margin so routes can exit past the boundary.
    # Grid resolution defaults to ~1/32 of the pitch - fine enough to place the
    # single-track channels and F.Cu corridors cleanly (coarser leaves a few
    # sub-clearance corridor clips), scaled to the array so big/fine boards stay
    # fast.
    if res is None:
        res = min(grid.pitch_x, grid.pitch_y) / 32.0
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

    # Static obstacles ---------------------------------------------------------
    # EVERY footprint pad blocks copper - including unconnected (net 0) balls,
    # which still occupy F.Cu. SMD blocks only F.Cu; through-hole blocks all
    # layers. (Use footprint.pads, not the netted subset, or NC balls get clipped.)
    for p in footprint.pads:
        if p.drill and p.drill > 0:
            occ.block_all(p.global_x, p.global_y, pad_keep)
        else:
            occ.block_layer(top_idx, p.global_x, p.global_y, pad_keep)
    # Plane balls tap their plane through a via -> block all layers.
    for p in plane_pads:
        occ.block_all(p.global_x, p.global_y, via_keep)
    # Existing vias / copper on the board within the region.
    for v in pcb_data.vias:
        if bounds[0] <= v.x <= bounds[2] and bounds[1] <= v.y <= bounds[3]:
            occ.block_all(v.x, v.y, v.size / 2 + track_width / 2 + clearance)
    layer_set = set(layers)
    route_net_ids = {p.net_id for p in signal_pads}
    for s in pcb_data.segments:
        if s.layer not in layer_set:
            continue
        if not (bounds[0] <= s.start_x <= bounds[2] and bounds[1] <= s.start_y <= bounds[3]):
            continue
        if s.net_id in route_net_ids:
            continue  # our own future nets - don't preblock (fanned nets are NOT
                      # in this set, so their existing copper IS kept as obstacle)
        li = layers.index(s.layer)
        occ.block_segment(li, (s.start_x, s.start_y), (s.end_x, s.end_y),
                          s.width / 2 + track_width / 2 + clearance)

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

    def astar(sx, sy, home, route_layers, allow_via):
        """Route from the pad to any boundary cell.

        `route_layers` = the set of layer indices the track may run on. The pad
        sits on the BGA's own layer (top_idx); if that layer is not in
        route_layers the route must drop a via in its own pad (the ONLY via
        allowed - mid-field vias don't fit in a dense array) and run on another
        layer. Straight moves are cheapest so escapes stay radial.
        """
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
                    if in_bbox(nx, ny) and occ.blocked(L, nx, ny) and (nx, ny) not in home:
                        continue
                    # No corner-cutting: a diagonal step past a blocked orthogonal
                    # neighbour clips that obstacle's clearance (the diagonal line
                    # passes nearer the via/pad than either cell). Forbid it.
                    if dx != 0 and dy != 0:
                        if (occ.blocked(L, cx + dx, cy) and (cx + dx, cy) not in home) or \
                           (occ.blocked(L, cx, cy + dy) and (cx, cy + dy) not in home):
                            continue
                    step = 1.0 if (dx == 0 or dy == 0) else 1.6   # discourage zig-zag
                    nxt = (nx, ny, L)
                    ng = cg + step
                    if ng < g.get(nxt, 1e18):
                        g[nxt] = ng
                        came[nxt] = cur
                        heapq.heappush(pq, (ng + heur(nx, ny), nxt))
            # The single via, only in the ball's own pad.
            if allow_via and (cx, cy) in home:
                for L2 in route_layers:
                    if L2 == L:
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
        sx, sy = occ.cell(p.global_x, p.global_y)
        # Exempt the ball's OWN pad+via keepout so the track can break out of its
        # own pad (the keepout ring is wider than the pad and would otherwise
        # trap a route). max(pad,via)_keep < pitch, so it never reaches a
        # neighbouring pad.
        return set(occ._disk(sx, sy, max(pad_keep, via_keep)))

    def commit(p, path):
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
        for ri, (li, cells) in enumerate(runs):
            for (ix, iy) in cells:
                if (ix, iy) not in home:
                    occ.block_layer(li, *occ.xy(ix, iy), trk_keep)
            pts = _segments_from_cells(occ, cells)
            if ri == 0 and pts:
                pts[0] = (p.global_x, p.global_y)  # start exactly at the pad
            for a, b in zip(pts, pts[1:]):
                tracks.append({'start': a, 'end': b, 'width': track_width,
                               'layer': layers[li], 'net_id': net_id})
        for (ix, iy) in via_cells:
            # snap to the pad centre only if the via is still in the pad cell
            vx, vy = (p.global_x, p.global_y) if (ix, iy) == (sx, sy) else occ.xy(ix, iy)
            occ.block_all(vx, vy, via_keep)
            vias_to_add.append({'x': vx, 'y': vy, 'size': via_size,
                                'drill': via_drill,
                                'layers': [layers[0], layers[-1]], 'net_id': net_id})
            nvia += 1

    # Phase 1: route the near-edge balls on the BGA layer (no via). Whatever
    # can't take a clean top-layer escape falls through to the inner phase.
    inner_pads = list(signal_pads)
    if nl > 1:
        inner_pads = []
        for p in signal_pads:
            if depth(p) > outer_depth:
                inner_pads.append(p)
                continue
            sx, sy = occ.cell(p.global_x, p.global_y)
            path = astar(sx, sy, home_of(p), {top_idx}, allow_via=False)
            if path is not None:
                commit(p, path)
                n_fcu += 1
            else:
                inner_pads.append(p)

    # Phase 2: reserve EVERY inner ball's via-in-pad keepout BEFORE routing any
    # inner track. Otherwise a track running under pad P (placed before P's via)
    # and P's later via collide (via-segment shorts). With the vias reserved,
    # inner tracks dodge them and run under the via-less F.Cu pads and through
    # the gaps - which keeps the geometry clean.
    for p in inner_pads:
        occ.block_all(p.global_x, p.global_y, via_keep)

    # Phase 3: route the inner balls (deepest-first - the interior claims the
    # scarce central space before the shallower balls).
    for p in inner_pads:
        sx, sy = occ.cell(p.global_x, p.global_y)
        path = astar(sx, sy, home_of(p), inner_layers, allow_via=True)
        if path is None and nl > 1:
            path = astar(sx, sy, home_of(p), set(range(nl)), allow_via=True)
        if path is None:
            failed.append(p.net_name)
            continue
        commit(p, path)

    if verbose:
        already = f", {len(fanned_net_ids)} already-fanned skipped" if fanned_net_ids else ""
        print(f"  Under-pad escape: {len(signal_pads)-len(failed)}/{len(signal_pads)} "
              f"signals escaped, {nvia} vias, {len(plane_pads)} plane balls skipped"
              f"{already}, {len(failed)} failed")
    return tracks, vias_to_add, failed
