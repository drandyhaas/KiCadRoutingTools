"""Plane-fragility soft routing costs (#424, planes-first chains).

A signal track only BISECTS a plane where the plane is NARROW: a crossing
through a wide pour region costs nothing (copper flows around it at fill
time), while a crossing at a neck severs the pour into islands the repair
step then has to stitch. Bisection is topological, but local narrowness is
a computable proxy: this module rasterizes the pour's copper at the routing
grid, measures every fill cell's distance to the copper boundary (iterative
erosion, no scipy), and stamps a soft per-cell cost on the zone's layer
that is negligible mid-pour and steep at necks.

The copper rasterized is the EXACT KiCad fill when available (a ZONE_FILLER
refill of the batch's input board via kicad_exact_fill -- so voids cut by
fanout barrels and earlier copper make the surviving straits expensive,
which is the "penalize what would break the plane" semantics), falling back
to the drawn zone outline when no board file / KiCad python exists (live GUI
boards). For a full-board outline the fallback prices only a board-edge
band -- near-useless; the exact fill is the real lever.

Delivery reuses the congestion-field transport (#424 Phase D): the field is
computed once at batch start into (N, 4) [layer, gx, gy, cost] rows under a
reserved track_proximity_cache key; merge_track_proximity_costs re-applies
it on every per-net prepare in every path. Vias in fragile cells pay
via_proximity_cost x the cell cost through the Rust via branch (the C6
coupling), which is exactly right: a via is a permanent hole in the plane.

Dynamic refresh (#466, default ON): the batch keeps each pour's rasterized
fill as live state; every committed net (and every rip/restore) carves its
foreign copper out of the affected pour windows and re-erodes ONLY those
windows, so necks CREATED by in-run copper become expensive for every later
net -- the cumulative-erosion blindness of static v1 (dozens of nets each
taking the once-cheap crossing under a BGA until the pour is confetti).
Incremental by construction: no mid-run KiCad refill, no full-board
recompute; a refresh touches a (route bbox + ramp width) window per
overlapped pour and costs ~ms. KICAD_PLANE_FRAGILITY_DYNAMIC=0 restores the
compute-once static field.

DEFAULT ON at 2.0 mm-equivalent (2026-08-01, Andy): with plane drops (#424
D2) and a planes-first chain this measured the winning composition on
ottercast (4 vs 5 conn issues, GND+power fully connected, +3V3 pour a
single intact island, GND weld segments 3075 -> 1063, 63/70 balls served
by the pour alone). Inert on boards with no zones (signals-then-planes
chains feel it only after the pour exists). Corpus A/B pending -- revert
per-run with KICAD_PLANE_FRAGILITY_COST=0.

Knobs via environment (no CLI flags; both fronts inherit the default
through batch_route, so CLI/GUI parity needs no wiring):
  KICAD_PLANE_FRAGILITY_COST   mm-equivalent per-cell cost at a zone
                               boundary cell (default 2.0; 0 = off)
  KICAD_PLANE_FRAGILITY_WIDTH  half-width in mm at which the ramp reaches
                               zero (default 2.0: cells deeper than this
                               into the pour cost nothing)

GUI caveat: a live pcbnew board has no source file, so the field falls
back to the drawn OUTLINE there (near-inert for full-board pours) -- a
known CLI/GUI fidelity gap of the same class as the other
source_path-gated exact-fill features.
"""
from __future__ import annotations

import math
import os
from typing import Dict

import numpy as np

from kicad_parser import PCBData
from routing_config import GridCoord, GridRouteConfig

# Reserved track_proximity_cache key (B1 uses -1, congestion -2).
PLANE_FRAGILITY_CACHE_KEY = -3


def fragility_cost_mm() -> float:
    try:
        return float(os.environ.get('KICAD_PLANE_FRAGILITY_COST', '2.0') or 0)
    except ValueError:
        return 2.0


def _rasterize_polygon(poly, coord, gx0, gy0, W, H) -> np.ndarray:
    """Boolean (H, W) fill mask of a polygon via even-odd crossing test,
    vectorized one scanline row at a time (grid rows are few thousand)."""
    xs = np.array([p[0] for p in poly])
    ys = np.array([p[1] for p in poly])
    mask = np.zeros((H, W), dtype=bool)
    col_x = np.array([coord.to_float(gx0 + i, 0)[0] for i in range(W)])
    x1s, y1s = xs, ys
    x2s, y2s = np.roll(xs, -1), np.roll(ys, -1)
    for j in range(H):
        y = coord.to_float(0, gy0 + j)[1]
        # edges straddling this scanline
        straddle = (y1s <= y) != (y2s <= y)
        if not straddle.any():
            continue
        xi = x1s[straddle] + (y - y1s[straddle]) / (y2s[straddle] - y1s[straddle]) \
            * (x2s[straddle] - x1s[straddle])
        # parity of crossings to the left of each column x
        mask[j] = (col_x[None, :] > xi[:, None]).sum(axis=0) % 2 == 1
    return mask


def _erode_depth(mask: np.ndarray, depth: int) -> np.ndarray:
    """Capped 4-neighbour erosion distance (int16): 0 outside fill, d = steps
    to the boundary, capped at `depth`. Pure numpy slicing, no scipy."""
    dist = np.zeros(mask.shape, dtype=np.int16)
    cur = mask.copy()
    for d in range(1, depth + 1):
        inner = cur.copy()
        inner[1:, :] &= cur[:-1, :]
        inner[:-1, :] &= cur[1:, :]
        inner[:, 1:] &= cur[:, :-1]
        inner[:, :-1] &= cur[:, 1:]
        band = cur & ~inner
        dist[band] = d
        cur = inner
        if not cur.any():
            break
    # Cells DEEPER than the ramp keep dist = 0 and are therefore NOT emitted
    # (`emitted = mask & (dist > 0)`), i.e. the pour interior is free -- the
    # field prices only the fragile EDGE band. Do not "complete" this with
    # `dist[cur] = depth`: that emits the whole interior at
    # frag = 1/depth, which at the 2.0mm default and a 0.1mm grid is
    # 0.05 * 20000 = 1000 units/cell -- exactly 1.00x a normal move, so
    # crossing any wide pour would cost DOUBLE. It also silently made
    # KICAD_PLANE_FRAGILITY_DYNAMIC=0 stop being a true revert, since the
    # static path shares this helper (measured: identical to the pre-#466
    # static implementation, which had no such branch).
    return dist


class _PourState:
    """Live raster of one filled pour island (dynamic mode)."""
    __slots__ = ('li', 'layer', 'net_id', 'gx0', 'gy0', 'W', 'H',
                 'orig', 'mask', 'dist', 'rows')

    def __init__(self, li, layer, net_id, gx0, gy0, mask, dist, rows):
        self.li, self.layer, self.net_id = li, layer, net_id
        self.gx0, self.gy0 = gx0, gy0
        self.H, self.W = mask.shape
        self.orig = mask            # fill as of batch start (never mutated)
        self.mask = mask.copy()     # fill minus in-run foreign copper
        self.dist = dist
        self.rows = rows


class FragilityField:
    """Dynamic plane-fragility field (#466). Holds every pour's raster and
    republishes the cost rows after each copper change, re-eroding only the
    dirty window (route bbox + ramp depth), never the whole board."""

    _disc_cache: Dict[int, np.ndarray] = {}

    def __init__(self, states, coord, depth, cell_cost, carve_r_mm, cache):
        self.states = states
        self.coord = coord
        self.depth = depth
        self.cell_cost = cell_cost
        self.carve_r_mm = carve_r_mm    # copper-to-pour clearance for carving
        self.cache = cache
        self.refreshes = 0
        self.refresh_s = 0.0

    @classmethod
    def _disc(cls, r: int) -> np.ndarray:
        d = cls._disc_cache.get(r)
        if d is None:
            ax = np.arange(-r, r + 1)
            d = (ax[:, None] ** 2 + ax[None, :] ** 2) <= r * r
            cls._disc_cache[r] = d
        return d

    def _stamp_disc(self, m, j, i, r):
        """OR a False-disc of cell-radius r into bool window m at (j, i)."""
        H, W = m.shape
        j0, j1 = max(0, j - r), min(H, j + r + 1)
        i0, i1 = max(0, i - r), min(W, i + r + 1)
        if j0 >= j1 or i0 >= i1:
            return
        d = self._disc(r)
        m[j0:j1, i0:i1] &= ~d[j0 - (j - r):j1 - (j - r), i0 - (i - r):i1 - (i - r)]

    def _carve_window(self, st, pcb_data, wj0, wj1, wi0, wi1):
        """Rebuild st.mask in the window from orig minus ALL foreign copper
        (segments on the pour's layer + via barrels) intersecting it.
        Idempotent: pre-existing copper is already a hole in orig."""
        step = self.coord.grid_step
        win = st.orig[wj0:wj1, wi0:wi1].copy()
        # window bounds in board mm (+ margin for the largest carve radius)
        wx0, wy0 = self.coord.to_float(st.gx0 + wi0, st.gy0 + wj0)
        wx1, wy1 = self.coord.to_float(st.gx0 + wi1, st.gy0 + wj1)
        marg = self.carve_r_mm + 1.0
        for s in pcb_data.segments:
            if s.net_id == st.net_id or s.layer != st.layer:
                continue
            if max(s.start_x, s.end_x) < wx0 - marg or \
               min(s.start_x, s.end_x) > wx1 + marg or \
               max(s.start_y, s.end_y) < wy0 - marg or \
               min(s.start_y, s.end_y) > wy1 + marg:
                continue
            r = max(1, int(round((s.width / 2.0 + self.carve_r_mm) / step)))
            n = max(1, int(math.hypot(s.end_x - s.start_x,
                                      s.end_y - s.start_y) / step))
            for t in range(n + 1):
                x = s.start_x + (s.end_x - s.start_x) * t / n
                y = s.start_y + (s.end_y - s.start_y) * t / n
                gi, gj = self.coord.to_grid(x, y)
                self._stamp_disc(win, gj - st.gy0 - wj0, gi - st.gx0 - wi0, r)
        for v in pcb_data.vias:
            if v.net_id == st.net_id:
                continue
            if not (wx0 - marg <= v.x <= wx1 + marg and
                    wy0 - marg <= v.y <= wy1 + marg):
                continue
            r = max(1, int(round((v.size / 2.0 + self.carve_r_mm) / step)))
            gi, gj = self.coord.to_grid(v.x, v.y)
            self._stamp_disc(win, gj - st.gy0 - wj0, gi - st.gx0 - wi0, r)
        st.mask[wj0:wj1, wi0:wi1] = win

    def _regen_rows(self, st):
        emitted = st.mask & (st.dist > 0)
        if not emitted.any():
            st.rows = None
            return
        jj, ii = np.nonzero(emitted)
        frag = 1.0 - (st.dist[jj, ii].astype(np.float64) - 1) / self.depth
        costs = np.maximum(1, (frag * self.cell_cost).astype(np.int32))
        st.rows = np.column_stack([
            np.full(len(jj), st.li, dtype=np.int32),
            (ii + st.gx0).astype(np.int32),
            (jj + st.gy0).astype(np.int32),
            costs,
        ])

    def refresh(self, pcb_data, segments, vias):
        """Copper changed (commit, rip, or restore): re-carve + re-erode the
        affected window of every overlapped pour and republish the rows."""
        import time as _time
        t0 = _time.perf_counter()
        # dirty bbox per layer-index from the changed copper (vias hit all)
        boxes: Dict[int, list] = {}
        allv = []
        for s in (segments or ()):
            li = None
            for st in self.states:
                if st.layer == s.layer:
                    li = st.li
                    break
            if li is None:
                continue
            b = boxes.setdefault(li, [np.inf, np.inf, -np.inf, -np.inf])
            b[0] = min(b[0], s.start_x, s.end_x); b[1] = min(b[1], s.start_y, s.end_y)
            b[2] = max(b[2], s.start_x, s.end_x); b[3] = max(b[3], s.start_y, s.end_y)
        for v in (vias or ()):
            allv.append((v.x, v.y))
        if not boxes and not allv:
            return
        pad_mm = self.carve_r_mm + self.depth * self.coord.grid_step
        changed = False
        for st in self.states:
            bb = [np.inf, np.inf, -np.inf, -np.inf]
            if st.li in boxes:
                b = boxes[st.li]
                bb = [min(bb[0], b[0]), min(bb[1], b[1]),
                      max(bb[2], b[2]), max(bb[3], b[3])]
            for (vx, vy) in allv:
                bb = [min(bb[0], vx), min(bb[1], vy),
                      max(bb[2], vx), max(bb[3], vy)]
            if bb[0] is np.inf or bb[0] == np.inf:
                continue
            gi0, gj0 = self.coord.to_grid(bb[0] - pad_mm, bb[1] - pad_mm)
            gi1, gj1 = self.coord.to_grid(bb[2] + pad_mm, bb[3] + pad_mm)
            # inner write window (state-local), compute window adds one more
            # depth margin so erosion distances at the write edge are exact
            wi0 = max(0, gi0 - st.gx0); wi1 = min(st.W, gi1 - st.gx0 + 1)
            wj0 = max(0, gj0 - st.gy0); wj1 = min(st.H, gj1 - st.gy0 + 1)
            if wi0 >= wi1 or wj0 >= wj1:
                continue
            ci0 = max(0, wi0 - self.depth); ci1 = min(st.W, wi1 + self.depth)
            cj0 = max(0, wj0 - self.depth); cj1 = min(st.H, wj1 + self.depth)
            self._carve_window(st, pcb_data, cj0, cj1, ci0, ci1)
            dist = _erode_depth(st.mask[cj0:cj1, ci0:ci1], self.depth)
            st.dist[wj0:wj1, wi0:wi1] = dist[wj0 - cj0:wj1 - cj0,
                                             wi0 - ci0:wi1 - ci0]
            self._regen_rows(st)
            changed = True
        if changed:
            self.publish()
            self.refreshes += 1
        self.refresh_s += _time.perf_counter() - t0

    def publish(self):
        rows = [st.rows for st in self.states if st.rows is not None]
        if rows:
            out = np.vstack(rows)
            # Same max-cost dedup the STATIC field applies (review DRC-7):
            # overlapping same-layer pours emit the shared cells once per
            # zone, and the initial registration lexsorts to per-cell max --
            # a refresh that just vstacks re-introduces the duplicates, so a
            # cost consumer that sums rows double-charges overlap cells
            # relative to the static field.
            order = np.lexsort((out[:, 3], out[:, 2], out[:, 1], out[:, 0]))
            out = out[order]
            keep = np.ones(len(out), dtype=bool)
            same = ((np.diff(out[:, 0]) == 0) & (np.diff(out[:, 1]) == 0)
                    & (np.diff(out[:, 2]) == 0))
            keep[:-1][same] = False   # lexsort put max cost last per cell
            self.cache[PLANE_FRAGILITY_CACHE_KEY] = out[keep]
        else:
            self.cache.pop(PLANE_FRAGILITY_CACHE_KEY, None)


def fragility_on_copper_change(config, pcb_data, segments, vias) -> None:
    """Hook for commit/rip/restore sites: incremental #466 refresh. No-op
    (one getattr) unless the dynamic field is active on this batch."""
    fld = getattr(config, '_fragility_field', None)
    if fld is None or (not segments and not vias):
        return
    try:
        fld.refresh(pcb_data, segments, vias)
    except Exception as e:  # soft cost only -- never kill a route over it
        print(f"Plane fragility: dynamic refresh failed ({e}); "
              f"field frozen at last state")
        config._fragility_field = None


def compute_plane_fragility_cells(pcb_data: PCBData,
                                  config: GridRouteConfig) -> np.ndarray:
    """(N, 4) [layer, gx, gy, cost] rows for every filled-zone cell within
    the fragility half-width of its zone's boundary. Empty when disabled or
    the board has no filled zones on routing layers."""
    cells, _states = _compute_cells_and_states(pcb_data, config,
                                               want_states=False)
    return cells


def _compute_cells_and_states(pcb_data: PCBData, config: GridRouteConfig,
                              want_states: bool):
    cost_mm = fragility_cost_mm()
    if cost_mm <= 0 or not pcb_data.zones:
        return np.empty((0, 4), dtype=np.int32), []
    try:
        width_mm = float(os.environ.get('KICAD_PLANE_FRAGILITY_WIDTH', '2.0') or 2.0)
    except ValueError:
        width_mm = 2.0

    coord = GridCoord(config.grid_step)
    layer_index = {l: i for i, l in enumerate(config.layers)}
    depth = max(1, int(round(width_mm / config.grid_step)))
    rows = []

    # Exact fill polygons (KiCad truth): the GUI's live-board provider first
    # (#424: an in-process ZONE_FILLER refill of the board object itself --
    # no save, no staleness), then a refill of the source file (CLI), then
    # the drawn zone outlines as the last resort. Each entry: (layer, poly).
    name_to_id = {n.name: n.net_id for n in pcb_data.nets.values()}
    polys = []
    provider = getattr(pcb_data, 'exact_fill_provider', None)
    if provider is not None:
        try:
            fills = provider()
            polys = [(name_to_id.get(_net, -1), layer, poly)
                     for (_net, layer), pp in fills.items() for poly in pp]
            if polys:
                print(f"Plane fragility: exact-fill geometry "
                      f"({len(polys)} filled island(s) from the LIVE board)")
        except Exception as e:
            print(f"Plane fragility: live-board fill unavailable ({e}); "
                  f"trying the source file")
            polys = []
    src = getattr(pcb_data, 'source_path', None)
    if not polys and src and os.path.isfile(src):
        try:
            from kicad_exact_fill import refill_islands, EXACT_FILL_TIMEOUT
            # BOUND THE FILL BY THE RUN'S REMAINING BUDGET. This call is reached
            # from route.py:batch_route, so it runs on EVERY batch_route -- and
            # on a board KiCad's ZONE_FILLER cannot fill (measured: a 217-part
            # 4-layer board) each one pays the full 300s EXACT_FILL_TIMEOUT.
            # The plane repair issues many batch_route calls, which is the root
            # cause of both non-terminations measured in run 9. No signature
            # between the CLI and here carries a budget, hence the global.
            _t = EXACT_FILL_TIMEOUT
            try:
                import krt_deadline
                _dl = krt_deadline.current()
            except Exception:                                  # noqa: BLE001
                _dl = None
            if _dl is not None:
                _left = _dl.remaining()
                if _dl.expired() or (_left is not None and _left < 5.0):
                    raise TimeoutError(
                        'run budget spent; not starting a KiCad refill')
                # Never let one fill eat the whole remaining budget.
                _t = max(5, int(min(_t, (_left or _t) * 0.5)))
            fills = refill_islands(src, timeout=_t, verbose=True)
            if fills is None:
                # refill_islands documents a None return, and this was the ONE
                # call site in the repo that dereferenced it anyway. The
                # resulting `'NoneType' object has no attribute 'items'` was
                # printed as if it were the diagnosis, while the real reason --
                # a 300s timeout, or pcbnew missing -- was computed inside
                # refill_islands and discarded. verbose=True above makes it say
                # so; this branch stops the AttributeError masquerading as one.
                raise RuntimeError(
                    f'KiCad refill returned nothing within {_t}s '
                    f'(fill timed out, or pcbnew unavailable)')
            polys = [(name_to_id.get(_net, -1), layer, poly)
                     for (_net, layer), pp in fills.items() for poly in pp]
            if polys:
                print(f"Plane fragility: exact-fill geometry "
                      f"({len(polys)} filled island(s) from KiCad refill)")
        except Exception as e:
            print(f"Plane fragility: exact refill unavailable ({e}); "
                  f"using zone outlines")
            polys = []
    if not polys:
        polys = [(z.net_id, z.layer, z.polygon) for z in pcb_data.zones]

    if os.environ.get('KICAD_FRAGILITY_DEBUG') == '1':
        # The field is a raster of the exact fill, so a GUI/CLI cell-count
        # difference is either the POLYGON or the GRID -- print both rather
        # than guessing which. (A 795-cell gap between the fronts on
        # splitflap_driver cost a long bisect that static file comparison
        # could not settle: every saved board agreed, only the live one did
        # not.)
        def _a(p):
            s = 0.0
            for i in range(len(p)):
                x1, y1 = p[i]
                x2, y2 = p[(i + 1) % len(p)]
                s += x1 * y2 - x2 * y1
            return abs(s) / 2.0
        print(f"  [frag-debug] grid_step={config.grid_step} depth={depth} "
              f"layers={config.layers} polys={len(polys)}")
        for _n, _l, _p in polys[:4]:
            xs = [q[0] for q in _p]
            ys = [q[1] for q in _p]
            print(f"  [frag-debug]   net={_n} layer={_l} verts={len(_p)} "
                  f"area={_a(_p):.4f} bbox=({min(xs):.4f},{min(ys):.4f})-"
                  f"({max(xs):.4f},{max(ys):.4f})")

    states = []
    for znet, zlayer, zpoly in polys:
        li = layer_index.get(zlayer)
        if li is None or len(zpoly) < 3:
            continue
        xs = [p[0] for p in zpoly]
        ys = [p[1] for p in zpoly]
        gx0, gy0 = coord.to_grid(min(xs), min(ys))
        gx1, gy1 = coord.to_grid(max(xs), max(ys))
        W, H = int(gx1 - gx0 + 1), int(gy1 - gy0 + 1)
        if W <= 2 or H <= 2 or W * H > 40_000_000:
            continue  # degenerate or absurdly large raster
        mask = _rasterize_polygon(zpoly, coord, gx0, gy0, W, H)
        if not mask.any():
            continue
        # distance-to-boundary in grid steps by iterative 4-neighbour erosion,
        # capped at `depth` (cells deeper than the ramp are never emitted)
        dist = _erode_depth(mask, depth)
        emitted = mask & (dist > 0)
        if not emitted.any():
            continue
        jj, ii = np.nonzero(emitted)
        frag = 1.0 - (dist[jj, ii].astype(np.float64) - 1) / depth  # 1 at edge -> ~0 deep
        cell_cost = config.cell_cost(cost_mm)
        costs = np.maximum(1, (frag * cell_cost).astype(np.int32))
        zone_rows = np.column_stack([
            np.full(len(jj), li, dtype=np.int32),
            (ii + gx0).astype(np.int32),
            (jj + gy0).astype(np.int32),
            costs,
        ])
        rows.append(zone_rows)
        if want_states:
            states.append(_PourState(li, zlayer, znet, gx0, gy0,
                                     mask, dist, zone_rows))

    if not rows:
        return np.empty((0, 4), dtype=np.int32), []
    out = np.vstack(rows)
    # overlapping zones on one layer: keep the max cost per cell
    order = np.lexsort((out[:, 3], out[:, 2], out[:, 1], out[:, 0]))
    out = out[order]
    keep = np.ones(len(out), dtype=bool)
    same = (np.diff(out[:, 0]) == 0) & (np.diff(out[:, 1]) == 0) & (np.diff(out[:, 2]) == 0)
    keep[:-1][same] = False  # lexsort put max cost last within a cell group
    return out[keep], states


def register_plane_fragility(pcb_data: PCBData, config: GridRouteConfig,
                             track_proximity_cache: Dict) -> None:
    """Compute and register the field under the reserved cache key (no-op
    when disabled or no zones). With KICAD_PLANE_FRAGILITY_DYNAMIC on (the
    default), also arm the #466 incremental field on `config` so the
    commit/rip/restore hooks keep it current as in-run copper lands."""
    dynamic = os.environ.get('KICAD_PLANE_FRAGILITY_DYNAMIC', '1') != '0'
    cells, states = _compute_cells_and_states(pcb_data, config,
                                              want_states=dynamic)
    if not len(cells):
        return
    track_proximity_cache[PLANE_FRAGILITY_CACHE_KEY] = cells
    n_zones = len({(z.layer, z.net_id) for z in pcb_data.zones})
    mode = 'static'
    if dynamic and states:
        try:
            width_mm = float(os.environ.get('KICAD_PLANE_FRAGILITY_WIDTH',
                                            '2.0') or 2.0)
        except ValueError:
            width_mm = 2.0
        coord = GridCoord(config.grid_step)
        depth = max(1, int(round(width_mm / config.grid_step)))
        config._fragility_field = FragilityField(
            states, coord, depth, config.cell_cost(fragility_cost_mm()),
            carve_r_mm=config.clearance, cache=track_proximity_cache)
        mode = 'DYNAMIC #466 (=0 via KICAD_PLANE_FRAGILITY_DYNAMIC reverts)'
    print(f"Plane fragility field: {len(cells)} cells near the edges of "
          f"{n_zones} pour(s) (KICAD_PLANE_FRAGILITY_COST="
          f"{fragility_cost_mm()}mm-equiv, {mode})")
