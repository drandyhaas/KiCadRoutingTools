"""#1843: route at the clearance a rule area sets.

A `.kicad_dru` clearance rule whose condition is one rule-area predicate
(`A.insideArea('ROOM')`, `intersectsArea`) binds a pair when either ITEM's
copper intersects the area on the layer the pair is on, and its value
REPLACES the class value (kicad-cli 10.0.6, probe boards): a track partly
inside the area is bound along its whole length, a track that stops short of
it (round cap included) is not, and a via in an area drawn on F.Cu only is
bound against F.Cu copper only. `design_rules.AreaIndex` grades the same way.

The obstacle map prices a keep-out per obstacle, not per position, so the
rule is honoured by COMPOSING two stamps of everything:

  * outside the area, the stamp at the class values, as without the rule;
  * inside it, the stamp at the rule's value.

A track cell is inside on the layers the area is drawn on; a via cell is
inside where the area covers every copper layer of the board. Around an area
whose rule is ABOVE the run's base clearance (a TIGHT area) a ring keeps both
stamps, wide enough to hold every item the run can lay that touches the area
at the rule's value; laid copper (pads, input tracks and vias) whose shape
touches a tight area is held at the rule's value along its whole extent (the
ring is widened around it on its layers).

The run's own tracks are SPLIT where they leave a tight area's split zone
(`split_reach` beyond the area: half the widest track the run draws plus
`SPLIT_MARGIN`), so the piece outside is an item of its own that KiCad does
not bind (`split_segments_at_zones`, run at the start and end of the cleanup
pipeline and once more on the board a route run ships). The cleanup passes
price each pair through `pair_excess`: the region an item binds, by the same
reading, own copper and foreign copper alike.

Every add and every remove of a keep-out goes through the same filter
(`outside` / `inside`), so the map's reference counts balance exactly as they
do without a rule.
"""

from dataclasses import replace
from functools import wraps
import copy as _copy
import inspect
import math
from typing import Dict, List, Optional

import numpy as np

#: how far past a tight area's edge a run's track is split, beyond its half
#: width: the piece outside then stops this far short of the area, round cap
#: included, so KiCad does not read it as in the area
SPLIT_MARGIN = 0.01
#: no split leaves a piece shorter than this
MIN_PIECE = 1e-3

# method name -> (row kind, index of the row argument)
_ROW_METHODS = {}
for _n in ('add_blocked_cells_batch', 'remove_blocked_cells_batch',
           'add_static_blocked_cells_batch'):
    _ROW_METHODS[_n] = ('cells', 0)
for _n in ('add_blocked_cell_spans_batch', 'remove_blocked_cell_spans_batch',
           'add_static_blocked_cell_spans_batch'):
    _ROW_METHODS[_n] = ('cell_spans', 0)
for _n in ('add_blocked_vias_batch', 'remove_blocked_vias_batch',
           'add_blocked_vias_small_batch', 'remove_blocked_vias_small_batch',
           'add_static_blocked_vias_batch'):
    _ROW_METHODS[_n] = ('vias', 0)
for _n in ('add_blocked_vias_rung_batch', 'remove_blocked_vias_rung_batch'):
    _ROW_METHODS[_n] = ('vias', 1)
for _n in ('add_blocked_via_spans_batch', 'remove_blocked_via_spans_batch',
           'add_static_blocked_via_spans_batch'):
    _ROW_METHODS[_n] = ('via_spans', 0)
# one cell / one via: (kind, index of gx)
_POINT_METHODS = {
    'add_blocked_cell': ('cell', 0), 'add_static_blocked_cell': ('cell', 0),
    'add_blocked_via': ('via', 0), 'add_static_blocked_via': ('via', 0),
    'add_blocked_via_small': ('via', 0), 'remove_blocked_via_small': ('via', 0),
    'add_blocked_via_rung': ('via', 1), 'remove_blocked_via_rung': ('via', 1),
}
# what a sub-build may ask the parent's map; every other call it makes
# (cost fields, zones, source/target cells) goes to a map of its own
_QUERY_PREFIXES = ('is_', 'get_', 'blocked_', 'rung_', 'segment_blocked',
                   'open_via', 'populated')


_ANNOUNCED = set()


class Region:
    """One rule area the router honours: ``value`` the clearance its rule
    sets (None: the class values stay), ``width`` the track width (None: the
    run's). ``ring`` > 0 marks a TIGHT region (its clearance is above the
    run's base clearance)."""
    __slots__ = ('rule', 'area', 'value', 'polygon', 'layers', 'ring', 'width', 'order',
                 'zone', 'bounds')

    def __init__(self, rule, area, value, polygon, layers, ring, width=None, order=0):
        self.rule, self.area, self.value = rule, area, value
        self.polygon, self.layers, self.ring = polygon, layers, ring
        self.width = width
        self.order = order        # the clearance rule's position in the file
        self.zone = None          # the split zone (tight regions)
        self.bounds = polygon.bounds

    @property
    def tight(self) -> bool:
        return self.ring > 0


class AreaClearance:
    """The board's area-scoped clearance rules, as the obstacle map reads
    them. ``regions`` are in file order: where two overlap, the later one
    owns the cell, as KiCad's last matching rule does.

    ``split_reach``: the distance from a tight area within which a run's
    track piece is read as in it (and beyond which a track is split off).
    ``extensions``: {region index: {layer: shape}} -- the laid copper on that
    layer that touches the tight region, grown by the rule's reach; the ring
    of the region covers it, so that copper is held at the rule's value along
    its whole extent."""

    def __init__(self, regions: List[Region], copper_layers, skipped=(),
                 layer_orders=None, split_reach: float = 0.0, extensions=None,
                 via_reach: float = 0.0):
        self.regions = list(regions)
        self.copper = list(copper_layers)
        self.skipped = list(skipped)
        #: {layer: file position of the rule its layer clearance comes from}
        self.layer_orders = dict(layer_orders or {})
        self.split_reach = float(split_reach)
        #: a via within this of a tight area (its radius, plus the margin)
        #: touches it; no via of the run may straddle a tight area's edge
        self.via_reach = float(via_reach or split_reach)
        self.extensions = dict(extensions or {})
        for r in self.regions:
            if r.tight:
                r.zone = r.polygon.buffer(self.split_reach)
        self._masks = {}
        self._later = {}
        self._rows = {}
        self._zones = None
        self._own_memo = {}
        self._item_memo = {}
        self._band = {}

    # ------------------------------------------------------------- masks
    def _region_masks(self, step: float):
        """Per region: (core, ring, {layer: ring with that layer's laid
        copper}, ring for via cells), each a raster or None."""
        key = round(float(step), 9)
        got = self._masks.get(key)
        if got is None:
            got = []
            for i, r in enumerate(self.regions):
                core = _raster(r.polygon, step)
                ring = ring_layers = ring_via = None
                ring_layers = {}
                if r.ring > 0:
                    shape = r.polygon.buffer(r.ring)
                    ring = _raster(shape, step)
                    ext = self.extensions.get(i) or {}
                    for lname, g in ext.items():
                        ring_layers[lname] = _raster(shape.union(g), step)
                    if ext:
                        from shapely.ops import unary_union
                        ring_via = _raster(unary_union([shape] + list(ext.values())), step)
                    else:
                        ring_via = ring
                got.append((core, ring, ring_layers, ring_via))
            self._masks[key] = got
        return got

    def _classify(self, step, gx, gy, layer_idx, layers, want: Optional[int]):
        """Rows kept: with ``want`` None the rows no region owns, else the
        rows region ``want`` owns plus the unowned rows in its ring.
        ``layer_idx`` None reads the rows as via cells."""
        n = len(gx)
        owner = np.full(n, -1, dtype=np.int32)
        masks = self._region_masks(step)
        for i, r in enumerate(self.regions):
            on = self._on_layers(r, layer_idx, layers, n, core=True)
            if on is None:
                continue
            hit = _lookup(masks[i][0], gx, gy)
            if on is not True:
                hit &= on
            owner[hit] = i
        if want is None:
            return owner < 0
        keep = owner == want
        _core, ring, ring_layers, ring_via = masks[want]
        if ring is not None:
            on = self._on_layers(self.regions[want], layer_idx, layers, n, core=False)
            if on is not None:
                if layer_idx is None:
                    hit = _lookup(ring_via, gx, gy)
                else:
                    hit = _lookup(ring, gx, gy)
                    for lname, m in ring_layers.items():
                        if lname not in layers:
                            continue
                        sel = np.asarray(layer_idx) == layers.index(lname)
                        if sel.any():
                            hit[sel] = _lookup(m, np.asarray(gx)[sel], np.asarray(gy)[sel])
                hit &= owner < 0
                if on is not True:
                    hit &= on
                keep |= hit
        return keep

    def _on_layers(self, region, layer_idx, layers, n, core: bool):
        """True (every row), a row mask, or None (no row) for the rows on a
        layer ``region`` reads."""
        if layer_idx is None:                       # via cells
            if core:
                return True if set(self.copper) <= region.layers else None
            return True
        idx = [k for k, name in enumerate(layers) if name in region.layers]
        if not idx:
            return None
        if len(idx) == len(layers):
            return True
        return np.isin(layer_idx, idx)

    # ----------------------------------------------------------- filters
    def keep(self, kind: str, rows, config, want: Optional[int]):
        """``rows`` of ``kind`` ('cells', 'cell_spans', 'vias', 'via_spans')
        cut down to one side of the composition."""
        a = np.asarray(rows, dtype=np.int32)
        if a.size == 0:
            return a
        step, layers = config.grid_step, config.layers
        if kind == 'cells':
            a = a.reshape(-1, 3)
            return np.ascontiguousarray(
                a[self._classify(step, a[:, 0], a[:, 1], a[:, 2], layers, want)])
        if kind == 'vias':
            a = a.reshape(-1, 2)
            return np.ascontiguousarray(
                a[self._classify(step, a[:, 0], a[:, 1], None, layers, want)])
        width = 4 if kind == 'cell_spans' else 3
        a = a.reshape(-1, width)
        near = self._spans_near(step, a, want)
        far = a[~near] if want is None else a[:0]
        cut = a[near]
        if len(cut) == 0:
            return np.ascontiguousarray(far)
        lo = cut[:, 1].astype(np.int64)
        counts = np.maximum(cut[:, 2].astype(np.int64) - lo + 1, 0)
        total = int(counts.sum())
        if total == 0:
            return np.ascontiguousarray(far)
        starts = np.zeros(len(counts), dtype=np.int64)
        np.cumsum(counts[:-1], out=starts[1:])
        gx = np.repeat(cut[:, 0], counts)
        gy = (np.repeat(lo, counts)
              + (np.arange(total, dtype=np.int64) - np.repeat(starts, counts))).astype(np.int32)
        lay = np.repeat(cut[:, 3], counts) if width == 4 else None
        k = self._classify(step, gx, gy, lay, layers, want)
        out = np.empty((int(k.sum()), width), dtype=np.int32)
        out[:, 0] = gx[k]
        out[:, 1] = gy[k]
        out[:, 2] = gy[k]
        if width == 4:
            out[:, 3] = lay[k]
        return np.ascontiguousarray(np.concatenate([far, out]) if len(far) else out)

    def _spans_near(self, step, spans, want):
        """Spans that reach the bounding box of a region's mask."""
        near = np.zeros(len(spans), dtype=bool)
        masks = self._region_masks(step)
        for i, (core, ring, ring_layers, ring_via) in enumerate(masks):
            ms = [core]
            if want is None or want == i:
                ms += [ring, ring_via] + list(ring_layers.values())
            for m in ms:
                if m is None:
                    continue
                gx0, gy0, arr = m
                near |= ((spans[:, 0] >= gx0) & (spans[:, 0] < gx0 + arr.shape[0])
                         & (spans[:, 2] >= gy0) & (spans[:, 1] < gy0 + arr.shape[1]))
        return near

    def keep_point(self, kind: str, gx, gy, layer, config, want) -> bool:
        lay = None if kind == 'via' else np.array([layer], dtype=np.int32)
        return bool(self._classify(config.grid_step, np.array([gx], dtype=np.int32),
                                   np.array([gy], dtype=np.int32), lay,
                                   config.layers, want)[0])

    # ----------------------------------------------------------- configs
    def outside_config(self, config):
        """``config`` as the stamp outside the areas reads it."""
        return replace(config, area_clearance=None)

    def region_config(self, config, want: int):
        """``config`` as the stamp inside region ``want`` reads it: the
        rule's clearance in place of every class value and of every rule
        BEFORE it in the file, and the room's track width where it is
        narrower than the run's. KiCad's last matching rule wins, so a layer
        rule or a class/net-scoped rule that comes after the area's still
        replaces it for what it binds; the track-to-track raises stay as
        they are (they only widen)."""
        r = self.regions[want]
        cfg = replace(config, area_clearance=None)
        if r.value is not None:
            layers = {l: v for l, v in (config.layer_clearances or {}).items()
                      if self.layer_orders.get(l, -1) > r.order}
            cfg = replace(cfg, clearance=r.value,
                          net_clearances={}, net_clearance_floor=None,
                          layer_clearances=layers,
                          scoped_clearance=self._scoped_after(config, r.order))
        if r.width is not None and r.width < cfg.track_width:
            cfg = replace(cfg, track_width=r.width)
        return cfg

    def _scoped_after(self, config, order: int):
        """The scoped channel of ``config`` cut down to the rules after file
        position ``order``, or None when none is."""
        inst = getattr(config, 'scoped_clearance', None)
        if inst is None:
            return None
        got = self._later.get((id(inst), order))
        if got is not None and got[0] is inst:
            return got[1]
        later = None
        try:
            from kicad_dru import ScopedClearanceInstall, ScopedClearanceRules
            rules = [x for x in inst.sc.rules if x.order > order]
            if rules:
                later = ScopedClearanceInstall(
                    ScopedClearanceRules(inst.sc.dr, rules, inst.sc.layer_orders),
                    inst.reps, {}, inst.fab_floor)
        except Exception:                                       # noqa: BLE001
            later = inst      # unreadable: keep every scoped rule (the wider side)
        self._later[(id(inst), order)] = (inst, later)
        return later

    def min_width(self, layer: str) -> Optional[float]:
        """The narrowest room track width on ``layer``, or None."""
        ws = [r.width for r in self.regions if r.width is not None and layer in r.layers]
        return min(ws) if ws else None

    def via_band_cells(self, step: float) -> np.ndarray:
        """(N, 2) via grid cells whose via would straddle a tight area's edge
        (its centre within `via_reach` of the edge, on either side). The run
        places no via there: a via is then wholly inside a tight area or
        clear of it, so its copper never reaches past the ring."""
        key = round(float(step), 9)
        got = self._band.get(key)
        if got is None:
            parts = []
            for r in self.regions:
                if not r.tight:
                    continue
                b = self.via_reach + step / 2.0
                band = r.polygon.buffer(b).difference(r.polygon.buffer(-b))
                if band.is_empty:
                    continue
                gx0, gy0, arr = _raster(band, step)
                ix, iy = np.nonzero(arr)
                parts.append(np.column_stack([ix + gx0, iy + gy0]).astype(np.int32))
            got = (np.unique(np.concatenate(parts), axis=0) if parts
                   else np.empty((0, 2), dtype=np.int32))
            self._band[key] = np.ascontiguousarray(got)
        return got

    # ------------------------------------------------ nearness (speed)
    def influence_bounds(self, i):
        """The bounding box of every grid row region ``i`` keeps (its area,
        ring and the laid copper its ring is widened over)."""
        r = self.regions[i]
        x0, y0, x1, y1 = r.bounds
        g = r.ring
        x0, y0, x1, y1 = x0 - g, y0 - g, x1 + g, y1 + g
        for shape in (self.extensions.get(i) or {}).values():
            a, b, c, d = shape.bounds
            x0, y0, x1, y1 = min(x0, a), min(y0, b), max(x1, c), max(y1, d)
        return x0, y0, x1, y1

    def stamp_reach(self, config, extra_clearance=0.0) -> float:
        """A bound on how far a keep-out row reaches past the copper it is
        stamped for, at any value a region or class sets."""
        vals = [float(config.clearance)] + [r.value for r in self.regions
                                            if r.value is not None]
        vals += [float(v) for v in (getattr(config, 'net_clearances', None) or {}).values()]
        vals += [float(v) for v in (getattr(config, 'layer_clearances', None) or {}).values()]
        tw, vs = _run_sizes(config)
        return max(vals) + 2.0 * max(tw, vs) + float(extra_clearance or 0.0) + 1.0

    @staticmethod
    def net_bounds(pcb_data, net_id):
        """The bounding box of a net's pads, tracks and vias, or None."""
        xs, ys = [], []
        for p in (getattr(pcb_data, 'pads_by_net', None) or {}).get(net_id, ()):
            hx = max(p.size_x, p.size_y) / 2.0
            xs += [p.global_x - hx, p.global_x + hx]
            ys += [p.global_y - hx, p.global_y + hx]
        for sg in getattr(pcb_data, 'segments', None) or ():
            if sg.net_id == net_id:
                hw = sg.width / 2.0
                xs += [min(sg.start_x, sg.end_x) - hw, max(sg.start_x, sg.end_x) + hw]
                ys += [min(sg.start_y, sg.end_y) - hw, max(sg.start_y, sg.end_y) + hw]
        for v in getattr(pcb_data, 'vias', None) or ():
            if v.net_id == net_id:
                xs += [v.x - v.size / 2.0, v.x + v.size / 2.0]
                ys += [v.y - v.size / 2.0, v.y + v.size / 2.0]
        if not xs:
            return None
        return min(xs), min(ys), max(xs), max(ys)

    def regions_near(self, bounds, margin) -> set:
        """The regions whose rows a stamp of copper inside ``bounds``, reaching
        ``margin`` past it, can touch. ``bounds`` None: every region."""
        if bounds is None:
            return set(range(len(self.regions)))
        x0, y0, x1, y1 = bounds
        out = set()
        for i in range(len(self.regions)):
            a, b, c, d = self.influence_bounds(i)
            if not (x1 + margin < a or x0 - margin > c or y1 + margin < b or y0 - margin > d):
                out.add(i)
        return out

    # ------------------------------------------------- the item reading
    # One reading of "the region an item binds", shared by every pass that
    # prices a pair after routing. It errs to the safe side either way:
    #   * a TIGHT region binds own copper whose centreline comes within
    #     `split_reach` of the area (wider than KiCad's half-width test, so it
    #     binds everything KiCad binds), and foreign copper by KiCad's test;
    #   * a RELAXING region binds a track or via whose centreline meets the
    #     area, and a pad whose centre is inside it -- each a case KiCad binds
    #     (its test reads the copper's full width).
    def own_index(self, x1, y1, x2, y2, layer) -> int:
        """The last region binding the moving copper (a track on ``layer``,
        or a via probed as a zero-length one), or -1."""
        key = (x1, y1, x2, y2, layer)
        got = self._own_memo.get(key)
        if got is not None:
            return got
        if len(self._own_memo) > 200000:
            self._own_memo.clear()
        found = self._own_memo[key] = self._own_index(x1, y1, x2, y2, layer)
        return found

    def _own_index(self, x1, y1, x2, y2, layer) -> int:
        found = -1
        lo_x, hi_x = min(x1, x2), max(x1, x2)
        lo_y, hi_y = min(y1, y2), max(y1, y2)
        # a zero-length probe is a via: it touches within its own reach
        reach = self.via_reach if (x1, y1) == (x2, y2) else self.split_reach
        geom = None
        for i, r in enumerate(self.regions):
            if layer not in r.layers or r.value is None:
                continue
            bx0, by0, bx1, by1 = r.bounds
            d = reach if r.tight else 0.0
            if hi_x < bx0 - d or lo_x > bx1 + d or hi_y < by0 - d or lo_y > by1 + d:
                continue
            if geom is None:
                geom = _line(x1, y1, x2, y2)
            if r.tight:
                if r.polygon.distance(geom) < reach:
                    found = i
            elif r.polygon.intersects(geom):
                found = i
        return found

    def rows_index(self, kind: str, layer, arrays) -> np.ndarray:
        """The last region binding each foreign row (-1 for none) of one of
        the distance kernels' cached arrays: kind 'seg' (`_foreign_seg_arrays`:
        net, ax, ay, bx, by, half width -- vias folded in), 'pad'
        (`_foreign_pad_arrays`) or 'via' (`_foreign_via_arrays`). Cached on the
        identity of the arrays."""
        key = (kind, layer)
        got = self._rows.get(key)
        if got is not None and got[0] is arrays[0]:
            return got[1]
        if kind == 'seg':
            _n, ax, ay, bx, by, hw = arrays[:6]
        elif kind == 'via':
            _n, ax, ay, hw = arrays[:4]
            bx, by = ax, ay
        else:
            _n, ax, ay = arrays[0], arrays[1], arrays[2]
            bx, by = ax, ay
            ex, ey = arrays[8], arrays[9]
        out = self.index_arrays(kind, layer, ax, ay, bx, by,
                                None if kind == 'pad' else hw,
                                *((ex, ey) if kind == 'pad' else (None, None)))
        self._rows[key] = (arrays[0], out)
        return out

    def index_arrays(self, kind, layer, ax, ay, bx, by, hw=None, ex=None, ey=None):
        """`rows_index` over plain arrays, uncached: tracks (ax,ay)-(bx,by) of
        half width ``hw`` (kind 'seg' / 'via'), or pads centred at (ax, ay) of
        board-axis half extents ``ex``, ``ey`` (kind 'pad')."""
        ax, ay = np.asarray(ax, dtype=float), np.asarray(ay, dtype=float)
        bx, by = np.asarray(bx, dtype=float), np.asarray(by, dtype=float)
        n = len(ax)
        out = np.full(n, -1, dtype=np.int64)
        import shapely
        for i, r in enumerate(self.regions):
            if layer not in r.layers or n == 0 or r.value is None:
                continue
            x0, y0, x1, y1 = r.bounds
            if kind == 'pad':
                cand = ((ax + ex >= x0) & (ax - ex <= x1) & (ay + ey >= y0) & (ay - ey <= y1))
            else:
                cand = ((np.maximum(ax, bx) + hw >= x0) & (np.minimum(ax, bx) - hw <= x1)
                        & (np.maximum(ay, by) + hw >= y0) & (np.minimum(ay, by) - hw <= y1))
            idx = np.nonzero(cand)[0]
            if not idx.size:
                continue
            if kind == 'pad':
                if r.tight:      # the pad's bounding box touches it (a superset)
                    g = shapely.box(ax[idx] - ex[idx], ay[idx] - ey[idx],
                                    ax[idx] + ex[idx], ay[idx] + ey[idx])
                    hit = shapely.intersects(r.polygon, g)
                else:            # its centre is inside (a subset)
                    hit = shapely.covers(r.polygon, shapely.points(ax[idx], ay[idx]))
            else:
                g = _lines(ax[idx], ay[idx], bx[idx], by[idx])
                if r.tight:
                    hit = shapely.distance(r.polygon, g) < hw[idx] - 1e-9
                else:
                    hit = shapely.intersects(r.polygon, g)
            out[idx[hit]] = i
        return out

    def item_index(self, x1, y1, x2, y2, hw, layer) -> int:
        """`rows_index` for one foreign track (or via: a zero-length one,
        ``hw`` its radius) of half width ``hw`` on ``layer``."""
        key = (x1, y1, x2, y2, hw, layer)
        got = self._item_memo.get(key)
        if got is not None:
            return got
        if len(self._item_memo) > 200000:
            self._item_memo.clear()
        found = self._item_memo[key] = self._item_index(x1, y1, x2, y2, hw, layer)
        return found

    def _item_index(self, x1, y1, x2, y2, hw, layer) -> int:
        found = -1
        geom = None
        for i, r in enumerate(self.regions):
            if layer not in r.layers or r.value is None:
                continue
            bx0, by0, bx1, by1 = r.bounds
            if (max(x1, x2) + hw < bx0 or min(x1, x2) - hw > bx1
                    or max(y1, y2) + hw < by0 or min(y1, y2) - hw > by1):
                continue
            if geom is None:
                geom = _line(x1, y1, x2, y2)
            if (r.polygon.distance(geom) < hw - 1e-9) if r.tight else r.polygon.intersects(geom):
                found = i
        return found

    def pair_value(self, own: int, row: int, default: float, layer) -> float:
        """`pair_excess` for one pair, as a clearance: the region's value
        where one binds it, else ``default``."""
        i = max(own, row)
        if i < 0:
            return default
        r = self.regions[i]
        if self.layer_orders.get(layer, -1) > r.order:
            return default
        return r.value

    def pad_index(self, pad, layer) -> int:
        """`rows_index` for one pad (a custom pad the kernels measure apart)."""
        from shapely.geometry import Point, box
        found = -1
        hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
        if getattr(pad, 'rect_rotation', 0.0):
            hx = hy = math.hypot(hx, hy)
        for i, r in enumerate(self.regions):
            if layer not in r.layers or r.value is None:
                continue
            if r.tight:
                if r.polygon.intersects(box(pad.global_x - hx, pad.global_y - hy,
                                            pad.global_x + hx, pad.global_y + hy)):
                    found = i
            elif r.polygon.covers(Point(pad.global_x, pad.global_y)):
                found = i
        return found

    def pair_excess(self, own: int, rows, base: float, layer) -> np.ndarray:
        """Per foreign row, the clearance the pair is held to minus ``base``
        where a region binds it (own copper or the row), else NaN. ``rows``
        is a `rows_index` slice. A layer rule later in the file than the
        region's rule replaces it there (NaN: the caller's own value)."""
        rows = np.asarray(rows)
        pair = np.maximum(rows, own)
        out = np.full(len(rows), np.nan)
        lo = self.layer_orders.get(layer, -1)
        for i in np.unique(pair[pair >= 0]):
            r = self.regions[int(i)]
            if lo > r.order:
                continue
            out[pair == i] = r.value - base
        return out

    def order_of(self, idx) -> np.ndarray:
        """The file position of each region index (-1 where none)."""
        idx = np.asarray(idx)
        orders = np.array([r.order for r in self.regions] + [-1])
        return orders[np.where(idx >= 0, idx, len(self.regions))]

    def region_of_segment(self, x1, y1, x2, y2, layer, width=0.0,
                          kind: str = 'any') -> Optional[int]:
        """The region whose rule binds a track by KiCad's reading (its
        copper intersects the area), the last one in the file, or None.
        ``kind`` 'clearance' / 'width' reads only the regions that set one."""
        geom = _line(x1, y1, x2, y2)
        found = None
        for i, r in enumerate(self.regions):
            if kind == 'clearance' and r.value is None:
                continue
            if kind == 'width' and r.width is None:
                continue
            if layer in r.layers and r.polygon.distance(geom) < width / 2.0 + 1e-9:
                found = i
        return found

    def segment_clearance(self, seg, default: float) -> float:
        i = self.region_of_segment(seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                                   seg.layer, float(getattr(seg, 'width', 0.0) or 0.0),
                                   kind='clearance')
        return default if i is None else self.regions[i].value

    def via_clearance(self, via, default: float) -> float:
        """The clearance a via is held to: a region's where its disc meets
        the area on a layer the area is drawn on, else ``default`` (the
        largest such value: a via is in the area on each of those layers)."""
        from shapely.geometry import Point
        disc = Point(via.x, via.y).buffer(float(getattr(via, 'size', 0.0) or 0.0) / 2.0)
        found = None
        for r in self.regions:
            if r.value is not None and r.polygon.intersects(disc):
                found = r.value if found is None else max(found, r.value)
        return default if found is None else found

    # -------------------------------------------------------------- split
    def split_zones(self) -> Dict[str, object]:
        """{layer: the union of the tight regions' split zones on it}: where a
        run's track is split so its piece outside is not in the area."""
        if self._zones is None:
            from shapely.ops import unary_union
            by = {}
            for r in self.regions:
                if r.zone is None:
                    continue
                for l in r.layers:
                    by.setdefault(l, []).append(r.zone)
            self._zones = {l: unary_union(g) for l, g in by.items()}
        return self._zones

    def outside(self, obstacles, config):
        return _FilterProxy(obstacles, self, config, None, None)

    def inside(self, obstacles, config, want: int, other=None):
        return _FilterProxy(obstacles, self, config, want, other)


def _line(x1, y1, x2, y2):
    from shapely.geometry import LineString, Point
    return Point(x1, y1) if (x1, y1) == (x2, y2) else LineString([(x1, y1), (x2, y2)])


def _lines(ax, ay, bx, by):
    import shapely
    coords = np.stack([np.stack([ax, ay], axis=1), np.stack([bx, by], axis=1)], axis=1)
    out = shapely.linestrings(coords)
    pt = (ax == bx) & (ay == by)
    if pt.any():
        out[pt] = shapely.points(ax[pt], ay[pt])
    return out


def split_points(x1, y1, x2, y2, zone, min_piece: float = MIN_PIECE) -> List[float]:
    """The parameters t in (0, 1), sorted, where segment (x1,y1)-(x2,y2)
    crosses the boundary of ``zone`` (a shapely area), none closer than
    ``min_piece`` to an end or to each other."""
    seg_len = math.hypot(x2 - x1, y2 - y1)
    if seg_len <= 2 * min_piece or zone is None or zone.is_empty:
        return []
    from shapely.geometry import LineString
    line = LineString([(x1, y1), (x2, y2)])
    bx0, by0, bx1, by1 = zone.bounds
    if (max(x1, x2) < bx0 or min(x1, x2) > bx1 or max(y1, y2) < by0
            or min(y1, y2) > by1):
        return []
    hit = line.intersection(zone.boundary)
    if hit.is_empty:
        return []
    pts = []
    for g in getattr(hit, 'geoms', [hit]):
        pts.extend(g.coords)
    ts = sorted({min(1.0, max(0.0, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1))
                                  / (seg_len * seg_len))) for px, py in pts})
    out = []
    for t in ts:
        if t * seg_len < min_piece or (1.0 - t) * seg_len < min_piece:
            continue
        if out and (t - out[-1]) * seg_len < min_piece:
            continue
        out.append(t)
    return out


def split_segment(seg, ts):
    """``seg`` cut at parameters ``ts`` (sorted), as copies in order; the
    pieces meet at bit-identical points, so their joints are exact."""
    if not ts:
        return [seg]
    x1, y1, x2, y2 = seg.start_x, seg.start_y, seg.end_x, seg.end_y
    verts = [(x1, y1)] + [(x1 + t * (x2 - x1), y1 + t * (y2 - y1)) for t in ts] + [(x2, y2)]
    out = []
    for (ax, ay), (bx, by) in zip(verts, verts[1:]):
        p = _copy.copy(seg)
        p.start_x, p.start_y, p.end_x, p.end_y = ax, ay, bx, by
        if hasattr(p, 'uuid'):
            p.uuid = ''
        for f in ('start_x_str', 'start_y_str', 'end_x_str', 'end_y_str'):
            if hasattr(p, f):
                setattr(p, f, '')
        out.append(p)
    return out


def split_segments_at_zones(segments, zones_by_layer, skip=None,
                            margin_of=None, min_piece: float = MIN_PIECE):
    """Split every segment where it crosses the boundary of its layer's zone
    (``zones_by_layer``: {layer: shapely area}). ``skip(seg)`` True leaves a
    segment whole; ``margin_of(seg)`` (optional) grows the zone for that
    segment by the given distance. Returns ``[(host, pieces), ...]`` for the
    segments that were split. Reusable by any pass that must keep a track's
    pieces on either side of an area apart (#1843)."""
    out = []
    for s in segments:
        zone = zones_by_layer.get(getattr(s, 'layer', None))
        if zone is None or getattr(s, 'graphic', False) or (skip is not None and skip(s)):
            continue
        if margin_of is not None:
            m = margin_of(s)
            if m > 0:
                zone = zone.buffer(m)
        ts = split_points(s.start_x, s.start_y, s.end_x, s.end_y, zone, min_piece)
        if ts:
            out.append((s, split_segment(s, ts)))
    return out


def split_run_copper(area, segments, skip=None):
    """`split_segments_at_zones` over the area's split zones: a track wider
    than the run's widest (2 x (split_reach - SPLIT_MARGIN)) is split at its
    own half width plus the margin instead."""
    zones = area.split_zones()
    if not zones:
        return []

    def _margin(s):
        return max(0.0, float(s.width) / 2.0 + SPLIT_MARGIN - area.split_reach)
    return split_segments_at_zones(segments, zones, skip=skip, margin_of=_margin)


def split_results(results, pcb_data, config, scope_net_ids=None) -> int:
    """Split the routed copper of ``results`` (their `new_segments`, which
    are the same objects as in ``pcb_data.segments``) at the tight areas'
    split zones, in both lists. Locked and sense-held copper stays whole.
    Returns the number of tracks split; 0 (and nothing touched) without a
    tight area."""
    area = getattr(config, 'area_clearance', None)
    if area is None or not area.split_zones():
        return 0
    try:
        from kicad_parser import is_immutable_copper
    except Exception:                                           # noqa: BLE001
        def is_immutable_copper(_s):
            return bool(getattr(_s, 'locked', False))
    scope = set(scope_net_ids) if scope_net_ids is not None else None
    segs = [s for r in results for s in (r.get('new_segments') or [])
            if (scope is None or s.net_id in scope)]
    pairs = split_run_copper(area, segs, skip=is_immutable_copper)
    if not pairs:
        return 0
    pieces = {id(h): ps for h, ps in pairs}
    for r in results:
        if r.get('new_segments'):
            r['new_segments'] = [p for x in r['new_segments'] for p in pieces.get(id(x), [x])]
    pcb_data.segments = [p for x in pcb_data.segments for p in pieces.get(id(x), [x])]
    if hasattr(pcb_data, '_foreign_seg_arr_cache'):
        pcb_data._foreign_seg_arr_cache = None
    return len(pairs)


class _FilterProxy:
    """A map that keeps one side of the composition of every keep-out row
    added to or removed from it. ``other`` (a sub-build's own map) takes the
    calls that are not keep-out rows or questions about the map."""
    __slots__ = ('_real', '_area', '_config', '_want', '_other')

    def __init__(self, real, area, config, want, other):
        object.__setattr__(self, '_real', real)
        object.__setattr__(self, '_area', area)
        object.__setattr__(self, '_config', config)
        object.__setattr__(self, '_want', want)
        object.__setattr__(self, '_other', other)

    def __getattr__(self, name):
        spec = _ROW_METHODS.get(name)
        if spec is not None:
            kind, at = spec
            fn = getattr(self._real, name)

            def rows(*args):
                kept = self._area.keep(kind, args[at], self._config, self._want)
                if len(kept):
                    fn(*args[:at], kept, *args[at + 1:])
            return rows
        spec = _POINT_METHODS.get(name)
        if spec is not None:
            kind, at = spec
            fn = getattr(self._real, name)

            def point(*args):
                layer = args[at + 2] if kind == 'cell' else None
                if self._area.keep_point(kind, args[at], args[at + 1], layer,
                                         self._config, self._want):
                    fn(*args)
            return point
        if self._other is not None and not name.startswith(_QUERY_PREFIXES):
            return getattr(self._other, name)
        return getattr(self._real, name)


def composed(fn):
    """A stamp (or unstamp) of clearance-priced copper, composed over the
    board's area rules: once at the class values outside the areas, once per
    area at its rule's value inside. Without an area rule the call is the
    function itself."""
    names = list(inspect.signature(fn).parameters)
    at = names.index('config')

    @wraps(fn)
    def stamp(obstacles, *args, **kwargs):
        config = kwargs['config'] if 'config' in kwargs else args[at - 1]
        area = getattr(config, 'area_clearance', None)
        if area is None:
            return fn(obstacles, *args, **kwargs)

        def call(target, cfg):
            if 'config' in kwargs:
                return fn(target, *args, **dict(kwargs, config=cfg))
            return fn(target, *args[:at - 1], cfg, *args[at:], **kwargs)
        out = call(area.outside(obstacles, config), area.outside_config(config))
        for i in range(len(area.regions)):
            call(area.inside(obstacles, config, i), area.region_config(config, i))
        return out
    return stamp


def composed_builder(layers_of=None, via_band: bool = False):
    """A BUILDER that makes its own map (`obstacles = _into or
    GridObstacleMap(n)`, so it must take an ``_into`` keyword), composed
    over the board's area rules as `composed` composes a stamp: run once at
    the class values into the real map through the outside filter, then once
    per area at its rule's value through that area's filter. The area passes
    send every call that is not a keep-out row (cost fields, zones, source
    cells) to a scratch map, so only the keep-outs compose. ``layers_of``
    (bound arguments -> layer names) names the map's layers when they are
    not ``config.layers`` (a one-layer routing map, a plane connector's
    routing layers). Without an area rule the call is the builder itself."""
    def deco(fn):
        sig = inspect.signature(fn)

        @wraps(fn)
        def build(*args, **kwargs):
            b = sig.bind(*args, **kwargs)
            b.apply_defaults()
            config = b.arguments['config']
            area = getattr(config, 'area_clearance', None)
            if area is None or b.arguments.get('_into') is not None:
                return fn(*args, **kwargs)
            from obstacle_map import GridObstacleMap
            names = list(layers_of(b.arguments) if layers_of else config.layers)
            fcfg = config if names == list(config.layers) else replace(config, layers=names)
            real = GridObstacleMap(len(names))

            def call(target, cfg, quiet):
                a = dict(b.arguments)
                a['config'], a['_into'] = cfg, target
                if quiet and 'verbose' in a:
                    a['verbose'] = False
                return fn(**a)
            out = call(area.outside(real, fcfg), area.outside_config(config), False)
            for i in range(len(area.regions)):
                call(area.inside(real, fcfg, i, other=GridObstacleMap(len(names))),
                     area.region_config(config, i), True)
            if via_band:
                band = area.via_band_cells(config.grid_step)
                if len(band):
                    real.add_blocked_vias_batch(band)
            return (real,) + tuple(out[1:]) if isinstance(out, tuple) else real
        return build
    return deco


def compose_route_stamp(fn, obstacles, config, layers, clearance, **kwargs):
    """`fn(obstacles, ..., clearance=..., config=...)` -- a stamp priced at an
    explicit ``clearance`` (a plane connector's routed connection) -- composed
    over the area rules: ``clearance`` outside the areas, each area's rule
    value inside it. ``layers`` names the map's layer indices."""
    area = getattr(config, 'area_clearance', None)
    if area is None:
        return fn(obstacles, clearance=clearance, config=config, **kwargs)
    names = list(layers)
    fcfg = config if names == list(config.layers) else replace(config, layers=names)
    fn(area.outside(obstacles, fcfg), clearance=clearance,
       config=area.outside_config(config), **kwargs)
    for i, r in enumerate(area.regions):
        fn(area.inside(obstacles, fcfg, i),
           clearance=clearance if r.value is None else r.value,
           config=area.region_config(config, i), **kwargs)


def compose_via_placement(area, config, layers, outer, inner):
    """#1843: `obstacle_cache.ViaPlacementObstacleData` composed as
    `build_via_obstacle_map` / `build_routing_obstacle_map` compose their
    stamps: ``outer`` at the class values, ``inner[i]`` at region i's value
    (None: no row of it reaches the region). ``layers`` are the layers of
    its `blocked_cells_by_layer`."""
    names = list(layers)
    fcfg = config if names == list(config.layers) else replace(config, layers=names)
    outer.blocked_vias = compose_rows(area, fcfg, 'vias', outer.blocked_vias,
                                      [d.blocked_vias if d is not None else None
                                       for d in inner])
    for k, L in enumerate(names):
        def _cells(arr):
            if arr is None or not len(arr):
                return None
            a = np.asarray(arr, dtype=np.int32).reshape(-1, 2)
            return np.column_stack([a, np.full(len(a), k, dtype=np.int32)])
        rows = compose_rows(area, fcfg, 'cells', _cells(outer.blocked_cells_by_layer.get(L)),
                            [_cells(d.blocked_cells_by_layer.get(L)) if d is not None else None
                             for d in inner])
        outer.blocked_cells_by_layer[L] = np.ascontiguousarray(rows[:, :2])
    return outer


def pair_gaps_below(area, pcb_data, pairs) -> List[dict]:
    """#1843: the routed diff pairs whose OWN P-to-N gap is below a tightened
    rule area's value where either member's copper is in the area. KiCad's
    custom rule outranks the pair's implicit gap rule there, so it grades
    P against N at the area's value, and a coupled pair is laid at one gap,
    which the composition cannot widen inside the area. ``pairs`` is
    `[(name, pair with p_net_id / n_net_id)]`. One row per pair and area:
    {pair, rule, area, value, gap}."""
    if area is None or not any(r.tight for r in area.regions):
        return []
    from geometry_utils import segment_to_segment_distance
    out = []
    for name, pair in pairs:
        p = [x for x in pcb_data.segments if x.net_id == pair.p_net_id]
        n = [x for x in pcb_data.segments if x.net_id == pair.n_net_id]
        for i, r in enumerate(area.regions):
            if not r.tight:
                continue
            worst = None
            for a in p:
                ai = area.item_index(a.start_x, a.start_y, a.end_x, a.end_y,
                                     a.width / 2.0, a.layer)
                for b in n:
                    if b.layer != a.layer:
                        continue
                    if ai != i and area.item_index(b.start_x, b.start_y, b.end_x, b.end_y,
                                                   b.width / 2.0, b.layer) != i:
                        continue
                    g = segment_to_segment_distance(
                        a.start_x, a.start_y, a.end_x, a.end_y,
                        b.start_x, b.start_y, b.end_x, b.end_y) - (a.width + b.width) / 2.0
                    if worst is None or g < worst:
                        worst = g
            if worst is not None and worst < r.value - 1e-4:
                out.append({'pair': name, 'rule': r.rule, 'area': r.area,
                            'value': r.value, 'gap': round(worst, 4)})
    return out


def compose_rows(area, config, kind, outer, inner_by_region):
    """One record of rows (``kind``) from the stamp at the class values
    (``outer``) and the stamps at each region's value."""
    parts = []
    if outer is not None and len(outer):
        parts.append(area.keep(kind, outer, config, None))
    for i, rows in enumerate(inner_by_region):
        if rows is not None and len(rows):
            parts.append(area.keep(kind, rows, config, i))
    parts = [p for p in parts if len(p)]
    if not parts:
        width = {'cells': 3, 'cell_spans': 4, 'vias': 2, 'via_spans': 3}[kind]
        return np.empty((0, width), dtype=np.int32)
    return np.ascontiguousarray(parts[0] if len(parts) == 1 else np.concatenate(parts))


def _raster(polygon, step: float):
    """(gx0, gy0, mask[gx - gx0, gy - gy0]) of the grid cells whose centre
    lies inside ``polygon``."""
    import shapely
    x0, y0, x1, y1 = polygon.bounds
    gx0, gy0 = int(np.floor(x0 / step)), int(np.floor(y0 / step))
    gx1, gy1 = int(np.ceil(x1 / step)), int(np.ceil(y1 / step))
    xs = np.arange(gx0, gx1 + 1) * step
    ys = np.arange(gy0, gy1 + 1) * step
    mask = np.empty((len(xs), len(ys)), dtype=bool)
    rows = max(1, 2_000_000 // max(1, len(ys)))
    for a in range(0, len(xs), rows):
        X, Y = np.meshgrid(xs[a:a + rows], ys, indexing='ij')
        mask[a:a + rows] = shapely.contains_xy(polygon, X, Y)
    return gx0, gy0, mask


def _lookup(mask, gx, gy):
    gx0, gy0, arr = mask
    ix = np.asarray(gx, dtype=np.int64) - gx0
    iy = np.asarray(gy, dtype=np.int64) - gy0
    ok = (ix >= 0) & (ix < arr.shape[0]) & (iy >= 0) & (iy < arr.shape[1])
    out = np.zeros(len(ix), dtype=bool)
    out[ok] = arr[ix[ok], iy[ok]]
    return out


def _area_names(ast):
    """The areas a condition names when it is one insideArea /
    intersectsArea predicate or an OR of them, else None."""
    if not isinstance(ast, tuple) or not ast:
        return None
    if (len(ast) == 4 and ast[0] == 'area'
            and ast[2] in ('insidearea', 'intersectsarea')):
        return [ast[3]]
    if ast[0] == 'or':
        out = []
        for part in ast[1:]:
            got = _area_names(part)
            if got is None:
                return None
            out += [n for n in got if n not in out]
        return out or None
    return None


def _fab_floor(copper, key: str) -> float:
    """The floor an area rule's value is raised to: the one every other
    rule channel uses (`fab_tiers.rule_fab_floors`, as the class/type-scoped
    rules and `DesignRules` read it, #1630), so a room at a value routes as
    the same value in a layer or class rule would, and a hard `--fab-tier`
    holds it."""
    try:
        from fab_tiers import rule_fab_floors
        return float(rule_fab_floors(len(copper) or 2).get(key) or 0.0)
    except Exception:                                           # noqa: BLE001
        return 0.0


KINDS = ('clearance', 'track_width')


def _rule_value(rule, kind) -> Optional[float]:
    c = rule.constraints.get(kind) or {}
    if kind == 'clearance':
        return c.get('min')
    return c.get('opt', c.get('min'))


def eligible(rules, pcb_data):
    """(the board's area index, [(rule, kind, area names, None) | (rule,
    kind, None, why it is not routed to)]) for the clearance and track_width
    rules scoped by a rule area the board has."""
    cands = [(r, k) for r in rules.rules for k in KINDS
             if getattr(r, 'area_scoped_condition', None) is not None
             and _rule_value(r, k) is not None]
    if not cands:
        return None, []
    index = rules.area_index(pcb_data)
    if index is None:
        return None, []
    out = []
    for r, kind in cands:
        ast = r.area_scoped_condition
        if ast is None:            # names an area the board does not have
            continue
        names = _area_names(ast)
        if names is None:
            out.append((r, kind, None, 'its condition is more than rule-area predicates'))
            continue
        if kind == 'track_width':
            # a later width rule that wants a WIDER track than the room's
            # would be broken by narrowing the nets it binds
            v = _rule_value(r, kind)
            later = [o.name for o in rules.rules
                     if o.order > r.order and o.unsupported is None
                     and ((o.constraints.get('track_width') or {}).get('min') or 0.0) > v]
            if later:
                out.append((r, kind, None,
                            f"track_width rule '{later[0]}' after it asks for a wider track"))
                continue
        out.append((r, kind, names, None))
    return index, out


def mark_honoured(rules, pcb_data, config=None) -> None:
    """Name, for the rules report, the area rules a routing step routes to
    (`Rule.honoured`). A rule is named only when its region was actually
    built: the report must not say "routed to" for a setup that failed, so
    the regions are derived here, as `install` derives them, and a failure
    names nothing (the report then reads NOT honoured, which is what the run
    will do)."""
    from types import SimpleNamespace
    probe = config if config is not None else SimpleNamespace(
        clearance=0.0, layer_clearances={}, track_width=0.0, via_size=0.0,
        grid_step=0.1, layers=list(getattr(getattr(pcb_data, 'board_info', None),
                                           'copper_layers', None) or []))
    try:
        _index, picked = eligible(rules, pcb_data)
        built = derive(rules, pcb_data, probe)
    except Exception as e:                                      # noqa: BLE001
        print(f"  ERROR: the board's area rules could not be set up "
              f"({type(e).__name__}: {e}); this run routes WITHOUT them")
        return
    regions = built.regions if built is not None else []
    copper = list(getattr(getattr(pcb_data, 'board_info', None), 'copper_layers', None) or [])
    said = {}
    for r, kind, names, _why in picked:
        if names is None:
            continue
        # the region of one of the rule's areas carries this kind's value
        if not any(reg.area in names
                   and (reg.value if kind == 'clearance' else reg.width) is not None
                   for reg in regions):
            continue
        v = float(_rule_value(r, kind))
        floor = _fab_floor(copper, kind)
        pinned = (f" (below the fab floor: {floor:g} mm is used; --fab-tier / "
                  f"--fab-overrides set the floor)" if v < floor - 1e-9 else '')
        said.setdefault(id(r), (r, names, []))[2].append(
            f"clearance min {v:g} mm: tracks and vias are routed to it there{pinned}"
            if kind == 'clearance' else
            f"track_width {v:g} mm: a track no wider than the run's may neck to it "
            f"there{pinned}")
    for r, names, parts in said.values():
        if not r.honoured:
            where = ', '.join(f"'{n}'" for n in names)
            r.honoured = f"in rule area {where}, " + '; '.join(parts)


def _layer_orders(rules, copper) -> dict:
    """{layer: file position of the rule its layer clearance comes from}."""
    try:
        import os
        from kicad_dru import _parse_dru_full
        src = getattr(rules, 'source', '') or ''
        dru = os.path.splitext(src)[0] + '.kicad_dru' if src else ''
        if dru and os.path.isfile(dru):
            with open(dru, encoding='utf-8') as fh:
                return dict(_parse_dru_full(fh.read(), list(copper))[4])
    except Exception:                                           # noqa: BLE001
        pass
    return {}


def _run_sizes(config):
    """(widest track, largest via) the run may draw: the config's own and
    every per-net / per-layer width and via it carries."""
    tw = [float(getattr(config, 'track_width', 0.0) or 0.0)]
    for name in ('power_net_widths', 'net_track_widths', 'layer_widths',
                 'coplanar_layer_widths'):
        tw += [float(v) for v in (getattr(config, name, None) or {}).values() if v]
    for d in (getattr(config, 'net_layer_widths', None) or {}).values():
        tw += [float(v) for v in (d or {}).values() if v]
    vs = [float(getattr(config, 'via_size', 0.0) or 0.0)]
    vs += [float(v[0]) for v in (getattr(config, 'net_via_sizes', None) or {}).values()
           if v and v[0]]
    return max(tw), max(vs)


def _laid_copper_extensions(regions, pcb_data, copper, hold, inner):
    """{region index: {layer: shape}}: per TIGHT region, the laid copper
    (pads, tracks, vias) on each of its layers whose shape touches the area
    and reaches further from it than ``inner``, grown by ``hold`` (the
    rule's value plus the reach of the copper routed past it, set per region
    by the caller as hold[i]). KiCad binds such an item along its whole
    extent, so the ring is widened over it."""
    from shapely.geometry import LineString, Point, box
    from shapely import affinity
    from shapely.ops import unary_union
    try:
        from net_queries import expand_pad_layers
    except Exception:                                           # noqa: BLE001
        expand_pad_layers = None
    tight = [(i, r) for i, r in enumerate(regions) if r.tight]
    if not tight or pcb_data is None:
        return {}
    items = []                                     # (layers, shape)
    for pads in (getattr(pcb_data, 'pads_by_net', None) or {}).values():
        for pad in pads:
            if getattr(pad, 'pad_type', '') == 'np_thru_hole':
                continue
            hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
            g = box(pad.global_x - hx, pad.global_y - hy, pad.global_x + hx, pad.global_y + hy)
            rot = getattr(pad, 'rect_rotation', 0.0) or 0.0
            if rot:
                g = affinity.rotate(g, rot, origin=(pad.global_x, pad.global_y))
            lay = (expand_pad_layers(pad.layers, copper) if expand_pad_layers
                   else [l for l in pad.layers if l in copper])
            items.append((set(lay), g))
    for s in getattr(pcb_data, 'segments', None) or []:
        w = float(getattr(s, 'width', 0.0) or 0.0)
        if (s.start_x, s.start_y) == (s.end_x, s.end_y):
            g = Point(s.start_x, s.start_y).buffer(w / 2.0)
        else:
            g = LineString([(s.start_x, s.start_y), (s.end_x, s.end_y)]).buffer(w / 2.0)
        items.append(({s.layer}, g))
    for v in getattr(pcb_data, 'vias', None) or []:
        items.append((set(copper), Point(v.x, v.y).buffer(float(v.size) / 2.0)))
    out = {}
    for i, r in tight:
        x0, y0, x1, y1 = r.bounds
        limit = r.polygon.buffer(inner)
        per = {}
        for lay, g in items:
            on = lay & r.layers
            if not on:
                continue
            gx0, gy0, gx1, gy1 = g.bounds
            if gx1 < x0 or gx0 > x1 or gy1 < y0 or gy0 > y1:
                continue
            if not r.polygon.intersects(g) or limit.contains(g):
                continue
            grown = g.buffer(hold[i])
            for l in on:
                per.setdefault(l, []).append(grown)
        if per:
            out[i] = {l: unary_union(gs) for l, gs in per.items()}
    return out


def derive(rules, pcb_data, config) -> Optional[AreaClearance]:
    """The area rules the board declares, or None when none is routed to.
    ``rules`` is the board's `DesignRules`. Rules on one area make one
    region; the later rule's value wins per kind."""
    if rules is None or pcb_data is None:
        return None
    index, picked = eligible(rules, pcb_data)
    if not picked:
        return None
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or config.layers)
    base = min([config.clearance] + list((config.layer_clearances or {}).values()))
    tw_max, vs_max = _run_sizes(config)
    reach = max(tw_max, vs_max) / 2.0
    # a run's track piece within split_reach of a tight area is in it; the
    # rest is split off (SPLIT_MARGIN past the round cap)
    split_reach = tw_max / 2.0 + SPLIT_MARGIN
    via_reach = vs_max / 2.0 + SPLIT_MARGIN
    # the furthest the copper of anything the run lays touching a tight area
    # reaches past it: a track piece's far edge, split_reach plus its half
    # width (no via of the run straddles the edge: `via_band_cells`)
    inner = split_reach + tw_max / 2.0
    found, skipped = {}, []
    for r, kind, names, why in picked:
        if names is None:
            skipped.append((r.name, f"{kind}: {why}"))
            continue
        on = r.layers_for(copper)
        value = max(float(_rule_value(r, kind)), _fab_floor(copper, kind))
        for k, entry in enumerate(index.areas):
            # (name, layers, polygon, ...): the index may carry more per area
            aname, alayers, poly = entry[0], entry[1], entry[2]
            if aname not in names:
                continue
            layers = frozenset(alayers if on is None else set(alayers) & set(on))
            if not layers:
                continue
            reg = found.get((k, layers))
            if reg is None:
                reg = found[(k, layers)] = Region(r.name, aname, None, poly, layers, 0.0)
            if kind == 'clearance':
                reg.value, reg.order = value, r.order
                # the ring holds, at the rule's value, the stamp of anything
                # the run lays touching the area: reach past the copper
                # itself, then the value and the half size of what is routed
                # past it
                reg.ring = ((inner + value + reach + config.grid_step)
                            if value > base + 1e-9 else 0.0)
            else:
                reg.width = value
    if not found and not skipped:
        return None
    regions = list(found.values())
    hold = [((r.value + reach + config.grid_step) if r.value is not None else 0.0)
            for r in regions]
    ext = _laid_copper_extensions(regions, pcb_data, copper, hold, inner)
    return AreaClearance(regions, copper, skipped, _layer_orders(rules, copper),
                         split_reach=split_reach, extensions=ext, via_reach=via_reach)


_DERIVED = {}


def _derive_key(rules, pcb_data, config):
    """What `derive` reads: the rules and board objects, and the config
    fields that size the regions."""
    tw, vs = _run_sizes(config)
    return (id(rules), id(pcb_data), round(float(config.clearance), 9),
            tuple(sorted((config.layer_clearances or {}).items())),
            round(tw, 9), round(vs, 9), round(float(config.grid_step), 9),
            tuple(getattr(config, 'layers', None) or ()))


class _RulesOnly:
    """The rule list of a `.kicad_dru`, as `eligible` reads a DesignRules."""

    def __init__(self, rules, copper_layers=()):
        self.rules = rules
        # what `AreaIndex` reads off a DesignRules besides the rules
        self.copper_layers = list(copper_layers)

    def area_index(self, pcb_data):
        from design_rules import AreaIndex
        return AreaIndex(self, pcb_data)


def room_width_test(pcb_data=None, *, board_path=None, copper=None):
    """`held(x1, y1, x2, y2, layer, width)`: is this track bound by a
    room's track_width rule that allows its width? Such a track is graded at
    the rule, not at Board Setup's minimum (measured on kicad-cli 10.0.8),
    so it says nothing about the board's `min_track_width`. None when the
    board has no such rule.

    Read from the rules file and the board's rule areas alone -- from
    ``pcb_data``, or (the GUI's live scan, which must not parse a board)
    from the text of ``board_path`` with the board's ``copper`` layers."""
    try:
        import os
        from types import SimpleNamespace
        from design_rules import parse_dru
        path = board_path or getattr(pcb_data, 'source_path', '') or ''
        dru = os.path.splitext(path)[0] + '.kicad_dru' if path else ''
        if not dru or not os.path.isfile(dru):
            return None
        with open(dru, encoding='utf-8') as fh:
            text = fh.read()
        if 'track_width' not in text or 'Area(' not in text:
            return None
        if pcb_data is None:
            from kicad_parser import extract_keepouts
            with open(path, encoding='utf-8') as fh:
                pcb_data = SimpleNamespace(board_info=SimpleNamespace(
                    keepouts=extract_keepouts(fh.read()), copper_layers=list(copper or [])))
        copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or copper or [])
        if not copper:
            return None
        rules, _notes = parse_dru(text)
        area = derive(_RulesOnly(rules, copper), pcb_data, SimpleNamespace(
            clearance=0.0, layer_clearances={}, track_width=0.0, via_size=0.0,
            grid_step=0.1, layers=copper))
    except Exception:                                           # noqa: BLE001
        return None
    rooms = [r for r in (area.regions if area is not None else []) if r.width is not None]
    if not rooms:
        return None
    from shapely.geometry import LineString, Point

    def held(x1, y1, x2, y2, layer, width) -> bool:
        geom = Point(x1, y1) if (x1, y1) == (x2, y2) else LineString([(x1, y1), (x2, y2)])
        return any(layer in r.layers and width >= r.width - 1e-9
                   and r.polygon.intersects(geom) for r in rooms)
    return held


def narrow_tracks(segments, config, net_id, pcb_data=None) -> int:
    """Neck each of ``segments`` to a room's track width where it needs it
    (in place; a segment is split where its width changes). The part of a
    track inside a room is drawn at the room's width, and then, given the
    board (``pcb_data``), every stretch of it where the net's own width
    keeps the room's clearance is put back to that width -- so a track is
    narrow between the balls it squeezes past and full width elsewhere.
    A net routed wider than the run's reserve width keeps its width
    throughout: its search margin covers it in the room too. Returns the
    pieces left narrow. Safe to call again on its own output."""
    area = getattr(config, 'area_clearance', None)
    if area is None or not segments:
        return 0
    rooms = [r for r in area.regions if r.width is not None]
    if not rooms:
        return 0
    import copy
    from shapely.geometry import LineString, Point
    reach = 1.5 * config.grid_step
    out, n = [], 0
    for seg in segments:
        todo = [seg]
        if not getattr(seg, 'graphic', False) and (
                config.get_net_track_width(net_id, seg.layer)
                <= config.route_reserve_width(seg.layer) + 1e-9):
            for r in rooms:
                if seg.layer not in r.layers:
                    continue
                nxt = []
                for s in todo:
                    if s.width <= r.width + 1e-9:
                        nxt.append(s)
                        continue
                    a, b = (s.start_x, s.start_y), (s.end_x, s.end_y)
                    length = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
                    if length < 1e-6:
                        if r.polygon.intersects(Point(a)):
                            s.width = r.width
                            n += 1
                        nxt.append(s)
                        continue
                    line = LineString([a, b])
                    cut = line.intersection(r.polygon.buffer(reach))
                    parts = [g for g in getattr(cut, 'geoms', [cut])
                             if g.geom_type == 'LineString' and g.length > 1e-6
                             and g.intersects(r.polygon)]
                    spans = sorted((line.project(Point(g.coords[0])),
                                    line.project(Point(g.coords[-1]))) for g in parts)
                    spans = [(min(t0, t1), max(t0, t1)) for t0, t1 in spans]
                    if not spans:
                        nxt.append(s)
                        continue
                    marks, t = [], 0.0
                    for t0, t1 in spans:
                        if t0 - t > 1e-6:
                            marks.append((t, t0, False))
                        marks.append((max(t, t0), t1, True))
                        t = t1
                    if length - t > 1e-6:
                        marks.append((t, length, False))
                    for k, (t0, t1, narrow) in enumerate(marks):
                        piece = s if k == 0 else copy.copy(s)
                        if k and hasattr(piece, 'uuid'):
                            piece.uuid = ''
                        p0 = a if t0 <= 1e-9 else (
                            round(a[0] + (b[0] - a[0]) * t0 / length, 6),
                            round(a[1] + (b[1] - a[1]) * t0 / length, 6))
                        p1 = b if length - t1 <= 1e-9 else (
                            round(a[0] + (b[0] - a[0]) * t1 / length, 6),
                            round(a[1] + (b[1] - a[1]) * t1 / length, 6))
                        piece.start_x, piece.start_y = p0
                        piece.end_x, piece.end_y = p1
                        if narrow:
                            piece.width = r.width
                            n += 1
                        nxt.append(piece)
                todo = nxt
        out.extend(todo)
    segments[:] = out
    if pcb_data is not None:
        return _widen_where_fits(segments, config, net_id, pcb_data, area)
    return n


def _widen_where_fits(segments, config, net_id, pcb_data, area) -> int:
    """Put the narrow pieces of a room back to the net's width wherever
    that width keeps the room's clearance to every neighbour, with half a
    grid step to spare (and the board's copper-to-hole floor where it is
    the larger). Returns the pieces still narrow."""
    import copy
    import math
    from single_ended_routing import _terminal_room
    try:
        from obstacle_map import resolve_hole_clearance
        hole = float(resolve_hole_clearance(pcb_data, config) or 0.0)
    except Exception:                                           # noqa: BLE001
        hole = 0.0
    step = max(4.0 * config.grid_step, 0.2)
    out, narrow = [], 0
    for seg in segments:
        full = config.get_net_track_width(net_id, seg.layer)
        room = None
        if not getattr(seg, 'graphic', False) and seg.width < full - 1e-9:
            for i, r in enumerate(area.regions):
                if (r.width is not None and seg.layer in r.layers
                        and abs(seg.width - r.width) < 1e-9
                        and area.region_of_segment(seg.start_x, seg.start_y, seg.end_x,
                                                   seg.end_y, seg.layer) is not None):
                    room = i
        a, b = (seg.start_x, seg.start_y), (seg.end_x, seg.end_y)
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if room is None or length < 1e-6:
            narrow += int(room is not None)
            out.append(seg)
            continue
        cfg = area.region_config(config, room)
        k = max(1, int(math.ceil(length / step)))
        pts = [a] + [(round(a[0] + (b[0] - a[0]) * j / k, 6),
                      round(a[1] + (b[1] - a[1]) * j / k, 6)) for j in range(1, k)] + [b]
        pieces = []
        for j in range(k):
            probe = copy.copy(seg)
            probe.start_x, probe.start_y = pts[j]
            probe.end_x, probe.end_y = pts[j + 1]
            probe.width = full
            try:
                own, d, edge_room, _sd = _terminal_room(probe, (), pcb_data, net_id, cfg)
                slack = config.grid_step / 2.0 + max(0.0, hole - own)
                fits = (d - full / 2.0 >= own + slack
                        and edge_room - full / 2.0 >= slack)
            except Exception:                                   # noqa: BLE001
                fits = False
            w = full if fits else seg.width
            if pieces and abs(pieces[-1][2] - w) < 1e-9:
                pieces[-1] = (pieces[-1][0], pts[j + 1], w)
            else:
                pieces.append((pts[j], pts[j + 1], w))
        for j, (p0, p1, w) in enumerate(pieces):
            piece = seg if j == 0 else copy.copy(seg)
            if j and hasattr(piece, 'uuid'):
                piece.uuid = ''
            piece.start_x, piece.start_y = p0
            piece.end_x, piece.end_y = p1
            piece.width = w
            narrow += int(w < full - 1e-9)
            out.append(piece)
    segments[:] = out
    return narrow


def install(config, pcb_data) -> None:
    """Read the board's area clearance rules onto ``config`` (both fronts,
    from `kicad_dru.install_layer_clearances`). Never raises into an engine."""
    rules = getattr(config, 'rules', None)
    key = _derive_key(rules, pcb_data, config)
    held = getattr(config, 'area_clearance', None)
    if held is not None and getattr(held, '_key', None) == key:
        return          # installed already: the maps built on it must keep it
    try:
        got = _DERIVED.get(key)
        if got is not None and got[0] is rules and got[1] is pcb_data:
            area = got[2]
        else:
            area = derive(rules, pcb_data, config)
            if area is not None:
                area._key = key
            if len(_DERIVED) > 8:
                _DERIVED.clear()
            _DERIVED[key] = (rules, pcb_data, area)
    except Exception as e:                                      # noqa: BLE001
        print(f"  ERROR: the board's area clearance rules could not be set up "
              f"({type(e).__name__}: {e}); this run routes WITHOUT them")
        area = None
    # every rule the report named as routed to must have its region here
    named = {r.name for r in (getattr(rules, 'rules', None) or [])
             if 'tracks and vias are routed to it there' in (r.honoured or '')}
    missing = sorted(named) if not (area and area.regions) else []
    if missing:
        print(f"  ERROR: area clearance rule(s) {', '.join(repr(n) for n in missing)} "
              f"were reported as routed to but have no region in this step; it "
              f"routes WITHOUT them")
    if area is not None:
        key = (getattr(pcb_data, 'source_path', '') or '', tuple(area.skipped))
        if area.skipped and key not in _ANNOUNCED:
            _ANNOUNCED.add(key)
            for name, why in area.skipped:
                kind, _, why = why.partition(': ')
                print(f"  .kicad_dru rule '{name}': its area {kind} is NOT routed "
                      f"to ({why}); check_drc grades it")
        if not area.regions:
            area = None
    config.area_clearance = area
