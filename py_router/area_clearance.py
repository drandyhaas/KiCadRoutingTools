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
    """One area rule the router honours. ``ring`` > 0 marks a TIGHT region
    (its rule is above the run's base clearance)."""
    __slots__ = ('rule', 'area', 'value', 'polygon', 'layers', 'ring', 'order',
                 'zone', 'bounds')

    def __init__(self, rule, area, value, polygon, layers, ring, order=0):
        self.rule, self.area, self.value = rule, area, value
        self.polygon, self.layers, self.ring = polygon, layers, ring
        self.order = order        # the rule's position in the file
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
        rule's value in place of every class value and of every rule BEFORE
        it in the file. KiCad's last matching rule wins, so a layer rule or a
        class/net-scoped rule that comes after the area's still replaces it
        for what it binds; the track-to-track raises stay as they are (they
        only widen)."""
        r = self.regions[want]
        layers = {l: v for l, v in (config.layer_clearances or {}).items()
                  if self.layer_orders.get(l, -1) > r.order}
        return replace(config, area_clearance=None, clearance=r.value,
                       net_clearances={}, net_clearance_floor=None,
                       layer_clearances=layers,
                       scoped_clearance=self._scoped_after(config, r.order))

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
        vals = [float(config.clearance)] + [r.value for r in self.regions]
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
            if layer not in r.layers:
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
            if layer not in r.layers or n == 0:
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
            if layer not in r.layers:
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
            if layer not in r.layers:
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

    def region_of_segment(self, x1, y1, x2, y2, layer, width=0.0) -> Optional[int]:
        """The region whose rule binds a track by KiCad's reading (its
        copper intersects the area), the last one in the file, or None."""
        geom = _line(x1, y1, x2, y2)
        found = None
        for i, r in enumerate(self.regions):
            if layer in r.layers and r.polygon.distance(geom) < width / 2.0 + 1e-9:
                found = i
        return found

    def segment_clearance(self, seg, default: float) -> float:
        i = self.region_of_segment(seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                                   seg.layer, float(getattr(seg, 'width', 0.0) or 0.0))
        return default if i is None else self.regions[i].value

    def via_clearance(self, via, default: float) -> float:
        """The clearance a via is held to: a region's where its disc meets
        the area on a layer the area is drawn on, else ``default`` (the
        largest such value: a via is in the area on each of those layers)."""
        from shapely.geometry import Point
        disc = Point(via.x, via.y).buffer(float(getattr(via, 'size', 0.0) or 0.0) / 2.0)
        found = None
        for r in self.regions:
            if r.polygon.intersects(disc):
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
        fn(area.inside(obstacles, fcfg, i), clearance=r.value,
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


def pair_gap_plan(area, pcb_data, pairs, gap_of) -> List[dict]:
    """#1843: how each coupled pair keeps a tightened rule area it would
    otherwise break. KiCad's custom rule outranks the pair's implicit gap
    rule inside the area, so a pair built at a gap below the area's value
    violates P against N wherever it runs there. A coupled pair is laid at
    one gap, so per pair and area:

      * 'widened': the area meets the hull of the pair's pads (a pad in it,
        or the area between them), so the pair has to pass it: it is built
        at the area's value for the run (its impedance changes, which the
        run discloses);
      * 'around': otherwise the pair's two nets are kept out of the area (a
        net-filtered keep-out, `keepouts_for`), which only takes a detour
        through it away, so the gap the designer set -- its impedance --
        holds wherever the pair runs.

    ``pairs`` is `[(name, pair)]`, ``gap_of(pair)`` the gap it is built at.
    Rows {pair, p_net_id, n_net_id, region, rule, area, value, gap, how}."""
    if area is None or not any(r.tight for r in area.regions):
        return []
    from shapely.geometry import box
    from shapely.ops import unary_union
    out = []
    for name, pair in pairs:
        g = float(gap_of(pair))
        hull = None
        for i, r in enumerate(area.regions):
            if not r.tight or g >= r.value - 1e-9:
                continue
            if hull is None:
                shapes = []
                for nid in (pair.p_net_id, pair.n_net_id):
                    for pad in (getattr(pcb_data, 'pads_by_net', None) or {}).get(nid, ()):
                        hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
                        if getattr(pad, 'rect_rotation', 0.0):
                            hx = hy = math.hypot(hx, hy)
                        shapes.append(box(pad.global_x - hx, pad.global_y - hy,
                                          pad.global_x + hx, pad.global_y + hy))
                hull = unary_union(shapes).convex_hull if shapes else None
            crosses = hull is not None and r.polygon.intersects(hull)
            out.append({'pair': name, 'p_net_id': pair.p_net_id, 'n_net_id': pair.n_net_id,
                        'region': i, 'rule': r.rule, 'area': r.area, 'value': r.value,
                        'gap': round(g, 4), 'how': 'widened' if crosses else 'around'})
    return out


def keepouts_for(area, pcb_data, plan) -> List[dict]:
    """The net-filtered keep-outs (`net_keepouts` region dicts) that keep the
    'around' pairs of a `pair_gap_plan` out of their areas."""
    out = []
    for row in plan:
        if row['how'] != 'around':
            continue
        r = area.regions[row['region']]
        nets = {getattr(pcb_data.nets.get(n), 'name', '') for n in
                (row['p_net_id'], row['n_net_id'])}
        polys = getattr(r.polygon, 'geoms', [r.polygon])
        for g in polys:
            out.append({'name': r.area, 'rule': f"{r.rule} (pair gap {row['gap']:g} mm)",
                        'polygon': [tuple(c) for c in g.exterior.coords][:-1],
                        'holes': [[tuple(c) for c in h.coords][:-1] for h in g.interiors],
                        'layers': set(r.layers), 'tracks_allowed': False,
                        'vias_allowed': False, 'nets': nets, 'area_pair_gap': True})
    return out


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


def eligible(rules, pcb_data):
    """(the board's area index, [(rule, area names, None) | (rule, None,
    why it is not routed to)]) for the clearance rules scoped by a rule area
    the board has."""
    # a rule with another constraint beside the clearance is applied whole or
    # not at all: it stays with the reader of that constraint
    cands = [r for r in rules.rules
             if getattr(r, 'area_scoped_condition', None) is not None
             and 'min' in (r.constraints.get('clearance') or {})
             and set(r.constraints) == {'clearance'}]
    if not cands:
        return None, []
    index = rules.area_index(pcb_data)
    if index is None:
        return None, []
    out = []
    for r in cands:
        ast = r.area_scoped_condition
        if ast is None:            # names an area the board does not have
            continue
        names = _area_names(ast)
        if names is None:
            out.append((r, None, 'its condition is more than rule-area predicates'))
            continue
        out.append((r, names, None))
    return index, out


def mark_honoured(rules, pcb_data, config=None) -> None:
    """Name, for the rules report, the area clearance rules a routing step
    routes to (`Rule.honoured`). A rule is named only when its regions were
    actually built: the report must not say "routed to" for a setup that
    failed, so the regions are derived here, as `install` derives them, and
    a failure names nothing (the report then reads NOT honoured, which is
    what the run will do)."""
    from types import SimpleNamespace
    probe = config if config is not None else SimpleNamespace(
        clearance=0.0, layer_clearances={}, track_width=0.0, via_size=0.0,
        grid_step=0.1, layers=list(getattr(getattr(pcb_data, 'board_info', None),
                                           'copper_layers', None) or []))
    try:
        _index, picked = eligible(rules, pcb_data)
        built = derive(rules, pcb_data, probe)
    except Exception as e:                                      # noqa: BLE001
        print(f"  ERROR: the board's area clearance rules could not be set up "
              f"({type(e).__name__}: {e}); this run routes WITHOUT them")
        return
    routed = {reg.rule for reg in (built.regions if built is not None else [])}
    copper = list(getattr(getattr(pcb_data, 'board_info', None), 'copper_layers', None) or [])
    floor = _fab_floor(copper, 'clearance')
    for r, names, _why in picked:
        if names is not None and not r.honoured and r.name in routed:
            where = ', '.join(f"'{n}'" for n in names)
            v = float(r.constraints['clearance']['min'])
            pinned = (f" (below the fab floor: routed at {floor:g} mm; --fab-tier / "
                      f"--fab-overrides set the floor)" if v < floor - 1e-9 else '')
            r.honoured = (f"clearance min {v:g} mm in rule area {where}: tracks and "
                          f"vias are routed to it there{pinned}")


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
    """The area clearance the board's rules declare, or None when none is
    routed to. ``rules`` is the board's `DesignRules`."""
    if rules is None or pcb_data is None:
        return None
    index, picked = eligible(rules, pcb_data)
    if not picked:
        return None
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or config.layers)
    floor = _fab_floor(copper, 'clearance')
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
    regions, skipped, hold = [], [], []
    for r, names, why in picked:
        if names is None:
            skipped.append((r.name, why))
            continue
        on = r.layers_for(copper)
        value = max(float(r.constraints['clearance']['min']), floor)
        for entry in index.areas:
            # (name, layers, polygon, ...): the index may carry more per area
            aname, alayers, poly = entry[0], entry[1], entry[2]
            if aname not in names:
                continue
            layers = frozenset(alayers if on is None else set(alayers) & set(on))
            if not layers:
                continue
            # the ring holds, at the rule's value, the stamp of anything the
            # run lays touching the area: reach past the copper itself, then
            # the value and the half size of what is routed past it
            ring = ((inner + value + reach + config.grid_step)
                    if value > base + 1e-9 else 0.0)
            regions.append(Region(r.name, aname, value, poly, layers, ring, r.order))
            hold.append(value + reach + config.grid_step)
    if not regions and not skipped:
        return None
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
    missing = sorted(named - {reg.rule for reg in (area.regions if area else [])})
    if missing:
        print(f"  ERROR: area clearance rule(s) {', '.join(repr(n) for n in missing)} "
              f"were reported as routed to but have no region in this step; it "
              f"routes WITHOUT them")
    if area is not None:
        key = (getattr(pcb_data, 'source_path', '') or '', tuple(area.skipped))
        if area.skipped and key not in _ANNOUNCED:
            _ANNOUNCED.add(key)
            for name, why in area.skipped:
                print(f"  .kicad_dru rule '{name}': its area clearance is NOT routed "
                      f"to ({why}); check_drc grades it")
        if not area.regions:
            area = None
    config.area_clearance = area
