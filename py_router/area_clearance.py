"""#1843: route at the clearance a rule area sets.

A `.kicad_dru` clearance rule whose condition is one rule-area predicate
(`A.insideArea('ROOM')`, `intersectsArea`) binds a pair when either item
stands in the area on the layer asked about, and its value REPLACES the
class value there (measured on kicad-cli 10.0.8; `design_rules.AreaIndex`
grades it the same way). The obstacle map prices a keep-out per obstacle, not
per position, so the rule is honoured by COMPOSING two stamps of everything:

  * outside the area, the stamp at the class values, as without the rule;
  * inside it, the stamp at the rule's value.

A track cell is inside on the layers the area is drawn on; a via cell is
inside where the area covers every copper layer of the board. Around an area
whose rule is ABOVE the run's base clearance, a ring one rule value plus half
a track wide keeps both stamps, so copper outside stays the rule's distance
from copper inside.

Every add and every remove of a keep-out goes through the same filter
(`outside` / `inside`), so the map's reference counts balance exactly as they
do without a rule.
"""

from dataclasses import replace
from functools import wraps
import inspect
from typing import List, Optional

import numpy as np

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
    run's)."""
    __slots__ = ('rule', 'area', 'value', 'polygon', 'layers', 'ring', 'width', 'order')

    def __init__(self, rule, area, value, polygon, layers, ring, width=None, order=0):
        self.rule, self.area, self.value = rule, area, value
        self.polygon, self.layers, self.ring = polygon, layers, ring
        self.width = width
        self.order = order        # the clearance rule's position in the file


class AreaClearance:
    """The board's area-scoped clearance rules, as the obstacle map reads
    them. ``regions`` are in file order: where two overlap, the later one
    owns the cell, as KiCad's last matching rule does."""

    def __init__(self, regions: List[Region], copper_layers, skipped=(),
                 layer_orders=None):
        self.regions = list(regions)
        self.copper = list(copper_layers)
        self.skipped = list(skipped)
        #: {layer: file position of the rule its layer clearance comes from}
        self.layer_orders = dict(layer_orders or {})
        self._masks = {}
        self._later = {}

    # ------------------------------------------------------------- masks
    def _region_masks(self, step: float):
        key = round(float(step), 9)
        got = self._masks.get(key)
        if got is None:
            got = [(_raster(r.polygon, step),
                    _raster(r.polygon.buffer(r.ring), step) if r.ring > 0 else None)
                   for r in self.regions]
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
        ring = masks[want][1]
        if ring is not None:
            on = self._on_layers(self.regions[want], layer_idx, layers, n, core=False)
            if on is not None:
                hit = _lookup(ring, gx, gy) & (owner < 0)
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
        for i, (core, ring) in enumerate(masks):
            for m in ((core, ring) if want is None or want == i else (core,)):
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

    def region_of_segment(self, x1, y1, x2, y2, layer, width=0.0) -> Optional[int]:
        """The region whose rule binds a track (KiCad: a track partly in an
        area is in it), the last one in the file, or None."""
        from shapely.geometry import LineString, Point
        geom = (Point(x1, y1) if (x1, y1) == (x2, y2)
                else LineString([(x1, y1), (x2, y2)]))
        found = None
        for i, r in enumerate(self.regions):
            if layer not in r.layers:
                continue
            if r.ring > 0:
                if r.polygon.distance(geom) < width / 2.0:
                    found = i
            elif r.polygon.intersects(geom):
                found = i
        return found

    def segment_clearance(self, seg, default: float) -> float:
        i = self.region_of_segment(seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                                   seg.layer, float(getattr(seg, 'width', 0.0) or 0.0))
        if i is None or self.regions[i].value is None:
            return default
        return self.regions[i].value

    def via_clearance(self, via, default: float) -> float:
        """The clearance a via is held to: a region's where its area holds
        the via on every copper layer, else ``default``."""
        from shapely.geometry import Point
        pt = Point(via.x, via.y)
        found = default
        for r in self.regions:
            if (r.value is not None and set(self.copper) <= r.layers
                    and r.polygon.contains(pt)):
                found = r.value
        return found

    def outside(self, obstacles, config):
        return _FilterProxy(obstacles, self, config, None, None)

    def inside(self, obstacles, config, want: int, other=None):
        return _FilterProxy(obstacles, self, config, want, other)

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


def config_for_segment(config, seg):
    """``config`` as a geometric check of track ``seg`` reads it: the
    region's when a rule area binds the track, else ``config`` itself."""
    area = getattr(config, 'area_clearance', None)
    if area is None:
        return config
    i = area.region_of_segment(seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                               seg.layer, float(getattr(seg, 'width', 0.0) or 0.0))
    return config if i is None else area.region_config(config, i)


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
    """The floor an area rule's value is raised to: what the fab can
    physically make (the advanced rung, or `--fab-overrides`). The rule is
    the board's own statement for that area, so it is held as an explicit
    request is, not at the tier automatic descents stop at."""
    try:
        from fab_tiers import physical_fab_floor
        return float(physical_fab_floor(len(copper) or 2).get(key) or 0.0)
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
        pinned = (f" (below the fab floor: {floor:g} mm is used; --fab-overrides "
                  f"declares a smaller one)" if v < floor - 1e-9 else '')
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
    reach = max(float(config.track_width or 0.0), float(config.via_size or 0.0)) / 2.0
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
                reg.ring = (value + reach + config.grid_step) if value > base + 1e-9 else 0.0
            else:
                reg.width = value
    if not found and not skipped:
        return None
    return AreaClearance(list(found.values()), copper, skipped,
                         _layer_orders(rules, copper))


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
    try:
        area = derive(rules, pcb_data, config)
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
