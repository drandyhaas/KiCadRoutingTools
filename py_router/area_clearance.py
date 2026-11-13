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
    """One area rule the router honours."""
    __slots__ = ('rule', 'area', 'value', 'polygon', 'layers', 'ring', 'order')

    def __init__(self, rule, area, value, polygon, layers, ring, order=0):
        self.rule, self.area, self.value = rule, area, value
        self.polygon, self.layers, self.ring = polygon, layers, ring
        self.order = order        # the rule's position in the file


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
        return default if i is None else self.regions[i].value

    def via_clearance(self, via, default: float) -> float:
        """The clearance a via is held to: a region's where its area holds
        the via on every copper layer, else ``default``."""
        from shapely.geometry import Point
        pt = Point(via.x, via.y)
        found = default
        for r in self.regions:
            if set(self.copper) <= r.layers and r.polygon.contains(pt):
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


def eligible(rules, pcb_data):
    """(the board's area index, [(rule, area names, None) | (rule, None,
    why it is not routed to)]) for the clearance rules scoped by a rule area
    the board has."""
    cands = [r for r in rules.rules
             if getattr(r, 'area_scoped_condition', None) is not None
             and 'min' in (r.constraints.get('clearance') or {})]
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
            pinned = (f" (below the fab floor: routed at {floor:g} mm; --fab-overrides "
                      f"declares a smaller one)" if v < floor - 1e-9 else '')
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
    reach = max(float(config.track_width or 0.0), float(config.via_size or 0.0)) / 2.0
    regions, skipped = [], []
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
            ring = (value + reach + config.grid_step) if value > base + 1e-9 else 0.0
            regions.append(Region(r.name, aname, value, poly, layers, ring, r.order))
    if not regions and not skipped:
        return None
    return AreaClearance(regions, copper, skipped, _layer_orders(rules, copper))


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
