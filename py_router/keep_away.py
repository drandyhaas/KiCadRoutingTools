"""Pairwise keep-away between net groups (#1146).

A rule ``AGGRESSOR:VICTIM:GAP`` asks copper of the nets on one side to stay
GAP mm (edge to edge, same layer) from copper of the nets on the other side.
The clearance is still the hard floor; the keep-away is a SOFT cost above it.
While a net on either side is routed, every grid cell where its track would
sit closer than GAP to the other side's copper (pads, tracks, vias) costs
``keep_away_cost`` per cell on the per-layer proximity map -- the same
mm-equivalent unit as every other proximity knob. Nets on the same side route
against each other at the normal clearance.

Within ``keep_away_free`` mm of the routed net's OWN pads the band is not
priced, so a pin can leave a package whose neighbouring pins belong to the
other group (a relay's coil pin beside its contact pins, a converter's clock
pin beside its analog pins).

Because the cost is soft, a band the router cannot avoid is still routed
through. ``keep_away_report`` measures the result on the copper itself -- per
net, the length that ended up inside a band beyond the free radius -- and the
route step carries it as ``JSON_SUMMARY['keep_away']``, so a run can be graded.

Rule syntax: each side is a comma-separated list of net patterns, resolved by
``expand_net_patterns`` exactly like ``--nets`` (``*``/``?`` wildcards,
``!`` exclusions), and/or net-class terms ``class=NAME`` (NAME may be a glob;
``!class=NAME`` takes a class out), so ``class=Digital:class=Audio:0.5`` is
KiCad's ``A.NetClass == 'Digital' && B.NetClass == 'Audio'`` as a soft rule.
Classes come from the board's project through the resolver the router and
check_drc share; a net no class claims is in ``Default``. Several rules may
share one string, separated by whitespace or ``;`` (the GUI's one-line
field). Net patterns and class names may therefore not contain ``:``, ``,``,
``;`` or whitespace; use ``?`` for such a character (``class=High?Speed``).
"""

from __future__ import annotations

import math
import re
from collections import OrderedDict
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

from routing_config import GridCoord
from routing_utils import _capsule_mask, pad_blocked_cells_array

# (layer, gx, gy) packed into one int64, as obstacle_costs packs its SUM rows:
# grid coordinates are well inside +/-2^23.
_OFF = 1 << 23
_XY_MASK = (1 << 48) - 1
_COORD_MASK = (1 << 24) - 1

# A side term naming a net CLASS rather than nets: `class=Audio`, or
# `!class=Default` to take a class out. (`:` already separates the rule.)
CLASS_PREFIX = 'class='

# Report tolerance (mm). A track whose cells all lie outside the priced band
# can still come a few microns closer than GAP between two cell centres (a
# chord across a round pad's band), so spacing within this much of GAP is not
# reported as inside the band.
REPORT_TOL = 0.01
# Sample pitch (mm) along each track for the report.
_REPORT_STEP = 0.05
# The widest GAP a rule may ask for (mm). A band is rasterised around every
# copper item of the other side, so its cost grows with GAP squared: a typo
# such as 50 for 0.5 would allocate gigabytes.
MAX_GAP = 10.0
# Memory the cached bands may hold (bytes): per-net fields, and composites.
_FIELD_BYTES_MAX = 256 * 2 ** 20
_COMPOSITE_BYTES_MAX = 256 * 2 ** 20


@dataclass(frozen=True)
class KeepAwayRule:
    aggressor: Tuple[str, ...]
    victim: Tuple[str, ...]
    gap: float

    @property
    def spec(self) -> str:
        return (f"{','.join(self.aggressor)}:{','.join(self.victim)}:"
                f"{self.gap:.12g}")


def split_keep_away_specs(specs) -> List[str]:
    """One token per rule. A CLI value or the GUI's text may hold several
    rules separated by whitespace or ';'."""
    if not specs:
        return []
    if isinstance(specs, str):
        specs = [specs]
    out = []
    for s in specs:
        out.extend(t for t in re.split(r'[;\s]+', s or '') if t)
    return out


def parse_keep_away_rules(specs) -> Tuple[KeepAwayRule, ...]:
    """Parse ``AGGRESSOR:VICTIM:GAP`` rules; raises ValueError naming the bad
    rule."""
    rules = []
    raw = [specs] if isinstance(specs, str) else list(specs or ())
    # Rules are split on whitespace, so a name with a space in it arrives
    # here in pieces; say so rather than blame the piece.
    hint = (" -- rules are separated by spaces, so write a space inside a net "
            "or class name as '?'" if any(re.search(r'\s', s or '') for s in raw)
            else "")
    for tok in split_keep_away_specs(specs):
        head, sep, gap_s = tok.rpartition(':')
        agg_s, sep2, vic_s = head.partition(':')
        if not sep or not sep2 or ':' in vic_s:
            raise ValueError(
                f"keep-away rule {tok!r}: expected AGGRESSOR:VICTIM:GAP "
                f"(net patterns may not contain ':'){hint}")
        try:
            gap = float(gap_s)
        except ValueError:
            raise ValueError(f"keep-away rule {tok!r}: GAP {gap_s!r} is not "
                             f"a number of mm{hint}") from None
        if not (math.isfinite(gap) and 0 < gap <= MAX_GAP):
            raise ValueError(f"keep-away rule {tok!r}: GAP must be > 0 and "
                             f"<= {MAX_GAP:g} mm")
        agg = tuple(p for p in agg_s.split(',') if p)
        vic = tuple(p for p in vic_s.split(',') if p)
        if not agg or not vic:
            raise ValueError(f"keep-away rule {tok!r}: both net groups need "
                             f"at least one pattern")
        if any(p.lstrip('!') == CLASS_PREFIX for p in agg + vic):
            raise ValueError(f"keep-away rule {tok!r}: '{CLASS_PREFIX}' needs a "
                             f"net-class name")
        rules.append(KeepAwayRule(agg, vic, gap))
    return tuple(rules)


def normalize_keep_away_specs(specs) -> Tuple[str, ...]:
    """Validated, canonical rule strings for GridRouteConfig.keep_away."""
    return tuple(r.spec for r in parse_keep_away_rules(specs))


def keep_away_knob_error(free: float, cost: float) -> Optional[str]:
    """Why --keep-away-free / --keep-away-cost is refused, or None. The range
    is the one the GUI's spin controls offer, so a CLI value a plan carries
    over is never clamped silently there."""
    import routing_defaults as defaults
    for flag, name, v in (('--keep-away-free', 'keep_away_free', free),
                          ('--keep-away-cost', 'keep_away_cost', cost)):
        r = defaults.PARAM_RANGES[name]
        if not (math.isfinite(v) and r['min'] <= v <= r['max']):
            return f"{flag} {v:g} is outside {r['min']:g}..{r['max']:g}"
    return None


def _pack(layer_idx: int, gx: np.ndarray, gy: np.ndarray) -> np.ndarray:
    return ((np.int64(layer_idx) << 48)
            | ((gx.astype(np.int64) + _OFF) << 24)
            | (gy.astype(np.int64) + _OFF))


def _pack_xy(cells: np.ndarray) -> np.ndarray:
    return (((cells[:, 0].astype(np.int64) + _OFF) << 24)
            | (cells[:, 1].astype(np.int64) + _OFF))


def _pad_corner_radius(pad) -> float:
    hw, hh = pad.size_x / 2, pad.size_y / 2
    if pad.shape in ('circle', 'oval'):
        return min(hw, hh)
    if pad.shape == 'roundrect':
        return getattr(pad, 'roundrect_rratio', 0.25) * min(pad.size_x, pad.size_y)
    return 0.0


def _pad_cells(pad, margin: float, coord: GridCoord) -> np.ndarray:
    """(N, 2) cells whose centre lies within `margin` of the pad's copper."""
    step = coord.grid_step
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
    return pad_blocked_cells_array(
        gx, gy, pad.size_x / 2, pad.size_y / 2, margin, step,
        _pad_corner_radius(pad),
        off_x=pad.global_x - gx * step, off_y=pad.global_y - gy * step,
        rotation_deg=getattr(pad, 'rect_rotation', 0.0) or 0.0)


def _pad_layers(pad, routing_layers) -> List[str]:
    from net_queries import expand_pad_layers
    if getattr(pad, 'pad_type', '') == 'np_thru_hole':
        return []          # no copper, only a hole
    return [L for L in expand_pad_layers(pad.layers, routing_layers)
            if L in routing_layers]


def _pads_sig(pcb_data, net_id: int) -> tuple:
    """A net's pads as geometry, order-free: what its band and its free
    radius are drawn from besides its copper."""
    return tuple(sorted(
        (p.global_x, p.global_y, p.size_x, p.size_y, str(p.shape),
         float(getattr(p, 'rect_rotation', 0.0) or 0.0),
         tuple(p.layers or ()), str(getattr(p, 'pad_type', '')))
        for p in pcb_data.pads_by_net.get(net_id, [])))


def _routed_half_width(config, net_ids, single_track=False) -> float:
    """Half the copper the routed object puts either side of the searched
    centreline: one track, or a diff pair's P + gap + N (a hybrid pair's
    single-ended legs are one track each)."""
    w = max(config.get_net_track_width(n, L)
            for n in net_ids for L in config.layers)
    if len(net_ids) == 2 and not single_track:
        return w + config.diff_pair_gap / 2
    return w / 2


class _KeepAwayState:
    """Rules resolved against one board, plus the caches the per-net field is
    assembled from. Lives on the PCBData (``_keep_away_state``) so it follows
    the board the run is routing, keyed by the rules and the grid."""

    def __init__(self, pcb_data, rules: Tuple[KeepAwayRule, ...]):
        self.rules = rules
        self.groups: List[Tuple[frozenset, frozenset, float]] = []
        self.unmatched: List[str] = []
        self.notes: List[str] = []
        self._classes = None
        for r in rules:
            sides = [self._resolve_side(pcb_data, pats)
                     for pats in (r.aggressor, r.victim)]
            if not sides[0] or not sides[1]:
                self.unmatched.append(r.spec)
            self.groups.append((sides[0], sides[1], r.gap))
        self.members = frozenset().union(*(a | v for a, v, _g in self.groups))
        self.copper: Dict[int, tuple] = {}
        self._fields: "OrderedDict[tuple, np.ndarray]" = OrderedDict()
        self._field_bytes = 0
        self._exempt: Dict[tuple, np.ndarray] = {}
        self._composites: "OrderedDict[tuple, Optional[np.ndarray]]" = OrderedDict()

    def _class_members(self, pcb_data) -> Dict[int, frozenset]:
        """{net_id: its net-class names} from the board's project, through
        the resolver the router and check_drc share; a net no class claims
        is in 'Default'."""
        if self._classes is None:
            from list_nets import net_class_memberships
            nets = {nid: n.name for nid, n in pcb_data.nets.items()
                    if nid and n.name}
            path = getattr(pcb_data, 'source_path', '') or ''
            try:
                raw = net_class_memberships(path, nets) if path else {}
            except Exception:                               # noqa: BLE001
                raw = {}
            self._classes = {nid: frozenset(raw.get(nid) or ('Default',))
                             for nid in nets}
        return self._classes

    def _resolve_side(self, pcb_data, pats) -> frozenset:
        """Net ids one side names. Without a class term the side is exactly
        what expand_net_patterns makes of it (as --nets). With one, the side
        is the union of its included net patterns and `class=NAME` terms
        (NAME may be a glob) minus its `!` terms of either kind -- the name
        terms read exactly as --nets reads them (a `!NAME` that names a real
        active-low net is that net, #177), and, as --nets does, a class never
        brings in an `unconnected-*` net."""
        from fnmatch import fnmatchcase
        from net_queries import expand_net_patterns
        name_to_id: Dict[str, int] = {}
        for nid, net in pcb_data.nets.items():
            if nid and net.name:
                name_to_id.setdefault(net.name, nid)

        def by_name(patterns):
            return {name_to_id[n] for n in expand_net_patterns(pcb_data, list(patterns))
                    if n in name_to_id}
        if not any(p.lstrip('!').startswith(CLASS_PREFIX) for p in pats):
            return frozenset(by_name(pats))
        classes = self._class_members(pcb_data)

        def by_class(cls_glob):
            ids = {nid for nid, cs in classes.items()
                   if any(fnmatchcase(c, cls_glob) for c in cs)
                   and not pcb_data.nets[nid].name.lower().startswith('unconnected-')}
            if not ids:
                known = sorted({c for cs in classes.values() for c in cs})
                self.notes.append(f"class '{cls_glob}' has no nets on this board "
                                  f"(its classes: {', '.join(known) or 'none'})")
            return ids
        incl, excl, incl_cls, excl_cls = [], [], [], []
        for p in pats:
            neg = p.startswith('!')
            body = p[1:] if neg else p
            if body.startswith(CLASS_PREFIX):
                (excl_cls if neg else incl_cls).append(body[len(CLASS_PREFIX):])
            elif neg and p not in name_to_id:
                excl.append(p)
            else:
                incl.append(p)
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):   # a reference set,
            everything = by_name(['*'])                   # not a request
        ids = by_name(incl) if incl else set()
        for c in incl_cls:
            ids |= by_class(c)
        if not incl and not incl_cls:  # exclusions only: everything else
            ids = set(everything)
        if excl:
            ids -= everything - by_name(['*'] + excl)
        for c in excl_cls:
            ids -= by_class(c)
        return frozenset(ids)

    def opposite(self, net_ids: Iterable[int]) -> Dict[int, float]:
        """{other net: GAP} the routed net(s) must keep away from; the largest
        GAP wins where rules overlap."""
        own = set(net_ids)
        out: Dict[int, float] = {}
        for a, v, gap in self.groups:
            for n in own:
                others = (v if n in a else frozenset()) | (a if n in v else frozenset())
                for o in others:
                    if o not in own and out.get(o, 0.0) < gap:
                        out[o] = gap
        return out

    def refresh(self, pcb_data) -> None:
        """Index the member nets' tracks and vias as they are NOW.

        Rebuilt on every call rather than once per `_copper_epoch`: rip-up
        and restore bump the epoch, but over a hundred other sites edit
        pcb_data.segments / .vias in place (a segment swapped for another, a
        coordinate nudged) without bumping it or changing a length, and a
        band built from copper that has since moved prices the wrong cells.
        A net's signature is its geometry, order-free, so a ripped and
        restored net gets its cached band back. Its PADS are part of it: a
        polarity swap moves a net's pads without touching its copper."""
        segs: Dict[int, list] = {n: [] for n in self.members}
        vias: Dict[int, list] = {n: [] for n in self.members}
        for s in pcb_data.segments:
            if s.net_id in segs and not getattr(s, 'graphic', False):
                segs[s.net_id].append(s)
        for v in pcb_data.vias:
            if v.net_id in vias:
                vias[v.net_id].append(v)
        self.copper = {}
        for n in self.members:
            sig = hash((tuple(sorted((s.start_x, s.start_y, s.end_x, s.end_y,
                                      s.width, s.layer) for s in segs[n])),
                        tuple(sorted((v.x, v.y, v.size, tuple(v.layers or ()))
                                     for v in vias[n])),
                        _pads_sig(pcb_data, n)))
            self.copper[n] = (sig, segs[n], vias[n])

    def field(self, pcb_data, config, net_id: int, margin: float,
              window=None) -> np.ndarray:
        """Packed cells within `margin` of `net_id`'s copper on the same layer
        (a band's GAP plus the routed track's half-width: the cells where
        that track would sit closer than GAP). With `window` (x0, y0, x1, y1
        in mm) only the copper whose band reaches the window is rasterised,
        and nothing is cached: a rescue rung routes inside a small window, at
        a grid fine enough that the whole board's band is ~1 minute and
        ~0.5 GB per net."""
        from connectivity import via_copper_layers
        sig, segs, vias = self.copper[net_id]
        margin = round(margin, 9)
        key = (net_id, margin, sig)
        if window is None:
            hit = self._fields.get(key)
            if hit is not None:
                self._fields.move_to_end(key)
                return hit
        pads = pcb_data.pads_by_net.get(net_id, [])
        if window is not None:
            x0, y0, x1, y1 = window

            def meets(lx, ly, hx, hy, r):
                return lx - r <= x1 and hx + r >= x0 and ly - r <= y1 and hy + r >= y0
            segs = [s for s in segs if meets(
                min(s.start_x, s.end_x), min(s.start_y, s.end_y),
                max(s.start_x, s.end_x), max(s.start_y, s.end_y),
                (s.width or 0.0) / 2 + margin)]
            vias = [v for v in vias if meets(v.x, v.y, v.x, v.y,
                                             (v.size or 0.0) / 2 + margin)]
            pads = [p for p in pads if meets(
                p.global_x, p.global_y, p.global_x, p.global_y,
                math.hypot(p.size_x, p.size_y) / 2 + margin)]
        coord = GridCoord(config.grid_step)
        step = config.grid_step
        lidx = {L: i for i, L in enumerate(config.layers)}
        parts = []
        for s in segs:
            li = lidx.get(s.layer)
            if li is None:
                continue
            _xs, _ys, gxg, gyg, mask = _capsule_mask(
                s.start_x, s.start_y, s.end_x, s.end_y,
                (s.width or 0.0) / 2 + margin, step)
            parts.append(_pack(li, gxg[mask], gyg[mask]))
        for v in vias:
            _xs, _ys, gxg, gyg, mask = _capsule_mask(
                v.x, v.y, v.x, v.y, (v.size or 0.0) / 2 + margin, step)
            gx, gy = gxg[mask], gyg[mask]
            for L in via_copper_layers(v, config.layers):
                if L in lidx:
                    parts.append(_pack(lidx[L], gx, gy))
        for pad in pads:
            layers = _pad_layers(pad, config.layers)
            if not layers:
                continue
            cells = _pad_cells(pad, margin, coord)
            for L in layers:
                parts.append(_pack(lidx[L], cells[:, 0], cells[:, 1]))
        out = (np.unique(np.concatenate(parts)) if parts
               else np.empty(0, dtype=np.int64))
        if window is None:
            self._fields[key] = out
            self._field_bytes += out.nbytes
            while self._field_bytes > _FIELD_BYTES_MAX and len(self._fields) > 1:
                self._field_bytes -= self._fields.popitem(last=False)[1].nbytes
        return out

    def exempt(self, pcb_data, config, net_ids: Tuple[int, ...],
               radius: float) -> np.ndarray:
        """Packed (gx, gy) cells within `radius` of the routed net's own pads."""
        key = (net_ids, radius, tuple(_pads_sig(pcb_data, n) for n in net_ids))
        hit = self._exempt.get(key)
        if hit is not None:
            return hit
        coord = GridCoord(config.grid_step)
        parts = [_pack_xy(_pad_cells(pad, radius, coord))
                 for n in net_ids for pad in pcb_data.pads_by_net.get(n, [])
                 if _pad_layers(pad, config.layers)]
        out = (np.unique(np.concatenate(parts)) if parts
               else np.empty(0, dtype=np.int64))
        if len(self._exempt) >= 256:
            self._exempt.clear()
        self._exempt[key] = out
        return out

    def rows(self, pcb_data, config, net_ids: Tuple[int, ...],
             single_track: bool = False, window=None) -> Optional[np.ndarray]:
        opp = self.opposite(net_ids)
        if not opp:
            return None
        self.refresh(pcb_data)
        cost = config.cell_cost(config.keep_away_cost)
        free = max(0.0, float(config.keep_away_free or 0.0))
        half_w = _routed_half_width(config, net_ids, single_track)
        parts_key = tuple(sorted((o, g, self.copper[o][0])
                                 for o, g in opp.items() if o in self.copper))
        ckey = (net_ids, half_w, free, cost, parts_key)
        if window is None and ckey in self._composites:
            self._composites.move_to_end(ckey)
            return self._composites[ckey]
        fields = [f for f in (self.field(pcb_data, config, o, g + half_w, window)
                              for o, g, _s in parts_key) if len(f)]
        rows = None
        if fields:
            ex = (self.exempt(pcb_data, config, net_ids, free) if free > 0
                  else np.empty(0, dtype=np.int64))
            # A NEW array per composition: merge_track_proximity_costs
            # memoizes on the ids of the arrays it is handed.
            rows = _compose(fields, ex, cost)
        if window is None:
            self._composites[ckey] = rows
            while len(self._composites) > 8 or (
                    len(self._composites) > 1
                    and sum(r.nbytes for r in self._composites.values()
                            if r is not None) > _COMPOSITE_BYTES_MAX):
                self._composites.popitem(last=False)
        return rows


# Largest bounding box (cells x layers) _compose unions on a dense bitmap;
# beyond it, it sorts instead.
_BITMAP_MAX_CELLS = 64_000_000


def _compose(fields: List[np.ndarray], exempt_xy: np.ndarray,
             cost: int) -> Optional[np.ndarray]:
    """Union of packed field cells minus the exempt (gx, gy) cells, as
    unique [layer, gx, gy, cost] rows in sorted order.

    Every cell carries the same cost, so the union only has to be unique (sum
    mode in merge_track_proximity_costs would count a duplicate twice). A
    dense bitmap over the cells' bounding box does that in linear time; the
    sort it replaces cost ~0.1 s per routed net on a 2 mm band."""
    cat = np.concatenate(fields) if len(fields) > 1 else fields[0]
    lay = cat >> 48
    gx = (cat >> 24) & _COORD_MASK
    gy = cat & _COORD_MASK
    x0, y0 = int(gx.min()), int(gy.min())
    n_l = int(lay.max()) + 1
    w, h = int(gx.max()) - x0 + 1, int(gy.max()) - y0 + 1
    if n_l * w * h <= _BITMAP_MAX_CELLS:
        bm = np.zeros(n_l * w * h, dtype=bool)
        bm[(lay * w + (gx - x0)) * h + (gy - y0)] = True
        if len(exempt_xy):
            ex = (exempt_xy >> 24) - x0, (exempt_xy & _COORD_MASK) - y0
            inside = (ex[0] >= 0) & (ex[0] < w) & (ex[1] >= 0) & (ex[1] < h)
            flat = ex[0][inside] * h + ex[1][inside]
            for li in range(n_l):
                bm[li * w * h + flat] = False
        nz = np.flatnonzero(bm)
        if not len(nz):
            return None
        rem = nz % (w * h)
        lay, gx, gy = nz // (w * h), rem // h + x0, rem % h + y0
    else:
        keys = np.unique(cat)
        if len(exempt_xy):
            keys = keys[~np.isin(keys & _XY_MASK, exempt_xy)]
        if not len(keys):
            return None
        lay, gx, gy = keys >> 48, (keys >> 24) & _COORD_MASK, keys & _COORD_MASK
    rows = np.empty((len(lay), 4), dtype=np.int32)
    rows[:, 0] = lay
    rows[:, 1] = gx - _OFF
    rows[:, 2] = gy - _OFF
    rows[:, 3] = cost
    return rows


def _state(config, pcb_data) -> Optional[_KeepAwayState]:
    specs = tuple(getattr(config, 'keep_away', None) or ())
    if not specs or pcb_data is None:
        return None
    store = getattr(pcb_data, '_keep_away_state', None)
    if store is None:
        store = pcb_data._keep_away_state = {}
    key = (specs, tuple(config.layers), config.grid_step)
    st = store.get(key)
    if st is None:
        st = store[key] = _KeepAwayState(pcb_data, parse_keep_away_rules(specs))
        for note in st.notes:
            print(f"WARNING: keep-away: {note}")
        for spec in st.unmatched:
            print(f"WARNING: keep-away rule '{spec}' has a side that matches "
                  f"no net on this board; it does nothing.")
    return st


def keep_away_rows(config, pcb_data, net_ids, single_track=False,
                   window=None) -> Optional[np.ndarray]:
    """[layer, gx, gy, cost] rows pricing the keep-away band for the routed
    net (an int) or diff pair (a tuple of its P and N ids); `single_track`
    sizes a pair's band for one track (a hybrid pair's legs); `window`
    (x0, y0, x1, y1 mm) prices only the band that reaches it, uncached. None
    when no rule concerns it or the cost is 0."""
    if getattr(config, 'keep_away_cost', 0) <= 0:
        return None
    st = _state(config, pcb_data)
    if st is None:
        return None
    ids = (net_ids,) if isinstance(net_ids, int) else tuple(sorted(net_ids))
    return st.rows(pcb_data, config, ids, single_track, window)


def stamp_keep_away(obstacles, config, pcb_data, net_ids,
                    single_track=False, window=None) -> bool:
    """Price the keep-away band on a map no proximity builder prepared: the
    per-attempt clones net_rescue's gap rescue and terminal escalation route
    on. A rescue rung that routes inside a window passes it as `window`
    (x0, y0, x1, y1 mm). Returns True when it stamped anything; on a map
    that outlives the route the caller clears it again with
    clear_layer_proximity(). Nothing is stamped without a rule, so a run
    without one routes exactly as before."""
    rows = keep_away_rows(config, pcb_data, net_ids, single_track, window)
    if rows is None:
        return False
    obstacles.set_layer_proximity_batch(rows)
    return True


def add_keepaway_source(ghosts, config, pcb_data, net_ids, single_track=False):
    """Fold the keep-away rows into a merge_track_proximity_costs ghost dict
    (the add_plan_source / add_history_source pattern; the key is the
    ('keepaway',) tuple). Returns the input unchanged when nothing applies."""
    rows = keep_away_rows(config, pcb_data, net_ids, single_track)
    if rows is None:
        return ghosts
    merged = dict(ghosts) if ghosts else {}
    merged[('keepaway',)] = rows
    return merged


# --------------------------------------------------------------------------
# Report: how much copper ended up inside a band, measured on the geometry.

def _net_geometry(st: _KeepAwayState, pcb_data, config, net_id: int) -> dict:
    """Float arrays of one net's copper, one row per (item, layer):
    segs [layer, x1, y1, x2, y2, half_width], vias [layer, x, y, radius],
    pads [layer, cx, cy, half_w, half_h, corner_r, cos, sin]."""
    from connectivity import via_copper_layers
    lidx = {L: i for i, L in enumerate(config.layers)}
    _sig, segs, vias = st.copper[net_id]
    s_rows = [(lidx[s.layer], s.start_x, s.start_y, s.end_x, s.end_y,
               (s.width or 0.0) / 2) for s in segs if s.layer in lidx]
    v_rows = [(lidx[L], v.x, v.y, (v.size or 0.0) / 2)
              for v in vias for L in via_copper_layers(v, config.layers)
              if L in lidx]
    p_rows = []
    for pad in pcb_data.pads_by_net.get(net_id, []):
        rot = math.radians(getattr(pad, 'rect_rotation', 0.0) or 0.0)
        for L in _pad_layers(pad, config.layers):
            p_rows.append((lidx[L], pad.global_x, pad.global_y, pad.size_x / 2,
                           pad.size_y / 2, _pad_corner_radius(pad),
                           math.cos(rot), math.sin(rot)))
    return {'segs': np.array(s_rows, dtype=np.float64).reshape(-1, 6),
            'vias': np.array(v_rows, dtype=np.float64).reshape(-1, 4),
            'pads': np.array(p_rows, dtype=np.float64).reshape(-1, 8)}


def _pad_sdf(px, py, pads):
    """(k, m) signed distance from points to rounded-rect pads (rows as in
    _net_geometry), negative inside."""
    dx = px[:, None] - pads[None, :, 1]
    dy = py[:, None] - pads[None, :, 2]
    c, s = pads[None, :, 6], pads[None, :, 7]
    lx = np.abs(dx * c + dy * s)
    ly = np.abs(-dx * s + dy * c)
    cr = pads[None, :, 5]
    qx = lx - (pads[None, :, 3] - cr)
    qy = ly - (pads[None, :, 4] - cr)
    outside = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
    inside = np.minimum(np.maximum(qx, qy), 0.0)
    return outside + inside - cr


def _seg_dist(px, py, segs):
    """(k, m) distance from points to segment centrelines."""
    x1, y1 = segs[None, :, 1], segs[None, :, 2]
    dx, dy = segs[None, :, 3] - x1, segs[None, :, 4] - y1
    l2 = dx * dx + dy * dy
    t = np.where(l2 > 0, ((px[:, None] - x1) * dx + (py[:, None] - y1) * dy)
                 / np.where(l2 > 0, l2, 1.0), 0.0)
    t = np.clip(t, 0.0, 1.0)
    return np.hypot(px[:, None] - (x1 + t * dx), py[:, None] - (y1 + t * dy))


def _opposite_rows(geo, opp: Dict[int, float], kind: str, width: int) -> np.ndarray:
    arrs = [np.column_stack([geo[o][kind], np.full(len(geo[o][kind]), opp[o]),
                             np.full(len(geo[o][kind]), o)])
            for o in sorted(opp) if o in geo and len(geo[o][kind])]
    return np.vstack(arrs) if arrs else np.empty((0, width + 2), dtype=np.float64)


def keep_away_report(pcb_data, config) -> Optional[dict]:
    """Measure every keep-away net's tracks against the other side's copper.

    Per net: the track length whose edge comes closer than GAP to copper of a
    net the rules set it against (same layer), excluding the stretch within
    the free radius of its own pads, where the band is not priced either.
    Board-scoped: it reads all copper on the board, not only this run's.
    None when no rule is set."""
    st = _state(config, pcb_data)
    if st is None:
        return None
    st.refresh(pcb_data)
    free = max(0.0, float(getattr(config, 'keep_away_free', 0.0) or 0.0))
    geo = {n: _net_geometry(st, pcb_data, config, n) for n in st.members}
    names = {n: (pcb_data.nets[n].name if n in pcb_data.nets else str(n))
             for n in st.members}
    per_net = {}
    checked = 0
    total_in = 0.0
    for n in sorted(st.members, key=lambda k: names[k]):
        own = geo[n]
        if not len(own['segs']):
            continue
        opp = st.opposite((n,))
        if not opp:
            continue
        checked += 1
        # Opposite copper, each row with its GAP and owning net appended.
        o_segs, o_vias, o_pads = (_opposite_rows(geo, opp, kind, width)
                                  for kind, width in (('segs', 6), ('vias', 4),
                                                      ('pads', 8)))
        # How far from an item's reference point its band can reach: the
        # bounding box of the samples grows by this before filtering items.
        reach = max(float(o_segs[:, 5].max()) if len(o_segs) else 0.0,
                    float(o_vias[:, 3].max()) if len(o_vias) else 0.0,
                    float(np.hypot(o_pads[:, 3], o_pads[:, 4]).max())
                    if len(o_pads) else 0.0) + max(opp.values())
        length = in_band = 0.0
        best = (math.inf, None, 0.0)      # (closest spacing, net, its GAP)
        for li, x1, y1, x2, y2, hw in own['segs']:
            seg_len = math.hypot(x2 - x1, y2 - y1)
            length += seg_len
            k = max(1, int(math.ceil(seg_len / _REPORT_STEP)))
            t = (np.arange(k) + 0.5) / k
            px, py = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
            keep = np.ones(k, dtype=bool)
            if free > 0 and len(own['pads']):
                keep &= ~(_pad_sdf(px, py, own['pads']) <= free).any(axis=1)
            if not keep.any():
                continue
            lo_x, hi_x = min(x1, x2) - hw - reach, max(x1, x2) + hw + reach
            lo_y, hi_y = min(y1, y2) - hw - reach, max(y1, y2) + hw + reach
            # Edge-to-edge spacing per (sample, item), and each item's GAP.
            spacing = [np.full((k, 1), math.inf)]
            gaps = [np.zeros(1)]
            owners = [np.full(1, -1.0)]
            sel = o_segs[(o_segs[:, 0] == li)
                         & (np.maximum(o_segs[:, 1], o_segs[:, 3]) >= lo_x)
                         & (np.minimum(o_segs[:, 1], o_segs[:, 3]) <= hi_x)
                         & (np.maximum(o_segs[:, 2], o_segs[:, 4]) >= lo_y)
                         & (np.minimum(o_segs[:, 2], o_segs[:, 4]) <= hi_y)]
            if len(sel):
                spacing.append(_seg_dist(px, py, sel) - sel[None, :, 5] - hw)
                gaps.append(sel[:, 6])
                owners.append(sel[:, 7])
            for arr, dist in ((o_vias, lambda a: np.hypot(
                                   px[:, None] - a[None, :, 1],
                                   py[:, None] - a[None, :, 2]) - a[None, :, 3]),
                              (o_pads, lambda a: _pad_sdf(px, py, a))):
                sel = arr[(arr[:, 0] == li)
                          & (arr[:, 1] >= lo_x) & (arr[:, 1] <= hi_x)
                          & (arr[:, 2] >= lo_y) & (arr[:, 2] <= hi_y)]
                if len(sel):
                    spacing.append(dist(sel) - hw)
                    gaps.append(sel[:, -2])
                    owners.append(sel[:, -1])
            sp = np.hstack(spacing)
            gap_of = np.concatenate(gaps)
            own_of = np.concatenate(owners)
            # In band where spacing < GAP for ANY item (each its own GAP).
            dmin = (sp - gap_of[None, :]).min(axis=1)
            hit = keep & (dmin < -REPORT_TOL)
            if hit.any():
                in_band += seg_len * float(hit.sum()) / k
            # The closest copper of the other side, over the samples the band
            # prices -- the spacing a reader compares with the GAP.
            near = sp.argmin(axis=1)
            spmin = sp[np.arange(k), near]
            kept = np.flatnonzero(keep)
            j = int(kept[spmin[kept].argmin()])
            if spmin[j] < best[0]:
                o = int(own_of[near[j]])
                best = (float(spmin[j]), o, opp.get(o, 0.0))
        if in_band > 0:
            total_in += in_band
            per_net[names[n]] = {
                'in_band_mm': round(in_band, 2),
                'length_mm': round(length, 2),
                'min_spacing_mm': round(best[0], 3),
                'gap_mm': best[2],
                'closest_net': names.get(best[1], str(best[1])),
            }
    return {
        'rules': [r.spec for r in st.rules],
        'free_radius_mm': free,
        'cost': float(getattr(config, 'keep_away_cost', 0.0)),
        'nets_checked': checked,
        'nets_in_band': len(per_net),
        'in_band_mm': round(total_in, 2),
        'nets': dict(sorted(per_net.items(),
                            key=lambda kv: -kv[1]['in_band_mm'])),
    }


def disclose_keep_away(board, config, title: str = 'Keep-away',
                       quiet: bool = False) -> Optional[dict]:
    """keep_away_report on `board`, printed unless `quiet`. None without a
    rule; {'error': ...} when the measurement raised, so a summary says the
    reading is missing rather than dropping the key."""
    if not getattr(config, 'keep_away', None):
        return None
    try:
        rep = keep_away_report(board, config)
    except Exception as e:                                     # noqa: BLE001
        return {'error': str(e)}
    if rep is not None and not quiet:
        print_keep_away_report(rep, title=title)
    return rep


def keep_away_entries(report: Optional[dict]) -> List[dict]:
    """The report's in-band nets as a list, {'net': name, ...} each -- the
    shape results_data carries (every field there is a list)."""
    return [{'net': n, **e} for n, e in ((report or {}).get('nets') or {}).items()]


def print_keep_away_report(report: Optional[dict], limit: int = 20,
                           title: str = 'Keep-away') -> None:
    if not report or 'rules' not in report:
        return
    n_in = report['nets_in_band']
    print(f"\n{title} ({len(report['rules'])} rule(s), free radius "
          f"{report['free_radius_mm']:g} mm): {n_in} of "
          f"{report['nets_checked']} net(s) run {report['in_band_mm']:g} mm "
          f"inside a band")
    for name, e in list(report['nets'].items())[:limit]:
        print(f"  {name}: {e['in_band_mm']:g} of {e['length_mm']:g} mm, "
              f"closest {e['min_spacing_mm']:g} mm (gap {e['gap_mm']:g}) "
              f"to {e['closest_net']}")
    if n_in > limit:
        print(f"  ... and {n_in - limit} more (JSON_SUMMARY keep_away)")
