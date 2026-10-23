"""Net-filtered copper keep-outs: a region some nets may cross and others may not.

A KiCad rule area has no net dimension, so a design that says "only GND and the
four RF nets may run tracks or vias through this corridor" is drawn as a NAMED
rule area plus a custom rule in the board's .kicad_dru:

    (rule "corridor" (constraint disallow track via)
          (condition "A.intersectsArea('corridor') && !(A.NetName == 'GND' || ...)"))

The resolver (design_rules) cannot evaluate an area predicate, so it reported
such a rule as NOT honoured and the router laid the filtered nets straight
through the region; KiCad's DRC then reported `items_not_allowed`.

`derive_area_keepouts` reads those rules -- the board's own .kicad_dru is the
single source of truth (as for #498's layer clearances: no flag, no GUI
control) -- into regions: one per rule area a `disallow` rule names, carrying
the polygon, the layers, which items it forbids and the set of nets it BINDS.
It is called by design_rules.install_design_rules, which every routing engine
passes through, so both front-ends get it from the shared engine. The bound
nets come from the resolver's own evaluator (NetName, NetClass, Type, layer
predicates ...), separately for tracks and for vias; a rule it cannot model
completely stays reported as NOT honoured rather than half-applied.

A region is stamped into the obstacle map only while a net it binds routes:
`stamp()` wraps the single-ended and diff-pair route entries, adds the cells
(the same rasterisation and clearance band `add_rule_area_keepout_obstacles`
uses for a native keep-out) and removes exactly those cells on exit, so refcounts
balance and an exempt net routes through the region as if it were not there.
Existing copper is never touched: copper already inside the region is an ordinary
obstacle, and nothing rips it.

Coverage is the route entries that stamp: route_net_with_obstacles,
route_multipoint_main / _taps (and with them net_rescue, stub-swap and terminal
escalation, which call them -- a rescue window is a shallow copy that carries
the regions), and on the diff-pair side the pair route (its centreline map at
`centerline_config`, its connectors' base map plain), the hybrid coupled middle
and its legs (`stamping`). Copper laid by anything else -- the plane finalize's
taps and region joins, which have their own A*, and post-route passes such as
meanders -- does not see a region, so the run ends with `audit()` of the board
it ships: bound-net copper found inside a region is printed and reported as
JSON_SUMMARY / --json-out `net_keepouts`, never shipped silently. KiCad's DRC of
the rule stays the authority.
"""
from __future__ import annotations

import contextlib
from itertools import combinations
from types import SimpleNamespace
from typing import Dict, FrozenSet, Iterable, List, Optional, Tuple

import numpy as np

#: `disallow` item tokens naming copper the router lays, by the kind it lays.
#: The router's vias are through vias; KiCad's other tokens (blind_via,
#: micro_via, buried_via, pad, zone, text, graphic, hole, footprint) name
#: nothing a routing step places, so they are left to KiCad's DRC.
_ROUTED_TOKENS = {'track': 'track', 'via': 'via', 'through_via': 'via'}

#: A condition naming more rule areas than this is not modelled: the union
#: check below walks every subset of them.
_MAX_AREAS = 6


class _NotAKeepout(Exception):
    """The rule cannot be honoured as a union of net-filtered regions."""


def _area_names(node, out: set) -> set:
    """Every rule-area name an areas=True condition AST mentions."""
    if isinstance(node, tuple) and node:
        if node[0] == 'area':
            out.add(node[3])
        else:
            for x in node[1:]:
                _area_names(x, out)
    return out


def _area_fns(node, out: set) -> set:
    """The area predicates (lower-cased) an areas=True AST uses."""
    if isinstance(node, tuple) and node:
        if node[0] == 'area':
            out.add(node[2])
        else:
            for x in node[1:]:
                _area_fns(x, out)
    return out


def _layer_dependent(node) -> bool:
    """Does the condition's answer depend on the copper layer?"""
    if not isinstance(node, tuple) or not node:
        return False
    if node[0] == 'L' or (node[0] == 'prop' and node[2] == 'layer'):
        return True
    if node[0] == 'call' and node[2] in ('onlayer', 'existsonlayer'):
        return True
    return any(_layer_dependent(x) for x in node[1:])


def _expand_layers(tokens, copper: List[str]) -> List[str]:
    """A rule area's layer tokens as board copper layers, in board order (no
    token = every layer; the composites the keep-out rasteriser expands)."""
    if not tokens:
        return list(copper)
    toks = set(tokens)
    return [l for l in copper
            if l in toks or '*.Cu' in toks
            or (l in ('F.Cu', 'B.Cu') and bool({'F&B.Cu', 'F&B'} & toks))]


def derive_area_keepouts(rules, pcb_data) -> List[dict]:
    """The net-filtered keep-outs the board's `disallow` rules on named rule
    areas define, as region dicts (the extract_keepouts shape plus `nets`,
    the frozenset of net names the region binds, and `rule`). Sets
    `rule.honoured` on each rule it honours, or a specific `rule.unsupported`
    reason on one it cannot, so the resolver's report says which.

    A rule is honoured when ALL of this holds; otherwise it stays NOT honoured
    (nothing of it is applied):
    - its only constraint is `disallow` and its condition parses with the
      area predicates (design_rules.parse_condition(areas=True));
    - it disallows `track`, `via` or `through_via` (other tokens alone: not
      copper the router lays);
    - for every net and item kind, the condition is false outside every
      named area (else it forbids the item everywhere -- not a keep-out),
      and the areas it names combine as a union (`a || b`, not `a && b`);
    - at least one area it names exists on the board (a name the board does
      not have is disclosed either way).
    `enclosedByArea` (KiCad: the item lies wholly inside) is read as
    `intersectsArea`, which keeps out more than KiCad requires; the report
    says so."""
    from dataclasses import replace
    from design_rules import Unsupported, _eval
    areas: Dict[str, List[dict]] = {}
    for ko in getattr(pcb_data.board_info, 'keepouts', None) or []:
        if ko.get('name'):
            areas.setdefault(ko['name'], []).append(ko)
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None)
                  or rules.copper_layers or [])
    nets = {nid: n.name for nid, n in pcb_data.nets.items() if nid and n.name}
    regions: List[dict] = []
    for rule in rules.rules:
        ast = getattr(rule, 'area_condition', None)
        if ast is None:
            continue
        rule.honoured = None
        if set(rule.constraints) != {'disallow'}:
            continue          # keeps its condition reason: the rest is not modelled
        tokens = set(rule.constraints['disallow'].get('disallow', ()))
        kinds = sorted({_ROUTED_TOKENS[t] for t in tokens if t in _ROUTED_TOKENS})
        if not kinds:
            rule.unsupported = (f"disallow {', '.join(sorted(tokens)) or 'nothing'} on a "
                                f"rule area: nothing the router lays")
            continue
        names = sorted(_area_names(ast, set()))
        missing = [n for n in names if n not in areas]
        if len(names) > _MAX_AREAS:
            rule.unsupported = (f"condition names {len(names)} rule areas "
                                f"(more than {_MAX_AREAS} is not modelled)")
            continue
        if len(missing) == len(names):
            rule.unsupported = (f"names rule area(s) {', '.join(repr(n) for n in missing)}, "
                                f"which the board does not have")
            continue
        rule_layers = rule.layers_for(copper)
        rule_layers = list(copper) if rule_layers is None else rule_layers
        eval_layers = rule_layers if _layer_dependent(ast) else [None]
        subsets = [frozenset(c) for k in range(2, len(names) + 1)
                   for c in combinations(names, k)]
        bound: Dict[Tuple[str, str], Dict[str, set]] = {}   # (area, kind) -> net -> layers
        try:
            for kind in kinds:
                facts = {'via_type': 'through'} if kind == 'via' else {}
                for nid, net in nets.items():
                    for layer in eval_layers:
                        item = rules.item_for_net(nid, kind, layer, **facts)

                        def ev(inside, item=item, layer=layer):
                            return bool(_eval(ast, replace(item, areas=inside), None, layer))

                        if ev(frozenset()):
                            raise _NotAKeepout(f"forbids {kind}s of {net} outside every "
                                               f"named area: not a keep-out")
                        single = {a: ev(frozenset([a])) for a in names}
                        for s in subsets:
                            if ev(s) != any(single[a] for a in s):
                                raise _NotAKeepout(
                                    f"combines rule areas {', '.join(sorted(s))} other "
                                    f"than as a union: not modelled")
                        for a, hit in single.items():
                            if hit and a in areas:
                                bound.setdefault((a, kind), {}).setdefault(net, set()).update(
                                    rule_layers if layer is None else [layer])
        except _NotAKeepout as e:
            rule.unsupported = str(e)
            continue
        except Unsupported as e:
            rule.unsupported = f"condition: {e}"
            continue
        # One region per (area polygon, bound nets, layers), tracks and vias
        # merged where they bind the same nets on the same layers.
        groups: Dict[Tuple[str, FrozenSet[str], FrozenSet[str]], set] = {}
        for (a, kind), by_net in bound.items():
            by_layers: Dict[FrozenSet[str], set] = {}
            for net, lays in by_net.items():
                by_layers.setdefault(frozenset(lays), set()).add(net)
            for lays, nset in by_layers.items():
                groups.setdefault((a, frozenset(nset), lays), set()).add(kind)
        kept_out: set = set()
        for (a, nset, lays), ks in sorted(groups.items(), key=lambda g: (g[0][0], sorted(g[0][1]))):
            for ko in areas[a]:
                layers = [l for l in _expand_layers(ko.get('layers'), copper) if l in lays]
                if not layers:
                    continue
                kept_out |= nset
                regions.append({
                    'name': a, 'rule': rule.name,
                    'polygon': ko['polygon'], 'holes': list(ko.get('holes') or []),
                    'layers': set(layers),
                    'tracks_allowed': 'track' not in ks, 'vias_allowed': 'via' not in ks,
                    'nets': nset})
        present = [n for n in names if n in areas]
        what = (f"honoured as a net-filtered keep-out: {len(kept_out)} of {len(nets)} "
                f"net(s) kept out of {', '.join(repr(n) for n in present)} "
                f"({', '.join(k + 's' for k in kinds)})")
        if missing:
            what += (f"; names rule area(s) {', '.join(repr(n) for n in missing)}, "
                     f"which the board does not have")
        if 'enclosedbyarea' in _area_fns(ast, set()):
            what += "; enclosedByArea read as intersectsArea (keeps out more)"
        rule.honoured = what
    return regions


def binds(ko: dict, net_name: str) -> bool:
    """True when keep-out `ko` forbids its items to net `net_name`."""
    return net_name in ko.get('nets', ())


def attach(pcb_data, keepouts: Optional[List[dict]]) -> None:
    """Record the run's net-filtered keep-outs on `pcb_data`. None or [] CLEARS
    any a previous run attached: the GUI hands every run the same long-lived
    PCBData, so a run on a board whose rule was removed (or a tab whose run
    reads no rule) must not keep stamping the previous run's regions."""
    if not keepouts:
        pcb_data.__dict__.pop('_net_keepouts', None)
        pcb_data.__dict__.pop('_net_keepout_cells', None)
        return
    pcb_data._net_keepouts = list(keepouts)
    pcb_data._net_keepout_cells = {}


class _Recorder:
    """Captures what add_rule_area_keepout_obstacles would stamp, so the keep-out
    rasterisation stays single-sourced."""

    def __init__(self):
        self.cells: List[np.ndarray] = []
        self.vias: List[np.ndarray] = []

    def add_blocked_cells_batch(self, arr):
        """Record (N, 3) track cells."""
        self.cells.append(np.asarray(arr, dtype=np.int32))

    def add_blocked_vias_batch(self, arr):
        """Record (N, 2) via cells."""
        self.vias.append(np.asarray(arr, dtype=np.int32))


def _binding(pcb_data, net_ids: Iterable[int]) -> Tuple[int, ...]:
    """Indices of the keep-outs that bind any of `net_ids`."""
    kos = getattr(pcb_data, '_net_keepouts', None) or []
    names = [getattr(pcb_data.nets.get(nid), 'name', '') for nid in set(net_ids) if nid]
    return tuple(i for i, ko in enumerate(kos)
                 if any(n and binds(ko, n) for n in names))


def _stamper(pcb_data, idx: Tuple[int, ...]):
    """`stamp(map, config)` that rasterises keep-outs `idx` into `map` with
    the native keep-out pass, so the rasterisation stays single-sourced."""
    from obstacle_map import add_rule_area_keepout_obstacles
    kos = pcb_data._net_keepouts
    shim = SimpleNamespace(board_info=SimpleNamespace(
        keepouts=[kos[i] for i in idx],
        board_bounds=getattr(pcb_data.board_info, 'board_bounds', None)))
    return lambda sink, cfg: add_rule_area_keepout_obstacles(sink, shim, cfg)


def _cells_for(pcb_data, idx: Tuple[int, ...], config) -> Tuple[np.ndarray, np.ndarray]:
    """(track cells (N, 3), via cells (M, 2)) keep-outs `idx` stamp at the
    run's geometry. Cached per (binding set, geometry, clip) on pcb_data.

    The clip (board_bounds) is part of the key because the rasteriser clips
    to it and a rescue window (plane_pad_tap.make_local_window) is a SHALLOW
    copy that shares this cache dict with the whole board: keyed on geometry
    alone, a window's clipped cells would be handed to a whole-board route
    at the same geometry, which would then see only part of the region."""
    key = (idx, config.grid_step, config.track_width, config.clearance,
           config.via_size, config.via_drill, tuple(config.layers),
           getattr(pcb_data.board_info, 'board_bounds', None))
    cache: Dict = pcb_data.__dict__.setdefault('_net_keepout_cells', {})
    if key not in cache:
        rec = _Recorder()
        _stamper(pcb_data, idx)(rec, config)
        cells = (np.unique(np.concatenate(rec.cells), axis=0) if rec.cells
                 else np.empty((0, 3), dtype=np.int32))
        vias = (np.unique(np.concatenate(rec.vias), axis=0) if rec.vias
                else np.empty((0, 2), dtype=np.int32))
        cache[key] = (cells, vias)
    return cache[key]


@contextlib.contextmanager
def stamp(obstacles, pcb_data, net_ids: Iterable[int], config):
    """Block the keep-outs binding any of `net_ids` in `obstacles` for the
    duration of the block, then remove exactly what was added.

    Via cells reach every via map the list twins (add/remove_vias_list) do:
    rung 0 at the run's via, the #568 small rung at the run's via (a superset
    for the smaller fab via), and each #530 per-net rung at its OWN via
    (#1226 `_stamp_rungs_at_own_via`, whose remove twin re-derives the same
    rows), so a search at any rung sees the region."""
    if obstacles is None or not getattr(pcb_data, '_net_keepouts', None):
        yield
        return
    idx = _binding(pcb_data, net_ids)
    if not idx:
        yield
        return
    cells, vias = _cells_for(pcb_data, idx, config)
    from obstacle_map import _rung_small_armed, _stamp_rungs_at_own_via
    small = _rung_small_armed() and hasattr(obstacles, 'add_blocked_vias_small_batch')
    rungs = len(vias) and hasattr(obstacles, 'rung_count')
    if len(cells):
        obstacles.add_blocked_cells_batch(cells)
    if len(vias):
        obstacles.add_blocked_vias_batch(vias)
        if small:
            obstacles.add_blocked_vias_small_batch(vias)
    if rungs:
        _stamp_rungs_at_own_via(obstacles, config, _stamper(pcb_data, idx))
    try:
        yield
    finally:
        if len(cells):
            obstacles.remove_blocked_cells_batch(cells)
        if len(vias):
            obstacles.remove_blocked_vias_batch(vias)
            if small:
                obstacles.remove_blocked_vias_small_batch(vias)
        if rungs:
            _stamp_rungs_at_own_via(obstacles, config, _stamper(pcb_data, idx),
                                    remove=True)


def centerline_config(config):
    """The geometry a diff-pair CENTRELINE map is stamped at. The members run
    (track_width + diff_pair_gap) / 2 either side of the centreline, so the
    band a region keeps the centreline out of grows by that offset -- the
    extra clearance route_diff builds every other obstacle of that map with.
    Stamped at the plain run geometry, a member could come up to
    (track_width + diff_pair_gap) / 2 - clearance INTO the region
    (lvds_converter_dualclk /CLK+: its copper over the edge by 0.5 um)."""
    from dataclasses import replace
    extra = (config.track_width + config.diff_pair_gap) / 2
    return replace(config, clearance=config.clearance + extra)


def stamping(net_ids_of, maps_of, centerline: bool = False):
    """Decorator for a route function that reaches the A* on its own maps:
    for the duration of each call, stamp the keep-outs binding
    ``net_ids_of(args)`` into every distinct map ``maps_of(args)`` names
    (``args`` = the call's bound arguments by parameter name; the function
    takes ``pcb_data`` and ``config``), at the centreline geometry
    (`centerline_config`) when the function lays a diff pair's centreline.
    Inert, beyond one getattr, on a board with no net-filtered keep-out.

    The body is called through ``__wrapped__`` at call time, so a test can
    swap in a probe and see what the A* would see."""
    def deco(fn):
        import functools
        import inspect
        sig = inspect.signature(fn)

        @functools.wraps(fn)
        def run(*args, **kwargs):
            pcb_data = args[0] if args else kwargs.get('pcb_data')
            if not getattr(pcb_data, '_net_keepouts', None):
                return run.__wrapped__(*args, **kwargs)
            a = sig.bind(*args, **kwargs).arguments
            nets, seen = tuple(net_ids_of(a)), []
            cfg = centerline_config(a['config']) if centerline else a['config']
            with contextlib.ExitStack() as stack:
                for m in maps_of(a):
                    if m is not None and not any(m is s for s in seen):
                        seen.append(m)
                        stack.enter_context(stamp(m, pcb_data, nets, cfg))
                return run.__wrapped__(*args, **kwargs)
        return run
    return deco


def _on_layer(ko_layers, layer: str) -> bool:
    """Does a region on `ko_layers` exist on copper layer `layer`? The
    composite tokens are the ones the keep-out rasteriser expands."""
    return (layer in ko_layers or '*.Cu' in ko_layers
            or (layer in ('F.Cu', 'B.Cu') and bool({'F&B.Cu', 'F&B'} & set(ko_layers))))


def _via_layers(via, copper: List[str]) -> List[str]:
    """The copper layers a via's barrel spans, in board order (a through via:
    all of them)."""
    ends = [l for l in (getattr(via, 'layers', None) or ()) if l in copper]
    if len(ends) < 2:
        return list(copper)
    i, j = sorted((copper.index(ends[0]), copper.index(ends[-1])))
    return copper[i:j + 1]


def audit(pcb_data, segments, vias,
          keepouts: Optional[List[dict]] = None) -> Optional[List[dict]]:
    """The copper of `segments` / `vias` that a net-filtered keep-out forbids
    and that INTERSECTS its region (the polygon minus its holes) -- what
    KiCad's DRC reports as items_not_allowed for the rule. One row per
    (region, net): {'region', 'net', 'tracks', 'vias'}, sorted; [] when the
    copper is clean, None when the run has no net-filtered keep-out.

    It reads only geometry, so it sees copper from every path, including the
    ones stamp() does not reach (the plane finalize's taps and joins, post-route
    meanders) and copper the input already had inside a region.

    `pcb_data` supplies the net names and copper layers of the copper given,
    and the keep-outs unless `keepouts` names them (a re-parsed output board
    carries none of its own)."""
    from geometry_utils import segment_to_segment_distance
    from obstacle_map import point_in_polygon, point_to_polygon_edge_distance
    kos = keepouts if keepouts is not None else getattr(pcb_data, '_net_keepouts', None)
    if not kos:
        return None
    names = {nid: n.name for nid, n in pcb_data.nets.items() if nid and n.name}
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    eps = 1e-6
    rows: Dict[Tuple[str, str], List[int]] = {}
    for ko in kos:
        ring = ko['polygon']
        rings = [ring] + [h for h in (ko.get('holes') or []) if len(h) >= 3]
        edges = [e for rg in rings for e in zip(rg, rg[1:] + rg[:1])]
        x0 = min(p[0] for p in ring)
        y0 = min(p[1] for p in ring)
        x1 = max(p[0] for p in ring)
        y1 = max(p[1] for p in ring)

        def inside(x, y):
            # even-odd over the outline and its holes
            return sum(point_in_polygon(x, y, rg) for rg in rings) % 2 == 1

        def near(x, y, r):
            return any(point_to_polygon_edge_distance(x, y, rg) < r - eps for rg in rings)

        def hit(xa, ya, xb, yb, r):
            if (max(xa, xb) < x0 - r or min(xa, xb) > x1 + r
                    or max(ya, yb) < y0 - r or min(ya, yb) > y1 + r):
                return False
            if inside(xa, ya) or inside(xb, yb):
                return True
            return any(segment_to_segment_distance(xa, ya, xb, yb, a[0], a[1], b[0], b[1])
                       < r - eps for a, b in edges)

        def row(nid):
            name = names.get(nid)
            if not name or not binds(ko, name):
                return None
            return rows.setdefault((ko['name'], name), [0, 0])

        if not ko.get('tracks_allowed', True):
            for s in segments:
                if getattr(s, 'graphic', False) or not _on_layer(ko['layers'], s.layer):
                    continue
                r = row(s.net_id)
                if r is not None and hit(s.start_x, s.start_y, s.end_x, s.end_y,
                                         s.width / 2):
                    r[0] += 1
        if not ko.get('vias_allowed', True):
            for v in vias:
                if not any(_on_layer(ko['layers'], l) for l in _via_layers(v, copper)):
                    continue
                r = row(v.net_id)
                if r is not None and (inside(v.x, v.y) or near(v.x, v.y, v.size / 2)):
                    r[1] += 1
    return [{'region': k[0], 'net': k[1], 'tracks': t, 'vias': n}
            for k, (t, n) in sorted(rows.items()) if t or n]


def format_audit(found: List[dict], where: str) -> str:
    """One line for the console: what `audit` found on `where`."""
    if not found:
        return f"Net-filtered keep-outs: no bound-net copper inside a region on {where}"
    items = sum(r['tracks'] + r['vias'] for r in found)
    parts = '; '.join(f"{r['region']}: {r['net']} "
                      f"({r['tracks']} track(s), {r['vias']} via(s))" for r in found)
    return (f"WARNING: net-filtered keep-outs: {items} bound-net item(s) inside a "
            f"region on {where} (KiCad reports them as items_not_allowed): {parts}")
