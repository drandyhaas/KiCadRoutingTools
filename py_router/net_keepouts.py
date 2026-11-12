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

An `edge_clearance` rule that binds some nets harder than others (#1678,
`derive_edge_bands`) takes the same stamp: the board's static edge band is the
lowest floor over the nets, and a net above it gets the rest while it routes.

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
and its legs (`stamping`). The plane finalize's region joins and oracle welds
route on a map built for one plane net, which `block()` stamps for good
(plane_region_connector.build_base_obstacles); its pad taps judge their sites
and copper by `audit_items`. Copper laid by anything else, such as post-route
meanders, does not see a region, so the run ends with `audit()` of the board
it ships: bound-net copper found inside a region is printed and reported as
JSON_SUMMARY / --json-out `net_keepouts`, never shipped silently. KiCad's DRC of
the rule stays the authority.
"""
from __future__ import annotations

import contextlib
import os
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


def derive_layer_bans(rules, pcb_data) -> List[dict]:
    """The track bans the board's scope-evaluable `disallow` rules define
    (#1527): `(rule "hs_layers" (constraint disallow track) (condition
    "A.NetClass == 'HS' && A.Layer != 'F.Cu'"))` keeps the HS class's tracks
    off every layer but F.Cu. Such a rule names no rule area, so it is not a
    keep-out region: what it binds is a set of (net, layer) pairs, and the
    router honours it by routing each bound net with those layers FORBIDDEN
    (`config_for_nets`: the Rust router's negative layer cost -- no track on
    the layer, through vias may still span it), exactly as KiCad grades it
    (items_not_allowed per track, never per via).

    The bound pairs come from the resolver's own evaluator, per net and
    copper layer, for a track item on that layer (`_rule_matches`: the
    rule's layer clause and its condition). Returns one region dict per
    (rule, bound layers, bound nets) in the `derive_area_keepouts` shape --
    `layer_ban` True, no polygon -- so `audit` grades the shipped copper
    against it like any region. Sets each rule's `honoured` to what it
    honours and leaves `router_unhonoured` naming only what it does not (a
    `via` token: no step keeps a net's vias off)."""
    from design_rules import _rule_matches, disallow_router_reason
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None)
                  or rules.copper_layers or [])
    nets = {nid: n.name for nid, n in pcb_data.nets.items() if nid and n.name}
    names = frozenset(nets.values())
    out: List[dict] = []
    for rule in rules.rules:
        if rule.unsupported or 'disallow' not in rule.constraints:
            continue
        tokens = set(rule.constraints['disallow'].get('disallow', ()))
        if 'track' not in tokens:
            continue
        by_layers: Dict[FrozenSet[str], set] = {}
        ctx = ('disallow', names)
        for nid, net in nets.items():
            banned = frozenset(
                l for l in copper
                if _rule_matches(rule, rules.item_for_net(nid, 'track', l), None,
                                 l, copper, ctx))
            if banned:
                by_layers.setdefault(banned, set()).add(net)
        n_nets = 0
        for lays, nset in sorted(by_layers.items(), key=lambda g: sorted(g[0])):
            n_nets += len(nset)
            # no rule area: the "region" is the rule's layers, board-wide
            out.append({'name': f"{rule.name} (forbidden layers)", 'rule': rule.name,
                        'polygon': None, 'holes': [], 'layers': set(lays),
                        'tracks_allowed': False, 'vias_allowed': True,
                        'nets': frozenset(nset), 'layer_ban': True})
        layers_hit = sorted({l for lays in by_layers for l in lays},
                            key=lambda l: copper.index(l) if l in copper else 0)
        rule.honoured = (f"honoured as forbidden layers: {n_nets} of {len(nets)} "
                         f"net(s) route no track on "
                         f"{', '.join(layers_hit) or 'any layer'}")
        rule.router_unhonoured = disallow_router_reason(tokens, {'track'})
    return out


#: The `disallow` tokens that forbid the through via, the only via a routing
#: step lays (`design_rules.DISALLOW_NOT_LAID` names the others)
_THROUGH_VIA_TOKENS = frozenset({'via', 'through_via'})


def planar_nets(pcb_data, board_path: Optional[str] = None) -> Dict[int, dict]:
    """#1818: the nets the board's `.kicad_dru` keeps on ONE copper layer with
    no via, read as the routing steps read them: the track bans
    `derive_layer_bans` derives leave the net one permitted layer, and a
    rule forbids its through via (`disallow via` / `through_via`) or allows
    it none (`via_count (max 0)`). `{net_id: {'name', 'layer', 'rules'}}`,
    {} on a board with no rules file or no such net. Such a net reaches a
    pad only on that layer: a via cannot carry it to a pad's other face."""
    path = board_path or getattr(pcb_data, 'source_path', '') or ''
    if not path or not os.path.isfile(os.path.splitext(path)[0] + '.kicad_dru'):
        return {}
    import io
    from design_rules import DesignRules
    with contextlib.redirect_stdout(io.StringIO()):
        dr = DesignRules.from_project(pcb_data, path)
    if not any('disallow' in r.constraints or 'via_count' in r.constraints
               for r in dr.rules):
        return {}
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None)
                  or dr.copper_layers or [])
    banned: Dict[str, set] = {}
    ban_rules: Dict[str, set] = {}
    for ko in derive_layer_bans(dr, pcb_data):
        for name in ko['nets']:
            banned.setdefault(name, set()).update(ko['layers'])
            ban_rules.setdefault(name, set()).add(ko['rule'])
    out: Dict[int, dict] = {}
    for nid, net in pcb_data.nets.items():
        name = getattr(net, 'name', '')
        if not nid or not name or name not in banned:
            continue
        left = [l for l in copper if l not in banned[name]]
        if len(left) != 1:
            continue
        via = dr.item_for_net(nid, 'via', via_type='through')
        d = dr.resolve('disallow', via, None, None)
        vc = dr.resolve('via_count', via, None, None)
        if _THROUGH_VIA_TOKENS & set(d.disallow or ()):
            why = d.source
        elif vc.max is not None and vc.max <= 0:
            why = vc.source
        else:
            continue
        rules = sorted(ban_rules[name]) + [why.replace('rule ', '').strip('"')]
        out[nid] = {'name': name, 'layer': left[0], 'rules': sorted(set(rules))}
    return out


def config_for_nets(pcb_data, net_ids: Iterable[int], config):
    """`config` for routing `net_ids` under the board's track bans
    (`derive_layer_bans`): a shallow copy whose `layer_costs` FORBID every
    layer a ban binding any of the nets names, or `config` itself when none
    binds them (every board without such a rule: one getattr)."""
    banned = banned_layers(pcb_data, net_ids)
    if not banned:
        return config
    layers = list(getattr(config, 'layers', None) or ())
    if not any(l in banned for l in layers):
        return config
    import copy
    from routing_constants import FORBIDDEN_LAYER_COST
    costs = list(getattr(config, 'layer_costs', None) or [])
    costs += [1.0] * (len(layers) - len(costs))
    cfg = copy.copy(config)
    cfg.layer_costs = [FORBIDDEN_LAYER_COST if l in banned else c
                       for l, c in zip(layers, costs)]
    return cfg


def banned_layers(pcb_data, net_ids: Iterable[int]) -> FrozenSet[str]:
    """The copper layers the board's track bans forbid any of `net_ids`."""
    kos = getattr(pcb_data, '_net_keepouts', None)
    if not kos:
        return frozenset()
    names = [getattr(pcb_data.nets.get(nid), 'name', '') for nid in set(net_ids) if nid]
    out = set()
    for ko in kos:
        if ko.get('layer_ban') and any(n and binds(ko, n) for n in names):
            out |= ko['layers']
    return frozenset(out)


def derive_edge_bands(floors, pcb_data) -> List[dict]:
    """One entry per distinct set of `design_rules.EdgeFloors` a net holds above
    the board's static band: {'name', 'edge_band': True, 'nets', 'track': {layer:
    floor}, 'via': floor}. [] when every net is at the band."""
    groups: Dict[tuple, set] = {}
    for nid, name in ((nid, getattr(n, 'name', '')) for nid, n in pcb_data.nets.items()):
        if not nid or not name or nid not in floors.track:
            continue
        track = {l: v for l, v in floors.track[nid].items() if v > floors.track_band + 1e-9}
        via = floors.via[nid] if floors.via[nid] > floors.via_band + 1e-9 else None
        if track or via is not None:
            groups.setdefault((tuple(sorted(track.items())), via), set()).add(name)
    return [{'name': 'edge clearance', 'edge_band': True, 'nets': frozenset(names),
             'track': dict(track), 'via': via, 'layers': ['*.Cu']}
            for (track, via), names in sorted(groups.items(), key=lambda kv: repr(kv[0]))]


def derive_contact_fingers(pcb_data) -> List[dict]:
    """One via-only region per copper pad of a mating-contact footprint
    (#1116): a PCB-edge plug (`floorplan.is_board_copper_plug`, #1098) or
    card-edge fingers (`part_class.is_card_fingers`). A drilled hole in a
    contact finger damages the plating the socket wipes against, so no net's
    via may touch the pad's copper; tracks stay allowed, since they must
    leave the finger. [] when the board has no such footprint."""
    import _placer_path  # noqa: F401  (placement lives in py_placer/)
    from placement.floorplan import is_board_copper_plug
    from placement.part_class import is_card_fingers
    from check_pads import pad_outline_polygon
    from paste_apertures import pad_has_copper
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    names = frozenset(n.name for nid, n in pcb_data.nets.items() if nid and n.name)
    out: List[dict] = []
    for ref, fp in sorted((getattr(pcb_data, 'footprints', None) or {}).items()):
        if not (is_board_copper_plug(fp) or is_card_fingers(fp)):
            continue
        for p in fp.pads:
            if not pad_has_copper(p) or p.size_x <= 0 or p.size_y <= 0:
                continue
            out.append({'name': f"contact finger {ref}.{p.pad_number}", 'rule': None,
                        'polygon': pad_outline_polygon(p), 'holes': [],
                        'layers': {l for l in p.layers if l in copper} or set(copper),
                        'tracks_allowed': True, 'vias_allowed': False, 'nets': names})
    return out


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


class _EdgeBandSink:
    """What add_board_edge_obstacles stamps as permanent board geometry, handed
    to a map a net's band comes and goes on: track cells on `layers` (indices)
    and via cells, each only when asked for."""

    def __init__(self, sink, layers=None, vias=False):
        self.sink, self.layers, self.vias = sink, layers, vias

    def _cells(self, arr):
        if self.layers is None:
            return
        a = np.asarray(arr, dtype=np.int32).reshape(-1, 3)
        a = a[np.isin(a[:, 2], list(self.layers))]
        if len(a):
            self.sink.add_blocked_cells_batch(a)

    def _vias(self, arr):
        if self.vias:
            a = np.asarray(arr, dtype=np.int32).reshape(-1, 2)
            if len(a):
                self.sink.add_blocked_vias_batch(a)

    add_static_blocked_cells_batch = add_blocked_cells_batch = _cells
    add_static_blocked_vias_batch = add_blocked_vias_batch = _vias


def _stamp_edge_band(sink, pcb_data, cfg, ko: dict) -> None:
    """The native edge pass, at the floors ko holds its nets to."""
    from dataclasses import replace
    from obstacle_map import add_board_edge_obstacles
    extra = getattr(cfg, 'edge_extra_clearance', 0.0) or 0.0
    index = {l: i for i, l in enumerate(cfg.layers)}
    by_value: Dict[float, list] = {}
    for layer, v in ko['track'].items():
        if layer in index:
            by_value.setdefault(v, []).append(index[layer])
    for v, layers in by_value.items():
        add_board_edge_obstacles(_EdgeBandSink(sink, layers=layers), pcb_data,
                                 replace(cfg, board_edge_clearance=v, edge_floors=None), extra)
    if ko['via'] is not None:
        add_board_edge_obstacles(_EdgeBandSink(sink, vias=True), pcb_data,
                                 replace(cfg, board_edge_clearance=ko['via'], edge_floors=None),
                                 extra)


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
    """Indices of the keep-out REGIONS that bind any of `net_ids` (a layer
    ban is no region: `config_for_nets` honours it)."""
    kos = getattr(pcb_data, '_net_keepouts', None) or []
    names = [getattr(pcb_data.nets.get(nid), 'name', '') for nid in set(net_ids) if nid]
    return tuple(i for i, ko in enumerate(kos)
                 if not ko.get('layer_ban') and any(n and binds(ko, n) for n in names))


def _stamper(pcb_data, idx: Tuple[int, ...]):
    """`stamp(map, config)` that rasterises keep-outs `idx` into `map` with
    the native keep-out pass, so the rasterisation stays single-sourced."""
    from obstacle_map import add_rule_area_keepout_obstacles
    from routing_config import extra_past_scalar
    kos = pcb_data._net_keepouts
    shim = SimpleNamespace(board_info=SimpleNamespace(
        keepouts=[kos[i] for i in idx if not kos[i].get('edge_band')],
        board_bounds=getattr(pcb_data.board_info, 'board_bounds', None)))
    bands = [kos[i] for i in idx if kos[i].get('edge_band')]

    def stamp_into(sink, cfg):
        if shim.board_info.keepouts:
            # a centreline config's clearance carries the member half-pitch's
            # scalar; what a layer's own half-pitch adds past it rides here
            add_rule_area_keepout_obstacles(
                sink, shim, cfg, extra_clearance=extra_past_scalar(
                    getattr(cfg, 'edge_extra_clearance', 0.0) or 0.0))
        for ko in bands:
            _stamp_edge_band(sink, pcb_data, cfg, ko)
    return stamp_into


def _cells_for(pcb_data, idx: Tuple[int, ...], config) -> Tuple[np.ndarray, np.ndarray]:
    """(track cells (N, 3), via cells (M, 2)) keep-outs `idx` stamp at the
    run's geometry. Cached per (binding set, geometry, clip) on pcb_data.

    The clip (board_bounds) is part of the key because the rasteriser clips
    to it and a rescue window (plane_pad_tap.make_local_window) is a SHALLOW
    copy that shares this cache dict with the whole board: keyed on geometry
    alone, a window's clipped cells would be handed to a whole-board route
    at the same geometry, which would then see only part of the region."""
    key = (idx, config.grid_step, config.track_width, config.clearance,
           getattr(config, 'edge_extra_clearance', 0.0),
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


def block(obstacles, pcb_data, net_ids: Iterable[int], config):
    """Block the keep-outs binding any of `net_ids` in `obstacles`; returns
    the call that removes exactly what was added (`stamp`). A map built for
    those nets alone (the plane finalize's base map) keeps them for good.

    Via cells reach every via map the list twins (add/remove_vias_list) do:
    rung 0 at the run's via, the #568 small rung at the run's via (a superset
    for the smaller fab via) when the map populates it, and each #530 per-net rung at its OWN via
    (#1226 `_stamp_rungs_at_own_via`, whose remove twin re-derives the same
    rows), so a search at any rung sees the region."""
    if obstacles is None or not getattr(pcb_data, '_net_keepouts', None):
        return lambda: None
    idx = _binding(pcb_data, net_ids)
    if not idx:
        return lambda: None
    cells, vias = _cells_for(pcb_data, idx, config)
    from obstacle_map import (_rung_small_armed, _stamp_rungs_at_own_via,
                              populated_via_rungs)
    # #1070: an unpopulated small rung falls back to rung 0, which sees the
    # region; stamped, it would hold the region ALONE (a rescue clone map)
    small = (_rung_small_armed() and hasattr(obstacles, 'add_blocked_vias_small_batch')
             and 1 in populated_via_rungs(obstacles))
    rungs = len(vias) and hasattr(obstacles, 'rung_count')
    if len(cells):
        obstacles.add_blocked_cells_batch(cells)
    if len(vias):
        obstacles.add_blocked_vias_batch(vias)
        if small:
            obstacles.add_blocked_vias_small_batch(vias)
    if rungs:
        _stamp_rungs_at_own_via(obstacles, config, _stamper(pcb_data, idx))

    def remove():
        if len(cells):
            obstacles.remove_blocked_cells_batch(cells)
        if len(vias):
            obstacles.remove_blocked_vias_batch(vias)
            if small:
                obstacles.remove_blocked_vias_small_batch(vias)
        if rungs:
            _stamp_rungs_at_own_via(obstacles, config, _stamper(pcb_data, idx),
                                    remove=True)
    return remove


@contextlib.contextmanager
def stamp(obstacles, pcb_data, net_ids: Iterable[int], config):
    """The keep-outs binding any of `net_ids`, blocked in `obstacles` for the
    duration of the block (`block`)."""
    remove = block(obstacles, pcb_data, net_ids, config)
    try:
        yield
    finally:
        remove()


def centerline_config(config):
    """The geometry a diff-pair CENTRELINE map is stamped at. The members run
    (layer width + diff_pair_gap) / 2 either side of the centreline, so the
    band a region keeps the centreline out of grows by that offset -- the
    extra clearance route_diff builds every other obstacle of that map with
    (`centreline_extra_clearance`, each layer's own, #1729): the clearance
    takes the call's, and `edge_extra_clearance` carries the per-layer values
    (the region stamp reads what a layer adds past the call's).
    Stamped at the plain run geometry, a member could come up to
    (track_width + diff_pair_gap) / 2 - clearance INTO the region
    (lvds_converter_dualclk /CLK+: its copper over the edge by 0.5 um)."""
    from dataclasses import replace
    from diff_pair_routing import centreline_extra_clearance
    extra = centreline_extra_clearance(config)
    return replace(config, clearance=config.clearance + float(extra),
                   edge_extra_clearance=extra)


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
            bound = sig.bind(*args, **kwargs)
            a = bound.arguments
            nets, seen = tuple(net_ids_of(a)), []
            cfg = centerline_config(a['config']) if centerline else a['config']
            # #1527: the nets' track bans, as forbidden layers of the
            # config the body routes with
            routed = config_for_nets(pcb_data, nets, a['config'])
            if routed is not a['config']:
                a['config'] = routed
                args, kwargs = bound.args, bound.kwargs
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


def audit_items(pcb_data, segments, vias, keepouts):
    """Every item of `segments` / `vias` a net-filtered keep-out forbids and
    that INTERSECTS its region (the polygon minus its holes), or -- a track
    ban (#1527) -- that lies on one of its layers: `(keep-out, 'track' |
    'via', item)`, in keep-out order. `audit` counts them per (region, net);
    check_drc grades each one as KiCad's items_not_allowed."""
    from geometry_utils import segment_to_segment_distance
    from obstacle_map import point_in_polygon, point_to_polygon_edge_distance
    names = {nid: n.name for nid, n in pcb_data.nets.items() if nid and n.name}
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    eps = 1e-6

    def bound(ko, nid):
        name = names.get(nid)
        return bool(name) and binds(ko, name)

    for ko in keepouts:
        if ko.get('edge_band'):
            continue    # edge_rule_grade's, and KiCad reports it under another name
        if ko.get('layer_ban'):
            # a track ban covers the board: every bound-net track on one of
            # its layers is an item KiCad does not allow
            for s in segments:
                if (not getattr(s, 'graphic', False) and s.layer in ko['layers']
                        and bound(ko, s.net_id)):
                    yield ko, 'track', s
            continue
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

        if not ko.get('tracks_allowed', True):
            for s in segments:
                if getattr(s, 'graphic', False) or not _on_layer(ko['layers'], s.layer):
                    continue
                if bound(ko, s.net_id) and hit(s.start_x, s.start_y, s.end_x, s.end_y,
                                               s.width / 2):
                    yield ko, 'track', s
        if not ko.get('vias_allowed', True):
            for v in vias:
                if not any(_on_layer(ko['layers'], l) for l in _via_layers(v, copper)):
                    continue
                if bound(ko, v.net_id) and (inside(v.x, v.y) or near(v.x, v.y, v.size / 2)):
                    yield ko, 'via', v


def via_site_filter(pcb_data, net_id: int, via_size: float, layers=None):
    """An (x, y) -> bool predicate: may a via of `net_id` (diameter
    `via_size`, spanning `layers`; None = a through via) sit at (x, y)
    without entering a region that forbids it the net's vias? None when no
    region does, so a caller adds nothing to its search.

    For a via search that places on maps of its own, which `stamp()` never
    reaches (the pad taps): skipping the sites `audit` would refuse lets the
    search go on to the next site rather than fail. It judges each site by
    `audit_items`, so the two cannot disagree; a bounding-box test keeps the
    far sites cheap."""
    net = pcb_data.nets.get(net_id)
    name = getattr(net, 'name', None)
    kos = [ko for ko in getattr(pcb_data, '_net_keepouts', None) or ()
           if not ko.get('edge_band') and not ko.get('layer_ban')
           and not ko.get('vias_allowed', True) and name and binds(ko, name)]
    if not kos:
        return None
    view = SimpleNamespace(nets={net_id: net}, board_info=pcb_data.board_info)
    r = via_size / 2
    boxes = [(min(p[0] for p in ko['polygon']) - r, min(p[1] for p in ko['polygon']) - r,
              max(p[0] for p in ko['polygon']) + r, max(p[1] for p in ko['polygon']) + r)
             for ko in kos]
    vlayers = list(layers or ())

    def allowed(x, y):
        near = [ko for ko, (x0, y0, x1, y1) in zip(kos, boxes)
                if x0 <= x <= x1 and y0 <= y <= y1]
        if not near:
            return True
        via = SimpleNamespace(x=x, y=y, size=via_size, layers=vlayers, net_id=net_id)
        return next(audit_items(view, (), (via,), near), None) is None

    return allowed


def audit(pcb_data, segments, vias,
          keepouts: Optional[List[dict]] = None) -> Optional[List[dict]]:
    """The copper of `segments` / `vias` that a net-filtered keep-out forbids
    and that INTERSECTS its region (the polygon minus its holes) -- what
    KiCad's DRC reports as items_not_allowed for the rule (`audit_items`).
    One row per (region, net): {'region', 'net', 'tracks', 'vias'}, sorted;
    [] when the copper is clean, None when the run has no net-filtered
    keep-out.

    It reads only geometry, so it sees copper from every path, including the
    ones stamp() does not reach (post-route meanders) and copper the input
    already had inside a region.

    `pcb_data` supplies the net names and copper layers of the copper given,
    and the keep-outs unless `keepouts` names them (a re-parsed output board
    carries none of its own)."""
    kos = keepouts if keepouts is not None else getattr(pcb_data, '_net_keepouts', None)
    kos = [ko for ko in kos or () if not ko.get('edge_band')]
    if not kos:
        return None
    rows: Dict[Tuple[str, str], List[int]] = {}
    for ko, kind, item in audit_items(pcb_data, segments, vias, kos):
        r = rows.setdefault((ko['name'], pcb_data.nets[item.net_id].name), [0, 0])
        r[0 if kind == 'track' else 1] += 1
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
