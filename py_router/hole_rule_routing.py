"""#1853: route to the board's `.kicad_dru` `hole_clearance` rules.

KiCad holds the copper of a track, via or pad off the HOLE of another net's
via, plated pad or NPTH at the last `hole_clearance` rule that binds the pair
(`check_drc.hole_rule_grade` grades it). The route steps kept copper off a
plated hole at the copper clearance to its ring or pad, and off an NPTH at
Board Setup's minimum, whatever a rule said.

`RoutedHoleRules` answers, for one hole and one copper layer, the value the
rules give against the copper a step may lay: the largest over the board's
nets (their own net aside, which KiCad exempts), since one obstacle map
serves every net a step routes. The walk is `design_rules.rule_may_match`,
the one the grader uses: last matching rule, condition read either way
round, the rule's `(layer ...)` clause against the copper's layer. A rule of
severity ignore binds nothing; a rule whose condition is not read binds
nothing here and is named.

A rule scoped by a rule area (`A.insideArea('ROOM')`, `intersectsArea`,
`enclosedByArea`) binds by where each item stands, as check_drc grades it
(`check_drc.HoleRuleBinder`): a hole of the board is measured against the
board's rule areas, so a via or pad hole inside the area is held at the rule
and one outside it is not, where the condition asks the hole. The copper a
step lays has no position here, so where the condition turns on THAT item
being in an area -- one drawn on the layer asked, within the rule's reach of
the hole (or the copper) it is held off, the reach the stamp itself has --
the walk is tighten-safe: the rule may bind, so the pair is held at the
larger of the rule's value and what the rules before it give. Copper that
reaches into the area only along a segment that passes the hole outside it
is the one case this does not hold; the step's own grade of the copper it
laid (`check_drc.hole_rule_step_grade`) names any such row.
"""

import fnmatch
from dataclasses import replace
from typing import Dict, Optional, Sequence


def _nodes(ast):
    """Every node of a condition AST."""
    if not isinstance(ast, tuple) or not ast:
        return
    yield ast
    for c in ast[1:]:
        if isinstance(c, tuple):
            yield from _nodes(c)


class RoutedHoleRules:
    """The walk, and the nets and layers it is asked for.

    Two nets read alike to these rules when they have the same classes and
    every net-dependent term of the rules' conditions (a net name, a class, a
    pair membership: each `cmp` and `call` node, evaluated by the grader's
    own evaluator) reads the same on both, so each such group stands in for
    all of its nets: two of them, since a hole is never held off its own net
    and a condition may compare the two items' nets. A condition on
    `AB.isCoupledDiffPair()` relates two nets, so under one every pair net
    is its own group. Two copper layers read alike when every rule's
    `(layer ...)` clause and every string the conditions compare holds or
    fails on both; the outer layers are never merged.

    Cost: the walk runs once per pair of groups (and layer) that read apart,
    so a condition naming nets one by one makes each named net a group of
    its own. Measured: a rule ORing 200 net names on a 250-net four-layer
    board adds about half a minute to a base-map build. A condition on a
    class or a name pattern adds nothing of the kind."""

    def __init__(self, dr, pcb_data, new_via_type: str = 'through',
                 new_via_layers: Optional[Sequence[str]] = None,
                 copper_reach: float = 0.0):
        from design_rules import parse_condition_maybe, _eval3, match_dp_suffix, _wx_matches
        self.dr = dr
        self.walk = []
        if pcb_data is not None and getattr(pcb_data, 'board_info', None) is not None and any(
                'hole_clearance' in r.constraints and r.area_scoped_condition is not None
                for r in dr.rules):
            try:
                dr.area_index(pcb_data)
            except Exception:                                   # noqa: BLE001
                pass                    # the rule stays unread, and is named
        index = getattr(dr, '_area_index', None)
        #: rule-area name -> the copper layers the area is drawn on
        self.area_layers = {a[0]: frozenset(a[1]) for a in (index.areas if index else ())}
        #: the rules that bind by a rule area, read by position (`_area_match`)
        self.positional = set()
        for r in dr.rules:
            spec = r.constraints.get('hole_clearance')
            if spec is None:
                continue
            if index is not None and r.area_scoped_condition is not None:
                ast = r.area_scoped_condition
                self.positional.add(id(r))
            else:
                ast = (('unknown',) if str(r.unsupported or '').startswith('constraint')
                       else parse_condition_maybe(r.condition_src))
            self.walk.append((r, ast, float(spec.get('min') or 0.0)))
        self._index = index if self.positional else None
        # how far from a measured item's copper the copper a step lays beside
        # it may reach: the widest positional rule, and the widest copper
        # (``copper_reach``: a new via's diameter, the track width)
        self._near_r = max([mn for r, _a, mn in self.walk if id(r) in self.positional]
                           or [0.0]) + float(copper_reach or 0.0)
        self.ctx = ('hole_clearance', dr._net_name_set())
        self.unread: Dict[str, str] = {}
        self._memo: Dict[tuple, Optional[float]] = {}
        self._decided: Dict[tuple, Optional[float]] = {}
        nodes = [n for _r, ast, _m in self.walk for n in _nodes(ast)]
        atoms, seen = [], set()
        for n in nodes:
            if n[0] in ('cmp', 'call') and repr(n) not in seen:
                seen.add(repr(n))
                atoms.append(n)
        coupled = any(n[0] == 'coupled' for n in nodes)
        # the item facts no condition reads, left out of a hole's memo key
        reads = {n[2] for n in nodes if n[0] in ('call', 'prop')}
        self._blind = {}
        if 'memberoffootprint' not in reads:
            self._blind.update(footprint_ref=None, footprint_lib=None)
        if 'pad_number' not in reads:
            self._blind['pad_number'] = None
        if 'memberofgroup' not in reads:
            self._blind['groups'] = frozenset()
        self._net_key: Dict[int, tuple] = {}
        groups: Dict[tuple, list] = {}
        for nid, net in sorted((getattr(pcb_data, 'nets', None) or {}).items()):
            name = getattr(net, 'name', None)
            if not nid or not name:
                continue
            it = dr.item_for_net(int(nid), 'track')
            key = (dr.memberships.get(nid, frozenset()), dr.effective_class(nid),
                   tuple(_eval3(n, it, it, None, self.ctx) for n in atoms),
                   match_dp_suffix(name)[::2] if coupled else None)
            self._net_key[int(nid)] = key
            g = groups.setdefault(key, [])
            if len(g) < 2:
                g.append(int(nid))
        self.groups = groups
        self.reps = sorted(n for g in groups.values() for n in g)
        # the layers a walk can tell apart
        cu = list(dr.copper_layers)
        lits = sorted({n[1] for n in nodes if n[0] == 'lit' and isinstance(n[1], str)}
                      | {a for n in nodes if n[0] == 'call' for a in n[3]
                         if isinstance(a, str)})
        lays = [r.layers_for(cu) for r, _a, _m in self.walk]

        areas = sorted(self.area_layers.items())

        def _sig(layer):
            return (layer == cu[0] if cu else False, layer == cu[-1] if cu else False,
                    tuple(lay is None or layer in lay for lay in lays),
                    tuple(layer in al for _n, al in areas),
                    tuple(layer == q or _wx_matches(q, layer) or fnmatch.fnmatchcase(layer, q)
                          for q in lits))
        rep: Dict[tuple, str] = {}
        self._layer_rep = {l: rep.setdefault(_sig(l), l) for l in cu}
        self.layers = [l for l in cu if self._layer_rep[l] == l]
        self.new_via_type = new_via_type or 'through'
        self.new_via_layers = [l for l in (new_via_layers or cu) if l in cu] or cu

    def __bool__(self):
        return bool(self.walk)

    def _others(self, own: int):
        """One net of each group other than ``own``: a group reads alike to
        the rules, and two different nets never compare equal by net."""
        for g in self.groups.values():
            nid = g[0] if g[0] != own else (g[1] if len(g) > 1 else None)
            if nid is not None:
                yield nid

    def _decide(self, a, b, layer):
        from design_rules import rule_may_match
        key = (a, b, layer)
        if key in self._decided:
            return self._decided[key]
        vals = []
        for r, ast, mn in reversed(self.walk):
            if id(r) in self.positional:
                m = self._area_match(r, ast, a, b, layer)
                if m is None:
                    # it may bind where the copper stands: tighten-safe, the
                    # larger of its value and what the rules before it give
                    if r.severity not in ('ignore', 'exclusion'):
                        vals.append(mn)
                    continue
            else:
                m = rule_may_match(r, ast, a, b, layer, self.dr.copper_layers, self.ctx)
            if m is None:
                # not read here: it binds nothing and is named NOT honoured;
                # the rules before it still decide the pair
                self.unread.setdefault(r.name, r.unsupported
                                       or 'its condition is not read here')
                continue
            if m:
                if r.severity not in ('ignore', 'exclusion'):
                    vals.append(mn)
                break
        out = max(vals) if vals else None
        self._decided[key] = out
        return out

    def _area_match(self, r, ast, a, b, layer):
        """`rule_may_match` for a rule scoped by a rule area: an item whose
        areas were measured (a hole of the board) is in an area or not; for
        one that was not (the copper a step lays), an area predicate is
        unknown where the area is drawn on the layer asked, else false."""
        from design_rules import rule_may_match, _eval3
        lay = rule_may_match(r, None, a, b, layer, self.dr.copper_layers, self.ctx)
        if lay is False:
            return False
        best = False
        for x, y in ((a, b), (b, a)) if b is not None else ((a, b),):
            v = _eval3(self._placed(ast, x, y, layer), x, y, layer, self.ctx)
            if v is True:
                return None if lay is None else True
            if v is None:
                best = None
        return best

    def _placed(self, node, a, b, layer):
        """``node`` with each rule-area predicate answered for the pair."""
        from design_rules import _eval, _item
        if not isinstance(node, tuple) or not node:
            return node
        if node[0] == 'area':
            it = _item(node[1], a, b)
            if it is None:
                return ('lit', False)
            if it.area_layers is not None:
                return ('lit', bool(_eval(node, a, b, layer)))
            other = _item('B' if node[1] == 'A' else 'A', a, b)
            if other is not None and other.area_near is not None:
                # beside a measured item: only an area within its reach
                on = frozenset(l for n, l in other.area_near if n == node[3])
            else:
                on = self.area_layers.get(node[3], frozenset())
            if node[2] == 'enclosedbyarea' and it.type == 'via':
                possible = bool(on)
            else:
                possible = bool(on) and (layer is None or layer in on)
            return ('unknown',) if possible else ('lit', False)
        return tuple(self._placed(c, a, b, layer) if isinstance(c, tuple) else c
                     for c in node)

    def _copper(self, nid, layer, kind='track'):
        """A net's copper a step may lay: a track on ``layer``, or a new via
        (of the type and span the step lays)."""
        key = ('item', nid, layer, kind)
        if key not in self._memo:
            if kind == 'via':
                facts = {'via_type': self.new_via_type}
                if self.new_via_type != 'through':
                    facts['other_layers'] = frozenset(self.new_via_layers)
                self._memo[key] = self.dr.item_for_net(nid, 'via', layer=layer, **facts)
            else:
                self._memo[key] = self.dr.item_for_net(nid, kind, layer=layer)
        return self._memo[key]

    def _blinded(self, item):
        return replace(item, **self._blind) if self._blind else item

    def value(self, hole_item, hole_net: int, layer: str,
              kind: str = 'track') -> Optional[float]:
        """The copper-to-hole value (mm) the rules give on ``layer`` between
        this hole and copper of another net a step may lay -- a track
        (``kind`` 'track') or a new via ('via') -- or None when no rule binds."""
        own = int(hole_net or 0)
        layer = self._layer_rep.get(layer, layer)
        hole_item = self._blinded(hole_item)
        key = (replace(hole_item, net_id=0, net_name='', xy=None),
               self._net_key.get(own), layer, kind)
        if key in self._memo:
            return self._memo[key]
        best = None
        for nid in self._others(own):       # KiCad exempts the hole's own net
            got = self._decide(self._copper(nid, layer, kind), hole_item, layer)
            if got is not None and (best is None or got > best):
                best = got
        self._memo[key] = best
        return best

    def new_via_value(self, kind: str = 'track') -> Optional[float]:
        """The largest value the rules give between the hole of a via a step
        may ADD and another net's ``kind`` of copper ('track' or 'via') the
        step lays, on any layer the new via is on."""
        mk = ('new', kind)
        if mk in self._memo:
            return self._memo[mk]
        best = None
        for g in self.groups.values():
            a = g[0]
            hole = self._copper(a, None, 'via')
            for layer in self.layers:
                if layer not in self._new_via_reps():
                    continue
                for b in self._others(a):
                    got = self._decide(self._copper(b, layer, kind), hole, layer)
                    if got is not None and (best is None or got > best):
                        best = got
        self._memo[mk] = best
        return best

    def _new_via_reps(self):
        got = self._memo.get('new_layers')
        if got is None:
            got = self._memo['new_layers'] = {self._layer_rep.get(l, l)
                                              for l in self.new_via_layers}
        return got

    def copper_value(self, item, net: int, layers) -> Optional[float]:
        """The largest value the rules give between the hole of a via a step
        may ADD (of a net other than ``net``) and this existing copper item
        (a pad or a via of the board) on ``layers``, the layers it has copper
        on."""
        own = int(net or 0)
        item = self._blinded(item)
        lays = sorted({self._layer_rep.get(l, l) for l in layers} & self._new_via_reps())
        key = ('copper', replace(item, net_id=0, net_name='', xy=None),
               self._net_key.get(own), tuple(lays))
        if key in self._memo:
            return self._memo[key]
        best = None
        for a in self._others(own):
            hole = self._copper(a, None, 'via')
            for layer in lays:
                got = self._decide(item, hole, layer)
                if got is not None and (best is None or got > best):
                    best = got
        self._memo[key] = best
        return best

    def _via_entry(self, via):
        hit = self._memo.get(('via', id(via)))
        if hit is None or hit[0] is not via:
            it = self.dr.via_item(via)
            vt = getattr(via, 'via_type', None)
            if vt:
                it = replace(it, via_type=vt)
            if self._index is not None:
                from shapely.geometry import Point
                it = replace(it, area_near=self._near(
                    Point(float(via.x), float(via.y)), float(getattr(via, 'size', 0.0) or 0.0)
                    / 2.0))
            # the via is kept with its entry, so its id cannot be reused
            hit = self._memo[('via', id(via))] = (via, it, {})
        return hit

    def via_value(self, via, layer: str, kind: str = 'track') -> Optional[float]:
        _via, it, vals = self._via_entry(via)
        if (layer, kind) not in vals:
            vals[(layer, kind)] = self.value(it, getattr(via, 'net_id', 0), layer, kind)
        return vals[(layer, kind)]

    def via_copper_value(self, via) -> Optional[float]:
        """`copper_value` for one via of the board."""
        from geometry_utils import via_copper_layers
        _via, it, vals = self._via_entry(via)
        if 'copper' not in vals:
            vals['copper'] = self.copper_value(
                it, getattr(via, 'net_id', 0), via_copper_layers(via, self.dr.copper_layers))
        return vals['copper']

    def _pad_entry(self, pad):
        """(pad, its RuleItem, its answers), built once per pad: the stamps
        ask about each pad on every layer, in every net's cache build."""
        hit = self._memo.get(('pad', id(pad)))
        if hit is None or hit[0] is not pad:
            it = self.dr.pad_item(pad)
            if self._index is not None:
                from design_rules import _pad_copper_geometry
                geom, r = _pad_copper_geometry(pad)
                if geom is None or getattr(pad, 'pad_type', None) == 'np_thru_hole':
                    # no copper: the hole itself
                    from shapely.geometry import LineString, Point
                    from kicad_parser import pad_drill_capsule
                    p1, p2, r = pad_drill_capsule(pad)
                    geom = Point(p1) if tuple(p1) == tuple(p2) else LineString([p1, p2])
                it = replace(it, area_near=self._near(geom, r))
            # the pad is kept with its entry, so its id cannot be reused
            hit = self._memo[('pad', id(pad))] = (pad, it, {})
        return hit

    def _near(self, geom, r: float):
        """The (area, layer) pairs within `_near_r` of copper ``geom`` grown
        by ``r``, on every copper layer the area is drawn on."""
        over, _inside = self._index._of(geom, r + self._near_r, self.dr.copper_layers)
        return over

    def pad_value(self, pad, layer: str, kind: str = 'track') -> Optional[float]:
        _pad, it, vals = self._pad_entry(pad)
        if (layer, kind) not in vals:
            vals[(layer, kind)] = self.value(it, getattr(pad, 'net_id', 0), layer, kind)
        return vals[(layer, kind)]

    def pad_copper_value(self, pad) -> Optional[float]:
        """`copper_value` for one pad of the board (None for a pad with no
        copper)."""
        if getattr(pad, 'pad_type', None) == 'np_thru_hole':
            return None
        _pad, it, vals = self._pad_entry(pad)
        if 'copper' not in vals:
            vals['copper'] = (self.copper_value(it, getattr(pad, 'net_id', 0), it.layers)
                              if it.layers else None)
        return vals['copper']


def _routed(config, pcb_data, rules=None):
    from via_span import via_kind
    span = getattr(config, 'via_span', None) if config is not None else None
    reach = (max(float(getattr(config, 'via_size', 0.0) or 0.0),
                 float(getattr(config, 'track_width', 0.0) or 0.0))
             if config is not None else 0.0)
    return RoutedHoleRules(rules if rules is not None else config.rules, pcb_data,
                           new_via_type=via_kind(config) if config is not None else 'through',
                           new_via_layers=list(span) if span else None,
                           copper_reach=reach)


def install(config, pcb_data) -> None:
    """Read the board's hole_clearance rules onto ``config.hole_rules`` (None
    without one). Never raises into an engine."""
    config.hole_rules = None
    dr = getattr(config, 'rules', None)
    if dr is None or pcb_data is None:
        return
    try:
        got = _routed(config, pcb_data)
    except Exception as e:                                      # noqa: BLE001
        print(f"  ERROR: the board's hole_clearance rules could not be set up "
              f"({type(e).__name__}: {e}); this run routes WITHOUT them")
        return
    if got:
        config.hole_rules = got


def mark_honoured(rules, pcb_data) -> None:
    """Name, for the rules report, the hole_clearance rules the route steps
    route to. A rule whose condition is not read stays NOT honoured: the
    walk passes over it (`RoutedHoleRules._decide`), so it binds nothing."""
    try:
        got = RoutedHoleRules(rules, pcb_data)
    except Exception as e:                                      # noqa: BLE001
        print(f"  ERROR: the board's hole_clearance rules could not be set up "
              f"({type(e).__name__}: {e}); this run routes WITHOUT them")
        return
    for r, ast, mn in got.walk:
        if (ast == ('unknown',) or r.severity in ('ignore', 'exclusion')
                or r.honoured or 'unknown' in str(ast)):
            continue
        r.honoured = (f"hole_clearance min {mn:g} mm: routed tracks and vias are kept "
                      f"that far from the holes of other nets' vias and pads")
        if id(r) in got.positional:
            r.honoured += (", where its rule area binds them: by where each hole "
                           "stands, and wherever the copper a step lays may be in "
                           "the area (tighten-safe)")
        r.router_unhonoured = None


def via_track_reach(config, via, layer: str, reserve_width: float,
                    extra: float = 0.0) -> Optional[float]:
    """The distance (mm) a track's centre must keep from this via's centre on
    ``layer`` under the board's hole_clearance rules, or None."""
    hr = getattr(config, 'hole_rules', None)
    drill = float(getattr(via, 'drill', 0.0) or 0.0)
    if hr is None or via is None or drill <= 0:
        return None
    v = hr.via_value(via, layer)
    if v is None:
        return None
    return drill / 2.0 + v + reserve_width / 2.0 + extra


def via_hole_extra(config, clearance: float, other_via=None, pad=None) -> float:
    """See `GridRouteConfig.via_hole_extra`."""
    hr = config.hole_rules
    out = 0.0
    # the new via's hole off this copper: a pad's, a via's, else a track's
    if pad is not None:
        v = hr.pad_copper_value(pad)
    elif other_via is not None:
        v = hr.via_copper_value(other_via)
    else:
        v = hr.new_via_value('track')
    if v is not None:
        out = max(out, config.via_drill / 2.0 + v - config.via_size / 2.0 - clearance)
    # ... and the new via's copper off another via's hole
    drill = float(getattr(other_via, 'drill', 0.0) or 0.0) if other_via is not None else 0.0
    if drill > 0:
        from geometry_utils import via_copper_layers
        size = float(getattr(other_via, 'size', 0.0) or 0.0) or config.via_size
        lays = set(via_copper_layers(other_via, hr.dr.copper_layers)) & set(hr.new_via_layers)
        vals = [hr.via_value(other_via, l, 'via') for l in hr.dr.copper_layers if l in lays]
        vals = [x for x in vals if x is not None]
        if vals:
            out = max(out, drill / 2.0 + max(vals) - size / 2.0 - clearance)
    return out
