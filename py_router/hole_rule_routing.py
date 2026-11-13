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
"""

import fnmatch
import re
from dataclasses import replace
from typing import Dict, Optional

_QUOTED = re.compile(r"'([^']*)'|\"([^\"]*)\"")


class RoutedHoleRules:
    """The walk, and the nets it is asked for.

    Two nets read alike to these rules when they have the same classes and
    match the same quoted names in the rules' conditions, so each such group
    stands in for all of its nets: two of them, since a hole is never held
    off its own net and a condition may compare the two items' net names."""

    def __init__(self, dr, pcb_data):
        from design_rules import parse_condition_maybe
        self.dr = dr
        self.walk = []
        for r in dr.rules:
            spec = r.constraints.get('hole_clearance')
            if spec is None:
                continue
            ast = (('unknown',) if str(r.unsupported or '').startswith('constraint')
                   else parse_condition_maybe(r.condition_src))
            self.walk.append((r, ast, float(spec.get('min') or 0.0)))
        self.ctx = ('hole_clearance', dr._net_name_set())
        self.unread: Dict[str, str] = {}
        self._memo: Dict[tuple, Optional[float]] = {}
        self._decided: Dict[tuple, Optional[float]] = {}
        quoted = sorted({(q1 or q2).lower() for r, _a, _m in self.walk
                         for q1, q2 in _QUOTED.findall(r.condition_src or '')})
        self._net_key: Dict[int, tuple] = {}
        groups: Dict[tuple, list] = {}
        for nid, net in sorted((getattr(pcb_data, 'nets', None) or {}).items()):
            name = getattr(net, 'name', None)
            if not nid or not name:
                continue
            low = name.lower()
            key = (dr.memberships.get(nid, frozenset()), dr.effective_class(nid),
                   tuple(i for i, q in enumerate(quoted)
                         if q == low or fnmatch.fnmatchcase(low, q)))
            self._net_key[int(nid)] = key
            g = groups.setdefault(key, [])
            if len(g) < 2:
                g.append(int(nid))
        self.groups = groups
        self.reps = sorted(n for g in groups.values() for n in g)
        # the layers a walk can tell apart: an outer one of each side, an
        # inner one, and every layer a rule names
        cu = list(dr.copper_layers)
        text = ' '.join(f"{r.layer_clause or ''} {r.condition_src or ''}"
                        for r, _a, _m in self.walk)
        pick = cu[:2] + cu[-1:] + [l for l in cu if l in text]
        self.layers = [l for l in cu if l in pick]

    def __bool__(self):
        return bool(self.walk)

    def _decide(self, a, b, layer):
        from design_rules import rule_may_match
        key = (a, b, layer)
        if key in self._decided:
            return self._decided[key]
        out = None
        for r, ast, mn in reversed(self.walk):
            m = rule_may_match(r, ast, a, b, layer, self.dr.copper_layers, self.ctx)
            if m is None:
                self.unread.setdefault(r.name, r.unsupported
                                       or 'its condition is not read here')
                break
            if m:
                out = None if r.severity in ('ignore', 'exclusion') else mn
                break
        self._decided[key] = out
        return out

    def _track(self, nid, layer, kind='track'):
        key = ('item', nid, layer, kind)
        if key not in self._memo:
            self._memo[key] = self.dr.item_for_net(nid, kind, layer=layer)
        return self._memo[key]

    def value(self, hole_item, hole_net: int, layer: str) -> Optional[float]:
        """The copper-to-hole value (mm) the rules give on ``layer`` between
        this hole and a track of another net, or None when no rule binds."""
        own = int(hole_net or 0)
        if layer not in self.layers and len(self.layers) > 2:
            layer = self.layers[1]      # an inner layer no rule names reads as any other
        key = (replace(hole_item, net_id=0, net_name='', xy=None),
               self._net_key.get(own), layer)
        if key in self._memo:
            return self._memo[key]
        best = None
        for nid in self.reps:
            if nid == own:
                continue        # KiCad exempts the hole's own net
            got = self._decide(self._track(nid, layer), hole_item, layer)
            if got is not None and (best is None or got > best):
                best = got
        self._memo[key] = best
        return best

    def new_via_value(self) -> Optional[float]:
        """The largest value the rules give between the hole of a via a step
        may ADD and another net's track or via, on any copper layer."""
        if 'new' in self._memo:
            return self._memo['new']
        best = None
        for g in self.groups.values():
            a = g[0]
            hole = self.dr.item_for_net(a, 'via')
            for layer in self.layers:
                for b in self.reps:
                    if b == a:
                        continue
                    for kind in ('track', 'via'):
                        got = self._decide(self._track(b, layer, kind), hole, layer)
                        if got is not None and (best is None or got > best):
                            best = got
        self._memo['new'] = best
        return best

    def via_value(self, via, layer: str) -> Optional[float]:
        hit = self._memo.get(('via', id(via)))
        if hit is None or hit[0] is not via:
            it = self.dr.via_item(via)
            vt = getattr(via, 'via_type', None)
            if vt:
                it = replace(it, via_type=vt)
            # the via is kept with its entry, so its id cannot be reused
            hit = self._memo[('via', id(via))] = (via, it, {})
        vals = hit[2]
        if layer not in vals:
            vals[layer] = self.value(hit[1], getattr(via, 'net_id', 0), layer)
        return vals[layer]

    def pad_value(self, pad, layer: str) -> Optional[float]:
        return self.value(self.dr.pad_item(pad), getattr(pad, 'net_id', 0), layer)


def install(config, pcb_data) -> None:
    """Read the board's hole_clearance rules onto ``config.hole_rules`` (None
    without one). Never raises into an engine."""
    config.hole_rules = None
    dr = getattr(config, 'rules', None)
    if dr is None or pcb_data is None:
        return
    try:
        got = RoutedHoleRules(dr, pcb_data)
    except Exception as e:                                      # noqa: BLE001
        print(f"  ERROR: the board's hole_clearance rules could not be set up "
              f"({type(e).__name__}: {e}); this run routes WITHOUT them")
        return
    if got:
        config.hole_rules = got


def mark_honoured(rules, pcb_data) -> None:
    """Name, for the rules report, the hole_clearance rules the route steps
    read. A rule whose condition is not read stays NOT honoured."""
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


def via_hole_extra(config, clearance: float, other_via=None) -> float:
    """See `GridRouteConfig.via_hole_extra`."""
    hr = config.hole_rules
    out = 0.0
    v = hr.new_via_value()
    if v is not None:
        out = max(out, config.via_drill / 2.0 + v - config.via_size / 2.0 - clearance)
    drill = float(getattr(other_via, 'drill', 0.0) or 0.0) if other_via is not None else 0.0
    if drill > 0:
        size = float(getattr(other_via, 'size', 0.0) or 0.0) or config.via_size
        vals = [hr.via_value(other_via, l) for l in hr.dr.copper_layers]
        vals = [x for x in vals if x is not None]
        if vals:
            out = max(out, drill / 2.0 + max(vals) - size / 2.0 - clearance)
    return out
