"""The clearance a rule area sets over the ball field (#1843 for the fanout).

KiCad's last matching rule wins. A `.kicad_dru` clearance rule conditioned on
a rule area (`A.insideArea('ROOM')`, `intersectsArea`) binds every pair of
items where either one's copper meets the area on the pair's layer, and its
value REPLACES the net classes and every rule BEFORE it in the file -- a
class- or type-scoped rule (#1491), a layer rule (#498). A rule after it
still replaces it for what that rule binds.

The escape engines price a pair by its nets alone, so a wide class-scoped
rule earlier in the file held the escape at its value inside a room that
relaxes it, and balls KiCad grades clean were dropped. This module reads the
room the way the route steps do (`area_clearance`, `design_rules.AreaIndex`):

  * `part_view` finds the area rule that holds the whole ball field: a region
    `area_clearance.derive` builds that encloses every ball's copper, drawn on
    every copper layer of the board, with no later area rule reaching the
    field. `None` when there is none; when an area rule touches the field but
    cannot be read that way, it says why once, and the escape keeps the
    values it had (the wider side).
  * `PartAreaView.apply` turns a fanout config into the room's view: the net
    classes cleared, the scoped channel and the layer map cut to the rules
    after the room's (`AreaClearance.region_config` reads them the same way),
    the run clearance raised to the room's value where the room tightens it.
  * `PartAreaView.holds` says whether an item's copper meets the area, as
    KiCad reads it. The class check grades a pair inside the area at the
    room's view and a pair wholly outside it at the full rules, so copper an
    escape lays past the area is still held to the wide rule.
"""
from __future__ import annotations

import contextlib
import io
from typing import Dict, Optional

_ANNOUNCED = set()
_MEMO: Dict[tuple, tuple] = {}


def _say(key, line):
    if key not in _ANNOUNCED:
        _ANNOUNCED.add(key)
        print(f"  {line}")


class PartAreaView:
    """The room that holds a part's ball field: ``region`` (an
    `area_clearance.Region`), ``area`` (its `AreaClearance`), ``rules`` (the
    board's `DesignRules`, whose `AreaIndex` measures items)."""

    def __init__(self, area, region, rules, index):
        self.area, self.region, self.rules, self.index = area, region, rules, index

    @property
    def value(self) -> float:
        return float(self.region.value)

    @property
    def order(self) -> int:
        return int(self.region.order)

    def layer_map(self, lmap) -> Dict[str, float]:
        """The layer rules of ``lmap`` that come after the room's rule."""
        lo = self.area.layer_orders
        return {l: v for l, v in (lmap or {}).items() if lo.get(l, -1) > self.order}

    def clearance(self, run_clearance: float) -> float:
        """The run clearance in the room: the room's value where it is above
        the run's, else the run's (a relaxing room never lowers what the
        run was asked for)."""
        return max(float(run_clearance), self.value)

    def apply(self, cfg, run_clearance: Optional[float] = None):
        """``cfg`` (after its .kicad_dru channels are installed) read inside
        the room, in place: no class map, the layer rules and the
        class/type-scoped rules after the room's rule only. Returns ``cfg``."""
        base = cfg.clearance if run_clearance is None else run_clearance
        cfg.clearance = self.clearance(base)
        cfg.layer_clearances = self.layer_map(cfg.layer_clearances)
        cfg.set_net_clearances({}, ())
        cfg.scoped_clearance = self.area._scoped_after(cfg, self.order)
        return cfg

    def holds(self, kind: str, obj, layer: Optional[str] = None) -> bool:
        """Does the item's copper meet the area (KiCad's `intersectsArea`)?
        ``kind`` 'seg', 'via' or 'pad'. The room is drawn on every copper
        layer, so any layer of the item's will do."""
        idx = self.index
        if kind == 'seg':
            over, _in = idx.of_track(obj)
        elif kind == 'via':
            over, _in = idx.of_via(obj, self.rules.copper_layers)
        else:
            over, _in = idx.of_pad(obj, self.rules.copper_layers)
        name = self.region.area
        return any(a == name for a, _l in over)

    def describe(self) -> str:
        return (f"rule '{self.region.rule}' in rule area '{self.region.area}' "
                f"({self.value:g}mm)")


def _derive(pcb_data, footprint):
    from routing_config import GridRouteConfig
    from kicad_dru import install_layer_clearances
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    if not copper:
        return None
    ref = getattr(footprint, 'reference', '?')
    src = getattr(pcb_data, 'source_path', '') or ''
    cfg = GridRouteConfig(layers=copper, clearance=0.0)
    with contextlib.redirect_stdout(io.StringIO()):
        install_layer_clearances(cfg, None, None, pcb_data)
    rules = getattr(cfg, 'rules', None)
    if rules is None:
        return None
    from area_clearance import derive, eligible
    index, picked = eligible(rules, pcb_data)
    if index is None or not picked:
        return None
    area = derive(rules, pcb_data, cfg)
    if area is None:
        return None
    pads = [p for p in footprint.pads if getattr(p, 'pad_type', '') != 'np_thru_hole']
    if not pads:
        return None
    from shapely.geometry import box
    x0 = min(p.global_x - p.size_x / 2.0 for p in pads)
    y0 = min(p.global_y - p.size_y / 2.0 for p in pads)
    x1 = max(p.global_x + p.size_x / 2.0 for p in pads)
    y1 = max(p.global_y + p.size_y / 2.0 for p in pads)
    field = box(x0, y0, x1, y1)
    # an area clearance rule the area reader does not read (its condition is
    # more than rule-area predicates): KiCad's order over the field cannot be
    # told, so the escape keeps the values it had
    from design_rules import _ast_area_names
    shapes = {}
    for entry in index.areas:
        shapes.setdefault(entry[0], []).append(entry[2])
    for r, kind, names, why in picked:
        if names is not None or kind != 'clearance':
            continue
        hit = sorted(n for n in _ast_area_names(r.area_scoped_condition, set())
                     if any(g.intersects(field) for g in shapes.get(n, ())))
        if hit:
            _say((src, ref, r.name, 'unread'),
                 f"Rule area: rule '{r.name}' on area {', '.join(map(repr, hit))} "
                 f"is not read for {ref}'s ball field ({why}); the escape keeps "
                 f"the class and rule values there (the wider side)")
            return None
    touching = [(i, r) for i, r in enumerate(area.regions)
                if r.value is not None and r.polygon.intersects(field)]
    if not touching:
        return None
    # the last area rule in the file reaching the field decides it
    i, reg = max(touching, key=lambda t: t[1].order)
    why = None
    if not set(copper) <= set(reg.layers):
        why = (f"its area is not drawn on every copper layer "
               f"({', '.join(sorted(set(copper) - set(reg.layers)))} missing)")
    else:
        for p in pads:
            _over, inside = index.of_pad(p, copper)
            if not any(a == reg.area for a, _l in inside):
                why = f"its area does not enclose ball {p.pad_number}"
                break
    if why is not None:
        _say((src, ref, reg.rule, 'not'),
             f"Rule area: rule '{reg.rule}' ({reg.value:g}mm in '{reg.area}') "
             f"is not applied to {ref}'s ball field: {why}; the escape keeps "
             f"the class and rule values outside the area (the wider side)")
        return None
    view = PartAreaView(area, reg, rules, index)
    _say((src, ref, reg.rule, 'yes'),
         f"Rule area: {ref}'s ball field lies in {view.describe()}: the escape "
         f"is spaced at it in place of the net classes and the rules before it "
         f"in the file (a rule after it still binds what it binds), and its "
         f"copper past the area is graded at the class and rule values there")
    return view


def part_view(pcb_data, footprint) -> Optional[PartAreaView]:
    """The room holding ``footprint``'s ball field on ``pcb_data``, or None
    (no area rule reaches it, or one does that cannot be read for the whole
    field -- said once). Memoized per board object and part."""
    if pcb_data is None or footprint is None:
        return None
    board = getattr(pcb_data, 'board_info', None)
    if not (getattr(board, 'keepouts', None)):
        return None
    key = (id(pcb_data), getattr(footprint, 'reference', None),
           id(board.keepouts), len(board.keepouts))
    got = _MEMO.get(key)
    if got is not None and got[0] is pcb_data:
        return got[1]
    try:
        view = _derive(pcb_data, footprint)
    except Exception as e:                                      # noqa: BLE001
        _say((getattr(pcb_data, 'source_path', ''), 'error', str(e)),
             f"Rule area: the area rules over the ball field could not be read "
             f"({type(e).__name__}: {e}); the escape keeps the class and rule "
             f"values (the wider side)")
        view = None
    if len(_MEMO) > 4:
        _MEMO.clear()
    _MEMO[key] = (pcb_data, view)
    return view
