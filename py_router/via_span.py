"""#1842: a route step whose vias span a contiguous range of copper layers.

The router lays one kind of via per step. Without a span that is a through
via, F.Cu to B.Cu, and every copper layer's obstacles block its site. With
`GridRouteConfig.via_span` -- a run of neighbouring copper layers, top to
bottom -- the step routes on those layers only and each via it lays joins
exactly that run: KiCad's micro via (two neighbouring layers, one of them an
outer layer), blind via (reaches one outer layer) or buried via (reaches
neither). On a 1+N+1 stackup each of those is the through via of a sub-stack,
so the search itself is unchanged: it is handed an obstacle map built from
`SpanView`, the board with the copper outside the span hidden.

What still blocks a via of the span: copper on the span's layers, and every
DRILLED pad (its barrel passes through the span, whatever layers its copper
is on). What does not: tracks, pours and undrilled pads on other layers, and
vias that have no copper on the span's layers.
"""
from dataclasses import replace
from typing import List, Optional, Sequence, Tuple

VIA_SPAN_TYPES = ('micro', 'blind', 'buried')


def resolve_via_span(copper_layers: Sequence[str], top: str, bottom: str,
                     via_type: Optional[str] = None) -> Tuple[List[str], Optional[str]]:
    """(the span's layers top to bottom, its via type) for the layer pair
    ``top`` / ``bottom`` of a board whose copper layers are ``copper_layers``
    in stack order. The whole stack is no span at all: ([...], None), a
    through via. ``via_type`` overrides the type read off the span ('micro'
    for two neighbouring layers one of which is an outer layer, 'blind' for a
    longer span that reaches one outer layer, 'buried' otherwise).

    Raises ValueError naming what is wrong: a layer the board does not have,
    one layer given twice, or a type the span cannot be."""
    copper = list(copper_layers)
    for name in (top, bottom):
        if name not in copper:
            raise ValueError(f"{name} is not a copper layer of this board "
                             f"({', '.join(copper)})")
    i, j = sorted((copper.index(top), copper.index(bottom)))
    if i == j:
        raise ValueError(f"a via span needs two different layers, got {top} twice")
    span = copper[i:j + 1]
    outer = (i == 0) + (j == len(copper) - 1)
    if outer == 2:
        if via_type not in (None, 'through'):
            raise ValueError(f"{span[0]}-{span[-1]} is the whole stack: its via is a "
                             f"through via, not {via_type}")
        return span, None
    natural = ('micro' if outer == 1 and len(span) == 2
               else 'blind' if outer == 1 else 'buried')
    if via_type in (None, natural):
        return span, natural
    if via_type not in VIA_SPAN_TYPES:
        raise ValueError(f"via type {via_type!r} is not one of {', '.join(VIA_SPAN_TYPES)}")
    if (via_type == 'buried') != (outer == 0):
        raise ValueError(f"a {via_type} via cannot span {span[0]}-{span[-1]}: a buried "
                         f"via reaches no outer layer, a blind or micro via reaches one")
    return span, via_type


class SpanView:
    """`base` (a PCBData) with the copper outside ``span`` hidden, for the
    obstacle builders of a via-span step: tracks and pours on other layers,
    vias with no copper on the span's layers, and undrilled pads with none.
    Drilled pads stay whole. Everything else is the base board's."""

    def __init__(self, base, span: Sequence[str]):
        from geometry_utils import via_copper_layers
        from check_drc import pad_copper_layers
        self._base = base
        keep = set(span)
        copper = list(base.board_info.copper_layers)
        self.segments = [s for s in base.segments if s.layer in keep]
        self.vias = [v for v in base.vias if keep & set(via_copper_layers(v, copper))]
        self.zones = [z for z in (getattr(base, 'zones', None) or []) if z.layer in keep]

        def shown(pad):
            return ((getattr(pad, 'drill', 0) or 0) > 0
                    or bool(keep & set(pad_copper_layers(pad, copper))))
        self.footprints = {ref: replace(fp, pads=[p for p in fp.pads if shown(p)])
                           for ref, fp in base.footprints.items()}
        self.pads_by_net = {nid: [p for p in pads if shown(p)]
                            for nid, pads in base.pads_by_net.items()}

    def __getattr__(self, name):
        return getattr(self._base, name)


def obstacle_board(pcb_data, config):
    """The board the obstacle builders of this step read: ``pcb_data``, or
    its `SpanView` when the step lays vias of a span."""
    span = getattr(config, 'via_span', None)
    return SpanView(pcb_data, span) if span else pcb_data


def via_layer_pair(config) -> List[str]:
    """The two end layers a via this step lays is written with."""
    span = getattr(config, 'via_span', None)
    return [span[0], span[-1]] if span else ['F.Cu', 'B.Cu']


def via_kind(config) -> str:
    """`Via.via_type` of a via this step lays."""
    return (getattr(config, 'via_span_type', None) or 'through'
            if getattr(config, 'via_span', None) else 'through')


def file_copper_layers(pcb_path: str) -> List[str]:
    """The board file's copper layers in stack order (F.Cu first, B.Cu
    last), read from its layer table without parsing the board. [] on any
    read error."""
    import re
    try:
        with open(pcb_path, encoding='utf-8') as fh:
            head = fh.read(200000)
    except (OSError, TypeError, ValueError):
        return []
    names = list(dict.fromkeys(re.findall(r'\(\d+\s+"([^"]*\.Cu)"', head)))
    inner = [n for n in names if n not in ('F.Cu', 'B.Cu')]
    return ([n for n in names if n == 'F.Cu'] + inner
            + [n for n in names if n == 'B.Cu'])


def default_micro_via(pcb_path: str) -> Tuple[Optional[float], Optional[float]]:
    """(diameter, drill) of the board's Default net-class micro via, from
    the sibling .kicad_pro; None for a value the project does not set."""
    import json
    import os
    try:
        with open(os.path.splitext(pcb_path)[0] + '.kicad_pro', encoding='utf-8') as fh:
            classes = (json.load(fh).get('net_settings') or {}).get('classes') or []
    except (OSError, TypeError, ValueError):
        return None, None
    for c in classes:
        if isinstance(c, dict) and c.get('name', 'Default') == 'Default':
            dia, drill = c.get('microvia_diameter'), c.get('microvia_drill')
            return (float(dia) if isinstance(dia, (int, float)) and dia > 0 else None,
                    float(drill) if isinstance(drill, (int, float)) and drill > 0 else None)
    return None, None


def apply_span(vias, config) -> int:
    """Give every via of ``vias`` (the vias a via-span step laid, objects or
    dicts) the step's layer pair and type. A site of the step that builds its
    via the through-via way is brought in line here: the obstacle map it was
    placed on was the span's. Returns how many were changed."""
    if not getattr(config, 'via_span', None):
        return 0
    pair, kind, changed = via_layer_pair(config), via_kind(config), 0
    for v in vias or ():
        if isinstance(v, dict):
            if list(v.get('layers') or []) != pair or v.get('via_type') != kind:
                v['layers'], v['via_type'] = list(pair), kind
                changed += 1
        elif list(v.layers) != pair or getattr(v, 'via_type', 'through') != kind:
            v.layers, v.via_type = list(pair), kind
            changed += 1
    return changed
