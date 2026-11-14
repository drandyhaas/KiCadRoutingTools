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

    A micro via is KiCad's laser-drilled via: it joins an outer layer and the
    layer next to it, and nothing longer (`design_rules.laser_drilled_span`,
    which the fab floor and check_drc read too). A longer span reaching one
    outer layer is a blind via, mechanically drilled, so `micro` is refused
    there rather than drawn at the micro via's size.

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
    if via_type == 'micro' and len(span) != 2:
        raise ValueError(f"a micro via cannot span {span[0]}-{span[-1]} ({len(span)} "
                         f"layers): a micro via is laser drilled from an outer layer to "
                         f"the layer next to it; a longer span is a blind via")
    return span, via_type


def laser_drilled(copper_layers: Sequence[str], via_span: Optional[Sequence[str]],
                  via_type: Optional[str] = None) -> bool:
    """Is a via of the step's ``via_span`` (its two end layers, or None) a
    laser-drilled micro via on a board whose copper layers are
    ``copper_layers``? Then the fab floor's via sizes, which are a mechanical
    drill's, do not pin it (`design_rules.laser_drilled_span`). The ONE
    reading both fronts take of an asked span: route.py's main and the GUI's
    via size and drill controls. False for no span and for a span the board
    does not have (the engine refuses that by name)."""
    if not via_span or len(via_span) < 2:
        return False
    from design_rules import laser_drilled_span
    copper = list(copper_layers or ())
    try:
        span, kind = resolve_via_span(copper, via_span[0], via_span[-1], via_type)
    except ValueError:
        return False
    return laser_drilled_span(kind, span, copper)


def file_via_type(via_type: Optional[str], kicad_version: int) -> Optional[str]:
    """The type word a via of ``via_type`` is written with in a board file of
    format ``kicad_version`` (its `(version ...)`, 0 when unknown). KiCad 10
    split KiCad 9's one BLIND_BURIED type into `blind` and `buried`; a 9.x
    file names both `blind` (its layer pair says which), and KiCad 9's parser
    has no `buried` keyword. So `buried` becomes `blind` in a file older than
    KiCad 10's format. An unknown version (a live board, where the running
    pcbnew picks its own constant) is left as asked."""
    from kicad_parser import KICAD_10_MIN_VERSION
    if via_type == 'buried' and 0 < int(kicad_version or 0) < KICAD_10_MIN_VERSION:
        return 'blind'
    return via_type


#: per-build obstacle records `build_base_obstacle_map` binds on the board
#: it is handed, and `routing_context.prepare_obstacles_inplace` (and the
#: other lift sites) read off the REAL board. A view writes them through.
LIFT_RECORDS = ('_net_tie_lift', '_graphic_own_pad_lift', '_graphic_own_pad_via_lift')


class SpanView:
    """`base` (a PCBData) with the copper outside ``span`` hidden, for the
    obstacle builders of a via-span step: tracks and pours on other layers,
    vias with no copper on the span's layers, and undrilled pads with none.
    Drilled pads stay whole. Everything else is the base board's.

    Each filtered list is read off the base when first asked for, so a view
    built for one net's cache costs only what that build reads. A view is a
    snapshot: build a new one after the board changes.

    The per-build lift records (`LIFT_RECORDS`) are written THROUGH to the
    base: the map built from the view is the one the router lifts them off,
    and every lift site reads them from the run's own board (#977: the
    record a map is read with must be that map's own). A shallow COPY of a
    view (the pad-tap search builds its window map on one) keeps its own
    records, as a copy of the board does: its map is not the run's."""

    _FILTERED = ('segments', 'vias', 'zones', 'footprints', 'pads_by_net')

    def __init__(self, base, span: Sequence[str]):
        object.__setattr__(self, '_base', base)
        object.__setattr__(self, '_span', tuple(span))

    def __setattr__(self, name, value):
        if name in LIFT_RECORDS and not self.__dict__.get('_detached'):
            setattr(self._base, name, value)
        else:
            object.__setattr__(self, name, value)

    def __copy__(self):
        new = object.__new__(SpanView)
        new.__dict__.update(self.__dict__)
        new.__dict__['_detached'] = True
        return new

    def _filtered(self, name):
        from geometry_utils import via_copper_layers
        from check_drc import pad_copper_layers
        base, keep = self._base, set(self._span)
        copper = list(base.board_info.copper_layers)

        def shown(pad):
            return ((getattr(pad, 'drill', 0) or 0) > 0
                    or bool(keep & set(pad_copper_layers(pad, copper))))
        if name == 'segments':
            return [s for s in base.segments if s.layer in keep]
        if name == 'vias':
            return [v for v in base.vias if keep & set(via_copper_layers(v, copper))]
        if name == 'zones':
            return [z for z in (getattr(base, 'zones', None) or []) if z.layer in keep]
        if name == 'footprints':
            return {ref: replace(fp, pads=[p for p in fp.pads if shown(p)])
                    for ref, fp in base.footprints.items()}
        return {nid: [p for p in pads if shown(p)]
                for nid, pads in base.pads_by_net.items()}

    def __getattr__(self, name):
        # a copy of the view is built attribute by attribute, before `_base`
        # is there to ask
        if name in ('_base', '_span') or name.startswith('__'):
            raise AttributeError(name)
        if name in SpanView._FILTERED:
            value = self._filtered(name)
            object.__setattr__(self, name, value)
            return value
        return getattr(self._base, name)


def obstacle_board(pcb_data, config):
    """The board the obstacle builders of this step read: ``pcb_data``, or
    its `SpanView` when the step lays vias of a span. A view of the step's
    span is returned as it is, so builders that call each other apply it
    once."""
    span = getattr(config, 'via_span', None)
    if not span or (isinstance(pcb_data, SpanView)
                    and pcb_data._span == tuple(span)):
        return pcb_data
    return SpanView(pcb_data, span)


def via_span_summary(config, pcb_data=None) -> Optional[dict]:
    """JSON_SUMMARY['via_span'] of a via-span step, None without a span: its
    end layers, the via type it writes, and `plane_finalize` -- 'not run',
    always, since a plane tap is a through via (the plane step connects the
    plane nets) -- with the board's zone count, so a reader can tell a skip
    that left plane nets to connect from one on a board with none."""
    span = getattr(config, 'via_span', None)
    if not span:
        return None
    return {'layers': [span[0], span[-1]],
            'type': getattr(config, 'via_span_type', None),
            'plane_finalize': 'not run',
            'zones': len(getattr(pcb_data, 'zones', None) or ())}


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


#: how far from its pad an escape via may be placed, mm
ESCAPE_RADIUS_MM = 1.5
#: an escape via stands BESIDE its pad, at least this far from the pad's
#: copper, joined by a short track.
ESCAPE_PAD_GAP_MM = 0.05


def escape_pads(pcb_data, config, net_ids, all_swap_vias, all_swap_segments,
                verbose: bool = False) -> dict:
    """Lay an ESCAPE at each surface pad of ``net_ids`` (a list of (name,
    id)) for a via-span step: a via of the span beside the pad
    (`ESCAPE_PAD_GAP_MM`) and the short track that joins them, so the pad has
    a landing on the span's other layers. That landing is what a later step
    on the next sub-stack routes from.

    A pad is escaped when its copper is on one of the span's END layers and
    nowhere else in the span, it is not drilled, and its net has no copper on
    the span's other layers within reach of it already. The site is found by
    `plane_pad_tap.tap_pad_with_escalation` on the board read through
    `SpanView`, each escape seeing the ones laid before it.

    The copper is committed the way `global_plan.apply_plan_escape_fanout`
    commits its own: appended to ``pcb_data`` (every later map sees it) and
    to ``all_swap_vias`` / ``all_swap_segments`` (the writer reads those).

    Returns {'placed', 'declined': [(net, pad ref)], 'served', 'drilled'}."""
    span = list(getattr(config, 'via_span', None) or ())
    out = {'placed': 0, 'declined': [], 'served': 0, 'drilled': 0}
    if not span:
        return out
    from check_drc import pad_copper_layers
    from kicad_parser import Segment, Via
    from plane_pad_tap import tap_pad_with_escalation
    copper = list(pcb_data.board_info.copper_layers)
    ends, inner = {span[0], span[-1]}, set(span)
    pair, kind = via_layer_pair(config), via_kind(config)
    snpc = max(float(getattr(config, 'same_net_pad_clearance', -1.0) or 0.0),
               ESCAPE_PAD_GAP_MM)
    laid_vias, laid_segs = [], []

    def has_landing(nid, pad, pad_layer):
        other = inner - {pad_layer}
        for s in pcb_data.segments:
            if s.net_id == nid and s.layer in other and min(
                    abs(s.start_x - pad.global_x) + abs(s.start_y - pad.global_y),
                    abs(s.end_x - pad.global_x) + abs(s.end_y - pad.global_y)) < ESCAPE_RADIUS_MM:
                return True
        for v in pcb_data.vias:
            if (v.net_id == nid and abs(v.x - pad.global_x) < ESCAPE_RADIUS_MM
                    and abs(v.y - pad.global_y) < ESCAPE_RADIUS_MM):
                from geometry_utils import via_copper_layers
                if other & set(via_copper_layers(v, copper)):
                    return True
        return False

    for name, nid in net_ids:
        for pad in list((getattr(pcb_data, 'pads_by_net', None) or {}).get(nid) or ()):
            if getattr(pad, 'pad_type', '') == 'np_thru_hole':
                continue
            on = set(pad_copper_layers(pad, copper)) & inner
            if (getattr(pad, 'drill', 0) or 0) > 0:
                out['drilled'] += 1          # its barrel already joins the span
                continue
            if len(on) != 1 or not (on & ends):
                continue
            pad_layer = next(iter(on))
            if has_landing(nid, pad, pad_layer):
                out['served'] += 1
                continue
            res = tap_pad_with_escalation(
                pad, pad_layer, nid, SpanView(pcb_data, span), config,
                max_search_radius=ESCAPE_RADIUS_MM,
                via_size=config.via_size, via_drill=config.via_drill,
                same_net_pad_clearance=snpc,
                extra_vias=laid_vias, extra_segments=laid_segs,
                verbose=verbose, fine_for_all=True, disable_reuse=True)
            ref = f"{pad.component_ref}.{pad.pad_number}"
            if not res.success or res.via is None:
                out['declined'].append((name, ref))
                continue
            v = res.via
            via_obj = Via(x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                          layers=list(pair), net_id=nid, via_type=kind)
            pcb_data.vias.append(via_obj)
            all_swap_vias.append(via_obj)
            laid_vias.append(dict(v, net_id=nid))
            for s in res.segments:
                seg_obj = Segment(start_x=s['start'][0], start_y=s['start'][1],
                                  end_x=s['end'][0], end_y=s['end'][1],
                                  width=s['width'], layer=s['layer'], net_id=nid)
                pcb_data.segments.append(seg_obj)
                all_swap_segments.append(seg_obj)
                laid_segs.append(dict(s, net_id=nid))
            out['placed'] += 1
    print(f"Via span escapes (#1842): {out['placed']} {kind} via(s) {pair[0]}-{pair[1]} "
          f"placed at surface pads, {len(out['declined'])} declined, {out['served']} "
          f"pad(s) already had a landing, {out['drilled']} drilled pad(s)")
    for name, ref in out['declined'][:12]:
        print(f"  no escape found for {ref} ({name})")
    if len(out['declined']) > 12:
        print(f"  ... {len(out['declined']) - 12} more")
    return out


#: `held_by` of copper a via-span step holds in place (`hold_outside_copper`)
HELD_OUTSIDE_SPAN = 'via-span: copper outside the span'


def hold_outside_copper(pcb_data, span) -> int:
    """Mark the board's copper that lies wholly or partly OUTSIDE ``span`` as
    held for this step (`held_by`, so `kicad_parser.is_immutable_copper` is
    true of it): tracks on other layers, and vias with copper on a layer the
    span does not have. A via-span step lays copper on the span only, so it
    could not lay such copper again -- an escape's stub and micro via, ripped
    to let another net by, left its own net with no way back to its pad. The
    hold is in memory, for the step; nothing is written to the board.
    Copper already locked or held is left as it is. Returns how many items
    were marked."""
    from geometry_utils import via_copper_layers
    keep = set(span)
    copper = list(pcb_data.board_info.copper_layers)
    n = 0
    for s in pcb_data.segments:
        if (s.layer in copper and s.layer not in keep
                and not getattr(s, 'graphic', False)
                and not getattr(s, 'locked', False) and not getattr(s, 'held_by', '')):
            s.held_by = HELD_OUTSIDE_SPAN
            n += 1
    for v in pcb_data.vias:
        if (set(via_copper_layers(v, copper)) - keep
                and not getattr(v, 'locked', False) and not getattr(v, 'held_by', '')):
            v.held_by = HELD_OUTSIDE_SPAN
            n += 1
    return n
