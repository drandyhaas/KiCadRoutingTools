"""#1842: a route step whose search chooses between several kinds of via.

`--via-span` gives a step ONE kind of via and the layers it joins. With
`GridRouteConfig.via_kinds` the step routes on every layer it was given and
its search may change layers through any of the declared kinds -- say a micro
via F.Cu-In1.Cu, a buried via In1.Cu-In6.Cu, a micro via In6.Cu-B.Cu and the
through via -- each legal where a via of THAT kind fits: what blocks it is the
copper on the layers it joins (and every drilled pad), at its own size. So
one net can go surface, micro via, inner layer, buried via, inner layer in
one search, which no single span can do.

How it is held together:

* a kind is `ViaKind`: its span, its type, its size, and the index of its
  own via map in the obstacle map (`map`; -1 for the through via, whose
  legality is the map's ordinary one);
* the obstacle builders stamp each kind's map from the board as that kind's
  span sees it (`via_span.SpanView`), by re-running the stamp with
  `kind_config` -- the base map, each net's cache entry and the raw adds;
* a stamp that knows nothing of kinds is repeated in every kind's map by the
  obstacle map itself (its kind mirror): what blocks a through via blocks a
  shorter via that is no larger, which is why a kind may not be larger than
  the step's own via;
* the router is armed with the kinds (`arm_router`) and every layer change
  it returns is looked up again with the search's own rule (`note_path`), so
  the via written is the kind the search laid.

Two consecutive vias of a path stand apart by the drill spacing
(`min_via_run`): the search reaches the second by one jog-then-via edge, so
a micro via and a buried via that share a layer are side by side. A
jog-only layer (`GridRouteConfig.jog_caps`) carries nothing but that jog, up
to a length -- the plane layer the two vias share.

Vias the step lays outside the search (pad taps, the plane finalize) stay
through vias, placed where a through via fits.
"""
from contextlib import contextmanager
from dataclasses import dataclass, replace
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class ViaKind:
    """One kind of via a step's search may lay."""
    span: Tuple[str, ...]     # the copper layers it joins, top to bottom
    via_type: str             # 'through', 'micro', 'blind' or 'buried'
    size: float
    drill: float
    map: int                  # its via map in the obstacle map; -1 = through

    @property
    def pair(self) -> List[str]:
        return [self.span[0], self.span[-1]]

    def label(self) -> str:
        return f"{self.via_type} {self.span[0]}-{self.span[-1]} {self.size:g}/{self.drill:g}"


def resolve_via_kinds(pcb_data, names: Optional[Sequence[str]], via_size: float,
                      via_drill: float, pcb_path: str = '') -> Optional[Tuple[ViaKind, ...]]:
    """The kinds ``names`` declares on this board: a flat list of layer
    names read as (top, bottom) pairs, in the order given. None for no list.

    A pair that is the whole stack is the through via, at the step's via
    size. A laser-drilled micro span (`via_span.laser_drilled`) draws the
    Default class's micro via when the project gives one; every other kind
    draws the step's via. `buried` is written `blind` in a file older than
    KiCad 10's format (`via_span.file_via_type`).

    Raises ValueError naming what is wrong: an odd number of names, a pair
    `via_span.resolve_via_span` refuses, one span given twice, or a kind
    larger than the step's via."""
    if not names:
        return None
    names = [str(n) for n in names]
    if len(names) % 2:
        raise ValueError(f"via kinds are (top, bottom) layer pairs; got {len(names)} "
                         f"layer name(s): {' '.join(names)}")
    from via_span import (resolve_via_span, laser_drilled, default_micro_via,
                          file_via_type)
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    version = getattr(pcb_data, 'kicad_version', 0) or 0
    path = pcb_path or getattr(pcb_data, 'source_path', '') or ''
    kinds, seen = [], set()
    for top, bottom in zip(names[0::2], names[1::2]):
        span, kind = resolve_via_span(copper, top, bottom)
        if tuple(span) in seen:
            raise ValueError(f"via kind {span[0]}-{span[-1]} is given twice")
        seen.add(tuple(span))
        if kind is None:
            kinds.append(ViaKind(tuple(span), 'through', via_size, via_drill, -1))
            continue
        size, drill = via_size, via_drill
        if laser_drilled(copper, (top, bottom)):
            dia, hole = default_micro_via(path)
            size, drill = dia or size, hole or drill
        if size > via_size + 1e-9 or drill > via_drill + 1e-9:
            raise ValueError(
                f"via kind {span[0]}-{span[-1]} ({size:g}/{drill:g} mm) is larger than "
                f"the step's via ({via_size:g}/{via_drill:g} mm): a kind may not be "
                f"larger than the via the step's other keep-outs are sized for")
        kinds.append(ViaKind(tuple(span), file_via_type(kind, version), size, drill,
                             sum(1 for k in kinds if k.map >= 0)))
    return tuple(kinds)


def resolve_jog_caps(pcb_data, layers: Optional[Sequence[str]], max_length: float,
                     kinds=None) -> Optional[Tuple[Tuple[str, float], ...]]:
    """((layer, cap in mm), ...) for a step's jog-only layers, None for none.
    Raises ValueError for a layer the board does not have, a length that is
    not positive, or jog-only layers on a step that declares no ``kinds``."""
    if not layers:
        return None
    if not kinds:
        raise ValueError("jog-only layers carry the run between two vias of different "
                         "kinds: they need via kinds (--via-kinds)")
    copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    for name in layers:
        if name not in copper:
            raise ValueError(f"{name} is not a copper layer of this board "
                             f"({', '.join(copper)})")
    if not max_length or max_length <= 0:
        raise ValueError(f"the jog length must be positive, got {max_length}")
    return tuple((str(name), float(max_length)) for name in dict.fromkeys(layers))


def check_cli(parser, args, pcb_path: str) -> None:
    """`--via-kinds` / `--jog-layers` checked against the board file's own
    stack before anything is routed: `parser.error` names what is wrong (the
    engine refuses the same things, for the callers that have no parser)."""
    names = getattr(args, 'via_kinds', None)
    jog = getattr(args, 'jog_layers', None)
    if not names and not jog:
        return
    from via_span import file_copper_layers, resolve_via_span
    copper = file_copper_layers(pcb_path)
    if names:
        if getattr(args, 'via_span', None):
            parser.error("--via-kinds: not with --via-span (a via span is a step of "
                         "one kind on its own layers)")
        if len(names) % 2:
            parser.error(f"--via-kinds: layer names come in TOP BOTTOM pairs, got "
                         f"{len(names)}: {' '.join(names)}")
        seen = set()
        for top, bottom in zip(names[0::2], names[1::2]):
            try:
                span, _kind = resolve_via_span(copper, top, bottom)
            except ValueError as e:
                parser.error(f"--via-kinds: {e}")
            if tuple(span) in seen:
                parser.error(f"--via-kinds: {span[0]}-{span[-1]} is given twice")
            seen.add(tuple(span))
    for name in jog or ():
        if name not in copper:
            parser.error(f"--jog-layers: {name} is not a copper layer of this board "
                         f"({', '.join(copper)})")
    if jog and not names:
        parser.error("--jog-layers: needs --via-kinds (a jog-only layer carries the run "
                     "between two vias of different kinds)")
    if jog and not getattr(args, 'jog_max_length', 0) > 0:
        parser.error(f"--jog-max-length: must be positive, got {args.jog_max_length}")


@contextmanager
def unmirrored(obstacles):
    """Run a via stamp on ``obstacles`` with its kind mirror off. For a LIFT:
    a remove of rows the base build stamped (and its later re-add). The base
    stamped each kind's map at that kind's own via, so the lifted rows are
    not the rows the kind maps hold, and a counted map stops at zero: taking
    them off there could unblock a cell another obstacle blocks. The kind
    maps keep such copper blocked instead."""
    set_mirror = getattr(obstacles, 'set_kind_mirror', None)
    was = set_mirror(False) if set_mirror is not None else False
    try:
        yield
    finally:
        if was:
            set_mirror(True)


def spanned(config) -> Tuple[ViaKind, ...]:
    """The step's kinds that have a via map of their own (all but the
    through via), () when the step declares none."""
    return tuple(k for k in (getattr(config, 'via_kinds', None) or ()) if k.map >= 0)


def kind_config(config, kind: ViaKind):
    """``config`` as a step that lays only ``kind``: its via size and its
    span, so the obstacle builders stamp what blocks a via of that kind. The
    clone declares no kinds of its own (a kind's stamp is one pass)."""
    memo = config.__dict__.setdefault('_kind_configs', {})
    key = (kind, config.via_size, config.via_drill)
    cfg = memo.get(key)
    if cfg is None:
        cfg = memo[key] = replace(config, via_size=kind.size, via_drill=kind.drill,
                                  via_span=kind.span, via_span_type=kind.via_type,
                                  via_kinds=None)
        cfg.__dict__['_kind_pass'] = True     # obstacle_map._off_kind_span
    return cfg


def arm_router(router, config) -> None:
    """Give ``router`` the step's via kinds and jog caps. Nothing is set for
    a step that declares neither, and the search is then unchanged."""
    kinds = getattr(config, 'via_kinds', None)
    caps = getattr(config, 'jog_caps', None)
    if not kinds and not caps:
        return
    index = {name: i for i, name in enumerate(config.layers)}
    if kinds:
        cost = config.via_cost_units()
        router.set_via_kinds([
            (sum(1 << index[l] for l in k.span if l in index and index[l] < 32),
             cost, k.map) for k in kinds], min_via_run(config))
    if caps:
        per_layer = [0] * len(config.layers)
        for name, mm in caps:
            if name in index:
                per_layer[index[name]] = max(min_via_run(config),
                                             int(mm / config.grid_step + 1e-9))
        router.set_layer_jog_caps(per_layer)


#: {(net id or None, gx, gy, lower layer index, upper layer index): ViaKind}
#: for the layer changes of the paths the armed searches returned
#: (`note_path`), read by the code that turns a path into vias (`kind_at`).
_CHOSEN = {}


def reset() -> None:
    """Forget every recorded choice (the start of a route step)."""
    _CHOSEN.clear()


def note_path(router, obstacles, config, path, net_id, via_rung: int = 0) -> None:
    """Record the kind of every layer change of ``path``, a path an armed
    search just returned on ``obstacles``. The lookup is the search's own
    (`GridRouter.via_kind_at`) on the map it searched, so it must run before
    anything else is stamped there. A layer change the search made through
    an existing hole of the net has no kind and is not recorded."""
    kinds = getattr(config, 'via_kinds', None)
    if not kinds or not path:
        return
    for (x1, y1, l1), (x2, y2, l2) in zip(path, path[1:]):
        if l1 == l2 or (x1, y1) != (x2, y2):
            continue
        key = (x1, y1, min(l1, l2), max(l1, l2))
        if obstacles.is_free_via(x1, y1):
            _CHOSEN.pop((net_id,) + key, None)
            _CHOSEN.pop((None,) + key, None)
            continue
        i = router.via_kind_at(obstacles, x1, y1, l1, l2, via_rung)
        if i >= 0:
            _CHOSEN[(net_id,) + key] = _CHOSEN[(None,) + key] = kinds[i]


def kind_at(config, net_id, gx: int, gy: int, layer1: int, layer2: int) -> Optional[ViaKind]:
    """The kind the search laid for net ``net_id``'s layer change at this
    cell, or None: the step declares no kinds, the kind is the through via,
    or the path came from a search that knows none (a through via then)."""
    if not getattr(config, 'via_kinds', None):
        return None
    key = (gx, gy, min(layer1, layer2), max(layer1, layer2))
    kind = _CHOSEN.get((net_id,) + key) or _CHOSEN.get((None,) + key)
    return kind if kind is not None and kind.map >= 0 else None


def _min_centres(config) -> float:
    """Grid cells two vias of one path must stand apart, centre to centre:
    the step's drill plus the hole-to-hole clearance (0 without one)."""
    h2h = float(getattr(config, 'hole_to_hole_clearance', 0.0) or 0.0)
    return (config.via_drill + h2h) / config.grid_step if h2h > 0 else 0.0


def min_via_run(config) -> int:
    """Grid cells the search keeps between two consecutive vias of a path
    when the step declares kinds: two kinds that share a layer are drilled
    side by side there, never one on the other."""
    import math
    return max(1, int(math.ceil(_min_centres(config) - 1e-9)))


def path_has_close_vias(path, config) -> bool:
    """Does ``path`` change layers twice on one cell, or on two cells nearer
    than the drills allow? The search keeps CONSECUTIVE vias apart; a path
    that doubles back can still bring an earlier one near a later one."""
    cells = [(a[0], a[1]) for a, b in zip(path or (), (path or ())[1:])
             if a[2] != b[2] and (a[0], a[1]) == (b[0], b[1])]
    if len(cells) != len(set(cells)):
        return True
    lim = _min_centres(config) ** 2
    cells = sorted(set(cells))
    return any((ax - bx) ** 2 + (ay - by) ** 2 < lim
               for i, (ax, ay) in enumerate(cells) for bx, by in cells[i + 1:])


def exclusion_radius(config) -> int:
    """Grid cells of the search's same-path via-spacing guard, for the
    re-route of a path `path_has_close_vias` flags."""
    import math
    return max(1, int(math.ceil(max(_min_centres(config), 1.0) / 2.0)))


def summary(config, vias) -> Optional[dict]:
    """JSON_SUMMARY['via_kinds'] of a step that declares kinds or jog-only
    layers, None otherwise: each kind and how many vias of its layer pair
    and type are on the board (``vias``), and the jog caps."""
    kinds = getattr(config, 'via_kinds', None)
    caps = getattr(config, 'jog_caps', None)
    if not kinds and not caps:
        return None
    on_board = {}
    for v in vias or ():
        key = (tuple(v.layers or ()), getattr(v, 'via_type', 'through') or 'through')
        on_board[key] = on_board.get(key, 0) + 1
    return {'kinds': [{'layers': k.pair, 'type': k.via_type, 'size': k.size,
                       'drill': k.drill,
                       'on_board': on_board.get((tuple(k.pair), k.via_type), 0)}
                      for k in kinds or ()],
            'jog_caps': {name: mm for name, mm in caps or ()}}
