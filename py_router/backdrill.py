"""#1925 piece 3: route to a back-drill's spacing, and define back-drills
when asked.

Two things, and only the second is an option:

1. A back-drill a VIA of the board declares (`Via.backdrills`) is an
   obstacle. On each layer it removes, the hole is the back-drill's diameter
   and has no copper, so other nets' copper is held off its wall like an
   unplated hole's: `max(the pair's clearance, the copper-to-hole floor)`,
   the floor being the one the NPTH keep-out uses. A new via is held off the
   wall the same way, and its drill off it at the hole-to-hole minimum. A
   board with no back-drilled via builds the same maps as before: every hook
   here returns its "nothing" value for a via without one. A drilled PAD's
   back-drill (`Pad.backdrills`) is read and kept, and is not an obstacle
   yet.

2. `--backdrill-nets` (a `BackdrillSpec` on `config.backdrill`) asks the
   route step to DEFINE back-drills on the vias it adds for those nets:

   - such a net is searched at a via-legality rung of its own
     (`legality_via`): a via site is legal there only where the back-drill's
     hole would clear other nets' copper on EVERY layer. The drilled span is
     known only once the search has picked its layers, and a rung holds one
     answer per site, so the test cannot be narrower in Python. A net that
     does not route at that rung -- or, for a net of three or more pads, whose
     tap edges do not all route at it -- is routed as it would be with no
     option (completion first), and its vias are drilled only where they turn
     out legal;
   - when a net is routed each new via is given the back-drill its stubs
     want (`plan_spans`), so the nets routed after it keep off the wall on
     the layers it removes;
   - at the end of the step every via the step added for those nets is
     judged again on the board that ships (`finalize`): the span is read
     from the copper the net really joins the via on, and the drill is kept
     only if its wall clears every other net's copper on the layers it
     removes. What was not drilled, and why, is in the step's summary.

KiCad 10.0.8 itself grades a back-drilled via, on each layer the drill
removes, as a disc of the back-drill's diameter at the pair's copper
clearance (measured with kicad-cli; `check_drc` does the same). It does not
hold a track to its hole clearance from that wall; the route steps do,
because the wall is a hole.
"""
import fnmatch
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

import routing_defaults as defaults

_FAR = 10 ** 6


def layer_rank(name: str) -> int:
    """Stack position of a copper layer from its KiCad name alone (F.Cu,
    In1.Cu ... InN.Cu, B.Cu), so a span reads the same with or without the
    board at hand."""
    if name == 'F.Cu':
        return 0
    if name == 'B.Cu':
        return _FAR
    if name.startswith('In') and name.endswith('.Cu'):
        try:
            return int(name[2:-3])
        except ValueError:
            return -1
    return -1


def _spans(item):
    """[(size, low rank, high rank)] of an item's back-drills."""
    out = []
    for bd in (getattr(item, 'backdrills', None) or {}).values():
        size, start, end = bd
        a, b = layer_rank(start), layer_rank(end)
        if size and size > 0 and a >= 0 and b >= 0:
            out.append((float(size), min(a, b), max(a, b)))
    return out


def size_on_layer(item, layer: str) -> float:
    """Diameter of the back-drill that removes `layer` from this via (the
    largest, if two do), or 0.0."""
    if not getattr(item, 'backdrills', None):
        return 0.0
    r = layer_rank(layer)
    return max([s for s, a, b in _spans(item) if a <= r <= b] or [0.0])


def hole_diameter(item) -> float:
    """The widest hole this via is drilled with: its own drill, or a
    back-drill when one is wider. For the hole-to-hole keep-outs, which hold
    on every layer."""
    drill = float(getattr(item, 'drill', 0.0) or 0.0)
    if not getattr(item, 'backdrills', None):
        return drill
    return max([drill] + [s for s, _a, _b in _spans(item)])


def wall_floor(config) -> float:
    """The copper-to-hole floor a back-drill's wall is held at: the board's
    own declared floor when `install` read one, never under the fab's
    NPTH-to-track value."""
    got = getattr(config, 'backdrill_wall_floor', None) or 0.0
    return max(defaults.NPTH_TO_TRACK_CLEARANCE, float(got))


def wall_clearance(config, clearance: float) -> float:
    """Copper of another net to a back-drill's wall, where the pair's copper
    clearance is `clearance`."""
    return max(float(clearance), wall_floor(config))


def track_reach(config, via, layer: str, clearance: float, reserve_width: float,
                extra: float = 0.0) -> Optional[float]:
    """The distance (mm) a track's centre must keep from this via's centre on
    `layer` because a back-drill removes the layer there, or None."""
    size = size_on_layer(via, layer)
    if size <= 0:
        return None
    return size / 2.0 + wall_clearance(config, clearance) + reserve_width / 2.0 + extra


def via_extra(config, clearance: float, other_via) -> float:
    """The mm a new via's keep-out from `other_via` grows by because
    `other_via` is back-drilled: the new via's copper is held off the wall,
    on top of the ring-to-ring distance priced at `clearance`. A new via that
    is itself to be back-drilled is asked for at its legality geometry
    (`legality_via`), whose size already carries its own wall; its real
    copper is what must clear the other wall."""
    if not getattr(other_via, 'backdrills', None):
        return 0.0
    wall = max([s for s, _a, _b in _spans(other_via)] or [0.0])
    if wall <= 0:
        return 0.0
    size = float(getattr(other_via, 'size', 0.0) or 0.0) or config.via_size
    new_size = config.via_size
    spec = getattr(config, 'backdrill', None)
    if spec is not None:
        real = spec.real_of.get((round(config.via_size, 4), round(config.via_drill, 4)))
        if real is not None:
            new_size = real[0]
    need = wall / 2.0 + wall_clearance(config, clearance) + new_size / 2.0
    have = size / 2.0 + config.via_size / 2.0 + clearance
    return max(0.0, need - have)


@dataclass
class BackdrillSpec:
    """What `--backdrill-nets` asked for. The three numbers are the
    fabricator's: none has a default.

    net_ids   the nets whose new vias are back-drilled
    oversize  mm the back-drill's diameter exceeds the via's drill by
    margin    mm of dielectric the drill must stop short of the last layer the
              net uses: a stub layer nearer than that is left
    min_stub  mm; a stub shorter than this is not drilled
    """
    net_ids: frozenset
    oversize: float
    margin: float
    min_stub: float
    #: {legality (diameter, drill): the real (diameter, drill)} of the rungs
    #: this spec adds, filled by `prepare`
    real_of: Dict[Tuple[float, float], Tuple[float, float]] = field(default_factory=dict)
    #: False on the copy a net falls back to when it did not route at its rung
    search: bool = True

    def lenient(self) -> 'BackdrillSpec':
        from dataclasses import replace
        return replace(self, search=False)


def resolve_nets(pcb_data, patterns: Iterable[str]) -> List[int]:
    """Net ids whose name matches any of `patterns` and none of those that
    start with `!` (fnmatch, case kept), as `--nets` reads its own."""
    pats = [p for p in (patterns or []) if p and not p.startswith('!')]
    nots = [p[1:] for p in (patterns or []) if p and p.startswith('!') and p[1:]]
    out = []
    for nid, net in sorted((getattr(pcb_data, 'nets', None) or {}).items()):
        name = getattr(net, 'name', '') or ''
        if (nid and name and any(fnmatch.fnmatchcase(name, p) for p in pats)
                and not any(fnmatch.fnmatchcase(name, p) for p in nots)):
            out.append(int(nid))
    return out


def make_spec(pcb_data, patterns, oversize, margin, min_stub) -> Optional[BackdrillSpec]:
    """The spec for a step, or None when no net is asked for. Raises
    ValueError, naming the missing number, when nets are asked for without
    all three of the fabricator's numbers."""
    pats = [p for p in (patterns or []) if p]
    if not [p for p in pats if not p.startswith('!')]:
        return None
    missing = [flag for flag, v in (('--backdrill-oversize', oversize),
                                    ('--backdrill-margin', margin),
                                    ('--backdrill-min-stub', min_stub)) if v is None]
    if missing:
        raise ValueError("--backdrill-nets needs " + ", ".join(missing)
                         + ": they are the fabricator's numbers and have no default")
    if float(oversize) <= 0:
        raise ValueError("--backdrill-oversize must be above 0")
    if float(margin) < 0 or float(min_stub) < 0:
        raise ValueError("--backdrill-margin and --backdrill-min-stub cannot be negative")
    return BackdrillSpec(frozenset(resolve_nets(pcb_data, pats)), float(oversize),
                         float(margin), float(min_stub))


def _min_clearance(config) -> float:
    vals = [config.clearance] + [float(v) for v in
                                 (getattr(config, 'layer_clearances', None) or {}).values()]
    return min(vals)


def legality_via(config, size: float, drill: float) -> Tuple[float, float]:
    """The (diameter, drill) a via of (`size`, `drill`) that is to be
    back-drilled is tested for legality at: its hole is the back-drill's on
    whatever layers the drill removes, so on every layer its copper keep-out
    must reach the wall's, and its drill is the back-drill's."""
    spec = config.backdrill
    wall = drill + spec.oversize
    # the wall's keep-out is never narrower than the wall: a clearance above
    # the copper-to-hole floor adds nothing to subtract
    reach = wall + 2.0 * max(0.0, wall_floor(config) - _min_clearance(config))
    return (round(max(size, reach), 4), round(wall, 4))


def prepare(config) -> None:
    """Fill `config.backdrill.real_of`: one legality rung per distinct via
    geometry the asked nets are drawn at. Called once, on the run's config,
    after the wall floor is installed."""
    spec = getattr(config, 'backdrill', None)
    if spec is None:
        return
    spec.real_of.clear()
    reals = sorted({tuple(round(x, 4) for x in config.net_via(n)) for n in spec.net_ids}
                   | {(round(config.via_size, 4), round(config.via_drill, 4))},
                   key=lambda p: (-p[0], -p[1]))
    for real in reals:
        spec.real_of[legality_via(config, real[0], real[1])] = real


def rung_pairs(config) -> List[Tuple[float, float]]:
    """The legality geometries `obstacle_cache.via_rungs` appends for the
    asked nets, in a fixed order."""
    spec = getattr(config, 'backdrill', None)
    if spec is None or not spec.net_ids:
        return []
    return sorted(spec.real_of, key=lambda p: (-p[0], -p[1]))


def rung_pair_for_net(config, net_id: int) -> Optional[Tuple[float, float]]:
    """The legality geometry of `net_id`'s rung, or None when the net is not
    asked for."""
    spec = getattr(config, 'backdrill', None)
    if spec is None or net_id not in spec.net_ids:
        return None
    run = getattr(config, 'run_via', None) or (config.via_size, config.via_drill)
    sizes = getattr(config, 'net_via_sizes', None) or {}
    real = sizes.get(net_id) or run
    real = (round(float(real[0]), 4), round(float(real[1]), 4))
    for pair, r in spec.real_of.items():
        if r == real:
            return pair
    return None


def searches_at_rung(config, net_id: int) -> bool:
    spec = getattr(config, 'backdrill', None)
    return bool(spec is not None and spec.search and net_id in spec.net_ids)


def install(config, pcb_data) -> None:
    """Read the board's copper-to-hole floor for back-drill walls onto
    `config.backdrill_wall_floor`, when the board declares a back-drill or
    the step is asked to define them; otherwise leave the config alone."""
    if pcb_data is None:
        return
    spec = getattr(config, 'backdrill', None)
    if spec is None and not board_has_backdrills(pcb_data):
        return
    try:
        from obstacle_map import resolve_hole_clearance
        config.backdrill_wall_floor = float(resolve_hole_clearance(pcb_data, config) or 0.0)
    except Exception as e:                                      # noqa: BLE001
        print(f"  WARNING: the board's copper-to-hole floor could not be read for "
              f"back-drill walls ({type(e).__name__}: {e}); "
              f"{defaults.NPTH_TO_TRACK_CLEARANCE:g} mm is used")
    if spec is not None:
        prepare(config)


#: Bumped whenever this module gives a via its back-drills, so an answer
#: kept by `board_has_backdrills` cannot outlive a mark made in place.
_MARKS = [0]


def board_has_backdrills(pcb_data) -> bool:
    """Does any via of the board declare a back-drill? Asked by the passes
    that would otherwise do nothing for a board without one. The answer is
    kept while the via list has the same length and the same last via
    (#339's key: a rip plus an add leaves the count alone and changes the
    tail) and no via has been marked since."""
    vias = pcb_data.vias
    tail = vias[-1] if vias else None
    key = (len(vias),
           (tail.x, tail.y, tail.net_id, bool(getattr(tail, 'backdrills', None)))
           if tail is not None else None, _MARKS[0])
    memo = getattr(pcb_data, '_has_backdrills', None)
    if memo is not None and memo[0] == key:
        return memo[1]
    got = any(getattr(v, 'backdrills', None) for v in vias)
    try:
        pcb_data._has_backdrills = (key, got)
    except Exception:                                           # noqa: BLE001
        pass
    return got


# ---------------------------------------------------------------------------
# Defining back-drills
# ---------------------------------------------------------------------------

def plan_spans(via, joined: Iterable[str], copper: List[str], depths: Dict[str, tuple],
               spec: BackdrillSpec) -> Tuple[dict, dict]:
    """The back-drills `via`'s stubs want: ({token: Backdrill}, {side: why
    not}) with side 'top' / 'bottom'.

    On each side the layers past the outermost layer the net joins the via on
    are the stub. A stub shorter than `spec.min_stub` is left. The drill
    removes the stub's layers from the board face inward, stopping at the
    last layer whose near surface is at least `spec.margin` of dielectric
    from the layer the net uses; a side where no layer is that far is left.
    Lengths come from the stackup, so a board without one is drilled only
    when both numbers are 0."""
    from geometry_utils import via_span_layers
    from kicad_parser import Backdrill
    on = via_span_layers(via, copper)
    if getattr(via, 'via_type', 'through') not in (None, '', 'through'):
        return {}, {'top': 'not a through via', 'bottom': 'not a through via'}
    j = [l for l in on if l in set(joined)]
    if not j:
        return {}, {'top': 'unjoined', 'bottom': 'unjoined'}
    i0, i1 = on.index(j[0]), on.index(j[-1])
    have_depths = all(l in depths for l in on)
    size = round(float(via.drill) + spec.oversize, 4)
    found, why = [], {}
    for side, stub in (('top', on[:i0]), ('bottom', on[i1 + 1:])):
        if not stub:
            continue
        if not have_depths:
            if spec.min_stub > 0 or spec.margin > 0:
                why[side] = 'no stackup'
                continue
            gone = stub
        else:
            if side == 'top':
                length = depths[j[0]][0] - depths[on[0]][0]
                gone = [l for l in stub
                        if depths[j[0]][0] - depths[l][1] >= spec.margin - 1e-9]
            else:
                length = depths[on[-1]][1] - depths[j[-1]][1]
                gone = [l for l in stub
                        if depths[l][0] - depths[j[-1]][1] >= spec.margin - 1e-9]
            if length < spec.min_stub - 1e-9:
                why[side] = 'short'
                continue
        if not gone:
            why[side] = 'margin'
            continue
        if side == 'top':
            found.append(Backdrill(size, on[0], gone[-1]))
        else:
            found.append(Backdrill(size, on[-1], gone[0]))
    tokens = {}
    for token, bd in zip(('backdrill', 'tertiary_drill'), found):
        tokens[token] = bd
    return tokens, why


def _net_copper(pcb_data, net_id, extra_segments=()):
    segs = [s for s in pcb_data.segments
            if s.net_id == net_id and not getattr(s, 'graphic', False)]
    segs += [s for s in extra_segments if s.net_id == net_id]
    zones = [z for z in (getattr(pcb_data, 'zones', None) or []) if z.net_id == net_id]
    pads = (getattr(pcb_data, 'pads_by_net', None) or {}).get(net_id) or ()
    return segs, pads, zones


def mark_new_vias(pcb_data, net_id: int, config, vias, segments, obstacles=None) -> int:
    """Give the vias a route of `net_id` just placed the back-drills their
    stubs want, before the route becomes an obstacle, so the nets routed
    after it keep off the walls. A via whose site is not legal at the net's
    rung is left undrilled. Provisional: `finalize` judges every one again on
    the board that ships. A via is marked ONCE: a later phase of the same net
    (its tap edges) leaves the vias of an earlier one as they are, because a
    via's keep-out may already be on a map, and the map's add and remove must
    read the same back-drill. Returns the number of vias marked."""
    spec = getattr(config, 'backdrill', None)
    if spec is None or net_id not in spec.net_ids or not vias:
        return 0
    try:
        from via_stubs import copper_depths, joined_layers
        copper = list(pcb_data.board_info.copper_layers or [])
        depths = copper_depths(pcb_data)
        segs, pads, zones = _net_copper(pcb_data, net_id, segments or ())
        rung = 0
        coord = None
        if obstacles is not None:
            from obstacle_cache import via_rungs
            from routing_config import GridCoord
            # the net's rung whether or not this route searched at it
            pair = rung_pair_for_net(config, net_id)
            rungs = via_rungs(config, pcb_data)
            rung = rungs.index(pair) + 1 if pair in rungs else 0
            coord = GridCoord(config.grid_step)
        n = 0
        for v in vias:
            if getattr(v, 'net_id', net_id) != net_id or getattr(v, '_backdrill_marked', False):
                continue
            v.backdrills = {}
            v._backdrill_marked = True
            if rung and coord is not None:
                gx, gy = coord.to_grid(v.x, v.y)
                if obstacles.is_via_blocked_rung(gx, gy, rung):
                    continue
            tokens, _why = plan_spans(v, joined_layers(v, segs, pads, zones, copper),
                                      copper, depths, spec)
            if tokens:
                v.backdrills = tokens
                n += 1
        if n:
            _MARKS[0] += 1
        return n
    except Exception as e:                                      # noqa: BLE001
        print(f"  WARNING: back-drills not marked on net {net_id} "
              f"({type(e).__name__}: {e})")
        return 0


class _Index:
    """The board's copper in 2 mm buckets, for `wall_conflicts`."""
    CELL = 2.0

    def __init__(self, pcb_data, copper):
        from net_queries import expand_pad_layers
        self.copper = copper
        self.segs: Dict[tuple, list] = {}
        self.vias: Dict[tuple, list] = {}
        self.pads: Dict[tuple, list] = {}
        c = self.CELL
        for s in pcb_data.segments:
            x0, x1 = sorted((s.start_x, s.end_x))
            y0, y1 = sorted((s.start_y, s.end_y))
            for bx in range(int(math.floor(x0 / c)), int(math.floor(x1 / c)) + 1):
                for by in range(int(math.floor(y0 / c)), int(math.floor(y1 / c)) + 1):
                    self.segs.setdefault((bx, by), []).append(s)
        for v in pcb_data.vias:
            self.vias.setdefault((int(math.floor(v.x / c)), int(math.floor(v.y / c))),
                                 []).append(v)
        self.pad_reach = 0.0
        for fp in pcb_data.footprints.values():
            for p in fp.pads:
                lays = set(expand_pad_layers(p.layers, copper))
                self.pad_reach = max(self.pad_reach, math.hypot(p.size_x, p.size_y) / 2.0)
                self.pads.setdefault((int(math.floor(p.global_x / c)),
                                      int(math.floor(p.global_y / c))), []).append((p, lays))

    def near(self, table, x, y, reach):
        c = self.CELL
        seen = set()
        for bx in range(int(math.floor((x - reach) / c)), int(math.floor((x + reach) / c)) + 1):
            for by in range(int(math.floor((y - reach) / c)),
                            int(math.floor((y + reach) / c)) + 1):
                for it in table.get((bx, by), ()):
                    if id(it) not in seen:
                        seen.add(id(it))
                        yield it


def wall_conflicts(index: _Index, via, layers: Iterable[str], size: float, config,
                   first_only: bool = True) -> List[tuple]:
    """The other nets' copper and holes a back-drill of diameter `size` at
    `via` would stand too close to on `layers`: [(layer, what, net id, gap
    mm, needed mm)]. A track or pad of another net on such a layer, another
    via's copper or its own back-drill's wall there, and any other net's
    drill at the hole-to-hole minimum. Pours are not read: KiCad's fill
    clears a back-drill itself."""
    from geometry_utils import point_to_segment_distance, via_spans_layer
    from check_drc import point_to_pad_distance
    out = []
    r = size / 2.0
    net = via.net_id
    floor = wall_floor(config)
    max_clr = max([config.clearance] + list((getattr(config, 'net_clearances', None)
                                             or {}).values())
                  + list((getattr(config, 'layer_clearances', None) or {}).values()))
    reach = r + max(floor, max_clr)
    layers = list(layers)
    lset = set(layers)

    own_clearance = config.obstacle_clearance(net)

    def need(other_net, layer):
        # the pair's clearance as KiCad grades the disc: the wider of the two
        # nets' classes, the via's own included
        return wall_clearance(config, config.layer_clearance(
            layer, max(own_clearance, config.obstacle_clearance(other_net)), other_net))

    for s in index.near(index.segs, via.x, via.y, reach + 1.0):
        if s.net_id == net or s.layer not in lset:
            continue
        gap = point_to_segment_distance(via.x, via.y, s.start_x, s.start_y,
                                        s.end_x, s.end_y) - s.width / 2.0 - r
        want = need(s.net_id, s.layer)
        if gap < want - 1e-6:
            out.append((s.layer, 'track', s.net_id, round(gap, 4), want))
            if first_only:
                return out
    h2h = float(getattr(config, 'hole_to_hole_clearance', 0.0) or 0.0)
    for o in index.near(index.vias, via.x, via.y, reach + 2.0):
        if o is via or (o.net_id == net and abs(o.x - via.x) < 1e-6 and abs(o.y - via.y) < 1e-6):
            continue
        d = math.hypot(o.x - via.x, o.y - via.y)
        if h2h > 0 and (o.drill or 0) > 0:
            gap = d - hole_diameter(o) / 2.0 - r
            if gap < h2h - 1e-6:
                out.append(('*', 'drill', o.net_id, round(gap, 4), h2h))
                if first_only:
                    return out
        if o.net_id == net:
            continue
        for layer in layers:
            if not via_spans_layer(o, layer):
                continue
            other = size_on_layer(o, layer) or o.size
            gap = d - other / 2.0 - r
            want = need(o.net_id, layer)
            if gap < want - 1e-6:
                out.append((layer, 'via', o.net_id, round(gap, 4), want))
                if first_only:
                    return out
                break
    for p, lays in index.near(index.pads, via.x, via.y, reach + index.pad_reach + 1.0):
        if p.net_id == net and p.net_id:
            continue
        if p.pad_type == 'np_thru_hole':
            hit = lset
        else:
            hit = lays & lset
        if not hit:
            continue
        gap = point_to_pad_distance(via.x, via.y, p) - r
        for layer in layers:
            if layer not in hit:
                continue
            want = need(p.net_id, layer)
            if gap < want - 1e-6:
                out.append((layer, 'pad', p.net_id, round(gap, 4), want))
                if first_only:
                    return out
                break
    return out


def input_snapshot(vias) -> List[tuple]:
    """`(net, x, y, size, back-drills, uuid)` of every via, taken BEFORE a
    step changes the board: what `finalize` tells the input's own vias by."""
    return [(int(v.net_id), float(v.x), float(v.y), float(v.size or 0.0),
             {t: tuple(bd) for t, bd in (getattr(v, 'backdrills', None) or {}).items()},
             getattr(v, 'uuid', '') or '')
            for v in vias or () if v.net_id is not None]


def _inputs_own(via, snap_by_net, input_uuids, given: bool) -> bool:
    """Is `via` one the input board already had, to be left as it is?

    On a written board (`given` False) it is the input's own block when it
    carries an input via's uuid: the writer keeps an untouched via's block
    and gives every via it lays a new one. Otherwise it is the input's only
    where an input via of the net stood at its spot (within half the smaller
    diameter, the reach of a nudge) with the SAME back-drills: a via laid
    again carrying the input's own drills. Standing beside an input via is
    not enough. A re-routed net's vias land there, and each carries the mark
    the route gave it, which nothing has judged yet. With no uuid to read on
    a written board, the same spot and the same (or no) drills is the
    input's."""
    uuid = getattr(via, 'uuid', '') or ''
    if not given and uuid and uuid in input_uuids:
        return True
    mine = {t: tuple(bd) for t, bd in (getattr(via, 'backdrills', None) or {}).items()}
    size = float(getattr(via, 'size', 0.0) or 0.0) or 0.6
    for x, y, isz, drills, _uuid in snap_by_net.get(via.net_id, ()):
        if math.hypot(via.x - x, via.y - y) > max(1e-3, min(size, isz or 0.6) / 2.0):
            continue
        if drills == mine and (drills or (not given and not uuid)):
            return True
    return False


def _drilled(tokens, copper):
    from via_stubs import drilled_layers
    return drilled_layers(tokens, copper)


def finalize(pcb_data, config, input_snapshot, candidates=None) -> Tuple[dict, dict]:
    """Decide, on the board that ships, the back-drills of every via the
    step added for the asked nets. `pcb_data` is that board, `candidates` its
    vias to judge (default: all of `pcb_data.vias`), `input_snapshot` is
    `input_snapshot` of the board's vias before the step.

    Returns ({id(via): (via, {token: Backdrill})}, record), and leaves each
    judged via's `backdrills` as decided. A via the input already had is left
    alone (`_inputs_own`); one the step laid again beside or on an input
    via's spot is the step's, and is judged. The record counts, over the
    step's new vias on
    the asked nets: `vias`, `drilled`, and per reason the sides left
    (`short`, `margin`, `no stackup`, `unjoined`), `conflict` (vias whose
    wall would stand too close to another net on a layer the drill removes,
    left undrilled), `no_stub` (vias with nothing to drill) and
    `conservative_only` (vias drilled here that the all-layer test of the
    search would have refused: what a per-span test in the search could add).
    """
    from via_stubs import copper_depths, joined_layers
    spec = config.backdrill
    copper = list(pcb_data.board_info.copper_layers or [])
    depths = copper_depths(pcb_data)
    snap_by_net: Dict[int, list] = {}
    for ent in input_snapshot or ():
        snap_by_net.setdefault(ent[0], []).append(ent[1:])
    input_uuids = {ent[5] for ent in input_snapshot or () if ent[5]}
    given = candidates is not None
    index = _Index(pcb_data, copper)
    out = {}
    rec = {'nets': len(spec.net_ids), 'oversize_mm': spec.oversize,
           'margin_mm': spec.margin, 'min_stub_mm': spec.min_stub,
           'vias': 0, 'drilled': 0, 'no_stub': 0, 'conflict': 0,
           'sides_left': {}, 'conservative_only': 0, 'conflicts': [],
           'not_covered': list(NOT_COVERED)}
    copper_by_net = {}
    judged = [v for v in (candidates if candidates is not None else pcb_data.vias)
              if v.net_id in spec.net_ids
              and not _inputs_own(v, snap_by_net, input_uuids, given)]
    # Judged afresh, one after the other: a via's wall is weighed against the
    # drills already kept, never against a mark that may yet be refused (two
    # neighbours would otherwise refuse each other).
    for v in judged:
        v.backdrills = {}
    for v in judged:
        rec['vias'] += 1
        if v.net_id not in copper_by_net:
            copper_by_net[v.net_id] = _net_copper(pcb_data, v.net_id)
        segs, pads, zones = copper_by_net[v.net_id]
        tokens, why = plan_spans(v, joined_layers(v, segs, pads, zones, copper),
                                 copper, depths, spec)
        for reason in why.values():
            rec['sides_left'][reason] = rec['sides_left'].get(reason, 0) + 1
        if not tokens:
            rec['no_stub'] += 1
            out[id(v)] = (v, {})
            continue
        size = max(bd[0] for bd in tokens.values())
        hits = wall_conflicts(index, v, _drilled(tokens, copper), size, config)
        if hits:
            rec['conflict'] += 1
            if len(rec['conflicts']) < 20:
                layer, what, other, gap, want = hits[0]
                rec['conflicts'].append({'x': v.x, 'y': v.y, 'layer': layer, 'with': what,
                                         'gap_mm': gap, 'needed_mm': round(want, 4)})
            out[id(v)] = (v, {})
            continue
        rec['drilled'] += 1
        from geometry_utils import via_span_layers
        if wall_conflicts(index, v, via_span_layers(v, copper), size, config):
            rec['conservative_only'] += 1
        v.backdrills = dict(tokens)
        out[id(v)] = (v, tokens)
    _MARKS[0] += 1
    return out, rec


#: What a step that defines back-drills does not do, said in its summary
#: (`not_covered`) and its log so the count is not read as more than it is.
NOT_COVERED = (
    "pours: the judge and the fill model do not read a back-drill "
    "(KiCad's own fill clears it)",
    "the plane steps and the diff-pair step define no back-drill",
    "a drilled pad's back-drill is not an obstacle to other nets",
)


def print_not_covered() -> None:
    print("  Back-drills, not covered: " + "; ".join(NOT_COVERED) + ".")


def print_record(rec, context: str = '') -> None:
    if not rec:
        return
    where = f" ({context})" if context else ""
    left = ', '.join(f"{n} side(s) {why}" for why, n in sorted(rec['sides_left'].items()))
    print(f"\n  BACK-DRILL{where}: {rec['drilled']} of {rec['vias']} new via(s) on "
          f"{rec['nets']} asked net(s) back-drilled (drill + {rec['oversize_mm']:g} mm); "
          f"{rec['no_stub']} with nothing to drill"
          + (f" [{left}]" if left else "")
          + (f"; {rec['conflict']} NOT drilled: the wall would stand too close to "
             f"another net on a layer the drill removes" if rec['conflict'] else "")
          + ".")
