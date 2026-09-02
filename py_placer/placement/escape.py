"""Per-face escape-lane ledger: can the nets on this part's face get OUT?

The question this answers is the one that separates *"the router failed"* from
*"this was never routable"*. For each face of a fine-pitch part, count the lanes
SUPPLIED (how many tracks physically fit alongside that face at the board's own
clearance) against the lanes DEMANDED (how many nets have to leave through it).
A face in deficit is a **binding constraint**: net ordering only chooses WHICH
nets strand there, never how many. That distinction has cost whole runs --
multiple ordering experiments against a face whose ledger would have said it in
seconds.

Three things make this honest rather than a plausible number:

* **Supply is read from the board, never assumed.** Track width and clearance
  come from the netclass / dru floor the board actually carries, so the lane
  pitch is the one a route step would use. A hardcoded 0.2mm pitch would
  manufacture deficits on a fine board and hide them on a coarse one.
* **`blockers` names who ate the lanes.** A count says a face is short; the
  blocker list says which neighbouring part to move. That is the difference
  between a signal and an action, and it is the field to read first. It is
  only that if the names are real: until #835 a neighbour was charged on XY
  overlap alone, so a part on the far side of the board -- copper these tracks
  never share -- and a module outline the part sits INSIDE were both named as
  the one to move. On ulx3s every reported deficit was one of those.
* **Interior pads count toward NO face.** A pad boxed in by its neighbours does
  not escape sideways at all -- it needs a via. Rolling it into a face's demand
  would blame the face for a fanout problem. Reported separately.

Deliberately NOT a min-cost-flow escape router (Yan & Wong, DAC 2009). That is
the right model eventually, but it needs a grid, a solver and a dependency, and
there is nothing in-repo to reuse -- `bga_fanout/escape.py` is ball-lattice
specific and `qfn_fanout/` has no escape model at all. It also has no baseline
to validate against. This ledger becomes that baseline.

numpy only; no networkx in the placement stack.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

FACES = ('north', 'east', 'south', 'west')

# A pad is "interior" when it is not on the part's own pad bounding box, i.e.
# it has neighbours on every side and cannot leave sideways at any pitch.
INTERIOR_EPS = 0.001

# Only parts this fine are worth a ledger. Above it, escape is not the binding
# constraint and the ledger reports noise. Same thresholds as the fanout
# decision in the routing skill, lifted here so there is ONE definition.
FINE_PITCH_MM = 0.6
FINE_PITCH_LARGE_MM = 0.8
LARGE_PAD_COUNT = 40
MIN_PADS = 6

_ARRAY_KEYS = ('BGA', 'PGA', 'LGA', 'CSP', 'WLCSP', 'WLP', 'CGA',
               'QFN', 'DFN', 'QFP')


@dataclass(frozen=True)
class FaceLedger:
    """One face of one part: lanes in, lanes out, and who is in the way."""
    ref: str
    face: str
    span_mm: float            # the face's own length
    lane_pitch_mm: float      # track + clearance, from the board's floor
    supply: int               # lanes that physically fit, after obstructions
    demand: int               # nets that must leave through this face
    blocked_mm: float         # span lost to neighbours / outline
    blockers: Tuple[str, ...]  # WHO ate it -- the actionable field
    nets: Tuple[int, ...]
    # --- the layer term (#700). APPENDED and DEFAULTED, so every existing
    # construction keeps working and every existing key keeps its value.
    via_pitch_mm: float = 0.0
    signal_layers: int = 1
    signal_layers_source: str = 'unknown'

    @property
    def deficit(self) -> int:
        """Lanes short. Positive means the face cannot pass its own nets.

        OWN-LAYER, and it stays that way. `deficit_floor` is the layer-aware
        number; this one is what `health_escape_worst_deficit`, board_brief's
        POSITION_DEPENDENT keys and `options.move_blocker`'s lanes-to-mm
        conversion have always meant.
        """
        return max(0, self.demand - self.supply)

    @property
    def via_slots(self) -> int:
        """Vias that fit in ONE row along this face -- UNOBSTRUCTED.

        Measured on `span_mm`, not on `span_mm - blocked_mm`, and the
        asymmetry with `supply` on the same row is deliberate: over-stating is
        inside this term's contract -- it may never be used to PASS a face --
        and under-stating is not. Feeding a `blocked_mm` that is too large into
        an UPPER bound turns it into a lower one with no warning.

        The original reason was that `_blocked_span` was "side-blind and
        container-blind", citing rp2350's U6: 6.90 of a 6.90mm face charged
        because U8's pad rect contains it, and a blocker on the opposite board
        side. #835 fixed both, so that reason is gone, and the witness was
        wrong about the side half anyway -- U6 is DRILLED, so it occupies both
        faces and its B-side blocker U1 was charged correctly.

        The asymmetry stays, on the argument rather than the example.
        `blocked_mm` is still a pad-BBOX model of a neighbour, which for a
        perimeter-pad part reads as a solid block; and `supply` is a floor
        while this is a ceiling, so they may not be built from the same
        subtraction even when both are sound. Post-#835, U6 is
        supply 2/11/10/12 against demand 13/13/14/14 -- still the board's worst
        part, and no longer fully blocked.

        ONE ROW, and the arithmetic is the reason rather than the caution. A
        second-row via must be reached BETWEEN two first-row vias: the copper
        gap there is `via_pitch - via_diameter`, and a track needs
        `track_width + 2*clearance`. At the repo defaults that is 0.25mm
        available against 0.80mm needed; at glasgow_revC's own netclass, 0.20
        against 0.60. Row two is reachable only by thinning row one to
        alternate slots, which nets about zero -- and that recursion is the
        min-cost-flow escape model this module's docstring declines to be.
        """
        if self.via_pitch_mm <= 0:
            return 0
        return int(self.span_mm // self.via_pitch_mm)

    @property
    def other_layer_lanes(self) -> int:
        """Lanes the OTHER signal layers could carry along this face.

        `int(span // lane)`, NOT `(signal_layers - 1) * supply`. Multiplying
        by `supply` makes the whole term vanish exactly where the deficit is:
        a face in deficit is overwhelmingly one whose channel was eaten, so
        `supply` is 0 and so is any multiple of it. Measured -- rp2350's U6,
        the worst part on the 6-layer board that motivated #700, has supply 0
        on all four faces and would have scored a layer term of zero. Inner
        copper layers carry no footprint bodies, so charging them the
        SURFACE's blockage was never right anyway.
        """
        if self.lane_pitch_mm <= 0 or self.signal_layers <= 1:
            return 0
        return (self.signal_layers - 1) * int(self.span_mm // self.lane_pitch_mm)

    @property
    def supply_other_max(self) -> int:
        """Upper bound on nets this face could shed onto the other layers."""
        return min(self.via_slots, self.other_layer_lanes)

    @property
    def supply_bound(self) -> str:
        """WHICH term binds -- the actionable half, and the honest one.

        And it is ALWAYS `via_slots` for `signal_layers >= 2` -- by
        ARITHMETIC, not by corpus accident, which is a stronger and less
        comfortable statement than "it saturates on the boards we have":

            layer_lanes binds  <=>  floor(span/via) > (L-1)*floor(span/lane)
            which for L >= 2 needs  via_pitch < lane_pitch
            i.e.  max(via_dia + clr, drill + h2h) < track + clr
            i.e.  via_diameter < track_width

        A via narrower than a track. Not a thing. Checked over all 27 in-repo
        boards at their own declared floors: zero.

        So the layer count enters every published number as exactly ONE BIT --
        `L == 1` (no other layer at all) versus `L >= 2` (the via row is the
        constraint). Naming the binding term is what makes that visible
        instead of hiding it inside a `min()`, which would reproduce #700 one
        level up: a number that does not move with the layer count and does
        not say why. Named, it is the finding, and the action follows from it
        -- "six layers do not help this face because the via row binds at 9"
        points at via geometry, underpad fanout or freeing span, none of which
        "add layers" ever did.
        """
        if self.signal_layers <= 1:
            return 'no_other_layer'
        # Guard the pitch the same way `via_slots` and `other_layer_lanes` do.
        # Without it a degenerate `lane_pitch_mm <= 0` makes `other_layer_lanes`
        # 0 and this blames the LAYER COUNT for what is a broken pitch.
        # Unreachable through `escape_ledger`, which floors both to the repo
        # defaults; reachable by constructing a FaceLedger directly.
        if self.lane_pitch_mm <= 0 or self.via_pitch_mm <= 0:
            return 'no_other_layer'
        return ('via_slots' if self.via_slots <= self.other_layer_lanes
                else 'layer_lanes')

    @property
    def deficit_floor(self) -> int:
        """A LOWER BOUND on the deficit: short even using every other layer.

        The dual of an upper bound on supply, and named so nobody reads it as
        "the real total". `deficit_floor > 0` is a STRICTLY STRONGER verdict
        than `deficit > 0`; `deficit_floor == 0` proves nothing at all, which
        is why nothing gates on it.
        """
        return max(0, self.demand - self.supply - self.supply_other_max)

    def to_dict(self):
        return {'ref': self.ref, 'face': self.face,
                'span_mm': round(self.span_mm, 3),
                'lane_pitch_mm': round(self.lane_pitch_mm, 4),
                'supply': self.supply, 'demand': self.demand,
                'deficit': self.deficit,
                'blocked_mm': round(self.blocked_mm, 3),
                'blockers': list(self.blockers), 'nets': len(self.nets),
                # The layer term. The board-wide values (via_pitch_mm,
                # signal_layers, signal_layers_source) live on PartEscape
                # rather than being repeated on all four faces.
                'via_slots': self.via_slots,
                'other_layer_lanes': self.other_layer_lanes,
                'supply_other_max': self.supply_other_max,
                'supply_bound': self.supply_bound,
                'deficit_floor': self.deficit_floor}


@dataclass(frozen=True)
class PartEscape:
    """Every face of one part, plus the pads no face can serve."""
    ref: str
    pitch_mm: float
    faces: Tuple[FaceLedger, ...]
    interior_pads: int        # need a via, not a lane -- a FANOUT signal
    interior_nets: Tuple[int, ...]
    plane_layers_found: Tuple[str, ...] = ()   # #700, disclosure only

    @property
    def worst(self) -> Optional[FaceLedger]:
        live = [f for f in self.faces if f.deficit > 0]
        return max(live, key=lambda f: (f.deficit, f.face)) if live else None

    @property
    def worst_floor(self) -> Optional[FaceLedger]:
        """The worst face by `deficit_floor` -- short on EVERY layer.

        A property as well as a `to_dict` key, deliberately: `worst_deficit`
        exists only as a key, and `getattr(p, 'worst_deficit', 0)` therefore
        returned 0 silently and turned "38 faces are short" into "0". That
        trap is documented in three places in this repo; one is enough.
        """
        live = [f for f in self.faces if f.deficit_floor > 0]
        return (max(live, key=lambda f: (f.deficit_floor, f.face))
                if live else None)

    def to_dict(self):
        f0 = self.faces[0] if self.faces else None
        return {'ref': self.ref, 'pitch_mm': round(self.pitch_mm, 4),
                'faces': [f.to_dict() for f in self.faces],
                'interior_pads': self.interior_pads,
                'interior_nets': len(self.interior_nets),
                # Board-wide, so reported once rather than on all four faces.
                'via_pitch_mm': round(f0.via_pitch_mm, 4) if f0 else 0.0,
                'signal_layers': f0.signal_layers if f0 else 1,
                'signal_layers_source': (f0.signal_layers_source if f0
                                         else 'unknown'),
                # Named even when the clamp kept signal_layers at 1: on
                # interf_u_plane BOTH copper layers are 98% pours, and "this
                # board has no signal layer left" is a fact a reader should
                # see rather than infer from a clamped 1.
                'plane_layers_found': list(self.plane_layers_found),
                'worst_deficit_floor': (self.worst_floor.deficit_floor
                                        if self.worst_floor else 0),
                'worst_face_floor': (self.worst_floor.face
                                     if self.worst_floor else None),
                'worst_face': self.worst.face if self.worst else None,
                'worst_deficit': self.worst.deficit if self.worst else 0}


def _min_step(vals: Sequence[float]) -> float:
    return min((b - a for a, b in zip(vals, vals[1:])), default=float('inf'))


def pad_pitch(fp) -> float:
    """The part's own minimum pad-to-pad spacing, in mm.

    Read off the pad lattice rather than parsed from the footprint NAME: a
    house library's `MY_LIB:U_TINY` carries no pitch in its name, and a name
    that does carry one can disagree with the geometry.
    """
    if len(fp.pads) < 2:
        return float('inf')
    xs = sorted({round(p.local_x, 3) for p in fp.pads})
    ys = sorted({round(p.local_y, 3) for p in fp.pads})
    return min(_min_step(xs), _min_step(ys))


def has_interior_pads(fp) -> bool:
    xs = sorted({round(p.local_x, 3) for p in fp.pads})
    ys = sorted({round(p.local_y, 3) for p in fp.pads})
    if len(xs) < 3 or len(ys) < 3:
        return False
    minx, maxx, miny, maxy = xs[0], xs[-1], ys[0], ys[-1]
    return any(minx < round(p.local_x, 3) < maxx
               and miny < round(p.local_y, 3) < maxy for p in fp.pads)


def fine_pitch_parts(pcb_data, min_pads: int = MIN_PADS) -> List[str]:
    """Refs worth a lane ledger, by PITCH and geometry -- not by pin count.

    The same test the routing skill spells out inline for the fanout decision,
    lifted here so it has one definition rather than a copy in prose. Raw pad
    count is not a signal on its own: a 44-pin THT socket, a 2x20 header and a
    1.27mm connector all clear 40 pads and none of them has an escape problem.

    Through-hole parts are excluded (except PGAs): a THT pin is reachable on
    EVERY copper layer, so there is no escape to be short of.

    NOTE this is deliberately WIDER than the fanout test the routing skill
    applies, and the difference is the point. Fanout asks "are pads boxed in
    such that they need a via", which is why it requires interior pads or a
    large pin count. Escape asks "do the pads on this face fit through the
    channel beside it", and a fine-pitch PERIMETER part has that problem with
    no interior pad at all -- a 0.4mm QFN-64 is the canonical case. The skill's
    inline test catches that one through the footprint NAME, which a house
    library will not carry, so here it is caught by pitch.
    """
    out: List[str] = []
    for ref in sorted(pcb_data.footprints):
        fp = pcb_data.footprints[ref]
        if len(fp.pads) < min_pads:
            continue
        name = (fp.footprint_name or '').upper()
        smd = sum(1 for p in fp.pads if p.drill == 0)
        tht = sum(1 for p in fp.pads if p.drill > 0)
        if tht > smd and 'PGA' not in name:
            continue
        pitch = pad_pitch(fp)
        named = any(k in name for k in _ARRAY_KEYS)
        fine = (pitch <= FINE_PITCH_MM
                or (len(fp.pads) > LARGE_PAD_COUNT
                    and pitch <= FINE_PITCH_LARGE_MM))
        # A named array package still has to BE fine-pitch to have a lane
        # problem: a 1.27mm BGA escapes without help.
        if fine or (named and pitch <= FINE_PITCH_LARGE_MM):
            out.append(ref)
    return out


#: A copper layer is a PLANE when a named-net, board-level zone covers at
#: least this much of the board's bounding box. Measured across the corpus:
#: real planes cover 0.94-0.98 (kit-dev-coldfire B.Cu 0.98 / In1.Cu 0.94 /
#: In2.Cu 0.95, interf_u_plane 0.98 on both, sonde_u 0.96, flat_hierarchy
#: 0.95), while lvds_converter_dualclk's B.Cu zone -- which carries an EMPTY
#: net name -- covers 0.0002. Nothing in the corpus sits between, so any
#: threshold in (0.001, 0.94) separates them; 0.5 is "covers most of the
#: board", which is what the word plane means.
#:
#: OVER-counting planes is the UNSOUND direction, not the safe one. Fewer
#: signal layers -> smaller `supply_other_max` -> LARGER `deficit_floor`, and
#: `deficit_floor` is a lower bound, so inflating it pushes it past the truth
#: and prints a structural finding that is not there. (`via_slots` errs the
#: other way on purpose and can afford to: over-stating supply keeps a lower
#: bound valid.) That asymmetry is why `_pour_layers` measures the polygon's
#: own area rather than its bounding box.
MIN_PLANE_AREA_FRACTION = 0.5


def via_pitch(pcb_data, pcb_file: Optional[str] = None,
              clearance: Optional[float] = None,
              via_diameter: Optional[float] = None,
              via_drill: Optional[float] = None,
              hole_to_hole: Optional[float] = None,
              design_rules: Optional[dict] = None) -> float:
    """Centre-to-centre spacing of two escape vias, at the board's OWN floor.

    The via-side twin of `lane_pitch`, and resolved by the same POLICY for the
    same reason -- board first, repo defaults second, and NO fab wrap. Not by
    the same call: this goes through `list_nets.board_floor`, which has a
    third tier `lane_pitch`'s `board_default_netclass_param` does not -- the
    board CONSTRAINTS (`min_via_diameter`, `min_hole_to_hole`). That tier is
    necessary, because `hole_to_hole` has no netclass at all. The two agree on
    `clearance` on all 27 in-repo boards, and rather than rely on that the
    ledger passes `lane_pitch`'s own resolved value in. Fab-flooring one half while the other stays board-first is not a
    conservative choice, it is an incommensurate one. Measured, summed
    `deficit_floor` over each whole board, shipped resolution against a
    fab-floored via pitch:

        tigard      10 -> 0     watchy       3 -> 0
        rp2350      39 -> 0     glasgow      3 -> 0
        orangecrab  17 -> 1     ulx3s        0 -> 0  (already zero)

    Four of the five boards with a non-zero floor collapse to nothing. A
    supply term that erases the finding it is annotating is not a supply term.

    (The numbers to compare are `deficit_floor`, not `deficit`: the own-layer
    deficit is `demand - supply` and no via-pitch change can move it. An
    earlier draft of this docstring quoted 41/121/19/25 -- the own-layer
    figures -- which overstated the collapse and, by including ulx3s, counted
    a board that was already at zero.)

    The rule itself is `fab_tiers.min_via_center_distance` -- the #491
    arithmetic, `max(via_diameter + clearance, via_drill + hole_to_hole)`,
    where using only the copper half ships legal copper the fab cannot drill.
    Reached through `fab_tiers` rather than `diff_pair_routing` because that
    import costs +126 modules including the Rust router and numpy, into a
    module whose docstring promises numpy only.

    `clearance` is passed in by the ledger from `lane_pitch`'s OWN resolution
    rather than resolved again here, so the two halves of a face's arithmetic
    cannot price the same board at different clearances.
    """
    import routing_defaults as defaults
    from fab_tiers import min_via_center_distance
    path = pcb_file or getattr(pcb_data, 'source_path', None)
    want = {'clearance': clearance, 'via_diameter': via_diameter,
            'via_drill': via_drill, 'hole_to_hole': hole_to_hole}
    fallback = {'clearance': defaults.CLEARANCE,
                'via_diameter': getattr(defaults, 'VIA_SIZE', 0.5),
                'via_drill': getattr(defaults, 'VIA_DRILL', 0.3),
                'hole_to_hole': getattr(defaults, 'HOLE_TO_HOLE_CLEARANCE',
                                        0.2)}
    if path and any(v is None for v in want.values()):
        try:
            import list_nets
            # The caller may already have read them -- `escape_ledger` does,
            # for the lane pitch. A second read per ledger is ~12ms against a
            # ~46ms ledger, on a path `portfolio` grades once per candidate.
            dr = (design_rules if design_rules is not None
                  else list_nets.read_design_rules(path))
            for key, val in want.items():
                if val is None:
                    got, _src = list_nets.board_floor(path, key, None,
                                                      fallback[key], dr)
                    want[key] = got
        except Exception:                       # noqa: BLE001 - best effort
            pass
    for key, val in want.items():
        if not val:
            want[key] = fallback[key]
    return min_via_center_distance(want['via_diameter'], want['clearance'],
                                   want['via_drill'], want['hole_to_hole'])


def signal_layer_count(pcb_data, *, signal_layers: Optional[int] = None,
                       plane_layers: Optional[Sequence[str]] = None
                       ) -> Tuple[int, str, Tuple[str, ...]]:
    """``(count, source, plane_layers_found)`` -- copper layers that can
    RECEIVE an escape.

    Precedence, most authoritative first:

      ``declared_count``   an explicit ``signal_layers=``. The only channel
                           `options.add_layers` can use, since it is asking
                           about a stackup this board does not have yet.
      ``declared_planes``  caller-named plane layers, spelled the way
                           ``health.ignore_net_ids`` is.
      ``zones``            OBSERVED: layers carrying a named-net, board-level
                           zone covering >= MIN_PLANE_AREA_FRACTION of the
                           board bbox.
      ``copper_layers``    every copper layer. The honest default, and the
                           one that will nearly always fire -- placement runs
                           BEFORE the pours exist, and measured, 0 of the six
                           boards the placer actually runs on (glasgow,
                           tigard, rp2350, watchy, ulx3s, orangecrab)
                           declares a single zone. `rp2350_fpga_eensy_prePlane`
                           says so in its own filename. This source is
                           OPTIMISTIC and every consumer must print it.
      ``unknown``          no copper-layer list at all. Returns 1, so the
                           layer term contributes exactly ZERO and the ledger
                           reports today's numbers unchanged. Note this is
                           tested BEFORE ``plane_layers``, so a declared plane
                           list on a board with no copper-layer list is
                           ignored rather than subtracted from nothing --
                           deliberate, and the only place the stated order
                           does not hold literally.

    There is deliberately no heuristic: `route.py`'s own docstring states that
    this toolchain "cannot auto-detect which layers are ground planes vs
    signal layers", and the parser DISCARDS the declared layer type (issue
    #76 widened the copper filter to accept 'power'/'mixed'/'jumper', and the
    pcbnew path has no type at all). So this either observes a pour or is
    told.

    CLAMPED AT 1, and the clamp is not padding: interf_u_plane is a 2-layer
    board with 98% pours on BOTH sides, so the raw subtraction is 0 and
    ``(L-1) * x`` would go NEGATIVE -- an upper bound TIGHTENING a verdict,
    the one thing this term must never do.

    The part's OWN layer is always counted, plane or not, because `supply`
    already measures that layer directly; this only decides how many OTHERS
    exist. Never returns None: `tests/test_board_brief.py` fails on any
    undeclared None leaf, with a message about position dependence that reads
    as unrelated.
    """
    cu = [l for l in (getattr(getattr(pcb_data, 'board_info', None),
                              'copper_layers', None) or [])
          if str(l).endswith('.Cu')]
    if signal_layers is not None:
        return max(1, int(signal_layers)), 'declared_count', ()
    if not cu:
        return 1, 'unknown', ()
    if plane_layers is not None:
        # A declaration that matched NOTHING must not read as a declaration
        # that was honoured. A typo (`In1.cu`), a net name where a layer was
        # meant, or a bare string instead of a list (which iterates into
        # characters) all yield an empty match -- and the answer would then be
        # the OPTIMISTIC `len(cu)` while the source still said
        # `declared_planes`. The user's input was ignored and the report said
        # it was used, which is the shape `bus_corridors_phantom` exists for.
        wanted = [l for l in plane_layers if isinstance(l, str)]
        found = tuple(sorted({l for l in wanted if l in cu}))
        if wanted and not found:
            return len(cu), 'declared_planes (none matched a copper layer)', ()
        return max(1, len(cu) - len(found)), 'declared_planes', found
    found = _pour_layers(pcb_data, cu)
    if found:
        return max(1, len(cu) - len(found)), 'zones', found
    bounds = getattr(getattr(pcb_data, 'board_info', None),
                     'board_bounds', None)
    # "I could not look" is not "there is nothing there". A board with no
    # bounds cannot be measured for pours, and the fallback is the OPTIMISTIC
    # answer, so the source string has to say why.
    return len(cu), ('copper_layers' if bounds else
                     'copper_layers (bounds unreadable; pours not measurable)'
                     ), ()


def _pour_layers(pcb_data, copper) -> Tuple[str, ...]:
    """Copper layers carrying a board-level, named-net, board-sized pour."""
    bounds = getattr(getattr(pcb_data, 'board_info', None),
                     'board_bounds', None)
    zones = getattr(pcb_data, 'zones', None) or []
    if not bounds or not zones:
        return ()
    board_area = max(1e-9, (bounds[2] - bounds[0]) * (bounds[3] - bounds[1]))
    out = set()
    for z in zones:
        layer = getattr(z, 'layer', None)
        if layer not in copper or layer in out:
            continue
        if not (getattr(z, 'net_name', '') or '').strip():
            continue          # a keep-out or an unnamed sliver, not a plane
        if getattr(z, 'in_footprint', False):
            continue          # a footprint's own local pour
        poly = getattr(z, 'polygon', None) or ()
        if len(poly) < 3:
            continue
        # The polygon's OWN area (shoelace), not its bounding box. A bbox
        # over-reports a non-convex pour, and over-reporting coverage is the
        # UNSOUND direction here: it drops a signal layer, which shrinks
        # `supply_other_max`, which RAISES `deficit_floor` -- a lower bound
        # pushed past the truth. Demonstrated on tigard with an injected
        # L-shaped GND pour covering 0.190 of the board whose bbox covers
        # 1.000: the bbox form declared In1.Cu a plane and took the board from
        # 4 signal layers to 3. No corpus board triggers it (the worst real
        # area/bbox ratio is 0.965), which is exactly why it had to be found
        # by construction rather than by fixture.
        if _polygon_area(poly) / board_area >= MIN_PLANE_AREA_FRACTION:
            out.add(layer)
    return tuple(sorted(out))


def _polygon_area(poly) -> float:
    """Absolute area of a closed polygon, by the shoelace formula."""
    n = len(poly)
    if n < 3:
        return 0.0
    total = 0.0
    for i in range(n):
        x1, y1 = poly[i][0], poly[i][1]
        x2, y2 = poly[(i + 1) % n][0], poly[(i + 1) % n][1]
        total += x1 * y2 - x2 * y1
    return abs(total) / 2.0


def lane_pitch_parts(pcb_data, pcb_file: Optional[str] = None,
                     track_width: Optional[float] = None,
                     clearance: Optional[float] = None,
                     return_rules: bool = False):
    """``(track_width, clearance)`` as `lane_pitch` resolves them.

    Split out so the VIA half of a face's arithmetic can be priced at the
    clearance the LANE half used, rather than resolving the board a second
    time down a slightly different path. Two reads that agree on 27 of 27
    in-repo boards still agree by coincidence; this makes them agree by
    construction.
    """
    tw, clr, dr = _lane_parts(pcb_data, pcb_file, track_width, clearance)
    return (tw, clr, dr) if return_rules else (tw, clr)


def lane_pitch(pcb_data, pcb_file: Optional[str] = None,
               track_width: Optional[float] = None,
               clearance: Optional[float] = None) -> float:
    """One lane = one track plus one clearance, at the board's OWN floor.

    Resolved from the board rather than defaulted, because the number this
    feeds is a hard verdict: at 0.2mm a face passes and at 0.35mm the same
    face is in deficit, and picking the wrong one manufactures or hides a
    structural finding. Explicit arguments win, then the board's Default
    netclass, then the repo defaults.
    """
    tw, clr, _dr = _lane_parts(pcb_data, pcb_file, track_width, clearance)
    return tw + clr


def _lane_parts(pcb_data, pcb_file, track_width, clearance):
    """``(track_width, clearance, design_rules_or_None)``.

    The rules come back so the VIA half can be priced off the same read.
    """
    import routing_defaults as defaults
    dr = None
    path = pcb_file or getattr(pcb_data, 'source_path', None)
    if path and (track_width is None or clearance is None):
        try:
            import list_nets
            dr = list_nets.read_design_rules(path)
            if clearance is None:
                clearance = list_nets.board_default_netclass_param(
                    path, 'clearance', dr)
            if track_width is None:
                track_width = list_nets.board_default_netclass_param(
                    path, 'track_width', dr)
        except Exception:                       # noqa: BLE001 - best effort
            pass
    if not clearance:
        clearance = defaults.CLEARANCE
    if not track_width:
        track_width = getattr(defaults, 'TRACK_WIDTH', 0.25)
    return float(track_width), float(clearance), dr


def _part_rect(fp) -> Tuple[float, float, float, float]:
    """The part's pad bounding box in board coordinates."""
    xs = [p.global_x for p in fp.pads]
    ys = [p.global_y for p in fp.pads]
    return min(xs), min(ys), max(xs), max(ys)


def _face_of(pad, rect, pitch) -> Optional[str]:
    """Which face a pad escapes through, or None when it is interior.

    A pad on the bounding box escapes through the side it sits on; a corner pad
    is assigned to the side it is closest to, deterministically. `pitch/2` is
    the tolerance, so a pad row that is not perfectly collinear still counts.
    """
    minx, miny, maxx, maxy = rect
    tol = max(pitch / 2.0, INTERIOR_EPS)
    d = {'west': pad.global_x - minx, 'east': maxx - pad.global_x,
         'north': pad.global_y - miny, 'south': maxy - pad.global_y}
    near = min(d.values())
    if near > tol:
        return None
    # Ties resolve by FACES order, not by dict order, so the answer does not
    # depend on how the pads were enumerated.
    for f in FACES:
        if abs(d[f] - near) < 1e-9:
            return f
    return None


def _face_geometry(rect, face) -> Tuple[float, float, float, float]:
    """The face as a segment (x1, y1, x2, y2), in board coordinates."""
    minx, miny, maxx, maxy = rect
    return {'north': (minx, miny, maxx, miny),
            'south': (minx, maxy, maxx, maxy),
            'west': (minx, miny, minx, maxy),
            'east': (maxx, miny, maxx, maxy)}[face]


def board_side_map(pcb_data) -> Dict[str, frozenset]:
    """{ref: frozenset} of the board sides each footprint obstructs (#835).

    Computed ONCE per ledger and threaded down: `_blocked_span` runs per part
    per face and walks every footprint, so resolving this inside it would
    rebuild the map `targets * 4` times. Measured on glasgow that is ~50% on
    top of the whole ledger; hoisted it is 0.1-0.7ms against 11-55ms.

    `placement.legality` is imported lazily, as this module imports
    `routing_defaults` and `fab_tiers`: escape.py keeps a stdlib-only import
    surface at module scope. There is no cycle -- `legality` imports neither
    `escape` nor anything heavy at module scope.
    """
    from .legality import (footprint_has_through_pads, footprint_side,
                           sides_occupied)
    return {r: sides_occupied(footprint_side(f), footprint_has_through_pads(f))
            for r, f in pcb_data.footprints.items()}


def board_container_refs(pcb_data, pcb_file=None) -> set:
    """Refs the courtyard channel already treats as containers (#835).

    Delegates to `legality.container_refs` rather than re-deriving the test,
    so a part cannot be a frame to one channel and a blocker to the other.
    """
    from .legality import container_refs, graded_parts_from_file
    try:
        return container_refs(pcb_data,
                              graded_parts_from_file(pcb_data, pcb_file))
    except Exception:                                        # noqa: BLE001
        # A board whose local bounds cannot be resolved keeps today's
        # answer -- nothing exempt -- rather than losing the whole ledger.
        return set()


def span_eaten(lo, hi, band, horizontal, obstacles):
    """How much of [lo, hi] the obstacles cover, and how much each took.

    The shared obstruction kernel (#835). `obstacles` is `[(ref, rect)]`;
    WHICH neighbours are in it, and which rectangle each contributes, is the
    caller's decision -- see `_blocked_span` below (pad bboxes, side- and
    container-filtered) and `routability.face_lane_ledger` (courtyards). The
    two callers deliberately disagree about that choice and agree about this
    arithmetic, which is the half they were getting different answers for.

    Returns `(blocked_mm, ((ref, mm), ...))`, the pairs ordered by how much
    each took so the first name is the one to move.

    Intervals are UNIONED, so two neighbours covering the same stretch are
    charged once. That is the correction: without it a face can be reported as
    more than fully blocked, and an over-100% total silently becomes "supply 0"
    rather than an error. Measured, glasgow_revC's J1 east has NINE neighbours
    covering 12.42mm of a 9.64mm face -- summed that is supply 0, unioned it is
    8.23mm and supply 3. (Nine, not the eight `face_lane_ledger` reports: its
    `eaten_by` is truncated to the top 8 for display, so counting that list
    undercounts the obstruction and sums to 11.48mm rather than 12.42mm.)

    Corpus-wide over the git-tracked boards: 25 of the 388 faces the ESCAPE
    ledger reports have neighbours covering the same stretch twice, and
    `face_lane_ledger` has 506 of 5276 taken over every footprint. Regenerate
    both by instrumenting this function -- the counts are scope-dependent, and
    an unscoped one is not a number.

    The `min(blocked, hi - lo)` below is BELT AND BRACES, and provably so while
    the union stands: every interval is already clipped to [lo, hi] as it is
    built, so their union is a subset of [lo, hi] and cannot measure more than
    `hi - lo`. It is kept because it is the guard that binds the moment the
    union is removed, and a mutation that drops it is recorded as an expected
    survivor rather than deleted (`tests/mutate_834_835.py`). Do not read it as
    the thing that fixed the 12.42-on-9.64 number; the union is.
    """
    intervals: List[Tuple[float, float, str]] = []
    for ref, (oxmin, oymin, oxmax, oymax) in obstacles:
        across = (oymin, oymax) if horizontal else (oxmin, oxmax)
        if across[1] < band[0] or across[0] > band[1]:
            continue                              # not in the escape band
        along = (oxmin, oxmax) if horizontal else (oymin, oymax)
        a, b = max(lo, along[0]), min(hi, along[1])
        if b > a:
            intervals.append((a, b, ref))

    intervals.sort()
    blocked = 0.0
    cur_a = cur_b = None
    for a, b, _name in intervals:
        if cur_a is None:
            cur_a, cur_b = a, b
        elif a <= cur_b:
            cur_b = max(cur_b, b)
        else:
            blocked += cur_b - cur_a
            cur_a, cur_b = a, b
    if cur_a is not None:
        blocked += cur_b - cur_a

    by_ref: Dict[str, float] = {}
    for a, b, name in intervals:
        by_ref[name] = by_ref.get(name, 0.0) + (b - a)
    order = tuple((r, by_ref[r])
                  for r in sorted(by_ref, key=lambda r: (-by_ref[r], r)))
    return min(blocked, hi - lo), order


def face_band(rect, face, reach):
    """`(lo, hi, band, horizontal)` for one face -- the kernel's coordinates.

    `reach` is how far off the face to look: past that, a track has room to
    turn and the neighbour is no longer on the escape path.
    """
    minx, miny, maxx, maxy = rect
    horizontal = face in ('north', 'south')
    lo, hi = (minx, maxx) if horizontal else (miny, maxy)
    if face == 'north':
        band = (miny - reach, miny)
    elif face == 'south':
        band = (maxy, maxy + reach)
    elif face == 'west':
        band = (minx - reach, minx)
    else:
        band = (maxx, maxx + reach)
    return lo, hi, band, horizontal


def _blocked_span(pcb_data, ref, rect, face, reach, courtyards=None,
                  sides=None, containers=None):
    """How much of a face's span is unusable, and WHO took it.

    A neighbour parked off a face does not merely crowd it -- its own body
    occupies the channel the escaping tracks need. The overlap of the
    neighbour's extent with the face's span, projected onto the face, is what
    those tracks cannot use.

    Two neighbours are NOT in the way, and were charged anyway (#835):

    * one that shares no board face with the escaping part. Copper on the far
      side is copper these tracks never have to share, and a THT part occupies
      both faces -- which is why the test is the symmetric `own & other` over
      `sides_occupied(...)`, the predicate the rest of the package uses
      (`legality.pair_min_gap`, `routability.pair_channel_widths`), rather
      than `face_lane_ledger`'s one-sided `own_side in g.sides`. Measured on
      rp2350, U6 is drilled and so occupies both faces: the symmetric form
      keeps all four of its blockers, the one-sided form would drop the B-side
      U1, losing a real obstruction.
    * a CONTAINER -- a module-outline footprint hosting the design, which by
      `legality.CONTAINER_RATIO` covers at least half the board. It is not
      parked off this face; the escaping part is inside it. rp2350's U8 is a
      Teensy module whose 66 perimeter pads bound 17.3 x 34.1mm of mostly
      empty interior, and every part under it was charged its whole face.
      `legality.py`'s own comment already says pairs with a container member
      are exempt from the courtyard channels EVERYWHERE; escape is the channel
      that never got the exemption.

    `sides` / `containers` are the per-board maps `escape_ledger` computes once
    (see `board_side_map`). Both are looked up with `.get`, and a ref they do
    not name is charged exactly as before -- a caller passing a partial map
    must not silently lose an obstruction.
    """
    lo, hi, band, horizontal = face_band(rect, face, reach)
    own = None if sides is None else sides.get(ref)

    obstacles = []
    for other in sorted(pcb_data.footprints):
        if other == ref:
            continue
        ofp = pcb_data.footprints[other]
        if not ofp.pads:
            continue
        if own is not None:
            oth = sides.get(other)
            if oth is not None and not (own & oth):
                continue
        if containers is not None and other in containers:
            continue
        obstacles.append((other, _part_rect(ofp)))

    blocked, order = span_eaten(lo, hi, band, horizontal, obstacles)
    return blocked, tuple(r for r, _mm in order)


def part_escape(pcb_data, ref, *, pitch_mm: Optional[float] = None,
                ignore_net_ids: Optional[Sequence[int]] = None,
                reach_mm: Optional[float] = None,
                via_pitch_mm: Optional[float] = None,
                signal_layers: int = 1,
                signal_layers_source: str = 'unknown',
                plane_layers_found: Sequence[str] = (),
                sides: Optional[Dict[str, frozenset]] = None,
                containers: Optional[set] = None,
                pcb_file: Optional[str] = None) -> PartEscape:
    """The full per-face ledger for one part.

    `ignore_net_ids` drops plane-routed rails, exactly as elsewhere in this
    module: a GND pad does not need a lane, it needs a via to the plane, and
    counting it as demand reports a deficit on every power-heavy face of every
    board.

    The layer arguments default to "no other layer", so a caller that does not
    pass them gets exactly today's numbers: `signal_layers=1` makes
    `other_layer_lanes` zero, which makes `supply_other_max` zero, which makes
    `deficit_floor == deficit`.
    """
    fp = pcb_data.footprints[ref]
    ignored = set(ignore_net_ids or ())
    lane = pitch_mm if pitch_mm is not None else lane_pitch(pcb_data)
    rect = _part_rect(fp)
    pitch = pad_pitch(fp)
    reach = reach_mm if reach_mm is not None else max(lane * 4.0, 1.0)
    # #835: resolved ONCE per part, not once per face. `escape_ledger` passes
    # both maps in; a direct caller gets them built here so this stays a
    # standalone entry point.
    if sides is None:
        sides = board_side_map(pcb_data)
    if containers is None:
        containers = board_container_refs(pcb_data, pcb_file)

    demand: Dict[str, List[int]] = {f: [] for f in FACES}
    interior: List[int] = []
    for pad in fp.pads:
        nid = getattr(pad, 'net_id', 0)
        if not nid or nid in ignored:
            continue
        face = _face_of(pad, rect, pitch if pitch != float('inf') else lane)
        if face is None:
            interior.append(nid)
        else:
            demand[face].append(nid)

    faces: List[FaceLedger] = []
    for f in FACES:
        x1, y1, x2, y2 = _face_geometry(rect, f)
        span = math.hypot(x2 - x1, y2 - y1)
        blocked, blockers = _blocked_span(pcb_data, ref, rect, f, reach,
                                          sides=sides,
                                          containers=containers)
        usable = max(0.0, span - blocked)
        nets = tuple(sorted(set(demand[f])))
        faces.append(FaceLedger(
            ref=ref, face=f, span_mm=span, lane_pitch_mm=lane,
            supply=int(usable // lane) if lane > 0 else 0,
            demand=len(nets), blocked_mm=blocked, blockers=blockers,
            nets=nets,
            via_pitch_mm=float(via_pitch_mm or 0.0),
            signal_layers=max(1, int(signal_layers)),
            signal_layers_source=signal_layers_source))
    return PartEscape(ref=ref, pitch_mm=pitch, faces=tuple(faces),
                      interior_pads=len(interior),
                      interior_nets=tuple(sorted(set(interior))),
                      plane_layers_found=tuple(plane_layers_found or ()))


def escape_ledger(pcb_data, *, refs: Optional[Sequence[str]] = None,
                  pcb_file: Optional[str] = None,
                  track_width: Optional[float] = None,
                  clearance: Optional[float] = None,
                  ignore_net_ids: Optional[Sequence[int]] = None,
                  reach_mm: Optional[float] = None,
                  via_diameter: Optional[float] = None,
                  via_drill: Optional[float] = None,
                  hole_to_hole: Optional[float] = None,
                  signal_layers: Optional[int] = None,
                  plane_layers: Optional[Sequence[str]] = None
                  ) -> List[PartEscape]:
    """Ledgers for every fine-pitch part, worst deficit first.

    Returns [] on a board with no fine-pitch parts, which is the common case
    and is not a failure -- there is simply no escape question to ask.

    `signal_layers` / `plane_layers` feed the #700 layer term. Left alone they
    are OBSERVED (a board-sized pour makes a layer a plane) and otherwise
    every copper layer counts -- see `signal_layer_count`, whose `source` is
    reported so a reader can see which answer they got. `reach_mm` reaches
    `part_escape`'s escape-band depth, which was previously unreachable
    through this entry point at all.
    """
    tw, clr, dr = lane_pitch_parts(pcb_data, pcb_file, track_width, clearance,
                                   return_rules=True)
    lane = tw + clr
    # The via half is priced at the SAME clearance the lane half resolved, so
    # the two numbers on one row cannot come from different rules.
    vpitch = via_pitch(pcb_data, pcb_file, clearance=clr,
                       via_diameter=via_diameter, via_drill=via_drill,
                       hole_to_hole=hole_to_hole, design_rules=dr)
    nsig, source, planes = signal_layer_count(
        pcb_data, signal_layers=signal_layers, plane_layers=plane_layers)
    targets = list(refs) if refs is not None else fine_pitch_parts(pcb_data)
    # #835: both per-board maps resolved ONCE, then threaded into every part.
    sides = board_side_map(pcb_data)
    containers = board_container_refs(pcb_data, pcb_file)
    out = [part_escape(pcb_data, r, pitch_mm=lane,
                       ignore_net_ids=ignore_net_ids, reach_mm=reach_mm,
                       via_pitch_mm=vpitch, signal_layers=nsig,
                       signal_layers_source=source, plane_layers_found=planes,
                       sides=sides, containers=containers)
           for r in targets if r in pcb_data.footprints]
    # Sorted by the OWN-LAYER deficit, unchanged. It selects `escape_lanes[:10]`
    # and board_brief's `worst[:WORST_N]`, and feeds
    # `health_escape_worst_deficit` -- so re-sorting on `deficit_floor` would
    # move a published number. Consequence to know: the part with the largest
    # `deficit_floor` is not necessarily first, and can be truncated away.
    out.sort(key=lambda p: (-(p.worst.deficit if p.worst else 0), p.ref))
    return out


def format_text(ledgers: Sequence[PartEscape], limit: int = 5) -> str:
    """The ledger as the table a human reads before blaming the router."""
    if not ledgers:
        return "escape lanes: no fine-pitch parts on this board"
    lines = []
    short = [p for p in ledgers if p.worst]
    lines.append(f"escape lanes ({len(ledgers)} fine-pitch part(s), "
                 f"{len(short)} with a face in deficit):")
    for p in ledgers[:limit]:
        flag = (f"  DEFICIT {p.worst.deficit} lane(s) on {p.worst.face}"
                if p.worst else "  ok")
        lines.append(f"  {p.ref} (pitch {p.pitch_mm:.2f}mm){flag}")
        for f in p.faces:
            if f.demand == 0 and f.supply == 0:
                continue
            mark = '!!' if f.deficit else '  '
            who = (f"; blocked {f.blocked_mm:.1f}mm by "
                   f"{', '.join(f.blockers[:3])}" if f.blockers else "")
            lines.append(f"   {mark} {f.face:<6} supply {f.supply:>3} / "
                         f"demand {f.demand:<3} "
                         f"(span {f.span_mm:.1f}mm @ {f.lane_pitch_mm:.3f}mm"
                         f"{who})")
        if p.interior_pads:
            lines.append(f"      {p.interior_pads} interior pad(s) escape "
                         f"through NO face -- that is a fanout question, not a "
                         f"lane one")
    if len(ledgers) > limit:
        lines.append(f"  ... {len(ledgers) - limit} more")
    return '\n'.join(lines)
