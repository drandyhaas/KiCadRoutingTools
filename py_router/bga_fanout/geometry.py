"""
Geometry calculations for BGA fanout routing.

Functions for calculating 45-degree stubs, exit points, and jog endpoints.
"""
from __future__ import annotations

import bisect
import math
from typing import List, Tuple, Optional, Dict

from bga_fanout.types import BGAGrid, Channel
from bga_fanout.constants import POSITION_TOLERANCE

# Reference prefixes place_fanout_clearance treats as movable passives (keep in
# sync with its --cap-prefix default): an unlocked <=2-copper-pad part with one
# of these prefixes will be nudged off fanout vias by the next pipeline step,
# so the fanout may ignore its copper when placing vias. Everything else --
# locked parts, connectors, test points, ICs -- never moves (#253/#254).
MOVABLE_PASSIVE_PREFIXES = ('C', 'R', 'FB')


def immovable_foreign_pads(pcb_data, exclude_ref: str) -> List:
    """Copper pads of foreign footprints that NO later pipeline step will move:
    locked footprints, and any part place_fanout_clearance won't relocate
    (i.e. not an unlocked 2-pad C*/R*/FB* passive). A through via placed by the
    fanout must clear these on every copper layer; movable caps are deliberately
    NOT included (cap-opt moves them off the vias afterwards).

    POST-ROUTE callers (the #666 in-chain bare-ball rescue) set
    `pcb_data._fanout_all_foreign_immovable = True`: the clearance step is
    pre-route only, so at rescue time NOTHING will move a cap off a via --
    a B-side decap pad under the ball field must then be cleared like any
    locked part (measured: a rescue via-in-pad landed 0.29mm from C59.1's
    B.Cu pad, 0.187mm overlap)."""
    all_immovable = getattr(pcb_data, '_fanout_all_foreign_immovable', False)
    out = []
    for fp in pcb_data.footprints.values():
        if fp.reference == exclude_ref:
            continue
        copper_pads = [p for p in fp.pads
                       if any(str(l).endswith('.Cu') for l in p.layers)]
        if not copper_pads:
            continue
        movable = (not all_immovable
                   and not getattr(fp, 'locked', False)
                   and fp.reference.startswith(MOVABLE_PASSIVE_PREFIXES)
                   and len(copper_pads) <= 2)
        if movable:
            continue
        out.extend(copper_pads)
    return out


def via_clears_pad_rects(x: float, y: float, v_half: float, clearance: float,
                         pads) -> bool:
    """True if a through via at (x, y) clears every pad in `pads` (rect model,
    rect_rotation-aware) by `clearance` edge-to-edge. Vias span all copper
    layers, so the pads' own layers are irrelevant. The previous axis-aligned
    bbox under-covered a ROTATED pad's protruding corners, passing via sites
    whose ring grazed the real copper."""
    from routing_utils import point_to_pad_rect_dist
    for p in pads:
        # Custom-shape pads: measure to the REAL polygon copper, not the
        # symmetric bbox rect, which for an off-centre outline (meander antenna,
        # comb pad) reports a false graze metres from any copper (#232 class).
        if getattr(p, 'shape', None) == 'custom' and getattr(p, 'polygons', None):
            from check_drc import _point_to_polys_distance
            if _point_to_polys_distance(x, y, p.polygons) < v_half + clearance - 1e-9:
                return False
        elif point_to_pad_rect_dist(x, y, p) < v_half + clearance - 1e-9:
            return False
    return True


def clamp_via_to_pad(via_size: float, via_drill: float, pad,
                     floors, via_x: float = None,
                     via_y: float = None) -> Tuple[float, float, str, int]:
    """Issue #202: shrink a via-in-pad so it never bulges past the pad edge.

    A via dropped at a pad centre whose diameter exceeds the pad's smallest
    dimension bulges past the pad copper. A neighbouring different-net trace that
    legitimately cleared the (smaller) pad then grazes the (larger) via -- a real
    VIA-SEGMENT DRC error baked into the board before any signal routing runs
    (the dominant via-seg source on keks: 163 grazes from Ø0.5 vias in 0.41 mm
    pads).

    Clamp the via to the pad's min dimension (a circular via inscribes a
    rect/oval pad at min(size_x, size_y), or at the reach left from an
    off-centre ``via_x``/``via_y``; for a circle pad that is the diameter),
    but NEVER below the fab via floor -- a pad smaller than the smallest
    manufacturable via keeps that floor via (it still bulges) rather than ship an
    unmanufacturable one. The drill follows to hold the annular ring at the fab
    floor.

    The clamp belongs at via PLACEMENT (not a post-pass) so the escape's own
    keep-out modelling uses the real, smaller via and neighbouring fanout tracks
    can route past it.

    ``floors`` is the fab-tier floor LADDER (nominal first, then escalation rungs;
    see fab_tiers.fab_floor_ladder): each rung is a flat floor dict. The via is
    clamped to the first rung whose via fits the pad; if the pad is smaller than
    even the deepest rung's via, it's held at that deepest floor (still bulges).

    ``via_x`` / ``via_y`` are the via's REAL centre, for a caller that has
    placed it. Given, the fit is measured per axis from that offset
    (``min(sx - 2|dx|, sy - 2|dy|)``) instead of from the pad centre; omitted,
    both reduce to ``min(sx, sy)`` and the answer is unchanged. Pass them
    whenever the via may be OFF-CENTRE -- since #846 the QFN commit loop
    clamps any via whose BARREL overlaps its pad, not only a centred one, and
    a centred clamp does not stop those bulging.

    Returns (size, drill, status, rung): status 'fits' (unchanged), 'clamped'
    (shrunk to fit), or 'floor' (the pad -- or the reach left at this offset --
    is smaller than the deepest fab floor, so the via is held there and STILL
    bulges). ``rung`` is the floor index used (0 = nominal/standard;
    >0 = escalated to a more-costly tier).
    """
    # #846 follow-up: the reach a via has INSIDE this pad, per axis. A circular
    # via of diameter d centred at offset (dx, dy) from a rect pad (sx, sy) is
    # contained iff d <= min(sx - 2|dx|, sy - 2|dy|); at dx = dy = 0 that is
    # exactly min(sx, sy), so a caller that does not pass a position keeps
    # today's answer and every predictive call site (via_for_pad and the
    # keep-out sizers, which ask "what size WOULD a via here be") is unchanged.
    #
    # It matters because #846 made this branch reachable for OFF-CENTRE vias --
    # a barrel that overlaps its pad without being centred on it. Clamping such
    # a via to min(sx, sy) does NOT stop it bulging: on qfn_underpad_coupling a
    # 0.25 via at dx 0.3375 in a 0.875 x 0.25 pad still reaches 0.025mm past the
    # edge, and the run reported it as `clamped ... to fit their pad edge`. With
    # the offset the target falls below the deepest fab rung, so the honest
    # answer is 'floor' -- the status this function already has for "the pad is
    # smaller than anything manufacturable; it still bulges".
    if via_x is None or via_y is None:
        pad_min = min(pad.size_x, pad.size_y)
    else:
        dx = abs(float(via_x) - pad.global_x)
        dy = abs(float(via_y) - pad.global_y)
        pad_min = min(pad.size_x - 2.0 * dx, pad.size_y - 2.0 * dy)
    if via_size <= pad_min + 1e-9:
        return via_size, via_drill, 'fits', 0
    if not floors:
        # --escalation off: no rung may be descended, so the via keeps the
        # caller's size (it bulges; the caller's 'floor' disclosure says so).
        return via_size, via_drill, 'floor', 0
    from fab_tiers import note_narrowing

    # First rung whose smallest manufacturable via fits this pad.
    for rung, fab in enumerate(floors):
        floor_dia = fab['via_diameter']
        floor_drill = fab['via_drill']
        if pad_min < floor_dia - 1e-9:
            continue
        # The fab's SMALLEST manufacturable annular ring for THIS rung -- the one its
        # via pair uses ((0.45-0.20)/2 standard, (0.25-0.15)/2 advanced), NOT the
        # larger fab['annular'] (which would force the drill below the floor and
        # needlessly over-shrink a via that fits the pad fine).
        min_annular = (floor_dia - floor_drill) / 2.0
        new_size = round(pad_min, 4)
        # Keep the configured drill, only thinning it as far as the ring needs, and
        # never below this rung's drill floor.
        new_drill = max(min(via_drill, new_size - 2 * min_annular), floor_drill)
        note_narrowing(getattr(pad, 'net_id', None), 'via_diameter', via_size, new_size,
                       'via-in-pad clamp')
        return new_size, round(new_drill, 4), 'clamped', rung

    # Even the deepest rung's via bulges past this pad: hold it at that floor.
    deepest = len(floors) - 1
    note_narrowing(getattr(pad, 'net_id', None), 'via_diameter', via_size,
                   floors[deepest]['via_diameter'], 'via-in-pad clamp (floor)')
    return floors[deepest]['via_diameter'], floors[deepest]['via_drill'], 'floor', deepest


def create_45_stub(pad_x: float, pad_y: float,
                   channel: Channel,
                   escape_dir: str,
                   channel_offset: float = 0.0) -> Tuple[float, float]:
    """
    Create 45-degree stub from pad to channel.

    For differential pairs, the offset is applied AFTER calculating the 45° stub
    based on the channel center. This ensures both P and N travel the same
    horizontal/vertical distance before applying the parallel offset.

    Args:
        pad_x, pad_y: Pad position
        channel: Target channel
        escape_dir: Direction of escape
        channel_offset: Offset from channel center (for diff pairs)

    Returns:
        End position of the 45° stub
    """
    if channel.orientation == 'horizontal':
        # Target Y includes the offset
        target_y = channel.position + channel_offset
        # For true 45°, dx = |dy|
        dy_to_target = target_y - pad_y
        if escape_dir == 'right':
            dx = abs(dy_to_target)
        else:
            dx = -abs(dy_to_target)

        return (pad_x + dx, target_y)
    else:
        # Target X includes the offset
        target_x = channel.position + channel_offset
        # For true 45°, dy = |dx|
        dx_to_target = target_x - pad_x
        if escape_dir == 'down':
            dy = abs(dx_to_target)
        else:
            dy = -abs(dx_to_target)

        return (target_x, pad_y + dy)


def calculate_exit_point(stub_end: Tuple[float, float],
                         channel: Channel,
                         escape_dir: str,
                         grid: BGAGrid,
                         margin: float = 0.5,
                         channel_offset: float = 0.0) -> Tuple[float, float]:
    """Calculate where the route exits the BGA boundary."""
    if channel.orientation == 'horizontal':
        exit_y = channel.position + channel_offset
        if escape_dir == 'right':
            return (grid.max_x + margin, exit_y)
        else:
            return (grid.min_x - margin, exit_y)
    else:
        exit_x = channel.position + channel_offset
        if escape_dir == 'down':
            return (exit_x, grid.max_y + margin)
        else:
            return (exit_x, grid.min_y - margin)


def calculate_jog_end(exit_pos: Tuple[float, float],
                      escape_dir: str,
                      layer: str,
                      layers: List[str],
                      jog_length: float,
                      is_diff_pair: bool = False,
                      is_outside_track: bool = False,
                      pair_spacing: float = 0.0,
                      grid_step: float = 0.0) -> Tuple[Tuple[float, float], Optional[Tuple[float, float]]]:
    """
    Calculate the end position of the 45° jog at the exit.

    For differential pairs, the outside track needs to extend further before
    bending to maintain constant spacing through the 45° turn.

    Jog direction depends on layer:
    - Top layer (F.Cu): 45° to the left (from perspective walking towards BGA edge)
    - Bottom layer (B.Cu): 45° to the right
    - Middle layers: linear interpolation

    Args:
        exit_pos: Starting point of jog
        escape_dir: Direction of escape ('left', 'right', 'up', 'down')
        layer: Current layer
        layers: List of all available layers
        jog_length: Length of the jog (distance from BGA edge to first pad row/col)
        is_diff_pair: Whether this is part of a differential pair
        is_outside_track: Whether this is the outside track of the pair (needs extension)
        pair_spacing: Spacing between P and N tracks

    Returns:
        (jog_end, extension_point) - extension_point is the intermediate point for outside tracks
    """
    # Calculate layer position: 0 = top (left jog), 1 = bottom (right jog)
    try:
        layer_idx = layers.index(layer)
    except ValueError:
        layer_idx = 0

    num_layers = len(layers)
    if num_layers <= 1:
        layer_factor = 0.0  # Default to left jog
    else:
        layer_factor = layer_idx / (num_layers - 1)  # 0 to 1

    # Jog angle: -1 = left, +1 = right (from perspective of walking towards edge)
    # layer_factor 0 (top) -> -1 (left)
    # layer_factor 1 (bottom) -> +1 (right)
    jog_direction = 2 * layer_factor - 1  # Maps 0->-1, 1->+1

    # Calculate jog components based on escape direction
    # At 45°, both components equal jog_length / sqrt(2)
    # Reduce by factor of 4 for a shorter angled segment at the end of each stub
    diag = jog_length / math.sqrt(2) / 4

    ex, ey = exit_pos
    extension_point = None

    # For differential pairs, outside track extends further before bending
    # To maintain constant perpendicular spacing through a 45° turn:
    # extension = pair_spacing * (sqrt(2) - 1) ≈ 0.414 * pair_spacing
    if is_diff_pair and is_outside_track:
        extension = pair_spacing * (math.sqrt(2) - 1)
        if escape_dir == 'right':
            extension_point = (ex + extension, ey)
            ex = ex + extension
        elif escape_dir == 'left':
            extension_point = (ex - extension, ey)
            ex = ex - extension
        elif escape_dir == 'down':
            extension_point = (ex, ey + extension)
            ey = ey + extension
        else:  # up
            extension_point = (ex, ey - extension)
            ey = ey - extension

    if escape_dir == 'right':
        # Walking right, left is up (-Y), right is down (+Y)
        jog_end = (ex + diag, ey + jog_direction * diag)
    elif escape_dir == 'left':
        # Walking left, left is down (+Y), right is up (-Y)
        jog_end = (ex - diag, ey - jog_direction * diag)
    elif escape_dir == 'down':
        # Walking down, left is right (+X), right is left (-X)
        jog_end = (ex - jog_direction * diag, ey + diag)
    else:  # up
        # Walking up, left is left (-X), right is right (+X)
        jog_end = (ex + jog_direction * diag, ey - diag)

    # Land the stub end on the routing grid (issue #149) so the router has an
    # on-grid terminal and a foreign track on the nearest cell can't graze this
    # end by a sub-cell amount. The grid is anchored at the origin (the router's
    # grid nodes are integer multiples of grid_step).
    if grid_step > 0:
        jog_end = (round(jog_end[0] / grid_step) * grid_step,
                   round(jog_end[1] / grid_step) * grid_step)

    return jog_end, extension_point


# --- #620: the run's own output, testable against itself ---------------------

def via_anchors_route(via_x: float, via_y: float, via_size: float,
                      pad_pos: Tuple[float, float],
                      track_width: float = 0.0) -> bool:
    """Does a via at (via_x, via_y) physically REACH this route's track start?

    #854. The question a via-merge must answer is not "is that via somewhere
    inside my pad's rectangle" -- it is "does that via touch the copper this
    route starts from". Every emission path in ``bga_fanout/tracks.py`` starts
    the route's first segment at ``route.pad_pos``, and that track lives on an
    INNER or bottom layer while the ball pad lives on the top one, so the pad
    reaches its own track only THROUGH the via. A via that does not overlap the
    track start therefore connects nothing, however deep inside the pad's
    bounding box its centre happens to sit.

    Keying on the pad box instead let a LARGE pad swallow a SMALLER overlapping
    same-net pad's committed via: the large pad's route then shipped an
    inner-layer track starting where no via reaches, and still counted as
    escaped -- the very defect ``would_overlap_existing_via``'s #620 half was
    fixed to prevent. Measured on ``kicad_files/kit-dev-coldfire-xilinx_5213``,
    footprint VR201 (TO-263-5, net GND): the 10.8 x 9.4mm tab's box is
    (5.41, 4.71), a 5.25 x 4.55 paste sub-pad's via sits at dx 2.775 / dy 2.425
    -- inside the box on both axes -- and the adopted via is **3.6853mm** from
    the tab route's track start, against a 0.225mm ring.

    ``track_width`` defaults to 0, i.e. the via's own copper only. That is the
    conservative reading and the right one for a caller that does not know the
    width; the fanout passes its real (fab-clamped) width.

    This is the SECOND time this rule has been a containment test standing in
    for a reach test -- see ``PendingVias.verdict``'s note on the scalar radius
    an adversarial review caught in PR #852, and #695 before it ("credits a pad
    by via centre but a track by via radius"). It is spelled out so it is not
    the third.
    """
    return (math.hypot(via_x - pad_pos[0], via_y - pad_pos[1])
            <= (via_size or 0.0) / 2.0 + (track_width or 0.0) / 2.0)


class PendingVias:
    """The via-in-pads ONE ``manage_vias`` call has already committed.

    Everything else in that function tests a candidate against the INPUT board:
    ``would_overlap_existing_via`` and ``via_in_pad_conflict`` both iterate
    ``pcb_data.vias``, and ``vias_to_add`` was appended to and never read back.
    So two vias placed in one call were spaced against the board and against
    nothing else. This is the missing half.

    WHAT IS TESTED, AND WHY IT IS NOT SYMMETRIC:

    * **The drill, always.** The balls are SMD and carry no hole, so every hole
      in the neighbourhood is one this pass is creating: a drill pair too close
      is never a condition the board shipped. It is also the arm with a LEVER --
      a thinner drill can cure it (``thin_drill_to_clear``), and a hole pair
      below the fab floor is not a DRC opinion but an unbuildable board.
    * **The ring, only when a via BULGES past its pad** (clamp status
      ``'floor'``: the ball pad is smaller than the deepest fab rung's via, so
      ``clamp_via_to_pad`` gives up and holds the floor, which the pass already
      warns "still bulges past the pad edge").

      The reason is NOT that a fitting via's ring is free. Two things are true
      at once and only the second is a justification:

        - Ring-to-ring spacing for a via clamped into its pad EQUALS the
          footprint's own pad-to-pad spacing, because both features sit at ball
          centres and the via is no wider than the pad. So the fanout is asking
          the fab for an etch pitch the footprint already demands. A bulging via
          asks for a TIGHTER one, which the board's own geometry does not
          demonstrate is achievable. That is the line the split is drawn on.
        - It is NOT true that a fitting via adds no copper. A ball pad is
          ``['F.Cu']``; the via spans F.Cu to B.Cu. On the inner layers and
          B.Cu there is no pad under the ring at all, so at the same spacing
          the copper IS new, and ``check_drc``'s VIA-VIA arm can legitimately
          flag it. An earlier draft of this docstring called such a pair
          "phantom" on the strength of the F.Cu picture, and an adversarial
          review was right to refuse that.

      There is also no lever on this arm: a via-in-pad site is the ball centre
      by definition, and a bulging via is already at the deepest fab rung, so
      neither moving nor shrinking is available. Refusing removes an escape and
      nothing else. That is why the arm is narrow and why it is disclosed.

      AN EARLIER DRAFT CITED A SWEEP HERE -- "of the ring-only rejections whose
      pads are not already sub-clearance, 100% are bulging vias, at every
      clearance" -- and presented it as the measurement the scope rested on.
      **It is a tautology**, and is recorded here so nobody re-derives it and
      believes it: a ring-only rejection needs ``pitch < via + clearance``, and
      "pads not already sub-clearance" is ``pitch >= pad + clearance``; together
      those give ``pad < via``, which IS the bulge condition. It holds for any
      clamp function whatsoever. ``tests/test_620_pending_via_pairs.py`` now
      pins it AS an identity, and measures the thing that is actually
      contingent: what a bulge-blind ring arm would additionally reject.

    The ring arm is foreign-net only: same-net copper in contact is not a
    clearance violation. (``would_overlap_existing_via``'s board-facing test is
    net-BLIND; that is pre-existing and deliberately not changed here.)

    Scalar ``math.hypot`` with a sorted-by-x broad phase, deliberately NOT
    numpy: numpy's hypot is not CPython's Neumaier-compensated one and the two
    disagree by 1 ULP on ~17% of off-grid inputs, each disagreement feeding a
    ``dist < floor`` comparison -- the finding that closed #786/#787. The broad
    phase is also simply faster at this size (2.0 ms vs 12.8 ms vectorised on
    the largest in-repo BGA, 529 balls).
    """

    __slots__ = ('_h2h', '_clearance', '_tol', '_xs', '_rows', '_max_drill',
                 '_max_size')

    def __init__(self, hole_to_hole: float, clearance: float,
                 site_tol: float = None):
        self._h2h = hole_to_hole
        self._clearance = clearance
        self._tol = POSITION_TOLERANCE if site_tol is None else site_tol
        self._xs: List[float] = []      # sorted, parallel to _rows
        self._rows: List[Tuple] = []    # (x, y, size, drill, net_id, bulges)
        self._max_drill = 0.0
        self._max_size = 0.0

    def __len__(self):
        return len(self._xs)

    def tighten(self, x, y, size, drill):
        """Shrink the via recorded at (x, y) to `size`/`drill`.

        The twin merge keeps ONE via for two routes, and which one it keeps was
        whichever arrived first -- so two coincident same-net pads of 0.25 and
        0.60 gave a 0.45 via when the big pad went first, bulging past the
        small pad and re-creating exactly the #202 violation
        ``clamp_via_to_pad`` exists to prevent. An adversarial review found it.
        The surviving via must be the TIGHTER pad's clamp, so the caller
        tightens on a twin hit. Returns True if anything changed.

        `_max_drill`/`_max_size` are deliberately NOT lowered here: they only
        size the broad-phase window, and leaving them high makes the window
        wider than needed, which cannot miss a conflict. Recomputing them would
        be the only way to make them wrong.
        """
        for i, (ox, oy, os_, od, onet, ob) in enumerate(self._rows):
            if ox == x and oy == y:
                if size < os_ - 1e-12 or drill < od - 1e-12:
                    self._rows[i] = (ox, oy, min(size, os_), min(drill, od),
                                     onet, ob)
                    return True
                return False
        return False

    def add(self, x, y, size, drill, net_id, bulges=False):
        """Record a via this call has committed."""
        d = drill or 0.0
        s = size or 0.0
        i = bisect.bisect_left(self._xs, x)
        self._xs.insert(i, x)
        self._rows.insert(i, (x, y, s, d, net_id, bool(bulges)))
        if d > self._max_drill:
            self._max_drill = d
        if s > self._max_size:
            self._max_size = s

    def verdict(self, x, y, size, drill, net_id, bulges=False, tol=1e-6,
                track_width=0.0):
        """How a candidate at (x, y) relates to what this call already placed.

        ``track_width`` is the width of the track this candidate's route will
        emit; a committed via is a twin when it REACHES that route's track
        start at ``via_radius + track_width / 2`` -- see ``via_anchors_route``.
        Default 0 = the via's own copper only.

        THE PAD IS NOT THE TEST, and this took three attempts to get right, so
        it is spelled out.

        * A SCALAR RADIUS was the first bug: ``max(size_x, size_y) / 2 + 0.01``
          against a straight-line distance reaches far outside the copper along
          an OBLONG pad's short axis, and two same-net 0.30 x 1.50 fingers
          0.50mm apart -- an ordinary fine-pitch QFP pair whose pads are 0.20mm
          apart and NOT touching -- were merged into one via, the second route
          shipping with no via at all while still counting as escaped. An
          adversarial review of PR #852 found it, with 17 footprints on 11
          in-repo boards matching the geometry.
        * A PER-AXIS RECTANGLE (``anchor_box``) fixed that and was still the
          wrong question (#854): it closes the equal-size case, because two
          same-net pads of the same size can only be twins if they overlap by
          more than half, but a LARGE pad's box still reaches a small pad whose
          overlap is slight -- and swallows its via. Measured on VR201 of
          ``kit-dev-coldfire-xilinx_5213``: a via **3.6853mm** from the route it
          was supposed to anchor, against a 0.225mm ring.
        * REACH is the question. A pad connects to its own inner-layer track
          only through the via, so a via that does not touch the track start
          connects nothing, wherever its centre sits.

        Returns ``(verdict, detail)``:

        ``('clear', None)``
            nothing committed so far is in the way.
        ``('twin', (x, y, size, drill))``
            the SAME net already has a via INSIDE this ball's pad, so that via
            already connects this ball and a second hole would buy nothing. Two
            routes, one physical hole. An exposed pad modelled as an F.Cu +
            B.Cu pair puts two routes at one coordinate (5 of this repo's 22
            boards do it; ``interf_u`` BUS1 has 31 such sites on one
            component), and distance 0 is below every threshold, so a plain
            spacing test makes twins refuse each other and takes their net with
            them -- while one via serving both routes is strictly better than
            today's, which appends a second identical dict that the writer
            emits as a second ``(via ...)`` at the same point.

            KEYED ON REACH, NOT ON AN EXACT MATCH. An exact-match rule has a
            1 um cliff: an adversarial review found same-net sites 0.0010mm
            apart merging into one via while 0.0011mm apart DROPPED an escape
            outright, because no fab rung can space two holes 1.1 um apart. A
            via that ANCHORS this ball is a via it needs no second hole for,
            which is the question actually being asked -- and every distance-0
            case (an exposed pad modelled as an F.Cu + B.Cu pair) is inside any
            reach, so the cliff stays closed.
        ``('conflict', (reason, x, y))``
            that via is closer than a floor allows. A DIFFERENT net at the same
            site lands here, not in ``'twin'``: two nets sharing one hole is a
            short, and no rung can ever clear distance 0.
        """
        d = drill or 0.0
        s = size or 0.0
        tw = track_width or 0.0
        # Broad phase: nothing outside this x-window can be within either floor
        # of the candidate, whatever its y. The third term is the ANCHOR reach
        # -- the widest committed via plus half a track -- because a twin is
        # decided by whether that via touches this route's track start.
        window = max(d / 2.0 + self._max_drill / 2.0 + self._h2h,
                     s / 2.0 + self._max_size / 2.0 + self._clearance,
                     self._max_size / 2.0 + tw / 2.0)
        lo = bisect.bisect_left(self._xs, x - window)
        hi = bisect.bisect_right(self._xs, x + window)
        best = None
        for (ox, oy, os_, od, onet, obulges) in self._rows[lo:hi]:
            dist = math.hypot(ox - x, oy - y)
            if onet == net_id and via_anchors_route(ox, oy, os_, (x, y), tw):
                return 'twin', (ox, oy, os_, od)
            if dist <= self._tol:
                return 'conflict', ("two nets would share one hole", ox, oy)
            need = d / 2.0 + od / 2.0 + self._h2h
            if dist < need - tol:
                if best is None or dist < best[0]:
                    best = (dist, "drill hole-to-hole vs this run's own via",
                            ox, oy)
                continue
            if (bulges or obulges) and onet != net_id:
                ring = s / 2.0 + os_ / 2.0 + self._clearance
                if dist < ring - tol and (best is None or dist < best[0]):
                    best = (dist, "via ring vs this run's own via (one held "
                            "at the fab floor bulges past its pad)", ox, oy)
        if best is not None:
            return 'conflict', (best[1], best[2], best[3])
        return 'clear', None


def thin_drill_to_clear(drill, floors, rung, clears):
    """The second rung ``manage_vias`` never had (#620).

    A refusal there DROPS the escape -- there is no re-sweep -- so a spacing
    rule that can only refuse turns escaped balls into failed nets. Before
    refusing, descend the fab ladder's drill floors: a thinner drill inside an
    unchanged ring is more manufacturable in every dimension but the drill, so
    what is being spent is fab tier, which is what ``warn_fab_escalation``
    exists to disclose.

    ``clears(candidate_drill)`` is the caller's predicate. Returns the largest
    drill that clears, or ``None`` if no rung does.

    **A ``--fab-overrides`` run has no ladder to descend.** ``fab_floor_ladder``
    collapses to ONE hard rung when an override file is supplied, by design:
    the file states the user's exact fab limits, so there is no deeper tier to
    escalate into. Such a run refuses instead -- the honest answer to a declared
    fab that cannot drill these holes at this pitch. It is also why the
    contributor who first built this fix measured it as pure loss: their arm
    raised ``via_drill`` to 0.35 through an override file, which both makes the
    floor unmeetable at 0.5 mm pitch AND removes every rung that could rescue
    it.
    """
    cands = sorted({f['via_drill'] for f in floors[rung:]
                    if f['via_drill'] < drill - 1e-9}, reverse=True)
    for cand in cands:
        if clears(cand):
            return cand
    return None
