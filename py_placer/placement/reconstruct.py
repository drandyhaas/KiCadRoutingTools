"""Tier classification and mounting-hole pattern fits.

What is left of the structural ("puzzle") reconstruction solver, kept for two
readers:

  * `floorplan.emit_intent`: `classify` the parts into tiers (locked/static,
    zero-net frame pieces, anchors, smalls), `fit_corner_insets` proposes the
    corner-inset poses of zero-net drilled parts (mounting holes), and
    `rigid_vectors` turns the offsets between current and proposed poses into
    the +/-v displacement vectors that mark a pattern as SUSPECT in the
    emitted intent's lock advice;
  * `lock_advisor`: `fit_family_orbits` / `orbit_seats` find parts seated on
    a ring around a centre (a radial LED or cap ring) so they are advised as
    a set.
"""
from __future__ import annotations

import math
from typing import Dict, List, Sequence, Set, Tuple


GRID_TOL = 0.05          # pattern-fit residual tolerance (one routing grid)


# --------------------------------------------------------------------------
# classify
# --------------------------------------------------------------------------

class Tiers:
    __slots__ = ('locked', 'zero_net', 'anchors', 'smalls', 'threshold',
                 'edge')

def part_extent_mm(state, ref: str) -> float:
    """Max pad-extent dimension of a part at its current rotation (mm)."""
    if state.legality_ctx is not None:
        pp = state.legality_ctx.parts.get(ref)
        if pp is not None:
            e = pp.extent_local(state.parts[ref].rot)
            if e is not None:
                return max(e[2] - e[0], e[3] - e[1])
    r = state.parts[ref].rect()
    return max(r[2] - r[0], r[3] - r[1])


def classify(state, intent=None, anchor_extent='auto') -> Tiers:
    """The puzzle tiers. Frame first: locked + zero-net (mechanical) parts;
    anchors = large pad extents (edge-connector intent refs always anchors);
    everything else is a small."""
    t = Tiers()
    t.locked = {r for r, p in state.parts.items() if p.locked}
    # A ref the intent declares must_lock belongs in the frame too, not only
    # one the file stamps: a hand-written must_lock is a statement that the
    # ref's pose is a decision, which is exactly what the frame tier means.
    if intent is not None and getattr(intent, 'must_lock', None):
        import fnmatch as _fn
        refs_all = sorted(state.parts)
        for pat in intent.must_lock:
            t.locked |= set(_fn.filter(refs_all, pat))
    # UNWIRED, not pin-count-zero (run-10 A1). The frame tier means "this
    # part's position is not a netlist question". `pin_count == 0` reads that
    # off the pad count, which assumes a net-less mounting hole is also
    # PAD-less -- true for a bare NPTH, false for every `MountingHole_*_Pad*`
    # footprint, whose plated pad KiCad gives a real, non-zero net id under an
    # `unconnected-(REF-PadN)` placeholder NAME. `quench` builds `pads_local`
    # with `net_id > 0`, so such a hole counts a pin and misses this tier.
    #
    # Measured on the run-10 subject, whose 9 mounting holes are all of that
    # kind: `zero_net` came out EMPTY, and two things followed silently.
    # `fit_corner_insets` scans `zero_net | locked`, so it saw only the 2
    # holes the FILE happened to lock, found no group of >=2 in distinct
    # corners, and returned {} -- disabling the hole-pattern fit, the one rung
    # that can carry a part an arbitrary distance home. `rigid_vectors` is
    # pattern-gated, so it then had nothing either, and the whole structural
    # ladder was inert while `legalize` (capped at --max-move) was left to do
    # the work alone. The other 7 holes also landed in `smalls`, i.e. free for
    # a search to move a mounting hole.
    #
    # Ask the netlist instead: a pad whose net touches no OTHER part is not a
    # connection -- which is the same >=2-parts rule the routable denominator
    # uses. Strictly wider than the old test (a part with no pads has no nets,
    # so `all(...)` is vacuously true) and it frames only parts nothing wires:
    # on the subject board exactly the 9 holes plus 6 `Z*` mechanical parts,
    # and none of the 11 parts the damage displaced.
    _net_parts: Dict[int, set] = {}
    for _r, _p in state.parts.items():
        for _nid in (getattr(_p, 'nets', ()) or ()):
            _net_parts.setdefault(_nid, set()).add(_r)
    #
    # AND DRILLED, when it has pads at all (run-10 W17). "Nothing wires it" is
    # necessary and not sufficient: on a single-part fixture board the sole
    # part's net-neighbourhood is trivially solo, so the unwired test alone
    # framed the one part the board exists to place -- measured on three
    # in-repo QFN fanout fixtures. A part carrying pads earns the frame only
    # if those pads are DRILLED, which is what separates a plated mounting
    # hole from an unwired SMD part, and is the same `has_tht` filter
    # `fit_corner_insets` applies to whatever this tier hands it. A part with
    # no pads at all keeps the old unconditional pass.
    t.zero_net = {r for r, p in state.parts.items()
                  if r not in t.locked
                  and all(len(_net_parts.get(nid, ())) < 2
                          for nid in (getattr(p, 'nets', ()) or ()))
                  and (p.pin_count == 0 or getattr(p, 'has_tht', False))}
    free = [r for r in state.parts if r not in t.locked | t.zero_net]
    exts = sorted(part_extent_mm(state, r) for r in free)
    if anchor_extent == 'auto':
        p75 = exts[int(0.75 * (len(exts) - 1))] if exts else 3.5
        thr = max(3.5, p75)
    else:
        thr = float(anchor_extent)
    t.threshold = round(thr, 3)
    # edge_claims(): forcing a connector_affinity part into the anchor tier
    # would let a class declaration reshape the reconstruction.
    edge_refs = ({c['ref'] for c in intent.edge_claims()}
                 if intent is not None else set())
    t.edge = edge_refs & set(state.parts)   # run-4 F2: kept, not discarded
    t.anchors = {r for r in free
                 if part_extent_mm(state, r) >= thr or r in edge_refs}
    t.smalls = {r for r in free if r not in t.anchors}
    return t


# --------------------------------------------------------------------------
# fit_pattern (propose-only)
# --------------------------------------------------------------------------

def _slot_on_board(state, x: float, y: float) -> bool:
    """Is a constructed slot actually on the board?

    Slots are built from the bounding box, and a notched or rounded board has
    bbox corners that are not on it. Enumerating those offers a hole a seat it
    could never occupy.
    """
    b = state.board
    if not (b[0] - 1e-6 <= x <= b[2] + 1e-6 and b[1] - 1e-6 <= y <= b[3] + 1e-6):
        return False
    gate = getattr(getattr(state, 'legality_ctx', None), 'gate', None)
    outer = getattr(gate, 'outer', None)
    if not outer:
        return True                       # bbox IS the outline; already checked
    try:
        from check_drc import _point_on_board
        return bool(_point_on_board(x, y, outer, gate.cutouts))
    except Exception:
        return True


def _pattern_slots(state, inset_x: float, inset_y: float, mid_edges: bool):
    """The seats a hole family at this inset can occupy: 4 corners, +4 mid-edges.

    A slot is (edge-perpendicular inset, along-edge position). A CORNER fixes
    the along-position to the inset from the adjacent perpendicular edge; a
    MID-EDGE slot fixes it to that edge's midpoint, at the same perpendicular
    inset. Both are derived from the one inset the survivors over-determine --
    a mid-edge slot invents no new parameter.
    """
    b = state.board
    mx, my = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
    slots = {'SW': (b[0] + inset_x, b[1] + inset_y),
             'NW': (b[0] + inset_x, b[3] - inset_y),
             'SE': (b[2] - inset_x, b[1] + inset_y),
             'NE': (b[2] - inset_x, b[3] - inset_y)}
    if mid_edges:
        slots.update({'W': (b[0] + inset_x, my), 'E': (b[2] - inset_x, my),
                      'S': (mx, b[1] + inset_y), 'N': (mx, b[3] - inset_y)})
    return {k: (round(x, 4), round(y, 4)) for k, (x, y) in slots.items()
            if _slot_on_board(state, x, y)}


def fit_corner_insets(state, tiers: Tiers) -> Dict[str, List[Tuple[float, float]]]:
    """Hole-pattern fit over zero-net DRILLED parts (mounting holes).

    Survivors: holes whose (inset_x, inset_y) agree with a common inset within
    GRID_TOL, in DISTINCT corners. >= 2 survivors over-determine a
    translation; holes seated by no family get every FREE slot of every fitted
    family as proposed positions. Propose-only: `rigid_vectors` turns the
    proposals into the displacement vectors the intent's lock advice reads.

    Four things this gets right that the corner-only version did not, each
    measured on a board damaged by a 66mm rigid swap:

    EVERY GROUP, not the largest. A board can carry two legitimate hole
    families (one corpus board has {3.302, 3.302} and {7.62, 1.27}). Fitting
    only the largest makes the second family's members read as displaced, and
    they then compete for the first family's free seat -- which is how two
    holes that had never moved became claimants.

    MID-EDGE SLOTS. A hole one inset in from an edge, mid-span along it, is an
    ordinary mounting pattern and the corner model has no hypothesis for it. On
    the measured board the displaced hole's true home is exactly the north
    edge's midpoint at the fitted inset; with no slot there its offset agreed
    with nothing, the support >= 2 rule discarded the one correct offset along
    with three wrong ones, and the whole ladder produced no vector.

    ...but ONLY WHEN OVER-SUBSCRIBED. Mid-edge slots sit nearer the board
    centre than the corners, so on a board whose corner model is sufficient
    they hand DIST_TIEBREAK_PER_MM a closer wrong answer. Enumerate them only
    when the corner model cannot seat everyone: survivors + unseated unlocked
    holes > 4. Counting only UNLOCKED holes is load-bearing -- one corpus board
    carries five locked zero-net through-hole connectors that would otherwise
    inflate demand on every board it appears with.

    AT SEAT IS GLOBAL. A hole matching any fitted family's inset is at seat: it
    is not a candidate, and its slot is taken. Scoping that to one family is
    what let a correct hole be offered another family's corner.
    """
    from collections import defaultdict
    b = state.board
    corners = {'SW': (b[0], b[1]), 'NW': (b[0], b[3]),
               'SE': (b[2], b[1]), 'NE': (b[2], b[3])}
    holes = []
    for ref in sorted(tiers.zero_net | (tiers.locked & set(state.parts))):
        p = state.parts[ref]
        if not p.has_tht:
            continue
        best = min(corners.items(),
                   key=lambda kv: abs(p.x - kv[1][0]) + abs(p.y - kv[1][1]))
        holes.append((ref, best[0], abs(p.x - best[1][0]),
                      abs(p.y - best[1][1])))
    groups = defaultdict(list)
    for ref, corner, ix, iy in holes:
        # PER-AXIS conformance (run-8 A1). This used to demand ix ~= iy, i.e.
        # a SQUARE inset, so an ordinary asymmetric pattern -- a hole 7.62mm
        # in from one edge and 1.27mm from the other, which is just an
        # imperial layout -- never conformed and the fit found nothing at all.
        # Group on the inset PAIR instead: the two axes have to agree across
        # holes, not with each other.
        groups[(round(ix / GRID_TOL), round(iy / GRID_TOL))].append(
            (ref, corner, (ix, iy)))

    fits = []
    for _key, members in sorted(groups.items(), key=lambda kv: -len(kv[1])):
        distinct = {m[1] for m in members}
        if len(members) < 2 or len(distinct) != len(members):
            continue
        fits.append((sum(m[2][0] for m in members) / len(members),
                     sum(m[2][1] for m in members) / len(members),
                     {m[0] for m in members}))
    if not fits:
        return {}

    # RECOGNISE GENEROUSLY, OFFER CONSERVATIVELY.
    #
    # "At seat" is positional -- is this hole standing on a pattern seat? -- and
    # is asked against every slot the families could have, mid-edge included,
    # whether or not those slots are offered to anyone. Asking it about INSETS
    # instead gets a mid-edge hole wrong: its nearest-corner inset is the
    # along-edge distance (99.06mm on the measured board), which matches no
    # family, so a hole standing exactly on its own seat reads as displaced --
    # and the corpus sweep duly caught this proposing a healthy board's hole a
    # move to the position it was already in.
    #
    # Being generous here is the safe direction: it only ever REMOVES
    # candidates. Offering a slot is the risky direction, and that stays behind
    # the pigeonhole gate below.
    every_slot = []
    for inset_x, inset_y, _s in fits:
        every_slot.extend(_pattern_slots(state, inset_x, inset_y, True).values())

    def at_seat(ref) -> bool:
        p = state.parts[ref]
        return any(abs(p.x - sx) <= GRID_TOL and abs(p.y - sy) <= GRID_TOL
                   for sx, sy in every_slot)

    # Demand is board-wide, so a hole seated by family B never inflates family
    # A's pigeonhole count.
    candidates = [ref for ref, _c, _ix, _iy in holes
                  if ref not in tiers.locked and not at_seat(ref)]

    proposals: Dict[str, List[Tuple[float, float]]] = defaultdict(list)
    for inset_x, inset_y, survivors in fits:
        over = len(survivors) + len(candidates) > 4
        slots = _pattern_slots(state, inset_x, inset_y, over)
        taken = {name for name, (sx, sy) in slots.items()
                 if any(abs(state.parts[r].x - sx) <= GRID_TOL
                        and abs(state.parts[r].y - sy) <= GRID_TOL
                        for r, _c, _ix, _iy in holes)}
        free = [xy for name, xy in sorted(slots.items()) if name not in taken]
        for ref in candidates:
            proposals[ref].extend(free)
    return {r: sorted(set(c)) for r, c in proposals.items() if c}


# --------------------------------------------------------------------------
# family orbits -- a DETECTOR, deliberately not a proposal source
# --------------------------------------------------------------------------

#: A family member's radius must agree with the consensus to this (mm).
ORBIT_RADIUS_TOL_MM = 0.20
#: ...and its angular residue with its class to this (degrees).
ORBIT_ANGLE_TOL_DEG = 0.75
#: Fewest members that may define an orbit. Measured: the corpus no-op below
#: holds at 5, 4 and 3 -- 5 is chosen because the fit has four continuous
#: parameters (cx, cy, r, theta0) plus a discrete m, and five members give ten
#: observations.
ORBIT_MIN_INLIERS = 5
#: Rotational orders tried. 2 is excluded by the CURVATURE guard: a 2-fold
#: "orbit" is a point reflection and has no curvature at all, so any two parts
#: anywhere define one.
ORBIT_M_RANGE = range(3, 25)
#: OVER-DETERMINATION: distinct SEATS a residue class must occupy. Seats, not
#: members -- two co-located parts are one piece of evidence about a pattern.
ORBIT_MIN_SEATS_PER_CLASS = 3
#: OCCUPANCY: fraction of the orbit's slots that must actually be filled.
ORBIT_MIN_OCCUPANCY = 0.60
#: SCALE: the fitted circle must be at most this fraction of the board
#: diagonal, and its centre on the board. A near-collinear family fits an
#: enormous circle exactly as a long airwire fits a damage vector.
ORBIT_MAX_RADIUS_FRAC = 0.75
#: Deterministic bound on circumcentre hypotheses per family. A 100-member
#: family has 161700 triples; the stride covers the whole family rather than a
#: prefix of it, so a ring whose refs sort late is not missed.
ORBIT_MAX_HYPOTHESES = 4000


class OrbitFit:
    """One fitted rotational orbit of a footprint family."""
    __slots__ = ('family', 'cx', 'cy', 'r', 'm', 'residues', 'slots',
                 'inliers', 'free_slots')

    def __init__(self, family, cx, cy, r, m, residues, slots, inliers,
                 free_slots):
        self.family = family
        self.cx, self.cy, self.r, self.m = cx, cy, r, m
        self.residues = residues
        self.slots = slots
        self.inliers = inliers
        self.free_slots = free_slots

    def __repr__(self):                     # pragma: no cover - diagnostics
        return (f'OrbitFit({self.family} m={self.m}x{len(self.residues)} '
                f'r={self.r:.4f} c=({self.cx:.4f},{self.cy:.4f}) '
                f'{len(self.inliers)}/{self.slots})')


def _circumcentre(p1, p2, p3):
    ax, ay = p1
    bx, by = p2
    cx, cy = p3
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        return None                     # collinear: no finite circumcentre
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    return (ux, uy)


def fit_family_orbits(state, tiers=None, *,
                      min_inliers: int = ORBIT_MIN_INLIERS
                      ) -> List[OrbitFit]:
    """Footprint families that lie on a regular rotational orbit.

    A DETECTOR and nothing else. It proposes no poses and contributes no gate
    term; its whole product is the fact "this part is AT SEAT on a fitted
    orbit, corroborated by N others". Two consumers want that fact and neither
    needs a proposal: `lock_advisor` (protect an intact array from a legality
    search) and any model that would otherwise claim an at-seat part is
    displaced -- `fit_corner_insets` has no hypothesis for a rotational
    pattern, and on the measured board read H2, standing on an exact 3-fold
    orbit with H1/H3 (radii agreeing to 0.8 nanometres), as displaced and
    offered it a 5.2 mm move.

    THE GUARDS ARE THE DESIGN. Un-guarded this is the shape that got
    `airwire_cluster_vectors` refuted, so the firing rate on the 33 HEALTHY
    in-repo corpus boards was measured guard by guard BEFORE shipping any of
    it, and `tests/test_orbit_fit_noop.py` pins the whole matrix:

        no guards at all                                 28 of 33 fire
        + scale        (r <= 0.75*diag, centre on board) 28 of 33
        + curvature    (m >= 3)                          28 of 33
        + over-determination (>= 3 distinct SEATS/class) 14 of 33
        + occupancy    (>= 0.60 of slots filled)          7 of 33
        + min_inliers 4                                   2 of 33
        + min_inliers 5   <- SHIPPED                      0 of 33

      scale               r <= ORBIT_MAX_RADIUS_FRAC * board diagonal, centre
                          inside the bbox
      curvature           m >= 3 (a 2-fold orbit is a point reflection)
      over-determination  every residue class occupies >= 3 distinct SEATS
      occupancy           filled seats >= ORBIT_MIN_OCCUPANCY * slots
      min_inliers         >= 5 members on the fitted orbit

    TWO CORRECTIONS TO THE INVESTIGATION THAT PROPOSED THIS, both measured
    here and both in the direction that flattered the proposal:

    1. It reported the un-guarded rate as 6 of 33 and claimed the guarded rate
       stayed 0 with min_inliers lowered to 4 and to 3. Neither reproduces: the
       un-guarded rate is 28 of 33, and relaxing min_inliers to 4 and 3 fires
       on 2 and 7 boards. `min_inliers` is a load-bearing guard here, not a
       formality, and lowering it is a regression rather than a free widening.
    2. Over-determination MUST count distinct seats, not members. Counting
       members, glasgow_revC's six `Fiducial_0.75mm_Mask1.5mm` -- which are
       three positions with a front/back pair at each -- read as six
       corroborations of a 10-fold orbit through three points, and that fit
       passed every other guard. Two co-located parts are one piece of evidence
       about a pattern.

    WHAT THE THRESHOLD COSTS, stated rather than left to be discovered: a
    3- or 4-member orbit is below `min_inliers` and is NOT detected. On the
    measured board that means the exact 3-fold mounting-hole orbit H1/H2/H3
    (radii agreeing to 0.8 nanometres) is invisible here, so this detector does
    NOT refute `fit_corner_insets`' claim that H2 -- which is at seat -- is
    displaced. That refutation needs either a hole-specific relaxation or a
    different model; it is not delivered by this one.

    A near-collinear line of parts is REFUSED, not fitted -- which is correct
    behaviour and not a gap: the case that motivates wanting one (six 0603s at
    1.5 mm pitch, all six displaced together) has no surviving member to anchor
    a line, and a pattern fitter with no survivors must propose nothing.

    `tiers` is accepted for symmetry with `fit_corner_insets` and is unused:
    an orbit is a property of a family's geometry, not of a part's size tier.
    """
    from collections import defaultdict
    from itertools import combinations

    b = state.board
    diag = math.hypot(b[2] - b[0], b[3] - b[1])
    r_max = ORBIT_MAX_RADIUS_FRAC * diag

    families: Dict[str, List[str]] = defaultdict(list)
    for ref in sorted(state.parts):
        fam = getattr(state.parts[ref], 'footprint_name', None)
        if fam:
            families[fam].append(ref)

    fits: List[OrbitFit] = []
    for fam in sorted(families):
        members = families[fam]
        if len(members) < min_inliers:
            continue
        pts = {r: (state.parts[r].x, state.parts[r].y) for r in members}

        # ---- centre hypotheses: circumcentres of member triples ------------
        total = len(members) * (len(members) - 1) * (len(members) - 2) // 6
        stride = max(1, total // ORBIT_MAX_HYPOTHESES)
        seen_c = set()
        centres = []
        for i, tri in enumerate(combinations(members, 3)):
            if i % stride:
                continue
            c = _circumcentre(*(pts[t] for t in tri))
            if c is None:
                continue
            key = (round(c[0] / 0.01), round(c[1] / 0.01))
            if key in seen_c:
                continue
            seen_c.add(key)
            centres.append(c)

        best = None
        for (cx, cy) in centres:
            if not (b[0] <= cx <= b[2] and b[1] <= cy <= b[3]):
                continue                                        # scale guard
            radii = sorted((math.hypot(pts[r][0] - cx, pts[r][1] - cy), r)
                           for r in members)
            # Consensus radius = the largest run within tolerance.
            i = 0
            while i < len(radii):
                j = i
                while (j + 1 < len(radii)
                       and radii[j + 1][0] - radii[i][0] <= ORBIT_RADIUS_TOL_MM):
                    j += 1
                run = radii[i:j + 1]
                i = j + 1
                if len(run) < min_inliers:
                    continue
                rr = sum(t[0] for t in run) / len(run)
                if rr > r_max or rr < 1e-6:
                    continue                                    # scale guard
                cand = _fit_orbit_m(fam, cx, cy, rr,
                                    [t[1] for t in run], pts, min_inliers)
                if cand is None:
                    continue
                key = (-len(cand.inliers), cand.slots - len(cand.inliers),
                       -cand.m)
                if best is None or key < best[0]:
                    best = (key, cand)
        if best is not None:
            fits.append(best[1])
    return fits


def _fit_orbit_m(family, cx, cy, r, refs, pts, min_inliers):
    """Best rotational order for `refs` on the circle (cx, cy, r), or None."""
    angles = {ref: math.degrees(math.atan2(pts[ref][1] - cy,
                                           pts[ref][0] - cx)) % 360.0
              for ref in refs}
    best = None
    for m in ORBIT_M_RANGE:
        step = 360.0 / m
        # Residue classes: the angle reduced mod the rotational step. A
        # complete orbit of an m-fold pattern with k independent seeds has
        # exactly k classes, each occupied m times.
        buckets: Dict[int, List[str]] = {}
        bucket_res: Dict[int, float] = {}       # each class's REFERENCE residue
        for ref, a in sorted(angles.items()):
            res = a % step
            hit = None
            for key, r0 in bucket_res.items():
                d = abs(res - r0)
                if min(d, step - d) <= ORBIT_ANGLE_TOL_DEG:
                    hit = key
                    break
            if hit is None:
                hit = round(res / ORBIT_ANGLE_TOL_DEG)
                bucket_res[hit] = res
            buckets.setdefault(hit, []).append(ref)
        # OVER-DETERMINATION, counted in SLOTS rather than in members. A
        # residue class seen once or twice is a fitted free parameter, not
        # evidence -- and the members holding it must be at DIFFERENT seats.
        # Measured: glasgow_revC carries six `Fiducial_0.75mm_Mask1.5mm` at
        # three positions (a front/back pair at each), and counting members
        # made 3 distinct points look like 6 corroborations, which was enough
        # for a 10-fold "orbit" through them to survive every other guard. Two
        # co-located parts are one piece of evidence about a pattern.
        #
        # The slot index is taken RELATIVE TO THE CLASS'S REFERENCE RESIDUE,
        # never by flooring the raw angle: a residue that wraps (an angle of
        # 179.99999 deg on a 45 deg step has residue 44.99999, which IS the
        # same class as 0.0) floors into the PREVIOUS slot, so an exact 8-fold
        # ring read as two members sharing a seat and was mis-fitted as 4-fold
        # with two residue classes.
        occupied = {}                   # (class, k) -> refs on that seat
        for k, v in sorted(buckets.items()):
            for ref in v:
                slot = int(round((angles[ref] - bucket_res[k]) / step)) % m
                occupied.setdefault((k, slot), []).append(ref)
        seats_per_class: Dict[int, set] = {}
        for (k, slot) in occupied:
            seats_per_class.setdefault(k, set()).add(slot)
        classes = {k: v for k, v in buckets.items()
                   if len(seats_per_class.get(k, ()))
                   >= ORBIT_MIN_SEATS_PER_CLASS}
        if not classes:
            continue
        inliers = sorted(ref for v in classes.values() for ref in v)
        if len(inliers) < min_inliers:
            continue
        slots = len(classes) * m
        filled = sum(len(seats_per_class[k]) for k in classes)
        # OCCUPANCY: 5 parts spread over a 21-slot orbit is a coincidence, and
        # so are 3 seats of 10.
        if filled < ORBIT_MIN_OCCUPANCY * slots:
            continue
        # Circular mean about the class reference, for the same wrap reason:
        # averaging 0.0 and 44.99999 arithmetically gives 22.5, which is not a
        # residue any member has.
        residues = tuple(sorted(
            (bucket_res[k] + sum(((angles[ref] % step) - bucket_res[k]
                                  + step / 2.0) % step - step / 2.0
                                 for ref in v) / len(v)) % step
            for k, v in classes.items()))
        # Pick the m with the FEWEST FREE SLOTS; ties go to the larger m.
        key = (slots - filled, -m)
        if best is None or key < best[0]:
            free = []
            seated = [angles[ref] for ref in inliers]
            for res in residues:
                for k in range(m):
                    a = (res + k * (360.0 / m)) % 360.0
                    if any(min(abs(a - oa), 360.0 - abs(a - oa))
                           <= ORBIT_ANGLE_TOL_DEG for oa in seated):
                        continue
                    free.append((cx + r * math.cos(math.radians(a)),
                                 cy + r * math.sin(math.radians(a))))
            best = (key, OrbitFit(family, cx, cy, r, m, residues, slots,
                                  tuple(inliers), tuple(sorted(free))))
    return None if best is None else best[1]


def orbit_seats(fits: Sequence['OrbitFit'], min_corroborating: int = 3
                ) -> Dict[str, 'OrbitFit']:
    """{ref: the fit it is AT SEAT on}, for fits with enough corroboration.

    `min_corroborating` counts the OTHER members holding the same orbit, so 3
    means a part plus three others agreeing on centre, radius and pitch. That
    is the threshold at which the fit stops being a description of the part
    itself: four points over-determine a circle's three parameters."""
    out: Dict[str, OrbitFit] = {}
    for f in fits:
        if len(f.inliers) < min_corroborating + 1:
            continue
        for ref in f.inliers:
            out.setdefault(ref, f)
    return out


# --------------------------------------------------------------------------
# rigid_vector
# --------------------------------------------------------------------------

def rigid_vectors(state, proposals: Dict[str, List[Tuple[float, float]]],
                  grid_tol: float = GRID_TOL) -> List[Tuple[float, float]]:
    """Candidate group vectors from the pattern proposals: current - proposed,
    deduped up to SIGN (a swap displaces two groups by +v and -v).

    Run-4 F3(a): a vector is kept only with support from >= 2 DISTINCT refs
    -- R4's own letter ("two or more agree, up to sign"), which the code did
    not implement: every proposal x free-corner pairing minted a vector, and
    run 3's spurious cross-corner vector mis-moved J7 to 31.6 mm from home,
    WORSE than its 15.8 mm input.

    Run-5: the symmetric survivor of that rule is gone too. A symmetric
    cross-pairing (two holes x two corners) self-supports with 2 refs: the
    same refs mapped onto the same slots CROSSED mint a second, spurious
    vector. Competing pairings of the SAME ref-set onto the SAME slot-set
    are one assignment question, and only the minimal-mean-displacement
    pairing is evidence (the F1 nearest-slot principle applied at the
    vector layer); the crossed pairing ships each part ~2x farther. The
    post-assign prune sweep (`prune_assignment`) remains the backstop."""
    support: Dict[Tuple[float, float], Set[str]] = {}
    pairs: Dict[Tuple[float, float],
                List[Tuple[str, Tuple[float, float], float]]] = {}
    order: List[Tuple[float, float]] = []
    for ref, cands in sorted(proposals.items()):
        p = state.parts[ref]
        for (px, py) in cands:
            v = (p.x - px, p.y - py)
            d = math.hypot(*v)
            if d < grid_tol:
                continue
            slot = (round(px, 4), round(py, 4))
            canon = v if (v[1], v[0]) > (0, 0) else (-v[0], -v[1])
            for w in order:
                if math.hypot(canon[0] - w[0], canon[1] - w[1]) <= 2 * grid_tol:
                    support[w].add(ref)
                    pairs[w].append((ref, slot, d))
                    break
            else:
                key = (round(canon[0], 4), round(canon[1], 4))
                order.append(key)
                support[key] = {ref}
                pairs[key] = [(ref, slot, d)]
    kept = [v for v in order if len(support[v]) >= 2]
    groups: Dict[Tuple[frozenset, frozenset], List[Tuple[float, float]]] = {}
    for v in kept:
        gk = (frozenset(r for r, _s, _d in pairs[v]),
              frozenset(s for _r, s, _d in pairs[v]))
        groups.setdefault(gk, []).append(v)
    out = []
    for v in kept:
        gk = (frozenset(r for r, _s, _d in pairs[v]),
              frozenset(s for _r, s, _d in pairs[v]))
        grp = groups[gk]
        if len(grp) > 1:
            best = min(grp, key=lambda w: (
                sum(d for _r, _s, d in pairs[w]) / len(pairs[w]), w))
            if v != best:
                continue
        out.append(v)
    return out
