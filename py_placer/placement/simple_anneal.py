"""The simple path's optimiser: simulated annealing on the exact cost.

The cost is `simple.Model`'s, exactly as it is priced -- each net's Manhattan
spanning tree, crossings among those trees, claims, decaps, each group's gap
tree, the halo, foreign-pad hits and congestion -- plus, while it runs, an
OVERLAP penalty: the area where two parts' courtyards, each grown by half
the clearance, intersect on a face they share (a drilled part presents only
its lead boxes on its far face, as the seat gate pairs them), how far a
courtyard is outside the board less its edge margin -- the real outline,
cutouts and notches too -- and how far a part breaks the intent's keep-outs
and zones and the board's rule-area keep-outs. Its weight rises from
`LAM_START` to `LAM_END` as the run cools, so parts may pass through each
other early and a group can cross a crowded board; only an overlap-free
state can become the best one,
and the best one is finally checked by the exact pad legality
(`simple.legal`), a part failing it moved to the nearest legal pose. Asked
to (`repair`: place_simple's last run), a run that never came clean is first
mended where it stands (`Annealer.repair`): its overlapping parts shoved
apart, the movable parts in their way pushed aside in turn.

Moves, each priced by `Model.local` against the committed state near it:

    shift   one part by up to `reach` mm
    turn    one part to another quarter turn, in place (a part every quarter
            turn leaves the same, `turns_alike`, keeps its turn in every move)
    swap    two parts of one footprint exchange poses
    jump    one part to its wiring target, at a random turn
    dock    one part against a partner: for one of its nets, the nearest
            partner pad's part, one of its four sides, one of the four
            turns -- the courtyards at minimum clearance, the part slid
            along that side to line its pad up with the partner pad
    gshift  a group's movable members by the same step
    gturn   a group's movable members turned together about their centre
    gswap   two groups with no fixed member exchange places
    escape  a part that overlaps something, to the least-overlap pose of
            ESCAPE_TRIES drawn at random over the board

A shift, turn, jump or dock of a chip carries the caps that hold its decap
limits (`Model.tethered`: the cap at one of its supply pins, a cap it is the
nearest rail chip of) when moving the chip alone would pull them past their
limits, and of any part the parts whose claim on it the move would put further
past its limit (`Model.claim_tethered`: an IC's crystal, a crystal's load caps
and series resistor): each keeps its offset from the part and turns with it.
Priced at the limit, a cap is otherwise glue: on fusb302 a crystal that started
across the board from its MCU stayed there, since jumping to its pins would
have left its load cap 25 mm behind.

Group turns and swaps are drawn only in the first quarter of the run
(`BIG_UNTIL`): they move a whole group far at once, and on every board
measured none was kept after the first fifth, while each one costs its
group's size in single moves to price.

A move never leaves the board: the parts it moves are translated together
until their courtyards are inside the board less its edge margin. A part
pushed off the board behind a fixed one (a header on the edge) could
otherwise only come back over it, a jump the overlap weight refuses late in
the run. Only the kinds a board can make are drawn: no group moves on a
board without groups, no swap without two parts of one footprint.

A part seated on an edge (`edge_seat`: a declared edge connector) is the
exception: every move of it is projected onto its edge line, into the
nearest stretch of the edge it may use, so it only slides along the edge --
and turns, when it has no mating face. It may abut another seated part, and
its stretches already keep it clear of the fixed ones. When a long one (its
pads spread past the anneal's longest step) can lie either way round, the
first quarter of the run is made twice (`anneal`).
At the end the seated parts on each edge are pushed apart along it in their
order (`edge_seat.pack`) before the legality pass.

`reach` falls from a sixth of the board as (1 - progress)^2, to 0.1 mm. A
move is kept when it lowers the cost, or with probability exp(-delta / T);
T falls geometrically from where a move changing the cost by the median
amount is taken uphill with probability `ACCEPT`, to `T_END`. The run is a
fixed number of moves from a seeded random stream, so the same board and
seed give the same placement; `seconds` only caps a slow machine. It starts
from `scatter`: every part at a random place on the board, or, for a
clustered start, each group's members gathered round one random point.
"""
from __future__ import annotations

import math
import random
import time
from typing import Dict, List, Tuple

import numpy as np

from placement import simple
from placement.legality import BOARD_TOL, far_boxes
from placement.edge_seat import (SEAT_GAP_MM, legal_on_edge, pack, pad_span, project,
                                 reseat, rot_key)
from placement.simple import ROTS, _turn, follow, legal, nearest_legal, target

MOVES = {'shift': 0.40, 'turn': 0.15, 'swap': 0.10, 'jump': 0.10, 'dock': 0.10,
         'gshift': 0.12, 'gturn': 0.08, 'gswap': 0.05, 'escape': 0.05}
BIG_GROUP_MOVES = ('gturn', 'gswap')
CARRY = ('shift', 'turn', 'jump', 'dock')    # one part's moves that carry its tethered caps
BIG_UNTIL = 0.25    # the share of the run group turns and swaps are drawn in
ESCAPE_TRIES = 48   # random poses an escape move draws, keeping the least overlap
SHOVE_PARTS = 32    # the most parts one shove pushes
SHOVE_GAP = 1e-3    # mm a shove leaves between two grown courtyards
REPAIR_ROUNDS = 3   # passes of the repair over the parts a stuck run left overlapping
CLEAN = 1e-3        # mm2 of overlap that counts as none
FLOOR = 2 * CLEAN   # the least an outline or keep-out-band breach costs: never clean
ACCEPT = 0.1        # the start: the median cost change taken uphill this often
T_END = 0.02
LAM_START, LAM_END = 1.0, 10000.0


def _place(st, ref, x, y, rot):
    """`QuenchState.apply_move` without its last step: the pose, and the
    legality caches a move makes stale, but not the state's own airwires --
    nothing the annealer prices or tests reads them, and rebuilding them
    was a quarter of each move. `anneal` puts every part through the real
    `apply_move` once at the end."""
    part = st.parts[ref]
    part.x, part.y, part.rot = x, y, rot
    st._inc_violation.clear()
    st._inc_intent.clear()


def _half_turn(a, b) -> bool:
    """Whether turns `a` and `b` are half a turn apart."""
    return abs((a - b) % 360.0 - 180.0) < 1e-6


def turns_alike(part, fp) -> bool:
    """Whether every quarter turn leaves `part` the same to the placer: its
    pads (place, net, size, shape, drill) map onto each other turned about
    its origin, and every box it is priced by -- courtyard or body, the
    grade's, the far side, the pad copper, the drawn box -- is the same at
    each turn. A one-pin test point or fiducial with a round pad and a
    square courtyard: turning it changes nothing but its picture, so the
    anneal leaves its turn alone (`Annealer.alike`)."""
    if fp is None or not fp.pads or part.rot % 90:
        return False
    pads = []
    for p in fp.pads:
        sx, sy, shape = float(p.size_x), float(p.size_y), (p.shape or '').lower()
        round_ = shape in ('circle', 'rect', 'roundrect', 'oval') and abs(sx - sy) < 1e-6
        if not round_ or getattr(p, 'hole_x', None) is not None:
            return False
        pads.append((round(p.local_x, 4), round(p.local_y, 4), p.net_id, shape, round(sx, 4),
                     round(float(p.drill or 0.0), 4), tuple(sorted(p.layers or ()))))
    turned = sorted((round(-y, 4) + 0.0, round(x, 4) + 0.0, *rest) for x, y, *rest in pads)
    if turned != sorted((x + 0.0, y + 0.0, *rest) for x, y, *rest in pads):
        return False

    def boxes(rot):
        out = [part.rect(0.0, 0.0, rot), part.grade_rect(0.0, 0.0, rot),
               part.padbox(0.0, 0.0, rot), part.drawn_rect(0.0, 0.0, rot)]
        far = part.tht_rect(0.0, 0.0, rot)
        out += sorted(far_boxes(far)) if far is not None else []
        return [None if b is None else tuple(round(v, 4) for v in b) for b in out]
    first = boxes(part.rot)
    for q in (90.0, 180.0, 270.0):
        rot = (part.rot + q) % 360.0
        part.ensure_rotation(rot)
        if boxes(rot) != first:
            return False
    return True


class Annealer:
    def __init__(self, model, movable: List[str], *, seed: int = 0, seats=None):
        self.model = model
        self.st = model.state
        st = self.st
        # parts held on a board edge (`edge_seat`): a move only slides them
        # along it, or turns one with no mating face
        self.seats = dict(seats or {})
        # the long seated parts whose way round the anneal has chosen
        # (`anneal`'s two starts): their turns then stop at a quarter
        self.no_flip = frozenset()
        self.mov = sorted(r for r in movable if r in st.parts)
        self.movset = movset = set(self.mov)
        # the parts every quarter turn leaves the same (`turns_alike`): a
        # move keeps their turn, so the anneal never prices a turn that
        # changes nothing
        fps = st.pcb_data.footprints
        self.alike = frozenset(r for r in self.mov if turns_alike(st.parts[r], fps.get(r)))
        # what a seated part's pairs are priced differently against: the
        # parts the anneal does not move, and the other seated parts
        self.walls = sorted((set(st.parts) - movset) | set(self.seats))
        self.rng = random.Random(seed)
        self.groups = [(n, [r for r in m if r in movset]) for n, m in model.groups]
        self.groups = [(n, m) for n, m in self.groups if len(m) >= 2]
        whole = dict(model.groups)
        self.free_groups = [(n, m) for n, m in self.groups
                            if all(r in movset for r in whole[n])]
        # twins: two parts of one footprint, seated on one edge or neither (a
        # seated part's pose is on its own edge, a free part's on none)
        def twin_key(r):
            return (st.parts[r].footprint_name, self.seats[r].edge if r in self.seats else None)
        by_fp: Dict[tuple, List[str]] = {}
        for r in self.mov:
            by_fp.setdefault(twin_key(r), []).append(r)
        self.twins = {r: [o for o in by_fp[twin_key(r)] if o != r] for r in self.mov}
        # the board less the margin the legality gate holds a courtyard to
        # (`QuenchState.usable`: the larger of the clearance and the edge
        # clearance), so a pose inside it is never refused for the edge
        self.inner = tuple(st.usable)
        # ... and the two board terms the bbox cannot hold: the real outline
        # (a cutout, a notch: `edge_gate`, None when the bbox is exact) and
        # the rule-area keep-out bands (`legality_ctx.keepouts`, #1031)
        self.gate = st.edge_gate if st.edge_gate.active else None
        ctx = st.legality_ctx
        self.bands = (ctx if ctx is not None and ctx.keepouts is not None
                      and ctx.keepouts.active else None)
        self.span = max(self.inner[2] - self.inner[0], self.inner[3] - self.inner[1])
        self.half = st.clearance / 2.0
        self.names = sorted(st.parts)
        self.everyone = frozenset(st.parts)
        # each part's faces as a bit mask: two parts can overlap when theirs
        # meet. A container (a pin frame, an outline: `QuenchState.
        # container_refs`) is a frame, not a body, and has none, as the halo
        # and the legality skip it; a pin frame's drilled pins are priced
        # instead (`_pins`, #1307).
        self.containers = frozenset(r for r in self.names if r in st.container_refs)
        self.frames = sorted(r for r in self.containers if r in st.pin_frame_refs)
        bit = {}
        self.faces = {r: 0 if r in self.containers else
                      sum(1 << bit.setdefault(f, len(bit)) for f in set(st.parts[r].sides))
                      for r in self.names}
        self.face_ids = np.asarray([self.faces[r] for r in self.names], dtype=np.int64)
        # The pairs on opposite faces with a drilled part among them, which
        # the seat gate prices by what each part presents on the face they
        # share -- a drilled part's far face is its lead boxes alone
        # (`legality.pair_overlap_area`) -- and the Rust sum by the whole
        # courtyards: `_far_correction` reprices them as the gate does. A
        # part can be in one only when the other face holds a part it can
        # pair with there (a drilled part if it is not drilled itself).
        P = st.parts
        bodies = [r for r in self.names if r not in self.containers]
        self.drilled = frozenset(r for r in bodies if P[r].tht_by_rot is not None)
        drilled_on = {P[r].side for r in self.drilled}
        on = {P[r].side for r in bodies}
        self.far_parts = frozenset(
            r for r in bodies
            if (on if r in self.drilled else drilled_on) - {P[r].side})
        # the drilled parts whose lead boxes reach past their courtyard (a
        # connector's pins beyond the courtyard drawn round its body)
        self.far_wide = frozenset(
            r for r in self.drilled
            if any(t[0] < c[0] or t[1] < c[1] or t[2] > c[2] or t[3] > c[3]
                   for c, f in [P[r].rects()] for t in far_boxes(f)))
        self.lam = LAM_START
        self.shoving = False        # an escape may shove: only in `repair`

    # -- poses -----------------------------------------------------------------
    def pose(self, r):
        p = self.st.parts[r]
        return (p.x, p.y, p.rot)

    def _centre(self, refs):
        rs = [self.st.parts[r].rect() for r in refs]
        return ((min(q[0] for q in rs) + max(q[2] for q in rs)) / 2.0,
                (min(q[1] for q in rs) + max(q[3] for q in rs)) / 2.0)

    # -- overlap ---------------------------------------------------------------
    def _grown(self, r):
        c, h = self.st.parts[r].rect(), self.half
        return (c[0] - h, c[1] - h, c[2] + h, c[3] + h)

    def _seat_pair(self, a, b):
        """For a pair with a seated part: None when it is not this term's to
        price -- the seat's stretches already keep a seated part legal against
        every fixed one -- else how much less than the clearance it keeps: two
        seated parts may abut (a declared pose's check,
        `edge_seat.legal_on_edge`; they keep SEAT_GAP_MM), a seated and a
        free part keep the clearance. 0.0 for any other pair."""
        sa, sb = a in self.seats, b in self.seats
        if not (sa or sb):
            return 0.0
        if (sa and b not in self.movset) or (sb and a not in self.movset):
            return None
        return 2.0 * self.half - SEAT_GAP_MM if sa and sb else 0.0

    def _seat_correction(self, mine):
        """What `_seat_pair` changes in a move's overlap, which the Rust sum
        prices at the plain clearance: each seated mover's pairs with the
        fixed parts (not priced) and with the other seated parts (SEAT_GAP_MM
        apart), each pair once."""
        mset = set(mine)
        fix = 0.0
        for a in mine:
            if a not in self.seats:
                continue
            A = self._grown(a)
            for b in self.walls:
                if b == a or (b in mset and b < a and b in self.seats) \
                        or not self.faces[a] & self.faces[b]:
                    continue
                B = self._grown(b)
                ox = min(A[2], B[2]) - max(A[0], B[0])
                oy = min(A[3], B[3]) - max(A[1], B[1])
                if ox <= 0 or oy <= 0:
                    continue
                fix -= ox * oy      # out: what the plain clearance priced
                less = self._seat_pair(a, b)
                if less is not None:
                    ox, oy = ox - less, oy - less
                    if ox > 0 and oy > 0:
                        fix += ox * oy
        return fix

    def _cross(self, a, b):
        """Whether `a` and `b` are on opposite faces with a drilled part
        among them: the pairs `_far_area` prices."""
        pa, pb = self.st.parts[a], self.st.parts[b]
        return pa.side != pb.side and (pa.tht_by_rot is not None
                                       or pb.tht_by_rot is not None)

    def _face_boxes(self, r, face, pose=None):
        """What `r` presents on `face`, each box grown by half the
        clearance: its courtyard on its own face, its lead boxes on the far
        one (`legality.rect_on`; none when it is not drilled). `pose`, else
        where it is."""
        p, h = self.st.parts[r], self.half
        if face == p.side:
            bs = (p.rect() if pose is None else p.rect(*pose),)
        else:
            bs = far_boxes(p.rects()[1] if pose is None else p.tht_rect(*pose))
        return [(b[0] - h, b[1] - h, b[2] + h, b[3] + h) for b in bs]

    def _far_area(self, a, b, pa=None, pb=None):
        """A `_cross` pair's overlap as the seat gate prices it: on each
        face the two share, the area where what each presents there
        overlaps, summed box by box, and the larger face's
        (`legality.pair_overlap_area`: two drilled parts show one
        interference on both faces). `pa`, `pb`: their poses, else where
        they are."""
        P = self.st.parts
        worst = 0.0
        for f in sorted(P[a].sides & P[b].sides):
            v = 0.0
            for A in self._face_boxes(a, f, pa):
                for B in self._face_boxes(b, f, pb):
                    ox = min(A[2], B[2]) - max(A[0], B[0])
                    if ox > 0:
                        oy = min(A[3], B[3]) - max(A[1], B[1])
                        if oy > 0:
                            v += ox * oy
            worst = max(worst, v)
        return worst

    def _reach(self, r):
        """`r`'s grown courtyard, widened to its grown lead boxes where they
        reach past it (`far_wide`): all a `_cross` pair can overlap in."""
        A = self._grown(r)
        if r not in self.far_wide:
            return A
        t, h = self.st.parts[r].rects()[1], self.half
        return (min(A[0], t[0] - h), min(A[1], t[1] - h),
                max(A[2], t[2] + h), max(A[3], t[3] + h))

    def _far_correction(self, mine):
        """What `_far_area` changes in a move's overlap, which the Rust sum
        prices on the whole courtyards: each `_cross` pair with a moving
        part, each pair once (two moving parts from the drilled one, the
        first by name when both are). A cap under a through-hole chip's
        body, clear of its leads, is legal to the gate; priced on the
        courtyards it was pushed out from under the chip. A seated part's
        pairs that `_seat_correction` reprices are left to it."""
        movers = [a for a in mine if a in self.far_parts]
        if not movers:
            return 0.0
        P, h = self.st.parts, self.half
        mset = set(mine)
        wide = self.far_wide - mset
        fix = 0.0
        for a in movers:
            RA = self._reach(a)
            # the index holds courtyards: a part whose leads reach past its
            # own is looked at wherever it is
            near = (self.model._ixp.query((RA[0] - h, RA[1] - h, RA[2] + h, RA[3] + h))
                    - mset) | wide
            if P[a].tht_by_rot is None:
                cand = sorted(near & self.drilled)
            else:
                cand = sorted(near) + [b for b in movers if b != a and (
                    P[b].tht_by_rot is None or b > a)]
            A = self._grown(a)
            for b in cand:
                if not self._cross(a, b) or (self.seats and self._seat_pair(a, b) != 0.0):
                    continue
                RB = self._reach(b)
                if not (min(RA[2], RB[2]) > max(RA[0], RB[0])
                        and min(RA[3], RB[3]) > max(RA[1], RB[1])):
                    continue
                B = self._grown(b)
                ox = min(A[2], B[2]) - max(A[0], B[0])
                oy = min(A[3], B[3]) - max(A[1], B[1])
                fix += self._far_area(a, b) - (ox * oy if ox > 0 and oy > 0 else 0.0)
        return fix

    def _pins(self, mine):
        """A pin frame's pins (mm2): courtyard over its drilled holes, the
        holes check_assembly's pin_in_courtyard tests (`QuenchState.
        pin_hole_boxes`), on either face; and pad copper over its pads' copper
        grown by the clearance (`QuenchState.pin_copper_boxes`), on a face
        both have copper on, the pad clearance the legality pass after the
        anneal enforces. Each (frame, part) pair with a part in `mine` once --
        a frame in `mine` with every part, any other frame with the parts in
        `mine`."""
        P, tot, mset = self.st.parts, 0.0, set(mine)
        for f in self.frames:
            holes = self.st.pin_hole_boxes(f)
            pads = self.st.pin_copper_boxes(f)
            if not holes and not pads:
                continue
            every = list(holes) + [q[:4] for q in pads]
            H = (min(h[0] for h in every), min(h[1] for h in every),
                 max(h[2] for h in every), max(h[3] for h in every))
            for b in (self.names if f in mset else mine):
                if b in self.containers:
                    continue
                pb = P[b]
                c = pb.rect()
                if c[2] <= H[0] or H[2] <= c[0] or c[3] <= H[1] or H[3] <= c[1]:
                    continue
                for h in holes:
                    ox = min(c[2], h[2]) - max(c[0], h[0])
                    oy = min(c[3], h[3]) - max(c[1], h[1])
                    if ox > 0 and oy > 0:
                        tot += ox * oy
                k = pb.padbox()
                if k is None:
                    continue
                for q in pads:
                    if not (q[4] & pb.sides):
                        continue
                    ox = min(k[2], q[2]) - max(k[0], q[0])
                    oy = min(k[3], q[3]) - max(k[1], q[1])
                    if ox > 0 and oy > 0:
                        tot += ox * oy
        return tot

    def _outside(self, r):
        """How far `r`'s courtyard is off the board, as an area: its overhang
        past each side times its size along that side -- the area off while
        it straddles one side, and growing the further off it is, so a part
        wholly off the board is still pulled back.

        Where the outline is not its bbox, plus the real outline's own
        measure (`rect_outside_amount`, what `seeder.pose_ok` refuses on)
        times the courtyard's half-perimeter, and never less than `FLOOR`:
        a pose the gate refuses is never clean. Only a courtyard near a
        cutout, a notch or a rounded corner pays for it; the rest are
        answered by the gate's grid (`rect_bbox_decides`)."""
        if r in self.seats:
            return 0.0      # on its edge by design, its overhang declared
        c = self.st.parts[r].rect()
        i = self.inner
        w, hh = c[2] - c[0], c[3] - c[1]
        v = (hh * (max(0.0, i[0] - c[0]) + max(0.0, c[2] - i[2]))
             + w * (max(0.0, i[1] - c[1]) + max(0.0, c[3] - i[3])))
        g = self.gate
        if g is not None and not g.rect_bbox_decides(c):
            amt = g.rect_outside_amount(c)
            if amt > BOARD_TOL:
                v += max(FLOOR, amt * (w + hh))
        return v

    def _bands(self, r):
        """How far `r`'s pads reach into the board's rule-area keep-out
        bands past what the gate admits (`keepout_excess`, #1031), times its
        courtyard's half-perimeter, and never less than `FLOOR`; 0 on a board
        without such a keep-out, or for a part whose copper cannot reach one."""
        if self.bands is None:
            return 0.0
        p = self.st.parts[r]
        ex = self.bands.keepout_excess(r, p.x, p.y, p.rot)
        if ex <= 0.0:
            return 0.0
        c = p.rect()
        return max(FLOOR, ex * ((c[2] - c[0]) + (c[3] - c[1])))

    def _intent(self, r):
        """How far `r` breaks the intent's claims on it -- keep-outs and
        exclusive zones (mm2 inside), containment zones (mm outside) --
        each term past its own threshold."""
        spec = self.st.intent_spec_for(r)
        if not spec:
            return 0.0
        p = self.st.parts[r]
        vals = self.st.intent_terms(r, p.grade_rects(p.x, p.y, p.rot))
        return sum(max(0.0, v - t.threshold) for v, t in zip(vals, spec))

    def overlap(self, refs=None) -> float:
        """Overlap (mm2) of every pair with a part in `refs` (all pairs when
        None), each pair once -- a pin frame by its pins (`_pins`), not its
        body -- plus those parts' distance off the board and
        into what the intent and the board's rule areas keep them out of. With
        `refs`, the pairs are summed in Rust (`grid_router.overlap_sum`): the
        moving parts at their live poses, every other part at its committed
        rect (`Model._CR`)."""
        mine = self.mov if refs is None else sorted(refs)
        total = sum(self._outside(r) + self._intent(r) + self._bands(r) for r in mine)
        h = self.half
        if refs is not None:
            P, pid = self.st.parts, self.model._pid
            total = simple.grid_router.overlap_sum(
                total, h, [pid(r) for r in mine],
                np.asarray([P[r].rect() for r in mine], dtype=float).reshape(-1, 4),
                self.model._CR, self.face_ids)
            if self.seats:
                total += self._seat_correction(mine)
            if self.far_parts:
                total += self._far_correction(mine)
            if self.frames:
                total += self._pins(mine)
            return total
        if self.frames:
            total += self._pins(mine)
        mset = set(mine)
        grown = {}
        for a in mine:
            A = grown.get(a) or grown.setdefault(a, self._grown(a))
            for b in self.names:
                if b == a or (b in mset and b < a) or not self.faces[a] & self.faces[b]:
                    continue
                less = self._seat_pair(a, b) if self.seats else 0.0
                if less is None:
                    continue
                if (not less and a in self.far_parts and b in self.far_parts
                        and self._cross(a, b)):
                    total += self._far_area(a, b)
                    continue
                B = grown.get(b) or grown.setdefault(b, self._grown(b))
                ox = min(A[2], B[2]) - max(A[0], B[0])
                if less:
                    ox -= less
                if ox <= 0:
                    continue
                oy = min(A[3], B[3]) - max(A[1], B[1])
                if less:
                    oy -= less
                if oy > 0:
                    total += ox * oy
        return total

    # -- proposals: {ref: new pose} or None --------------------------------------
    def kinds(self) -> List[str]:
        """The move kinds this board can make."""
        out = [k for k in MOVES if k not in ('swap', 'gshift', 'gturn', 'gswap')]
        if any(self.twins.values()):
            out.append('swap')
        if self.groups:
            out += ['gshift', 'gturn']
        if len(self.free_groups) >= 2:
            out.append('gswap')
        return [k for k in MOVES if k in out]

    def propose(self, kind, reach):
        """A move of `kind`, its parts translated together onto the board,
        or None."""
        new = self._move(kind, reach)
        if not new:
            return new
        if kind in CARRY:
            new = self._carry(new)
        out = {r: v for r, v in new.items() if r not in self.seats}
        rs = []
        for r, (x, y, rot) in out.items():
            p = self.st.parts[r]
            p.ensure_rotation(rot)
            rs.append(p.rect(x, y, rot))
        if rs:
            u = (min(q[0] for q in rs), min(q[1] for q in rs),
                 max(q[2] for q in rs), max(q[3] for q in rs))
            i = self.inner
            if u[2] - u[0] > i[2] - i[0] or u[3] - u[1] > i[3] - i[1]:
                return None
            dx = max(0.0, i[0] - u[0]) - max(0.0, u[2] - i[2])
            dy = max(0.0, i[1] - u[1]) - max(0.0, u[3] - i[3])
            if dx or dy:
                out = {r: (x + dx, y + dy, rot) for r, (x, y, rot) in out.items()}
        for r, pose in new.items():     # an edge part slides along its edge
            if r in self.seats:
                now = self.st.parts[r].rot
                if r in self.no_flip and _half_turn(pose[2], now):
                    pose = (pose[0], pose[1], now)
                out[r] = project(self.seats[r], pose, now)
        for r in self.alike.intersection(out):      # its turn changes nothing
            out[r] = (out[r][0], out[r][1], self.st.parts[r].rot)
        return out

    def _carry(self, new):
        """One part's move `new` with the caps that hold its decap limits
        (`Model.tethered`), the parts that hold their claim limits at it
        (`Model.claim_tethered`: a crystal at its IC, a load cap at its
        crystal) and the members of the datasheet template it anchors
        (`Model.template_members`, always) moved as one with it: each at its
        offset from the part, turned with it (`simple.follow`)."""
        (r, pose), = new.items()
        sats = [c for c in dict.fromkeys(self.model.tethered(r, pose)
                                          + self.model.claim_tethered(r, pose)
                                          + self.model.template_members(r))
                if c in self.movset and c != r]
        if not sats:
            return new
        was = self.pose(r)
        out = dict(new)
        for c in sats:
            out[c] = follow(was, pose, self.pose(c))
        return out

    def _move(self, kind, reach):
        rng, st = self.rng, self.st
        if not self.mov:
            return None
        if kind == 'shift':
            r = rng.choice(self.mov)
            x, y, rot = self.pose(r)
            return {r: (x + rng.uniform(-reach, reach), y + rng.uniform(-reach, reach), rot)}
        if kind == 'turn':
            r = rng.choice(self.mov)
            x, y, rot = self.pose(r)
            turns = self.seats[r].lines if r in self.seats else ROTS
            turns = [q for q in turns if abs(q - rot % 360.0) > 1e-6
                     and not (r in self.no_flip and _half_turn(q, rot))]
            return {r: (x, y, rng.choice(turns))} if turns else None
        if kind == 'swap':
            r = rng.choice(self.mov)
            if not self.twins[r]:
                return None
            o = rng.choice(self.twins[r])
            return {r: self.pose(o), o: self.pose(r)}
        if kind == 'jump':
            r = rng.choice(self.mov)
            home = self.model.template_pose(r)
            if home is not None:        # a template member: back to its pose
                return {r: home}
            tx, ty = target(self.model, r, self.everyone)
            p = st.parts[r]
            rot = rng.choice(ROTS)
            p.ensure_rotation(rot)
            c = p.rect(0.0, 0.0, rot)
            return {r: (tx - (c[0] + c[2]) / 2.0, ty - (c[1] + c[3]) / 2.0, rot)}
        if kind == 'dock':
            return self._dock(rng.choice(self.mov))
        if kind == 'escape':
            return self._escape(rng.choice(self.mov))
        if kind == 'gshift':
            if not self.groups:
                return None
            _n, m = rng.choice(self.groups)
            dx, dy = rng.uniform(-reach, reach), rng.uniform(-reach, reach)
            return {r: (self.pose(r)[0] + dx, self.pose(r)[1] + dy, self.pose(r)[2]) for r in m}
        if kind == 'gturn':
            if not self.groups:
                return None
            _n, m = rng.choice(self.groups)
            q = rng.choice((90.0, 180.0, 270.0))
            cx, cy = self._centre(m)
            out = {}
            for r in m:
                x, y, rot = self.pose(r)
                tx, ty = _turn(x - cx, y - cy, q)
                out[r] = (cx + tx, cy + ty, (rot + q) % 360.0)
            return out
        if kind == 'gswap':
            if len(self.free_groups) < 2:
                return None
            (_a, ma), (_b, mb) = rng.sample(self.free_groups, 2)
            ca, cb = self._centre(ma), self._centre(mb)
            out = {}
            for m, (fx, fy), (tx, ty) in ((ma, ca, cb), (mb, cb, ca)):
                for r in m:
                    x, y, rot = self.pose(r)
                    out[r] = (x + tx - fx, y + ty - fy, rot)
            return out
        return None

    def _grown_at(self, r, pose):
        c, h = self.st.parts[r].rect(*pose), self.half
        return (c[0] - h, c[1] - h, c[2] + h, c[3] + h)

    def _hits(self, r, G, poses, at=None):
        """[(part, its grown courtyard)]: the parts whose grown courtyards
        overlap `G` (r's, grown, at pose `at`, else where r is) on a face r
        has, each at its pose in `poses`, else where it is -- a `_cross`
        pair only when what the two present on a shared face overlaps
        (`_far_area`)."""
        h = self.half
        out = []
        for b in sorted(self.model._ixp.query((G[0] - h, G[1] - h, G[2] + h, G[3] + h))
                        | set(poses)):
            if b == r or not self.faces[r] & self.faces[b]:
                continue
            B = self._grown_at(b, poses[b]) if b in poses else self._grown(b)
            if min(G[2], B[2]) - max(G[0], B[0]) > 0 and min(G[3], B[3]) - max(G[1], B[1]) > 0:
                if (r in self.far_parts and b in self.far_parts and self._cross(r, b)
                        and self._far_area(r, b, at, poses.get(b)) <= 0.0):
                    continue
                out.append((b, B))
        return out

    def _onto(self, r, pose):
        """`pose` translated until r's courtyard is on the board (less its
        margin)."""
        c, i = self.st.parts[r].rect(*pose), self.inner
        dx = max(0.0, i[0] - c[0]) - max(0.0, c[2] - i[2])
        dy = max(0.0, i[1] - c[1]) - max(0.0, c[3] - i[3])
        return (pose[0] + dx, pose[1] + dy, pose[2])

    def _shove(self, r, dx, dy):
        """{part: pose}: `r` translated by (dx, dy), and each movable part it
        then overlaps pushed out of its way by the shortest straight step
        that clears it on the board -- one clear of the fixed parts and of
        `r` first -- then the parts those run into, and so on, a part pushed
        again from where it was pushed to: at most SHOVE_PARTS parts, else
        None. `r` and the fixed parts are never pushed (their overlaps stay,
        priced)."""
        x, y, rot = self.pose(r)
        new = {r: self._onto(r, (x + dx, y + dy, rot))}
        queue = [r]
        pushes = 0
        e = SHOVE_GAP
        while queue:
            a = queue.pop(0)
            A = self._grown_at(a, new[a])
            for b, B in self._hits(a, A, new, new[a]):
                if b == r or b not in self.movset:
                    continue
                x, y, rot = new.get(b) or self.pose(b)
                best = None
                for sx, sy in ((A[2] - B[0] + e, 0.0), (A[0] - B[2] - e, 0.0),
                               (0.0, A[3] - B[1] + e), (0.0, A[1] - B[3] - e)):
                    pose = self._onto(b, (x + sx, y + sy, rot))
                    G = self._grown_at(b, pose)
                    left = (max(0.0, min(A[2], G[2]) - max(A[0], G[0]))
                            * max(0.0, min(A[3], G[3]) - max(A[1], G[1])))
                    fixed = sum((min(G[2], F[2]) - max(G[0], F[0]))
                                * (min(G[3], F[3]) - max(G[1], F[1]))
                                for f, F in self._hits(b, G, new, pose)
                                if f == r or f not in self.movset)
                    key = (left > CLEAN, fixed > CLEAN, abs(sx) + abs(sy))
                    if best is None or key < best[0]:
                        best = (key, pose)
                new[b] = best[1]
                queue.append(b)
                pushes += 1
                if len(new) > SHOVE_PARTS or pushes > 4 * SHOVE_PARTS:
                    return None
        return new

    def _shove_best(self, r):
        """(overlap change, {part: pose}): of the four straight steps that
        clear each part (or cutout) `r` overlaps, the shove (`_shove`) that
        lowers the overlap and is the cheapest as the trial prices it (the
        cost change plus the overlap weight times the overlap change), or
        None when none lowers it."""
        st = self.st
        A = self._grown(r)
        obstacles = [B for _b, B in self._hits(r, A, {})]
        if self.gate is not None:       # a cutout's box, as near as the gate lets r come
            m = self.gate.margin + self.half
            for c in self.gate.cutouts:
                x0, y0 = min(p[0] for p in c) - m, min(p[1] for p in c) - m
                x1, y1 = max(p[0] for p in c) + m, max(p[1] for p in c) + m
                if x0 < A[2] and A[0] < x1 and y0 < A[3] and A[1] < y1:
                    obstacles.append((x0, y0, x1, y1))
        e = SHOVE_GAP
        steps = set()
        for B in obstacles:
            steps |= {(B[0] - A[2] - e, 0.0), (B[2] - A[0] + e, 0.0),
                      (0.0, B[1] - A[3] - e), (0.0, B[3] - A[1] + e)}
        best = None
        for dx, dy in sorted(steps):
            new = self._shove(r, dx, dy)
            if not new:
                continue
            refs = sorted(new)
            was = {q: self.pose(q) for q in refs}
            before = self.overlap(refs)
            for q in refs:
                _place(st, q, *new[q])
            gain = self.overlap(refs) - before
            for q in refs:
                _place(st, q, *was[q])
            if gain >= -CLEAN:
                continue
            delta = self.price(new) + self.lam * gain
            if best is None or delta < best[0]:
                best = (delta, gain, new)
        return (best[1], best[2]) if best else None

    def _board_pose(self, r):
        """(overlap, pose): the least-overlap pose of `r` among ESCAPE_TRIES
        drawn at random over the board, at random turns."""
        st, rng, p = self.st, self.rng, self.st.parts[r]
        was = self.pose(r)
        i = self.inner
        best = None
        for _ in range(ESCAPE_TRIES):
            rot = rng.choice(ROTS)
            p.ensure_rotation(rot)
            c = p.rect(0.0, 0.0, rot)
            w, h = c[2] - c[0], c[3] - c[1]
            cx = rng.uniform(i[0] + w / 2.0, max(i[0] + w / 2.0, i[2] - w / 2.0))
            cy = rng.uniform(i[1] + h / 2.0, max(i[1] + h / 2.0, i[3] - h / 2.0))
            pose = (cx - (c[0] + c[2]) / 2.0, cy - (c[1] + c[3]) / 2.0, rot)
            if r in self.seats:     # a random spot on its own edge
                pose = project(self.seats[r], pose, was[2])
                p.ensure_rotation(pose[2])
            _place(st, r, *pose)
            ov = self.overlap([r])
            if best is None or ov < best[0]:
                best = (ov, pose)
        p.ensure_rotation(was[2])
        _place(st, r, *was)
        return best

    def _escape(self, r):
        """`r` out of an overlap small steps cannot leave: when it overlaps
        anything, the pose of least overlap among ESCAPE_TRIES drawn at
        random over the board -- a small part finds a hole somewhere -- or,
        in `repair`, when none of those is clear, the SHOVE that lowers the
        overlap more (`_shove_best`)."""
        before = self.overlap([r])
        if before <= CLEAN:
            return None
        ov, pose = self._board_pose(r)
        best = (ov - before, {r: pose})
        if self.shoving and ov > CLEAN:
            got = self._shove_best(r)
            if got and got[0] < best[0]:
                best = got
        return best[1]

    def _dock(self, r):
        """`r` against the part of the nearest partner pad on one of its nets."""
        st, rng = self.st, self.rng
        part = st.parts[r]
        nets = [n for n in sorted(set(part.nets)) if n in self.model.signal]
        if not nets:
            return None
        n = rng.choice(nets)
        mine = [(x, y) for x, y, nn in part.pad_globals() if nn == n]
        partners = [(q, x, y) for q in sorted(st.net_refs.get(n, ())) if q != r
                    for x, y, nn in st.parts[q].pad_globals() if nn == n]
        if not mine or not partners:
            return None
        mx, my = mine[0]
        q, qx, qy = min(partners, key=lambda t: abs(t[1] - mx) + abs(t[2] - my))
        Q = st.parts[q].rect()
        rot = rng.choice(ROTS)
        part.ensure_rotation(rot)
        c = part.rect(0.0, 0.0, rot)
        j = next(i for i, (_x, _y, nn) in enumerate(part.pad_globals(0.0, 0.0, rot)) if nn == n)
        px, py, _ = part.pad_globals(0.0, 0.0, rot)[j]
        gap = st.clearance + 1e-3
        side = rng.randrange(4)
        if side == 0:       # to the right of the partner
            x, y = Q[2] + gap - c[0], qy - py
        elif side == 1:     # left
            x, y = Q[0] - gap - c[2], qy - py
        elif side == 2:     # below
            x, y = qx - px, Q[3] + gap - c[1]
        else:               # above
            x, y = qx - px, Q[1] - gap - c[3]
        return {r: (x, y, rot)}

    # -- one trial -------------------------------------------------------------
    def trial(self, new: Dict[str, Tuple], T: float) -> Tuple[bool, float, float]:
        """Price `new` (the cost plus lam x overlap) and keep it by the
        Metropolis rule, committing it to the model if kept. Returns (kept,
        cost delta, overlap delta)."""
        st, model = self.st, self.model
        refs = sorted(new)
        for r, (_x, _y, rot) in new.items():
            st.parts[r].ensure_rotation(rot)
        t = model.touched(refs)
        base, ob = model.local(refs, t, reprice=False), self.overlap(refs)
        was = {r: self.pose(r) for r in refs}
        for r in refs:
            _place(st, r, *new[r])
        do = self.overlap(refs) - ob
        # the moved parts' cost after the move is never below 0, so the move
        # changes the cost by at least lam x do - base: when that is already
        # uphill, the rule's one draw is made now, and a draw that refuses
        # even that change refuses the move unpriced -- the same draw, the
        # same answer as pricing it would give
        u, lb = None, self.lam * do - base
        if lb > 0 and T > 0:
            u = self.rng.random()
            if u >= math.exp(-lb / T):
                for r in refs:
                    _place(st, r, *was[r])
                return False, lb, do
        snap = model.snapshot(t, refs)
        dc = model.local(refs, t) - base
        delta = dc + self.lam * do
        if delta <= 0 or (T > 0 and (self.rng.random() if u is None else u) < math.exp(-delta / T)):
            model.commit(refs, t[0])
            return True, dc, do
        for r in refs:
            _place(st, r, *was[r])
        model.restore(snap)
        return False, dc, do

    def price(self, new: Dict[str, Tuple]) -> float:
        """The cost change `new` would make, the state left as it was."""
        st, model = self.st, self.model
        refs = sorted(new)
        for r, (_x, _y, rot) in new.items():
            st.parts[r].ensure_rotation(rot)
        t = model.touched(refs)
        base = model.local(refs, t, reprice=False)
        snap = model.snapshot(t, refs)
        was = {r: self.pose(r) for r in refs}
        for r in refs:
            _place(st, r, *new[r])
        dc = model.local(refs, t) - base
        for r in refs:
            _place(st, r, *was[r])
        model.restore(snap)
        return dc

    def start_temperature(self, samples: int = 200) -> float:
        """T at which a move changing the cost by the median amount, either
        way, is taken with probability `ACCEPT` uphill. The overlap is left
        out: from a scatter most moves lower it."""
        ds = []
        for _ in range(samples):
            r = self.rng.choice(self.mov)
            x, y, rot = self.pose(r)
            reach = self.span / 6.0
            pose = (x + self.rng.uniform(-reach, reach), y + self.rng.uniform(-reach, reach), rot)
            if r in self.seats:
                pose = project(self.seats[r], pose, rot)
            dc = self.price({r: pose})
            if abs(dc) > 1e-9:
                ds.append(abs(dc))
        if not ds:
            return 1.0
        ds.sort()
        return ds[len(ds) // 2] / -math.log(ACCEPT)

    def repair(self, T: float) -> float:
        """A run that never came clean, mended where it stands: up to
        REPAIR_ROUNDS passes over the parts still overlapping, each one's
        escape -- with the SHOVE -- kept by the trial at the run's last
        temperature and overlap weight. A big part wedged among its
        neighbours (z80_devboard's Z80, 0.88 mm into a locked switch, RAMs
        and glue DIPs on its other sides) needs them to make room together,
        which no move of one part makes, and no hole elsewhere is big
        enough for it; one shove of four parts clears it.

        The LAST resort, not instead of a fresh start: a run that got stuck
        is a worse arrangement than a fresh run that did not (measured on
        the seven stuck runs of noahfc, tigard and z80_devboard: 5-30% more
        cost when mended than the next fresh run, and noahfc's three routed
        worse), so it only saves a run that would otherwise leave a part
        illegal. Returns the overlap left."""
        self.shoving = True
        try:
            for _ in range(REPAIR_ROUNDS):
                stuck = [r for r in self.mov if self.overlap([r]) > CLEAN]
                if not stuck:
                    break
                for r in stuck:
                    new = self._escape(r)
                    if new:
                        self.trial(new, T)
            return self.overlap()
        finally:
            self.shoving = False


#: The anchored start: each pull's weight is scaled by exp(N(0, SPREAD)) per
#: seed, and each solved centre moved by N(0, JITTER x the board's short side).
ANCHOR_SPREAD = 0.5
ANCHOR_JITTER = 0.15


def _anchors(model, movable: List[str], rng, inner) -> Tuple[List[List[str]], np.ndarray]:
    """`(nodes, centres)`: each group's movable members (and each movable part
    in no group, alone) as one node, and the node's centre that minimises the
    squared wire to what it is wired to -- the fixed parts' pads on its signal
    nets, the other nodes on them (each net a clique, 1/(k-1) per pair) -- and
    its `near` hints to other groups and `edge` hints to its edge. Every pull
    is scaled by a random factor of this seed's, so seeds solve different
    arrangements near good ones; a faint pull to the board's centre keeps a
    node wired to nothing in place."""
    st = model.state
    mov = set(movable)
    nodes: List[List[str]] = []
    of: Dict[str, int] = {}
    for _n, members in model.groups:
        m = [r for r in members if r in mov and r not in of]
        if m:
            for r in m:
                of[r] = len(nodes)
            nodes.append(m)
    for r in movable:
        if r not in of:
            of[r] = len(nodes)
            nodes.append([r])
    k = len(nodes)
    L = np.zeros((k, k))
    dx, dy = np.zeros(k), np.zeros(k)
    bx, by = np.zeros(k), np.zeros(k)
    cx, cy = (inner[0] + inner[2]) / 2.0, (inner[1] + inner[3]) / 2.0

    def jit():
        return math.exp(rng.gauss(0.0, ANCHOR_SPREAD))

    def link(i, j, w):
        L[i, j] -= w
        L[j, i] -= w
        L[i, i] += w
        L[j, j] += w

    def pin(i, x, y, w):
        if x is not None:
            dx[i] += w
            bx[i] += w * x
        if y is not None:
            dy[i] += w
            by[i] += w * y
    for n in model.nets:
        ends, fixed = set(), []
        for r in st.net_refs.get(n, ()):
            if r in of:
                ends.add(of[r])
            elif r in st.parts:
                pts = [(x, y) for x, y, nn in st.parts[r].pad_globals() if nn == n]
                if pts:
                    fixed.append((sum(p[0] for p in pts) / len(pts),
                                  sum(p[1] for p in pts) / len(pts)))
        size = len(ends) + len(fixed)
        if not ends or size < 2:
            continue
        w0 = 1.0 / (size - 1)
        ends = sorted(ends)
        for a, i in enumerate(ends):
            for j in ends[a + 1:]:
                link(i, j, w0 * jit())
            for x, y in fixed:
                pin(i, x, y, w0 * jit())
    for h in model.hints:
        ga = {of[r] for r in h.get('a') or () if r in of}
        if h['kind'] == 'near':
            gb = {of[r] for r in h.get('b') or () if r in of}
            for i in ga:
                for j in gb - {i}:
                    link(i, j, h['weight'] * jit())
        elif h['kind'] == 'edge':
            x = {'W': inner[0], 'E': inner[2]}.get(h['edge'])
            y = {'N': inner[1], 'S': inner[3]}.get(h['edge'])
            for i in ga:
                pin(i, x, y, h['weight'] * jit())
    for i in range(k):
        pin(i, cx, cy, 1e-3)
    xs = np.linalg.solve(L + np.diag(dx), bx)
    ys = np.linalg.solve(L + np.diag(dy), by)
    sd = ANCHOR_JITTER * min(inner[2] - inner[0], inner[3] - inner[1])
    centres = np.array([[x + rng.gauss(0.0, sd), y + rng.gauss(0.0, sd)]
                        for x, y in zip(xs, ys)])
    return nodes, centres


def scatter(model, movable: List[str], seed: int = 0, clustered: bool = False,
            anchored: bool = False):
    """Every movable part at a random place on the board (less its edge
    margin), at its present turn: the start, a part's neighbourhood local
    from the first move. `clustered` then gathers each group's movable
    members round one random point, in a square of their courtyards' area,
    so the anneal arranges whole groups from the first move. `anchored`
    gathers them round the point its wiring prefers instead (`_anchors`),
    each part in no group set at its own."""
    st = model.state
    rng = random.Random(seed)
    inner = tuple(st.usable)
    for r in movable:
        p = st.parts[r]
        c = p.rect()
        w, h = c[2] - c[0], c[3] - c[1]
        x = rng.uniform(inner[0] + w / 2, max(inner[0] + w / 2, inner[2] - w / 2))
        y = rng.uniform(inner[1] + h / 2, max(inner[1] + h / 2, inner[3] - h / 2))
        _place(st, r, p.x + x - (c[0] + c[2]) / 2.0, p.y + y - (c[1] + c[3]) / 2.0, p.rot)
    if anchored:
        nodes, centres = _anchors(model, movable, rng, inner)
        for m, (gx, gy) in zip(nodes, centres):
            area = sum((q[2] - q[0]) * (q[3] - q[1]) for q in (st.parts[r].rect() for r in m))
            half = min(math.sqrt(area) / 2.0, (inner[2] - inner[0]) / 2.0,
                       (inner[3] - inner[1]) / 2.0)
            gx = min(max(gx, inner[0] + half), inner[2] - half)
            gy = min(max(gy, inner[1] + half), inner[3] - half)
            for r in m:
                p = st.parts[r]
                c = p.rect()
                x, y = (gx, gy) if len(m) == 1 else (gx + rng.uniform(-half, half),
                                                     gy + rng.uniform(-half, half))
                _place(st, r, p.x + x - (c[0] + c[2]) / 2.0, p.y + y - (c[1] + c[3]) / 2.0, p.rot)
    elif clustered:
        mov = set(movable)
        for _n, members in model.groups:
            m = [r for r in members if r in mov]
            if len(m) < 2:
                continue
            area = sum((q[2] - q[0]) * (q[3] - q[1]) for q in (st.parts[r].rect() for r in m))
            half = min(math.sqrt(area) / 2.0, (inner[2] - inner[0]) / 2.0, (inner[3] - inner[1]) / 2.0)
            gx = rng.uniform(inner[0] + half, max(inner[0] + half, inner[2] - half))
            gy = rng.uniform(inner[1] + half, max(inner[1] + half, inner[3] - half))
            for r in m:
                p = st.parts[r]
                c = p.rect()
                x, y = gx + rng.uniform(-half, half), gy + rng.uniform(-half, half)
                _place(st, r, p.x + x - (c[0] + c[2]) / 2.0, p.y + y - (c[1] + c[3]) / 2.0, p.rot)
    # each datasheet-template member at its template pose round its anchor's
    # start: the arrangement as drawn, for the anneal to keep
    mov = set(movable)
    for i, (_a, m) in enumerate(model.tmpl):
        r = m['ref']
        if r in mov:
            x, y, rot = model.template_target(i)
            st.parts[r].ensure_rotation(rot)
            _place(st, r, x, y, rot)
    model.reprice_all()


def anneal(model, movable: List[str], *, moves: int = 11000, seconds: float = 0.0,
           seed: int = 0, log=None, repair: bool = False, seats=None) -> Dict:
    """Anneal from the live poses and leave the best overlap-free placement
    seen applied, each part then held to the exact legality. With `repair`,
    a run that never came clean is first mended where it stands
    (`Annealer.repair`). Returns
    {'moves' (executed), 'seconds', 'loop_seconds', 'start', 'best',
    'tracked_best', 't0', 'stats': {kind: (tried, kept)}, 'capped',
    'reseated' (parts moved to their nearest legal pose afterwards),
    'repaired' (the run never came clean and `Annealer.repair` mended it)}."""
    a = Annealer(model, movable, seed=seed, seats=seats)
    st = model.state
    for r, seat in a.seats.items():     # back on its edge after the scatter
        p = st.parts[r]
        pose = project(seat, (p.x, p.y, p.rot))
        p.ensure_rotation(pose[2])
        _place(st, r, *pose)
    t_start = time.time()
    model.reprice_all()         # parts may have moved outside the model
    model.total()               # ... and the index and grids with them
    t0 = a.start_temperature()
    kinds = a.kinds()
    reach0 = a.span / 6.0
    late = int(BIG_UNTIL * moves)       # from here no group turns or swaps
    run = {'capped': False, 'done': 0}  # `seconds` stops the loop early; moves executed
    stats = {k: [0, 0] for k in kinds}

    def begin():
        cur = model.total()['cost']
        ov = a.overlap()
        best = (cur, {r: a.pose(r) for r in a.mov}) if ov < CLEAN else (math.inf, None)
        return {'cur': cur, 'ov': ov, 'best': best, 'window': [0, 0]}

    def loop(v, k0, k1):
        wts = [MOVES[k] if k0 < late or k not in BIG_GROUP_MOVES else 0.0 for k in kinds]
        for k in range(k0, k1):
            if seconds and time.time() - t_start > seconds:
                run['capped'] = True
                return
            run['done'] += 1
            frac = k / moves
            T = t0 * (T_END / t0) ** frac
            a.lam = LAM_START * (LAM_END / LAM_START) ** frac
            reach = max(0.1, reach0 * (1.0 - frac) ** 2)
            if k == late:
                wts = [0.0 if kk in BIG_GROUP_MOVES else w for kk, w in zip(kinds, wts)]
            kind = a.rng.choices(kinds, wts)[0]
            new = a.propose(kind, reach)
            stats[kind][0] += 1
            if not new:
                continue
            if kind == 'turn' and a.alike.issuperset(new) and all(new[r] == a.pose(r) for r in new):
                # a turn of a part every turn leaves the same changes
                # nothing: kept, as the rule keeps a zero change, unpriced
                kept, dc, do = True, 0.0, 0.0
            else:
                kept, dc, do = a.trial(new, T)
            if kept:
                stats[kind][1] += 1
                v['cur'] += dc
                v['ov'] = max(0.0, v['ov'] + do)
                if v['ov'] < CLEAN and v['cur'] < v['best'][0] - 1e-9:
                    v['best'] = (v['cur'], {r: a.pose(r) for r in a.mov})
            v['window'][0] += 1
            v['window'][1] += int(kept)
            if log and moves >= 10 and (k + 1) % (moves // 10) == 0:
                log(f"move {k + 1:6d}: T {T:7.3f} lam {a.lam:7.1f} reach {reach:5.2f} "
                    f"cost {v['cur']:7.1f} overlap {v['ov']:6.2f} mm2 best {v['best'][0]:7.1f} "
                    f"accepted {v['window'][1] / max(1, v['window'][0]):.2f}")
                v['window'] = [0, 0]

    # A LONG seated part with no mating face may lie either way round along
    # its edge, and which way is one decision for the whole board: a 1x24
    # header turned half round reverses its pins against every part wired to
    # them, and once those parts gather round one order the anneal cannot
    # turn it back (fusb302_pd_dev, its two headers free: 7 of 12 seeds
    # ended reversed, 100-180 airwire crossings against 26-34). Long is
    # pads spread along the edge further than the anneal's longest step
    # (`reach0`, a sixth of the board): its partners cannot follow it
    # there, where a short part's half turn is a local change its ordinary
    # turn moves still make. So the first quarter runs twice from the same
    # start, those parts as they are and each turned half round, and the
    # anneal goes on from the cheaper; after it they still turn, but never
    # half round.
    flips = sorted(r for r, sd in a.seats.items()
                   if any(rot_key(q + 180.0) in sd.lines for q in sd.lines)
                   and pad_span(st, sd) > reach0)
    t_loop = time.time()
    if flips and late > 0:
        a.no_flip = frozenset(flips)
        p0 = {r: a.pose(r) for r in a.mov}
        rng0 = a.rng.getstate()
        tried = []
        for turned in (False, True):
            for r, pose in p0.items():
                if turned and r in flips:
                    pose = project(a.seats[r], (pose[0], pose[1], pose[2] + 180.0), pose[2])
                st.parts[r].ensure_rotation(pose[2])
                _place(st, r, *pose)
            model.reprice_all()
            a.rng.setstate(rng0)
            v = begin()
            if not turned:
                start = v['cur']
            loop(v, 0, late)
            tried.append((v['cur'] + a.lam * v['ov'], turned, v,
                          {r: a.pose(r) for r in a.mov}, a.rng.getstate()))
        _score, _turned, v, poses, rng = min(tried, key=lambda t: (t[0], t[1]))
        for r, pose in poses.items():
            st.parts[r].ensure_rotation(pose[2])
            _place(st, r, *pose)
        model.reprice_all()
        model.total()
        a.rng.setstate(rng)
        loop(v, late, moves)
    else:
        v = begin()
        start = v['cur']
        loop(v, 0, moves)
    best, best_poses = v['best']
    capped, done = run['capped'], run['done']
    loop_seconds = time.time() - t_loop
    repaired = False
    if repair and best_poses is None and not capped and a.repair(T_END) < CLEAN:
        best_poses, repaired = {r: a.pose(r) for r in a.mov}, True
    for r in a.mov:     # the QuenchState's own state, airwires and all
        pose = best_poses[r] if best_poses is not None else a.pose(r)
        st.parts[r].ensure_rotation(pose[2])
        st.apply_move(r, *pose)
    # the parts seated on one edge pushed apart along it, in their order;
    # then the free parts, each to its nearest legal pose; then a seated
    # part still illegal slides along its edge to a legal spot on it
    reseated = pack(st, a.seats) if a.seats else []
    for r in [q for q in a.mov if q not in a.seats] + sorted(a.seats):
        p = st.parts[r]
        if r in a.seats:
            if not legal_on_edge(st, r, (p.x, p.y, p.rot)):
                got = reseat(st, a.seats[r])
                if got:
                    st.parts[r].ensure_rotation(got[2])
                    st.apply_move(r, *got)
                    if r not in reseated:
                        reseated.append(r)
            continue
        if legal(st, r, p.x, p.y, p.rot):
            continue
        for rot in (p.rot,) + tuple(q for q in ROTS if q != p.rot):
            got = nearest_legal(st, r, p.x, p.y, rot, frozenset(), reach=6.0)
            if got:
                st.apply_move(r, *got)
                reseated.append(r)
                break
    model.reprice_all()
    final = model.total()['cost']
    return {'moves': done, 'seconds': round(time.time() - t_start, 2), 'start': start,
            'best': final, 'tracked_best': best, 't0': t0, 'capped': capped,
            'reseated': reseated, 'repaired': repaired, 'loop_seconds': round(loop_seconds, 3),
            'stats': {kk: tuple(v) for kk, v in stats.items()}}
