"""Edge connectors placed from a pile: each unlocked edge part (a connector,
receptacle, switch or test pad: SEATED_CLASSES, by the entry's declared
`class`, else `part_class.classify_part`'s) the intent declares on a
board edge (`edge_connectors[]` with an `edge`) is seated on that edge before
the anneal, and the anneal then only slides it along the edge (and turns it,
when it has no mating face).

The seat is read with the grade's own measurements (`rule_edge_connector`):

* the TURN points its mating face off the board (`mating.mating_face_local`,
  `mating.facing_edge`). A part with no in-plane face (a vertical header, a
  test pad, a symmetric terminal) may take any quarter turn, each with its
  own seat, and the anneal chooses: designers lay such headers both ways --
  noahfc's five north-edge 1x03 headers stand ACROSS the edge, five abreast
  where only three fit lying along it, and tigard's 2x05 J5 lies along its
  edge;
* the distance from the edge puts the part past the outline by the low end
  of the entry's `overhang_mm` band as the grade reads it
  (`connector_geometry.band_amount`: the drawn body, else the courtyard, at
  zero margin, #1301) -- flush, without one, and a hair inside a positive
  minimum -- and no nearer the edge than keeps every pad's copper (the
  pads on a copper layer, not a silk-drawn pin) the board's edge clearance
  inside the outline where the part stands, not its bounding box (#1325);
* ALONG the edge it may go wherever every conjunct the grade puts on it
  holds -- the band (a corner adds the side it crosses), the pad copper's
  edge clearance against the true outline (a rounded corner), the edge the
  grade reads it on (`floorplan._nearest_edge` on the rule's own seat rect),
  a declared `along_edge_band` / `center_on_edge` -- and a declared pose's
  check passes against every part the anneal does not move. Each turn's
  stretches are read by testing poses every SCAN_STEP_MM and at every
  position where the part's courtyard or pad copper just meets a fixed
  part's courtyard or pad copper, or a board margin, each change between
  two of them bisected to REFINE_MM, so a gap barely wider than the part
  (tigard's J7 between H3 and J6: 0.1 mm of play) is found. A move that
  lands outside them snaps to the nearest one, so a connector hops past a
  locked switch rather than pushing through it.

Its legality is a declared pose's (`seeder._fixed_pose_check`): its courtyard
may cross the outline when every pad and hole stays on the board, and
courtyards may abut -- two seated parts keep SEAT_GAP_MM, and `pack` pushes
the parts on one edge apart along it after the anneal.

A declared edge part with no seat is placed as any other part, and the reason
says what stopped it (`_unseated_reason`): an overhang that never comes down
to its band at any pose (a courtyard longer than its edge), pads longer than
the edge, or the conjuncts every position tested failed, counted, with the
parts in the way.

Parts that each have a seat may still not fit on their edge TOGETHER
(`_crowding`): CM5_MINIMA's J501 and J802, both declared on its south edge,
16.90 and 17.10 mm long, can stand only at x 89.43-94.10 and 89.59-103.40
between the fixed parts, so at most one of them is ever there. Seated anyway,
every seed annealed three times and left both illegal. The seat's own lines
are packed along the edge as `pack` packs a row (courtyards SEAT_GAP_MM apart,
each in its free stretches, at any turn it seats at), exactly over every
subset, one row per edge and face. When they do not all fit, the most that
fit together are seated, the earlier declared on a tie (the intent's
`edge_connectors[]` order), and the rest are placed as any other part. The
record (`crowding_text`) is the intent's to fix, the same in every seed.

A line is read against the fixed parts alone, so a part seated on ANOTHER
edge is walled off it separately (`_corners`): what that part covers at
every pose its own seat allows (`_covered`) is cut from the line, to a fixed
point, before the rows are packed. CM5_MINIMA's J501, on the south edge,
covers x 85.70-98.00 over y 68.8-80.7 wherever it stands, and the west
edge's J101 and USB701 were given their line up to y 76.0 regardless: of
seeds 0-3 and 7, four annealed three times and three ended with J501 and J101
or USB701 illegal at that corner. A part
the cut leaves no position is placed as any other part, its reason naming
the parts that walled it.
"""
from __future__ import annotations

from collections import namedtuple
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple

ROTS = (0.0, 90.0, 180.0, 270.0)
#: The step a seat line is scanned at, and the precision each change found
#: between two steps is then bisected to.
SCAN_STEP_MM = 0.5
REFINE_MM = 0.02
#: The step and the reach of a part's slide to a legal spot on its edge after
#: the anneal.
SLIDE_STEP_MM = 0.25
SLIDE_REACH_MM = 12.0
#: Pad copper is held this much beyond the edge clearance, so a pose written
#: to the board's 0.1 um never reads as copper exactly AT the clearance (the
#: copper grader fails 0.3000 against 0.3).
COPPER_SLACK_MM = 0.005
#: A positive band minimum is aimed this much inside (at most half the band),
#: for the same reason: seated exactly at it, daplink's J3 wrote back as
#: 7.0000 under its declared 7.0 (#1325).
BAND_SLACK_MM = 0.005
#: Parts farther than this from a seat line cannot take part in its checks.
NEAR_MM = 3.0
#: The gap two seated parts keep along an edge: their courtyards may abut (a
#: declared pose's check), and 10 um keeps a touch rounded in the written
#: poses from reading as an overlap.
SEAT_GAP_MM = 0.01
EPS = 1e-6
#: The most parts on one edge and face `_crowding` checks together: it tries
#: every subset of them. A longer row is left to the anneal and `pack`.
ROW_MAX = 12

#: The part classes (`part_class.classify_part`) a declared edge seats: what
#: plugs in, is pressed or is probed at the edge. The intent emitter claims an
#: edge only for these (`floorplan.emit_intent`): a cap or a solder jumper a
#: designer put by the edge keeps its observed overhang as a band and no edge.
#: Held there, noahfc's C6 and C8 cost 3 blocking nets and +6% copper over 16
#: seeds against letting them sit by their ICs.
SEATED_CLASSES = ('edge_receptacle', 'edge_actuator', 'connector_affinity', 'testpoint')

#: One turn's seat: `coord` is the origin's fixed coordinate across the edge,
#: `free` the stretches `(a0, a1)` of the origin's coordinate along it.
Line = namedtuple('Line', 'coord free')
#: `axis` is the coordinate the part slides along: 0 (x) on a north or south
#: edge, 1 (y) on east or west. `lines` maps each turn it may take to its
#: Line; `face` is its mating face ('' when it has none in the plane).
EdgeSeat = namedtuple('EdgeSeat', 'ref edge axis face lines')


def rot_key(rot) -> float:
    return round(float(rot) % 360.0, 6) % 360.0


def _turns(state, pcb, geometry, entry, edge) -> Tuple[List[float], str]:
    """The turns `entry`'s part may take on `edge` and its mating face: the
    turns that point the face at the edge -- the face its body reads, else
    the one the entry declares (`mating_face`) -- or every quarter turn when
    it has none in the plane (its own turn first)."""
    from .mating import facing_edge, mating_face_local
    ref = str(entry['ref'])
    lf = mating_face_local(pcb.footprints[ref], geometry.get(ref))
    face = lf.face or entry.get('mating_face')
    if face:
        return [r for r in ROTS if facing_edge(face, r) == edge], face
    own = rot_key(state.parts[ref].rot)
    return [own] + [r for r in ROTS if r != own], ''


def pad_span(state, seat) -> float:
    """How far apart along its edge `seat`'s part spreads its pad centres,
    at the turn of its seat that spreads them furthest."""
    part = state.parts[seat.ref]
    span = 0.0
    for rot in seat.lines:
        part.ensure_rotation(rot)
        along = [p[seat.axis] for p in part.pad_globals(0.0, 0.0, rot)]
        if along:
            span = max(span, max(along) - min(along))
    part.ensure_rotation(part.rot)
    return span


def _pad_box(geometry, ref, pose):
    """The box of `ref`'s pad COPPER at `pose`: the pads `copper_clear`
    grades (`connector_geometry.pad_boxes`), each its rotated rectangle's box,
    or None when it has none. A pad on no copper layer is not copper:
    daplink's Harwin M20 headers draw each pin as a `pad "" smd` on F.SilkS,
    6 mm long, and counted here it held the seat 0.55 mm inside its edge
    (#1325)."""
    import math
    from .connector_geometry import pad_boxes
    x, y, rot = pose
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    box = None
    for _i, lx, ly, hx, hy, tilt in pad_boxes(geometry, ref):
        a = math.radians(tilt + rot)
        ca, sa = abs(math.cos(a)), abs(math.sin(a))
        ex, ey = hx * ca + hy * sa, hx * sa + hy * ca
        px, py = x + c * lx + s * ly, y - s * lx + c * ly
        q = (px - ex, py - ey, px + ex, py + ey)
        box = q if box is None else (min(box[0], q[0]), min(box[1], q[1]),
                                     max(box[2], q[2]), max(box[3], q[3]))
    return box


class _Probe:
    """The grade's conjuncts on one declared edge part, at a trial pose."""

    def __init__(self, state, pcb, pcb_file, bodies, outline):
        from .connector_geometry import geometry_for
        from .legality import BoardOutlineGate
        self.state, self.pcb, self.bodies = state, pcb, bodies
        self.geo = geometry_for(state, pcb, pcb_file)
        self.bounds = outline.get('bounds') or pcb.board_info.board_bounds
        self.copper = BoardOutlineGate(pcb.board_info,
                                       state.board_edge_clearance + COPPER_SLACK_MM)
        #: each part's mating face in its own frame ('' for none)
        self.faces: Dict[str, str] = {}

    def band(self, ref, edge, pose) -> float:
        return self.band_read(ref, edge, pose)[0]

    def band_read(self, ref, edge, pose) -> Tuple[float, str]:
        """`(amount, basis)`: the overhang the grade reads at `pose`, and
        the currency it reads it in (`connector_geometry.band_amount`)."""
        from .connector_geometry import band_amount, courtyard_overhang
        # #1301: the courtyard at ZERO margin, as the grade reads it
        legacy = courtyard_overhang(self.geo, self.state.parts[ref].rect(*pose))
        return band_amount(self.geo, ref, edge, legacy, 0.0, pose=pose)[:2]

    def copper_clear(self, ref, pose) -> bool:
        """Every pad's copper the board's edge clearance inside the outline
        (`connector_geometry.pad_copper_outside`), asked pad by pad only when
        the box round all of them is not clear."""
        from .connector_geometry import pad_copper_outside
        box = _pad_box(self.geo, ref, pose)
        if box is None or self.copper.rect_outside_amount(box) <= 1e-9:
            return True
        return pad_copper_outside(self.geo, self.copper, ref, pose) <= 1e-9

    def why(self, entry, edge, pose, obstacles) -> Tuple[str, object]:
        """`('', None)` when every conjunct `rule_edge_connector` and check_drc
        put on the part at `pose` holds, and a declared pose's check against
        `obstacles` passes; else the first that fails and what it read:
        `('band_min', amount)`, `('band_max', amount)`, `('copper', None)`,
        `('edge', the edge it reads nearest)` or `('fixed', [the parts in
        the way, else the declared-pose check's reasons])`."""
        from .floorplan import (_nearest_edge, drawn_body_rect, edge_reach_mm,
                                edge_seat_rect)
        from .mating import facing_edge
        from .seeder import _fixed_pose_check
        st, ref = self.state, entry['ref']
        x, y, rot = pose
        band = self.band(ref, edge, pose)
        lim = entry.get('overhang_mm') or {}
        if band < float(lim.get('min', 0.0)) - EPS:
            return 'band_min', band
        if lim.get('max') is not None and band > float(lim['max']) + EPS:
            return 'band_max', band
        if not self.copper_clear(ref, pose):
            return 'copper', None
        proxy = SimpleNamespace(x=x, y=y, rotation=rot)
        seat_rect, _basis = edge_seat_rect(
            entry, st.parts[ref].rect(x, y, rot),
            lambda: drawn_body_rect(self.bodies.get(ref), proxy))
        face = self.faces.get(ref)
        near = _nearest_edge(seat_rect, self.bounds, edge_reach_mm(st),
                             facing=facing_edge(face, rot) if face else None)
        if near != edge:
            return 'edge', near
        how, reasons, conflicts = _fixed_pose_check(st, ref, pose, obstacles)
        if how is None:
            return 'fixed', sorted(conflicts) or list(reasons)[:1]
        return '', None


def _near(state, ref, pose, pool):
    """`{other: pose}` for the parts of `pool` within NEAR_MM of `ref` at
    `pose`: the only ones a declared pose's check can find a conflict with."""
    c = state.parts[ref].rect(*pose)
    out = {}
    for o, op in pool.items():
        q = state.parts[o].rect(*op)
        if (q[0] < c[2] + NEAR_MM and c[0] < q[2] + NEAR_MM
                and q[1] < c[3] + NEAR_MM and c[1] < q[3] + NEAR_MM):
            out[o] = op
    return out


def _seat_coord(probe, ref, edge, rot, axis, along, target):
    """`(coord, floor)`: the origin's coordinate across `edge` that puts
    `ref` `target` past the outline as the grade reads it, held back so
    every pad's copper is the board's edge clearance inside the outline AT
    THE SEAT; `floor` is the least overhang the part reads on the line (its
    reading centred across the board). `coord` is None when even `floor` is
    past `target`: sonde_xilinx's DB25 J1, 54.1 mm along a 43.18 mm edge,
    reads 11.43 mm wherever it stands (#1325)."""
    state, pcb = probe.state, probe.pcb
    part = state.parts[ref]
    bounds = pcb.board_info.board_bounds
    out = -1.0 if edge in ('north', 'west') else 1.0     # outward along the coordinate
    edge_at = {'north': bounds[1], 'south': bounds[3], 'west': bounds[0], 'east': bounds[2]}[edge]

    def pose(c):
        return (along, c, rot) if axis == 0 else (c, along, rot)
    # from the part centred across the board (its band lowest) to well past the edge
    cr = part.rect(0.0, 0.0, rot)
    perp = 1 - axis
    centre = (bounds[perp] + bounds[perp + 2]) / 2.0 - (cr[perp] + cr[perp + 2]) / 2.0
    floor = probe.band(ref, edge, pose(centre))
    if floor > target + EPS:
        return None, floor
    inner, outer = centre, edge_at + out * 40.0
    for _ in range(50):
        mid = (inner + outer) / 2.0
        if probe.band(ref, edge, pose(mid)) <= target + EPS:
            inner = mid
        else:
            outer = mid
    # The copper limit, read on the OUTLINE where the part stands, by the
    # very check `_Probe.why` makes: the bounding box is not the edge on a
    # board with ears. CM5_MINIMA's east bounds are its corner ears at
    # 136.25 while the straight edge it seats on is at 132.90, so a limit
    # taken off the bounds never bound and every pose failed (#1325).
    if probe.copper_clear(ref, pose(inner)) or not probe.copper_clear(ref, pose(centre)):
        return inner, floor     # clear; or clear nowhere, which `_free_stretches` names
    good, bad = centre, inner
    for _ in range(50):
        mid = (good + bad) / 2.0
        if probe.copper_clear(ref, pose(mid)):
            good = mid
        else:
            bad = mid
    return good, floor


def _free_stretches(probe, entry, edge, axis, coord, rot, lo, hi, fixed, tally=None):
    """The stretches of [lo, hi] where `entry`'s part seats at `rot` with
    every conjunct holding (`_Probe.why`): tested every SCAN_STEP_MM and at
    each position where its courtyard or pad copper just meets a fixed
    part's courtyard or pad copper, or a board margin -- where a stretch
    narrower than a step can lie -- and each change between two tested
    positions bisected to REFINE_MM. `tally`, a dict, collects why each
    tested position failed (`_Probe.why`), `{kind: [count, readings]}`."""
    if hi < lo - EPS:
        return []
    state, ref = probe.state, entry['ref']
    cr = state.parts[ref].rect(0.0, 0.0, rot)
    perp = 1 - axis
    lo_p, hi_p = coord + cr[perp], coord + cr[perp + 2]     # the courtyard across the edge
    cand = [lo + k * SCAN_STEP_MM for k in range(int((hi - lo) / SCAN_STEP_MM) + 1)] + [hi]
    # and where its pad copper meets a fixed part's courtyard, or its
    # courtyard a fixed part's pad copper: a through-hole part's copper is on
    # both faces, its courtyard on one (ch32v003_usb's F-side J1 between the
    # B-side rows J2 and J3 seats only in 0.46 mm lying between two courtyard
    # touches)
    cp = state.parts[ref].padbox(0.0, 0.0, rot)
    for o, op in fixed.items():
        q = state.parts[o].rect(*op)
        if q[perp] < hi_p + NEAR_MM and lo_p < q[perp + 2] + NEAR_MM:
            qp = state.parts[o].padbox(*op)
            for mine, theirs in ((cr, q), (cp, q), (cr, qp)):
                if mine is not None and theirs is not None:
                    cand += [theirs[axis] - mine[axis + 2], theirs[axis + 2] - mine[axis]]
    from .floorplan import edge_reach_mm
    b = probe.pcb.board_info.board_bounds
    for m in (state.edge_gate.margin, edge_reach_mm(state), 0.0):
        # the gate's margin (the declared-pose check's outline), the edge
        # reach (`_nearest_edge`'s), and the band's, at zero margin
        cand += [b[axis] + m - cr[axis], b[axis + 2] - m - cr[axis + 2]]
    cand = sorted({round(a, 6) for a in cand if lo - EPS <= a <= hi + EPS})
    # only the fixed parts the line passes near can conflict with it
    span = ((lo + cr[0], lo_p, hi + cr[2], hi_p) if axis == 0
            else (lo_p, lo + cr[1], hi_p, hi + cr[3]))
    pool = {o: op for o, op in fixed.items() if _overlaps(state.parts[o].rect(*op), span, NEAR_MM)}

    def ok(a, record=False):
        pose = (a, coord, rot) if axis == 0 else (coord, a, rot)
        kind, read = probe.why(entry, edge, pose, _near(state, ref, pose, pool))
        if kind and record and tally is not None:
            row = tally.setdefault(kind, [0, []])
            row[0] += 1
            row[1].append(read)
        return not kind
    tested = [(a, ok(a, record=True)) for a in cand]
    pts = tested[:1]
    for (a0, k0), (a1, k1) in zip(tested, tested[1:]):
        if k0 != k1:        # the last good position before the change
            good, bad = (a0, a1) if k0 else (a1, a0)
            while abs(bad - good) > REFINE_MM:
                mid = (good + bad) / 2.0
                if ok(mid):
                    good = mid
                else:
                    bad = mid
            pts.append((good, True))
        pts.append((a1, k1))
    free, start, last = [], None, None
    for a, k in pts:
        if k:
            start = a if start is None else start
            last = a
        elif start is not None:
            free.append((start, last))
            start = None
    if start is not None:
        free.append((start, last))
    return free


def _unseated_reason(probe, entry, edge, axis, target, e, floors, spans, tally,
                     squeezed=()) -> str:
    """Why `entry`'s part has no seat on `edge`, naming what failed (#1325):
    the band never coming down to `target` (`floors`, per turn), pads longer
    than the edge (`spans`), a declared along-edge window too narrow for
    them (`squeezed`), and the conjuncts the tested positions failed
    (`tally`, from `_free_stretches`). One string, "no room along the <edge>
    edge" used to stand for all of them."""
    ref = entry['ref']
    part = probe.state.parts[ref]
    bounds = probe.pcb.board_info.board_bounds
    edge_len = bounds[axis + 2] - bounds[axis]
    lim = entry.get('overhang_mm') or {}
    out = []
    if floors:
        rot = min(floors, key=floors.get)
        part.ensure_rotation(rot)
        cr = part.rect(0.0, 0.0, rot)
        basis = probe.band_read(ref, edge, (part.x, part.y, rot))[1]
        part.ensure_rotation(part.rot)
        why = (f"its overhang reads at least {floors[rot]:.2f}mm wherever it stands ({basis}), "
               f"never the {target:.2f}mm at the low end of its band that the seat puts it at")
        if cr[axis + 2] - cr[axis] > edge_len + EPS:
            why += (f": its courtyard is {cr[axis + 2] - cr[axis]:.2f}mm along a "
                    f"{edge_len:.2f}mm edge")
        out.append(why + "; declare overhang_mm.min at the reading where its mating face "
                         "meets the edge")
    if spans:
        out.append(f"its pad copper spans {min(spans.values()):.2f}mm along a {edge_len:.2f}mm "
                   f"edge, so no position keeps both ends {e - COPPER_SLACK_MM:.2f}mm inside it")
    if squeezed:
        key = 'along_edge_band' if entry.get('along_edge_band') is not None else 'center_on_edge'
        out.append(f"no position inside its declared {key} keeps its pads "
                   f"{e - COPPER_SLACK_MM:.2f}mm inside the edge's ends")
    if tally:
        said = []
        for kind, (n, reads) in sorted(tally.items(), key=lambda kv: (-kv[1][0], kv[0])):
            if kind == 'copper':
                said.append(f"{n} with pad copper inside the board's "
                            f"{e - COPPER_SLACK_MM:.2f}mm edge clearance")
            elif kind == 'band_min':
                said.append(f"{n} reading an overhang of at most {max(reads):.2f}mm, under "
                            f"the declared minimum {float(lim.get('min', 0.0)):.2f}mm")
            elif kind == 'band_max':
                said.append(f"{n} reading an overhang of at least {min(reads):.2f}mm, past "
                            f"the declared maximum {float(lim['max']):.2f}mm")
            elif kind == 'edge':
                said.append(f"{n} reading nearest the {'/'.join(sorted(set(reads)))} edge")
            else:
                seen = []
                for r in reads:
                    seen += [str(x) for x in r if str(x) not in seen]
                refs = [x for x in seen if ' ' not in x]
                said.append(f"{n} refused by the declared-pose check"
                            + (f" against {', '.join(refs[:4])}"
                               + (f" and {len(refs) - 4} more" if len(refs) > 4 else '')
                               if refs else f" ({seen[0]})" if seen else ''))
        out.append(f"of the {sum(v[0] for v in tally.values())} positions tested, "
                   + ', '.join(said))
    return f"no room along the {edge} edge: " + ('; '.join(out) or 'nothing was tested')


def _overlaps(q, c, grow):
    return (q[0] < c[2] + grow and c[0] < q[2] + grow
            and q[1] < c[3] + grow and c[1] < q[3] + grow)


def seats(state, pcb, pcb_file, intent, movable, crowded=None
          ) -> Tuple[Dict[str, EdgeSeat], List[str], Dict[str, str]]:
    """`({ref: EdgeSeat}, notes, {ref: why unseated})` for the movable parts
    the intent puts on an edge, each already moved onto its edge, in the
    middle of the longest stretch it has at its own turn (else at the turn
    with the most room). A declared edge part that finds no seat is placed as
    any other part, and named in the third value: the caller warns.

    Parts seated on one edge and face that do not all fit there together
    keep the most that do (`_crowding`), and the rest are placed as any
    other part, back where they stood; `crowded`, a list, collects one
    record per such edge (`crowding_text` words it)."""
    from .body import board_bodies
    from .floorplan import edge_span, outline_state
    out: Dict[str, EdgeSeat] = {}
    notes: List[str] = []
    unseated: Dict[str, str] = {}
    # every entry naming an edge whose part is an edge part by class (a
    # `connector_affinity` entry an author gave an edge included): the class
    # the entry DECLARES, else `classify_part`'s, as the grade reads it
    # (`floorplan.edge_facing_policy`, the receptacle seat). Read off the
    # name alone, CM5's HDMI J501 (`EDAC 690-019-298-412`) has no class, and
    # its declared `edge_receptacle` was ignored here while the grade failed
    # it as one (#1331).
    from .part_class import classify_part
    entries = []
    for c in intent.edge_connectors:
        ref = c.get('ref')
        if ref not in movable or c.get('edge') not in ('north', 'south', 'east', 'west'):
            continue
        cls = c.get('class') or classify_part(pcb.footprints[ref], ref).name
        if cls not in SEATED_CLASSES:
            what = (f"declared class {cls}" if c.get('class')
                    else cls or 'no part class')
            notes.append(f"{ref} is declared on the {c['edge']} edge but is not an edge part "
                         f"({what}): placed as any other part")
            continue
        entries.append(c)
    if not entries:
        return out, notes, unseated
    bodies = board_bodies(pcb, pcb_file)
    outline = outline_state(pcb, pcb_file)
    probe = _Probe(state, pcb, pcb_file, bodies, outline)
    bounds = pcb.board_info.board_bounds
    fixed = {o: (q.x, q.y, q.rot) for o, q in state.parts.items() if o not in movable}
    was: Dict[str, Tuple[float, float, float]] = {}     # each seated part's pose before
    said: Dict[str, str] = {}                           # and its note
    for c in entries:
        ref, edge = str(c['ref']), c['edge']
        part = state.parts[ref]
        rots, face = _turns(state, pcb, bodies, c, edge)
        probe.faces[ref] = face
        if not rots:
            unseated[ref] = (f"its mating face ({face}) points at the {edge} edge at no "
                             f"quarter turn")
            continue
        lim = c.get('overhang_mm') or {}
        target = float(lim.get('min', 0.0))
        aim = target
        if target > EPS:
            room = (float(lim['max']) - target) / 2.0 if lim.get('max') is not None else BAND_SLACK_MM
            aim += max(0.0, min(BAND_SLACK_MM, room))
        axis = 0 if edge in ('north', 'south') else 1
        e = state.board_edge_clearance + COPPER_SLACK_MM
        lines = {}
        # why each turn has no seat, for the reason a caller prints
        floors: Dict[float, float] = {}       # the band never comes down to `target`
        spans: Dict[float, float] = {}        # the pads do not fit along the edge
        squeezed = set()                      # nor inside a declared along-edge window
        tally: Dict[str, list] = {}           # every position tested failed a conjunct
        for rot in rots:
            part.ensure_rotation(rot)
            # along: its pads the edge clearance inside the board's span (a
            # long header runs corner to corner, its courtyard past both
            # ends), narrowed to a declared band
            cr = part.rect(0.0, 0.0, rot)
            pb = _pad_box(probe.geo, ref, (0.0, 0.0, rot)) or cr
            lo = bounds[axis] + e - pb[axis]
            hi = bounds[axis + 2] - e - pb[axis + 2]
            if hi < lo - EPS:
                spans[rot] = pb[axis + 2] - pb[axis]
                continue
            coord, floor = _seat_coord(probe, ref, edge, rot, axis, (lo + hi) / 2.0, aim)
            if coord is None:
                floors[rot] = floor
                continue
            along = c.get('along_edge_band')
            if along is None and c.get('center_on_edge'):
                along = {'mm': float(c['center_on_edge']['tolerance_mm'])}
            if along is not None:
                s0, s1, _basis = edge_span(state.edge_gate, bounds, edge, outline)
                if s0 is not None:
                    mid_off = (cr[axis] + cr[axis + 2]) / 2.0    # the courtyard centre from the origin
                    if 'mm' in along:
                        centre = (s0 + s1) / 2.0
                        f0, f1 = centre - along['mm'], centre + along['mm']
                    else:
                        f0 = s0 + float(along['from']) * (s1 - s0)
                        f1 = s0 + float(along['to']) * (s1 - s0)
                    lo, hi = max(lo, f0 - mid_off), min(hi, f1 - mid_off)
                    if hi < lo - EPS:
                        squeezed.add(rot)
                        continue
            free = _free_stretches(probe, c, edge, axis, coord, rot, lo, hi, fixed, tally)
            if free:
                lines[rot] = Line(coord, tuple(free))
        if not lines:
            part.ensure_rotation(part.rot)
            unseated[ref] = _unseated_reason(probe, c, edge, axis, target, e,
                                             floors, spans, tally, squeezed)
            continue
        seat = EdgeSeat(ref, edge, axis, face, lines)
        out[ref] = seat
        was[ref] = (part.x, part.y, part.rot)
        _sit(part, seat)
        said[ref] = (f"edge connector {ref} on the {edge} edge"
                     + (f", facing it ({face})" if face else
                        f", no mating face: {len(lines)} turn(s) seat there"))
        notes.append(said[ref])
    # What a part seated on one edge covers wherever it stands is a wall on
    # every other edge's line (`_corners`), and then the parts on one edge
    # that do not all fit there together (`_crowding`): either kind goes back
    # where it stood and is placed as any other part. A part dropped no
    # longer walls the other edges, so the lines are cut again without it,
    # and of parts walled off at once (two that wall each other) the later
    # declared goes first, as `_crowding` breaks a tie.
    order = [str(c['ref']) for c in entries]
    gone: Dict[str, str] = {}
    while True:
        narrowed, lost = _corners(state, {r: s for r, s in out.items() if r not in gone})
        if lost:
            ref = max(lost, key=order.index)
            gone[ref] = 'corner'
            unseated[ref] = _cornered_reason(out[ref], lost[ref])
            continue
        recs = _crowding(state, narrowed, order)
        for rec in recs:
            for ref in rec['unseated']:
                gone[ref] = 'crowded'
            if crowded is not None:
                crowded.append(rec)
        if not recs:
            break
    for ref in gone:
        del out[ref]
        notes.remove(said[ref])
        part = state.parts[ref]
        part.x, part.y, part.rot = was[ref]
        part.ensure_rotation(part.rot)
    for ref, seat in narrowed.items():
        if seat.lines != out[ref].lines:
            out[ref] = seat
            _sit(state.parts[ref], seat)
    return out, notes, unseated


def seat_state(pcb, pcb_file, intent, blocks, clearance, edge, fixed=(),
               estimate_courtyards=True, estimate_skip=()):
    """The placement state `place_simple` seats and anneals on: the board's
    floors, the intent's `fixed_poses[]` refs held (`fixed`), its keep-outs,
    and every declared zone binding its members (containment) and, when
    exclusive, keeping strangers out -- the board's bounds open a zone side
    on the outline to an edge-claim member (#1174). A part that draws no
    courtyard is seated on an estimated one (#1300), here as in place_simple,
    so `oversubscribed` predicts the crowding place_simple will meet. A ref
    in `estimate_skip` keeps what the board draws (place_simple's fallback)."""
    import pose_score
    from . import floorplan as fp
    zones = (fp.zone_entries(intent, blocks or {}, bounds=pcb.board_info.board_bounds)
             if intent is not None else ())
    return pose_score.make_state(pcb, pcb_file, clearance=clearance,
                                 board_edge_clearance=edge,
                                 extra_locked_refs=set(fixed) or None,
                                 keepouts=intent.keepouts if intent is not None else (),
                                 intent_zones=list(zones),
                                 exclusive_zones=[z for z in zones if z['exclusive']],
                                 estimate_courtyards=estimate_courtyards,
                                 estimate_skip=estimate_skip)


def oversubscribed(pcb, pcb_file, intent, clearance, edge, group_sources=()
                   ) -> List[dict]:
    """The `_crowding` records `place_simple` will make on this board and
    intent: its edges whose declared parts cannot all seat together, the
    same records in every seed, measured BEFORE placing, so a check of the
    plan warns where it used to say nothing and only the placement run
    reported the edge. One measure, two readers: the grade
    (`floorplan.grade(edge_seats=True)`, `plan_check`) calls this, and this
    calls `seats` on the board `place_simple` seats on -- each part on its
    declared face and at its declared pose (`floorplan.declared_moves`,
    written to a scratch copy) and the same state (`seat_state`). Nothing on
    `pcb` or its file is changed. The faces a groups file adds are not known
    here; a declaration contradicting itself leaves its part where it is; a
    board whose declared faces cannot be written measures nothing, since
    place_simple refuses it.

    A part's seat lines depend on the parts that do not move, never on where
    the movable ones stand, so a placed board and its pile give the same
    records."""
    import os
    import shutil
    import tempfile
    from . import floorplan as fp
    if not any(c.get('edge') for c in intent.edge_connectors):
        return []
    blocks = fp.resolve_blocks(intent, pcb, group_sources)[0]
    sides = fp.declared_sides(fp.declared_side_claims(intent, blocks))[0]
    fixed, moves = fp.declared_moves(intent, {r: s for r, s in sides.items()
                                              if r in pcb.footprints}, pcb)
    tmp, board, src = None, pcb, pcb_file
    try:
        if moves:
            from kicad_parser import parse_kicad_pcb
            from .portfolio import copy_siblings
            from .writer import SideFlipUnsupported, write_placed_output
            tmp = tempfile.mkdtemp(prefix='edge_seat_')
            src = os.path.join(tmp, os.path.basename(pcb_file))
            try:
                write_placed_output(pcb_file, src, moves)
            except (ValueError, SideFlipUnsupported):
                return []       # place_simple refuses such a board, and seats nothing
            copy_siblings(pcb_file, src)
            board = parse_kicad_pcb(src)
            blocks = fp.resolve_blocks(intent, board, group_sources)[0]
        state = seat_state(board, src, intent, blocks, clearance, edge, set(fixed))
        movable = {r for r, p in state.parts.items() if not p.locked}
        crowded: List[dict] = []
        seats(state, board, src, intent, movable, crowded=crowded)
        return crowded
    finally:
        if tmp:
            shutil.rmtree(tmp, ignore_errors=True)


def _sit(part, seat) -> None:
    """Move `part` onto `seat`: at its own turn when the seat has it (else
    the turn with the most room), in the middle of that turn's longest
    stretch."""
    lines = seat.lines
    own = rot_key(part.rot)
    rot = own if own in lines else max(lines, key=lambda r: _room(lines[r]))
    a0, a1 = max(lines[rot].free, key=lambda f: f[1] - f[0])
    line = lines[rot]
    x, y = ((a0 + a1) / 2.0, line.coord) if seat.axis == 0 else (line.coord, (a0 + a1) / 2.0)
    part.ensure_rotation(rot)
    part.x, part.y, part.rot = x, y, rot


def _pose_on(seat, coord, a, rot):
    return (a, coord, rot) if seat.axis == 0 else (coord, a, rot)


def _meet(p, q):
    """The overlap of two boxes, or None when it has no area."""
    box = (max(p[0], q[0]), max(p[1], q[1]), min(p[2], q[2]), min(p[3], q[3]))
    return box if box[2] - box[0] > EPS and box[3] - box[1] > EPS else None


def _covered(state, seat) -> Dict[str, list]:
    """`{face: [box, ...]}`: what `seat`'s part occupies on each board face
    at EVERY pose its seat allows, as the declared-pose check reads a part
    (`legality.rect_on`: its courtyard on its own face, its drilled pads'
    clusters, `legality.far_boxes`, on the other).

    Each turn's boxes are held at both ends of the span its stretches cover
    -- a box translated along a line covers, everywhere between two
    positions, what it covers at both -- one pad cluster against itself, and
    then only what every turn covers. A subset of what the part covers, never
    more: at CM5_MINIMA's south-west corner the HDMI receptacle J501 (16.90
    mm, its origin anywhere in x 89.50-94.10) covers x 85.70-98.00 wherever
    it stands."""
    from .legality import far_boxes, rect_on
    part = state.parts[seat.ref]
    out: Dict[str, list] = {}
    for face in sorted(part.sides):
        common = None
        for rot, line in sorted(seat.lines.items()):
            ends = [a for f in line.free for a in f]
            part.ensure_rotation(rot)
            at = [_pose_on(seat, line.coord, a, rot) for a in (min(ends), max(ends))]
            got = [rect_on(face, part.side, part.grade_rect(*p), part.tht_rect(*p)) for p in at]
            boxes = []
            if got[0] is not None and got[1] is not None:
                boxes = [m for b0, b1 in zip(far_boxes(got[0]), far_boxes(got[1]))
                         if (m := _meet(b0, b1)) is not None]
            common = boxes if common is None else [
                m for p in common for q in boxes if (m := _meet(p, q)) is not None]
            if not common:
                break
        if common:
            out[face] = common
    part.ensure_rotation(part.rot)
    return out


def _walled(state, seat, rot, covered) -> List[Tuple[float, float]]:
    """The open intervals of `seat`'s origin along its edge, at `rot`, where
    its part overlaps a box of `covered` (`_covered`'s) on a face both
    occupy, held SEAT_GAP_MM clear as `pack` holds a wall."""
    from .legality import far_boxes, rect_on
    part = state.parts[seat.ref]
    ax, perp = seat.axis, 1 - seat.axis
    part.ensure_rotation(rot)
    at = _pose_on(seat, seat.lines[rot].coord, 0.0, rot)
    out = []
    for face, boxes in covered.items():
        if face not in part.sides:
            continue
        mine = rect_on(face, part.side, part.grade_rect(*at), part.tht_rect(*at))
        if mine is None:
            continue
        for m in far_boxes(mine):
            for q in boxes:
                if min(m[perp + 2], q[perp + 2]) - max(m[perp], q[perp]) <= EPS:
                    continue
                out.append((q[ax] - m[ax + 2] - SEAT_GAP_MM, q[ax + 2] - m[ax] + SEAT_GAP_MM))
    return out


def _corners(state, seats) -> Tuple[Dict[str, EdgeSeat], Dict[str, List[str]]]:
    """`(seats, lost)`: each seat's lines less every position where its part
    would overlap what a part seated on ANOTHER edge covers wherever that one
    stands (`_covered`), cut until nothing changes; `lost` names, for a part
    left no position at all, the parts that walled it.

    A seat line was read against the fixed parts alone, so a part seated on
    the adjacent edge near a corner was invisible to it: CM5_MINIMA's J501,
    seated on the south edge, covers x 85.70-98.00 over y 68.8-80.7 wherever
    it stands, and the west edge's J101 and USB701 were still given the line
    up to y 76.0, so the anneal put them there and annealed again. Parts on
    the same edge are `_crowding`'s; a project
    that waives KiCad's courtyard rule (#1101) walls nothing."""
    lines = {r: dict(s.lines) for r, s in seats.items()}
    walled_by: Dict[str, set] = {r: set() for r in seats}
    if not getattr(state, 'courtyards_ignored', False):
        for _ in range(2 * len(seats) + 1):
            covered = {r: _covered(state, seats[r]._replace(lines=lines[r]))
                       for r in seats if lines[r]}
            changed = False
            for a, sa in seats.items():
                for rot in list(lines[a]):
                    line = lines[a][rot]
                    free = line.free
                    for b, cov in covered.items():
                        if b == a or seats[b].edge == sa.edge or not cov:
                            continue
                        cut = _walled(state, sa._replace(lines=lines[a]), rot, cov)
                        less = tuple(p for f0, f1 in free for p in _less(f0, f1, cut))
                        if less != free:
                            walled_by[a].add(b)
                            free = less
                    if free != line.free:
                        changed = True
                        if free:
                            lines[a][rot] = Line(line.coord, free)
                        else:
                            del lines[a][rot]
            if not changed:
                break
    for r, s in seats.items():
        state.parts[r].ensure_rotation(state.parts[r].rot)
    narrowed = {r: s._replace(lines=lines[r]) for r, s in seats.items() if lines[r]}
    lost = {r: sorted(walled_by[r]) for r in seats if not lines[r]}
    return narrowed, lost


def _cornered_reason(seat, who) -> str:
    return (f"no room along the {seat.edge} edge: every position its seat line had overlaps "
            f"{', '.join(who)}, seated on another edge, wherever that one stands; declare "
            f"one of them on another edge or drop a declaration")


def _crowding(state, out, order) -> List[dict]:
    """One record per edge and face whose seated parts (`out`) do not all fit
    there together, each naming the most that do (`seated`) and the rest
    (`unseated`); `order` is the intent's declared order, which breaks a tie.

    A row is the parts seated on one edge on one face (courtyards on two
    faces never meet; through-hole copper across the faces only takes more
    room, so splitting the faces can miss a short row but never invent
    one), packed as `pack` packs it: courtyards SEAT_GAP_MM
    apart along the edge, each part at any turn its seat has and only in
    that turn's free stretches. Every subset is tried exactly -- the least
    far end any order of a subset can reach, each part at its earliest spot
    past the one before -- so a row reported short of room is short at
    every arrangement. Not checked, and left to the anneal and `pack` as
    before: a row longer than ROW_MAX, and a row where two parts at some
    pair of turns do not overlap across the edge (they could then stand
    abreast, which a row cannot say)."""
    rows: Dict[Tuple[str, str], List[str]] = {}
    for ref in order:
        if ref in out:
            rows.setdefault((out[ref].edge, state.parts[ref].side), []).append(ref)
    found = []
    for (edge, side), refs in sorted(rows.items()):
        if not 2 <= len(refs) <= ROW_MAX:
            continue
        axis, perp = out[refs[0]].axis, 1 - out[refs[0]].axis
        # each part, each turn: (rot, courtyard's low end from the origin,
        # its length along the edge, the stretches of its origin, its extent
        # across the edge)
        items = []
        for ref in refs:
            turns = []
            for rot, line in sorted(out[ref].lines.items()):
                c = state.parts[ref].rect(0.0, 0.0, rot)
                turns.append((rot, c[axis], c[axis + 2] - c[axis], line.free,
                              (line.coord + c[perp], line.coord + c[perp + 2])))
            items.append(turns)
        if any(min(a[4][1], b[4][1]) - max(a[4][0], b[4][0]) <= EPS
               for i, ti in enumerate(items) for tj in items[i + 1:]
               for a in ti for b in tj):
            continue
        n = len(items)

        def reach(turns, frontier):
            """The far end of a part's courtyard, plus the gap, at its
            earliest spot past `frontier` (inf when there is none)."""
            best = float('inf')
            for _rot, lo_c, w, free, _across in turns:
                for f0, f1 in free:
                    at = max(frontier, f0 + lo_c)
                    if at <= f1 + lo_c + EPS:
                        best = min(best, at + w + SEAT_GAP_MM)
            return best
        far = [float('inf')] * (1 << n)
        far[0] = float('-inf')
        for s in range(1, 1 << n):
            far[s] = min((reach(items[i], far[s ^ (1 << i)]) for i in range(n)
                          if s >> i & 1 and far[s ^ (1 << i)] < float('inf')),
                         default=float('inf'))
        if far[-1] < float('inf'):
            continue
        # the most that fit together; on a tie, the earlier declared
        keep = min((s for s in range(1, 1 << n) if far[s] < float('inf')),
                   key=lambda s: (-bin(s).count('1'),
                                  [i for i in range(n) if s >> i & 1]))
        parts, spans = [], []
        for i, ref in enumerate(refs):
            turns = []
            for rot, lo_c, w, free, _across in items[i]:
                span = [[round(f0 + lo_c, 2), round(f1 + lo_c + w, 2)] for f0, f1 in free]
                spans += span
                turns.append({'rot': rot, 'length_mm': round(w, 2),
                              'at': [[round(f0, 2), round(f1, 2)] for f0, f1 in free],
                              'span': span})
            parts.append({'ref': ref, 'seated': bool(keep >> i & 1), 'turns': turns})
        merged = []
        for a, b in sorted(spans):
            if merged and a <= merged[-1][1] + EPS:
                merged[-1][1] = max(merged[-1][1], b)
            else:
                merged.append([a, b])
        found.append({'edge': edge, 'side': side, 'axis': 'xy'[axis], 'parts': parts,
                      'free': merged, 'fit': bin(keep).count('1'),
                      'seated': [r for i, r in enumerate(refs) if keep >> i & 1],
                      'unseated': [r for i, r in enumerate(refs) if not keep >> i & 1]})
    return found


def crowding_text(rec) -> str:
    """One `_crowding` record in words: the edge, each part's length along it
    and where it can stand, the free span, what was seated, and what to
    change."""
    ax = rec['axis']

    def at(stretches):
        return ', '.join(f"{a:.2f}-{b:.2f}" for a, b in stretches)
    each = []
    for p in rec['parts']:
        turns = p['turns']
        long_ = '/'.join('%.2f' % t['length_mm'] for t in turns)
        by = '/'.join('%g' % t['rot'] for t in turns)
        each.append(f"{p['ref']} {long_} mm long"
                    + (f" (by turn {by})" if len(turns) > 1 else '')
                    + f", its courtyard within {ax} "
                    + '; '.join(at(t['span']) for t in turns)
                    + f" (origin at {ax} " + '; '.join(at(t['at']) for t in turns) + ")")
    room = sum(b - a for a, b in rec['free'])
    need = sum(min(t['length_mm'] for t in p['turns']) for p in rec['parts'])
    refs = [p['ref'] for p in rec['parts']]
    return (f"the {rec['edge']} edge ({rec['side']} side) cannot seat "
            f"{', '.join(refs[:-1])} and {refs[-1]} together: {'; '.join(each)}; "
            f"{need:.2f} mm of parts, and the free span {ax} {at(rec['free'])} "
            f"({room:.2f} mm) holds at most "
            f"{rec['fit']} of the {len(refs)}. Seated: {', '.join(rec['seated'])} (the most "
            f"that fit, the earlier declared first); placed without its edge claim: "
            f"{', '.join(rec['unseated'])}. This is the intent's to fix, the same in every "
            f"seed: move a connector to another edge, drop a declaration, or widen a band, "
            f"then place again")


def _room(line) -> float:
    return sum(f1 - f0 for f0, f1 in line.free)


def project(seat: EdgeSeat, pose, rot_now=None) -> Tuple[float, float, float]:
    """`pose` moved onto `seat`'s edge: its turn when the seat has that turn
    (else `rot_now`, else the seat's first), the distance from the edge that
    turn's, and its position along the edge in that turn's nearest free
    stretch."""
    x, y, rot = pose
    rot = rot_key(rot)
    if rot not in seat.lines:
        rot = rot_key(rot_now) if rot_now is not None and rot_key(rot_now) in seat.lines \
            else next(iter(seat.lines))
    line = seat.lines[rot]
    a = x if seat.axis == 0 else y
    a = min((min(max(a, f0), f1) for f0, f1 in line.free), key=lambda v: abs(v - a))
    return (a, line.coord, rot) if seat.axis == 0 else (line.coord, a, rot)


def legal_on_edge(state, ref, pose) -> bool:
    """A declared pose's legality (`seeder._fixed_pose_check`) against every
    other part where it is."""
    from .seeder import _fixed_pose_check
    pool = {o: (p.x, p.y, p.rot) for o, p in state.parts.items() if o != ref}
    state.parts[ref].ensure_rotation(pose[2])
    how, _reasons, _conflicts = _fixed_pose_check(state, ref, tuple(pose),
                                                  _near(state, ref, tuple(pose), pool))
    return how is not None


def reseat(state, seat: EdgeSeat) -> Optional[Tuple[float, float, float]]:
    """The legal pose on `seat`'s edge nearest where the part is -- sliding
    out to SLIDE_REACH_MM each way, at its own turn first and then at its
    others -- or None."""
    p = state.parts[seat.ref]
    own = rot_key(p.rot)
    best = None
    for rot in [own] + [r for r in seat.lines if r != own]:
        if rot not in seat.lines:
            continue
        here = project(seat, (p.x, p.y, rot))
        a0 = here[0] if seat.axis == 0 else here[1]
        line = seat.lines[rot]
        tries = [a0] + [a0 + s * k * SLIDE_STEP_MM
                        for k in range(1, int(SLIDE_REACH_MM / SLIDE_STEP_MM) + 1) for s in (1, -1)]
        tries += [f for pair in line.free for f in pair]      # a stretch narrower than a step
        for a in sorted(set(tries), key=lambda v: abs(v - a0)):
            if best is not None and abs(a - a0) >= best[0]:
                break
            if not any(f0 - 1e-9 <= a <= f1 + 1e-9 for f0, f1 in line.free):
                continue
            pose = (a, line.coord, rot) if seat.axis == 0 else (line.coord, a, rot)
            if legal_on_edge(state, seat.ref, pose):
                best = (abs(a - a0), pose)
                break
        if best is not None and best[0] == 0.0:
            break
    p.ensure_rotation(p.rot)
    return best[1] if best else None


def pack(state, seats: Dict[str, EdgeSeat]) -> List[str]:
    """Close the overlaps between parts seated on one edge, keeping their
    order along it: each edge's row of seated parts is laid out at the least
    squared displacement from where they are with SEAT_GAP_MM between
    courtyards (clusters of touching parts placed at their mean, the
    Abacus row legaliser), each held to its own free stretch. A row that
    does not fit at its parts' turns takes the narrowest turn of its widest
    part, as often as that helps. A part seated on another edge (at a
    corner) is a wall the row keeps clear of. A row that cannot fit is left
    as it was. Returns the parts it moved by more than a micrometre or
    turned."""
    moved: List[str] = []
    by_edge: Dict[str, List[str]] = {}
    for r, s in seats.items():
        by_edge.setdefault(s.edge, []).append(r)
    for edge, refs in sorted(by_edge.items()):
        turns = {r: rot_key(state.parts[r].rot) for r in refs}
        for _ in range(len(refs) + 1):
            got = _pack_row(state, seats, refs, turns,
                            [r for r in seats if seats[r].edge != edge])
            if got is not None:
                break
            # narrow the widest part that has a narrower turn
            best = None
            for r in refs:
                w = _width(state, seats[r], turns[r])
                alt = min(seats[r].lines, key=lambda q: _width(state, seats[r], q))
                if _width(state, seats[r], alt) < w - EPS and (best is None or w > best[0]):
                    best = (w, r, alt)
            if best is None:
                break
            turns[best[1]] = best[2]
        if got is None:
            continue
        for r, pose in got.items():
            p = state.parts[r]
            step = max(abs(pose[0] - p.x), abs(pose[1] - p.y))
            turned = rot_key(pose[2]) != rot_key(p.rot)
            if step > 1e-9 or turned:
                p.ensure_rotation(pose[2])
                state.apply_move(r, *pose)
                if step > 1e-3 or turned:       # not a gap trued to SEAT_GAP_MM
                    moved.append(r)
    return moved


def _width(state, seat, rot) -> float:
    c = state.parts[seat.ref].rect(0.0, 0.0, rot)
    return c[seat.axis + 2] - c[seat.axis]


def _pack_row(state, seats, refs, turns, walls=()):
    """`{ref: pose}` for one edge's row at `turns`, or None when it does not
    fit. Positions are the courtyards' low ends along the edge; each part's
    range is its free stretch less where a part of `walls` (seated on
    another edge) stands across its line."""
    items = []
    for r in refs:
        s, rot = seats[r], turns[r]
        state.parts[r].ensure_rotation(rot)
        line = s.lines[rot]
        c = state.parts[r].rect(0.0, 0.0, rot)
        lo_c, w = c[s.axis], c[s.axis + 2] - c[s.axis]
        p = state.parts[r]
        a = project(s, (p.x, p.y, rot), rot)
        a = a[0] if s.axis == 0 else a[1]
        perp = 1 - s.axis
        p0, p1 = line.coord + c[perp], line.coord + c[perp + 2]
        blocked = []
        for o in walls:
            q = state.parts[o].rect()
            if q[perp] < p1 + SEAT_GAP_MM and p0 < q[perp + 2] + SEAT_GAP_MM:
                blocked.append((q[s.axis] - w - SEAT_GAP_MM, q[s.axis + 2] + SEAT_GAP_MM))
        pieces = []
        for f0, f1 in line.free:
            pieces += _less(f0 + lo_c, f1 + lo_c, blocked)
        if not pieces:
            pieces = [(f0 + lo_c, f1 + lo_c) for f0, f1 in line.free]
        want = a + lo_c
        f0, f1 = min(pieces, key=lambda f: 0.0 if f[0] <= want <= f[1]
                     else min(abs(want - f[0]), abs(want - f[1])))
        items.append([want, w, f0, f1, r, lo_c])
    items.sort(key=lambda t: (t[0] + t[1] / 2.0, t[4]))
    # Abacus over the row: a cluster's members abut at SEAT_GAP_MM; its
    # position is their mean desired position, held to every member's stretch
    clusters = []       # [sum of (want - offset), n, lo, hi, members, width]
    for k, (want, w, lo, hi, r, _lo_c) in enumerate(items):
        clusters.append([want, 1, lo, hi, [k], w + SEAT_GAP_MM])
        while True:
            cl = clusters[-1]
            x = _cluster_x(cl)
            if x is None:
                return None
            if len(clusters) < 2:
                break
            prev = clusters[-2]
            px = _cluster_x(prev)
            if px is None:
                return None
            if px + prev[5] <= x + EPS:
                break
            # merge: the later members sit at offsets after the earlier ones
            off = prev[5]
            prev[0] += cl[0] - cl[1] * off
            prev[1] += cl[1]
            prev[2] = max(prev[2], cl[2] - off)
            prev[3] = min(prev[3], cl[3] - off)
            prev[4] += cl[4]
            prev[5] += cl[5]
            clusters.pop()
    out = {}
    for cl in clusters:
        x = _cluster_x(cl)
        off = 0.0
        for k in cl[4]:
            want, w, _lo, _hi, r, lo_c = items[k]
            s = seats[r]
            a = x + off - lo_c
            line = s.lines[turns[r]]
            out[r] = (a, line.coord, turns[r]) if s.axis == 0 else (line.coord, a, turns[r])
            off += w + SEAT_GAP_MM
    return out


def _less(lo, hi, blocked):
    """[lo, hi] less the open intervals `blocked`, as closed pieces."""
    pieces = [(lo, hi)]
    for b0, b1 in blocked:
        nxt = []
        for a0, a1 in pieces:
            if b1 <= a0 or b0 >= a1:
                nxt.append((a0, a1))
                continue
            if a0 <= b0:
                nxt.append((a0, b0))
            if b1 <= a1:
                nxt.append((b1, a1))
        pieces = nxt
    return pieces


def _cluster_x(cl):
    """A cluster's low end: its members' mean desired position, held inside
    the stretch every member allows; None when they allow none."""
    lo, hi = cl[2], cl[3]
    if lo > hi + EPS:
        return None
    return min(max(cl[0] / cl[1], lo), hi)
