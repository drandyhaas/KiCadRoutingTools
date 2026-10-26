"""Seat legality, wiring targets, fixed-pose checks and lock stamps.

What is left of the intent seeder once the simple path (`place_simple.py`)
replaced its placement stages, kept because the simple path and the tools
around it use these pieces:

  * `pose_ok`: THE seat predicate -- fully contained (courtyard and, where it
    reaches past the courtyard, pad copper), clear of the intent's keep-outs
    and exclusive zones, and clear of every neighbour (`candidate_valid`).
    `simple.legal` and `nearest_legal` call it to hold the annealed placement
    to exact legality;
  * `_wiring_target`: where a part adds the least wire -- each net one vote
    at its nearest placed pad, combined by geometric median (`simple.target`,
    the anneal's jump move);
  * `fixed_pose_pair_conflicts` and `_fixed_pose_check`: declared fixed poses
    against each other (`floorplan.plan_check`), with `_materialise_rotation`
    filling a declared angle's rotation caches;
  * `stamp_locked` / `stamp_unlocked`: KiCad `(locked yes)` on a written
    board (`place_pose lock` / `unlock`).
"""
from __future__ import annotations

import math
import re
from typing import Dict, List, Optional, Sequence, Set, Tuple


def pose_ok(state, ref: str, x: float, y: float, rot: float,
            exclude: Set[str]) -> bool:
    """THE seat predicate: fully contained, and clear of everything not in
    `exclude`.

    ONE predicate, so every caller that asks "may this part sit here"
    (`simple.legal`, `nearest_legal`, the pose tools) gets the same answer.

    Note `candidate_valid` alone is not enough: for a part whose incumbent
    pose is off the board, its #456 branch accepts poses that move strictly
    TOWARD the board while still outside it. Placement from scratch has no
    incumbent worth improving on, so full containment is demanded explicitly.

    The third conjunct is the intent's declared KEEP-OUTS (#701), without
    which `rule_keepout` would grade a region the search walked parts into.
    It sits BEFORE `candidate_valid` because it is a handful of float
    compares against a usually-empty tuple, where `candidate_valid` ends in
    the neighbour loop.

    Containment is of the COURTYARD and, since #1257, of the PAD COPPER too,
    for a part whose copper reaches past its own courtyard by more than the
    edge margin the courtyard is held to: there a contained courtyard proves
    nothing about the pads, and a search could seat a part with a pad 4 mm
    east of its courtyard past the outline -- the
    top-priority placement defect, which check_assembly gates on. The copper
    is judged by check_assembly's own gate (`legality.pad_copper_gating_mm`),
    so a castellated module on its edge stays exempt as it is there. Every
    other part skips it (`_pad_copper_overhang`): its copper lies within the
    margin of a courtyard the outline test just passed, which is on the board.
    """
    part = state.parts[ref]
    # The grade ladder (#1182): board, keep-out and exclusive zone are the
    # floorplan grade's questions; `candidate_valid` asks the neighbours.
    r, tht = part.grade_rects(x, y, rot)
    # `BOARD_TOL`: rounding, the same allowance candidate_valid's own board
    # tests make, so the two never split on a courtyard flush with the edge.
    from .legality import BOARD_TOL
    if state.edge_gate.rect_outside_amount(r) > BOARD_TOL:
        return False
    # #1257: the pad copper, for the few parts the courtyard test above cannot
    # vouch for -- a cached float compare for every other one. 1e-6 is the
    # gate's own threshold (`legality.EPS`, check_assembly's `d > EPS`).
    if (_pad_copper_overhang(state, ref) > state.edge_gate.margin
            and _pad_copper_off_board(state, ref, x, y, rot) > 1e-6):
        return False
    # #962: and the copper the footprint draws (a tab, an antenna), for the
    # parts whose drawn copper reaches past the courtyard by more than the
    # margin -- the same cached compare, then the grade's own measure.
    if (_graphic_copper_overhang(state, ref) > state.edge_gate.margin
            and _graphic_copper_off_board(state, ref, x, y, rot) > 1e-6):
        return False
    # BOTH rects, because `rule_keepout` grades both: a through-hole part's
    # leads pass through a keep-out even when its body sits on the far side,
    # so a courtyard-only seat would be accepted here and flagged there --
    # exit 4 on a board this function placed correctly.
    # ABSOLUTE, via the state's shared loop. `candidate_valid` asks the same
    # loop with a MONOTONE policy (#702), which this predicate must not
    # inherit: placing from scratch has no incumbent worth improving on, and
    # "no worse than where you already are" would admit a part whose current
    # pose is fully inside a keep-out. One loop, two policies, both named.
    if not state.keepout_clear(ref, (r, tht)):
        return False
    # #797, the FOURTH conjunct: without it a search could seat an unrelated
    # part in the middle of a region the intent reserved, and
    # `rule_zone_exclusive` would then fail the board against the intent it
    # was built from. The keep-out conjunct above, one rule over.
    #
    # ABSOLUTE, for the reason the paragraph above gives, and BEFORE
    # `candidate_valid` for the reason the keep-out one is: a handful of float
    # compares against a usually-empty dict, where `candidate_valid` ends in
    # the neighbour loop.
    #
    # BOTH rects are handed over and the answer is nonetheless COURTYARD ONLY.
    # `intent_term_values` takes `rects[0]` for a zone_exclusive term because
    # `rule_zone_exclusive` reads `part.rect` and never `tht_rect`; passing
    # `(r,)` here would put a SECOND copy of that decision in this file, and
    # the two would drift the first time one of them was revisited.
    if not state.exclusive_clear(ref, (r, tht)):
        return False
    return state.candidate_valid(ref, x, y, rot, exclude=exclude)


def _pad_copper_overhang(state, ref: str) -> float:
    """How far `ref`'s pads can reach past its grade rect (the courtyard,
    else the pad bbox), in mm, at ANY pose; 0.0 when they cannot (#1257).
    Cached per part, because `pose_ok` is the seat search's inner loop.

    Measured once, in the footprint's own frame: the pad box
    (`_Part.padbox_local`, every pad at its full extent) against the grade
    rect at rotation 0, as the DIAGONAL of the two axes' overhangs, which
    bounds every pad point's distance from that rect. A turn moves both
    together, so the bound holds at every angle the search tries.

    Why that settles most parts: `pose_ok` has just held the courtyard inside
    the outline at `edge_gate.margin`, so no ring edge comes within the
    margin of it and everything within the margin is board. Copper reaching
    no further than that is on the board; only a part reaching further needs
    `_pad_copper_off_board`. (At a zero margin the courtyard test cannot see
    a notch crossing a courtyard either; this inherits that, not more.)

    Three readers ask the pair in this order: `pose_ok`, `_fixed_pose_check`
    on a contained pose, and the quench's `candidate_valid` (through this
    module, `QuenchState._pad_copper_past`), each after its own courtyard
    test at the same margin.
    """
    cache = getattr(state, '_pad_copper_overhang_cache', None)
    if cache is None:
        cache = state._pad_copper_overhang_cache = {}
    over = cache.get(ref)
    if over is None:
        part = state.parts[ref]
        box = part.padbox_local
        over = 0.0
        if box is not None:
            g = part.grade_by_rot[0.0]
            over = math.hypot(max(0.0, g[0] - box[0], box[2] - g[2]),
                              max(0.0, g[1] - box[1], box[3] - g[3]))
        cache[ref] = over
    return over


def _pad_copper_off_board(state, ref: str, x: float, y: float,
                          rot: float) -> float:
    """How far `ref`'s pad copper reaches past the board outline at
    `(x, y, rot)` by check_assembly's gate (`legality.pad_copper_gating_mm`,
    zero margin, an edge-straddling castellated pad exempt); 0.0 when it
    would not gate there (#1257). The part's `PartPads` is the state's own
    when it carries a legality context, else built once and cached."""
    from . import legality
    pads = getattr(state, '_pad_copper_pads_cache', None)
    if pads is None:
        pads = state._pad_copper_pads_cache = {}
    pp = pads.get(ref)
    fp = state.pcb_data.footprints[ref]
    if pp is None:
        ctx = state.legality_ctx
        pp = ctx.parts.get(ref) if ctx is not None else None
        if pp is None:
            pp = legality.PartPads(fp, state.clearance)
        pads[ref] = pp
    return legality.pad_copper_gating_mm(pp, fp, (x, y, rot),
                                         _zero_edge_gate(state))


def _graphic_copper_shapes(state, ref: str) -> list:
    """`ref`'s own graphic copper shapes (#962: a drawn tab, an antenna, a
    jumper bridge), as `check_drc.graphic_copper_shapes` groups them, at the
    pose the board was parsed at. The board's shapes are grouped once per
    state and cached by owner; [] for the parts that draw none."""
    by_owner = getattr(state, '_graphic_copper_shapes_cache', None)
    if by_owner is None:
        from check_drc import graphic_copper_shapes
        by_owner = {}
        for shape in graphic_copper_shapes(state.pcb_data):
            if shape['owner']:
                by_owner.setdefault(shape['owner'], []).append(shape)
        state._graphic_copper_shapes_cache = by_owner
    return by_owner.get(ref, [])


def _graphic_copper_overhang(state, ref: str) -> float:
    """How far `ref`'s own graphic copper can reach past its grade rect at
    ANY pose, in mm; 0.0 when it cannot. `_pad_copper_overhang`'s bound for
    the copper a footprint draws rather than its pads: measured once in the
    footprint's own frame (the parse pose's `_global_to_local`, the frame
    `padbox_local` is in), every vertex and circle grown by half its drawn
    stroke, as the diagonal of the two axes' overhangs. Cached per part.

    Without it, the seat gate held only courtyards and pads to the outline,
    so on esp_prog `place_pose set U2 ... --snap` ranked six poses with U2's
    F.Cu tab past the edge ahead of any other, re-graded them all and
    refused, though a clean pose sat 1.36 mm away."""
    cache = getattr(state, '_graphic_copper_overhang_cache', None)
    if cache is None:
        cache = state._graphic_copper_overhang_cache = {}
    over = cache.get(ref)
    if over is None:
        over = 0.0
        fp = state.pcb_data.footprints.get(ref)
        shapes = _graphic_copper_shapes(state, ref)
        if shapes and fp is not None and getattr(fp, 'parsed_pose', None):
            from kicad_parser import _global_to_local
            x0, y0, r0, _layer = fp.parsed_pose
            lo = [math.inf, math.inf, -math.inf, -math.inf]
            for shape in shapes:
                for sg in shape['segs']:
                    half = (getattr(sg, 'drawn_width', None) or sg.width or 0.0) / 2.0
                    pts = [(sg.start_x, sg.start_y, half), (sg.end_x, sg.end_y, half)]
                    circ = getattr(sg, 'graphic_circle', None)
                    if circ is not None:
                        pts.append((circ[0], circ[1], circ[2] + half))
                    for gx, gy, grow in pts:
                        lx, ly = _global_to_local(x0, y0, r0, gx, gy)
                        lo = [min(lo[0], lx - grow), min(lo[1], ly - grow),
                              max(lo[2], lx + grow), max(lo[3], ly + grow)]
            g = state.parts[ref].grade_by_rot[0.0]
            over = math.hypot(max(0.0, g[0] - lo[0], lo[2] - g[2]),
                              max(0.0, g[1] - lo[1], lo[3] - g[3]))
        cache[ref] = over
    return over


def _graphic_copper_off_board(state, ref: str, x: float, y: float,
                              rot: float) -> float:
    """How far `ref`'s own graphic copper reaches past the board outline at
    `(x, y, rot)`, by the grade's own measure (`check_drc.
    graphic_outline_overrun` at margin 0, what `grade_pad_legality`'s
    `oob_graphic_copper_*` and place_pose gate on), the milled rings the
    part's pads sit in at its seed pose exempt as the census exempts them;
    0.0 when it stays on the board. Also 0.0 where it cannot be measured --
    no parse pose, a side changed since parse, or the board-outline owner
    the census waives -- and the grade still sees those."""
    from check_drc import (GRAPHIC_WAIVED_STATES, _graphic_outline_geometry,
                           _reposed_shape, graphic_outline_overrun,
                           graphic_owner_state, milled_rings_enclosing)
    from types import SimpleNamespace
    fps = state.pcb_data.footprints
    fp = fps.get(ref)
    shapes = _graphic_copper_shapes(state, ref)
    if (not shapes or fp is None or not getattr(fp, 'parsed_pose', None)
            or graphic_owner_state(ref, fps) in GRAPHIC_WAIVED_STATES):
        return 0.0
    bi = state.pcb_data.board_info
    geom = getattr(state, '_graphic_outline_geom', None)
    if geom is None:
        geom = state._graphic_outline_geom = _graphic_outline_geometry(bi)
    owned_cache = getattr(state, '_graphic_owned_rings', None)
    if owned_cache is None:
        owned_cache = state._graphic_owned_rings = {}
    owned = owned_cache.get(ref)
    if owned is None:
        pts = [(p.global_x, p.global_y) for p in fp.pads]
        owned = owned_cache[ref] = (milled_rings_enclosing(geom[3], pts)
                                    if pts else frozenset())
    probe = SimpleNamespace(parsed_pose=fp.parsed_pose, x=x, y=y, rotation=rot,
                            layer=fp.layer)
    worst = 0.0
    for shape in shapes:
        moved = _reposed_shape(shape, probe)
        if moved is False:
            return 0.0
        ov, sg, _pt = graphic_outline_overrun(moved or shape, bi, owned, _geom=geom)
        if sg is not None:
            worst = max(worst, ov)
    return worst


def _materialise_rotation(part, rot: float) -> float:
    """Make `part.bounds_by_rot` (and `tht_by_rot`) hold an entry for `rot`.

    Returns the normalised angle, which is the key `_Part.rect` looks up.

    A DECLARED angle need not lie on the part's 90-degree lattice, and
    `_Part.rect` silently falls back to `bounds_by_rot[0.0]` for an angle it
    has no entry for -- so a declared 45 on a part seeded at 0 would be judged
    for overlap, halo and containment on the UNROTATED box and then written
    out at 45. Every seat search must materialise the entry before using it.

    KEYED BY THE NORMALISED ANGLE, which is what `_Part.rect` looks up
    (`bounds_by_rot.get(rot % 360)`): a part whose board spells its rotation
    -45 would otherwise get an entry at -45 that `rect` never reads. Declared
    angles reach here already normalised (`floorplan._rotation`).
    """
    rot = rot % 360.0
    part.ensure_rotation(rot)
    return rot


def _zero_edge_gate(state):
    """The board outline at ZERO margin, built once per state: the gate a
    containment question (is this copper on the board?) is asked of. The
    state's own `edge_gate` carries the board-edge margin, which is the
    clearance question instead."""
    zero = getattr(state, '_zero_edge_gate', None)
    if zero is None:
        from .legality import BoardOutlineGate
        zero = BoardOutlineGate(state.pcb_data.board_info, 0.0)
        state._zero_edge_gate = zero
    return zero


def _nearest_pad(pts: List[Tuple[float, float]], x: float, y: float) -> Tuple[float, float]:
    """The point of `pts` nearest `(x, y)`."""
    return min(pts, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)


def _wiring_target(state, ref: str, placed: Set[str],
                      max_fanout: int = 20) -> Optional[Tuple[float, float]]:
    """Where `ref` adds the least wire to its nets: each net is priced by
    its MINIMUM SPANNING TREE, not by its connections one by one.

    Joining a net whose other pads are placed lengthens its pad tree by the
    distance to the nearest of them (Prim's step), so the target minimises
    the sum of those distances over `ref`'s nets: the geometric median of
    each net's nearest placed pad (Weiszfeld, re-choosing the nearest pad as
    it moves), started from the least-squares point, which picks the middle
    of a flat valley (a series part costs the same anywhere between its two
    ends). So each NET is one vote, whatever its part count: voting once per
    (partner, net), the earlier rule, let a net on six parts outvote a
    two-pin net 5:1 and pulled the part into the middle of the big net, where
    no partner is. Duplicated pins (a USB-C receptacle's A6/B6) are two pads
    of one net and cast no extra vote.

    The nearest PAD, not the nearest point of the net's airwires: the polish
    prices a net by its pads' tree and its crossings, and a part aimed at
    the middle of another net's airwire (a track could tee in there) was
    seated between its partners with its own airwires across the bundle --
    bread_dcmotor's motor driver, 130 -> 168 crossings. Nets owned by more
    than `max_fanout` parts are excluded for the routability.py reason: they
    reach everywhere by design. Plane nets are NOT otherwise excluded here --
    for a decap, the rail net is exactly what tethers it to its IC."""
    part = state.parts.get(ref)
    if part is None:
        return None
    votes = []
    for nid in sorted(part.nets):
        owners = state.net_refs.get(nid, ())
        if len(owners) > max_fanout:
            continue
        pts = [(gx, gy) for other in sorted(owners)
               if other != ref and other in placed
               for gx, gy, pn in state.parts[other].pad_globals() if pn == nid]
        if pts:
            votes.append((pts,
                          (sum(p[0] for p in pts) / len(pts),
                           sum(p[1] for p in pts) / len(pts))))
    if not votes:
        return None
    tx = sum(c[0] for _s, c in votes) / len(votes)
    ty = sum(c[1] for _s, c in votes) / len(votes)
    # least squares: each net's nearest pad, averaged
    for _ in range(20):
        near = [_nearest_pad(s, tx, ty) for s, _c in votes]
        nx = sum(p[0] for p in near) / len(near)
        ny = sum(p[1] for p in near) / len(near)
        moved = math.hypot(nx - tx, ny - ty)
        tx, ty = nx, ny
        if moved < 1e-3:
            break
    # the sum of distances (Weiszfeld), from there
    for _ in range(50):
        near = [_nearest_pad(s, tx, ty) for s, _c in votes]
        wsum = sx = sy = rx = ry = 0.0
        at = 0
        for px, py in near:
            d = math.hypot(px - tx, py - ty)
            if d < 1e-6:
                at += 1
                continue
            wsum += 1.0 / d
            sx += px / d
            sy += py / d
            rx += (px - tx) / d
            ry += (py - ty) / d
        # optimal: no vote left to pull, or the votes sitting AT the point
        # outweigh the others' pull (the test at a data point)
        if wsum <= 0.0 or (at and math.hypot(rx, ry) <= at):
            break
        nx, ny = sx / wsum, sy / wsum
        cost_now = sum(math.hypot(px - tx, py - ty) for px, py in near)
        cost_new = sum(math.hypot(px - nx, py - ny) for px, py in
                       (_nearest_pad(s, nx, ny) for s, _c in votes))
        if cost_new > cost_now - 1e-9:
            break
        moved = math.hypot(nx - tx, ny - ty)
        tx, ty = nx, ny
        if moved < 1e-3:
            break
    return tx, ty


def _holes_outside(state, gate, ref: str, x: float, y: float,
                   rot: float) -> Tuple[int, int]:
    """`(holes, outside)`: how many drill holes (plated and NPTH) `ref` has,
    and how many leave `gate`'s outline at (x, y, rot), on the hole itself
    (`pad_drill_capsule`, a circle or a milled slot, sampled), so a hole
    straddling the edge counts and one whose bounding square alone crosses a
    curved edge does not. A castellated pad's half-hole is exempt while it
    straddles the outline, as check_assembly's copper gate exempts its pad
    (`legality.pad_copper_overrun_mm`; #1297: rp2350's Teensy U8, 33)."""
    from kicad_parser import pad_drill_capsule
    from .legality import _on_board, _pad_has_no_copper, footprint_at_pose
    fp = (state.pcb_data.footprints or {}).get(ref)
    if fp is None:
        return 0, 0
    n = out = 0
    for p in footprint_at_pose(fp, (x, y, rot)).pads:
        if float(getattr(p, 'drill', 0) or 0) <= 0:
            continue
        n += 1
        (x1, y1), (x2, y2), r = pad_drill_capsule(p)
        pts = [(cx + r * math.cos(k * math.pi / 8), cy + r * math.sin(k * math.pi / 8))
               for cx, cy in {(x1, y1), (x2, y2)} for k in range(16)]
        # A castellated half-hole, or a copper-less NPTH drill (a mouse-bite
        # perforation, a tooling hole cut into the edge): its centre lies ON
        # the outline by design, so it straddles it while still touching the
        # board (#1413). One verdict for every such drill: the same 0.5 mm hole
        # on a slot's contour (a milled ring, on-board inside) and on the outer
        # edge used to be judged differently. A hole wholly off the board is
        # still past it.
        if (getattr(p, 'castellated', False)
                or (getattr(p, 'pad_type', '') == 'np_thru_hole'
                    and _pad_has_no_copper(p))) and any(
                _on_board(gate, px, py) for px, py in pts):
            continue
        if gate.points_overrun_mm(pts) > 1e-9:
            out += 1
    return n, out


def _exact_pad_conflict(state, ctx, a: str, pose_a, b: str, pose_b):
    """`(hit, mm)`: do `a`'s and `b`'s pads at their poses fall short of each
    pair's own requirement on check_drc's exact pad geometry -- the census
    `grade_pad_legality` re-verifies the seat gate's rect gaps with
    (`legality.pad_pair_conflict`). The rect gap over-reads round pads:
    glasgow's FID1 read 0.25 mm short of MK1 with the copper 0.842 mm apart
    against 0.375 required (#1297). `(True, None)` when it cannot be asked,
    so the rect verdict stands."""
    from .legality import footprint_at_pose, pad_pair_conflict
    try:
        from check_drc import check_pad_pad_overlap as exact
    except Exception:                                       # noqa: BLE001
        return True, None
    pa, pb = ctx.parts.get(a), ctx.parts.get(b)
    fps = state.pcb_data.footprints or {}
    if pa is None or pb is None or a not in fps or b not in fps:
        return True, None
    layers = list(getattr(state.pcb_data.board_info, 'copper_layers', None) or [])
    mm, hit, _req, _src = pad_pair_conflict(
        pa, pa.pad_rects(*pose_a), footprint_at_pose(fps[a], tuple(pose_a)).pads,
        pb, pb.pad_rects(*pose_b), footprint_at_pose(fps[b], tuple(pose_b)).pads,
        ctx.clearance, ctx.pad_clearance_model, exact, layers)
    return hit, mm


#: #1054: a courtyard overlap AREA (mm^2) above this is an overlap; below it
#: is float noise on two courtyards that abut. KiCad's `courtyards_overlap`
#: is an intersection test, so abutting courtyards (gap 0) are legal.
FIXED_OVERLAP_EPS_MM2 = 1e-6


def _courtyard_overlap(state, a: str, pose_a, b: str, pose_b):
    """`(area mm^2, w, h)` of the courtyard overlap of `a` at `pose_a` with
    `b` at `pose_b`. The VERDICT is `legality.pair_overlap_area` -- the
    side-aware measure -- on each part's DRAWN courtyard, the way KiCad
    judges a declared pose (`grade_rect`), and where those overlap,
    `legality.pair_overlap_area_exact` on the drawn outlines (#1094:
    StickHub's declared -135 degree human poses were refused on rects
    alone). A part that draws NO courtyard is screened on the search's own
    rects: on its drawn-body occupancy, at this screen's 1e-6 mm2, it refused
    human poses check_assembly calls advisory (phase-4 verifier: esp_prog's
    R1, U2 and CON2 at their own file poses, One-Air-Max's C53/U8 at 0.135
    mm2) -- the courtyard-blocking floors and the silk exemption are the
    grader's, not this screen's. `w` x `h` is the rects' intersection, for
    the refusal's text only.

    #1300: a part the state seats on an ESTIMATED courtyard is screened on
    the estimate when the pose was SEARCHED (an edge seat against the parts
    round it), and on its pad box, as before, when both parts stand where
    they were declared or locked: a declared arrangement is judged as KiCad
    judges it, never refused on an estimate KiCad does not draw."""
    from .legality import (graded_part_at_pose, pair_overlap_area,
                           pair_overlap_area_exact)
    if getattr(state, 'courtyards_ignored', False):
        # #1101: the project waives KiCad's courtyard rule, so a declared
        # pose is not refused for one (its drill holes still are, below).
        return 0.0, 0.0, 0.0
    pa, pb = state.parts[a], state.parts[b]
    cache = state.__dict__.setdefault('_exact_overlap_cache', {})
    pcb_file = getattr(state, 'pcb_file', None)
    ta, tb = pa.tht_rect(*pose_a), pb.tht_rect(*pose_b)
    if pa.locked and pb.locked:
        ra, rb = pa.drawn_rect(*pose_a), pb.drawn_rect(*pose_b)
    else:
        ra, rb = pa.grade_rect(*pose_a), pb.grade_rect(*pose_b)
    area = pair_overlap_area(pa.sides, pa.side, ra, ta,
                             pb.sides, pb.side, rb, tb)
    if area > FIXED_OVERLAP_EPS_MM2:
        ga = graded_part_at_pose(state.pcb_data, a, pose_a, pa.side, ra, ta,
                                 ta is not None, pcb_file, cache)
        gb = graded_part_at_pose(state.pcb_data, b, pose_b, pb.side, rb, tb,
                                 tb is not None, pcb_file, cache)
        if ga.poly is not None or gb.poly is not None:
            area = pair_overlap_area_exact(ga, gb)
    w = max(0.0, min(ra[2], rb[2]) - max(ra[0], rb[0]))
    h = max(0.0, min(ra[3], rb[3]) - max(ra[1], rb[1]))
    return area, w, h


def _drill_conflict(state, a: str, pose_a, b: str, pose_b) -> Optional[str]:
    """The closest pair of DRILL holes of `a` and `b` (plated or not) at their
    poses, as a refusal, when they are closer than the board's own
    `min_hole_to_hole` (else touching). None when clear.

    The courtyard branch of `_fixed_pose_check` was the only thing that
    caught two holes stacked on each other -- `pair_shortfall` checks a hole
    against the other part's COPPER, never against its hole -- so a courtyard
    waiver must re-ask it (#1060, phase-4 verifier: two coincident NPTH drills
    seated under `accept_courtyard_overlap`)."""
    from .legality import footprint_at_pose

    def drills(ref, pose):
        fp = (state.pcb_data.footprints or {}).get(ref)
        if fp is None:
            return []
        out = []
        for p in footprint_at_pose(fp, pose).pads:
            d = max(float(getattr(p, 'drill', 0) or 0),
                    float(getattr(p, 'drill_w', 0) or 0),
                    float(getattr(p, 'drill_h', 0) or 0))
            if d <= 0:
                continue
            hx = p.hole_x if getattr(p, 'hole_x', None) is not None \
                else p.global_x
            hy = p.hole_y if getattr(p, 'hole_y', None) is not None \
                else p.global_y
            out.append((hx, hy, d / 2.0, str(p.pad_number)))
        return out
    da, db = drills(a, pose_a), drills(b, pose_b)
    if not da or not db:
        return None
    # Read once per state: the waived-courtyard seat (#1101) calls this per
    # candidate pose, and the constraint is a file read.
    floor = getattr(state, '_h2h_floor', None)
    if floor is None:
        floor = 0.0
        try:
            from list_nets import board_constraint
            floor = float(board_constraint(state.pcb_file,
                                           'min_hole_to_hole') or 0.0)
        except Exception:                                  # noqa: BLE001
            floor = 0.0
        try:
            state._h2h_floor = floor
        except Exception:                                  # noqa: BLE001
            pass
    worst = None
    for ax, ay, ar, an in da:
        for bx, by, br, bn in db:
            d = math.hypot(ax - bx, ay - by)
            if d < 1e-3:
                # Holes at one centre are KiCad's `holes_co_located` (a
                # warning), not a hole-to-hole violation: one module drawn as
                # two footprints, CM5's Module301/302 (#1297).
                continue
            gap = d - ar - br
            if gap < floor - 1e-6 and (worst is None or gap < worst[0]):
                worst = (gap, an, bn)
    if worst is None:
        return None
    def name(ref, num):
        return f"{ref}.{num}" if num else f"{ref}'s hole"
    return (f"drill {name(a, worst[1])} is {worst[0]:.3f}mm from "
            f"{name(b, worst[2])} (hole-to-hole floor {floor:g}mm; a negative "
            f"gap is holes overlapping)")


def _fixed_pose_check(state, ref: str, pose, obstacles: Dict[str, Tuple],
                      waived=frozenset(),
                      waived_out: Optional[Dict[str, Dict]] = None
                      ) -> Tuple[Optional[str], List[str], Dict[str, str]]:
    """`(how, reasons, conflicts)` for seating `ref` EXACTLY at `pose`.

    A CHECK, never a search (#1054): the pose is a mechanical fact, so a
    pose that fails is refused with its reasons and never nudged.

    `obstacles` is `{ref: pose}`: every part already placed, at its pose,
    AND every other declared fixed pose at ITS declared pose -- so two
    declarations are judged against each other symmetrically, whatever
    order they are seated in. `conflicts` is `{other: reason}` for the
    obstacles this pose collides with, so the caller can refuse both halves
    of a clash between two declarations.

    COURTYARDS, the way KiCad judges them: an OVERLAP (area above
    `FIXED_OVERLAP_EPS_MM2`, `legality.pair_overlap_area`) is illegal;
    courtyards that ABUT (gap >= 0) are legal. Only for a DECLARED pose:
    every searched seat keeps the board clearance. A human layout packs
    courtyards edge to edge (glasgow's RN banks and SN74LVC1T45 buffers: 14
    pairs at exactly 0.000mm, which kicad-cli's DRC accepts) and a fixed
    pose exists to express THAT arrangement -- held to a searched seat's
    floor, 15 of 30 human glasgow poses were refused, each "within 0.02mm".

    Everything else at its normal rule, ABSOLUTE (a declared pose has no
    incumbent to be "no worse than"): the outline (containment at the
    board-edge margin, and since #1257 the pad copper of a part whose pads
    reach past its courtyard), declared keep-outs and exclusive zones, #1031's
    rule-area keep-out band (`legality_ctx.keepout_amount`), and to every
    obstacle `legality_ctx.pair_shortfall`'s pad clearance, hole clearance,
    pad short and cross-part pad stack -- `pads_ok`'s conjuncts, called.

    `waived` (#1060) is a set of unordered pairs `frozenset({a, b})` whose
    COURTYARD overlap is declared -- `fixed_poses[].accept_courtyard_overlap`
    and `overlap_waivers[]`. For such a pair the courtyard branch alone is
    skipped, and the measurement is recorded in `waived_out[other]` instead
    of refusing; pad clearance, pad shorts, hole clearance, the keep-out band
    and the outline stay absolute, because a waiver is a claim about two
    courtyards and nothing else.

    * `'contained'`: inside the outline, and nothing above fails.
    * `'overhang'`: the courtyard leaves the outline -- a connector or a
      mounting part may overhang by design, which is stage 1's exemption --
      accepted only when every pad's copper is on the board at zero margin
      and every drill hole (NPTH included) too, and a part with neither is
      judged by its courtyard at zero margin (the pad test is vacuous for a
      part with no copper).

    `how` None means refused; `reasons` then says why, with the measurement.
    """
    part = state.parts[ref]
    x, y, rot = pose
    reasons: List[str] = []
    conflicts: Dict[str, str] = {}
    r, tht = part.grade_rects(x, y, rot)
    outside = state.edge_gate.rect_outside_amount(r) > 1e-9
    reasons.extend(f"keep-out {n!r}"
                   for n in state.keepout_blockers(ref, (r, tht)))
    reasons.extend(f"exclusive zone {n!r}"
                   for n in state.exclusive_blockers(ref, (r, tht)))
    containers = getattr(state, 'container_refs', ()) or ()
    ctx = state.legality_ctx
    # #1212: a declared pose on a frame's pin is refused like a courtyard
    # overlap -- the frame's rect is skipped below, its pins are not. Judged
    # against the obstacles at THEIR poses, and, like a courtyard overlap,
    # recorded instead of refused when the pair's overlap is declared: the
    # grader waives an intent-declared pin pair (`intent_declared`) too.
    if getattr(state, 'pin_frame_refs', None):
        for _hit in state._pin_hits_at(
                ref, *pose, poses={o: tuple(p) for o, p in obstacles.items()
                                   if o != ref}):
            _other = _hit.b if _hit.a == ref else _hit.a
            if _other not in obstacles:
                continue
            _pins = ', '.join(_hit.pins)
            if frozenset((ref, _other)) in waived:
                if waived_out is not None:
                    waived_out.setdefault(_other, {})['pins'] = list(
                        _hit.pins)
                continue
            conflicts[_other] = (
                f"sits on {_other}'s pin(s) {_pins} (pin_in_courtyard)"
                if _other in state.pin_frame_refs else
                f"pin(s) {_pins} under {_other}'s courtyard "
                f"(pin_in_courtyard)")
    for other in sorted(obstacles):
        if other == ref or other not in state.parts:
            continue
        opose = obstacles[other]
        # #1443: a pair the heights judge -- one part under the other's body
        # -- is judged on them, at zero clearance as the courtyards are here;
        # its holes still bind
        hv = None
        covers = getattr(state, 'covers', None)
        if covers and (ref in covers or other in covers) \
                and ref not in containers and other not in containers:
            o = state.parts[other]
            hv = state.heights.pair(ref, tuple(pose), part.bounds_by_rot[0.0], part.side,
                                    other, tuple(opose), o.bounds_by_rot[0.0], o.side)
        if hv is not None:
            if hv > 0.0 and frozenset((ref, other)) not in waived:
                conflicts[other] = (f"under {other}'s body, or {other} under its, "
                                    f"too low ({hv:.3f}mm2)")
            else:
                _dh = _drill_conflict(state, ref, pose, other, opose)
                if _dh:
                    conflicts[other] = _dh
        elif ref not in containers and other not in containers:
            area, w, h = _courtyard_overlap(state, ref, pose, other, opose)
            if area > FIXED_OVERLAP_EPS_MM2:
                if frozenset((ref, other)) in waived:
                    if waived_out is not None:
                        waived_out.setdefault(other, {}).update(
                            {'area_mm2': round(area, 4),
                             'w_mm': round(w, 3), 'h_mm': round(h, 3)})
                    # The courtyard is waived; its holes are not.
                    _dh = _drill_conflict(state, ref, pose, other, opose)
                    if _dh:
                        conflicts[other] = _dh
                else:
                    conflicts[other] = (f"courtyard overlaps {other} by "
                                        f"{w:.2f}x{h:.2f}mm ({area:.3f}mm2)")
            elif getattr(state, 'courtyards_ignored', False):
                # #1101: the project waives the courtyard, and with it the
                # only branch above that asked about stacked holes; holes are
                # not waived (review: two coincident NPTH seated).
                _dh = _drill_conflict(state, ref, pose, other, opose)
                if _dh:
                    conflicts[other] = _dh
        if ctx is not None:
            sf = ctx.pair_shortfall(ref, other, pose_a=pose, pose_b=opose)
            # `pads_ok`'s conjuncts, ABSOLUTE rather than seed-relative: a
            # declared pose has no incumbent to be "no worse than".
            pad_mm, short = sf.pad, sf.pad_overlap
            if ((pad_mm > 1e-6 or short) and sf.gpad <= 1e-6
                    and not sf.gstack):
                # The gate reads rect gaps; a pose that is a fact is refused
                # only on what the grader confirms on the pads' true copper.
                hit, mm = _exact_pad_conflict(state, ctx, ref, pose, other, opose)
                if not hit:
                    pad_mm, short = 0.0, False
                elif mm is not None:
                    pad_mm = mm
            what = []
            if short:
                what.append(f"pads short {other}'s (different-net copper "
                            f"intersects)")
            elif sf.stack:
                what.append(f"pads stack on {other}'s copper")
            if pad_mm > 1e-6:
                what.append(f"pad clearance to {other} short by "
                            f"{pad_mm:.3f}mm")
            if sf.hole > 1e-6:
                what.append(f"hole clearance to {other} short by "
                            f"{sf.hole:.3f}mm")
            if what:
                conflicts[other] = '; '.join(
                    ([conflicts[other]] if other in conflicts else [])
                    + what)
    reasons.extend(conflicts[o] for o in sorted(conflicts))
    # #1031's rule-area keep-out band, ABSOLUTE (`keepout_ok` is seed-
    # relative, and a pile seed is no licence): pad copper the band forbids
    # at this pose refuses it, measured in the band's own currency. Without
    # it a fixed pose seated test_1031's R2 at (37.6, 15) -- a pose place_pose
    # refuses and grade_pad_legality reports under oob_keepout_copper_refs.
    if ctx is not None:
        ko = ctx.keepout_amount(ref, x, y, rot)
        if ko > 1e-6:
            reasons.append(f"pad copper {ko:.3f}mm into a rule-area "
                           f"keep-out band")
    if outside:
        from .connector_geometry import geometry_for, pad_boxes
        from .legality import BoardOutlineGate
        zero = getattr(state, '_zero_edge_gate', None)
        if zero is None:
            zero = BoardOutlineGate(state.pcb_data.board_info, 0.0)
            state._zero_edge_gate = zero
        geometry = geometry_for(state, state.pcb_data, state.pcb_file)
        # A pad-less part is judged by its HOLES (the whole drill circle,
        # zero margin), else by its courtyard at zero margin -- not by the
        # margin-gated courtyard: a mounting hole's courtyard legitimately
        # crosses the edge (tigard H1 at its own human pose is 1.3mm past
        # the margin gate, and must seat). Without this a part with no copper
        # passed "every pad on the board" vacuously (tigard H1 at (300, 300)).
        n_holes, holes_out = _holes_outside(state, zero, ref, x, y, rot)
        if not pad_boxes(geometry, ref) and not n_holes:
            amt = zero.rect_outside_amount(r)
            if amt > 1e-9:
                reasons.append(f"no copper pad or hole to anchor an "
                               f"overhang, and its courtyard is {amt:.3f}mm "
                               f"past the outline")
        # check_assembly's own gate, per pad on its true outline, a
        # castellated pad on its edge exempt (#1297: CM5's H102..H105 read
        # 2.15 mm past the outline on their pad boxes, 0 on their pads)
        off = _pad_copper_off_board(state, ref, x, y, rot)
        if off > 1e-6:
            reasons.append(f"pad copper {off:.3f}mm past the outline")
        if holes_out:
            reasons.append(f"{holes_out} of {n_holes} drill hole(s) past "
                           f"the outline")
        return (None if reasons else 'overhang'), reasons, conflicts
    # #1257: a CONTAINED courtyard proves nothing about a pad that reaches
    # past it, so such a part's copper is asked too, by check_assembly's gate
    # (`pose_ok`'s conjunct) -- and refused with its measurement, exactly as
    # the overhang branch above refuses pad copper past the outline: the pose
    # is a fact, never nudged.
    if _pad_copper_overhang(state, ref) > state.edge_gate.margin:
        off = _pad_copper_off_board(state, ref, x, y, rot)
        if off > 1e-6:
            reasons.append(f"pad copper {off:.3f}mm past the outline")
    if _graphic_copper_overhang(state, ref) > state.edge_gate.margin:
        off = _graphic_copper_off_board(state, ref, x, y, rot)
        if off > 1e-6:
            reasons.append(f"graphic copper {off:.3f}mm past the outline")
    return (None if reasons else 'contained'), reasons, conflicts


def fixed_pose_pair_conflicts(state, entries, waived=frozenset()
                              ) -> List[Dict[str, object]]:
    """Pairwise contradictions among declared ``fixed_poses[]`` entries.

    The read-only geometry check ``plan_check`` runs on a plan's fixed
    poses.  It deliberately uses ``_fixed_pose_check``'s own courtyard, pad
    and hole predicates instead of maintaining a second preflight
    approximation.  Entries that cannot be resolved, or whose declared face
    differs from the board, are left to their existing resolution/refusal
    paths; inventing flipped geometry here would compare a part that cannot
    actually be seated.

    One record is returned per unordered pair.  Cross-face SMD courtyards and
    pads do not conflict because the shared legality predicates are
    side-aware; drilled geometry still reaches both faces. `waived` is the
    set of courtyard-waiver pairs (#1060), so a waived pair is not refused
    here.
    """
    declared: Dict[str, Tuple] = {}
    for f in sorted(entries, key=lambda row: str(row['ref'])):
        ref = str(f['ref'])
        part = state.parts.get(ref)
        if part is None:
            continue
        side = f.get('side')
        if side in ('F', 'B') and side != part.side:
            continue
        rot_decl = f.get('rot')
        rot = (part.rot % 360.0 if rot_decl is None or rot_decl == 'unknown'
               else float(rot_decl) % 360.0)
        declared[ref] = (round(float(f['x']), 3),
                         round(float(f['y']), 3),
                         _materialise_rotation(part, rot))

    out: List[Dict[str, object]] = []
    refs = sorted(declared)
    for i, a in enumerate(refs):
        for b in refs[i + 1:]:
            _how, _reasons, conflicts = _fixed_pose_check(
                state, a, declared[a], {b: declared[b]}, waived=waived)
            reason = conflicts.get(b)
            if reason is None:
                continue
            out.append({'pair': [a, b], 'reason': reason,
                        'poses': {a: list(declared[a]),
                                  b: list(declared[b])}})
    return out


#: KiCad 6's bare footprint lock, ``(footprint "X" locked ...``; group 1 is
#: everything before the word, which the unlock keeps.
_BARE_FP_LOCK_RE = re.compile(
    r'^(\(\s*(?:footprint|module)\s+(?:"(?:[^"\\]|\\.)*"|[^\s()"]+)'
    r'(?:\s+[A-Za-z_]+)*?)\s+locked\b')


def stamp_locked(board_file: str, refs: Sequence[str]) -> int:
    """Insert `(locked yes)` into the named footprints, in place.

    Inserted immediately after the footprint's opening token, which is before
    the first pad -- the position placement/parser.extract_locked_refs (and
    KiCad itself) reads it from. The grade's must_lock rule demands the lock
    IN THE FILE, so writing the intent's locks here is what makes the emitted
    seed grade clean rather than merely hoped-correct."""
    from kicad_parser import footprint_head_flags, iter_footprint_blocks
    with open(board_file, 'r', encoding='utf-8') as f:
        content = f.read()
    want = set(refs)
    count = 0
    # Named by the parser's own resolver (#726), so locking `TP4` stamps the
    # one block the parser calls `TP4`. It used to match the reference STRING,
    # so on watchy it locked both test points from one name -- and the refs
    # handed in here come from `pcb.footprints` keys, which now address the
    # twins separately. Reverse order keeps the spans valid as text is inserted.
    for start, end, fp_text, _raw_ref, key in reversed(
            list(iter_footprint_blocks(content))):
        if key not in want:
            continue
        if (re.search(r'\(locked(?:\s+yes)?\)', fp_text[:fp_text.find('(pad')
                                                   if '(pad' in fp_text else len(fp_text)])
                or 'locked' in footprint_head_flags(fp_text)):  # KiCad 6: bare
            continue
        open_m = re.match(r'\(footprint\s+"[^"]*"', fp_text)
        if not open_m:
            continue
        at = open_m.end()
        content = (content[:start + at] + '\n\t\t(locked yes)'
                   + content[start + at:])
        count += 1
    with open(board_file, 'w', encoding='utf-8') as f:
        f.write(content)
    return count


def stamp_unlocked(board_file: str, refs: Sequence[str]) -> int:
    """Remove `(locked yes)` from the named footprints, in place (#892).

    The inverse of `stamp_locked`, and it lives beside it deliberately: this
    repo had a stamper and no un-stamper, so a model that locked a rotation
    decision could not change its mind without hand-editing the board -- which
    is the class of hand script #892 exists to remove. This half reads exactly
    the window `stamp_locked` writes into -- the header, up to `(pad` -- and
    addresses blocks by the parser's own key (#726), so lock and unlock cannot
    disagree about which block they mean.

    Note for anyone tightening this: `placement/parser.extract_locked_refs`
    cuts at `'(pad '` WITH the space and falls back to the first 500
    characters when a block has no pad, so a header containing a token like
    `(padstack` is read differently there than here. Both stamping halves
    share that divergence and it predates them. Measured over the 22 boards in
    `kicad_files/` (1349 footprint blocks): 0 blocks contain `(pad` before
    `(pad `, and 0 disagree about a `(locked yes)`. 30 blocks DO take the
    reader's 500-character fallback (they have no pad at all) -- they simply
    carry no late lock, so the two windows still agree today. It is called out
    because the
    consequence is asymmetric: the reader would call a footprint locked that
    this cannot unlock, which is why `pose_ops.apply_poses` VERIFIES the
    unlock on the staged board before promoting anything.

    Returns the number of footprints actually changed; a ref that was not
    locked contributes 0 rather than raising, so unlocking twice is idempotent.
    Only KiCad's footprint `(locked yes)` is touched -- locked SEGMENTS and
    VIAS are copper, read by a different rule (#521), and are not footprint
    blocks, so nothing here can reach them.
    """
    from kicad_parser import iter_footprint_blocks
    # newline='' both ways: only text is removed, so the board keeps its own
    # line endings (make_unplaced verifies its output byte-for-byte nearby).
    with open(board_file, 'r', encoding='utf-8', newline='') as f:
        content = f.read()
    want = set(refs)
    count = 0
    # Reverse order keeps the spans valid as text is removed, exactly as the
    # stamping half relies on it while text is inserted.
    for start, end, fp_text, _raw_ref, key in reversed(
            list(iter_footprint_blocks(content))):
        if key not in want:
            continue
        head_end = fp_text.find('(pad') if '(pad' in fp_text else len(fp_text)
        head, tail = fp_text[:head_end], fp_text[head_end:]
        new_head, n = re.subn(r'\s*\(locked(?:\s+yes)?\)', '', head)
        # KiCad 6 writes the lock as a bare word after the name:
        # (footprint "X" locked (layer ... -- leave that and KiCad keeps it locked.
        new_head, n_bare = _BARE_FP_LOCK_RE.subn(r'\1', new_head, count=1)
        n += n_bare
        if not n:
            continue
        content = content[:start] + new_head + tail + content[end:]
        count += 1
    with open(board_file, 'w', encoding='utf-8', newline='') as f:
        f.write(content)
    return count
