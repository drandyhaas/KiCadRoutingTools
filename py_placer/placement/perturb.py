"""Manufacture a known-bad placement from a known-good one (#411).

The stress corpus has no bad seeds. It strips tracks, vias and pours but keeps
the placement, so every board arrives with a floorplan a human iterated until it
routed and shipped. #411's proposal: perturb that placement by a controlled
amount, recover, and grade against the original -- free ground truth, exactly
the way the original routing already is.

This module makes the bad seed. `placement/recovery.py` grades it, and is called
from here to fill this module's own record, so the perturber and the grader
cannot drift apart about what the dose was.

Three mechanics, each guarding a specific failure:

**Snap the OFFSET, never the poses.** Snapping each member independently shears
the block by up to a grid step, which destroys both the rigid-body ground truth
and the exact-inverse round trip. `quench._group_offsets` does the same and says
so.

**Do not route the emit through `group_move_valid` / `apply_group_move`.** Those
call `candidate_valid`, which will veto a 40 mm dose on principle. The
perturbation is *supposed* to be illegal -- that is what a bad seed is. Legality
is measured at emit and recorded, not enforced.

**Never call `build_neighbor_lists`.** The travel budget then stays infinite,
`BoardOutlineGate.edges_near`'s per-ref cache is sized conservatively and the
exact ring test is never skipped -- which is precisely why the pruning caveat in
`quench._group_offsets` (its docstring explains that lifting the per-member seed
cap makes the pruned lists lossy) does not apply to anything here.

Two things are deliberately NOT offered as mutations:

- **Block rotation.** It confounds position and orientation in one dose, and on
  any non-square block the emitted board is dominated by courtyard damage rather
  than displacement -- you would be measuring legality recovery. A single-part
  flip stays available as a test fixture for the rotation term of the metric,
  never as an experiment arm.
- **Layer flip (F.Cu <-> B.Cu).** `placement.writer.write_placed_output` rewrites
  `(at ...)` and pad angles only: it writes no `(layer ...)` and mirrors no pads.
  A "flip" through it produces a board whose pads contradict its side, which
  corrupts `footprint_side` / `sides_occupied` and therefore every overlap number
  downstream. If the wrong SIDE is wanted rather than the wrong HALF, that is a
  writer capability first.
"""
from __future__ import annotations

import json
import math
import os
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from kicad_parser import parse_kicad_pcb

KINDS = ('translate', 'wrong_side', 'swap', 'scatter', 'pile')

# A unit smaller than this is not a block, and one larger than this fraction of
# the movable parts is the board rather than a block in it.
BLOCK_MIN_PARTS = 3
BLOCK_MAX_FRACTION = 0.40


@dataclass(frozen=True)
class BlockPick:
    name: str
    members: Tuple[str, ...]
    source: str
    reason: str = ''


# ------------------------------------------------------------- block choice

def _anchor_unit(state, free, max_fanout: int) -> Optional[BlockPick]:
    """Highest-pin-count free part plus what it actually talks to.

    Defined on every board with an IC, which `sheet` is not -- measured, `sheet`
    yields nothing at all on a third of the corpus. And it is exactly the part
    class `place_route_loop --max-target-pins 40` cannot select, so it is the
    unit that tests #411's own prediction head-on rather than around it.
    """
    cands = sorted((r for r in free if r in state.parts),
                   key=lambda r: (-state.parts[r].pin_count, r))
    if not cands:
        return None
    anchor = cands[0]
    nets = {n for n in state.parts[anchor].nets
            if len(state.net_refs.get(n, ())) <= max_fanout}
    members = {anchor}
    for n in sorted(nets):
        for r in state.net_refs.get(n, ()):
            if r in state.parts and r in free:
                members.add(r)
    return BlockPick(name=f'anchor:{anchor}', members=tuple(sorted(members)),
                     source='anchor',
                     reason=f'{anchor} ({state.parts[anchor].pin_count} pins) '
                            f'and the {len(members) - 1} part(s) sharing its '
                            f'non-rail nets')


def _region_unit(state, free, pcb_data, quadrant: int = 0) -> Optional[BlockPick]:
    """The footprints inside one quadrant of the board outline.

    Needs no netlist structure at all, so it is defined on every board -- which
    is what makes "move a subsystem to the wrong side of the board" runnable on
    the boards that derive no blocks from any source.
    """
    bb = getattr(pcb_data.board_info, 'board_bounds', None)
    if not bb:
        return None
    x0, y0, x1, y1 = bb
    mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    qx = (x0, mx) if quadrant in (0, 2) else (mx, x1)
    qy = (y0, my) if quadrant in (0, 1) else (my, y1)
    members = tuple(sorted(
        r for r in free
        if r in state.parts
        and qx[0] <= state.parts[r].x < qx[1]
        and qy[0] <= state.parts[r].y < qy[1]))
    if not members:
        return None
    return BlockPick(name=f'region:q{quadrant}', members=members,
                     source='region',
                     reason=f'{len(members)} part(s) in board quadrant {quadrant}')


def candidate_blocks(pcb_data, pcb_file: str, *, group_by: str = 'kicad,sheet',
                     lock_globs: Optional[Sequence[str]] = None,
                     max_fanout: int = 20,
                     min_parts: int = BLOCK_MIN_PARTS,
                     max_fraction: float = BLOCK_MAX_FRACTION,
                     state=None) -> List[BlockPick]:
    """Every unit this board can offer, ranked, with its rejections stated.

    A block containing a LOCKED part is rejected rather than having the locked
    member dropped: a block that cannot translate rigidly is not the block you
    named, and half-moving one is a lie about the board.
    """
    from placement.groups import derive_groups, parse_sources
    from placement.portfolio import free_refs
    from pose_score import make_state

    st = state if state is not None else make_state(pcb_data, pcb_file)
    free = set(free_refs(pcb_data, pcb_file, lock_globs))
    locked = {r for r in st.parts if r not in free}
    out: List[BlockPick] = []

    try:
        sources = parse_sources(group_by)
    except Exception:                                       # noqa: BLE001
        sources = ()
    if sources:
        for name, refs in sorted(derive_groups(pcb_data, sources).items()):
            members = tuple(sorted(r for r in refs if r in st.parts))
            if not members:
                continue
            if set(members) & locked:
                continue
            if len(members) < min_parts:
                continue
            if len(members) > max(min_parts, max_fraction * max(1, len(free))):
                continue
            out.append(BlockPick(name=name, members=members,
                                 source=sources[0] if len(sources) == 1 else 'group',
                                 reason=f'{len(members)} part(s)'))

    a = _anchor_unit(st, free, max_fanout)
    if a and len(a.members) >= min_parts and not (set(a.members) & locked):
        out.append(a)
    for q in range(4):
        r = _region_unit(st, free, pcb_data, q)
        if r and len(r.members) >= min_parts and not (set(r.members) & locked):
            out.append(r)
    return out


def pick_block(pcb_data, pcb_file: str, *, name: Optional[str] = None,
               refs: Optional[Sequence[str]] = None, state=None,
               **kw) -> BlockPick:
    """Choose the unit deterministically. `refs` bypasses derivation entirely."""
    if refs:
        members = tuple(sorted(refs))
        return BlockPick(name='refs:' + '+'.join(members[:4]),
                         members=members, source='explicit',
                         reason='members named on the command line')
    cands = candidate_blocks(pcb_data, pcb_file, state=state, **kw)
    if not cands:
        raise ValueError(
            'no perturbable unit on this board: no group source produced a '
            'block of >= %d free parts, and neither the anchor nor any region '
            'unit qualified. Name the members with --refs.' % BLOCK_MIN_PARTS)
    if name:
        for c in cands:
            if c.name == name:
                return c
        raise ValueError(f'no unit named {name!r}; have: '
                         + ', '.join(c.name for c in cands))
    return sorted(cands, key=unit_rank)[0]


# Source precedence, not size. Ranking purely by size picks a board QUADRANT
# over a functional block on any board that has both -- measured on
# kit-dev-coldfire-xilinx_5213, where `region:q0` (44 parts) outranked the two
# sheet blocks, and a quadrant is the one unit whose "away from what it connects
# to" direction points straight off the board. A designer's own grouping is the
# unit the experiment is about; a region is the fallback that exists so boards
# with no grouping at all are still testable.
_SOURCE_RANK = {'explicit': 0, 'kicad': 1, 'group': 1, 'sheet': 1,
                'netprefix': 2, 'decap': 3, 'anchor': 4, 'region': 5}


def unit_rank(c: BlockPick) -> Tuple[int, int, str]:
    return (_SOURCE_RANK.get(c.source, 9), -len(c.members), c.name)


# ---------------------------------------------------------------- direction

def block_direction(state, members: Sequence[str], pcb_data,
                    ignore_net_ids=None, max_fanout: int = 20
                    ) -> Tuple[Tuple[float, float], str]:
    """A unit vector, a function of (board, unit) ALONE.

    Held fixed across every dose of a sweep. Otherwise the x-axis of the
    dose-response curve confounds dose with direction and the "curve" is a
    scatter plot of unrelated experiments.

    Primary: away from the centroid of the pads the unit connects to but does
    not own. That choice makes the perturbation exact rather than approximate --
    `block_displacements` computes the foreign centroid from parts OUTSIDE the
    unit, which a rigid translate of the unit leaves invariant, so translating
    by D*u away from it moves the reported distance by exactly D.
    """
    from placement import routability
    bd = routability.block_displacements(
        state, {'u': list(members)}, ignore_net_ids=ignore_net_ids,
        max_fanout=max_fanout)
    if bd:
        cx, cy = bd[0].centroid
        fx, fy = bd[0].net_centroid
        dx, dy = cx - fx, cy - fy
        if math.hypot(dx, dy) >= 1.0:
            n = math.hypot(dx, dy)
            return (dx / n, dy / n), 'away_from_net_centroid'
    bb = getattr(pcb_data.board_info, 'board_bounds', None)
    if bb:
        pts = [(state.parts[r].x, state.parts[r].y)
               for r in members if r in state.parts]
        if pts:
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            bx, by = (bb[0] + bb[2]) / 2.0, (bb[1] + bb[3]) / 2.0
            dx, dy = cx - bx, cy - by
            if math.hypot(dx, dy) >= 1e-6:
                n = math.hypot(dx, dy)
                return (dx / n, dy / n), 'away_from_board_centre'
    return (1.0, 0.0), 'plus_x'


# ------------------------------------------------------------- feasibility

def max_feasible_dose(state, members: Sequence[str],
                      u: Tuple[float, float], *, grid_step: float,
                      dose_cap: float, tol_mm: float = 1e-6) -> float:
    """Slide the unit along `u` until it starts leaving the outline, and return
    how far it got.

    Slide-until-contact, not largest-feasible-k: monotone by construction, so a
    notched outline or an interior cutout cannot let the search jump a hole and
    report a dose the unit could not actually travel.

    **Measured against each member's own baseline, not against zero.** Real
    boards ship parts outside the outline by design -- edge connectors,
    castellated rows, card edges -- and an absolute containment test vetoes the
    very first step on any board that has one. Measured: on
    kit-dev-coldfire-xilinx_5213 the absolute test returned a feasible dose of
    **0.0 mm at every requested dose**, silently clipping every cell to a no-op
    while reporting `clipped: true`; the corpus original of glasgow_revC
    likewise ships 15 out-of-outline parts. So the test is "no member may leave
    the outline by MORE than it already does".
    """
    from placement.utility import snap_to_grid
    base = {}
    for ref in members:
        p = state.parts.get(ref)
        if p is not None:
            base[ref] = state.edge_gate.rect_outside_amount(p.rect())
    step = max(grid_step, 1e-3)
    reached = 0.0
    k = 1
    while k * step <= dose_cap + 1e-9:
        dx = snap_to_grid(k * step * u[0], grid_step)
        dy = snap_to_grid(k * step * u[1], grid_step)
        ok = True
        for ref, base_amt in base.items():
            p = state.parts[ref]
            amt = state.edge_gate.rect_outside_amount(
                p.rect(p.x + dx, p.y + dy, p.rot))
            if amt > base_amt + tol_mm:
                ok = False
                break
        if not ok:
            break
        reached = math.hypot(dx, dy)
        k += 1
    return round(reached, 6)


# ------------------------------------------------------------------ mutate

def rigid_offsets(state, members: Sequence[str], t: Tuple[float, float], *,
                  grid_step: float) -> Dict[str, Tuple[float, float]]:
    """One snapped offset, shared by every member, so the unit stays rigid."""
    from placement.utility import snap_to_grid
    dx = snap_to_grid(t[0], grid_step)
    dy = snap_to_grid(t[1], grid_step)
    return {r: (dx, dy) for r in members if r in state.parts}


def _placements(state, offsets: Dict[str, Tuple[float, float]]) -> List[Dict]:
    out = []
    for ref, (dx, dy) in sorted(offsets.items()):
        p = state.parts[ref]
        out.append({'reference': ref, 'new_x': round(p.x + dx, 6),
                    'new_y': round(p.y + dy, 6), 'new_rotation': p.rot})
    return out


def _all_at_current(state, placements: List[Dict], pcb_data=None) -> List[Dict]:
    """Every part, so the FORMATTING of `(at)` says nothing about the draw.

    `placement.writer.write_placed_output` rewrites `(at x y)` with six decimals
    for exactly the footprints it is handed, and leaves every other one
    byte-identical. That makes the moved set readable straight off the output
    text, with no tooling and no truth: run 14's perturbed block was recovered
    by counting six-decimal `(at)` forms -- 24 of 235, against 0 on two
    untouched corpus boards -- and it survived a copper strip. This module's own
    docstring says the kind, dose, seed and block are "disclosed nowhere", and
    the block was in the file the whole time. Every past run leaked its the same
    way.

    So hand the writer EVERY part: the members at their new pose, everyone else
    at the pose they already have. One formatter, one precision, no subset to
    count. Poses are unchanged for non-members -- only their formatting is.
    """
    by_ref = {p['reference']: p for p in placements}
    fps = (pcb_data.footprints or {}) if pcb_data is not None else {}
    out = []
    for ref in sorted(state.parts):
        if ref in by_ref:
            out.append(by_ref[ref])
            continue
        # The pose AS PARSED, not the state's. `make_state` normalises rotation
        # into [0, 360), so a footprint written `-90` comes back as `270` -- the
        # same angle, and a 360 delta to anything that compares the numbers
        # without normalising. These parts did not move; nothing about them
        # should change but the number of decimals.
        fp = fps.get(ref)
        if fp is not None:
            out.append({'reference': ref, 'new_x': round(fp.x, 6),
                        'new_y': round(fp.y, 6), 'new_rotation': fp.rotation})
            continue
        p = state.parts[ref]
        out.append({'reference': ref, 'new_x': round(p.x, 6),
                    'new_y': round(p.y, 6), 'new_rotation': p.rot})
    return out


def _pile_placements(state, members: Sequence[str], pcb_data) -> List[Dict]:
    """Collapse `members` onto ONE coordinate.

    Sized from `placement_state`'s own gate rather than from a round number:
    `unplaced` needs `duplicate_fraction >= 0.5` AND
    `distinct_positions <= max(3, 0.1 * n)`, so stacking half the parts is not
    enough -- they must collapse to a handful of coordinates. Landing them all
    on one satisfies both terms.

    The caller passes EVERY FREE PART for this kind, not the chosen unit.
    Measured: piling only the unit gave 67 of glasgow_revC's 266 footprints
    (25%) and 33 of tigard's 92, so `assess_placement` reported merely
    `partially_unplaced` and `place_seed` would have seeded the pile beside the
    existing placement instead of placing the board. This kind exists to
    produce the place-from-scratch task, and that needs the whole board.
    """
    bb = getattr(pcb_data.board_info, 'board_bounds', None)
    if bb:
        cx, cy = (bb[0] + bb[2]) / 2.0, (bb[1] + bb[3]) / 2.0
    else:
        cx = cy = 0.0
    return [{'reference': r, 'new_x': round(cx, 6), 'new_y': round(cy, 6),
             'new_rotation': state.parts[r].rot}
            for r in sorted(members) if r in state.parts]


def perturb(board: str, out_board: str, *, kind: str = 'translate',
            dose_mm: float = 10.0, block: Optional[str] = None,
            refs: Optional[Sequence[str]] = None,
            group_by: str = 'kicad,sheet',
            lock_globs: Optional[Sequence[str]] = None,
            ignore_nets: Optional[Sequence[str]] = None,
            max_fanout: int = 20, grid_step: float = 0.1,
            clearance: float = 0.25, board_edge_clearance: float = 0.55,
            on_overflow: str = 'clip', seed: int = 0,
            write_record: bool = True,
            control_out: Optional[str] = None) -> Dict:
    """Write a perturbed board plus its ground-truth record. Returns the record.

    The CONTROL is the `--dose 0` pass of this same writer, never the raw input
    board: `write_placed_output` is not poses-only -- it runs
    `move_copper_text_to_silkscreen` over the whole file and can inject via
    moves -- so comparing a perturbed output against the raw input would differ
    by a text transformation nobody accounted for.

    `control_out` IS THE FENCE, and the default is the unsafe one kept only for
    compatibility. The control is the human placement pose-for-pose, so writing
    it beside `out_board` -- which is what happens when this is None -- drops
    ground truth INSIDE the directory the run then works in, where any glob, any
    `--before`, any tool taking a directory can read it. Run 12 caught it by
    reading a directory listing and moved the file out by hand; a run that did
    not look would have been fenced on paper and open in fact. Pass a path
    outside the work dir (`<subject>/_truth/`, see tests/stress/RUNBOOK.md).

    Either way the destination is PRINTED, so a staging script cannot be
    unaware of where truth landed -- silence was the other half of the defect.
    """
    if kind not in KINDS:
        raise ValueError(f'unknown kind {kind!r}; have {", ".join(KINDS)}')
    if on_overflow not in ('clip', 'skip', 'force'):
        raise ValueError("on_overflow must be clip, skip or force")

    from pose_score import make_state
    from placement import recovery
    from placement.portfolio import copy_siblings, ignore_net_ids
    from placement.writer import write_placed_output

    pcb = parse_kicad_pcb(board)
    ids = list(ignore_net_ids(pcb, ignore_nets) or ())
    st = make_state(pcb, board, clearance=clearance,
                    board_edge_clearance=board_edge_clearance,
                    grid_step=grid_step, ignore_net_ids=ids or None)

    pick = pick_block(pcb, board, name=block, refs=refs, state=st,
                      group_by=group_by, lock_globs=lock_globs,
                      max_fanout=max_fanout)
    members = list(pick.members)

    u, dsrc = block_direction(st, members, pcb, ignore_net_ids=ids or None,
                              max_fanout=max_fanout)
    bb = getattr(pcb.board_info, 'board_bounds', None)
    diag = (math.hypot(bb[2] - bb[0], bb[3] - bb[1]) if bb else 0.0)

    # rng is used ONLY where a choice is genuinely arbitrary (swap partner).
    # Named and scoped, matching portfolio's convention; no RNG reaches an
    # engine path.
    rng = random.Random(f'{seed}:{kind}:{dose_mm}')

    # The primary direction can be geometrically dead -- a unit hard against an
    # edge, or a region unit whose "away from what it connects to" points off
    # the board. Fall back through a FIXED candidate list and record which one
    # was taken, so the direction stays a deterministic function of (board,
    # unit) -- frozen across every dose -- without the experiment silently
    # becoming a no-op. Measured: without this, every dose on
    # kit-dev-coldfire-xilinx_5213 clipped to 0.0 mm and reported `clipped:
    # true`, which is a cell that looks like a result and is not one.
    _cands = [(u, dsrc), ((-u[0], -u[1]), dsrc + '_reversed'),
              ((-u[1], u[0]), dsrc + '_perp'),
              ((u[1], -u[0]), dsrc + '_perp_reversed')]
    feasible = 0.0
    for _u, _src in _cands:
        feasible = max_feasible_dose(st, members, _u, grid_step=grid_step,
                                     dose_cap=max(dose_mm, 1e-9))
        if feasible > 0.0 or dose_mm <= 0.0:
            u, dsrc = _u, _src
            break
    clipped = False
    applied_req = dose_mm
    if kind in ('translate', 'wrong_side') and feasible + 1e-9 < dose_mm:
        if on_overflow == 'skip':
            return {'status': 'infeasible', 'board': board, 'kind': kind,
                    'dose_mm_requested': dose_mm,
                    'max_feasible_dose_mm': feasible,
                    'block': {'name': pick.name, 'members': members},
                    'reason': 'the unit cannot travel the requested dose '
                              'without leaving the outline'}
        if on_overflow == 'clip':
            applied_req = feasible
            clipped = True

    if kind == 'pile':
        from placement.portfolio import free_refs
        members = sorted(r for r in free_refs(pcb, board, lock_globs)
                         if r in st.parts)
        placements = _pile_placements(st, members, pcb)
        # The record must name what actually moved. Leaving the ranked unit's
        # name here would report `anchor:U30` on a board where all 243 free
        # parts were piled.
        pick = BlockPick(name='pile:all-free', members=tuple(members),
                         source='pile',
                         reason=f'every free part ({len(members)}) collapsed to '
                                f'one coordinate to trip the unplaced gate')
    elif kind == 'scatter':
        # The POSITIVE CONTROL, and it reuses the shipped perturbation rather
        # than a second implementation: this is the arm that MUST recover,
        # because per-part jitter inside a block is exactly what the quench is
        # built for. If it does not recover, the instrument is wrong, not the
        # optimiser.
        from placement.portfolio import perturb_jitter
        poses = perturb_jitter(st, members, rng, max(applied_req, grid_step))
        placements = [{'reference': p['reference'], 'new_x': p['new_x'],
                       'new_y': p['new_y'], 'new_rotation': p['new_rotation']}
                      for p in poses]
    elif kind == 'swap':
        # The primary unit is chosen by rank, which prefers the LARGEST block of
        # the best source -- and on a real board that block can overlap every
        # other candidate, leaving nothing disjoint to trade with (measured:
        # glasgow_revC's 67-part anchor unit and tigard's 33-part one both
        # returned "no second disjoint unit"). So a swap re-picks the PAIR
        # rather than inheriting the primary: the highest-ranked two units that
        # are actually disjoint. Deterministic, and reported.
        cands = sorted(candidate_blocks(pcb, board, state=st, group_by=group_by,
                                        lock_globs=lock_globs,
                                        max_fanout=max_fanout), key=unit_rank)
        pair = None
        for i, a in enumerate(cands):
            for bcand in cands[i + 1:]:
                if not (set(a.members) & set(bcand.members)):
                    pair = (a, bcand)
                    break
            if pair:
                break
        if pair is None:
            return {'status': 'infeasible', 'board': board, 'kind': kind,
                    'reason': f'no two disjoint units among the '
                              f'{len(cands)} candidate(s) on this board',
                    'block': {'name': pick.name, 'members': members}}
        pick, partner = pair
        members = list(pick.members)
        def _c(refs_):
            pts = [(st.parts[r].x, st.parts[r].y) for r in refs_ if r in st.parts]
            return (sum(p[0] for p in pts) / len(pts),
                    sum(p[1] for p in pts) / len(pts))
        ax, ay = _c(members)
        bx, by = _c(partner.members)
        placements = _placements(st, rigid_offsets(st, members,
                                                   (bx - ax, by - ay),
                                                   grid_step=grid_step))
        placements += _placements(st, rigid_offsets(st, list(partner.members),
                                                    (ax - bx, ay - by),
                                                    grid_step=grid_step))
        members = sorted(set(members) | set(partner.members))
    else:                                              # translate / wrong_side
        if kind == 'wrong_side' and bb:
            pts = [(st.parts[r].x, st.parts[r].y) for r in members
                   if r in st.parts]
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            bx, by = (bb[0] + bb[2]) / 2.0, (bb[1] + bb[3]) / 2.0
            t = (2.0 * (bx - cx), 2.0 * (by - cy))
        else:
            t = (applied_req * u[0], applied_req * u[1])
        placements = _placements(st, rigid_offsets(st, members, t,
                                                   grid_step=grid_step))

    # The control: the SAME writer, dose 0.
    control = (control_out or
               os.path.splitext(out_board)[0] + '.control.kicad_pcb')
    _cdir = os.path.dirname(os.path.abspath(control))
    if _cdir:
        os.makedirs(_cdir, exist_ok=True)
    # EVERY part, not none: same reason the perturbed board below gets the
    # full list. A control formatted differently from its subject is itself a
    # comparison anyone could make.
    write_placed_output(board, control, _all_at_current(st, [], pcb))
    copy_siblings(board, control)
    # Say where truth went, ALWAYS. This board carries the original placement
    # pose-for-pose; a caller that does not know its path cannot fence it.
    _inside = (os.path.dirname(os.path.abspath(control)) ==
               os.path.dirname(os.path.abspath(out_board)))
    print(f"CONTROL (GROUND TRUTH) -> {os.path.abspath(control)}", flush=True)
    if _inside and control_out is None:
        print("  WARNING: that is the SAME directory as the perturbed output, "
              "so the human placement is inside the work dir the run will "
              "read. Pass control_out=<somewhere outside> (the staging recipe "
              "uses a sibling _truth/), or fence_audit --mode create will "
              "report it as a LEAK -- which it is.", flush=True)
    write_placed_output(board, out_board, _all_at_current(st, placements, pcb))
    copy_siblings(board, out_board)

    ctrl_pcb = parse_kicad_pcb(control)
    orig_poses = recovery.board_poses(ctrl_pcb)
    scoring = {
        'clearance': clearance, 'board_edge_clearance': board_edge_clearance,
        'grid_step': grid_step, 'max_fanout': max_fanout,
        'ignore_net_ids': ids, 'ignore_nets': list(ignore_nets or ()),
        'group_by': group_by,
        'net_ids': recovery.frozen_net_ids(ctrl_pcb, ids),
    }
    tally = recovery.connectivity_tally(ctrl_pcb, scoring['net_ids'])
    scoring['prr_denominator'] = tally['prr_denominator']
    scoring['rr_denominator'] = tally['rr_denominator']

    record = {
        'schema': 1,
        'tool': 'placement/perturb.py',
        'original_board': os.path.abspath(board),
        'control_board': os.path.abspath(control),
        'output_board': os.path.abspath(out_board),
        'kind': kind,
        'seed': seed,
        'dose_mm_requested': dose_mm,
        'clipped': clipped,
        'max_feasible_dose_mm': feasible,
        'board_diagonal_mm': round(diag, 6),
        'dose_norm': (round(applied_req / diag, 6) if diag else None),
        'direction': [round(u[0], 9), round(u[1], 9)],
        'direction_source': dsrc,
        # FROZEN. The scorer must never re-derive this: netprefix blocks gate on
        # a 20 mm spatial radius and decap tethers on 5 mm, so a 40 mm dose
        # CHANGES what the block is, and re-deriving would score a different set
        # of parts at every dose.
        'block': {'name': pick.name, 'source': pick.source,
                  'reason': pick.reason, 'members': sorted(members)},
        'scoring': scoring,
        'original_poses': {r: [round(v, 6) for v in p]
                           for r, p in sorted(orig_poses.items())},
        'status': 'ok',
    }
    record['at_original'] = recovery.score_board(control, record)
    record['at_emit'] = recovery.score_board(out_board, record)
    # The dose as the GRADER measures it, which is what recovery divides by --
    # not the number that was requested, and not |t| before the grid snap.
    record['dose_mm_applied'] = record['at_emit']['displacement'][
        'perturbed_pad_rms']

    if write_record:
        # THE RECORD FOLLOWS THE CONTROL, because it carries `original_poses`
        # and is therefore ground truth just as literally as the control board
        # is. Leaving it in the work dir while the control was fenced would move
        # the leak rather than close it (run 12 moved it out by hand for exactly
        # this reason). With `control_out` unset the control sits beside
        # `out_board`, so this resolves to the historical path unchanged.
        _stem = os.path.splitext(os.path.basename(out_board))[0]
        record_path = (os.path.splitext(out_board)[0] + '.perturb.json'
                       if control_out is None
                       else os.path.join(os.path.dirname(os.path.abspath(control)),
                                         _stem + '.perturb.json'))
        with open(record_path, 'w', encoding='utf-8') as fh:
            json.dump(record, fh, indent=1, sort_keys=True)
            fh.write('\n')
        print(f"RECORD (GROUND TRUTH) -> {os.path.abspath(record_path)}",
              flush=True)
    return record
