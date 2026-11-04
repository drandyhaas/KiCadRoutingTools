"""Which face a flippable part stands on (place_simple).

The groups file names the parts that MAY stand on the other face (each
group's `flippable`, the agent's judgement). place_simple keeps every part on
the face it has -- the front, on an unplaced board -- and moves a flippable
part to the other face only where it is NEEDED: after the anneal, a part the
front had no room for -- no legal pose, or past a decap or claim limit --
goes to the back, or, when it may not, the flippable parts of its group
nearest where it wants to be do -- enough of them to free its own area
(#1482) -- and the board is annealed again (`needed_flips`). place_simple
keeps that run only when it leaves fewer parts without room, and tries one
more round when it kept one and parts still lack room (`FACE_ROUNDS`).

A face the intent's `assembly.sides` does not populate is never chosen, and a
part whose face something declares (the intent, a group's `side`) keeps it.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Set, Tuple

#: How many times place_simple anneals again with needed flips (#1482): a
#: second round only after a first that was kept and left parts without room.
FACE_ROUNDS = 2

def other(face: str) -> str:
    return 'B' if face == 'F' else 'F'


def face_of(fp) -> str:
    """'F' or 'B': `legality.footprint_side`, the one side rule (#878)."""
    from placement.legality import footprint_side
    return footprint_side(fp)


def flippable_refs(doc, movable: Iterable[str], declared: Dict[str, str],
                   assembly_sides: str) -> Tuple[Set[str], List[str]]:
    """`(refs, notes)`: the groups file's `flippable` parts that may change face
    here -- movable, no declared face, and the board populates both faces."""
    named = {r for g in doc['groups'] for r in g.get('flippable') or ()}
    notes = []
    if not named:
        return set(), notes
    if assembly_sides in ('F', 'B'):
        notes.append(f"the intent's assembly.sides is {assembly_sides!r}, so no part changes "
                     f"face ({len(named)} named flippable)")
        return set(), notes
    mov = set(movable)
    fixed = sorted(named - mov)
    if fixed:
        notes.append(f"flippable but not moved here (locked or seated): {', '.join(fixed)}")
    held = sorted(named & mov & set(declared))
    if held:
        notes.append(f"flippable but on a declared face: {', '.join(held)}")
    return (named & mov) - set(declared), notes


def flip_refusals(board_path: str, refs: Set[str]) -> Dict[str, str]:
    """`{ref: reason}` for the parts in `refs` the placement writer cannot
    flip (`writer.SideFlipUnsupported`), tried on their own blocks."""
    from kicad_parser import footprint_at_match, iter_footprint_blocks
    from placement.writer import (SideFlipUnsupported, _flip_footprint_block,
                                  board_inner_copper_count)
    with open(board_path, encoding='utf-8') as f:
        text = f.read()
    inner = board_inner_copper_count(text)
    out = {}
    for _s, _e, fp_text, _raw, key in iter_footprint_blocks(text):
        if key not in refs:
            continue
        m = footprint_at_match(fp_text)
        rot = float(m.group(3)) if m and m.group(3) else 0.0
        try:
            _flip_footprint_block(fp_text, key, rot, -rot,
                                  old_y=m.group(2) if m else None,
                                  inner_count=inner)
        except SideFlipUnsupported as ex:
            out[key] = str(ex)
    return out


def needs(model, illegal: Iterable[str]) -> Dict[str, str]:
    """`{ref: why}`: the parts the committed placement left without room --
    no legal pose, past a decap limit, past a claim's limit."""
    from placement import decap_repair
    from placement.legality import EPS
    out = {r: 'no legal pose' for r in illegal}
    for r in decap_repair.residual(model):
        out.setdefault(r['cap'], 'past its decap limit')
    for c in model.claims:
        if model._clo.get(id(c), 0.0) > EPS:
            out.setdefault(c[0], 'past its claim limit')
    return out


def needed_flips(need: Dict[str, str], refs: Set[str], doc, done: Set[str],
                 model=None) -> Dict[str, str]:
    """`{ref: why}`: each needy part when it is flippable, else room for it:
    the flippable parts of its group on its face within its own longer side
    of where it wants to be (`simple.target`: its claimed partner's pins,
    its decap site, its wiring), nearest first, until their courtyards add
    up to its own -- not the whole group, whose far members free no room
    where it needs it (#1482); none in reach, none flipped. None already
    moved (`done`). Without a `model` (no poses to measure), the group's
    every flippable part, as before."""
    group_of = {r: g for g in doc['groups'] for r in g['refs']}
    out = {}
    for r, why in sorted(need.items()):
        if r in refs and r not in done:
            out.setdefault(r, f"{why}")
            continue
        g = group_of.get(r)
        cand = sorted(m for m in (refs & set(g['refs']) if g else ()) if m not in done)
        if model is not None and cand:
            cand = _nearest_room(model, r, cand)
        for m in cand:
            out.setdefault(m, f"room for {r} ({why}) in group {g['name']}")
    return out


def _nearest_room(model, ref: str, cand: List[str]) -> List[str]:
    """The parts of `cand` on `ref`'s face within `ref`'s longer side of
    where it wants to be, nearest first, until their courtyard areas reach
    `ref`'s own."""
    from placement.simple import target
    st = model.state
    me = st.parts[ref]
    tx, ty = target(model, ref, set(st.parts))

    def area(q):
        return (q[2] - q[0]) * (q[3] - q[1])

    def dist(q):
        return math.hypot(max(q[0] - tx, 0.0, tx - q[2]), max(q[1] - ty, 0.0, ty - q[3]))
    q = me.rect()
    reach = max(q[2] - q[0], q[3] - q[1])
    mine = [m for m in cand if st.parts[m].sides & me.sides
            and dist(st.parts[m].rect()) <= reach]
    out, got, need = [], 0.0, area(q)
    for m in sorted(mine, key=lambda m: (dist(st.parts[m].rect()), m)):
        if got >= need:
            break
        out.append(m)
        got += area(st.parts[m].rect())
    return out
