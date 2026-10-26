#!/usr/bin/env python3
"""Pile a board's components off its outline: the input for a from-scratch run.

    python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/make_unplaced.py SRC DST [--keep-locked] [--keep-local-zones]

Nothing else in the toolchain PRODUCES an unplaced board, and every corpus
board ships placed. This script builds that input deterministically, so the
run is replayable. Measured on an 18-part
2-layer board and a 264-part 4-layer board.

- **Every pad-bearing part moves**, into a tight cluster south of the outline.
  The pile satisfies `assess_placement`'s own thresholds by construction: its
  spread is under SPREAD_RATIO, and it clears the outline by the tallest
  part's own reach. A wide grid would read `unplaced=False` on a board that is
  plainly unplaced.
- **Locked parts are moved and UNLOCKED by default.** The toolchain never
  moves a KiCad-locked part (no override), so a locked part left in the pile
  would stay there forever. The facts a lock carried (a connector's edge, a
  mounting hole's position) must be DECLARED instead, in the design brief or
  the run's prompt. `--keep-locked` leaves locked parts where they are, still
  locked. That was run 31/32's input, which only reaches `partially_unplaced`.
- **Every moved part's texts are reset to absolute 0** (#1163). KiCad stores
  a text angle as an absolute board angle and humans keep labels upright, so
  a text composed with the rotation change into the pile (as the writer does
  for a real rotation) reads `T - R` and hands back the human rotation `R`.
- **A board-level zone that outlines the placement is DROPPED** (#1163): one
  covering under a quarter of the board's bounding box that holds a pad of a
  part this script moves -- a local pour, an antenna keep-out -- traces where
  that part was, which is exactly what a pile must not carry. Every other
  zone and rule area stays where it is: a board-wide pour says which net
  pours on which layer, and a zone holding no moved part's pad locates none.
  A dropped zone's FACT (that net pours there, nothing may go here) is the
  run's to DECLARE, in the design brief or the prompt, as a lock's is.
  `--keep-local-zones` keeps them and only reports them. (Zones a footprint
  owns move with it.)
- **Padless artwork stays.** It has no nets, and nothing could bring it back.
- **Refuses (exit 3) when a footprint draws the board outline** (#829):
  moving it would move the outline.
- **Refuses (exit 3) a routed input.** Strip it first with
  `tests/stress/strip_copper_only.py`, which keeps the outline bit-identical.

The OUTPUT is verified, not the plan: every moved part's courtyard clears the
outline, siblings (.kicad_pro/.kicad_prl/.kicad_dru) are carried, no copper,
no new stack, every unlocked part really is unlocked, every moved part's
every text reads absolute 0, and no kept board-level zone outlines a moved
part (unless `--keep-local-zones`). Exit 0 when all of that holds, 1 when a
check fails.
"""
from __future__ import annotations

import argparse
import os
import re
import sys

KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'actor'}

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
for _d in ('py_router', 'py_placer'):
    _p = os.path.join(ROOT, _d)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import (parse_kicad_pcb, iter_footprint_blocks,  # noqa: E402
                          non_aperture_pads, find_matching_paren)
from placement.placement_state import assess_placement          # noqa: E402
from placement.utility import compute_footprint_bbox_local       # noqa: E402
from placement.portfolio import copy_siblings                    # noqa: E402
from placement.writer import (write_placed_output,              # noqa: E402
                              text_node_angles)

CLUSTER_FRAC = 0.10     # of the board diagonal; SPREAD_RATIO (s2) is 0.15
GAP_MARGIN = 2.0        # mm of clear air beyond the tallest part's reach
PILE_ROT = 0.0          # a pile carries no orientation information
PILE_TEXT_ANGLE = 0.0   # ...and neither do its texts (#1163): absolute degrees
LOCAL_ZONE_FRAC = 0.25  # a zone under this share of the bbox outlines a placement
SIBLINGS = ('.kicad_pro', '.kicad_prl', '.kicad_dru')


def text_angle_leaks(path, refs):
    """({ref: angles} for each named block with a text NOT written at
    absolute PILE_TEXT_ANGLE, texts checked, texts that passed) (#1163).

    The token is checked as well as the value: KiCad reads an absent token as
    0, but the writer adds one only to a text whose angle came out non-zero,
    so whether a token is there would itself say something about the pose.
    """
    with open(path, encoding='utf-8', newline='') as fh:
        text = fh.read()
    bad, n, good = {}, 0, 0
    for _s, _e, fp_text, _raw, key in iter_footprint_blocks(text):
        if key not in refs:
            continue
        angles = text_node_angles(fp_text)
        ok = [a is not None and a % 360 == PILE_TEXT_ANGLE for a in angles]
        n += len(angles)
        good += sum(ok)
        if not all(ok):
            bad[key] = angles
    return bad, n, good


def _shoelace(ring):
    return abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1)
                   in zip(ring, ring[1:] + ring[:1]))) / 2.0


def _inside(x, y, ring):
    hit = False
    for (x0, y0), (x1, y1) in zip(ring, ring[1:] + ring[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            hit = not hit
    return hit


#: A top-level `(zone` header, as the parser's own `_iter_zone_blocks` finds it.
_ZONE_START = re.compile(r'\r?\n([\t ]*)\(zone(?=[\s(])')
#: A vertex of a zone outline: `(xy x y)`, or an arc's start/mid/end.
_ZONE_PT = re.compile(r'\((?:xy|start|mid|end)\s+(-?[\d.]+(?:[eE][-+]?\d+)?)'
                      r'\s+(-?[\d.]+(?:[eE][-+]?\d+)?)\)')


def board_zone_blocks(text):
    """[(start, end, rings)] for every BOARD-LEVEL `(zone ...)` block -- copper
    pours and rule areas alike -- with each outline `(polygon ...)` as a ring
    (never a `filled_polygon`). `start` is the newline before the block, so
    cutting `text[start:end]` leaves the file's layout intact. A zone a
    footprint owns is nested deeper (the parser's own indentation rule) and
    moves with its part, so it is not listed."""
    out = []
    for m in _ZONE_START.finditer(text):
        ind = m.group(1)
        if ind.count('\t') >= 2 or ('\t' not in ind and len(ind) >= 4):
            continue
        open_idx = text.index('(', m.start())
        end = find_matching_paren(text, open_idx)
        block = text[open_idx:end]
        rings = []
        for pm in re.finditer(r'\(polygon(?=[\s(])', block):
            ring = [(float(a), float(b)) for a, b in
                    _ZONE_PT.findall(block[pm.start():
                                           find_matching_paren(block, pm.start())])]
            if len(ring) >= 3:
                rings.append(ring)
        out.append((m.start(), end, rings))
    return out


def placement_zones(text, pcb, movable, bounds):
    """[(start, end, refs)] for each board-level zone that OUTLINES THE
    PLACEMENT (#1163): it covers under LOCAL_ZONE_FRAC of the bounding box and
    holds a pad of a moved part (`refs`), measured on the SOURCE poses."""
    min_x, min_y, max_x, max_y = bounds
    bbox = (max_x - min_x) * (max_y - min_y)
    pads = [(ref, p.global_x, p.global_y)
            for ref in movable for p in pcb.footprints[ref].pads]
    out = []
    for start, end, rings in board_zone_blocks(text):
        if not rings or bbox <= 0 or \
                sum(_shoelace(r) for r in rings) >= LOCAL_ZONE_FRAC * bbox:
            continue
        refs = sorted({ref for ref, x, y in pads
                       if any(_inside(x, y, r) for r in rings)})
        if refs:
            out.append((start, end, refs))
    return out


def _drop_blocks(path, spans):
    """Cut `spans` ([(start, end, ...)]) out of the file at `path`."""
    with open(path, encoding='utf-8', newline='') as fh:
        text = fh.read()
    for start, end, *_rest in sorted(spans, reverse=True):
        text = text[:start] + text[end:]
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)


def _unlock(dst, refs):
    """Unlock the named blocks; returns the ones it changed.

    Through `seeder.stamp_unlocked`, the one unlock writer, which reads every
    lock spelling the parser does: `(locked yes)`, the 2021 `(locked)`, and
    KiCad 6's bare word in the header, `(footprint "X" locked (layer ...`.
    This used to strip a `(locked yes)` line only, so on a KiCad 6 board a
    locked part was piled and left locked -- a part no placer may move, and
    the run failed its own check (esp-rust-board's U4)."""
    from placement.seeder import stamp_unlocked
    return [ref for ref in sorted(refs) if stamp_unlocked(dst, [ref])]


def build(src: str, dst: str, keep_locked: bool = False,
          keep_local_zones: bool = False) -> int:
    pcb = parse_kicad_pcb(src)
    bounds = pcb.board_info.board_bounds
    if bounds is None:
        print('refuse: board has no Edge.Cuts outline', file=sys.stderr)
        return 3
    min_x, min_y, max_x, max_y = bounds
    owners = sorted(r for r, f in pcb.footprints.items()
                    if getattr(f, 'owns_board_outline', False))
    if owners:
        print(f'refuse: {len(owners)} footprint(s) draw the board boundary '
              f'({", ".join(owners)}); moving them moves the outline',
              file=sys.stderr)
        return 3

    # A routed input is refused BEFORE anything is written: its copper would
    # be left where the parts were, and that copper encodes the original
    # poses. Most corpus boards ship routed.
    if assess_placement(pcb, src).has_copper:
        print(f'refuse: {src} carries routed copper. Strip it first, keeping '
              f'the outline bit-identical:\n    python3 -X utf8 '
              f'tests/stress/strip_copper_only.py {src} <unrouted.kicad_pcb>',
              file=sys.stderr)
        return 3
    # A part whose only pads are apertures (paste/mask windows) is artwork,
    # not a pad-bearing part (#1143).
    was_locked = sorted(r for r, f in pcb.footprints.items()
                        if non_aperture_pads(f) and getattr(f, 'locked', False))
    held = was_locked if keep_locked else []
    movable = sorted(r for r, f in pcb.footprints.items()
                     if non_aperture_pads(f) and r not in held)
    artwork = sorted(r for r, f in pcb.footprints.items()
                     if not non_aperture_pads(f))
    if not movable:
        print('refuse: no movable pad-bearing part', file=sys.stderr)
        return 3

    reach = 0.0
    for ref in movable:
        lb = compute_footprint_bbox_local(pcb.footprints[ref])
        if lb:
            reach = max(reach, -lb[1])
    diag = ((max_x - min_x) ** 2 + (max_y - min_y) ** 2) ** 0.5
    extent = CLUSTER_FRAC * diag / (2 ** 0.5)
    cols = max(1, int(len(movable) ** 0.5 + 0.5))
    rows = (len(movable) + cols - 1) // cols
    pitch_x = extent / max(1, cols - 1)
    pitch_y = extent / max(1, rows - 1)
    cx = (min_x + max_x) / 2.0
    cy = max_y + reach + GAP_MARGIN + extent / 2.0
    x0 = cx - (cols - 1) * pitch_x / 2.0
    y0 = cy - (rows - 1) * pitch_y / 2.0
    placements = [{'reference': ref,
                   'new_x': round(x0 + (i % cols) * pitch_x, 4),
                   'new_y': round(y0 + (i // cols) * pitch_y, 4),
                   'new_rotation': PILE_ROT,
                   'new_text_angle': PILE_TEXT_ANGLE}
                  for i, ref in enumerate(movable)]
    if not write_placed_output(src, dst, placements, pcb_data=pcb):
        print('refuse: write_placed_output returned False', file=sys.stderr)
        return 1
    unlocked = _unlock(dst, set(movable) & set(was_locked))
    # #1163: the zones that outline the placement go, unless asked to stay.
    # Measured on the written board's text (the writer does not touch a
    # board-level zone) against the SOURCE pad poses.
    with open(dst, encoding='utf-8', newline='') as fh:
        _dst_text = fh.read()
    n_zones = len(board_zone_blocks(_dst_text))
    local = placement_zones(_dst_text, pcb, movable, bounds)
    if local and not keep_local_zones:
        _drop_blocks(dst, local)
    copy_siblings(src, dst)

    # --- verify the OUTPUT, not the plan -----------------------------------
    st_in = assess_placement(pcb, src)
    out = parse_kicad_pcb(dst)
    st = assess_placement(out, dst)
    ok = True
    for ref in artwork + held:
        a, b = pcb.footprints[ref], out.footprints[ref]
        if (round(a.x, 4), round(a.y, 4), round(a.rotation or 0, 3)) != \
           (round(b.x, 4), round(b.y, 4), round(b.rotation or 0, 3)):
            print(f'FAIL: {ref} was meant to stay and moved', file=sys.stderr)
            ok = False
    worst = None
    for ref in movable:
        f = out.footprints[ref]
        lb = compute_footprint_bbox_local(f)
        if lb is None:
            continue
        gap = f.y + lb[1] - max_y
        if worst is None or gap < worst[1]:
            worst = (ref, gap)
    if worst is not None and worst[1] <= 0:
        print(f'FAIL: {worst[0]} reaches back into the outline by '
              f'{-worst[1]:.2f} mm', file=sys.stderr)
        ok = False
    sibs = [e for e in SIBLINGS if os.path.isfile(os.path.splitext(dst)[0] + e)]
    want = [e for e in SIBLINGS if os.path.isfile(os.path.splitext(src)[0] + e)]
    if sibs != want:
        print(f'FAIL: siblings not carried -- source has {want}, output has '
              f'{sibs}', file=sys.stderr)
        ok = False
    if not held and not st.unplaced:
        print('FAIL: assess_placement does not call this board unplaced',
              file=sys.stderr)
        ok = False
    if st.has_copper:
        print('FAIL: the output carries copper', file=sys.stderr)
        ok = False
    new_stack = sorted(set(st.stacked_suspect_refs) - set(st_in.stacked_suspect_refs))
    if new_stack:
        print(f'FAIL: {len(new_stack)} part(s) newly stacked: '
              f'{", ".join(new_stack[:8])}', file=sys.stderr)
        ok = False
    to_unlock = sorted(set(was_locked) - set(held))
    still = sorted(r for r in to_unlock if getattr(out.footprints[r], 'locked', False))
    if still or unlocked != to_unlock:
        print(f'FAIL: meant to unlock {to_unlock}, unlocked {unlocked}, still '
              f'locked {still}', file=sys.stderr)
        ok = False
    leaks, n_texts, n_text_ok = text_angle_leaks(dst, set(movable))
    if leaks:
        print(f'FAIL: {len(leaks)} moved part(s) carry a text not at absolute '
              f'{PILE_TEXT_ANGLE:g}: '
              + ', '.join(f'{r} {a}' for r, a in sorted(leaks.items())[:8]),
              file=sys.stderr)
        ok = False
    with open(dst, encoding='utf-8', newline='') as fh:
        _out_text = fh.read()
    still_local = placement_zones(_out_text, pcb, movable, bounds)
    dropped = 0 if keep_local_zones else len(local)
    if not keep_local_zones and still_local:
        print(f'FAIL: {len(still_local)} kept zone(s) still outline a moved '
              f'part: {", ".join(sorted({r for z in still_local for r in z[2]})[:8])}',
              file=sys.stderr)
        ok = False
    if len(board_zone_blocks(_out_text)) != n_zones - dropped:
        print(f'FAIL: meant to drop {dropped} of {n_zones} board-level zone(s), '
              f'the output carries {len(board_zone_blocks(_out_text))}',
              file=sys.stderr)
        ok = False
    in_local = sorted({r for z in local for r in z[2]})

    print(f'board      {max_x - min_x:.2f} x {max_y - min_y:.2f} mm')
    print(f'moved      {len(movable)} pad-bearing part(s) to a pile at '
          f'({cx:.2f}, {cy:.2f}); closest to the outline: '
          f'{worst[0] if worst else "-"} at '
          f'{worst[1] if worst else float("nan"):.2f} mm below it')
    print(f'unlocked   {len(unlocked)}: {", ".join(unlocked) or "-"}')
    print(f'held       {len(held)} locked part(s), {len(artwork)} padless artwork block(s)')
    print(f'texts      {n_text_ok} of {n_texts} text(s) on {len(movable)} '
          f'moved part(s) at absolute {PILE_TEXT_ANGLE:g}')
    named = ', '.join(in_local[:8]) + (' ...' if len(in_local) > 8 else '')
    if not local:
        zl = (f'zones      {n_zones} board-level zone(s)/rule area(s) kept; '
              f'none covers under {LOCAL_ZONE_FRAC:.0%} of the bounding box and '
              f'holds a moved part\'s pad')
    elif keep_local_zones:
        zl = (f'zones      {n_zones} board-level zone(s)/rule area(s) kept where '
              f'they were (--keep-local-zones); {len(local)} cover under '
              f'{LOCAL_ZONE_FRAC:.0%} of the bounding box and hold a pad of '
              f'{len(in_local)} moved part(s) ({named}), so their outlines '
              f'still trace the placement (#1163)')
    else:
        zl = (f'zones      {n_zones - dropped} of {n_zones} board-level zone(s)/'
              f'rule area(s) kept; dropped {dropped} that cover under '
              f'{LOCAL_ZONE_FRAC:.0%} of the bounding box and held a pad of '
              f'{len(in_local)} moved part(s) ({named}): their outlines traced '
              f'the placement (#1163). Declare what they meant in the design '
              f'brief, or pass --keep-local-zones')
    print(zl)
    print(f'siblings   {sibs or "none (the source had none either)"}')
    print(f'verdict    unplaced={st.unplaced} partially_unplaced='
          f'{st.partially_unplaced} has_copper={st.has_copper}')
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--keep-locked', action='store_true',
                    help='leave KiCad-locked parts in place and locked')
    ap.add_argument('--keep-local-zones', action='store_true',
                    help='keep board-level zones that outline the placement '
                         f'(under {round(LOCAL_ZONE_FRAC * 100)}%% of the '
                         'bounding box, holding a moved part\'s pad) instead '
                         'of dropping them; they are reported')
    a = ap.parse_args(argv)
    return build(a.src, a.dst, a.keep_locked, a.keep_local_zones)


if __name__ == '__main__':
    sys.exit(main())
