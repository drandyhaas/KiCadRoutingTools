#!/usr/bin/env python3
"""Pile a board's components off its outline: the input for a from-scratch run.

    python3 -X utf8 .claude/skills/pcb-free-agent/scripts/make_unplaced.py SRC DST [--keep-locked]

Nothing else in the toolchain PRODUCES an unplaced board. `perturb.py` damages
a placement, and every corpus board ships placed. This script builds that
input deterministically, so the run is replayable. Measured on an 18-part
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
- **Padless artwork stays.** It has no nets, and nothing could bring it back.
- **Refuses (exit 3) when a footprint draws the board outline** (#829):
  moving it would move the outline.
- **Refuses (exit 3) a routed input.** Strip it first with
  `tests/stress/strip_copper_only.py`, which keeps the outline bit-identical.

The OUTPUT is verified, not the plan: every moved part's courtyard clears the
outline, siblings (.kicad_pro/.kicad_prl/.kicad_dru) are carried, no copper,
no new stack, and every unlocked part really is unlocked. Exit 0 when all of
that holds, 1 when a check fails.
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
                          non_aperture_pads)
from placement.placement_state import assess_placement          # noqa: E402
from placement.utility import compute_footprint_bbox_local       # noqa: E402
from placement.portfolio import copy_siblings                    # noqa: E402
from placement.writer import write_placed_output                 # noqa: E402

CLUSTER_FRAC = 0.10     # of the board diagonal; SPREAD_RATIO (s2) is 0.15
GAP_MARGIN = 2.0        # mm of clear air beyond the tallest part's reach
PILE_ROT = 0.0          # a pile carries no orientation information
SIBLINGS = ('.kicad_pro', '.kicad_prl', '.kicad_dru')


def _unlock(dst, refs):
    """Remove `(locked yes)` from the named blocks, through the parser's keys."""
    with open(dst, encoding='utf-8', newline='') as fh:
        text = fh.read()
    parts, pos, done = [], 0, []
    for start, end, fp_text, _raw, key in iter_footprint_blocks(text):
        if key not in refs or '(locked yes)' not in fp_text:
            continue
        new = re.sub(r'\n[ \t]*\(locked yes\)[ \t]*(?=\r?\n)', '', fp_text,
                     count=1)
        if new != fp_text:
            parts += [text[pos:start], new]
            pos = end
            done.append(key)
    parts.append(text[pos:])
    with open(dst, 'w', encoding='utf-8', newline='') as fh:
        fh.write(''.join(parts))
    return sorted(done)


def build(src: str, dst: str, keep_locked: bool = False) -> int:
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
                   'new_rotation': PILE_ROT}
                  for i, ref in enumerate(movable)]
    if not write_placed_output(src, dst, placements, pcb_data=pcb):
        print('refuse: write_placed_output returned False', file=sys.stderr)
        return 1
    unlocked = _unlock(dst, set(movable))
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

    print(f'board      {max_x - min_x:.2f} x {max_y - min_y:.2f} mm')
    print(f'moved      {len(movable)} pad-bearing part(s) to a pile at '
          f'({cx:.2f}, {cy:.2f}); closest to the outline: '
          f'{worst[0] if worst else "-"} at '
          f'{worst[1] if worst else float("nan"):.2f} mm below it')
    print(f'unlocked   {len(unlocked)}: {", ".join(unlocked) or "-"}')
    print(f'held       {len(held)} locked part(s), {len(artwork)} padless artwork block(s)')
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
    a = ap.parse_args(argv)
    return build(a.src, a.dst, a.keep_locked)


if __name__ == '__main__':
    sys.exit(main())
