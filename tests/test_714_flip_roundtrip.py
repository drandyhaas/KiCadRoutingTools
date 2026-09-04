#!/usr/bin/env python3
"""#714 RT: flip, flip back, and the bytes must come back exactly.

The issue proposes this as an acceptance criterion, and it is the only gate in
the set that can see a transform which is *self-consistent but not an
involution* -- a coordinate negated through `float` instead of by toggling the
sign character, which rewrites `0.500` as `0.5`, or a `(justify mirror)` token
that is added on the way out and never removed on the way back.

WHAT "BYTE-IDENTICAL" IS MEASURED AGAINST, and it is not the input board.
`write_placed_output` runs `move_copper_text_to_silkscreen` over the whole file
and reformats `(at)` to six decimals for every ref it is handed, so a raw
comparison fails for reasons that have nothing to do with the flip. The baseline
is the DOSE-0 PASS OF THE SAME WRITER -- exactly the control board
`perturb.perturb` builds before it perturbs anything, and for exactly this
reason.

WHY IT IS NOT VACUOUS, in four arms, all required:

  1. `sha256(flipped) != sha256(control)`, ASSERTED FIRST. Without this a writer
     that ignores the side key entirely round-trips perfectly. It is the single
     most important line in the file, and it is the arm that is red today.
  2. The delta between control and flipped is CONFINED to the target block's
     span. Nothing else in the file may move.
  3. A FLOOR on how much of the block changed, per surface. A flip that rewrote
     only the footprint's `(layer ...)` line round-trips flawlessly and is the
     exact silent failure #714 is about.
  4. It runs on fixtures that already carry `(justify mirror)`, so the
     add-but-never-remove asymmetry has somewhere to show up.

DIRECTION IS PINNED. Measured (`tests/measure_714_mirror_convention.py`
section F): `FLIP_DIRECTION_TOP_BOTTOM` is an involution on 15/15 fixtures;
`LEFT_RIGHT` is not, on 4 of them, because its text-angle rule collapses to
`a mod 180` (glasgow_revC SW1 180 -> 0, orangecrab_ext_pll J4 270 -> 90). So
byte-identity is statable only for TOP_BOTTOM -- which is the argument for the
writer supporting no other direction.

    python3 tests/test_714_flip_roundtrip.py
"""
from __future__ import annotations

import hashlib
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)

RUN_ALL_TIMEOUT = 600

# `orangecrab_ext_pll J4` and `rp2350 J2` already carry `(justify mirror)`, so
# the removal branch is exercised rather than only the insertion branch.
FIXTURES = [
    ('tigard', 'J7'),
    ('tigard', 'J1'),
    ('glasgow_revC', 'SW1'),
    ('orangecrab_ext_pll', 'J4'),
    ('rp2350_fpga_eensy_prePlane', 'J2'),
    ('rp2350_fpga_eensy_prePlane', 'SW1'),
]

# Per-surface floors on how much of the block a real flip must change. A flip
# that rewrote one line would satisfy an involution perfectly.
MIN_CHANGED_LINES = 4


def _sha(path):
    with open(path, 'rb') as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def _control(src, dst):
    """The dose-0 pass of the same writer: every part at the pose it has."""
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    pcb = parse_kicad_pcb(src)
    write_placed_output(src, dst, [
        {'reference': r, 'new_x': round(f.x, 6), 'new_y': round(f.y, 6),
         'new_rotation': f.rotation} for r, f in sorted(pcb.footprints.items())])
    return dst


def _flip(src, dst, ref, side):
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    fp = parse_kicad_pcb(src).footprints[ref]
    write_placed_output(src, dst, [{
        'reference': ref, 'new_x': round(fp.x, 6), 'new_y': round(fp.y, 6),
        'new_rotation': fp.rotation, 'new_side': side}])
    return dst


def _block_span(path, ref):
    from kicad_parser import iter_footprint_blocks
    text = open(path, encoding='utf-8').read()
    for s, e, _t, _raw, key in iter_footprint_blocks(text):
        if key == ref:
            return text, s, e
    return text, None, None


def main():
    from kicad_parser import parse_kicad_pcb

    tmp = tempfile.mkdtemp(prefix='krt714rt')
    problems, checked = [], 0
    try:
        for board, ref in FIXTURES:
            src = os.path.join(REPO, 'kicad_files', board + '.kicad_pcb')
            if not os.path.isfile(src):
                problems.append(f"{board}: board not tracked")
                continue
            fp0 = parse_kicad_pcb(src).footprints.get(ref)
            if fp0 is None:
                problems.append(f"{board}:{ref} missing -- fixture stale")
                continue
            here = (fp0.layer or 'F.Cu')[0]
            other = 'F' if here == 'B' else 'B'

            ctrl = _control(src, os.path.join(tmp, f"{board}_{ref}_ctrl.kicad_pcb"))
            f1 = _flip(ctrl, os.path.join(tmp, f"{board}_{ref}_f1.kicad_pcb"),
                       ref, other)
            f2 = _flip(f1, os.path.join(tmp, f"{board}_{ref}_f2.kicad_pcb"),
                       ref, here)

            h_ctrl, h_f1, h_f2 = _sha(ctrl), _sha(f1), _sha(f2)

            # --- arm 1: the flip did something. Red today; without it every
            # other arm passes on a writer that ignores the side key.
            if h_f1 == h_ctrl:
                problems.append(
                    f"{board}:{ref} the flipped board is byte-identical to the "
                    f"control -- nothing was flipped, so the round trip below "
                    f"would pass for the wrong reason")
                continue
            checked += 1

            # --- arm 2: the delta is confined to this block
            t_ctrl, cs, ce = _block_span(ctrl, ref)
            t_f1, fs, fe = _block_span(f1, ref)
            if cs is None or fs is None:
                problems.append(f"{board}:{ref} block not locatable in the output")
            else:
                if t_ctrl[:cs] != t_f1[:fs] or t_ctrl[ce:] != t_f1[fe:]:
                    problems.append(
                        f"{board}:{ref} the flip changed bytes OUTSIDE its own "
                        f"footprint block")
                # --- arm 3: enough of the block actually changed
                a = t_ctrl[cs:ce].splitlines()
                b = t_f1[fs:fe].splitlines()
                changed = sum(1 for x, y in zip(a, b) if x != y) + abs(len(a) - len(b))
                if changed < MIN_CHANGED_LINES:
                    problems.append(
                        f"{board}:{ref} only {changed} line(s) of the block "
                        f"changed -- a flip that rewrites one line round-trips "
                        f"perfectly and is the silent failure this gate exists "
                        f"for")

            # --- arm 4: and back again, exactly
            if h_f2 != h_ctrl:
                problems.append(
                    f"{board}:{ref} flip-and-flip-back is NOT byte-identical\n"
                    f"      control {h_ctrl}\n      round   {h_f2}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if checked == 0 and not problems:
        problems.append("no fixture round-tripped -- nothing was graded")

    if problems:
        print(f"FAIL: {len(problems)} problem(s)")
        for p in problems[:25]:
            print("  " + p)
        return 1
    print(f"PASS: {checked} fixtures flip and flip back byte-identically")
    return 0


if __name__ == '__main__':
    sys.exit(main())
