#!/usr/bin/env python3
"""#714 SELF: a flipped board must not contradict itself about which face it is on.

The failure mode #714 is written around is silent. `legality.footprint_side`
decides a part's side from `Footprint.layer`'s first character ALONE, with no
pad cross-check, so a block whose `(layer ...)` says B.Cu while its pads still
say F.Cu reports side B and computes every overlap, courtyard and clearance
number from F geometry. `perturb.py`'s module docstring names exactly this:

    A "flip" through it produces a board whose pads contradict its side, which
    corrupts `footprint_side` / `sides_occupied` and therefore every overlap
    number downstream.

This gate is the cheapest thing that can see it: no pcbnew, no golden file,
milliseconds. It compares two derivations of a part's side that are wholly
independent of each other and that NOTHING IN THE REPO COMPARES TODAY --

    legality.PartPads.side       from fp.layer[0]          (legality.py:239-242)
    legality.PartPads.pad_sides  from the pads' (layers)   (legality.py:2087-2159)

THE ARM THAT MAKES IT RED, and the reason it is listed first: a *consistency*
check alone passes vacuously on the shipped writer, because a writer that
ignores the side key emits the input board, which is perfectly consistent about
being on the front. So D4 -- "the flip actually took" -- is asserted BEFORE
consistency, and it is what fails on `upstream/main`.

THE COURTYARD ARM is the one that is easy to leave out and load-bearing.
`parser.courtyard_for_side` FALLS BACK to the other side's box when only one is
drawn, so a footprint whose F.CrtYd stayed on F while the block moved to B
still hands every placement consumer a plausible box. That fallback is
precisely how a half-done flip stays silent, so the geometry side is asserted
directly rather than through the accessor.

    python3 tests/test_714_side_self_consistency.py
"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)

RUN_ALL_TIMEOUT = 300

# (board, ref). Chosen so the SMD-only floor below is met without counting a
# through-hole part, whose pads legitimately occupy both faces and therefore
# carry no side claim to contradict.
FIXTURES = [
    ('tigard', 'J7'),
    ('tigard', 'U3'),
    ('glasgow_revC', 'U7'),
    ('glasgow_revC', 'SW1'),
    ('glasgow_revC', 'U1'),
    ('orangecrab_ext_pll', 'J4'),
    ('orangecrab_ext_pll', 'U9'),
    ('rp2350_fpga_eensy_prePlane', 'SW1'),
]

# A gate that graded two SMD parts is not a gate. Below the fixture count so
# one board changing is not a failure; far above zero so a vacuous run is.
MIN_SMD_GRADED = 6


def _flip_one(src, dst, ref, new_side):
    """Ask the writer for a side change, holding x/y/rot exactly."""
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    fp = parse_kicad_pcb(src).footprints[ref]
    write_placed_output(src, dst, [{
        'reference': ref, 'new_x': round(fp.x, 6), 'new_y': round(fp.y, 6),
        'new_rotation': fp.rotation, 'new_side': new_side}])
    return dst


def main():
    from kicad_parser import parse_kicad_pcb
    from placement import parser as pparser
    from placement.legality import PartPads, footprint_side

    tmp = tempfile.mkdtemp(prefix='krt714self')
    problems, smd_graded, checked = [], 0, 0
    try:
        for board, ref in FIXTURES:
            src = os.path.join(REPO, 'kicad_files', board + '.kicad_pcb')
            if not os.path.isfile(src):
                problems.append(f"{board}: fixture board missing")
                continue
            before = parse_kicad_pcb(src).footprints.get(ref)
            if before is None:
                problems.append(f"{board}:{ref} not in the board -- fixture stale")
                continue
            want = 'F' if footprint_side(before) == 'B' else 'B'

            dst = os.path.join(tmp, f"{board}__{ref}.kicad_pcb")
            _flip_one(src, dst, ref, want)
            after = parse_kicad_pcb(dst).footprints.get(ref)
            if after is None:
                problems.append(f"{board}:{ref} vanished from the output")
                continue
            checked += 1

            # ---- D4: the flip TOOK. Asserted first; this is the arm that is
            # red on the shipped writer, and without it every check below
            # passes on an untouched board.
            got = footprint_side(after)
            if got != want:
                problems.append(
                    f"{board}:{ref} asked for side {want}, got {got} "
                    f"(layer {after.layer!r}) -- the writer did not flip")
                continue

            # ---- the contradiction check proper
            pp = PartPads(after, clearance=0.2)
            if pp.side != want:
                problems.append(f"{board}:{ref} PartPads.side {pp.side} != {want}")
            if pp.pad_sides == {'F', 'B'}:
                # Through-hole or *.Cu: the pads legitimately occupy both
                # faces and make no claim this gate can contradict.
                pass
            else:
                smd_graded += 1
                if pp.pad_sides != {want}:
                    problems.append(
                        f"{board}:{ref} SIDE CONTRADICTION: fp.layer says "
                        f"{pp.side}, the pads' own (layers) say "
                        f"{sorted(pp.pad_sides)} -- every overlap number for "
                        f"this part is now computed from the wrong face")

            # ---- the graphic channel. Asserted DIRECTLY, not through
            # courtyard_for_side, whose fallback would hide it.
            for label, fn in (('courtyard', pparser.extract_courtyard_sides),
                              ('fab', pparser.extract_fab_sides)):
                sides = fn(dst).get(ref)
                if not sides:
                    continue  # the library drew none; nothing to contradict
                if want not in sides:
                    problems.append(
                        f"{board}:{ref} {label} geometry is on "
                        f"{sorted(sides)} but the part is on {want} -- "
                        f"courtyard_for_side will silently serve the other "
                        f"side's box to every placement consumer")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if checked < MIN_SMD_GRADED:
        problems.append(f"only {checked} fixture(s) reached the checks -- "
                        f"this gate cannot pass on what it did not grade")
    if smd_graded < MIN_SMD_GRADED and not problems:
        problems.append(f"only {smd_graded} SMD-only part(s) carried a side "
                        f"claim; the contradiction check is near-vacuous")

    if problems:
        print(f"FAIL: {len(problems)} problem(s)")
        for p in problems[:25]:
            print("  " + p)
        return 1
    print(f"PASS: {checked} flipped parts self-consistent "
          f"({smd_graded} SMD-only side claims graded)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
