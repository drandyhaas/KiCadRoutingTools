#!/usr/bin/env python3
"""#714 VAC: the mirror is asserted with numbers, on fixtures PROVEN to discriminate.

Measured on the 22 tracked boards (`tests/measure_714_mirror_convention.py`
section C): **1198 of 1319 pad-bearing footprints -- 90.8% -- have a pad
POSITION multiset that is closed under `y -> -y`.** For those parts the whole
local-y mirror is a no-op, so a fixture drawn from ordinary 0402s, 0603s and
symmetric QFNs passes every geometric assertion **with the mirror deleted
entirely**. That is the trap this file exists to not fall into.

So the fixtures are not asserted through; they are FIRST PROVEN to discriminate,
per surface, and the run REFUSES if the set has gone vacuous:

    D1  the y-mirror is observable        mirror_y(pads) != pads
    D2  a wrong-AXIS mutant is observable mirror_y(pads) != mirror_x(pads)
    D4  the flip took at all              the emitted (layer ...) changed

D1 is evaluated on the full pad TUPLE `(number, type, shape, x, y, angle)`, not
on positions. That is the predicate that answers "is the mirror observable",
and it is a different number: 532/1319 = 40.3% symmetric, against 90.8% by
position. A mirror does not renumber pads, so a part with pad 1 at (0,-2) and
pad 2 at (0,+2) has a symmetric position set and a plainly observable flip.
Grading discrimination on positions would write off `tigard U3` and every
non-orthogonal fixture as useless when they are not.

D2 matters because only two fixtures are x-asymmetric. Drop `glasgow_revC U7`
or `orangecrab_ext_pll J4` and the "mirror the wrong axis" mutation goes green
for free.

THE ROTATION CONTRACT, asserted here because it is the one place this writer
and pcbnew deliberately differ: `new_rotation` stays the FINAL footprint
orientation and the writer does not secretly negate it. With `a = R + p` (p the
pad-local angle) and `p -> -p` under a flip, a pad's stored angle must become
`a' = R_new - (a - R_old)`. Holding rotation constant that is `2R - a`; it is
NOT `a + delta_rot`, which is what `_rotate_pad_angles` computes and which
agrees only when p is 0 or 180.

    python3 tests/test_714_mirror_discriminates.py
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

FIXTURES = [
    ('tigard', 'J7'),
    ('tigard', 'J1'),
    ('tigard', 'U3'),
    ('glasgow_revC', 'U7'),
    ('glasgow_revC', 'SW1'),
    ('orangecrab_ext_pll', 'J4'),
    ('orangecrab_ext_pll', 'U9'),
    ('rp2350_fpga_eensy_prePlane', 'U8'),
]

# Named anchors, measured against pcbnew 10.0.0. Stated as exact values so a
# mirror that is merely "different" cannot pass: `x` is pinned as hard as `y`,
# which is what kills a mutant that negates the wrong axis.
#   (board, ref, pad_number, (x_before, y_before), (x_after, y_after))
ANCHORS = [
    ('tigard', 'J7', '1', (-1.5, -2.0), (-1.5, 2.0)),
    ('tigard', 'J7', '2', (-0.5, -2.0), (-0.5, 2.0)),
]


def _tuple_key(p):
    return (p.pad_number, p.pad_type, p.shape,
            round(p.local_x, 6), round(p.local_y, 6),
            round((getattr(p, 'rotation', 0.0) or 0.0) % 360, 4))


def _mirror_key(k, axis):
    x, y, a = k[3], k[4], k[5]
    if axis == 'y':
        y = -y
    else:
        x = -x
    return (k[0], k[1], k[2], round(x, 6), round(y, 6), round((-a) % 360, 4))


def _keys(fp):
    return sorted(_tuple_key(p) for p in (fp.pads or []))


def preflight(fixtures, parse):
    """Prove the fixture set can see what the assertions below claim to test.

    Returns (rows, problems). A fixture set that cannot discriminate is a
    refusal, not a pass -- 90.8% of real footprints are y-mirror no-ops by
    position, so "the tests passed" carries almost no information until this
    has run.
    """
    rows, problems = [], []
    for board, ref in fixtures:
        src = os.path.join(REPO, 'kicad_files', board + '.kicad_pcb')
        if not os.path.isfile(src):
            problems.append(f"{board}: board not tracked")
            continue
        fp = parse(src).footprints.get(ref)
        if fp is None:
            problems.append(f"{board}:{ref} missing -- fixture stale")
            continue
        ks = _keys(fp)
        my = sorted(_mirror_key(k, 'y') for k in ks)
        mx = sorted(_mirror_key(k, 'x') for k in ks)
        rows.append({'board': board, 'ref': ref, 'pads': len(ks),
                     'rot': fp.rotation, 'side': (fp.layer or 'F')[0],
                     'D1': ks != my, 'D2': my != mx})
    if not any(r['D1'] for r in rows):
        problems.append("NO fixture discriminates the y-mirror (D1). Every "
                        "assertion below would pass with the mirror deleted.")
    if not any(r['D2'] for r in rows):
        problems.append("NO fixture discriminates the mirror AXIS (D2). A "
                        "mutant that negates x instead of y would pass.")
    if not any(r['side'] == 'B' for r in rows):
        problems.append("NO fixture starts on B.Cu. A direction-asymmetric "
                        "rule would pass.")
    if not any(abs((r['rot'] or 0) % 180) not in (0.0,) for r in rows):
        problems.append("NO fixture is at a rotation where the flip composes "
                        "with a non-trivial angle.")
    return rows, problems


def main():
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output

    rows, problems = preflight(FIXTURES, parse_kicad_pcb)
    print("pre-flight -- does this fixture set discriminate?")
    for r in rows:
        print(f"   {r['board']:<28} {r['ref']:<5} pads={r['pads']:3d} "
              f"rot={str(r['rot']):>6} side={r['side']} "
              f"D1(y-mirror)={r['D1']} D2(axis)={r['D2']}")
    if problems:
        print("REFUSING to assert through a fixture set that cannot see the "
              "thing being asserted:")
        for p in problems:
            print("  " + p)
        return 1

    tmp = tempfile.mkdtemp(prefix='krt714vac')
    checked = 0
    try:
        for board, ref in FIXTURES:
            src = os.path.join(REPO, 'kicad_files', board + '.kicad_pcb')
            before = parse_kicad_pcb(src).footprints[ref]
            want = 'F' if (before.layer or 'F').startswith('B') else 'B'
            dst = os.path.join(tmp, f"{board}__{ref}.kicad_pcb")
            write_placed_output(src, dst, [{
                'reference': ref, 'new_x': round(before.x, 6),
                'new_y': round(before.y, 6), 'new_rotation': before.rotation,
                'new_side': want}])
            after = parse_kicad_pcb(dst).footprints.get(ref)
            if after is None:
                problems.append(f"{board}:{ref} vanished from the output")
                continue

            if (after.layer or 'F')[0] != want:
                problems.append(f"{board}:{ref} layer {after.layer!r} -- asked "
                                f"for side {want}; the writer did not flip")
                continue
            checked += 1

            # The caller owns rotation: it must land exactly as handed over.
            if round((after.rotation or 0) % 360, 4) != round(
                    (before.rotation or 0) % 360, 4):
                problems.append(f"{board}:{ref} rotation {before.rotation} -> "
                                f"{after.rotation}; the writer negated a "
                                f"rotation the caller owns")

            bpads = {p.pad_number: p for p in (before.pads or [])}
            R = round((before.rotation or 0.0) % 360, 4)
            for p in (after.pads or []):
                q = bpads.get(p.pad_number)
                if q is None:
                    continue
                if round(p.local_x, 6) != round(q.local_x, 6):
                    problems.append(
                        f"{board}:{ref} pad {p.pad_number} local X moved "
                        f"{q.local_x} -> {p.local_x}; a flip mirrors Y only")
                if round(p.local_y, 6) != round(-q.local_y, 6):
                    problems.append(
                        f"{board}:{ref} pad {p.pad_number} local Y "
                        f"{q.local_y} -> {p.local_y}, expected "
                        f"{round(-q.local_y, 6)}")
                want_a = round((2 * R - (q.rotation or 0.0)) % 360, 4)
                got_a = round((p.rotation or 0.0) % 360, 4)
                if got_a != want_a:
                    problems.append(
                        f"{board}:{ref} pad {p.pad_number} angle {q.rotation} "
                        f"-> {p.rotation}, expected {want_a} "
                        f"(a' = R_new - (a - R_old); NOT a + delta_rot)")
                for lay in (p.layers or []):
                    s = str(lay)
                    if s.startswith('*'):
                        continue          # wildcards are not sided
                    if s.startswith(('F.', 'B.')) and s[0] != want:
                        problems.append(
                            f"{board}:{ref} pad {p.pad_number} still on {s} "
                            f"after a flip to {want}")
                        break
                # A wildcard must survive VERBATIM -- `*.Cu` becoming `B*.Cu`
                # or `B.Cu` is a real corpus hazard (rp2350 U8, 66 pads).
                for a, b in zip(sorted(str(l) for l in (q.layers or [])),
                                sorted(str(l) for l in (p.layers or []))):
                    if a.startswith('*') and a != b:
                        problems.append(f"{board}:{ref} pad {p.pad_number} "
                                        f"wildcard {a} rewritten to {b}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # The named anchors, checked last so their failure is not buried.
    for board, ref, num, exp_before, exp_after in ANCHORS:
        src = os.path.join(REPO, 'kicad_files', board + '.kicad_pcb')
        fp = parse_kicad_pcb(src).footprints.get(ref)
        got = next((p for p in (fp.pads or []) if p.pad_number == num), None)
        if got is None:
            problems.append(f"anchor {board}:{ref} pad {num} is gone -- "
                            f"re-anchor rather than delete")
        elif (round(got.local_x, 6), round(got.local_y, 6)) != exp_before:
            problems.append(
                f"anchor {board}:{ref} pad {num} is at "
                f"({got.local_x}, {got.local_y}), recorded as {exp_before} -- "
                f"the board moved under the anchor")

    if checked == 0:
        problems.append("no fixture reached the geometry checks")

    if problems:
        print(f"\nFAIL: {len(problems)} problem(s)")
        for p in problems[:25]:
            print("  " + p)
        return 1
    print(f"\nPASS: mirror asserted on {checked} fixtures proven to discriminate")
    return 0


if __name__ == '__main__':
    sys.exit(main())
