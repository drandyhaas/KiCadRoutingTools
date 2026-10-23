#!/usr/bin/env python3
"""`quench._Part.pad_globals` is memoised; it must stay BIT-IDENTICAL to
`kicad_parser.local_to_global`, pad by pad, at every pose.

The memo (a per-rotation offset table plus the last pose's answer) took the
pad transform out of the hot path -- it was 78% of test_431's routed_output
quench. The danger in a memo of float geometry is a value that agrees to
fifteen digits and differs in the last bit, which is enough to flip an MST
tie-break and change a placement. So this compares `float.hex`, not values
within a tolerance, against the un-memoised reference computed right here
from `local_to_global` -- which also makes this the tripwire if
`local_to_global` itself ever changes and the memo's copy of its arithmetic
does not.

Poses cover what the quench and seeder hand it: the live and seed poses,
the 90-degree lattice and off-lattice angles, the equal-but-different keys
(-0.0 vs 0.0, an int vs a float, a numpy float64), an origin at 0, repeats
(memo hits) and interleaved poses (memo misses). Boards: tigard (B.Cu
jumpers), rp2350_fpga_eensy_prePlane (B.Cu parts seated at -90 and 180) and
routed_output (the parts whose cost motivated the memo).

    python3 tests/test_quench_pad_globals_memo.py
"""
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import local_to_global, parse_kicad_pcb  # noqa: E402
from placement.quench import _Part  # noqa: E402

BOARDS = ('tigard.kicad_pcb', 'rp2350_fpga_eensy_prePlane.kicad_pcb',
          'routed_output.kicad_pcb')
ROTS = (0.0, 90.0, 180.0, 270.0, -90.0, 45.0, 359.9, 30.000000001, 450.0,
        -0.0, 0, 90, np.float64(180.0), 0.1 + 0.2)


def _reference(part, x, y, rot):
    return [(*local_to_global(x, y, rot, lx, ly), n)
            for lx, ly, n in part.pads_local]


def _bits(pads):
    return [(float(gx).hex(), float(gy).hex(), n) for gx, gy, n in pads]


def _parts():
    out = []
    for name in BOARDS:
        pcb = parse_kicad_pcb(os.path.join(ROOT, 'kicad_files', name))
        for ref, fp in pcb.footprints.items():
            if any(p.net_id > 0 for p in fp.pads):
                out.append((name, _Part(ref, fp, {}, False)))
    return out


PARTS = _parts()


def _poses(part):
    yield None, None, None                       # the live pose
    yield part.seed_x, part.seed_y, part.orig_rot
    for i, rot in enumerate(ROTS):
        dx = (0.1 + 0.2) * i - 1.7
        yield part.x + dx, part.y - dx / 3.0, rot
        yield part.x + dx, part.y - dx / 3.0, rot   # a memo hit
    yield 0.0, -0.0, 90.0                        # an origin at zero
    yield -0.0, 0.0, -0.0
    yield np.float64(part.x), np.float64(part.y), 90.0
    yield int(round(part.x)), int(round(part.y)), 180


def test_parts_cover_both_sides():
    sides = {p.side for _, p in PARTS}
    assert sides == {'F', 'B'}, f"fixture: sides {sides}"
    assert max(p.pin_count for _, p in PARTS) > 100, "fixture: no big part"
    print(f"  PASS: {len(PARTS)} parts, sides {sorted(sides)}")


def test_bit_identical_to_local_to_global():
    n = 0
    for name, part in PARTS:
        for x, y, rot in _poses(part):
            got = part.pad_globals(x, y, rot)
            rx = part.x if x is None else x
            ry = part.y if y is None else y
            rr = part.rot if rot is None else rot
            want = _reference(part, rx, ry, rr)
            assert _bits(got) == _bits(want), (name, part.ref, x, y, rot)
            assert all(type(v) is float for t in got for v in t[:2])
            n += 1
    print(f"  PASS: {n} poses bit-identical to local_to_global")


def test_interleaved_poses_never_serve_a_stale_answer():
    """A, B, A, the live pose moving under it: every answer is the pose's."""
    for name, part in PARTS[:40]:
        a = (part.x + 1.25, part.y, 90.0)
        b = (part.x, part.y - 0.75, 270.0)
        for pose in (a, b, a, a, b):
            assert (_bits(part.pad_globals(*pose))
                    == _bits(_reference(part, *pose))), (name, part.ref, pose)
        saved = part.x, part.y, part.rot
        try:
            part.pad_globals()
            part.x, part.y, part.rot = a          # how the quench moves a part
            assert (_bits(part.pad_globals())
                    == _bits(_reference(part, *a))), (name, part.ref)
        finally:
            part.x, part.y, part.rot = saved
        assert (_bits(part.pad_globals())
                == _bits(_reference(part, *saved))), (name, part.ref)
    print("  PASS: interleaved and moved poses")


def test_a_reassigned_pad_list_is_never_served_from_the_memo():
    name, part = next((nm, p) for nm, p in PARTS if p.pin_count >= 2)
    saved = part.pads_local
    try:
        part.pad_globals()
        part.pad_globals(part.x, part.y, 90.0)
        part.pads_local = [(lx + 0.5, ly - 0.25, nt) for lx, ly, nt in saved]
        # The pose asked LAST first, so the pose memo would answer it; then
        # the live pose, whose rotation the offset memo already holds.
        for pose in ((part.x, part.y, 90.0), (None, None, None)):
            rx = part.x if pose[0] is None else pose[0]
            ry = part.y if pose[1] is None else pose[1]
            rr = part.rot if pose[2] is None else pose[2]
            assert (_bits(part.pad_globals(*pose))
                    == _bits(_reference(part, rx, ry, rr))), (name, part.ref)
    finally:
        part.pads_local = saved
    print("  PASS: a new pads_local list rebuilds both memos")


def test_every_call_returns_its_own_list():
    """A caller that edits its result must not edit the memo."""
    _name, part = PARTS[0]
    pose = (part.x + 0.5, part.y, part.rot)
    want = _bits(_reference(part, *pose))
    miss = part.pad_globals(*pose)        # computed: the memo is filled
    miss.clear()
    hit = part.pad_globals(*pose)         # served from the memo
    assert _bits(hit) == want, "editing a computed result edited the memo"
    hit.clear()
    again = part.pad_globals(*pose)
    assert _bits(again) == want, "editing a memo hit edited the memo"
    print("  PASS: fresh list per call")


TESTS = [
    test_parts_cover_both_sides,
    test_bit_identical_to_local_to_global,
    test_interleaved_poses_never_serve_a_stale_answer,
    test_a_reassigned_pad_list_is_never_served_from_the_memo,
    test_every_call_returns_its_own_list,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
