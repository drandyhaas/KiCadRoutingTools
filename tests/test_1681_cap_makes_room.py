"""#1681: a part must not end on foreign copper while a clear pose exists that
only another mover stands in, and a part moved off a short is named.

The sweep moves each cap with every other held where it stands, so a part that
gets to its room last finds it taken. Every graze-free pose such a part had was
blocked by other movers alone, and it stopped inside a via's clearance (or on
the via) with the fallback reporting nothing better. The fallback now also looks
for a clear pose ONE other mover could vacate, and moves that mover to a clear
pose of its own.

The rig: a real board cut to a BGA and two caps, in a lattice of foreign vias
dense enough that every pose touches some via, with two holes cut in it. Cap A
sits on a via. Hole 1 is within A's displacement cap and holds cap B; hole 2 is
within B's cap and out of A's. The truth is check_drc's distance to the REAL
pads at the poses the pass returns, independent of the pass's own model.

1. Both holes: A takes hole 1, B moves to hole 2, neither touches nor grazes a
   via, nothing is unresolved, and `made_room` names the move.
2. No hole 2: B has nowhere to go, so nothing is moved to make room, B stays
   clear and A is named `shorted`.
3. A cap the DESCENT walked off a short into a graze is on the `parted` line,
   which used to name only the moves the fallback made: all the lattice on one
   net, so the descent's per-net gate lets it through.

Runs in-process in about a minute; nothing shells out.
"""
from __future__ import annotations

import contextlib
import io
import math
import os
import sys
import unittest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

from kicad_parser import parse_kicad_pcb
from check_drc import point_to_pad_distance
from synth import make_via
from placement.legality import footprint_at_pose
import placement.fanout_clearance as FC
from placement.fanout_clearance import repair_fanout_clearance

BOARD = os.path.join(_ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
CLEAR = 0.1
A, B, BGA = 'C16', 'C17', 'U2'
VIA = 0.3
PITCH = 0.6          # every pose of a cap touches a via of this lattice
NET0 = 5000
HOLE1 = (1.9, 0.0)   # offsets from A's seed: inside A's 3 mm cap
HOLE2 = (4.6, 0.0)   # 2.7 from hole 1, so inside B's cap and out of A's


def _copper(fp):
    return [p for p in fp.pads if any(str(l).endswith('.Cu') for l in p.layers)]


def _in_hole(x, y, c):
    """A via this close to a hole's centre would graze a cap in it."""
    return abs(x - c[0]) < 0.55 + VIA / 2 + CLEAR and abs(y - c[1]) < 0.2 + VIA / 2 + CLEAR


def _run(holes, one_net=False, pitch=PITCH):
    """The cut board with B parked in hole 1, the lattice around both, one via
    centred on A's pad 1. The via nudge (#313, the rung after this one) is held
    off: these vias carry no escape, so it would simply move them. Returns
    (result, seed A, seed B, B's parked pose, vias)."""
    pcb = parse_kicad_pcb(BOARD)
    fa, fb = pcb.footprints[A], pcb.footprints[B]
    for ref in list(pcb.footprints):
        if ref not in (A, B, BGA):
            del pcb.footprints[ref]
    p1 = _copper(fa)[0]
    park = (fa.x + HOLE1[0], fa.y + HOLE1[1], fa.rotation)
    fb_parked = footprint_at_pose(fb, park)
    pcb.footprints[B] = fb_parked
    for p in _copper(fb):
        pcb.pads_by_net[p.net_id] = [q for q in pcb.pads_by_net.get(p.net_id, ())
                                     if q is not p]
    for p in _copper(fb_parked):
        pcb.pads_by_net.setdefault(p.net_id, []).append(p)
    centres = [(fa.x + dx, fa.y + dy) for dx, dy in holes]
    vias = []
    n = int(9.6 / pitch)
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x, y = p1.global_x + i * pitch, p1.global_y + j * pitch
            if any(_in_hole(x, y, c) for c in centres):
                continue
            vias.append(make_via(x, y, net_id=NET0 + (0 if one_net else len(vias)),
                                 size=VIA))
    pcb.vias[:] = vias
    pcb.segments[:] = []
    nudge = FC.nudge_vias_for_unresolved
    FC.nudge_vias_for_unresolved = lambda *a, **k: ([], [])
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            res = repair_fanout_clearance(pcb, BOARD, clearance=CLEAR,
                                          near_margin=8.0)
    finally:
        FC.nudge_vias_for_unresolved = nudge
    return res, fa, fb_parked, vias


def _pose(res, fp, ref):
    for m in res['placements']:
        if m['reference'] == ref:
            return (m['new_x'], m['new_y'], m['new_rotation'])
    return (fp.x, fp.y, fp.rotation)


def _gaps(fp, pose, vias):
    """Edge gap from each via to the cap's REAL copper at `pose`, smallest
    first; negative is contact."""
    pads = _copper(footprint_at_pose(fp, pose))
    return sorted(point_to_pad_distance(v.x, v.y, p) - v.size / 2.0
                  for v in vias for p in pads if v.net_id != p.net_id)


class TestTheRig(unittest.TestCase):
    def test_a_starts_on_a_via_and_b_starts_clear(self):
        res, fa, fb, vias = _run([HOLE1, HOLE2])
        self.assertLess(_gaps(fa, (fa.x, fa.y, fa.rotation), vias)[0], 0.0,
                        'A does not start on a via: this file tests nothing')
        self.assertGreaterEqual(_gaps(fb, (fb.x, fb.y, fb.rotation), vias)[0],
                                CLEAR, 'B does not start clear in its hole')
        self.assertGreater(math.hypot(*HOLE2), 3.0,
                           'hole 2 is inside A\'s cap: A would just take it')


class TestAMoverMakesRoom(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res, cls.fa, cls.fb, cls.vias = _run([HOLE1, HOLE2])

    def test_both_end_clear(self):
        for fp, ref in ((self.fa, A), (self.fb, B)):
            pose = _pose(self.res, fp, ref)
            self.assertGreaterEqual(_gaps(fp, pose, self.vias)[0], CLEAR - 1e-6,
                                    '%s ends inside a via clearance at %s'
                                    % (ref, pose,))
        self.assertEqual(self.res['unresolved'], [])
        self.assertEqual(self.res.get('shorted'), [])

    def test_the_move_is_named(self):
        self.assertEqual(self.res.get('made_room'), {A: B})
    # MUTATION: `if best is None:` -> `if False:` at the #1681 search: A stays
    # on its via, `shorted` names it, and test_both_end_clear fails.


class TestNoRoomMeansNoMove(unittest.TestCase):
    def test_b_is_not_moved_for_nothing_and_a_is_named(self):
        res, fa, fb, vias = _run([HOLE1])
        self.assertEqual(res.get('made_room'), {})
        self.assertEqual(_pose(res, fb, B), (fb.x, fb.y, fb.rotation),
                         'B left its hole with nowhere clear to go')
        self.assertIn(A, res['shorted'])


class TestADescentMoveOffAShortIsNamed(unittest.TestCase):
    def test_parted_names_the_descent_too(self):
        res, fa, _fb, vias = _run([], one_net=True, pitch=0.7)
        pose = _pose(res, fa, A)
        gap = _gaps(fa, pose, vias)[0]
        self.assertGreaterEqual(gap, 0.0, 'the rig left A on a via: %r' % gap)
        self.assertLess(gap, CLEAR, 'the rig cleared A: it tests no graze')
        self.assertIn(A, res['unresolved'])
        self.assertIn(A, res['parted'])
    # MUTATION: `parted` back to the fallback's own list -> this cap, which the
    # descent moved, is on no line but Unresolved.


if __name__ == '__main__':
    unittest.main()
