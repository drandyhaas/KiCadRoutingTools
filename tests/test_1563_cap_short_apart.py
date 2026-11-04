"""#1563: place_fanout_clearance must not leave a cap SHORTED to foreign
copper when a pose apart from it exists, and must name a cap it cannot part.

The pass had two ways off foreign copper: the descent, whose per-net gate
(#441) refuses any pose grazing a net more than the seed did, and the
fallback, which takes only a pose clear of everything. Under a ball field
dense enough that no pose is clear, a cap whose pad sat ON a foreign via kept
it: every pose off the via grazed some other net, which the gate refused, and
none was clear, which the fallback required. It shipped the short and
reported it in the same Unresolved list as a graze.

The rig: a real decoupling cap with a lattice of foreign vias around it, each
its own net, one on the cap's pad 1. The truth is check_drc's distance to the
REAL pads at the pose the pass returns, independent of the pass's own model.

1. A lattice no pose clears but a pose apart from every via exists: the cap
   is moved apart (no via inside its copper), named in `parted`, and not
   in `shorted`; it still grazes, so it stays unresolved.
2. A lattice too dense for any pose apart: the cap is named in `shorted`.

Runs in-process in a few seconds; nothing shells out.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True

import contextlib
import io
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

# Declares no netclass, no .kicad_dru and no pad override: the clearance model
# is inert, so a via's keep-out is its radius + CLEAR and nothing else moves.
BOARD = os.path.join(_ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
CLEAR = 0.1
CAP = 'C16'
BGA = 'U2'           # the ball field C16 is near; every other part is removed
VIA = 0.3
NET0 = 5000          # lattice nets: NET0, NET0+1, ... -- none of the board's


def _copper(fp):
    return [p for p in fp.pads if any(str(l).endswith('.Cu') for l in p.layers)]


def _run(pitch):
    """The board cut to the BGA and the cap, with a square lattice of foreign
    vias at `pitch` around the cap, one centred on its pad 1, as the only
    fanout copper. The via-nudge (#313, the rung AFTER the fallback) is held
    off: these vias carry no escape, so it would simply move them, and the
    question here is what the cap move does. Returns (result, seed
    footprint, vias)."""
    pcb = parse_kicad_pcb(BOARD)
    fp = pcb.footprints[CAP]
    for ref in list(pcb.footprints):
        if ref not in (CAP, BGA):
            del pcb.footprints[ref]
    p1 = _copper(fp)[0]
    vias = []
    n = int(4.5 / pitch)
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            vias.append(make_via(p1.global_x + i * pitch, p1.global_y + j * pitch,
                                 net_id=NET0 + len(vias), size=VIA))
    pcb.vias[:] = vias
    pcb.segments[:] = []
    nudge = FC.nudge_vias_for_unresolved
    FC.nudge_vias_for_unresolved = lambda *a, **k: ([], [])
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            res = repair_fanout_clearance(pcb, BOARD, clearance=CLEAR)
    finally:
        FC.nudge_vias_for_unresolved = nudge
    return res, fp, vias


def _pose(res, fp):
    for m in res['placements']:
        if m['reference'] == CAP:
            return (m['new_x'], m['new_y'], m['new_rotation'])
    return (fp.x, fp.y, fp.rotation)


def _touching(fp, pose, vias):
    """Vias whose barrel overlaps the cap's REAL copper at `pose`."""
    pads = _copper(footprint_at_pose(fp, pose))
    return [(round(v.x, 3), round(v.y, 3)) for v in vias for p in pads
            if v.net_id != p.net_id
            and point_to_pad_distance(v.x, v.y, p) < VIA / 2.0 - 1e-6]


class TestTheRigIsAShort(unittest.TestCase):
    def test_the_seed_pad_sits_on_a_foreign_via(self):
        pcb = parse_kicad_pcb(BOARD)
        fp = pcb.footprints[CAP]
        p1 = _copper(fp)[0]
        v = make_via(p1.global_x, p1.global_y, net_id=NET0, size=VIA)
        self.assertTrue(_touching(fp, (fp.x, fp.y, fp.rotation), [v]),
                        'the seed is not in contact: this file tests nothing')


class TestAPoseApartIsTaken(unittest.TestCase):
    PITCH = 0.7      # no pose clears; some are apart, all grazing a new net

    def test_the_cap_is_moved_apart_from_every_via(self):
        res, fp, vias = _run(self.PITCH)
        pose = _pose(res, fp)
        self.assertEqual(_touching(fp, pose, vias), [],
                         'the cap still shorts a foreign via at %s' % (pose,))
        self.assertIn(CAP, res.get('parted', []))
        self.assertNotIn(CAP, res.get('shorted', []))
        # apart is not clear: it grazes, and says so
        self.assertIn(CAP, res['unresolved'])
    # MUTATION: drop the #1563 rung (`best, broke = _best_apart(...)` ->
    # `best, broke = None, None`) -> the cap stays on its via.


class TestACapThatCannotPartIsNamed(unittest.TestCase):
    PITCH = 0.6      # every pose touches some via

    def test_the_cap_is_named_shorted(self):
        res, fp, vias = _run(self.PITCH)
        self.assertTrue(_touching(fp, _pose(res, fp), vias),
                        'the rig parted the cap: it no longer tests a boxed one')
        self.assertIn(CAP, res.get('shorted', []))
        self.assertNotIn(CAP, res.get('parted', []))


if __name__ == '__main__':
    unittest.main()
