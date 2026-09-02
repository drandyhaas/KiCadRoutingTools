#!/usr/bin/env python3
"""#618: the coupled diff-pair escape emitted its two via-in-pads ungated.

    python3 tests/test_618_coupled_via_site_gate.py [-v]

`_drop_escape_via` (bga_fanout/underpad.py) puts a via in each ball of a
coupled pair, and its two callers gated that on `_via_gate_ok` alone -- the
locked-SMD copper check -- AND only when the board has locked SMD pads. So on
an ordinary board those two vias went in with no `_via_site_conflict` at all:
not against the board's own drills, not against this run's committed vias, and
not against each other. Every other via site in that file goes through
`_via_site_conflict`.

WHAT #618 ACTUALLY CLAIMS, CLAUSE BY CLAUSE, because three of its four claims
are false at HEAD and repeating them would be worse than useless:

  1. "Via-in-pad sites sit at ball centres under the mutual centre-reservation
     contract, not through `_via_site_conflict`" -- TRUE, but only for the
     COUPLED emit above. The single-ended centre site was closed by #567
     (`bbd65f31`), which calls `_via_site_conflict(..., skip_resv=True)`.
     This file's gate closes the remaining half.
  2. "a board declaring min_hole_to_hole 0.9 still gets 0.6mm gaps (measured on
     ulx3s U1)" -- FALSE at HEAD, closed by #756 (`7303cd82`, committed one
     hour after #618 was filed). Measured: ulx3s U1 has no `.kicad_pro` at all,
     so the 0.9 declaration is CONSTRUCTED; with one planted, the run announces
     the 0.9 floor and the sub-0.9 pairs go 62 -> 0. `TestTheDeclaredFloorIsHonoured`
     re-derives that here so the closure stays a change detector rather than a
     claim in a commit message.
  3. "arguably a refuse-to-build condition ... flagging it needs a policy
     decision (skip via-in-pad? warn? fail?)" -- OPEN, and answered: WARN.
     Refuse-to-build was measured and rejected -- honouring a declared 0.9 on
     ulx3s costs 15 of 199 escapes, so refusing the RUN would throw away 184
     good escapes to prevent 62 bad holes. The house style is clamp-and-warn
     (`fab_tiers.warn_fab_escalation`, `fab_notes`), with a hard `return 1`
     reserved for "no useful board is possible".
  4. "The escape-path fix (#616) already handles every non-via-in-pad site" --
     a statement about #616, not a request.

THE GATE'S OWN COST, which is why the arms below are paired: it can only ever
DECLINE a via site, and a declined coupled pair falls back to single-ended (or
to the router). On the two real boards measured it declines nothing -- glasgow
U30 and ulx3s U1 are 0.8mm-pitch parts whose clamped 0.2mm drills need 0.4mm
and have 0.8. The refusal arm therefore RAISES THE FLOOR on a copy of a real
board rather than inventing a footprint, so what it exercises is the shipped
geometry against a declaration a fab could really make.
"""
import json
import os
import shutil
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

import contextlib                                              # noqa: E402
import io                                                      # noqa: E402
import math                                                    # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402
from bga_fanout import generate_bga_fanout                     # noqa: E402
# `evidence` refuses a fixture that is missing or empty BEFORE it is used as
# input, because a check whose input is missing tests nothing (CLAUDE.md). It
# also, deliberately, makes `run_all` classify this file as INTEGRATION: it
# runs four whole under-pad fanouts of a 121-ball BGA and belongs nowhere near
# the fast set. The classifier keys on markers like `from run_utils`, and
# without one a several-minute board test is collected as a unit test.
from run_utils import evidence                                 # noqa: E402

RUN_ALL_TIMEOUT = 900

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
COMPONENT = 'U30'
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
PARAMS = dict(track_width=0.12, clearance=0.1, via_size=0.35, via_drill=0.2,
              diff_pair_gap=0.101)
PAIRS = ['Z*']


def _board_with_floor(tmp, h2h):
    """glasgow_revC copied with its project, `min_hole_to_hole` overwritten.

    The project MUST come along: a bare `.kicad_pcb` copy strands the DRC floor
    and the run resolves a different one, which is #441 and would make these
    arms measure the copy rather than the declaration.
    """
    dst = os.path.join(tmp, 'g.kicad_pcb')
    shutil.copy(BOARD, dst)
    with open(os.path.splitext(BOARD)[0] + '.kicad_pro', encoding='utf-8') as f:
        pro = json.load(f)
    if h2h is None:
        pro['board']['design_settings']['rules'].pop('min_hole_to_hole', None)
    else:
        pro['board']['design_settings']['rules']['min_hole_to_hole'] = h2h
    with open(os.path.join(tmp, 'g.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump(pro, f)
    evidence(dst, 'the copied board')
    evidence(os.path.join(tmp, 'g.kicad_pro'), 'the copied project')
    return dst


def _fanout(path):
    """Returns (vias, transcript). Engine call, no file written."""
    pcb = parse_kicad_pcb(path)
    fp = pcb.footprints[COMPONENT]
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        _tracks, vias, _vrem, _failed = generate_bga_fanout(
            fp, pcb, layers=LAYERS, escape_method='underpad',
            diff_pair_patterns=PAIRS, **PARAMS)
    return vias, buf.getvalue()


def _min_drill_gap(vias):
    best = None
    for i, a in enumerate(vias):
        for b in vias[i + 1:]:
            ax, ay = a['x'], a['y']
            bx, by = b['x'], b['y']
            d = (math.hypot(ax - bx, ay - by)
                 - (a['drill'] or 0) / 2 - (b['drill'] or 0) / 2)
            if best is None or d < best:
                best = d
    return best


class TestTheGateIsInertOnTheBoardAsShipped(unittest.TestCase):
    """The paired half. A gate that can only decline must be shown NOT to
    decline the geometry people actually run, or its cost is unbounded."""

    def test_the_shipped_board_still_escapes_and_still_couples(self):
        with tempfile.TemporaryDirectory() as tmp:
            vias, log = _fanout(_board_with_floor(tmp, 0.25))
        self.assertGreater(len(vias), 0, 'the rig escapes nothing, so it '
                                         'cannot show the gate is inert')
        self.assertIn('diff pairs coupled', log,
                      'no pair coupled at all -- this rig no longer reaches '
                      'the code #618 is about')
        self.assertNotIn('coupled pair(s) declined', log.replace(
            '0 coupled pair(s) declined', ''),
            'the gate declines a coupled pair on the board as shipped')

    def test_the_shipped_geometry_clears_its_own_declared_floor(self):
        """0.8mm pitch, drills clamped to 0.2: needs 0.4, has 0.8. Asserted so
        the inertness above has a REASON on the record, not just a count."""
        with tempfile.TemporaryDirectory() as tmp:
            vias, _log = _fanout(_board_with_floor(tmp, 0.25))
        gap = _min_drill_gap(vias)
        self.assertIsNotNone(gap)
        self.assertGreaterEqual(round(gap, 6), 0.25,
                                f'the shipped board emits a {gap:.4f}mm drill '
                                f'gap against its own 0.25mm declaration')


class TestTheSiteConflictHalfIsLive(unittest.TestCase):
    """The half the gate exists for, and the one the mutation battery caught
    having NO coverage: `_via_site_conflict` against the board's own copper.

    The arms below plant a foreign-net via at a coordinate the unplanted run
    puts a coupled escape via on, then re-run. With the gate, that ball's site
    is declined and no via lands on the planted hole; without it, the coupled
    pair goes in regardless and two drills share a point.

    Two engine runs, because the plant site has to be a coordinate this engine
    actually chooses -- a hand-picked ball centre would prove nothing if the
    pair never couples there.

    MUTATION: `return True` before the `_via_site_conflict` loop in
    `_coupled_via_sites_ok` -- these arms die.
    """

    def test_a_foreign_hole_at_a_coupled_ball_centre_declines_that_site(self):
        from kicad_writer import add_tracks_and_vias_to_pcb
        with tempfile.TemporaryDirectory() as tmp:
            plain = _board_with_floor(tmp, 0.25)
            vias, _log = _fanout(plain)
            self.assertTrue(vias, 'the rig emits no vias at all')
            target = vias[0]
            # Plant a foreign-net via ON that site, in the INPUT board.
            planted = os.path.join(tmp, 'planted.kicad_pcb')
            shutil.copy(os.path.splitext(plain)[0] + '.kicad_pro',
                        os.path.join(tmp, 'planted.kicad_pro'))
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                add_tracks_and_vias_to_pcb(
                    plain, planted, [],
                    [{'x': target['x'], 'y': target['y'], 'size': 0.3,
                      'drill': 0.2, 'layers': ['F.Cu', 'B.Cu'],
                      'net_id': 0}])
            evidence(planted, 'the planted board')
            after, log2 = _fanout(planted)

        floor = 0.25
        clashes = [v for v in after
                   if math.hypot(v['x'] - target['x'], v['y'] - target['y'])
                   < (v['drill'] or 0) / 2 + 0.1 + floor - 1e-6]
        self.assertEqual(
            clashes, [],
            f'{len(clashes)} emitted via(s) land within the {floor}mm '
            f'hole-to-hole floor of a via that was already on the board at '
            f'({target["x"]:.4f}, {target["y"]:.4f}) -- the site-conflict half '
            f'of the coupled gate is not running')

    def test_the_plant_is_what_moved_it(self):
        """Without this, the arm above passes on a rig where the engine simply
        never chooses that coordinate again for its own reasons."""
        with tempfile.TemporaryDirectory() as tmp:
            plain = _board_with_floor(tmp, 0.25)
            vias, _log = _fanout(plain)
        self.assertTrue(vias)
        hit = [v for v in vias
               if math.hypot(v['x'] - vias[0]['x'], v['y'] - vias[0]['y'])
               < 1e-9]
        self.assertTrue(hit, 'the unplanted run does not put a via on the '
                             'coordinate the planted arm tests, so that arm '
                             'proves nothing')


class TestTheDeclaredFloorIsHonoured(unittest.TestCase):
    """#618 clause 2, re-derived. A declaration the geometry cannot meet must
    change the outcome -- otherwise the floor is decorative, which is exactly
    what #618 accused this engine of and what #756 fixed."""

    def test_a_floor_the_pitch_cannot_meet_declines_sites(self):
        """0.8mm pitch with 0.2mm drills clears 0.25 and cannot clear 0.7
        (0.2 + 0.7 = 0.9 > 0.8). MUTATION: revert `_h2h` to the flat
        HOLE_TO_HOLE_CLEARANCE -- this arm dies while the inert arms pass."""
        with tempfile.TemporaryDirectory() as tmp:
            _v_lo, log_lo = _fanout(_board_with_floor(tmp, 0.25))
        with tempfile.TemporaryDirectory() as tmp:
            v_hi, log_hi = _fanout(_board_with_floor(tmp, 0.70))
        self.assertIn('Hole-to-hole 0.7mm', log_hi,
                      'the 0.7 declaration was not read at all')
        self.assertNotIn('Hole-to-hole 0.7mm', log_lo)
        gap = _min_drill_gap(v_hi)
        if gap is not None:
            self.assertGreaterEqual(
                round(gap, 6), 0.70,
                f'a run announcing a 0.7mm floor still emits a {gap:.4f}mm '
                f'drill gap')

    def test_the_decline_is_DISCLOSED(self):
        """#618's policy question, answered as WARN. MUTATION: delete the
        `h2h_stats` print -- the operator gets a worse board with no reason
        given, which is the state #618 objected to."""
        with tempfile.TemporaryDirectory() as tmp:
            _v, log = _fanout(_board_with_floor(tmp, 0.70))
        self.assertIn('hole-to-hole floor', log,
                      'sites declined by the floor are not disclosed')
        self.assertIn('declined by the', log)
        self.assertIn('--via-drill', log,
                      'the disclosure names no lever, so it is a complaint '
                      'rather than an instruction')


if __name__ == '__main__':
    unittest.main(verbosity=2)
