#!/usr/bin/env python3
"""Issue #746: `resolved` was graded against a board that no longer existed.

`repair_fanout_clearance` built its two summary lists at different times.
`resolved` was computed right after the cap-move descent; `unresolved` was
computed there too and then RECOMPUTED after the via-nudge had moved vias and
drawn connector copper.  `resolved` never was.  So the operator's line mixed a
pre-nudge R with a post-nudge U, and it fails in BOTH directions:

  A. A cap the NUDGE frees reaches neither list.  It was grazing when the
     credit was computed and clean when the debit was.

         before   resolved []      unresolved []
                  "Moved 0 cap(s); resolved 0/1 initial violations; 0 unresolved."
         after    resolved ['C1']  unresolved []  via_resolved ['C1']
                  "... resolved 1/1 initial violations (1 freed by via-nudge); 0 unresolved."

  B. A cap the SWEEP cleaned that a connector this same pass drew then grazes
     appears in BOTH lists.

         before   resolved ['C2']  unresolved ['C1','C2']   <- C2 in both
                  "Moved 2 cap(s); resolved 1/2 initial violations; 2 unresolved."
         after    resolved []      unresolved ['C1','C2']   regrazed ['C2']
                  "Moved 2 cap(s); resolved 0/2 initial violations; 2 unresolved."
                  "  Re-grazed by this pass's own connector copper: C2"

     1 + 2 = 3 credits on a 2-cap board.  Reachable only after #736 made the
     post-nudge re-grade see the connectors, which is why the file declared it
     a KNOWN GAP there instead of closing it.

A REAL TRACKED BOARD DEMONSTRATES HALF A, which is what makes this more than a
synthetic curiosity.  `kicad_files/orangecrab_ext_pll.kicad_pcb` at clearance
0.1 in the FORCING configuration below -- ALL FOUR knobs, not just the
fallback: at `via_clear_fallback=False` alone the descent clears all 14 and
the nudger is never called -- went

    before   "Moved 18 cap(s); resolved 1/14 initial violations; 10 unresolved."
    after    "Moved 18 cap(s); resolved 4/14 initial violations
              (3 freed by via-nudge); 10 unresolved."

-- C19, C44 and C45 were freed by the nudge and credited nowhere.  Half B has
no tracked board -- measured, see below -- and is synthetic here.

WHY ARM B'S RIG IS SHAPED THE WAY IT IS.  Half B needs the connector to graze
a cap the relocated via does not, and `valid_via_pos` gates a landing against
every cap rect with NO layer gate.  The geometry is a capsule against a disc:
the via is tested as ONE disc of radius `R_v = vr + via_req` at the LANDING
(`_point_to_rect_dist`), while the connector is charged as a capsule of radius
`R_c = hw + pair` around the whole path from the old position to the new
(`_seg_to_rect_dist`).  A point beside the path's INTERIOR is in the capsule
and in neither endpoint's disc, so the reachable condition is on the nudge
LENGTH, not on the two radii alone:

    reachable  <=>  R_c >= R_v,  or  L >= 2 * sqrt(R_v^2 - R_c^2)

with `L <= max_shift` (0.6).  A FIRST VERSION OF THIS FILE GOT THAT WRONG: it
said `hw <= vr` made half B unreachable, which is the answer for one point
(arm A's 0.8mm via and 0.3mm connector need L >= 0.866 and cannot get it) read
as a general law.  A review refuted it with a working rig at `hw == vr`, where
the bound is 0 and any nudge at all suffices.  The floors DO cancel wherever
they resolve equal, and that half survived the same review: `via_req` spans
`_all_cu`, which contains the connector's one layer, so `via_req >= pair`
always.

Arm B sits far inside the reachable region rather than on its boundary --
`R_c = 1.60` against `R_v = 0.30`, so no length is required at all -- because
it is a demonstration, not a boundary test.  It also needs #738's layer
window: the cap pads are THROUGH-HOLE, so their copper spans the connector's
inner layer while `connector_clear` tests only the footprint's own side.  That
is the single-variable control
`test_the_regraze_is_the_off_layer_window_not_a_generic_effect`, which flips
the stub to F.Cu and watches the pass decline to draw the connector at all.

THAT NO TRACKED BOARD SHOWS HALF B IS MEASURED, NOT DERIVED -- `regrazed` is
empty on all 88 corpus rows below.  The corrected condition above is wide
enough that it would be dishonest to claim otherwise.

The corpus is the inertness argument, measured rather than asserted: over the
22 boards `git ls-files` returns, at clearance 0.25 and 0.10, at the shipped
defaults AND at the forcing configuration, 88 rows differ in exactly ONE place
-- the orangecrab row above.  At the shipped defaults no tracked board NUDGES
A VIA (one, rp2350_fpga_eensy_prePlane, does reach the nudger and is refused
every landing), so all 44 of those rows print an identical summary and
`regrazed` is empty on all 88.

MUTATION BATTERY, 12 rows against the engine, 12 killed, 0 survivors.  The
count per row is what makes the `# MUTATION:` notes checkable rather than
decorative, so it is recorded here:

    resolved-refresh-reverted            11 arms   the defect itself
    deltas-swapped                       11        via_resolved <-> regrazed
    deltas-hoisted-out-of-the-guard       8        credit on a run with no nudge
    swept-bound-after-the-regrade         7        both deltas empty everywhere
    credit-clause-unconditional           4        "(0 freed by via-nudge)"
    via_resolved-seeded-with-resolved     4        the descent's credit, restamped
    registrar-after-the-regrade           4        #736's ordering
    violators0-gate-dropped               3        "resolved 55/14"
    regrazed-print-deleted                2
    grade-returns-sorted                  1        only the real board's order
    via_resolved-is-all-of-resolved       1        only the real board
    regrazed-keyed-on-swept               1        only arm C

Three rows are killed by exactly one arm, and each names why that arm exists
rather than being redundant with the rest.

TWO OF MY OWN `# MUTATION:` NOTES WERE WRONG, in two DIFFERENT ways, and are
corrected in place rather than quietly deleted:

  * one named an arm that CANNOT reach the mutation.  `via_resolved =
    list(resolved)` lives inside the `if via_moves:` block, so the sweep-only
    class -- which exists precisely because that guard is False there -- can
    never execute it.  I had written that this class was what caught it; the
    battery says one arm does, and a different one.
  * the other claimed SOLE credit for a mutation ten arms catch.  The
    partition arm does reach the delta swap and does kill it; it is simply not
    "silent" anywhere, and the note said it was.

A note claiming a gate that does not gate is worth less than no note, and so
is one claiming coverage it shares with nine others.

ARM C EXISTS BECAUSE A REVIEW FOUND THE FIRST `regrazed` KEYED ON THE WRONG
THING.  Spelled `[r for r in swept if r not in resolved]` it can only ever
name a cap that was ALSO a seed violator, so a cap that arrived perfectly
clean and was broken by this pass's own connector was the single kind of
re-graze with no cause named -- the exact case the engine docstring says
exists.  Re-keyed on the transition (`unresolved` minus the pre-nudge
`unresolved`), and arm C is the only arm in this file that tells the two
spellings apart, because arm B's C2 happens to be a seed violator too.

Runtime ~30s warm, in-process apart from one `git ls-files`.

    python3 tests/test_746_fanout_clearance_resolved_credit.py
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 900

import contextlib
import inspect
import io
import os
import sys
import tempfile
import unittest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

import run_utils
from kicad_parser import (BoardInfo, Footprint, Zone, detect_package_type,
                          parse_kicad_pcb)
from placement import fanout_clearance as FC
from placement.fanout_clearance import repair_fanout_clearance
from synth import make_net, make_pad, make_pcb, make_seg, make_via

# --- the rig, and every number it is worth --------------------------------
CLEAR = 0.1                 # place_fanout_clearance.py's own --help example
CU = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
PAD_HALF = 0.3              # 0.6mm square pads at local +-0.5

# Arm A: one boxed cap, one foreign via the nudger can move out of its way.
A_BOUNDS = (0.0, 0.0, 60.0, 20.0)
A_Y = 10.0
A_BGA = (7.6, A_Y)
A_CAP = (10.0, A_Y)
A_VIA_X = 11.175            # 0.375 off pad 2's edge against a 0.400 keep-out
A_VIA_SIZE, A_VIA_DRILL = 0.8, 0.3
A_LANDING = (11.325, A_Y)   # MEASURED: where the nudger parks it

# Arm B: a wide stub into a small via, so the connector's keep-out EXCEEDS the
# relocated via's -- see the window derivation in the module docstring.
B_BOUNDS = (0.0, 0.0, 60.0, 24.0)
B_Y = 12.0
B_BGA = (6.0, B_Y)
B_C1 = (9.8, B_Y)           # buried in the stub: the sweep can never free it
B_C2 = (12.0, B_Y)          # a seed violator the sweep CAN free
B_VIA = (10.0, B_Y)
B_VIA_SIZE, B_VIA_DRILL = 0.4, 0.2
B_STUB_W = 3.0              # hw 1.50 vs vr 0.20 -> a +1.30mm window
B_STUB_X0 = 4.0
B_LANDING = (10.230, 12.554)    # MEASURED
B_C3 = (11.4, 13.9)         # NEVER a seed violator: 1.709mm off the stub
                            # against its 1.600 keep-out, and 1.110 off the
                            # connector's far end against the same -- so the
                            # pass breaks a cap that arrived perfectly clean
B_NEAR = 8.0                # the caps sit further from the BGA than the 1.0
                            # default admits; without this `st.caps` is empty
                            # and the pass early-returns having graded nothing
B_DISP = 0.4

# The forcing configuration: no cap may move and the fallback is off, so a cap
# can only be freed by the nudge.  #313's boxed cap as a budget rather than as
# a wall of obstacle geometry.
FORCING = dict(max_displacement=0.0, max_displacement_cap=0.0, max_passes=1,
               via_clear_fallback=False)

REAL_BOARD = os.path.join(_ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
REAL_RESOLVED = ['C19', 'C44', 'C45', 'C67']
REAL_VIA_RESOLVED = ['C19', 'C44', 'C45']       # C67 is the SWEEP's one credit
REAL_LINE = ('Moved 18 cap(s); resolved 4/14 initial violations '
             '(3 freed by via-nudge); 10 unresolved.')
REAL_LINE_BEFORE = 'Moved 18 cap(s); resolved 1/14 initial violations; 10 unresolved.'


def _bga(bx, by):
    """detect_package_type keys on the footprint NAME, so a 3x3 grid whose
    name says BGA is enough to give _Repair a ball field to sit caps near."""
    pads = []
    for i, dx in enumerate((-0.8, 0.0, 0.8)):
        for j, dy in enumerate((-0.8, 0.0, 0.8)):
            pads.append(make_pad(net_id=100 + 3 * i + j, x=bx + dx, y=by + dy,
                                 ref='U1', num='%d%d' % (i, j), size_x=0.4,
                                 size_y=0.4, shape='circle', layers=['F.Cu'],
                                 local_x=dx, local_y=dy))
    return Footprint(reference='U1', footprint_name='lib:BGA-9_3x3_0.8mm',
                     x=bx, y=by, rotation=0.0, layer='F.Cu', pads=pads)


def _cap(ref, cx, cy, na, nb, through=False):
    """A C-prefixed 2-copper-pad part -- what _Repair's cap detection admits.

    `through` is the ONE variable that opens #738's layer window: the pads
    become `*.Cu` plated through-hole, so their copper spans every layer while
    the FOOTPRINT still sits on F.Cu and `connector_clear` tests only that
    side.  Arm B needs it; every other arm here leaves it False.
    """
    kw = dict(size_x=2 * PAD_HALF, size_y=2 * PAD_HALF, shape='rect')
    if through:
        kw.update(layers=['*.Cu', '*.Mask'], drill=0.3, pad_type='thru_hole')
    else:
        kw.update(layers=['F.Cu'])
    pads = [make_pad(net_id=na, x=cx - 0.5, y=cy, ref=ref, num='1',
                     local_x=-0.5, local_y=0.0, **kw),
            make_pad(net_id=nb, x=cx + 0.5, y=cy, ref=ref, num='2',
                     local_x=0.5, local_y=0.0, **kw)]
    return Footprint(reference=ref, footprint_name='lib:C_0402_1005Metric',
                     x=cx, y=cy, rotation=0.0, layer='F.Cu', pads=pads)


def _drive(bi, vias, segs, fps, zones, **kw):
    """Build the PCBData, stage a bare board FILE so the path-reading
    collaborators degrade cleanly (courtyards -> {}, locked refs -> set(),
    PadClearanceModel -> INERT), run the WHOLE pass, capture stdout.

    `on_move` is the supported hook the animator uses and the only way to see
    the internal `_Repair` -- in particular `st.caps`, whose iteration order
    both returned lists must preserve.
    """
    pads_by_net = {}
    for fp in fps.values():
        for p in fp.pads:
            pads_by_net.setdefault(p.net_id, []).append(p)
    nets = {0: make_net(0, '')}
    for k in list(pads_by_net) + [v.net_id for v in vias]:
        nets.setdefault(k, make_net(k, 'N%d' % k))
    pcb = make_pcb(board_info=bi, nets=nets, vias=list(vias),
                   segments=list(segs), footprints=fps,
                   pads_by_net=pads_by_net, zones=list(zones))
    seen = []
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'b.kicad_pcb')
        with io.open(path, 'w', encoding='utf-8') as f:
            f.write('(kicad_pcb (version 20240108))\n')
        # A check whose INPUT is missing tests nothing (CLAUDE.md/run_utils).
        run_utils.evidence(path, 'the staged #746 rig board')
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            res = repair_fanout_clearance(pcb, path, clearance=CLEAR,
                                          cap_prefix='C',
                                          on_move=lambda st: seen.append(st),
                                          **kw)
    return res, (seen[0] if seen else None), buf.getvalue()


def arm_a(**kw):
    """One cap, one via, nothing may move: only the nudge can free it."""
    bi = BoardInfo(layers={}, copper_layers=list(CU), board_bounds=A_BOUNDS)
    v = make_via(A_VIA_X, A_Y, net_id=3, size=A_VIA_SIZE, drill=A_VIA_DRILL)
    # A same-net zone under the via is what gives the relocated via a
    # connector to draw -- without a tie of some kind conn_layers is empty and
    # the via moves with no copper behind it.
    zone = Zone(net_id=3, net_name='N3', layer='In1.Cu',
                polygon=[(A_VIA_X - 1.0, A_Y - 1.0), (A_VIA_X + 1.0, A_Y - 1.0),
                         (A_VIA_X + 1.0, A_Y + 1.0), (A_VIA_X - 1.0, A_Y + 1.0)])
    fps = {'U1': _bga(*A_BGA), 'C1': _cap('C1', A_CAP[0], A_CAP[1], 11, 12)}
    opts = dict(FORCING, allow_rotations=False)
    opts.update(kw)
    return _drive(bi, [v], [], fps, [zone], **opts)


def arm_b(*, layer='In1.Cu', through=True, stub_w=B_STUB_W, **kw):
    """Two caps on a wide same-net stub.  C1 is buried and can never clear, so
    it is what calls the nudger; C2 is a seed violator the sweep frees, and
    the connector drawn for C1's via then grazes it."""
    bi = BoardInfo(layers={}, copper_layers=list(CU), board_bounds=B_BOUNDS)
    v = make_via(B_VIA[0], B_VIA[1], net_id=3, size=B_VIA_SIZE,
                 drill=B_VIA_DRILL)
    stub = make_seg(B_STUB_X0, B_Y, B_VIA[0], B_Y, width=stub_w, layer=layer,
                    net_id=3)
    fps = {'U1': _bga(*B_BGA),
           'C1': _cap('C1', B_C1[0], B_C1[1], 11, 12, through=through),
           'C2': _cap('C2', B_C2[0], B_C2[1], 13, 14, through=through)}
    opts = dict(max_displacement=B_DISP, max_displacement_cap=B_DISP,
                max_passes=30, allow_rotations=False, near_margin=B_NEAR,
                via_clear_fallback=False)
    opts.update(kw)
    return _drive(bi, [v], [stub], fps, [], **opts)


def arm_c():
    """Arm B's board with C2 replaced by a cap that is CLEAN at the seed.

    C1 is still buried and still calls the nudger; C3 sits outside every
    keep-out on the board it was handed and is broken purely by the connector
    this pass then draws.  It is never a violator, so it never enters
    `violators0`, never enters `resolved`, and is invisible to any disclosure
    keyed on the seed -- which is what an adversarial review of the first
    version of this fix found.
    """
    bi = BoardInfo(layers={}, copper_layers=list(CU), board_bounds=B_BOUNDS)
    v = make_via(B_VIA[0], B_VIA[1], net_id=3, size=B_VIA_SIZE,
                 drill=B_VIA_DRILL)
    stub = make_seg(B_STUB_X0, B_Y, B_VIA[0], B_Y, width=B_STUB_W,
                    layer='In1.Cu', net_id=3)
    fps = {'U1': _bga(*B_BGA),
           'C1': _cap('C1', B_C1[0], B_C1[1], 11, 12, through=True),
           'C3': _cap('C3', B_C3[0], B_C3[1], 15, 16, through=True)}
    return _drive(bi, [v], [stub], fps, [], max_displacement=B_DISP,
                  max_displacement_cap=B_DISP, max_passes=30,
                  allow_rotations=False, near_margin=B_NEAR,
                  via_clear_fallback=False)


def sweep_only(disp=0.3):
    """Arm A's board with a displacement budget, so the SWEEP clears the cap
    and the nudger is never called at all (`if unresolved:` is False)."""
    return arm_a(max_displacement=disp, max_displacement_cap=disp,
                 max_passes=30)


def _summary(out):
    return next((l for l in out.splitlines() if l.startswith('Moved ')), None)


def _line(out, prefix):
    return next((l.rstrip() for l in out.splitlines()
                 if l.startswith(prefix)), None)


_REAL_CACHE = {}


def _real():
    """The one tracked board that reaches the nudger, memoised: it costs a few
    seconds and four arms want it."""
    if 'r' not in _REAL_CACHE:
        run_utils.evidence(REAL_BOARD, 'the orangecrab_ext_pll fixture')
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            res = repair_fanout_clearance(parse_kicad_pcb(REAL_BOARD),
                                          REAL_BOARD, clearance=CLEAR,
                                          **FORCING)
        _REAL_CACHE['r'] = (res, buf.getvalue())
    return _REAL_CACHE['r']


class TestTheNudgeIsCreditedInsteadOfLost(unittest.TestCase):
    """Half A: the cap the last resort exists to free."""

    @classmethod
    def setUpClass(cls):
        cls.res, cls.st, cls.out = arm_a()

    def test_the_rig_reaches_the_nudger_and_the_nudger_moves_the_via(self):
        # ON THE BRANCH: without this every assertion below is about a pass
        # that graded a board nobody touched, and half A cannot arise.
        self.assertIn('Caps initially grazing foreign copper (via/track/pad): '
                      '1 (C1)', self.out,
                      'C1 is not a SEED violator, so there is nothing for the '
                      'nudge to be credited WITH')
        self.assertIn('via-nudge: moved N3 via (%.3f,%.3f) -> (%.3f,%.3f) to '
                      'free C1' % (A_VIA_X, A_Y, A_LANDING[0], A_LANDING[1]),
                      self.out, 'the nudger did not move the via it names')
        self.assertEqual(len(self.res['via_moves']), 1)
        self.assertEqual(len(self.res['new_segments']), 1)
        self.assertEqual(self.res['placements'], [],
                         'a cap MOVED, so this arm no longer isolates the '
                         'nudge from the sweep')
    # MUTATION: none -- this arm licenses the claim rather than testing the fix.

    def test_the_freed_cap_is_credited_to_resolved(self):
        # The PRINT first: assert it before the counts and no mutation can
        # make the silence be the reported failure.
        self.assertEqual(
            _summary(self.out),
            'Moved 0 cap(s); resolved 1/1 initial violations '
            '(1 freed by via-nudge); 0 unresolved.')
        self.assertEqual(self.res['resolved'], ['C1'])
        self.assertEqual(self.res['unresolved'], [])
    # MUTATION: drop the `resolved, unresolved = _grade()` refresh from the
    # `if via_moves:` block (leave only the `unresolved` recompute) -- resolved
    # goes back to [] and the line reads "resolved 0/1 initial violations;
    # 0 unresolved." about a run that fixed the one violation it found.

    def test_the_credit_names_the_nudge_as_the_mechanism(self):
        self.assertEqual(self.res['via_resolved'], ['C1'])
        self.assertEqual(self.res['regrazed'], [])
    # MUTATION: `via_resolved = list(resolved)`. This arm does NOT kill it --
    # `swept` is empty on a rig where no cap moves, so the two spellings agree
    # here. Measured: of the 26 arms in this file exactly ONE goes red, the
    # real-board one, where the descent's own credit (C67) is in `resolved`
    # and must not be attributed to the nudge. The first draft of this comment
    # claimed the sweep-only class caught it; the battery said otherwise, and
    # that mutation is unreachable from there (it lives inside `if via_moves:`).

    def test_a_second_rig_frees_a_cap_the_same_way(self):
        """Arm B's board with SMD pads: a different stub, a different landing,
        the same credit.  One rig proving half A would leave the finding
        resting on one set of coordinates."""
        res, _st, out = arm_b(through=False)
        self.assertIn('via-nudge: moved N3 via (10.000,12.000) -> '
                      '(10.000,12.550) to free C1', out)
        self.assertEqual(res['via_resolved'], ['C1'])
        self.assertEqual(res['resolved'], ['C1'])
        self.assertEqual(res['unresolved'], [])
        self.assertEqual(
            _summary(out),
            'Moved 1 cap(s); resolved 1/1 initial violations '
            '(1 freed by via-nudge); 0 unresolved.')
    # MUTATION: same as test_the_freed_cap_is_credited_to_resolved.


class TestACapCleanedThenReGrazedLeavesResolved(unittest.TestCase):
    """Half B: the pass's own connector copper breaking a cap it had fixed."""

    @classmethod
    def setUpClass(cls):
        cls.res, cls.st, cls.out = arm_b()

    def test_the_rig_reaches_the_nudger_and_the_sweep_frees_C2(self):
        # ON THE BRANCH, three ways: both caps must be SEED violators, the
        # sweep must actually move C2, and a connector must be drawn.  Miss
        # any one and "C2 left resolved" is true for the wrong reason.
        self.assertIn('Caps initially grazing foreign copper (via/track/pad): '
                      '2 (C1, C2)', self.out)
        moved = {p['reference']: (round(p['new_x'], 3), round(p['new_y'], 3))
                 for p in self.res['placements']}
        self.assertEqual(moved, {'C1': (10.2, 12.0), 'C2': (12.4, 12.0)},
                         'the sweep no longer frees C2, so there is no credit '
                         'for the connector to take away')
        self.assertEqual(len(self.res['via_moves']), 1)
        self.assertEqual([(s['layer'], s['width'])
                          for s in self.res['new_segments']],
                         [('In1.Cu', B_STUB_W)],
                         'the connector this arm is about was not drawn')
    # MUTATION: none -- an on-the-branch guard.

    def test_C2_is_in_unresolved_and_NOT_also_in_resolved(self):
        self.assertEqual(
            _summary(self.out),
            'Moved 2 cap(s); resolved 0/2 initial violations; 2 unresolved.')
        self.assertEqual(self.res['unresolved'], ['C1', 'C2'])
        self.assertEqual(self.res['resolved'], [])
        self.assertEqual(sorted(set(self.res['resolved'])
                                & set(self.res['unresolved'])), [])
    # MUTATION: drop the `resolved` half of the refresh -> resolved == ['C2']
    # while unresolved == ['C1','C2'], i.e. 1 + 2 = 3 credits on a 2-cap board,
    # and the line reads "resolved 1/2 initial violations; 2 unresolved."

    def test_the_cause_is_named_rather_than_left_anonymous(self):
        self.assertEqual(
            _line(self.out, '  Re-grazed'),
            "  Re-grazed by this pass's own connector copper: C2")
        self.assertEqual(self.res['regrazed'], ['C2'])
        self.assertEqual(self.res['via_resolved'], [])
    # MUTATION: delete the `if regrazed:` print -> the cap is still correctly
    # reported unresolved, but nothing says the pass did it, which is the
    # whole difference between a board that arrived broken and one we broke.

    def test_the_regraze_is_the_off_layer_window_not_a_generic_effect(self):
        """Single-variable control: put the stub -- and so the connector -- on
        the caps' OWN side.  There `connector_clear` and the grader agree
        exactly (#736), so the pass declines to draw the connector at all and
        nothing is re-grazed.  This is what stops half B from reading as 'the
        nudger routinely wrecks caps'."""
        res, _st, out = arm_b(layer='F.Cu')
        self.assertIn("via-nudge: no clear spot for C1's offending via", out)
        self.assertEqual(res['via_moves'], [])
        self.assertEqual(res['new_segments'], [])
        self.assertEqual(res['regrazed'], [])
        # ... and the pre-nudge credit stands, unrefreshed and correct.
        self.assertEqual(res['resolved'], ['C2'])
        self.assertEqual(res['unresolved'], ['C1'])
    # MUTATION: none -- a control.

    def test_a_cap_that_was_NEVER_broken_still_gets_its_cause_named(self):
        """The case a review of the first version of this fix found missing.

        C3 arrives clean -- not in `violators0`, so it can never be in
        `resolved` -- and this pass's own connector breaks it.  Keyed on the
        seed (`[r for r in swept if r not in resolved]`) that cap is the ONE
        kind of re-graze with no cause named, which is the opposite of what
        the disclosure is for.  Keyed on the TRANSITION it is named."""
        res, _st, out = arm_c()
        # ON THE BRANCH, three ways: C3 must not be a seed violator, must not
        # have moved, and the connector must exist. Miss any and the arm is
        # about something else.
        self.assertIn('Caps initially grazing foreign copper (via/track/pad): '
                      '1 (C1)', out,
                      'C3 is a SEED violator here, so this rig no longer '
                      'isolates the never-broken case')
        self.assertEqual([p['reference'] for p in res['placements']], ['C1'],
                         'C3 moved; it must be broken where it stands')
        self.assertEqual(len(res['new_segments']), 1)

        self.assertEqual(_line(out, '  Re-grazed'),
                         "  Re-grazed by this pass's own connector copper: C3")
        self.assertEqual(res['regrazed'], ['C3'])
        self.assertEqual(res['unresolved'], ['C1', 'C3'])
        self.assertEqual(res['resolved'], [])
    # MUTATION: `regrazed = [r for r in swept if r not in resolved]` -- the
    # first spelling. C3 was never in `violators0`, so `swept` cannot contain
    # it and regrazed goes to []: the pass breaks a cap that arrived clean and
    # reports it as an anonymous `unresolved`. Arm B alone does NOT catch this,
    # because its C2 happens to be a seed violator as well.


class TestTheSweepOnlyRunIsUnchanged(unittest.TestCase):
    """A run that never reaches the via-nudge must print what it always did."""

    @classmethod
    def setUpClass(cls):
        cls.res, cls.st, cls.out = sweep_only()

    def test_the_nudger_is_never_called(self):
        # ON THE BRANCH: `if unresolved:` must be False, not merely "the
        # nudger found nothing" -- a rig that reaches it and declines would
        # exercise a different path than the one this class claims.
        self.assertNotIn('via-nudge', self.out,
                         'the nudger ran; this class is about the path where '
                         'it is not even called')
        self.assertEqual(self.res['via_moves'], [])
        self.assertEqual(self.res['new_segments'], [])
    # MUTATION: none -- an on-the-branch guard.

    def test_the_sweeps_own_credit_is_not_attributed_to_the_nudge(self):
        self.assertEqual(self.res['resolved'], ['C1'])
        self.assertEqual(self.res['via_resolved'], [])
        self.assertEqual(self.res['regrazed'], [])
    # MUTATION: seed the declaration -- `via_resolved: List[str] = list(resolved)`
    # instead of `[]` -- so a cap the DESCENT walked clear is credited to the
    # nudge on every board that never nudges. Also killed by hoisting the two
    # delta comprehensions out of the `if via_moves:` guard. Note the mutation
    # INSIDE that guard (`via_resolved = list(resolved)`) cannot be caught
    # here: the guard is False on this path, so the block never runs.

    def test_both_new_clauses_are_suppressed_so_the_line_is_the_old_line(self):
        self.assertEqual(
            _summary(self.out),
            'Moved 1 cap(s); resolved 1/1 initial violations; 0 unresolved.')
        self.assertIsNone(_line(self.out, '  Re-grazed'))
        self.assertNotIn('freed by via-nudge', self.out)
    # MUTATION: emit `_credit` unconditionally -> "(0 freed by via-nudge)" on
    # every board that never nudges, i.e. on all 22 tracked ones at the
    # shipped defaults.


class TestTheTwoListsAreDisjointAndOrdered(unittest.TestCase):
    """The shape contract both front ends read, on every arm at once."""

    ARMS = ('arm_a', 'arm_b', 'arm_c', 'arm_b_smd', 'arm_b_onlayer',
            'sweep_only')

    @classmethod
    def setUpClass(cls):
        cls.runs = {
            'arm_a': arm_a(),
            'arm_b': arm_b(),
            'arm_c': arm_c(),
            'arm_b_smd': arm_b(through=False),
            'arm_b_onlayer': arm_b(layer='F.Cu'),
            'sweep_only': sweep_only(),
        }

    def test_no_ref_is_in_both_lists(self):
        for name in self.ARMS:
            res, _st, _out = self.runs[name]
            with self.subTest(arm=name):
                self.assertEqual(
                    sorted(set(res['resolved']) & set(res['unresolved'])), [],
                    'a cap is credited and debited by the same run')
    # MUTATION: drop the `resolved` half of the refresh -> arm_b fails.

    def test_resolved_is_a_subset_of_the_seed_violators(self):
        """`violators0` never leaves the engine, so it is re-derived from the
        printed disclosure -- which is the operator's only view of it too."""
        for name in self.ARMS:
            res, _st, out = self.runs[name]
            head = _line(out, 'Caps initially grazing foreign copper')
            self.assertIsNotNone(head, 'the seed disclosure vanished')
            seeds = set()
            if '(' in head:
                seeds = {s.strip() for s in
                         head.split('(', 2)[-1].rstrip(')').split(',')}
            with self.subTest(arm=name):
                self.assertLessEqual(set(res['resolved']), seeds,
                                     'a cap was credited as resolved that was '
                                     'never a violator')
    # MUTATION: drop the `elif ref in violators0` gate in _grade(), so every
    # clean cap is credited whether or not it was ever broken. Measured: three
    # arms go red, this one among them, and the real board's line becomes
    # "resolved 55/14 initial violations" -- a numerator above its own
    # denominator, from a pass claiming caps it never touched.

    def test_both_lists_follow_st_caps_order(self):
        """test_736:506 asserts `res['unresolved'] == ['C1']` by LIST equality,
        so order is a contract, not an accident."""
        for name in self.ARMS:
            res, st, _out = self.runs[name]
            self.assertIsNotNone(st, 'the on_move hook never fired')
            order = list(st.caps)
            with self.subTest(arm=name):
                self.assertEqual(
                    res['resolved'],
                    [r for r in order if r in set(res['resolved'])])
                self.assertEqual(
                    res['unresolved'],
                    [r for r in order if r in set(res['unresolved'])])
    # MUTATION: `return sorted(res), sorted(unres)` in _grade() -- survives on
    # these rigs (C1 < C2 already) and is killed by the real-board arm, where
    # `unresolved` comes out R19, C1, C25, C7, ... and sorted() does not.

    def test_the_two_disclosure_keys_partition_against_the_pre_nudge_credit(self):
        """via_resolved and regrazed are DISJOINT and each is a subset of the
        list it is a delta into."""
        for name in self.ARMS:
            res, _st, _out = self.runs[name]
            with self.subTest(arm=name):
                self.assertEqual(
                    sorted(set(res['via_resolved']) & set(res['regrazed'])), [])
                self.assertLessEqual(set(res['via_resolved']),
                                     set(res['resolved']))
                self.assertLessEqual(set(res['regrazed']),
                                     set(res['unresolved']))
    # MUTATION: swap the two comprehensions' direction. Measured: this arm
    # goes red on three of its five subTests -- but so do nine other arms, so
    # it is corroboration rather than the only guard. The shape it uniquely
    # holds is the SUBSET half: a delta key that names a ref outside the list
    # it is a delta into.


class TestTheEarlyReturnsOmitTheNudgeKeys(unittest.TestCase):
    """The documented contract: the early returns carry neither key, exactly
    as they carry no via_moves / new_segments, and every caller uses .get."""

    def _early(self, **kw):
        bi = BoardInfo(layers={}, copper_layers=list(CU), board_bounds=A_BOUNDS)
        fps = {'U1': _bga(*A_BGA)}
        fps.update(kw.pop('extra_fps', {}))
        return _drive(bi, kw.pop('vias', []), [], fps, [], **dict(FORCING, **kw))

    def test_a_board_with_no_vias_returns_the_six_key_shape(self):
        res, _st, out = self._early()
        self.assertIn('No vias on the board', out)
        for k in ('via_moves', 'new_segments', 'via_resolved', 'regrazed'):
            self.assertNotIn(k, res, 'the early return grew a %r key; if that '
                                     'is deliberate, say so here' % (k,))
        self.assertEqual(res['resolved'], [])
        self.assertEqual(res['unresolved'], [])

    def test_a_board_with_no_movable_caps_returns_the_same_shape(self):
        v = make_via(A_VIA_X, A_Y, net_id=3, size=A_VIA_SIZE, drill=A_VIA_DRILL)
        res, _st, out = self._early(vias=[v])
        self.assertIn('No movable caps near a BGA', out)
        for k in ('via_moves', 'new_segments', 'via_resolved', 'regrazed'):
            self.assertNotIn(k, res)
    # MUTATION: add the two keys to either early return -- harmless, but it
    # would make the docstring's "exactly as they carry no via_moves" false.

    def test_the_gui_read_survives_the_early_shape(self):
        """fanout_gui.py reads all four with .get; this is that read, spelled
        the way the plugin spells it, against the dict that lacks them."""
        res, _st, _out = self._early()
        self.assertEqual(len(res.get('via_moves', [])), 0)
        self.assertEqual(list(res.get('via_resolved') or []), [])
        self.assertEqual(list(res.get('regrazed') or []), [])
    # MUTATION: none -- a control on the plugin's own spelling.


class TestOnARealTrackedBoard(unittest.TestCase):
    """Half A is not synthetic.  This is the arm that says so."""

    @classmethod
    def setUpClass(cls):
        if not os.path.isfile(REAL_BOARD):
            raise unittest.SkipTest('SKIP: %s is absent' % REAL_BOARD)
        cls.res, cls.out = _real()

    def test_the_board_reaches_the_nudger_in_this_configuration(self):
        # ON THE BRANCH: `via_clear_fallback=False` is not decoration -- with
        # the fallback ON this board reports 0 unresolved, the nudger is never
        # called, and every assertion below would be about nothing.
        self.assertEqual(len(self.res['via_moves']), 9)
        self.assertEqual(len(self.res['new_segments']), 17)
        self.assertEqual(len(self.res['placements']), 18)

    def test_three_real_caps_were_freed_by_the_nudge_and_credited_nowhere(self):
        self.assertEqual(_summary(self.out), REAL_LINE)
        self.assertEqual(self.res['resolved'], REAL_RESOLVED)
        self.assertEqual(self.res['via_resolved'], REAL_VIA_RESOLVED)
        self.assertEqual(len(self.res['unresolved']), 10)
    # MUTATION: drop the `resolved` half of the refresh -> the line reverts to
    # REAL_LINE_BEFORE ("resolved 1/14"), losing C19, C44 and C45.

    def test_the_pass_did_not_break_a_cap_it_had_fixed_on_this_board(self):
        self.assertEqual(self.res['regrazed'], [],
                         'a tracked board now re-grazes a cap the sweep '
                         'cleaned -- grade it with check_drc before believing '
                         'the summary, and re-record the window derivation in '
                         'this file, which says no tracked board can')
        self.assertIsNone(_line(self.out, '  Re-grazed'))
    # MUTATION: none -- a self-expiring bound. It is also what would catch a
    # future change that made the nudger draw connectors a real board's caps
    # sit on, which is the failure the window derivation says cannot happen.

    def test_the_order_here_is_NOT_sorted(self):
        """What kills `sorted()` in _grade(): this board's `unresolved`
        comes out R19-first, so a sorted list is observably different.  (The
        head of `st.caps` itself is R17, R13, C22, ... -- also unsorted, but it
        is the RETURNED list that carries the contract, and `resolved` here
        happens to be in sorted order, so `unresolved` is the one that
        discriminates.)"""
        self.assertNotEqual(self.res['unresolved'],
                            sorted(self.res['unresolved']))
        self.assertEqual(self.res['unresolved'][0], 'R19')
    # MUTATION: `return sorted(res), sorted(unres)` -> this fails; without
    # this arm that mutation survives the whole file.


class TestTheRefreshStillRunsAfterTheRegistrar(unittest.TestCase):
    """#736's ordering constraint, which #746 must not disturb: the connector
    copper is absorbed into the track view BEFORE anything re-grades against
    it.  Asserted on LINE NUMBERS rather than on source text -- an assertNotIn
    against a 130KB module prints 393KB on failure (#732)."""

    @staticmethod
    def _lines(needle):
        src = inspect.getsource(FC).splitlines()
        return [i for i, l in enumerate(src, 1) if needle in l]

    def test_register_new_segments_precedes_every_grade_call(self):
        reg = self._lines('st.register_new_segments(new_segs)')
        grade = self._lines('resolved, unresolved = _grade()')
        self.assertEqual(len(reg), 1, 'the registrar moved or was duplicated')
        self.assertEqual(len(grade), 2, 'expected exactly two _grade() calls: '
                                        'one before the nudge, one after')
        self.assertLess(grade[0], reg[0], 'the pre-nudge grade must come first')
        self.assertLess(reg[0], grade[1],
                        'the re-grade now runs BEFORE the connectors are '
                        'registered, so it reads a track view missing exactly '
                        'the copper this pass drew (#736)')

    def test_the_via_view_refresh_also_precedes_the_re_grade(self):
        """#775 replaced the inline `cap_vias` comprehension with a method
        on _Repair; the ordering constraint is unchanged and the spelling
        follows it.

        The RELOCATION half is asserted here too, which it was not before.
        The refresh READS the list the registrar repairs, so refreshing
        first leaves every cap holding pre-move tuples -- a phantom via at
        the landing and a hole where the barrel really is. That ordering had
        only a behavioural killer (mutate_747's own row); now it has a
        structural one too."""
        via = self._lines('st.refresh_cap_vias()')
        reloc = self._lines('st.relocate_vias(via_moves)')
        grade = self._lines('resolved, unresolved = _grade()')
        self.assertEqual(len(via), 1, 'the refresh moved or was duplicated')
        self.assertEqual(len(reloc), 1)
        self.assertLess(reloc[0], via[0],
                        'the refresh now runs BEFORE the registrar it reads')
        self.assertLess(via[0], grade[1])

    def test_swept_is_bound_before_the_re_grade_rebinds_resolved(self):
        """`swept = resolved` is a NAME binding; if it lands after the
        rebinding it captures the post-nudge list and both deltas are
        empty on every board, silently."""
        swept = self._lines('swept = resolved')
        grade = self._lines('resolved, unresolved = _grade()')
        self.assertEqual(len(swept), 1)
        self.assertLess(swept[0], grade[1])
    # MUTATION: move `swept = resolved` below the re-grade -> via_resolved and
    # regrazed are [] everywhere; arm_a and arm_b both fail.


class TestInertOnTheTrackedCorpus(unittest.TestCase):
    """A self-expiring bound: at the SHIPPED defaults no tracked board reaches
    the via-nudge, so neither new key can be non-empty and neither new clause
    can print.  If that ever changes, the PR's inertness claim has expired and
    this says so rather than quietly passing."""

    @staticmethod
    def _reaching():
        boards = run_utils.corpus_boards()
        if not boards:
            return None
        out = []
        for b in boards:
            try:
                pcb = parse_kicad_pcb(b)
            except Exception:                                    # noqa: BLE001
                continue
            if not pcb.vias:
                continue
            # The ENGINE's own selector, not a name grep. `find_components_by_type`
            # goes through detect_package_type, which also answers 'BGA' for
            # LGA / CSP / WLCSP / WLP / CGA and for a geometric ball field with
            # no keyword at all. Measured: a name grep selects 3 of the 22
            # tracked boards and the engine's detector selects 4 -- and the
            # missed one, rp2350_fpga_eensy_prePlane (a WLCSP-49), is exactly
            # the board that REACHES the nudger at the shipped defaults. A
            # bound that excludes its own counterexample is not a bound.
            if not any(detect_package_type(f) == 'BGA'
                       for f in pcb.footprints.values()):
                continue
            out.append(b)
        return out

    def test_no_tracked_board_nudges_a_via_at_the_shipped_defaults(self):
        """NUDGES, not `reaches`: rp2350_fpga_eensy_prePlane does reach the
        nudger at clearance 0.25 and is refused every landing it tries, so it
        moves no via and neither new key can be non-empty. That is a weaker
        and TRUE statement, and it is the one this arm holds."""
        reaching = self._reaching()
        if reaching is None:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        self.assertGreaterEqual(len(reaching), 4,
                                'the candidate set collapsed; the filter is '
                                'wrong and this arm proves nothing. It was 3 '
                                'while it grepped names, which passed this '
                                'guard at the boundary while silently '
                                'dropping the one board that reaches the '
                                'nudger')
        noisy = []
        for b in reaching:
            for clr in (0.25, 0.10):
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    r = repair_fanout_clearance(parse_kicad_pcb(b), b,
                                                clearance=clr)
                out = buf.getvalue()
                if (r.get('via_resolved') or r.get('regrazed')
                        or 'freed by via-nudge' in out
                        or _line(out, '  Re-grazed')):
                    noisy.append((os.path.basename(b), clr,
                                  r.get('via_resolved'), r.get('regrazed')))
                # and the invariant holds on every board, nudge or not
                self.assertEqual(
                    sorted(set(r.get('resolved') or [])
                           & set(r.get('unresolved') or [])), [],
                    '%s @ %s reports a cap in both lists' % (b, clr))
        self.assertEqual(noisy, [],
                         'a tracked board now reaches the via-nudge at the '
                         'shipped defaults: %r. The "inert at the defaults" '
                         'claim in the #746 PR has EXPIRED -- re-run the '
                         'before/after sweep and record the new numbers.'
                         % (noisy,))
    # MUTATION: none -- a self-expiring corpus bound, not a fix assertion.


if __name__ == '__main__':
    unittest.main(verbosity=2)
