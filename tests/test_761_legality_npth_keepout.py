#!/usr/bin/env python3
"""#761: legality's NPTH copper keep-out is the inflation PLUS the clearance.

`PartPads` stores each NPTH hole at the growth ABOVE `clearance`
(`max(0, requirement - clearance)`) and the copper consumers compared that
circle against RAW pad rects, adding nothing back -- so the modelled standoff
from the hole wall was `max(0, requirement - clearance)`, which COLLAPSES TO
ZERO once `clearance` reaches the requirement.

Both siblings add the term at their own gap test: `fanout_clearance` through
`_pair_or_flat` (an NPTH tuple files floor `None`, so it hands back the flat
scalar) and `labels.py` by comparing against `silk_pad_clearance`. legality
was the only place in the repo using the constant as a subtrahend with no
matching addend, and the unit test that guarded it
(`test_npth_hole_inflation_matches_fanout_convention`) is named for the half
that was right and runs only at `clearance = 0.09`, below the fab floor,
where the bug is invisible.

Four things this file pins that are easy to get wrong:

  * The keep-out is measured against `check_drc`'s own REQUIREMENT, not
    against legality's own inflation -- that comparison is the one the defect
    survived for want of. It is a hand MIRROR and this file says so: a
    fact-check of the #761 premise established that `check_drc` has NO
    pad-vs-NPTH-hole arm at all (its `holes` list feeds a TRACK arm and a VIA
    arm, and the pad-pad check skips NPTH pads), so "grade the corpus with
    check_drc" is not available as a check on this fix. See
    `TestParityWithTheChecker`, which pins both the absence and the
    expression.
  * The broad phase must reach the hole or the fix is INERT: both early-outs
    are bounded by pad floors, and an NPTH pad reaches no pad floor.
    `TestTheBroadPhaseReachesTheHole` fails if either term is dropped.
  * SILK and EXTENTS must not move. `local_clearance` and the board's declared
    `min_hole_clearance` are copper rules; `extent_local` asks where a part
    physically IS, and #730 shipped and then fixed an off-board regression
    caused by conflating the two.
  * The corpus bound is SELF-EXPIRING. It is inert at file poses today with
    ulx3s clearing by 0.010mm; if a board moves, the PR's claim has expired
    and the arm says so rather than being nudged.

`RUN_ALL_FAST_OK`: every arm is in-process geometry plus board parses. No
chain runs here.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 900

import io
import math
import os
import sys
import unittest
from types import SimpleNamespace

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

import run_utils
import routing_defaults as defaults
from kicad_parser import parse_kicad_pcb, pad_drill_circles
from check_drc import _pad_has_no_copper
from obstacle_map import resolve_hole_clearance
from synth import make_pad, make_pcb
from placement.legality import (EPS, PAIR_TEST_CAP, LegalityContext,
                                PartPads,
                                _circle_rect_penetration, build_part_pads,
                                format_required_clause, grade_pad_legality,
                                resolve_npth_floor)

ULX3S = os.path.join(_ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
FLAT_HIER = os.path.join(_ROOT, 'kicad_files', 'flat_hierarchy.kicad_pcb')
SPLITFLAP = os.path.join(_ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

# The fab floor, IMPORTED rather than mirrored -- saying so because a reader
# who believes it is mirrored will look for a source guard that should not
# exist.
NPTH_FLOOR = defaults.NPTH_TO_TRACK_CLEARANCE                        # 0.20

# ulx3s AUDIO1: the two np_thru_hole pads, the ONLY two on the board.
AUDIO1_DRILL = 1.700
AUDIO1_LC = 0.400
AUDIO1_HR = AUDIO1_DRILL / 2.0

# Three holes on ONE footprint whose overrides STRADDLE the fab floor. One
# hole cannot distinguish per-hole from hoisted, and one override on the same
# side of the floor as the others cannot distinguish `max` from a bare read.
E_LCS = (0.00, 0.40, 0.15)

# The measured collapse, before -> after, as the STANDOFF FROM THE HOLE WALL
# for ulx3s's AUDIO1. `after` is flat because `lc` outranks every clearance
# here; `before` is the arithmetic the defect implemented.
COLLAPSE = {0.10: 0.300, 0.20: 0.200, 0.25: 0.150, 0.40: 0.000}

H_DRILL = 2.0

# `grade_pad_legality`'s neighbour-grid cell, hand-mirrored from the engine's
# `cell = 4.0` local. Only the halo arm below depends on it, and it depends on
# it structurally (is the term's reach wider than one cell?), not numerically.
CENSUS_CELL = 4.0


def _npth(x, y, lc, ref='H1', drill=H_DRILL, num='H1'):
    """A copper-less drilled pad. REAL parser dataclasses via synth, not a
    duck-typed fake: `grade_pad_legality`'s exact re-verification hands pads
    to check_drc's perimeter sampler, which reads fields a minimal fake does
    not carry -- and a fake that silently skipped the exact path would test
    the AABB fallback while claiming to test the census."""
    p = make_pad(net_id=0, x=x, y=y, ref=ref, num=num,
                 size_x=drill, size_y=drill, shape='circle',
                 layers=['F.Mask', 'B.Mask'], drill=drill,
                 pad_type='np_thru_hole')
    p.local_clearance = lc
    return p


def _cu(x, y, ref='C1', net=7, sx=1.0, sy=1.0, num='1'):
    return make_pad(net_id=net, x=x, y=y, ref=ref, num=num,
                    size_x=sx, size_y=sy, shape='rect', layers=['F.Cu'],
                    drill=0.0, pad_type='smd')


def _fp(ref, x, y, pads, rot=0.0):
    return SimpleNamespace(reference=ref, x=x, y=y, rotation=rot,
                           layer='F.Cu', pads=pads)


def _e_footprint():
    """One footprint carrying THREE NPTH holes with different overrides."""
    return _fp('H1', 10.0, 10.0,
               [_npth(10.0, 10.0 + 3.0 * i, lc, num='H%d' % (i + 1))
                for i, lc in enumerate(E_LCS)])


def _hole_pair(wall_gap, lc=AUDIO1_LC, clearance=0.25, drill=H_DRILL):
    """An NPTH hole at the origin and ONE foreign copper pad whose near edge
    sits `wall_gap` from the hole WALL. Returns (pcb, ctx, poses)."""
    hr = drill / 2.0
    cx = hr + wall_gap + 0.5
    h = _fp('H1', 0.0, 0.0, [_npth(0.0, 0.0, lc)])
    c = _fp('C1', cx, 0.0, [_cu(cx, 0.0)])
    fps = {'H1': h, 'C1': c}
    parts = build_part_pads(fps, clearance)
    poses = {r: (f.x, f.y, f.rotation) for r, f in fps.items()}
    ctx = LegalityContext(parts, None, clearance,
                          pose_of=lambda r: poses[r],
                          seed_of=lambda r: poses[r])
    return make_pcb(footprints=fps), ctx, parts


class TestTheKeepOutIsTheRequirement(unittest.TestCase):
    """The collapse, as a change detector."""

    def test_the_standoff_is_max_clearance_floor_lc_at_every_clearance(self):
        fp = _e_footprint()
        # ON THE BRANCH: the three overrides straddle the fab floor, else
        # "different" is vacuous.
        self.assertGreater(E_LCS[1], NPTH_FLOOR)
        self.assertLess(E_LCS[2], NPTH_FLOOR)
        for clearance in (0.05, 0.10, 0.20, 0.25, 0.40, 0.60):
            pp = PartPads(fp, clearance)
            got = [round(r, 9) for _x, _y, r in
                   pp.hole_keepouts(0.0, 0.0, 0.0)]
            want = [round(H_DRILL / 2.0 + max(clearance, NPTH_FLOOR, lc), 9)
                    for lc in E_LCS]
            with self.subTest(clearance=clearance):
                self.assertEqual(got, want)
    # MUTATION: keepout-drops-the-clearance (KILLED),
    # keepout-adds-clearance-twice (KILLED), site-E-loses-the-fab-floor
    # (KILLED).

    def test_the_standoff_is_NEVER_zero(self):
        """The specific thing that was wrong. A radius equal to the bare drill
        means the placer charges only literal drill overlap."""
        fp = _e_footprint()
        for clearance in (0.20, 0.25, 0.40, 0.60, 1.0):
            pp = PartPads(fp, clearance)
            for (_x, _y, r), (_ex, _ey, _er) in zip(
                    pp.hole_keepouts(0.0, 0.0, 0.0), pp.holes_local):
                with self.subTest(clearance=clearance):
                    self.assertGreater(r - H_DRILL / 2.0, 0.0)
                    # places=9, not >=: `hd/2 + x` then `- hd/2` is not
                    # exactly `x` in binary, and 0.3999999999999999 failing a
                    # `>= 0.4` is a float artefact, not a finding.
                    self.assertGreaterEqual(
                        round(r - H_DRILL / 2.0, 9), round(clearance, 9))

    def test_the_measured_collapse_on_ulx3s(self):
        """The table in the PR, re-derived from the board rather than typed."""
        pcb = parse_kicad_pcb(ULX3S)
        fp = pcb.footprints['AUDIO1']
        for clearance, before in sorted(COLLAPSE.items()):
            pp = PartPads(fp, clearance)
            circles = pp.hole_circles(fp.x, fp.y, fp.rotation or 0.0)
            keepouts = pp.hole_keepouts(fp.x, fp.y, fp.rotation or 0.0)
            with self.subTest(clearance=clearance):
                # what the defect modelled: the INFLATION alone
                self.assertAlmostEqual(circles[0][2] - AUDIO1_HR, before,
                                       places=6)
                # what it is now: check_drc's requirement, flat in clearance
                self.assertAlmostEqual(keepouts[0][2] - AUDIO1_HR, AUDIO1_LC,
                                       places=6)
        print('        ulx3s AUDIO1 standoff, was %s -> now %.4f at every one'
              % ({k: round(v, 3) for k, v in sorted(COLLAPSE.items())},
                 AUDIO1_LC))

    def test_a_SLOT_hole_keeps_the_term_on_EVERY_sampled_circle(self):
        """`pad_drill_circles` samples a slot along its capsule axis. A term
        applied to the first circle only would leave the rest of the slot at
        the bare drill, which is exactly the shape of the original defect.

        `pad_drill_capsule` reads `drill_w`/`drill_h`, NOT size/shape: a pad
        with only `shape='oval'` yields a zero-length capsule, and an arm
        built on one passes while testing nothing. Measured -- the first draft
        of this arm did exactly that."""
        p = _npth(5.0, 5.0, AUDIO1_LC, drill=3.0)
        p.drill_w, p.drill_h = 1.0, 3.0
        p.size_x, p.size_y = 1.0, 3.0
        p.shape = 'oval'
        fp = _fp('H1', 5.0, 5.0, [p])
        clearance = 0.25
        pp = PartPads(fp, clearance)
        circles = pad_drill_circles(p)
        self.assertGreater(len(circles), 1, 'not a sampled slot; arm is void')
        self.assertEqual(len(pp.holes_local), len(circles))
        want = round(circles[0][2] / 2.0
                     + max(clearance, NPTH_FLOOR, AUDIO1_LC), 9)
        self.assertEqual(
            sorted({round(r, 9) for _x, _y, r in
                    pp.hole_keepouts(0.0, 0.0, 0.0)}), [want])


class TestTheBoardFloorReachesTheKeepOut(unittest.TestCase):
    """#761's third term: the board's own declared min_hole_clearance."""

    def test_the_floor_moves_a_radius_and_the_default_does_not(self):
        fp = _e_footprint()
        clearance = 0.10
        flat = [round(r, 9) for _x, _y, r in
                PartPads(fp, clearance).hole_keepouts(0, 0, 0)]
        raised = [round(r, 9) for _x, _y, r in
                  PartPads(fp, clearance, None, True,
                           0.25).hole_keepouts(0, 0, 0)]
        # Only the two holes whose own lc is BELOW 0.25 move; the 0.40 one is
        # already above it, which is what makes this a max and not a set.
        self.assertEqual([a == b for a, b in zip(flat, raised)],
                         [False, True, False])
        self.assertEqual(
            flat, [round(r, 9) for _x, _y, r in
                   PartPads(fp, clearance, None, True, None
                            ).hole_keepouts(0, 0, 0)])
    # MUTATION: npth-floor-ignored (KILLED).

    def test_a_floor_BELOW_the_fab_floor_cannot_lower_anything(self):
        fp = _e_footprint()
        base = [round(r, 9) for _x, _y, r in
                PartPads(fp, 0.05).hole_keepouts(0, 0, 0)]
        for lowered in (0.0, 0.05, 0.19):
            with self.subTest(npth_floor=lowered):
                self.assertEqual(
                    base, [round(r, 9) for _x, _y, r in
                           PartPads(fp, 0.05, None, True, lowered
                                    ).hole_keepouts(0, 0, 0)])

    def test_flat_hierarchy_is_the_ONE_witness_on_the_corpus(self):
        """Selected the way the engine selects -- `resolve_hole_clearance` per
        board, never a name grep -- so it expires if another board declares."""
        boards = run_utils.corpus_boards()
        if not boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        declaring = {}
        for b in boards:
            pcb = parse_kicad_pcb(b)
            floor = resolve_npth_floor(pcb)
            if floor > NPTH_FLOOR + 1e-9:
                declaring[os.path.basename(b)] = round(floor, 4)
        self.assertEqual(declaring, {'flat_hierarchy.kicad_pcb': 0.25},
                         'the set of tracked boards declaring above the fab '
                         'floor has CHANGED: %r -- the #761 PR says '
                         'flat_hierarchy is the single witness, and that claim '
                         'has EXPIRED' % declaring)
        # ...and it is a witness only because its own holes declare nothing,
        # so the floor is the term that moves them.
        pcb = parse_kicad_pcb(FLAT_HIER)
        lcs = {round(getattr(p, 'local_clearance', 0.0) or 0.0, 4)
               for f in pcb.footprints.values() for p in f.pads
               if (getattr(p, 'drill', 0) or 0) > 0 and _pad_has_no_copper(p)}
        self.assertEqual(lcs, {0.0})

    def test_the_two_HOLE_consumers_pass_it_and_the_others_do_not(self):
        """A source guard, by LINE, and it exists because behaviour cannot
        reach it: the one tracked board that declares above the fab floor
        clears its nearest copper by 3.69mm, so dropping the term from the
        census changes no ANSWER on any board we have. Measured -- the
        mutation row `the-census-passes-no-board-floor` SURVIVED every
        behavioural arm in this file.

        The distinction being pinned is the whole point of the #730 tripwire's
        "needs its own review": of the six `build_part_pads` call sites, the
        two that read hole KEEP-OUTS carry the board floor, and the four that
        read only pad rects, extents or silk do not -- for them it would
        resolve a board and change nothing."""
        pairs = (('py_placer/placement/legality.py', 1, 'grade_pad_legality'),
                 ('py_placer/placement/quench.py', 1, 'quench'))
        for rel, want, who in pairs:
            src = io.open(os.path.join(_ROOT, *rel.split('/')),
                          encoding='utf-8').read()
            hits = [i + 1 for i, l in enumerate(src.splitlines())
                    if 'npth_floor=' in l.split('#')[0]]
            self.assertEqual(len(hits), want,
                             '%s passes npth_floor at %d site(s), expected '
                             '%d (%s)' % (rel, len(hits), want, who))
            # ...and passes the RESOLVER, not just the keyword. A site
            # spelling `npth_floor=None` satisfies a keyword count while
            # supplying nothing -- which is exactly what the battery row
            # `quench-passes-no-board-floor` does, and it SURVIVED until this
            # clause existed.
            resolved = [i + 1 for i, l in enumerate(src.splitlines())
                        if 'resolve_npth_floor(' in l.split('#')[0]
                        and not l.lstrip().startswith('def ')]
            self.assertGreaterEqual(
                len(resolved), want,
                '%s names npth_floor but never calls resolve_npth_floor '
                '(%s)' % (rel, who))
        for rel in ('py_placer/placement/labels.py',
                    'py_placer/placement/routability.py',
                    'py_placer/placement/seeder.py'):
            src = io.open(os.path.join(_ROOT, *rel.split('/')),
                          encoding='utf-8').read()
            hits = [i + 1 for i, l in enumerate(src.splitlines())
                    if 'npth_floor=' in l.split('#')[0]]
            self.assertEqual(hits, [],
                             '%s passes npth_floor at line(s) %r -- it reads '
                             'extents or silk, not hole keep-outs, so the '
                             'term resolves a board and changes nothing '
                             'there' % (rel, hits))

    def test_the_CALLERS_path_wins_over_the_parsed_source_path(self):
        """The #498 rule every sibling here follows. A board staged into a
        temp dir and parsed from there carries a `source_path` that is not the
        board the caller means, and `grade_pad_legality` already threads
        `pcb_file` to `PadClearanceModel.for_board` for exactly that reason.

        flat_hierarchy is the fixture because it is the one tracked board that
        declares above the fab floor -- on any other board both answers are
        0.2000 and the arm would be vacuous."""
        import shutil
        import tempfile
        real = resolve_npth_floor(parse_kicad_pcb(FLAT_HIER))
        self.assertAlmostEqual(real, 0.25, places=6)
        d = tempfile.mkdtemp(prefix='t761_')
        try:
            staged = os.path.join(d, 'staged.kicad_pcb')
            shutil.copyfile(FLAT_HIER, staged)      # the board ALONE, #441
            pcb = parse_kicad_pcb(staged)
            # ...the staged board on its own cannot answer,
            self.assertAlmostEqual(resolve_npth_floor(pcb), NPTH_FLOOR,
                                   places=6)
            # ...and the caller's path restores it.
            self.assertAlmostEqual(resolve_npth_floor(pcb, FLAT_HIER), 0.25,
                                   places=6)
        finally:
            shutil.rmtree(d, ignore_errors=True)
    # MUTATION: resolve-npth-floor-ignores-pcb_file (KILLED).

    def test_a_failed_resolution_is_NOTED_not_swallowed(self):
        """`grade_pad_legality` returns `clearance_notes` for exactly this --
        "anything that went WRONG resolving the requirement". A silent
        fallback drops the modelled floor by up to the declared value and
        returns a byte-identical report, which is the shape of silence this
        issue was filed about."""
        import obstacle_map
        pcb = parse_kicad_pcb(FLAT_HIER)
        notes = []
        self.assertAlmostEqual(resolve_npth_floor(pcb, None, notes), 0.25)
        self.assertEqual(notes, [])
        real = obstacle_map.resolve_hole_clearance

        def boom(*_a, **_k):
            raise IOError('the .kicad_pro is unreadable')

        obstacle_map.resolve_hole_clearance = boom
        try:
            self.assertAlmostEqual(resolve_npth_floor(pcb, None, notes),
                                   NPTH_FLOOR, places=6)
        finally:
            obstacle_map.resolve_hole_clearance = real
        self.assertEqual(len(notes), 1)
        self.assertIn('unreadable', notes[0])
        self.assertIn('fab floor', notes[0])
    # MUTATION: failed-floor-resolution-is-silent (KILLED).

    def test_resolve_npth_floor_survives_a_board_it_cannot_read(self):
        """A caller with no board gets the fab floor, not a crash and not 0."""
        self.assertAlmostEqual(resolve_npth_floor(None), NPTH_FLOOR)
        self.assertAlmostEqual(
            resolve_npth_floor(SimpleNamespace()), NPTH_FLOOR)


class TestParityWithTheChecker(unittest.TestCase):
    """The comparison the defect survived for want of. legality was only ever
    checked against its own inflation."""

    def test_the_gate_flips_at_check_drcs_requirement(self):
        req = max(0.25, NPTH_FLOOR, AUDIO1_LC)
        for gap, want_flag in ((req - 0.01, True), (req + 0.01, False)):
            _pcb, ctx, _parts = _hole_pair(gap)
            sf = ctx.pair_shortfall('H1', 'C1')
            with self.subTest(gap=round(gap, 4)):
                self.assertEqual(sf.hole > EPS, want_flag)
                if want_flag:
                    self.assertAlmostEqual(sf.hole, req - gap, places=6)

    def test_it_flips_the_same_way_with_the_ARGUMENTS_SWAPPED(self):
        """`pair_shortfall` runs the hole loop TWICE -- once per direction --
        and a fixture that only ever puts the hole on one side leaves the
        other loop untested. Measured: reverting exactly one of the two loops
        SURVIVED the battery until this arm existed."""
        req = max(0.25, NPTH_FLOOR, AUDIO1_LC)
        for gap, want_flag in ((req - 0.01, True), (req + 0.01, False)):
            _pcb, ctx, _parts = _hole_pair(gap)
            with self.subTest(gap=round(gap, 4)):
                # ...and asserted equal to each other, not just to the flag:
                # a loop that ran but measured the wrong pair would still
                # produce a True here.
                self.assertEqual(ctx.pair_shortfall('C1', 'H1').hole,
                                 ctx.pair_shortfall('H1', 'C1').hole)
                self.assertEqual(
                    ctx.pair_shortfall('C1', 'H1').hole > EPS, want_flag)

    def test_there_IS_no_pad_vs_hole_arm_in_check_drc_to_compare_against(self):
        """A disclosure, and the reason this file cannot close the loop by
        grading the corpus.

        #761 says "check_drc grades the same geometry at max(npth_clr, lc)".
        It grades the same RULE, not the same geometry: the `holes` list it
        builds is consumed in exactly two places, a TRACK arm and a VIA arm,
        and the pad-to-pad check skips NPTH pads outright. Nothing in this
        repo grades pad copper against an NPTH hole -- which is precisely why
        the placer could model that pair at zero for as long as it did, and
        why "run check_drc on the corpus" is not available as a check on this
        fix.

        Reported by LINE, never `assertIn` over the module source: test_732
        measured a 393KB failure message doing that."""
        src = io.open(os.path.join(_ROOT, 'py_router', 'check_drc.py'),
                      encoding='utf-8').read().splitlines()
        consumers = [i + 1 for i, l in enumerate(src)
                     if l.strip().startswith('if holes and ')]
        self.assertEqual(len(consumers), 2,
                         'check_drc consumes its NPTH `holes` list at %d '
                         'places now, not 2. If one of them is a PAD arm, '
                         'this file should compare against it instead of '
                         'mirroring the requirement' % len(consumers))
        kinds = [src[i - 1].strip() for i in consumers]
        self.assertEqual(kinds, ['if holes and segs:',
                                 'if holes and pcb_data.vias:'])
        skips = [i + 1 for i, l in enumerate(src)
                 if 'NPTH hole: no copper to short/graze' in l]
        self.assertEqual(len(skips), 2,
                         'the pad-pad check no longer skips NPTH pads at two '
                         'sites (found %r) -- re-read what it does now' % skips)

    def test_the_requirement_matches_check_drcs_own_expression(self):
        """A hand MIRROR, said out loud. `npth_clr` is a function-local inside
        `run_drc`, so it cannot be imported; this pins the line it lives on so
        the mirror breaks loudly rather than drifting."""
        src = io.open(os.path.join(_ROOT, 'py_router', 'check_drc.py'),
                      encoding='utf-8').read().splitlines()
        want = ('npth_clr = max(clearance, defaults.NPTH_TO_TRACK_CLEARANCE, '
                'hole_clearance)')
        hits = [i + 1 for i, l in enumerate(src) if l.strip() == want]
        self.assertEqual(len(hits), 1,
                         'check_drc\'s NPTH requirement is no longer spelled '
                         '%r (found at %r); the mirror below is stale' %
                         (want, hits))
        for board, ref in ((ULX3S, 'AUDIO1'), (FLAT_HIER, 'HOLE4')):
            pcb = parse_kicad_pcb(board)
            fp = pcb.footprints[ref]
            floor = resolve_npth_floor(pcb)
            for clearance in (0.10, 0.25, 0.40):
                pp = PartPads(fp, clearance, None, True, floor)
                # check_drc.py:2714 then :2733, spelled the same way.
                # Compared as SETS, not per-pad against the footprint max: a
                # max hides a hole whose own requirement is lower, which is
                # exactly the per-hole property #730 shipped.
                npth_clr = max(clearance, defaults.NPTH_TO_TRACK_CLEARANCE,
                               resolve_hole_clearance(pcb, None))
                want = sorted({
                    round(max(npth_clr,
                              getattr(p, 'local_clearance', 0.0) or 0.0), 9)
                    for p in fp.pads
                    if _pad_has_no_copper(p) and (p.drill or 0) > 0})
                with self.subTest(board=os.path.basename(board),
                                  clearance=clearance):
                    self.assertTrue(want, 'no NPTH pad; the arm is void')
                    self.assertEqual(sorted({round(r, 9)
                                             for r in pp.holes_req}), want)

    def test_the_requirement_is_DISCLOSED_not_just_counted(self):
        pcb, _ctx, _parts = _hole_pair(0.30)
        rep = grade_pad_legality(pcb, 0.25, worst_n=4, exact=False)
        self.assertEqual(rep['hole_conflicts'], 1)
        self.assertEqual(rep['required'],
                         [['C1', 'H1', round(AUDIO1_LC, 4), 'NPTH hole']])
        self.assertIn('NPTH hole', format_required_clause(rep))
    # MUTATION: hole-disclosure-removed (KILLED).

    def test_a_hole_at_the_FLAT_scalar_discloses_nothing(self):
        """`required` names pairs graded ABOVE the board-wide clearance, so an
        ordinary hole stays quiet.

        NOT literally the pad channel's bar -- that one is `pair_source != ''`
        and therefore sits at `model.base`, which a netclass can lift above
        `--clearance`. This docstring said "the same bar" until a review
        measured otherwise."""
        pcb, _ctx, _parts = _hole_pair(0.10, lc=0.0, clearance=0.25)
        rep = grade_pad_legality(pcb, 0.25, worst_n=4, exact=False)
        self.assertEqual(rep['hole_conflicts'], 1)
        self.assertEqual(rep['required'], [])


class TestTheBroadPhaseReachesTheHole(unittest.TestCase):
    """Both early-outs are bounded by PAD floors, and an NPTH pad reaches no
    pad floor -- it `continue`s before `pad_floors.append`. Without the
    `hole_reach` term the keep-out fix is INERT for exactly the pairs it was
    written for."""

    def test_a_pair_beyond_the_pad_reach_is_still_measured(self):
        clearance = 0.25
        req = max(clearance, NPTH_FLOOR, AUDIO1_LC)                  # 0.400
        gap = 0.30
        # ON THE BRANCH: the gap sits in the band the old bound dropped --
        # beyond `clearance` but inside the requirement. Without this, the arm
        # would pass for a reason that has nothing to do with the fix.
        self.assertGreater(gap, clearance)
        self.assertLess(gap, req)
        _pcb, ctx, parts = _hole_pair(gap, clearance=clearance)
        self.assertAlmostEqual(parts['H1'].hole_reach, req, places=6)
        sf = ctx.pair_shortfall('H1', 'C1')
        self.assertAlmostEqual(sf.hole, req - gap, places=6)
    # MUTATION: reach-drops-hole-reach (KILLED).

    def test_the_census_halo_reaches_it_too(self):
        clearance = 0.25
        pcb, _ctx, _parts = _hole_pair(0.30, clearance=clearance)
        rep = grade_pad_legality(pcb, clearance, worst_n=4, exact=False)
        self.assertEqual(rep['hole_conflicts'], 1)

    def test_the_census_HALO_needs_the_term_once_the_reach_clears_a_cell(self):
        """The census neighbour search is a 4.0mm cell hash expanded by
        `census_reach + 0.5`, so the term only changes an ANSWER once the
        reach can carry a conflict across a cell boundary -- below that the
        quantisation covers it and dropping the term is inert.

        Measured: with a 0.400 requirement the row
        `census-reach-drops-hole_reach` SURVIVED the battery, because the two
        parts share a cell whatever the halo is. So this arm uses a keep-clear
        WIDER than one cell. Six millimetres around a mounting hole is not a
        stunt value -- it is a screw-head/washer keep-out, which is what a
        `(clearance ...)` on an NPTH pad is usually for -- but the arm is
        honest that a corpus-scale override cannot reach this branch.

        The SEPARATION is load-bearing too, and getting it wrong is how the
        second draft of this arm still survived. The halo is symmetric -- the
        census walks every ref -- so BOTH parts' un-widened halos have to miss
        the other's cells. At 2.0mm apart they still shared cell 0 and the arm
        passed for free. 4.0mm puts the pad's own extent in cell 1 with H1
        registered in {-1, 0}, which no 0.75mm halo can bridge."""
        clearance = 0.25
        big_lc = 6.0
        self.assertGreater(big_lc, CENSUS_CELL,
                           'the census cell size; below it the halo term '
                           'cannot change an answer')
        pcb, _ctx, parts = _hole_pair(4.0, lc=big_lc, clearance=clearance)
        self.assertAlmostEqual(parts['H1'].hole_reach, big_lc, places=6)
        rep = grade_pad_legality(pcb, clearance, worst_n=4, exact=False)
        self.assertEqual(rep['hole_conflicts'], 1)
        self.assertEqual(rep['required'],
                         [['C1', 'H1', round(big_lc, 4), 'NPTH hole']])
    # MUTATION: census-reach-drops-hole_reach (KILLED by this arm, NOT by the
    # one above it -- see its docstring).

    def test_the_PAIR_TEST_CAP_branch_still_measures_the_hole(self):
        """`pair_shortfall` returns early when `pa.n_pads * pb.n_pads` exceeds
        the cap, and that branch used to hardcode `hole=0.0` -- so the hole
        channel could never fire for exactly the dense connectors that carry
        mounting holes. Corpus-reachable: glasgow_revC's J5 x U30 is 5324.

        Two grids of copper pads whose PRODUCT clears the cap. The first
        draft asserted `n * n > PAIR_TEST_CAP` on the GRID size and then built
        parts of `n*n + 1` pads each, so at n=40 it refused on 1600 while the
        real product was 1601 x 1601. The pre-check now guards the thing the
        engine actually compares -- `parts[a].n_pads * parts[b].n_pads` -- and
        n is 8, giving 64 x 65 = 4160."""
        clearance = 0.25
        n = 8
        a_pads = [_npth(0.0, 0.0, AUDIO1_LC)]
        a_pads += [_cu(-2.0 - 0.1 * i, -2.0 - 0.1 * j, 'H1', net=5,
                       sx=0.05, sy=0.05, num='p%d_%d' % (i, j))
                   for i in range(n) for j in range(n)]
        gap = 0.30
        cx = H_DRILL / 2.0 + gap + 0.5
        b_pads = [_cu(cx, 0.0, 'C1', num='b0')]
        b_pads += [_cu(cx + 2.0 + 0.1 * i, 2.0 + 0.1 * j, 'C1',
                       sx=0.05, sy=0.05, num='q%d_%d' % (i, j))
                   for i in range(n) for j in range(n)]
        fps = {'H1': _fp('H1', 0.0, 0.0, a_pads),
               'C1': _fp('C1', cx, 0.0, b_pads)}
        parts = build_part_pads(fps, clearance)
        self.assertGreater(parts['H1'].n_pads * parts['C1'].n_pads,
                           PAIR_TEST_CAP,
                           'the fixture no longer trips the cap; this arm '
                           'would then test the ordinary path')
        poses = {r: (f.x, f.y, f.rotation) for r, f in fps.items()}
        ctx = LegalityContext(parts, None, clearance,
                              pose_of=lambda r: poses[r],
                              seed_of=lambda r: poses[r])
        req = max(clearance, NPTH_FLOOR, AUDIO1_LC)
        self.assertAlmostEqual(ctx.pair_shortfall('H1', 'C1').hole,
                               req - gap, places=6)
    # MUTATION: pair-cap-goes-back-to-a-hardcoded-zero (KILLED).

    def test_the_cap_branch_does_not_bill_the_hole_to_the_PAD_channel(self):
        """`reach` feeds two things: the early-out, and the PAD shortfall the
        cap branch charges. Folding `hole_reach` into one number billed a
        single physical violation to two independent gates -- `pads_ok` tests
        `cur.pad` and `cur.hole` as separate conjuncts and quench sums
        `sf.hole` on its own.

        The tell is that the answer depended on the PAD-PAIR PRODUCT, which is
        a performance switch and must never change a verdict. Found by an
        adversarial review of this branch, not by me."""
        clearance = 0.25
        lc = 1.0
        gap = 0.30
        seen = {}
        for n in (7, 9):              # 49 and 81 copper pads: under, over
            a_pads = [_npth(0.0, 0.0, lc)]
            a_pads += [_cu(-3.0 - 0.1 * i, -3.0 - 0.1 * j, 'H1', net=5,
                           sx=0.05, sy=0.05, num='p%d_%d' % (i, j))
                       for i in range(n) for j in range(n)]
            cx = H_DRILL / 2.0 + gap + 0.5
            b_pads = [_cu(cx, 0.0, 'C1', num='b0')]
            b_pads += [_cu(cx + 3.0 + 0.1 * i, 3.0 + 0.1 * j, 'C1',
                           sx=0.05, sy=0.05, num='q%d_%d' % (i, j))
                       for i in range(n) for j in range(n)]
            fps = {'H1': _fp('H1', 0.0, 0.0, a_pads),
                   'C1': _fp('C1', cx, 0.0, b_pads)}
            parts = build_part_pads(fps, clearance)
            poses = {r: (f.x, f.y, f.rotation) for r, f in fps.items()}
            ctx = LegalityContext(parts, None, clearance,
                                  pose_of=lambda r: poses[r],
                                  seed_of=lambda r: poses[r])
            seen[n] = (parts['H1'].n_pads * parts['C1'].n_pads,
                       ctx.pair_shortfall('H1', 'C1'))
        # ON THE BRANCH: the two fixtures straddle the cap, else this arm is
        # comparing a branch with itself.
        self.assertLess(seen[7][0], PAIR_TEST_CAP)
        self.assertGreater(seen[9][0], PAIR_TEST_CAP)
        req = max(clearance, NPTH_FLOOR, lc)
        for n, (_prod, sf) in sorted(seen.items()):
            with self.subTest(pads=n):
                self.assertAlmostEqual(sf.hole, req - gap, places=6)
                self.assertAlmostEqual(sf.pad, 0.0, places=6)
    # MUTATION: cap-branch-charges-the-hole-reach-as-PAD (KILLED).

    def test_hole_reach_is_zero_for_a_part_with_no_holes(self):
        fp = _fp('C1', 0.0, 0.0, [_cu(0.0, 0.0)])
        self.assertEqual(PartPads(fp, 0.25).hole_reach, 0.0)

    def test_the_three_hole_lists_stay_index_aligned(self):
        """`hole_reach` zips two of them and the census zips a third. A length
        or order mismatch would silently mis-pair a radius with a
        requirement."""
        for fp in (_e_footprint(),
                   _fp('H1', 0, 0, [_npth(0, 0, 0.4), _cu(3, 0)])):
            pp = PartPads(fp, 0.25)
            self.assertEqual(len(pp.holes_local), len(pp.holes_extent))
            self.assertEqual(len(pp.holes_local), len(pp.holes_req))
            for (kx, ky, _kr), (ex, ey, _er) in zip(pp.holes_local,
                                                    pp.holes_extent):
                self.assertAlmostEqual(kx, ex, places=12)
                self.assertAlmostEqual(ky, ey, places=12)


class TestSilkAndExtentsDoNotMove(unittest.TestCase):
    """`local_clearance` and the board's declared floor are COPPER rules.
    #730 shipped an off-board regression by conflating the two and then fixed
    it; this change must not reopen it."""

    def test_the_silk_consumer_gets_the_same_circles_as_before(self):
        fp = _e_footprint()
        for clearance in (0.10, 0.15, 0.25):
            silk = PartPads(fp, clearance, None, False)
            with self.subTest(clearance=clearance):
                # no override term...
                self.assertEqual(
                    sorted({round(r, 9) for _x, _y, r in silk.holes_local}),
                    [round(H_DRILL / 2.0
                           + max(0.0, NPTH_FLOOR - clearance), 9)])
                # ...and no board floor either, even when one is OFFERED.
                # Offered, not merely defaulted, and the distinction is the
                # whole arm: `labels.py` -- the only `copper_holes=False`
                # caller in the tree -- passes NO floor, so the default path
                # is silent about the branch. This is a latent hole for a
                # future silk caller, not a shipped defect; silk output is
                # byte-identical across the whole of #761.
                self.assertEqual(
                    [round(r, 9) for _x, _y, r in silk.holes_local],
                    [round(r, 9) for _x, _y, r in
                     PartPads(fp, clearance, None, False, 0.90).holes_local])
    # MUTATION: silk-takes-the-board-floor (KILLED).

    def test_labels_calls_hole_circles_not_hole_keepouts(self):
        """A source guard by LINE, not `assertIn` over the module text --
        test_732 measured a 393KB failure message doing that."""
        src = io.open(os.path.join(_ROOT, 'py_placer', 'placement',
                                   'labels.py'), encoding='utf-8').read()
        hits = [i + 1 for i, l in enumerate(src.splitlines())
                if 'hole_keepouts' in l.split('#')[0]]
        self.assertEqual(hits, [],
                         'labels.py reaches the COPPER keep-out accessor at '
                         'line(s) %r; silk has no such rule' % hits)

    def test_extents_are_untouched_by_either_copper_term(self):
        fp = _e_footprint()
        for clearance in (0.10, 0.25):
            base = [round(r, 9) for _x, _y, r in
                    PartPads(fp, clearance).holes_extent]
            want = [round(H_DRILL / 2.0
                          + max(0.0, NPTH_FLOOR - clearance), 9)] * len(E_LCS)
            with self.subTest(clearance=clearance):
                self.assertEqual(base, want)
                self.assertEqual(
                    base, [round(r, 9) for _x, _y, r in
                           PartPads(fp, clearance, None, True,
                                    0.90).holes_extent])

    def test_a_huge_override_does_not_push_a_part_off_the_outline(self):
        """The #730 regression, re-measured on the board that produced it."""
        pcb = parse_kicad_pcb(SPLITFLAP)
        clearance = 0.25
        holes = [(ref, p) for ref, f in pcb.footprints.items() for p in f.pads
                 if (getattr(p, 'drill', 0) or 0) > 0 and _pad_has_no_copper(p)]
        self.assertGreater(len(holes), 0, 'no NPTH pads; the arm is void')
        base = grade_pad_legality(pcb, clearance, worst_n=0, exact=False)
        for lc in (0.90, 0.95, 1.50):
            for _ref, p in holes:
                p.local_clearance = lc
            got = grade_pad_legality(pcb, clearance, worst_n=0, exact=False)
            with self.subTest(lc=lc):
                self.assertEqual(got['oob_pad_count'],
                                 base['oob_pad_count'])
                self.assertEqual(got['oob_pad_refs'], base['oob_pad_refs'])


class TestInertOnTheTrackedCorpusAtFilePoses(unittest.TestCase):
    """Self-expiring. Corpus-inert AT FILE POSES is the #761 claim; the
    change is live during pose SEARCH, which no corpus board exercises at
    rest. Every clause selects the way the engine selects."""

    @classmethod
    def setUpClass(cls):
        cls.boards = run_utils.corpus_boards()

    def _scan(self, clearance):
        """(rows, conflicts). One row per board carrying an NPTH hole:
        (name, requirement, tightest hole-WALL-to-foreign-copper gap)."""
        rows, conflicts = [], {}
        for b in self.boards:
            pcb = parse_kicad_pcb(b)
            holes = [(ref, p) for ref, f in pcb.footprints.items()
                     for p in f.pads
                     if (getattr(p, 'drill', 0) or 0) > 0
                     and _pad_has_no_copper(p)]
            if not holes:
                continue
            floor = resolve_npth_floor(pcb)
            parts = build_part_pads(pcb.footprints, clearance, None, True,
                                    floor)
            rects = {}
            for ref, pp in parts.items():
                f = pcb.footprints[ref]
                rects[ref] = pp.pad_rects(f.x, f.y, f.rotation or 0.0)
            worst, worst_req, hits = 1e9, 0.0, 0
            for href, hp in holes:
                req = max(clearance, floor,
                          getattr(hp, 'local_clearance', 0.0) or 0.0)
                for hx, hy, hd in pad_drill_circles(hp):
                    hr = hd / 2.0
                    for ref, rs in rects.items():
                        if ref == href:
                            continue
                        for x0, y0, x1, y1, _n, _s in rs:
                            dx = max(x0 - hx, 0.0, hx - x1)
                            dy = max(y0 - hy, 0.0, hy - y1)
                            g = math.hypot(dx, dy) - hr
                            if g < req - 1e-9:
                                hits += 1
                            if g - req < worst - worst_req:
                                worst, worst_req = g, req
            rows.append((os.path.basename(b), worst_req, worst))
            if hits:
                conflicts[os.path.basename(b)] = hits
        return rows, conflicts

    def test_no_tracked_board_gains_a_hole_conflict_at_its_file_poses(self):
        if not self.boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        self.assertGreaterEqual(len(self.boards), 20,
                                'the tracked corpus collapsed; this arm '
                                'proves nothing')
        for clearance in (0.10, 0.25):
            rows, conflicts = self._scan(clearance)
            with self.subTest(clearance=clearance):
                self.assertGreaterEqual(
                    len(rows), 8,
                    'only %d boards carry an NPTH hole; the scan is not '
                    'reading them' % len(rows))
                self.assertEqual(
                    conflicts, {},
                    'the #761 "corpus-inert at file poses" claim has EXPIRED: '
                    '%r. Re-run the before/after sweep and record the numbers '
                    'rather than relaxing this arm' % conflicts)
                if clearance == 0.25:
                    for name, req, gap in sorted(rows, key=lambda r: r[2] - r[1]):
                        print('        %-30s req %.3f  gap %.4f  margin %+.4f'
                              % (name, req, gap, gap - req))

    def test_the_POSITIVE_control_the_inertness_claim_needs(self):
        """"Inert on the corpus" and "wired to nothing" print the same PASS.

        This is the arm that tells them apart: raise `--clearance` until the
        keep-out DOES bind and assert exactly which boards light up. At 0.20 --
        what `corpus_noop_sweep` and `test_placement_ab` both run at -- the
        census is silent on all 22 boards; at 0.50 it fires on three, and only
        in the hole channel.

        Measured independently twice, by me and by a reviewer who had not seen
        my numbers, and they agree to the count."""
        if not self.boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        seen = {}
        for clr in (0.20, 0.50):
            hits = {}
            for b in self.boards:
                pcb = parse_kicad_pcb(b)
                rep = grade_pad_legality(pcb, clr, worst_n=0, exact=False,
                                         pcb_file=b)
                if rep['hole_conflicts']:
                    hits[os.path.basename(b)] = rep['hole_conflicts']
            seen[clr] = hits
        self.assertEqual(seen[0.20], {},
                         'the corpus is no longer inert at the clearance the '
                         'standing gates run at: %r' % seen[0.20])
        self.assertEqual(
            seen[0.50],
            {'orangecrab_ext_pll.kicad_pcb': 2,
             'rp2350_fpga_eensy_prePlane.kicad_pcb': 4,
             'ulx3s.kicad_pcb': 1},
            'the positive control has MOVED: %r. Either the keep-out stopped '
            'binding -- in which case the corpus arms above are passing '
            'vacuously -- or a board changed. Re-measure; do not adjust this '
            'dict to match.' % seen[0.50])

    def test_ulx3s_is_the_tightest_and_it_clears_by_ten_microns(self):
        """The tightest margin in the change, asserted against the bound
        itself with the margin printed -- not against an invented threshold."""
        if not self.boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        rows, _ = self._scan(0.25)
        tightest = min(rows, key=lambda r: r[2] - r[1])
        self.assertEqual(tightest[0], 'ulx3s.kicad_pcb',
                         'the tightest tracked board is now %r, not ulx3s -- '
                         'the PR names ulx3s and that has EXPIRED' % (tightest,))
        self.assertAlmostEqual(tightest[1], AUDIO1_LC, places=4)
        self.assertAlmostEqual(tightest[2], 0.4100, places=4)
        self.assertGreater(tightest[2] - tightest[1], 0.0)
        self.assertLess(tightest[2] - tightest[1], 0.02,
                        'the margin GREW; ulx3s is no longer the near miss '
                        'the PR describes')


if __name__ == '__main__':
    unittest.main(verbosity=2)
