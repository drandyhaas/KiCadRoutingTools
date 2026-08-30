#!/usr/bin/env python3
"""#768/#769: the cap pass priced on one branch of the `--clearance` rule and
wrote back on the other, in the same invocation.

CLAUDE.md specifies two branches and only two. GIVEN, `--clearance` is a CEILING:
every class is priced at ``min(class, it)`` and the output project is clamped
down to it. OMITTED, there is no ceiling: each pair is priced at its own class
and the project is preserved.

This step did the OMITTED branch for pricing -- `PadClearanceModel.for_board`
admits any class above the flat scalar and `pair_with_source` raises the pair to
it, i.e. ``max(base, ncl)`` -- and the GIVEN branch for the writeback, calling
`fix_project_for_output(clearance=args.clearance)`, which clamps
``rules.min_clearance``, ``rules.min_hole_clearance``, the Default class and (by
default) every non-Default class. Measured at cd623938 on two real declaring
boards, no staging::

    glasgow_revC    --clearance 0.1 -> "above the 0.1mm floor: ... requires
                    0.2mm (netclass)" x19, ships net_class[Default] = 0.1
    ottercast_audio --clearance 0.1 -> same two numbers, x79

It could not do EITHER branch, because `--clearance` was declared with
``default=defaults.CLEARANCE`` -- a non-None default, so "given" and "omitted"
were indistinguishable after parse_args(). It was the only one of the seven
clearance-taking CLIs shaped that way; route.py:5871 and all five placement
siblings use ``default=None``.

WHERE THE CAP LANDS IS THE DESIGN. Not in `pair_with_source` -- in `for_board`'s
netclass admission map, which IS this pass's netclass tier and is the same place
the router caps (route.py pre-caps the netclass map before `set_net_clearances`
installs it). Three things follow, and each has an arm below:

  * The dru REPLACE and the pad override run AFTER it, so they keep OUTRANKING
    the ceiling. They must: the writeback clamps neither, so KiCad goes on
    enforcing both. That is the maintainer's Q1 on PR #729, answered in code.
  * `ceiling=None` is the default, so `grade_pad_legality` and `quench` -- the
    other two `for_board` callers, neither of which writes a project -- are
    untouched. `test_697`'s `test_default_netclass_is_not_dropped` is the
    closest thing to a collision in the repo and stays green BECAUSE of that.
  * The cap is a ``min``, not a drop. A class UNDER the ceiling still raises the
    pair normally; only one above it collapses. A resolver that merely deleted
    the map would pass a test that only checks the collapse.

TWO THINGS THE ISSUE DID NOT RECORD, both found by measurement and both fixed:

  * THE DEFECT FIRED WITH NO FLAG AT ALL. argparse supplied 0.25 and
    `fix_project_for_output`'s `clamp_nondefault_netclasses` defaults True, so
    merely RUNNING the pass on a board with a class above 0.25 clamped it.
  * `--clearance` ALSO WROTE ``min_hole_clearance``, because `compute_targets`
    defaults the copper-to-hole target to the copper clearance. The same
    glasgow run that printed "Copper-to-hole clearance 0.25mm (from the board's
    own min_hole_clearance)" then wrote 0.25 -> 0.1 into the project. That is
    #768's defect one key over, so the writeback passes the key explicitly now.

AND THE WRITEBACK IS NOT SKIPPED WHEN THE FLAG IS OMITTED, which is what an
earlier draft of this change did. Measured: that call does TWO jobs, and with no
--clearance at all it still writes 16 values on glasgow -- the board's own size
minima, the severities, `fab_floor_origin`. Skipping it throws the #441 custody
half away. The rule is per KEY: write back the number the run actually used.

#769 rides here rather than in its own PR because it is the same rule applied
twice. The clamp used to run only when a cap actually MOVED, so a run asked for
a ceiling that legitimately moved nothing took the unchanged-copy path and
shipped the INPUT's spec for the next step to grade against.

LAYER COUNT. The cap has no layer axis -- it operates on a net-name -> clearance
map -- and `TestLayerCountIsNotAnAxis` is what says so rather than assumes it.
The one layer-dependent quantity in reach is the FAB clearance floor, which
buckets at 2 (0.10mm) vs 4 (0.09mm) with the boundary at THREE copper layers.
It is disclosed and deliberately NOT applied: `check_drc` does not fab-floor
copper clearance either (`_pair_cl` has no `fab_floor_min` in its chain), and
#756's rule for this function's two sibling resolvers is MATCH THE GRADER.
Wrapping would refuse cap landings this pass's own checker passes.

Conventions (from #725/#731/#732/#733/#737/#750/#756 and CLAUDE.md): REAL parser
dataclasses; every assertion names the single-line MUTATION that must kill it,
with the count the battery measured; assert you are ON the branch before
asserting about it; every refusal paired with an acceptance that still happens.
The battery ships as `tests/mutate_768.py`. Measured, 31 rows:

    31 KILLED, 0 SURVIVED, 0 BROKEN

RE-MEASURED AT #780, AND THE PREVIOUS NUMBER WAS ALREADY WRONG. It said
28 rows / 27 KILLED / 1 expected SURVIVOR; the branch that shipped it had
30 rows and no survivor -- the row it named
(`writeback-spends-the-flag-not-the-priced-value`) is not in ROWS and was
not in ROWS then either. #780 adds one row, takes the total to 31, and
re-derives the whole line from a run rather than editing the old one to
agree. Two rows in that run reported BROKEN before being re-anchored, both
because #780 moved text a row quoted; a BROKEN row exits the battery
non-zero, which is how they were found rather than assumed.

Seven of those rows exist because a CLI/GUI parity review found the GUI gate
reading the WRONG control (`fix_drc_settings`, a box that clamps no net class at
all) and, on the standalone call path, reading a key that config never carries.
Two string-count assertions in this file passed it. The rows that now catch it
name `tests/gui_parity/test_768_cap_ceiling_real_dialog.py`, which drives the
real headless dialog and captures the kwargs the engine is handed -- the only
thing that can decide a VALUE.

The expected survivor is `writeback-spends-the-flag-not-the-priced-value`:
`clearance=_priced` and `clearance=args.clearance` are provably equal on every
board reachable here, so nothing can prove the honest spelling is the one that
shipped. It is recorded rather than deleted, because an inert row quietly
removed is a hole and an inert row recorded is a finding.

TWO OF THESE ARMS EXIST BECAUSE THE BATTERY FOUND THEM HOLLOW, and both are
labelled where they sit: the layer-degradation arm's three degenerate lists all
had length 0 or 2, so a resolver that dropped the `.Cu` filter still bucketed
correctly and the row SURVIVED; and the cap-is-a-min arm compared the value SET
only, so a mutant that put EVERY net at the ceiling matched it. Six `# MUTATION:`
notes also carried counts written before the run, one of them naming the wrong
test entirely -- corrected here against what was measured, which is the whole
reason `test_750:151-155` records the same mistake.
"""
RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 1800

import contextlib
import io
import json
import os
import re
import shutil
import subprocess
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
import routing_defaults as defaults
from fab_tiers import fab_floor_min
from kicad_parser import parse_kicad_pcb
from placement import fanout_clearance as FC
from placement.fanout_clearance import (fab_pair_clearance_floor,
                                        repair_fanout_clearance,
                                        resolve_pair_clearance)
from placement.legality import PadClearanceModel

# --- the rig ---------------------------------------------------------------
# flat_hierarchy is one of only TWO tracked boards carrying a sibling .kicad_pro
# (test_756:865 pins that set), and the only one declaring a NON-Default class.
# Default 0.2 + Wide 0.4 is exactly the shape this change is about, so it is the
# positive control rather than a staged fixture.
FLAT = os.path.join(_ROOT, 'kicad_files', 'flat_hierarchy.kicad_pcb')
# No sibling project at all -- the "20 of 22" case, where every branch below
# must resolve to the packaged default and change nothing.
NOPRO = os.path.join(_ROOT, 'kicad_files', 'tigard.kicad_pcb')
# Moves caps at file poses, and carries no project, so a staged one is entirely
# this file's own. The only tracked board that can witness the writeback
# END TO END, because the writeback only runs on a board that moved something.
MOVER = os.path.join(_ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')

FLAT_DEFAULT = 0.2      # its Default net class
FLAT_WIDE = 0.4         # its second class, the one clamp_nondefault_netclasses
                        # silently took to 0.25 with no flag given at all
CLI = os.path.join(_ROOT, 'py_placer', 'place_fanout_clearance.py')
ANIM = os.path.join(_ROOT, 'py_tools', 'animate_fanout_clearance.py')


def _classes(clearance, wide=None):
    cs = [{'name': 'Default', 'clearance': clearance, 'track_width': 0.2,
           'via_diameter': 0.6, 'via_drill': 0.4, 'priority': 2147483647}]
    if wide is not None:
        cs.append({'name': 'Wide', 'clearance': wide, 'track_width': 0.4,
                   'via_diameter': 0.8, 'via_drill': 0.4, 'priority': 1})
    return cs


def _stage(td, src, name, classes=None, hole_clearance=None, dru=None):
    """Copy a board + siblings into `td`, optionally rewriting its project.

    copy_board, never shutil: a bare .kicad_pcb copy strands the DRC floor and
    every number in this file would then be measured against the stock netclass
    (#441). The WARNING it prints for a project-less source is expected."""
    from copy_board import copy_board
    dst = os.path.join(td, name + '.kicad_pcb')
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        copy_board(src, dst)
    pro = os.path.splitext(dst)[0] + '.kicad_pro'
    if classes is not None or hole_clearance is not None:
        doc = json.load(open(pro, encoding='utf-8')) if os.path.isfile(pro) \
            else {'board': {'design_settings': {'rules': {}}},
                  'net_settings': {'classes': []}}
        doc.setdefault('board', {}).setdefault('design_settings', {}) \
           .setdefault('rules', {})
        if classes is not None:
            doc.setdefault('net_settings', {})['classes'] = classes
        if hole_clearance is not None:
            doc['board']['design_settings']['rules']['min_hole_clearance'] = \
                hole_clearance
        json.dump(doc, open(pro, 'w', encoding='utf-8'), indent=2)
    if dru is not None:
        with open(os.path.splitext(dst)[0] + '.kicad_dru', 'w',
                  encoding='utf-8') as fh:
            fh.write(dru)
    return dst


def _pro(board_path):
    p = os.path.splitext(board_path)[0] + '.kicad_pro'
    if not os.path.isfile(p):
        return None
    doc = json.load(open(p, encoding='utf-8'))
    rules = doc.get('board', {}).get('design_settings', {}).get('rules', {})
    return {
        'min_clearance': rules.get('min_clearance'),
        'min_hole_clearance': rules.get('min_hole_clearance'),
        'classes': dict((c.get('name'), c.get('clearance'))
                        for c in doc.get('net_settings', {}).get('classes', [])),
    }


def _run_cli(*argv, **kw):
    r = subprocess.run([sys.executable, '-X', 'utf8', CLI] + [str(a) for a in argv],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=_ROOT, timeout=kw.get('timeout', 900))
    if r.returncode != 0:
        raise AssertionError('the CLI failed (exit %d):\n%s'
                             % (r.returncode, (r.stdout + r.stderr)[-2000:]))
    return r.stdout + r.stderr


def _nets_by_class(model, value):
    return sorted(k for k, v in model.net_floor.items()
                  if abs(v - value) < 1e-9)


# ---------------------------------------------------------------------------
class TestTheCapIsAMinNotADrop(unittest.TestCase):
    """The arithmetic, on the one tracked board that declares two classes.

    A resolver that DELETED the netclass map whenever a ceiling was given would
    pass a test that only checks the collapse case, and would be wrong on every
    board whose class sits under the ceiling. Both directions are asserted."""

    @classmethod
    def setUpClass(cls):
        if not os.path.exists(FLAT):
            raise unittest.SkipTest('flat_hierarchy fixture not present')
        cls.pcb = parse_kicad_pcb(FLAT)

    def _model(self, base, ceiling=None):
        return PadClearanceModel.for_board(self.pcb, base, FLAT,
                                           ceiling=ceiling)

    def test_ON_THE_BRANCH_flat_hierarchy_declares_the_two_classes_this_file_assumes(self):
        """Assert the rig before asserting about it. Every number below is
        derived from these two, and a fixture edit must fail HERE rather than
        silently turn the arms into tautologies."""
        from list_nets import read_design_rules
        classes = read_design_rules(FLAT).get('classes') or {}
        self.assertAlmostEqual(classes.get('Default', {}).get('clearance'),
                               FLAT_DEFAULT, places=9)
        self.assertAlmostEqual(classes.get('Wide', {}).get('clearance'),
                               FLAT_WIDE, places=9)

    def test_no_ceiling_is_byte_identical(self):
        """The opt-in default. test_697:396-405 asserts this from the other
        side; it is restated here so the default can never quietly become
        opt-out without a named test going red.

        MUTATION: `if ceiling is not None:` -> `if ceiling is None:`, which
        sends the uncapped path into `min(v, None)` -- battery row
        `cap-guard-inverted`, the widest in the battery (KILLED, 10 assertions,
        3 of them in test_697)."""
        m = self._model(FLAT_DEFAULT)
        self.assertEqual({round(v, 6) for v in m.net_floor.values()},
                         {FLAT_WIDE})
        self.assertIsNone(m.ceiling)
        self.assertTrue(m.active)

    def test_a_class_UNDER_the_ceiling_still_raises_the_pair(self):
        """base 0.2, ceiling 0.3: Wide is 0.4, capped to 0.3, and 0.3 is still
        above the base, so it stays in the map AT 0.3 and still raises.

        This is the arm a `del by_name` or a `by_name = {}` mutant fails while
        the collapse arm below stays green.

        THE MEMBERSHIP MATTERS AS MUCH AS THE VALUE, and the battery is why
        this arm says so. It first compared only the value SET, and the row
        `cap-is-assignment-not-min` -- `{n: ceiling}` instead of
        `{n: min(v, ceiling)}` -- SURVIVED it: that mutant puts EVERY net at
        0.3, so `{0.3}` still matched while the map had gone from 4 nets to
        111. Only `test_a_ceiling_ABOVE_every_class_changes_nothing` caught it.

        MUTATION: `min(v, ceiling)` -> `ceiling` -- battery row
        `cap-is-assignment-not-min` (KILLED, 2 assertions, one of them here)."""
        m = self._model(FLAT_DEFAULT, ceiling=0.3)
        bare = self._model(FLAT_DEFAULT)
        self.assertEqual({round(v, 6) for v in m.net_floor.values()}, {0.3})
        self.assertEqual(sorted(m.net_floor), sorted(bare.net_floor),
                         'the ceiling changed WHICH nets are admitted, not '
                         'just their value: %d nets capped vs %d admitted '
                         'without a ceiling'
                         % (len(m.net_floor), len(bare.net_floor)))
        self.assertTrue(m.active)
        # and the pair really resolves there, not just the map
        wide = _nets_by_class(m, 0.3)
        self.assertTrue(wide, 'no net carries the capped Wide class')
        fa = FC.PadFloor(0.3, 0.0, None)
        fb = FC.PadFloor(0.0, 0.0, None)
        self.assertEqual(m.pair_with_source(fa, fb), (0.3, 'netclass'))

    def test_a_class_OVER_the_ceiling_collapses_to_the_base(self):
        """base 0.1, ceiling 0.1: every class caps to 0.1, none is above the
        base, the map empties and the model goes INERT -- which is the same
        state a board declaring nothing produces, so every consumer takes its
        original flat path.

        MUTATION: drop the `min(v, ceiling)` line entirely -- battery row
        `cap-removed` (KILLED, 3 assertions)."""
        m = self._model(0.1, ceiling=0.1)
        self.assertEqual(m.net_floor, {})
        self.assertFalse(m.active,
                         'the model must go inert, not merely empty: every '
                         'consumer branches on .active')
        self.assertAlmostEqual(m.ceiling, 0.1, places=9)

    def test_a_ceiling_ABOVE_every_class_changes_nothing(self):
        """The no-op direction. `--clearance 0.5` on a 0.4 board is not a
        licence to move anything."""
        loose = self._model(FLAT_DEFAULT, ceiling=0.5)
        bare = self._model(FLAT_DEFAULT)
        self.assertEqual(loose.net_floor, bare.net_floor)

    def test_the_cap_is_DISCLOSED_and_names_both_the_value_and_the_count(self):
        """A silent cap is the same defect in the other direction: the operator
        cannot tell a class that was honoured from one that was overruled.

        MUTATION: delete the notes.append -> battery row `cap-not-disclosed`
        (KILLED, 1 assertion -- this one)."""
        m = self._model(0.1, ceiling=0.1)
        capped = [n for n in m.notes if 'capped at the' in n]
        self.assertEqual(len(capped), 1, m.notes)
        self.assertIn('0.4 -> 0.1', capped[0])
        self.assertIn('0.2 -> 0.1', capped[0])
        # and NOT disclosed when nothing was capped
        self.assertEqual([n for n in self._model(FLAT_DEFAULT, ceiling=0.5).notes
                          if 'capped at the' in n], [])


# ---------------------------------------------------------------------------
class TestTheTiersAboveTheCeilingSurviveIt(unittest.TestCase):
    """The maintainer's Q1 on PR #729, answered in code.

    "Should the netclass term be capped at --clearance, with pad overrides and
    .kicad_dru rules still outranking it?" -- yes, and these are the two arms
    that hold it to that. Both must survive, because the writeback clamps
    NEITHER: a pad `(clearance ...)` is a per-item KiCad rule and a dru rule is
    a custom rule, and KiCad goes on enforcing both whatever the project's
    classes say."""

    def test_a_dru_layer_rule_ABOVE_the_ceiling_still_wins(self):
        """MUTATION: move the `min(v, ceiling)` cap to AFTER the layer-rule
        branch in pair_with_source -> battery row `cap-applied-after-dru`."""
        if not os.path.exists(FLAT):
            self.skipTest('flat_hierarchy fixture not present')
        with tempfile.TemporaryDirectory() as td:
            b = _stage(td, FLAT, 'ruled',
                       dru='(version 1)\n(rule wide_front (layer "F.Cu") '
                           '(constraint clearance (min 0.6mm)))\n')
            pcb = parse_kicad_pcb(b)
            m = PadClearanceModel.for_board(pcb, 0.1, b, ceiling=0.1)
            self.assertEqual(m.layer_rules, {'F.Cu': 0.6},
                             'the .kicad_dru did not parse; this arm would '
                             'pass for the wrong reason')
            f = FC.PadFloor(0.0, 0.0, frozenset({'F.Cu'}))
            self.assertEqual(m.pair_with_source(f, f), (0.6, 'layer rule'))

    def test_a_pad_override_ABOVE_the_ceiling_still_wins(self):
        m = PadClearanceModel(0.1, has_overrides=True, ceiling=0.1)
        fa = FC.PadFloor(0.0, 1.016, None)
        fb = FC.PadFloor(0.0, 0.0, None)
        self.assertEqual(m.pair_with_source(fa, fb), (1.016, 'pad override'))

    def test_pair_with_source_ITSELF_is_untouched(self):
        """The cap deliberately does NOT live here. `pair_with_source` is the
        shared grader-mirror reached by `grade_pad_legality` and `quench`,
        neither of which writes a project -- capping there would change what
        three unrelated CLIs grade at, to fix a contradiction only one of them
        has. Asserted on the source so a later edit has to argue with it."""
        import inspect
        src = inspect.getsource(PadClearanceModel.pair_with_source)
        stripped = '\n'.join(l.split('#')[0] for l in src.splitlines())
        for banned in ('ceiling', 'min('):
            self.assertNotIn(banned, stripped,
                             'the ceiling leaked into pair_with_source; it '
                             'belongs in for_board\'s admission map, which is '
                             'this pass\'s netclass tier and the only tier the '
                             'writeback clamps')


# ---------------------------------------------------------------------------
class TestTheOtherTwoCallersAreUntouched(unittest.TestCase):
    """`ceiling=None` is the default, and these are the callers that rely on
    it. Both write no project, so a class they price at is a class KiCad will
    still enforce -- capping there would price below the grade."""

    def test_grade_pad_legality_builds_an_UNCAPPED_model(self):
        """This is test_697's `test_default_netclass_is_not_dropped` restated
        from the #768 side: it is the closest thing to a collision in the repo
        and it stays green BECAUSE the ceiling is opt-in.

        Asserted on the MODEL and on the report's own notes, not on `required`.
        A first draft of this arm asserted a `netclass` row in
        `grade_pad_legality(pcb, 0.1, pcb_file=FLAT)['required']` and failed --
        then measured IDENTICALLY empty at cd623938, because no pad pair on
        flat_hierarchy is within charging reach at all. It was asserting
        something that had never been true rather than something #768 could
        break, which is the failure mode a test like this exists to avoid."""
        if not os.path.exists(FLAT):
            self.skipTest('flat_hierarchy fixture not present')
        from placement.legality import grade_pad_legality
        pcb = parse_kicad_pcb(FLAT)
        # The model grade_pad_legality builds internally, built the same way.
        m = PadClearanceModel.for_board(pcb, 0.1, FLAT)
        self.assertEqual({round(v, 6) for v in m.net_floor.values()},
                         {FLAT_DEFAULT, FLAT_WIDE},
                         'a class above an explicit clearance stopped '
                         'entering the map for the graders')
        fa = FC.PadFloor(FLAT_WIDE, 0.0, None)
        self.assertEqual(m.pair_with_source(fa, FC.PadFloor(0.0, 0.0, None)),
                         (FLAT_WIDE, 'netclass'))
        # and the caller really did build it uncapped: a capped one discloses.
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            g = grade_pad_legality(pcb, 0.1, worst_n=0, pcb_file=FLAT)
        self.assertEqual([n for n in g['clearance_notes']
                          if 'capped at the' in n], [],
                         'grade_pad_legality capped a net class; it writes no '
                         'project, so that class is one KiCad will enforce')

    def test_neither_caller_passes_a_ceiling(self):
        """MUTATION: add `ceiling=clearance` at either site -> this goes red,
        and so does the arm above."""
        import inspect
        from placement import quench
        from placement import legality
        for mod in (quench, legality):
            src = inspect.getsource(mod)
            for m in re.finditer(r'PadClearanceModel\.for_board\(([^)]*)\)',
                                 src, re.S):
                self.assertNotIn('ceiling', m.group(1),
                                 '%s passes a ceiling; it writes no project, '
                                 'so it must price at the class KiCad will '
                                 'enforce' % mod.__name__)


# ---------------------------------------------------------------------------
class TestResolvePairClearance(unittest.TestCase):
    """The two branches, as route.py spells them, and the ONE input where they
    deliberately differ.

    A DECLARED clearance of 0 or less: route.py reads it straight
    (`board_default_netclass_clearance` returns 0.0, and only the hole and edge
    floors go through `resolve_cli_floor`'s unset rule), so it would price at 0.
    This follows `board_floor_knobs` (list_nets.py:352-355) and treats it as
    UNSET, because KiCad writes 0 into these fields for "not configured" and a
    pass that priced every pair at 0 would move nothing and grade everything
    clean.

    Reachable, not theoretical: `allwinner_h3_ddr3` and `spartan6_4layer` both
    declare exactly 0.0. This class used to claim the two resolutions were the
    same; a fact-check refuted it."""

    def test_omitted_takes_the_boards_own_default_class(self):
        self.assertEqual(resolve_pair_clearance(FLAT, None),
                         (FLAT_DEFAULT, 'board netclass'))

    def test_omitted_on_a_project_less_board_takes_the_packaged_default(self):
        """The 20-of-22 case. MUTATION: return 0.0 here -> every corpus arm
        below goes red."""
        self.assertEqual(resolve_pair_clearance(NOPRO, None),
                         (defaults.CLEARANCE, 'fixed default'))
        self.assertEqual(resolve_pair_clearance('', None),
                         (defaults.CLEARANCE, 'fixed default'))

    def test_given_caps_the_DEFAULT_class_too(self):
        """`min(Default, ceiling)`, not the ceiling and not the class. There is
        nothing special about the Default class -- CLAUDE.md says so in as many
        words, and route.py's base resolution is the same expression.

        MUTATION: `min(declared, clearance)` -> `clearance` -- battery row
        `base-ignores-the-declaration` (KILLED, 1 assertion: the ceiling-ABOVE
        case below. The ceiling-BELOW case cannot catch it, because there the
        two expressions agree)."""
        # ceiling BELOW the class: the ceiling wins
        self.assertEqual(resolve_pair_clearance(FLAT, 0.1), (0.1, 'cli'))
        # ceiling ABOVE the class: the CLASS wins, because a ceiling is a cap
        # and not a floor. A step that read 0.3 here would price a board
        # declaring 0.2 at 0.3 and move caps that needed no moving.
        self.assertEqual(resolve_pair_clearance(FLAT, 0.3),
                         (FLAT_DEFAULT, 'cli'))

    def test_given_on_a_project_less_board_is_the_value_itself(self):
        self.assertEqual(resolve_pair_clearance(NOPRO, 0.15), (0.15, 'cli'))

    def test_a_declared_zero_is_UNSET_not_a_floor_of_zero(self):
        """KiCad writes 0 for "not configured". Read straight it collapses the
        pass to no clearance at all -- `board_floor_knobs` (list_nets.py:352)
        encodes the same rule and this must not diverge from it.

        MUTATION: drop the `declared <= 0` guard -> battery row
        `zero-declaration-honoured` (KILLED, 1 assertion)."""
        with tempfile.TemporaryDirectory() as td:
            b = _stage(td, FLAT, 'zeroed', classes=_classes(0.0))
            self.assertEqual(resolve_pair_clearance(b, None),
                             (defaults.CLEARANCE, 'fixed default'))
            self.assertEqual(resolve_pair_clearance(b, 0.15), (0.15, 'cli'))

    def test_it_DIVERGES_from_route_py_on_a_declared_zero(self):
        """Pinned rather than left in prose, because the divergence is the
        thing a reader is most likely to assume away.

        MUTATION: make the `declared <= 0` guard match route.py (drop it) ->
        battery row `zero-declaration-honoured`, and this arm names WHY."""
        with tempfile.TemporaryDirectory() as td:
            b = _stage(td, FLAT, 'zero_div', classes=_classes(0.0))
            from list_nets import board_default_netclass_clearance
            # route.py's input: the raw declaration, which is 0.0 and not None.
            self.assertEqual(board_default_netclass_clearance(b), 0.0,
                             'the fixture no longer declares a zero class; '
                             'this arm proves nothing')
            # ours: UNSET, so the packaged default rather than 0.
            self.assertEqual(resolve_pair_clearance(b, None),
                             (defaults.CLEARANCE, 'fixed default'))

    def test_an_unreadable_project_does_not_raise(self):
        with tempfile.TemporaryDirectory() as td:
            b = _stage(td, FLAT, 'broken')
            with open(os.path.splitext(b)[0] + '.kicad_pro', 'w',
                      encoding='utf-8') as fh:
                fh.write('{ this is not json')
            self.assertEqual(resolve_pair_clearance(b, None),
                             (defaults.CLEARANCE, 'fixed default'))


# ---------------------------------------------------------------------------
class TestLayerCountIsNotAnAxis(unittest.TestCase):
    """The cap operates on a net-name -> clearance map, which has no layer
    axis. Said with a test rather than assumed, because the ONE layer-dependent
    quantity in reach -- the fab clearance floor -- sits two lines away."""

    def test_the_capped_map_is_identical_at_2_4_8_and_16_copper_layers(self):
        """MUTATION: make the cap consult `board_copper` in any way -> red."""
        if not os.path.exists(FLAT):
            self.skipTest('flat_hierarchy fixture not present')
        pcb = parse_kicad_pcb(FLAT)
        got = {}
        for n in (2, 4, 8, 16):
            layers = ['F.Cu'] + ['In%d.Cu' % i for i in range(1, n - 1)] + ['B.Cu']
            self.assertEqual(len(layers), n)
            pcb.board_info.copper_layers = layers
            m = PadClearanceModel.for_board(pcb, FLAT_DEFAULT, FLAT,
                                            ceiling=0.3)
            got[n] = {k: round(v, 9) for k, v in m.net_floor.items()}
        self.assertEqual(len(set(map(str, got.values()))), 1,
                         'the capped netclass map moved with the layer '
                         'count: %r' % {k: len(v) for k, v in got.items()})
        self.assertTrue(got[2], 'the map is empty; this arm proves nothing')

    def test_the_fab_clearance_floor_DOES_bucket_and_the_boundary_is_three(self):
        """A change detector, not a mirror. `hole_to_hole` is flat across
        buckets (which is why #756 could ignore layers); `clearance` is not,
        so the disclosure must read the bucket rather than a literal.

        If a future tier flattens this, or moves the boundary, this goes red
        and the disclosure's wording has to be revisited."""
        two, four = fab_floor_min(2)['clearance'], fab_floor_min(4)['clearance']
        self.assertNotEqual(two, four,
                            'the fab clearance floor stopped depending on the '
                            'layer count; the disclosure text below now says '
                            'something vacuous')
        self.assertEqual(fab_floor_min(3)['clearance'], four,
                         'the bucket boundary moved off 3 copper layers')
        self.assertEqual(fab_floor_min(1)['clearance'], two)
        # the sibling floors #756 relies on being flat, asserted here so the
        # two claims cannot drift apart unnoticed
        for k in ('hole_to_hole', 'pad_hole_to_hole'):
            self.assertEqual(fab_floor_min(2)[k], fab_floor_min(4)[k], k)

    def test_the_layer_count_degrades_exactly_as_check_drc_does(self):
        """`len(copper_layers) if copper_layers else 2` (check_drc.py:2070):
        an unreadable layer list takes the CONSERVATIVE bucket, never bucket 0.

        MUTATIONS: drop the `or 2` -> battery row `layer-fallback-dropped`
        (KILLED, 1). Drop the `.Cu` filter -> `layer-filter-dropped`, which
        SURVIVED until this arm grew its last case: the three degenerate lists
        all have length 0 or 2, so an unfiltered resolver still landed in
        bucket 2 and the row passed against a defect it had introduced."""
        class _BI:
            def __init__(self, layers):
                self.copper_layers = layers

        class _P:
            def __init__(self, layers):
                self.board_info = _BI(layers)

        two = fab_floor_min(2)['clearance']
        for layers in ([], None, ['F.Mask', 'Edge.Cuts']):
            self.assertEqual(fab_pair_clearance_floor(_P(layers)), (two, 2),
                             'layers=%r took the wrong bucket' % (layers,))
        # THE DISCRIMINATING CASE, and the battery is why it is here: the three
        # above all have length 0 or 2, so a resolver that dropped the `.Cu`
        # filter outright still landed in bucket 2 and the row
        # `layer-filter-dropped` SURVIVED. A real 2-layer board carries mask,
        # silk and Edge.Cuts entries too, so the unfiltered length is 5 -- a
        # DIFFERENT bucket from the 2 copper layers it actually has.
        two_layer_board = ['F.Cu', 'B.Cu', 'F.Mask', 'F.SilkS', 'Edge.Cuts']
        self.assertEqual(fab_pair_clearance_floor(_P(two_layer_board)),
                         (two, 2),
                         'the .Cu filter was dropped: a 2-copper-layer board '
                         'carrying mask and silk entries bucketed as if it '
                         'had %d copper layers' % len(two_layer_board))

        class _NoInfo:
            board_info = None
        self.assertEqual(fab_pair_clearance_floor(_NoInfo()), (two, 2))
        self.assertEqual(
            fab_pair_clearance_floor(_P(['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'])),
            (fab_floor_min(4)['clearance'], 4))

    def test_a_sub_fab_ceiling_is_DISCLOSED_and_NOT_applied(self):
        """MATCH THE GRADER (#756's rule for this function's two siblings).
        `check_drc` does not fab-floor copper clearance -- `_pair_cl` is
        max(clearance, ncl_a, ncl_b) -> dru -> pad override with no
        `fab_floor_min` in the chain -- so raising here would refuse cap
        landings this pass's own checker passes.

        Worth an arm of its own because the netclass tier USED to rescue a
        too-small value by raising the pair back up to the class, and a ceiling
        deliberately stops it doing that.

        MUTATION: `if clearance < _fab_clr` -> `clearance = max(clearance,
        _fab_clr)` -- battery row `sub-fab-clamped` (KILLED, 1 assertion)."""
        if not os.path.exists(NOPRO):
            self.skipTest('tigard fixture not present')
        pcb = parse_kicad_pcb(NOPRO)
        fab, n = fab_pair_clearance_floor(pcb)
        tiny = round(fab / 2.0, 6)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            repair_fanout_clearance(pcb, NOPRO, clearance=tiny,
                                    netclass_ceiling=tiny)
        out = buf.getvalue()
        self.assertIn('cap pair clearance: %gmm (cli)' % tiny, out,
                      'the sub-fab value was NOT priced as asked')
        self.assertRegex(out, r'below the %d-layer fab\s+clearance floor' % n)
        self.assertIn('%g' % fab, out)
        # and silent when the value is at or above the floor
        buf2 = io.StringIO()
        with contextlib.redirect_stdout(buf2):
            repair_fanout_clearance(pcb, NOPRO, clearance=fab,
                                    netclass_ceiling=fab)
        self.assertNotIn('fab', buf2.getvalue().split('BGAs:')[0].lower())


# ---------------------------------------------------------------------------
class TestTheEngineResolvesAndDiscloses(unittest.TestCase):

    def test_the_engine_clearance_default_is_None(self):
        """The whole defect was that the CLI could not tell given from
        omitted. `board_edge_clearance` has had this contract since #733 and
        test_733:564 pins it the same way."""
        import inspect
        sig = inspect.signature(repair_fanout_clearance)
        self.assertIsNone(sig.parameters['clearance'].default,
                          'the engine cannot tell an omitted --clearance from '
                          'a given one')
        self.assertIsNone(sig.parameters['netclass_ceiling'].default,
                          'a ceiling must be opt-in, or grade_pad_legality '
                          'and quench inherit it')

    def test_the_priced_value_is_printed_with_its_SOURCE(self):
        """Printed from the ENGINE, so the CLI and the GUI plugin both inherit
        it -- the reason the edge margin is printed there too. On main NOTHING
        disclosed this number: the measured glasgow "omitted" run names no
        copper clearance anywhere in its transcript.

        MUTATION: delete the print -> battery row `priced-not-disclosed`
        (KILLED, 3 assertions across three arms)."""
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            repair_fanout_clearance(parse_kicad_pcb(FLAT), FLAT)
        self.assertIn('cap pair clearance: %gmm (board netclass)' % FLAT_DEFAULT,
                      buf.getvalue())
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            repair_fanout_clearance(parse_kicad_pcb(NOPRO), NOPRO)
        self.assertIn('cap pair clearance: %gmm (fixed default)'
                      % defaults.CLEARANCE, buf.getvalue())


# ---------------------------------------------------------------------------
class TestTheCLI(unittest.TestCase):

    def test_neither_front_declares_a_numeric_default(self):
        """The source guard, over the SAME two scripts test_733:638-644 covers
        for --board-edge-clearance. The animator is the third front, and the
        placement README promises it "gets the same answer" -- left with a
        numeric default it would price with max(base, netclass) while the CLI
        priced with the ceiling, so the GIF would show a repair the tool does
        not perform.

        Comment-stripped, because a docstring quoting the old line would
        otherwise satisfy a plain `in` test -- the exact false pass #756's
        fact-check found in its own source arm."""
        for script in (CLI, ANIM):
            with open(script, encoding='utf-8') as fh:
                src = '\n'.join(l.split('#')[0]
                                for l in fh.read().splitlines())
            self.assertIn('"--clearance", type=float, default=None', src,
                          os.path.basename(script))
            self.assertNotIn('"--clearance", type=float, default=defaults',
                             src, os.path.basename(script))

    def test_the_animator_passes_the_same_ceiling(self):
        """It writes no project, so it clamps nothing; it takes the ceiling
        anyway because its job is to SHOW what the CLI does."""
        with open(ANIM, encoding='utf-8') as fh:
            src = '\n'.join(l.split('#')[0] for l in fh.read().splitlines())
        self.assertIn('netclass_ceiling=args.clearance', src)

    def test_the_ceiling_reaches_the_engine(self):
        with open(CLI, encoding='utf-8') as fh:
            src = '\n'.join(l.split('#')[0] for l in fh.read().splitlines())
        self.assertIn('netclass_ceiling=args.clearance', src,
                      'the CLI resolves a ceiling it never passes')


# ---------------------------------------------------------------------------
class TestTheWritebackWritesWhatWasPriced(unittest.TestCase):
    """#768's second half and #769, which are the same rule applied twice.

    Every arm here drives the REAL CLI as a subprocess, because the rule lives
    in main() and an in-process call would test nothing."""

    def test_a_ZERO_MOVE_run_asked_for_a_ceiling_still_ships_it(self):
        """#769. flat_hierarchy has no BGA, so the pass legitimately moves
        nothing and takes the unchanged-copy path -- which on main skipped the
        writeback entirely and shipped the INPUT's spec for the next step to
        grade against.

        MUTATION: drop the `_write_drc_floors()` call from the copy branch --
        battery row `769-copy-branch-unwritten` (KILLED, 2 assertions: this one
        and the custody arm, which is the pairing that makes #769 checkable)."""
        with tempfile.TemporaryDirectory() as td:
            src = _stage(td, FLAT, 'zin', classes=_classes(FLAT_DEFAULT,
                                                           FLAT_WIDE))
            dst = os.path.join(td, 'zout.kicad_pcb')
            out = _run_cli(src, dst, '--clearance', 0.1)
            self.assertIn('unchanged copy', out,
                          'this board moved a cap; it is no longer the '
                          'zero-move witness this arm needs')
            got = _pro(dst)
            self.assertIsNotNone(got, 'the no-op path lost the project')
            self.assertAlmostEqual(got['classes']['Default'], 0.1, places=9)
            self.assertAlmostEqual(got['classes']['Wide'], 0.1, places=9,
                                   msg='a ceiling caps EVERY class')

    def test_an_OMITTED_run_preserves_every_class(self):
        """The other branch of the same rule. On main a bare run with no flag
        at all clamped Wide 0.4 -> 0.25, because argparse supplied 0.25 and
        clamp_nondefault_netclasses defaults True.

        MUTATION: `clamp_nondefault_netclasses=args.clearance is not None` ->
        `True` -- battery row `omitted-clamps-anyway` (KILLED, 1 assertion)."""
        with tempfile.TemporaryDirectory() as td:
            src = _stage(td, FLAT, 'oin', classes=_classes(FLAT_DEFAULT,
                                                           FLAT_WIDE))
            dst = os.path.join(td, 'oout.kicad_pcb')
            _run_cli(src, dst)
            got = _pro(dst)
            self.assertAlmostEqual(got['classes']['Default'], FLAT_DEFAULT,
                                   places=9)
            self.assertAlmostEqual(got['classes']['Wide'], FLAT_WIDE, places=9,
                                   msg='an OMITTED --clearance must PRESERVE '
                                       'the classes (CLAUDE.md); this is the '
                                       'defect that fired with no flag given')

    def test_the_custody_half_still_runs_when_the_flag_is_OMITTED(self):
        """An earlier draft of this change skipped fix_project_for_output
        outright when --clearance was omitted. Measured, that call also writes
        the board's own size minima and `fab_floor_origin` -- 16 values on
        glasgow_revC with no flag at all -- so skipping it throws the #441
        custody half away.

        MUTATION: `return` early when args.clearance is None -> red here,
        green everywhere else. That is the whole point of this arm."""
        with tempfile.TemporaryDirectory() as td:
            src = _stage(td, FLAT, 'cin', classes=_classes(FLAT_DEFAULT))
            dst = os.path.join(td, 'cout.kicad_pcb')
            out = _run_cli(src, dst)
            self.assertIn('DRC settings', out,
                          'the writeback did not run at all on the omitted '
                          'branch; the custody half is gone')
            doc = json.load(open(os.path.splitext(dst)[0] + '.kicad_pro',
                                 encoding='utf-8'))
            self.assertIn('fab_floor_origin',
                          doc.get('kicad_routing_tools', {}),
                          'the fab-floor provenance key was not written')

    def test_min_hole_clearance_is_not_redefined_by_the_copper_ceiling(self):
        """#768's defect one key over. `compute_targets` defaults the
        copper-to-hole target to the copper clearance, so the same glasgow run
        that PRINTED "Copper-to-hole clearance 0.25mm (from the board's own
        min_hole_clearance)" then wrote 0.25 -> 0.1 into the project.

        --clearance is a copper-clearance ceiling. It says nothing about
        copper-to-hole, and this pass does not price that from it.

        MUTATION: drop the `hole_clearance=` argument -> battery row
        `hole-clearance-rides-the-ceiling` (KILLED, 1 assertion -- this one)."""
        with tempfile.TemporaryDirectory() as td:
            src = _stage(td, FLAT, 'hin', classes=_classes(FLAT_DEFAULT),
                         hole_clearance=0.25)
            dst = os.path.join(td, 'hout.kicad_pcb')
            _run_cli(src, dst, '--clearance', 0.1)
            self.assertAlmostEqual(_pro(dst)['min_hole_clearance'], 0.25,
                                   places=9,
                                   msg='the copper ceiling redefined the '
                                       'copper-to-hole floor')

    def test_a_class_BETWEEN_the_default_and_the_ceiling_ships_at_the_ceiling(self):
        """The writeback clamps to the CEILING, not to the resolved base.

        This is the one case the two differ, and my battery could not see it:
        `_priced` is `min(Default, ceiling)`, so on a board whose Default sits
        BELOW the ceiling, a class BETWEEN them is priced at the ceiling and was
        shipped at the Default. Measured before the fix, Default 0.2 / Wide 0.4
        / `--clearance 0.3`: 10 pairs priced at 0.3, project shipped Wide 0.2.
        #768's own shape in the safe direction, and still #768's shape. Found by
        an adversarial review, not by the battery, whose row for it SURVIVED
        because every board it could reach has Default >= ceiling.

        The writeback is lower-only, so passing the ceiling leaves each class at
        min(its own, ceiling): Default 0.2 stays, Wide 0.4 becomes 0.3.

        MUTATION: `_target = args.clearance if ... else _priced` -> `_priced`
        -- battery row `writeback-clamps-to-the-base-not-the-ceiling`."""
        with tempfile.TemporaryDirectory() as td:
            src = _stage(td, FLAT, 'bin', classes=_classes(FLAT_DEFAULT,
                                                           FLAT_WIDE))
            dst = os.path.join(td, 'bout.kicad_pcb')
            _run_cli(src, dst, '--clearance', 0.3)
            got = _pro(dst)['classes']
            self.assertAlmostEqual(got['Wide'], 0.3, places=9,
                                   msg='a class between the Default and the '
                                       'ceiling must ship AT the ceiling')
            self.assertAlmostEqual(got['Default'], FLAT_DEFAULT, places=9,
                                   msg='and one below it must not be raised')

    def test_min_hole_clearance_is_not_INVENTED_from_the_ceiling_either(self):
        """The board declares none, so `compute_targets` defaults the
        copper-to-hole target to the copper clearance and `apply_targets`
        CREATES the rule. Measured before the fix: a board declaring no
        `min_hole_clearance`, run at `--clearance 0.1`, shipped
        `min_hole_clearance: 0.1` while the pass priced that geometry at the
        0.2 NPTH floor.

        So the value written is the one the pass USED, board-declared or not.

        MUTATION: drop the `if _hc is None` fallback -> battery row
        `hole-clearance-invented-from-the-ceiling`."""
        with tempfile.TemporaryDirectory() as td:
            src = _stage(td, FLAT, 'nhc', classes=_classes(FLAT_DEFAULT))
            pro = os.path.splitext(src)[0] + '.kicad_pro'
            doc = json.load(open(pro, encoding='utf-8'))
            doc['board']['design_settings']['rules'].pop('min_hole_clearance',
                                                         None)
            json.dump(doc, open(pro, 'w', encoding='utf-8'), indent=2)
            dst = os.path.join(td, 'nhcout.kicad_pcb')
            _run_cli(src, dst, '--clearance', 0.1)
            got = _pro(dst)['min_hole_clearance']
            self.assertIsNotNone(got)
            self.assertGreater(got, 0.1,
                               'the copper ceiling was written in as the '
                               'copper-to-hole rule on a board that declares '
                               'none; got %r' % (got,))

    def test_a_non_finite_clearance_is_REFUSED(self):
        """`type=float` accepts nan and inf, and `min(v, nan)` is `v` -- so a
        nan ceiling disables the cap while the writeback still clamps, which
        reproduces #768 through the code that fixes it.

        MUTATION: drop the isfinite guard -> battery row `nan-ceiling-accepted`."""
        with tempfile.TemporaryDirectory() as td:
            src = _stage(td, FLAT, 'nan')
            dst = os.path.join(td, 'nanout.kicad_pcb')
            r = run_utils.check(
                [sys.executable, '-X', 'utf8', CLI, src, dst,
                 '--clearance', 'nan'],
                refuse='--clearance must be a finite number',
                allow=('error: argument',))
            self.assertNotEqual(r.returncode, 0)

    def test_a_run_that_MOVED_something_agrees_with_what_it_priced(self):
        """End to end on the one tracked board that actually moves caps, with
        a project this file staged. The claim is AGREEMENT, not a smaller
        number: the value the transcript says it priced at is the value the
        project it ships declares."""
        if not os.path.exists(MOVER):
            self.skipTest('orangecrab fixture not present')
        with tempfile.TemporaryDirectory() as td:
            src = _stage(td, MOVER, 'min', classes=_classes(FLAT_DEFAULT,
                                                            FLAT_WIDE))
            dst = os.path.join(td, 'mout.kicad_pcb')
            out = _run_cli(src, dst, '--clearance', 0.1, timeout=1800)
            self.assertNotIn('unchanged copy', out,
                             'orangecrab stopped moving caps; this arm needs '
                             'the writeback branch')
            m = re.search(r'cap pair clearance: ([0-9.]+)mm', out)
            self.assertIsNotNone(m, out[-1500:])
            priced = float(m.group(1))
            got = _pro(dst)
            self.assertAlmostEqual(priced, got['classes']['Default'], places=9,
                                   msg='priced at %s, ships %s -- the #768 '
                                       'inversion' % (priced,
                                                      got['classes']['Default']))
            self.assertEqual(
                [r for r in re.findall(r'requires ([0-9.]+)mm \(netclass\)', out)],
                [], 'a pair was still charged ABOVE the ceiling by the '
                    'netclass tier, which the ceiling is supposed to cap')


# ---------------------------------------------------------------------------
class TestTheGUICarriesTheSameSwitch(unittest.TestCase):
    """wx-free, and DELIBERATELY LIMITED to the two things source text can
    actually decide. The behaviour lives in
    `tests/gui_parity/test_768_cap_ceiling_real_dialog.py`, which drives the
    real headless dialog and captures the kwargs the engine is handed.

    That split is not tidiness. The first version of this class was two string
    counts, and both were TRUE of a gate that was wrong (it read
    `fix_drc_settings`, a box that clamps no net class at all) and, on the
    standalone call path, inert (that key is not in the config that path
    builds). A parity review found it by measuring the value; nothing textual
    could have."""

    GUI = os.path.join(_ROOT, 'kicad_routing_plugin', 'fanout_gui.py')
    SWIG = os.path.join(_ROOT, 'kicad_routing_plugin', 'routing_dialog.py')

    def _src(self, path):
        with open(path, encoding='utf-8') as fh:
            return '\n'.join(l.split('#')[0] for l in fh.read().splitlines())

    def test_the_cap_step_passes_a_ceiling_gated_on_the_override(self):
        """The gate must be `clamp_netclasses` -- the Min Clearance override,
        which is the GUI's counterpart of "--clearance was GIVEN" everywhere
        else in this dialog -- and must default to NO ceiling when the key is
        absent, because an absent key means the operator never ticked it.

        MUTATION: swap the key for `fix_drc_settings` -> this fails, and so do
        4 checks in the real-dialog gate."""
        src = self._src(self.GUI)
        m = re.search(r'netclass_ceiling=([^\n]*)\n', src)
        self.assertIsNotNone(m, 'the GUI cap step passes no ceiling at all, so '
                                'it still prices with max() while clamping '
                                'with min() -- the #768 defect, GUI side')
        gate = m.group(1)
        self.assertIn("clearance_ceiling", gate,
                      'the ceiling must be the RAW override the tab exports, '
                      'not a value already resolved to min(Default, it)')
        self.assertNotIn("fix_drc_settings", gate,
                         'that box writes m_MinClearance and the Default class '
                         'only; it clamps no net class, so it cannot be the '
                         'ceiling switch')
        self.assertNotIn("BGA_CLEARANCE", gate,
                         'an absent ceiling must be None, not a packaged '
                         'default: presence IS the switch')

    def test_the_fanout_tab_exports_the_override_at_all(self):
        """It was the one step tab whose shared params did not carry it, so the
        gate above had nothing to read on either call path."""
        src = self._src(self.SWIG)
        self.assertIn("'clamp_netclasses': self.clearance_check.GetValue(),",
                      src)
        self.assertIn("'clearance_ceiling': (self.clearance.GetValue()", src,
                      'the ceiling must be the RAW spin value; '
                      '_effective_clearance() is already min(Default, it)')

    def test_the_standalone_path_carries_it_into_its_own_config(self):
        """`run_cap_optimization` builds a config from a handful of shared keys
        rather than passing them through, so a key it does not list reads as
        its `.get` default however correct the gate is. That is how the first
        cut of this change was inert on the plan-executor path while looking
        right inline -- the #693 shape the parity ledger records.

        THE INDENT IS PART OF THE ASSERTION (#780). This arm used to strip
        it, and when the INLINE config gained the same two lines one block
        deeper, the inline copy satisfied an arm named for the standalone
        path -- deleting the standalone pair outright left it green. A
        source grep that cannot tell two call sites apart is not a guard on
        either of them.
        """
        src = self._src(self.GUI)
        for key in ("\n            'clamp_netclasses': "
                    "shared.get('clamp_netclasses', False),",
                    "\n            'clearance_ceiling': "
                    "shared.get('clearance_ceiling'),"):
            self.assertIn(key, src,
                          'the STANDALONE cfg (run_cap_optimization, one '
                          'indent shallower than the inline one) must '
                          'carry it')

    def test_the_inline_path_carries_it_too(self):
        """#780: `_run_bga_fanout` builds its own dict as well, and did not.

        Named separately from the arm above rather than folded into it,
        because one assertion covering two call sites is what let the
        standalone one go dark. Neither of these can see a WRONG value --
        that is the real-dialog gate's job, and both call sites are driven
        there now."""
        src = self._src(self.GUI)
        self.assertIn("\n                'clearance_ceiling': "
                      "shared.get('clearance_ceiling'),", src,
                      'the INLINE cap config must carry the ceiling; '
                      'without it the checkbox path runs the OMITTED '
                      'branch whatever the operator ticked')

    def test_the_real_dialog_gate_exists_and_is_registered(self):
        """A behavioural claim this file cannot make must be made SOMEWHERE."""
        gate = os.path.join(_ROOT, 'tests', 'gui_parity',
                            'test_768_cap_ceiling_real_dialog.py')
        self.assertTrue(os.path.isfile(gate),
                        'the wx-free half above cannot detect a wrong gate; '
                        'the real-dialog half is not optional')


# ---------------------------------------------------------------------------
class TestTheManifestCarriesTheFlag(unittest.TestCase):
    """A recorded `place_fanout_clearance.py --clearance 0.1` used to lose its
    clearance converting to a GUI plan -- on the one parameter whose presence
    decides whether the step clamps the project at all."""

    def test_the_flag_survives_conversion(self):
        sys.path.insert(0, os.path.join(_ROOT, 'tests', 'stress'))
        import manifest_to_plan as M
        step = M.cap_optimization_step(
            ['py_placer/place_fanout_clearance.py', 'b.kicad_pcb',
             '--clearance', '0.1', '--near-margin', '1.5'])
        self.assertEqual(step['params'].get('clearance'), 0.1)
        self.assertEqual(step['params'].get('cap_near_margin'), 1.5)

    def test_an_OMITTED_flag_is_not_materialised(self):
        """The negative control that matters most here: a plan carrying an
        invented `clearance` would turn every omitted-branch run into a
        given-branch one, silently, on the GUI side only."""
        sys.path.insert(0, os.path.join(_ROOT, 'tests', 'stress'))
        import manifest_to_plan as M
        step = M.cap_optimization_step(
            ['py_placer/place_fanout_clearance.py', 'b.kicad_pcb',
             '--near-margin', '1.5'])
        self.assertIsNone(step['params'].get('clearance'))


# ---------------------------------------------------------------------------
class TestInertOnTheTrackedCorpus(unittest.TestCase):
    """The self-expiring bound. 20 of the 22 tracked boards carry no sibling
    project at all, so there is no class to cap and no project to preserve, and
    every branch above resolves to the packaged default.

    A "0 diffs on the corpus" run therefore proves NOTHING on its own, which is
    why this class asserts the REASON as well as the outcome."""

    def setUp(self):
        self.boards = run_utils.corpus_boards()
        if not self.boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        self.assertGreaterEqual(len(self.boards), 22,
                                'the tracked corpus collapsed to %d boards; '
                                'nothing below is a bound' % len(self.boards))

    def test_exactly_two_tracked_boards_can_declare_anything(self):
        withpro = sorted(os.path.basename(b) for b in self.boards
                         if os.path.exists(os.path.splitext(b)[0] + '.kicad_pro'))
        self.assertEqual(
            withpro, ['flat_hierarchy.kicad_pcb', 'routed_output.kicad_pcb'],
            'the set of tracked boards carrying a project has CHANGED: %r. '
            'The inertness claim in the #768 PR has EXPIRED -- re-run the '
            'four-arm A/B and record the new numbers' % withpro)

    def test_every_project_less_board_resolves_to_the_packaged_default(self):
        odd = []
        for b in self.boards:
            if os.path.exists(os.path.splitext(b)[0] + '.kicad_pro'):
                continue
            got = resolve_pair_clearance(b, None)
            if got != (defaults.CLEARANCE, 'fixed default'):
                odd.append((os.path.basename(b), got))
        self.assertEqual(odd, [], odd)

    def test_the_two_declaring_boards_move_and_in_which_direction(self):
        """NOT inert on these two, and the PR must say so rather than let a
        reviewer find it. Both move to the board's OWN declaration, which is
        what KiCad grades them at."""
        got = {}
        for b in self.boards:
            if not os.path.exists(os.path.splitext(b)[0] + '.kicad_pro'):
                continue
            got[os.path.basename(b)] = resolve_pair_clearance(b, None)
        self.assertEqual(got, {
            'flat_hierarchy.kicad_pcb': (0.2, 'board netclass'),
            'routed_output.kicad_pcb': (0.09, 'board netclass'),
        }, 'the declaring boards moved: %r' % got)


if __name__ == '__main__':
    unittest.main(verbosity=2)
