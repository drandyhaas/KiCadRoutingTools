#!/usr/bin/env python3
"""#782: the fanout tab prices its cap pass at the #768 ceiling and must clamp
the NON-Default net classes to match.

    python3 tests/test_782_nondefault_netclass_clamp.py

THE DEFECT. #768 gave the decoupling-cap pass a netclass CEILING with the CLI's
contract: GIVEN `--clearance`, every class is priced at min(class, it) AND the
output project is clamped down to it. The GUI fanout tab implemented the pricing
half and not the writeback half -- it finishes with
`gui_utils.update_live_drc_floors`, which touches `m_MinClearance` and the
DEFAULT class only, and it never called `apply_targets_to_board`. So a Wide-class
pair was priced at min(0.4, 0.2) = 0.2 and then graded by KiCad at the still-0.4
class: violations on copper the pass considered legal.

THE SHAPE OF THE FIX, and why it is not a fourth copy. The live-board clamp
already existed, inline inside `apply_targets_to_board`, which is what the
signal / differential / planes tabs reach. It is now
`fix_kicad_drc_settings.clamp_nondefault_netclasses_on_board`, called from that
function and from `update_live_drc_floors`. One spelling, two callers -- the same
move #736 / #747 / #775 each made once in the placement engine, for the same
reason: a rule that lives in two places is kept in step by hand until it isn't.

WHAT THIS FILE CAN AND CANNOT PROVE. Everything here runs WITHOUT pcbnew, so it
grades the helper's semantics against fake net-class objects, plus the two
structural claims (one spelling; both call sites pass the ceiling). It CANNOT
prove the real pcbnew netclass enumeration works, because that API varies across
KiCad versions and is exactly the part a fake cannot stand in for -- and it
cannot prove `update_live_drc_floors` reaches the clamp at all, since that body
bails at its own `import pcbnew` with no KiCad python. Both are the job of
`tests/gui_parity/test_782_fanout_netclass_clamp.py`, which drives a real board.
This file is deliberately not the evidence for the fix; it is the fast half.
"""
import ast
import os
import sys
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, REPO)

from fix_kicad_drc_settings import clamp_nondefault_netclasses_on_board  # noqa: E402

MM = 1e6


class FakeNC:
    """Minimal net class: the four setters the clamp knows, in internal nm."""

    def __init__(self, clearance_mm=None, dp_gap_mm=None, dp_width_mm=None):
        self._c = None if clearance_mm is None else round(clearance_mm * MM)
        self._g = None if dp_gap_mm is None else round(dp_gap_mm * MM)
        self._w = None if dp_width_mm is None else round(dp_width_mm * MM)

    def GetClearance(self):
        return self._c

    def SetClearance(self, v):
        self._c = v

    def GetDiffPairGap(self):
        return self._g

    def SetDiffPairGap(self, v):
        self._g = v

    def GetDiffPairWidth(self):
        return self._w

    def SetDiffPairWidth(self, v):
        self._w = v

    @property
    def clearance_mm(self):
        return None if self._c is None else self._c / MM


class FakeNetSettings:
    def __init__(self, default_nc, classes):
        self._d = default_nc
        self._all = classes

    def GetDefaultNetclass(self):
        return self._d

    def GetNetclasses(self):
        return self._all


class FakeBDS:
    def __init__(self, ns):
        self.m_NetSettings = ns


class FakeBoard:
    def __init__(self, bds):
        self._bds = bds

    def GetDesignSettings(self):
        return self._bds


def board_with(default_mm=0.2, others=None):
    """A board whose enumeration returns the Default class TOO, like pcbnew's."""
    d = FakeNC(default_mm)
    classes = {'Default': d}
    made = {}
    for name, mm in (others or {}).items():
        made[name] = FakeNC(mm)
        classes[name] = made[name]
    return FakeBoard(FakeBDS(FakeNetSettings(d, classes))), d, made


class TestTheClampItself(unittest.TestCase):

    def test_a_non_default_class_ABOVE_the_ceiling_is_lowered_to_it(self):
        """The headline: Wide 0.4, ceiling 0.3 -> Wide 0.3."""
        b, _d, o = board_with(0.2, {'Wide': 0.4})
        ch = clamp_nondefault_netclasses_on_board(b, {'min_clearance': 0.3})
        self.assertEqual(o['Wide'].clearance_mm, 0.3)
        self.assertEqual(len(ch), 1)
        self.assertIn('Wide', ch[0])

    def test_the_DEFAULT_class_is_never_touched(self):
        """It is in the enumeration -- skipped by identity AND by name.

        This is the arm that matters for the fanout tab: `update_live_drc_floors`
        owns the Default class (it min-merges it against the board's actual
        minima), and a clamp that also wrote it would be a second, disagreeing
        writer of one value.
        """
        b, d, _o = board_with(0.2, {'Wide': 0.4})
        clamp_nondefault_netclasses_on_board(b, {'min_clearance': 0.05})
        self.assertEqual(d.clearance_mm, 0.2)

    def test_a_class_ALREADY_BELOW_the_ceiling_is_left_alone(self):
        """Only-loosen. A tighter class survives a looser ceiling, which is the
        half of #439 that keeps a genuine impedance spec intact."""
        b, _d, o = board_with(0.2, {'Tight': 0.1})
        ch = clamp_nondefault_netclasses_on_board(b, {'min_clearance': 0.3})
        self.assertEqual(o['Tight'].clearance_mm, 0.1)
        self.assertEqual(ch, [])

    def test_a_class_EQUAL_to_the_ceiling_reports_no_change(self):
        b, _d, o = board_with(0.2, {'Same': 0.3})
        ch = clamp_nondefault_netclasses_on_board(b, {'min_clearance': 0.3})
        self.assertEqual(o['Same'].clearance_mm, 0.3)
        self.assertEqual(ch, [])

    def test_no_target_is_a_no_op(self):
        """`min_clearance` absent means the caller had no ceiling -- #768's
        OMITTED branch. Nothing is written, rather than a class going to None."""
        b, _d, o = board_with(0.2, {'Wide': 0.4})
        self.assertEqual(clamp_nondefault_netclasses_on_board(b, {}), [])
        self.assertEqual(o['Wide'].clearance_mm, 0.4)

    def test_every_non_default_class_is_visited_not_just_the_first(self):
        b, _d, o = board_with(0.2, {'Wide': 0.4, 'Wider': 0.5, 'Tight': 0.1})
        ch = clamp_nondefault_netclasses_on_board(b, {'min_clearance': 0.3})
        self.assertEqual(o['Wide'].clearance_mm, 0.3)
        self.assertEqual(o['Wider'].clearance_mm, 0.3)
        self.assertEqual(o['Tight'].clearance_mm, 0.1)   # untouched
        self.assertEqual(len(ch), 2)

    def test_diff_pair_draw_defaults_are_NOT_lowered_even_when_given(self):
        """#842: diff_pair_gap / diff_pair_width are draw defaults (KiCad loads
        them SetOpt). The kwargs survive for signature compatibility and do
        nothing; lowering them was the same ratchet as track_width."""
        b, _d, o = board_with(0.2, {'Wide': 0.4})
        o['Wide']._g = round(0.4 * MM)
        o['Wide']._w = round(0.4 * MM)
        clamp_nondefault_netclasses_on_board(
            b, {'min_clearance': 0.3}, diff_pair_gap=0.15, diff_pair_width=0.12)
        self.assertEqual(o['Wide']._g, round(0.4 * MM))
        self.assertEqual(o['Wide']._w, round(0.4 * MM))

    def test_track_and_via_geometry_is_NOT_lowered(self):
        """Parity with the CLI's _NONDEFAULT_CLAMP_FIELDS. Only `clearance` is
        DRC-enforced per class; the width/via setters are DRAW defaults, and
        lowering them once overwrote a board's declared per-class geometry with a
        local escape's stub width."""
        b, _d, o = board_with(0.2, {'Wide': 0.4})
        calls = []
        for bad in ('SetTrackWidth', 'SetViaDiameter', 'SetViaDrill'):
            setattr(o['Wide'], bad,
                    (lambda n: (lambda v: calls.append(n)))(bad))
        clamp_nondefault_netclasses_on_board(b, {'min_clearance': 0.3})
        self.assertEqual(calls, [])

    def test_an_unreadable_board_no_ops_rather_than_raising(self):
        """Best-effort: this runs after the pass has already placed its copper,
        so an unknown pcbnew shape must not raise into it."""
        class Broken:
            def GetDesignSettings(self):
                raise RuntimeError('no settings on this build')
        self.assertEqual(
            clamp_nondefault_netclasses_on_board(Broken(), {'min_clearance': 0.3}),
            [])

    def test_a_board_with_NO_non_default_class_is_a_no_op(self):
        """Most boards. Nothing to clamp, and no spurious change line."""
        b, d, _o = board_with(0.2, {})
        self.assertEqual(
            clamp_nondefault_netclasses_on_board(b, {'min_clearance': 0.3}), [])
        self.assertEqual(d.clearance_mm, 0.2)

    def test_the_default_class_is_resolved_when_the_caller_passes_none(self):
        """The fanout tab does not resolve it -- that is why the parameter is
        optional. Without this the Default class would be clamped like any
        other, since the enumeration hands it back under the same map."""
        b, d, o = board_with(0.2, {'Wide': 0.4})
        clamp_nondefault_netclasses_on_board(
            b, {'min_clearance': 0.3}, default_nc=None)
        self.assertEqual(d.clearance_mm, 0.2)
        self.assertEqual(o['Wide'].clearance_mm, 0.3)


class TestOneSpelling(unittest.TestCase):
    """The structural half: nobody re-implements the clamp."""

    def test_apply_targets_to_board_DELEGATES(self):
        src = open(os.path.join(REPO, 'py_router',
                                'fix_kicad_drc_settings.py')).read()
        fn = _func_src(src, 'apply_targets_to_board')
        self.assertIn('clamp_nondefault_netclasses_on_board(', fn,
                      'apply_targets_to_board must call the shared clamp')
        # `nd_map` is the removed block's own local, and it is the right
        # marker precisely because the obvious one is NOT: `SetDiffPairViaGap`
        # also appears in this function's legitimate DEFAULT-class `nc_map`, so
        # a guard on it fails on correct code. (It did, on the first run.)
        self.assertNotIn('nd_map', fn,
                         'the inline clamp body is still in '
                         'apply_targets_to_board -- two spellings again')

    def test_the_clamp_body_exists_exactly_once_in_the_repo(self):
        """`nd_map` is the body's own local. If it appears in a second file,
        someone has copied the clamp rather than called it."""
        hits = []
        for root, _dirs, files in os.walk(REPO):
            # Test the path RELATIVE to the repo: an absolute path that
            # contains '.claude' (a worktree under .claude/worktrees/) made
            # this skip every file and report the body missing.
            rel_root = os.path.relpath(root, REPO)
            if any(p in rel_root for p in ('.git', 'node_modules', '.claude')):
                continue
            for f in files:
                if not f.endswith('.py'):
                    continue
                p = os.path.join(root, f)
                try:
                    txt = open(p, encoding='utf-8', errors='replace').read()
                except OSError:
                    continue
                if 'nd_map' in txt and 'tests' not in p:
                    # POSIX separators: `relpath` returns the HOST's, so on
                    # Windows this compared 'py_router\fix_...' against the
                    # forward-slash literal below and failed for a reason that
                    # has nothing to do with where the clamp body lives.
                    hits.append(os.path.relpath(p, REPO).replace(os.sep, '/'))
        self.assertEqual(
            hits, ['py_router/fix_kicad_drc_settings.py'],
            f'the non-Default clamp body should live in one file, found {hits}')


class TestBothInteractiveFanoutPathsPassTheCeiling(unittest.TestCase):
    """The wiring. A source guard, and it is the WEAK half on purpose -- the
    real evidence is the gui_parity gate. It is here so a refactor that drops
    one of the two call sites fails something fast."""

    def setUp(self):
        self.src = open(os.path.join(REPO, 'kicad_routing_plugin',
                                     'fanout_gui.py')).read()

    def test_the_inline_path_passes_the_CEILING_to_the_floor_update(self):
        fn = _func_src(self.src, '_apply_fanout_results')
        self.assertIn('update_live_drc_floors(', fn)
        self.assertIn("nondefault_clamp_mm=_fcfg.get('clearance_ceiling')", fn,
                      'the inline path must clamp to the ceiling')

    def test_the_standalone_path_clamps_too(self):
        fn = _func_src(self.src, 'run_cap_optimization')
        self.assertIn('clamp_nondefault_netclasses_on_board(', fn,
                      'the standalone cap button must clamp as well -- which '
                      'button was pressed cannot decide what the board ships')
        self.assertIn("cfg.get('clearance_ceiling')", fn)

    def test_neither_path_clamps_to_the_EFFECTIVE_clearance(self):
        """The trap #768 named: `clearance` is min(Default, override), so
        clamping to it ships a class BELOW the value the pass priced at whenever
        the board's Default sits under the operator's ceiling."""
        for name in ('_apply_fanout_results', 'run_cap_optimization'):
            fn = _func_src(self.src, name)
            self.assertNotIn("nondefault_clamp_mm=_fcfg.get('clearance')", fn)
            self.assertNotIn("{'min_clearance': float(cfg['clearance'])}", fn)


def _func_src(src, name):
    """The source text of one top-level-or-method function, by AST span."""
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) \
                and node.name == name:
            return ast.get_source_segment(src, node) or ''
    raise AssertionError(f'{name} not found -- renamed? this guard is stale')


if __name__ == '__main__':
    unittest.main(verbosity=2)
