#!/usr/bin/env python3
"""#697: the placement pad census must grade at the pair's REAL requirement.

`py_placer/placement/legality.py` priced every pad pair at one flat `clearance`
scalar and never read `pad.local_clearance`, so a board that could not pass DRC
reported nothing to fix. Measured on `kicad_files/esp_prog.kicad_pcb`, whose
fiducial pad declares a 1.016 mm keep-clear: nudged 0.40 mm toward USB1 the pads
sit 0.9355 mm apart -- `check_drc` flags it, and `grade_pad_legality` /
`place_reconstruct --stages legalize` both reported 0 conflict pairs.

The requirement is check_drc's, over three stacked sources::

    base = max(global clearance, netclass(a), netclass(b))
    eff  = <.kicad_dru layer rules over the SHARED copper layers>   # REPLACES
    eff  = max(eff, lc_a, lc_b)                                     # override wins

Every test below names the single-line reversion of the fix that must make it
fail -- a test that still passes with the bug present is worth nothing.
"""
import math
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))   # placement split

from placement.legality import (                                   # noqa: E402
    LegalityContext, PadClearanceModel, build_part_pads,
    format_required_clause, grade_pad_legality)

ESP_PROG = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FLAT_HIER = os.path.join(ROOT, 'kicad_files', 'flat_hierarchy.kicad_pcb')


# --------------------------------------------------------------------------
# Synthetic geometry. REAL parser dataclasses via tests/synth.py, not duck-typed
# fakes: `grade_pad_legality`'s exact re-verification hands the pads to
# check_drc's perimeter sampler, which reads fields (pad_number, shape,
# polygons, ...) a minimal fake does not carry -- and a fake that silently
# skipped the exact path would test the AABB fallback while claiming to test
# the census. `BarePad` below is the one deliberate exception.
# --------------------------------------------------------------------------
from kicad_parser import BoardInfo, Footprint                 # noqa: E402
from synth import make_pad, make_pcb                          # noqa: E402


class BarePad:
    """A pad object from BEFORE #697: no `local_clearance` attribute at all.
    Pins that the model reads the field with getattr, not a bare attribute."""

    def __init__(self):
        self.global_x = 0.0
        self.global_y = 0.0
        self.size_x = 1.0
        self.size_y = 1.0
        self.net_id = 1
        self.layers = ['F.Cu']
        self.drill = 0.0
        self.pad_type = 'smd'
        self.shape = 'rect'
        self.pad_number = '1'


def pad(x, y, *, net=1, lc=0.0, layers=('F.Cu',), ref='A', **kw):
    p = make_pad(net_id=net, x=x, y=y, ref=ref, size_x=1.0, size_y=1.0,
                 layers=layers, **kw)
    p.local_clearance = lc
    return p


def fp_with(ref, x, y, pads):
    return Footprint(reference=ref, footprint_name='test:P', x=x, y=y,
                     rotation=0.0, layer='F.Cu', pads=list(pads))


def board(fps, copper=('F.Cu', 'B.Cu')):
    info = BoardInfo(layers={i: l for i, l in enumerate(copper)},
                     copper_layers=list(copper), stackup=[], board_outline=[],
                     board_cutouts=[], board_outlines=[],
                     board_edge_contours=[])
    return make_pcb(footprints={f.reference: f for f in fps}, board_info=info)


def two_pads(gap, lc_a=0.0, lc_b=0.0, **kw):
    """Two 1x1 mm pads on different nets, `gap` mm apart edge to edge."""
    a = fp_with('A', 0.0, 0.0, [pad(0.0, 0.0, net=1, lc=lc_a, ref='A', **kw)])
    b = fp_with('B', 1.0 + gap, 0.0,
                [pad(1.0 + gap, 0.0, net=2, lc=lc_b, ref='B', **kw)])
    return board([a, b])


class TestPairRequirement(unittest.TestCase):
    """The formula itself, in isolation."""

    def test_override_on_either_pad_raises_the_pair(self):
        # check_drc needs a SECOND max at its call site because _pad_pair_cl
        # folds only pad1's override; a copy that keeps one max reports nothing
        # when only the second pad carries the keep-clear.
        m = PadClearanceModel(0.15, has_overrides=True)
        fa = m.pad_floor(pad(0, 0, lc=1.016))
        fb = m.pad_floor(pad(5, 0))
        # MUTATION: drop `fa.lc` from pair_with_source -> first assert fails.
        self.assertAlmostEqual(m.pair(fa, fb), 1.016)
        # MUTATION: drop `fb.lc` -> this one fails.
        self.assertAlmostEqual(m.pair(fb, fa), 1.016)

    def test_source_is_recorded_where_it_is_applied(self):
        # A layer rule REPLACES, so it can LOWER the value below the netclass
        # that preceded it. Re-deriving the source from the final number then
        # says "netclass" for a value the RULE set.
        m = PadClearanceModel(0.15, net_floor={7: 0.5},
                              layer_rules={'F.Cu': 0.3},
                              board_copper=('F.Cu', 'B.Cu'))
        fa = m.pad_floor(pad(0, 0, net=7, layers=('F.Cu',)))
        fb = m.pad_floor(pad(5, 0, net=8, layers=('F.Cu',)))
        eff, src = m.pair_with_source(fa, fb)
        self.assertAlmostEqual(eff, 0.3)
        # MUTATION: restore the old `pair_source` re-derivation -> 'netclass'.
        self.assertEqual(src, 'layer rule')

    def test_layer_rule_scopes_to_SHARED_layers_and_replaces(self):
        # An inner-layer rule must not touch a pair that only meets on F.Cu.
        m = PadClearanceModel(0.2, layer_rules={'In1.Cu': 0.5},
                              board_copper=('F.Cu', 'In1.Cu', 'B.Cu'))
        th = m.pad_floor(pad(0, 0, layers=('*.Cu',), drill=0.3, pad_type='thru_hole'))
        smd = m.pad_floor(pad(5, 0, layers=('F.Cu',)))
        # MUTATION: fold the dru per pad instead of over the shared set
        # (max over each pad's own layers) -> 0.5, a false positive.
        self.assertAlmostEqual(m.pair(th, smd), 0.2)
        # Both reach In1 -> the rule binds, and REPLACES rather than maxes.
        th2 = m.pad_floor(pad(5, 0, layers=('*.Cu',), drill=0.3, pad_type='thru_hole'))
        self.assertAlmostEqual(m.pair(th, th2), 0.5)

    def test_relaxing_rule_replaces_downward(self):
        m = PadClearanceModel(0.2, layer_rules={'F.Cu': 0.05},
                              board_copper=('F.Cu', 'B.Cu'))
        fa = m.pad_floor(pad(0, 0, layers=('F.Cu',)))
        fb = m.pad_floor(pad(5, 0, layers=('F.Cu',)))
        # MUTATION: implement _pads_cl as max(base, rules) -> 0.2, and the
        # relaxation is silently lost.
        self.assertAlmostEqual(m.pair(fa, fb), 0.05)

    def test_bare_pad_without_the_attribute_is_not_a_crash(self):
        m = PadClearanceModel(0.15, has_overrides=True)
        p = BarePad()
        self.assertFalse(hasattr(p, 'local_clearance'))
        # MUTATION: read `pad.local_clearance` directly -> AttributeError.
        self.assertAlmostEqual(m.pad_floor(p).lc, 0.0)

    def test_model_with_no_sources_is_inert(self):
        m = PadClearanceModel(0.15)
        self.assertFalse(m.active)


class TestCensus(unittest.TestCase):
    """grade_pad_legality: the four points a pair used to be dropped at."""

    def _grade(self, pcb, clearance=0.15):
        return grade_pad_legality(pcb, clearance, worst_n=0)

    def test_conflict_seen_when_pad_A_carries_the_override(self):
        # gap 0.40: clear at the 0.15 floor, in violation of the 1.0 keep-clear
        g = self._grade(two_pads(0.40, lc_a=1.0))
        # MUTATION: revert the census reject to `g >= clearance - EPS` -> 0.
        self.assertEqual(g['pad_conflicts'], 1, g)
        self.assertEqual(g['required'], [['A', 'B', 1.0, 'pad override']])

    def test_conflict_seen_when_pad_B_carries_the_override(self):
        g = self._grade(two_pads(0.40, lc_b=1.0))
        self.assertEqual(g['pad_conflicts'], 1, g)

    def test_beyond_the_old_cell_hash_halo(self):
        """The pair must be reachable through the CELL HASH, not just past the
        per-pair reject.

        `near_refs` walks a 4mm cell grid inflated by its halo, so a gap under
        ~4mm lands in an adjacent cell and is enumerated whatever the halo is --
        a test at 0.9mm exercises the reject and silently claims the halo. The
        separation has to exceed a cell for the halo to be the thing under test.
        """
        # gap 5.0, requirement 6.0: the old halo (0.15 + 0.5) reaches neither
        # the cell nor the pair.
        g = self._grade(two_pads(5.0, lc_a=6.0))
        # MUTATION: revert near_refs to `m = clearance + 0.5` -> 0.
        self.assertEqual(g['pad_conflicts'], 1, g)
        self.assertEqual(g['required'], [['A', 'B', 6.0, 'pad override']])

    def test_inert_when_nothing_is_declared(self):
        clean = self._grade(two_pads(0.40))
        self.assertEqual(clean['pad_conflicts'], 0)
        self.assertEqual(clean['required'], [])
        # and a genuine sub-floor pair is still caught, unchanged
        self.assertEqual(self._grade(two_pads(0.05))['pad_conflicts'], 1)

    def test_worst_stays_a_three_tuple(self):
        # seeder/floorplan/lock_advisor all unpack `worst` positionally as
        # (ref_a, ref_b, mm). The disclosure rides `required`, not a 4th slot.
        g = self._grade(two_pads(0.40, lc_a=1.0))
        for row in g['worst']:
            self.assertEqual(len(row), 3, row)
        ra, rb, mm = g['worst'][0]
        self.assertEqual({ra, rb}, {'A', 'B'})
        self.assertGreater(mm, 0.0)

    def test_required_clause_names_the_source(self):
        g = self._grade(two_pads(0.40, lc_a=1.0))
        self.assertIn('pad override', format_required_clause(g))
        self.assertEqual(format_required_clause(self._grade(two_pads(0.05))), '')


class TestGate(unittest.TestCase):
    """LegalityContext: the per-candidate path quench calls per pose."""

    def _ctx(self, pcb, clearance=0.15):
        model = PadClearanceModel.for_board(pcb, clearance)
        model = model if model.active else None
        parts = build_part_pads(pcb.footprints, clearance, model)
        poses = {r: (f.x, f.y, f.rotation) for r, f in pcb.footprints.items()}
        return LegalityContext(parts, None, clearance,
                               pose_of=lambda r: poses[r],
                               seed_of=lambda r: poses[r],
                               model=model), poses

    def test_extent_short_circuit_no_longer_hides_the_pair(self):
        # The parts' EXTENTS are 0.40 apart -- past the 0.15 floor, so
        # pair_shortfall used to return ZERO_SHORTFALL before any pad was read.
        pcb = two_pads(0.40, lc_a=1.0)
        ctx, _ = self._ctx(pcb)
        sf = ctx.pair_shortfall('A', 'B')
        # MUTATION: revert the reach at the extent gate to self.clearance -> 0.
        self.assertGreater(sf.pad, 0.0)
        self.assertFalse(sf.pad_overlap)   # still no physical intersection
        self.assertFalse(sf.stack)

    def test_seed_that_already_violates_stays_placeable(self):
        # Both `cur` and `base` are measured at the same widened requirement,
        # so a board seeded in violation keeps a matching baseline. Refusing it
        # would make such a board unplaceable rather than repairable.
        pcb = two_pads(0.40, lc_a=1.0)
        ctx, poses = self._ctx(pcb)
        self.assertTrue(ctx.pads_ok('A', *poses['A'], neighbors=['B']))
        # ... but a pose that makes the pair WORSE is refused
        x, y, r = poses['A']
        self.assertFalse(ctx.pads_ok('A', x + 0.2, y, r, neighbors=['B']))
        # ... and one that improves it is admitted
        self.assertTrue(ctx.pads_ok('A', x - 0.2, y, r, neighbors=['B']))

    def test_gate_is_inert_without_declarations(self):
        pcb = two_pads(0.40)
        ctx, _ = self._ctx(pcb)
        self.assertEqual(ctx.pair_shortfall('A', 'B').pad, 0.0)


class TestRealBoards(unittest.TestCase):
    """The reported failure, on the board that reproduces it."""

    def _nudge_fiducial(self, board, mm):
        """Rigidly move the override-carrying footprint toward USB1.

        The candidate is chosen as the one NEAREST USB1, not `[0]`. esp_prog
        carries two `Ref*` fiducial blocks, and until #726 the parser kept only
        the second -- the one 0.61 mm from USB1, which is what makes a 0.40 mm
        nudge cross its 1.016 mm keep-clear. With both blocks present, `[0]`
        picks the OTHER one, 24.05 mm away, where no nudge this small can
        violate anything: the arm then fails on its own honest guard
        ("check_drc sees no violation -- fixture drifted") while the engine is
        fine. Selecting on the property the test actually depends on says what
        the fixture is for, and cannot be re-broken by a dict order.
        """
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(board)
        tgt = pcb.footprints['USB1']
        cands = [fp for fp in pcb.footprints.values()
                 if any((getattr(q, 'local_clearance', 0.0) or 0.0) > 0.5
                        for q in fp.pads)]
        assert cands, 'no pad-override footprint on this board'
        fid = min(cands, key=lambda f: math.hypot(f.x - tgt.x, f.y - tgt.y))
        vx, vy = tgt.x - fid.x, tgt.y - fid.y
        n = math.hypot(vx, vy)
        dx, dy = vx / n * mm, vy / n * mm
        fid.x += dx
        fid.y += dy
        for q in fid.pads:
            q.global_x += dx
            q.global_y += dy
        return pcb, fid.reference

    def test_esp_prog_fiducial_keep_clear(self):
        if not os.path.exists(ESP_PROG):
            self.skipTest('esp_prog fixture not present')
        # Untouched: the keep-clear is satisfied, and the census says so.
        from kicad_parser import parse_kicad_pcb
        stock = grade_pad_legality(parse_kicad_pcb(ESP_PROG), 0.15, worst_n=0)
        self.assertEqual(stock['pad_conflicts'], 0, stock['worst'])
        self.assertEqual(stock['required'], [])

        pcb, ref = self._nudge_fiducial(ESP_PROG, 0.40)
        g = grade_pad_legality(pcb, 0.15, worst_n=0)
        # MUTATION: any of the four reverts -> 0, which is what the bug shipped.
        self.assertEqual(g['pad_conflicts'], 1, g['worst'])
        self.assertEqual(len(g['required']), 1, g['required'])
        ra, rb, mm, src = g['required'][0]
        self.assertEqual({ra, rb}, {ref, 'USB1'})
        self.assertAlmostEqual(mm, 1.016)
        self.assertEqual(src, 'pad override')

    def test_esp_prog_agrees_with_check_drc(self):
        """The point of the fix: the two graders must not disagree."""
        if not os.path.exists(ESP_PROG):
            self.skipTest('esp_prog fixture not present')
        from check_drc import check_pad_pad_overlap
        pcb, ref = self._nudge_fiducial(ESP_PROG, 0.40)
        rl = list(pcb.board_info.copper_layers)
        fid = pcb.footprints[ref]
        truth = set()
        for q in fid.pads:
            lc = getattr(q, 'local_clearance', 0.0) or 0.0
            for r2, fp2 in pcb.footprints.items():
                if r2 == ref:
                    continue
                for t in fp2.pads:
                    eff = max(0.15, lc,
                              getattr(t, 'local_clearance', 0.0) or 0.0)
                    if q.net_id and q.net_id == t.net_id:
                        continue
                    hit, _ov, _pt = check_pad_pad_overlap(q, t, eff, rl, 0.0)
                    if hit:
                        truth.add(tuple(sorted((ref, r2))))
        self.assertTrue(truth, 'check_drc sees no violation -- fixture drifted')
        census = {tuple(sorted((a, b))) for a, b, _mm in
                  grade_pad_legality(pcb, 0.15, worst_n=0)['worst']}
        self.assertEqual(census, truth)

    def test_reference_less_footprint_ref_survives(self):
        """esp_prog's fiducial footprint is literally named `Ref*`, which flows
        into fnmatch globs and report text. It must round-trip intact."""
        if not os.path.exists(ESP_PROG):
            self.skipTest('esp_prog fixture not present')
        pcb, ref = self._nudge_fiducial(ESP_PROG, 0.40)
        g = grade_pad_legality(pcb, 0.15, worst_n=0)
        self.assertIn(ref, [r for row in g['required'] for r in row[:2]])
        self.assertIn(ref, format_required_clause(g))

    def test_netclass_half_on_flat_hierarchy(self):
        """The board's `Wide` class is 0.4 against a 0.2 Default."""
        if not os.path.exists(FLAT_HIER):
            self.skipTest('flat_hierarchy fixture not present')
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(FLAT_HIER)
        model = PadClearanceModel.for_board(pcb, 0.2, FLAT_HIER)
        # MUTATION: drop the net_clearance_map_by_id read -> empty, inactive.
        self.assertTrue(model.net_floor, 'no non-Default netclass resolved')
        self.assertTrue(model.active)
        wide = [nid for nid, v in model.net_floor.items() if v >= 0.4]
        self.assertTrue(wide)
        fa = model.pad_floor(pad(0, 0, net=wide[0]))
        fb = model.pad_floor(pad(5, 0, net=0))
        eff, src = model.pair_with_source(fa, fb)
        self.assertAlmostEqual(eff, 0.4)
        self.assertEqual(src, 'netclass')

    def test_netclass_alone_reaches_the_census(self):
        """A netclass with NO pad override anywhere must still raise the census.

        Every other census test here carries a `local_clearance`, so they would
        all pass with the netclass term deleted. This one has no override at
        all: its only source is the class.
        """
        if not os.path.exists(FLAT_HIER):
            self.skipTest('flat_hierarchy fixture not present')
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(FLAT_HIER)
        wide = [nid for nid, v in PadClearanceModel.for_board(
            pcb, 0.2, FLAT_HIER).net_floor.items() if v >= 0.4]
        self.assertTrue(wide)
        # Two 1x1 pads, one on a `Wide` net, 0.3mm apart: clear at the 0.2
        # Default floor, in violation of the 0.4 class.
        fps = {'A': fp_with('A', 0.0, 0.0,
                            [pad(0.0, 0.0, net=wide[0], ref='A')]),
               'B': fp_with('B', 1.3, 0.0, [pad(1.3, 0.0, net=0, ref='B')])}
        pcb.footprints = fps
        # MUTATION: drop the net_clearance_map_by_id read -> 0.
        g = grade_pad_legality(pcb, 0.2, worst_n=0, pcb_file=FLAT_HIER)
        self.assertEqual(g['pad_conflicts'], 1, g)
        self.assertEqual(g['required'], [['A', 'B', 0.4, 'netclass']])

    def test_default_netclass_is_not_dropped(self):
        """A Default class ABOVE an explicit --clearance must still raise.

        `net_clearance_map_by_id` (the ROUTER's map) omits every net that
        resolves only to Default, because a Default net routes at
        config.clearance. Using it here re-created this whole issue on any
        board graded below its own Default class -- which check_assembly
        explicitly tells users they may do. check_drc reads `net_clearance_map`
        (Default included, admitted when it exceeds the floor), so this must.
        """
        if not os.path.exists(FLAT_HIER):
            self.skipTest('flat_hierarchy fixture not present')
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(FLAT_HIER)
        gnd = [nid for nid, n in pcb.nets.items() if n.name == 'GND']
        self.assertTrue(gnd, 'fixture drifted: no GND net')
        # graded BELOW the 0.2 Default class -> the class is the requirement
        m_lo = PadClearanceModel.for_board(pcb, 0.1, FLAT_HIER)
        # MUTATION: read net_clearance_map_by_id instead -> None.
        self.assertAlmostEqual(m_lo.net_floor.get(gnd[0]), 0.2)
        fa = m_lo.pad_floor(pad(0, 0, net=gnd[0]))
        fb = m_lo.pad_floor(pad(5, 0, net=0))
        self.assertEqual(m_lo.pair_with_source(fa, fb), (0.2, 'netclass'))
        # graded AT or ABOVE it -> inert, exactly check_drc's `c > clearance`
        for cl in (0.2, 0.3):
            m = PadClearanceModel.for_board(pcb, cl, FLAT_HIER)
            self.assertIsNone(m.net_floor.get(gnd[0]), cl)

    def test_unreadable_source_is_not_silent(self):
        """A source that FAILS to resolve must be NAMED, not swallowed.

        Silence drops the census back to the flat scalar and reports 0
        conflicts -- the exact silence this issue was filed for. Forced here by
        making the netclass read raise, because a merely absent or malformed
        sibling is handled gracefully upstream (an absent .kicad_dru is a
        legitimate no-op, and a truncated one parses to an empty map).
        """
        if not os.path.exists(FLAT_HIER):
            self.skipTest('flat_hierarchy fixture not present')
        import list_nets
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(FLAT_HIER)
        real = list_nets.net_clearance_map

        def boom(*a, **kw):
            raise OSError('permission denied')

        list_nets.net_clearance_map = boom
        try:
            model = PadClearanceModel.for_board(pcb, 0.1, FLAT_HIER)
            g = grade_pad_legality(pcb, 0.1, worst_n=0, pcb_file=FLAT_HIER)
        finally:
            list_nets.net_clearance_map = real
        # MUTATION: restore the bare `except Exception: net_floor = {}` -> the
        # failure is invisible and the board silently grades at the flat scalar.
        self.assertTrue(model.notes, 'the failure was swallowed')
        self.assertIn('permission denied', ' '.join(model.notes))
        # ... and it must REACH the report, not just the model
        self.assertTrue(g['clearance_notes'], g)

    def test_dru_half_end_to_end(self):
        """A sibling .kicad_dru must reach the census. No board in the repo
        ships one, so write one next to a staged copy -- the same one-line
        pattern tests/test_dru_layer_clearance_e2e.py uses."""
        if not os.path.exists(ESP_PROG):
            self.skipTest('esp_prog fixture not present')
        from copy_board import copy_board
        with tempfile.TemporaryDirectory() as td:
            staged = os.path.join(td, 'ruled.kicad_pcb')
            copy_board(ESP_PROG, staged)
            with open(os.path.splitext(staged)[0] + '.kicad_dru', 'w',
                      encoding='utf-8') as fh:
                # 1.2 mm, not the 0.6 this test first used: esp_prog's only
                # sub-0.6 pairs sit on 13 pads carrying a 0.0508 mm clearance
                # OVERRIDE, and a pad override REPLACES a rule (KiCad 10,
                # measured -- tests/oracle/constraint_agreement.py
                # pad_override_beats_rule), so the rule must bind on a pair
                # WITHOUT overrides to be seen binding at all.
                fh.write('(version 1)\n(rule wide_front (layer "F.Cu") '
                         '(constraint clearance (min 1.2mm)))\n')
            from kicad_parser import parse_kicad_pcb
            pcb = parse_kicad_pcb(staged)
            model = PadClearanceModel.for_board(pcb, 0.15, staged)
            # MUTATION: drop the read_board_layer_clearances read -> empty.
            self.assertEqual(model.layer_rules, {'F.Cu': 1.2})
            g = grade_pad_legality(pcb, 0.15, worst_n=0, pcb_file=staged)
            self.assertGreater(g['pad_conflicts'], 0,
                               'a 1.2mm F.Cu rule must bind somewhere')
            self.assertTrue(any(r[3] == 'layer rule' for r in g['required']),
                            g['required'])


if __name__ == '__main__':
    unittest.main()
