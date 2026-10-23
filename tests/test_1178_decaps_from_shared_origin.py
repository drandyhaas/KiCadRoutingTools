#!/usr/bin/env python3
"""#1178: `--decaps-from` reads a placed reference whose parts share an origin.

The reference gate refused a reference that was `unplaced` OR
`partially_unplaced`, and `partially_unplaced` fires on ONE stacked pair. Two
parts share an origin by design on placed boards: on the tracked
rp2350_fpga_eensy_prePlane, J2 (a Tag-Connect TC2030-NL on B.Cu, whose NPTH
alignment holes put it on both faces) sits over U3 (a SOT-666 on F.Cu). So the
designer's own board, offered as its own reference, withheld both decap limits
"it is not placed ... a limit read off it would bless a pile", and the decap
rules stayed dark. The `--auto-declare-decaps` arm used the same predicate.

Both now gate on `PlacementState.pile` -- the one pile test (#1109) the
emitter's own pile gate already used -- and a placed reference with stacked
parts names them in `context.decap_census.reference_stacked_refs`. The limit
is read off EVERY part: it is the reference's own fixed point
(`_decap_derivation`), so the reference must still grade clean against it.

What still refuses: a reference that IS a pile (the same board run through the
placement skill's `make_unplaced.py`), and the auto arm on run 29's heap,
whose 83% stacked parts read `pile` through its heap arm.

Run: python3 -X utf8 tests/test_1178_decaps_from_shared_origin.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import floorplan as fp                          # noqa: E402
from placement.placement_state import assess_placement         # noqa: E402
from run_utils import check, evidence                          # noqa: E402

PY = sys.executable
RP = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
RUN29 = os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
MAKE_UNPLACED = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                             'scripts', 'make_unplaced.py')
STRIP = os.path.join(ROOT, 'tests', 'stress', 'strip_copper_only.py')


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def _parse(path):
    return _quiet(parse_kicad_pcb, evidence(path))


def _emit(board, **kw):
    return _quiet(fp.emit_intent, _parse(board), board, **kw)


class APlacedReferenceWithASharedOrigin(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.doc = _emit(RP, decaps_from=RP)

    def test_the_fixture_is_the_issue_shape(self):
        """Placed, partially unplaced over J2/U3, and not a pile."""
        st = assess_placement(_parse(RP), RP)
        self.assertFalse(st.unplaced, st.reasons)
        self.assertTrue(st.partially_unplaced, st.reasons)
        self.assertEqual(st.stacked_suspect_refs, ['J2', 'U3'])
        self.assertFalse(st.pile, st.signals)

    def test_both_limits_are_derived(self):
        decaps = self.doc.get('decaps') or {}
        # 1.3073 since #1351: a passive part owns no decap, so the cap that
        # was 1.09 mm from it is measured to its chip (owners 7 -> 6)
        self.assertEqual(decaps.get('max_distance_mm'), 1.3073, decaps)
        # U5 pin 6; connector J3's +3V3 pin (6.713) owns no decap, so it is
        # not one of the reference's supply pins
        self.assertEqual(decaps.get('max_pin_distance_mm'), 5.904, decaps)
        withheld = self.doc['context']['budget_withheld']
        self.assertNotIn('decaps.max_distance_mm', withheld)
        cen = self.doc['context']['decap_census']
        self.assertNotIn('pin_limit_withheld', cen)

    def test_the_shared_origin_is_named(self):
        cen = self.doc['context']['decap_census']
        self.assertEqual(cen.get('reference_stacked_refs'), ['J2', 'U3'])

    def test_the_reference_grades_clean_against_its_own_limits(self):
        """The fixed point the limit is derived for: read off every part,
        so the reference passes it."""
        g = _quiet(fp.grade, fp.intent_from_dict(self.doc), _parse(RP), RP)
        bad = [v for v in g.errors
               if v.rule in ('decap_distance', 'decap_pin_distance',
                             'decap_ungraded')]
        self.assertEqual(bad, [], [str(v) for v in bad])

    def test_the_cli_says_so(self):
        td = tempfile.mkdtemp(prefix='t1178_')
        try:
            out = os.path.join(td, 'i.json')
            r = check([PY, '-X', 'utf8',
                       os.path.join(ROOT, 'py_tools', 'check_floorplan.py'),
                       RP, '--emit-intent', out, '--decaps-from', RP],
                      accept=True)
            self.assertIn('decaps: max_distance_mm 1.3073', r.stdout)
            self.assertIn('J2, U3 share an origin', r.stdout)
            self.assertNotIn('WITHHELD', r.stdout)
        finally:
            shutil.rmtree(td, ignore_errors=True)


class APileStillRefuses(unittest.TestCase):

    def test_a_piled_reference_is_refused(self):
        td = tempfile.mkdtemp(prefix='t1178p_')
        try:
            # The tracked board carries its routed signals; make_unplaced
            # refuses copper, and names this stripper.
            bare = os.path.join(td, 'bare.kicad_pcb')
            check([PY, '-X', 'utf8', STRIP, RP, bare], accept=True)
            pile = os.path.join(td, 'pile.kicad_pcb')
            check([PY, '-X', 'utf8', MAKE_UNPLACED, evidence(bare), pile],
                  accept=True)
            st = assess_placement(_parse(pile), pile)
            self.assertTrue(st.pile, st.signals)
            doc = _emit(RP, decaps_from=pile)
        finally:
            shutil.rmtree(td, ignore_errors=True)
        self.assertNotIn('max_distance_mm', doc.get('decaps') or {})
        why = doc['context']['budget_withheld'].get('decaps.max_distance_mm')
        self.assertIn('it is not placed', why or '')
        self.assertIn('bless a pile', why or '')
        cen = doc['context']['decap_census']
        self.assertIn('pin_limit_withheld', cen)
        self.assertNotIn('reference_stacked_refs', cen)


class TheAutoArm(unittest.TestCase):

    def test_auto_derives_off_the_placed_board(self):
        doc = _emit(RP, derive_decaps='auto')
        self.assertEqual((doc.get('decaps') or {}).get('max_distance_mm'),
                         1.3073, doc.get('decaps'))
        self.assertNotIn('auto_withheld', doc['context']['decap_census'])

    def test_auto_still_withholds_on_run29s_heap(self):
        st = assess_placement(_parse(RUN29), RUN29)
        self.assertTrue(st.pile and not st.unplaced, st.signals)
        doc = _emit(RUN29, derive_decaps='auto')
        self.assertNotIn('max_distance_mm', doc.get('decaps') or {})
        self.assertIn('bless a pile',
                      doc['context']['decap_census'].get('auto_withheld', ''))


if __name__ == '__main__':
    unittest.main()
