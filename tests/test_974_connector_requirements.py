#!/usr/bin/env python3
"""#974: `floorplan.connector_requirements`, the engine half of place_seed's
`JSON_SUMMARY.connector_requirements`.

In-process on synthetic boards: every channel the report names is built here
from a real `floorplan.grade`, and the CLI half (that place_seed carries the
key on every summary it prints, at the exit code it had before) is
`test_974_place_seed_connector_requirements.py`.

The contract under test:

* `complete` is "every declared requirement was MEASURED" -- a legacy-basis
  band, an uncertified copper conjunct, an absent part, an unmeasured
  along-edge claim or a dropped band makes it false; errors never do, and
  neither do channels `GradeResult.complete` counts that are not connectors.
* `errors_own` / `errors_pinned` are the caller's own split, filtered to the
  rule -- never recomputed, so they cannot disagree with the exit code.
* The report is JSON a strict parser reads, and it never raises.
"""
import dataclasses
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for folder in ('py_router', 'py_placer', 'py_tools', 'tests'):
    sys.path.insert(0, str(ROOT / folder))

from kicad_parser import parse_kicad_pcb           # noqa: E402
from placement import floorplan                    # noqa: E402
import place_seed                                   # noqa: E402

RECT_OUTLINE = '(gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))'
FAB_2MM = '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))'
PAD = '(pad "1" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu"))'


def _fp(ref, at, body='', pads=PAD, locked=False):
    return (f'  (footprint "t" (layer "F.Cu") (at {at})'
            + (' (locked yes)' if locked else '') + '\n'
            f'    (property "Reference" "{ref}")\n'
            f'    {body}\n    {pads})\n')


def _intent(entries, **extra):
    return floorplan.intent_from_dict(dict({
        'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
        'units': 'mm', 'edge_connectors': list(entries)}, **extra))


class _Case(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='t974_')
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def board(self, name, *footprints, outline=RECT_OUTLINE):
        path = self.root / name
        path.write_text('(kicad_pcb (version 20241229) (generator "t974")\n'
                        f'  {outline}\n' + ''.join(footprints) + ')\n',
                        encoding='utf-8')
        return path

    def grade(self, path, intent):
        return floorplan.grade(intent, parse_kicad_pcb(str(path)), str(path),
                               clearance=.25, board_edge_clearance=.55)

    def report(self, path, intent, bands_dropped=None):
        """Grade, split exactly as place_seed does, report -- and hold every
        report to the invariants that do not depend on the fixture."""
        graded = self.grade(path, intent)
        own, pinned = place_seed._split_pinned(graded, str(path), intent)
        r = floorplan.connector_requirements(graded, own, pinned,
                                             bands_dropped=bands_dropped)
        self.assertNotIn('reason', r, r)
        self.invariants(r, graded, own, pinned)
        return graded, own, pinned, r

    def invariants(self, r, graded, own, pinned):
        # Strict JSON, and a round trip changes nothing.
        text = json.dumps(r, sort_keys=True, allow_nan=False)
        self.assertEqual(json.loads(text), r)
        # The partition: every declared ref has an evidence row or a
        # `presence` entry, and nothing else is either.
        evidence = {e['ref'] for e in r['overhang_evidence']}
        presence = {u['ref'] for u in r['unmeasured']
                    if u['requirement'] == 'presence'}
        self.assertEqual(set(r['declared_refs']), evidence | presence)
        self.assertFalse(evidence & presence)
        self.assertTrue({u['ref'] for u in r['unmeasured']}
                        <= set(r['declared_refs']))
        self.assertFalse({b['ref'] for b in r['bands_dropped']}
                         & set(r['declared_refs']))
        # `complete` is exactly its definition.
        self.assertEqual(r['complete'],
                         not r['unmeasured'] and not r['bands_dropped'])
        # The errors are the caller's lists, filtered -- nothing more.
        self.assertEqual(r['errors_own'], [v.to_dict() for v in own
                                           if v.rule == 'edge_connector'])
        self.assertEqual(r['errors_pinned'], [v.to_dict() for v in pinned
                                              if v.rule == 'edge_connector'])
        for u in r['unmeasured']:
            self.assertTrue(u['reason'], u)


class Basis(_Case):
    def test_a_legacy_basis_band_is_listed_unmeasured(self):
        """Acceptance 1's engine half: no drawn body, a band with a max."""
        path = self.board('pads_only.kicad_pcb', _fp('J1', '0.3 10 0'))
        graded, _own, _pinned, r = self.report(path, _intent([
            {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': .5}}]))
        row, = graded.edge_connector_evidence
        self.assertFalse(row['body_measured'])
        self.assertFalse(r['complete'])
        self.assertEqual(r['declared_refs'], ['J1'])
        # Two requirements went unmeasured, not one: the band, and the copper
        # conjunct that is graded on the body path only.
        self.assertEqual([u['requirement'] for u in r['unmeasured']],
                         ['overhang_body', 'pad_copper_outside'])
        entry = r['unmeasured'][0]
        self.assertEqual(entry['reason'], row['body_unmeasured_reason'])
        self.assertTrue(entry['graded_on'].startswith('legacy_occupancy@'),
                        entry)

    def test_a_measured_body_in_its_band_is_complete(self):
        path = self.board('flush.kicad_pcb', _fp('J1', '1 10 0', FAB_2MM))
        graded, _own, _pinned, r = self.report(path, _intent([
            {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': .5}}]))
        self.assertTrue(r['complete'], r)
        self.assertEqual(r['unmeasured'], [])
        self.assertEqual((r['errors_own'], r['errors_pinned'], r['warnings']),
                         ([], [], []))
        ev, = r['overhang_evidence']
        self.assertTrue(ev['overhang_basis'].startswith('body:'), ev)
        # The projection: every row key kept, the per-pad lists counted.
        src, = graded.edge_connector_evidence
        self.assertEqual(set(ev), set(src))
        copper = ev['pad_copper_edge']
        for name in ('findings', 'unmeasured', 'rules_unmeasured'):
            self.assertNotIn(name, copper)
            self.assertEqual(copper['n_' + name],
                             len(src['pad_copper_edge'][name]))
        # Nothing shared with the grade: editing the report edits nothing.
        before = json.dumps(graded.edge_connector_evidence, sort_keys=True)
        ev['pad_copper_edge']['disposition'] = 'edited'
        ev['overhang_mm'] = -1
        self.assertEqual(json.dumps(graded.edge_connector_evidence,
                                    sort_keys=True), before)

    def test_nothing_declared_is_complete_and_empty(self):
        path = self.board('none.kicad_pcb', _fp('J1', '1 10 0', FAB_2MM))
        _g, _o, _p, r = self.report(path, _intent([]))
        self.assertEqual(r, {'complete': True, 'declared_refs': [],
                             'errors_own': [], 'errors_pinned': [],
                             'warnings': [], 'overhang_evidence': [],
                             'unmeasured': [], 'bands_dropped': []})

    def test_a_declared_ref_missing_from_the_board_is_presence(self):
        path = self.board('missing.kicad_pcb', _fp('J1', '1 10 0', FAB_2MM))
        _g, _o, _p, r = self.report(path, _intent([
            {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': .5}},
            {'ref': 'J9', 'edge': 'east', 'overhang_mm': {'min': 0, 'max': .5}}]))
        self.assertEqual(r['declared_refs'], ['J1', 'J9'])
        self.assertEqual(r['unmeasured'], [{'ref': 'J9',
                                            'requirement': 'presence',
                                            'reason': 'not on this board'}])
        self.assertEqual([e['ref'] for e in r['overhang_evidence']], ['J1'])
        self.assertFalse(r['complete'])

    def test_a_vacuous_band_needs_no_body(self):
        """An emitted `connector_affinity` entry: no edge, `{min: 0}`. No
        reading can fail it, so its unmeasured body is not a gap -- and the
        same entry WITH a max is."""
        path = self.board('affinity.kicad_pcb', _fp('J1', '10 10 0'))
        entry = {'ref': 'J1', 'class': 'connector_affinity',
                 'overhang_mm': {'min': 0}}
        graded, _o, _p, r = self.report(path, _intent([entry]))
        row, = graded.edge_connector_evidence
        self.assertFalse(row['body_measured'])
        self.assertEqual(r['unmeasured'], [])
        self.assertTrue(r['complete'])
        # Its interior setback is a WARN, reported apart from the errors.
        self.assertTrue(r['warnings'], r)
        self.assertEqual(r['errors_own'], [])
        for band in ({'min': 0, 'max': 2.0}, {'min': .3}):
            g, _o, _p, r = self.report(path, _intent([dict(entry,
                                                          overhang_mm=band)]))
            self.assertEqual([u['requirement'] for u in r['unmeasured']],
                             ['overhang_body'], band)
            self.assertEqual(r['unmeasured'][0]['reason'],
                             g.edge_connector_evidence[0]['body_unmeasured_reason'])


class Copper(_Case):
    def test_an_uncertified_copper_conjunct_is_listed(self):
        """A trapezoid pad the edge grader cannot model, on the body path:
        its `outside_mm` of 0 is not a reading, so the requirement it grades
        was not measured."""
        body = '(fp_rect (start -1.2 -1) (end 1 1) (layer "F.Fab"))'
        pads = ('(pad "1" smd trapezoid (at -.55 0) (size .5 .5) '
                '(rect_delta 0 .2) (layers "F.Cu"))\n'
                '    (pad "2" smd rect (at .5 0) (size .5 .5) (layers "F.Cu"))')
        path = self.board('trap.kicad_pcb', _fp('J1', '1 10 0', body, pads))
        graded, _o, _p, r = self.report(path, _intent([
            {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': .5}}]))
        row, = graded.edge_connector_evidence
        self.assertTrue(row['body_measured'])
        self.assertFalse(row['pad_copper_edge']['certified'])
        self.assertEqual([u['requirement'] for u in r['unmeasured']],
                         ['pad_copper_outside'])
        self.assertIn('J1.1', r['unmeasured'][0]['reason'])
        self.assertFalse(r['complete'])

    def test_a_sampled_outline_reports_the_body_and_the_skipped_copper(self):
        """The copper conjunct is graded on the body path only; on a sampled
        outline the body is unmeasured, so both the band and the copper
        conjunct are -- while the evidence row names copper off the edge."""
        outline = (RECT_OUTLINE + '\n  (gr_circle (center 10 15) (end 11 15) '
                   '(layer "Edge.Cuts"))')
        path = self.board('sampled.kicad_pcb',
                          _fp('J1', '0.1 10 0', FAB_2MM,
                              '(pad "1" smd rect (at 0 0) (size .5 .5) '
                              '(layers "F.Cu"))'),
                          outline=outline)
        graded, _o, _p, r = self.report(path, _intent([
            {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': 5}}]))
        row, = graded.edge_connector_evidence
        self.assertIsNone(row['pad_copper_edge']['outside_mm'])
        self.assertEqual([u['requirement'] for u in r['unmeasured']],
                         ['overhang_body', 'pad_copper_outside'])
        self.assertIn(row['body_unmeasured_reason'], r['unmeasured'][1]['reason'])

    def test_a_vacuous_band_never_hides_the_skipped_copper(self):
        """An entry that claims an EDGE with a band no reading can fail --
        `{min: 0}`, or no band at all, which is what a brief row merged over an
        emitted `connector_affinity` entry becomes. Its band needs no body;
        its copper conjunct does."""
        pads = ('(pad "1" smd rect (at -1.1 0) (size .5 .5) (layers "F.Cu"))'
                '\n    (pad "2" smd rect (at .5 0) (size .5 .5) (layers "F.Cu"))')
        for band in ({'overhang_mm': {'min': 0}}, {}):
            entry = dict({'ref': 'J1', 'edge': 'west'}, **band)
            bare = self.board('bare.kicad_pcb', _fp('J1', '1 10 0', '', pads))
            graded, _o, _p, r = self.report(bare, _intent([entry]))
            self.assertGreater(graded.edge_connector_evidence[0]
                               ['pad_copper_edge']['outside_mm'], 0.3)
            self.assertEqual([u['requirement'] for u in r['unmeasured']],
                             ['pad_copper_outside'], (band, r))
            self.assertFalse(r['complete'])
            # With a drawn body the conjunct GRADES, and fails: measured.
            drawn = self.board('drawn.kicad_pcb',
                               _fp('J1', '1 10 0', FAB_2MM, pads))
            _g, _o, _p, r = self.report(drawn, _intent([entry]))
            self.assertEqual(r['unmeasured'], [], (band, r))
            self.assertTrue(r['complete'])
            self.assertTrue([e for e in r['errors_own']
                             if 'pad_copper_outside_mm' in e['measured']], r)
        # A part with no copper pads has no copper conjunct to skip.
        npth = self.board('npth.kicad_pcb', _fp(
            'J1', '1 10 0', '', '(pad "" np_thru_hole circle (at 0 0) '
            '(size 1 1) (drill 1) (layers "F&B.Cu" "*.Mask"))'))
        _g, _o, _p, r = self.report(npth, _intent([
            {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0}}]))
        self.assertEqual(r['unmeasured'], [], r)

    def test_a_measured_body_with_no_copper_amount_is_listed(self):
        """`outside_mm: None` on the body path. No real grade produces it
        today (a sampled outline also unmeasures the body), so the row is
        built by hand: the report must not read a missing amount as zero."""
        path = self.board('flush.kicad_pcb', _fp('J1', '1 10 0', FAB_2MM))
        intent = _intent([{'ref': 'J1', 'edge': 'west',
                           'overhang_mm': {'min': 0, 'max': .5}}])
        graded, own, pinned, r = self.report(path, intent)
        self.assertTrue(r['complete'])
        row = dict(graded.edge_connector_evidence[0])
        row['pad_copper_edge'] = dict(row['pad_copper_edge'], outside_mm=None)
        self.assertTrue(row['pad_copper_edge']['certified'])
        forged = dataclasses.replace(graded, edge_connector_evidence=[row])
        r = floorplan.connector_requirements(forged, own, pinned)
        self.invariants(r, forged, own, pinned)
        self.assertEqual([u['requirement'] for u in r['unmeasured']],
                         ['pad_copper_outside'])

    def test_a_custom_edge_rule_is_evidence_not_a_gap(self):
        """A `.kicad_dru` edge_clearance rule marks EVERY row's copper
        clearance unmeasured, board-wide. That half is evidence only, so it
        must not make a measured board incomplete."""
        path = self.board('ruled.kicad_pcb', _fp('J1', '1 10 0', FAB_2MM))
        path.with_suffix('.kicad_dru').write_text(
            '(version 1)\n(rule "edge floor" '
            '(constraint edge_clearance (min 0.75mm)))\n', encoding='utf-8')
        graded, _o, _p, r = self.report(path, _intent([
            {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': .5}}]))
        row, = graded.edge_connector_evidence
        self.assertEqual(row['pad_copper_edge']['disposition'], 'unmeasured')
        self.assertTrue(row['pad_copper_edge']['rules_unmeasured'])
        self.assertEqual(r['overhang_evidence'][0]['pad_copper_edge']
                         ['n_rules_unmeasured'], 1)
        self.assertTrue(r['complete'], r['unmeasured'])


class AlongEdge(_Case):
    def test_an_abstained_along_edge_claim_carries_the_grade_reason(self):
        path = self.board('noedge.kicad_pcb', _fp('J1', '1 10 0', FAB_2MM))
        for claim, spec in (('center_on_edge', {'tolerance_mm': 1.0}),
                            ('along_edge_band', {'from': .2, 'to': .8})):
            graded, _o, _p, r = self.report(path, _intent([
                {'ref': 'J1', claim: spec}]))
            key = f'edge_connectors[J1].{claim}'
            self.assertIn(key, graded.budget_abstained)
            along = [u for u in r['unmeasured'] if u['requirement'] == claim]
            self.assertEqual(along, [{'ref': 'J1', 'requirement': claim,
                                      'reason': graded.budget_abstained[key]}])

    def test_a_silent_along_edge_gap_is_still_named(self):
        """`_grade_along_edge` returns WITHOUT a row or an abstention on a
        zero-length edge span. Simulated by removing the row a real grade
        recorded: a declared claim nobody measured must not read complete."""
        path = self.board('centred.kicad_pcb', _fp('J1', '1 10 0', FAB_2MM))
        intent = _intent([{'ref': 'J1', 'edge': 'west',
                           'overhang_mm': {'min': 0, 'max': .5},
                           'center_on_edge': {'tolerance_mm': 1.0}}])
        graded, own, pinned, r = self.report(path, intent)
        self.assertTrue(r['complete'], r['unmeasured'])
        silent = dataclasses.replace(graded, edge_seating=[])
        r = floorplan.connector_requirements(silent, own, pinned)
        self.invariants(r, silent, own, pinned)
        self.assertEqual(r['unmeasured'], [{
            'ref': 'J1', 'requirement': 'center_on_edge',
            'reason': floorplan.NO_ALONG_EDGE_MEASUREMENT}])
        self.assertFalse(r['complete'])

    def test_the_along_edge_gap_is_judged_per_ref(self):
        """Another connector's measured row must not cover this one's gap."""
        path = self.board('two.kicad_pcb', _fp('J1', '1 6 0', FAB_2MM),
                          _fp('J2', '1 14 0', FAB_2MM))
        claim = {'edge': 'west', 'overhang_mm': {'min': 0, 'max': .5},
                 'center_on_edge': {'tolerance_mm': 9.0}}
        intent = _intent([dict(claim, ref='J1'), dict(claim, ref='J2')])
        graded, own, pinned, r = self.report(path, intent)
        self.assertTrue(r['complete'], r['unmeasured'])
        silent = dataclasses.replace(graded, edge_seating=[
            row for row in graded.edge_seating if row.get('ref') != 'J2'])
        r = floorplan.connector_requirements(silent, own, pinned)
        self.invariants(r, silent, own, pinned)
        self.assertEqual([(u['ref'], u['requirement'])
                          for u in r['unmeasured']],
                         [('J2', 'center_on_edge')])

    def test_a_recorded_measurement_outranks_an_abstention_key(self):
        """The grade abstains only when it records no row, so an abstention
        beside a measured row is a hand-written `context.budget_withheld`
        key -- and the claim WAS measured."""
        path = self.board('centred.kicad_pcb', _fp('J1', '1 10 0', FAB_2MM))
        intent = _intent([{'ref': 'J1', 'edge': 'west',
                           'overhang_mm': {'min': 0, 'max': .5},
                           'center_on_edge': {'tolerance_mm': 1.0}}])
        graded, own, pinned, _r = self.report(path, intent)
        forged = dataclasses.replace(graded, budget_abstained={
            'edge_connectors[J1].center_on_edge': 'withheld by hand'})
        r = floorplan.connector_requirements(forged, own, pinned)
        self.invariants(r, forged, own, pinned)
        self.assertEqual(r['unmeasured'], [])


class Errors(_Case):
    def _band_failure(self, locked):
        path = self.board(f'over_{locked}.kicad_pcb',
                          _fp('J1', '0 10 0', FAB_2MM,
                              '(pad "1" smd rect (at .6 0) (size .5 .5) '
                              '(layers "F.Cu"))', locked=locked))
        return self.report(path, _intent([
            {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': .5}}]))

    def test_a_pinned_band_failure_is_pinned_and_does_not_gate(self):
        """Acceptance 3's engine half, through place_seed's own split and
        gate: the report can only say what the exit code already decided."""
        _g, own, pinned, r = self._band_failure(locked=True)
        band = [e for e in r['errors_pinned'] if 'overhang_mm' in
                (e.get('measured') or {})]
        self.assertTrue(band, r)
        self.assertGreater(band[0]['measured']['overhang_mm'],
                           band[0]['expected']['max'])
        self.assertEqual(r['errors_own'], [])
        self.assertEqual(r['warnings'], [])
        self.assertIsNone(place_seed.gate_reason([], own, [], 0))

    def test_an_unpinned_band_failure_is_own_and_gates(self):
        _g, own, pinned, r = self._band_failure(locked=False)
        self.assertTrue([e for e in r['errors_own']
                         if 'overhang_mm' in (e.get('measured') or {})], r)
        self.assertEqual(r['errors_pinned'], [])
        self.assertEqual(r['warnings'], [])
        self.assertIsNotNone(place_seed.gate_reason([], own, [], 0))

    def test_a_demoted_band_failure_is_a_warning(self):
        path = self.board('demoted.kicad_pcb',
                          _fp('J1', '0 10 0', FAB_2MM,
                              '(pad "1" smd rect (at .6 0) (size .5 .5) '
                              '(layers "F.Cu"))'))
        graded, _o, _p, r = self.report(path, _intent(
            [{'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': .5}}],
            severity={'edge_connector': 'warn'}))
        self.assertEqual(graded.errors, [])
        self.assertEqual(r['errors_own'], [])
        self.assertTrue([w for w in r['warnings']
                         if 'overhang_mm' in (w.get('measured') or {})], r)

    def test_only_connector_findings_are_reported(self):
        path = self.board('filter.kicad_pcb', _fp('J1', '1 10 0', FAB_2MM))
        graded = self.grade(path, _intent([
            {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': .5}}]))
        zone = floorplan.Violation(rule='zone_containment',
                                   severity=floorplan.ERROR, message='x',
                                   ref='U1')
        conn = floorplan.Violation(rule='edge_connector',
                                   severity=floorplan.ERROR, message='y',
                                   ref='J1')
        r = floorplan.connector_requirements(graded, [zone, conn], [zone])
        self.assertEqual(r['errors_own'], [conn.to_dict()])
        self.assertEqual(r['errors_pinned'], [])
        # ...and `warnings` holds connector findings below error, only.
        warn = floorplan.Violation(rule='decap_distance',
                                   severity=floorplan.WARN, message='w',
                                   ref='C1')
        cwarn = floorplan.Violation(rule='edge_connector',
                                    severity=floorplan.WARN, message='cw',
                                    ref='J1')
        mixed = dataclasses.replace(graded, violations=list(
            graded.violations) + [warn, cwarn, conn])
        r = floorplan.connector_requirements(mixed, [conn], [])
        self.assertEqual(r['warnings'], [cwarn.to_dict()])


class Order(_Case):
    def test_entries_are_sorted_and_only_exact_duplicates_merge(self):
        """Several refs, several requirements per ref, one ref declared twice:
        the list is in (ref, requirement) order, the duplicate declaration
        adds nothing, and two requirements on one ref stay two."""
        path = self.board('many.kicad_pcb', _fp('J1', '0.3 10 0'),
                          _fp('J2', '10 10 0'))
        legacy = {'ref': 'J1', 'edge': 'west',
                  'overhang_mm': {'min': 0, 'max': .5}}
        intent = _intent([
            {'ref': 'J2', 'overhang_mm': {'min': 0, 'max': 1.0},
             'along_edge_band': {'from': .2, 'to': .8}},
            legacy,
            {'ref': 'J0', 'edge': 'east', 'overhang_mm': {'min': 0, 'max': .5}},
            dict(legacy)])
        _g, _o, _p, r = self.report(path, intent)
        self.assertEqual([(u['ref'], u['requirement']) for u in r['unmeasured']],
                         [('J0', 'presence'),
                          ('J1', 'overhang_body'),
                          ('J1', 'pad_copper_outside'),
                          ('J2', 'along_edge_band'),
                          ('J2', 'overhang_body')])


class Wire(_Case):
    def test_dropped_bands_are_listed_and_incomplete(self):
        path = self.board('dropped.kicad_pcb', _fp('J1', '1 10 0', FAB_2MM))
        intent = _intent([{'ref': 'J1', 'edge': 'west',
                           'overhang_mm': {'min': 0, 'max': .5}}])
        for none in (None, {}):
            _g, _o, _p, r = self.report(path, intent, bands_dropped=none)
            self.assertEqual(r['bands_dropped'], [])
            self.assertTrue(r['complete'])
        _g, _o, _p, r = self.report(path, intent,
                                    bands_dropped={'J2': .5, 'J0': 0.0})
        self.assertEqual(r['bands_dropped'], [
            {'ref': 'J0', 'band_max_mm': 0.0, 'graded': False,
             'reason': floorplan.BAND_DROPPED_REASON},
            {'ref': 'J2', 'band_max_mm': .5, 'graded': False,
             'reason': floorplan.BAND_DROPPED_REASON}])
        self.assertEqual(r['unmeasured'], [])
        self.assertFalse(r['complete'])

    def test_the_ungraded_form_is_exact(self):
        self.assertEqual(floorplan.connector_requirements_ungraded('dry-run'),
                         {'complete': False, 'reason': 'dry-run'})

    def test_it_never_raises(self):
        r = floorplan.connector_requirements(None, [], [])
        self.assertEqual(set(r), {'complete', 'reason'})
        self.assertFalse(r['complete'])
        self.assertTrue(r['reason'].startswith(
            'connector_requirements failed: AttributeError'), r)

    def test_even_an_unprintable_exception_is_reported(self):
        class Unprintable(Exception):
            def __str__(self):
                raise ValueError('no')

        class Graded:
            @property
            def intent(self):
                raise Unprintable()

        r = floorplan.connector_requirements(Graded(), [], [])
        self.assertEqual(r, {'complete': False, 'reason':
                             'connector_requirements failed: Unprintable: '
                             '<unprintable>'})

    def test_the_report_is_strict_json_and_shares_nothing(self):
        """Through the helper, not `_json_plain` alone: a non-finite number
        reaches the wire as null, and scrambling EVERY container of the report
        leaves the grade untouched -- the nested dicts a shallow copy would
        share (`overhang_limit_mm`, a violation's `measured`) included."""
        path = self.board('over.kicad_pcb',
                          _fp('J1', '0 10 0', FAB_2MM,
                              '(pad "1" smd rect (at .6 0) (size .5 .5) '
                              '(layers "F.Cu"))'))
        graded, own, pinned, _r = self.report(path, _intent([
            {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': .5}}]))
        r = floorplan.connector_requirements(
            graded, own, pinned, bands_dropped={'J2': float('nan')})
        self.assertIsNone(r['bands_dropped'][0]['band_max_mm'])
        json.dumps(r, allow_nan=False)
        self.assertTrue(r['errors_own'])

        def snapshot():
            return json.dumps([graded.edge_connector_evidence,
                               [v.to_dict() for v in graded.violations]],
                              sort_keys=True)

        def scramble(node):
            if isinstance(node, dict):
                for k in list(node):
                    scramble(node[k])
                    node[k] = 'scrambled'
            elif isinstance(node, list):
                for item in node:
                    scramble(item)
                node.append('scrambled')

        before = snapshot()
        scramble(r)
        self.assertEqual(snapshot(), before)

    def test_plain_json(self):
        plain = floorplan._json_plain(
            {1: (1, 2.5), 'nan': float('nan'), 'inf': float('-inf'),
             's': {'b', 'a'}, 'o': object, 'b': True, 'n': None})
        self.assertEqual(plain['1'], [1, 2.5])
        self.assertIsNone(plain['nan'])
        self.assertIsNone(plain['inf'])
        self.assertEqual(plain['s'], ['a', 'b'])
        self.assertIsInstance(plain['o'], str)
        self.assertIs(plain['b'], True)
        self.assertIsNone(plain['n'])
        json.dumps(plain, allow_nan=False)


if __name__ == '__main__':
    unittest.main(verbosity=2)
