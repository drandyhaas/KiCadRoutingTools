#!/usr/bin/env python3
"""#961: a declared `overhang_mm` band is graded on the DRAWN BODY.

`rule_edge_connector` graded the band on `rect_outside_amount(part.rect)`,
which is a clearance shortfall at the gate's margin, not an overhang. On the
tracked esp_prog board USB1's courtyard is its pad box, 1.6 mm inboard of a
drawn Fab body that sits flush with the west edge.

Measured by `test_margin_matrix` (USB1 moved dx mm along x from that flush
pose; "legacy" is the occupancy reading at clearance = edge floor = margin):

      dx     body   legacy@.25  legacy@.55
    +0.00  0.0000      0.0000      0.0000
    -1.45  1.4500      0.1000      0.4000
    -2.10  2.1000      0.7500      1.0500
    +0.40  0.0000      0.0000      0.0000

At -1.45 the legacy reading satisfied `{min .05, max .20}` at margin .25 and
failed it at .55, for identical geometry. The body reading fails it at both.
"""
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for folder in ('py_router', 'py_placer', 'py_tools', 'tests'):
    sys.path.insert(0, str(ROOT / folder))

from kicad_parser import parse_kicad_pcb           # noqa: E402
from placement import floorplan, seeder            # noqa: E402
from placement.body import board_bodies            # noqa: E402
from placement.connector_geometry import ConnectorGeometry  # noqa: E402
from placement.portfolio import copy_siblings      # noqa: E402
from placement.writer import write_placed_output   # noqa: E402
import pose_score                                   # noqa: E402

SOURCE = ROOT / 'kicad_files' / 'esp_prog.kicad_pcb'
CORPUS = ('esp_prog', 'tigard', 'splitflap_driver', 'ulx3s')
EPS = 1e-6


def _intent(**entry):
    extra = entry.pop('_intent', {})
    return floorplan.intent_from_dict(dict({
        'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
        'units': 'mm', 'edge_connectors': [dict(entry)]}, **extra))


def _grade(path, band, clearance=.25, edge=.25, **extra):
    intent = _intent(ref='USB1', edge='west', overhang_mm=band, **extra)
    return floorplan.grade(intent, parse_kicad_pcb(str(path)), str(path),
                           clearance=clearance, board_edge_clearance=edge)


def _overhang_violations(result, ref='USB1'):
    return [v for v in result.violations
            if v.rule == 'edge_connector' and v.ref == ref
            and 'overhang' in v.message]


def _evidence(result, ref='USB1'):
    rows = [e for e in result.edge_connector_evidence if e['ref'] == ref]
    assert len(rows) == 1, rows
    return rows[0]


class _Boards(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='t961_')
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def usb1_at(self, dx, rot=180.0):
        """esp_prog with USB1 moved `dx` mm along x from its flush pose."""
        path = self.root / f'usb1_{dx:+.3f}_{rot:g}.kicad_pcb'
        write_placed_output(str(SOURCE), str(path), [
            {'reference': 'USB1', 'new_x': 117.5 + dx, 'new_y': 100.0,
             'new_rotation': rot}])
        copy_siblings(str(SOURCE), str(path))
        return path

    def synthetic(self, name, body, at='5 10 0', pads=None):
        pads = pads if pads is not None else (
            '(pad "1" smd rect (at 0 0) (size .5 .5) (layers "F.Cu"))')
        path = self.root / name
        path.write_text(
            '(kicad_pcb (version 20241229) (generator "t961")\n'
            '  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))\n'
            f'  (footprint "t" (layer "F.Cu") (at {at})\n'
            '    (property "Reference" "J1")\n'
            f'    {body}\n    {pads}))\n', encoding='utf-8')
        return path


class BodyCurrency(_Boards):
    def test_margin_matrix(self):
        """Issue acceptance 1: the body overhang reads `x` at every margin,
        and the band's verdict follows it -- while the legacy occupancy
        reading, measured here on the same boards, moves with the margin."""
        table = []
        for dx in (0.0, -1.45, -2.10, 0.4):
            path = self.usb1_at(dx)
            legacy = {}
            for clr, edge in ((.25, .25), (.55, .55), (.25, 0.0)):
                pcb = parse_kicad_pcb(str(path))
                st = pose_score.make_state(pcb, str(path), clearance=clr,
                                           board_edge_clearance=edge)
                legacy[(clr, edge)] = st.edge_gate.rect_outside_amount(
                    st.parts['USB1'].rect())
                for band in ({'min': .05, 'max': .20}, {'min': 0.0, 'max': .65}):
                    r = _grade(path, band, clr, edge)
                    ev = _evidence(r)
                    want = max(0.0, -dx)
                    self.assertAlmostEqual(ev['overhang_mm'], want, places=4)
                    self.assertEqual(ev['overhang_basis'], 'body:F.Fab')
                    self.assertAlmostEqual(ev['body_setback_mm'],
                                           max(0.0, dx), places=4)
                    inside = band['min'] - EPS <= want <= band['max'] + EPS
                    hits = _overhang_violations(r)
                    self.assertEqual(not hits, inside, (dx, clr, edge, band))
                    self.assertEqual(ev['overhang_disposition'],
                                     'pass' if inside else 'fail')
                    for v in hits:
                        self.assertAlmostEqual(v.measured['overhang_mm'],
                                               ev['overhang_mm'], places=4)
                        self.assertEqual(v.measured['overhang_basis'],
                                         'body:F.Fab')
                    # Issue acceptance 3: the edge_seating row carries it.
                    rows = [e for e in r.edge_seating if e['ref'] == 'USB1']
                    self.assertEqual(len(rows), 1)
                    # Assert the key is THERE before reading it: indexing a
                    # missing key fails as an ERROR, which reads as a crash
                    # rather than as the enrichment being gone.
                    self.assertIn('overhang_mm', rows[0])
                    self.assertIn('overhang_basis', rows[0])
                    self.assertAlmostEqual(rows[0]['overhang_mm'],
                                           ev['overhang_mm'], places=6)
                    self.assertEqual(rows[0]['overhang_basis'], 'body:F.Fab')
            table.append((dx, max(0.0, -dx), legacy))
        # The fixture reproduces the defect: the legacy number moves with the
        # margin on the -1.45 board (0.10 at .25, 0.40 at .55).
        at145 = {dx: legacy for dx, _body, legacy in table}[-1.45]
        self.assertGreater(abs(at145[(.25, .25)] - at145[(.55, .55)]), 0.2)
        print('\n  dx      body   legacy@.25  legacy@.55')
        for dx, body, legacy in table:
            print(f'  {dx:+.2f}  {body:.4f}  {legacy[(.25, .25)]:.4f}'
                  f'      {legacy[(.55, .55)]:.4f}')

    def test_minimum_is_a_real_conjunct(self):
        """A positive minimum fires on an inboard body; a zero one does not."""
        path = self.usb1_at(0.4)
        self.assertTrue(_overhang_violations(
            _grade(path, {'min': .05, 'max': 1.0})))
        self.assertFalse(_overhang_violations(
            _grade(path, {'min': 0.0, 'max': 1.0})))
        # A zero MAXIMUM fires on a body 0.10 mm over, where the occupancy
        # reading of this pad-box courtyard (1.5 mm inboard) reads 0.
        hits = _overhang_violations(_grade(self.usb1_at(-0.10),
                                           {'min': 0.0, 'max': 0.0}, edge=.55))
        self.assertTrue(hits)
        self.assertAlmostEqual(hits[0].measured['overhang_mm'], 0.10, places=4)

    def test_copper_evidence_agrees_with_check_drc(self):
        """Issue acceptance 2, by CALLING check_drc rather than mirroring it."""
        from run_utils import evidence
        path = self.usb1_at(-1.45)
        ev = _evidence(_grade(path, {'min': 0.0, 'max': 2.0}, edge=.25))
        copper = ev['pad_copper_edge']
        out = self.root / 'drc.json'
        subprocess.run([sys.executable, str(ROOT / 'py_router' / 'check_drc.py'),
                        str(path), '--check-pad-edge',
                        '--board-edge-clearance', '0.25',
                        '--clearance-margin', '0', '--json', str(out)],
                       cwd=str(ROOT), capture_output=True, text=True,
                       timeout=600)
        doc = json.loads(Path(evidence(str(out))).read_text(encoding='utf-8'))
        drc = sorted((i['pad_ref'], round(i['overlap_mm'], 4))
                     for i in doc['items']
                     if i['type'] == 'pad-board-edge'
                     and str(i['pad_ref']).startswith('USB1.'))
        ours = sorted((f['pad_ref'], round(f['shortfall_mm'], 4))
                      for f in copper['findings'])
        self.assertTrue(drc, 'check_drc found no USB1 pad-edge violation')
        self.assertEqual(ours, drc)
        self.assertEqual(copper['disposition'], 'fail')
        self.assertAlmostEqual(copper['shortfall_mm'], 0.10, places=4)
        # ... and the body band still passes: the channels are independent.
        self.assertEqual(ev['overhang_disposition'], 'pass')

    def test_oob_exempt_is_decided_in_the_band_currency(self):
        path = self.usb1_at(-1.45)
        budget = {'_intent': {'legality_budget': {'oob_count': 99}}}

        def exempt(band):
            r = floorplan.grade(
                _intent(ref='USB1', edge='west', overhang_mm=band, **budget),
                parse_kicad_pcb(str(path)), str(path),
                clearance=.25, board_edge_clearance=.25)
            return r.legality.get('oob_count_exempt')
        # The occupancy reading (0.10) sits inside {0, .65} and outside
        # {1.25, 1.65}; the body (1.45) is the other way round.
        self.assertEqual(exempt({'min': 0.0, 'max': .65}), 0)
        self.assertEqual(exempt({'min': 1.25, 'max': 1.65}), 1)


class Conjuncts(_Boards):
    def test_a_corner_body_counts_every_side(self):
        """The occupancy reading summed every side it crossed, so a corner
        overhang counted against the band. The body reading keeps that form:
        the graded number is the sum, and nothing else is added."""
        path = self.synthetic('corner.kicad_pcb',
                              '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))',
                              at='0.5 0.5 0')
        pcb = parse_kicad_pcb(str(path))

        def grade(band, budget=None):
            extra = {'_intent': {'legality_budget': budget}} if budget else {}
            return floorplan.grade(
                _intent(ref='J1', edge='west', overhang_mm=band, **extra),
                pcb, str(path), clearance=.25, board_edge_clearance=.55)
        wide = grade({'min': 0.0, 'max': 2.0}, {'oob_count': 0})
        ev = _evidence(wide, 'J1')
        self.assertAlmostEqual(ev['body_overhang_mm'], 0.5, places=4)
        self.assertAlmostEqual(ev['overhang_mm'], 1.0, places=4)
        self.assertEqual(ev['overhang_disposition'], 'pass')
        self.assertFalse(_overhang_violations(wide, 'J1'))
        self.assertEqual(wide.legality.get('oob_count_exempt'), 1)
        tight = _overhang_violations(grade({'min': 0.0, 'max': 0.8}), 'J1')
        self.assertEqual(len(tight), 1)
        self.assertIn('past the declared maximum', tight[0].message)
        self.assertAlmostEqual(tight[0].measured['overhang_mm'], 1.0, places=4)

    def test_setback_gate_keeps_the_occupancy_reading(self):
        """Run 4 A's seat conjunct opens only when the part does not overhang,
        and #961 leaves that gate on the occupancy reading: a body 0.3 mm
        inboard whose courtyard grazes the edge margin stays ungraded by it."""
        path = self.synthetic(
            'setback.kicad_pcb',
            '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab")) '
            '(fp_rect (start -1.1 -1.1) (end 1.1 1.1) (layer "F.CrtYd"))',
            at='1.3 10 0')
        r = floorplan.grade(
            _intent(ref='J1', edge='west', overhang_mm={'min': 0.0, 'max': 2.0},
                    max_setback_mm=0.1),
            parse_kicad_pcb(str(path)), str(path),
            clearance=.25, board_edge_clearance=.55)
        self.assertEqual(_evidence(r, 'J1')['overhang_basis'], 'body:F.Fab')
        self.assertAlmostEqual(_evidence(r, 'J1')['body_setback_mm'], 0.3,
                               places=4)
        self.assertFalse([v for v in r.violations
                          if v.ref == 'J1' and 'seated' in v.message])


class Coverage(_Boards):
    """Branches a verifier's extra mutations showed nothing else pinned."""

    def test_emitter_widens_a_band_only_to_cover_the_body(self):
        # Pad copper 0.25 mm inside the west edge (an occupancy reading of
        # 0.30 at the 0.55 margin) under a Fab body 1.5 mm over it. (esp_prog's
        # translated USB1 cannot serve: the translation makes it a SUSPECT
        # pad-legality entry, which is emitted with no edge at all.)
        path = self.synthetic('widen.kicad_pcb',
                              '(fp_rect (start -2 -1) (end 1 1) (layer "F.Fab"))',
                              at='0.5 10 0')
        pcb = parse_kicad_pcb(str(path))
        doc = floorplan.emit_intent(pcb, str(path))
        j1 = [e for e in doc['edge_connectors'] if e['ref'] == 'J1']
        self.assertEqual(len(j1), 1, doc['edge_connectors'])
        self.assertEqual(j1[0].get('edge'), 'west', j1[0])
        self.assertAlmostEqual(j1[0]['overhang_mm']['max'], 2.0, places=3)
        self.assertIn("drawn body's", j1[0].get('note', ''))
        # ... and the emitted intent still grades the band clean.
        r = floorplan.grade(floorplan.intent_from_dict(doc), pcb, str(path))
        self.assertEqual(_evidence(r, 'J1')['overhang_basis'], 'body:F.Fab')
        self.assertFalse(_overhang_violations(r, 'J1'), [
            v.message for v in _overhang_violations(r, 'J1')])

    def test_copper_evidence_is_per_part(self):
        from placement.legality import grade_pad_edge_clearance
        from copy import copy
        path = self.usb1_at(-1.45)
        pcb = parse_kicad_pcb(str(path))
        other = next(ref for ref in sorted(pcb.footprints)
                     if ref not in ('USB1',) and pcb.footprints[ref].pads
                     and not ref.startswith(('Ref', 'REF', '#')))
        intent = floorplan.intent_from_dict({
            'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
            'units': 'mm', 'edge_connectors': [
                {'ref': 'USB1', 'edge': 'west',
                 'overhang_mm': {'min': 0.0, 'max': 2.0}},
                {'ref': other, 'edge': 'east',
                 'overhang_mm': {'min': 0.0, 'max': 99.0}}]})
        r = floorplan.grade(intent, pcb, str(path), clearance=.25,
                            board_edge_clearance=.25)
        mine = _evidence(r, other)['pad_copper_edge']
        usb1 = _evidence(r, 'USB1')['pad_copper_edge']
        self.assertTrue(usb1['findings'])
        self.assertTrue(all(f['pad_ref'].startswith(other + '.')
                            for f in mine['findings'] + mine['unmeasured']))
        alone = copy(pcb)
        alone.footprints = {other: pcb.footprints[other]}
        direct = grade_pad_edge_clearance(alone, .25, str(path))
        self.assertAlmostEqual(mine['minimum_gap_mm'],
                               round(direct['minimum_gap_mm'], 4), places=4)
        self.assertNotAlmostEqual(mine['minimum_gap_mm'],
                                  usb1['minimum_gap_mm'], places=3)
        text = floorplan.format_text(r)
        self.assertIn('edge connector overhang', text)
        self.assertIn('USB1 west: overhang 1.4500mm [body:F.Fab]', text)

    def test_back_side_body_is_read_from_its_own_layer(self):
        path = self.root / 'bside.kicad_pcb'
        path.write_text(
            '(kicad_pcb (version 20241229) (generator "t961")\n'
            '  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))\n'
            '  (footprint "t" (layer "B.Cu") (at 5 10 0)\n'
            '    (property "Reference" "J1")\n'
            '    (fp_rect (start -1 -1) (end 1 1) (layer "B.Fab"))\n'
            '    (fp_rect (start -3 -3) (end 3 3) (layer "F.Fab"))\n'
            '    (pad "1" smd rect (at 0 0) (size .5 .5) (layers "B.Cu"))))\n',
            encoding='utf-8')
        pcb = parse_kicad_pcb(str(path))
        rect, layer, _ = ConnectorGeometry(pcb, str(path)).rect('J1')
        self.assertEqual(layer, 'B.Fab')
        for a, b in zip(rect, (4.0, 9.0, 6.0, 11.0)):
            self.assertAlmostEqual(a, b, places=6)

    def test_a_marker_is_not_a_body(self):
        """A closed convex pin-1 triangle is not the part's outline."""
        pads = ('(pad "1" smd rect (at 0 0) (size .5 .5) (layers "F.Cu"))\n'
                '    (pad "2" smd rect (at 4 0) (size .5 .5) (layers "F.Cu"))')
        path = self.synthetic(
            'marker.kicad_pcb',
            '(fp_poly (pts (xy -.6 -.6) (xy 0 -.6) (xy -.6 0)) (layer "F.Fab"))',
            at='1.25 10 0', pads=pads)
        pcb = parse_kicad_pcb(str(path))
        row = ConnectorGeometry(pcb, str(path)).measure('J1', 'west')
        self.assertFalse(row['body_measured'])
        self.assertIn('marker', row['body_unmeasured_reason'])
        # The same triangle drawn around both pads is a body.
        path = self.synthetic(
            'marker_ok.kicad_pcb',
            '(fp_poly (pts (xy -1 -1) (xy 6 -1) (xy -1 6)) (layer "F.Fab"))',
            at='1.25 10 0', pads=pads)
        pcb = parse_kicad_pcb(str(path))
        self.assertTrue(ConnectorGeometry(pcb, str(path)).measure(
            'J1', 'west')['body_measured'])

    def test_unquoted_layer_token(self):
        path = self.synthetic('unquoted.kicad_pcb',
                              '(fp_rect (start -1 -1) (end 1 1) (layer F.Fab))')
        pcb = parse_kicad_pcb(str(path))
        self.assertTrue(ConnectorGeometry(pcb, str(path)).measure(
            'J1', 'west')['body_measured'])


class Round2(_Boards):
    """The round-2 review's regression, and branches nothing pinned."""

    def _grade_j1(self, path, band, budget=None):
        extra = {'_intent': {'legality_budget': budget}} if budget else {}
        return floorplan.grade(
            _intent(ref='J1', edge='west', overhang_mm=band, **extra),
            parse_kicad_pcb(str(path)), str(path),
            clearance=.25, board_edge_clearance=.55)

    def test_pad_copper_off_the_outline_is_never_licensed(self):
        """Body flush with the edge, one pad 1.0 mm past it. The body band
        passes; the copper must still fail the rule AND stay counted."""
        body = '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))'
        pads = ('(pad "1" smd rect (at -1.75 0) (size .5 .5) (layers "F.Cu"))\n'
                '    (pad "2" smd rect (at .5 0) (size .5 .5) (layers "F.Cu"))')
        path = self.synthetic('copper_off.kicad_pcb', body, at='1 10 0',
                              pads=pads)
        r = self._grade_j1(path, {'min': 0.0, 'max': 0.5}, {'oob_count': 0})
        self.assertEqual(_evidence(r, 'J1')['overhang_basis'], 'body:F.Fab')
        self.assertFalse(_overhang_violations(r, 'J1'))
        hits = [v for v in r.violations
                if v.ref == 'J1' and 'pad copper leaves' in v.message]
        self.assertEqual(len(hits), 1)
        self.assertAlmostEqual(hits[0].measured['outside_mm'], 1.0, places=4)
        self.assertEqual(r.legality.get('oob_count_exempt'), 0)
        self.assertFalse(r.passed)
        # A CASTELLATED pad straddles the outline by design.
        pads = ('(pad "1" thru_hole circle (at -1.75 0) (size .5 .5) '
                '(drill .3) (layers "*.Cu") (property pad_prop_castellated))\n'
                '    (pad "2" smd rect (at .5 0) (size .5 .5) (layers "F.Cu"))')
        path = self.synthetic('castellated.kicad_pcb', body, at='1 10 0',
                              pads=pads)
        r = self._grade_j1(path, {'min': 0.0, 'max': 0.5}, {'oob_count': 0})
        self.assertFalse([v for v in r.violations
                          if v.ref == 'J1' and 'pad copper leaves' in v.message])
        self.assertEqual(r.legality.get('oob_count_exempt'), 1)

    def test_exemption_needs_the_census_to_have_counted_the_part(self):
        path = self.usb1_at(-0.10)
        r = floorplan.grade(
            _intent(ref='USB1', edge='west', overhang_mm={'min': 0.0, 'max': 1.0},
                    _intent={'legality_budget': {'oob_count': 99}}),
            parse_kicad_pcb(str(path)), str(path),
            clearance=.25, board_edge_clearance=.55)
        # The body is 0.10 over and inside the band, but the pad box is
        # 1.5 mm inboard, so the census never counted USB1: nothing to exempt.
        self.assertAlmostEqual(_evidence(r)['overhang_mm'], 0.10, places=4)
        self.assertEqual(r.legality.get('oob_count_exempt'), 0)

    def test_unusable_fab_does_not_fall_back_to_silk(self):
        path = self.synthetic(
            'fab_open.kicad_pcb',
            '(fp_line (start -1 -1) (end 1 -1) (layer "F.Fab")) '
            '(fp_rect (start -1 -1) (end 1 1) (layer "F.SilkS"))')
        row = ConnectorGeometry(parse_kicad_pcb(str(path)), str(path)).measure(
            'J1', 'west')
        self.assertFalse(row['body_measured'])
        self.assertEqual(row['body_layer'], 'F.Fab')

    def test_a_cutout_makes_the_boundary_unmeasured(self):
        path = self.root / 'cutout.kicad_pcb'
        path.write_text(
            '(kicad_pcb (version 20241229) (generator "t961")\n'
            '  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))\n'
            '  (gr_circle (center 10 15) (end 11 15) (layer "Edge.Cuts"))\n'
            '  (footprint "t" (layer "F.Cu") (at 5 10 0)\n'
            '    (property "Reference" "J1")\n'
            '    (fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))\n'
            '    (pad "1" smd rect (at 0 0) (size .5 .5) (layers "F.Cu"))))\n',
            encoding='utf-8')
        row = ConnectorGeometry(parse_kicad_pcb(str(path)), str(path)).measure(
            'J1', 'west')
        self.assertFalse(row['body_measured'])
        self.assertIn('boundary', row['body_unmeasured_reason'])

    def test_a_text_box_is_not_body_geometry(self):
        path = self.synthetic(
            'textbox.kicad_pcb',
            '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab")) '
            '(fp_text_box "VAL" (start -3 -3) (end 3 3) (layer "F.Fab") '
            '(effects (font (size 1 1) (thickness .15))))')
        row = ConnectorGeometry(parse_kicad_pcb(str(path)), str(path)).measure(
            'J1', 'west')
        self.assertTrue(row['body_measured'])
        self.assertAlmostEqual(row['body_setback_mm'], 4.0, places=6)

    def test_emitter_widens_on_the_summed_body(self):
        """West overhang 0.2 and north 0.3 (sum 0.5) under an occupancy
        reading of 0.30: the declared edge alone would not widen."""
        path = self.synthetic(
            'widen_sum.kicad_pcb',
            '(fp_rect (start -1.2 -.8) (end 1 1.6) (layer "F.Fab"))',
            at='1.0 0.5 0',
            pads='(pad "1" smd rect (at -.5 1.2) (size .5 .5) (layers "F.Cu"))')
        pcb = parse_kicad_pcb(str(path))
        doc = floorplan.emit_intent(pcb, str(path))
        j1 = [e for e in doc['edge_connectors'] if e['ref'] == 'J1']
        self.assertEqual(len(j1), 1, doc['edge_connectors'])
        self.assertEqual(j1[0].get('edge'), 'west', j1[0])
        self.assertAlmostEqual(j1[0]['overhang_mm']['max'], 1.0, places=3)
        self.assertIn("drawn body's", j1[0].get('note', ''))

    def test_second_rung_moves_the_body_onto_target_on_every_edge(self):
        from placement.connector_geometry import geometry_for
        path = self.synthetic('rung.kicad_pcb',
                              '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))',
                              at='10 10 0')
        pcb = parse_kicad_pcb(str(path))
        st = pose_score.make_state(pcb, str(path), clearance=.25,
                                   board_edge_clearance=.55)
        geo = geometry_for(st, st.pcb_data, st.pcb_file)
        rot = st.parts['J1'].rot
        band = (0.25, 0.35)
        for edge, start, want in (
                ('west', (0.2, 10.0), (0.7, 10.0)),
                ('east', (19.8, 10.0), (19.3, 10.0)),
                ('north', (10.0, 0.2), (10.0, 0.7)),
                ('south', (10.0, 19.8), (10.0, 19.3)),
                # An INBOARD start is below `lo` and must move out too.
                ('west', (1.5, 10.0), (0.7, 10.0))):
            x, y, ok = seeder._body_band_correct(st, 'J1', edge, *start,
                                                 0.3, band)
            self.assertTrue(ok, (edge, start))
            self.assertAlmostEqual(x, want[0], places=6, msg=(edge, start))
            self.assertAlmostEqual(y, want[1], places=6, msg=(edge, start))
            self.assertAlmostEqual(geo.measure('J1', edge, (x, y, rot))[
                'body_outside_mm'], 0.3, places=6)
        # A corner the rung cannot resolve (0.5 over the north edge already)
        # is reported as not converged, never claimed.
        _x, _y, ok = seeder._body_band_correct(st, 'J1', 'west', 0.2, 0.5,
                                               0.3, band)
        self.assertFalse(ok)

    def test_seat_edge_ladder_seats_on_the_body_band(self):
        """A body reaching 3 mm west of its only pad: the occupancy walk
        puts the pad off the board, so only the body rung can seat it."""
        from placement.connector_geometry import geometry_for
        path = self.synthetic('seat.kicad_pcb',
                              '(fp_rect (start -3 -1) (end 1 1) (layer "F.Fab"))',
                              at='10 10 0')
        pcb = parse_kicad_pcb(str(path))
        st = pose_score.make_state(pcb, str(path), clearance=.25,
                                   board_edge_clearance=.55)
        notes = []
        entry = {'ref': 'J1', 'edge': 'west',
                 'overhang_mm': {'min': 0.9, 'max': 1.1}}
        self.assertTrue(seeder._seat_edge(st, 'J1', dict(entry), set(), notes),
                        notes)
        p = st.parts['J1']
        self.assertAlmostEqual(geometry_for(st, st.pcb_data, st.pcb_file).measure(
            'J1', 'west', (p.x, p.y, p.rot))['body_outside_mm'], 1.0, delta=0.02)

    def test_stage_one_seats_on_the_body_band(self):
        """esp_prog, USB1 declared west {1.25, 1.35}: stage 1's slide and its
        final walk both take the body rung. The pose is a change detector,
        measured at the commit that added it."""
        import random
        entry = {'ref': 'USB1', 'edge': 'west',
                 'overhang_mm': {'min': 1.25, 'max': 1.35}}
        res = seeder.seed_from_intent(
            parse_kicad_pcb(str(SOURCE)), str(SOURCE), _intent(**entry),
            random.Random(0), clearance=.25, board_edge_clearance=.55,
            seed_refs={'USB1'})
        usb1 = [q for q in res['placements'] if q['reference'] == 'USB1']
        self.assertEqual(len(usb1), 1, res.get('notes'))
        self.assertAlmostEqual(usb1[0]['new_x'], 116.2, places=3)
        self.assertAlmostEqual(usb1[0]['new_y'], 98.25, places=3)


class Round3(_Boards):
    """The round-3 review: the seat predicate's copper, and the branches its
    own mutations showed nothing pinned."""

    def _state(self, path):
        return pose_score.make_state(parse_kicad_pcb(str(path)), str(path),
                                     clearance=.25, board_edge_clearance=.55)

    def test_seat_refuses_a_pose_whose_pad_copper_is_off_the_board(self):
        """The band reads the body, which carries no copper, so the seat has
        to: otherwise it accepts a pose `rule_edge_connector` refuses."""
        path = self.synthetic(
            'seat_copper.kicad_pcb',
            '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))', at='1 10 0',
            pads='(pad "1" smd rect (at -1.5 0) (size .5 .5) (layers "F.Cu"))\n'
                 '    (pad "2" smd rect (at .5 0) (size .5 .5) (layers "F.Cu"))')
        st = self._state(path)
        part = st.parts['J1']
        reasons = []
        # Body flush with the west edge (band 0..0.5 accepts it), pad copper
        # 0.75 mm past that edge.
        self.assertFalse(seeder.edge_seat_ok(st, part, 1.0, 10.0, 'west',
                                             0.0, 0.5, reasons))
        self.assertTrue([r for r in reasons if 'pad copper' in r], reasons)
        # 1 mm further in, every pad is on the board and the seat holds.
        self.assertTrue(seeder.edge_seat_ok(st, part, 2.0, 10.0, 'west',
                                            0.0, 0.5))
        # ... and the grade agrees with the seat at that pose.
        out = self.root / 'seat_copper_ok.kicad_pcb'
        write_placed_output(str(path), str(out), [
            {'reference': 'J1', 'new_x': 2.0, 'new_y': 10.0,
             'new_rotation': part.rot}])
        r = floorplan.grade(
            _intent(ref='J1', edge='west', overhang_mm={'min': 0.0, 'max': 0.5}),
            parse_kicad_pcb(str(out)), str(out),
            clearance=.25, board_edge_clearance=.55)
        self.assertFalse([v for v in r.violations if v.ref == 'J1'],
                         [v.message for v in r.violations])

    def test_the_marker_check_is_taken_at_the_footprint_rotation(self):
        pads = ('(pad "1" smd rect (at 0 0) (size .5 .5) (layers "F.Cu"))\n'
                '    (pad "2" smd rect (at 4 0) (size .5 .5) (layers "F.Cu"))')
        marker = ('(fp_poly (pts (xy -.6 -.6) (xy 0 -.6) (xy -.6 0)) '
                  '(layer "F.Fab"))')
        body = ('(fp_poly (pts (xy -1 -1) (xy 6 -1) (xy -1 6)) '
                '(layer "F.Fab"))')
        for rot in (0, 90, 180, 270):
            for drawing, measured in ((marker, False), (body, True)):
                path = self.synthetic(f'rot{rot}_{measured}.kicad_pcb',
                                      drawing, at=f'8 9 {rot}', pads=pads)
                row = ConnectorGeometry(parse_kicad_pcb(str(path)),
                                        str(path)).measure('J1', 'west')
                self.assertEqual(row['body_measured'], measured, (rot, drawing))

    def test_pads_that_carry_no_copper_do_not_decide_the_marker_check(self):
        # An NPTH hole outside the body, a copper pad inside it: the copper
        # pad is what the envelope must enclose.
        path = self.synthetic(
            'npth.kicad_pcb',
            '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))', at='5 10 0',
            pads='(pad "" np_thru_hole circle (at 6 0) (size 1 1) (drill 1) '
                 '(layers "F&B.Cu" "*.Mask"))\n'
                 '    (pad "1" smd rect (at 0 0) (size .5 .5) (layers "F.Cu"))')
        row = ConnectorGeometry(parse_kicad_pcb(str(path)),
                                str(path)).measure('J1', 'west')
        self.assertTrue(row['body_measured'], row.get('body_unmeasured_reason'))
        # A part with no pads at all is not checked.
        path = self.synthetic(
            'padless.kicad_pcb',
            '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))', at='5 10 0',
            pads='')
        self.assertTrue(ConnectorGeometry(parse_kicad_pcb(str(path)), str(path))
                        .measure('J1', 'west')['body_measured'])

    def test_the_copper_conjunct_follows_the_body_path(self):
        """The conjunct is body-scoped, so the legacy path grades exactly as
        it did before #961 -- even here, where that path's own reading (a
        courtyard that does not enclose its pads) cannot see the copper
        either. Pinning the gap, not endorsing it: `check_drc` names it."""
        path = self.synthetic(
            'copper_legacy.kicad_pcb',
            '(fp_rect (start -1 -1) (end 1 1) (layer "F.CrtYd"))', at='1 10 0',
            pads='(pad "1" smd rect (at -1.5 0) (size .5 .5) (layers "F.Cu"))')
        r = floorplan.grade(
            _intent(ref='J1', edge='west', overhang_mm={'min': 0.0, 'max': 5.0}),
            parse_kicad_pcb(str(path)), str(path),
            clearance=.25, board_edge_clearance=.55)
        self.assertTrue(_evidence(r, 'J1')['overhang_basis'].startswith(
            'legacy_occupancy@'))
        self.assertAlmostEqual(
            _evidence(r, 'J1')['pad_copper_edge']['outside_mm'], 0.75, places=4)
        self.assertFalse([v for v in r.violations
                          if v.ref == 'J1' and 'pad copper leaves' in v.message])

    def test_two_parts_on_one_board_get_their_own_verdicts(self):
        path = self.root / 'two_parts.kicad_pcb'
        pads = ('(pad "1" smd rect (at 0 0) (size .5 .5) (layers "F.Cu"))\n'
                '    (pad "2" smd rect (at 4 0) (size .5 .5) (layers "F.Cu"))')
        path.write_text(
            '(kicad_pcb (version 20241229) (generator "t961")\n'
            '  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))\n'
            '  (footprint "t" (layer "F.Cu") (at 8 9 0)\n'
            '    (property "Reference" "J1")\n'
            '    (fp_poly (pts (xy -.6 -.6) (xy 0 -.6) (xy -.6 0)) '
            '(layer "F.Fab"))\n'
            f'    {pads})\n'
            '  (footprint "t" (layer "F.Cu") (at 8 15 0)\n'
            '    (property "Reference" "J2")\n'
            '    (fp_poly (pts (xy -1 -1) (xy 6 -1) (xy -1 6)) (layer "F.Fab"))\n'
            f'    {pads}))\n', encoding='utf-8')
        geo = ConnectorGeometry(parse_kicad_pcb(str(path)), str(path))
        self.assertFalse(geo.measure('J1', 'west')['body_measured'])
        self.assertTrue(geo.measure('J2', 'west')['body_measured'])

    def test_the_enclosure_verdict_is_per_board(self):
        pads = ('(pad "1" smd rect (at 0 0) (size .5 .5) (layers "F.Cu"))\n'
                '    (pad "2" smd rect (at 4 0) (size .5 .5) (layers "F.Cu"))')
        marker = self.synthetic(
            'cache_marker.kicad_pcb',
            '(fp_poly (pts (xy -.6 -.6) (xy 0 -.6) (xy -.6 0)) (layer "F.Fab"))',
            at='8 9 0', pads=pads)
        body = self.synthetic(
            'cache_body.kicad_pcb',
            '(fp_poly (pts (xy -1 -1) (xy 6 -1) (xy -1 6)) (layer "F.Fab"))',
            at='8 9 0', pads=pads)
        for first, second in ((marker, body), (body, marker)):
            a = ConnectorGeometry(parse_kicad_pcb(str(first)), str(first))
            b = ConnectorGeometry(parse_kicad_pcb(str(second)), str(second))
            self.assertEqual(a.measure('J1', 'west')['body_measured'],
                             first is body)
            self.assertEqual(b.measure('J1', 'west')['body_measured'],
                             second is body)

    def test_evidence_reaches_the_json_and_keeps_the_row_it_annotates(self):
        path = self.usb1_at(-1.45)
        intent = floorplan.intent_from_dict({
            'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
            'units': 'mm', 'edge_connectors': [
                {'ref': 'USB1', 'edge': 'west',
                 'overhang_mm': {'min': 0.0, 'max': 2.0},
                 'center_on_edge': {'tolerance_mm': 0.5}}]})
        r = floorplan.grade(intent, parse_kicad_pcb(str(path)), str(path),
                            clearance=.25, board_edge_clearance=.55)
        doc = floorplan.to_json(r)
        self.assertEqual([e['ref'] for e in doc['edge_connector_evidence']],
                         ['USB1'])
        row = [e for e in r.edge_seating if e['ref'] == 'USB1'][0]
        # The along-edge row keeps its own keys AND gains the overhang ones.
        self.assertTrue(row['declared'])
        self.assertIn('along_edge_offset_pct', row)
        self.assertIn('span_mm', row)
        self.assertAlmostEqual(row['overhang_mm'], 1.45, places=4)
        self.assertAlmostEqual(row['effective_margin_mm'], 0.55, places=6)

    def test_an_entry_with_no_edge_is_unmeasured(self):
        path = self.usb1_at(-1.45)
        r = floorplan.grade(
            _intent(ref='USB1', overhang_mm={'min': 0.0, 'max': 2.0}),
            parse_kicad_pcb(str(path)), str(path),
            clearance=.25, board_edge_clearance=.55)
        ev = _evidence(r)
        self.assertTrue(ev['overhang_basis'].startswith('legacy_occupancy@'))
        self.assertIn('no declared mating edge', ev['body_unmeasured_reason'])

    def test_copper_dispositions_and_the_sampled_outline(self):
        # No copper pads at all: nothing to measure, and it says so.
        path = self.synthetic(
            'npth_only.kicad_pcb',
            '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))', at='5 10 0',
            pads='(pad "" np_thru_hole circle (at 0 0) (size 1 1) (drill 1) '
                 '(layers "F&B.Cu" "*.Mask"))')
        r = floorplan.grade(
            _intent(ref='J1', edge='west', overhang_mm={'min': 0.0, 'max': 5.0}),
            parse_kicad_pcb(str(path)), str(path),
            clearance=.25, board_edge_clearance=.55)
        self.assertEqual(_evidence(r, 'J1')['pad_copper_edge']['disposition'],
                         'no copper pads measured')
        # A sampled (non-rectangular) outline: the copper amount is UNKNOWN,
        # never reported as zero, and the body is unmeasured there anyway.
        path = self.root / 'sampled.kicad_pcb'
        path.write_text(
            '(kicad_pcb (version 20241229) (generator "t961")\n'
            '  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))\n'
            '  (gr_circle (center 10 15) (end 11 15) (layer "Edge.Cuts"))\n'
            '  (footprint "t" (layer "F.Cu") (at 0.1 10 0)\n'
            '    (property "Reference" "J1")\n'
            '    (fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))\n'
            '    (pad "1" smd rect (at 0 0) (size .5 .5) (layers "F.Cu"))))\n',
            encoding='utf-8')
        r = floorplan.grade(
            _intent(ref='J1', edge='west', overhang_mm={'min': 0.0, 'max': 5.0}),
            parse_kicad_pcb(str(path)), str(path),
            clearance=.25, board_edge_clearance=.55)
        copper = _evidence(r, 'J1')['pad_copper_edge']
        # The sampler still FINDS the pad, so the disposition is a fail; what
        # it cannot say is by how much, and that is reported as unknown
        # rather than as zero.
        self.assertEqual(copper['disposition'], 'fail')
        self.assertIsNone(copper['outside_mm'])
        self.assertTrue(copper['unmeasured'])
        self.assertTrue(_evidence(r, 'J1')['overhang_basis'].startswith(
            'legacy_occupancy@'))

    def test_only_the_declared_connectors_pads_are_walked(self):
        """`measured_pads` is the grade's own count over the DECLARED
        connectors -- one number, repeated on every row, never the board's."""
        path = self.usb1_at(0.0)
        pcb = parse_kicad_pcb(str(path))
        from placement.legality import _pad_has_no_copper
        board = sum(1 for fp in pcb.footprints.values() for p in fp.pads
                    if not _pad_has_no_copper(p))
        mine = sum(1 for p in pcb.footprints['USB1'].pads
                   if not _pad_has_no_copper(p))
        r = floorplan.grade(
            _intent(ref='USB1', edge='west', overhang_mm={'min': 0.0, 'max': 2.0}),
            pcb, str(path), clearance=.25, board_edge_clearance=.55)
        walked = _evidence(r)['pad_copper_edge']['measured_pads']
        self.assertLessEqual(walked, mine)
        self.assertLess(walked, board)
        # A second declared connector raises that one count, and BOTH rows
        # carry it: a per-part reading would differ between them.
        other = next(ref for ref in sorted(pcb.footprints)
                     if ref != 'USB1' and pcb.footprints[ref].pads
                     and not ref.startswith(('Ref', 'REF', '#')))
        two = floorplan.grade(floorplan.intent_from_dict({
            'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
            'units': 'mm', 'edge_connectors': [
                {'ref': 'USB1', 'edge': 'west',
                 'overhang_mm': {'min': 0.0, 'max': 2.0}},
                {'ref': other, 'edge': 'east',
                 'overhang_mm': {'min': 0.0, 'max': 99.0}}]}),
            pcb, str(path), clearance=.25, board_edge_clearance=.55)
        counts = {e['ref']: e['pad_copper_edge']['measured_pads']
                  for e in two.edge_connector_evidence}
        self.assertEqual(set(counts), {'USB1', other})
        self.assertEqual(counts['USB1'], counts[other])
        self.assertGreater(counts['USB1'], walked)
        self.assertLess(counts['USB1'], board)

    def test_per_part_minimum_gap_is_the_smallest_of_its_pads(self):
        from placement.legality import grade_pad_edge_clearance
        path = self.synthetic(
            'gaps.kicad_pcb',
            '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))', at='5 10 0',
            pads='(pad "1" smd rect (at -2 0) (size .5 .5) (layers "F.Cu"))\n'
                 '    (pad "2" smd rect (at 4 0) (size .5 .5) (layers "F.Cu"))')
        graded = grade_pad_edge_clearance(parse_kicad_pcb(str(path)), 0.25,
                                          str(path))
        # pad 1 sits 2.75 mm from the west edge, pad 2 sits 8.75 mm from it.
        self.assertAlmostEqual(graded['minimum_gap_by_ref_mm']['J1'], 2.75,
                               places=4)

    def test_the_rung_leaves_an_unmeasured_body_where_the_walk_put_it(self):
        path = self.synthetic(
            'rung_courtyard.kicad_pcb',
            '(fp_rect (start -1 -1) (end 1 1) (layer "F.CrtYd"))', at='1 10 0')
        st = self._state(path)
        x, y, ok = seeder._body_band_correct(st, 'J1', 'west', 1.0, 10.0,
                                             0.3, (0.25, 0.35))
        self.assertTrue(ok)
        self.assertEqual((x, y), (1.0, 10.0))


class LegacyFallback(_Boards):
    def _legacy(self, path, clr=.25, edge=.55):
        pcb = parse_kicad_pcb(str(path))
        st = pose_score.make_state(pcb, str(path), clearance=clr,
                                   board_edge_clearance=edge)
        return st.edge_gate.rect_outside_amount(st.parts['J1'].rect())

    def test_unmeasured_bodies_grade_on_the_legacy_reading(self):
        cases = {
            'courtyard.kicad_pcb': '(fp_rect (start -1 -1) (end 1 1) '
                                   '(layer "F.CrtYd"))',
            'pads_only.kicad_pcb': '',
            'arc.kicad_pcb': '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab")) '
                             '(fp_circle (center 0 0) (end .2 0) '
                             '(layer "F.Fab"))',
        }
        for name, body in cases.items():
            for at in ('0.3 10 0', '0.9 10 0'):
                path = self.synthetic(name, body, at=at)
                legacy = self._legacy(path)
                for band in ({'min': 0.0, 'max': .5}, {'min': .6, 'max': 2.0}):
                    r = floorplan.grade(
                        _intent(ref='J1', edge='west', overhang_mm=band),
                        parse_kicad_pcb(str(path)), str(path),
                        clearance=.25, board_edge_clearance=.55)
                    ev = _evidence(r, 'J1')
                    self.assertTrue(ev['overhang_basis'].startswith(
                        'legacy_occupancy@margin='), (name, ev))
                    self.assertTrue(ev['body_unmeasured_reason'])
                    self.assertAlmostEqual(ev['overhang_mm'], round(legacy, 4))
                    inside = band['min'] - EPS <= legacy <= band['max'] + EPS
                    self.assertEqual(not _overhang_violations(r, 'J1'), inside,
                                     (name, at, band, legacy))


class Geometry(_Boards):
    def test_rotation_and_missing_geometry(self):
        for drawing, measured in (
                ('(fp_poly (pts (xy 0 0) (xy 4 0) (xy 0 4)) (layer "F.Fab"))',
                 True),
                ('(fp_line (start 0 0) (end 4 0) (layer "F.Fab"))', False),
                ('(fp_circle (center 0 0) (end 1 0) (layer "F.Fab"))', False),
                # A rounded KiCad 10 rect: sharp corners would overstate it.
                ('(fp_rect (start 0 0) (end 4 4) (radius 0.5) (layer "F.Fab"))',
                 False),
                # An arc inside `pts` bulges past its chord. FOUR `xy` points,
                # so without the arc refusal this would measure as the chord
                # square (a two-point fixture is unmeasured either way, which
                # let that refusal be deleted with this test green).
                ('(fp_poly (pts (xy 0 0) (xy 4 0) (arc (start 4 0) (mid 5 2) '
                 '(end 4 4)) (xy 4 4) (xy 0 4)) (layer "F.Fab"))', False),
                ('', False)):
            path = self.synthetic('shape.kicad_pcb', drawing, at='2.5 10 45')
            pcb = parse_kicad_pcb(str(path))
            row = ConnectorGeometry(pcb, str(path)).measure('J1', 'west')
            self.assertEqual(row['body_measured'], measured, drawing)
            if measured:
                # The triangle's transformed VERTICES, not a rotated bbox.
                self.assertAlmostEqual(row['body_setback_mm'], 2.5)
                self.assertAlmostEqual(row['body_bounds_mm'][2],
                                       2.5 + math.sqrt(8))
            else:
                self.assertIsNone(row['body_overhang_mm'])
                self.assertTrue(row['body_unmeasured_reason'])

    def test_rewritten_file_is_not_answered_from_the_old_text(self):
        path = self.synthetic('same.kicad_pcb',
                              '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))')
        pcb = parse_kicad_pcb(str(path))
        self.assertTrue(ConnectorGeometry(pcb, str(path)).measure(
            'J1', 'west')['body_measured'])
        path = self.synthetic('same.kicad_pcb',
                              '(fp_line (start -1 -1) (end 1 -1) (layer "F.Fab"))')
        pcb = parse_kicad_pcb(str(path))
        self.assertFalse(ConnectorGeometry(pcb, str(path)).measure(
            'J1', 'west')['body_measured'])

    def test_agrees_with_the_902_drawn_body_ladder(self):
        """Where both readers measure a Fab rectangle they must agree; silk is
        where they legitimately part (the #902 ladder unions pads into it)."""
        for rot in (0, 90, 180, 270):
            path = self.synthetic(f'fab{rot}.kicad_pcb',
                                  '(fp_rect (start -1.5 -.5) (end 2.5 1) '
                                  '(layer "F.Fab"))', at=f'8 9 {rot}')
            pcb = parse_kicad_pcb(str(path))
            ours, layer, _ = ConnectorGeometry(pcb, str(path)).rect('J1')
            theirs, source = floorplan.drawn_body_rect(
                board_bodies(pcb, str(path)).get('J1'), pcb.footprints['J1'])
            self.assertEqual((layer, source), ('F.Fab', 'fab'))
            for a, b in zip(ours, theirs):
                self.assertAlmostEqual(a, b, places=6, msg=(rot, ours, theirs))
        path = self.synthetic('silk.kicad_pcb',
                              '(fp_rect (start -1 -1) (end 1 1) (layer "F.SilkS"))')
        pcb = parse_kicad_pcb(str(path))
        row = ConnectorGeometry(pcb, str(path)).measure('J1', 'west')
        self.assertEqual(row['body_layer'], 'F.SilkS')
        self.assertTrue(row['body_measured'])


class SeederAgreement(_Boards):
    def test_seat_and_grade_agree_on_the_body(self):
        src = self.usb1_at(0.0)
        pcb = parse_kicad_pcb(str(src))
        st = pose_score.make_state(pcb, str(src), clearance=.25,
                                   board_edge_clearance=.55)
        part = st.parts['USB1']
        x, y = 117.5 - 1.3, 100.0
        for band, seat in (((1.25, 1.35), True), ((0.0, .65), False)):
            self.assertEqual(seeder.edge_seat_ok(st, part, x, y, 'west', *band),
                             seat, band)
            written = self.root / f'seat_{band[1]}.kicad_pcb'
            write_placed_output(str(src), str(written), [
                {'reference': 'USB1', 'new_x': x, 'new_y': y,
                 'new_rotation': part.rot}])
            copy_siblings(str(src), str(written))
            r = _grade(written, {'min': band[0], 'max': band[1]}, edge=.55)
            self.assertEqual(not _overhang_violations(r), seat, band)

    def test_edge_correct_second_rung_reaches_the_body_target(self):
        src = self.usb1_at(0.0)
        pcb = parse_kicad_pcb(str(src))
        st = pose_score.make_state(pcb, str(src), clearance=.25,
                                   board_edge_clearance=.55)
        part = st.parts['USB1']
        x, y, ok = seeder._edge_correct(st, 'USB1', 'west', part.x, part.y,
                                        1.3, band=(1.25, 1.35))
        self.assertTrue(ok)
        from placement.connector_geometry import geometry_for
        row = geometry_for(st, st.pcb_data, st.pcb_file).measure(
            'USB1', 'west', (x, y, part.rot))
        self.assertAlmostEqual(row['body_overhang_mm'], 1.3, delta=0.02)
        # Without a band the walk is the upstream one, untouched.
        x0, y0, _ = seeder._edge_correct(st, 'USB1', 'west', part.x, part.y,
                                         1.3)
        self.assertAlmostEqual(st.edge_gate.rect_outside_amount(
            part.rect(x0, y0, part.rot)), 1.3, delta=0.02)


class Corpus(unittest.TestCase):
    def test_emitted_bands_and_other_edges_on_tracked_boards(self):
        """On the tracked corpus the emitter's widening never fires (every
        body-measured part reads no more than the occupancy reading), and no
        body-measured emitted connector crosses a second edge."""
        for name in CORPUS:
            path = ROOT / 'kicad_files' / f'{name}.kicad_pcb'
            pcb = parse_kicad_pcb(str(path))
            doc = floorplan.emit_intent(pcb, str(path))
            geometry = ConnectorGeometry(pcb, str(path))
            for entry in doc.get('edge_connectors', []):
                self.assertNotIn("drawn body's", entry.get('note', ''),
                                 (name, entry))
                if not entry.get('edge'):
                    continue
                row = geometry.measure(entry['ref'], entry['edge'])
                if row['body_measured']:
                    self.assertFalse(
                        [e for e, v in row['other_body_edge_overhang_mm'].items()
                         if v > EPS], (name, entry['ref'], row))


if __name__ == '__main__':
    unittest.main(verbosity=2)
