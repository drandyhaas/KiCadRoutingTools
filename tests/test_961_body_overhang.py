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
    def test_a_band_licenses_its_own_edge_only(self):
        """The occupancy reading summed every side it crossed, so a corner
        overhang counted against the band; on the body path a second edge is
        named instead -- when the band has a maximum, as the sum needed."""
        path = self.synthetic('corner.kicad_pcb',
                              '(fp_rect (start -1 -1) (end 1 1) (layer "F.Fab"))',
                              at='0.5 0.5 0')

        def crossing(band):
            r = floorplan.grade(_intent(ref='J1', edge='west', overhang_mm=band),
                                parse_kicad_pcb(str(path)), str(path),
                                clearance=.25, board_edge_clearance=.55)
            return [v for v in r.violations
                    if v.ref == 'J1' and 'also overhangs the north' in v.message]
        hits = crossing({'min': 0.0, 'max': 2.0})
        self.assertEqual(len(hits), 1)
        self.assertAlmostEqual(hits[0].measured['overhang_mm'], 0.5, places=4)
        self.assertFalse(crossing({'min': 0.0}))

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
