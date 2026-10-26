"""#1201: a pin-1 dot on F.Fab is not a part's body.

`body.body_geometry` accepted the .Fab rung unconditionally while silk had a
tick-mark guard, so an EasyEDA USB-C receptacle whose only F.Fab item is a
0.06 mm pin-1 circle had a 0.06 x 0.06 mm "body" at its inboard corner, and
`rule_edge_connector` graded its seat on that point: CM5_MINIMA_3's J101 and
USB701, declared on the west edge, "seated 8.90mm / 11.71mm from the nearest
edge" while their pads were 3.8 mm from it.

`body.fab_cannot_be_body` rejects a fab bbox under `FAB_MIN_PAD_AREA_FRAC`
(0.02) of the pad bbox's area. The DRAWN body falls through to silk united
with the pads (else none), the part records `fab_rejected`, and what it
OCCUPIES is unchanged: its courtyard, else the marking united with its pads.
The quench's own .Fab reader (`QuenchState.fab_rect` / `fab_shape`) honours
the same rejection, so the search and the grader read one body.

Silk containment stays advisory: a silk-drawn body never gates (#896). What
changes is that check_assembly now SAYS so per pair (`gate_exempt`), instead
of calling every non-gating containment "a marker or a board-sized container".
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import body, floorplan, legality              # noqa: E402
from run_utils import corpus_boards                          # noqa: E402
from test_1184_container_classification import _board        # noqa: E402

PY = sys.executable
CM5 = os.path.expanduser('~/Documents/prive/git/krt-fa10/wk/fa10/_truth/'
                         'CM5_MINIMA_3/human_stripped.kicad_pcb')
CM5_INTENT = os.path.expanduser('~/Documents/prive/git/krt-fa10/wk/fa10/'
                                '_issues/batch4/work/S8/intent_j101.json')


def _usb_c(x, y, rot):
    """J101's shape: 22 SMD pads over 7.3 x 4.1 mm, no courtyard, a single
    F.Fab item -- a 0.06 mm pin-1 circle at one corner -- and a silk shell
    that runs 2.2 mm past the pads on local +x (the opening)."""
    pads = ''.join(
        f'    (pad "{i}" smd rect (at {round(-3.5 + 0.7 * (i % 11), 2)} '
        f'{-1.55 if i < 11 else 1.55}) (size 0.3 1.0) (layers "F.Cu")'
        f' (net 1 "N1"))\n' for i in range(22))
    return (f'  (footprint "t:USB-C-SMD_TYPEC" (layer "F.Cu") '
            f'(at {x} {y} {rot})\n'
            '    (property "Reference" "J101" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -3.8 -2.6) (end 5.9 2.6) (stroke (width'
            ' 0.12) (type default)) (layer "F.SilkS"))\n'
            '    (fp_circle (center -3.95 -2.23) (end -3.92 -2.23) (stroke'
            ' (width 0.05) (type default)) (layer "F.Fab"))\n'
            + pads + '  )\n')


def _holder_silk(closed=True):
    """One-Air-Max BAT1's REAL shape (the comment on #1201): clips 40 mm
    apart, a silk outline running PAST the clips in y, and a pin-1 dot on
    F.Fab. (test_1184's `_holder` draws its silk inside the pads, so there
    the silk is a tick and the body falls to the pad box.) `closed=False`
    draws the same extent as four corner brackets, which never close (#1401)."""
    if closed:
        outline = ('    (fp_rect (start -21 -13) (end 21 13) (stroke (width 0.12)'
                   ' (type default)) (layer "F.SilkS"))\n')
    else:
        outline = ''.join(
            f'    (fp_line (start {sx * 21} {sy * 13}) (end {sx * 18} {sy * 13})'
            f' (stroke (width 0.12) (type default)) (layer "F.SilkS"))\n'
            f'    (fp_line (start {sx * 21} {sy * 13}) (end {sx * 21} {sy * 10})'
            f' (stroke (width 0.12) (type default)) (layer "F.SilkS"))\n'
            for sx in (-1, 1) for sy in (-1, 1))
    clips = ''.join(
        f'    (pad "{i}" smd rect (at {x} {y}) (size 3 4) (layers "F.Cu")'
        f' (net 1 "N1"))\n'
        for i, (x, y) in enumerate([(-20, -8), (-20, 0), (-20, 8),
                                    (20, -8), (20, 0), (20, 8)], 1))
    return ('  (footprint "t:HOLDER" (layer "F.Cu") (at 22 15)\n'
            '    (property "Reference" "BAT1" (at 0 0) (layer "F.SilkS"))\n'
            + outline +
            '    (fp_circle (center -18 -9) (end -17.97 -9) (stroke (width'
            ' 0.05) (type default)) (layer "F.Fab"))\n'
            + clips + '  )\n')


def _dc1(x, y):
    return (f'  (footprint "t:P" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "DC1" (at 0 0) (layer "F.SilkS"))\n'
            f'    (fp_rect (start -2 -2) (end 2 2) (stroke (width 0.05)'
            f' (type default)) (layer "F.CrtYd"))\n'
            f'    (fp_rect (start -1 -1) (end 1 1) (stroke (width 0.05)'
            f' (type default)) (layer "F.Fab"))\n'
            f'    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu")'
            f' (net 2 "N2")))\n')


class TheGuard(unittest.TestCase):

    def test_a_pin_one_dot_is_rejected_and_silk_answers(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, _usb_c(6, 15, 180), size=(40, 30))
            pcb = parse_kicad_pcb(path)
            g = body.board_bodies(pcb, path)['J101']
            lb = legality.part_local_bounds(pcb, path)['J101']
        self.assertTrue(g.fab_rejected)
        # ...and the provenance reaches every LocalBounds consumer.
        self.assertTrue(lb.fab_rejected)
        # Its DRAWN body is the silk, united with the pads, never the dot...
        self.assertEqual(g.drawn_source, body.SOURCE_SILK)
        self.assertIsNone(g.drawn_shape_local)
        self.assertLess(g.drawn_local[0], -3.7)
        self.assertGreater(g.drawn_local[2], 5.8)
        # ...but what it OCCUPIES is unchanged by the guard: the marking
        # united with the pads, as before, named `fab_marking` (a silk
        # occupancy never gates, #896, so silk may not answer it).
        dot = (-3.98, -2.26, -3.92, -2.2)
        self.assertEqual(g.source, body.SOURCE_FAB_MARKING)
        self.assertEqual(lb.source, body.SOURCE_FAB_MARKING)
        want = body._union(dot, g.pads_local)
        for got, exp in zip(g.occupancy_local, want):
            self.assertAlmostEqual(got, exp, places=6)
        # its BODY is the silk, never the dot
        self.assertEqual(g.body_local, g.drawn_local)

    def test_a_marking_without_silk_falls_to_the_pads(self):
        text = _usb_c(6, 15, 180).replace(
            '    (fp_rect (start -3.8 -2.6) (end 5.9 2.6) (stroke (width'
            ' 0.12) (type default)) (layer "F.SilkS"))\n', '')
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, text, size=(40, 30))
            pcb = parse_kicad_pcb(path)
            g = body.board_bodies(pcb, path)['J101']
            rect, src = floorplan.drawn_body_rect(g, pcb.footprints['J101'])
        self.assertTrue(g.fab_rejected)
        self.assertIsNone(g.drawn_local)
        self.assertEqual(g.source, body.SOURCE_FAB_MARKING)
        # no silk: its body is its pads, and a drawn-body reader says so
        self.assertEqual(g.body_local, g.pads_local)
        self.assertEqual(src, 'pad_bbox')

    def test_the_threshold_brackets(self):
        pads = (-2.0, -1.0, 2.0, 1.0)                        # 8 mm2
        f = body.fab_cannot_be_body
        frac = body.FAB_MIN_PAD_AREA_FRAC
        self.assertTrue(f((0, 0, 0.06, 0.06), pads))
        self.assertTrue(f((0, 0, 0.4, 0.38), pads))          # 0.019
        self.assertFalse(f((0, 0, 0.4, 0.42), pads))         # 0.021
        # exactly AT the threshold is a body: 1 x 2 over 10 x 10, and
        # 0.02 * 100 is exactly 2.0 in floating point
        self.assertEqual(frac * 100.0, 2.0)
        self.assertFalse(f((0, 0, 1.0, 2.0), (0, 0, 10.0, 10.0)))
        # a zero-width line is a marking (CM5 Module301's 0 x 48 mm fab)
        self.assertTrue(f((0, 0, 0.0, 48.0), (-20, -27, 20, 27)))
        self.assertFalse(f(None, pads))
        self.assertFalse(f((0, 0, 0.06, 0.06), None))
        # the stock library's least-covering real bodies sit above it
        self.assertLess(frac, 0.0537)
        self.assertGreater(frac, 0.00466)

    def test_a_padless_fab_body_is_kept(self):
        """watchy's e-paper display: no pads, a real F.Fab body."""
        disp = ('  (footprint "t:DISPLAY" (layer "F.Cu") (at 20 15)\n'
                '    (property "Reference" "REF**" (at 0 0)'
                ' (layer "F.SilkS"))\n'
                '    (fp_rect (start -10 -8) (end 10 8) (stroke (width'
                ' 0.1) (type default)) (layer "F.Fab")))\n')
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, disp, size=(40, 30))
            g = body.board_bodies(parse_kicad_pcb(path), path)['REF**']
        self.assertFalse(g.fab_rejected)
        self.assertEqual(g.drawn_source, body.SOURCE_FAB)

    def test_the_edge_seat_is_graded_on_the_body_not_the_dot(self):
        """The issue's shape: rot 180 puts the dot at the INBOARD corner
        (x 9.95) and the silk shell at the west edge (x 0.1)."""
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, _usb_c(6, 15, 180), size=(40, 30))
            doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
                   'units': 'mm',
                   'envelope': {'rect': [0, 0, 40, 30]},
                   'edge_connectors': [{'ref': 'J101', 'edge': 'west',
                                        'class': 'edge_receptacle'}]}
            r = floorplan.grade(
                floorplan.intent_from_dict(doc, source_path='i.json'),
                parse_kicad_pcb(path), path)
        errs = [v.message for v in r.errors if v.rule == 'edge_connector']
        self.assertEqual(errs, [])
        ev = [e for e in r.edge_connector_evidence if e.get('ref') == 'J101']
        self.assertTrue(ev)


class TheCorpus(unittest.TestCase):

    def test_no_tracked_part_is_rejected(self):
        boards = corpus_boards()
        if not boards:
            self.skipTest('git cannot list the tracked boards')
        import measure_1201_fab_and_silk as m
        low, rejected = None, []
        for path in boards:
            pcb = parse_kicad_pcb(path)
            rejected += [(os.path.basename(path), r) for r, g in
                         body.board_bodies(pcb, path).items()
                         if g.fab_rejected]
            for ref, ratio, _f, _p in m.ratio_rows(pcb, path):
                if low is None or ratio < low[0]:
                    low = (ratio, os.path.basename(path), ref)
        self.assertEqual(rejected, [])
        # The change detector the threshold rests on: the least-covering
        # tracked fab body, 0.3114 of its pads (orangecrab J4).
        self.assertEqual((round(low[0], 4), low[1], low[2]),
                         (0.3114, 'orangecrab_ext_pll.kicad_pcb', 'J4'))
        self.assertGreater(low[0], 6 * body.FAB_MIN_PAD_AREA_FRAC)


def _fab_part(ref, x, y, half, name='t:QFN', pads=4):
    """A part whose F.Fab body is a +-`half` square, with `pads` pads."""
    pad = ''.join(
        f'    (pad "{k}" smd rect (at {round(-half / 2 + k * 0.5, 2)} 0) '
        f'(size 0.3 0.6) (layers "F.Cu") (net 2 "N2"))\n'
        for k in range(pads))
    return (f'  (footprint "{name}" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            f'    (fp_rect (start {-half} {-half}) (end {half} {half}) (stroke'
            f' (width 0.05) (type default)) (layer "F.Fab"))\n'
            + pad + '  )\n')


def _fiducial(ref, x, y):
    return (f'  (footprint "Fiducial:Fiducial_1mm_Mask2mm" (layer "F.Cu") '
            f'(at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            f'    (fp_circle (center 0 0) (end 1 0) (stroke (width 0.05)'
            f' (type default)) (layer "F.Fab"))\n'
            f'    (pad "1" smd circle (at 0 0) (size 1 1) (layers "F.Cu")))\n')


def _check_assembly(td, path):
    out_json = os.path.join(td, 'a.json')
    p = subprocess.run(
        [PY, '-X', 'utf8', 'py_tools/check_assembly.py', path,
         '--json', out_json],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT, timeout=600)
    with open(out_json, encoding='utf-8') as fh:
        return p.stdout + p.stderr, json.load(fh)


class TheQuenchReadsTheSameBody(unittest.TestCase):

    def _state(self, td, text):
        import pose_score
        path = _board(td, text, size=(40, 30))
        return pose_score.make_state(parse_kicad_pcb(path), path,
                                     clearance=0.2)

    def test_fab_rect_and_shape_refuse_the_dot(self):
        with tempfile.TemporaryDirectory() as td:
            st = self._state(td, _usb_c(6, 15, 180) + _dc1(30, 15))
            self.assertIsNone(st.fab_rect('J101'))
            self.assertIsNone(st.fab_shape('J101'))
            # A real fab body is still read, exactly as before.
            self.assertIsNotNone(st.fab_rect('DC1'))
            self.assertEqual(st.fab_rect('DC1'), (29.0, 14.0, 31.0, 16.0))

    def test_body_less_means_what_the_grader_means(self):
        """`_bodyless_refs` reads the grader's drawn body: J101's silk is a
        body (not body-less); a twin that draws only the dot is body-less,
        so the pads-under-body channel judges it, as the grader does."""
        bare = _usb_c(30, 15, 0).replace('"J101"', '"J102"').replace(
            '    (fp_rect (start -3.8 -2.6) (end 5.9 2.6) (stroke (width'
            ' 0.12) (type default)) (layer "F.SilkS"))\n', '')
        with tempfile.TemporaryDirectory() as td:
            st = self._state(td, _usb_c(6, 15, 180) + bare)
            got = st._bodyless_refs()
        self.assertNotIn('J101', got)
        self.assertIn('J102', got)


class SilkContainmentIsNamed(unittest.TestCase):

    def test_bat1_shape(self):
        """One-Air-Max BAT1's real shape (#1201's comment). Its drawn body
        is its silk, drawn here as corner brackets that never close, so DC1
        wholly inside it is a containment that does not gate (#896) -- named
        as such -- while its OCCUPANCY is its pad box,
        so the courtyard channel still gates the pair, as it did before the
        guard. A fiducial inside it is exempt as a marker, and says so."""
        import pose_score
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, _holder_silk(closed=False) + _dc1(22, 15)
                          + _fiducial('FID1', 30, 15), size=(44, 30))
            pcb = parse_kicad_pcb(path)
            g = legality.grade_body_overlap(pcb, 0.2, pcb_file=path)
            st = pose_score.make_state(pcb, path, clearance=0.2)
            veto = st.candidate_veto('DC1', 22.0, 15.0, 0.0)
            out, js = _check_assembly(td, path)
        self.assertEqual(g['fab_rejected_refs'], ['BAT1'])
        self.assertEqual(g['body_sources'].get('BAT1'), body.SOURCE_SILK)
        why = {frozenset((q.a, q.b)): q.gate_exempt
               for q in g['containment_pairs']}
        self.assertEqual(why.get(frozenset(('BAT1', 'DC1'))), 'silk_drawn')
        # a marker's own exemption outranks the silk one
        self.assertEqual(why.get(frozenset(('BAT1', 'FID1'))), 'marker_class')
        self.assertEqual(g['containment_blocking'], 0)
        # the pad box gates the courtyard channel, as before the guard
        self.assertTrue([q for q in g['courtyard_blocking_pairs']
                         if {q.a, q.b} == {'BAT1', 'DC1'}])
        self.assertEqual((veto or (None,))[0], 'courtyard', veto)
        # one pair, one label, wherever it is listed
        for lst in ('pairs', 'advisory_pairs'):
            for q in g[lst]:
                if q.contained:
                    self.assertEqual(q.gate_exempt,
                                     why[frozenset((q.a, q.b))], (lst, q))
        self.assertIn('does not gate: a silk-drawn body', out)
        self.assertIn('does not gate: a mount-hole/fiducial/testpoint', out)
        self.assertIn('None of these BLOCK: 1 a mount-hole/fiducial/testpoint '
                      'is involved; 1 a silk-drawn body', out)
        self.assertNotIn('a marker or a board-sized container', out)
        self.assertIn('BAT1', out[out.find('.Fab marking'):])
        self.assertEqual(js['fab_rejected_refs'], ['BAT1'])
        self.assertEqual(sorted(c['gate_exempt'] for c in js['containments']),
                         ['marker_class', 'silk_drawn'])

    def test_a_closed_silk_body_gates(self):
        """#1401: the same holder with its silk CLOSED is a housing, and DC1's
        pad copper wholly inside it is listed as `silk_enclosed` -- a pair
        that gates where a member MOVED vs --baseline (check_assembly), not
        absolutely; the marker inside it is still exempt as a marker."""
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, _holder_silk() + _dc1(22, 15)
                          + _fiducial('FID1', 30, 15), size=(44, 30))
            g = legality.grade_body_overlap(parse_kicad_pcb(path), 0.2,
                                            pcb_file=path)
            out, _js = _check_assembly(td, path)
        why = {frozenset((q.a, q.b)): q.gate_exempt
               for q in g['containment_pairs']}
        self.assertEqual(why.get(frozenset(('BAT1', 'DC1'))), 'silk_enclosed')
        self.assertEqual(why.get(frozenset(('BAT1', 'FID1'))), 'marker_class')
        self.assertEqual({frozenset((q.a, q.b))
                          for q in g['silk_enclosed_pairs']},
                         {frozenset(('BAT1', 'DC1'))})
        self.assertEqual(g['containment_blocking_pairs'], [])

    def test_a_gating_containment_is_labelled_as_gating(self):
        """A part wholly inside another's FAB body gates; its label is '',
        and an edge-class waiver on it is named as one that still gates."""
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, _fab_part('U1', 15, 15, 6)
                          + _fab_part('R1', 15, 15, 1, pads=2)
                          + _fab_part('J9', 32, 15, 5,
                                      name='Connector_USB:USB_C_Receptacle_X')
                          + _fab_part('R2', 32, 15, 1, pads=2),
                          size=(44, 30))
            g = legality.grade_body_overlap(parse_kicad_pcb(path), 0.2,
                                            pcb_file=path)
            out, _js = _check_assembly(td, path)
        blk = {frozenset((q.a, q.b)): q
               for q in g['containment_blocking_pairs']}
        self.assertIn(frozenset(('U1', 'R1')), blk)
        self.assertEqual(blk[frozenset(('U1', 'R1'))].gate_exempt, '')
        self.assertIn(frozenset(('J9', 'R2')), blk)
        self.assertEqual(blk[frozenset(('J9', 'R2'))].waiver, 'edge_class')
        self.assertIn('[waived:edge_class, still gates]', out)

    def test_a_container_only_board_says_by_design(self):
        """watchy's e-paper display shape: a pad-less fab outline over most
        of the board. A part under it is a container containment, and the
        board-sized-container sentence is TRUE here, so it prints."""
        disp = ('  (footprint "t:DISPLAY" (layer "F.Cu") (at 22 12)\n'
                '    (property "Reference" "REF**" (at 0 0)'
                ' (layer "F.SilkS"))\n'
                '    (fp_rect (start -20 -10) (end 20 10) (stroke (width'
                ' 0.1) (type default)) (layer "F.Fab")))\n')
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, disp + _fab_part('U1', 22, 12, 1, pads=2))
            out, _js = _check_assembly(td, path)
        self.assertIn('a board-sized container is involved', out)
        self.assertIn('Each is a by-design containment', out)


class ConnectorGeometryReadsTheSameBody(unittest.TestCase):
    """#1243: the #961 band reader takes the guard's verdict too."""

    def _row(self, text):
        from placement.connector_geometry import ConnectorGeometry, band_amount
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, text, size=(40, 30))
            pcb = parse_kicad_pcb(path)
            geo = ConnectorGeometry(pcb, path)
            return band_amount(geo, 'J101', 'west', 9.0, 0.25)

    def test_a_dot_on_fab_measures_the_silk_shell(self):
        # at 180 deg the silk's local +x (5.9) faces west: x=5.8 is 0.1 over
        amount, basis, row = self._row(_usb_c(5.8, 15, 180))
        self.assertEqual(basis, 'body:F.SilkS', row)
        self.assertAlmostEqual(amount, 0.1, places=6)

    def test_a_dot_with_open_silk_stays_unmeasured(self):
        text = _usb_c(5.8, 15, 180).replace(
            '    (fp_rect (start -3.8 -2.6) (end 5.9 2.6) (stroke (width'
            ' 0.12) (type default)) (layer "F.SilkS"))\n',
            '    (fp_line (start -3.8 -2.6) (end 5.9 -2.6) (stroke (width'
            ' 0.12) (type default)) (layer "F.SilkS"))\n'
            '    (fp_line (start -3.8 2.6) (end 5.9 2.6) (stroke (width'
            ' 0.12) (type default)) (layer "F.SilkS"))\n')
        amount, basis, row = self._row(text)
        self.assertEqual(basis, 'legacy_occupancy@margin=0.25')
        self.assertEqual(amount, 9.0)
        self.assertEqual(row['body_layer'], 'F.SilkS')
        self.assertIn('open or concave', row['body_unmeasured_reason'])

    def test_a_dot_without_silk_stays_unmeasured(self):
        text = _usb_c(5.8, 15, 180).replace(
            '    (fp_rect (start -3.8 -2.6) (end 5.9 2.6) (stroke (width'
            ' 0.12) (type default)) (layer "F.SilkS"))\n', '')
        amount, basis, row = self._row(text)
        self.assertEqual(basis, 'legacy_occupancy@margin=0.25')
        self.assertIn('marking', row['body_unmeasured_reason'])
        # the layer it looked at and refused, not "no layer"
        self.assertEqual(row['body_layer'], 'F.Fab')


class TheIssueBoard(unittest.TestCase):

    def test_cm5_receptacles_are_graded_on_their_bodies(self):
        if not (os.path.isfile(CM5) and os.path.isfile(CM5_INTENT)):
            self.skipTest('the fa10 CM5_MINIMA_3 truth board is not on this '
                          'machine (it is not tracked)')
        p = subprocess.run(
            [PY, '-X', 'utf8', 'py_tools/check_floorplan.py', CM5,
             '--intent', CM5_INTENT, '--no-brief', '--no-mechanical'],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT, timeout=900)
        out = p.stdout + p.stderr
        # Positive evidence first: the run graded both receptacles and passed
        # (a crash before grading would print none of the strings below).
        self.assertEqual(p.returncode, 0, out[-1500:])
        self.assertIn('J101 west: overhang', out)
        self.assertIn('USB701 west: overhang', out)
        self.assertNotIn('body:fab', out)
        self.assertNotIn('seated 8.90mm', out)
        self.assertNotIn('seated 11.71mm', out)


if __name__ == '__main__':
    unittest.main()
