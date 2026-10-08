"""#1099: three placement gaps run 36 exposed on KiCad's StickHub demo.

The human placement routed by our own router reached blocking 4; run 36's
placement reached 24. The causes this file pins:

(a) No decoupling limit was armed. On a pile there is no cap-to-pin distance
    to read, so `--declare-decaps` withholds (or once wrote 0.0), and the
    skill never mentioned it. `check_floorplan --decaps-from <placed board>`
    reads the limit off a placed reference of the same design (the human
    board gives 2.18 mm from 38 tethers; run 36's hub decaps sat 2.1-9.8 mm
    from their pins).
(b) The seeder only tried 0/90/180/270. A part the 90-degree lattice seats
    nowhere now gets a second pass at the diagonals -- prefer, then fall
    back, so a part that fits orthogonally seats exactly as before.
(c) place_seed's exit-4 line did not name the unseated parts; the agent
    read "grade errors" and routed with C20 still in the pile.
"""

import json
import os
import random
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

PILE = os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
PLACED = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')

#: A 6 x 6 board and a 7 x 0.6 part parked off it. At 0/90/180/270 the part
#: is longer than the board; at 45 its rotated box is (7 + 0.6) / sqrt2 =
#: 5.37 mm square and fits. A 2 x 1 part fits at 0 and must seat as before.
BOARD = (
    '(kicad_pcb (version 20240108) (generator pcbnew)\n'
    '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
    '  (net 0 "") (net 1 "A") (net 2 "B")\n'
    '  (gr_rect (start 0 0) (end 6 6) (stroke (width 0.1) (type default))'
    ' (layer "Edge.Cuts"))\n'
    '{parts})\n')
PART = (
    '  (footprint "t:{ref}" (layer "F.Cu") (at {x} {y})\n'
    '    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
    '    (fp_rect (start -{hx} -{hy}) (end {hx} {hy}) (stroke (width 0.05)'
    ' (type default)) (layer "F.CrtYd"))\n'
    '    (pad "1" smd rect (at -{px} 0) (size 0.4 0.4) (layers "F.Cu")'
    ' (net 1 "A"))\n'
    '    (pad "2" smd rect (at {px} 0) (size 0.4 0.4) (layers "F.Cu")'
    ' (net 2 "B")))\n')


def seed(parts, diagonal):
    """Seed `parts` = [(ref, x, y, half_x, half_y, pad_x)] with no intent;
    returns {ref: (x, y, rot)} or None for unseated, and the result dict."""
    from kicad_parser import parse_kicad_pcb
    from placement import seeder
    from placement.floorplan import empty_intent
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(BOARD.format(parts=''.join(
                PART.format(ref=r, x=x, y=y, hx=hx, hy=hy, px=px)
                for r, x, y, hx, hy, px in parts)))
        res = seeder.seed_from_intent(
            parse_kicad_pcb(path), path, empty_intent(path), random.Random('1'),
            clearance=0.1, board_edge_clearance=0.1, grid_step=0.1,
            diagonal_rotations=diagonal)
    poses = {p[0]: tuple(p[1:4]) for p in res['placements']} \
        if res['placements'] and isinstance(res['placements'][0],
                                            (list, tuple)) else None
    return res, poses


class TestUnseatedNamed(unittest.TestCase):
    def test_the_exit_line_names_the_parts(self):
        import place_seed
        line = place_seed.gate_reason(['C20', 'C14', 'C16'], [], [], 0)
        self.assertIn('3 part(s) UNSEATED', line)
        self.assertIn('C14, C16, C20', line)
        self.assertIn('does NOT satisfy its intent', line)

    def test_grade_errors_alone_keep_their_line(self):
        import place_seed
        line = place_seed.gate_reason([], ['an error'], [], 0)
        self.assertNotIn('UNSEATED', line)
        self.assertIn('see the errors above', line)


class TestDecapsFrom(unittest.TestCase):
    def test_a_pile_takes_its_limit_from_the_placed_reference(self):
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'i.json')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), PILE,
                 '--allow-unplaced', '--emit-intent', out,
                 '--decaps-from', PLACED],
                capture_output=True, text=True, cwd=ROOT)
            self.assertTrue(os.path.isfile(out), r.stdout[-1500:]
                            + r.stderr[-1500:])
            with open(out, encoding='utf-8') as fh:
                doc = json.load(fh)
        census = doc['context']['decap_census']
        self.assertEqual(census['decaps_basis'], 'reference:esp_prog.kicad_pcb')
        self.assertEqual(doc['decaps']['max_distance_mm'],
                         census['emitted_max_distance_mm'])
        self.assertGreater(doc['decaps']['max_distance_mm'], 0.0)

    def _emit(self, board, *extra):
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'i.json')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), board,
                 '--allow-unplaced', '--emit-intent', out, *extra],
                capture_output=True, text=True, cwd=ROOT)
            doc = None
            if os.path.isfile(out):
                with open(out, encoding='utf-8') as fh:
                    doc = json.load(fh)
        return r, doc

    def test_another_design_is_not_a_reference(self):
        """#1099 verifier: esp_prog's pile took glasgow's 4.787 mm without
        a word. The reference's parts must be on this board."""
        glasgow = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
        _r, doc = self._emit(PILE, '--decaps-from', glasgow)
        self.assertNotIn('max_distance_mm', doc.get('decaps') or {})
        held = doc['context']['budget_withheld']['decaps.max_distance_mm']
        self.assertIn('not a placement of this design', held)

    def test_a_small_design_of_generic_parts_is_not_a_reference(self):
        """#1098 review: an H3/DDR board (20 parts, 18 of them 0402 C1-C12
        and R1-R6, all on tigard under the same ref and footprint) armed
        tigard's intent with its 4.11 mm. The match is taken both ways."""
        tigard = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
        h3 = os.path.join(ROOT, 'awx', 'fb_t2q_fresh.kicad_pcb')
        _r, doc = self._emit(tigard, '--decaps-from', h3)
        self.assertNotIn('max_distance_mm', doc.get('decaps') or {})
        held = doc['context']['budget_withheld']['decaps.max_distance_mm']
        self.assertIn('not a placement of this design', held)
        self.assertLess(doc['context']['decap_census']['reference_part_match'],
                        0.9)

    def test_a_pile_is_not_a_reference(self):
        _r, doc = self._emit(PLACED, '--decaps-from', PILE)
        self.assertNotIn('max_distance_mm', doc.get('decaps') or {})
        held = doc['context']['budget_withheld']['decaps.max_distance_mm']
        self.assertIn('not placed', held)

    def test_the_basis_names_the_reference(self):
        _r, doc = self._emit(PILE, '--decaps-from', PLACED)
        self.assertEqual(doc['context']['basis']['decaps.max_distance_mm'],
                         'reference:esp_prog.kicad_pcb')

    def test_a_missing_reference_is_a_clean_error(self):
        r, doc = self._emit(PILE, '--decaps-from', 'no/such/board.kicad_pcb')
        self.assertEqual(r.returncode, 2, r.stderr[-800:])
        self.assertIn('no such board', r.stderr)
        self.assertNotIn('Traceback', r.stderr)

    def test_the_same_limit_as_declaring_on_the_reference(self):
        """`--decaps-from X` on a pile writes what `--declare-decaps` on X
        itself writes: one derivation, two sources."""
        with tempfile.TemporaryDirectory() as td:
            a, b = os.path.join(td, 'a.json'), os.path.join(td, 'b.json')
            for args, out in (([PILE, '--allow-unplaced', '--decaps-from',
                                PLACED], a),
                              ([PLACED, '--declare-decaps'], b)):
                subprocess.run([sys.executable, '-X', 'utf8',
                                os.path.join(ROOT, 'py_tools',
                                             'check_floorplan.py'),
                                *args, '--emit-intent', out],
                               capture_output=True, text=True, cwd=ROOT)
            with open(a, encoding='utf-8') as fh:
                da = json.load(fh)
            with open(b, encoding='utf-8') as fh:
                db = json.load(fh)
        self.assertEqual(da['decaps'].get('max_distance_mm'),
                         db['decaps'].get('max_distance_mm'))


class TestDiagonalFallback(unittest.TestCase):
    LONG = ('L1', 20.0, 3.0, 3.5, 0.3, 3.0)

    def test_a_part_that_fits_only_diagonally_is_seated(self):
        res_off, _ = seed([self.LONG], diagonal=False)
        res_on, _ = seed([self.LONG], diagonal=True)
        self.assertEqual(res_off['unseated'], ['L1'])
        self.assertEqual(res_on['unseated'], [])

    def test_a_part_that_fits_orthogonally_seats_as_before(self):
        small = ('R1', 20.0, 3.0, 1.0, 0.5, 0.5)
        res_off, _ = seed([small], diagonal=False)
        res_on, _ = seed([small], diagonal=True)
        self.assertEqual(res_off['unseated'], [])
        self.assertEqual(res_off['placements'], res_on['placements'])


if __name__ == '__main__':
    unittest.main()
