"""#1100, #1102, #1103: three gaps run 37 (StickHub from a pile) found.

#1100  place_pose wrote a board with a NEW pad short because the conflict
       COUNT tied: C18 left an overlap it had in the staging pile and made one
       with D11 on the board (2 -> 2). Pairs are now compared as sets.
#1102  `--decaps-from` armed only the tether limit, whose currency (cap to
       the chip's inflated pad bbox, within 5 mm) let run 37 strand C3, C7,
       C12 at 7.8-10.2 mm with no error. It now also derives the PIN limit
       (`decaps.max_pin_distance_mm`) and, when the reference keeps every
       rail cap inside the radius, promotes `decap_ungraded` to error.
#1103  `--emit-intent` on a pile read edge claims, zones and an oob budget
       off staging poses (87 "edge connectors" on StickHub's pile). On a pile
       nothing is read off an unlocked part's pose.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from test_1094_rotated_courtyards import stickhub  # noqa: E402

PILE = os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
PLACED = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FLOORPLAN = os.path.join(ROOT, 'py_tools', 'check_floorplan.py')

R = ('  (footprint "t:R" (layer "F.Cu") (at {x} {y})\n'
     '    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
     '    (fp_rect (start -1 -0.5) (end 1 0.5) (stroke (width 0.05)'
     ' (type default)) (layer "F.CrtYd"))\n'
     '    (pad "1" smd rect (at -0.5 0) (size 0.6 0.6) (layers "F.Cu")'
     ' (net {a} "N{a}"))\n'
     '    (pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "F.Cu")'
     ' (net {b} "N{b}")))\n')


def emit(board, *extra):
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'i.json')
        r = subprocess.run([sys.executable, '-X', 'utf8', FLOORPLAN, board,
                            '--allow-unplaced', '--emit-intent', out, *extra],
                           capture_output=True, text=True, cwd=ROOT)
        assert os.path.isfile(out), r.stdout[-1500:] + r.stderr[-1500:]
        with open(out, encoding='utf-8') as fh:
            return r, json.load(fh)


class TestNewPairAtATiedCount(unittest.TestCase):
    """#1100. R2 and R3 overlap in the staging pile; R1 sits on the board."""

    def _board(self, td):
        text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
                ' (44 "Edge.Cuts" user))\n'
                '  (net 0 "") (net 1 "N1") (net 2 "N2") (net 3 "N3")'
                ' (net 4 "N4")\n'
                '  (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1)'
                ' (type default)) (layer "Edge.Cuts"))\n'
                + R.format(ref='R1', x=10, y=10, a=1, b=2)
                + R.format(ref='R2', x=40, y=10, a=3, b=4)
                + R.format(ref='R3', x=40.4, y=10, a=1, b=2) + ')\n')
        p = os.path.join(td, 'b.kicad_pcb')
        with open(p, 'w', encoding='utf-8') as fh:
            fh.write(text)
        return p

    def _pose(self, board, x, y):
        out = board[:-10] + '_o.kicad_pcb'
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_placer', 'place_pose.py'),
                            board, out, 'set', 'R2', str(x), str(y),
                            '--rot', '0'],
                           capture_output=True, text=True, cwd=ROOT)
        return r, os.path.exists(out)

    def test_a_new_short_is_refused_when_the_count_ties(self):
        with tempfile.TemporaryDirectory() as td:
            r, wrote = self._pose(self._board(td), 10.4, 10)
        self.assertEqual((r.returncode, wrote), (4, False),
                         r.stdout[-1500:])
        self.assertIn('pad_conflict_pairs new: R1/R2', r.stdout + r.stderr)

    def test_the_control_leaving_the_pile_for_a_clear_spot_is_accepted(self):
        with tempfile.TemporaryDirectory() as td:
            r, wrote = self._pose(self._board(td), 20, 20)
        self.assertEqual((r.returncode, wrote), (0, True), r.stdout[-1500:])

    def test_the_arm_is_off_for_an_older_report(self):
        from placement.pose_ops import worsened
        before = {'pad_conflicts': 1}
        after = {'pad_conflicts': 1, 'pad_conflict_pairs': [['R1', 'R2']]}
        self.assertEqual(worsened(before, after), [])
        before['pad_conflict_pairs'] = [['R2', 'R3']]
        self.assertEqual(worsened(before, after), ['pad_conflict_pairs'])


class TestPinLimitFromTheReference(unittest.TestCase):
    """#1102."""

    def test_a_withheld_pin_limit_is_a_note_not_debt(self):
        """esp_prog covers too few supply pins to derive a pin limit: the
        reason is a census NOTE, and the board still grades pass against
        its own intent (it went pass -> exit 4 when the withholding was
        written to budget_withheld)."""
        _r, doc = emit(PLACED, '--decaps-from', PLACED)
        census = doc['context']['decap_census']
        self.assertNotIn('max_pin_distance_mm', doc.get('decaps') or {})
        self.assertIn('supply pin', census.get('pin_limit_withheld') or '')
        self.assertNotIn('decaps.max_pin_distance_mm',
                         doc['context'].get('budget_withheld') or {})
        with tempfile.TemporaryDirectory() as td:
            ip = os.path.join(td, 'i.json')
            with open(ip, 'w', encoding='utf-8') as fh:
                json.dump(doc, fh)
            g = subprocess.run([sys.executable, '-X', 'utf8', FLOORPLAN,
                                PLACED, '--intent', ip],
                               capture_output=True, text=True, cwd=ROOT)
        self.assertEqual(g.returncode, 0, g.stdout[-1500:])

    def test_a_promotion_without_a_pin_limit_is_said(self):
        """The run25 esp_prog fixture keeps all 4 tethered caps inside the
        radius, so `decap_ungraded` is promoted to error, while only 2 supply
        pins are covered and the pin limit is withheld. The promotion was
        printed only beside a derived pin limit, so here the emit raised a
        severity without a word.

        Since #1142 the promotion is PER CAP: the held caps are listed in
        `decaps.within_radius_refs` rather than raised board-wide through a
        top-level `severity`, and the print still says so."""
        board = os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                             'esp_prog_placed.kicad_pcb')
        r, doc = emit(board, '--decaps-from', board)
        held = (doc.get('decaps') or {}).get('within_radius_refs')
        self.assertTrue(held, doc.get('decaps'))
        self.assertNotIn('decap_ungraded', doc.get('severity') or {})
        self.assertNotIn('max_pin_distance_mm', doc.get('decaps') or {})
        self.assertIn('decap_ungraded promoted to error', r.stdout)

    def test_a_derived_pin_limit_carries_its_basis(self):
        """flat_hierarchy derives a pin limit with no tether limit beside it;
        the pin limit is still labelled with its reference."""
        board = os.path.join(ROOT, 'kicad_files', 'flat_hierarchy.kicad_pcb')
        _r, doc = emit(board, '--decaps-from', board)
        self.assertIn('max_pin_distance_mm', doc['decaps'])
        self.assertEqual(doc['context']['basis']
                         ['decaps.max_pin_distance_mm'],
                         'reference:flat_hierarchy.kicad_pcb')

    def test_stickhub_reference_arms_the_pin_rule_and_the_horizon(self):
        path = stickhub()
        if not path:
            self.skipTest('KiCad StickHub demo not installed')
        _r, doc = emit(path, '--decaps-from', path)
        self.assertEqual(doc['decaps']['max_pin_distance_mm'], 1.7716)
        # #1142: the horizon is held per cap, not raised board-wide.
        self.assertTrue(doc['decaps'].get('within_radius_refs'), doc['decaps'])
        self.assertNotIn('decap_ungraded', doc.get('severity') or {})
        # The reference grades clean against what it produced (the fixed
        # point); rounding before the ceiling once put the limit BELOW the
        # true max and failed it.
        from kicad_parser import parse_kicad_pcb
        from placement.floorplan import grade, load_intent
        with tempfile.TemporaryDirectory() as td:
            ip = os.path.join(td, 'i.json')
            with open(ip, 'w', encoding='utf-8') as fh:
                json.dump(doc, fh)
            g = grade(load_intent(ip), parse_kicad_pcb(path), path)
        self.assertEqual([v for v in g.errors if v.rule.startswith('decap')],
                         [])


class TestPileEmitsNoPoseClaims(unittest.TestCase):
    """#1103."""

    def test_a_pile_withholds_what_its_poses_would_claim(self):
        """esp_prog with every unlocked part parked off the right edge, in
        a column: each of them overhangs and would be read as an edge
        connector nearest the east edge."""
        from kicad_parser import iter_footprint_blocks
        from placement.parser import extract_locked_refs
        text = open(PLACED, encoding='utf-8').read()
        locked = extract_locked_refs(PLACED)
        blocks = [(s, e, key) for s, e, _t, _r, key
                  in iter_footprint_blocks(text) if key not in locked]
        for i, (s, e, key) in reversed(list(enumerate(blocks))):
            blk = text[s:e]
            a = blk.index('(at ')
            b = blk.index(')', a)
            blk = blk[:a] + f'(at 200 {60 + 4 * i}' + blk[b:]
            text = text[:s] + blk + text[e:]
        with tempfile.TemporaryDirectory() as td:
            pile = os.path.join(td, 'pile.kicad_pcb')
            with open(pile, 'w', encoding='utf-8') as fh:
                fh.write(text)
            _r, doc = emit(pile, '--declare-classes')
        w = doc['context']['pose_claims_withheld']
        self.assertIn('pile', w['reason'])
        self.assertGreaterEqual(len(w['refs_off_board']), 10)
        self.assertEqual([c['ref'] for c in doc['edge_connectors']
                          if c.get('edge') and c['ref'] not in locked], [])
        self.assertEqual([b for b in doc['blocks'] if 'zone' in b], [])
        self.assertNotIn('oob_count', doc.get('legality_budget') or {})

    def test_a_pile_inside_the_board_keeps_an_honest_zero(self):
        """run 29's heap sits on the board: its oob_count 0 is true and
        stays a budget rather than going dark."""
        _r, doc = emit(PILE, '--declare-classes')
        self.assertIn('pose_claims_withheld', doc['context'])
        self.assertEqual((doc.get('legality_budget') or {}).get('oob_count'),
                         0)

    def test_a_placed_board_keeps_its_claims(self):
        _r, doc = emit(PLACED, '--declare-classes')
        self.assertNotIn('pose_claims_withheld', doc['context'])


if __name__ == '__main__':
    unittest.main()
