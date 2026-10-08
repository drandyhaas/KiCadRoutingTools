"""#1096: pad copper off the outline makes check_assembly NOT BUILDABLE.

Run 36 (KiCad's StickHub demo, placed from a pile) left C20 at its staging
pose, its pad copper 7.84 mm below the board's south edge. check_assembly printed
"pad copper genuinely off the outline (per-pad, margin 0): C20 (36.8mm)" and
then `VERDICT: buildable (blocking 0)`; the run routed the board and the
router took GND off the board to reach C20.

Two things are pinned here:
- the per-pad, margin-0 channel is a verdict conjunct (and board_score's);
- the printed number is a DISTANCE (`oob_pad_copper_overrun_mm`). The old
  "36.8mm" was rect_outside_amount's ranking sum -- one overshoot term per
  off-board corner plus the bbox term -- for copper that reaches 7.84 mm out.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

BOARD = (
    '(kicad_pcb (version 20240108) (generator pcbnew)\n'
    '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
    '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
    # A notched outline, not a plain rectangle: on a rectangle the ranking
    # magnitude and the distance coincide, and a test there cannot tell them
    # apart (mutate_1094 `overrun-printed-as-the-sum` survived it).
    + ''.join(f'  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (stroke'
              f' (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
              for a, b in zip(
                  [(0, 0), (15, 0), (15, 5), (20, 5), (20, 20), (0, 20)],
                  [(15, 0), (15, 5), (20, 5), (20, 20), (0, 20), (0, 0)]))
    +
    '  (footprint "t:R" (layer "F.Cu") (at 10 {y})\n'
    '    (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))\n'
    '    (fp_rect (start -1 -0.5) (end 1 0.5) (stroke (width 0.05)'
    ' (type default)) (layer "F.CrtYd"))\n'
    '    (pad "1" smd rect (at -0.5 0) (size 0.6 0.6) (layers "F.Cu")'
    ' (net 1 "N1"))\n'
    '    (pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "F.Cu")'
    ' (net 2 "N2"))))\n')


def run(y):
    """check_assembly on a 20x20 board with R1 at (10, y)."""
    return _run_text(BOARD.format(y=y))


def _run_text(text):
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(text)
        js = os.path.join(td, 'a.json')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_tools',
                                         'check_assembly.py'),
                            path, '--json', js],
                           capture_output=True, text=True, cwd=ROOT)
        assert os.path.isfile(js), r.stderr[-2000:]
        with open(js, encoding='utf-8') as fh:
            return r, json.load(fh)


class TestOffOutline(unittest.TestCase):
    def test_a_part_off_the_board_is_not_buildable(self):
        """R1 at y=27: its pads span y 26.7-27.3, 6.7-7.3 mm below y=20."""
        r, d = run(27)
        self.assertEqual(r.returncode, 4, r.stdout[-1500:])
        self.assertFalse(d['buildable'])
        self.assertEqual([x[0] for x in d['oob_pad_copper_refs']], ['R1'])
        self.assertAlmostEqual(d['oob_pad_copper_overrun_mm']['R1'],
                               27.3 - 20, places=3)
        self.assertIn('R1 (7.3mm past the outline)', r.stdout)
        self.assertIn('NOT BUILDABLE', r.stdout)

    def test_a_part_half_off_names_its_overhang(self):
        """R1 at y=20: half of each pad is past the edge by 0.3 mm."""
        _r, d = run(20)
        self.assertFalse(d['buildable'])
        self.assertAlmostEqual(d['oob_pad_copper_overrun_mm']['R1'], 0.3,
                               places=3)

    def test_the_control_on_the_board_is_buildable(self):
        r, d = run(10)
        self.assertEqual(r.returncode, 0, r.stdout[-1500:])
        self.assertTrue(d['buildable'])
        self.assertEqual(d['oob_pad_copper_refs'], [])

    def test_a_castellated_pad_on_the_edge_does_not_gate(self):
        """A castellated module's half-holes are ON the outline by design
        (rp2350's Teensy U8, 0.8 mm past it): disclosed, not gated."""
        board = BOARD.replace(
            '(pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "F.Cu")',
            '(pad "2" thru_hole rect (at 0.5 0.4) (size 0.6 0.6) (drill 0.3)'
            ' (layers "*.Cu") (property pad_prop_castellated)')
        r, d = _run_text(board.format(y=19.5))
        self.assertTrue(d['buildable'], r.stdout[-1500:])
        self.assertEqual(d['oob_pad_copper_gating_refs'], [])
        self.assertEqual([x[0] for x in d['oob_pad_copper_refs']], ['R1'])
        self.assertIn('on the outline by design, not gated: R1', r.stdout)

    def test_a_castellated_part_off_the_board_still_gates(self):
        """The exemption holds only while a castellated pad STRADDLES the
        edge: the same part parked 10 mm off the board is a part off the
        board (#1096 verifier D8)."""
        board = BOARD.replace(
            '(pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "F.Cu")',
            '(pad "2" thru_hole rect (at 0.5 0.4) (size 0.6 0.6) (drill 0.3)'
            ' (layers "*.Cu") (property pad_prop_castellated)')
        # BOTH pads castellated, so nothing but the straddle rule can make it
        # gate (mutate_1094 `castellated-exempt-off-the-board` survived a
        # one-pad version: its plain pad gated on its own).
        board = board.replace(
            '(pad "1" smd rect (at -0.5 0) (size 0.6 0.6) (layers "F.Cu")',
            '(pad "1" thru_hole rect (at -0.5 0.4) (size 0.6 0.6) (drill 0.3)'
            ' (layers "*.Cu") (property pad_prop_castellated)')
        self.assertEqual(board.count('pad_prop_castellated'), 2)
        r, d = _run_text(board.format(y=30))
        self.assertFalse(d['buildable'], r.stdout[-1500:])
        self.assertEqual(d['oob_pad_copper_gating_refs'], ['R1'])

    def test_a_castellated_pad_on_a_plain_rectangle(self):
        """A plain rectangular outline parses to no rings at all; the
        straddle test falls back to the bounds (final review: only the
        notched outline was tested)."""
        notch = ''.join(
            f'  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (stroke'
            f' (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
            for a, b in zip(
                [(0, 0), (15, 0), (15, 5), (20, 5), (20, 20), (0, 20)],
                [(15, 0), (15, 5), (20, 5), (20, 20), (0, 20), (0, 0)]))
        self.assertIn(notch, BOARD)
        rect = BOARD.replace(
            notch, '  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1)'
                   ' (type default)) (layer "Edge.Cuts"))\n')
        rect = rect.replace(
            '(pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "F.Cu")',
            '(pad "2" thru_hole rect (at 0.5 0.4) (size 0.6 0.6) (drill 0.3)'
            ' (layers "*.Cu") (property pad_prop_castellated)')
        self.assertIn('gr_rect', rect)
        _r, d = _run_text(rect.format(y=19.5))
        self.assertTrue(d['buildable'])
        self.assertEqual(d['oob_pad_copper_gating_refs'], [])

    def test_render_placement_gates_the_same_parts(self):
        """render_placement's `pad_copper_gating` is check_assembly's list:
        the castellated edge part is in neither, the part off the board in
        both (verifier D9: --gate and the free-agent's grade.py read it)."""
        board = BOARD.replace(
            '(pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "F.Cu")',
            '(pad "2" thru_hole rect (at 0.5 0.4) (size 0.6 0.6) (drill 0.3)'
            ' (layers "*.Cu") (property pad_prop_castellated)')
        for y, want in ((19.5, []), (30, ['R1'])):
            with tempfile.TemporaryDirectory() as td:
                path = os.path.join(td, 'b.kicad_pcb')
                with open(path, 'w', encoding='utf-8') as fh:
                    fh.write(board.format(y=y))
                js = os.path.join(td, 'r.json')
                subprocess.run([sys.executable, '-X', 'utf8',
                                os.path.join(ROOT, 'py_tools',
                                             'render_placement.py'),
                                path, '-o', os.path.join(td, 'r.png'),
                                '--json-out', js],
                               capture_output=True, text=True, cwd=ROOT)
                with open(js, encoding='utf-8') as fh:
                    off = json.load(fh)['checklist']['a_off_outline']
            self.assertEqual([r for r, _d in off['pad_copper_gating']], want,
                             (y, off))
            _r, d = _run_text(board.format(y=y))
            self.assertEqual(d['oob_pad_copper_gating_refs'], want, y)

    def test_a_round_pad_inside_a_round_board_does_not_gate(self):
        """A 1.6 mm round pad 0.3 mm inside a round outline at 45 degrees:
        its bounding-box corner is 0.03 mm past the circle, its copper is
        not."""
        c = 10 + (10 - 0.8 - 0.3) / 2 ** 0.5
        board = (
            '(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "N1")\n'
            '  (gr_circle (center 10 10) (end 20 10) (stroke (width 0.1)'
            ' (type default)) (layer "Edge.Cuts"))\n'
            f'  (footprint "t:TP" (layer "F.Cu") (at {c:.4f} {c:.4f})\n'
            '    (property "Reference" "TP1" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" smd circle (at 0 0) (size 1.6 1.6) (layers "F.Cu")'
            ' (net 1 "N1"))))\n')
        r, d = _run_text(board)
        self.assertTrue(d['buildable'], r.stdout[-1500:])
        self.assertEqual(d['oob_pad_copper_gating_refs'], [])

    def test_board_score_counts_it(self):
        """board_score reads the verdict: NOT BUILDABLE at blocking 0 is 1,
        and the conjunct is named among the live ones."""
        import board_score
        _r, d = run(27)
        comp = board_score.assembly_component(d, 4)
        self.assertEqual(comp['count'], 1)
        self.assertIn('oob_pad_copper_gating_count', comp['live_conjuncts_fired'])


if __name__ == '__main__':
    unittest.main()
