#!/usr/bin/env python3
"""#1109: one pile predicate, and a 2D film fallback that is said out loud.

(a) board_brief's `unplaced` read false on run 38's StickHub pile (a staging
    RING, 93% of parts off the outline) while check_floorplan's emitter
    called it a pile; the free-agent skill chose its mode from `unplaced`.
    `PlacementState.pile` is now the one test; board_brief publishes it.
(b) make_film fell back to the 2D X-ray with only a stderr line mid-log.
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in ('py_placer', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from placement.placement_state import PlacementState  # noqa: E402

# A 40x40 board with 20 small parts on a ring OUTSIDE it: spread, not
# stacked, so `unplaced` (S1, or S2+S3) does not fire; S3 does.
RING = ['(kicad_pcb (version 20240108) (generator pcbnew)',
        '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))',
        '  (net 0 "") (net 1 "A")',
        '  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default))'
        ' (layer "Edge.Cuts"))']
for i in range(20):
    x, y = (-30 + 6 * i, -20) if i < 10 else (-30 + 6 * (i - 10), 70)
    RING.append(
        f'  (footprint "t:R" (layer "F.Cu") (at {x} {y})\n'
        f'    (property "Reference" "R{i + 1}" (at 0 0) (layer "F.SilkS"))\n'
        '    (fp_rect (start -1 -0.6) (end 1 0.6) (stroke (width 0.05)'
        ' (type default)) (layer "F.CrtYd"))\n'
        '    (pad "1" smd rect (at -0.5 0) (size 0.6 0.8) (layers "F.Cu")'
        ' (net 1 "A"))\n'
        '    (pad "2" smd rect (at 0.5 0) (size 0.6 0.8) (layers "F.Cu")'
        ' (net 1 "A")))')
RING.append(')')


class TestPilePredicate(unittest.TestCase):
    def test_each_arm(self):
        self.assertTrue(PlacementState(unplaced=True).pile)
        self.assertTrue(PlacementState(signals={'s3_outside': True}).pile)
        heap = PlacementState(partially_unplaced=True, n_footprints=10,
                              stacked_suspect_refs=[f'R{i}' for i in range(5)])
        self.assertTrue(heap.pile)
        few = PlacementState(partially_unplaced=True, n_footprints=10,
                             stacked_suspect_refs=['R1', 'R2'])
        self.assertFalse(few.pile)
        self.assertFalse(PlacementState().pile)

    def test_board_brief_publishes_pile_on_a_staging_ring(self):
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, 'ring.kicad_pcb')
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write('\n'.join(RING))
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_tools', 'board_brief.py'), p],
                capture_output=True, text=True, cwd=ROOT, timeout=600)
        line = [ln for ln in r.stdout.splitlines()
                if ln.startswith('JSON_SUMMARY: ')]
        self.assertTrue(line, r.stdout[-2000:] + r.stderr[-2000:])
        summary = json.loads(line[-1][len('JSON_SUMMARY: '):])
        self.assertIs(summary['unplaced'], False)   # the gap #1109 is about
        self.assertIs(summary['pile'], True)
        # #1115: the second key the skill reads, on the line it reads, and a
        # text state line that agrees with `pile` instead of saying `placed`.
        self.assertIs(summary['has_copper'], False)
        self.assertIn('state: PILE;', r.stdout)

    def test_a_placed_board_is_not_a_pile(self):
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_tools', 'board_brief.py'),
             os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')],
            capture_output=True, text=True, cwd=ROOT, timeout=600)
        line = [ln for ln in r.stdout.splitlines()
                if ln.startswith('JSON_SUMMARY: ')]
        self.assertTrue(line, r.stderr[-2000:])
        summary = json.loads(line[-1][len('JSON_SUMMARY: '):])
        self.assertIs(summary['pile'], False)
        self.assertIs(summary['has_copper'], False)
        self.assertIn('state: placed;', r.stdout)

    def test_board_brief_publishes_has_copper(self):
        """#1115: `has_copper` on the JSON_SUMMARY line is the board's own
        copper, not a constant -- the ring with one routed segment reads
        True where the bare ring reads False."""
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, 'ring_cu.kicad_pcb')
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write('\n'.join(
                    RING[:-1]
                    + ['  (segment (start 1 1) (end 5 1) (width 0.2)'
                       ' (layer "F.Cu") (net 1))', ')']))
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_tools', 'board_brief.py'), p],
                capture_output=True, text=True, cwd=ROOT, timeout=600)
        line = [ln for ln in r.stdout.splitlines()
                if ln.startswith('JSON_SUMMARY: ')]
        self.assertTrue(line, r.stdout[-2000:] + r.stderr[-2000:])
        self.assertIs(json.loads(line[-1][len('JSON_SUMMARY: '):])
                      ['has_copper'], True)
        self.assertIn('copper yes (1 segs', r.stdout)

    def test_a_heap_prints_pile_too(self):
        """#1115: the other pile `unplaced` misses -- 15 of 20 parts stacked
        on one spot, inside the outline. The text line printed `partially
        unplaced` while the JSON said `pile: true`."""
        import re
        heap = RING[:4] + [
            re.sub(r'\(at -?\d+ -?\d+\)',
                   '(at 20 20)' if i < 15 else f'(at {5 + 6 * (i - 15)} 5)',
                   RING[4 + i], count=1)
            for i in range(20)] + [')']
        self.assertEqual(sum('(at 20 20)' in ln for ln in heap), 15)
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, 'heap.kicad_pcb')
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write('\n'.join(heap))
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_tools', 'board_brief.py'), p],
                capture_output=True, text=True, cwd=ROOT, timeout=600)
        line = [ln for ln in r.stdout.splitlines()
                if ln.startswith('JSON_SUMMARY: ')]
        self.assertTrue(line, r.stdout[-2000:] + r.stderr[-2000:])
        summary = json.loads(line[-1][len('JSON_SUMMARY: '):])
        self.assertIs(summary['pile'], True)
        self.assertIs(summary['unplaced'], False)
        self.assertIn('state: PILE;', r.stdout)

    def test_the_skill_command_runs_on_a_fresh_checkout(self):
        """#1115: the free-agent skill's first command writes
        `--json wk/<run>/brief.json`, and wk/ does not exist on a fresh
        checkout. It died with a traceback before JSON_SUMMARY."""
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'wk', 'r1', 'brief.json')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_tools', 'board_brief.py'),
                 os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'),
                 '--json', out],
                capture_output=True, text=True, cwd=ROOT, timeout=600)
            self.assertEqual(r.returncode, 0, r.stdout[-1500:] + r.stderr[-1500:])
            self.assertTrue(os.path.isfile(out))
            with open(out, encoding='utf-8') as fh:
                self.assertIn('has_copper', json.load(fh)['state'])
        self.assertTrue(any(ln.startswith('JSON_SUMMARY: ')
                            for ln in r.stdout.splitlines()))


class TestFilmBoardBoxLine(unittest.TestCase):
    def _line(self, report):
        import make_film
        from stage3d import film
        film.LAST_REPORT.clear()
        film.LAST_REPORT.update(report)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            make_film._board_box_line()
        film.LAST_REPORT.clear()
        return buf.getvalue()

    def test_fallback_is_a_warning_on_stdout(self):
        out = self._line({'mode': '2d', 'asked': 'auto',
                          'why': 'playwright-core is not installed'})
        self.assertIn('WARNING board box: 2D X-ray', out)
        self.assertIn('playwright-core is not installed', out)

    def test_asked_2d_and_3d_are_not_warnings(self):
        self.assertNotIn('WARNING', self._line({'mode': '2d', 'asked': '2d'}))
        out = self._line({'mode': '3d', 'asked': 'auto', 'renderer': 'x'})
        self.assertIn('board box: 3D', out)
        self.assertNotIn('WARNING', out)

    def test_apply_records_its_report(self):
        from stage3d import film
        film.LAST_REPORT.clear()
        frames, rep = film.apply([], None, None, None, None,
                                 stage_present=False, mode='2d')
        self.assertEqual(film.LAST_REPORT.get('mode'), '2d')
        self.assertEqual(film.LAST_REPORT.get('asked'), '2d')


if __name__ == '__main__':
    unittest.main()
