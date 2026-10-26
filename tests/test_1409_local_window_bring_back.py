"""#1409: a part's carried Edge.Cuts window is judged in the part's OWN frame.

A footprint's closed Edge.Cuts ring that lies within its own extent -- the
bounding box, in its local frame, of its pads' copper and its courtyard -- is a
relief the part carries (`owns_board_outline` False), wherever the part sits.
Judged only against the board outline at the part's current pose, the same
window read carried on the board and structural once off it, so on a pile made
by any means `place_pose set` refused to bring the part back ("draws the board
outline ... moving it would resize the board").

Fixture: the tracked esp_prog board with a 1.0 x 0.6 mm closed Edge.Cuts
window carried by U1 at its body centre, and the same board with U1 moved
12 mm south, off the board.

A ring that is NOT inside its footprint's own extent keeps #829's rules: a
footprint drawing a closed ring around the whole board stays structural,
whether the board also has board-level Edge.Cuts or not.
"""
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import (find_matching_paren, footprint_outline_owners,  # noqa: E402
                          parse_kicad_pcb)
from run_utils import evidence, tool, tool_env                          # noqa: E402

BOARD = evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'), 'tracked board')
MAKE_UNPLACED = evidence(os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                                      'scripts', 'make_unplaced.py'), 'make_unplaced')
ANCHOR = '\t\t(tags "CMS SSOP SMD")\n'
POSE_ON = '\t\t(at 127 100)\n'
POSE_OFF = '\t\t(at 127 112)\n'


def _rect(x0, y0, x1, y1, uid):
    return (f'\t\t(fp_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.05) '
            f'(type default)) (fill no) (layer "Edge.Cuts") (uuid "{uid}"))\n')


WINDOW = _rect(-0.5, -0.3, 0.5, 0.3, '0badc0de-0000-4000-8000-000000000001')
# The board's Edge.Cuts spans x 114..145.75, y 91..105.5; U1 sits at (127, 100)
# unrotated, so this ring encloses the whole board with a margin.
ENCLOSING = _rect(-14, -10, 19.75, 6.5, '0badc0de-0000-4000-8000-000000000002')


def run(*argv):
    r = subprocess.run([sys.executable, '-X', 'utf8'] + [str(a) for a in argv],
                       capture_output=True, text=True, env=tool_env(), cwd=ROOT, timeout=600)
    return r.returncode, r.stdout + r.stderr


def board_text(extra=WINDOW, pose=POSE_ON):
    with open(BOARD, encoding='utf-8', newline='') as fh:
        text = fh.read()
    assert text.count(ANCHOR) == 1, 'control: the anchor line is not unique'
    assert text.count(POSE_ON) == 1, "control: U1's pose line is not unique"
    return text.replace(ANCHOR, ANCHOR + extra).replace(POSE_ON, pose)


def write(td, name, text):
    path = os.path.join(td, name)
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)
    return path


class TestLocalWindow(unittest.TestCase):
    def test_off_the_board_the_window_is_still_carried(self):
        on, off = board_text(), board_text(pose=POSE_OFF)
        self.assertNotEqual(on, off, 'control: U1 was not moved')
        self.assertFalse(footprint_outline_owners(on)['U1'])
        self.assertFalse(footprint_outline_owners(off)['U1'],
                         'a window inside its own part reads structural off the board')
        # turned, too: the extent is read in the part's own frame
        turned = board_text(pose='\t\t(at 127 112 33)\n')
        self.assertFalse(footprint_outline_owners(turned)['U1'])
        with tempfile.TemporaryDirectory() as td:
            fp = parse_kicad_pcb(write(td, 'off.kicad_pcb', off)).footprints['U1']
            self.assertTrue(fp.owns_edge_cuts)
            self.assertFalse(fp.owns_board_outline)
            self.assertAlmostEqual(fp.y, 112.0, places=3)

    def test_place_pose_brings_the_part_back(self):
        with tempfile.TemporaryDirectory() as td:
            src = write(td, 'off.kicad_pcb', board_text(pose=POSE_OFF))
            out = os.path.join(td, 'back.kicad_pcb')
            code, log = run(tool('place_pose.py'), src, out, 'set', 'U1', '127', '100')
            self.assertNotIn('draws the board outline', log, log[-800:])
            self.assertNotIn('CHANGED THE BOARD OUTLINE', log, log[-800:])
            self.assertEqual(code, 0, log[-800:])
            fp = parse_kicad_pcb(evidence(out, 'board with U1 back')).footprints['U1']
            self.assertAlmostEqual(fp.x, 127.0, places=3)
            self.assertAlmostEqual(fp.y, 100.0, places=3)
            self.assertFalse(fp.owns_board_outline)

    def test_make_unplaced_writes_the_pile(self):
        with tempfile.TemporaryDirectory() as td:
            src = write(td, 'win.kicad_pcb', board_text())
            out = os.path.join(td, 'pile.kicad_pcb')
            code, log = run(MAKE_UNPLACED, src, out)
            self.assertEqual(code, 0, log[-800:])
            self.assertIn('unplaced=True', log)
            self.assertFalse(parse_kicad_pcb(evidence(out, 'pile')).footprints['U1']
                             .owns_board_outline)

    def test_a_ring_around_the_board_stays_structural(self):
        text = board_text(extra=ENCLOSING)
        self.assertTrue(footprint_outline_owners(text)['U1'])
        # and with no board-level Edge.Cuts at all: the part IS the outline
        bare = text
        while '\t(gr_line' in bare:
            i = bare.index('\t(gr_line')
            bare = bare[:i] + bare[find_matching_paren(bare, i + 1):]
        self.assertNotEqual(bare, text, 'control: the board had no board-level edges')
        self.assertNotIn('(gr_line', bare, 'control: board-level edges remain')
        self.assertTrue(footprint_outline_owners(bare)['U1'])

    def test_a_ring_around_one_of_its_own_pads_is_left_to_the_outline(self):
        # A ring enclosing one of the part's own pads is not judged by the
        # part alone (a plated slot looks like this): on the board it is
        # carried by containment, off the board it reads structural, as
        # before #1409.
        pad_ring = _rect(-3.2, 3.2, -2.6, 4.0, '0badc0de-0000-4000-8000-000000000003')
        self.assertFalse(footprint_outline_owners(board_text(extra=pad_ring))['U1'])
        self.assertTrue(footprint_outline_owners(
            board_text(extra=pad_ring, pose=POSE_OFF))['U1'])


if __name__ == '__main__':
    unittest.main()
