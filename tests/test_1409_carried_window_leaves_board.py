"""#1409: a part's carried Edge.Cuts ring may travel with it off the board.

A closed ring that encloses none of its part's pads is carried wherever the
part sits (`classify_outline_owners` rule 2a; pinned by
test_1409_local_window_bring_back). A ring the part alone cannot settle -- one
around one of its own pads, the shape of a plated slot -- is still judged by
containment in the outline at each footprint's CURRENT pose (rule 2b): carried
while the part is on the board, structural once it is off. The writer's
outline tripwire compared the BEFORE and AFTER texts each classified on their
own, so such a part could not be moved off the board (make_unplaced raised
`OutlineOwnerMove` and wrote no pile, `place_pose set` refused). The write now
decides carried-versus-structural once, on the input, and masks both texts
with that one set.

Fixture: the tracked esp_prog board with a 0.6 x 0.8 mm closed Edge.Cuts ring
around U1's pad 1.
"""
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import (footprint_outline_owners, parse_kicad_pcb,   # noqa: E402
                          structural_outline_fingerprint)
from run_utils import evidence, tool, tool_env                          # noqa: E402

BOARD = evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'), 'tracked board')
MAKE_UNPLACED = evidence(os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                                      'scripts', 'make_unplaced.py'), 'make_unplaced')
WINDOW = ('\t\t(fp_rect (start -3.2 3.2) (end -2.6 4.0) (stroke (width 0.05) (type default)) '
          '(fill no) (layer "Edge.Cuts") (uuid "0badc0de-0000-4000-8000-000000000001"))\n')


def run(*argv):
    r = subprocess.run([sys.executable, '-X', 'utf8'] + [str(a) for a in argv],
                       capture_output=True, text=True, env=tool_env(), cwd=ROOT, timeout=600)
    return r.returncode, r.stdout + r.stderr


def windowed(td):
    with open(BOARD, encoding='utf-8', newline='') as fh:
        text = fh.read()
    key = '\t\t(tags "CMS SSOP SMD")\n'
    assert text.count(key) == 1, 'control: the anchor line is not unique'
    path = os.path.join(td, 'win.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text.replace(key, key + WINDOW))
    return path


class TestCarriedWindow(unittest.TestCase):
    def test_the_window_is_carried_on_the_board_and_structural_off_it(self):
        with tempfile.TemporaryDirectory() as td:
            on = windowed(td)
            with open(on, encoding='utf-8', newline='') as fh:
                text = fh.read()
            off = text.replace('\t\t(at 127 100)\n', '\t\t(at 127 112)\n', 1)
            self.assertNotEqual(off, text, 'control: U1 was not moved')
            self.assertFalse(footprint_outline_owners(text)['U1'])
            self.assertTrue(footprint_outline_owners(off)['U1'],
                            'control: the premise (judged at the pose) no longer holds')
            # one decision, made on the input, masks both: the fingerprint of
            # the moved board equals the input's
            self.assertEqual(structural_outline_fingerprint(text, carried={'U1'}),
                             structural_outline_fingerprint(off, carried={'U1'}))

    def test_place_pose_moves_the_part_off_the_board(self):
        with tempfile.TemporaryDirectory() as td:
            src = windowed(td)
            out = os.path.join(td, 'out.kicad_pcb')
            code, log = run(tool('place_pose.py'), src, out, 'set', 'U1', '127', '112', '--force')
            self.assertNotIn('draws the board outline', log, log[-800:])
            self.assertNotIn('CHANGED THE BOARD OUTLINE', log, log[-800:])
            self.assertEqual(code, 0, log[-800:])
            self.assertAlmostEqual(parse_kicad_pcb(evidence(out, 'moved board')).footprints['U1'].y,
                                   112.0, places=3)

    def test_make_unplaced_writes_the_pile(self):
        with tempfile.TemporaryDirectory() as td:
            src = windowed(td)
            out = os.path.join(td, 'pile.kicad_pcb')
            code, log = run(MAKE_UNPLACED, src, out)
            self.assertEqual(code, 0, log[-800:])
            self.assertIn('unplaced=True', log)
            evidence(out, 'pile')


if __name__ == '__main__':
    unittest.main()
