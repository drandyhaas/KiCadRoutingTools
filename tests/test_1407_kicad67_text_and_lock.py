"""#1407 / #1408: the placement writer reads KiCad 6/7 spellings.

#1407: KiCad 6/7 write a free-orientation text as `(at x y unlocked)` or
`(at x y 90 unlocked)`; `place_pose rotate` composes the angle and keeps the
word. #1408: KiCad 6's `locked` as a bare head flag, or as a `(locked)` child
(20210108-20210423), is removed by make_unplaced, so the piled part can be
brought back. (make_unplaced's own pile + text reset is pinned in
test_plan_pcb_placement_scripts.)
"""
import os
import re
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from run_utils import evidence, tool, tool_env  # noqa: E402
from kicad_parser import parse_kicad_pcb        # noqa: E402

BOARD = evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'), 'tracked board')
MAKE_UNPLACED = evidence(os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                                      'scripts', 'make_unplaced.py'), 'make_unplaced')


def run(*argv):
    r = subprocess.run([sys.executable, '-X', 'utf8'] + [str(a) for a in argv],
                       capture_output=True, text=True, env=tool_env(), cwd=ROOT, timeout=600)
    return r.returncode, r.stdout + r.stderr


def written(td, name, edit):
    with open(BOARD, encoding='utf-8', newline='') as fh:
        text = fh.read()
    new = edit(text)
    assert new != text, 'control: the edit changed nothing'
    path = os.path.join(td, name)
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(new)
    return path


class TestKicad67Spellings(unittest.TestCase):
    def _rotate(self, spelling, expect):
        with tempfile.TemporaryDirectory() as td:
            src = written(td, 'src.kicad_pcb',
                          lambda t: t.replace('(at 0.02 1.42 90)', spelling))
            out = os.path.join(td, 'out.kicad_pcb')
            code, log = run(tool('place_pose.py'), src, out, 'rotate', 'C1', '180', '--force')
            self.assertEqual(code, 0, log[-600:])
            with open(evidence(out, 'rotated board'), encoding='utf-8') as fh:
                self.assertIn(expect, fh.read())

    def test_rotate_keeps_the_word_with_no_angle_token(self):
        # C1 turns 90 -> 180: the text composes 0 + 90
        self._rotate('(at 0.02 1.42 unlocked)', '(at 0.02 1.42 90 unlocked)')

    def test_rotate_keeps_the_word_with_an_angle_token(self):
        self._rotate('(at 0.02 1.42 90 unlocked)', '(at 0.02 1.42 180 unlocked)')

    def _pile(self, edit):
        with tempfile.TemporaryDirectory() as td:
            src = written(td, 'k6.kicad_pcb', edit)
            self.assertTrue(parse_kicad_pcb(src).footprints['U1'].locked, 'control: lock not read')
            out = os.path.join(td, 'pile.kicad_pcb')
            code, log = run(MAKE_UNPLACED, src, out)
            self.assertEqual(code, 0, log[-600:])
            self.assertIn('unlocked   1: U1', log)
            self.assertFalse(parse_kicad_pcb(evidence(out, 'pile')).footprints['U1'].locked)

    def test_a_bare_head_lock_is_removed(self):
        self._pile(lambda t: t.replace('(footprint "OLIMEX_IC-FP:SSOP-20W"',
                                       '(footprint "OLIMEX_IC-FP:SSOP-20W" locked', 1))

    def test_a_locked_child_is_removed(self):
        self._pile(lambda t: re.sub(r'(\(footprint "OLIMEX_IC-FP:SSOP-20W")',
                                    r'\1\n\t\t(locked)', t, count=1))


if __name__ == '__main__':
    unittest.main()
