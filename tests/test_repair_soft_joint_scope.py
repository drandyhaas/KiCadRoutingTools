"""repair_planes closes soft joints on its own nets only.

Its via-to-pad graze bridge used to run with no net scope, so a plane repair
of one net added a stub to every via on the board that grazes a same-net
pad, on nets the run never touched. Here a /GND plane repair runs on a board
with two grazing vias: one on /OUT_A_R (an unrelated net; it carries a short
track of its own, so it stays on its net when loaded) and one on /GND. The
/OUT_A_R via must be left as it is; the /GND one is still bridged.

    python3 tests/test_repair_soft_joint_scope.py
"""
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk_gnd.kicad_pcb')


def _via(x, y, net, n):
    return (f'\t(via\n\t\t(at {x} {y})\n\t\t(size 0.5)\n\t\t(drill 0.3)\n'
            f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "{net}")\n'
            f'\t\t(uuid "00000000-0000-4000-8000-00000000000{n}")\n\t)\n')


def _seg(x1, y1, x2, y2, net, n):
    return (f'\t(segment\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n'
            f'\t\t(width 0.2)\n\t\t(layer "F.Cu")\n\t\t(net "{net}")\n'
            f'\t\t(uuid "00000000-0000-4000-8000-00000000001{n}")\n\t)\n')


def _segs(board, name):
    nid = next(k for k, n in board.nets.items() if n.name == name)
    return {(round(s.start_x, 3), round(s.start_y, 3), round(s.end_x, 3),
             round(s.end_y, 3), s.layer)
            for s in board.segments if s.net_id == nid}


class TestRepairSoftJointScope(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        src = open(BOARD, encoding='utf-8').read()
        # R2.2 (/OUT_A_R) is 1.02 mm wide at x 118.41, C5.2 (/GND) 1.15 mm at
        # x 94.3: each via's barrel edge sits 0.05 mm off the pad's edge
        extra = (_via(119.22, 68.0, '/OUT_A_R', 2)
                 + _seg(119.22, 68.0, 120.6, 68.0, '/OUT_A_R', 2)
                 + _via(95.175, 76.17, '/GND', 1))
        i = src.index('\n\t(via') + 1
        cls.td = tempfile.TemporaryDirectory()
        cls.inp = os.path.join(cls.td.name, 'in.kicad_pcb')
        cls.out = os.path.join(cls.td.name, 'out.kicad_pcb')
        with open(cls.inp, 'w', encoding='utf-8', newline='\n') as f:
            f.write(src[:i] + extra + src[i:])
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'repair_planes.py'),
             cls.inp, cls.out, '--nets', '/GND', '--plane-layers', 'B.Cu',
             '--no-kicad-recheck'],
            capture_output=True, text=True, cwd=ROOT,
            env=dict(os.environ, PYTHONIOENCODING='utf-8', MSYS2_ARG_CONV_EXCL='*'))
        assert r.returncode == 0, r.stdout[-1500:] + r.stderr[-800:]
        cls.before, cls.after = parse_kicad_pcb(cls.inp), parse_kicad_pcb(cls.out)

    @classmethod
    def tearDownClass(cls):
        cls.td.cleanup()

    def test_unrelated_net_untouched(self):
        self.assertEqual(_segs(self.after, '/OUT_A_R'), _segs(self.before, '/OUT_A_R'))

    def test_own_net_joint_still_bridged(self):
        new = _segs(self.after, '/GND') - _segs(self.before, '/GND')
        self.assertTrue(any(95.175 in (s[0], s[2]) and 94.3 in (s[0], s[2]) for s in new), new)


if __name__ == '__main__':
    unittest.main()
