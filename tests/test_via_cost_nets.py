#!/usr/bin/env python3
"""route.py --via-cost-nets / --via-cost-nets-costs: a via penalty for some nets only.

Board: a wall of another net's track between two pads of XTAL and two pads of
SIG, open only near the top edge. Under the wall costs 10 mm plus two vias
(2 x 7.5 mm at the default via cost); the F.Cu detour round the wall's end is
about 40 mm. With XTAL's via cost raised, XTAL takes the detour and SIG still
takes its two vias.
"""
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCRIPT = os.path.join(ROOT, 'py_router', 'route.py')
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
_UUID = iter(f'00000000-0000-0000-0000-{i:012d}' for i in range(1, 1000))


def _fp(ref, x, y, net, name):
    return (f'  (footprint "t:P" (layer "F.Cu") (uuid "{next(_UUID)}") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 -1.5) (layer "F.SilkS"))\n'
            f'    (pad "1" smd rect (at 0 0) (size 0.8 0.8) (layers "F.Cu") (net {net} "{name}")))')


def board(third_pad=False):
    return '\n'.join([
        '(kicad_pcb (version 20240108) (generator pcbnew)',
        '  (general (thickness 1.6))',
        '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))',
        '  (net 0 "") (net 1 "XTAL") (net 2 "WALL") (net 3 "SIG")',
        '  (gr_rect (start 0 0) (end 30 40) (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))',
        _fp('A1', 10, 20, 1, 'XTAL'), _fp('A2', 20, 20, 1, 'XTAL'),
        *([_fp('A3', 8, 22, 1, 'XTAL')] if third_pad else []),
        _fp('B1', 10, 30, 3, 'SIG'), _fp('B2', 20, 30, 3, 'SIG'),
        _fp('W1', 15, 39, 2, 'WALL'), _fp('W2', 15, 3, 2, 'WALL'),
        f'  (segment (start 15 39) (end 15 3) (width 0.3) (layer "F.Cu") (net 2) (uuid "{next(_UUID)}"))',
        ')'])


def route(*args, third_pad=False):
    td = tempfile.mkdtemp()
    src, dst = os.path.join(td, 'in.kicad_pcb'), os.path.join(td, 'out.kicad_pcb')
    with open(src, 'w', encoding='utf-8') as fh:
        fh.write(board(third_pad))
    r = subprocess.run([sys.executable, '-X', 'utf8', SCRIPT, src, dst, '--nets', 'XTAL', 'SIG', *args],
                       capture_output=True, text=True, cwd=ROOT, timeout=900)
    if not os.path.exists(dst):
        return r, None
    out = parse_kicad_pcb(dst)
    return r, {name: sum(1 for v in out.vias if v.net_id == nid)
               for nid, name in ((1, 'XTAL'), (3, 'SIG'))}


class TestViaCostNets(unittest.TestCase):
    def test_default_both_take_vias(self):
        r, vias = route()
        self.assertEqual(r.returncode, 0, r.stdout[-2000:] + r.stderr[-2000:])
        self.assertEqual(vias, {'XTAL': 2, 'SIG': 2})

    def test_priced_net_detours_the_other_keeps_its_vias(self):
        r, vias = route('--via-cost-nets', 'XT*', '--via-cost-nets-costs', '500')
        self.assertEqual(r.returncode, 0, r.stdout[-2000:] + r.stderr[-2000:])
        self.assertEqual(vias, {'XTAL': 0, 'SIG': 2})
        self.assertIn('Per-net via cost for 1 net(s): XTAL=500', r.stdout)

    def test_multipoint_net_takes_its_cost(self):
        # a crystal net has three pads (pin, crystal, load cap): a multipoint net
        r, vias = route(third_pad=True)
        self.assertEqual(vias['XTAL'], 2, r.stdout[-2000:])
        r, vias = route('--via-cost-nets', 'XTAL', '--via-cost-nets-costs', '500', third_pad=True)
        self.assertEqual(r.returncode, 0, r.stdout[-2000:] + r.stderr[-2000:])
        self.assertEqual(vias, {'XTAL': 0, 'SIG': 2})

    def test_lengths_must_match(self):
        r, _ = route('--via-cost-nets', 'XTAL', 'SIG', '--via-cost-nets-costs', '500')
        self.assertNotEqual(r.returncode, 0)
        self.assertIn('--via-cost-nets-costs (1)', r.stdout + r.stderr)


if __name__ == '__main__':
    unittest.main()
