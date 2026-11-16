"""#1750: a PLATED hole whose pads are no pins is a `mount_hole`.

`classify_part` knew a hole two ways: an NPTH-only pad set, or an item name
saying `MountingHole`. A house library's plated screw hole (`HOLE_M2_TH`: one
unnumbered, unnetted `thru_hole` pad) matched neither, so check_assembly gated
its containments against the switches it sits between and graded an
unmodified human board NOT BUILDABLE. A pad with no number answers to no
schematic pin, so a part made only of unnumbered, unnetted through-hole pads
is mechanical whatever it is called. A numbered pad or a netted one keeps the
part out of the rule, and a pad that carries no number FIELD says nothing.
"""

import os
import sys
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from placement.part_class import classify_part  # noqa: E402


def _pad(pad_type='thru_hole', number='', net=0, **extra):
    p = SimpleNamespace(pad_type=pad_type, net_id=net, pinfunction='',
                        drill=2.2, size_x=4.0, size_y=4.0, layers=['*.Cu'],
                        **extra)
    if number is not None:
        p.pad_number = number
    return p


def _cls(name, pads, ref='H1'):
    return classify_part(SimpleNamespace(footprint_name=name, pads=pads),
                         ref).name


class PlatedHole(unittest.TestCase):

    def test_unnumbered_unnetted_plated_hole_is_a_mount_hole(self):
        self.assertEqual(_cls('house:HOLE_M2_TH', [_pad()]), 'mount_hole')
        # several holes, plated and not, under a part's own name
        self.assertEqual(_cls('house:Bracket', [_pad(), _pad(),
                                                _pad('np_thru_hole')], 'U1'),
                         'mount_hole')

    def test_a_pin_keeps_the_part_out(self):
        # a numbered pad answers to a pin (a THT test pad)
        self.assertNotEqual(_cls('house:Pad_D2.0mm', [_pad(number='1')],
                                 'X1'), 'mount_hole')
        # a netted pad (a stitching-via footprint)
        self.assertNotEqual(_cls('house:VIA-0.6mm', [_pad(net=5)], 'X1'),
                            'mount_hole')
        # one real pin among unnumbered holes
        self.assertNotEqual(_cls('house:Module', [_pad(), _pad(number='1')],
                                 'X1'), 'mount_hole')
        # an SMD pad is no hole
        self.assertNotEqual(_cls('house:Pad', [_pad('smd')], 'X1'),
                            'mount_hole')

    def test_an_absent_number_says_nothing(self):
        self.assertNotEqual(_cls('house:Thing', [_pad(number=None)], 'X1'),
                            'mount_hole')


if __name__ == '__main__':
    unittest.main()
