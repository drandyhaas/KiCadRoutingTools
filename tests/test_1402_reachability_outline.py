"""#1402: check_reachability walls the board outline, cutouts and slots.

`slack_field` rasterised segments, vias, copper pads and rule areas only, so a
net whose only way across the board is a pair of narrow webs between drilled
Edge.Cuts slots (a mouse-bite row) read PASSABLE while the router -- which
keeps copper `min_copper_edge_clearance` from the outline -- failed it with no
rippable blockers. With `edge_clearance` given (the CLI reads the board's own
floor) the outline, every cutout and slot is an obstacle at that clearance, and
an NPTH hole is an obstacle at the base clearance.

Fixture: the tracked esp_prog board plus one internal Edge.Cuts slot between
USB1 and the rest of /D_P. 0.3 mm webs (< 2 x 0.2 mm edge floor) close the
lane; 1.0 mm webs leave it open.
"""
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb              # noqa: E402
from placement import reachability                    # noqa: E402
from run_utils import evidence                        # noqa: E402

BOARD = evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'), 'tracked board')
SEED = (120.65, 100.0)          # the /D_P pad on the west side of the slot


def slotted(td, y0, y1):
    with open(BOARD, encoding='utf-8') as fh:
        text = fh.read()
    key = '\t(embedded_fonts no)\n)'
    assert text.count(key) == 1
    slot = ('\t(gr_rect\n\t\t(start 122 %g)\n\t\t(end 123 %g)\n\t\t(stroke\n\t\t\t(width 0.05)\n'
            '\t\t\t(type default)\n\t\t)\n\t\t(fill no)\n\t\t(layer "Edge.Cuts")\n'
            '\t\t(uuid "00000000-0000-0000-0000-00000000511e")\n\t)\n' % (y0, y1))
    path = os.path.join(td, 'slot.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text.replace(key, slot + key))
    return path


def reach(path, edge):
    pcb = parse_kicad_pcb(path)
    return reachability.pad_reachability(
        pcb, SEED, net_name='/D_P', track_mm=0.15, via_mm=0.6, base_clearance=0.2,
        step=0.03, margin_mm=12.0, edge_clearance=edge)


class TestOutlineIsAnObstacle(unittest.TestCase):
    def test_narrow_webs_cage_the_net_once_the_outline_is_walled(self):
        with tempfile.TemporaryDirectory() as td:
            path = slotted(td, 91.3, 105.2)         # 0.3 mm webs
            blind = reach(path, None)
            walled = reach(path, 0.2)
        self.assertFalse(blind.caged, 'the copper-only field never saw the slot')
        self.assertTrue(walled.caged, walled.bottleneck_mm)

    def test_wide_webs_stay_passable(self):
        with tempfile.TemporaryDirectory() as td:
            walled = reach(slotted(td, 92.5, 103.8), 0.2)      # 1.5 mm webs
        self.assertFalse(walled.caged, walled.bottleneck_mm)

    def test_the_unslotted_board_is_unchanged(self):
        self.assertFalse(reach(BOARD, 0.2).caged)


if __name__ == '__main__':
    unittest.main()
