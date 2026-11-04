"""#1424: place_pose's `legal` agrees with check_assembly on a castellated module.

`grade_pad_legality`'s whole-part census (`oob_pad_count` / `oob_pad_amount`:
the pad extent against the outline inflated by the grading clearance) counted
a castellated module's half-holes as pad copper off the board, while the
per-pad channel check_assembly gates on exempts them (#1096) -- so `legal`
(place_pose) was false on a module sitting on its edge where `buildable`
(check_assembly) was true. The census now exempts a part whose other pads and
holes are inside the gate while every castellated pad still straddles the
outline.

Fixture: the tracked rp2350 board with U4, U5 and C6 dropped (their custom
pads would mask the module's contribution). U8 is the Teensy-style module
with 33 castellated half-holes on its edges.
"""
import os
import re
import sys
import unittest
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import iter_footprint_blocks, parse_kicad_pcb   # noqa: E402
from placement.legality import grade_pad_legality                 # noqa: E402
from run_utils import evidence                                    # noqa: E402

BOARD = evidence(os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb'),
                 'tracked board')


def variant(td, name, drop=('U4', 'U5', 'C6'), move_u8=None):
    with open(BOARD, encoding='utf-8', newline='') as fh:
        text = fh.read()
    out, last = [], 0
    for start, end, fp_text, _raw, key in iter_footprint_blocks(text):
        if key in drop:
            out.append(text[last:start])
            last = end
        elif key == 'U8' and move_u8 is not None:
            out.append(text[last:start])
            out.append(re.sub(r'\(at\s+\S+\s+\S+(\s+\S+)?\)',
                              lambda m: '(at %g %g%s)' % (move_u8[0], move_u8[1], m.group(1) or ''),
                              fp_text, count=1))
            last = end
    out.append(text[last:])
    path = os.path.join(td, name)
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(''.join(out))
    return path


def grade(path):
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return grade_pad_legality(parse_kicad_pcb(path), 0.09, pcb_file=path)


class TestCastellatedCensus(unittest.TestCase):
    def test_a_module_on_its_edge_is_not_counted_off_the_board(self):
        with tempfile.TemporaryDirectory() as td:
            g = grade(variant(td, 'edge.kicad_pcb'))
        self.assertEqual(g['oob_pad_count'], 0, g['oob_pad_refs'])
        self.assertEqual(g['oob_pad_amount'], 0.0)

    def test_a_module_parked_wholly_off_the_board_still_counts(self):
        with tempfile.TemporaryDirectory() as td:
            g = grade(variant(td, 'away.kicad_pcb', move_u8=(400.0, 400.0)))
        self.assertEqual([r[0] for r in g['oob_pad_refs']], ['U8'], g['oob_pad_refs'])


if __name__ == '__main__':
    unittest.main()
