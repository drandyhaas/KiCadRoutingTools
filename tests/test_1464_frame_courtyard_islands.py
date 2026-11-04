"""#1464: a container that KEEPS a drawn courtyard is graded on it, as drawn.

A courtyard covering little of a container's frame -- rings round its posts
-- leaves the part a frame (`legality.FRAME_COURTYARD_COVER`), so its rect is
no body. The rings are still courtyards KiCad's courtyards_overlap tests, so
the grader (`CourtyardCensus._frame_court_pairs`), the seat (through
`pin_hits`, the census the seat already asks about a frame's pins) and the
anneal (`QuenchState.frame_court_boxes`) all read them, island by island.
"""

import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import legality                               # noqa: E402
import pose_score                                            # noqa: E402
from test_1212_container_pins import _frame_board            # noqa: E402

#: Board coordinates of four of the frame's pins, ringed r = 1.5 mm.
RINGED = ((3.0, 1.5), (36.6, 1.5), (3.0, 28.5), (36.6, 28.5))
#: P (a 2 x 2 mm courtyard) on the first ring's band, 0.5 mm deep, its pad
#: and courtyard clear of the pin's copper and hole.
ON_RING = (3.0, 3.5, 0.0)
CLEAR = (20.0, 15.0, 0.0)


def _board(td, rings=True):
    path = _frame_board(td, [('P', 20, 15, 1.0)])
    if rings:
        crt = ''.join(
            f'    (fp_circle (center {x - 20} {y - 15}) (end {x - 18.5}'
            f' {y - 15}) (stroke (width 0.05) (type default)) (fill no)'
            f' (layer "F.CrtYd"))\n' for x, y in RINGED)
        with open(path, encoding='utf-8') as fh:
            text = fh.read()
        text = text.replace('(net 1 "N1"))\n  )\n',
                            '(net 1 "N1"))\n' + crt + '  )\n', 1)
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(text)
    return path


def _ring_pairs(g):
    return [p for p in g.pairs
            if p.basis == legality.CourtyardCensus.FRAME_COURT_BASIS]


class TheGraderReadsTheRings(unittest.TestCase):

    def test_a_part_on_a_ring_blocks_and_gates(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td)
            pcb = parse_kicad_pcb(path)
            c = legality.CourtyardCensus(pcb, path)
            self.assertEqual(c.court_frames, {'FR'})
            self.assertEqual(c.kinds, {'FR': 'pin_frame'})
            g = c.grade({'P': ON_RING}, moved={'P'})
            pairs = _ring_pairs(g)
            self.assertEqual([(p.a, p.b, p.kind) for p in pairs],
                             [('FR', 'P', 'courtyard')])
            self.assertFalse(pairs[0].waived)
            self.assertEqual([(p.a, p.b) for p in g.gating], [('FR', 'P')])
            self.assertEqual(g.pin_blocking, [])      # no pin rule involved
            # control: clear of the rings, inside the frame
            self.assertEqual(_ring_pairs(c.grade({'P': CLEAR})), [])

    def test_no_rings_no_pair(self):
        """A frame drawing no courtyard is unchanged: its rect is no body."""
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, rings=False)
            pcb = parse_kicad_pcb(path)
            c = legality.CourtyardCensus(pcb, path)
            self.assertEqual(c.court_frames, frozenset())
            g = c.grade({'P': ON_RING}, moved={'P'})
            self.assertEqual([p for p in g.pairs if 'FR' in (p.a, p.b)
                              and p.kind == 'courtyard'], [])

    def test_the_project_severity_waives_it(self):
        """courtyards_overlap's own severity, like any courtyard pair."""
        with tempfile.TemporaryDirectory() as td:
            path = _board(td)
            pcb = parse_kicad_pcb(path)
            c = legality.CourtyardCensus(pcb, path, courtyard_severity='ignore')
            g = c.grade({'P': ON_RING}, moved={'P'})
            self.assertTrue(all(p.waived for p in _ring_pairs(g)))
            self.assertEqual(g.gating, [])


class TheSeatAndTheAnnealReadThem(unittest.TestCase):

    def test_the_seat_refuses_the_ring(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td)
            st = pose_score.make_state(parse_kicad_pcb(path), path)
            self.assertEqual(st.court_frame_refs, {'FR'})
            self.assertFalse(st.candidate_valid('P', *ON_RING))
            hit = st._pin_conflict_at('P', *ON_RING)
            self.assertEqual((hit.a, hit.b, hit.kind), ('FR', 'P', 'courtyard'))
            self.assertTrue(st.candidate_valid('P', *CLEAR))

    def test_the_anneal_prices_each_island(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td)
            st = pose_score.make_state(parse_kicad_pcb(path), path)
            boxes = st.frame_court_boxes('FR')
        self.assertEqual(len(boxes), 4)
        for (x, y), b in zip(sorted(RINGED), sorted(boxes)):
            self.assertEqual(b[4], frozenset(('F',)))
            self.assertAlmostEqual((b[0] + b[2]) / 2, x, places=3)
            self.assertAlmostEqual((b[1] + b[3]) / 2, y, places=3)


if __name__ == '__main__':
    unittest.main()
