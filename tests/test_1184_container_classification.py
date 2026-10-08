"""#1184: a container is decided by GEOMETRY -- not by area, not by a lock.

`CONTAINER_RATIO` alone called One-Air-Max's BAT1 (an SMD 18650 holder,
87.5 x 50.6 mm, 0.62 of the board) a frame, so the seeder skipped every
courtyard test against it and seeded DC1 wholly inside it, while check_assembly
gated the pair because BAT1 was KiCad-locked. A lock and an area ratio cannot
tell BAT1 from rp2350's U8 (a Teensy frame the designer DOES put parts inside;
the issue's own comment). Geometry can (`legality.container_kinds`):

  * 'pin_frame' -- >= FRAME_DRILLED_FRAC drilled pads, none in the interior;
  * 'outline'   -- no copper and no holes at all;
  * never a part with a drawn courtyard, and never anything else.

A container is then exempt LOCKED OR NOT, a body is not exempt LOCKED OR NOT,
and the seeder and check_assembly read the one classifier.
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
from test_1212_container_pins import _frame_board            # noqa: E402

#: Every container on the tracked corpus, by kind. A change here is a change
#: to which parts the courtyard channel treats as frames: read it.
CORPUS_CONTAINERS = {
    'rp2350_fpga_eensy_prePlane.kicad_pcb': {'U8': 'pin_frame'},
    'watchy.kicad_pcb': {'REF**': 'outline'},
}


def _board(td, body, size=(44, 24)):
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
            f'  (gr_rect (start 0 0) (end {size[0]} {size[1]}) (stroke'
            ' (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
            + body + ')\n')
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def _holder(locked):
    """BAT1's shape: six SMD clips in two columns 40 mm apart, one drilled
    pad, a silk outline and a 0.06 mm F.Fab pin-1 dot (#1201's shape)."""
    clips = ''.join(
        f'    (pad "{i}" smd rect (at {x} {y}) (size 3 4) (layers "F.Cu")'
        f' (net 1 "N1"))\n'
        for i, (x, y) in enumerate([(-20, -8), (-20, 0), (-20, 8),
                                    (20, -8), (20, 0), (20, 8)], 1))
    lock = '    (locked yes)\n' if locked else ''
    return ('  (footprint "t:HOLDER" (layer "F.Cu") (at 22 12)\n' + lock
            + '    (property "Reference" "BAT1" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -21 -10) (end 21 10) (stroke (width 0.12)'
            ' (type default)) (layer "F.SilkS"))\n'
            '    (fp_circle (center -18 -9) (end -17.97 -9) (stroke (width'
            ' 0.05) (type default)) (layer "F.Fab"))\n'
            + clips +
            '    (pad "7" thru_hole circle (at -15 -9) (size 1.8 1.8)'
            ' (drill 1.0) (layers "*.Cu" "*.Mask") (net 1 "N1")))\n')


def _part(ref, x, y, h=2.0):
    return (f'  (footprint "t:P" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            f'    (fp_rect (start {-h} {-h}) (end {h} {h}) (stroke'
            f' (width 0.05) (type default)) (layer "F.CrtYd"))\n'
            f'    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu")'
            f' (net 2 "N2")))\n')


class TheCorpus(unittest.TestCase):

    def test_exactly_the_two_known_containers(self):
        import run_utils
        got = {}
        for b in run_utils.corpus_boards():
            path = os.path.join(ROOT, b)
            pcb = parse_kicad_pcb(path)
            k = legality.container_kinds(pcb, legality.part_local_bounds(
                pcb, path))
            if k:
                got[os.path.basename(b)] = k
        self.assertEqual(got, CORPUS_CONTAINERS)


class AnSmdHolderIsABody(unittest.TestCase):

    def _grade(self, locked):
        import pose_score
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, _holder(locked) + _part('DC1', 22, 12))
            pcb = parse_kicad_pcb(path)
            g = legality.grade_body_overlap(pcb, 0.2, pcb_file=path)
            st = pose_score.make_state(pcb, path, clearance=0.2)
            veto = st.candidate_veto('DC1', 22.0, 12.0, 0.0)
        return g, veto

    def test_unlocked(self):
        g, veto = self._grade(locked=False)
        self.assertEqual(g['containers'], {})
        self.assertTrue([p for p in g['courtyard_blocking_pairs']
                         if {p.a, p.b} == {'BAT1', 'DC1'}])
        self.assertEqual((veto or (None,))[0], 'courtyard', veto)

    def test_locked(self):
        """The issue: locked, the seeder exempted it and the grader did
        not. Now neither does, locked or not."""
        g, veto = self._grade(locked=True)
        self.assertEqual(g['containers'], {})
        self.assertTrue([p for p in g['courtyard_blocking_pairs']
                         if {p.a, p.b} == {'BAT1', 'DC1'}])
        self.assertEqual((veto or (None,))[0], 'courtyard', veto)


class AnOutlineNeverGates(unittest.TestCase):

    def test_a_locked_outline_is_still_waived(self):
        """watchy's e-paper REF** shape: no pads, a .Fab body over most of
        the board, parts under it. Locked, its pairs used to GATE (no class
        waiver for a locked part); an outline is waived for what it is."""
        outline = ('  (footprint "t:DISPLAY" (layer "F.Cu") (at 22 12)\n'
                   '    (locked yes)\n'
                   '    (property "Reference" "REF**" (at 0 0)'
                   ' (layer "F.SilkS"))\n'
                   '    (fp_rect (start -20 -10) (end 20 10) (stroke (width'
                   ' 0.1) (type default)) (layer "F.Fab")))\n')
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, outline + _part('U1', 22, 12))
            pcb = parse_kicad_pcb(path)
            g = legality.grade_body_overlap(pcb, 0.2, pcb_file=path)
        self.assertEqual(g['containers'], {'REF**': 'outline'})
        pair = [p for p in g['pairs'] if p.kind == 'courtyard'
                and {p.a, p.b} == {'REF**', 'U1'}]
        self.assertTrue(pair and pair[0].waiver == 'container_class', pair)
        self.assertFalse([p for p in g['courtyard_blocking_pairs']
                          if 'REF**' in (p.a, p.b)])


class TheGuards(unittest.TestCase):
    """One assertion per condition of the rule, each on a frame that is a
    container until that one condition changes."""

    def _kinds(self, td, extra=''):
        path = _frame_board(td, [])
        if extra:
            with open(path, encoding='utf-8') as fh:
                text = fh.read()
            text = text.replace('(net 1 "N1"))\n  )\n',
                                '(net 1 "N1"))\n' + extra + '  )\n', 1)
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(text)
        pcb = parse_kicad_pcb(path)
        return legality.container_kinds(pcb, legality.part_local_bounds(
            pcb, path))

    def test_the_ring_is_a_frame(self):
        with tempfile.TemporaryDirectory() as td:
            self.assertEqual(self._kinds(td), {'FR': 'pin_frame'})

    def test_an_interior_pin_makes_it_a_body(self):
        with tempfile.TemporaryDirectory() as td:
            self.assertEqual(self._kinds(td, (
                '    (pad "99" thru_hole circle (at 0 0) (size 1.8 1.8)'
                ' (drill 1.0) (layers "*.Cu" "*.Mask") (net 1 "N1"))\n')), {})

    def test_mostly_smd_copper_makes_it_a_body(self):
        smd = ''.join(
            f'    (pad "s{i}" smd rect (at {-17 + i} 13.5) (size 0.5 0.5)'
            f' (layers "F.Cu") (net 1 "N1"))\n' for i in range(30))
        with tempfile.TemporaryDirectory() as td:
            self.assertEqual(self._kinds(td, smd), {})

    def test_a_drawn_courtyard_makes_it_a_body(self):
        crt = ('    (fp_rect (start -20 -15) (end 20 15) (stroke (width'
               ' 0.05) (type default)) (layer "F.CrtYd"))\n')
        with tempfile.TemporaryDirectory() as td:
            self.assertEqual(self._kinds(td, crt), {})


class PoseIndependence(unittest.TestCase):

    def test_a_turn_does_not_make_a_container(self):
        """sonde_u's J1 covers 0.29 of the board; at 135 degrees its rotated
        RECT covered more than half and it became a 'container'. The
        classifier reads local geometry, so a pose cannot change it."""
        path = os.path.join(ROOT, 'kicad_files', 'sonde_u.kicad_pcb')
        pcb = parse_kicad_pcb(path)
        c = legality.CourtyardCensus(pcb, path)
        j = pcb.footprints['J1']
        g = c.grade({'J1': (j.x, j.y, 135.0)})
        self.assertNotIn('J1', g.containers)
        self.assertEqual(g.containers, c.containers)


if __name__ == '__main__':
    unittest.main()
