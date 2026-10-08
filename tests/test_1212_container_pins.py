"""#1212: a pin frame is graded on its PINS -- KiCad's pth_inside_courtyard.

rp2350's U8 is a Teensy 4.0 frame: 66 drilled pins on the board edges and
nothing drawn. Graded as its pad-bbox RECT it covered the board, so
check_assembly listed 60 courtyard pairs `X <-> U8` -- 59 not physical -- and
could not locate the real one: on fa10 seed 4, SW1's courtyard over U8 pins 16
and 17, which kicad-cli reports as two pth_inside_courtyard errors and which no
repo checker saw. The seeder skipped every courtyard test against a container,
which is how SW1 was seated there.

Now (`legality.container_kinds`, `CourtyardCensus._pin_pairs`):

  * a pin frame's rect pairs leave the courtyard channel;
  * each of its drilled HOLES (not the copper ring) is tested against every
    other part's occupancy, kind 'pin_in_courtyard', gating absolutely in
    check_assembly (its eighth conjunct);
  * the search refuses a pose on a pin (`container_pin`), through the same
    function the grader runs.
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

RP2350 = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
#: Seed 4's pose for SW1 (the issue's reproduction).
SW1_ON_PINS = (147.8, 119.135, 0.0)


def _frame_board(td, others):
    """A 40 x 30 board ringed by a 24-pin THT frame FR (nothing drawn: its
    occupancy is its pad bbox, like U8's), plus `others`:
    [(ref, x, y, half)] F-side parts with a square courtyard."""
    pins = []
    n = 0
    for i in range(8):
        for y in (1.5, 28.5):
            n += 1
            pins.append((n, 3.0 + i * 4.8, y))
    for j in range(4):
        for x in (1.5, 38.5):
            n += 1
            pins.append((n, x, 7.0 + j * 5.0))
    fr = ''.join(
        f'    (pad "{k}" thru_hole circle (at {x - 20} {y - 15}) (size 1.8 1.8)'
        f' (drill 1.0) (layers "*.Cu" "*.Mask") (net 1 "N1"))\n'
        for k, x, y in pins)
    parts = ('  (footprint "t:FRAME" (layer "F.Cu") (at 20 15)\n'
             '    (property "Reference" "FR" (at 0 0) (layer "F.SilkS"))\n'
             + fr + '  )\n')
    for ref, x, y, h in others:
        parts += (f'  (footprint "t:P" (layer "F.Cu") (at {x} {y})\n'
                  f'    (property "Reference" "{ref}" (at 0 0)'
                  f' (layer "F.SilkS"))\n'
                  f'    (fp_rect (start {-h} {-h}) (end {h} {h}) (stroke'
                  f' (width 0.05) (type default)) (layer "F.CrtYd"))\n'
                  f'    (pad "1" smd rect (at 0 0) (size 0.4 0.4)'
                  f' (layers "F.Cu") (net 2 "N2")))\n')
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
            '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1)'
            ' (type default)) (layer "Edge.Cuts"))\n' + parts + ')\n')
    path = os.path.join(td, 'f.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


class Synthetic(unittest.TestCase):

    def test_a_hole_is_a_hit_and_the_ring_is_not(self):
        with tempfile.TemporaryDirectory() as td:
            # P's courtyard (1.0 half) is centred on pin 1's hole (3, 1.5);
            # Q's (0.3 half) only touches pin 3's copper ring, 0.65 mm from
            # its hole centre -- outside the 0.5 mm drill radius.
            path = _frame_board(td, [('P', 3.0, 1.5, 1.0),
                                     ('Q', 7.8 + 0.95, 1.5, 0.3),
                                     ('MID', 20.0, 15.0, 2.0)])
            pcb = parse_kicad_pcb(path)
            g = legality.grade_body_overlap(pcb, 0.2, pcb_file=path)
            self.assertEqual(g['containers'], {'FR': 'pin_frame'})
            pins = {(p.a, p.b): p.pins for p in g['pin_in_courtyard_pairs']}
            self.assertEqual(pins, {('FR', 'P'): ('1',)})
            # The frame's rect pairs are gone: MID sits inside the frame.
            self.assertFalse([p for p in g['pairs'] if p.kind == 'courtyard'
                              and 'FR' in (p.a, p.b)])

    def test_check_assembly_gates_it_without_a_baseline(self):
        """On the pin ALONE: P's courtyard covers pin 1's hole and its own
        pad is 0.5 mm clear of the ring, so no other conjunct can fire (P
        centred on the hole, its pad on the ring, gated as a pad
        intersection too and hid a dropped conjunct)."""
        from run_utils import check
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [('P', 3.0, 3.1, 2.0)])
            g = legality.grade_body_overlap(parse_kicad_pcb(path), 0.2,
                                            pcb_file=path)
            self.assertEqual((g['blocking'], g['containment_blocking'],
                              g['pin_in_courtyard']), (0, 0, 1))
            check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_tools', 'check_assembly.py'), path],
                  refuse='PIN IN COURTYARD', code=4)


class RepairChargesIt(unittest.TestCase):
    """legalize moves a part off a frame pin -- and the PIN census is what
    makes it: the fixture's only defect is the pin (no pad conflict, no
    containment, no other courtyard pair), so nothing else charges P."""

    def test_legalize_moves_the_part_off_the_pin(self):
        import subprocess
        with tempfile.TemporaryDirectory() as td:
            # P's courtyard (2.0 half) covers pin 1's hole; its own pad sits
            # 0.5 mm clear of pin 1's copper ring.
            path = _frame_board(td, [('P', 3.0, 3.1, 2.0)])
            g = legality.grade_body_overlap(parse_kicad_pcb(path), 0.2,
                                            pcb_file=path)
            self.assertEqual((g['blocking'], g['containment_blocking']),
                             (0, 0))
            self.assertEqual([(p.a, p.b, p.kind) for p in g['pairs']],
                             [('FR', 'P', 'pin_in_courtyard')])
            out = os.path.join(td, 'o.kicad_pcb')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_placer', 'place_reconstruct.py'),
                 path, out, '--stages', 'legalize', '--clearance', '0.2'],
                capture_output=True, text=True, encoding='utf-8',
                errors='replace', cwd=ROOT, timeout=600,
                env=dict(os.environ, KRT_NO_BANNER='1'))
            self.assertIn('Pin census: 1 part(s)', r.stdout, r.stdout[-800:])
            self.assertIn('legalize: 1 repaired', r.stdout, r.stdout[-800:])
            after = legality.grade_body_overlap(parse_kicad_pcb(out), 0.2,
                                                pcb_file=out)
            self.assertEqual(after['pin_in_courtyard'], 0)


class TheSeederHelpers(unittest.TestCase):
    """The seeder's own two readings of a frame pin: a DECLARED pose
    (`_fixed_pose_check`) and the eviction rung's seated count
    (`_seated_violations`)."""

    def _state(self, td):
        import pose_score
        path = _frame_board(td, [('P', 20.0, 15.0, 2.0)])
        return pose_score.make_state(parse_kicad_pcb(path), path,
                                     clearance=0.2)

    def test_a_declared_pose_on_a_pin_names_the_frame(self):
        from placement.seeder import _fixed_pose_check
        with tempfile.TemporaryDirectory() as td:
            st = self._state(td)
            _how, reasons, conflicts = _fixed_pose_check(
                st, 'P', (3.0, 3.1, 0.0), {'FR': (20.0, 15.0, 0.0)})
            self.assertEqual(list(conflicts), ['FR'])
            self.assertIn("sits on FR's pin(s) 1", conflicts['FR'])
            self.assertEqual(sum('pin_in_courtyard' in r for r in reasons), 1)
            # The frame's DECLARED pose decides, not where it sits now:
            # moved 10 mm right, its pin 1 is nowhere near P.
            _how, _r, conflicts = _fixed_pose_check(
                st, 'P', (3.0, 3.1, 0.0), {'FR': (30.0, 15.0, 0.0)})
            self.assertNotIn('FR', conflicts)

    def test_a_declared_waiver_records_the_pins_instead(self):
        from placement.seeder import _fixed_pose_check, _waived_what
        with tempfile.TemporaryDirectory() as td:
            st = self._state(td)
            out = {}
            _how, _r, conflicts = _fixed_pose_check(
                st, 'P', (3.0, 3.1, 0.0), {'FR': (20.0, 15.0, 0.0)},
                waived={frozenset(('P', 'FR'))}, waived_out=out)
            self.assertNotIn('FR', conflicts)
            self.assertEqual(out, {'FR': {'pins': ['1']}})
            self.assertEqual(_waived_what(out['FR']), 'pin(s) 1')

    def test_a_seated_part_on_a_pin_is_a_violation(self):
        from placement.seeder import _seated_violations
        with tempfile.TemporaryDirectory() as td:
            st = self._state(td)
            self.assertEqual(_seated_violations(st, {'P', 'FR'})[0], 0)
            st.parts['P'].x, st.parts['P'].y = 3.0, 3.1
            self.assertEqual(_seated_violations(st, {'P', 'FR'})[0], 1)


def _add(path, text):
    """Append footprint text to a `_frame_board` board."""
    with open(path, encoding='utf-8') as fh:
        body = fh.read().rstrip()
    assert body.endswith(')')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(body[:-1] + text + ')\n')


def _pro(path, **rules):
    import json
    with open(os.path.splitext(path)[0] + '.kicad_pro', 'w',
              encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rule_severities': rules}}},
                  fh)


def _assembly(path, *extra):
    import json
    import subprocess
    jp = path + '.json'
    r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(
        ROOT, 'py_tools', 'check_assembly.py'), path, '--json', jp] +
        list(extra), capture_output=True, text=True, encoding='utf-8',
        errors='replace', cwd=ROOT)
    with open(jp, encoding='utf-8') as fh:
        return r.returncode, json.load(fh), r.stdout


#: P's courtyard over pin 1's hole, its pad clear of the ring: the pin alone.
PIN_ONLY = [('P', 3.0, 3.1, 2.0)]


class AsKiCadGradesIt(unittest.TestCase):
    """The pin is KiCad's pth/npth_inside_courtyard: its own severity, the
    DRAWN courtyards on both faces, nothing where none is drawn."""

    def test_courtyards_ignored_does_not_waive_a_pin(self):
        """The phase-3 verifier's BLOCKING case: courtyards_overlap=ignore
        with pth_inside_courtyard=error (StickHub's and One-Air-Max's kind
        of project) -- KiCad reports the pin, so the grade gates it and the
        search refuses it."""
        import pose_score
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, PIN_ONLY)
            _pro(path, courtyards_overlap='ignore',
                 pth_inside_courtyard='error')
            rc, doc, _out = _assembly(path)
            self.assertEqual((rc, doc['pin_in_courtyard']), (4, 1), doc)
            st = pose_score.make_state(parse_kicad_pcb(path), path,
                                       clearance=0.2)
            self.assertEqual(st.candidate_veto('P', 3.0, 3.1, 0.0,
                                               exclude=set()),
                             ('container_pin', 'FR'))

    def test_pth_ignored_waives_it(self):
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, PIN_ONLY)
            _pro(path, courtyards_overlap='error',
                 pth_inside_courtyard='ignore')
            rc, doc, _out = _assembly(path)
        self.assertEqual((rc, doc['pin_in_courtyard']), (0, 0), doc)

    def test_the_npth_rule_is_its_own(self):
        """An NPTH pin follows npth_inside_courtyard, a PTH pin pth_: the
        census keeps them in separate pairs, and only its own rule's ignore
        waives one."""
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, PIN_ONLY)
            c = legality.CourtyardCensus(parse_kicad_pcb(path), path)
            c.pin_severity = {'pth': 'error', 'npth': 'ignore'}
            raw = c._pin_pairs({r: c.graded_part(r) for r in c.lbs}, {})
            self.assertEqual([q.hole for q in raw], ['pth'])
            npth = raw[0]._replace(hole='npth')
            g = c._select({r: c.graded_part(r) for r in c.lbs}, [], None,
                          [raw[0], npth])
        self.assertEqual([(q.hole, q.waived) for q in g.pin_pairs],
                         [('pth', False), ('npth', True)])

    def test_a_far_face_courtyard_over_a_pin(self):
        """E5: P draws a small F.CrtYd clear of pin 1 and a B.CrtYd over
        it -- KiCad reports pth_inside_courtyard; the own-face occupancy
        alone missed it."""
        part = ('  (footprint "t:P2" (layer "F.Cu") (at 8 6)\n'
                '    (property "Reference" "P" (at 0 0) (layer "F.SilkS"))\n'
                '    (fp_rect (start -0.5 -0.5) (end 0.5 0.5) (stroke (width'
                ' 0.05) (type default)) (layer "F.CrtYd"))\n'
                '    (fp_rect (start -6 -5.5) (end -4 -3.5) (stroke (width'
                ' 0.05) (type default)) (layer "B.CrtYd"))\n'
                '    (pad "1" thru_hole circle (at 0 0) (size 0.8 0.8)'
                ' (drill 0.4) (layers "*.Cu" "*.Mask") (net 2 "N2")))\n')
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [])
            _add(path, part)
            g = legality.grade_body_overlap(parse_kicad_pcb(path), 0.2,
                                            pcb_file=path)
        self.assertEqual([(q.a, q.b, q.pins) for q in
                          g['pin_in_courtyard_pairs']],
                         [('FR', 'P', ('1',))])

    def test_a_part_drawing_no_courtyard_is_listed_not_gated(self):
        """E6 / G1: P has only an F.Fab rect over pin 1. KiCad has no
        courtyard to report; the pair is listed, and gates nothing."""
        part = ('  (footprint "t:P3" (layer "F.Cu") (at 3 3.1)\n'
                '    (property "Reference" "P" (at 0 0) (layer "F.SilkS"))\n'
                '    (fp_rect (start -2 -2) (end 2 2) (stroke (width 0.1)'
                ' (type default)) (layer "F.Fab"))\n'
                '    (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers'
                ' "F.Cu") (net 2 "N2")))\n')
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [])
            _add(path, part)
            rc, doc, _out = _assembly(path)
            g = legality.grade_body_overlap(parse_kicad_pcb(path), 0.2,
                                            pcb_file=path)
        self.assertEqual((rc, doc['pin_in_courtyard']), (0, 0), doc)
        listed = [q for q in g['pairs'] if q.kind == 'pin_in_courtyard']
        self.assertEqual([(q.b, q.basis) for q in listed], [('P', 'fab')])

    def test_an_intent_waiver_excuses_it(self):
        """G3: the pair named in the intent's overlap_waivers."""
        import json
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, PIN_ONLY)
            ip = os.path.join(td, 'i.json')
            with open(ip, 'w', encoding='utf-8') as fh:
                json.dump({'schema': 1, 'kind': 'floorplan-intent',
                           'units': 'mm', 'overlap_waivers': [
                               {'pair': ['FR', 'P'], 'reason': 'test'}]}, fh)
            rc, doc, _out = _assembly(path, '--intent', ip)
        self.assertEqual((rc, doc['pin_in_courtyard']), (0, 0), doc)

    def test_a_logo_is_never_graded_on_a_pin(self):
        """G5: a pad-less silk logo (a synthetic part) over a pin forms no
        pin pair at all."""
        logo = ('  (footprint "t:LOGO" (layer "F.Cu") (at 3 1.5)\n'
                '    (property "Reference" "G1" (at 0 0) (layer "F.SilkS"))\n'
                '    (fp_rect (start -2 -1) (end 2 1) (stroke (width 0.1)'
                ' (type default)) (layer "F.SilkS")))\n')
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [])
            _add(path, logo)
            g = legality.grade_body_overlap(parse_kicad_pcb(path), 0.2,
                                            pcb_file=path)
        self.assertFalse([q for q in g['pairs']
                          if q.kind == 'pin_in_courtyard'], g['pairs'])

    def test_the_json_publishes_the_count_and_pairs(self):
        """G14."""
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, PIN_ONLY)
            rc, doc, out = _assembly(path)
        self.assertEqual(rc, 4)
        self.assertEqual(doc['pin_in_courtyard'], 1)
        self.assertEqual(len(doc['pin_in_courtyard_pairs']), 1)
        self.assertIn('P over FR pin(s) 1', out)


class AMovingFrame(unittest.TestCase):
    """G6 / G8: the frame is the part being moved."""

    def _board(self, td):
        # Q sits between pins 1 (x=3) and 3 (x=7.8) on the y=1.5 row; FR
        # moved 2.4 mm right puts pin 1's hole under it. Q's FILE pose is
        # the board centre, far from every pin.
        path = _frame_board(td, [('Q', 20.0, 15.0, 0.5)])
        return path

    def test_pin_hits_sees_every_part(self):
        with tempfile.TemporaryDirectory() as td:
            path = self._board(td)
            c = legality.CourtyardCensus(parse_kicad_pcb(path), path)
            hits = c.pin_hits('FR', (22.4, 15.0, 0.0),
                              {'Q': (5.4, 1.5, 0.0)})
        self.assertEqual([(q.a, q.b) for q in hits], [('FR', 'Q')])

    def test_the_search_reads_the_states_poses(self):
        import pose_score
        with tempfile.TemporaryDirectory() as td:
            path = self._board(td)
            st = pose_score.make_state(parse_kicad_pcb(path), path,
                                       clearance=0.2)
            self.assertIsNone(st._pin_conflict_at('FR', 22.4, 15.0, 0.0))
            st.apply_move('Q', 5.4, 1.5, 0.0)
            hit = st._pin_conflict_at('FR', 22.4, 15.0, 0.0)
        self.assertEqual((hit.a, hit.b) if hit else None, ('FR', 'Q'))

    def test_an_absent_frame_refuses_no_declared_pose(self):
        """G16: a declared pose is judged against its OBSTACLES; a frame
        that is not one (still unseated) refuses nothing."""
        import pose_score
        from placement.seeder import _fixed_pose_check
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [('P', 20.0, 15.0, 2.0)])
            st = pose_score.make_state(parse_kicad_pcb(path), path,
                                       clearance=0.2)
            _how, _r, conflicts = _fixed_pose_check(st, 'P', (3.0, 3.1, 0.0),
                                                    {})
        self.assertNotIn('FR', conflicts)


class TheRestOfTheChain(unittest.TestCase):

    def test_the_optimizers_overlap_leaves_the_frame_out(self):
        """G9: rp2350's search overlap without U8's rect (it read 425.18 mm2
        with it)."""
        import pose_score
        pcb = parse_kicad_pcb(RP2350)
        st = pose_score.make_state(pcb, RP2350, clearance=0.2)
        self.assertLess(st.legality_metrics()['overlap_area'], 100.0)

    def test_converge_points_a_turned_pin_veto_at_rank_rotations(self):
        """G10."""
        import converge
        self.assertIn('container_pin', converge._NEIGHBOUR_CHECKS)
        txt = converge._in_place_clause('SW1', {
            'dropped_in_place_by': [{'rot': 90.0, 'check': 'container_pin',
                                     'blocker': 'U8'}],
            'in_place_evaluated': True, 'input_rotation': 0.0})
        self.assertIn('rank_rotations.py', txt)

    def test_board_score_names_the_conjunct(self):
        """G15."""
        import board_score
        self.assertIn('pin_in_courtyard', board_score.ASSEMBLY_CONJUNCTS)
        r = board_score.assembly_component(
            {'blocking': 0, 'buildable': False, 'verdict': 'NOT BUILDABLE',
             'locked_contacts': 0, 'coincident_origins': 0,
             'containment_blocking': 0, 'courtyard_blocking_gating': None,
             'oob_pad_copper_gating_count': 0, 'mating_keepout_count': 0,
             'pin_in_courtyard': 1}, 4)
        self.assertEqual(r['conjuncts_fired'], ['pin_in_courtyard'])
        self.assertEqual(r['live_conjuncts_fired'], ['pin_in_courtyard'])

    def _legalize(self, td, path, *extra):
        import subprocess
        out = os.path.join(td, 'o.kicad_pcb')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_placer', 'place_reconstruct.py'),
             path, out, '--stages', 'legalize', '--clearance', '0.2']
            + list(extra), capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT, timeout=600,
            env=dict(os.environ, KRT_NO_BANNER='1'))
        return r, out

    def test_the_repair_never_moves_a_locked_part(self):
        """G11."""
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, PIN_ONLY)
            with open(path, encoding='utf-8') as fh:
                text = fh.read()
            text = text.replace('(footprint "t:P" (layer "F.Cu") (at 3.0 3.1)',
                                '(footprint "t:P" (layer "F.Cu") (at 3.0 3.1)'
                                '\n    (locked yes)', 1)
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(text)
            r, out = self._legalize(td, path)
            self.assertIn('pin_in_courtyard P over FR pin(s) 1: P is '
                          'file-locked', r.stdout, r.stdout[-1200:])
            fp = parse_kicad_pcb(out).footprints['P']
        self.assertEqual((fp.x, fp.y), (3.0, 3.1))

    def test_the_repair_honours_a_declared_pair(self):
        """G13."""
        import json
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, PIN_ONLY)
            ip = os.path.join(td, 'i.json')
            with open(ip, 'w', encoding='utf-8') as fh:
                json.dump({'schema': 1, 'kind': 'floorplan-intent',
                           'units': 'mm', 'overlap_waivers': [
                               {'pair': ['FR', 'P'], 'reason': 'test'}]}, fh)
            r, _out = self._legalize(td, path, '--intent', ip)
        self.assertNotIn('Pin census', r.stdout, r.stdout[-1200:])


class OnRp2350(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.pcb = parse_kicad_pcb(RP2350)
        cls.census = legality.CourtyardCensus(cls.pcb, RP2350)

    def test_the_designers_board_is_clean(self):
        g = self.census.grade()
        self.assertEqual(g.pin_pairs, [])
        self.assertFalse([p for p in g.pairs if 'U8' in (p.a, p.b)])

    def test_sw1_on_pins_16_and_17(self):
        """The issue's case, in memory: exactly SW1 x U8, pins 16 and 17 --
        the two pth_inside_courtyard kicad-cli reports there."""
        g = self.census.grade({'SW1': SW1_ON_PINS})
        self.assertEqual([(p.a, p.b, sorted(p.pins)) for p in g.pin_blocking],
                         [('SW1', 'U8', ['16', '17'])])
        hits = self.census.pin_hits('SW1', SW1_ON_PINS)
        self.assertEqual([(p.a, p.b) for p in hits], [('SW1', 'U8')])

    def test_the_search_refuses_it_through_the_graders_function(self):
        import pose_score
        calls = []
        real = legality.CourtyardCensus._pin_pairs

        def spy(self, *a, **k):
            calls.append(1)
            return real(self, *a, **k)
        st = pose_score.make_state(self.pcb, RP2350, clearance=0.2)
        # Everything but U8 lifted: on the designer's board C20 sits there
        # and its courtyard would answer first.
        others = set(st.parts) - {'SW1', 'U8'}
        legality.CourtyardCensus._pin_pairs = spy
        try:
            veto = st.candidate_veto('SW1', *SW1_ON_PINS, exclude=others)
        finally:
            legality.CourtyardCensus._pin_pairs = real
        self.assertEqual(veto, ('container_pin', 'U8'))
        self.assertTrue(calls, 'the search did not call the grader')
        # ...and it stays a violation for the optimizer.
        self.assertGreater(st.violation_parts('SW1', *SW1_ON_PINS,
                                              exclude=others)[1], 0.0)


if __name__ == '__main__':
    unittest.main()
