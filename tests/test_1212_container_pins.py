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
        self.assertIn('P over FR PTH pin(s) 1', out)


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


def _frame_board_npth(td, others, npth):
    """`_frame_board` with the pins in `npth` ({pin: new number}) made
    unplated -- np_thru_hole, no net; '' is an unnumbered NPTH."""
    path = _frame_board(td, others)
    with open(path, encoding='utf-8') as fh:
        text = fh.read()
    for pin, new in npth.items():
        old = f'(pad "{pin}" thru_hole circle'
        assert text.count(old) == 1, pin
        i = text.index(old)
        j = text.index('\n', i)
        line = text[i:j].replace(old, f'(pad "{new}" np_thru_hole circle')
        text = text[:i] + line.replace(' (net 1 "N1")', '') + text[j:]
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def _crtyd_part(ref, at, shapes, pad=True):
    """An F-side part at `at` drawing `shapes`: [(layer, s-expr body)]."""
    s = (f'  (footprint "t:{ref}" (layer "F.Cu") (at {at[0]} {at[1]})\n'
         f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n')
    for layer, body in shapes:
        s += (f'    ({body} (stroke (width 0.05) (type default))'
              f' (layer "{layer}"))\n')
    if pad:
        s += ('    (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu")'
              ' (net 2 "N2"))\n')
    return s + '  )\n'


#: An L whose bbox covers pin 1's hole and whose outline does not, for a
#: part at (6, 4): its bars are x 5..7 (all y) and y 5..7 (x 2..7).
L_SHAPE = ('fp_poly (pts (xy -1 -3.5) (xy 1 -3.5) (xy 1 3) (xy -4 3)'
           ' (xy -4 1) (xy -1 1))')
#: P over pin 1 (made NPTH) and pin 3 (PTH) of the y = 1.5 row, its own pad
#: clear of both rings.
TWO_PINS = [('P', 5.4, 3.1, 3.0)]
#: E5: a small F.CrtYd clear of pin 1, a B.CrtYd over it.
E5_PART = ('  (footprint "t:P2" (layer "F.Cu") (at 8 6)\n'
           '    (property "Reference" "P" (at 0 0) (layer "F.SilkS"))\n'
           '    (fp_rect (start -0.5 -0.5) (end 0.5 0.5) (stroke (width'
           ' 0.05) (type default)) (layer "F.CrtYd"))\n'
           '    (fp_rect (start -6 -5.5) (end -4 -3.5) (stroke (width'
           ' 0.05) (type default)) (layer "B.CrtYd"))\n'
           '    (pad "1" thru_hole circle (at 0 0) (size 0.8 0.8)'
           ' (drill 0.4) (layers "*.Cu" "*.Mask") (net 2 "N2")))\n')


class TheSecondPhase3VerifiersCases(unittest.TestCase):
    """Each finding of the second verifier on the phase-3 fixes, alone."""

    def _grade(self, path, **kw):
        return legality.CourtyardCensus(parse_kicad_pcb(path), path,
                                        **kw).grade()

    def test_each_hole_follows_its_own_rule(self):
        """An unnumbered NPTH pin and a PTH pin under one courtyard are two
        pairs -- one per KiCad rule -- and each rule's ignore waives only
        its own."""
        want = {('error', 'error'): ['npth', 'pth'],
                ('error', 'ignore'): ['pth'],
                ('ignore', 'error'): ['npth']}
        for (pth, npth), gating in sorted(want.items()):
            with tempfile.TemporaryDirectory() as td:
                path = _frame_board_npth(td, TWO_PINS, {'1': ''})
                _pro(path, pth_inside_courtyard=pth,
                     npth_inside_courtyard=npth)
                g = self._grade(path)
            self.assertEqual(sorted((q.hole, q.pins) for q in g.pin_pairs),
                             [('npth', ('',)), ('pth', ('3',))], (pth, npth))
            self.assertEqual(sorted(q.hole for q in g.pin_blocking), gating,
                             (pth, npth))

    def test_a_warning_gates(self):
        """StickHub warns on pth_inside_courtyard: KiCad reports the pin, so
        it gates -- only `ignore` waives."""
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, PIN_ONLY)
            _pro(path, pth_inside_courtyard='warning')
            rc, doc, _out = _assembly(path)
        self.assertEqual((rc, doc['pin_in_courtyard']), (4, 1), doc)

    def test_the_authors_saved_value_is_read_for_the_pin_rule(self):
        """A tool changed pth_inside_courtyard to error and kept the
        author's `ignore` in saved_severities: the author's value decides."""
        import json
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, PIN_ONLY)
            with open(os.path.splitext(path)[0] + '.kicad_pro', 'w',
                      encoding='utf-8') as fh:
                json.dump({'board': {'design_settings': {'rule_severities': {
                    'pth_inside_courtyard': 'error'}}},
                    'kicad_routing_tools': {'saved_severities': {
                        'pth_inside_courtyard': 'ignore'}}}, fh)
            rc, doc, _out = _assembly(path)
        self.assertEqual((rc, doc['pin_in_courtyard']), (0, 0), doc)

    def test_ignoring_the_project_grades_a_pin_at_error(self):
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, PIN_ONLY)
            _pro(path, pth_inside_courtyard='ignore')
            rc0, doc0, _o = _assembly(path)
            rc, doc, _o = _assembly(path, '--ignore-project-severity')
        self.assertEqual((rc0, doc0['pin_in_courtyard']), (0, 0), doc0)
        self.assertEqual((rc, doc['pin_in_courtyard']), (4, 1), doc)

    def test_the_census_reads_the_pin_rule_off_its_source(self):
        """No `pcb_file`: the pin rules come from the parsed board's own
        source, as the courtyard rule does."""
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, PIN_ONLY)
            _pro(path, pth_inside_courtyard='ignore')
            g = legality.CourtyardCensus(parse_kicad_pcb(path)).grade()
        self.assertEqual(len(g.pin_pairs), 1)
        self.assertEqual(g.pin_blocking, [])

    def test_the_own_courtyard_is_graded_as_drawn(self):
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [])
            _add(path, _crtyd_part('P', (6, 4), [('F.CrtYd', L_SHAPE)]))
            g = self._grade(path)
        self.assertEqual(g.pin_pairs, [])

    def test_a_far_courtyard_is_graded_as_drawn(self):
        """The L on B.CrtYd behind a small F.CrtYd: its bbox covers pin 1
        and KiCad reports nothing."""
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [])
            _add(path, _crtyd_part('P', (6, 4), [
                ('F.CrtYd', 'fp_rect (start -0.5 -0.5) (end 0.5 0.5)'),
                ('B.CrtYd', L_SHAPE)]))
            g = self._grade(path)
        self.assertEqual(g.pin_pairs, [])

    def test_an_open_courtyard_is_listed_not_gated(self):
        """Three sides of a square over pin 1: KiCad's malformed_courtyard,
        no courtyard to test a pin against."""
        side = 'fp_line (start {} {}) (end {} {})'
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [])
            _add(path, _crtyd_part('P', (3, 3.1), [
                ('F.CrtYd', side.format(-2, -2, 2, -2)),
                ('F.CrtYd', side.format(2, -2, 2, 2)),
                ('F.CrtYd', side.format(2, 2, -2, 2))]))
            rc, doc, _out = _assembly(path)
            g = self._grade(path)
        self.assertEqual([(q.b, q.basis) for q in g.pin_pairs],
                         [('P', legality.CourtyardCensus
                           .MALFORMED_COURTYARD_BASIS)])
        self.assertEqual((rc, doc['pin_in_courtyard']), (0, 0), doc)

    def test_a_hole_must_reach_in_before_it_counts(self):
        """KiCad reports a hole 6 um into a courtyard and not 5 um
        (`PIN_HOLE_TOLERANCE_MM`, measured by measure_1212 --onset). Pin 1's
        hole ends at y = 2.0; P's courtyard starts at y - 2."""
        got = {}
        for y in (3.995, 3.990):
            with tempfile.TemporaryDirectory() as td:
                path = _frame_board(td, [('P', 3.0, y, 2.0)])
                got[y] = len(self._grade(path).pin_blocking)
        self.assertEqual(got, {3.995: 0, 3.990: 1})

    def test_an_unnumbered_npth_is_labelled(self):
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board_npth(td, PIN_ONLY, {'1': ''})
            rc, _doc, out = _assembly(path)
        self.assertEqual(rc, 4)
        self.assertIn('P over FR NPTH pin(s) (unnumbered NPTH)', out)

    def test_the_search_sees_a_far_courtyard_over_a_pin(self):
        """E5 through the search: `pin_hits`' broad phase covers the drawn
        far courtyard, so the search refuses the pose the grader gates."""
        import pose_score
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [])
            _add(path, E5_PART)
            pcb = parse_kicad_pcb(path)
            hits = legality.CourtyardCensus(pcb, path).pin_hits(
                'P', (8.0, 6.0, 0.0))
            st = pose_score.make_state(pcb, path, clearance=0.2)
            veto = st.candidate_veto('P', 8.0, 6.0, 0.0, exclude=set())
        self.assertEqual([(q.a, q.b) for q in hits], [('FR', 'P')])
        self.assertEqual(veto, ('container_pin', 'FR'))

    def test_a_waived_courtyard_does_not_waive_the_escape(self):
        """courtyards_overlap ignored, pth_inside_courtyard at error, P
        coming home from off the board: the escape rule asks
        `violation_parts`, which counts the pin on the waived path too."""
        import pose_score
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [('P', 45.0, 3.1, 2.0)])
            _pro(path, courtyards_overlap='ignore',
                 pth_inside_courtyard='error')
            st = pose_score.make_state(parse_kicad_pcb(path), path,
                                       clearance=0.2)
            on_pin = st.candidate_valid('P', 3.0, 3.1, 0.0)
            clear = st.candidate_valid('P', 20.0, 15.0, 0.0)
            vio = st.violation_parts('P', 3.0, 3.1, 0.0)[1]
        self.assertFalse(on_pin)
        self.assertTrue(clear)
        self.assertGreater(vio, 0.0)


def _square(gap):
    """Four F.CrtYd lines of a 4 x 4 square, the last stopping `gap` short
    of the first's start (a corner that does not quite close)."""
    return [('F.CrtYd', f'fp_line (start {a} {b}) (end {c} {d})')
            for a, b, c, d in ((-2, -2, 2, -2), (2, -2, 2, 2), (2, 2, -2, 2),
                               (-2, 2, -2, round(-2 + gap, 6)))]


#: kicad-cli's verdict on 214 synthetic courtyard drawings (the verifiers'
#: shapes: gaps at every chain position and across the 20 um boundary, on
#: and off the 0.1 um grid and on rotated footprints; crossings, stubs, T's,
#: nested and corner-touching squares, fillets, arcs, subdivided sides, B
#: side, rotations; zero-length and near-zero elements; a piece shorter
#: than the gaps around it; a three-way corner beside a loose end; an arc
#: whose ends meet, which KiCad draws as a full circle, alone, beside, inside
#: and touching a square). Re-measure with
#: `tests/measure_1212_kicad_pins.py --chaining`.
CHAINING = os.path.join(ROOT, 'tests', 'fixtures',
                        '1212_courtyard_chaining.json')
#: Drawings KiCad flags malformed_courtyard that the parser closes. KiCad
#: still tests a pin against the contours of such a drawing that DO close
#: (final P1 verifier: stub_tiny_0.015 reports its pin), so where those
#: contours are the drawing's own outline both grade the pin, and the two
#: differ only in the flag; where they are not, the polygon here is the
#: larger, conservative reading. A T or stub, a crossing, a duplicate or
#: overlapping segment, debris the join absorbs (near-zero lines, a flat
#: rect inside), a gap KiCad rounds past 20 um only after rotation, gaps of
#: 20 to ~20.14 um (the snap's reach, `parser._OUTLINE_JOIN_REACH_MM`), an
#: arc whose ends miss by a few nm (KiCad flags it; here it is the near-full
#: arc it draws), and a full circle started on a square's edge (flagged
#: malformed by KiCad, its pin still reported -- as here). Pinned, so a
#: change of the model in EITHER direction shows here.
KNOWN_CONSERVATIVE = {
    'cross_0.015', 'cross_0.019', 'divider_exact', 'divider_short',
    'dup_line', 'gap_0.0201', 'gapfirst_0.0201', 'gapmid_0.0201',
    'overlap_collinear', 'rot30_gap0.02', 'stub_in', 'stub_out_0.015',
    'stub_tiny_0.015', 'tee_exact', 'tee_short', 'two_squares_gap',
    'v9_flat_rect_in', 'v9_flat_rect_on_edge', 'v9_iso_piece_in_0.012',
    'v9_iso_piece_near_corner_in', 'v9_nz50nm_snapsame',
    'v9_nz_line_cornerout_0.0001', 'v9_nz_line_cornerout_0.0005',
    'v9_nz_line_cornerout_0.001', 'v9_nz_line_cornerout_0.005',
    'v9_nz_line_cornerout_1e-05', 'v9_nz_line_cornerout_5e-05',
    'v9_nz_line_far_1e-05', 'v9_nz_line_far_5e-05',
    'v9_nz_line_in_0.0001', 'v9_nz_line_in_0.0005', 'v9_nz_line_in_0.001',
    'v9_nz_line_in_0.005', 'v9_nz_line_in_1e-05', 'v9_nz_line_in_5e-05',
    'v9_nz_line_out_1e-05', 'v9_nz_line_out_5e-05', 'v9_threepieces_10_12',
    'v9_twopieces_5_8', 'v10_c1_ctrl_r0_gap20001', 'v10_c1_r0_diag14143_half',
    'v10_c2_near10_skew', 'v10_c2_near198_skew', 'v10_c2_near49_skew',
    'v10_c2t_arc_start_on_right_edge', 'v10_c2t_circle_first_on_edge'}
#: Drawings with NO outline here (nothing of area to read), with KiCad's
#: flag. A zero-radius circle: no courtyard in either. A lone 12 um piece or
#: a lone flat fp_rect over a hole: KiCad flags malformed and STILL reports
#: the pin; here no outline, so no pin is listed -- the disclosed class.
KNOWN_NO_OUTLINE = {
    'v9_z_circle_alone': True, 'v9_flat_rect_alone_pin1': False,
    'v9_lone_piece_0.012_pin1': False,
    'v9_lone_piece_0.012_pin1_holeedge': False}


class KiCadsChaining(unittest.TestCase):
    """The parser's OUTLINE_HULL is how the pin channel knows a courtyard
    is malformed (`courtyard_malformed`: listed, never gating), so it must
    never read OPEN a drawing KiCad closes -- that would pass a real
    pth_inside_courtyard. It closes generously instead (`parser.
    _outline_shapes_by_side`: snap, then a join of loose ends, within
    20 um), and the drawings it closes that KiCad does not are pinned.
    Not covered by this flag-level comparison: a drawing KiCad flags
    malformed while still testing pins against the part that closes (a
    square plus debris no join absorbs) reads as the hull here, its pins
    listed and not gating -- see the parser's comment."""

    def test_no_drawing_kicad_closes_reads_open(self):
        import json
        from placement import parser
        with open(CHAINING, encoding='utf-8') as fh:
            cases = json.load(fh)['cases']
        self.assertGreater(len(cases), 210)
        fn, cons, none = [], set(), {}
        for name, c in sorted(cases.items()):
            r = parser._outline_shapes_by_side(
                '\n'.join(c['courtyard']), parser._CRTYD_LAYER,
                even_odd=True)
            if not r:
                none[name] = c['kicad_closed']
                continue
            closed = all(v[1] == parser.OUTLINE_POLYGON for v in r.values())
            if c['kicad_closed'] and not closed:
                fn.append(name)
            if closed and not c['kicad_closed']:
                cons.add(name)
        self.assertEqual(fn, [], 'drawings KiCad closes read OPEN here')
        self.assertEqual(cons, KNOWN_CONSERVATIVE)
        self.assertEqual(none, KNOWN_NO_OUTLINE)

    def test_a_full_circle_from_an_edge_is_outline(self):
        """An fp_arc whose ends meet is a full circle in KiCad; inside a
        square it is a hole -- unless its START lies on the square's edge,
        because KiCad decides hole or outline from a contour's first point
        (third fix verifier, kicad-cli 10: started on the edge, the pin
        under it is reported; started inside with its mid on the edge,
        none). The flag-level fixture cannot see this: both read closed."""
        from placement import parser
        sq = [f'(fp_line (start {a} {b}) (end {c} {d}) (layer "F.CrtYd"))'
              for a, b, c, d in ((-2.4, -2.4, 2.4, -2.4), (2.4, -2.4, 2.4, 2.4),
                                 (2.4, 2.4, -2.4, 2.4), (-2.4, 2.4, -2.4, -2.4))]

        def area(start, mid):
            arc = (f'(fp_arc (start 0 {start}) (mid 0 {mid}) (end 0 {start})'
                   ' (layer "F.CrtYd"))')
            g, how = parser._outline_shapes_by_side(
                '\n'.join(sq + [arc]), parser._CRTYD_LAYER,
                even_odd=True)['F']
            self.assertEqual(how, parser.OUTLINE_POLYGON)
            return g.area
        self.assertAlmostEqual(area(-2.4, -0.8), 4.8 * 4.8, 3)    # outline
        self.assertAlmostEqual(area(-0.8, -2.4), 4.8 * 4.8 - 2.0096, 2)  # hole

    def test_the_bbox_bounds_the_outline(self):
        """`extract_courtyard_sides`' bbox is the broad phase in front of the
        outline `extract_courtyard_shapes` reads, so on every drawing it
        must contain it -- an arc whose ends meet, read as its zero-length
        chord, gave a one-point bbox around a full circle (fix verifier)."""
        import json
        from placement import parser
        with open(CHAINING, encoding='utf-8') as fh:
            cases = json.load(fh)['cases']
        bad = []
        for name, c in sorted(cases.items()):
            text = '\n'.join(c['courtyard'])
            pts = parser._courtyard_points_by_side(text)
            for side, (g, _how) in parser._outline_shapes_by_side(
                    text, parser._CRTYD_LAYER, even_odd=True).items():
                bb, gb = parser._bbox(pts[side]), g.bounds
                if not (gb[0] >= bb[0] - 1e-3 and gb[1] >= bb[1] - 1e-3
                        and gb[2] <= bb[2] + 1e-3 and gb[3] <= bb[3] + 1e-3):
                    bad.append((name, side, bb, gb))
        self.assertEqual(bad, [])


class TheThirdVerifiersCases(unittest.TestCase):
    """The verifier on 79cfc025: KiCad's chaining, the broad phase at every
    rotation, the gating shape first, the counts."""

    def _grade(self, path):
        return legality.CourtyardCensus(parse_kicad_pcb(path), path).grade()

    def test_a_corner_kicad_closes_is_a_courtyard(self):
        """KiCad chains courtyard ends within 0.02 mm: 8 and 19 um gaps
        close (a pin under them is reported), 30 um does not (malformed)."""
        got = {}
        for gap in (0.008, 0.019, 0.03):
            with tempfile.TemporaryDirectory() as td:
                path = _frame_board(td, [])
                _add(path, _crtyd_part('P', (3, 3.1), _square(gap)))
                g = self._grade(path)
            got[gap] = [(q.b, q.basis, q.waived) for q in g.pin_pairs]
        self.assertEqual(got[0.008], [('P', 'courtyard', False)])
        self.assertEqual(got[0.019], [('P', 'courtyard', False)])
        self.assertEqual(got[0.03], [(
            'P', legality.CourtyardCensus.MALFORMED_COURTYARD_BASIS, False)])

    def test_glasgow_j4_closes(self):
        """The real case: J4's F.CrtYd ends 8 um apart at one corner."""
        from placement import parser
        shapes = parser.extract_courtyard_shapes(os.path.join(
            ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb'))
        self.assertEqual(shapes['J4']['F'][1], parser.OUTLINE_POLYGON)

    def test_the_onset_is_six_microns(self):
        """5 um in: silent; 6 um in: reported -- the step KiCad reports
        from (measure_1212 --onset). Pin 1's hole ends at y = 2.0."""
        got = {}
        for y in (3.995, 3.994):
            with tempfile.TemporaryDirectory() as td:
                path = _frame_board(td, [('P', 3.0, y, 2.0)])
                got[y] = len(self._grade(path).pin_blocking)
        self.assertEqual(got, {3.995: 0, 3.994: 1})

    def test_the_search_and_the_grade_agree_at_every_rotation(self):
        """E5's far courtyard swept over pin 1 at four rotations: the
        search's `pin_hits` finds exactly the pins the grade gates (its
        broad phase turns the far courtyard and covers both of its sides)."""
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [])
            _add(path, E5_PART)
            c = legality.CourtyardCensus(parse_kicad_pcb(path), path)
            bad, seen = [], 0
            for rot in (0.0, 90.0, 180.0, 270.0):
                for x in range(1, 16):
                    for y in range(1, 14):
                        pose = (float(x), float(y), rot)
                        hits = bool(c.pin_hits('P', pose))
                        graded = any(q.b == 'P' for q in
                                     c.grade({'P': pose}).pin_blocking)
                        seen += graded
                        if hits != graded:
                            bad.append((pose, hits, graded))
        self.assertGreater(seen, 8)
        self.assertEqual(bad, [])

    def test_a_closed_courtyard_gates_before_an_open_one(self):
        """A closed F.CrtYd and an open B.CrtYd both over pin 1: the pin
        is graded on the closed one, and gates."""
        shapes = ([('F.CrtYd', 'fp_rect (start -2 -2) (end 2 2)')]
                  + [('B.CrtYd', b) for _l, b in _square(0.5)])
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [])
            _add(path, _crtyd_part('P', (3, 3.1), shapes))
            g = self._grade(path)
        self.assertEqual([(q.b, q.basis) for q in g.pin_blocking],
                         [('P', 'courtyard')])

    def test_a_part_over_two_hole_kinds_is_one_part(self):
        """P over an NPTH pin and a PTH pin: two KiCad findings, one part
        pair -- check_assembly counts both, the repair charges one part."""
        import subprocess
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board_npth(td, TWO_PINS, {'1': ''})
            rc, doc, out = _assembly(path)
            self.assertEqual(rc, 4)
            self.assertIn('PIN IN COURTYARD (1 part pair(s), 2 by hole rule)',
                          out)
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_placer', 'place_reconstruct.py'),
                 path, os.path.join(td, 'o.kicad_pcb'), '--stages',
                 'legalize', '--clearance', '0.2'],
                capture_output=True, text=True, encoding='utf-8',
                errors='replace', cwd=ROOT, timeout=600)
        self.assertIn('Pin census: 1 part(s)', r.stdout, r.stdout[-800:])


if __name__ == '__main__':
    unittest.main()
