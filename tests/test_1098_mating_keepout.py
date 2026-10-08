"""#1098: nothing may sit on a PCB-edge plug's mating region, either face.

Run 36 placed 8 back-side parts (C2 C21 C24 C28 D23 D24 JP1 R1) on the
tongue of KiCad StickHub's USB-A plug J1 -- the part of the board that slides
into a socket -- and check_assembly, check_floorplan and place_pose all
passed it. J1 (`USB_A_PCB_traces_small`) is board copper: SMD fingers,
`exclude_from_pos_files`, no 3D model, a courtyard on F.CrtYd only. Since
courtyards are per side, a B-side part never paired with it.

The fix derives the region from the footprint (`floorplan.
derived_mating_keepouts`: courtyard inset 0.25 mm, both faces, the plug and
copper-less parts such as a slot allowed) and hands it to the #701 keep-out
channel the seeder and quench enforce, to `grade_pad_legality` (place_pose
gates on it) and to check_assembly (a NOT BUILDABLE conjunct). A declared
keep-out named `mating:<ref>` replaces the derived one.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from test_1094_rotated_courtyards import stickhub  # noqa: E402

#: Run 36's final board is a local run artifact (and CC BY-NC-SA); point
#: KICAD_RUN36_FINAL at it to run that arm.
RUN36 = [os.environ.get('KICAD_RUN36_FINAL', '')]

OUTLINE = [(0, 0), (30, 0), (30, 20), (20, 20), (20, 32), (10, 32), (10, 20),
           (0, 20)]


def board(td, *, r1=(25, 10, 'B.Cu'), attr='exclude_from_pos_files',
          model=False, drilled=False, hole=True, keepouts_note='',
          xs=(-3.8, -1.3, 1.3, 3.8)):
    """A 30x20 board with a 10x12 tongue at x 10-20, y 20-32. J1 is the
    plug: its F courtyard IS the tongue, its four SMD fingers (at local
    `xs`) carry nets."""
    segs = ''.join(
        f'  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (stroke '
        f'(width 0.1) (type default)) (layer "Edge.Cuts"))\n'
        for a, b in zip(OUTLINE, OUTLINE[1:] + OUTLINE[:1]))
    fingers = ''.join(
        f'    (pad "{i + 1}" {"thru_hole" if drilled and i == 0 else "smd"} '
        f'rect (at {x} -6) (size 1.5 8)'
        + (' (drill 0.8)' if drilled and i == 0 else '')
        + f' (layers "F.Cu") (net {i + 1} "N{i + 1}"))\n'
        for i, x in enumerate(xs))
    x, y, side = r1
    s = side[0]
    text = (
        '(kicad_pcb (version 20240108) (generator pcbnew)\n'
        '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
        ' (44 "Edge.Cuts" user))\n'
        '  (net 0 "") (net 1 "N1") (net 2 "N2") (net 3 "N3") (net 4 "N4")\n'
        + segs +
        '  (footprint "t:USB_A_PCB_traces" (layer "F.Cu") (at 15 32)\n'
        '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))\n'
        + (f'    (attr smd {attr})\n' if attr else '    (attr smd)\n') +
        '    (fp_rect (start -5 -12) (end 5 0) (stroke (width 0.05)'
        ' (type default)) (layer "F.CrtYd"))\n'
        + fingers
        + ('    (model "x.wrl")\n' if model else '') +
        '  )\n'
        + ('  (footprint "t:Slot" (layer "F.Cu") (at 15 21)\n'
           '    (property "Reference" "H1" (at 0 0) (layer "F.SilkS"))\n'
           '    (attr exclude_from_pos_files)\n'
           '    (pad "" np_thru_hole circle (at 0 0) (size 1.5 1.5)'
           ' (drill 1.5) (layers "*.Cu" "*.Mask")))\n' if hole else '') +
        f'  (footprint "t:R" (layer "{side}") (at {x} {y})\n'
        '    (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))\n'
        '    (attr smd)\n'
        f'    (fp_rect (start -1 -0.5) (end 1 0.5) (stroke (width 0.05)'
        f' (type default)) (layer "{s}.CrtYd"))\n'
        f'    (pad "1" smd rect (at -0.5 0) (size 0.6 0.6) (layers "{side}")'
        ' (net 1 "N1"))\n'
        f'    (pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "{side}")'
        ' (net 2 "N2")))\n'
        ')\n')
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def keepouts(path):
    from kicad_parser import parse_kicad_pcb
    from placement.floorplan import derived_mating_keepouts
    return derived_mating_keepouts(parse_kicad_pcb(path), path)


def findings(path):
    from kicad_parser import parse_kicad_pcb
    from placement.floorplan import mating_keepout_findings
    return mating_keepout_findings(parse_kicad_pcb(path), path)


def check_assembly(path):
    js = path + '.json'
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_tools', 'check_assembly.py'),
                        path, '--json', js],
                       capture_output=True, text=True, cwd=ROOT)
    assert os.path.isfile(js), r.stderr[-2000:]
    with open(js, encoding='utf-8') as fh:
        return r, json.load(fh)


class TestDerivation(unittest.TestCase):
    def test_the_plug_derives_its_tongue_inset(self):
        with tempfile.TemporaryDirectory() as td:
            ks = keepouts(board(td))
        self.assertEqual([k['name'] for k in ks], ['mating:J1'])
        self.assertEqual(ks[0]['rect'], (10.25, 20.25, 19.75, 31.75))
        self.assertEqual(ks[0]['sides'], ('F', 'B'))
        # The plug itself, and the copper-less slot, are allowed.
        self.assertEqual(ks[0]['allow'], ('J1', 'H1'))

    def test_what_is_not_a_plug(self):
        """An assembled part (no such attr), a part with a model, and one
        with a drilled pad derive nothing."""
        for kw in ({'attr': ''}, {'model': True}, {'drilled': True}):
            with tempfile.TemporaryDirectory() as td:
                self.assertEqual(keepouts(board(td, **kw)), (), kw)

    def test_a_net_tie_or_a_part_off_the_board_is_not_a_plug(self):
        """#1098 verifier: SolderJumper-3 parts (net-ties, no model,
        exclude_from_pos_files) parked wholly OFF a board read as plugs."""
        with tempfile.TemporaryDirectory() as td:
            p = board(td)
            text = open(p, encoding='utf-8').read()
            tie = text.replace('(attr smd exclude_from_pos_files)',
                               '(attr smd exclude_from_pos_files)\n'
                               '    (net_tie_pad_groups "1,2")')
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write(tie)
            self.assertEqual(keepouts(p), ())
            off = text.replace('(at 15 32)\n', '(at 60 60)\n', 1)
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write(off)
            self.assertEqual(keepouts(p), ())

    def test_fingers_that_do_not_reach_the_edge_are_not_a_plug(self):
        """The courtyard reaches the tip, but the netted pads are 0.5 x 1 mm
        islands more than 1 mm from every edge of the tongue: a jumper or a
        logo with a courtyard, not a plug."""
        with tempfile.TemporaryDirectory() as td:
            p = board(td)
            text = open(p, encoding='utf-8').read()
            text = text.replace('(size 1.5 8)', '(size 0.5 1)').replace(
                '(at -3.8 -6)', '(at -2.5 -6)').replace('(at 3.8 -6)',
                                                        '(at 2.5 -6)')
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write(text)
            self.assertEqual(keepouts(p), ())

    def test_board_only_counts_too(self):
        with tempfile.TemporaryDirectory() as td:
            self.assertEqual(len(keepouts(board(td, attr='board_only'))), 1)


class TestGraders(unittest.TestCase):
    def test_a_back_side_part_on_the_tongue_is_not_buildable(self):
        with tempfile.TemporaryDirectory() as td:
            p = board(td, r1=(15, 26, 'B.Cu'))
            self.assertEqual([f['ref'] for f in findings(p)], ['R1'])
            r, d = check_assembly(p)
        self.assertEqual(r.returncode, 4, r.stdout[-1500:])
        self.assertFalse(d['buildable'])
        self.assertEqual([m['ref'] for m in d['mating_keepout_refs']],
                         ['R1'])
        self.assertIn("ON A PLUG'S MATING REGION", r.stdout)

    def test_the_control_off_the_tongue_is_buildable(self):
        with tempfile.TemporaryDirectory() as td:
            p = board(td)
            self.assertEqual(findings(p), [])
            r, d = check_assembly(p)
        self.assertTrue(d['buildable'], r.stdout[-1500:])
        self.assertEqual(d['mating_keepout_refs'], [])

    def test_the_floorplan_grade_charges_it(self):
        """check_floorplan -- and place_seed --repair, which charges grade
        errors -- see the derived keep-out with an intent that declares
        none (#1098 verifier D4)."""
        from kicad_parser import parse_kicad_pcb
        from placement.floorplan import empty_intent, grade
        with tempfile.TemporaryDirectory() as td:
            p = board(td, r1=(15, 26, 'B.Cu'))
            g = grade(empty_intent(p), parse_kicad_pcb(p), p)
            p2 = board(td, r1=(25, 10, 'B.Cu'))
            g2 = grade(empty_intent(p2), parse_kicad_pcb(p2), p2)
        self.assertEqual([(v.rule, v.ref) for v in g.errors
                          if v.rule == 'keepout'], [('keepout', 'R1')])
        self.assertEqual([v for v in g2.errors if v.rule == 'keepout'], [])

    def test_the_slot_in_the_tongue_is_allowed(self):
        """H1, a copper-less slot at the tongue's root, is not a finding
        (StickHub's H1 sits exactly there)."""
        with tempfile.TemporaryDirectory() as td:
            self.assertEqual(findings(board(td, r1=(25, 10, 'B.Cu'))), [])

    def test_place_pose_refuses_a_move_onto_the_tongue(self):
        with tempfile.TemporaryDirectory() as td:
            p = board(td)
            out = os.path.join(td, 'o.kicad_pcb')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_placer', 'place_pose.py'), p, out,
                 'set', 'R1', '15', '26', '--rot', '0'],
                capture_output=True, text=True, cwd=ROOT)
            wrote = os.path.exists(out)
        self.assertEqual(r.returncode, 4, (r.stdout + r.stderr)[-2000:])
        self.assertFalse(wrote)
        self.assertIn('mating_keepout', r.stdout)


class TestGenerator(unittest.TestCase):
    def test_the_seat_predicate_refuses_the_tongue(self):
        """The seeder's `pose_ok` refuses R1 on the tongue and accepts it on
        the board body: the derived keep-out is in the quench state even
        though no intent declared it."""
        import pose_score
        from kicad_parser import parse_kicad_pcb
        from placement import seeder
        with tempfile.TemporaryDirectory() as td:
            p = board(td)
            st = pose_score.make_state(parse_kicad_pcb(p), p, clearance=0.1,
                                       board_edge_clearance=0.1)
            self.assertIn('mating:J1', [k['name'] for k in st.keepouts])
            self.assertIn('mating:J1', [k['name'] for k in
                                        st.keepouts_for.get('R1', ())])
            on = seeder.pose_ok(st, 'R1', 15.0, 26.0, 0.0, set())
            off = seeder.pose_ok(st, 'R1', 5.0, 10.0, 0.0, set())
        self.assertFalse(on)
        self.assertTrue(off)

    def test_the_seat_and_the_checker_measure_one_rect(self):
        """#1098 verifier D3: the seat tested the courtyard, the checker the
        courtyard plus pads, so a part whose pad pokes past its courtyard
        into the tongue was seated and then graded NOT BUILDABLE. Both now
        read the quench's rect, and a pose they judge alike stays alike on
        each side of the region's edge."""
        import pose_score
        from kicad_parser import parse_kicad_pcb
        from placement import seeder
        for x, y in ((15, 26), (25, 10), (15, 19.9), (9.6, 26)):
            with tempfile.TemporaryDirectory() as td:
                p = board(td, r1=(x, y, 'B.Cu'))
                hit = bool(findings(p))
                st = pose_score.make_state(parse_kicad_pcb(p), p,
                                           clearance=0.1,
                                           board_edge_clearance=0.0)
                seat = seeder.pose_ok(st, 'R1', float(x), float(y), 0.0,
                                      set())
                clear = st.keepout_clear('R1', st.parts['R1'].rects(
                    float(x), float(y), 0.0))
            self.assertEqual(hit, not clear, (x, y))
            if hit:
                self.assertFalse(seat, (x, y))

    def test_a_seated_plug_does_not_move(self):
        """Moving the plug inland took its region with it and read as an
        improvement (final review). The quench locks it and place_pose
        refuses to move it unless it is named in `unlock`."""
        import pose_score
        from kicad_parser import parse_kicad_pcb
        with tempfile.TemporaryDirectory() as td:
            p = board(td, r1=(15, 26, 'B.Cu'))
            st = pose_score.make_state(parse_kicad_pcb(p), p, clearance=0.1,
                                       board_edge_clearance=0.1)
            self.assertTrue(st.parts['J1'].locked)
            out = os.path.join(td, 'o.kicad_pcb')
            cmd = [sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_placer', 'place_pose.py'), p, out]
            r = subprocess.run(cmd + ['set', 'J1', '15', '10', '--rot', '0'],
                               capture_output=True, text=True, cwd=ROOT)
            refused = (r.returncode, os.path.exists(out))
            r2 = subprocess.run(cmd + ['unlock', 'J1', 'set', 'J1', '15',
                                       '10', '--rot', '0', '--force'],
                                capture_output=True, text=True, cwd=ROOT)
        self.assertEqual(refused, (4, False), r.stdout[-1500:])
        # The REASON, not just the refusal: without the plug guard the move
        # is still refused, by the ordinary legality gate, and every summary
        # carries "PCB-edge plug" in its legal_scope (mutate_1094's
        # `place-pose-moves-the-plug` survived that weaker assertion).
        self.assertIn('is a PCB-edge plug seated at its edge',
                      r.stdout + r.stderr)
        self.assertNotIn('is a PCB-edge plug seated at its edge',
                         r2.stdout + r2.stderr)

    def test_a_declared_keepout_of_that_name_wins(self):
        from kicad_parser import parse_kicad_pcb
        from placement.floorplan import with_derived_keepouts
        with tempfile.TemporaryDirectory() as td:
            p = board(td)
            mine = {'name': 'mating:J1', 'rect': (10, 30, 20, 32),
                    'sides': ('F', 'B'), 'allow': ('J1',)}
            got = with_derived_keepouts([mine], parse_kicad_pcb(p), p)
        self.assertEqual(got, (mine,))


class TestRealBoards(unittest.TestCase):
    def test_stickhub_human_board_is_clean(self):
        path = stickhub()
        if not path:
            self.skipTest('KiCad StickHub demo not installed')
        from kicad_parser import parse_kicad_pcb
        from placement.floorplan import (derived_mating_keepouts,
                                         mating_keepout_findings)
        pcb = parse_kicad_pcb(path)
        self.assertEqual([k['name'] for k in
                          derived_mating_keepouts(pcb, path)], ['mating:J1'])
        self.assertEqual(mating_keepout_findings(pcb, path), [])

    def test_run36_final_names_the_eight_parts(self):
        path = next((p for p in RUN36 if p and os.path.isfile(p)), None)
        if not path:
            self.skipTest('run 36 final board not present')
        got = sorted({f['ref'] for f in findings(path)})
        self.assertEqual(got, ['C2', 'C21', 'C24', 'C28', 'D23', 'D24',
                               'JP1', 'R1'])

    def test_no_tracked_board_has_a_plug(self):
        """The detector fires on nothing in the tracked corpus (glasgow's
        overhanging mounting holes and JP-style jumpers are the near misses
        it must not take)."""
        from kicad_parser import parse_kicad_pcb
        ls = subprocess.run(['git', 'ls-files', '*.kicad_pcb'], cwd=ROOT,
                            capture_output=True, text=True).stdout.split()
        self.assertGreaterEqual(len(ls), 20)
        from placement.floorplan import derived_mating_keepouts
        fired = {}
        for b in ls:
            path = os.path.join(ROOT, b)
            try:
                pcb = parse_kicad_pcb(path)
            except Exception:
                continue
            ks = derived_mating_keepouts(pcb, path)
            if ks:
                fired[b] = [k['name'] for k in ks]
        self.assertEqual(fired, {})



def write_intent(td, keepouts):
    """A minimal intent declaring `keepouts` (the #701 channel)."""
    path = os.path.join(td, 'intent.json')
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump({'schema': 1, 'kind': 'floorplan-intent',
                   'keepouts': keepouts}, fh)
    return path


def place_pose(board_path, out, *args):
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_placer', 'place_pose.py'), board_path, out,
         *args], capture_output=True, text=True, cwd=ROOT)


#: #1098 review: tigard's JP1 (`Jumper:SolderJumper-2_P1.3mm_Bridged_...`,
#: exclude_from_pos_files, no model, no net-tie group, two netted pads)
#: moved to the board's north edge, pads 0.55 mm from it, under J3.
TIGARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
TIGARD_JP1_AT = '(at 35.85 53.4 180)'
TIGARD_JP1_EDGE = '(at 45 31.3 180)'


class TestReviewFollowUps(unittest.TestCase):
    """The #1098 review's findings, each pinned where it failed."""

    def test_two_or_three_fingers_are_not_a_plug(self):
        """A 2- or 3-pad part at the edge (KiCad's solder jumpers, a DNP
        passive) is not a plug; four fingers still are."""
        for xs, want in (((-3.8, 3.8), 0), ((-3.8, 0.0, 3.8), 0),
                         ((-3.8, -1.3, 1.3, 3.8), 1)):
            with tempfile.TemporaryDirectory() as td:
                self.assertEqual(len(keepouts(board(td, xs=xs))), want, xs)

    def test_a_solder_jumper_at_tigards_edge_stays_buildable(self):
        with open(TIGARD, encoding='utf-8') as fh:
            text = fh.read()
        self.assertEqual(text.count(TIGARD_JP1_AT), 1)
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'tigard_jp1.kicad_pcb')
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(text.replace(TIGARD_JP1_AT, TIGARD_JP1_EDGE))
            self.assertEqual(keepouts(path), ())
            r, d = check_assembly(path)
        self.assertEqual(d['mating_keepout_refs'], [], r.stdout[-1500:])
        self.assertTrue(d['buildable'], r.stdout[-1500:])

    def test_a_plug_hanging_across_an_edge_is_not_seated(self):
        """J1 parked across the body's west edge, and J1 slid 3 mm out past
        the tongue's tip (its fingers 1 mm off the board, every other seat
        condition met -- the case only the copper-overrun test refuses): no
        region and no lock, so the search can still move it (and #1096
        reports its copper)."""
        import pose_score
        from kicad_parser import parse_kicad_pcb
        for at in ('(at 3 10 90)', '(at 15 35)'):
            with tempfile.TemporaryDirectory() as td:
                p = board(td)
                with open(p, encoding='utf-8') as fh:
                    text = fh.read()
                with open(p, 'w', encoding='utf-8') as fh:
                    fh.write(text.replace('(at 15 32)\n', at + '\n', 1))
                self.assertEqual(keepouts(p), (), at)
                st = pose_score.make_state(parse_kicad_pcb(p), p,
                                           clearance=0.1,
                                           board_edge_clearance=0.1)
                self.assertFalse(st.parts['J1'].locked, at)

    def test_check_assembly_grades_the_declared_region(self):
        """A declared `mating:J1` replaces the derived region in
        check_assembly as in the floorplan grade: R1 on the tongue's root is
        clean against a region declared at its tip, and R1 on the body is
        caught by a region declared there."""
        tip = {'name': 'mating:J1', 'rect': [10, 30, 20, 32],
               'sides': ['F', 'B'], 'allow': ['J1']}
        body = {'name': 'mating:J1', 'rect': [20.5, 5, 29.5, 15],
                'sides': ['F', 'B'], 'allow': ['J1']}
        for r1, decl, buildable in (((15, 26, 'B.Cu'), tip, True),
                                    ((25, 10, 'B.Cu'), body, False)):
            with tempfile.TemporaryDirectory() as td:
                p = board(td, r1=r1)
                intent = write_intent(td, [decl])
                js = p + '.json'
                r = subprocess.run(
                    [sys.executable, '-X', 'utf8',
                     os.path.join(ROOT, 'py_tools', 'check_assembly.py'), p,
                     '--intent', intent, '--json', js],
                    capture_output=True, text=True, cwd=ROOT)
                with open(js, encoding='utf-8') as fh:
                    d = json.load(fh)
            self.assertEqual(d['buildable'], buildable,
                             (r1, r.stdout[-1500:]))

    def test_place_pose_grades_the_declared_region(self):
        tip = {'name': 'mating:J1', 'rect': [10, 30, 20, 32],
               'sides': ['F', 'B'], 'allow': ['J1']}
        with tempfile.TemporaryDirectory() as td:
            p = board(td)
            intent = write_intent(td, [tip])
            out = os.path.join(td, 'o.kicad_pcb')
            r = place_pose(p, out, 'set', 'R1', '15', '26', '--rot', '0',
                           '--intent', intent)
            wrote = os.path.exists(out)
        self.assertEqual(r.returncode, 0, (r.stdout + r.stderr)[-2000:])
        self.assertTrue(wrote)

    def test_place_pose_locks_the_plug_a_declaration_names(self):
        """place_pose refuses to move a SEATED plug a declared `mating:J1`
        names (here J1 carries a model, so nothing is derived), like the
        quench does -- and lets the same plug out of the pile."""
        tip = {'name': 'mating:J1', 'rect': [10, 20, 20, 32],
               'sides': ['F', 'B'], 'allow': ['J1']}
        said = 'is a PCB-edge plug seated at its edge'
        with tempfile.TemporaryDirectory() as td:
            p = board(td, model=True)
            intent = write_intent(td, [tip])
            out = os.path.join(td, 'o.kicad_pcb')
            seated = place_pose(p, out, 'set', 'J1', '15', '10', '--rot',
                                '0', '--intent', intent)
            with open(p, encoding='utf-8') as fh:
                text = fh.read()
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write(text.replace('(at 15 32)\n', '(at 60 60)\n', 1))
            pile = place_pose(p, out, 'set', 'J1', '15', '32', '--rot', '0',
                              '--intent', intent)
        self.assertIn(said, seated.stdout + seated.stderr)
        self.assertNotIn(said, pile.stdout + pile.stderr)

    def test_a_declared_plug_in_the_pile_is_not_locked(self):
        """A plug in the staging pile that a declared `mating:J1` names is
        free: the seeder seats it (or names it unseated) instead of writing
        it back where it was."""
        import dataclasses
        import random
        import pose_score
        from kicad_parser import parse_kicad_pcb
        from placement import seeder
        from placement.floorplan import empty_intent
        decl = {'name': 'mating:J1', 'rect': (10.0, 20.0, 20.0, 32.0),
                'sides': ('F', 'B'), 'allow': ('J1',)}
        with tempfile.TemporaryDirectory() as td:
            p = board(td, r1=(60, 70, 'B.Cu'))
            with open(p, encoding='utf-8') as fh:
                text = fh.read()
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write(text.replace('(at 15 32)\n', '(at 60 60)\n', 1))
            it = dataclasses.replace(empty_intent(p), keepouts=(decl,))
            st = pose_score.make_state(parse_kicad_pcb(p), p, clearance=0.1,
                                       board_edge_clearance=0.1,
                                       keepouts=(decl,))
            self.assertFalse(st.parts['J1'].locked)
            res = seeder.seed_from_intent(
                parse_kicad_pcb(p), p, it, random.Random('1'), clearance=0.1,
                board_edge_clearance=0.1, grid_step=0.25)
        at = {q['reference']: (q['new_x'], q['new_y'])
              for q in res['placements']}
        self.assertTrue('J1' in res['unseated'] or at.get('J1') != (60, 60),
                        (at.get('J1'), res['unseated']))

    def test_place_pose_fails_closed_on_an_unmeasured_region(self):
        """check_assembly reads an unmeasured tongue as NOT BUILDABLE; the
        pose verb refuses on it too instead of reading a count of 0."""
        from placement import floorplan, pose_ops
        with tempfile.TemporaryDirectory() as td:
            p = board(td)
            real = floorplan.mating_keepout_findings

            def boom(*_a, **_k):
                raise RuntimeError('no tongue today')
            floorplan.mating_keepout_findings = boom
            try:
                with self.assertRaises(pose_ops.PoseRefusal) as cm:
                    pose_ops.apply_poses(
                        p, None, [{'kind': 'set', 'ref': 'R1', 'x': 25,
                                   'y': 12, 'rot': 0}], dry_run=True)
            finally:
                floorplan.mating_keepout_findings = real
        self.assertIn('mating_keepout_error', cm.exception.reason)


if __name__ == '__main__':
    unittest.main()
