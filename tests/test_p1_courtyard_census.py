"""fa10 P1 phase 0: check_assembly's courtyard channel, callable at any pose.

`legality.CourtyardCensus` is the courtyard channel lifted out of
`grade_body_overlap`'s closures (the waiver ladder, the run-23 floors, the
moved-vs-baseline gate), so the seeder, `--repair`, `--reseat` and the
floorplan budget can ask the GRADER'S question about a pose nobody has
written instead of rebuilding it on their own rects (#1182, #1162). What has
to hold for that to be worth anything:

  * at the file's poses it IS the channel (check_assembly reads it);
  * at a moved pose it answers exactly what `grade_body_overlap` answers on a
    board that has the part written there -- the pose path is not a second
    implementation that agrees by luck;
  * `grade_ref` (one part's pairs, O(n)) is `grade` restricted to that part;
  * the moved-vs-baseline currency is the one check_assembly gates on;
  * `measure_gate_vs_grader` -- the instrument every later phase is judged
    by -- can actually report a disagreement.
"""

import os
import random
import shutil
import sys
import tempfile
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

#: No subprocess.
RUN_ALL_FAST_OK = True

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import legality                               # noqa: E402

BOARDS = ('glasgow_revC', 'esp_prog', 'watchy', 'rp2350_fpga_eensy_prePlane')


def _board(name):
    return os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')


def _key(p):
    return (p.a, p.b, p.kind, p.area_mm2, p.depth_mm, p.side, p.waiver,
            p.contained_frac)


class FilePoses(unittest.TestCase):
    """At the file's poses the census IS check_assembly's channel."""

    def test_census_equals_grade_body_overlap(self):
        for name in BOARDS:
            path = _board(name)
            pcb = parse_kicad_pcb(path)
            g = legality.grade_body_overlap(pcb, 0.2, pcb_file=path)
            cg = legality.courtyard_census(pcb, path)
            want = [_key(p) for p in g['pairs'] if p.kind == 'courtyard']
            self.assertEqual([_key(p) for p in cg.pairs], want, name)
            self.assertEqual([_key(p) for p in cg.blocking],
                             [_key(p) for p in g['courtyard_blocking_pairs']],
                             name)
            self.assertEqual(sorted(cg.synthetic_refs),
                             g['courtyard_synthetic_refs'], name)
            # Not vacuous: every board here carries courtyard pairs.
            self.assertTrue(want, f'{name}: no courtyard pair to compare')


class MovedPoses(unittest.TestCase):
    """At a moved pose the census answers what the grader answers on a board
    with the part WRITTEN there -- by the real writer, re-parsed, graded from
    its own file. (A copy built with `footprint_at_pose` would share the pose
    path's code and could not tell the two apart; the phase-0 verifier.)"""

    @classmethod
    def setUpClass(cls):
        cls._td = tempfile.TemporaryDirectory()

    @classmethod
    def tearDownClass(cls):
        cls._td.cleanup()

    def _written_grade(self, path, moves):
        """`grade_body_overlap` of `path` with `moves` ({ref: pose}) written
        by `write_placed_output`, and the poses as the written file reads
        them back (the census is asked about exactly those)."""
        from placement.writer import write_placed_output
        out = os.path.join(self._td.name,
                           f'w{len(os.listdir(self._td.name))}.kicad_pcb')
        write_placed_output(path, out, [
            {'reference': r, 'new_x': x, 'new_y': y, 'new_rotation': rot}
            for r, (x, y, rot) in moves.items()])
        for ext in ('.kicad_pro', '.kicad_dru'):
            sib = os.path.splitext(path)[0] + ext
            if os.path.exists(sib):
                shutil.copyfile(sib, os.path.splitext(out)[0] + ext)
        pcb = parse_kicad_pcb(out)
        back = {r: (pcb.footprints[r].x, pcb.footprints[r].y,
                    pcb.footprints[r].rotation or 0.0) for r in moves}
        return legality.grade_body_overlap(pcb, 0.2, pcb_file=out), back

    def _agree(self, census, path, moves, label):
        g, back = self._written_grade(path, moves)
        got = census.grade(back)
        want = [_key(p) for p in g['pairs'] if p.kind == 'courtyard']
        self.assertEqual([_key(p) for p in got.pairs], want, label)
        self.assertEqual([_key(p) for p in got.blocking],
                         [_key(p) for p in g['courtyard_blocking_pairs']],
                         label)
        return got, back

    def test_random_single_part_moves(self):
        rng = random.Random(1182)
        checked = nonempty = 0
        for name in ('glasgow_revC', 'esp_prog', 'sonde_u'):
            path = _board(name)
            pcb = parse_kicad_pcb(path)
            census = legality.CourtyardCensus(pcb, path)
            refs = sorted(census.lbs)
            for _ in range(8):
                ref = rng.choice(refs)
                fp = pcb.footprints[ref]
                pose = (round(fp.x + rng.uniform(-3, 3), 3),
                        round(fp.y + rng.uniform(-3, 3), 3),
                        rng.choice((0.0, 90.0, 180.0, 270.0, 45.0)))
                got, back = self._agree(census, path, {ref: pose},
                                        f'{name} {ref} at {pose}')
                # grade_ref is grade restricted to the moved part.
                one = census.grade_ref(ref, back[ref])
                self.assertEqual(
                    [_key(p) for p in one.pairs],
                    [_key(p) for p in got.pairs if ref in (p.a, p.b)],
                    f'{name} {ref} at {pose}')
                checked += 1
                nonempty += bool(one.pairs)
        # A sample that never produced a pair would compare nothing.
        self.assertGreaterEqual(nonempty, 3, f'{nonempty} of {checked}')

    def test_a_turn_that_changes_container_status(self):
        """sonde_u's J1 covers 0.29 of the board at its file pose and more
        than half at 135 degrees, where its rotated rect grows: it becomes a
        container. Graded at the file's container set, the census labelled
        its pairs `edge_class` and BLOCKING where the written board waives
        them; graded the other way (built on J1 at 45, graded at -90) it
        waived 6 pairs the written board gates."""
        path = _board('sonde_u')
        pcb = parse_kicad_pcb(path)
        census = legality.CourtyardCensus(pcb, path)
        self.assertNotIn('J1', census.containers)
        got, _b = self._agree(census, path, {'J1': (104.55, 88.019, 135.0)},
                              'J1 at 135')
        self.assertIn('J1', got.containers, 'the fixture no longer turns J1 '
                      'into a container; this arm tests nothing')
        g45, _b = self._written_grade(path, {'J1': (104.55, 88.019, 45.0)})
        turned = os.path.join(self._td.name,
                              f'w{len(os.listdir(self._td.name)) - 1}'
                              '.kicad_pcb')
        census45 = legality.CourtyardCensus(parse_kicad_pcb(turned), turned)
        self.assertIn('J1', census45.containers)
        got2, _b = self._agree(census45, turned,
                               {'J1': (109.512, 77.103, -90.0)},
                               'J1 back at -90')
        self.assertTrue(got2.blocking, 'the dangerous direction: the written '
                        'board gates pairs here')

    def test_the_edge_waiver_reads_the_graded_pose(self):
        """An edge-class waiver holds only for a part AT an edge, so it must
        be judged where the part is being graded, not where the file has it:
        ulx3s B1 moved inland loses its waiver (19 blocking pairs, not 18)."""
        path = _board('ulx3s')
        census = legality.CourtyardCensus(parse_kicad_pcb(path), path)
        before = len(census.grade().blocking)
        got, _b = self._agree(census, path, {'B1': (97.53, 81.36, 0.0)},
                              'B1 inland')
        self.assertNotEqual(len(got.blocking), before)

    def test_grade_ref_takes_the_other_parts_poses(self):
        """`grade_ref(ref, pose, poses)` grades the OTHER parts at `poses`
        too: esp_prog's CON2 moved away must not still be paired with USB1
        from its file pose."""
        path = _board('esp_prog')
        pcb = parse_kicad_pcb(path)
        census = legality.CourtyardCensus(pcb, path)
        u, c = pcb.footprints['USB1'], pcb.footprints['CON2']
        pose = (u.x, u.y, u.rotation or 0.0)
        # CON2 moved ONTO USB1: the pair exists only at the moved pose, so a
        # grade_ref that ignored `poses` would not see it.
        onto = (u.x + 1.0, u.y, c.rotation or 0.0)
        at_file = census.grade_ref('USB1', pose)
        self.assertFalse(any({'USB1', 'CON2'} == {p.a, p.b}
                             for p in at_file.pairs))
        one = census.grade_ref('USB1', pose, {'CON2': onto})
        full = census.grade({'USB1': pose, 'CON2': onto})
        self.assertEqual([_key(p) for p in one.pairs],
                         [_key(p) for p in full.pairs
                          if 'USB1' in (p.a, p.b)])
        self.assertTrue(any({'USB1', 'CON2'} == {p.a, p.b}
                            for p in one.pairs))


class Determinism(unittest.TestCase):
    """An exact area tie between the two faces must not be settled by
    PYTHONHASHSEED: `courtyard_pair` walks the shared faces SORTED."""

    def test_a_face_tie_names_the_same_side_under_every_hash_seed(self):
        import subprocess
        code = (
            "import sys; sys.path[:0] = ['py_placer', 'py_router']\n"
            "from placement import legality as L\n"
            "a = L.GradedPart('A', 'F', (0, 0, 2, 2), (0, 0, 2, 2), True)\n"
            "b = L.GradedPart('B', 'B', (1, 0, 3, 2), (1, 0, 3, 2), True)\n"
            "print(L.courtyard_pair(a, b).side)\n")
        sides = set()
        for seed in range(8):
            p = subprocess.run([sys.executable, '-B', '-c', code],
                               capture_output=True, text=True, cwd=ROOT,
                               env=dict(os.environ,
                                        PYTHONHASHSEED=str(seed)))
            self.assertEqual(p.returncode, 0, p.stderr[-500:])
            sides.add(p.stdout.strip())
        self.assertEqual(sides, {'B'})


class Gating(unittest.TestCase):

    def test_moved_refs_currency(self):
        def fp(x, y, rot=0.0, layer='F.Cu'):
            return types.SimpleNamespace(x=x, y=y, rotation=rot, layer=layer)
        base = types.SimpleNamespace(footprints={
            'A': fp(0, 0), 'B': fp(1, 1, 90), 'C': fp(2, 2), 'D': fp(3, 3)})
        now = types.SimpleNamespace(footprints={
            'A': fp(0, 0.0005),           # under eps: not moved
            'B': fp(1, 1, 450),           # 450 == 90 mod 360: not moved
            'C': fp(2, 2, 0, 'B.Cu'),     # flipped: moved
            'D': fp(3.01, 3),             # moved
            'E': fp(9, 9),                # absent from the baseline: moved
            'F': fp(4, 4, 0)})            # rotation-only, a NEGATIVE delta
        base.footprints['F'] = fp(4, 4, 90)
        base.footprints['G'] = fp(5, 5, 360)
        now.footprints['G'] = fp(5, 5, 10)  # 10 vs 360: moved 10 degrees
        self.assertEqual(legality.moved_refs(now, base),
                         {'C', 'D', 'E', 'F', 'G'})

    def test_gate_matches_check_assembly_on_run23(self):
        placed = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                              'tigard_placed.kicad_pcb')
        damaged = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                               'tigard_damaged.kicad_pcb')
        pcb, base = parse_kicad_pcb(placed), parse_kicad_pcb(damaged)
        moved = legality.moved_refs(pcb, base)
        cg = legality.courtyard_census(pcb, placed, moved=moved)
        # tests/test_run23_courtyard_channel.py pins check_assembly at 9.
        self.assertEqual(len(cg.gating), 9)
        self.assertEqual(legality.courtyard_census(pcb, placed).gating, None)


class AuditCanFail(unittest.TestCase):
    """The gate-vs-grader instrument reports UNDER when the generator stops
    testing courtyards -- otherwise a clean audit means nothing."""

    def test_disabled_courtyard_test_reports_under(self):
        import measure_gate_vs_grader as audit
        import pose_score
        # tigard: a dense courtyard-drawing board where a 1 mm move reaches a
        # neighbour past the floors. (esp_prog cannot serve: its sparse
        # admitted set never reaches a gating pair even with the courtyard
        # test switched off -- measured, 0 of 52.)
        path = _board('tigard')
        pcb = parse_kicad_pcb(path)
        clean = audit.audit_board(path, radius=1, clearance=0.25,
                                  max_parts=40)
        state = pose_score.make_state(pcb, path, clearance=0.25)
        # Every part a "container": the seat skips every courtyard test.
        state.container_refs = set(state.parts)
        broken = audit.audit_board(path, radius=1, clearance=0.25,
                                   state=state, max_parts=40)
        self.assertEqual(clean['under'], 0, clean['under_cases'][:3])
        self.assertGreater(broken['under'], 0)


if __name__ == '__main__':
    unittest.main()
