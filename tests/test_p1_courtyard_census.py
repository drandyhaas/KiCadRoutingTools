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

import copy
import os
import random
import sys
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
    with the part WRITTEN there."""

    def _written(self, pcb, ref, pose):
        out = copy.copy(pcb)
        out.footprints = dict(pcb.footprints)
        out.footprints[ref] = legality.footprint_at_pose(pcb.footprints[ref],
                                                         pose)
        return out

    def test_random_single_part_moves(self):
        rng = random.Random(1182)
        checked = nonempty = 0
        for name in ('glasgow_revC', 'esp_prog'):
            path = _board(name)
            pcb = parse_kicad_pcb(path)
            census = legality.CourtyardCensus(pcb, path)
            refs = sorted(census.lbs)
            for _ in range(12):
                ref = rng.choice(refs)
                fp = pcb.footprints[ref]
                pose = (round(fp.x + rng.uniform(-3, 3), 3),
                        round(fp.y + rng.uniform(-3, 3), 3),
                        rng.choice((0.0, 90.0, 180.0, 270.0, 45.0)))
                got = census.grade({ref: pose})
                g = legality.grade_body_overlap(
                    self._written(pcb, ref, pose), 0.2, pcb_file=path)
                want = [_key(p) for p in g['pairs'] if p.kind == 'courtyard']
                self.assertEqual([_key(p) for p in got.pairs], want,
                                 f'{name} {ref} at {pose}')
                self.assertEqual(
                    [_key(p) for p in got.blocking],
                    [_key(p) for p in g['courtyard_blocking_pairs']],
                    f'{name} {ref} at {pose}')
                # grade_ref is grade restricted to the moved part.
                one = census.grade_ref(ref, pose)
                self.assertEqual(
                    [_key(p) for p in one.pairs],
                    [_key(p) for p in got.pairs if ref in (p.a, p.b)],
                    f'{name} {ref} at {pose}')
                checked += 1
                nonempty += bool(one.pairs)
        # A sample that never produced a pair would compare nothing.
        self.assertGreaterEqual(nonempty, 3, f'{nonempty} of {checked}')


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
            'E': fp(9, 9)})               # absent from the baseline: moved
        self.assertEqual(legality.moved_refs(now, base), {'C', 'D', 'E'})

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
