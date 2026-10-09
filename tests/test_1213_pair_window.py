"""#1213: the pad gate prices a hollow part by its pads, not its extent.

`LegalityContext.pair_shortfall` skipped its per-pad sweep once a pair's pad
product passed `PAIR_TEST_CAP` and charged the gap between the two parts'
EXTENTS as pad shortfall. rp2350's U8 is the Teensy 4.0 frame -- 66 pins on
the board edge -- and U6 has 71 pads, so 4686 > 4096: U8's extent covers the
whole board, every U6 pose read as pad overlap plus stack, and `place_seed`
exited 4 with `U6: no legal pose anywhere on the board`, never naming U8.

The fix windows the sweep (`_pad_windows`): only pads within reach of the
other part count, toward the sweep AND toward the cap. What must hold:

  * rp2350 U6/U8 grades clean at the real cap, and its windowed product is 0;
  * the windowed sweep returns EXACTLY what the uncapped full sweep returns,
    bit for bit, wherever it does not hand over to the extent branch --
    randomized over corpus pairs, including boards whose per-pad clearance
    model is active;
  * the window keeps ORIGINAL pad indices (the per-pad floors and the #1127
    exact-stack confirmation key on them);
  * the hole channel still reads the FULL pad lists;
  * the extent branch is still reachable, by pads that really face each
    other;
  * under the old gate, the no-pose census names the refusing pair: lifting
    U8 frees U6 (`frozen_blocks`).
"""

import os
import random
import sys
import tempfile
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import legality                               # noqa: E402
from placement.legality import (LegalityContext, PAIR_TEST_CAP,  # noqa: E402
                                PadClearanceModel, build_part_pads)
from synth import make_pad                                   # noqa: E402

RP2350 = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')


def _full(rects_a, ea, rects_b, reach):
    """The pre-#1213 sweep: every pad of each part."""
    return list(enumerate(rects_a)), list(enumerate(rects_b))


def _ctx(fps, clearance, model=None):
    parts = build_part_pads(fps, clearance, model)
    poses = {r: (f.x, f.y, f.rotation) for r, f in fps.items()}
    return LegalityContext(parts, None, clearance, pose_of=poses.get,
                           seed_of=poses.get, model=model), parts


def _fp(ref, x, y, pads, rot=0.0):
    return SimpleNamespace(reference=ref, x=x, y=y, rotation=rot,
                           layer='F.Cu', pads=pads)


def _cu(x, y, ref, num, net=7, sx=1.0, sy=1.0, lc=None):
    p = make_pad(net_id=net, x=x, y=y, ref=ref, num=num, size_x=sx,
                 size_y=sy, shape='rect', layers=['F.Cu'], drill=0.0,
                 pad_type='smd', local_x=x, local_y=y)
    if lc is not None:
        p.local_clearance = lc
    return p


class Witness(unittest.TestCase):

    def test_rp2350_u6_inside_the_teensy_frame(self):
        pcb = parse_kicad_pcb(RP2350)
        fps = {r: pcb.footprints[r] for r in ('U6', 'U8')}
        ctx, parts = _ctx(fps, 0.09)
        # The fixture still trips the old cap -- else this tests nothing.
        self.assertGreater(parts['U6'].n_pads * parts['U8'].n_pads,
                           PAIR_TEST_CAP)
        sf = ctx.pair_shortfall('U6', 'U8')
        self.assertEqual(tuple(sf), (0.0, False, 0.0, False))
        u6, u8 = fps['U6'], fps['U8']
        ra = parts['U6'].pad_rects(u6.x, u6.y, u6.rotation)
        rb = parts['U8'].pad_rects(u8.x, u8.y, u8.rotation)
        wa, wb = legality._pad_windows(
            ra, parts['U6'].extent(u6.x, u6.y, u6.rotation), rb, 0.09)
        self.assertEqual(len(wa) * len(wb), 0)


class Equivalence(unittest.TestCase):
    """Windowed == uncapped full sweep, bit for bit, below the cap."""

    def test_randomized_corpus_pairs(self):
        import pose_score
        rng = random.Random(1213)
        compared = active_boards = 0
        old_cap = legality.PAIR_TEST_CAP
        try:
            for name in ('esp_prog', 'glasgow_revC', 'ulx3s',
                         'rp2350_fpga_eensy_prePlane'):
                path = os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')
                pcb = parse_kicad_pcb(path)
                st = pose_score.make_state(pcb, path, clearance=0.2)
                ctx = st.legality_ctx
                active_boards += ctx._floors is not None
                refs = sorted(r for r in st.parts if r in ctx.parts)
                for _ in range(150):
                    a = rng.choice(refs)
                    pa = st.parts[a]
                    near = sorted(refs, key=lambda r: (
                        (st.parts[r].x - pa.x) ** 2
                        + (st.parts[r].y - pa.y) ** 2))[1:6]
                    b = rng.choice(near)
                    pose = (pa.x + rng.uniform(-1.5, 1.5),
                            pa.y + rng.uniform(-1.5, 1.5),
                            rng.choice((pa.rot, pa.rot + 90, pa.rot + 45)))
                    legality.PAIR_TEST_CAP = old_cap
                    got = ctx.pair_shortfall(a, b, pose_a=pose)
                    legality.PAIR_TEST_CAP = 10 ** 9
                    real = legality._pad_windows
                    legality._pad_windows = _full
                    try:
                        want = ctx.pair_shortfall(a, b, pose_a=pose)
                    finally:
                        legality._pad_windows = real
                    self.assertEqual(tuple(got), tuple(want),
                                     f'{name} {a}/{b} at {pose}')
                    compared += 1
        finally:
            legality.PAIR_TEST_CAP = old_cap
        self.assertEqual(compared, 600)
        # The per-pad floor path is exercised, not just the flat scalar.
        self.assertGreaterEqual(active_boards, 2)

    def test_the_window_keeps_original_pad_indices(self):
        """A's index-0 pad is far away and carries a 2.0 mm keep-clear; its
        index-1 pad is 0.6 mm from B's pad and carries none. The window drops
        index 0. Read by window POSITION, A's near pad would be priced at
        2.0 mm and charged 1.4 mm of shortfall."""
        a = _fp('A', 0.0, 0.0, [_cu(-20.0, 0.0, 'A', '1', lc=2.0),
                                _cu(0.0, 0.0, 'A', '2')])
        b = _fp('B', 1.6, 0.0, [_cu(1.6, 0.0, 'B', '1', net=8)])
        model = PadClearanceModel(0.25, has_overrides=True)
        ctx, parts = _ctx({'A': a, 'B': b}, 0.25, model)
        ra = parts['A'].pad_rects(0.0, 0.0, 0.0)
        rb = parts['B'].pad_rects(1.6, 0.0, 0.0)
        reach = max(0.25, parts['A'].max_floor, parts['B'].max_floor)
        wa, wb = legality._pad_windows(ra, parts['A'].extent(0.0, 0.0, 0.0),
                                       rb, reach)
        self.assertEqual([i for i, _r in wa], [1], 'the fixture must drop '
                         'index 0, or it cannot tell index from position')
        self.assertEqual(ctx.pair_shortfall('A', 'B').pad, 0.0)

    def test_the_hole_channel_reads_the_full_lists(self):
        """An NPTH keep-out reaching B's pad while every COPPER pad of A is
        out of reach: the windows are empty, and the hole is still billed."""
        hole = make_pad(net_id=0, x=0.0, y=0.0, ref='H', num='H1',
                        size_x=2.0, size_y=2.0, shape='circle',
                        layers=['F.Mask', 'B.Mask'], drill=2.0,
                        pad_type='np_thru_hole')
        hole.local_clearance = 1.0
        h = _fp('H', 0.0, 0.0, [hole, _cu(-15.0, 0.0, 'H', '2')])
        c = _fp('C', 1.5, 0.0, [_cu(1.5, 0.0, 'C', '1', sx=0.4, sy=0.4)])
        ctx, parts = _ctx({'H': h, 'C': c}, 0.25)
        ra = parts['H'].pad_rects(0.0, 0.0, 0.0)
        rb = parts['C'].pad_rects(1.5, 0.0, 0.0)
        wa, _wb = legality._pad_windows(ra, parts['H'].extent(0, 0, 0), rb,
                                        0.25)
        self.assertEqual(wa, [], 'A copper pad within reach makes this a '
                         'test of the sweep, not of the hole channel')
        self.assertGreater(ctx.pair_shortfall('H', 'C').hole, 0.0)

    def test_the_extent_branch_is_still_reachable(self):
        """Two 9x9 lattices that genuinely face each other: 6561 windowed
        pad pairs, over the cap, so the extent verdict still answers."""
        def lattice(ref, x0, net):
            return [_cu(x0 + 0.3 * i, 0.3 * j, ref, f'{i}_{j}', net=net,
                        sx=0.1, sy=0.1) for i in range(9) for j in range(9)]
        a = _fp('A', 0.0, 0.0, lattice('A', 0.0, 5))
        b = _fp('B', 0.0, 0.0, lattice('B', 0.15, 6))
        ctx, parts = _ctx({'A': a, 'B': b}, 0.25)
        calls = []
        real = legality.PartPads.extent_side

        def spy(self, *args):
            calls.append(args)
            return real(self, *args)
        legality.PartPads.extent_side = spy
        try:
            sf = ctx.pair_shortfall('A', 'B')
        finally:
            legality.PartPads.extent_side = real
        self.assertTrue(calls, 'the extent branch was not taken')
        self.assertTrue(sf.stack and sf.pad_overlap)


class Refusers(unittest.TestCase):
    """`seeder._frozen_refusers`: who the census may NAME. A frozen part
    that refuses only SOME open poses is in the way, not the refusing
    member of the pair, and naming it would send the reader to unlock the
    wrong part."""

    def test_only_a_total_refusal_or_a_freeing_lift_is_named(self):
        from placement import seeder
        census = {'baseline': 0, 'open_poses': 64,
                  'frozen_alone': {'A': 0, 'B': 10, 'C': 64},
                  'frozen_lifted': {'A': 0, 'B': 0, 'C': 0, 'D': 5}}
        got = seeder._frozen_refusers(census)
        self.assertEqual([r for r, _how in got], ['D', 'A'])
        self.assertIn('frees 5', got[0][1])
        # 64 IS the census cap, so the note says so rather than printing
        # the cap as if it were a count.
        self.assertIn('alone refuses all of the first 64 open poses '
                      'censused (the census cap)', got[1][1])
        below = dict(census, open_poses=12, frozen_alone={'A': 0})
        self.assertIn('alone refuses all 12 open pose(s)',
                      seeder._frozen_refusers(below)[-1][1])

    def test_the_immovable_note_carries_the_counts(self):
        from placement import seeder
        census = {'baseline': 0, 'open_poses': 12, 'censused': 0,
                  'frozen': {'U8': 'file-locked'},
                  'frozen_alone': {'U8': 0}, 'frozen_lifted': {'U8': 0}}
        note = seeder._no_pose_note('U6', 'immovable_given_frozen', census)
        self.assertIn('measured: U8 alone refuses all 12', note)

    def test_no_open_pose_names_nobody_by_the_alone_count(self):
        """With nothing open at all, "refuses every open pose" is vacuous."""
        from placement import seeder
        census = {'baseline': 0, 'open_poses': 0,
                  'frozen_alone': {'A': 0}, 'frozen_lifted': {'A': 0}}
        self.assertEqual(seeder._frozen_refusers(census), [])


class OnThePile(unittest.TestCase):
    """The issue's own shape on TRACKED data: rp2350 staged as an unaided
    pile (U8 file-locked, the mechanical refs at their poses, everything else
    in one stack), seeded from its emitted intent -- the A/B harness's own
    `_pile_inputs`, so this is the basis the placement A/B measures. On a
    pile U6 has no seed licence against U8, which is what made the extent
    verdict refuse it everywhere.

    The board the #982 measurement seeds (the designer's poses, re-seated)
    cannot stand in: there U6's seed pose licenses its overlap with U8, and
    measured, U8 alone then admits every censused pose."""

    @classmethod
    def setUpClass(cls):
        import test_placement_ab as ab
        cls._td = tempfile.TemporaryDirectory()
        cls.pile, cls.intent, _doc, _refs = ab._pile_inputs(
            RP2350, cls._td.name, require_decaps=False)

    @classmethod
    def tearDownClass(cls):
        cls._td.cleanup()

    def _seed(self):
        from placement import seeder
        return seeder.seed_from_intent(
            parse_kicad_pcb(self.pile), self.pile, self.intent,
            random.Random('0'), clearance=0.2, board_edge_clearance=0.55,
            grid_step=0.1)

    def test_u6_seats(self):
        res = self._seed()
        self.assertNotIn('U6', res['unseated'] or [])

    def test_old_gate_refusal_names_u8(self):
        """`_pad_windows` patched back to the full lists: the pre-#1213 gate.
        The census must name U8 as the refusing member."""
        real = legality._pad_windows
        legality._pad_windows = _full
        try:
            res = self._seed()
        finally:
            legality._pad_windows = real
        self.assertIn('U6', res['unseated'] or [], 'the old gate no longer '
                      'refuses U6 on the pile: this arm tests nothing')
        self.assertEqual(res['no_pose_verdict'].get('U6'), 'frozen_blocks')
        census = res['no_pose_census']['U6']
        # Lifting U8 ALONE frees nothing (the frame is full by U6's turn) --
        # which is why `frozen_alone` exists: with every other neighbour
        # lifted, U8 alone still admits none of the open poses...
        self.assertGreater(census['open_poses'], 0)
        self.assertEqual(census['frozen_alone'].get('U8'), 0)
        # ...and it is U8 that does it, not every frozen part.
        self.assertTrue(any(n > 0 for r, n in census['frozen_alone'].items()
                            if r != 'U8'), census['frozen_alone'])
        self.assertTrue(any('U6/U8 is the refusing pair' in n
                            for n in res['notes']), res['notes'])


if __name__ == '__main__':
    unittest.main()
