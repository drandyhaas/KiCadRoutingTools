"""Run-5 exchange stage (commit 3): the joint +/-v lattice ILP.

Board-only assertions: the swap-corpus tests check gate facts (conflicts,
oob, hpwl direction) and the tool's own reports -- never ground truth.
"""

import json
import math
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_utils import tool as _tool, tool_env as _tool_env  # noqa: E402
SWAP = os.path.join(ROOT, 'wk', 'b2', 'tigard__swap', 'd0',
                    'perturbed.kicad_pcb')
HEALTHY = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')


def _state(board):
    from kicad_parser import parse_kicad_pcb
    from pose_score import make_state
    return make_state(parse_kicad_pcb(board), board, clearance=0.09)


def _run(script, *argv):
    # #718/#522: check_floorplan.py -> py_tools/, place_reconstruct.py ->
    # py_placer/ (a241b5ac, not an ancestor of this file's branch). The joined
    # ROOT path went stale on merge, CPython exited 2 with "can't open file" on
    # STDERR, and `assertEqual(r0.returncode, 0)` read that as a tool exit code
    # -- so TestLadderEndToEnd's body never ran at all. `tool()` raises here.
    env = _tool_env(dict(os.environ, PYTHONIOENCODING='utf-8',
                         KRT_NO_BANNER='1'))
    return subprocess.run(
        [sys.executable, '-X', 'utf8', _tool(script), *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


def _summary(stdout):
    line = [l for l in stdout.splitlines() if l.startswith('JSON_SUMMARY:')]
    return json.loads(line[0].split('JSON_SUMMARY:', 1)[1])


class TestExchangeUnit(unittest.TestCase):
    def test_no_vectors_is_a_noop(self):
        """Pattern-gating: without vectors the stage does nothing -- the
        healthy-board no-op guarantee rides on this."""
        from placement import reconstruct as R
        st = _state(HEALTHY)
        before = {r: (p.x, p.y) for r, p in st.parts.items()}
        rep, old = R.exchange_stage(st, [])
        self.assertEqual(rep['accepted'], [])
        self.assertEqual(old, {})
        for r, (x, y) in before.items():
            self.assertEqual((st.parts[r].x, st.parts[r].y), (x, y), r)

    def test_fabricated_vector_on_healthy_board_restores_exactly(self):
        """A vector forced onto a healthy board must either be rejected
        (poses bit-identical after the snapshot restore) or strictly
        improve the gate -- no drift either way."""
        from placement import reconstruct as R
        st = _state(HEALTHY)
        before = {r: (p.x, p.y, p.rot) for r, p in st.parts.items()}
        gate0 = R.measure(st)
        rep, old = R.exchange_stage(st, [(4.0, 7.0)])
        gate1 = R.measure(st)
        self.assertLessEqual(gate1, gate0)
        if not rep['accepted']:
            for r, pose in before.items():
                p = st.parts[r]
                self.assertEqual((p.x, p.y, p.rot), pose,
                                 f'{r} drifted on a rejected exchange')

    def test_input_lattice_bounds_every_move(self):
        """No accepted exchange may put a part more than one lattice step
        from its INPUT pose (the anti-gaming rule: relocating
        never-displaced parts for hpwl is structurally unreachable)."""
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        from placement import reconstruct as R
        st = _state(SWAP)
        tiers = R.classify(st, None, 'auto')
        excl = set(tiers.locked) | set(tiers.zero_net)
        proposals = R.fit_corner_insets(st, tiers)
        vectors = R.rigid_vectors(st, proposals)
        self.assertTrue(vectors, 'fixture: the swap board yields a vector')
        vx, vy = vectors[0]
        rep, old = R.exchange_stage(st, vectors, exclude=excl)
        for r in old:
            p = st.parts[r]
            dx, dy = p.x - p.seed_x, p.y - p.seed_y
            j = min((math.hypot(dx - j * vx, dy - j * vy), j)
                    for j in (-1, 0, 1))[0]
            self.assertLessEqual(j, R.EXCHANGE_LATTICE_TOL,
                                 f'{r} ended off the +/-1 input lattice')


class TestSymmetricVectorElimination(unittest.TestCase):
    def test_crossed_pairing_is_dropped(self):
        """Two refs offered each other's slots mint two vectors (straight
        and crossed) with identical 2-ref support; only the
        minimal-mean-displacement pairing survives (run-3's spurious
        cross-corner vector, eliminated rather than contained)."""
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        from placement import reconstruct as R
        st = _state(SWAP)
        a, b = sorted(st.parts)[:2]
        pa, pb = st.parts[a], st.parts[b]
        # straight: each ref 10mm east; crossed: each to the OTHER's slot
        slots = {a: [(pa.x + 10.0, pa.y), (pb.x + 10.0, pb.y)],
                 b: [(pb.x + 10.0, pb.y), (pa.x + 10.0, pa.y)]}
        vecs = R.rigid_vectors(st, slots)
        self.assertEqual(len(vecs), 1,
                         f'only the straight pairing may survive: {vecs}')
        # canonicalized up to sign: the straight pairing's vector is
        # (+/-10, 0); the crossed one depends on the ref offset and differs
        self.assertAlmostEqual(abs(vecs[0][0]), 10.0, places=3)
        self.assertAlmostEqual(vecs[0][1], 0.0, places=3)


class TestLadderEndToEnd(unittest.TestCase):
    def test_exchange_homes_the_island(self):
        """The full ladder on the swap board. Board-only contract: the
        run-4 measured ceiling (SW1's pad copper off-board at the end, J1
        unrepairable, hpwl stuck) is BROKEN by the exchange -- final oob 0,
        0 conflicts, an accepted joint move containing SW1 AND J1, and
        exit 0."""
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        with tempfile.TemporaryDirectory() as td:
            intent = os.path.join(td, 'auto.json')
            r0 = _run('check_floorplan.py', SWAP, '--emit-intent', intent,
                      '--declare-classes')
            self.assertEqual(r0.returncode, 0)
            out = os.path.join(td, 'r.kicad_pcb')
            r = _run('place_reconstruct.py', SWAP, out,
                     '--clearance', '0.09', '--intent', intent,
                     '--assign-rounds', '3')
            self.assertEqual(r.returncode, 0,
                             r.stdout[-900:] + r.stderr[-400:])
            doc = _summary(r.stdout)
            xch = doc.get('exchange') or {}
            self.assertTrue(xch.get('accepted'),
                            'the exchange must accept a joint move')
            moved = set(xch.get('moved') or [])
            self.assertIn('SW1', moved)
            self.assertIn('J1', moved)
            fin = doc['final']
            self.assertEqual(fin['pad_conflicts'], 0)
            self.assertEqual(fin['hole_conflicts'], 0)
            self.assertEqual(fin['oob_pad_count'], 0,
                             'SW1 must end fully on-board (run-4 residue)')
            # gate hpwl must IMPROVE across the exchange (structure, not
            # legality, is what the island costs)
            self.assertLess(doc['gate_after_exchange'][3],
                            doc['gate_before'][3])


if __name__ == '__main__':
    unittest.main()
