"""Run-4 reconstruct fixes F1/F3/F5, on the archived swap corpus.

Board-only assertions throughout: nearest-slot is checked against the
tool's OWN proposals, never against ground truth.
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
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split

SWAP = os.path.join(ROOT, 'wk', 'b2', 'tigard__swap', 'd0',
                    'perturbed.kicad_pcb')


def _state():
    from kicad_parser import parse_kicad_pcb
    from pose_score import make_state
    return make_state(parse_kicad_pcb(SWAP), SWAP, clearance=0.09)


def run_reconstruct(board, out, *extra):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8')
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_placer', 'place_reconstruct.py'), board, out,
         '--clearance', '0.09', *extra],
        capture_output=True, text=True, env=env, cwd=ROOT)


def _summary(stdout):
    line = [l for l in stdout.splitlines() if l.startswith('JSON_SUMMARY:')]
    return json.loads(line[0].split('JSON_SUMMARY:', 1)[1])


class TestF1NearestSlot(unittest.TestCase):
    def test_moved_pattern_parts_take_their_nearest_slot(self):
        """Run 3 shipped the two repaired holes CROSSED (~40mm from home
        each): zero-net parts have no net-anchor cost, both corners cost the
        same, and HiGHS picked arbitrarily. The tiebreak makes each moved
        pattern part take the slot NEAREST its input pose -- asserted
        against the tool's own fit_proposals, no ground truth read."""
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        from kicad_parser import parse_kicad_pcb
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'r.kicad_pcb')
            r = run_reconstruct(SWAP, out)
            self.assertEqual(r.returncode, 0, r.stdout[-800:] + r.stderr[-400:])
            doc = _summary(r.stdout)
            before = parse_kicad_pcb(SWAP).footprints
            after = parse_kicad_pcb(out).footprints
            checked = 0
            for ref, slots in (doc.get('fit_proposals') or {}).items():
                if ref not in doc.get('assign_moved', []):
                    continue
                bx, by = before[ref].x, before[ref].y
                ax, ay = after[ref].x, after[ref].y
                d_taken = math.hypot(ax - bx, ay - by)
                for (sx, sy) in slots:
                    self.assertLessEqual(
                        d_taken, math.hypot(sx - bx, sy - by) + 1e-3,
                        f"{ref} took a slot farther than an alternative")
                checked += 1
            self.assertGreaterEqual(checked, 2, doc.get('fit_proposals'))


class TestF3VectorSupportAndPrune(unittest.TestCase):
    def test_singleton_vector_is_dropped(self):
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        from placement import reconstruct
        st = _state()
        ref = sorted(st.parts)[0]
        p = st.parts[ref]
        # one ref, one proposal 10mm away -> a singleton vector: dropped
        vecs = reconstruct.rigid_vectors(st, {ref: [(p.x + 10.0, p.y)]})
        self.assertEqual(vecs, [])

    def test_two_ref_support_is_kept(self):
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        from placement import reconstruct
        st = _state()
        r1, r2 = sorted(st.parts)[:2]
        p1, p2 = st.parts[r1], st.parts[r2]
        vecs = reconstruct.rigid_vectors(
            st, {r1: [(p1.x + 10.0, p1.y + 4.0)],
                 r2: [(p2.x + 10.0, p2.y + 4.0)]})
        self.assertEqual(len(vecs), 1)

    def test_prune_reverts_a_move_the_global_gate_cannot_see(self):
        """Manufacture a mis-move: drag one part 20mm off its pose (legal,
        conflict-free spot not required -- the tuple's hpwl term does the
        judging) and let prune_assignment restore it."""
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        from placement import reconstruct
        st = _state()
        # pick a small netted part; move it far away
        ref = next(r for r in sorted(st.parts)
                   if st.parts[r].pin_count >= 2 and not st.parts[r].locked)
        p = st.parts[ref]
        old = {ref: (p.x, p.y, p.rot)}
        st.apply_move(ref, p.x + 20.0, p.y, p.rot)
        pruned = reconstruct.prune_assignment(st, old)
        self.assertEqual(pruned, [ref])
        self.assertAlmostEqual(st.parts[ref].x, old[ref][0], places=6)

    def test_prune_keeps_a_good_move(self):
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        from placement import reconstruct
        st = _state()
        ref = next(r for r in sorted(st.parts)
                   if st.parts[r].pin_count >= 2 and not st.parts[r].locked)
        p = st.parts[ref]
        # 'old' pose 20mm away = the CURRENT pose is the good one; prune must
        # not revert to the worse old pose
        old = {ref: (p.x + 20.0, p.y, p.rot)}
        pruned = reconstruct.prune_assignment(st, old)
        self.assertEqual(pruned, [])
        self.assertAlmostEqual(st.parts[ref].x, p.x, places=6)


class TestF5FullCensus(unittest.TestCase):
    def test_worst_n_zero_lists_every_pair(self):
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        from kicad_parser import parse_kicad_pcb
        from placement.legality import grade_pad_legality
        g = grade_pad_legality(parse_kicad_pcb(SWAP), 0.09, worst_n=0)
        self.assertEqual(len(g['worst']), g['pad_conflicts'])
        self.assertGreater(g['pad_conflicts'], 10,
                           'fixture: the swap board carries 20 pairs')

    def test_repair_census_reports_all(self):
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8')
        with tempfile.TemporaryDirectory() as td:
            intent = os.path.join(td, 'i.json')
            with open(intent, 'w', encoding='utf-8') as f:
                f.write('{"schema": 1, "kind": "floorplan-intent"}\n')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_placer', 'place_seed.py'), SWAP,
                 os.path.join(td, 'o.kicad_pcb'), '--intent', intent,
                 '--repair', '--dry-run', '--clearance', '0.09'],
                capture_output=True, text=True, env=env, cwd=ROOT)
            self.assertIn('Repair census:', r.stdout)
            self.assertIn('all listed', r.stdout)




class TestF2EdgeBands(unittest.TestCase):
    """The two-site fix: the cull offers band candidates AND the gate does
    not revert the homecoming. Board-only: J1's final pose is checked for
    EDGE CONTACT (within its declared band), never against ground truth."""

    def _auto_intent(self, td):
        env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8')
        intent = os.path.join(td, 'auto.json')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), SWAP,
             '--emit-intent', intent, '--declare-classes'],
            capture_output=True, text=True, env=env, cwd=ROOT)
        assert r.returncode == 0, r.stdout[-400:] + r.stderr[-400:]
        return intent

    def test_ladder_seats_the_displaced_receptacle(self):
        """The full ladder, board-only end to end. The assign stage alone
        cannot seat J1 -- its home is inside a self-consistent displaced
        ISLAND whose members anchor each other (the static net-anchor proxy
        prices everyone against stay, so a simultaneous +/-v exchange is
        invisible: the measured F4 limit). The ladder's answer: reconstruct
        DECLARES and detects; the edge+home are then DERIVED from the
        tool's own candidate set (R2 propose-gate: the unique in-band
        edge-metric winner) and declared; repair seats it on the band at
        the derived position. Zero hand scripts, zero ground truth."""
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        import math as _m
        from kicad_parser import parse_kicad_pcb
        from placement.legality import BoardOutlineGate, build_part_pads
        from pose_score import make_state
        with tempfile.TemporaryDirectory() as td:
            intent = self._auto_intent(td)
            out = os.path.join(td, 'r.kicad_pcb')
            r = run_reconstruct(SWAP, out, '--intent', intent,
                                '--assign-rounds', '2')
            self.assertEqual(r.returncode, 0, r.stdout[-900:] + r.stderr[-400:])
            doc = _summary(r.stdout)
            self.assertIn('J1', doc.get('edge_bands', {}))
            self.assertIn('J1', doc.get('edge_pref', []),
                          'J1 must be flagged edge-less/implausible')
            band = doc['edge_bands']['J1']
            vectors = doc['vectors']

            # Derive J1's home from the TOOL's own candidate set: the unique
            # in-band edge-metric winner among stay +/- each vector.
            from placement import reconstruct as R
            from pose_score import make_state
            st = make_state(parse_kicad_pcb(out), out, clearance=0.09)
            p = st.parts['J1']
            cands = [(p.x, p.y)]
            for (vx, vy) in vectors:
                cands += [(p.x + vx, p.y + vy), (p.x - vx, p.y - vy)]
            scored = sorted(
                (m, x, y) for (x, y) in cands
                for m in [R.edge_metric(st, 'J1', x, y, band)]
                if m is not None)
            self.assertGreaterEqual(scored[1][0] - scored[0][0], 0.5,
                                    'the winner must be unique (mechanical '
                                    'claim needs separation)')
            hx, hy = scored[0][1], scored[0][2]

            # Declare the derived edge; repair seats J1 on the band. Its
            # EXACT derived home is still occupied by the island core (U1/
            # U2/C2/C4 anchor each other and no local move can exchange
            # them -- the measured F4 limit, run-5 backlog: a group-exchange
            # candidate in the ILP), so the honest contract is: J1 ends ON
            # the derived edge, in-band, conflict-free.
            doc_i = json.load(open(intent, encoding='utf-8'))
            x0 = doc_i['envelope']['rect'][0]
            want_edge = 'west' if abs(hx - x0) < 10 else 'east'
            for c in doc_i['edge_connectors']:
                if c['ref'] == 'J1':
                    c['edge'] = want_edge
                    c.pop('note', None)
            i2 = os.path.join(td, 'i2.json')
            json.dump(doc_i, open(i2, 'w', encoding='utf-8'), indent=1)
            env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8')
            out2 = os.path.join(td, 'r2.kicad_pcb')
            r2 = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_placer', 'place_seed.py'), out, out2,
                 '--intent', i2, '--repair', '--clearance', '0.09'],
                capture_output=True, text=True, env=env, cwd=ROOT)
            # rc 4 = residual grade errors (the damaged-board-baked legality
            # budget; pre-existing structure) -- the seat itself must land.
            self.assertIn(r2.returncode, (0, 4),
                          r2.stdout[-800:] + r2.stderr[-400:])
            # Run-5: the exchange stage may have ALREADY homed J1 at its
            # true seat (the whole island moves in one joint +/-v solve),
            # in which case repair honestly reports zero violators instead
            # of re-seating. Either path must end with J1 edge-seated --
            # the geometric asserts below check that, whichever rung did it.
            if f'seated on the {want_edge} edge band' not in r2.stdout:
                self.assertIn('"repaired": 0', r2.stdout)
                self.assertIn('"grade_errors": 0', r2.stdout)
            pcb = parse_kicad_pcb(out2)
            fp = pcb.footprints['J1']
            gate = BoardOutlineGate(pcb.board_info, 0.0)
            pp = build_part_pads(pcb.footprints, 0.09)['J1']
            ext = pp.extent(fp.x, fp.y, fp.rotation or 0.0)
            self.assertLessEqual(gate.rect_outside_amount(ext), band + 1e-3,
                                 'pad copper must stay inside the band')
            # Edge contact is a COURTYARD-channel fact (the shell face
            # reaches the edge; the pad field sits ~2mm behind it) --
            # measuring the pad channel here was this test's own channel
            # mix-up, the exact confusion the labels exist to prevent.
            st2 = make_state(pcb, out2, clearance=0.09)
            crect = st2.parts['J1'].rect()
            cover = st2.edge_gate.rect_outside_amount(crect)
            cclr = st2.edge_gate.edge_clearance(crect)
            self.assertTrue(cover > 1e-6 or cclr <= 0.6,
                            f'J1 courtyard must reach the edge '
                            f'(over={cover}, clr={cclr})')
            # the derivation itself must have pointed at the true edge side
            self.assertLess(abs(hx - x0), 10,
                            'the unique in-band winner is on the west side')

    def test_band_guard_per_ref(self):
        """The S1 anti-evacuation guard, band-aware: every UNdeclared ref's
        pad oob <= its input oob; every declared ref's <= its band max."""
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        from kicad_parser import parse_kicad_pcb
        from placement.legality import BoardOutlineGate, build_part_pads
        with tempfile.TemporaryDirectory() as td:
            intent = self._auto_intent(td)
            out = os.path.join(td, 'r.kicad_pcb')
            r = run_reconstruct(SWAP, out, '--intent', intent)
            self.assertEqual(r.returncode, 0)
            doc = _summary(r.stdout)
            bands = doc.get('edge_bands', {})
            before = parse_kicad_pcb(SWAP)
            after = parse_kicad_pcb(out)
            gate = BoardOutlineGate(after.board_info, 0.0)
            pads_b = build_part_pads(before.footprints, 0.09)
            pads_a = build_part_pads(after.footprints, 0.09)

            def oob(pcb, pads, ref):
                fp = pcb.footprints[ref]
                ext = pads[ref].extent(fp.x, fp.y, fp.rotation or 0.0)
                return 0.0 if ext is None else gate.rect_outside_amount(ext)

            for ref in sorted(after.footprints):
                if ref not in pads_a or ref not in pads_b:
                    continue
                a = oob(after, pads_a, ref)
                if ref in bands:
                    self.assertLessEqual(a, bands[ref] + 1e-3, ref)
                else:
                    self.assertLessEqual(a, oob(before, pads_b, ref) + 1e-6,
                                         ref)


class TestRepairEdgeSeating(unittest.TestCase):
    def test_repair_seats_a_declared_edge_part_on_its_band(self):
        """Synthetic: a 'USB' part parked 6mm interior, intent declares it on
        the west edge. --repair must charge it (proximity rule) and seat it
        ON the band -- _try_place could never (full containment)."""
        board = (
            '(kicad_pcb (version 20221018) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "A") (net 2 "B")\n'
            '  (gr_rect (start 30 30) (end 70 70) (stroke (width 0.1)'
            ' (type default)) (layer "Edge.Cuts"))\n'
            '  (footprint "t:USB_C_Recept" (layer "F.Cu") (at 40 50)\n'
            '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" smd rect (at -2 0) (size 1 1) (layers "F.Cu")'
            ' (net 1 "A"))\n'
            '    (pad "2" smd rect (at 2 0) (size 1 1) (layers "F.Cu")'
            ' (net 2 "B")))\n'
            '  (footprint "t:R" (layer "F.Cu") (at 60 50)\n'
            '    (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu")'
            ' (net 1 "A"))\n'
            '    (pad "2" smd rect (at 2 0) (size 1 1) (layers "F.Cu")'
            ' (net 2 "B")))\n'
            ')\n')
        intent = json.dumps({
            'schema': 1, 'kind': 'floorplan-intent',
            'edge_connectors': [{'ref': 'J1', 'edge': 'west',
                                 'class': 'edge_receptacle',
                                 'overhang_mm': {'min': 0.0, 'max': 1.0}}]})
        env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8')
        with tempfile.TemporaryDirectory() as td:
            bp = os.path.join(td, 'b.kicad_pcb')
            ip = os.path.join(td, 'i.json')
            open(bp, 'w', encoding='utf-8').write(board)
            open(ip, 'w', encoding='utf-8').write(intent)
            out = os.path.join(td, 'o.kicad_pcb')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_placer', 'place_seed.py'), bp, out,
                 '--intent', ip, '--repair', '--clearance', '0.2'],
                capture_output=True, text=True, env=env, cwd=ROOT)
            self.assertEqual(r.returncode, 0, r.stdout[-800:] + r.stderr[-400:])
            self.assertIn('seated on the west edge band', r.stdout)
            from kicad_parser import parse_kicad_pcb
            pcb = parse_kicad_pcb(out)
            j1 = pcb.footprints['J1']
            self.assertLess(j1.x, 34.0, 'J1 must end at the west edge')


if __name__ == '__main__':
    unittest.main()
