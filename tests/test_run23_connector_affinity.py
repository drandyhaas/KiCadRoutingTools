"""connector_affinity: generic connectors get a WEAK class (run-23).

Run 23 seated J2/J5/J6/J7 mid-board and no instrument could say so: generic
connector keywords deliberately map to NO edge class (a JST wire-to-board
part is legitimately interior -- that guard stands), and a part with no
class is invisible to every intent rule. The weak class exists to be
DECLARED (--declare-classes) and graded at ADVISORY severity: an INTERIOR
pose (past part_class.INTERIOR_AFFINITY_MM from every edge) is a warning for
the boundary review, never an error, and `pass` never flips on it.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer')):
    sys.path.insert(0, p)

PLACED = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                      'tigard_placed.kicad_pcb')
SPLITFLAP = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _cf(*argv):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1')
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


class TestClass(unittest.TestCase):
    def test_generic_connectors_classify_weak_not_edge(self):
        from kicad_parser import parse_kicad_pcb
        from placement.part_class import (MECHANICAL_CLASSES, classify_part,
                                          pose_plausible)
        pcb = parse_kicad_pcb(PLACED)
        for ref in ('J2', 'J4', 'J5', 'J6', 'J7'):
            cls = classify_part(pcb.footprints[ref], ref)
            self.assertEqual(cls.name, 'connector_affinity', ref)
            self.assertEqual(cls.confidence, 'low', ref)
        # The class makes NO pose claim and is NOT mechanically pinned --
        # both would change reconstruct/stager behavior, which this class
        # must never do.
        self.assertNotIn('connector_affinity', MECHANICAL_CLASSES)
        self.assertTrue(pose_plausible('connector_affinity', 0.0, 99.0))
        # The receptacle guard stands: J1 (USB-C) is still edge_receptacle.
        self.assertEqual(classify_part(pcb.footprints['J1'], 'J1').name,
                         'edge_receptacle')


class TestDeclareAndGrade(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.td = tempfile.TemporaryDirectory()
        cls.intent = os.path.join(cls.td.name, 'intent.json')
        r = _cf(PLACED, '--emit-intent', cls.intent, '--declare-classes')
        assert r.returncode == 0, r.stdout[-400:]
        cls.grade = os.path.join(cls.td.name, 'grade.json')
        cls.gr = _cf(PLACED, '--intent', cls.intent, '--json', cls.grade)

    @classmethod
    def tearDownClass(cls):
        cls.td.cleanup()

    def test_declare_classes_declares_the_connector_family(self):
        doc = json.load(open(self.intent, encoding='utf-8'))
        by_ref = {c['ref']: c for c in doc['edge_connectors']}
        for ref in ('J2', 'J4', 'J5', 'J6', 'J7'):
            self.assertIn(ref, by_ref, 'connector not declared')
            self.assertEqual(by_ref[ref]['class'], 'connector_affinity')
            # NO invented edge (the run-4 rule): the note carries the
            # measured distance instead.
            self.assertNotIn('edge', by_ref[ref])
            self.assertIn('measured', by_ref[ref]['note'])
        self.assertEqual(by_ref['J1']['class'], 'edge_receptacle')

    def test_interior_connector_flags_advisory_pass_survives(self):
        self.assertEqual(self.gr.returncode, 0, self.gr.stdout[-400:])
        doc = json.load(open(self.grade, encoding='utf-8'))
        self.assertTrue(doc['pass'])
        sev = [v['severity'] for v in doc['violations']]
        self.assertNotIn('error', sev)
        # J4 sits 6.03mm interior: flagged, as a WARNING.
        j4 = [v for v in doc['violations'] if v['ref'] == 'J4']
        self.assertEqual([v['severity'] for v in j4], ['warn'])
        self.assertEqual(j4[0]['expected'], {'max_setback_mm': 3.0})
        self.assertIn('J4 is an edge part seated', self.gr.stdout)
        self.assertIn('[warn ]', self.gr.stdout)
        # J7 (2.55mm, inside the affinity band) must NOT be flagged --
        # legitimately-interior connectors are the false-positive guard.
        self.assertNotIn('J7 is an edge part seated', self.gr.stdout)

    def test_healthy_board_gains_no_errors(self):
        with tempfile.TemporaryDirectory() as td:
            intent = os.path.join(td, 'i.json')
            r = _cf(SPLITFLAP, '--emit-intent', intent, '--declare-classes')
            self.assertEqual(r.returncode, 0, r.stdout[-400:])
            g = os.path.join(td, 'g.json')
            r2 = _cf(SPLITFLAP, '--intent', intent, '--json', g)
            self.assertEqual(r2.returncode, 0, r2.stdout[-400:])
            doc = json.load(open(g, encoding='utf-8'))
            self.assertTrue(doc['pass'])
            self.assertNotIn(
                'error', [v['severity'] for v in doc['violations']])


class TestInertToThePlacementEngines(unittest.TestCase):
    """The class DECLARES; it must not reach the placement search.

    `edge_connectors` is one wire key holding two populations. The engines
    act on an edge claim -- place_seed LOCKS those refs for the polish
    quench, place_reconstruct grants each a banded off-outline allowance and
    excludes it from the exchange stage, and reconstruct.classify forces them
    into the anchor tier. Handing them the connector_affinity entries would
    put 6 more tigard refs through all four on the strength of a class
    nobody acted on before. `Intent.edge_claims()` is the split, in one
    place; this pins both halves.
    """

    @classmethod
    def setUpClass(cls):
        from placement import floorplan
        cls.td = tempfile.TemporaryDirectory()
        cls.path = os.path.join(cls.td.name, 'intent.json')
        r = _cf(PLACED, '--emit-intent', cls.path, '--declare-classes')
        assert r.returncode == 0, r.stdout[-400:]
        cls.doc = json.load(open(cls.path, encoding='utf-8'))
        cls.intent = floorplan.load_intent(cls.path)
        # The same intent with the weak class removed -- what upstream main
        # emits, and the control every "inert" claim is measured against.
        stripped = dict(cls.doc)
        stripped['edge_connectors'] = [
            c for c in cls.doc['edge_connectors']
            if c.get('class') != 'connector_affinity']
        cls.stripped_path = os.path.join(cls.td.name, 'stripped.json')
        with open(cls.stripped_path, 'w', encoding='utf-8') as fh:
            json.dump(stripped, fh)
        cls.stripped = floorplan.load_intent(cls.stripped_path)

    @classmethod
    def tearDownClass(cls):
        cls.td.cleanup()

    def test_the_wire_key_carries_both_populations(self):
        """upstream/main's tests/test_part_class.py reads the declaration out
        of doc['edge_connectors'] by ref, so the entries stay there."""
        refs = {c['ref'] for c in self.doc['edge_connectors']}
        self.assertEqual(refs, {'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7'})

    def test_edge_claims_is_exactly_upstream_mains_population(self):
        claimed = {c['ref'] for c in self.intent.edge_claims()}
        self.assertEqual(claimed, {'J1'})
        self.assertEqual(claimed,
                         {c['ref'] for c in self.stripped.edge_claims()})
        self.assertEqual(len(self.intent.edge_claims()),
                         len(self.stripped.edge_connectors))

    def test_edge_claims_is_what_a_band_map_would_be_built_from(self):
        """The shape of the trap, stated once: every consumer's band map
        defaults a missing `max` to 2.0mm, and a connector_affinity entry
        carries `overhang_mm` with only a `min`. The real-call-path proofs
        are in TestSeederDrivesTheSplitForReal below; this pins WHY."""
        bands = {c['ref']: float((c.get('overhang_mm') or {}).get('max') or 2.0)
                 for c in self.intent.edge_claims()}
        self.assertEqual(sorted(bands), ['J1'])
        raw = {c['ref']: float((c.get('overhang_mm') or {}).get('max') or 2.0)
               for c in self.intent.edge_connectors}
        self.assertEqual(raw.get('J7'), 2.0)

    def test_reconstruct_tiers_are_byte_identical_either_way(self):
        """The real function, not a re-derivation: classify() forces edge
        refs into the anchor tier."""
        from kicad_parser import parse_kicad_pcb
        import pose_score
        from placement import reconstruct
        pcb = parse_kicad_pcb(PLACED)
        st = pose_score.make_state(pcb, PLACED, clearance=0.15,
                                   board_edge_clearance=0.55, grid_step=0.1)
        with_cls = reconstruct.classify(st, self.intent)
        without = reconstruct.classify(st, self.stripped)
        self.assertEqual(with_cls.edge, without.edge)
        self.assertEqual(with_cls.edge & {'J7'}, set(),
                         'a connector_affinity ref reached the edge tier')
        self.assertEqual(with_cls.anchors, without.anchors)
        self.assertEqual(with_cls.smalls, without.smalls)

    def test_the_grade_still_reads_the_whole_key(self):
        """Inert to the SEARCH, live to the RULE -- otherwise the class
        declares nothing and J4's interior seat goes back to being
        invisible."""
        from placement import floorplan
        self.assertTrue(floorplan._wants(self.intent, 'edge_connector'))
        only_weak = dict(self.doc)
        only_weak['edge_connectors'] = [
            c for c in self.doc['edge_connectors']
            if c.get('class') == 'connector_affinity']
        p = os.path.join(self.td.name, 'weak.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(only_weak, fh)
        weak = floorplan.load_intent(p)
        self.assertEqual(weak.edge_claims(), ())
        # An intent of nothing BUT weak entries still runs the rule.
        self.assertTrue(floorplan._wants(weak, 'edge_connector'))

    ENGINES = ('py_placer/place_seed.py',
               'py_placer/place_reconstruct.py',
               'py_placer/placement/reconstruct.py',
               # seeder.py is driven straight from place_seed (reseat_scope
               # at :222, repair_placement at :284) and had SIX raw reads,
               # two of them acting on entries as parts. It was missing from
               # this list, which is how they survived the first pass.
               'py_placer/placement/seeder.py')
    # A raw read is allowed only where the entry is being copied, not acted
    # on, and only with this marker on the line. One exists (seeder's
    # intent2 rebuild); a second one has to be argued for.
    EXEMPT = 'edge-claims-exempt'

    def _raw_reads(self, rel):
        import re
        src = open(os.path.join(ROOT, rel), encoding='utf-8').read()
        pat = r'^(?!\s*#).*[.]edge_connectors[^_a-zA-Z0-9].*$'
        return [m.group(0).strip() for m in re.finditer(pat, src, re.M)]

    def test_no_engine_reads_the_raw_key(self):
        """A filter that must be remembered is a filter that gets forgotten.
        Every engine read goes through edge_claims(); floorplan.py owns the
        raw key (the rule and the applicability test read it on purpose)."""
        waived = []
        for rel in self.ENGINES:
            for line in self._raw_reads(rel):
                if self.EXEMPT in line:
                    waived.append((rel, line))
                    continue
                self.fail(f'{rel} reads intent.edge_connectors directly -- '
                          f'use edge_claims(): {line}')
        # Exactly one exemption in the tree, and it is the one argued for in
        # seeder.reseat_scope. A new one is a decision, not a default.
        self.assertEqual(len(waived), 1, str(waived))
        self.assertEqual(waived[0][0], 'py_placer/placement/seeder.py')

    def test_the_guard_actually_looks_at_seeder(self):
        """The guard is the only mechanism protecting the converted call
        sites, so its FILE LIST has to be pinned: it passed the first review
        while missing the engine carrying the regression."""
        self.assertIn('py_placer/placement/seeder.py', self.ENGINES)
        # ...and the regex really does find reads in that file (the one
        # exempted copy), so a silently-empty scan cannot look like a pass.
        self.assertTrue(self._raw_reads('py_placer/placement/seeder.py'))


class TestSeederDrivesTheSplitForReal(unittest.TestCase):
    """The engines `place_seed` actually calls, called.

    The first pass at this fix converted four call sites and pinned them with
    comprehensions re-derived in the test, which proved nothing about the
    code: reverting `place_seed.py:429` alone left every behavioural test
    green. Worse, it MISSED `placement/seeder.py` -- the engine
    `place_seed.py` drives at :222 (`reseat_scope`) and :284
    (`repair_placement`) -- where the raw key was doing real damage.

    Every test here calls the engine entry point and compares the FULL
    intent against the same intent with the connector_affinity entries
    stripped, which is exactly what upstream main emits. Equal is the whole
    claim: the declaration must change nothing.
    """

    # The fixture is OWNED here, and small on purpose: the same proof on
    # tests/fixtures/run23/tigard_damaged.kicad_pcb costs 160 s per
    # repair_placement call and this class makes four. One connector_affinity
    # part pushed off the south edge of the tracked splitflap board makes it
    # the sole violator, which is all the dispatch below needs.
    DY = 55.5
    # A ref that is NOT the damaged one, for the gate-band test: `gate_bands`
    # is `edge_bands` minus the scope, so a band is only visible to the gate
    # when its owner sits OUTSIDE the scope.
    SPARE = 'R8'

    @classmethod
    def setUpClass(cls):
        for p in (ROOT, os.path.join(ROOT, 'py_router'),
                  os.path.join(ROOT, 'py_placer'),
                  os.path.join(ROOT, 'py_tools')):
            if p not in sys.path:
                sys.path.insert(0, p)
        from kicad_parser import parse_kicad_pcb
        from placement import floorplan
        from placement.writer import write_placed_output
        cls.td = tempfile.TemporaryDirectory()
        pcb = parse_kicad_pcb(SPLITFLAP)
        cls.doc = floorplan.emit_intent(pcb, SPLITFLAP, declare_classes=True)
        cls.weak = sorted(c['ref'] for c in cls.doc['edge_connectors']
                          if c.get('class') == 'connector_affinity')
        assert cls.weak, 'fixture declares no connector_affinity part'
        cls.victim = cls.weak[0]
        cls.spare = cls.SPARE

        fp = pcb.footprints[cls.victim]
        cls.board = os.path.join(cls.td.name, 'damaged.kicad_pcb')
        write_placed_output(SPLITFLAP, cls.board, [
            {'reference': cls.victim, 'new_x': fp.x, 'new_y': cls.DY,
             'new_rotation': fp.rotation or 0.0}])

        full = os.path.join(cls.td.name, 'full.json')
        with open(full, 'w', encoding='utf-8') as fh:
            json.dump(cls.doc, fh)
        cls.full = floorplan.load_intent(full)
        stripped = dict(cls.doc)
        stripped['edge_connectors'] = [
            c for c in cls.doc['edge_connectors']
            if c.get('class') != 'connector_affinity']
        sp = os.path.join(cls.td.name, 'stripped.json')
        with open(sp, 'w', encoding='utf-8') as fh:
            json.dump(stripped, fh)
        cls.stripped = floorplan.load_intent(sp)

        cls.rep_full = cls._repair(cls.full)
        cls.rep_strip = cls._repair(cls.stripped)

    @classmethod
    def tearDownClass(cls):
        cls.td.cleanup()

    @classmethod
    def _repair(cls, intent):
        from kicad_parser import parse_kicad_pcb
        from placement import seeder
        return seeder.repair_placement(
            parse_kicad_pcb(cls.board), cls.board, intent, group_sources=(),
            clearance=0.15, board_edge_clearance=0.55, grid_step=0.1)

    def test_the_weak_violator_reaches_the_ordinary_repair_loop(self):
        """THE REGRESSION. `repair_placement` dispatches every entry in its
        edge map that carries no `edge` into a refusal BEFORE the ordinary
        _try_place loop, so reading the raw key took the connector_affinity
        declarations -- which never carry an edge, by design -- out of repair
        entirely, and mislabelled them 'declared edge part' on the way out.
        Measured on tigard_damaged: J5, J6 and J7 refused that way, where
        upstream main tries them and reports 'no legal pose within any cap'."""
        rep = self.rep_full
        self.assertIn(self.victim, rep['violators'],
                      'the fixture no longer makes the weak part a violator, '
                      'so this test proves nothing')
        bad = [n for n in rep['notes']
               if 'declared edge part misplaced' in n
               and any(n.startswith(w + ':') for w in self.weak)]
        self.assertEqual(bad, [], 'a connector_affinity part was refused as a '
                                  'declared edge part instead of repaired')
        mine = [n for n in rep['notes'] if n.startswith(self.victim + ':')]
        self.assertTrue(mine, 'the weak violator produced no note at all, so '
                              'it never reached the loop')
        # Whatever the loop decides is fine -- what matters is that the loop
        # decided it. These are the verdicts only the ordinary path emits.
        self.assertTrue(
            any(k in mine[0] for k in ('re-seated', 'no legal pose',
                                       'disproportionate', 'seated its pair')),
            mine[0])

    def test_repair_is_identical_with_and_without_the_declaration(self):
        """The strongest form: same engine, same board, two intents that
        differ only by the weak entries. Every output must match."""
        a, b = self.rep_full, self.rep_strip
        self.assertEqual(sorted(a['violators']), sorted(b['violators']))
        self.assertEqual(sorted(a['repaired']), sorted(b['repaired']))
        self.assertEqual(sorted(a['unrepairable']), sorted(b['unrepairable']))
        self.assertEqual(sorted(n['reference'] for n in a['moves']),
                         sorted(n['reference'] for n in b['moves']))
        self.assertEqual(sorted(a['notes']), sorted(b['notes']))

    def _reseat(self, intent, refs):
        from kicad_parser import parse_kicad_pcb
        from placement import seeder
        return seeder.reseat_scope(
            parse_kicad_pcb(self.board), self.board, intent, refs=refs,
            group_sources=(), clearance=0.15, board_edge_clearance=0.55,
            grid_step=0.1, seed=0)

    def test_reseat_gate_bands_are_identical_either_way(self):
        """`reseat_scope` builds {ref: band} from the entries and defaults a
        missing `max` to 2.0mm -- an off-outline licence charged in the gate
        tuple. The tuples it reports are measured against that map, so equal
        tuples is the proof the weak entries bought no band.

        THE SCOPE MUST EXCLUDE THE OFF-BOARD WEAK PART, and that is the whole
        trick of this test rather than an accident: `gate_bands` is
        `edge_bands` minus the scope, so a scope containing the damaged part
        drops its band before the gate ever sees it and the raw key looks
        innocent. Measured with the raw key restored and the victim OUT of
        scope, the oob term reads 0.882 against 1.902 -- 1.02mm of off-board
        copper forgiven on the strength of a declaration nobody made. With
        the victim IN scope both read 1.902 and the mutation survives."""
        refs = [self.spare]
        self.assertNotIn(self.victim, refs)
        a = self._reseat(self.full, refs)
        b = self._reseat(self.stripped, refs)
        # Ask the module for the column rather than hardcoding 3: a new
        # gate term inserted before 'oob' would move it, and this
        # assertion would then quietly measure something else.
        from placement import reconstruct as _recon_mod
        _oob = _recon_mod.GATE_TERMS.index('oob')
        self.assertGreater(a['gate_before'][_oob], 0.0,
                           'the fixture charges no off-board amount, so this '
                           'test cannot see a band either way')
        self.assertEqual(a['gate_before'], b['gate_before'])
        self.assertEqual(a['gate_after'], b['gate_after'])
        self.assertEqual(a['accepted'], b['accepted'])
        self.assertEqual(a['reseated'], b['reseated'])
        self.assertEqual(sorted(a['notes']), sorted(b['notes']))

    def test_reseat_drops_no_declaration_it_never_made(self):
        """A weak entry has no band to forfeit, so it must not be reported
        as a dropped edge declaration -- and it must survive into the
        sub-intent the pass seeds from (the one raw read left in seeder.py,
        which exists so intent2 stays a faithful copy)."""
        a = self._reseat(self.full, [self.victim])
        self.assertNotIn(self.victim, a['edge_bands_dropped'])
        kept = {c['ref'] for c in a['intent_used'].edge_connectors}
        self.assertTrue(set(self.weak) <= kept,
                        'the sub-intent lost the class declarations')
        # ...and the sub-intent's CLAIMS are still only the real ones.
        self.assertNotIn(self.victim,
                         {c['ref'] for c in a['intent_used'].edge_claims()})


class TestPlaceSeedLocksOnlyClaims(unittest.TestCase):
    """place_seed:429 passes `edge_refs` to the polish quench as `lock_refs`.

    Driven through `place_seed.main()` with the real argv, capturing the
    kwargs the quench is actually called with. A comprehension re-derived in
    the test would pass with the line reverted, which is how this one got
    through the first review.
    """

    def test_lock_refs_are_the_edge_claims(self):
        for p in (ROOT, os.path.join(ROOT, 'py_router'),
                  os.path.join(ROOT, 'py_placer'),
                  os.path.join(ROOT, 'py_tools')):
            if p not in sys.path:
                sys.path.insert(0, p)
        from kicad_parser import parse_kicad_pcb
        from placement import quench as quench_mod
        from placement.floorplan import emit_intent
        from placement.writer import write_placed_output
        import place_seed

        board = SPLITFLAP
        pcb = parse_kicad_pcb(board)
        with tempfile.TemporaryDirectory() as td:
            pile = os.path.join(td, 'pile.kicad_pcb')
            write_placed_output(board, pile, [
                {'reference': r, 'new_x': 10.0, 'new_y': 10.0,
                 'new_rotation': 0.0} for r in sorted(pcb.footprints)])
            ip = os.path.join(td, 'i.json')
            with open(ip, 'w', encoding='utf-8') as fh:
                json.dump(emit_intent(pcb, board, declare_classes=True), fh)
            doc = json.load(open(ip, encoding='utf-8'))
            weak = {c['ref'] for c in doc['edge_connectors']
                    if c.get('class') == 'connector_affinity'}
            claims = {c['ref'] for c in doc['edge_connectors']
                      if c.get('class') != 'connector_affinity'}
            self.assertTrue(weak and claims, str(doc['edge_connectors'])[:200])

            seen = {}

            class _Stop(RuntimeError):
                pass

            def _spy(*a, **kw):
                # BOTH channels. #702 moved the edge claims out of the
                # `lock_refs=` kwarg and into the resolved intent bundle, which
                # `quench()` unions into the same frozen set -- the refs are
                # unchanged, the parameter carrying them is not.
                #
                # Reading only `lock_refs` does not FAIL loudly when that
                # happens: `kw.get` stores None, so `assertIn('lock_refs',
                # seen)` still passes and `got & weak == set()` passes
                # VACUOUSLY on an empty set. The connector_affinity guard --
                # the entire reason this test exists -- would go on reporting
                # green while asserting nothing. Hence the union, and hence
                # the assertion below that it is non-empty.
                seen['lock_refs'] = kw.get('lock_refs')
                seen['intent_lock_refs'] = (
                    (kw.get('intent_gate') or {}).get('lock_refs'))
                raise _Stop()

            real, quench_mod.quench = quench_mod.quench, _spy
            argv = sys.argv
            sys.argv = ['place_seed.py', pile,
                        os.path.join(td, 'out.kicad_pcb'), '--intent', ip]
            try:
                try:
                    place_seed.main()
                except _Stop:
                    pass
            finally:
                quench_mod.quench = real
                sys.argv = argv

        self.assertIn('lock_refs', seen, 'the polish quench was never reached')
        got = set(seen['lock_refs'] or ()) | set(seen['intent_lock_refs'] or ())
        # Non-empty FIRST. The two assertions below are both satisfied by an
        # empty set, so without this the guard passes when the polish stops
        # being told about the edge claims at all -- which is exactly how a
        # channel move would slip through unnoticed.
        self.assertTrue(got, 'the polish was handed NO locked refs by either '
                             'channel: the connector_affinity guard below '
                             'would pass vacuously')
        self.assertEqual(got & weak, set(),
                         f'connector_affinity refs locked in the polish: '
                         f'{sorted(got & weak)}')
        self.assertEqual(got, claims)


if __name__ == '__main__':
    unittest.main(verbosity=1)
