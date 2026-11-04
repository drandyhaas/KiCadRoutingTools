"""#1174: an emitted intent grades clean on the board it was emitted from.

`emit_intent` promises "The emitted intent grades CLEAN by construction", and
two as-built boards broke it, each for its own reason:

  * ROUNDING. The zone was rounded to 3 dp after being clamped to the board,
    the envelope to 4 dp, and `validate_intent` tests containment at tol 0:
    an outline coordinate whose 4th decimal rounds OUTWARD put the zone 0.4 um
    outside the envelope. Now the zone is clamped into the envelope AS WRITTEN
    (`floorplan.emitted_zone_rect`).
  * THE EDGE MEMBER. rp2350's J1 (USB-C, north) passes the edge rule with its
    body 1.43 mm past the edge, and its block's zone -- clamped to the outline
    -- then failed it by 1.93 mm. An edge-claim member's declared edge is the
    edge rule's to grade, so when the zone side lies ON the outline it does
    not bound that member there (`floorplan.zone_open_sides`). The quench term
    reads the same open side from the zone row, so the gate and the grade
    agree.
"""

import contextlib
import copy
import io
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan                              # noqa: E402
from placement.edge_seat import seat_state                   # noqa: E402
from run_utils import check                                  # noqa: E402

PY = sys.executable
RP = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
ZONE = 'sheet:558c3023'          # J1's block; its north side is the outline


def _emit(path):
    return floorplan.emit_intent(parse_kicad_pcb(path), path,
                                 group_sources=('kicad', 'sheet'))


def _grade(doc, path):
    it = floorplan.intent_from_dict(doc, source_path='emitted.json')
    return floorplan.grade(it, parse_kicad_pcb(path), path,
                           group_sources=('kicad', 'sheet'))


def _zone_errors(r, ref='J1'):
    return [v for v in r.errors
            if v.rule == 'zone_containment' and v.ref == ref]


class Rounding(unittest.TestCase):

    def test_a_zone_never_leaves_the_envelope_as_written(self):
        # x-min 75.0504 rounds OUTWARD to 75.05; x-max 136.2499 to 136.25.
        bounds = (75.0504, 10.0, 136.2499, 50.0)
        z = floorplan.emitted_zone_rect((75.0504, 12.0, 136.2499, 40.0),
                                        bounds)
        env = [round(v, 4) for v in bounds]
        self.assertTrue(floorplan._rect_contains(env, z, tol=0.0), (z, env))
        self.assertEqual(z, [75.0504, 12.0, 136.2499, 40.0])

    def test_an_interior_zone_is_only_rounded(self):
        z = floorplan.emitted_zone_rect((80.12345, 20.5, 90.0, 30.98765),
                                        (75.0, 10.0, 136.0, 50.0))
        self.assertEqual(z, [80.123, 20.5, 90.0, 30.988])

    def test_the_issue_board_round_trips_through_the_cli(self):
        """The issue's own repro: rp2350 with its top edge moved by 0.4 um."""
        with tempfile.TemporaryDirectory() as td:
            with open(RP, encoding='utf-8') as fh:
                text = fh.read()
            self.assertIn('86.971)', text)
            b = os.path.join(td, 'r.kicad_pcb')
            with open(b, 'w', encoding='utf-8') as fh:
                fh.write(text.replace('86.971)', '86.9714)'))
            i = os.path.join(td, 'ri.json')
            check([PY, '-X', 'utf8', 'py_tools/check_floorplan.py', b,
                   '--emit-intent', i], accept=True)
            with open(i, encoding='utf-8') as fh:
                doc = json.load(fh)
            env = doc['envelope']['rect']
            for blk in doc['blocks']:
                if blk.get('zone'):
                    self.assertTrue(floorplan._rect_contains(
                        env, blk['zone'], tol=0.0), (blk['name'], env))
            check([PY, '-X', 'utf8', 'py_tools/check_floorplan.py', b,
                   '--intent', i], accept=True)


class TheEdgeMember(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.doc = _emit(RP)
        cls.claim = next(c for c in cls.doc['edge_connectors']
                         if c['ref'] == 'J1')
        cls.block = next(b for b in cls.doc['blocks'] if b['name'] == ZONE)

    def _variant(self, fn):
        doc = copy.deepcopy(self.doc)
        fn(doc)
        return _grade(doc, RP)

    def test_the_emitted_intent_grades_clean(self):
        self.assertEqual(self.claim['edge'], 'north')
        self.assertIn('J1', self.block['refs'])
        r = _grade(self.doc, RP)
        self.assertEqual([v.message for v in r.errors], [])

    def test_a_member_without_an_edge_claim_is_still_flagged(self):
        def drop(doc):
            doc['edge_connectors'] = [c for c in doc['edge_connectors']
                                      if c['ref'] != 'J1']
        errs = _zone_errors(self._variant(drop))
        self.assertEqual(len(errs), 1)
        self.assertEqual(errs[0].measured['axis'], 'north')
        self.assertEqual(errs[0].measured['open_sides'], [])

    def test_a_connector_affinity_entry_claims_no_edge(self):
        def demote(doc):
            for c in doc['edge_connectors']:
                if c['ref'] == 'J1':
                    c['class'] = 'connector_affinity'
        self.assertEqual(len(_zone_errors(self._variant(demote))), 1)

    def test_a_claim_on_another_edge_opens_nothing(self):
        def west(doc):
            for c in doc['edge_connectors']:
                if c['ref'] == 'J1':
                    c['edge'] = 'west'
        r = self._variant(west)
        self.assertEqual(len(_zone_errors(r)), 1)

    def test_a_zone_side_inside_the_board_stays_closed(self):
        def shrink(doc):
            for b in doc['blocks']:
                if b['name'] == ZONE:
                    b['zone'][1] = round(b['zone'][1] + 1.0, 4)
        errs = _zone_errors(self._variant(shrink))
        self.assertEqual(len(errs), 1)
        self.assertEqual(errs[0].measured['open_sides'], [])

    def test_the_other_sides_still_bound_the_member(self):
        def narrow(doc):
            for b in doc['blocks']:
                if b['name'] == ZONE:
                    b['zone'][2] = round(b['zone'][0] + 3.0, 4)
        errs = _zone_errors(self._variant(narrow))
        self.assertEqual([e.measured['axis'] for e in errs], ['east'])
        self.assertEqual(errs[0].measured['open_sides'], ['north'])

    def test_an_anchor_graded_member_pushed_off_its_pads_is_named(self):
        """A zone smaller than the courtyard grades the courtyard CENTRE
        (`zone_fits_courtyard`). J1 moved north until its centre is 0.91 mm
        past the edge, in a 5 x 5 zone: its pads are then 3.32 mm off the
        board, so the open side does not excuse it -- the pads are named
        (the synthetic twin in EveryReaderAgrees is the legitimate case)."""
        with tempfile.TemporaryDirectory() as td:
            with open(RP, encoding='utf-8') as fh:
                text = fh.read()
            self.assertEqual(text.count('(at 148.5 89.19 180)'), 1)
            b = os.path.join(td, 'north.kicad_pcb')
            with open(b, 'w', encoding='utf-8') as fh:
                fh.write(text.replace('(at 148.5 89.19 180)',
                                      '(at 148.5 85.5 180)'))
            north = self.block['zone'][1]

            def small(doc, claim=True):
                for blk in doc['blocks']:
                    if blk['name'] == ZONE:
                        blk['zone'] = [146.0, north, 151.0, north + 5.0]
                if not claim:
                    doc['edge_connectors'] = [
                        c for c in doc['edge_connectors'] if c['ref'] != 'J1']
            doc = copy.deepcopy(self.doc)
            small(doc)
            errs = _zone_errors(_grade(doc, b))
            self.assertEqual(len(errs), 1, errs)
            self.assertTrue(errs[0].measured['pads_graded'])
            self.assertAlmostEqual(errs[0].measured['outside_mm'], 3.32,
                                   places=2)
            doc = copy.deepcopy(self.doc)
            small(doc, claim=False)
            errs = _zone_errors(_grade(doc, b))
            self.assertEqual(len(errs), 1)
            self.assertTrue(errs[0].measured['anchor_graded'])
            self.assertEqual(errs[0].measured['axis'], 'north')


class OpenSides(unittest.TestCase):

    B = (0.0, 0.0, 50.0, 40.0)

    def test_only_a_declared_edge_on_the_outline_opens(self):
        f = floorplan.zone_open_sides
        self.assertEqual(f((0, 0, 10, 10), self.B, ('north',)), ('north',))
        self.assertEqual(f((0, 0, 10, 10), self.B, ('north', 'west')),
                         ('west', 'north'))
        self.assertEqual(f((0, 0.5, 10, 10), self.B, ('north',)), ())
        self.assertEqual(f((0, 0, 10, 10), self.B, ('east',)), ())
        self.assertEqual(f((0, 0, 10, 10), self.B, ()), ())
        self.assertEqual(f((0, 0, 10, 10), None, ('north',)), ())
        # Within the 0.01 mm span tolerance, and beyond the outline.
        self.assertEqual(f((0, 0.009, 10, 10), self.B, ('north',)),
                         ('north',))
        self.assertEqual(f((0, -2, 10, 10), self.B, ('north',)), ('north',))

    def test_zone_escape_skips_only_the_open_side(self):
        z, part = (0, 0, 10, 10), (2, -3, 8, 12)
        self.assertEqual(floorplan.zone_escape(z, part, False), (3, 'north'))
        self.assertEqual(floorplan.zone_escape(z, part, False, ('north',)),
                         (2, 'south'))
        self.assertEqual(floorplan.zone_escape(z, part, False,
                                               ('north', 'south')), (0.0, ''))


class EveryReaderOpensTheSameSide(unittest.TestCase):
    """#799's feasibility kernel and the plan check read the grade's open
    side, so neither refuses what the grade passes."""

    def test_feasibility_admits_a_pose_past_the_open_side(self):
        # A 4x4 part in a 10x10 zone; a keep-out leaves only the strip y < 1.
        # Closed, no pose clears it; open north, the part sits past the edge.
        part = floorplan._LocalPart(0.0, (-2.0, -2.0, 2.0, 2.0))
        ko = [{'name': 'K', 'rect': (-50.0, 1.0, 60.0, 60.0),
               'sides': ('F', 'B'), 'allow': ()}]
        closed = floorplan.zone_pose_feasibility((0, 0, 10, 10), 0.0, part,
                                                 ko)
        self.assertFalse(closed['feasible'])
        opened = floorplan.zone_pose_feasibility((0, 0, 10, 10), 0.0, part,
                                                 ko, open_sides=('north',))
        self.assertTrue(opened['feasible'], opened)
        self.assertLessEqual(opened['witness'][1] + 2.0, 1.0 + 1e-6)

    def test_the_plan_check_does_not_refuse_a_locked_edge_member(self):
        """Row 4 of `plan_check` is the grade's rule restricted to FILE-locked
        members, so it inherits the open side. (Row 4b, a fixed pose outside
        its zone, cannot meet an edge member: the loader refuses a fixed pose
        on an `edge_connectors` part.)"""
        doc = _emit(RP)
        it = floorplan.intent_from_dict(doc, source_path='emitted.json')
        with tempfile.TemporaryDirectory() as td:
            b = os.path.join(td, 'locked.kicad_pcb')
            check([PY, '-X', 'utf8', 'py_placer/place_pose.py', RP, b,
                   '--allow-routed', 'lock', 'J1'], accept=True)
            viols, _m = floorplan.plan_check(it, parse_kicad_pcb(b), b,
                                             group_sources=('kicad', 'sheet'))
        self.assertEqual([v.message for v in viols
                          if v.rule == 'plan_fixed_outside_zone'], [])



class TheGateAgrees(unittest.TestCase):
    """The quench term reads the grade's open side: one measurement. The
    zone rows are `floorplan.zone_entries`, the ones place_simple's state is
    built on (`edge_seat.seat_state`)."""

    def test_the_zone_term_equals_the_grade_on_a_lattice(self):
        doc = _emit(RP)
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, 'i.json')
            with open(p, 'w', encoding='utf-8') as fh:
                json.dump(doc, fh)
            intent = floorplan.load_intent(p)
        pcb = parse_kicad_pcb(RP)
        blocks, _ = floorplan.resolve_blocks(intent, pcb, ('kicad', 'sheet'))
        zones = floorplan.zone_entries(intent, blocks,
                                       bounds=pcb.board_info.board_bounds)
        row = next(z for z in zones if z['name'] == ZONE)
        self.assertEqual(row['open_sides'].get('J1'), ('north',))
        with contextlib.redirect_stdout(io.StringIO()):
            st = seat_state(pcb, RP, intent, blocks, 0.25, 0.55)
        spec = [t for t in st.intent_spec_for('J1')
                if t.rule == 'zone_containment']
        self.assertEqual(len(spec), 1)
        self.assertEqual(spec[0].open_sides, ('north',))
        part = st.parts['J1']
        bounds = pcb.board_info.board_bounds
        for dx in (-4.0, -1.0, 0.0, 1.0, 4.0):
            for dy in (-3.0, -1.0, 0.0, 1.5, 3.0, 8.0):
                x, y = part.x + dx, part.y + dy
                rects = part.grade_rects(x, y, part.rot)
                term = st.intent_terms('J1', rects)[
                    list(st.intent_spec_for('J1')).index(spec[0])]
                opened = floorplan.zone_open_sides(
                    row['rect'], bounds, ('north',))
                want = floorplan.zone_escape(row['rect'], rects[0],
                                             spec[0].anchor, opened)[0]
                self.assertAlmostEqual(term, want, places=9, msg=(dx, dy))


def _synthetic(td, parts, size=40):
    """A size x size board with the given footprint texts."""
    from test_1184_container_classification import _board
    return _board(td, ''.join(parts), size=(size, size))


def _box(ref, x, y, hw, hh, name='t:BOX', pad=(0, 0), pad_extra=''):
    """A part with a +-hw x +-hh courtyard and one pad at `pad` (local).
    `pad_extra` is appended inside the pad, e.g. a castellation property."""
    return (f'  (footprint "{name}" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            f'    (fp_rect (start {-hw} {-hh}) (end {hw} {hh}) (stroke'
            f' (width 0.05) (type default)) (layer "F.CrtYd"))\n'
            f'    (pad "1" smd rect (at {pad[0]} {pad[1]}) (size 0.6 0.6)'
            f' (layers "F.Cu") (net 2 "N2"){pad_extra}))\n')


def _doc(blocks, claims=(), **extra):
    doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
           'units': 'mm', 'blocks': list(blocks),
           'edge_connectors': list(claims)}
    doc.update(extra)
    return floorplan.intent_from_dict(doc, source_path='t.json')


J1_NORTH = {'ref': 'J1', 'edge': 'north', 'overhang_mm': {'min': 0, 'max': 5}}


class EveryReaderAgrees(unittest.TestCase):
    """The phase-1 verifier's counterexamples, each with its control."""

    def _grade(self, path, intent):
        return floorplan.grade(intent, parse_kicad_pcb(path), path)

    def test_a_claim_without_a_band_max_opens_nothing(self):
        """The edge rule bounds the overhang only through the band's max; a
        claim without one would let a part sit wholly off the board."""
        with tempfile.TemporaryDirectory() as td:
            path = _synthetic(td, [_box('J1', 10, 1, 2, 2)])
            zone = [{'name': 'b', 'refs': ['J1'], 'zone': [4, 0, 16, 8],
                     'tolerance_mm': 0}]
            bounded = self._grade(path, _doc(zone, [J1_NORTH]))
            unbounded = self._grade(path, _doc(zone, [
                {'ref': 'J1', 'edge': 'north'}]))
        self.assertEqual(_zone_errors(bounded), [])
        self.assertEqual(len(_zone_errors(unbounded)), 1)

    def test_an_open_side_does_not_let_the_pads_leave(self):
        """Second verifier: the band's max bounds the body's far side only,
        so a body-less 4 mm part wholly past the edge (its pad too) passed
        both rules with `max 5`. Past an open side the PADS stay graded."""
        with tempfile.TemporaryDirectory() as td:
            gone = _synthetic(td, [_box('J1', 10, -2.1, 2, 2)])
            zone = [{'name': 'b', 'refs': ['J1'], 'zone': [4, 0, 16, 8],
                     'tolerance_mm': 0}]
            errs = _zone_errors(self._grade(gone, _doc(zone, [J1_NORTH])))
        self.assertEqual(len(errs), 1, errs)
        self.assertIn("pads reach", errs[0].message)
        self.assertTrue(errs[0].measured['pads_graded'])
        self.assertEqual(errs[0].measured['axis'], 'north')
        # The pad box past the side, not the courtyard: 2.1 + the 0.3 half-pad.
        self.assertAlmostEqual(errs[0].measured['outside_mm'], 2.4, places=3)

    def test_pads_past_an_open_side_are_graded_at_zero(self):
        """The zone's tolerance is slack on where a part sits in its zone,
        not on its pads leaving the board: a pad 0.3 mm past the open north
        side is an error under the DEFAULT tolerance (0.5 mm), which it used
        to pass. Control: the same pad flush with the edge."""
        zone = [{'name': 'b', 'refs': ['J1'], 'zone': [4, 0, 16, 8]}]
        self.assertGreater(_doc(zone).zone_tolerance(_doc(zone).blocks[0]),
                           0.3)
        with tempfile.TemporaryDirectory() as td:
            # The 0.6 mm pad centred on y = 0: its copper reaches y = -0.3.
            past = _synthetic(td, [_box('J1', 10, 0, 2, 2)])
            errs = _zone_errors(self._grade(past, _doc(zone, [J1_NORTH])))
        self.assertEqual(len(errs), 1, errs)
        self.assertTrue(errs[0].measured['pads_graded'])
        self.assertEqual(errs[0].measured['axis'], 'north')
        self.assertAlmostEqual(errs[0].measured['outside_mm'], 0.3, places=3)
        self.assertEqual(errs[0].expected['tolerance_mm'], 0.0)
        with tempfile.TemporaryDirectory() as td:
            flush = _synthetic(td, [_box('J1', 10, 0.3, 2, 2)])
            self.assertEqual(_zone_errors(self._grade(
                flush, _doc(zone, [J1_NORTH]))), [])

    def test_a_castellated_pad_past_an_open_side_is_not_graded(self):
        """A castellated pad straddles the outline by design -- the edge
        rule's copper conjunct skips it too -- so the same 0.3 mm reads
        clean when the pad is castellated."""
        zone = [{'name': 'b', 'refs': ['J1'], 'zone': [4, 0, 16, 8]}]
        with tempfile.TemporaryDirectory() as td:
            path = _synthetic(td, [_box(
                'J1', 10, 0, 2, 2,
                pad_extra=' (property pad_prop_castellated)')])
            pcb = parse_kicad_pcb(path)
            self.assertTrue(pcb.footprints['J1'].pads[0].castellated)
            errs = _zone_errors(self._grade(path, _doc(zone, [J1_NORTH])))
        self.assertEqual(errs, [])

    def test_an_anchor_graded_member_opens_the_same_side(self):
        """A zone smaller than the courtyard grades the courtyard CENTRE
        (`zone_fits_courtyard`); the open side applies there too. The part's
        centre sits 1 mm past the north edge while its pad stays 0.5 mm
        inside it -- an overhang, not a part leaving the board."""
        with tempfile.TemporaryDirectory() as td:
            path = _synthetic(td, [_box('J1', 10, -1, 2, 2, pad=(0, 1.5))])
            zone = [{'name': 'b', 'refs': ['J1'], 'zone': [9, 0, 11, 2],
                     'tolerance_mm': 0}]
            opened = self._grade(path, _doc(zone, [J1_NORTH]))
            closed = self._grade(path, _doc(zone, []))
        self.assertEqual(_zone_errors(opened), [])
        errs = _zone_errors(closed)
        self.assertEqual(len(errs), 1, errs)
        self.assertTrue(errs[0].measured['anchor_graded'])
        self.assertEqual(errs[0].measured['axis'], 'north')

    def test_plan_row5_does_not_charge_an_open_side_member(self):
        with tempfile.TemporaryDirectory() as td:
            path = _synthetic(td, [_box('J1', 10, 1, 5, 2.5),
                                   _box('X1', 10, 6, 5, 2.5)])
            zone = [{'name': 'b', 'refs': ['J1', 'X1'],
                     'zone': [5, 0, 15, 8.5], 'tolerance_mm': 0}]
            budget = {'overlap_area': 0.0, 'oob_count': 1}
            pcb = parse_kicad_pcb(path)

            def overfull(claims):
                v, _m = floorplan.plan_check(
                    _doc(zone, claims, legality_budget=budget), pcb, path)
                return [x for x in v if x.rule == 'plan_zone_overfull'
                        and x.severity == 'error']
            self.assertEqual(overfull([J1_NORTH]), [])
            self.assertTrue(overfull([]))          # control: J1 charged

    def test_feasibility_refusals_read_the_open_side(self):
        """#799 per member: the keep-out leaves only the strip past the
        zone's north side, which an edge member may use."""
        with tempfile.TemporaryDirectory() as td:
            path = _synthetic(td, [_box('J1', 10, 1, 2, 2)])
            zone = [{'name': 'b', 'refs': ['J1'], 'zone': [5, 0, 15, 10],
                     'tolerance_mm': 0}]
            ko = [{'name': 'K', 'rect': [0, 2, 40, 40]}]
            pcb = parse_kicad_pcb(path)

            def refused(claims):
                it = _doc(zone, claims, keepouts=ko)
                blocks, _p = floorplan.resolve_blocks(it, pcb, ())
                return [v for v in floorplan.intent_zone_keepout_problems(
                    it, blocks, pcb, path)
                    if v.rule == 'intent_zone_in_keepout']
            self.assertEqual(refused([J1_NORTH]), [])
            self.assertTrue(refused([]))            # control

    def test_total_coverage_reads_the_open_side(self):
        """#702: a keep-out equal to the zone -- an edge member past the
        open side is in neither."""
        with tempfile.TemporaryDirectory() as td:
            path = _synthetic(td, [_box('J1', 10, -2.6, 2, 2)])
            zone = [{'name': 'b', 'refs': ['J1'], 'zone': [4, 0, 16, 8],
                     'tolerance_mm': 0}]
            ko = [{'name': 'K', 'rect': [4, 0, 16, 8]}]
            pcb = parse_kicad_pcb(path)

            def refused(claims):
                it = _doc(zone, claims, keepouts=ko)
                blocks, _p = floorplan.resolve_blocks(it, pcb, ())
                return [v for v in floorplan.intent_zone_keepout_problems(
                    it, blocks, pcb, path)
                    if v.rule == 'intent_zone_in_keepout']
            self.assertEqual(refused([J1_NORTH]), [])
            self.assertTrue(refused([]))            # control

    def test_the_exclusive_plan_check_reads_the_open_side(self):
        with tempfile.TemporaryDirectory() as td:
            path = _synthetic(td, [_box('J1', 10, 1, 2, 2),
                                   _box('X2', 20, 20, 1, 1)])
            blocks = [{'name': 'b', 'refs': ['J1'], 'zone': [5, 0, 15, 10],
                       'tolerance_mm': 0},
                      {'name': 'a', 'refs': ['X2'], 'zone': [0, 2, 40, 40],
                       'exclusive': True, 'tolerance_mm': 0}]
            pcb = parse_kicad_pcb(path)

            def unsat(claims):
                v, _m = floorplan.plan_check(_doc(blocks, claims), pcb, path)
                return [x for x in v
                        if x.rule == 'plan_zone_exclusive_unsatisfiable']
            self.assertEqual(unsat([J1_NORTH]), [])
            self.assertTrue(unsat([]))              # control

    def test_the_reach_covers_a_pose_far_past_the_side(self):
        """Only poses whose centre is 7 mm past the side clear the keep-out;
        a short reach would refuse an intent the grade passes."""
        part = floorplan._LocalPart(0.0, (-2.0, -2.0, 2.0, 2.0))
        ko = [{'name': 'K', 'rect': (-50.0, -5.0, 60.0, 60.0),
               'sides': ('F', 'B'), 'allow': ()}]
        v = floorplan.zone_pose_feasibility((0, 0, 10, 10), 0.0, part, ko,
                                            open_sides=('north',))
        self.assertTrue(v['feasible'], v)
        self.assertLessEqual(v['witness'][1], -7.0 + 1e-6)

    def test_every_zone_side_is_clamped(self):
        self.assertEqual(floorplan.emitted_zone_rect(
            (1, 1, 9, 49.9996), (0, 0, 10, 49.9996)), [1, 1, 9, 49.9996])
        self.assertEqual(floorplan.emitted_zone_rect(
            (0.0004, 0.0004, 9, 9), (0.0004, 0.0004, 10, 10)),
            [0.0004, 0.0004, 9, 9])

    def test_a_side_two_hundredths_inside_stays_closed(self):
        self.assertEqual(floorplan.zone_open_sides(
            (0, 0.02, 10, 10), (0, 0, 50, 40), ('north',)), ())



#: One zone per side of the 40 mm board, each reaching the outline there, and
#: a 4 mm part whose centre sits 2.1 mm past that side (its pad with it).
_SIDES = {'north': ([4, 0, 16, 8], (10, -2.1)),
          'south': ([4, 32, 16, 40], (10, 42.1)),
          'west': ([0, 4, 8, 16], (-2.1, 10)),
          'east': ([32, 4, 40, 16], (42.1, 10))}


class PadsPastAnOpenSide(unittest.TestCase):
    """The round-2 verifier's: each of these SURVIVED test_1174 before."""

    def _grade(self, path, intent):
        return floorplan.grade(intent, parse_kicad_pcb(path), path)

    def _claim(self, edge):
        return {'ref': 'J1', 'edge': edge,
                'overhang_mm': {'min': 0, 'max': 5}}

    def test_every_side_is_measured(self):
        for edge, (zone, (x, y)) in sorted(_SIDES.items()):
            with self.subTest(edge=edge), \
                    tempfile.TemporaryDirectory() as td:
                path = _synthetic(td, [_box('J1', x, y, 2, 2)])
                errs = _zone_errors(self._grade(path, _doc(
                    [{'name': 'b', 'refs': ['J1'], 'zone': zone,
                      'tolerance_mm': 0}], [self._claim(edge)])))
                self.assertEqual(len(errs), 1, errs)
                self.assertTrue(errs[0].measured['pads_graded'])
                self.assertEqual(errs[0].measured['axis'], edge)
                self.assertAlmostEqual(errs[0].measured['outside_mm'], 2.4,
                                       places=3)

    def test_a_small_escape_is_an_error_and_none_is_not(self):
        """Half a 0.6 mm pad past the edge is 0.3 mm of copper off the
        board, at the zone's tolerance 0; the same pad 0.6 mm inboard is
        an overhang."""
        zone = [{'name': 'b', 'refs': ['J1'], 'zone': [4, 0, 16, 8],
                 'tolerance_mm': 0}]
        with tempfile.TemporaryDirectory() as td:
            over = _synthetic(td, [_box('J1', 10, 0, 2, 2)])
            errs = _zone_errors(self._grade(over, _doc(
                zone, [self._claim('north')])))
        with tempfile.TemporaryDirectory() as td:
            held = _synthetic(td, [_box('J1', 10, 0, 2, 2, pad=(0, 0.6))])
            ok = _zone_errors(self._grade(held, _doc(
                zone, [self._claim('north')])))
        self.assertEqual(len(errs), 1, errs)
        self.assertAlmostEqual(errs[0].measured['outside_mm'], 0.3, places=3)
        self.assertEqual(ok, [])

    def test_a_zone_drawn_past_the_board_measures_the_board(self):
        """A side opens when it lies ON or BEYOND the outline, so the pads
        are measured against the board's side: a zone 5 mm past the north
        edge used to let pads 2.4 mm off the board grade clean. Every side
        (round-3 verifier: only north was pinned)."""
        for edge, (zone, (x, y)) in sorted(_SIDES.items()):
            past = {'north': [zone[0], zone[1] - 5, zone[2], zone[3]],
                    'south': [zone[0], zone[1], zone[2], zone[3] + 5],
                    'west': [zone[0] - 5, zone[1], zone[2], zone[3]],
                    'east': [zone[0], zone[1], zone[2] + 5, zone[3]]}[edge]
            with self.subTest(edge=edge),                     tempfile.TemporaryDirectory() as td:
                gone = _synthetic(td, [_box('J1', x, y, 2, 2)])
                errs = _zone_errors(self._grade(gone, _doc(
                    [{'name': 'b', 'refs': ['J1'], 'zone': past,
                      'tolerance_mm': 0}], [self._claim(edge)])))
                self.assertEqual(len(errs), 1, errs)
                self.assertEqual(errs[0].measured['axis'], edge)
                self.assertAlmostEqual(errs[0].measured['outside_mm'], 2.4,
                                       places=3)
                self.assertIn(f"past the board's {edge} edge",
                              errs[0].message)

    def test_one_finding_per_member(self):
        """Pads past the open north side AND a courtyard 1 mm past the
        closed west side: the member is named once, for its pads -- every
        branch of the rule yields at most one finding per member."""
        with tempfile.TemporaryDirectory() as td:
            path = _synthetic(td, [_box('J1', 5, -2.1, 2, 2)])
            errs = _zone_errors(self._grade(path, _doc(
                [{'name': 'b', 'refs': ['J1'], 'zone': [4, 0, 16, 8],
                  'tolerance_mm': 0}], [self._claim('north')])))
        self.assertEqual(len(errs), 1, errs)
        self.assertTrue(errs[0].measured.get('pads_graded'), errs)





if __name__ == '__main__':
    unittest.main()
