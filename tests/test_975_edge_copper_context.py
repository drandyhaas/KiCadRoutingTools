"""#975: `EdgeCopperContext` -- the pad-edge grade with its board constants
read once, and the per-pad reading a seat search asks at a trial pose.

This is a HOIST plus a new primitive, so the arms are mechanism arms:

  A. THE GRADE IS UNCHANGED. A context graded after many pose readings equals
     a fresh `grade_pad_edge_clearance` by repr, on rectangular, sampled,
     polygon and castellated boards; and values recorded at f343befc (the
     parent commit) pin what neither side of that equality can move.
     `tests/measure_975_pad_edge_census.py` is the whole-corpus version:
     22 boards x 9 arms + 13 sibling/outline variants, 180 of 180 identical
     against f343befc.
  B. THE HOIST HAPPENED. Spies on every reader count 0 reads during `grade`,
     `pad_copper(pose)` and `pose_copper`, and the build's own counts.
  C. PADS ARE LIVE: a pad moved in place after the build is graded moved.
  D. EACH RESULT IS FRESH: mutating one result, nested rule rows included,
     does not reach the next.
  E. EXCEPTIONS AND ROWS KEEP THEIR ORDER: an invalid requirement raises
     before any file is opened; a malformed project raises from the context
     as it did from the grader; the unreadable-rules row precedes the project
     row; an int requirement stays an int.
  F. THE POSE READING AGREES WITH AN INDEPENDENT ORACLE: the part written at
     the pose by `write_placed_output`, re-parsed, and graded. Parts cover a
     tilted pad on a -45 degree footprint, castellated pads with a drill
     offset, a B-side part, custom polygons, a sampled outline and a slanted
     edge that proves the fixture sees the sign of the residual tilt.
  G. THE READING IS THE GRADE: at the file pose, the pads that read short are
     exactly the grade's findings, the reasons are its unmeasured rows, and
     `pose_copper.clears` is exactly "no finding", at the EPS boundary too.
  H. `_PosedPad` CARRIES EVERY ATTRIBUTE THE GRADER READS, on every path.
  J. `edge_copper_for` REUSES, REBUILDS AND CACHES A FAILURE as documented.
  K. `grade_pad_legality` KEEPS ITS `source` AND `argument_mm` LABELS.

Speed is measured, not gated, by `tests/measure_975_seat_copper.py`.
"""
import copy
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / p) for p in ('py_router', 'py_placer', 'tests')]

import builtins  # noqa: E402
import design_rules  # noqa: E402
import kicad_parser  # noqa: E402
import list_nets  # noqa: E402
from check_drc import check_pad_board_edge  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import legality as L  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402

RUN_ALL_TIMEOUT = 600
EPS = L.EPS
# Two independent integer-nanometre snaps (the parser's pad globals, and the
# written footprint origin), each <= 0.5 nm per axis, rotate into <= 1.21 nm.
# That holds for the poses below, which the writer stores exactly (3-decimal
# coordinates, short angles); an arbitrary-decimal pose adds the writer's own
# rounding, and `pads_at_pose`'s docstring says so.
POSE_TOL = 1.5e-6

#: Every file open in this process, whatever API asked for it. `builtins.open`
#: alone misses `io.open`, `pathlib` and C-level opens: a `Path.read_bytes()`
#: added to `grade` passed arm B while it spied on `open` only. An audit hook
#: cannot be removed, so it counts only while `_OPENS['on']`.
_OPENS = {'on': False, 'paths': []}


def _audit(event, args):
    if event == 'open' and _OPENS['on']:
        _OPENS['paths'].append(args[0])


sys.addaudithook(_audit)
BOARDS = {name: str(ROOT / 'kicad_files' / f'{name}.kicad_pcb') for name in (
    'esp_prog', 'tigard', 'watchy', 'rp2350_fpga_eensy_prePlane', 'orangecrab_ext_pll')}
_PARSED = {}


def board(name):
    if name not in _PARSED:
        _PARSED[name] = parse_kicad_pcb(BOARDS[name])
    return _PARSED[name]


def fresh(name):
    return parse_kicad_pcb(BOARDS[name])


class Spy:
    """Counts calls to the readers the context must not touch after its build."""
    def __init__(self):
        self.counts = {}

    def wrap(self, name, fn):
        def spy(*args, **kw):
            self.counts[name] = self.counts.get(name, 0) + 1
            return fn(*args, **kw)
        return spy

    def __enter__(self):
        self.patches = [
            patch.object(builtins, 'open', self.wrap('open', builtins.open)),
            patch.object(os.path, 'exists', self.wrap('exists', os.path.exists)),
            patch.object(kicad_parser, '_collect_edge_cuts_segments',
                         self.wrap('edge_cuts', kicad_parser._collect_edge_cuts_segments)),
            patch.object(list_nets, 'read_design_rules',
                         self.wrap('read_design_rules', list_nets.read_design_rules)),
            patch.object(design_rules, 'parse_dru', self.wrap('parse_dru', design_rules.parse_dru)),
            patch.object(design_rules, 'validate_dru_structure',
                         self.wrap('validate_dru', design_rules.validate_dru_structure)),
        ]
        for p in self.patches:
            p.start()
        _OPENS['paths'] = []
        _OPENS['on'] = True
        return self

    def __exit__(self, *exc):
        _OPENS['on'] = False
        if _OPENS['paths']:
            self.counts['audit_open'] = len(_OPENS['paths'])
        for p in reversed(self.patches):
            p.stop()


def single(pcb, ref):
    one = copy.copy(pcb)
    one.footprints = {ref: pcb.footprints[ref]}
    return one


def by_index(grade):
    """{pad_index: shortfall} and {pad_index: reason} for a one-footprint grade."""
    return ({f['pad_index']: f['shortfall_mm'] for f in grade['findings']},
            {u['pad_index']: u['reason'] for u in grade['unmeasured']})


class Grade(unittest.TestCase):
    """A. The grade is unchanged."""

    def test_cached_context_equals_a_fresh_grade(self):
        for name, path in BOARDS.items():
            pcb = board(name)
            for required in (0.0, 0.55, 1000.0):
                with self.subTest(board=name, required=required):
                    ctx = L.EdgeCopperContext(pcb, required, path)
                    for ref, fp in list(pcb.footprints.items())[:6]:
                        ctx.pose_copper(fp, (fp.x + 0.5, fp.y - 0.25, (fp.rotation or 0.0) + 90))
                    self.assertEqual(repr(ctx.grade()),
                                     repr(L.grade_pad_edge_clearance(fresh(name), required, path)))

    def test_values_recorded_at_the_parent_commit(self):
        # Recorded at f343befc with `grade_pad_edge_clearance(pcb, .55, path)`.
        # These are the oracle arms A's equality cannot provide: a change to
        # what the context BUILDS moves both sides of that equality alike.
        expect = {
            'esp_prog': dict(measured=75, findings=0, complete=True, unmeasured=0),
            'tigard': dict(measured=428, findings=2, complete=False, unmeasured=4),
            'watchy': dict(measured=0, findings=5, complete=False, unmeasured=288),
        }
        for name, want in expect.items():
            with self.subTest(board=name):
                g = L.EdgeCopperContext(board(name), 0.55, BOARDS[name]).grade()
                self.assertEqual(dict(measured=g['measured_pads'], findings=len(g['findings']),
                                      complete=g['complete'], unmeasured=len(g['unmeasured'])),
                                 want)
        g = L.EdgeCopperContext(board('tigard'), 0.55, BOARDS['tigard']).grade()
        self.assertEqual(sorted(f['pad_ref'] for f in g['findings']), ['J7.MP', 'J7.MP'])
        self.assertEqual([f['shortfall_mm'] for f in g['findings']],
                         [0.15000000000000857, 0.15000000000000857])


class Reads(unittest.TestCase):
    """B. The hoist happened."""

    def test_no_file_is_read_after_the_build(self):
        for name in ('esp_prog', 'watchy', 'rp2350_fpga_eensy_prePlane'):
            pcb = board(name)
            with Spy() as build:
                ctx = L.EdgeCopperContext(pcb, 0.55, BOARDS[name])
            self.assertEqual(build.counts.get('edge_cuts'), 1, name)
            self.assertEqual(build.counts.get('read_design_rules'), 1, name)
            with Spy() as use:
                ctx.grade()
                for fp in list(pcb.footprints.values())[:8]:
                    ctx.pad_copper(fp, (fp.x + 1, fp.y, 33.0))
                    ctx.pose_copper(fp, (fp.x, fp.y + 1, 270.0))
                    ctx.pose_copper(fp)
            self.assertEqual(use.counts, {}, name)

    def test_for_board_reads_the_project_once_more_only_when_it_resolves(self):
        pcb = board('esp_prog')
        with Spy() as resolved:
            ctx = L.EdgeCopperContext.for_board(pcb, BOARDS['esp_prog'], 0.25, None)
        self.assertEqual(resolved.counts.get('read_design_rules'), 2)
        self.assertEqual((ctx.required, ctx.source), (0.55, 'fixed default'))
        with Spy() as given:
            ctx = L.EdgeCopperContext.for_board(pcb, BOARDS['esp_prog'], 0.25, 0.4)
        self.assertEqual(given.counts.get('read_design_rules'), 1)
        self.assertEqual((ctx.required, ctx.source, ctx.argument_mm), (0.4, 'caller argument', 0.4))


class Live(unittest.TestCase):
    """C. Pads are read live; D. each result is fresh."""

    def test_a_pad_moved_in_place_is_graded_moved(self):
        pcb = fresh('esp_prog')
        ctx = L.EdgeCopperContext(pcb, 0.55, BOARDS['esp_prog'])
        before = ctx.grade()
        pad = pcb.footprints['USB1'].pads[6]
        pad.global_x = pcb.board_info.board_bounds[0] + 0.1
        pad.rect_rotation = 20.0
        after = ctx.grade()
        self.assertNotEqual(repr(before), repr(after))
        self.assertEqual(repr(after), repr(L.grade_pad_edge_clearance(pcb, 0.55, BOARDS['esp_prog'])))
        del pcb.footprints['USB1']
        self.assertEqual(repr(ctx.grade()),
                         repr(L.grade_pad_edge_clearance(pcb, 0.55, BOARDS['esp_prog'])))

    def test_mutating_a_result_does_not_reach_the_next(self):
        with tempfile.TemporaryDirectory(prefix='krt975_') as tmp:
            path = os.path.join(tmp, 'b.kicad_pcb')
            shutil.copyfile(BOARDS['tigard'], path)
            Path(tmp, 'b.kicad_dru').write_text(
                '(version 1)\n(rule "edge" (constraint edge_clearance (min 0.75mm)))\n',
                encoding='utf-8')
            ctx = L.EdgeCopperContext(parse_kicad_pcb(path), 0.55, path)
            first = ctx.grade()
            pristine = repr(ctx.grade())
            first['findings'].clear()
            first['minimum_gap_by_ref_mm']['J7'] = -1
            first['rules_unmeasured'][0]['declared']['min'] = 99
            first['rules_unmeasured'].append({'forged': True})
            self.assertEqual(repr(ctx.grade()), pristine)
            self.assertEqual(ctx.grade()['rules_unmeasured'][0]['declared'], {'min': 0.75})


class Order(unittest.TestCase):
    """E. Exceptions and rows keep their order."""

    def test_an_invalid_requirement_raises_before_anything_is_opened(self):
        for bad in (float('nan'), float('inf'), -1):
            with self.subTest(required=bad), Spy() as spy:
                with self.assertRaisesRegex(ValueError, 'finite and nonnegative'):
                    L.EdgeCopperContext(board('esp_prog'), bad, BOARDS['esp_prog'])
                self.assertEqual(spy.counts, {})

    def test_malformed_projects_raise_where_the_grader_raised(self):
        with tempfile.TemporaryDirectory(prefix='krt975_') as tmp:
            path = os.path.join(tmp, 'b.kicad_pcb')
            shutil.copyfile(BOARDS['esp_prog'], path)
            pcb = parse_kicad_pcb(path)
            for text, kind in (('[]', AttributeError), ('"abc"', AttributeError)):
                Path(tmp, 'b.kicad_pro').write_text(text, encoding='utf-8')
                with self.subTest(project=text):
                    with self.assertRaises(kind):
                        L.EdgeCopperContext(pcb, 0.55, path)
                    with self.assertRaises(kind):
                        L.grade_pad_edge_clearance(pcb, 0.55, path)
                    with self.assertRaises(kind):
                        L.grade_pad_legality(pcb, 0.25, exact=False, pcb_file=path)
                    self.assertEqual(list_nets.board_floor_knobs(path, 0.25, None)[1], 0.55)

    def test_the_unreadable_rules_row_precedes_the_project_row(self):
        with tempfile.TemporaryDirectory(prefix='krt975_') as tmp:
            path = os.path.join(tmp, 'b.kicad_pcb')
            shutil.copyfile(BOARDS['esp_prog'], path)
            Path(tmp, 'b.kicad_dru').write_text('(version 1) (rule "unterminated',
                                                encoding='utf-8')
            Path(tmp, 'b.kicad_pro').write_text(json.dumps({'board': {'design_settings': {
                'rules': {'min_copper_edge_clearance': float('nan')}}}}), encoding='utf-8')
            rows = L.EdgeCopperContext(parse_kicad_pcb(path), 0.55, path).grade()['rules_unmeasured']
            self.assertEqual(len(rows), 2)
            self.assertIn('unreadable', rows[0]['reason'])
            self.assertEqual(rows[1]['constraint'], 'min_copper_edge_clearance')

    def test_missing_source_and_int_requirement(self):
        pcb = board('esp_prog')
        missing = L.EdgeCopperContext(pcb, 1, os.path.join(tempfile.gettempdir(), 'no-such.kicad_pcb'))
        g = missing.grade()
        self.assertFalse(missing.rectangular)
        self.assertFalse(g['complete'])
        self.assertIs(type(g['required_mm']), int)
        bare = copy.copy(pcb)
        bare.source_path = None
        with Spy() as spy:
            ctx = L.EdgeCopperContext(bare, 0.55)
        self.assertEqual(spy.counts, {})
        self.assertFalse(ctx.grade()['complete'])


def slanted_esp_prog(tmp):
    """esp_prog with its west edge slanted 4 mm over its height."""
    text = Path(BOARDS['esp_prog']).read_text(encoding='utf-8')
    top = '(start 114 91)\n\t\t(end 145.75 91)'
    west = '(start 114 105.5)\n\t\t(end 114 91)'
    assert text.count(top) == 1 and text.count(west) == 1, 'esp_prog outline moved'
    text = text.replace(top, '(start 118 91)\n\t\t(end 145.75 91)').replace(
        west, '(start 114 105.5)\n\t\t(end 118 91)')
    path = os.path.join(tmp, 'slanted.kicad_pcb')
    Path(path).write_text(text, encoding='utf-8')
    return path


class PoseOracle(unittest.TestCase):
    """F. The pose reading agrees with the written, re-parsed, graded part."""

    def check(self, path, moves):
        pcb = parse_kicad_pcb(path)
        ctx = L.EdgeCopperContext(pcb, 1000.0, path)
        with tempfile.TemporaryDirectory(prefix='krt975_') as tmp:
            out = os.path.join(tmp, 'posed.kicad_pcb')
            write_placed_output(path, out, [
                {'reference': ref, 'new_x': x, 'new_y': y, 'new_rotation': rot}
                for ref, (x, y, rot) in moves.items()])
            written = parse_kicad_pcb(out)
            worst = 0.0
            for ref, pose in moves.items():
                with self.subTest(board=os.path.basename(path), ref=ref, pose=pose):
                    oracle = L.grade_pad_edge_clearance(single(written, ref), 1000.0, out)
                    short, reasons = by_index(oracle)
                    readings = ctx.pad_copper(pcb.footprints[ref], pose)
                    self.assertTrue(readings, ref)
                    got = {r.index: r.amount_mm for r in readings if r.amount_mm is not None
                           and r.amount_mm > EPS}
                    self.assertEqual(set(got), set(short))
                    self.assertEqual({r.index: r.reason for r in readings if r.reason}, reasons)
                    for index, amount in short.items():
                        worst = max(worst, abs(got[index] - amount))
                        self.assertLessEqual(abs(got[index] - amount), POSE_TOL, (ref, index))
            return worst

    def test_rectangular_boards(self):
        for rot_offset in (33.0, 90.5, 270.0):
            with self.subTest(rotation=rot_offset):
                pcb = board('esp_prog')
                usb = pcb.footprints['USB1']
                self.check(BOARDS['esp_prog'], {
                    'USB1': (usb.x + 1.234, usb.y - 0.5, (usb.rotation + rot_offset) % 360)})
                pcb = board('rp2350_fpga_eensy_prePlane')
                moves = {}
                for ref in ('C28', 'U8', 'J2', 'U4'):
                    fp = pcb.footprints[ref]
                    moves[ref] = (round(fp.x + 1.234, 3), round(fp.y - 0.5, 3),
                                  round((fp.rotation + rot_offset) % 360, 3))
                self.check(BOARDS['rp2350_fpga_eensy_prePlane'], moves)
                pcb = board('orangecrab_ext_pll')
                fp = pcb.footprints['U10']
                self.check(BOARDS['orangecrab_ext_pll'], {
                    'U10': (round(fp.x - 0.75, 3), round(fp.y + 0.25, 3),
                            round((fp.rotation + rot_offset) % 360, 3))})

    def test_sampled_outline(self):
        fp = board('watchy').footprints['J2']
        for rot_offset in (33.0, 180.0):
            self.check(BOARDS['watchy'], {
                'J2': (round(fp.x + 0.5, 3), round(fp.y - 0.25, 3),
                       round((fp.rotation + rot_offset) % 360, 3))})

    def test_a_slanted_edge_sees_the_sign_of_the_tilt(self):
        with tempfile.TemporaryDirectory(prefix='krt975_') as tmp:
            path = slanted_esp_prog(tmp)
            pose = (120.0, 97.0, 33.0)
            self.check(path, {'USB1': pose})
            pcb = parse_kicad_pcb(path)
            ctx = L.EdgeCopperContext(pcb, 1000.0, path)
            self.assertFalse(ctx.rectangular)
            posed = L.pads_at_pose(pcb.footprints['USB1'], pose)
            moved = 0.0
            for pad in posed:
                if L._pad_has_no_copper(pad) or not pad.rect_rotation:
                    continue
                _, amount, _ = check_pad_board_edge(pad, ctx.rings, ctx.outer, ctx.cutouts,
                                                    1000.0, ctx.bounds, 0.0)
                pad.rect_rotation = -pad.rect_rotation
                _, flipped, _ = check_pad_board_edge(pad, ctx.rings, ctx.outer, ctx.cutouts,
                                                     1000.0, ctx.bounds, 0.0)
                moved = max(moved, abs(amount - flipped))
            self.assertGreater(moved, 0.01, 'fixture cannot tell the tilt sign apart')


class ReadingIsTheGrade(unittest.TestCase):
    """G. At the file pose the reading is the grade, at the EPS boundary too."""

    def test_findings_and_reasons_match_per_footprint(self):
        for name in BOARDS:
            pcb = board(name)
            ctx = L.EdgeCopperContext(pcb, 0.55, BOARDS[name])
            grade = ctx.grade()
            for ref, fp in pcb.footprints.items():
                short = {f['pad_index']: f for f in grade['findings']
                         if f['pad_ref'].startswith(ref + '.') and
                         pcb.footprints[ref].pads[f['pad_index']].pad_number ==
                         f['pad_ref'][len(ref) + 1:]}
                pc = ctx.pose_copper(fp)
                got = {r.index: r for r in pc.pads if r.amount_mm is not None
                       and r.amount_mm > EPS}
                with self.subTest(board=name, ref=ref):
                    self.assertEqual(set(got), set(short))
                    self.assertEqual(pc.clears, not short)
                    for index, f in short.items():
                        self.assertEqual(got[index].amount_mm, f['shortfall_mm'])
                        self.assertEqual(got[index].edge, f['edge'])
                    moved = ctx.pad_copper(fp, (fp.x, fp.y, fp.rotation or 0.0))
                    for a, b in zip(moved, pc.pads):
                        self.assertEqual((a.index, a.reason, a.edge), (b.index, b.reason, b.edge))
                        if a.amount_mm is not None:
                            self.assertAlmostEqual(a.amount_mm, b.amount_mm, delta=1e-9)

    def test_the_eps_boundary(self):
        pcb = board('esp_prog')
        fp = pcb.footprints['USB1']
        probe = L.EdgeCopperContext(pcb, 0.55, BOARDS['esp_prog']).pose_copper(fp)
        gap = min(r.gap_mm for r in probe.pads if r.gap_mm is not None)
        for extra, clears in ((EPS / 2, True), (EPS * 3, False)):
            ctx = L.EdgeCopperContext(pcb, gap + extra, BOARDS['esp_prog'])
            pc = ctx.pose_copper(fp)
            with self.subTest(extra=extra):
                self.assertGreater(pc.worst_mm, 0.0)
                self.assertEqual(pc.clears, clears)
                self.assertEqual(pc.clears, not [
                    f for f in ctx.grade()['findings'] if f['pad_ref'].startswith('USB1.')])

    def test_a_nan_reading_is_not_the_worst_pad(self):
        pcb = fresh('esp_prog')
        fp = pcb.footprints['USB1']
        copper = [i for i, p in enumerate(fp.pads) if not L._pad_has_no_copper(p)]
        fp.pads[copper[0]].size_x = float('nan')
        fp.pads[copper[-1]].global_x = pcb.board_info.board_bounds[0] + 0.1
        ctx = L.EdgeCopperContext(pcb, 0.55, BOARDS['esp_prog'])
        pc = ctx.pose_copper(fp)
        self.assertEqual(pc.worst_index, copper[-1])
        self.assertFalse(pc.clears)
        grade = ctx.grade()
        self.assertEqual({f['pad_index'] for f in grade['findings']
                          if f['pad_ref'].startswith('USB1.')}, {copper[-1]})

    def test_certified_and_fallback(self):
        pcb = fresh('esp_prog')
        ctx = L.EdgeCopperContext(pcb, 0.55, BOARDS['esp_prog'])
        fp = pcb.footprints['USB1']
        self.assertTrue(ctx.pose_copper(fp).certified)
        first, second = [i for i, p in enumerate(fp.pads) if not L._pad_has_no_copper(p)][:2]
        shape = fp.pads[first].shape
        fp.pads[first].shape = 'trapezoid'
        pc = ctx.pose_copper(fp)
        self.assertEqual(pc.fallback, (first,))
        self.assertFalse(pc.certified)
        reading = [r for r in pc.pads if r.index == first][0]
        self.assertIsNone(reading.amount_mm)
        self.assertEqual(reading.reason, 'unsupported pad geometry')
        fp.pads[first].shape = shape
        self.assertTrue(ctx.pose_copper(fp).certified)
        fp.pads[second].geometry_approximations = ('chamfer',)
        pc = ctx.pose_copper(fp)
        self.assertEqual(pc.fallback, ())
        self.assertFalse(pc.certified)
        watchy = L.EdgeCopperContext(board('watchy'), 0.55, BOARDS['watchy'])
        self.assertFalse(watchy.pose_copper(board('watchy').footprints['J2']).certified)


class Recorder:
    """A pad whose every attribute read is logged."""
    def __init__(self, pad, log):
        object.__setattr__(self, '_pad', pad)
        object.__setattr__(self, '_log', log)

    def __getattr__(self, name):
        self._log.add(name)
        return getattr(self._pad, name)


class PosedPadSlots(unittest.TestCase):
    """H. `_PosedPad` carries every attribute the grader reads."""

    def test_every_path_reads_only_carried_attributes(self):
        log = set()
        for name in ('esp_prog', 'watchy', 'rp2350_fpga_eensy_prePlane', 'orangecrab_ext_pll'):
            pcb = fresh(name)
            ctx = L.EdgeCopperContext(pcb, 0.55, BOARDS[name])
            for fp in pcb.footprints.values():
                list(ctx._readings([Recorder(p, log) for p in fp.pads]))
        pcb = fresh('esp_prog')
        pcb.footprints['USB1'].pads[0].shape = 'trapezoid'
        ctx = L.EdgeCopperContext(pcb, 0.55, BOARDS['esp_prog'])
        list(ctx._readings([Recorder(p, log) for p in pcb.footprints['USB1'].pads]))
        bare = copy.copy(pcb)
        bare.board_info = copy.copy(pcb.board_info)
        bare.board_info.board_bounds = None
        list(L.EdgeCopperContext(bare, 0.55)._readings(
            [Recorder(p, log) for p in pcb.footprints['USB1'].pads]))
        self.assertIn('rect_rotation', log)
        self.assertIn('polygons', log)
        self.assertLessEqual(log, set(L._PosedPad.__slots__))


class Holder(unittest.TestCase):
    """J. `edge_copper_for` reuses, rebuilds and caches a failure."""

    def test_reuse_and_rebuild(self):
        class State:
            pass
        pcb, path = board('esp_prog'), BOARDS['esp_prog']
        state, other = State(), State()
        ctx, err = L.edge_copper_for(state, pcb, path, 0.25, 0.55)
        self.assertIsNone(err)
        self.assertIs(L.edge_copper_for(state, pcb, path, 0.25, 0.55)[0], ctx)
        self.assertIsNot(L.edge_copper_for(other, pcb, path, 0.25, 0.55)[0], ctx)
        self.assertIsNot(L.edge_copper_for(state, fresh('esp_prog'), path, 0.25, 0.55)[0], ctx)
        ctx = L.edge_copper_for(state, pcb, path, 0.25, 0.55)[0]
        self.assertEqual(L.edge_copper_for(state, pcb, path, 0.25, 0.3)[0].required, 0.3)
        with tempfile.TemporaryDirectory(prefix='krt975_') as tmp:
            copy_path = os.path.join(tmp, 'b.kicad_pcb')
            shutil.copyfile(path, copy_path)
            self.assertEqual(L.edge_copper_for(state, pcb, copy_path, 0.25, 0.3)[0].path, copy_path)

    def test_a_failure_is_cached(self):
        class State:
            pass
        with tempfile.TemporaryDirectory(prefix='krt975_') as tmp:
            path = os.path.join(tmp, 'b.kicad_pcb')
            shutil.copyfile(BOARDS['esp_prog'], path)
            Path(tmp, 'b.kicad_pro').write_text('[]', encoding='utf-8')
            pcb, state = parse_kicad_pcb(path), State()
            with Spy() as spy:
                for _ in range(5):
                    ctx, err = L.edge_copper_for(state, pcb, path, 0.25, 0.55)
                    self.assertIsNone(ctx)
                    self.assertIn('AttributeError', err)
            self.assertEqual(spy.counts.get('read_design_rules'), 1)


class Labels(unittest.TestCase):
    """K. `grade_pad_legality` keeps its `source` and `argument_mm` labels."""

    def test_source_and_argument(self):
        pcb, path = board('esp_prog'), BOARDS['esp_prog']
        given = L.grade_pad_legality(pcb, 0.25, exact=False, edge_margin=0.5, pcb_file=path)
        self.assertEqual((given['pad_edge']['source'], given['pad_edge']['argument_mm'],
                          given['pad_edge']['required_mm']), ('caller argument', 0.5, 0.5))
        resolved = L.grade_pad_legality(pcb, 0.25, exact=False, pcb_file=path)
        self.assertEqual((resolved['pad_edge']['source'], resolved['pad_edge']['argument_mm'],
                          resolved['pad_edge']['required_mm']), ('fixed default', None, 0.55))


if __name__ == '__main__':
    unittest.main()
