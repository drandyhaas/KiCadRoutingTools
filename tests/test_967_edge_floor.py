"""Resolved placement edge floor on the published esp_prog poses (#967).

Reconstruct geometry from the existing repository board, never edit it.
Frozen public board/brief identities and independent native evidence are in
docs/issue-967-verification.md. These tests need no KiCad installation.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import os
import stat
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / p) for p in ('py_router', 'py_placer', 'tests')]
from copy_board import copy_board
from kicad_parser import parse_kicad_pcb, iter_footprint_blocks, find_matching_paren
from placement.legality import grade_pad_legality, grade_pad_edge_clearance
from placement.writer import write_placed_output
from placement.seeder import stamp_locked

BOARD = ROOT / 'kicad_files/esp_prog.kicad_pcb'
POSES = {
    'bad': {'C2': (126.9, 103.5, 90), 'C4': (122.4, 102.2, 90),
            'U1': (126.6, 97.75, 270), 'Y1': (124.7, 103.2, 270)},
    'control': {'C2': (125.4, 93.4, 90), 'C4': (122.7, 95.6, 180),
                'U1': (127.6, 99.7, 180), 'Y1': (123, 93.4, 0)},
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class EdgeFloor(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='krt967_')
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.source_hash = digest(BOARD)
        self.addCleanup(lambda: self.assertEqual(digest(BOARD), self.source_hash))

    def fixture(self, case='bad'):
        out = self.work / (case + '.kicad_pcb')
        copy_board(str(BOARD), str(out))
        rows = [dict(reference=r, new_x=p[0], new_y=p[1], new_rotation=p[2])
                for r, p in POSES[case].items()]
        write_placed_output(str(out), str(out), rows)
        pcb = parse_kicad_pcb(str(out))
        stamp_locked(str(out), set(pcb.footprints) - set(POSES[case]))
        return out

    def cli(self, board, *verbs, floor='.55', expected=0, extra=(), output=None):
        out = output or self.work / 'out.kicad_pcb'
        args = [sys.executable, '-X', 'utf8', str(ROOT/'py_placer/place_pose.py'),
                str(board), str(out), '--clearance', '.25']
        if floor is not None:
            args += ['--board-edge-clearance=' + floor]
        args += list(extra) + list(verbs)
        result = subprocess.run(args, cwd=ROOT, capture_output=True,
                                text=True, encoding='utf-8', timeout=90)
        self.assertNotIn('Traceback', result.stdout + result.stderr)
        self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        summaries = [json.loads(s.split(': ', 1)[1]) for s in result.stdout.splitlines()
                     if s.startswith('JSON_SUMMARY: ')]
        self.assertEqual(len(summaries), 1, result.stdout)
        if expected == 4:
            self.assertTrue(summaries[0]['refused'])
            self.assertIsNone(summaries[0]['output'])
        return summaries[0], out

    def test_frozen_bad_and_positive_control(self):
        board = self.fixture()
        for floor, count in (('.50', 0), ('.55', 2), ('.60', 2)):
            s, out = self.cli(board, 'set', 'Y1', '124.7', '103.2', '--rot', '270', floor=floor)
            self.assertEqual(s['legal'], count == 0)
            self.assertTrue(s['no_worse'])
            self.assertEqual(s['pad_edge_conflicts_after'], count)
            self.assertEqual(s['oob_pad_copper_count_after'], 0)
            self.assertEqual(s['pad_conflicts_after'], 0)
            if count:
                rows = s['pad_edge_after']['findings']
                self.assertEqual({r['pad_ref'] for r in rows}, {'Y1.2', 'Y1.3'})
                self.assertEqual(len({r['pad_index'] for r in rows}), 2)
                for r in rows:
                    self.assertAlmostEqual(r['gap_mm'], .50)
                    self.assertAlmostEqual(r['shortfall_mm'], float(floor)-.50)
        control = self.fixture('control')
        s, out = self.cli(control, 'set', 'Y1', '123', '93.4', '--rot', '0',
                          extra=('--strict-legal',))
        self.assertTrue(s['legal'])
        self.assertTrue(s['pad_edge_after']['complete'])
        self.assertEqual(parse_kicad_pcb(str(out)).footprints['Y1'].x, 123)

    def test_boundary_improving_and_worsening(self):
        board = self.fixture()
        for y, clean in (('103.18', False), ('103.15', True), ('103.10', True)):
            s, _ = self.cli(board, 'set', 'Y1', '124.7', y, '--rot', '270')
            self.assertTrue(s['no_worse'])
            self.assertEqual(s['legal'], clean)
        before = digest(board)
        s, _ = self.cli(board, 'set', 'Y1', '124.7', '103.21', '--rot', '270',
                        expected=4, output=board)
        self.assertIn('pad_edge_shortfall', s['refused'])
        self.assertEqual(digest(board), before)
        s, _ = self.cli(board, 'set', 'Y1', '124.7', '103.2', '--rot', '270',
                        expected=4, extra=('--strict-legal',), output=board)
        self.assertFalse(s['legal'])
        self.assertEqual(digest(board), before)

    def test_atomic_multi_near_and_dry_run(self):
        board = self.fixture()
        _, clean = self.cli(board, 'set', 'Y1', '124.7', '103.1', '--rot', '270')
        # A coordinated request must refuse as a whole when just one member
        # introduces an edge shortfall. Preserve sibling requirements as well.
        brief = clean.with_suffix('.design-brief.json')
        brief.write_text('{"version": 1, "purpose": "fixed test requirement"}')
        before, sibling = digest(clean), digest(brief)
        s, _ = self.cli(clean, 'set', 'Y1', '124.7', '103.2', '--rot', '270',
                        'set', 'C2', '126.9', '103.5', '--rot', '90',
                        expected=4, output=clean)
        self.assertIn('pad_edge_conflicts', s['refused'])
        self.assertEqual((digest(clean), digest(brief)), (before, sibling))
        s, out = self.cli(clean, 'set', 'Y1', '--near', '124.7', '103.2', '--rot', '270',
                          extra=('--radius', '.5', '--snap-step', '.05', '--snap-tries', '24'),
                          output=self.work/'snapped.kicad_pcb')
        self.assertTrue(s['snapped'])
        written = grade_pad_legality(parse_kicad_pcb(str(out)), .25, edge_margin=.55,
                                     pcb_file=str(out))
        self.assertEqual(written['pad_edge_conflicts'], 0)
        self.assertEqual(digest(out.with_suffix('.design-brief.json')), sibling)
        s, out = self.cli(board, 'set', 'Y1', '124.7', '103.15', '--rot', '270',
                          'set', 'C2', '126.9', '103.5', '--rot', '90',
                          extra=('--dry-run',), output=self.work/'dry.kicad_pcb')
        self.assertTrue(s['legal'])
        self.assertFalse(out.exists())

    def test_rule_sources_and_local_copper_override(self):
        board = self.fixture()
        s, _ = self.cli(board, 'set', 'Y1', '124.7', '103.2', '--rot', '270', floor=None)
        self.assertEqual(s['knobs']['board_edge_clearance']['requested'], None)
        self.assertEqual(s['pad_edge_after']['source'], 'fixed default')
        pro = board.with_suffix('.kicad_pro')
        pro.write_text(json.dumps({'board': {'design_settings': {'rules': {
            'min_copper_edge_clearance': .60}}},
            'net_settings': {'classes': [{'name': 'Default', 'clearance': .25}]}}))
        s, out = self.cli(board, 'set', 'Y1', '124.7', '103.2', '--rot', '270', floor=None)
        self.assertEqual(s['board_edge_clearance'], .60)
        self.assertEqual(s['pad_edge_after']['source'], 'board constraint')
        self.assertIsNone(s['pad_edge_after']['requested_mm'])
        self.assertEqual(digest(pro), digest(out.with_suffix('.kicad_pro')))
        s, _ = self.cli(board, 'set', 'Y1', '124.7', '103.2', '--rot', '270', floor='.50')
        self.assertEqual(s['pad_edge_after']['source'], 'cli')
        self.assertEqual(s['pad_edge_after']['requested_mm'], .50)
        self.assertTrue(s['legal'])  # placement's existing explicit override precedence
        pcb = parse_kicad_pcb(str(board))
        pcb.footprints['Y1'].pads[0].local_clearance = 3
        a = grade_pad_legality(pcb, .25, edge_margin=.50, pcb_file=str(board))
        b = grade_pad_legality(pcb, .25, edge_margin=.60, pcb_file=str(board))
        self.assertTrue(a['required'])
        for key in ('required', 'pad_conflicts', 'pad_shortfall', 'oob_pad_copper_refs'):
            self.assertEqual(a[key], b[key])

    def test_rotated_support_tolerance_and_unsupported(self):
        board = self.fixture()
        pcb = parse_kicad_pcb(str(board))
        fp = copy.deepcopy(pcb.footprints['Y1'])
        pad = fp.pads[0]
        fp.pads = [pad]
        pcb.footprints = {'Y1': fp}
        pad.shape, pad.size_x, pad.size_y, pad.rect_rotation = 'rect', 2, 1, 33
        pad.polygons = None
        pad.global_x = 130
        # Independent rectangle corner transform, not production support formula.
        angle = math.radians(33)
        support = max(x*math.sin(angle)+y*math.cos(angle)
                      for x in (-1, 1) for y in (-.5, .5))
        for gap, count in ((.50, 1), (.55, 0), (.60, 0), (.55-5e-7, 0), (.55-2e-6, 1)):
            pad.global_y = 105.5-support-gap
            report = grade_pad_edge_clearance(pcb, .55, str(board))
            self.assertEqual(len(report['findings']), count)
            self.assertAlmostEqual(report['minimum_gap_mm'], gap)
        # Rounded shapes rotated away from cardinal axes must not acquire
        # the sharp AABB corners of their outer rectangle.
        pad.shape, pad.size_x, pad.size_y = 'circle', 1, 1
        pad.global_y = 105.5-.5-.55
        self.assertEqual(grade_pad_edge_clearance(pcb, .55, str(board))['findings'], [])
        pad.shape = 'custom'
        report = grade_pad_edge_clearance(pcb, .55, str(board))
        self.assertFalse(report['complete'])
        self.assertIn('unsupported', report['unmeasured'][0]['reason'])
        # Existing custom-circle polygonization is inscribed (32 vertices).
        # At half a sample step it understates the radius by 0.0024076 mm:
        # a true .548 gap can look >= .55, so this cannot certify native copper.
        centre_y = 105.5-.5-.548
        pad.polygons = [[(130+.5*math.cos(math.radians(5.625+i*11.25)),
                          centre_y+.5*math.sin(math.radians(5.625+i*11.25)))
                         for i in range(32)]]
        report = grade_pad_edge_clearance(pcb, .55, str(board))
        self.assertGreater(report['minimum_gap_mm'], .55)
        self.assertFalse(report['complete'])
        self.assertIn('parsed polygons', report['unmeasured'][0]['reason'])
        pad.shape = 'rect'
        pad.polygons = None
        # A bounding box without a source/ring is not evidence of a closed board.
        pcb.source_path = None
        report = grade_pad_edge_clearance(pcb, .55)
        self.assertFalse(report['complete'])

    def test_simplified_primitive_coverage_survives_cli_write(self):
        board = self.fixture('control')
        original = board.read_text(encoding='utf-8')
        _, _, block, _, _ = next(b for b in iter_footprint_blocks(original)
                                if b[4] == 'Y1')
        start = block.index('(pad ')
        end = find_matching_paren(block, start)
        pad_text = block[start:end]
        variants = (
            ('chamfered pad', '(roundrect_rratio 0.25) (chamfer_ratio 0.05) '
                              '(chamfer top_left top_right bottom_left bottom_right)'),
            ('per-layer padstack', '(padstack (mode custom) '
                                  '(layer "F.Cu" (shape rect) (size 2 1)))'),
        )
        for reason, spec in variants:
            with self.subTest(reason=reason):
                replacement = pad_text[:-1] + spec + ')'
                board.write_text(original.replace(block, block[:start] + replacement
                                                   + block[end:], 1), encoding='utf-8')
                parsed = parse_kicad_pcb(str(board))
                self.assertEqual(parsed.footprints['Y1'].pads[0].geometry_approximations,
                                 (reason,))
                before = digest(board)
                s, out = self.cli(board, 'rotate', 'Y1', '0', '--relative')
                self.assertTrue(s['no_worse'])
                self.assertFalse(s['legal'])
                self.assertFalse(s['pad_edge_after']['complete'])
                self.assertIn(reason, s['pad_edge_after']['unmeasured'][0]['reason'])
                self.assertEqual(parse_kicad_pcb(str(out)).footprints['Y1'].pads[0]
                                 .geometry_approximations, (reason,))
                s, _ = self.cli(board, 'rotate', 'Y1', '0', '--relative', expected=4,
                                extra=('--strict-legal',), output=board)
                self.assertFalse(s['legal'])
                self.assertEqual(digest(board), before)

    def test_custom_edge_requirements_are_explicitly_unmeasured(self):
        board = self.fixture()
        dru = board.with_suffix('.kicad_dru')
        dru.write_text('(version 1)\n(rule "edge floor" '
                       '(constraint edge_clearance (min 0.75mm)))\n', encoding='utf-8')
        identity = digest(dru)
        s, out = self.cli(board, 'set', 'Y1', '124.7', '103.15', '--rot', '270')
        self.assertTrue(s['no_worse'])
        self.assertFalse(s['legal'])
        self.assertEqual(s['pad_edge_conflicts_after'], 0)  # scalar .55 only
        self.assertFalse(s['pad_edge_after']['complete'])
        missing = s['pad_edge_after']['rules_unmeasured']
        self.assertEqual(missing[0]['rule'], 'edge floor')
        self.assertEqual(missing[0]['declared'], {'min': .75})
        self.assertIn('not evaluated', missing[0]['reason'])
        self.assertEqual(digest(out.with_suffix('.kicad_dru')), identity)
        before = digest(out)
        self.cli(out, 'rotate', 'Y1', '0', '--relative', expected=4,
                 extra=('--strict-legal',), output=out)
        self.assertEqual(digest(out), before)
        # A copper-only custom rule is not an unmeasured edge requirement.
        dru.write_text('(version 1)\n(rule "copper" '
                       '(constraint clearance (min 0.25mm)))\n', encoding='utf-8')
        s, _ = self.cli(board, 'set', 'Y1', '124.7', '103.15', '--rot', '270')
        self.assertTrue(s['legal'])
        self.assertEqual(s['pad_edge_after']['rules_unmeasured'], [])

    def test_open_edge_is_not_hidden_by_closed_rectangle(self):
        board = self.fixture('control')
        text = board.read_text(encoding='utf-8')
        extra = ('(gr_line (start 122 105.1) (end 127 105.1) '
                 '(stroke (width 0.05) (type default)) (layer "Edge.Cuts"))\n')
        board.write_text(text[:text.rfind(')')] + extra + ')\n', encoding='utf-8')
        s, _ = self.cli(board, 'rotate', 'Y1', '0', '--relative')
        self.assertTrue(s['no_worse'])
        self.assertFalse(s['legal'])
        self.assertFalse(s['pad_edge_after']['complete'])
        self.assertTrue(s['pad_edge_after']['unmeasured'])
        # Subdividing an actual rectangle side still supplies full coverage.
        from placement.legality import _segments_cover_rectangle
        edges = [((0, 0), (2, 0)), ((2, 0), (4, 0)), ((4, 0), (4, 3)),
                 ((4, 3), (0, 3)), ((0, 3), (0, 0))]
        self.assertTrue(_segments_cover_rectangle(edges, (0, 0, 4, 3)))
        self.assertFalse(_segments_cover_rectangle(edges[:-1], (0, 0, 4, 3)))
        self.assertFalse(_segments_cover_rectangle(edges + [edges[0]], (0, 0, 4, 3)))

    def test_invalid_floors_and_malformed_rules(self):
        board = self.fixture('control')
        for floor in ('nan', 'inf', '-inf', '-1'):
            s, out = self.cli(board, 'rotate', 'Y1', '0', '--relative', floor=floor,
                              expected=2, extra=('--strict-legal',))
            self.assertIn('finite and nonnegative', s['refused'])
            self.assertFalse(out.exists())
        valid = '(version 1) (rule "edge" (constraint edge_clearance (min .75mm)))'
        for text in (valid[:-1], valid + ')', '(version 1) (rule "unterminated',
                     '(' + valid + ')', valid.replace('"edge"', '")"')):
            board.with_suffix('.kicad_dru').write_text(text, encoding='utf-8')
            s, _ = self.cli(board, 'rotate', 'Y1', '0', '--relative',
                            expected=4, extra=('--strict-legal',))
            self.assertFalse(s['pad_edge_after']['complete'])
            self.assertIn('unreadable', s['pad_edge_after']['rules_unmeasured'][0]['reason'])
        board.with_suffix('.kicad_dru').unlink()
        for value in (float('nan'), -1):
            board.with_suffix('.kicad_pro').write_text(json.dumps({
                'board': {'design_settings': {'rules': {'min_copper_edge_clearance': value}}}
            }), encoding='utf-8')
            s, _ = self.cli(board, 'rotate', 'Y1', '0', '--relative', floor=None,
                            expected=4, extra=('--strict-legal',))
            self.assertIn('finite and nonnegative',
                          s['pad_edge_after']['rules_unmeasured'][0]['reason'])
        from design_rules import validate_dru_structure
        validate_dru_structure('# ignored )\n(version 1) (rule "name (quoted)" '
                               '(constraint clearance (min .25mm)))')

    def test_strict_near_requires_clean_candidates(self):
        board = self.fixture()
        for y in ('103.21', '103.2'):
            s, out = self.cli(board, 'set', 'Y1', '--near', '124.7', y, '--rot', '270',
                              extra=('--strict-legal', '--radius', '.5', '--snap-step', '.05',
                                     '--snap-tries', '24'))
            self.assertTrue(s['legal'])
            self.assertTrue(s['snapped'])
            measured = grade_pad_legality(parse_kicad_pcb(str(out)), .25, edge_margin=.55,
                                          pcb_file=str(out))
            self.assertEqual(measured['pad_edge_conflicts'], 0)

    def test_output_requirements_and_late_write_rollback(self):
        board = self.fixture('control')
        out = self.work/'existing.kicad_pcb'
        copy_board(str(board), str(out))
        rule = out.with_suffix('.kicad_dru')
        rule.write_text('(version 1) (rule "edge" (constraint edge_clearance (min .75mm)))')
        identities = digest(out), digest(rule)
        s, _ = self.cli(board, 'rotate', 'Y1', '0', '--relative', expected=2,
                        extra=('--strict-legal',), output=out)
        self.assertIn('requirement siblings absent', s['refused'])
        self.assertEqual((digest(out), digest(rule)), identities)
        rule.unlink()
        src_brief = board.with_suffix('.design-brief.json')
        dst_brief = out.with_suffix('.design-brief.json')
        src_brief.write_text('{"requirement": "input"}')
        dst_brief.write_text('{"requirement": "prior output"}')
        identities = digest(out), digest(dst_brief)
        if os.name == 'nt':
            out.chmod(stat.S_IREAD)
            try:
                s, _ = self.cli(board, 'rotate', 'Y1', '0', '--relative', expected=2, output=out)
                self.assertEqual(s['output_state'], 'unchanged')
                self.assertEqual(s['rollback_errors'], [])
            finally:
                out.chmod(stat.S_IREAD | stat.S_IWRITE)
            self.assertEqual((digest(out), digest(dst_brief)), identities)
        # Portable fault injection reaches the same late replacement failure;
        # a second fault tests honest disclosure when rollback also fails.
        from placement.pose_ops import _promote, PoseRefusal
        real_replace = os.replace
        for fail_restore in (False, True):
            def replace(src, dst):
                if str(dst) == str(out) or (fail_restore and '.krt-backup-' in str(src)):
                    raise OSError('injected replacement failure')
                return real_replace(src, dst)
            with patch('placement.pose_ops.os.replace', side_effect=replace):
                with self.assertRaises(PoseRefusal) as refusal:
                    _promote(str(board), str(out))
            summary = refusal.exception.extra['summary']
            self.assertEqual(summary['output_state'], 'partial' if fail_restore else 'unchanged')
            if fail_restore:
                self.assertNotIn('Nothing was written', summary['refused'])
                backup = Path(summary['rollback_errors'][0]['backup'])
                self.assertTrue(backup.is_file())
                real_replace(backup, dst_brief)
            self.assertEqual((digest(out), digest(dst_brief)), identities)
        self.assertFalse(list(self.work.glob('.krt-backup-*')))


if __name__ == '__main__':
    unittest.main()
