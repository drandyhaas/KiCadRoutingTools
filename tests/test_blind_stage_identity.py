#!/usr/bin/env python3
"""A blind work dir must not carry the source board's NAME.

The staged board has to travel with its project or it grades at the stock
netclass (#441). But KiCad writes `meta.filename` into that project, and
`pcbnew.last_paths` holds the author's own directories, so a verbatim copy
puts the source's identity in the work dir. A `.kicad_pro` carries no poses,
so nothing that looks for poses ever noticed.

Two halves are pinned here, because a fix on one side alone is unfalsifiable:

  * `stage_blind.sanitize_staged_project` withholds the identity and DECLARES
    what it withheld.
  * `fence_audit._names_control` catches it from the other side, so a work dir
    staged by some other means is still audited.

The stem sweep is content shaped rather than key shaped on purpose. Measured
on tigard, the leak was under `kicad_routing_tools.floor_provenance` -- prose
this repo writes itself, in a node no key list contained -- naming the source
board and linking the URL that serves its original placement. A key list
cannot anticipate the next carrier; a string that spells the source's stem is
a path back to the poses whatever key it sits under.

The other half of that trade is that NUMBERS must survive: every value the
#441 floor is graded at is numeric, and redacting one would move the grading
goalposts rather than hide a name.

    python3 -X utf8 tests/test_blind_stage_identity.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (ROOT, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools'),
           os.path.join(ROOT, 'tests', 'stress')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fence_audit                                              # noqa: E402
import stage_blind                                              # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

#: A project shaped like the ones that leaked: the KiCad-written identity, the
#: author's directories, and this repo's OWN provenance annotation, which is
#: the carrier no key list contained.
SOURCE_STEM = 'watchy'
PROJECT = {
    'meta': {'filename': SOURCE_STEM + '.kicad_pro', 'version': 1},
    'board': {
        'design_settings': {
            'rules': {'min_clearance': 0.13, 'min_track_width': 0.15},
            'drc_exclusions': ['clearance|123456|789012|00000000'],
        },
        'viewports': [{'name': 'MCU corner', 'center': [110.0, 90.0]}],
        '3dviewports': [],
        'layer_presets': [],
    },
    'pcbnew': {'last_paths': {'gencad': '/home/someone/boards/' + SOURCE_STEM}},
    'kicad_routing_tools': {
        'floor_provenance': {
            '_note': SOURCE_STEM + ' is a KiCad 5 design, so the floor was '
                     'resolved from its netclass',
            'source': 'https://raw.githubusercontent.com/x/y/main/'
                      + SOURCE_STEM + '.kicad_pcb',
            'min_clearance': 0.13,
        },
    },
}


def _stage(td):
    """A staged board plus the leaky project and a .kicad_prl beside it."""
    out = os.path.join(td, 'board.kicad_pcb')
    shutil.copyfile(BOARD, out)
    pro = os.path.join(td, 'board.kicad_pro')
    with open(pro, 'w', encoding='utf-8') as f:
        json.dump(PROJECT, f, indent=2)
    prl = os.path.join(td, 'board.kicad_prl')
    with open(prl, 'w', encoding='utf-8') as f:
        json.dump({'meta': {'filename': SOURCE_STEM + '.kicad_prl'}}, f)
    return out, pro, prl


class TestSanitizeStagedProject(unittest.TestCase):
    def test_the_source_name_is_gone_from_every_carrier(self):
        with tempfile.TemporaryDirectory() as td:
            out, pro, prl = _stage(td)
            decl = stage_blind.sanitize_staged_project(out, SOURCE_STEM)
            self.assertTrue(decl['carried'], decl)
            raw = open(pro, encoding='utf-8').read()
            self.assertNotIn(SOURCE_STEM, raw,
                             'the work dir project still names the source')
            self.assertFalse(os.path.exists(prl),
                             '.kicad_prl carries the name the same way and '
                             'nothing in this repo reads it')
            self.assertTrue(decl['dropped_prl'])

    def test_it_declares_what_it_withheld(self):
        """A silent sanitiser cannot be told apart from one that did nothing."""
        with tempfile.TemporaryDirectory() as td:
            out, _pro, _prl = _stage(td)
            decl = stage_blind.sanitize_staged_project(out, SOURCE_STEM)
            keys = decl['sanitized_keys']
            self.assertIn('meta.filename', keys)
            self.assertIn('board.design_settings.drc_exclusions', keys)
            self.assertIn('board.viewports', keys)
            self.assertIn('pcbnew.last_paths', keys)
            # The content-shaped backstop, which is the half a key list
            # cannot supply.
            self.assertTrue(
                any('floor_provenance' in k
                    for k in decl['redacted_source_strings']),
                decl['redacted_source_strings'])
            self.assertIn('sha256', decl)

    def test_numbers_survive_so_the_441_floor_still_grades(self):
        with tempfile.TemporaryDirectory() as td:
            out, pro, _prl = _stage(td)
            stage_blind.sanitize_staged_project(out, SOURCE_STEM)
            doc = json.load(open(pro, encoding='utf-8'))
            rules = doc['board']['design_settings']['rules']
            self.assertEqual(rules['min_clearance'], 0.13)
            self.assertEqual(rules['min_track_width'], 0.15)
            self.assertEqual(
                doc['kicad_routing_tools']['floor_provenance']['min_clearance'],
                0.13, 'a numeric floor must never be redacted')

    def test_a_project_that_never_named_the_source_is_left_alone(self):
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'board.kicad_pcb')
            shutil.copyfile(BOARD, out)
            pro = os.path.join(td, 'board.kicad_pro')
            clean = {'meta': {'filename': 'board.kicad_pro'},
                     'board': {'design_settings':
                               {'rules': {'min_clearance': 0.2}}}}
            with open(pro, 'w', encoding='utf-8') as f:
                json.dump(clean, f, indent=2)
            decl = stage_blind.sanitize_staged_project(out, SOURCE_STEM)
            self.assertEqual(decl['sanitized_keys'], [], decl)
            self.assertEqual(
                json.load(open(pro, encoding='utf-8'))['board'],
                clean['board'])


class TestTheSweepDoesNotEatTheSPEC(unittest.TestCase):
    """The content-shaped backstop has to stay a backstop.

    A blind sweep over every string in a project will happily redact a
    NETCLASS NAME, which changes what the board is graded at -- the #441
    ratchet by another route. Measured on a routed project: stem `default`
    destroyed the Default netclass name, `error` and `warning` destroyed 30
    and 19 `rule_severities` values, `a` hit 26 and `n` 29.
    """

    def _sweep(self, doc, stem):
        d = json.loads(json.dumps(doc))
        hits = stage_blind._redact_source_stem(d, stem.lower())
        return d, hits

    SPEC = {
        'net_settings': {'classes': [{'name': 'Default', 'clearance': 0.2},
                                     {'name': 'tigard_hs', 'clearance': 0.1}]},
        'board': {'design_settings': {
            'rule_severities': {'clearance': 'error',
                                'silk_overlap': 'warning'},
            # OUTSIDE the excluded subtrees, so only the LENGTH guard protects
            # these. A 3-char stem like `pro` is a substring of half the
            # vocabulary a project file uses, and `pro` is exactly the stem
            # that ate `meta.filename`.
            'defaults': {'zone_fill_strategy': 'preserve_outline',
                         'other_text_upright': 'inherit'},
        }},
    }

    def test_a_short_stem_is_a_substring_not_evidence(self):
        for stem in ('a', 'n', 'pro', 'b2'):
            d, hits = self._sweep(self.SPEC, stem)
            self.assertEqual(hits, [], f'stem {stem!r} redacted {hits}')
            self.assertEqual(d, self.SPEC)

    def test_the_spec_subtrees_are_never_entered(self):
        """`default` is 7 chars, so only the exclusion saves the netclass."""
        d, hits = self._sweep(self.SPEC, 'default')
        self.assertEqual(hits, [], hits)
        self.assertEqual(d['net_settings']['classes'][0]['name'], 'Default')
        d2, hits2 = self._sweep(self.SPEC, 'warning')
        self.assertEqual(hits2, [], hits2)
        self.assertEqual(
            d2['board']['design_settings']['rule_severities']['silk_overlap'],
            'warning')

    def test_a_real_carrier_outside_those_subtrees_is_still_swept(self):
        """Or the guards above have turned the backstop off."""
        doc = dict(self.SPEC, kicad_routing_tools={
            'floor_provenance': {'_note': 'tigard is a KiCad 5 design',
                                 'min_clearance': 0.13}})
        d, hits = self._sweep(doc, 'tigard')
        self.assertTrue(hits, 'the backstop stopped working')
        self.assertEqual(
            d['kicad_routing_tools']['floor_provenance']['min_clearance'],
            0.13, 'numbers must survive')
        # ...and the netclass named after the source is still spared, because
        # a netclass NAME is spec the board grades at.
        self.assertEqual(d['net_settings']['classes'][1]['name'], 'tigard_hs')

    def test_the_sweep_runs_before_meta_filename_is_set(self):
        """Otherwise a stem that matches the NEW name eats the value."""
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'board.kicad_pcb')
            shutil.copyfile(BOARD, out)
            pro = os.path.join(td, 'board.kicad_pro')
            with open(pro, 'w', encoding='utf-8') as f:
                json.dump({'meta': {'filename': 'boarding.kicad_pro'}}, f)
            stage_blind.sanitize_staged_project(out, 'boarding')
            self.assertEqual(
                json.load(open(pro, encoding='utf-8'))['meta']['filename'],
                'board.kicad_pro')

    def test_the_sibling_probe_is_not_a_report_on_its_own_side_effect(self):
        with tempfile.TemporaryDirectory() as td:
            out, _pro, prl = _stage(td)
            self.assertTrue(os.path.isfile(prl))
            decl = stage_blind.sanitize_staged_project(out, SOURCE_STEM)
            self.assertTrue(decl['dropped_prl'])
            self.assertTrue(decl['kicad_prl'],
                            'the .kicad_prl probe ran after the remove, so it '
                            'could only ever report False')


class TestFenceAuditSeesIt(unittest.TestCase):
    """The audit half, driven through `main()`.

    Calling `_names_control` directly proves the STRING SEARCH works and
    nothing else. It cannot see the branch that decides what the finding
    means, which is where this went wrong: identity rows fell through every
    pose-shaped branch of the audit chain to the `else`, and `--mode audit`
    printed the leak as
    `ok board.kicad_pro: matches the control (1.000) -- produced by the run`.
    `audit` is the watcher-time mode, so that is the mode a real recovery run
    uses.
    """

    def _fence(self, td, mode, allow=()):
        """Run fence_audit.main over a work dir. Returns (exit, stdout)."""
        wk, truth = os.path.join(td, 'wk'), os.path.join(td, 'truth')
        argv = ['--workdir', wk, '--mode', mode,
                '--control', os.path.join(truth, 'control.kicad_pcb')]
        for a in allow:
            argv += ['--allow', a]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = fence_audit.main(argv)
        return code, buf.getvalue()

    def _workdir(self, td, project):
        """A staged work dir whose board is declared, so only the project can
        be the finding."""
        wk, truth = os.path.join(td, 'wk'), os.path.join(td, 'truth')
        os.makedirs(wk)
        os.makedirs(truth)
        shutil.copyfile(BOARD, os.path.join(truth, 'control.kicad_pcb'))
        with open(os.path.join(truth, 'draw.json'), 'w',
                  encoding='utf-8') as f:
            json.dump({'source': SOURCE_STEM + '.kicad_pcb'}, f)
        shutil.copyfile(BOARD, os.path.join(wk, 'board.kicad_pcb'))
        with open(os.path.join(wk, 'board.kicad_pro'), 'w',
                  encoding='utf-8') as f:
            json.dump(project, f)

    def test_audit_reports_an_identity_leak_as_a_leak(self):
        with tempfile.TemporaryDirectory() as td:
            self._workdir(td, {'meta': {'filename': 'board.kicad_pro'}})
            # A CLEAN project emits no row, so it can never be in the
            # creation manifest -- which is why the manifest cannot excuse a
            # name that arrives later.
            self._fence(td, 'create')
            with open(os.path.join(td, 'wk', 'board.kicad_pro'), 'w',
                      encoding='utf-8') as f:
                json.dump({'meta': {'filename': SOURCE_STEM + '.kicad_pro'}}, f)
            code, out = self._fence(td, 'audit', allow=['board.kicad_pcb'])
            self.assertIn('LEAK board.kicad_pro', out, out)
            self.assertIn('NAMES THE SOURCE', out, out)
            self.assertNotIn('ok  board.kicad_pro', out, out)
            self.assertEqual(code, 4, out)

    def test_a_clean_project_is_not_a_finding(self):
        with tempfile.TemporaryDirectory() as td:
            self._workdir(td, {'meta': {'filename': 'board.kicad_pro'},
                               'board': {'design_settings':
                                         {'rules': {'min_clearance': 0.2}}}})
            self._fence(td, 'create')
            code, out = self._fence(td, 'audit', allow=['board.kicad_pcb'])
            self.assertNotIn('board.kicad_pro', out, out)
            self.assertEqual(code, 0, out)

    def test_a_project_carries_no_pose_score(self):
        """It read `1.000`, which the report prints as "matches the control".

        A fabricated pose match on a file with no poses is how the row read
        as a successful recovery.
        """
        with tempfile.TemporaryDirectory() as td:
            self._workdir(td, {'meta': {'filename': SOURCE_STEM + '.kicad_pro'}})
            wk = os.path.join(td, 'wk')
            rows = fence_audit.scan(wk, {}, [], None, control_stem=SOURCE_STEM)
            pro = [r for r in rows if r['file'].endswith('.kicad_pro')]
            self.assertEqual(len(pro), 1, rows)
            self.assertIsNone(pro[0]['match_frac'])
            self.assertEqual(pro[0]['truth_channel'], 'identity')

    def test_the_sanitised_project_passes_the_same_audit(self):
        """End to end: the stager's output survives the auditor."""
        with tempfile.TemporaryDirectory() as td:
            self._workdir(td, PROJECT)
            wk = os.path.join(td, 'wk')
            stage_blind.sanitize_staged_project(
                os.path.join(wk, 'board.kicad_pcb'), SOURCE_STEM)
            self._fence(td, 'create')
            code, out = self._fence(td, 'audit', allow=['board.kicad_pcb'])
            self.assertEqual(code, 0, out)

    def test_the_needle_is_case_insensitive(self):
        """The stager's sweep is; a title block that says `Tigard` is not a
        different board from `tigard`, and the two halves of one fence must
        not disagree about the same file."""
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, 'x.kicad_pro')
            with open(p, 'w', encoding='utf-8') as f:
                json.dump({'meta': {'filename': 'Watchy.kicad_pro'}}, f)
            self.assertTrue(fence_audit._names_control(p, SOURCE_STEM))

    def test_an_undecodable_byte_does_not_read_as_clean(self):
        """A bare `except: return []` reports CLEAN for a file it could not
        read, which is the defeat `_json_poses` was hardened against."""
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, 'x.kicad_pro')
            with open(p, 'wb') as f:
                f.write(b'{"meta": {"filename": "' + bytes([0xff, 0xfe]) + b'watchy.kicad_pro"}}')
            self.assertTrue(fence_audit._names_control(p, SOURCE_STEM))

    def test_a_project_is_scanned_at_all(self):
        """`.kicad_pro` has to be in the walked set or the check is inert."""
        self.assertIn('.kicad_pro', fence_audit.SCANNED_EXT)


class TestTheProductionPathRecordsIt(unittest.TestCase):
    """`stage_blind.main()` is where "declares what it withheld" is a claim.

    Calling `sanitize_staged_project` directly exercises the RETURN VALUE, and
    the return value was computed and dropped: nothing wrote it anywhere, so a
    run could not tell a sanitised work dir from an unsanitised one, and a
    unit test that calls the function passes either way. The declaration goes
    into the TRUTH dir -- naming the withheld strings inside the fence would
    be the leak itself.
    """

    def _stage_via_main(self, td):
        src = os.path.join(td, SOURCE_STEM + '.kicad_pcb')
        shutil.copyfile(BOARD, src)
        with open(os.path.join(td, SOURCE_STEM + '.kicad_pro'), 'w',
                  encoding='utf-8') as f:
            json.dump(PROJECT, f)
        wk, truth = os.path.join(td, 'wk'), os.path.join(td, 'truth')
        with contextlib.redirect_stdout(io.StringIO()):
            stage_blind.main(src, wk, truth)
        return wk, truth

    def test_draw_json_carries_the_declaration(self):
        with tempfile.TemporaryDirectory() as td:
            _wk, truth = self._stage_via_main(td)
            with open(os.path.join(truth, 'draw.json'), encoding='utf-8') as f:
                draw = json.load(f)
            self.assertIn('staged_project', draw,
                          'the declaration was computed and thrown away')
            decl = draw['staged_project']
            self.assertTrue(decl['carried'])
            self.assertIn('meta.filename', decl['sanitized_keys'])
            self.assertIn('floors', decl)

    def test_and_the_work_dir_does_not_name_the_source(self):
        """End to end through the production path, checked by the auditor."""
        with tempfile.TemporaryDirectory() as td:
            wk, _truth = self._stage_via_main(td)
            checked = 0
            for root, _dirs, files in os.walk(wk):
                for name in files:
                    if not name.endswith('.kicad_pro'):
                        continue
                    checked += 1
                    self.assertEqual(
                        fence_audit._names_control(os.path.join(root, name),
                                                   SOURCE_STEM), [],
                        'the staged project still names the source')
            self.assertTrue(checked, 'no project reached the work dir at all, '
                                     'so this proved nothing')


if __name__ == '__main__':
    unittest.main(verbosity=1)
