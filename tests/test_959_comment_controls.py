#!/usr/bin/env python3
"""#959: the follow-up comment's own controls, asserting the AFTER state.

The comment (2026-09-13) measured the declare stage with a harness
(`wk/astra-evidence/requirements/reproduce.py` and `mechanical_probe.py` at
commit e65e33da). This ports its controls into temp dirs and asserts what
they now show:

  * section 1: `carried_changed` (user_top_side, mount_mode and cable_entry
    changed) measured NOTHING different from the base -- drift 0, complete
    true. It now drifts, on the clause ids of what changed, and coverage is
    incomplete; `edge_changed` still drifts exactly once.
  * section 1: `complete: true` sat beside 8 carried facts. The ledger now
    names the carried and unmeasured facts beside it, and the derived
    defaults apart from both.
  * section 2: fixture 902 fails at its declared 2 mm and passes at a loose
    100 mm, on the same board hash -- unchanged, and pinned.
  * the stale-provenance control: a legality budget with no `overlap_area`
    and a withheld note is OWED at P1 (`rules_dark_undispositioned`); a
    declared 0.0 is not.
  * `mechanical_probe`: an absent and a contradictory `mechanical.json` gave
    BYTE-IDENTICAL intents. They now differ, and the contradictions are
    named with both values and their authority.

The comment's hashes were taken on a CRLF checkout (esp_prog 165302e6...,
fixture 711 c1869c27...); the LF blob hashes pinned here are the same files.
"""
import copy
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tests'))

import run_utils                                            # noqa: E402

RUN_ALL_TIMEOUT = 900

BOARD = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
BRIEF = os.path.join(REPO, 'tests', 'fixtures', '711',
                     'esp_prog.design-brief.json')
PROX = os.path.join(REPO, 'tests', 'fixtures', '902',
                    'esp_prog_proximity.design-brief.json')
FLAT = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
CF = run_utils.tool('check_floorplan.py')

#: LF blob sha256 of the comment's inputs.
IDENTITY = {
    BOARD: 'a9945bb0940f79672b7c6e32b7a6b9d0b135bf78030e19fcdb88e65c2139903f',
    BRIEF: 'befc78d1bbb14d1d362792f478c17f125a33a0f2c87e5fbad9646ec13c085359',
    PROX: 'd498edc01442b0c931771146c581f8134b274c57d15f3a01b60b2c9a0bf73c7c',
    FLAT: 'b6eb7016ba4d04135cb8754fd4793091a316808f8d8d9493e734d4248e814e17',
}


def _sha_lf(path):
    with open(path, 'rb') as fh:
        return hashlib.sha256(fh.read().replace(b'\r\n', b'\n')).hexdigest()


def _cf(*args, code=(0, 4)):
    """check_floorplan, returning (rc, JSON_SUMMARY); an emit prints no
    summary, so its evidence is the file it wrote."""
    r = subprocess.run([sys.executable, '-X', 'utf8', CF] + [str(a) for a in
                                                             args],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO, timeout=600)
    line = [x for x in r.stdout.splitlines() if x.startswith('JSON_SUMMARY:')]
    emit = '--emit-intent' in args
    assert r.returncode in code and (line or emit), (
        args, r.returncode, r.stdout[-1500:], r.stderr[-1500:])
    if emit:
        run_utils.evidence(str(args[args.index('--emit-intent') + 1]),
                           'the emitted intent')
    return r.returncode, (json.loads(line[-1].split('JSON_SUMMARY: ', 1)[1])
                          if line else None)


def _dump(path, doc):
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=2)


def test_the_inputs_are_the_comments():
    for path, want in IDENTITY.items():
        run_utils.evidence(path, 'a control input')
        assert _sha_lf(path) == want, (path, _sha_lf(path))
    print("  PASS: board and briefs are the files the comment measured")


def test_section_1_the_carried_fields_now_measure_something():
    with tempfile.TemporaryDirectory() as tmp:
        intent = os.path.join(tmp, 'esp_observed.json')
        _cf(BOARD, '--emit-intent', intent, '--declare-classes', '--brief',
            BRIEF, '--no-mechanical')
        grades = {}
        with open(BRIEF, encoding='utf-8') as fh:
            original = json.load(fh)
        for tag in ('observed', 'carried_changed', 'edge_changed'):
            brief = BRIEF
            if tag != 'observed':
                v = copy.deepcopy(original)
                if tag == 'carried_changed':
                    v['product']['user_top_side'] = 'B'
                    v['interfaces'][0]['mount_mode'] = 'through_edge'
                    v['interfaces'][0]['cable_entry'] = 'perpendicular_top'
                else:
                    v['interfaces'][0]['edge'] = 'west'
                brief = os.path.join(tmp, tag + '.design-brief.json')
                _dump(brief, v)
            js = os.path.join(tmp, f'grade_{tag}.json')
            _rc, s = _cf(BOARD, '--intent', intent, '--brief', brief,
                         '--require-brief-coverage', '--json', js,
                         '--no-mechanical')
            with open(js, encoding='utf-8') as fh:
                grades[tag] = (s, json.load(fh))
    s, doc = grades['observed']
    assert (s['rules_run'], s['brief_declared'], s['brief_clauses'],
            s['brief_drift'], s['brief_coverage_complete']) == \
        (6, 16, 19, 0, True), s
    # `complete` no longer stands alone: what was NOT physically checked is
    # named beside it, and the defaults this code chose are listed apart.
    assert s['carried_facts'] == [
        'interfaces[USB1].cable_entry', 'product.form_factor',
        'product.held_by', 'product.primary_axis'], s['carried_facts']
    assert set(s['unmeasured_facts']) == {
        'derived:interfaces[CON2].cable_envelope_mm',
        'derived:interfaces[USB1].cable_envelope_mm'}, s['unmeasured_facts']
    assert set(s['derived_default_clauses']) == {
        'derived:interfaces[CON2].mount_mode',
        'derived:interfaces[USB1].mount_mode'}, s['derived_default_clauses']
    assert doc['brief_coverage']['graded'] == 12, doc['brief_coverage']
    # The comment's row: drift 0, complete true -- indistinguishable from
    # the base. Now: drift on the clause ids of what changed.
    s, doc = grades['carried_changed']
    assert s['brief_drift'] == 2 and not s['brief_coverage_complete'], s
    drifted = {c['id'] for c in doc['brief_coverage']['clauses']
               if c['drifted']}
    assert drifted == {'interfaces[CON2].cable_entry',
                       'interfaces[USB1].cable_entry'}, drifted
    s, _doc = grades['edge_changed']
    assert s['brief_drift'] == 1 and not s['brief_coverage_complete'], s
    print("  PASS: carried_changed drifts on CON2/USB1 cable_entry (was 0); "
          "edge_changed still drifts once; the carried, unmeasured and "
          "derived-default facts are named beside `complete`")


def test_section_2_fixture_902_fails_tight_and_passes_loose():
    with tempfile.TemporaryDirectory() as tmp:
        intent = os.path.join(tmp, 'esp_proximity.json')
        _cf(BOARD, '--emit-intent', intent, '--brief', PROX,
            '--no-mechanical')
        rc, s = _cf(BOARD, '--intent', intent, '--brief', PROX,
                    '--require-brief-coverage', '--no-mechanical')
        assert rc == 4 and s['errors'] == 3, s
        with open(intent, encoding='utf-8') as fh:
            doc = json.load(fh)
        for row in doc['proximity']:
            row['max_mm'] = 100.0
        relaxed_i = os.path.join(tmp, 'esp_proximity_relaxed.json')
        _dump(relaxed_i, doc)
        with open(PROX, encoding='utf-8') as fh:
            rb = json.load(fh)
        for row in rb['proximity']:
            row['max_mm'] = 100.0
        relaxed_b = os.path.join(tmp, 'proximity_relaxed.design-brief.json')
        _dump(relaxed_b, rb)
        rc, s = _cf(BOARD, '--intent', relaxed_i, '--brief', relaxed_b,
                    '--require-brief-coverage', '--no-mechanical')
        assert rc == 0 and s['errors'] == 0, s
    assert _sha_lf(BOARD) == IDENTITY[BOARD]
    print("  PASS: 3 proximity failures at the declared 2 mm, 0 at 100 mm, "
          "same board")


def test_stale_provenance_is_owed_and_a_declared_zero_is_not():
    with tempfile.TemporaryDirectory() as tmp:
        emitted = os.path.join(tmp, 'flat_observed.json')
        _cf(FLAT, '--emit-intent', emitted, '--no-mechanical')
        with open(emitted, encoding='utf-8') as fh:
            doc = json.load(fh)
        doc.setdefault('legality_budget', {}).pop('overlap_area', None)
        doc['context'].setdefault('budget_withheld', {})['overlap_area'] = (
            '83 blocking body pair(s) on the emitting board (controlled '
            'stale provenance)')
        stale = os.path.join(tmp, 'flat_stale.json')
        _dump(stale, doc)
        _rc, s = _cf(FLAT, '--intent', stale, '--no-mechanical')
        assert 'legality' in s['rules_dark_undispositioned'], s
        doc['legality_budget']['overlap_area'] = 0.0
        zero = os.path.join(tmp, 'flat_declared_zero.json')
        _dump(zero, doc)
        _rc, s = _cf(FLAT, '--intent', zero, '--no-mechanical')
        assert 'legality' not in s['rules_dark_undispositioned'], s
    print("  PASS: a withheld overlap budget is owed at P1; a declared 0.0 "
          "answers it")


def test_mechanical_probe_absent_and_contradictory_now_differ():
    with tempfile.TemporaryDirectory() as tmp:
        case = os.path.join(tmp, 'mechanical_case')
        os.makedirs(case)
        board = os.path.join(case, 'esp_prog.kicad_pcb')
        shutil.copy2(BOARD, board)
        shutil.copy2(BRIEF, os.path.join(case, 'esp_prog.design-brief.json'))
        out = {}
        for tag in ('absent', 'contradictory'):
            mech = os.path.join(case, 'mechanical.json')
            if tag == 'contradictory':
                _dump(mech, {'interfaces': [{'ref': 'USB1', 'edge': 'west'}],
                             'fixed': [{'ref': 'C1', 'x': 0, 'y': 0,
                                        'reason': 'controlled deliberately '
                                                  'incompatible declaration'}],
                             'project': {'floors': {'clearance': 0.4}}})
            intent = os.path.join(tmp, f'mechanical_{tag}.json')
            _cf(board, '--emit-intent', intent, '--declare-classes')
            with open(intent, 'rb') as fh:
                raw = fh.read()
            out[tag] = (hashlib.sha256(raw).hexdigest(),
                        json.loads(raw.decode('utf-8')))
    assert out['absent'][0] != out['contradictory'][0]
    ctx = out['contradictory'][1]['context']
    contra = ctx['brief']['contradictions']
    assert any(c.startswith('CONTRADICTION USB1:edge') and "'west'" in c
               and '[recorded_fact]' in c for c in contra), contra
    assert any(c.startswith('CONTRADICTION C1:on_board') for c in contra), \
        contra
    assert out['absent'][1]['context']['brief']['contradictions'] == [], \
        out['absent'][1]['context']['brief']['contradictions']
    rows = {r['id']: r for r in ctx['reconciliation']}
    fl = rows['floors:clearance']
    assert fl['kind'] == 'report' and fl['values']['mechanical'][
        'value'] == 0.4 and fl['values']['graded']['value'] == 0.25, fl
    assert ctx['mechanical']['provenance'] == 'unverified', ctx['mechanical']
    print("  PASS: absent and contradictory mechanical.json give different "
          "intents; USB1:edge and C1:on_board are named with their "
          "authority; the 0.4 floor is reported beside the 0.25 graded")


TESTS = [
    test_the_inputs_are_the_comments,
    test_section_1_the_carried_fields_now_measure_something,
    test_section_2_fixture_902_fails_tight_and_passes_loose,
    test_stale_provenance_is_owed_and_a_declared_zero_is_not,
    test_mechanical_probe_absent_and_contradictory_now_differ,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
