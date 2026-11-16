"""#1901: `check_complete --kicad-rules` read a rule `met` when KiCad could
not report it.

1. A violation type the project sets to `ignore` gets no row from KiCad, so
   a rule reporting under it had 0 rows and read met. The staged copy KiCad
   grades now sets each such type a judged rule reports under
   (`check_complete._KICAD_ROW_TYPES`) to `error`, and the grade lists them
   (`unignored_types`). A type at `warning` is left alone: KiCad lists those
   rows, and a warning row counts 0 as before (#1626).
2. A diff-pair gap or uncoupled-length row names its rule bare --
   `Differential pair gap out of range (NAME minimum gap ...` -- not as
   `(rule 'NAME'`, so no row was counted against such a rule. Rows are now
   read by `check_complete._row_rule_name`.

Part 1 injects KiCad's report and needs no KiCad. Part 2 asks kicad-cli on
the probe board of `test_1865_capped_type_scope.py`; without kicad-cli it is
skipped and the test exits 77 (after part 1 has passed).

    python3 tests/test_1901_kicad_rules_unreported.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
import run_utils  # noqa: E402
import check_complete  # noqa: E402
from test_1865_capped_type_scope import probe_board  # noqa: E402

failures = []

#: the rows kicad-cli 10.0.6 printed for two violated diff-pair rules, one
#: of them named with spaces and brackets
GAP_MIN = {'type': 'diff_pair_gap_out_of_range', 'severity': 'error',
           'description': 'Differential pair gap out of range (pair_gap_probe minimum '
                          'gap 1.0000 mm; actual 0.3000 mm)'}
GAP_MAX = {'type': 'diff_pair_gap_out_of_range', 'severity': 'error',
           'description': 'Differential pair gap out of range (pair gap (max) probe '
                          'maximum gap 0.1000 mm; actual 0.3000 mm)'}
UNCOUPLED = {'type': 'diff_pair_uncoupled_length_too_long', 'severity': 'error',
             'description': 'Differential uncoupled length too long (pair unc probe '
                            'maximum uncoupled length 2.0000 mm; actual 54.8000 mm)'}
LOAD = {'type': 'assertion_failure', 'severity': 'warning',
        'description': "Assertion failure (rule 'check_complete load probe')"}


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def stage(td, name, dru, severities=None, text=None):
    """A board directory: the probe board (or TEXT), DRU, and a project whose
    rule_severities are SEVERITIES (no project when None)."""
    d = os.path.join(td, name)
    os.makedirs(d)
    board = os.path.join(d, 'b.kicad_pcb')
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write(text if text is not None else probe_board())
    with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write(dru)
    if severities is not None:
        with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump({'board': {'design_settings': {'rule_severities': severities}}}, fh)
    return run_utils.evidence(board)


def grade_with(board, rows):
    """`kicad_rule_grade` over an injected report of ROWS -> (grade, the
    staged project as KiCad would have read it, or None: no project edit)."""
    seen = {}
    real = check_complete._kicad_drc_report

    def fake(b, timeout=3600, project_edit=None, rules_edit=None):
        if project_edit is not None:
            pro = os.path.splitext(b)[0] + '.kicad_pro'
            with open(pro, encoding='utf-8') as fh:
                proj = json.load(fh)
            project_edit(proj)
            seen['proj'] = proj
        return {'kicad_version': '10.9.9', 'violations': [LOAD] + list(rows)}
    check_complete._kicad_drc_report = fake
    try:
        return check_complete.kicad_rule_grade(board), seen.get('proj')
    finally:
        check_complete._kicad_drc_report = real


def part1(td):
    print('1a. _row_rule_name: the rule a row names')
    name = getattr(check_complete, '_row_rule_name', None)
    check('check_complete reads a row\'s rule with one function', callable(name))
    if callable(name):
        for row, want in ((GAP_MIN, 'pair_gap_probe'), (GAP_MAX, 'pair gap (max) probe'),
                          (UNCOUPLED, 'pair unc probe'), (LOAD, 'check_complete load probe'),
                          ({'type': 'clearance', 'description':
                            "Clearance violation (rule 'a b' clearance 0.6000 mm; actual "
                            "0.2000 mm)"}, 'a b'),
                          ({'type': 'clearance', 'description':
                            'Clearance violation (netclass \'Default\' clearance 0.2000 mm; '
                            'actual 0.0500 mm)'}, None),
                          ({'type': 'diff_pair_gap_out_of_range', 'description':
                            'Differential pair gap out of range'}, None)):
            got = name(row)
            check(f'{row["description"][:60]!r}... -> {want!r}', got == want, repr(got))

    print('1b. kicad_rule_grade: diff-pair rows count against their rule')
    dru = ('(version 1)\n'
           '(rule "pair_gap_probe" (constraint diff_pair_gap (min 1mm)))\n'
           '(rule "pair unc probe" (constraint diff_pair_uncoupled (max 2mm)))\n')
    board = stage(td, 'rows', dru)
    g, proj = grade_with(board, [GAP_MIN, GAP_MIN, UNCOUPLED])
    check('2 gap rows and 1 uncoupled row, counted per rule',
          g.get('errors_by_rule') == {'pair_gap_probe': 2, 'pair unc probe': 1}, str(g))
    j = check_complete.judge_rules(['pair_gap_probe', 'pair unc probe'], g,
                                   ['pair_gap_probe', 'pair unc probe'],
                                   {'pair_gap_probe': ['diff_pair_gap'],
                                    'pair unc probe': ['diff_pair_uncoupled']})
    check('... so both rules read broken',
          [j[r]['verdict'] for r in ('pair_gap_probe', 'pair unc probe')]
          == ['broken', 'broken'], str(j))
    check('no project: no project edit, nothing unignored',
          proj is None and g.get('unignored_types') == [], str((proj, g)))

    print('1c. kicad_rule_grade: an ignored type is asked at error')
    dru = '(version 1)\n(rule "holes" (constraint physical_hole_clearance (min 1.2mm)))\n'
    sev = {'hole_clearance': 'ignore', 'silk_overlap': 'ignore',
           'text_height': 'warning', 'lib_footprint_issues': 'ignore',
           'track_width': 'error'}
    board = stage(td, 'ignored', dru, sev)
    g, proj = grade_with(board, [])
    got = (((proj or {}).get('board') or {}).get('design_settings') or {}).get(
        'rule_severities') or {}
    check('the ignored types a judged rule reports under are set to error',
          got.get('hole_clearance') == 'error' and got.get('silk_overlap') == 'error',
          str(got))
    check('... and nothing else changes (a warning type, a type no judged rule '
          'reports under, an error type)',
          got.get('text_height') == 'warning' and got.get('lib_footprint_issues') == 'ignore'
          and got.get('track_width') == 'error', str(got))
    check('the grade lists them', g.get('unignored_types') == ['hole_clearance', 'silk_overlap'],
          str(g))
    j = check_complete.judge_rules(['holes'], g, ['holes'],
                                   {'holes': ['physical_hole_clearance']})['holes']
    check('a met rule under such a type says it was asked at error',
          j['verdict'] == 'met' and 'asked with hole_clearance at error' in j['why'], str(j))
    board = stage(td, 'not_ignored', dru, {'hole_clearance': 'warning'})
    g, proj = grade_with(board, [])
    check('a project that ignores none of them is not edited',
          proj is None and g.get('unignored_types') == [], str((proj, g)))


def part2(cli, td):
    print('2a. kicad-cli: a rule under a type the project ignores')
    dru = '(version 1)\n(rule "holes" (constraint physical_hole_clearance (min 1.2mm)))\n'
    verdicts = {}
    for label, sev in (('as shipped', {}), ('hole_clearance ignored', {'hole_clearance': 'ignore'})):
        board = stage(td, label.replace(' ', '_'), dru, sev)
        with contextlib.redirect_stdout(io.StringIO()):
            g = check_complete.kicad_rule_grade(board)
        verdicts[label] = check_complete.judge_rules(
            ['holes'], g, ['holes'], {'holes': ['physical_hole_clearance']})['holes']
    a, b = verdicts['as shipped'], verdicts['hole_clearance ignored']
    check('the rule KiCad finds broken on the board as shipped', a['verdict'] == 'broken'
          and a['kicad_errors'], str(a))
    check('... is broken by the same count when the project ignores its type '
          '(it read met (0 KiCad violations))',
          b['verdict'] == 'broken' and b['kicad_errors'] == a['kicad_errors']
          and 'asked with hole_clearance at error' in b['why'], str(b))

    print('2b. kicad-cli: violated diff-pair rules, named with spaces')
    dru = ('(version 1)\n'
           '(rule "pair gap probe" (constraint diff_pair_gap (min 1mm)) '
           '(condition "A.inDiffPair(\'DP\')"))\n'
           '(rule "pair unc probe" (constraint diff_pair_uncoupled (max 2mm)) '
           '(condition "A.inDiffPair(\'DP\')"))\n')
    board = stage(td, 'pairs', dru)
    with contextlib.redirect_stdout(io.StringIO()):
        g = check_complete.kicad_rule_grade(board)
    names = ['pair gap probe', 'pair unc probe']
    j = check_complete.judge_rules(names, g, names, {'pair gap probe': ['diff_pair_gap'],
                                                     'pair unc probe': ['diff_pair_uncoupled']})
    for nm in names:
        check(f'{nm!r} reads broken (it read met (0 KiCad violations))',
              j[nm]['verdict'] == 'broken' and j[nm]['kicad_errors'], str(j[nm]))


def main():
    td = tempfile.mkdtemp(prefix='krt1901_')
    skipped = False
    try:
        part1(td)
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        if cli:
            part2(cli, td)
        else:
            print('2. SKIPPED: no kicad-cli (set KICAD_CLI)')
            skipped = True
    except AssertionError as exc:
        check('a run_utils assertion held', False, str(exc)[:1500])
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    if skipped:
        print('PART 1 PASS')
        print('SKIP: part 2 needs kicad-cli (none found; set KICAD_CLI) -- what '
              'KiCad reports for an ignored type and a diff-pair rule was not checked')
        return 77
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
