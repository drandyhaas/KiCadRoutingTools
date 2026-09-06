#!/usr/bin/env python3
"""#887: the cmd_timing.jsonl reader that replaces a hand-run watch subagent.

`wk/run24/esp_prog/watch/timing.md` mandated a subagent that read the ledger at
end of run and produced a step table, stage subtotals, totals and the three
longest steps -- an LLM doing arithmetic over a JSONL, once, at the end. This
file grades the script that replaces it.

THIS ARM CARRIES THE WHOLE REGRESSION. `wk/` is gitignored (`.gitignore:66`;
`git ls-files | grep -c '^wk/'` is 0), so the real 153-row run-24 ledger is not
committable and is not committed. Every bucketing rule, every total and every
formatting rule is pinned here against two small TRACKED fixtures, so this file
passes in full on a fresh clone with no `wk/` tree at all.
`tests/test_887_run24_regression.py` corroborates it against the real ledger
when one happens to be present, and self-skips when it is not.

RUN_ALL_FAST_OK because it shells out only for the CLI and the no-Pillow probe;
there is no board and no routing here.
"""
import ast
import json
import os
import subprocess
import sys

RUN_ALL_FAST_OK = True

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

import run_utils                                          # noqa: E402
import cmd_timing as ct                                   # noqa: E402

FIXDIR = os.path.join(TESTS, 'fixtures', 'cmd_timing')
FIX = os.path.join(FIXDIR, 'synthetic_run.jsonl')
OOO = os.path.join(FIXDIR, 'out_of_order.jsonl')
MODULE = os.path.join(ROOT, 'py_router', 'cmd_timing.py')

#: Hand-computed from the fixture table, NOT copied from a run of the code.
#: (stage, steps, wall_s)
WANT_SUBTOTALS = [
    ('staging',   1,  1.0),
    ('P*',        4,  8.0),
    ('L*',        3,  1.0),
    ('R*',        6, 87.0),
    ('V*',        2,  8.7),
    ('close-out', 3,  3.0),
    ('other',     1,  1.0),
]

BAD = []


def want(cond, label, extra=''):
    if cond:
        print('  PASS: %s' % label)
    else:
        BAD.append(label)
        print('  FAIL: %s %s' % (label, extra))


# --------------------------------------------------------------------- rules

def test_the_synthetic_ledger_reproduces_its_declared_subtotals():
    got = ct.subtotals(ct.load_rows(FIX))
    want(got == WANT_SUBTOTALS, 'the seven subtotals are exactly as declared',
         '\n    got  %s\n    want %s' % (got, WANT_SUBTOTALS))


def test_pclose_counts_under_P_not_close_out():
    want(ct.stage_of('Pclose-q-render') == 'P*',
         'Pclose-* is P-prefixed, so it counts under P*')
    want(ct.stage_of('close-drc-final') == 'close-out',
         'close-* counts under close-out')
    p = [r['label'] for r in ct.bucket_rows(ct.load_rows(FIX))['P*']]
    want('Pclose-q-render' in p and 'Pclose-film' in p,
         'both Pclose-* rows land in the P* bucket', p)


def test_fence_audit_start_buckets_under_close_out_though_it_ran_first():
    rows = ct.load_rows(FIX)
    # BOTH clauses matter. The rule is about the LABEL, not the position, and a
    # bucketer that keyed on "everything after the last route step" would pass
    # the first clause alone -- which is why fence-audit-start is row 1 here.
    want(rows[0]['label'] == 'fence-audit-start',
         'the fence row really is FIRST in the ledger', rows[0]['label'])
    want(ct.stage_of('fence-audit-start') == 'close-out',
         'and it still buckets under close-out, per the fence-* rule')


def test_a_repeated_label_is_counted_once_per_entry():
    rows = ct.load_rows(FIX)
    r5 = [r for r in ct.bucket_rows(rows)['R*'] if r['label'] == 'R5-prune']
    want(len(r5) == 3, 'three R5-prune rows are three steps, not one', len(r5))
    l5 = [r for r in ct.bucket_rows(rows)['L*'] if r['label'] == 'L5-final-record']
    want(len(l5) == 2, 'two L5-final-record rows are two steps', len(l5))


def test_an_unmatched_label_lands_in_other_and_other_is_always_reported():
    want(ct.stage_of('route4') == 'other',
         'lowercase route4 is not R* -- matching is case-sensitive')
    stages = [s for s, _, _ in ct.subtotals(ct.load_rows(FIX))]
    want(stages[-1] == 'other', 'other is reported, and reported last', stages)
    # ... and reported even when it is empty, which is what makes the run-24
    # footnote's "the other bucket is empty" a checkable claim rather than an
    # absence nobody can see.
    only_named = [r for r in ct.load_rows(FIX) if ct.stage_of(r['label']) != 'other']
    stages2 = [s for s, _, _ in ct.subtotals(only_named)]
    want('other' in stages2, 'other is still a row when nothing is in it', stages2)


def test_the_subtotals_sum_to_the_tool_total():
    rows = ct.load_rows(FIX)
    s = round(sum(w for _, _, w in ct.subtotals(rows)), 3)
    want(s == round(ct.totals(rows).tool_s, 3),
         'no label can be silently dropped: the buckets sum to the tool total',
         '%s vs %s' % (s, ct.totals(rows).tool_s))


# -------------------------------------------------------------------- totals

def test_outside_the_tools_is_the_gap_and_the_fixture_has_one():
    t = ct.totals(ct.load_rows(FIX))
    want(t.n == 20, 'twenty rows', t.n)
    want(round(t.tool_s, 1) == 109.7, 'tool time 109.7 s', t.tool_s)
    want(round(t.run_s, 1) == 412.0, 'run span 412.0 s', t.run_s)
    want(round(t.outside_s, 1) == 302.3, 'outside the tools 302.3 s', t.outside_s)
    want(round(t.outside_s, 3) == round(t.run_s - t.tool_s, 3),
         'and it is exactly run minus tool')


def test_the_span_uses_max_t_end_not_the_last_row():
    rows = ct.load_rows(OOO)
    t = ct.totals(rows)
    # D-longest starts at +300 and runs 400 s, so it ENDS at +700 -- after
    # E-last, which starts at +600 and ends at +602.
    want(rows[-1]['label'] == 'E-last', 'E-last is the last row by t_start',
         rows[-1]['label'])
    want(round(t.run_s, 1) == 700.0,
         'the span is max(t_end) - min(t_start), not the last row\'s t_end',
         t.run_s)


def test_rows_are_sorted_by_t_start_even_when_the_file_is_not():
    order = [r['label'] for r in ct.load_rows(OOO)]
    want(order == ['A-first', 'B-second', 'D-longest', 'E-last'],
         'rows come back in run order whatever order the file holds', order)
    raw = [json.loads(x)['label'] for x in open(OOO, encoding='utf-8') if x.strip()]
    want(raw != order, 'and the fixture really is out of order on disk', raw)


def test_a_malformed_line_names_its_line_number(tmp=None):
    import tempfile
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'broken.jsonl')
    lines = [x for x in open(FIX, encoding='utf-8')]
    lines.insert(6, '{not json at all\n')
    open(p, 'w', encoding='utf-8').writelines(lines)
    try:
        ct.load_rows(p)
        want(False, 'a malformed line raises')
    except SystemExit as e:
        msg = str(e)
        want(':7:' in msg, 'the parse error names the LINE NUMBER', msg)
        want('broken.jsonl' in msg, 'and the file', msg)


def test_a_missing_ledger_is_empty_not_an_error():
    want(ct.load_rows(os.path.join(FIXDIR, 'nope.jsonl')) == [],
         'a missing ledger reads as empty')
    t = ct.totals([])
    want(t.n == 0 and t.run_s is None and t.tool_s == 0.0,
         'and totals of nothing is not an exception', t)


# ------------------------------------------------------------------ fmt_hms

#: The nine H:MM:SS values in the hand-written run-24 report, which were
#: produced BY HAND. Truncation gets four of them wrong.
PUBLISHED_HMS = [
    (0.4, '0:00:00'), (6.8, '0:00:07'), (23.7, '0:00:24'), (26.3, '0:00:26'),
    (54.8, '0:00:55'), (141.3, '0:02:21'), (253.3, '0:04:13'),
    (4405.4, '1:13:25'), (4658.7, '1:17:39'),
]


def test_fmt_hms_rounds_to_the_nearest_second():
    bad = [(s, w, ct.fmt_hms(s)) for s, w in PUBLISHED_HMS if ct.fmt_hms(s) != w]
    want(not bad, 'all nine published H:MM:SS values reproduce', bad)
    trunc = [s for s, w in PUBLISHED_HMS
             if '%d:%02d:%02d' % (int(s) // 3600, int(s) % 3600 // 60,
                                  int(s) % 60) != w]
    want(len(trunc) == 4,
         'and truncation would get exactly four of them wrong', trunc)


def test_fmt_hms_does_not_use_bankers_rounding():
    want(ct.fmt_hms(0.5) == '0:00:01',
         "0.5 s rounds UP -- Python's round() would give 0", ct.fmt_hms(0.5))
    want(ct.fmt_hms(1.5) == '0:00:02', '1.5 s rounds up too', ct.fmt_hms(1.5))


def test_the_three_longest_are_named_and_ordered():
    got = [(r['label'], r['wall_s']) for r in ct.longest(ct.load_rows(FIX))]
    exp = [('R3-route', 60.0), ('R7-layercosts', 24.0), ('V-complete', 8.2)]
    want(got == exp, 'the three longest steps, longest first', got)


# ------------------------------------------------------------------- report

def test_the_report_has_the_four_sections_the_mandate_names():
    md = ct.report_markdown(ct.load_rows(FIX), 'x.jsonl')
    for head in ('### 1. Step table', '### 2. Stage subtotals',
                 '### 3. Totals', '### 4. Three longest steps'):
        want(head in md, 'the report carries %r' % head)
    want('Time outside the tools' in md,
         'the difference is reported EXPLICITLY, in those words')


def test_the_report_states_the_clock_is_descriptive():
    md = ct.report_markdown(ct.load_rows(FIX), 'x.jsonl')
    want('no budget' in md,
         "tee_cmd's posture travels with the artifact, not just the source")


def test_the_report_note_is_generated_from_the_rules():
    # A prose note that is TYPED cannot drift from the code; one that is
    # GENERATED cannot either. Check it is the latter by changing the rules.
    saved = ct.STAGE_RULES
    try:
        ct.STAGE_RULES = (('staging', ('zzz',)),)
        note = ct._rules_note()
        want('zzz*' in note, 'the bucketing note is derived from STAGE_RULES',
             note[:80])
    finally:
        ct.STAGE_RULES = saved


# -------------------------------------------------------------- shape gates

def test_the_reader_imports_without_pillow():
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', '-c',
         'import cmd_timing, sys;'
         'bad=[m for m in sys.modules if m.split(".")[0] in ("PIL","numpy",'
         '"scipy","shapely")];'
         'print("LEAKED:" + ",".join(bad) if bad else "CLEAN")'],
        cwd=os.path.join(ROOT, 'py_router'), capture_output=True, text=True)
    want('CLEAN' in r.stdout,
         'import cmd_timing pulls in no third-party module, so the report CLI '
         'runs on a machine with no Pillow', r.stdout + r.stderr)


def test_nothing_in_this_module_predicts():
    """No rate, no projection, no ETA -- asserted on the AST, not the text.

    Asserting on the text would make the module's own docstring ("there is no
    ETA anywhere in here") trip its own gate, and the lesson this repo learned
    the hard way is that a comment quoting code satisfies a naive grep. So walk
    the tree and look at IDENTIFIERS.
    """
    banned = ('eta', 'estimate', 'forecast', 'predict', 'extrapolat')
    tree = ast.parse(open(MODULE, encoding='utf-8').read())
    hits = []
    for node in ast.walk(tree):
        names = []
        if isinstance(node, ast.Name):
            names = [node.id]
        elif isinstance(node, ast.Attribute):
            names = [node.attr]
        elif isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            names = [node.name]
        elif isinstance(node, ast.arg):
            names = [node.arg]
        for nm in names:
            low = nm.lower()
            for b in banned:
                # 'eta' as a substring would hit 'metadata'; require it as a
                # whole word-ish token for that one only.
                if b == 'eta':
                    if low == 'eta' or low.startswith('eta_') or low.endswith('_eta'):
                        hits.append(nm)
                elif b in low:
                    hits.append(nm)
    want(not hits, 'no identifier in cmd_timing.py predicts anything', hits)


def test_the_module_declares_no_threshold():
    """tee_cmd says there is no budget, no cap and no timeout. Keep it that way."""
    tree = ast.parse(open(MODULE, encoding='utf-8').read())
    hits = []
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name):
                    up = t.id.upper()
                    if any(k in up for k in ('MAX_', 'LIMIT', 'BUDGET', 'TIMEOUT',
                                             'DEADLINE')):
                        hits.append(t.id)
    want(not hits,
         'the reader grows no threshold constant -- it reports, it does not '
         'judge', hits)


# ---------------------------------------------------------------------- CLI

def test_the_cli_prints_the_report_and_refuses_a_missing_ledger():
    cli = os.path.join(ROOT, 'py_router', 'cmd_timing.py')
    run_utils.evidence(FIX, 'the synthetic ledger')
    r = run_utils.check([sys.executable, '-X', 'utf8', cli, FIX], accept=True)
    want('### 2. Stage subtotals' in r.stdout, 'the CLI prints the report')

    import tempfile
    empty = tempfile.mkdtemp()
    run_utils.check([sys.executable, '-X', 'utf8', cli, empty],
                    refuse='no cmd_timing.jsonl', code=2)
    want(True, 'and refuses an empty directory by naming what is missing')


def test_the_cli_json_mode_carries_the_same_numbers():
    cli = os.path.join(ROOT, 'py_router', 'cmd_timing.py')
    r = run_utils.check([sys.executable, '-X', 'utf8', cli, FIX, '--json'],
                        accept=True)
    d = json.loads(r.stdout)
    want(round(d['totals']['outside_s'], 1) == 302.3,
         '--json reports the same outside-the-tools figure',
         d['totals']['outside_s'])
    want([s['stage'] for s in d['subtotals']] == [s for s, _, _ in WANT_SUBTOTALS],
         '--json carries every bucket, in report order')


TESTS_TO_RUN = [
    test_the_synthetic_ledger_reproduces_its_declared_subtotals,
    test_pclose_counts_under_P_not_close_out,
    test_fence_audit_start_buckets_under_close_out_though_it_ran_first,
    test_a_repeated_label_is_counted_once_per_entry,
    test_an_unmatched_label_lands_in_other_and_other_is_always_reported,
    test_the_subtotals_sum_to_the_tool_total,
    test_outside_the_tools_is_the_gap_and_the_fixture_has_one,
    test_the_span_uses_max_t_end_not_the_last_row,
    test_rows_are_sorted_by_t_start_even_when_the_file_is_not,
    test_a_malformed_line_names_its_line_number,
    test_a_missing_ledger_is_empty_not_an_error,
    test_fmt_hms_rounds_to_the_nearest_second,
    test_fmt_hms_does_not_use_bankers_rounding,
    test_the_three_longest_are_named_and_ordered,
    test_the_report_has_the_four_sections_the_mandate_names,
    test_the_report_states_the_clock_is_descriptive,
    test_the_report_note_is_generated_from_the_rules,
    test_the_reader_imports_without_pillow,
    test_nothing_in_this_module_predicts,
    test_the_module_declares_no_threshold,
    test_the_cli_prints_the_report_and_refuses_a_missing_ledger,
    test_the_cli_json_mode_carries_the_same_numbers,
]


def main():
    for fn in TESTS_TO_RUN:
        print('--- %s' % fn.__name__)
        fn()
    if BAD:
        print('\nFAILED: %d' % len(BAD))
        for b in BAD:
            print('  - %s' % b)
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
