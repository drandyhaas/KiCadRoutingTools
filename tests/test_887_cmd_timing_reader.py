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
import re
import subprocess
import sys
import tempfile

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
    # NOT `outside_s == run_s - tool_s`: that is true by construction and sees
    # nothing. What is worth asserting is that the fixture's gaps are real, so
    # the 302.3 above is measuring something rather than a rounding artefact.
    gaps = sum(b['t_start'] - a['t_end']
               for a, b in zip(ct.load_rows(FIX), ct.load_rows(FIX)[1:]))
    want(abs(gaps - t.outside_s) < 1e-6,
         'and it equals the summed GAPS between consecutive commands, which is '
         'what "time outside the tools" means on a serial ledger',
         (gaps, t.outside_s))


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


#: Words that would mean this module had started forecasting.
#:
#: `eta` and `rate` are matched as whole TOKENS, because as substrings they are
#: everywhere in ordinary code: `metadata` contains one and `enumerate`
#: contains the other -- which this list caught the moment it was written.
#: The rest are specific enough to match as substrings.
_BANNED_TOKENS = ('eta', 'rate')
_BANNED_SUBSTRINGS = ('estimate', 'forecast', 'predict', 'extrapolat',
                      'projection', 'remaining')


def _predicting_names(src):
    """Identifiers in `src` that would mean this module forecasts something."""
    tree = ast.parse(src)
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
        elif isinstance(node, ast.keyword) and node.arg:
            names = [node.arg]                 # call-site kwargs count too
        for nm in names:
            low = nm.lower()
            # SPLIT INTO TOKENS rather than testing prefixes. The first version
            # used `low == 'eta' or low.startswith('eta_') or
            # low.endswith('_eta')`, which missed `compute_eta_seconds`,
            # `etaSeconds`, `eta2` and `run_eta_s` -- i.e. most of the ways
            # anyone would actually spell it.
            # Split on non-alphanumerics, on camelCase humps, AND on the
            # letter->digit boundary: without that last one `eta2` stayed a
            # single token and slipped through.
            spaced = re.sub(r'(?<=[a-z0-9])(?=[A-Z])', '_', nm)
            spaced = re.sub(r'(?<=[A-Za-z])(?=[0-9])', '_', spaced)
            toks = [t for t in re.split(r'[^a-z0-9]+', spaced.lower()) if t]
            if any(t in _BANNED_TOKENS for t in toks):
                hits.append(nm)
            elif any(b in low for b in _BANNED_SUBSTRINGS):
                hits.append(nm)
    return hits


def test_nothing_in_this_module_predicts():
    """No rate, no projection, no ETA -- asserted on the AST, not the text.

    Asserting on the text would make the module's own docstring ("there is no
    ETA anywhere in here") trip its own gate, and the lesson this repo learned
    the hard way is that a comment quoting code satisfies a naive grep. So walk
    the tree and look at IDENTIFIERS.
    """
    hits = _predicting_names(open(MODULE, encoding='utf-8').read())
    want(not hits, 'no identifier in cmd_timing.py predicts anything', hits)


def test_the_prediction_detector_catches_how_people_actually_spell_it():
    """The detector is the gate; a gate nobody tested is not one.

    Every name below evaded the first version of this check.
    """
    spellings = ['compute_eta_seconds', 'etaSeconds', 'eta2', 'run_eta_s',
                 'remaining_eta', 'eta', 'projected_rate', 'estimate_total',
                 'forecast', 'predict_end', 'extrapolate', 'fill_rate']
    missed = [s for s in spellings if not _predicting_names('%s = 1' % s)]
    want(not missed, 'every spelling of a forecast is caught', missed)
    # `enumerate` contains 'rate' and `metadata` contains 'eta'; both are
    # ordinary and neither may trip the gate. A gate that cries wolf on
    # `enumerate` gets its banned list edited into uselessness.
    ok = ['metadata', 'beta_flag', 'theta', 'iso_start', 'wall_s', 'totals',
          'enumerate', 'accurate', 'separator']
    false_pos = [s for s in ok if _predicting_names('%s = 1' % s)]
    want(not false_pos,
         'and ordinary names containing those letters are not', false_pos)


def test_no_rule_prefix_shadows_another():
    """THE guard the ordering comment should have had.

    The first version claimed STAGE_RULES' order was load-bearing. It is not:
    the prefixes are pairwise incomparable, so at most one rule can match any
    label and reversing the tuple changes no answer -- three reordering
    mutations survived the whole suite. What IS worth pinning is the property
    that makes the order inert, because a future rule like 'Pc' would quietly
    end that and make precedence start deciding answers.
    """
    prefixes = [p for _stage, ps in ct.STAGE_RULES for p in ps]
    shadowed = [(a, b) for a in prefixes for b in prefixes
                if a != b and b.startswith(a)]
    want(not shadowed,
         'no rule prefix is a prefix of another, so the rule ORDER decides '
         'nothing and may be read in any order', shadowed)
    # And prove the claim directly: every permutation buckets identically.
    import itertools
    labels = [r['label'] for r in ct.load_rows(FIX)] + [
        'Pclose-x', 'fence-y', 'close-z', 'route4', 'staging-q']
    base = [ct.stage_of(x) for x in labels]
    saved = ct.STAGE_RULES
    try:
        differing = 0
        for perm in itertools.permutations(saved):
            ct.STAGE_RULES = perm
            if [ct.stage_of(x) for x in labels] != base:
                differing += 1
    finally:
        ct.STAGE_RULES = saved
    want(differing == 0,
         'and all %d orderings of the rules agree, label for label'
         % len(list(itertools.permutations(saved))), differing)


def test_the_report_note_states_the_real_mechanism():
    """The generated note used to ship the FALSE ordering claim to every reader.

    `_rules_note` is part generated, part typed, and only the generated half
    was covered -- deleting the typed tail survived the suite. The tail is
    where the explanation lives, so it is the half most worth pinning.
    """
    note = ct._rules_note()
    want('pairwise incomparable' in note,
         'the note says the prefixes cannot shadow each other', note[:120])
    want('order does not arbitrate' in note or
         'order does not' in note,
         'and therefore that the order decides nothing', note[:200])
    want('tested before' not in note,
         'and no longer claims fence-* wins by being tested first', note)
    want('Pclose' in note and 'is not a prefix OF' in note,
         'and gives the real reason Pclose-* lands in P*', note)


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

    # Nested, because find_ledger walks up 3 parents: a bare mkdtemp() sits
    # under %TEMP%\...\AppData, so a stray ledger up there would decide this.
    empty = os.path.join(tempfile.mkdtemp(), 'a', 'b', 'c')
    os.makedirs(empty)
    r2 = run_utils.check([sys.executable, '-X', 'utf8', cli, empty],
                         refuse='no cmd_timing.jsonl', code=2)
    want('no cmd_timing.jsonl' in (r2.stdout + r2.stderr),
         'and refuses an empty directory by naming what is missing',
         (r2.stdout + r2.stderr)[:120])


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


def _tmp_ledger(rows, name='cmd_timing.jsonl', encoding='utf-8', sub=''):
    """Write `rows` as a ledger in a fresh nested dir; return its path.

    NESTED on purpose: `find_ledger` walks up to 3 parents, and a bare
    `tempfile.mkdtemp()` sits under %TEMP%\\...\\AppData, so a stray
    cmd_timing.jsonl anywhere up there would decide a test's outcome for an
    environmental reason.
    """
    d = os.path.join(tempfile.mkdtemp(), 'a', 'b', 'c', sub) if sub else \
        os.path.join(tempfile.mkdtemp(), 'a', 'b', 'c')
    os.makedirs(d, exist_ok=True)
    # abspath, so the returned string compares equal to what find_ledger gives
    # back -- a stray '.' segment made an otherwise-correct answer look wrong.
    p = os.path.abspath(os.path.join(d, name))
    with open(p, 'w', encoding=encoding) as f:
        for r in rows:
            f.write(json.dumps(r) + '\n')
    return p


def _row(label, t0, wall, code=0, iso='2026-01-01T00:00:00', **kw):
    r = {'label': label, 't_start': t0, 't_end': t0 + wall, 'wall_s': wall,
         'exit': code, 'iso_start': iso, 'argv': ['x'], 'cmdline': 'x'}
    r.update(kw)
    return r


def test_longest_breaks_ties_by_run_order():
    """`longest`'s docstring promises it; neither fixture has a tie, so nothing
    checked it and two tie-break mutations survived."""
    rows = [_row('late', 300, 5.0), _row('early', 100, 5.0),
            _row('mid', 200, 5.0), _row('small', 400, 1.0)]
    got = [r['label'] for r in ct.longest(rows, 3)]
    want(got == ['early', 'mid', 'late'],
         'equal durations come back in RUN ORDER, not file or hash order', got)


def test_a_row_with_no_label_gets_tee_cmds_own_default():
    p = _tmp_ledger([{'t_start': 1.0, 't_end': 2.0, 'wall_s': 1.0, 'exit': 0}])
    rows = ct.load_rows(p)
    want(rows[0]['label'] == 'unlabelled',
         "a label-less row mirrors tee_cmd's own default rather than raising",
         rows[0].get('label'))
    want(ct.stage_of(rows[0]['label']) == 'other',
         'and buckets to other')


def test_a_string_or_garbage_clock_is_coerced_not_trusted():
    """Coercion is the stated reason not to reuse predictor_study.read_jsonl,
    and no fixture exercised it: passing values through raw survived."""
    p = _tmp_ledger([
        _row('P1', 1.0, 1.0),
        {'label': 'P2', 't_start': '10.0', 't_end': '12.5', 'wall_s': '2.5',
         'exit': 0, 'iso_start': 'x'},
        {'label': 'P3', 't_start': 'not-a-number', 't_end': None,
         'wall_s': 'nope', 'exit': 0, 'iso_start': 'x'},
    ])
    rows = ct.load_rows(p)
    by = {r['label']: r for r in rows}
    want(by['P2']['wall_s'] == 2.5,
         'a numeric STRING is coerced, so sum() cannot raise later',
         by['P2']['wall_s'])
    want(by['P3']['wall_s'] is None and by['P3']['t_start'] is None,
         'and unparseable garbage becomes None rather than propagating',
         (by['P3']['wall_s'], by['P3']['t_start']))
    md = ct.report_markdown(rows, 'x')
    want('P3' in md, 'the garbage row still appears in the step table')


def test_a_non_finite_number_does_not_kill_the_report():
    """json.loads accepts bare NaN/Infinity, and float() is happy with both."""
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'cmd_timing.jsonl')
    with open(p, 'w', encoding='utf-8') as f:
        f.write('{"label":"P1","t_start":1,"t_end":2,"wall_s":NaN,"exit":0}\n')
        f.write('{"label":"P2","t_start":3,"t_end":4,"wall_s":Infinity,'
                '"exit":0}\n')
        f.write(json.dumps(_row('P3', 5.0, 1.0)) + '\n')
    rows = ct.load_rows(p)
    want(all(r['wall_s'] is None or r['wall_s'] == 1.0 for r in rows),
         'NaN and Infinity are dropped as garbage, not carried',
         [r['wall_s'] for r in rows])
    md = ct.report_markdown(rows, 'x')          # used to raise ValueError
    want('### 3. Totals' in md,
         'and the whole report still renders instead of dying on one cell')


def test_a_utf8_bom_does_not_reject_the_ledger():
    """PowerShell's `>` and Out-File write UTF-8 WITH BOM, and this is a
    Windows-primary repo -- a copied ledger arrives with one."""
    p = _tmp_ledger([_row('P1', 1.0, 1.0)], encoding='utf-8-sig')
    raw = open(p, 'rb').read()
    want(raw.startswith(b'\xef\xbb\xbf'), 'the fixture really has a BOM')
    rows = ct.load_rows(p)
    want(len(rows) == 1 and rows[0]['label'] == 'P1',
         'and it reads as one row, not as a refusal', rows)


def test_find_ledger_walks_up_from_a_board():
    """The parent walk is the stated contract with make_movie's board-sequence
    form, and no test called find_ledger at all -- max_up=0 survived."""
    p = _tmp_ledger([_row('P1', 1.0, 1.0)])
    wd = os.path.dirname(p)
    boards = os.path.join(wd, 'boards')
    os.makedirs(boards, exist_ok=True)
    board = os.path.join(boards, 'x.kicad_pcb')
    open(board, 'w').close()
    want(ct.find_ledger(wd) == p, 'found from the work dir', ct.find_ledger(wd))
    want(ct.find_ledger(board) == p,
         'and from a BOARD one directory down -- the board-sequence form',
         ct.find_ledger(board))
    deep = os.path.join(boards, 'x', 'y')
    os.makedirs(deep, exist_ok=True)
    want(ct.find_ledger(os.path.join(deep, 'z.kicad_pcb')) == p,
         'and three levels down')
    want(ct.find_ledger(board, max_up=0) is None,
         'but max_up=0 really does stop at the board\'s own directory')
    want(ct.find_ledger(None) is None, 'None is not a path')
    want(ct.find_ledger(board, max_up=None) is None,
         'and max_up=None is treated as 0 rather than raising')


def test_outside_the_tools_is_never_clamped():
    """A declared decision ("reported as-is, even if negative") that no fixture
    could reach: clamping it at zero survived."""
    rows = [_row('P1', 0.0, 100.0), _row('P2', 10.0, 100.0)]
    t = ct.totals(rows)
    want(t.run_s == 110.0, 'the span is max(t_end) - min(t_start)', t.run_s)
    want(t.tool_s == 200.0, 'while the tools claim more than the span',
         t.tool_s)
    want(t.outside_s == -90.0,
         'so outside-the-tools goes NEGATIVE and is reported that way -- two '
         'overlapping wrapped commands is information, not an error to hide',
         t.outside_s)


def test_the_subtotals_sum_at_full_precision():
    """The fixture's buckets are exact at 1 dp, so rounding the subtotals to 1
    dp still summed correctly there -- and broke on the real ledger."""
    rows = [_row('P1', 0.0, 0.041), _row('P2', 1.0, 0.037),
            _row('R1', 2.0, 0.026)]
    s = sum(w for _st, _n, w in ct.subtotals(rows))
    want(abs(s - ct.totals(rows).tool_s) < 1e-9,
         'the buckets sum to the tool total at full precision, not just at the '
         'one decimal the table prints', (s, ct.totals(rows).tool_s))


def test_the_step_table_carries_every_cell_the_mandate_names():
    """The mandate's item 1 is `label | started | wall seconds | exit`, and the
    153-row table is the report's largest artifact -- yet no assertion read a
    single cell, so dropping the exit column and coarsening the wall column
    both survived."""
    rows = [_row('R3-route', 100.0, 23.002, code=4, iso='2026-08-20T12:03:09')]
    md = ct.report_markdown(rows, 'x')
    line = [ln for ln in md.splitlines() if ln.startswith('| 1 |')]
    want(len(line) == 1, 'there is one numbered step row', line)
    cells = [c.strip() for c in line[0].strip('|').split('|')]
    want(cells[1] == 'R3-route', 'the label cell', cells)
    want(cells[2] == '2026-08-20T12:03:09',
         'the started cell, echoed verbatim from iso_start', cells)
    want(cells[3] == '23.002',
         'the wall cell at FULL precision -- 23.0 would lose the 2 ms the '
         'ledger recorded', cells)
    want(cells[4] == '4', 'and the exit cell, carried and not judged', cells)


def test_the_stage_titles_are_not_crossed():
    md = ct.report_markdown(ct.load_rows(FIX), 'x')
    for stage, title in ct.STAGE_TITLES.items():
        want(('| %s |' % title) in md,
             'the subtotals table shows %r for %r' % (title, stage))
    want(ct.STAGE_TITLES['R*'].startswith('R*')
         and ct.STAGE_TITLES['L*'].startswith('L*')
         and ct.STAGE_TITLES['P*'].startswith('P*'),
         'and each title names its own bucket, so the table cannot mislabel '
         'every row while still looking complete')


def test_the_totals_block_reports_tool_time_against_run_time():
    """Mandate item 3 is "TOTAL tool time vs total run time"; only the phrase
    "Time outside the tools" was asserted, so eliding the tool-time line
    survived."""
    md = ct.report_markdown(ct.load_rows(FIX), 'x')
    want('Tool time (sum of wall_s): 109.7 s' in md, 'the tool-time line', )
    want('Total run time' in md and '412.0 s' in md, 'the run-time line')
    want('Time outside the tools' in md and '302.3 s' in md,
         'and the difference, in those words')


def test_report_data_carries_the_fields_a_consumer_reads():
    """report_data is the API the movie overlay consumes; only two of its keys
    were pinned, so wrong `steps` and `longest` payloads survived."""
    d = ct.report_data(ct.load_rows(FIX), 'x.jsonl')
    want(len(d['steps']) == 20, 'one step entry per row', len(d['steps']))
    want(d['steps'][0]['label'] == 'fence-audit-start',
         'in run order', d['steps'][0]['label'])
    want(all(set(s) >= {'label', 'iso_start', 'wall_s', 'exit'}
             for s in d['steps']), 'each carrying the mandate\'s four fields')
    want([x['label'] for x in d['longest']] ==
         ['R3-route', 'R7-layercosts', 'V-complete'],
         'the three longest, longest first', d['longest'])
    want(d['totals']['n'] == 20 and d['totals']['tool_s'] == 109.7,
         'and the totals block', d['totals'])
    want('last_iso_start' in d['totals'] and 'iso_end' not in d['totals'],
         'named last_iso_start, because tee_cmd records no end ISO and the old '
         'name promised one it did not have', sorted(d['totals']))


def test_unmatched_labels_are_called_out_not_just_counted():
    """The mandate defined close-out as a CATCH-ALL; these rules send anything
    unrecognised to `other`. Printing an empty `other` row is not enough to
    notice a run whose labels follow another convention entirely."""
    want(ct.unmatched_note(ct.load_rows(FIX)),
         'the fixture has route4, so the note fires')
    note = ct.unmatched_note(ct.load_rows(FIX))
    want('route4' in note, 'and names the offending label', note)
    ok = [_row('P1', 0.0, 1.0), _row('R1', 2.0, 1.0)]
    want(ct.unmatched_note(ok) == '',
         'while a conventional run gets no warning at all')
    lower = [_row('p0-driver', 0.0, 1.0), _row('r3-route', 2.0, 1.0)]
    note2 = ct.unmatched_note(lower)
    want('100%' in note2,
         'and a wholly lowercase convention is reported as 100%, not as a '
         'quietly empty set of stage buckets', note2)
    want(ct.unmatched_note(lower) in ct.report_markdown(lower, 'x'),
         'the warning reaches the report, not just the API')


def test_the_other_bucket_is_reported_as_actually_empty():
    """Asserting only that 'other' is a KEY would pass for a bucketer that
    invented content in it."""
    ok = [_row('P1', 0.0, 1.0), _row('R1', 2.0, 1.0)]
    subs = {s: (n, w) for s, n, w in ct.subtotals(ok)}
    want(subs['other'] == (0, 0.0),
         'other is present AND measured zero, which is what makes the run-24 '
         'footnote a checkable claim', subs['other'])


def test_the_subtotals_table_columns_agree():
    """A bucket of 26.46 s printed `| 26.5 | 0:00:26 |` -- two columns of the
    same number disagreeing."""
    rows = [_row('V1', 0.0, 26.46)]
    md = ct.report_markdown(rows, 'x')
    line = [ln for ln in md.splitlines() if 'V* verification' in ln][0]
    cells = [c.strip() for c in line.strip('|').split('|')]
    secs = float(cells[2])
    want(ct.fmt_hms(secs) == cells[3],
         'the H:MM:SS column is the seconds column, formatted -- not a second '
         'rounding of a different value', (cells[2], cells[3]))


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
    test_no_rule_prefix_shadows_another,
    test_the_report_note_states_the_real_mechanism,
    test_longest_breaks_ties_by_run_order,
    test_a_row_with_no_label_gets_tee_cmds_own_default,
    test_a_string_or_garbage_clock_is_coerced_not_trusted,
    test_a_non_finite_number_does_not_kill_the_report,
    test_a_utf8_bom_does_not_reject_the_ledger,
    test_find_ledger_walks_up_from_a_board,
    test_outside_the_tools_is_never_clamped,
    test_the_subtotals_sum_at_full_precision,
    test_the_step_table_carries_every_cell_the_mandate_names,
    test_the_stage_titles_are_not_crossed,
    test_the_totals_block_reports_tool_time_against_run_time,
    test_report_data_carries_the_fields_a_consumer_reads,
    test_unmatched_labels_are_called_out_not_just_counted,
    test_the_other_bucket_is_reported_as_actually_empty,
    test_the_subtotals_table_columns_agree,
    test_the_prediction_detector_catches_how_people_actually_spell_it,
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
    # Each test is isolated. Without this an AssertionError from run_utils.check
    # aborted the file, and every test after it neither ran nor was reported --
    # indistinguishable, in the output, from tests that were never written.
    for fn in TESTS_TO_RUN:
        print('--- %s' % fn.__name__)
        try:
            fn()
        except Exception as exc:                            # noqa: BLE001
            import traceback
            BAD.append('%s RAISED %s' % (fn.__name__, exc))
            traceback.print_exc()
    if BAD:
        print('\nFAILED: %d' % len(BAD))
        for b in BAD:
            print('  - %s' % b)
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
