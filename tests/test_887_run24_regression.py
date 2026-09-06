#!/usr/bin/env python3
"""#887: corroborate the cmd_timing reader against the one REAL ledger (run 24).

This is the OPT-IN arm. `wk/` is gitignored, so `wk/run24/esp_prog/cmd_timing.jsonl`
-- 153 rows, 104 KB -- is not in the repository and cannot be. The regression
lives in `tests/test_887_cmd_timing_reader.py`, against small tracked fixtures,
and passes on a fresh clone with no `wk/` tree at all. THIS file only says
"and it also reproduces the hand-written audit of a real run, to the digit",
which is a different and weaker claim: corroboration, not coverage.

It self-skips (exit 77 with a SKIP: line) when the ledger is absent.
`KRT_CMD_TIMING_LEDGER` points it at a ledger somewhere else -- but every
assertion here is a RUN-24 LITERAL, so on any other ledger it prints the report
and passes rather than failing ten times for a non-defect. The first version
invited exactly that in its own docstring: following the documented invocation
on this commit's own test fixture turned the suite red with `FAILED: 10`.

The numbers asserted below are transcribed from `wk/run24/esp_prog/watch/timing.md`
-- the report a WATCH SUBAGENT produced by hand -- not from a run of this code.
"""
import os
import sys

RUN_ALL_TIMEOUT = 120

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import cmd_timing as ct                                   # noqa: E402

LEDGER = (os.environ.get('KRT_CMD_TIMING_LEDGER')
          or os.path.join(ROOT, 'wk', 'run24', 'esp_prog', 'cmd_timing.jsonl'))

if not os.path.isfile(LEDGER):
    print('SKIP: %s is absent. wk/ is gitignored (0 files tracked), so the real '
          'run-24 ledger is not in the repo; the synthetic fixtures in '
          'tests/test_887_cmd_timing_reader.py carry the regression. Set '
          'KRT_CMD_TIMING_LEDGER to point this arm at a teed run.' % LEDGER)
    sys.exit(77)

#: From wk/run24/esp_prog/watch/timing.md:200-224, the hand-written audit.
PUBLISHED_SUBTOTALS = [
    ('staging',   1,   0.4),
    ('P*',       81,  54.8),
    ('L*',       19,   6.8),
    ('R*',       36, 141.3),
    ('V*',        8,  26.3),
    ('close-out', 8,  23.7),
    ('other',     0,   0.0),      # the footnote's own claim, asserted
]
PUBLISHED_HMS = ['0:00:00', '0:00:55', '0:00:07', '0:02:21', '0:00:26',
                 '0:00:24', '0:00:00']
PUBLISHED_LONGEST = [('R7-layercosts', 24.0), ('R3-route', 23.0),
                     ('V-complete', 8.4)]

ROWS = ct.load_rows(LEDGER)

#: Run 24's fingerprint. Every literal below is that run's; on any OTHER ledger
#: they are all wrong, and asserting them would report ten defects that are not
#: defects. So a foreign ledger gets the report printed and a pass.
IS_RUN24 = (len(ROWS) == 153
            and any(r.get('label') == 'V-oracle' for r in ROWS))
if not IS_RUN24:
    print('NOTE: %s is not run 24 (%d rows). Printing its report; the run-24 '
          'literals below are not assertable against it.'
          % (LEDGER, len(ROWS)))
    sys.stdout.write(ct.report_markdown(ROWS, LEDGER))
    print('\nALL PASS (nothing to assert on a foreign ledger)')
    sys.exit(0)

BAD = []


def want(cond, label, extra=''):
    if cond:
        print('  PASS: %s' % label)
    else:
        BAD.append(label)
        print('  FAIL: %s %s' % (label, extra))


def test_run24_totals_match_the_published_report():
    t = ct.totals(ROWS)
    want(t.n == 153, '153 wrapped commands', t.n)
    want(round(t.tool_s, 1) == 253.3, 'tool time 253.3 s', t.tool_s)
    want(round(t.run_s, 1) == 4658.7, 'run span 4658.7 s', t.run_s)
    want(round(t.outside_s, 1) == 4405.4,
         'time outside the tools 4405.4 s -- 94.6 percent of the run', t.outside_s)


def test_run24_stage_subtotals_match_the_published_table():
    got = [(s, n, round(w, 1)) for s, n, w in ct.subtotals(ROWS)]
    want(got == PUBLISHED_SUBTOTALS,
         'every stage subtotal reproduces the hand-written table',
         '\n    got  %s\n    want %s' % (got, PUBLISHED_SUBTOTALS))


def test_run24_hms_column_matches_the_hand_written_one():
    got = [ct.fmt_hms(w) for _, _, w in ct.subtotals(ROWS)]
    want(got == PUBLISHED_HMS, 'the H:MM:SS column reproduces', got)
    t = ct.totals(ROWS)
    want(ct.fmt_hms(t.tool_s) == '0:04:13', 'tool time 0:04:13', ct.fmt_hms(t.tool_s))
    want(ct.fmt_hms(t.run_s) == '1:17:39', 'run 1:17:39', ct.fmt_hms(t.run_s))
    want(ct.fmt_hms(t.outside_s) == '1:13:25', 'outside 1:13:25',
         ct.fmt_hms(t.outside_s))


def test_run24_three_longest_match_the_published_names():
    got = [(r['label'], round(r['wall_s'], 1)) for r in ct.longest(ROWS)]
    want(got == PUBLISHED_LONGEST, 'the three longest steps, as published', got)


def test_the_ledger_rows_are_disjoint_in_time():
    """tee_cmd runs commands serially and blocks on each, so no two rows overlap.

    This is not decoration: the board->command mapping (#887) resolves a board
    by finding the row whose [t_start, t_end] contains the board's mtime, and
    that is unique ONLY because the rows are disjoint. If this ever fails on a
    real ledger, the mapping needs a tie-break and this message is the warning.
    """
    # Partition the clockless rows OUT before sorting. `sorted(key=r['t_start'])`
    # died with a bare TypeError on a row whose t_start was missing -- a check
    # that fails before it checks anything, reporting as an exit code that
    # reads exactly like a real overlap finding.
    timed = [r for r in ROWS if r.get('t_start') is not None
             and r.get('t_end') is not None]
    clockless = [r.get('label') for r in ROWS if r not in timed]
    want(not clockless,
         'every row carries a usable clock (rows without one are excluded from '
         'the overlap check, and named here rather than crashing it)',
         clockless[:5])
    ordered = sorted(timed, key=lambda r: r['t_start'])
    overlaps = [(a['label'], b['label'])
                for a, b in zip(ordered, ordered[1:])
                if b['t_start'] < a['t_end']]
    want(not overlaps,
         'no two wrapped commands overlap, so an mtime lands in at most one row',
         overlaps[:5])


def test_the_pclose_subcount_the_hand_report_got_wrong():
    """The hand-written audit says "16 of its 81 steps are the Pclose placement
    close-out" (timing.md:36). Counted from the ledger it is 21.

    This corroborates the commit's whole thesis -- a subagent doing arithmetic
    over a JSONL at the end of a run gets a number wrong and nobody notices --
    and it is three lines to check, which is why it is here rather than in the
    prose. The reader's P* total (81) is right; only the human's sub-count was
    off, so the published subtotals stand.
    """
    p_rows = [r for r in ROWS if ct.stage_of(r['label']) == 'P*']
    pclose = [r for r in p_rows if r['label'].startswith('Pclose')]
    want(len(p_rows) == 81, 'the P* bucket really is 81 steps', len(p_rows))
    want(len(pclose) == 21,
         'of which 21 are Pclose-*, not the 16 the hand-written report claims',
         len(pclose))


TESTS_TO_RUN = [
    test_the_pclose_subcount_the_hand_report_got_wrong,
    test_run24_totals_match_the_published_report,
    test_run24_stage_subtotals_match_the_published_table,
    test_run24_hms_column_matches_the_hand_written_one,
    test_run24_three_longest_match_the_published_names,
    test_the_ledger_rows_are_disjoint_in_time,
]


def main():
    print('ledger: %s (%d rows)' % (LEDGER, len(ROWS)))
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
