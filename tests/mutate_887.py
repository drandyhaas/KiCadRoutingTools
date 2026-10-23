#!/usr/bin/env python3
"""#887 mutation battery: is the cmd_timing reader covered, or do its tests
merely run?

    python3 tests/mutate_887.py
    python3 tests/mutate_887.py --verify-anchors  # PRE-FLIGHT only, ~1 second
    python3 tests/mutate_887.py --row the-order-of-the-stage-rules-is-reversed

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES engine files
in place and restores them, and a suite running beside it would grade a mutated
tree. One writer per tree.

A row is KILLED when any named test exits non-zero -- a failed assertion and an
ERROR count the same, because a mutation that makes the graders crash is still a
mutation the graders noticed. **A row whose anchor does not match EXACTLY ONCE
is BROKEN, not skipped**: an anchor that silently matches nothing reports every
mutation as killed and is the most flattering possible bug. The shared
`mutation_anchors.preflight` (#877) runs that check on every invocation, and
`--verify-anchors` runs it alone in about a second -- so a stale anchor is found
before a full run rather than fifty minutes into one.

Expected SURVIVORS are declared WITH THE REASON they are not a test hole.

The measured table is in the header of `tests/test_887_cmd_timing_reader.py`,
written from a run, and is never edited to match a prediction.

One row here is worth reading even if you skip the rest.
`the-order-of-the-stage-rules-is-reversed` is an expected SURVIVOR, and that is
the finding: the first version of cmd_timing.py claimed that order was
load-bearing, three reordering mutations survived a full suite, and the claim
was simply false -- the prefixes are pairwise incomparable, so nothing can
shadow anything. The row stays, expecting SURVIVED, so the day someone adds a
prefix that DOES shadow another it will start disagreeing.
"""
import argparse
import os
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

TIMING = os.path.join(ROOT, 'py_router', 'cmd_timing.py')
TARGETS = {'t': TIMING}

T_READER = os.path.join(TESTS, 'test_887_cmd_timing_reader.py')

#: (name, target, old, new, tests, expectation)
ROWS = [
    # ---- the bucketer ----------------------------------------------------
    ('pclose-is-routed-to-close-out', 't',
     "    ('close-out', ('fence', 'close')),\n    ('P*',        ('P',)),",
     "    ('close-out', ('fence', 'close', 'Pclose')),\n    ('P*',        ('P',)),",
     (T_READER,), 'KILLED'),
    ('fence-is-no-longer-close-out', 't',
     "    ('close-out', ('fence', 'close')),",
     "    ('close-out', ('close',)),",
     (T_READER,), 'KILLED'),
    ('stage_of-becomes-case-insensitive', 't',
     "        for pre in prefixes:\n            if s.startswith(pre):",
     "        for pre in prefixes:\n            if s.lower().startswith(pre.lower()):",
     (T_READER,), 'KILLED'),
    # THE ROW WORTH READING. See the module docstring: the order is inert, the
    # original comment said otherwise, and this survivor is the proof. It stays
    # so that adding a shadowing prefix starts disagreeing.
    ('the-order-of-the-stage-rules-is-reversed', 't',
     "STAGE_RULES = (\n    ('staging',   ('staging',)),\n"
     "    ('close-out', ('fence', 'close')),",
     "STAGE_RULES = (\n    ('close-out', ('fence', 'close')),\n"
     "    ('staging',   ('staging',)),",
     (T_READER,), 'SURVIVED'),   # the prefixes cannot shadow each other
    ('the-other-bucket-is-dropped-from-the-report', 't',
     "STAGE_ORDER = ('staging', 'P*', 'L*', 'R*', 'V*', 'close-out', OTHER)",
     "STAGE_ORDER = ('staging', 'P*', 'L*', 'R*', 'V*', 'close-out')",
     (T_READER,), 'KILLED'),
    ('unmatched-labels-stop-being-announced', 't',
     "    odd = [r.get('label') for r in rows if stage_of(r.get('label')) == OTHER]\n"
     "    if not odd:\n        return ''",
     "    odd = []\n    if not odd:\n        return ''",
     (T_READER,), 'KILLED'),

    # ---- reading and totals ---------------------------------------------
    ('rows-are-no-longer-sorted-by-t_start', 't',
     "    rows.sort(key=lambda r: (r['t_start'] is None, r['t_start'] or 0.0))",
     "    pass",
     (T_READER,), 'KILLED'),
    ('the-span-uses-the-last-rows-t_end', 't',
     "    t0, t1 = min(starts), max(ends)",
     "    t0, t1 = min(starts), ends[-1]",
     (T_READER,), 'KILLED'),
    ('outside-the-tools-is-clamped-at-zero', 't',
     "    return Totals(n, tool_s, t0, t1, run_s, round(run_s - tool_s, 3),",
     "    return Totals(n, tool_s, t0, t1, run_s, max(0.0, round(run_s - tool_s, 3)),",
     (T_READER,), 'KILLED'),
    ('numeric-coercion-is-removed', 't',
     "    return f if math.isfinite(f) else None",
     "    return v",
     (T_READER,), 'KILLED'),
    ('non-finite-values-are-passed-through', 't',
     "    return f if math.isfinite(f) else None",
     "    return f",
     (T_READER,), 'KILLED'),
    ('the-utf8-bom-codec-is-reverted', 't',
     "    with open(path, encoding='utf-8-sig') as f:",
     "    with open(path, encoding='utf-8') as f:",
     (T_READER,), 'KILLED'),
    ('a-label-less-row-is-left-without-one', 't',
     "            row.setdefault('label', 'unlabelled')",
     "            pass",
     (T_READER,), 'KILLED'),
    ('longest-breaks-ties-the-other-way', 't',
     "                                   r.get('t_start') if r.get('t_start') is not None\n"
     "                                   else float('inf')))",
     "                                   -(r.get('t_start') or 0.0)))",
     (T_READER,), 'KILLED'),
    ('find_ledger-stops-walking-up', 't',
     "    for _ in range(max(0, int(max_up or 0)) + 1):",
     "    for _ in range(1):",
     (T_READER,), 'KILLED'),
    ('fmt_hms-truncates-instead-of-rounding', 't',
     "    total = int(math.floor(float(seconds) + 0.5))",
     "    total = int(float(seconds))",
     (T_READER,), 'KILLED'),
    ('fmt_hms-uses-bankers-rounding', 't',
     "    total = int(math.floor(float(seconds) + 0.5))",
     "    total = int(round(float(seconds)))",
     (T_READER,), 'KILLED'),

    # ---- the report -----------------------------------------------------
    ('the-step-tables-exit-column-is-dropped', 't',
     "    out.append('| # | label | started | wall s | exit |')",
     "    out.append('| # | label | started | wall s |')",
     (T_READER,), 'KILLED'),
    ('the-step-table-loses-its-precision', 't',
     "                      ('%.3f' % r['wall_s']) if r.get('wall_s') is not None else '-',",
     "                      ('%.1f' % r['wall_s']) if r.get('wall_s') is not None else '-',",
     (T_READER,), 'KILLED'),
    ('the-tool-time-line-is-elided', 't',
     "    out.append('- Tool time (sum of wall_s): %.1f s = **%s**'\n"
     "               % (tot.tool_s, fmt_hms(tot.tool_s)))",
     "    pass",
     (T_READER,), 'KILLED'),
    ('the-stage-titles-are-crossed', 't',
     "    'R*': 'R* route',",
     "    'R*': 'L* driver',",
     (T_READER,), 'KILLED'),
    ('the-subtotals-columns-round-separately', 't',
     "        shown = round(wall, 1)\n"
     "        out.append('| %s | %d | %.1f | %s |'\n"
     "                   % (STAGE_TITLES[stage], n, shown, fmt_hms(shown)))",
     "        out.append('| %s | %d | %.1f | %s |'\n"
     "                   % (STAGE_TITLES[stage], n, wall, fmt_hms(wall)))",
     (T_READER,), 'KILLED'),
    ('report_data-loses-its-step-list', 't',
     "        'steps': [{'label': r.get('label'), 'iso_start': r.get('iso_start'),",
     "        'steps': [] and [{'label': r.get('label'), 'iso_start': r.get('iso_start'),",
     (T_READER,), 'KILLED'),

]


# Every anchor must match its target exactly once BEFORE anything is
# rewritten. A stale anchor otherwise reports BROKEN mid-run, after the
# witnesses have been paid for; this is the one second (#877). It also handles
# --verify-anchors and strips it from sys.argv, so this file needs no flag of
# its own -- the shared module is deliberately the ONLY implementation of that
# question in the tree, and a 38th hand-rolled copy here (which this file did
# carry) is exactly what it exists to stop.
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def run(tests):
    for t in tests:
        r = subprocess.run([sys.executable, '-X', 'utf8', t],
                           cwd=ROOT, capture_output=True, text=True)
        if r.returncode != 0:
            return True, os.path.basename(t)
    return False, ''


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row')
    ap.add_argument('--list', action='store_true',
                    help='name every row and exit. The anchor check has '
                         'already run by the time this prints -- '
                         'mutation_anchors.preflight() fires at IMPORT -- so a '
                         'stale anchor surfaces in a second here instead of '
                         'partway through the battery.')
    a = ap.parse_args()
    if a.list:
        for n, t, _o, _w, _tests, exp in ROWS:
            print('  %-52s %-20s %s'
                  % (n, os.path.basename(TARGETS[t]), exp))
        print('')
        print('%d row(s)' % len(ROWS))
        return 0

    # A dirty engine tree would be RESTORED to its committed text, silently
    # destroying uncommitted work. Refuse rather than help.
    dirty = subprocess.run(['git', 'diff', '--quiet', '--'] + list(TARGETS.values()),
                           cwd=ROOT).returncode
    if dirty:
        print('REFUSED: the files this battery rewrites have uncommitted '
              'changes.\nRestoring them would write the COMMITTED text back '
              'over your work. Commit first.')
        return 2

    rows = [r for r in ROWS if not a.row or r[0] == a.row]
    if not rows:
        print('no row named %r' % a.row)
        return 2
    originals = {k: open(v, encoding='utf-8').read() for k, v in TARGETS.items()}
    killed = survived = broken = disagree = 0
    try:
        for name, tgt, old, new, tests, exp in rows:
            src = originals[tgt]
            if src.count(old) != 1:
                print('  %-46s BROKEN (anchor matched %dx)' % (name, src.count(old)))
                broken += 1
                continue
            with open(TARGETS[tgt], 'w', encoding='utf-8', newline='') as fh:
                fh.write(src.replace(old, new))
            try:
                died, by = run(tests)
            finally:
                with open(TARGETS[tgt], 'w', encoding='utf-8', newline='') as fh:
                    fh.write(src)
            got = 'KILLED' if died else 'SURVIVED'
            mark = '' if got == exp else '   *** DISAGREES with ' + exp
            if got != exp:
                disagree += 1
            killed += died
            survived += not died
            print('  %-46s %-9s %s%s' % (name, got, by, mark))
    finally:
        for k, v in TARGETS.items():
            with open(v, 'w', encoding='utf-8', newline='') as fh:
                fh.write(originals[k])
    print('\n%d row(s): %d killed, %d survived, %d broken, %d disagreeing '
          'with expectation' % (len(rows), killed, survived, broken, disagree))
    return 1 if (broken or disagree) else 0


if __name__ == '__main__':
    sys.exit(main())
