#!/usr/bin/env python3
"""#789 mutation battery: are the tau kernel and the slate analysis actually
covered, or do their tests merely run?

    python3 tests/mutate_789.py
    python3 tests/mutate_789.py --row tau-b-uses-tau-a-denominator
    python3 tests/mutate_789.py --list

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES engine files
in place and restores them, and a suite running beside it would grade a
mutated tree. One writer per tree.

A row is KILLED when any named test exits non-zero -- a failed assertion and an
ERROR count the same, because a mutation that makes the graders crash is still
a mutation the graders noticed. A row whose anchor does not match EXACTLY ONCE
is BROKEN, not skipped: an anchor that silently matches nothing reports every
mutation as killed and is the most flattering possible bug.

Expected SURVIVORS are declared with the reason they are not a test hole. The
measured table lives in the header of `tests/test_789_slate_harness.py`, from
the run, and is never edited to match a prediction.
"""
import argparse
import os
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

STATS = os.path.join(ROOT, 'tests', 'stress', 'rank_stats.py')
SLATE = os.path.join(ROOT, 'tests', 'stress', 'slate_study.py')
TARGETS = {'s': STATS, 'l': SLATE}

T_HARNESS = os.path.join(TESTS, 'test_789_slate_harness.py')
T_STATS = os.path.join(TESTS, 'test_703_rank_stats.py')

#: (name, target, old, new, tests, expectation)
ROWS = [
    # ---- the tau kernel -------------------------------------------------
    ('tau-b-uses-tau-a-denominator', 's',
     "    return (k['concordant'] - k['discordant']) / math.sqrt(da * db)",
     "    return (k['concordant'] - k['discordant']) / k['n0']",
     (T_STATS, T_HARNESS), 'KILLED'),
    ('tau-b-drops-the-tie-correction', 's',
     "    da = k['n0'] - k['ties_a']\n    db = k['n0'] - k['ties_b']",
     "    da = k['n0']\n    db = k['n0']",
     (T_STATS, T_HARNESS), 'KILLED'),
    ('tau-sign-is-flipped', 's',
     "    return (k['concordant'] - k['discordant']) / math.sqrt(da * db)",
     "    return (k['discordant'] - k['concordant']) / math.sqrt(da * db)",
     (T_STATS, T_HARNESS), 'KILLED'),
    # The anchor reaches the line BELOW the return, because `kendall_tau` and
    # `tau_a` open identically -- a two-line anchor matched both, and the
    # battery reported it BROKEN rather than silently mutating one of them.
    # That is the anchor rule working: an anchor that matches twice is a bug,
    # not a skip.
    ('tau-returns-zero-not-nan', 's',
     "        return NAN\n    k = tau_counts(a, b)\n    da = k['n0']",
     "        return 0.0\n    k = tau_counts(a, b)\n    da = k['n0']",
     (T_STATS,), 'KILLED'),
    ('tau-min-n-lowered-to-two', 's',
     'MIN_N_FOR_TAU = MIN_N',
     'MIN_N_FOR_TAU = 2',
     (T_STATS,), 'KILLED'),
    ('board-tau-drops-the-pooling-guard', 's',
     "    _one_board(rows, 'board_tau')",
     "    pass",
     (T_STATS,), 'KILLED'),
    ('fmt-tau-drops-the-LOO-span', 's',
     "    span = (f'LOO {fmt(lo, 0)}..{fmt(hi, 0)}' if lo == lo and hi == hi\n"
     "            else 'LOO not computed')\n"
     "    kk = f', K={k}' if k is not None else ''\n"
     "    return (f'tau={tau:+.3f} [{span}{kk}, tied_a={ties_a}, "
     "tied_b={ties_b}]')",
     "    return f'tau={tau:+.3f}'",
     (T_STATS, T_HARNESS), 'KILLED'),

    # ---- the slate analysis ---------------------------------------------
    # THE MOST VALUABLE ROW IN THE BATTERY. If nothing kills this, the study is
    # correlating the static order with its own tiebreak and every tau it has
    # ever printed is worthless.
    ('tau-measured-against-the-static-order-itself', 'l',
     "    xs = [pos[r['row_id']] for r in viable]\n"
     "    ys = [r['truth']['blocking'] for r in viable]",
     "    xs = [pos[r['row_id']] for r in viable]\n"
     "    ys = [pos[r['row_id']] for r in viable]",
     (T_HARNESS,), 'KILLED'),
    ('fire-drops-the-hpwl-conjunct', 'l',
     "    fire = [r for r in graded\n"
     "            if not r['rule1_would_bar_on_hpwl']\n"
     "            and r['truth']['blocking'] < b0]",
     "    fire = [r for r in graded\n"
     "            if r['truth']['blocking'] < b0]",
     (T_HARNESS,), 'KILLED'),
    ('a-clean-baseline-is-scored-not-classified', 'l',
     "    if b0 == 0:\n        out['verdict'] = 'cannot_fire'\n"
     "        out['reason'] = 'baseline_clean'",
     "    if False:\n        out['verdict'] = 'cannot_fire'\n"
     "        out['reason'] = 'baseline_clean'",
     (T_HARNESS,), 'KILLED'),
    ('a-none-blocking-is-read-as-zero', 'l',
     "    graded = [r for r in barred_x if r['truth'].get('blocking') is not None]",
     "    graded = [dict(r, truth=dict(r['truth'],\n"
     "                   blocking=r['truth'].get('blocking') or 0))\n"
     "             for r in barred_x]",
     (T_HARNESS,), 'KILLED'),
    ('a-saturated-board-scores-tau-zero', 'l',
     "    side = rs.constant_side(xs, ys)\n    if side:",
     "    side = rs.constant_side(xs, ys)\n    if False:",
     (T_HARNESS,), 'KILLED'),
    ('rule5-tolerates-one-dissenting-board', 'l',
     "    agrees = (n >= rs.MIN_SIGN_BOARDS and len(pos) >= max(1, n - 1) "
     "and not neg)",
     "    agrees = (n >= rs.MIN_SIGN_BOARDS and len(pos) >= max(1, n - 1))",
     (T_HARNESS,), 'KILLED'),
    ('rule5-drops-the-three-board-floor', 'l',
     "    agrees = (n >= rs.MIN_SIGN_BOARDS and len(pos) >= max(1, n - 1) "
     "and not neg)",
     "    agrees = (n >= 1 and len(pos) >= max(1, n - 1) and not neg)",
     (T_HARNESS,), 'KILLED'),
    ('cannot-fire-boards-join-the-denominator', 'l',
     "    able = [b for b, d in per.items()\n"
     "            if d['qbar'].get('verdict') in ('fires', 'does_not_fire')]",
     "    able = list(per)",
     (T_HARNESS,), 'KILLED'),
    ('the-null-rate-is-never-computed', 'l',
     "                  'qbar_null': qbar_null(rr)}",
     "                  'qbar_null': None}",
     (T_HARNESS,), 'KILLED'),
]


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
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list:
        for n, t, _o, _w, tests, exp in ROWS:
            print(f'  {n:48s} {TARGETS[t].split(os.sep)[-1]:18s} {exp}')
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
        print(f'no row named {a.row!r}')
        return 2
    originals = {k: open(v, encoding='utf-8').read() for k, v in TARGETS.items()}
    killed = survived = broken = disagree = 0
    try:
        for name, tgt, old, new, tests, exp in rows:
            src = originals[tgt]
            if src.count(old) != 1:
                print(f'  {name:48s} BROKEN (anchor matched {src.count(old)}x)')
                broken += 1
                continue
            with open(TARGETS[tgt], 'w', encoding='utf-8', newline='') as fh:
                fh.write(src.replace(old, new))
            try:
                died, by = run(tests)
            finally:
                with open(TARGETS[tgt], 'w', encoding='utf-8',
                          newline='') as fh:
                    fh.write(src)
            got = 'KILLED' if died else 'SURVIVED'
            mark = '' if got == exp else '   *** DISAGREES with ' + exp
            if got != exp:
                disagree += 1
            killed += died
            survived += not died
            print(f'  {name:48s} {got:9s} {by}{mark}')
    finally:
        for k, v in TARGETS.items():
            with open(v, 'w', encoding='utf-8', newline='') as fh:
                fh.write(originals[k])
    print(f'\n{len(rows)} row(s): {killed} killed, {survived} survived, '
          f'{broken} broken, {disagree} disagreeing with expectation')
    return 1 if (broken or disagree) else 0


if __name__ == '__main__':
    sys.exit(main())
