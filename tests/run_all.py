#!/usr/bin/env python3
"""#382 E7: run the bare-__main__ test scripts and aggregate their exit codes.

The suite is 100+ standalone `if __name__ == "__main__": sys.exit(main())`
scripts (0 = pass, non-zero = fail). This runner discovers them, runs each as
`python3 tests/test_*.py` from the repo root (so the sys.path / kicad_files
conventions hold), and reports a pass/fail/skip summary. Exit code is 0 iff
every non-skipped test passed -- the same convention the individual scripts use.

Usage:
    python3 tests/run_all.py                 # run everything
    python3 tests/run_all.py --fast          # skip integration (CLI/board) tests
    python3 tests/run_all.py pad via         # only files whose name matches a term
    python3 tests/run_all.py --list          # print classification, run nothing
    python3 tests/run_all.py --timeout 300   # per-test timeout (seconds)
    python3 tests/run_all.py -j 1            # serial (default runs 4 in parallel)

A test is "integration" (slow; skipped by --fast) if its source shells out --
it imports run_utils or uses subprocess. That auto-classification needs no
maintained list; a new pytest-style test can instead use @pytest.mark.integration.
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import time

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)

# Runner/helper modules that match test_*.py? None do, but be explicit.
_EXCLUDE = {'run_all.py', 'run_utils.py', 'run_doc_examples.py', 'conftest.py', 'synth.py'}

_INTEGRATION_MARKERS = ('import run_utils', 'from run_utils', 'subprocess')

# A test exits with this when it CANNOT run -- a fixture it needs is absent --
# as distinct from passing. 77 is the autotools convention. Before this, a test
# that printed "SKIP: ..." and exited 0 was indistinguishable from a green one,
# which is how a headline acceptance test reported PASS on every clone while
# asserting nothing. A self-skip gets its own bucket and never counts as a pass.
SKIP_EXIT = 77

#: A test may declare its own budget with a module-level
#: `RUN_ALL_TIMEOUT = <seconds>`. Read from the SOURCE, not by importing --
#: importing a test runs it.
#:
#: Why this exists: three tests carried internal budgets ABOVE this runner's
#: cap (test_compare_seeds sets timeout=3600 on its own subprocess,
#: test_obstacle_map_balance 1200) and were killed at 600 s before their own
#: deadline could fire -- so they produced no partial result and no
#: diagnosis, and a machine-speed fact was reported as a code fact. Measured:
#: test_obstacle_map_balance passes ALONE in 681 s with all 18 checks green.
#:
#: A declared budget is a claim the test makes about itself, in the test,
#: where the next reader will look. Raising the GLOBAL --timeout instead
#: would hide a genuinely hung test behind the slow ones.
_BUDGET_RE = re.compile(r'^RUN_ALL_TIMEOUT\s*=\s*([0-9.]+)', re.M)


def _declared_budget(path, default):
    try:
        m = _BUDGET_RE.search(open(path, encoding='utf-8',
                                   errors='replace').read())
    except OSError:
        return default
    return max(float(m.group(1)), default) if m else default



def is_integration(path: str) -> bool:
    try:
        src = open(path, encoding='utf-8').read()
    except OSError:
        return False
    return any(m in src for m in _INTEGRATION_MARKERS)


def discover(filters):
    files = sorted(glob.glob(os.path.join(TESTS_DIR, 'test_*.py')))
    out = []
    for f in files:
        if os.path.basename(f) in _EXCLUDE:
            continue
        if filters and not any(term in os.path.basename(f) for term in filters):
            continue
        out.append(f)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('filters', nargs='*', help='only run files whose name contains a term')
    ap.add_argument('--fast', action='store_true', help='skip integration (CLI/board) tests')
    ap.add_argument('--timeout', type=float, default=600.0, help='per-test timeout in seconds')
    ap.add_argument('--jobs', '-j', type=int, default=4,
                    help='run this many tests in parallel (default 4; 1 = serial)')
    ap.add_argument('--list', action='store_true', help='list tests + classification, run nothing')
    args = ap.parse_args()

    tests = discover(args.filters)
    if not tests:
        print('No tests matched.')
        return 1

    if args.list:
        for f in tests:
            kind = 'integration' if is_integration(f) else 'unit'
            print(f'{kind:12s} {os.path.basename(f)}')
        print(f'\n{len(tests)} tests '
              f'({sum(is_integration(f) for f in tests)} integration).')
        return 0

    passed, failed, skipped = [], [], []
    to_run = []
    for f in tests:
        name = os.path.basename(f)
        if args.fast and is_integration(f):
            skipped.append(name)
            print(f'SKIP  {name}  (integration; --fast)')
            continue
        to_run.append(f)

    jobs = max(1, args.jobs)
    if jobs > 1 and to_run:
        # Pre-build the shared fixture boards ONCE, serially, before fanning
        # out: fixture_boards.ensure() builds into a shared kicad_files/ path,
        # and two workers racing the same build would collide (the module's
        # own __main__ exists exactly for this pre-build).
        try:
            subprocess.run([sys.executable,
                            os.path.join(TESTS_DIR, 'fixture_boards.py')],
                           cwd=ROOT, capture_output=True, text=True,
                           timeout=args.timeout)
        except subprocess.TimeoutExpired:
            print('WARN  fixture pre-build timed out; continuing')

    def run_one(f):
        name = os.path.basename(f)
        budget = _declared_budget(f, args.timeout)
        try:
            # `text=True` alone decodes with the LOCALE default (cp1252 on
            # Windows) and raises UnicodeDecodeError in the reader thread the
            # moment any child prints a byte it cannot decode -- a degree sign,
            # an ohm, a micro. That killed the whole runner mid-suite with a
            # threading traceback and NO summary, which reads as "the tests
            # crashed" rather than "the runner cannot read them". Every other
            # subprocess call in this repo already pins utf-8 + replace.
            r = subprocess.run([sys.executable, '-X', 'utf8', f], cwd=ROOT,
                               capture_output=True, text=True,
                               encoding='utf-8', errors='replace',
                               timeout=budget)
        except subprocess.TimeoutExpired:
            return name, ('timeout', budget), (f'TIME  {name}  (timeout after '
                                f'{budget:.0f}s -- NOT a failed '
                                f'assertion; re-run it alone before treating '
                                f'it as one)')
        if r.returncode == SKIP_EXIT:
            # A test that cannot run (a fixture it needs is absent) must not
            # report PASS. Before this existed, `sys.exit(0)` after printing
            # "SKIP: ..." was indistinguishable from a green run -- which is
            # how the placement branch's headline acceptance test reported
            # PASS on every clone while asserting nothing, for weeks.
            why = ''
            for line in (r.stdout or '').splitlines():
                if line.strip().upper().startswith('SKIP'):
                    why = line.strip()
                    break
            return name, 'skip', f'SKIP  {name}  ({why or "self-skipped"})'
        if r.returncode == 0:
            return name, True, f'PASS  {name}'
        tail = (r.stdout or '')[-800:] + (r.stderr or '')[-800:]
        return name, False, f'FAIL  {name}  (exit {r.returncode})\n{tail}'

    timed_out = []
    self_skipped = []

    def record(name, ok, line):
        if ok == 'skip':
            # NOT a pass and NOT a timeout: the test declined to run. Kept in
            # its own bucket so the summary cannot read as green.
            self_skipped.append(name)
        elif isinstance(ok, tuple) and ok and ok[0] == 'timeout':
            # Carry the budget this test ACTUALLY got. The summary used to
            # print the global --timeout for every row, so a test killed at
            # its own declared 1800s was reported as "Timed out at 600s" --
            # which sends the reader to raise a cap the test had already been
            # given 3x of. The per-test TIME line was right all along; only
            # the line people read was wrong.
            timed_out.append((name, ok[1]))
        elif ok:
            passed.append(name)
        else:
            failed.append(name)
        print(line)

    t0 = time.time()
    if jobs == 1:
        for f in to_run:
            record(*run_one(f))
    else:
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(max_workers=jobs) as ex:
            for name, ok, line in ex.map(run_one, to_run):
                record(name, ok, line)

    dt = time.time() - t0
    # A TIMEOUT AND A FAILED ASSERTION ARE DIFFERENT FACTS: a timeout moving
    # in or out of the list is a machine-speed fact, not a code fact, so it
    # gets its own bucket and never joins the exit-deciding `failed` count on
    # its own -- but the exit code still goes non-zero, because an unfinished
    # suite is not a green one.
    if self_skipped:
        print(f'\nSELF-SKIPPED ({len(self_skipped)}) -- these asserted '
              f'NOTHING; they are not passes:')
        for n in self_skipped:
            print(f'  {n}')
    print(f'\n{len(passed)} passed, {len(failed)} failed, '
          f'{len(timed_out)} timed out, {len(skipped)} skipped '
          f'(+{len(self_skipped)} self-skipped) in {dt:.1f}s')
    if failed:
        print('Failed: ' + ', '.join(failed))
    if timed_out:
        print('Timed out: ' + ', '.join(
            f'{n} (at its own {b:.0f}s budget)' if b > args.timeout
            else f'{n} (at {b:.0f}s)' for n, b in timed_out))
        print('  A timeout is not evidence of a broken test. Re-run each one '
              'alone (or raise --timeout) before recording it as a failure.')
    return 1 if (failed or timed_out) else 0


if __name__ == '__main__':
    sys.exit(main())
