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
    python3 tests/run_all.py --shard 3/50    # one of 50 duration-balanced slices
    python3 tests/run_all.py --durations-out d.json   # per-test wall seconds

A `--shard` run ends with a `DURATIONS: {...}` line (wall seconds of each
test that passed or timed out); tests/stress/modal_suite/run_all_modal.py
--write-durations gathers those into tests/run_all_durations.json, which
`--shard` balances on once it is committed.

A test is "integration" (slow; skipped by --fast) if its source shells out --
it imports run_utils or uses subprocess. That auto-classification needs no
maintained list.

Each test runs with TMPDIR/TEMP/TMP pointed at a scratch dir of its own, which
is removed when it ends: `tempfile` honours those variables in the test and in
every child it spawns, so whatever a test forgets to clean up goes with it
(a full local run used to leave ~525 fixture copies, ~220 MB, in the system
temp dir). Run a single test directly to keep its temp output for a look.
"""
import argparse
import glob
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import time

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)

# Runner/helper modules that match test_*.py? None do, but be explicit.
_EXCLUDE = {'run_all.py', 'run_utils.py', 'run_doc_examples.py', 'conftest.py', 'synth.py'}

_INTEGRATION_MARKERS = ('import run_utils', 'from run_utils', 'subprocess')

#: A test may declare a module-level `RUN_ALL_FAST_OK = True` to opt OUT of the
#: auto-classification above. Read from the SOURCE, not by importing (importing
#: a test runs it) -- same convention as RUN_ALL_TIMEOUT below.
#:
#: The markers are a PROXY for "slow because it shells out", and the proxy has
#: a false-positive class: a fast unit test that makes one cheap `git ls-files`
#: call to identify the TRACKED board corpus (run_utils.corpus_boards). That is
#: milliseconds, not a CLI/board integration run. Without an override, adopting
#: that helper silently drops a test out of `--fast` -- which is exactly what
#: happened to test_run8_locked_contact.py: it went from RUNNING under --fast
#: to being skipped, so a red appeared to have been fixed partly by the test no
#: longer running there. Measured: both opt-out users take ~4 s.
#:
#: Use it only when the shelling-out really is cheap; a test that drives a
#: routing chain belongs in the integration bucket whatever it imports.
_FAST_OK_MARKER = re.compile(r'^RUN_ALL_FAST_OK\s*=\s*True\s*$', re.M)

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
#: internal limit could fire -- so they produced no partial result and no
#: diagnosis, and a machine-speed fact was reported as a code fact. Measured:
#: test_obstacle_map_balance passes ALONE in 681 s with all 18 checks green.
#:
#: A declared budget is a claim the test makes about itself, in the test,
#: where the next reader will look. Raising the GLOBAL --timeout instead
#: would hide a genuinely hung test behind the slow ones.
_BUDGET_RE = re.compile(r'^RUN_ALL_TIMEOUT\s*=\s*([0-9.]+)', re.M)


#: A test may declare `RUN_ALL_PARTS = N` (read from the source, like
#: RUN_ALL_TIMEOUT): it then runs as N units, `name.py[i/N]`, each invoked
#: with `--part i/N` under the test's own budget, and they shard like any
#: other test. It is for a test of independent rows whose length alone sets
#: the floor of every fan-out: test_placement_ab ran 2364 s of a 2405 s
#: slowest shard, against a 194 s mean over 50.
_PARTS_RE = re.compile(r'^RUN_ALL_PARTS\s*=\s*([0-9]+)', re.M)
# Joins a file to its part in a unit; no path contains it.
_PART_SEP = '::'


def _declared_parts(path):
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            m = _PARTS_RE.search(f.read())
    except OSError:
        return 1
    return max(1, int(m.group(1))) if m else 1


def unit_file(unit):
    """The test file a unit runs."""
    return unit.split(_PART_SEP, 1)[0]


def unit_name(unit):
    """What a unit is called in every line, summary and durations table."""
    f, _, part = unit.partition(_PART_SEP)
    return os.path.basename(f) + (f'[{part}]' if part else '')


def unit_argv(unit):
    """The arguments a unit's test is run with."""
    _, _, part = unit.partition(_PART_SEP)
    return ['--part', part] if part else []


def _declared_budget(path, default):
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            m = _BUDGET_RE.search(f.read())
    except OSError:
        return default
    return max(float(m.group(1)), default) if m else default



def is_integration(path: str) -> bool:
    try:
        src = open(path, encoding='utf-8').read()
    except OSError:
        return False
    if _FAST_OK_MARKER.search(src):
        return False       # declared fast despite a marker; see _FAST_OK_MARKER
    return any(m in src for m in _INTEGRATION_MARKERS)


def discover(filters):
    files = sorted(glob.glob(os.path.join(TESTS_DIR, 'test_*.py')))
    out = []
    for f in files:
        if os.path.basename(f) in _EXCLUDE:
            continue
        if filters and not any(term in os.path.basename(f) for term in filters):
            continue
        n = _declared_parts(f)
        out.extend([f] if n == 1 else
                   [f'{f}{_PART_SEP}{i}/{n}' for i in range(n)])
    return out


#: Measured per-test wall seconds, {file name: seconds}, from a fan-out run
#: (run_all_modal.py --write-durations). Read by `shard`; regenerate it when
#: the suite's cost shape moves -- a stale entry only makes a shard a little
#: less even, never wrong.
DURATIONS_FILE = os.path.join(TESTS_DIR, 'run_all_durations.json')


def load_durations(path=DURATIONS_FILE):
    """{test file name: seconds}, or {} when there is no usable table."""
    try:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}
    if not isinstance(data, dict):
        return {}
    return {k: float(v) for k, v in data.items()
            if isinstance(v, (int, float)) and not isinstance(v, bool)}


def estimated_costs(tests, durations, fast=False):
    """{test path: seconds} for balancing. A test the table knows costs what
    it measured; one it does not is priced at the median of the known tests
    of its kind (integration or unit), or of all known tests when its kind has
    none. Under --fast an integration test is skipped, so it costs 0 -- pricing
    it would pile the unit tests onto whichever shards drew no heavy ones."""
    kind = {f: is_integration(unit_file(f)) for f in tests}
    known = {True: [], False: []}
    for f in tests:
        name = unit_name(f)
        if name in durations:
            known[kind[f]].append(durations[name])

    def median(xs):
        xs = sorted(xs)
        return xs[len(xs) // 2] if xs else 1.0
    fallback = {k: median(v or known[True] + known[False]) for k, v in known.items()}
    return {f: (0.0 if fast and kind[f] else
                durations.get(unit_name(f), fallback[kind[f]]))
            for f in tests}


def shard(tests, index, count, durations=None, fast=False):
    """The `index`-th of `count` disjoint slices of `tests` (0-based index).

    With a `durations` table ({file name: seconds}) the slices are BALANCED:
    longest first, each test onto the shard with the least time so far (LPT).
    The wall-clock of a fan-out is its slowest shard, and a strided split
    measured 2462 s on one shard against a 195 s mean, because cost is nothing
    like uniform across files. A test the table does not know is priced at the
    median of the known tests of its kind (integration or unit). Deterministic:
    ties go to the lowest shard index, tests are taken in (-seconds, name)
    order, and each shard lists its tests by name. Without a table it is the
    strided split below.

    STRIDED (`tests[index::count]`), not contiguous blocks, and that is the
    whole point: `discover` returns the list SORTED BY NAME, so adjacent
    entries are the related-and-similarly-priced ones (a `test_908_*` family
    costs about the same as its siblings). Contiguous blocks would pile one
    family onto one shard and hand the neighbours nothing, so the slowest
    shard -- which is the wall-clock of the whole fan-out -- would be set by
    whichever block happened to hold the integration tests. Striding spreads
    each family across every shard.

    The union of all `count` shards is exactly `tests`, with no overlap, for
    any `count` -- including `count` > `len(tests)`, where the tail shards are
    empty. An empty shard is a legitimate result, NOT "no tests matched": a
    50-way fan-out over 30 files must report 20 empty shards green rather
    than failing 20 times.
    """
    if not durations:
        return tests[index::count]
    cost = estimated_costs(tests, durations, fast)
    loads = [0.0] * count
    mine = []
    for f in sorted(tests, key=lambda f: (-cost[f], unit_name(f))):
        j = min(range(count), key=lambda k: (loads[k], k))
        loads[j] += cost[f]
        if j == index:
            mine.append(f)
    return sorted(mine)


def _parse_shard(spec):
    """`"I/N"` -> `(I, N)`, 0-based and validated.

    Refuses out-of-range rather than silently clamping: a driver that computes
    a shard index wrong would otherwise run shard 0 fifty times and report a
    green suite that never ran 49/50ths of the tests.
    """
    try:
        i_s, n_s = spec.split('/', 1)
        i, n = int(i_s), int(n_s)
    except (ValueError, AttributeError):
        raise argparse.ArgumentTypeError(
            f'--shard wants "I/N" (0-based), got {spec!r}')
    if n < 1:
        raise argparse.ArgumentTypeError(f'--shard count must be >= 1, got {n}')
    if not (0 <= i < n):
        raise argparse.ArgumentTypeError(
            f'--shard index {i} is out of range for {n} shard(s) (want 0..{n - 1})')
    return i, n


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('filters', nargs='*', help='only run files whose name contains a term')
    ap.add_argument('--fast', action='store_true', help='skip integration (CLI/board) tests')
    ap.add_argument('--timeout', type=float, default=600.0, help='per-test timeout in seconds')
    ap.add_argument('--jobs', '-j', type=int, default=4,
                    help='run this many tests in parallel (default 4; 1 = serial)')
    ap.add_argument('--list', action='store_true', help='list tests + classification, run nothing')
    ap.add_argument('--shard', type=_parse_shard, metavar='I/N', default=None,
                    help='run only the I-th of N disjoint slices (0-based), '
                         'for fanning the suite out across machines; see '
                         'tests/stress/modal_suite/run_all_modal.py')
    ap.add_argument('--shard-by', choices=('time', 'stride'), default='time',
                    help='time (default): balance the slices on the measured '
                         'durations in tests/run_all_durations.json (stride '
                         'when there is none); stride: every N-th file by name')
    ap.add_argument('--durations-out', metavar='FILE', default=None,
                    help='also write this run\'s per-test wall seconds (passed '
                         'and timed-out tests) as JSON')
    args = ap.parse_args()

    tests = discover(args.filters)
    if not tests:
        print('No tests matched.')
        return 1

    if args.shard is not None:
        _i, _n = args.shard
        _all = len(tests)
        _table = load_durations() if args.shard_by == 'time' else {}
        tests = shard(tests, _i, _n, _table, fast=args.fast)
        # Announced on its own line so a shard's log says what it covered --
        # an aggregating driver that mis-sharded is otherwise invisible -- and
        # how it was cut: the same index over a different table is a
        # different slice.
        print(f'shard {_i}/{_n}: {len(tests)} of {_all} test file(s), '
              + (f'balanced on {len(_table)} measured durations'
                 if _table else 'strided by name'))
        if not tests:
            # NOT the 'No tests matched' error above: more shards than files
            # is a legitimate fan-out, and this shard passing vacuously is the
            # correct answer. Say it asserted nothing, so nobody reads the
            # green as coverage.
            print('shard is EMPTY (more shards than test files) -- '
                  'nothing to run, asserting nothing')
            print('\n0 passed, 0 failed, 0 timed out, 0 skipped '
                  '(+0 self-skipped) in 0.0s')
            return 0

    if args.list:
        for f in tests:
            kind = 'integration' if is_integration(unit_file(f)) else 'unit'
            print(f'{kind:12s} {unit_name(f)}')
        print(f'\n{len(tests)} tests '
              f'({sum(is_integration(unit_file(f)) for f in tests)} integration).')
        return 0

    passed, failed, skipped = [], [], []
    to_run = []
    for f in tests:
        name = unit_name(f)
        if args.fast and is_integration(unit_file(f)):
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

    # Short names on purpose: Windows paths cap at 260 characters, and tests
    # build deep trees (git object stores, run/board/stage dirs) under TEMP.
    scratch_root = tempfile.mkdtemp(prefix='krt_')

    durations = {}

    def run_one(f):
        name = unit_name(f)
        budget = _declared_budget(unit_file(f), args.timeout)
        tdir = tempfile.mkdtemp(prefix='t', dir=scratch_root)
        env = dict(os.environ, TMPDIR=tdir, TEMP=tdir, TMP=tdir)
        t_start = time.time()
        try:
            result = _run_test(f, name, budget, env)
            # Only a test that ran to the end, or to its budget, says what it
            # COSTS: a failure or a self-skip can stop in a second and would
            # price the test as cheap.
            ok = result[1]
            if ok is True or (isinstance(ok, tuple) and ok and ok[0] == 'timeout'):
                durations[name] = round(time.time() - t_start, 1)
            return result
        finally:
            _rmtree_scratch(tdir)

    def _run_test(f, name, budget, env):
        try:
            # `text=True` alone decodes with the LOCALE default (cp1252 on
            # Windows) and raises UnicodeDecodeError in the reader thread the
            # moment any child prints a byte it cannot decode -- a degree sign,
            # an ohm, a micro. That killed the whole runner mid-suite with a
            # threading traceback and NO summary, which reads as "the tests
            # crashed" rather than "the runner cannot read them". Every other
            # subprocess call in this repo already pins utf-8 + replace.
            r = subprocess.run([sys.executable, '-X', 'utf8', unit_file(f)]
                               + unit_argv(f), cwd=ROOT,
                               capture_output=True, text=True,
                               encoding='utf-8', errors='replace',
                               timeout=budget, env=env)
        except subprocess.TimeoutExpired:
            return name, ('timeout', budget), (f'TIME  {name}  (timeout after '
                                f'{budget:.0f}s -- NOT a failed '
                                f'assertion; re-run it alone before treating '
                                f'it as one)')
        if r.returncode == SKIP_EXIT:
            # A test that cannot run (a fixture or dependency it needs is
            # absent) must not report PASS. Before this existed, `sys.exit(0)`
            # after printing "SKIP: ..." was indistinguishable from a green
            # run -- which is how the placement branch's headline acceptance
            # test reported PASS on every clone while asserting nothing.
            #
            # THE REASON LINE IS THE EVIDENCE, not decoration. 77 is an
            # ordinary exit code for a crash or a propagated child status, and
            # a bucket that trusts the NUMBER alone turns any such failure
            # into a green suite -- strictly worse than the `FAIL (exit 77)`
            # it replaced. A declared skip SAYS so on stdout; anything else
            # falls through to FAIL below.
            # `SKIP:` WITH THE COLON. Bare `SKIP` matches ordinary prose --
            # this repo's tools print lines like
            # "skipping d... (already routed)" -- so a genuine crash that
            # exited 77 while being chatty was re-bucketed as a skip and the
            # suite went green. All four self-skipping tests on main print
            # the colon form.
            why = ''
            for line in (r.stdout or '').splitlines():
                if line.strip().upper().startswith('SKIP:'):
                    why = line.strip()
                    break
            if why:
                return name, 'skip', 'SKIP  %s  (%s)' % (name, why)
        if r.returncode == 0:
            return name, True, f'PASS  {name}'
        tail = (r.stdout or '')[-800:] + (r.stderr or '')[-800:]
        undeclared = ''
        if r.returncode == SKIP_EXIT:
            undeclared = ('  -- exit 77 with no "SKIP: <reason>" line on '
                          'stdout, so this is a failure, not a self-skip')
        return name, False, ('FAIL  %s  (exit %s)%s\n%s'
                             % (name, r.returncode, undeclared, tail))

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
    try:
        if jobs == 1:
            for f in to_run:
                record(*run_one(f))
        else:
            from concurrent.futures import ThreadPoolExecutor
            with ThreadPoolExecutor(max_workers=jobs) as ex:
                for name, ok, line in ex.map(run_one, to_run):
                    record(name, ok, line)
    finally:
        _rmtree_scratch(scratch_root)

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
    # Per-test wall seconds, for balancing shards: one line, after the
    # summary, on a SHARD's run only (run_all_modal.py always passes --shard
    # and collects it); a full local run would print ~30 KB of it.
    if args.shard is not None:
        print('DURATIONS: ' + json.dumps(dict(sorted(durations.items()))))
    if args.durations_out:
        with open(args.durations_out, 'w', encoding='utf-8') as f:
            json.dump(dict(sorted(durations.items())), f, indent=0)
    return 1 if (failed or timed_out) else 0


def _rmtree_scratch(path, waits=(0.25, 0.5, 1.0, 2.0)):
    """Remove a test's scratch dir, read-only files included.

    Git writes its object files read-only, and on Windows rmtree cannot unlink
    one, so a plain rmtree(ignore_errors=True) silently keeps any test's git
    fixture. Clear the bit and retry.

    A file still OPEN cannot be removed on Windows either, and a test's child
    can hold one for a moment after the test itself has exited -- measured: a
    run left two scratch dirs behind, one holding KiCad's single-instance lock
    (org.kicad.kicad/instances) from a kicad-cli it spawned, and a rerun of the
    same tests left none. So a dir that survives the first pass is retried
    after short waits (3.75 s in all, and only then). Anything still held after
    that (a child a timeout orphaned) is NAMED, never hidden.
    """
    def _writable_retry(func, p, _exc):
        try:
            os.chmod(p, stat.S_IWRITE)
            func(p)
        except OSError:
            pass
    handler = ({'onexc': _writable_retry} if sys.version_info >= (3, 12)
               else {'onerror': _writable_retry})
    shutil.rmtree(path, **handler)
    for wait in waits:
        if not os.path.exists(path):
            return
        time.sleep(wait)
        shutil.rmtree(path, **handler)
    if os.path.exists(path):
        print(f'WARN  could not fully remove scratch dir {path}')


if __name__ == '__main__':
    sys.exit(main())
