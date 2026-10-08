"""run_all's shards balance on measured test durations.

A fan-out's wall-clock is its slowest shard. The strided split (every N-th
file by name) measured 2462 s on one shard against a 195 s mean over 50, since
test cost is nothing like uniform. With tests/run_all_durations.json present,
`run_all.shard` packs longest-first onto the least-loaded shard instead.

Rows:
  - for any shard count (more shards than tests included) the slices are
    disjoint and their union is every test, with or without a table;
  - with a table the slowest shard is no slower than the strided split's,
    and strictly faster on a skewed suite;
  - a test the table does not know is priced at the median of the known
    tests of ITS kind (integration or unit), or of all known tests when its
    kind has none;
  - under --fast an integration test costs 0, since it will be skipped;
  - without a table the split is exactly the old strided one;
  - a test declaring RUN_ALL_PARTS = N runs as N units, `name.py[i/N]` with
    `--part i/N`, each priced and sharded on its own;
  - a table that is not a {name: seconds} object reads as no table;
  - the committed table, when there is one, parses into seconds.

    python3 tests/test_run_all_shard_balance.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TESTS)

import run_all  # noqa: E402

# Its fixture WRITES the word subprocess into scratch files; it runs nothing.
RUN_ALL_FAST_OK = True

NAMES = [f'test_{i:03d}.py' for i in range(40)]
PATHS = [os.path.join('nowhere', n) for n in NAMES]     # classified as unit
# Skewed like the real suite: a few very slow files, many quick ones, and the
# slow ones adjacent by name (a family), which is what striding lands badly.
COST = {n: (900.0 if i in (0, 1, 2, 3) else 60.0 if i < 10 else 5.0)
        for i, n in enumerate(NAMES)}


def _slices(count, durations):
    return [run_all.shard(PATHS, i, count, durations) for i in range(count)]


def _wall(slices, cost):
    return max(sum(cost[os.path.basename(p)] for p in s) for s in slices)


def test_slices_cover_exactly_once():
    for durations in (None, COST):
        for count in (1, 3, 7, 40, 55):
            slices = _slices(count, durations)
            flat = [p for s in slices for p in s]
            assert sorted(flat) == sorted(PATHS), (count, durations is None)
            assert len(flat) == len(set(flat)), count


def test_balanced_beats_strided():
    for count in (2, 4, 8):
        strided = _wall(_slices(count, None), COST)
        balanced = _wall(_slices(count, COST), COST)
        assert balanced <= strided, (count, balanced, strided)
    assert _wall(_slices(4, COST), COST) < _wall(_slices(4, None), COST)


def _kinds_dir():
    """Real files, since is_integration reads the source: u*.py are unit
    tests, i*.py shell out."""
    d = tempfile.mkdtemp(prefix='shard_kinds_')
    for i in range(5):
        for kind, body in (('u', 'x = 1\n'), ('i', 'import subprocess\n')):
            with open(os.path.join(d, f'test_{kind}{i}.py'), 'w') as f:
                f.write(body)
    paths = sorted(os.path.join(d, n) for n in os.listdir(d))
    assert [run_all.is_integration(p) for p in paths].count(True) == 5
    return d, paths


def test_unknown_tests_get_their_kinds_median():
    d, paths = _kinds_dir()
    try:
        # Units measured at 2..4 s, integrations at 100..300 s; u4 and i4 are
        # new. One median over both kinds would price them alike (4.0 here).
        table = {'test_u0.py': 2.0, 'test_u1.py': 3.0, 'test_u2.py': 3.0,
                 'test_u3.py': 4.0, 'test_i0.py': 100.0, 'test_i1.py': 200.0,
                 'test_i2.py': 200.0, 'test_i3.py': 300.0}
        cost = {os.path.basename(p): c
                for p, c in run_all.estimated_costs(paths, table).items()}
        assert cost['test_u4.py'] == 3.0, cost
        assert cost['test_i4.py'] == 200.0, cost
        assert cost['test_i0.py'] == 100.0 and cost['test_u3.py'] == 4.0
        # A kind with nothing measured falls back to every known test.
        units_only = {k: v for k, v in table.items() if k.startswith('test_u')}
        cost = {os.path.basename(p): c
                for p, c in run_all.estimated_costs(paths, units_only).items()}
        assert cost['test_i0.py'] == 3.0, cost
        # Under --fast the integrations will be skipped: they weigh nothing,
        # so the unit tests spread across every shard instead of piling onto
        # whichever drew no heavy test.
        fast = {os.path.basename(p): c
                for p, c in run_all.estimated_costs(paths, table, fast=True).items()}
        assert all(fast[f'test_i{i}.py'] == 0.0 for i in range(5)), fast
        assert fast['test_u1.py'] == 3.0, fast
        slices = [run_all.shard(paths, i, 4, table, fast=True) for i in range(4)]
        units = [sum(not run_all.is_integration(p) for p in s) for s in slices]
        assert max(units) - min(units) <= 1, units
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_no_table_is_the_strided_split():
    for count in (3, 50):
        for i in range(count):
            assert run_all.shard(PATHS, i, count, None) == PATHS[i::count]
            assert run_all.shard(PATHS, i, count, {}) == PATHS[i::count]


def test_a_declared_split_runs_as_parts():
    """`RUN_ALL_PARTS = N` makes one test file N units, named and argued
    apart, classified by their file, priced by their own name, and sharded
    like any other test."""
    d = tempfile.mkdtemp(prefix='shard_parts_')
    try:
        whole = os.path.join(d, 'test_whole.py')
        split = os.path.join(d, 'test_split.py')
        with open(whole, 'w') as f:
            f.write('x = 1\n')
        with open(split, 'w') as f:
            f.write('import subprocess\nRUN_ALL_PARTS = 3\n')
        old = run_all.TESTS_DIR
        run_all.TESTS_DIR = d
        try:
            units = run_all.discover([])
        finally:
            run_all.TESTS_DIR = old
        assert [run_all.unit_name(u) for u in units] == [
            'test_split.py[0/3]', 'test_split.py[1/3]', 'test_split.py[2/3]',
            'test_whole.py'], units
        assert all(run_all.unit_file(u) == split for u in units[:3])
        assert run_all.unit_argv(units[1]) == ['--part', '1/3']
        assert run_all.unit_file(whole) == whole and run_all.unit_argv(whole) == []
        cost = run_all.estimated_costs(units, {'test_split.py[1/3]': 50.0,
                                               'test_whole.py': 2.0})
        assert cost[units[1]] == 50.0 and cost[units[3]] == 2.0, cost
        table = {'test_split.py[0/3]': 1.0, 'test_split.py[1/3]': 50.0,
                 'test_split.py[2/3]': 1.0, 'test_whole.py': 1.0}
        slices = [run_all.shard(units, i, 2, table) for i in range(2)]
        assert sorted(u for s in slices for u in s) == sorted(units)
        assert any(units[1] in s and len(s) == 1 for s in slices), slices
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_unusable_table_reads_as_none():
    d = tempfile.mkdtemp(prefix='shard_table_')
    try:
        path = os.path.join(d, 't.json')
        for doc, want in (([1, 2], {}), ('"x"', {}), ('{not json', {}),
                          ({'test_a.py': 2, 'test_b.py': True,
                            'test_c.py': 'slow'}, {'test_a.py': 2.0})):
            with open(path, 'w') as f:
                f.write(doc if isinstance(doc, str) else json.dumps(doc))
            assert run_all.load_durations(path) == want, (doc, want)
        assert run_all.load_durations(os.path.join(d, 'absent.json')) == {}
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_committed_table_parses():
    table = run_all.load_durations()
    if not table:
        print('    (no committed table yet -- the strided split is in use)')
        return
    assert all(isinstance(v, float) and v >= 0 for v in table.values())
    import re
    unit = re.compile(r'^test_\w+\.py(\[\d+/\d+\])?$')
    assert all(unit.match(k) for k in table), [k for k in table if not unit.match(k)]
    # Every split test's parts are measured: the writer once dropped them as
    # "files that no longer exist", pricing the heaviest units at a median.
    for f in run_all.discover([]):
        if run_all.unit_argv(f):
            assert run_all.unit_name(f) in table, run_all.unit_name(f)


TESTS_LIST = [test_slices_cover_exactly_once, test_balanced_beats_strided,
              test_unknown_tests_get_their_kinds_median,
              test_no_table_is_the_strided_split,
              test_a_declared_split_runs_as_parts,
              test_unusable_table_reads_as_none, test_committed_table_parses]


if __name__ == '__main__':
    fails = 0
    for t in TESTS_LIST:
        try:
            t()
            print(f"  PASS {t.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"  FAIL {t.__name__}: {e}")
    print('ALL PASS' if not fails else f'{fails} FAILED')
    sys.exit(1 if fails else 0)
