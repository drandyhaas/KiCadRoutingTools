#!/usr/bin/env python3
"""#713 item 2: a probe's verdict is not erased by a clock, and its absence is
never silently dropped.

`place_portfolio` and `compare_seeds` each wrapped their probe route in
`subprocess.run(timeout=...)` -- 900 s and 1800 s, two clocks with two defaults
-- and recorded the expiry as `failures: None`. BOTH tools then filter
`failures is not None` out of their rankings, so a clock did not rank a
candidate worse; it stopped it being a contender.

The issue proposes `--max-iterations` as the deterministic replacement. That is
wrong and the gate says so below: it is a per-A*-SEARCH cap, not a run budget,
exhausting it makes one net fail while the run completes normally, and it is
extended at runtime by #529. Lowering it changes the routing RESULT, not the
duration. The correct bound is scope, which both tools already pass.
"""
import ast
import inspect
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


print("--- the two clocks are gone")
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
# Each tool's OWN required arguments must be supplied, or argparse reports
# those first and the run exits 2 for a reason that has nothing to do with the
# removed flag -- an rc-only assertion would pass on that and prove nothing.
_REQUIRED = {
    'place_portfolio.py': [BOARD, '--out-dir', 'x'],
    'compare_seeds.py': [BOARD, '--out-dir', 'x', '--intent', 'i.json'],
}
for tool in ('place_portfolio.py', 'compare_seeds.py'):
    p = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_placer', tool)]
                       + _REQUIRED[tool] + ['--route-timeout', '60'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    check(f"{tool}: --route-timeout is an argparse error", p.returncode == 2,
          f"rc={p.returncode}")
    check(f"{tool}: argparse names it",
          'unrecognized arguments: --route-timeout' in (p.stdout + p.stderr),
          (p.stdout + p.stderr)[-160:])
    h = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_placer', tool), '--help'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    check(f"{tool}: --help advertises no probe timeout",
          'route-timeout' not in h.stdout)

import converge  # noqa: E402
check("scoped_route no longer takes a timeout at all",
      'timeout' not in inspect.signature(converge.scoped_route).parameters,
      "a parameter no caller passes invites a clock straight back in")

print("\n--- one probe helper, one row shape")
check("converge.probe_route exists", hasattr(converge, 'probe_route'))
check("and a declared status vocabulary",
      converge.PROBE_STATUSES == ('ok', 'crashed', 'no_summary', 'screened'),
      str(converge.PROBE_STATUSES))

_KEYS = {'failures', 'status', 'note', 'iterations', 'vias', 'nets',
         'returncode'}


class _FakeRes(dict):
    pass


_real = converge.scoped_route
try:
    # Every outcome must produce the SAME key set. The two old helpers dropped
    # different subsets on their timeout rows, which is how
    # test_compare_seeds.py became a latent KeyError instead of an assertion.
    converge.scoped_route = lambda *a, **k: {
        'returncode': 0, 'summary': {'failed_single': ['A'],
                                     'total_iterations': 5, 'total_vias': 2},
        'board': 'b', 'json': 'j', 'argv': [], 'stdout_tail': ''}
    ok_row = converge.probe_route('b', ['*'])
    converge.scoped_route = lambda *a, **k: {
        'returncode': 1, 'summary': {}, 'board': 'b', 'json': 'j',
        'argv': [], 'stdout_tail': ''}
    crash_row = converge.probe_route('b', ['*'])
    converge.scoped_route = lambda *a, **k: {
        'returncode': 0, 'summary': {}, 'board': 'b', 'json': 'j',
        'argv': [], 'stdout_tail': ''}
    nosum_row = converge.probe_route('b', ['*'])
finally:
    converge.scoped_route = _real
screened = converge.screened_row(['*'], 'route skipped: too similar')

check("a successful probe carries a verdict",
      ok_row['failures'] == 1 and ok_row['status'] == 'ok', str(ok_row))
check("a CRASH is distinguishable from a silent run",
      crash_row['status'] == 'crashed' and nosum_row['status'] == 'no_summary',
      f"{crash_row['status']} / {nosum_row['status']}")
check("a deliberate SKIP is distinguishable from a failure",
      screened['status'] == 'screened')
for name, row in (('ok', ok_row), ('crashed', crash_row),
                  ('no_summary', nosum_row), ('screened', screened)):
    check(f"the {name} row has the identical key set",
          set(row) == _KEYS, str(sorted(set(row) ^ _KEYS)))
check("every non-ok row reports failures None, and only those",
      all((r['failures'] is None) == (r['status'] != 'ok')
          for r in (ok_row, crash_row, nosum_row, screened)))

print("\n--- a contender with no verdict is NAMED, not silently dropped")
_pp = open(os.path.join(ROOT, 'py_placer', 'place_portfolio.py'),
           encoding='utf-8').read()
check("place_portfolio warns when a probed contender produced no verdict",
      'produced NO VERDICT' in _pp,
      "both tools filter `failures is not None` out of the ranking, so an "
      "absent verdict stops a candidate being a contender at all")
check("and says the winner was decided without it",
      'decided without it' in _pp)
check("a deliberate ratsnest SKIP is exempt from that warning",
      "get('status') != 'screened'" in _pp,
      "a skip is a decision, not a lost verdict")

from placement import portfolio as pf  # noqa: E402
check("select_best still returns None when everything ranked violates rule 1",
      pf.select_best([3, 4], [4, 3], {3, 4}) is None,
      "returning 0 here would suppress the caller's 'every ranked candidate "
      "violates rule 1' note -- the disclosure a reader needs before adopting")
check("the baseline still wins when it is ranked and the rest violate",
      pf.select_best([0, 3], [3], {3}) == 0)

print("\n--- compare_seeds' exit code matches its documented contract")
_cs = open(os.path.join(ROOT, 'py_placer', 'compare_seeds.py'),
           encoding='utf-8').read()
check("no winner is exit 4, unconditionally",
      "return 4 if not any(r['probe'] for r in rows) else 0" not in _cs,
      "a no-verdict row is a TRUTHY dict, so `any(r['probe'])` returned 0 "
      "with best_seed null -- docs/utilities.md promises 4")
_tree = ast.parse(_cs)
_main = next(n for n in ast.walk(_tree)
             if isinstance(n, ast.FunctionDef) and n.name == 'main')
_tail = [n for n in _main.body if isinstance(n, ast.Return)]
check("the last statement of main() returns 4",
      isinstance(_main.body[-1], ast.Return)
      and isinstance(_main.body[-1].value, ast.Constant)
      and _main.body[-1].value.value == 4,
      "the fall-through must be 'nothing rankable', not 'success'")

print("\n--- the issue's proposed replacement is NOT a run budget")
_docs = open(os.path.join(ROOT, 'docs', 'configuration.md'),
             encoding='utf-8').read()
check("--max-iterations is documented as a PER-ROUTE cap",
      'per route' in _docs.lower().split('--max-iterations')[1][:300]
      if '--max-iterations' in _docs else False,
      "so exhausting it fails one net; the run completes and exits 0")
_rt = open(os.path.join(ROOT, 'py_router', 'route.py'), encoding='utf-8').read()
check("route.py still takes no wall-clock budget of its own",
      '--deadline' not in _rt)

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
