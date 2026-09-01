#!/usr/bin/env python3
"""The #830 mutation battery, shipped so its numbers can be re-derived.

Same contract as `tests/mutate_799.py`, which this copies: a row is KILLED by a
failure OR an error; an anchor that does not match EXACTLY ONCE is BROKEN,
never silently skipped; `str.replace(old, new, 1)`, never `sed`; originals
restored in a `finally`; and it REFUSES to start on a dirty engine, because
restoring would write the committed text back over uncommitted work.

Every row here reverts or weakens one specific decision in the fix, and names
the test that must notice. The two that matter most are the ones that do NOT
simply undo the change:

  * `the-guard-is-narrowed-to-JSONDecodeError` -- the clause the issue itself
    suggested. It must be killed, because json.load decodes the handle first
    and a tail cut mid-UTF-8-sequence raises UnicodeDecodeError.
  * `the-presence-test-demands-every-key` -- `any` -> `all`, which still
    refuses `{'x': 1}` and so passes the headline check, but breaks the
    older-summary degradation that test_open_single_verdict.py pins. It exists
    to prove the boundary tests are load-bearing rather than decorative.

NOT named `test_*.py`, so `tests/run_all.py` never collects it: it rewrites
engine files in place. One writer per tree.

    python3 tests/mutate_830.py
    python3 tests/mutate_830.py --row the-read-is-unguarded-again
    python3 tests/mutate_830.py --list
    python3 tests/mutate_830.py --selftest
"""
from __future__ import annotations

import argparse
import io
import os
import shutil
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

CONVERGE = os.path.join(_ROOT, 'py_placer', 'converge.py')
ROUTE = os.path.join(_ROOT, 'py_router', 'route.py')
RSUM = os.path.join(_ROOT, 'py_router', 'route_summary.py')
TARGETS = {'cv': CONVERGE, 'rt': ROUTE, 'rs': RSUM}

T830 = os.path.join(_TESTS, 'test_830_probe_summary_guard.py')
TCV = os.path.join(_TESTS, 'test_converge.py')
TOS = os.path.join(_TESTS, 'test_open_single_verdict.py')

ROWS = [
    # ---- the reader guard --------------------------------------------------
    ('the-read-is-unguarded-again', 'cv',
     "        try:\n"
     "            with open(js, encoding='utf-8') as f:\n"
     "                summary = json.load(f)\n",
     "        if True:\n"
     "            with open(js, encoding='utf-8') as f:\n"
     "                summary = json.load(f)\n",
     (T830,), 'KILLED'),

    # The anchor carries the NEXT line too: the except clause is byte-identical
    # to _load_defects' further down the file (the idiom was copied from it),
    # so the one-line form matched twice and reported BROKEN.
    ('the-guard-is-narrowed-to-JSONDecodeError', 'cv',
     "        except (OSError, ValueError) as exc:                    # noqa: BLE001\n"
     "            summary_error = f'{type(exc).__name__}: {exc}'\n",
     "        except json.JSONDecodeError as exc:                     # noqa: BLE001\n"
     "            summary_error = f'{type(exc).__name__}: {exc}'\n",
     (T830,), 'KILLED'),

    ('the-cause-is-swallowed-not-named', 'cv',
     "            summary_error = f'{type(exc).__name__}: {exc}'\n",
     "            summary_error = None\n",
     (T830,), 'KILLED'),

    ('the-key-is-conditional-on-the-outcome', 'cv',
     "            'json': js, 'summary': summary, 'summary_error': summary_error,\n",
     "            'json': js, 'summary': summary,\n",
     (T830,), 'KILLED'),

    ('the-warning-goes-to-stdout', 'cv',
     "                  f\"no summary\", file=sys.stderr)\n",
     "                  f\"no summary\")\n",
     (T830,), 'KILLED'),

    # ---- the route_verdict tripwire ----------------------------------------
    ('a-keyless-summary-scores-as-clean-again', 'cv',
     "    if not any(k in summary for k in ('failed_single', 'open_single',\n"
     "                                      'failed_multipoint',\n"
     "                                      'multipoint_pads_total',\n"
     "                                      'multipoint_pads_connected')):\n"
     "        return None, 'unreadable summary'\n",
     "    if False:\n"
     "        return None, 'unreadable summary'\n",
     (T830,), 'KILLED'),

    # `any` -> `all`. Still refuses {'x': 1}, so the headline check passes;
    # what it breaks is an older summary that predates open_single. This is
    # the row that proves the boundary tests are load-bearing.
    ('the-presence-test-demands-every-key', 'cv',
     "    if not any(k in summary for k in ('failed_single', 'open_single',\n",
     "    if not all(k in summary for k in ('failed_single', 'open_single',\n",
     (T830, TOS, TCV), 'KILLED'),

    # protected_skipped only decorates the note; admitting it would let a
    # document with no failure terms unlock the arithmetic and score 0.
    ('protected-skipped-alone-unlocks-the-verdict', 'cv',
     "                                      'multipoint_pads_connected')):\n",
     "                                      'multipoint_pads_connected',\n"
     "                                      'protected_skipped')):\n",
     (T830,), 'KILLED'),

    # ---- the publisher -----------------------------------------------------
    ('batch-route-streams-into-json-out-again', 'rt',
     "            write_summary_file(json_out, _merged)\n",
     "            with open(json_out, 'w', encoding='utf-8') as _jf:\n"
     "                json.dump(_merged if _merged is not None else {}, _jf,\n"
     "                          indent=1)\n",
     (T830,), 'KILLED'),

    ('the-publish-is-not-atomic', 'rs',
     "    tmp = f'{path}.{os.getpid()}.tmp'\n",
     "    tmp = path\n",
     (T830,), 'KILLED'),

    ('a-failed-write-leaves-its-litter', 'rs',
     "        if not published:\n"
     "            try:\n"
     "                os.remove(tmp)\n"
     "            except OSError:\n"
     "                pass\n",
     "        if False:\n"
     "            try:\n"
     "                os.remove(tmp)\n"
     "            except OSError:\n"
     "                pass\n",
     (T830,), 'KILLED'),

    # The on-disk format is a published contract: --json-out is read by
    # place_route_loop --accept-cmd, i.e. by judges this repo does not own.
    ('the-on-disk-format-drifts', 'rs',
     "    text = json.dumps({} if merged is None else merged, indent=1)\n",
     "    text = json.dumps({} if merged is None else merged, indent=2)\n",
     (T830,), 'KILLED'),

    ('a-none-merge-writes-nothing', 'rs',
     "    text = json.dumps({} if merged is None else merged, indent=1)\n",
     "    text = json.dumps(merged, indent=1)\n",
     (T830,), 'KILLED'),

    # ---- the poses row -----------------------------------------------------
    ('the-poses-row-cannot-say-what-happened', 'cv',
     "                              'nets': len(a.affected),\n"
     "                              'returncode': res['returncode'],\n"
     "                              'summary_error': res.get('summary_error')}\n",
     "                              'nets': len(a.affected)}\n",
     (T830,), 'KILLED'),
]


def _git_clean(paths):
    r = subprocess.run(['git', 'diff', '--quiet', '--'] + list(paths),
                       cwd=_ROOT)
    return r.returncode == 0


def _drop_pyc():
    """Remove every cached bytecode file under the trees we mutate.

    CPython validates a `.pyc` on the source's mtime with ONE-SECOND
    granularity and its size. Two size-preserving rows applied inside the same
    second can therefore leave the second import reading the FIRST mutant --
    reported as a survivor for a row that was never really applied. Several
    rows here are size-preserving one-token edits (`any` -> `all`, `indent=1`
    -> `indent=2`), so this is not hypothetical.
    """
    for base in (os.path.join(_ROOT, 'py_placer'),
                 os.path.join(_ROOT, 'py_router')):
        for dirpath, dirnames, _files in os.walk(base):
            if os.path.basename(dirpath) == '__pycache__':
                shutil.rmtree(dirpath, ignore_errors=True)
                dirnames[:] = []


def _run(tests):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    for t in tests:
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', t],
                           cwd=_ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', env=env)
        if r.returncode != 0:
            return True, f"{os.path.basename(t)} exit {r.returncode}"
    return False, "all named tests passed"


def _selftest():
    """Prove the .pyc defence instead of asserting it.

    Applies a size-preserving mutation to route_summary.py twice within one
    second and requires the SECOND probe to observe the SECOND mutant. The
    probe prints the published bytes, which differ per mutant -- a probe that
    printed only "it wrote something" would report OK against a stale cache.
    """
    if not _git_clean([RSUM]):
        print("REFUSED: route_summary.py is dirty", file=sys.stderr)
        return 2
    src = io.open(RSUM, encoding='utf-8').read()
    anchor = "    text = json.dumps({} if merged is None else merged, indent=1)\n"
    if src.count(anchor) != 1:
        print(f"BROKEN selftest: anchor matched {src.count(anchor)} times",
              file=sys.stderr)
        return 2
    probe = [sys.executable, '-B', '-X', 'utf8', '-c',
             "import sys, os, tempfile; sys.path.insert(0, 'py_router');"
             "from route_summary import write_summary_file as w;"
             "d = tempfile.mkdtemp(); p = os.path.join(d, 'r.json');"
             "w(p, {'a': [1]});"
             "print(repr(open(p, encoding='utf-8').read()))"]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    seen = []
    try:
        for repl in (anchor.replace('indent=1', 'indent=2'),
                     anchor.replace('indent=1', 'indent=3')):
            _drop_pyc()
            io.open(RSUM, 'w', encoding='utf-8', newline='').write(
                src.replace(anchor, repl, 1))
            r = subprocess.run(probe, cwd=_ROOT, capture_output=True,
                               text=True, encoding='utf-8', env=env)
            seen.append(r.stdout.strip())
    finally:
        _drop_pyc()
        io.open(RSUM, 'w', encoding='utf-8', newline='').write(src)
    ok = len(seen) == 2 and all(seen) and seen[0] != seen[1]
    print(f"  selftest: two same-second size-preserving mutations, probe read "
          f"{seen} -- "
          + ('OK: the second import saw the SECOND mutant' if ok else
             'the probe cannot tell the mutants apart, or the second read a '
             'STALE .pyc'))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--row', action='append', default=None)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()

    if a.list:
        for name, tgt, _o, _n, tests, exp in ROWS:
            print(f"  {exp:9} {name}  [{tgt}] "
                  f"-> {', '.join(os.path.basename(t) for t in tests)}")
        return 0
    if a.selftest:
        return _selftest()

    rows = ROWS
    if a.row:
        unknown = [n for n in a.row if n not in {r[0] for r in ROWS}]
        if unknown:
            print(f"no such row: {', '.join(unknown)}; try --list",
                  file=sys.stderr)
            return 2
        rows = [r for r in ROWS if r[0] in set(a.row)]

    if not _git_clean(TARGETS.values()):
        print("REFUSED: the target files are dirty. Restoring would write the "
              "COMMITTED text back over uncommitted work.", file=sys.stderr)
        return 2

    # The battery is only evidence if the gate passes UNMUTATED first.
    _drop_pyc()
    baseline_killed, why = _run((T830, TCV, TOS))
    if baseline_killed:
        print(f"BROKEN: the gate does not pass on the UNMUTATED tree ({why}).",
              file=sys.stderr)
        return 2

    originals = {k: io.open(p, encoding='utf-8').read()
                 for k, p in TARGETS.items()}
    verdicts = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            src = originals[tgt]
            n = src.count(old)
            if n != 1:
                verdicts.append((name, 'BROKEN', f"anchor matched {n} times"))
                print(f"  BROKEN   {name} -- anchor matched {n} times")
                continue
            _drop_pyc()
            io.open(TARGETS[tgt], 'w', encoding='utf-8', newline='').write(
                src.replace(old, new, 1))
            killed, why = _run(tests)
            io.open(TARGETS[tgt], 'w', encoding='utf-8', newline='').write(src)
            _drop_pyc()
            got = 'KILLED' if killed else 'SURVIVED'
            mark = 'ok' if got == expect else 'WRONG'
            verdicts.append((name, got, why))
            print(f"  {got:9}{'' if mark == 'ok' else ' WRONG'} "
                  f"{name} -- {why}")
    finally:
        for k, p in TARGETS.items():
            io.open(p, 'w', encoding='utf-8', newline='').write(originals[k])
        _drop_pyc()

    killed = sum(1 for _n, g, _w in verdicts if g == 'KILLED')
    survived = sum(1 for _n, g, _w in verdicts if g == 'SURVIVED')
    broken = [n for n, g, _w in verdicts if g == 'BROKEN']
    wrong = [r[0] for r, (_n, g, _w) in zip(rows, verdicts)
             if g != r[5] and g != 'BROKEN']
    print(f"\n{len(verdicts)} row(s): {killed} killed, {survived} survived, "
          f"{len(broken)} broken, {len(wrong)} disagreeing with expectation")
    if wrong:
        print("  WRONG: " + ', '.join(wrong))
    if broken:
        print("  BROKEN: " + ', '.join(broken))
    return 1 if (wrong or broken) else 0


if __name__ == '__main__':
    sys.exit(main())
