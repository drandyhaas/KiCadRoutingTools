#!/usr/bin/env python3
"""#963 mutation battery: is each gate ARMED, or does its test merely run?

    python3 tests/mutate_963.py                 # every battery
    python3 tests/mutate_963.py converge        # one
    python3 tests/mutate_963.py --row <name>
    python3 tests/mutate_963.py --list

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES engine and
skill files in place and restores them, and a suite running beside it would
grade a mutated tree. One writer per tree.

WHY THIS ISSUE NEEDS ONE PARTICULARLY. Six fresh verifiers read these six
commits, and between them they found FIVE guards that could not fail:

  * the L5 terminal ship refusal -- deleting the whole 50-line block passed
    --self-test, both dumps, and every 963/904/431/923 test, because
    `_refusal_sites` builds its obligation by AST-walking the same file it
    checks;
  * the bare-`--quiet` clause in test_431, sitting behind a `continue` on its
    own condition;
  * four discovery rows, because every test drove `_cross_check` with a
    hand-built namespace and nothing asserted that l5 CALLS discovery;
  * `_await_report` never re-running the board audits when DONE changed -- the
    branch its own commit called load-bearing;
  * and `arm_report` writing no marker, which is the whole production-caller
    claim.

Every one of those was green. So a row here is not a formality: it is the only
instrument that distinguishes "the test passes" from "the test would notice".

A row is KILLED when any named test exits non-zero -- a failed assertion and an
ERROR count the same, because a mutation that makes the graders crash is still
one the graders noticed. A row whose anchor does not match EXACTLY ONCE is
BROKEN, not skipped: an anchor that silently matches nothing reports every
mutation as killed and is the most flattering possible bug.

PER-ROW TEST LISTS. `test_431_skill_commands.py` is the only gate that can see
a missing `--quiet` in an emitted command, and it costs ~850 s, so it is named
by the three rows that need it rather than by every row in its battery.

BYTECODE. Each row rewrites a file and restores it within the same second, so
the runner drops `__pycache__` and runs every test with `-B`; without that a
later row imports an earlier row's mutant and the results are fiction.
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
sys.path.insert(0, _TESTS)

CONVERGE = os.path.join(_ROOT, 'py_placer', 'converge.py')
DRIVER = os.path.join(_ROOT, '.claude', 'skills',
                      'plan-pcb-placement-and-routing', 'scripts',
                      'loop_driver.py')
RENDER = os.path.join(_ROOT, 'py_tools', 'render_placement.py')
WATCHER = os.path.join(_ROOT, 'tests', 'stress', 'run_watch.py')

T_CLASS = os.path.join(_TESTS, 'test_963_classification_evidence.py')
T_EXH = os.path.join(_TESTS, 'test_963_exhaustion_binding.py')
T_BIND = os.path.join(_TESTS, 'test_963_one_binding_predicate.py')
T_TEMPL = os.path.join(_TESTS, 'test_963_render_templates.py')
T_MARK = os.path.join(_TESTS, 'test_963_report_marker.py')
T_LAP = os.path.join(_TESTS, 'test_904_not_a_lap.py')
T_ORDER = os.path.join(_TESTS, 'test_904_closeout_order.py')
T_898 = os.path.join(_TESTS, 'test_898_review_sheet_without_json.py')
T_431 = os.path.join(_TESTS, 'test_431_skill_commands.py')

# --- converge: the record's own gates -------------------------------------
CONVERGE_ROWS = [
    # The predicate #963 exists to make singular. Collapsing it to a bool
    # deletes cmd_record's "payload carries no board_sha" disclosure, which is
    # a different operator action from "grades a different board".
    ('binding-collapsed-to-a-bool',
     "        return ('this' if board_sha == psha else 'other', psha)",
     "        return ('this' if board_sha == psha else 'other', psha)\n"
     "    # mutant: unbound reads as other\n"
     "    if True:\n"
     "        return ('other', psha)",
     None),
    # "I could not tell" must never read as a mismatch.
    ('unknown-reads-as-other',
     "    if not board or not os.path.isfile(board):\n"
     "        return ('unknown', psha)",
     "    if not board or not os.path.isfile(board):\n"
     "        return ('other', psha)",
     None),
    # The row's own board, which is what binds a declaration to a board.
    ('declaration-drops-the-board',
     "            sha = r.get('result_sha')",
     "            sha = None",
     None),
    # The contributor's own "safe initial implementation", as a mutation. A
    # test must kill it, or the argument for reporting rather than gating is
    # unproven.
    ('any-digest-change-invalidates',
     "            if board_sha and board_sha != dec[2]:\n"
     "                out['declared_stale_board'] = dec[2]",
     "            if board_sha and board_sha != dec[2]:\n"
     "                out['declared_stale_board'] = dec[2]\n"
     "                out.update(flat=False, why='too-few-laps')",
     None),
    # A declaration is a claim about --board; an ordinary row is not.
    ('exhausted-score-mismatch-back-to-a-warning',
     "    if a.exhausted and isinstance(_score_doc, dict):",
     "    if False and isinstance(_score_doc, dict):",
     None),
    # The shape IS the decision.
    ('classification-shape-not-required',
     "    if a.kind == 'classification' and not a.shape:",
     "    if False and not a.shape:",
     None),
    # ...and the measurement that named it. Without this the gate is cleared
    # by one command that records nothing, which is how run 29's exact false
    # close-out was accepted in a verifier's reconstruction.
    ('classification-lever-not-required',
     "    if a.kind == 'classification' and not (a.lever or '').strip():",
     "    if False and not (a.lever or '').strip():",
     None),
    # A decision that was thrown away is not one the next lap can act on.
    ('a-rejected-classification-counts',
     "        if (r.get('kind') or '') == 'classification' and r.get('accepted'):",
     "        if (r.get('kind') or '') == 'classification':",
     None),
    # The LAST decision, not the first.
    ('classification-picks-the-first-not-the-last',
     "            found, idx = r, i\n    if found is None:",
     "            found, idx = found or r, idx if idx >= 0 else i\n"
     "    if found is None:",
     None),
    # convergence.md's strongest claim, on a ledger that measured nothing.
    ('final-stop-4-gate-deleted',
     "    if a.final and _stop_token in UNFIXABLE_STOPS:",
     "    if False and _stop_token in UNFIXABLE_STOPS:",
     None),
    # "This board cannot be fixed" is a claim about the whole board, so a
    # placement lap makes it as stale as a routing lap does.
    ('stop-4-counts-routing-only',
     "        _since = None if _cls is None else sum(_cls['laps_since'].values())",
     "        _since = None if _cls is None else _cls['laps_since']['routing']",
     None),
    # A reader crashing on a bad row rather than reading it as "not this half".
    ('is_lap-unhashable-kind-crashes',
     "    if _HALF.get(str(row.get('kind') or '')) != half:",
     "    if _HALF.get(row.get('kind')) != half:",
     None),
]

# --- the loop driver: the gates the stages apply --------------------------
DRIVER_ROWS = [
    # THE headline. Deleting the call is the defect restated.
    ('l5-continue-gate-deleted',
     "        _u = _unclassified_retry(a, doc)\n        if _u:\n            return _u",
     "        _u = None\n        if _u:\n            return _u",
     None),
    # The plausible "reuse the existing knob" simplification. --flat is 5 and
    # means something else.
    ('l5-threshold-reuses-flat',
     "    if since <= _LAPS_PER_CLASSIFICATION:",
     "    if since <= (getattr(a, 'flat', 5) or 5):",
     None),
    # Run 29 recorded ZERO classification rows, so "laps since the last one"
    # is vacuously false there. Without this arm the gate cannot catch the
    # case it was written for.
    ('no-classification-arm-deleted',
     "    if _cls is None:\n        since, _where = (doc.get('routing') or {}).get('laps') or 0, None",
     "    if _cls is None:\n        return None",
     None),
    # L3 at blocking == 0 refuses to classify, so a polishing run cannot
    # produce the row the gate wants.
    ('blocking-zero-conjunct-deleted',
     "    if doc.get('blocking') in (0, None):\n        return None",
     "    if False:\n        return None",
     None),
    # A placement lap after shape=placement is the decision being acted on.
    ('gate-counts-placement-laps-too',
     "        since = ((_cls.get('laps_since') or {}).get('routing')) or 0",
     "        since = sum((_cls.get('laps_since') or {}).values())",
     None),
    # The pin that keeps this evidence-bound rather than topology-bound.
    ('refusal-names-the-stage-again',
     "This is not a demand that a particular stage ran",
     "Run --stage L3 first. This is not a demand that a particular stage ran",
     None),
    # A whitespace reason waived the gate and recorded nothing.
    ('waiver-accepts-whitespace',
     "    if (getattr(a, 'accept_unclassified', None) or '').strip():",
     "    if getattr(a, 'accept_unclassified', None) is not None:",
     None),
    # #904 made "is this row a lap" ONE predicate; l4 asking a kind tuple was
    # a fourth reader, and the `routing` half of it was dead.
    ('l4-back-to-the-dead-kind-tuple',
     "    routed = [r for r in rows if _cv_is_lap(r, 'routing')]",
     "    routed = [r for r in rows\n"
     "              if (r.get('kind') or '') in ('completion', 'routing')]",
     None),
    # One unimportable module must not switch five gates off in silence.
    ('binding-import-failure-goes-silent',
     "            _BINDING_BLIND = ('the board could not be hashed from here -- '",
     "            _BINDING_BLIND = None or ('unused -- '",
     None),
    # Guarding the import and not the attribute turns an older converge into
    # a traceback on a path whose contract is that it degrades to a refusal.
    ('converge-attribute-unchecked',
     "    return converge if all(hasattr(converge, n) for n in attrs) else None",
     "    return converge",
     None),
    # Discovery is the claim the whole of item C rests on.
    ('discovery-deleted',
     "    a._discovered_verdicts, a._absent_verdicts = _discover_verdicts(a)",
     "    a._discovered_verdicts, a._absent_verdicts = [], []",
     [T_ORDER]),
    # A glob fires on eight files nobody passed, measured on run 29's disk.
    ('discovery-globs-the-work-dir',
     "    for lens in _DISCOVER_LENSES:\n        p = P.get(f'verdict_{lens}.txt')",
     "    import glob as _g\n"
     "    for _p in _g.glob(os.path.join(_work(a), 'verdict_*.txt')):\n"
     "        rows.append({'lens': None, 'path': _p, 'line': '', 'lineno': 1,\n"
     "                     'error': None, 'expected': None})\n"
     "    for lens in ():\n        p = P.get(f'verdict_{lens}.txt')",
     [T_ORDER]),
    # Naming a file claims its verdict is recorded; finding one is evidence
    # the record is not written yet. Refusing the second refuses the remedy.
    ('discovered-disagreement-refuses-again',
     "                _notes.append(\n"
     "                    f'{_base}: says {_dline.split(\";\")[0]} and iteration '",
     "                vpairs.append((_base, _dline, 'mutant'))\n"
     "                _notes.append(\n"
     "                    f'{_base}: says {_dline.split(\";\")[0]} and iteration '",
     [T_ORDER]),
    # A row quoting turn1/verdict_drc.txt is not this cycle's file.
    ('freshness-matches-basename-only',
     "                _ap = str(_src.get('abspath') or '')",
     "                _ap = ''",
     [T_ORDER]),
    # The hand-off renders' own text sends the reader to a stdout block that
    # is in neither the JSON nor the sheet.
    ('handoff-render-quieted-again',
     "      --review-sheet {_hos} --json-out {_hoj} -o {_hop}\n\n"
     "Its WHAT THIS PANEL SHOWS block is what routing is being given.",
     "      --review-sheet {_hos} --json-out {_hoj} -o {_hop} --quiet\n\n"
     "Its WHAT THIS PANEL SHOWS block is what routing is being given.",
     [T_431]),
    # The boundary whose text says FIRST, LOOK wrote its PNG beside the board.
    ('close-sheet-loses-its-o',
     "      -o wk/close_sheet_panels.png --quiet",
     "      --quiet",
     [T_431]),
]

# --- render_placement: the document must survive a sheet failure ----------
RENDER_ROWS = [
    ('sheet-failure-eats-the-document',
     "    _sheet_exit = 2 if _sheet_failed else 0\n"
     "    if _sheet_failed:",
     "    _sheet_exit = 0\n"
     "    if _sheet_failed:\n"
     "        return 2\n"
     "    if _sheet_failed:",
     None),
    ('gate-4-hides-the-sheet-refusal',
     "            return _sheet_exit or 4",
     "            return 4",
     None),
]

# --- run_watch: the second marker -----------------------------------------
WATCHER_ROWS = [
    # `return 0 or _await_report(...)` was a BAD MUTATION, not a missing test:
    # `0 or X` evaluates X, so the mutant behaved identically and reported
    # SURVIVED about a row that had changed nothing. This one restores the
    # pre-#963 contract -- exit at DONE, and never audit the report.
    ('report-marker-never-waited',
     "            return _await_report(workdir, done_path, _done_sha, truthdir,",
     "            return 0\n"
     "            return _await_report(workdir, done_path, _done_sha, truthdir,",
     None),
    ('report-audit-is-a-noop',
     "    out = []\n    if not os.path.isfile(report_path):",
     "    return []\n    out = []\n    if not os.path.isfile(report_path):",
     None),
    ('bounded-wait-removed',
     "    _deadline = time.monotonic() + report_wait if report_wait else None",
     "    _deadline = None",
     None),
    ('done-rewrite-not-detected',
     "            if now and done_sha and now != done_sha:",
     "            if False:",
     None),
    ('shipped-sha-back-to-every-hex-token',
     "    shipped = set(_DONE_SHIPPED_RE.findall(done_raw))",
     "    shipped = set(_SHA_RE.findall(done_raw))",
     None),
]

ARM_ROWS = [
    # The production caller, without which the guard never arms: the watcher
    # then waits for a file only an agent would ever write. Its own list,
    # because a battery's rows are resolved against that battery's FILE and a
    # row living in the wrong list reports STALE.
    ('report-marker-has-no-producer',
     "        with open(_marker, 'w', encoding='utf-8') as fh:",
     "        with open(_marker + '.disabled', 'w', encoding='utf-8') as fh:",
     None),
]

ARM_REPORT = os.path.join(_ROOT, 'tests', 'stress', 'arm_report.py')

BATTERIES = {
    'converge': (CONVERGE, [T_CLASS, T_EXH, T_LAP, T_BIND], CONVERGE_ROWS),
    'driver': (DRIVER, [T_CLASS, T_ORDER, T_BIND], DRIVER_ROWS),
    'render': (RENDER, [T_TEMPL, T_898], RENDER_ROWS),
    'watcher': (WATCHER, [T_MARK], WATCHER_ROWS),
    'armreport': (ARM_REPORT, [T_MARK], ARM_ROWS),
}

from mutation_anchors import preflight                        # noqa: E402
preflight(__file__)


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def _drop_pyc(path):
    cache = os.path.join(os.path.dirname(path), '__pycache__')
    if os.path.isdir(cache):
        shutil.rmtree(cache, ignore_errors=True)


def run(which, only=None):
    src_path, tests, rows = BATTERIES[which]
    rows = [r for r in rows if only is None or r[0] == only]
    if not rows:
        return None
    if _dirty(src_path):
        print('REFUSING: %s has uncommitted changes. Commit first -- this '
              'battery restores by overwriting.' % os.path.basename(src_path))
        return 2

    orig = io.open(src_path, encoding='utf-8', newline='').read()
    results = []
    try:
        for row in rows:
            name, old, new = row[0], row[1], row[2]
            extra = row[3] if len(row) > 3 else None
            n = orig.count(old)
            if n != 1:
                results.append((name, 'BROKEN', 'anchor matched %d times' % n,
                                []))
                continue
            io.open(src_path, 'w', encoding='utf-8', newline='').write(
                orig.replace(old, new, 1))
            _drop_pyc(src_path)
            killers = []
            for t in list(tests) + list(extra or []):
                p = subprocess.run(
                    [sys.executable, '-X', 'utf8', '-B', t],
                    capture_output=True, text=True, timeout=2400, cwd=_ROOT)
                if p.returncode:
                    killers.append(os.path.basename(t))
                    break          # one killer is enough; the rest cost time
            io.open(src_path, 'w', encoding='utf-8', newline='').write(orig)
            _drop_pyc(src_path)
            results.append((name, 'KILLED' if killers else 'SURVIVED',
                            '%d' % len(killers), killers))
    finally:
        io.open(src_path, 'w', encoding='utf-8', newline='').write(orig)
        _drop_pyc(src_path)

    w = max(len(r[0]) for r in results)
    for name, verdict, cnt, killers in results:
        print('%-*s  %-9s  %s' % (w, name, verdict, cnt))
        for f in killers:
            print('%s      %s' % (' ' * w, f))
    killed = sum(1 for r in results if r[1] == 'KILLED')
    broken = sum(1 for r in results if r[1] == 'BROKEN')
    print('\n%s: killed %d / %d%s'
          % (which, killed, len(results),
             ', %d BROKEN ANCHOR(S)' % broken if broken else ''))
    return 0 if killed == len(results) else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('battery', nargs='?', choices=sorted(BATTERIES),
                    help='default: every battery')
    ap.add_argument('--row', help='run a single row by name')
    ap.add_argument('--list', action='store_true',
                    help='row names and their battery, run nothing')
    a = ap.parse_args()
    if a.list:
        for which in sorted(BATTERIES):
            for row in BATTERIES[which][2]:
                print('%-10s %s' % (which, row[0]))
        return 0
    worst = 0
    for which in ([a.battery] if a.battery else sorted(BATTERIES)):
        rc = run(which, a.row)
        if rc is None:
            continue
        worst = max(worst, rc)
    if a.row and worst == 0 and not any(
            r[0] == a.row for w in BATTERIES for r in BATTERIES[w][2]):
        print('no row named %r' % a.row)
        return 1
    return worst


if __name__ == '__main__':
    sys.exit(main())
