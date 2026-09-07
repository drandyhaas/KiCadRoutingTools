#!/usr/bin/env python3
"""The #904 mutation battery, shipped so its numbers can be re-derived.

`tests/test_904_*.py` claim to guard the close-out's ordering, its lens binding
and the not-a-lap predicate. A claim like that is only checkable if the exact
source edit that should break each test is written down, so the edits live here
as data, next to the numbers they produced.

The rows are the load-bearing lines, one per mechanism, chosen so that each one
is a plausible SIMPLIFICATION rather than obvious vandalism -- the shape a
future reader would actually write. Three of them were reached in this branch's
own review: a `--lens-file` slot reverted to `--lens`, the `_is_lap` clause that
only one direct predicate call touched, and the GUI label copy whose test proved
nothing until the import was blocked.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. Every row is restored in a
`finally`, and each battery refuses to start on a dirty file.

    python3 tests/mutate_904.py                  # every battery
    python3 tests/mutate_904.py converge         # or one of them
    python3 tests/mutate_904.py --row spec-mapping-deleted
    python3 tests/mutate_904.py --verify-anchors # one second, mutates nothing

Each row is (name, exact_old_text, exact_new_text). An anchor that does not
match EXACTLY ONCE is reported as BROKEN rather than skipped -- a battery that
silently applies nothing reports every row as a survivor, which reads as a
catastrophic test failure and is really a stale anchor.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
sys.path.insert(0, _TESTS)

CONVERGE = os.path.join(_ROOT, 'py_placer', 'converge.py')
DRIVER = os.path.join(_ROOT, '.claude', 'skills',
                      'plan-pcb-placement-and-routing', 'scripts',
                      'loop_driver.py')
GUI = os.path.join(_ROOT, 'kicad_routing_plugin', 'placement_run.py')

LENS_TEST = os.path.join(_TESTS, 'test_904_lens_file_binding.py')
LAP_TEST = os.path.join(_TESTS, 'test_904_not_a_lap.py')
COVER_TEST = os.path.join(_TESTS, 'test_904_lens_components_cover_blocking.py')
ORDER_TEST = os.path.join(_TESTS, 'test_904_closeout_order.py')

# --- converge: the arithmetic and the binding -----------------------------
CONVERGE_ROWS = [
    # the defect itself: `spec` mapped to nothing, so a PASS on it could
    # contradict no number in its own row
    ('spec-mapping-deleted',
     "    'spec': ('impedance', 'floorplan', 'length', 'net_widths'),\n",
     ''),
    # the lens that literally grades check_floorplan
    ('intent-mapping-deleted',
     "    'intent': ('floorplan',),\n",
     ''),
    # a --final row is the RECORD of a verdict; counting it as a lap lets it
    # move the verdict it records
    ('is_lap-final-clause-deleted',
     """    if row.get('final'):
        return False
""",
     ''),
    # a cross-half declaration is not this half going back to work
    ('is_lap-exhausted-clause-deleted',
     """    if isinstance(row.get('exhausted'), dict):
        return False
""",
     ''),
    # the supersession call site -- the sharper half, and the one the issue
    # does not name
    ('declaration-supersession-back-to-_HALF',
     "        elif found is not None and _is_lap(r, half):",
     "        elif found is not None and _HALF.get(r.get('kind')) == half:"),
    # the window call site
    ('half_state-window-back-to-_HALF',
     "        if not _is_lap(r, half):\n            continue",
     "        if _HALF.get(r.get('kind')) != half:\n            continue"),
    # the third call site, which no issue named: the close-out becomes the
    # baseline the next lap is measured against
    ('record-lookback-back-to-_HALF',
     "            if _r.get('accepted') and _is_lap(_r, _half) \\",
     "            if _r.get('accepted') and _HALF.get(_r.get('kind')) == _half \\"),
    # the selection rule: skipping to a well-formed verdict below a malformed
    # one is the normalisation verifier-prompts.md forbids
    ('read_lens_file-skips-to-a-well-formed-line',
     "            if line.strip().startswith('VERDICT='):",
     "            if re.match(_LENS_RE, line.strip()):"),
    # hashing the LINE lets the finding and evidence around it be rewritten
    # with the row still verifying
    ('lens_source-hashes-the-line-not-the-file',
     "        _lens_src.append({'path': _p, 'abspath': os.path.abspath(_p),\n"
     "                          'sha256': sha256_file(_p), 'line': _no})",
     "        import hashlib as _h\n"
     "        _lens_src.append({'path': _p, 'abspath': os.path.abspath(_p),\n"
     "                          'sha256': _h.sha256(_line.encode()).hexdigest(),\n"
     "                          'line': _no})"),
    # the whole point: a close-out verdict with no artifact behind it
    ('unsourced-lens-gate-deleted',
     "    if a.final:\n        _unsourced = [v for v, s in zip(a.lens or [], _lens_src)",
     "    if False:\n        _unsourced = [v for v, s in zip(a.lens or [], _lens_src)"),
    # scoping it to completion is the `--kind systemic --final` bypass, which
    # _cross_check then reads as the live claim
    ('unsourced-gate-scoped-to-completion',
     "    if a.final:\n        _unsourced =",
     "    if a.final and a.kind == 'completion':\n        _unsourced ="),
    # the count contradicting its own detail
    ('unjudged_iterations-drops-unnumbered-rows',
     "            out['unjudged_iterations'] = list(unjudged_its)",
     "            out['unjudged_iterations'] = [i for i in unjudged_its\n"
     "                                          if i is not None]"),
    # the all-rejected plateau returned before the evidence was attached
    ('window_iterations-attached-after-the-early-return',
     "    out['window_iterations'] = [i for i, _a, _k, _s in window]\n"
     "    if not any(acc for _i, acc, _k, _s in window):",
     "    if not any(acc for _i, acc, _k, _s in window):"),
    # `improving` meaning "not flat" is what let a half that had declared
    # itself exhausted be reported as getting better
    ('improving-unanswerable-split-reverted',
     "        doc.update(verdict='CONTINUE', improving=_improving,\n"
     "                   unanswerable=_unanswerable,",
     "        doc.update(verdict='CONTINUE', improving=still,\n"
     "                   unanswerable=_unanswerable,"),
    # the itemisation nested back under the systemic warning, where an
    # ordinary ledger never reaches it
    ('status-itemisation-renested',
     "    for e in _unlevered:\n"
     "        print(f\"  i{e.get('iteration')}  {row_label(e)[:80]}\", file=sys.stderr)",
     "    for e in []:\n"
     "        print(f\"  i{e.get('iteration')}  {row_label(e)[:80]}\", file=sys.stderr)"),
    # the ladder that stops at the first rung
    ('row_label-stops-at-the-lever',
     """    dec = row.get('exhausted')
    if isinstance(dec, dict) and str(dec.get('reason') or '').strip():
        return 'declared exhausted: ' + str(dec['reason']).strip()""",
     "    dec = None"),
]

# --- loop_driver: the order and the paths ---------------------------------
DRIVER_ROWS = [
    # the printed command reverted to the placeholder form converge refuses
    ('lens-file-slots-back-to-bare-lens',
     "    slots = ''.join(f\"      --lens-file {verdicts[f'verdict_{lens}.txt']} \\\\\\n\"\n"
     "                    for lens in ('connectivity', 'drc', 'spec'))",
     "    slots = ''.join(f\"      --lens '<the {lens} VERDICT= line, verbatim>' \\\\\\n\"\n"
     "                    for lens in ('connectivity', 'drc', 'spec'))"),
    # a silent fallback to paths outside the run
    ('required-verdict-paths-made-optional',
     """    if missing:
        raise ValueError(""",
     """    if False:
        raise ValueError("""),
    # the close-out row goes back to being the one row every lap-renderer
    # prints as a blank
    ('close-out-lever-deleted',
     "        f'      --lever \"L5 close-out: {name}\" \\\\\\n'\n",
     ''),
    # the verdict files stop following the ledger and stop cycling
    ('verdict-artifacts-unregistered',
     "              'verdict_connectivity.txt', 'verdict_drc.txt',\n"
     "              'verdict_spec.txt', 'verdict_record.txt')",
     "              'handoff.png2')"),
    # one blanket waiver again: clearing a spurious check_complete pair also
    # clears the verifier
    ('waiver-buckets-merged',
     """    if _accept_close(a, 'agreement'):
        pairs = []
    if _accept_close(a, 'verifier'):
        vpairs = []""",
     """    if _accept_close(a, 'agreement'):
        pairs = []
        vpairs = []"""),
    # the verifier's file stops being compared with the ledger at all
    ('verifier-verdict-comparison-deleted',
     "    for _p in (getattr(a, 'verifier_verdict', None) or []):",
     "    for _p in []:"),
    # the CONTINUE header asserts "still improving" about every half again
    ('continue-header-asserts-improving',
     """        elif _una:
            _head = (f'whether {", ".join(_una)} plateaued is NOT ANSWERABLE '
                     f'-- which is not the same as "it is still improving"')""",
     """        elif _una:
            _head = f'{", ".join(_una)} is still improving'"""),
    # the freeze row becomes a placement lap again and retracts the
    # declaration before it
    ('freeze-row-back-to-kind-placement',
     "      --board {_frozen} --kind systemic \\\\",
     "      --board {_frozen} --kind placement \\\\"),
    # the stage text stops being archived beside its hash
    ('stage-text-archive-deleted',
     "                    with open(_fp, 'x', encoding='utf-8') as _fh:\n"
     "                        _fh.write(out or '')",
     "                    with open(_fp, 'x', encoding='utf-8') as _fh:\n"
     "                        pass"),
]

# --- the GUI copy of the label ladder -------------------------------------
GUI_ROWS = [
    ('gui-copy-drifts',
     '        dec = row.get("exhausted")\n'
     '        if isinstance(dec, dict) and str(dec.get("reason") or "").strip():\n'
     '            return "declared exhausted: " + str(dec["reason"]).strip()',
     '        dec = None'),
    ('gui-copy-returns-the-question-mark',
     '        return "(no lever recorded)"',
     '        return "?"'),
]

BATTERIES = {
    'converge': (CONVERGE, [LENS_TEST, LAP_TEST, COVER_TEST], CONVERGE_ROWS),
    'driver': (DRIVER, [ORDER_TEST, LENS_TEST], DRIVER_ROWS),
    'gui': (GUI, [LENS_TEST], GUI_ROWS),
}

from mutation_anchors import preflight                        # noqa: E402
preflight(__file__)


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def run(which, only=None):
    src_path, tests, rows = BATTERIES[which]
    rows = [r for r in rows if only is None or r[0] == only]
    if not rows:
        print('no row named %r in the %s battery' % (only, which))
        return 1
    if _dirty(src_path):
        # Restoring would write the COMMITTED text back over uncommitted work.
        print('REFUSING: %s has uncommitted changes. Commit or stash first -- '
              'this battery restores by overwriting.'
              % os.path.basename(src_path))
        return 2

    orig = io.open(src_path, encoding='utf-8', newline='').read()
    results = []
    try:
        for name, old, new in rows:
            n = orig.count(old)
            if n != 1:
                results.append((name, 'BROKEN', 'anchor matched %d times' % n,
                                []))
                continue
            io.open(src_path, 'w', encoding='utf-8', newline='').write(
                orig.replace(old, new, 1))
            killers = []
            for t in tests:
                # A stale .pyc outlives a size-preserving mutation applied
                # inside the same second, and every later import then reads the
                # mutant. Cheap insurance, and it has bitten this repo.
                p = subprocess.run(
                    [sys.executable, '-X', 'utf8', '-B', t],
                    capture_output=True, text=True, timeout=1800, cwd=_ROOT)
                if p.returncode:
                    killers.append(os.path.basename(t))
            io.open(src_path, 'w', encoding='utf-8', newline='').write(orig)
            results.append((name, 'KILLED' if killers else 'SURVIVED',
                            '%d' % len(killers), killers))
    finally:
        io.open(src_path, 'w', encoding='utf-8', newline='').write(orig)

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
                    help='default: all three')
    ap.add_argument('--row', help='run a single row by name')
    a = ap.parse_args()
    rc = 0
    for which in ([a.battery] if a.battery else sorted(BATTERIES)):
        rc |= run(which, a.row)
        print()
    return rc


if __name__ == '__main__':
    sys.exit(main())
