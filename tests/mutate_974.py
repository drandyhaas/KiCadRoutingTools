#!/usr/bin/env python3
"""The #974 mutation battery: does anything notice when the report lies?

`floorplan.connector_requirements` and place_seed's three call sites exist to
say, in JSON_SUMMARY, which declared edge-connector requirements were measured
and which errors the exit code counted. Every way that report can go wrong
while still looking plausible -- `complete` read off the wrong thing, an
unmeasured requirement quietly dropped, the pinned split recomputed or
swapped, the report leaking into the exit code -- is a row here, next to the
tests that must fail when it happens.

Every row carries an EXPECTATION, and a verdict that does not match it is
reported as WRONG. An anchor that does not match its target EXACTLY ONCE is
BROKEN, checked before anything is written (`preflight`, #877).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- run it in a checkout nothing else
is reading. It refuses to start while a target has uncommitted changes,
because it restores by overwriting.

    python3 -X utf8 tests/mutate_974.py
    python3 -X utf8 tests/mutate_974.py --row complete-reads-the-grade
    python3 -X utf8 tests/mutate_974.py --list

A row is KILLED by any non-zero exit of a listed test, a raised error
included. `_uncache` is carried over from `tests/mutate_797.py`: several rows
are same-size edits, and CPython trusts a `.pyc` on (mtime seconds, size).

THE MEASURED RESULT is recorded below from the run, never predicted.

MEASURED: (filled in from the run)
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

FLOORPLAN = os.path.join(_ROOT, 'py_placer', 'placement', 'floorplan.py')
PLACE_SEED = os.path.join(_ROOT, 'py_placer', 'place_seed.py')
EVIDENCE_MAP = os.path.join(_ROOT, '.claude', 'skills',
                            'plan-pcb-placement-and-routing', 'references',
                            'evidence-map.md')
TARGETS = {'fp': FLOORPLAN, 'ps': PLACE_SEED, 'em': EVIDENCE_MAP}

T974U = os.path.join(_TESTS, 'test_974_connector_requirements.py')
T974C = os.path.join(_TESTS, 'test_974_place_seed_connector_requirements.py')
T923 = os.path.join(_TESTS, 'test_923_output_key_claims.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    # ---- `complete` ---------------------------------------------------
    ('complete-ignores-unmeasured', 'fp',
     "        'complete': not deduped and not dropped,",
     "        'complete': not dropped,",
     (T974U,), 'KILLED'),
    ('complete-ignores-dropped-bands', 'fp',
     "        'complete': not deduped and not dropped,",
     "        'complete': not deduped,",
     (T974U,), 'KILLED'),
    ('complete-reads-the-grade', 'fp',
     "        'complete': not deduped and not dropped,",
     "        'complete': graded.complete,",
     (T974U,), 'KILLED'),

    # ---- the errors: the caller's split, filtered ------------------------
    ('errors-own-is-every-grade-error', 'fp',
     "        'errors_own': [v.to_dict() for v in own\n",
     "        'errors_own': [v.to_dict() for v in graded.errors\n",
     (T974U,), 'KILLED'),
    ('own-and-pinned-swapped', 'fp',
     "        'errors_own': [v.to_dict() for v in own\n"
     "                       if v.rule == 'edge_connector'],\n"
     "        'errors_pinned': [v.to_dict() for v in pinned\n",
     "        'errors_own': [v.to_dict() for v in pinned\n"
     "                       if v.rule == 'edge_connector'],\n"
     "        'errors_pinned': [v.to_dict() for v in own\n",
     (T974U,), 'KILLED'),
    ('errors-not-filtered-to-the-rule', 'fp',
     "        'errors_own': [v.to_dict() for v in own\n"
     "                       if v.rule == 'edge_connector'],",
     "        'errors_own': [v.to_dict() for v in own\n"
     "                       if v.rule],",
     (T974U,), 'KILLED'),
    ('warnings-are-every-violation', 'fp',
     "        'warnings': [v.to_dict() for v in graded.warnings\n",
     "        'warnings': [v.to_dict() for v in graded.violations\n",
     (T974U,), 'KILLED'),
    ('warnings-not-filtered-to-the-rule', 'fp',
     "        'warnings': [v.to_dict() for v in graded.warnings\n"
     "                     if v.rule == 'edge_connector'],",
     "        'warnings': [v.to_dict() for v in graded.warnings\n"
     "                     if v.rule],",
     (T974U,), 'KILLED'),

    # ---- `unmeasured`: presence and along-edge ----------------------------
    ('presence-entry-skipped', 'fp',
     "        if ref not in evidence_refs:\n"
     "            unmeasured.append({",
     "        if ref not in evidence_refs:\n"
     "            continue\n"
     "            unmeasured.append({",
     (T974U,), 'KILLED'),
    ('presence-reason-always-the-fallback', 'fp',
     "                'reason': ('not on this board' if ref in not_found",
     "                'reason': ('not on this board' if False",
     (T974U,), 'KILLED'),
    ('along-edge-claims-skipped', 'fp',
     "        if centre is None and band is None:\n"
     "            continue",
     "        if True:\n"
     "            continue",
     (T974U,), 'KILLED'),
    ('measured-row-of-any-ref-counts', 'fp',
     "               str(row.get('ref')) == ref and row.get('declared')\n",
     "               row.get('declared')\n",
     (T974U,), 'KILLED'),
    ('measured-row-of-an-undeclared-entry-counts', 'fp',
     "               str(row.get('ref')) == ref and row.get('declared')\n",
     "               str(row.get('ref')) == ref\n",
     (T974U,), 'KILLED'),
    ('measured-row-on-another-edge-counts', 'fp',
     "               and row.get('edge') == c.get('edge')\n",
     "               and True\n",
     (T974U,), 'KILLED'),
    ('an-edgeless-claim-borrows-a-row', 'fp',
     "        if c.get('edge') is not None and any(\n",
     "        if any(\n",
     (T974U,), 'KILLED'),
    ('an-abstained-row-counts-as-measured', 'fp',
     "               and 'along_edge_offset_mm' in row\n",
     "               and True\n",
     (T974U,), 'KILLED'),
    ('abstention-outranks-a-measured-row', 'fp',
     "        if c.get('edge') is not None and any(\n",
     "        if 'edge_connectors[' + ref + '].' + claim not in "
     "graded.budget_abstained and c.get('edge') is not None and any(\n",
     (T974U,), 'KILLED'),
    ('abstention-reason-ignored', 'fp',
     '        why = graded.budget_abstained.get(f"edge_connectors[{ref}].{claim}")',
     '        why = None',
     (T974U,), 'KILLED'),

    # ---- `unmeasured`: the band and the copper conjunct -------------------
    ('legacy-band-never-listed', 'fp',
     "        if not row.get('body_measured') and not vacuous:",
     "        if False:",
     (T974U,), 'KILLED'),
    ('vacuous-exemption-dropped', 'fp',
     "        if not row.get('body_measured') and not vacuous:",
     "        if not row.get('body_measured'):",
     (T974U,), 'KILLED'),
    ('vacuous-needs-only-one-condition', 'fp',
     "        vacuous = (lim.get('max') is None\n"
     "                   and float(lim.get('min') or 0.0) <= legality.EPS)",
     "        vacuous = (lim.get('max') is None\n"
     "                   or float(lim.get('min') or 0.0) <= legality.EPS)",
     (T974U,), 'KILLED'),
    ('graded-on-dropped', 'fp',
     "                'graded_on': row.get('overhang_basis')})",
     "                })",
     (T974U,), 'KILLED'),
    ('certified-disjunct-dropped', 'fp',
     "            if (copper.get('certified') is False\n",
     "            if (False\n",
     (T974U,), 'KILLED'),
    ('outside-none-disjunct-dropped', 'fp',
     "                    or ('outside_mm' in copper\n"
     "                        and copper['outside_mm'] is None)):",
     "                    or False):",
     (T974U,), 'KILLED'),
    ('custom-edge-rules-counted', 'fp',
     "            if (copper.get('certified') is False\n",
     "            if (copper.get('rules_unmeasured')\n"
     "                    or copper.get('certified') is False\n",
     (T974U,), 'KILLED'),
    ('skipped-copper-never-listed', 'fp',
     "        elif row.get('edge') is not None and (\n",
     "        elif False and (\n",
     (T974U,), 'KILLED'),
    ('skipped-copper-ignores-the-edge-claim', 'fp',
     "        elif row.get('edge') is not None and (\n",
     "        elif (\n",
     (T974U,), 'KILLED'),
    ('copper-seen-only-as-unmeasured-pads-ignored', 'fp',
     "                or copper.get('findings') or copper.get('unmeasured')):",
     "                or copper.get('findings')):",
     (T974U,), 'KILLED'),
    ('board-wide-edge-rules-read-as-copper', 'fp',
     "                or copper.get('findings') or copper.get('unmeasured')):",
     "                or copper.get('findings') or copper.get('unmeasured')\n"
     "                or copper.get('rules_unmeasured')):",
     (T974U,), 'KILLED'),
    ('skipped-copper-ignores-whether-there-is-copper', 'fp',
     "        elif row.get('edge') is not None and (\n"
     "                copper.get('minimum_gap_mm') is not None\n"
     "                or copper.get('findings') or copper.get('unmeasured')):",
     "        elif row.get('edge') is not None:",
     (T974U,), 'KILLED'),

    # ---- order and duplicates ---------------------------------------------
    ('no-deduplication', 'fp',
     "        if key not in seen:\n",
     "        if True:\n",
     (T974U,), 'KILLED'),
    ('deduplicated-by-ref-alone', 'fp',
     "        key = tuple(sorted((k, str(v)) for k, v in u.items()))",
     "        key = (u['ref'],)",
     (T974U,), 'KILLED'),
    ('deduplicated-ignoring-the-reason', 'fp',
     "        key = tuple(sorted((k, str(v)) for k, v in u.items()))",
     "        key = tuple(sorted((k, str(v)) for k, v in u.items()\n"
     "                           if k != 'reason'))",
     (T974U,), 'KILLED'),
    ('unmeasured-in-discovery-order-reversed', 'fp',
     "    for u in sorted(unmeasured, key=lambda u: (u['ref'], u['requirement'],\n"
     "                                               str(u['reason']),\n"
     "                                               str(u.get('graded_on')))):",
     "    for u in list(reversed(unmeasured)):",
     (T974U,), 'KILLED'),

    # ---- bands, evidence, the wire -----------------------------------------
    ('dropped-band-reads-graded', 'fp',
     "    dropped = [{'ref': str(ref), 'band_max_mm': mm, 'graded': False,",
     "    dropped = [{'ref': str(ref), 'band_max_mm': mm, 'graded': True,",
     (T974U,), 'KILLED'),
    ('evidence-emptied', 'fp',
     "        'overhang_evidence': projected,",
     "        'overhang_evidence': [],",
     (T974U,), 'KILLED'),
    ('evidence-keeps-the-per-pad-lists', 'fp',
     "                copper['n_' + name] = len(copper.pop(name) or ())",
     "                copper['n_' + name] = len(copper.get(name) or ())",
     (T974U,), 'KILLED'),
    ('report-not-made-plain', 'fp',
     "        return _json_plain(_connector_requirements(graded, own, pinned,\n",
     "        return (_connector_requirements(graded, own, pinned,\n",
     (T974U,), 'KILLED'),
    ('nan-reaches-the-wire', 'fp',
     "        return float(value) if math.isfinite(value) else None",
     "        return float(value)",
     (T974U,), 'KILLED'),
    ('the-report-can-raise', 'fp',
     "    except Exception as exc:  # noqa: BLE001 -- a report must not fail the run",
     "    except ZeroDivisionError as exc:  # noqa: BLE001 -- a report must not fail the run",
     (T974U, T974C), 'KILLED'),
    ('an-unprintable-message-can-raise', 'fp',
     "        try:\n"
     "            detail = str(exc)\n"
     "        except Exception:  # noqa: BLE001 -- nor may the message of one\n"
     "            detail = '<unprintable>'\n",
     "        detail = str(exc)\n",
     (T974U,), 'KILLED'),
    ('ungraded-form-says-complete', 'fp',
     "    return {'complete': False, 'reason': str(reason)}",
     "    return {'complete': True, 'reason': str(reason)}",
     (T974U, T974C), 'KILLED'),

    # ---- place_seed: every summary, the same lists, no exit code -----------
    ('repair-summary-lacks-the-key', 'ps',
     "            summary['connector_requirements'] = floorplan.connector_requirements(\n"
     "                graded, own, pinned,\n",
     "            summary['connector_requirements_'] = floorplan.connector_requirements(\n"
     "                graded, own, pinned,\n",
     (T974C,), 'KILLED'),
    ('repair-split-swapped', 'ps',
     "                graded, own, pinned,\n"
     "                bands_dropped=",
     "                graded, pinned, own,\n"
     "                bands_dropped=",
     (T974C,), 'KILLED'),
    ('reseat-drops-not-passed', 'ps',
     "                bands_dropped=(reseat['edge_bands_dropped']\n"
     "                               if reseat is not None else None))",
     "                bands_dropped=None)",
     (T974C,), 'KILLED'),
    ('dry-run-reason-misspelt', 'ps',
     "                floorplan.connector_requirements_ungraded('dry-run'))",
     "                floorplan.connector_requirements_ungraded('dryrun'))",
     (T974C,), 'KILLED'),
    ('fresh-summary-lacks-the-key', 'ps',
     "    summary['connector_requirements'] = floorplan.connector_requirements(\n"
     "        graded, own, pinned)\n",
     "    summary['connector_requirements_'] = floorplan.connector_requirements(\n"
     "        graded, own, pinned)\n",
     (T974C,), 'KILLED'),
    ('fresh-split-bypassed', 'ps',
     "    summary['connector_requirements'] = floorplan.connector_requirements(\n"
     "        graded, own, pinned)\n",
     "    summary['connector_requirements'] = floorplan.connector_requirements(\n"
     "        graded, graded.errors, [])\n",
     (T974C,), 'KILLED'),
    ('repair-exit-code-follows-the-report', 'ps',
     "            if own:\n"
     "                exit_rc = 4\n"
     "        else:\n",
     "            if own or not summary['connector_requirements']['complete']:\n"
     "                exit_rc = 4\n"
     "        else:\n",
     (T974C,), 'KILLED'),
    ('fresh-exit-code-follows-the-report', 'ps',
     "    _reason = gate_reason(result['unseated'], own, _my_pads, _hole_delta)",
     "    _reason = gate_reason(result['unseated'] or not "
     "summary['connector_requirements']['complete'], own, _my_pads, "
     "_hole_delta)",
     (T974C,), 'KILLED'),

    # ---- the documentation gate --------------------------------------------
    ('evidence-map-key-misspelt', 'em',
     "| `connector_requirements.declared_refs` |",
     "| `connector_requirements.declared_ref` |",
     (T923,), 'KILLED'),
]

# Every anchor must match its target exactly once BEFORE anything is
# rewritten (#877).
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _uncache(path):
    """Delete the target's cached bytecode -- see this module's docstring."""
    import importlib
    import importlib.util
    try:
        cached = importlib.util.cache_from_source(path)
        if os.path.exists(cached):
            os.remove(cached)
    except (OSError, ValueError, NotImplementedError):
        pass
    importlib.invalidate_caches()


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def run(only=None):
    rows = [r for r in ROWS if only is None or r[0] == only]
    if not rows:
        print('no row named %r' % only)
        return 1
    for path in TARGETS.values():
        if _dirty(path):
            print('REFUSING: %s has uncommitted changes. Commit or stash '
                  'first -- this battery restores by overwriting.'
                  % os.path.basename(path))
            return 2

    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path, base = TARGETS[tgt], orig[tgt]
            n = base.count(old)
            if n != 1:
                results.append((name, 'BROKEN', expect,
                                'anchor matched %d times' % n, []))
                print('  ran %-52s BROKEN' % name)
                continue
            io.open(path, 'w', encoding='utf-8', newline='').write(
                base.replace(old, new, 1))
            _uncache(path)
            killed, failed = False, []
            for t in tests:
                p = subprocess.run([sys.executable, '-X', 'utf8', t],
                                   capture_output=True, text=True,
                                   encoding='utf-8', errors='replace',
                                   timeout=1800, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                if p.returncode:
                    killed = True
                failed += [l.strip()[5:].strip()[:90]
                           for l in out.splitlines()
                           if l.strip().startswith('FAIL')]
                if 'Traceback' in out:
                    failed.append('raised: '
                                  + out.strip().splitlines()[-1][:70])
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            _uncache(path)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed[:3]))
            print('  ran %-52s %s' % (name, results[-1][1]))
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])
            _uncache(v)

    print()
    w = max(len(r[0]) for r in results)
    wrong = 0
    for name, verdict, expect, cnt, failed in results:
        mark = ''
        if verdict != expect:
            mark = '   <-- WRONG, expected %s' % expect
            wrong += 1
        print('%-*s  %-9s  %-3s%s' % (w, name, verdict, cnt, mark))
        for f in failed:
            print('%s      %s' % (' ' * w, f))
    killed = sum(1 for r in results if r[1] == 'KILLED')
    survived = sum(1 for r in results if r[1] == 'SURVIVED')
    broken = sum(1 for r in results if r[1] == 'BROKEN')
    print('\n%d rows: %d killed, %d survived (%d of them expected), %d broken'
          % (len(results), killed, survived,
             sum(1 for r in results if r[1] == r[2] == 'SURVIVED'), broken))
    if wrong or broken:
        print('%d row(s) did not match their expectation' % (wrong + broken))
    return 1 if (wrong or broken) else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row', default=None, help='run one row by name')
    ap.add_argument('--list', action='store_true', help='list the row names')
    a = ap.parse_args()
    if a.list:
        for r in ROWS:
            print('%-52s %-4s %s' % (r[0], r[1], r[5]))
        return 0
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
