#!/usr/bin/env python3
"""The #746 mutation battery, shipped so its numbers can be re-derived.

`tests/test_746_fanout_clearance_resolved_credit.py` records a kill count per
row in its module docstring.  A count is only checkable if the exact source
edit is written down, and a review of this branch made that concrete: two
reviewers reconstructed the rows from their names and got 9 where the
docstring says 11, because a plausible-looking reconstruction of one row was
semantically inert.  So the edits live here, as data, next to the numbers they
produced.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place.  One writer per tree -- do not run it while a suite, an
A/B replay or a review is reading the same checkout.  Every row is restored in
a `finally`, and the file refuses to start on a dirty engine.

    python3 tests/mutate_746.py             # both batteries
    python3 tests/mutate_746.py engine      # or one of them
    python3 tests/mutate_746.py gui
    python3 tests/mutate_746.py --row regrazed-keyed-on-swept

Each row is (name, exact_old_text, exact_new_text).  An anchor that does not
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

ENGINE = os.path.join(_ROOT, 'py_placer', 'placement', 'fanout_clearance.py')
GUI = os.path.join(_ROOT, 'kicad_routing_plugin', 'fanout_gui.py')
ENGINE_TEST = os.path.join(_TESTS, 'test_746_fanout_clearance_resolved_credit.py')
GUI_TEST = os.path.join(_TESTS, 'gui_parity', 'test_746_cap_summary_gui.py')

# --- the engine battery: 12 rows, 12 killed ------------------------------
_REFRESH = """            swept = resolved
            was_grazing = set(unresolved)
            resolved, unresolved = _grade()
            via_resolved = [r for r in resolved if r not in swept]
            regrazed = [r for r in unresolved if r not in was_grazing]"""

ENGINE_ROWS = [
    # the defect itself: `resolved` computed before the nudge, never refreshed
    ('resolved-refresh-reverted', _REFRESH,
     """            _unused, unresolved = _grade()"""),
    # the spelling a review replaced: can only name a cap that was ALSO a
    # seed violator, so a cap that arrived clean gets no cause named
    ('regrazed-keyed-on-swept',
     "            regrazed = [r for r in unresolved if r not in was_grazing]",
     "            regrazed = [r for r in swept if r not in resolved]"),
    # credit the nudge with everything the descent did too
    ('via_resolved-is-all-of-resolved',
     "            via_resolved = [r for r in resolved if r not in swept]",
     "            via_resolved = list(resolved)"),
    # ... the same error made at the DECLARATION, where it also reaches the
    # path on which the nudger is never called
    ('via_resolved-seeded-with-resolved',
     "    via_resolved: List[str] = []",
     "    via_resolved: List[str] = list(resolved)"),
    ('deltas-swapped',
     """            via_resolved = [r for r in resolved if r not in swept]
            regrazed = [r for r in unresolved if r not in was_grazing]""",
     """            via_resolved = [r for r in swept if r not in resolved]
            regrazed = [r for r in was_grazing if r not in unresolved]"""),
    ('regrazed-print-deleted',
     """    if regrazed:
        print(f"  Re-grazed by this pass's own connector copper: "
              f"{', '.join(sorted(regrazed))}")""",
     "    pass"),
    # "(0 freed by via-nudge)" on every board that never nudges
    ('credit-clause-unconditional',
     """    _credit = (f" ({len(via_resolved)} freed by via-nudge)"
               if via_resolved else "")""",
     '    _credit = f" ({len(via_resolved)} freed by via-nudge)"'),
    # the returned order is a contract: test_736 asserts unresolved == ['C1']
    ('grade-returns-sorted',
     "        return res, unres",
     "        return sorted(res), sorted(unres)"),
    # credit caps that were never broken -> "resolved 55/14"
    ('violators0-gate-dropped',
     """            elif ref in violators0:
                res.append(ref)""",
     """            else:
                res.append(ref)"""),
    # both deltas become empty on every board, silently
    ('swept-bound-after-the-regrade',
     """            swept = resolved
            was_grazing = set(unresolved)
            resolved, unresolved = _grade()""",
     """            resolved, unresolved = _grade()
            swept = resolved
            was_grazing = set(unresolved)"""),
    # #736's ordering: the connectors must reach the track view first
    ('registrar-after-the-regrade',
     "            st.register_new_segments(new_segs)",
     "            pass  # registrar moved below"),
    # NOTE the dedent: the point of this row is that the deltas escape the
    # `if via_moves:` guard, so a run that never nudged still credits one.
    # Dedenting only the two comprehensions (leaving them semantically the
    # same) is the inert reconstruction that produced a false survivor.
    ('deltas-hoisted-out-of-the-guard', _REFRESH,
     """            resolved, unresolved = _grade()
    via_resolved = list(resolved)
    regrazed = []"""),
]

# --- the GUI battery: 5 rows, 5 killed ------------------------------------
GUI_ROWS = [
    ('old-wording-restored',
     """        summary += (f"; {len(unresolved)} still grazing foreign copper "
                    f"(via/track/pad) "
                    f"(manual: {', '.join(sorted(unresolved))})")""",
     """        summary += (f"; {len(unresolved)} could not clear a foreign via "
                    f"(manual: {', '.join(sorted(unresolved))})")"""),
    ('credit-clause-deleted',
     """    if via_resolved:
        summary += f"; {len(via_resolved)} cap(s) freed by that nudge\"""",
     "    pass"),
    ('regrazed-clause-deleted',
     """    if regrazed:
        summary += (f"; {len(regrazed)} re-grazed by this pass's own "
                    f"connector copper: {', '.join(sorted(regrazed))}")""",
     "    pass"),
    ('credit-clause-unconditional',
     """    if via_resolved:
        summary += f"; {len(via_resolved)} cap(s) freed by that nudge\"""",
     '    summary += f"; {len(via_resolved)} cap(s) freed by that nudge"'),
    # the engine's two early returns carry neither new key
    ('bare-get-no-or-default',
     "    via_resolved = result.get('via_resolved') or []",
     "    via_resolved = result['via_resolved']"),
]

BATTERIES = {
    'engine': (ENGINE, ENGINE_TEST, ENGINE_ROWS),
    'gui': (GUI, GUI_TEST, GUI_ROWS),
}


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def run(which, only=None):
    src_path, test_path, rows = BATTERIES[which]
    rows = [r for r in rows if only is None or r[0] == only]
    if not rows:
        print('no row named %r in the %s battery' % (only, which))
        return 1
    if _dirty(src_path):
        # Restoring would write the COMMITTED text back over uncommitted work.
        print('REFUSING: %s has uncommitted changes. Commit or stash first -- '
              'this battery restores by overwriting.' % os.path.basename(src_path))
        return 2

    orig = io.open(src_path, encoding='utf-8', newline='').read()
    results = []
    try:
        for name, old, new in rows:
            n = orig.count(old)
            if n != 1:
                results.append((name, 'BROKEN', 'anchor matched %d times' % n, []))
                continue
            io.open(src_path, 'w', encoding='utf-8', newline='').write(
                orig.replace(old, new, 1))
            p = subprocess.run([sys.executable, '-X', 'utf8', test_path],
                               capture_output=True, text=True, timeout=1800,
                               cwd=_ROOT)
            io.open(src_path, 'w', encoding='utf-8', newline='').write(orig)
            out = (p.stderr or '') + (p.stdout or '')
            if which == 'engine':
                failed = [l.split('(')[0].replace('FAIL: ', '')
                          .replace('ERROR: ', '').strip()
                          for l in out.splitlines()
                          if l.startswith(('FAIL:', 'ERROR:'))]
            else:
                failed = [l.strip()[5:].strip() for l in out.splitlines()
                          if l.strip().startswith('FAIL')]
            results.append((name, 'KILLED' if p.returncode else 'SURVIVED',
                            '%d' % len(failed), failed))
    finally:
        io.open(src_path, 'w', encoding='utf-8', newline='').write(orig)

    w = max(len(r[0]) for r in results)
    for name, verdict, cnt, failed in results:
        print('%-*s  %-9s  %s' % (w, name, verdict, cnt))
        for f in failed:
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
                    help='default: both')
    ap.add_argument('--row', help='run a single row by name')
    a = ap.parse_args()
    rc = 0
    for which in ([a.battery] if a.battery else sorted(BATTERIES)):
        rc |= run(which, a.row)
        print()
    return rc


if __name__ == '__main__':
    sys.exit(main())
