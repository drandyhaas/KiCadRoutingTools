#!/usr/bin/env python3
"""The #775 mutation battery, shipped so its numbers can be re-derived.

`tests/test_775_fanout_clearance_via_prune_refresh.py` records what each arm
kills. A count is only checkable if the exact source edit is written down --
two reviewers of the #746 branch reconstructed its rows from their names and
both got the wrong answer, because a plausible-looking reconstruction of one
row was semantically inert. So the edits live here, as data, next to the
numbers they produced.

Every row carries an EXPECTATION. An inert row recorded as an expected
survivor is a finding; an inert row quietly deleted is a hole. A row whose
verdict does not match its expectation is reported as WRONG.

ITS OWN FILE RATHER THAN ROWS IN `mutate_747.py`, deliberately. That battery's
recorded table has already been corrected once from a run, and its header says
so at length; splicing a second issue's rows into it makes that correction
unreadable. The ONE #775 row that belongs there stays there -- the ordering of
the registrar against the refresh is #747's invariant, and it is re-anchored in
place rather than duplicated here.

WHY SO MANY ROWS NAME ONLY `test_775`. #775 is a COST change: the grade is
bit-identical pruned or de-pruned, which is the whole claim. So most mutations
here cannot be caught by a number moving, only by a mechanism arm -- and a
battery that graded them against the whole family would spend forty minutes
proving that four other files are indifferent. Each row names the smallest set
that can actually see it.

One target file (`py_placer/placement/fanout_clearance.py`).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. The file refuses to start on a
dirty engine, because restoring would write the COMMITTED text back over
uncommitted work.

    python3 tests/mutate_775.py
    python3 tests/mutate_775.py --row revert-the-refresh-to-a-de-prune
    python3 tests/mutate_775.py --list

A row is KILLED by a FAILURE **or an ERROR**: several of these mutations make
an arm raise rather than fail, and a battery that counted only failures would
call that a survivor.

An anchor that does not match EXACTLY ONCE is reported as BROKEN rather than
skipped -- a battery that silently applies nothing reports every row as a
survivor, which reads as a catastrophic test failure and is really a stale
anchor. `str.replace(old, new, 1)` of an absent needle returns the file
unchanged, which is why the count is checked BEFORE the write.

Python `str.replace`, never `sed`: commit `bb8f4477` records two rows of
`mutate_761` leaving a `SyntaxError` behind because `sed` ate an unescaped
metacharacter, and a battery that cannot start reports nothing at all.

THE MEASURED TABLE IS IN THE HEADER OF `test_775_...py`, FROM THE RUN -- never
predicted here and never edited afterwards to match. A battery whose
expectations are rewritten to agree with its results measures nothing.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

FC = os.path.join(_ROOT, 'py_placer', 'placement', 'fanout_clearance.py')
TARGETS = {'fc': FC}

T775 = os.path.join(_TESTS, 'test_775_fanout_clearance_via_prune_refresh.py')
T736 = os.path.join(_TESTS, 'test_736_fanout_clearance_regrade_view.py')
T746 = os.path.join(_TESTS, 'test_746_fanout_clearance_resolved_credit.py')
T725 = os.path.join(_TESTS, 'test_725_fanout_clearance_pad_floors.py')

# --- the two lines of the refresh that most rows edit ----------------------
_ASSIGN = ('            self.cap_vias[ref] = (list(self.vias) if geom is None\n'
           '                                  else self._prune_vias(cap, geom,'
           ' self.vias))\n')
_GEOM_THEN_ASSIGN = ('            geom = self._cap_geom.get(ref)\n' + _ASSIGN)
_LOOP = '        n = 0\n        for ref, cap in self.caps.items():\n'
_INIT_CALL = ('            self.cap_vias[ref] = self._prune_vias(cap,'
              ' cap_geom[ref],\n'
              '                                                  self.vias)\n')

ROWS = [
    # ---- the defect itself -------------------------------------------------
    ('revert-the-refresh-to-a-de-prune', 'fc',
     _ASSIGN, '            self.cap_vias[ref] = self.vias\n',
     (T775, T746), 'KILLED'),

    ('delete-the-refresh-call', 'fc',
     '            st.refresh_cap_vias()\n', '',
     (T775, T736, T746), 'KILLED'),

    ('refresh-only-the-first-cap', 'fc',
     _LOOP, '        n = 0\n'
            '        for ref, cap in list(self.caps.items())[:1]:\n',
     (T775,), 'KILLED'),

    # ---- the SEED-pose anchor, which is what makes the prune exact ---------
    ('prune-from-the-MOVED-pose', 'fc',
     _GEOM_THEN_ASSIGN,
     '            geom = (cap.rect()[0], cap.rect()[1],\n'
     '                    self._cap_geom[ref][2], None) \\\n'
     '                if ref in self._cap_geom else None\n' + _ASSIGN,
     (T775,), 'KILLED'),

    # ---- each term of the predicate ----------------------------------------
    # The cap-side slack is #725's, and before #775 a mutation could zero it in
    # __init__ and leave a second, untouched copy running at the refresh.
    ('drop-the-via_slack-term', 'fc',
     '        via_slack = max(0.0, cap.max_floor - self.clearance)\n',
     '        via_slack = 0.0\n',
     (T725, T775), 'KILLED'),

    ('drop-the-keepout-term', 'fc',
     '                    self._max_disp_cap + span + v[3] + via_slack):\n',
     '                    self._max_disp_cap + span + 0.0 + via_slack):\n',
     (T775,), 'KILLED'),

    ('drop-the-span-term', 'fc',
     '                    self._max_disp_cap + span + v[3] + via_slack):\n',
     '                    self._max_disp_cap + 0.0 + v[3] + via_slack):\n',
     (T775,), 'KILLED'),

    ('flat-clearance-instead-of-the-keepout', 'fc',
     '                    self._max_disp_cap + span + v[3] + via_slack):\n',
     '                    self._max_disp_cap + span + self.clearance'
     ' + via_slack):\n',
     (T775,), 'KILLED'),

    # ---- the identity contract ---------------------------------------------
    # _via_effs memoises on the source list's IDENTITY, so an in-place extend
    # leaves a memo that still passes the identity test while being one column
    # short.
    ('extend-the-existing-list-in-place', 'fc',
     _ASSIGN,
     '            self.cap_vias[ref] += (list(self.vias) if geom is None\n'
     '                                   else self._prune_vias(cap, geom,'
     ' self.vias))\n',
     (T775,), 'KILLED'),

    # ---- the fallback the track registrar does NOT need --------------------
    ('fallback-skips-the-geomless-cap', 'fc',
     _GEOM_THEN_ASSIGN,
     '            geom = self._cap_geom.get(ref)\n'
     '            if geom is None:\n'
     '                continue\n'
     '            self.cap_vias[ref] = self._prune_vias(cap, geom, self.vias)\n',
     (T775,), 'KILLED'),

    ('fallback-aliases-the-whole-list', 'fc',
     _ASSIGN,
     '            self.cap_vias[ref] = (self.vias if geom is None\n'
     '                                  else self._prune_vias(cap, geom,'
     ' self.vias))\n',
     (T775,), 'KILLED'),

    # ---- the guard #736 owns, and the reason its arm was re-armed ----------
    # THIS ROW IS THE POINT OF THE RE-ARMING. Before #775 the mutation was
    # caught on `cap_vias[ref] is not st.vias`; a re-pruning refresh never
    # yields the board list, so that spelling went vacuous and this row would
    # have SURVIVED. It is graded by test_736 alone, because test_736 is where
    # the guard lives.
    ('guard-made-unconditional', 'fc',
     '\n        if via_moves:\n', '\n        if True:\n',
     (T736,), 'KILLED'),

    # ---- the construction call site ----------------------------------------
    ('init-prunes-from-an-empty-source', 'fc',
     _INIT_CALL,
     '            self.cap_vias[ref] = self._prune_vias(cap, cap_geom[ref],\n'
     '                                                  [])\n',
     (T725, T775), 'KILLED'),

    # ---- INERT PROBES, which must change nothing ---------------------------
    # If either of these KILLS, a guard is matching prose rather than code and
    # the source arms above are looser than they look.
    ('a-trailing-comment-names-the-de-pruned-write', 'fc',
     '            near_vias.append(v)\n',
     '            near_vias.append(v)  # self.cap_vias[ref] = self.vias\n',
     (T775,), 'SURVIVED'),

    ('refresh-iterates-in-sorted-order', 'fc',
     _LOOP,
     '        n = 0\n        for ref, cap in sorted(self.caps.items()):\n',
     (T775,), 'SURVIVED'),
]


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
            edits = old if isinstance(old, list) else [(old, new)]
            counts = [base.count(o) for o, _n in edits]
            if counts != [1] * len(edits):
                results.append((name, 'BROKEN', expect,
                                'anchors matched %s times' % counts, []))
                print('  ran %-44s BROKEN' % name)
                continue
            mutated = base
            for o, nw in edits:
                mutated = mutated.replace(o, nw, 1)
            io.open(path, 'w', encoding='utf-8', newline='').write(mutated)
            killed, failed = False, []
            for t in tests:
                p = subprocess.run([sys.executable, '-X', 'utf8', t],
                                   capture_output=True, text=True,
                                   encoding='utf-8', errors='replace',
                                   timeout=2400, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                if p.returncode:
                    killed = True
                failed += ['%s::%s' % (os.path.basename(t)[5:8],
                                       l.split(' ')[1].split('(')[0].strip())
                           for l in out.splitlines()
                           if l.startswith(('FAIL:', 'ERROR:'))]
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed))
            print('  ran %-44s %s' % (name, results[-1][1]))
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])

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
    ap.add_argument('--row', help='run a single row by name')
    ap.add_argument('--list', action='store_true', help='print row names only')
    a = ap.parse_args()
    if a.list:
        for r in ROWS:
            print('%-44s %s' % (r[0], r[5]))
        return 0
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
