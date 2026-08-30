#!/usr/bin/env python3
"""The #779 mutation battery, shipped so its numbers can be re-derived.

`tests/test_779_seed_half_via_disclosure.py` records what each arm kills. A
count is only checkable if the exact source edit is written down -- two
reviewers of the #746 branch reconstructed its rows from their names and both
got the wrong answer, because a plausible-looking reconstruction of one row was
semantically inert. So the edits live here, as data, next to the numbers they
produced.

Every row carries an EXPECTATION. An inert row recorded as an expected survivor
is a finding; an inert row quietly deleted is a hole. A row whose verdict does
not match its expectation is reported as WRONG.

WHY MOST ROWS NAME ONLY `test_779`. #779 fixes a number that is COMPUTED AND
THEN DISCARDED on every tracked board: `required_rows` returns [] on the one
board that relocates barrels, because none of its caps declares a floor. So no
end-to-end output moves, and only the mechanism arms can see these mutations.
That is the finding, not a weakness of the battery -- and it is why the row
that reverts the whole fix is graded by test_779 alone while the rows that
touch shared machinery (the registrar, the radius map) are graded by #747 and
#725 too.

One target file (`py_placer/placement/fanout_clearance.py`).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. The file refuses to start on a
dirty engine, because restoring would write the COMMITTED text back over
uncommitted work.

    python3 tests/mutate_779.py
    python3 tests/mutate_779.py --row revert-the-seed-position-lookup
    python3 tests/mutate_779.py --list

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

THE MEASURED TABLE IS IN THE HEADER OF `test_779_...py`, FROM THE RUN -- never
predicted here and never edited afterwards to match.
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

T779 = os.path.join(_TESTS, 'test_779_seed_half_via_disclosure.py')
T747 = os.path.join(_TESTS, 'test_747_fanout_clearance_via_registrar.py')
T775 = os.path.join(_TESTS, 'test_775_fanout_clearance_via_prune_refresh.py')
T725 = os.path.join(_TESTS, 'test_725_fanout_clearance_pad_floors.py')

_RESOLVE = ("        at = None\n"
            "        if seed_pos and self._via_seed_xy:\n"
            "            _sx = self._via_seed_xy\n"
            "            at = [_sx[id(t)][1:] if id(t) in _sx else (t[0], t[1])\n"
            "                  for t in vias]\n")
_RECORD = ("                    _sd = self._via_seed_xy.get(id(t))\n"
           "                    self._via_seed_xy[id(moved)] = (\n"
           "                        (moved, _sd[1], _sd[2]) if _sd is not None\n"
           "                        else (moved, t[0], t[1]))\n")

ROWS = [
    # ---- the defect itself -------------------------------------------------
    ('revert-the-seed-position-lookup', 'fc',
     _RESOLVE, "        at = None\n",
     (T779,), 'KILLED'),

    ('record-nothing-at-all', 'fc',
     _RECORD, '',
     (T779,), 'KILLED'),

    ('the-via-kind-stops-asking-for-it', 'fc',
     "                     both(self._via_shortfalls, {'seed_pos': True}),\n"
     "                     None, None),\n",
     "                     both(self._via_shortfalls), None, None),\n",
     (T779,), 'KILLED'),

    # ---- each half of the substitution -------------------------------------
    ('drop-the-swap-in-the-EFF-loop', 'fc',
     "                if at is not None:\n"
     "                    vx, vy = at[j]\n"
     "                keepout = row[j]\n",
     "                keepout = row[j]\n",
     (T779,), 'KILLED'),

    # The FLAT loop runs only for a cap offering neither floors nor layers
    # (the duck-typed path). orangecrab resolves a matrix for every cap, so no
    # arm here reaches it -- recorded as an expected survivor with the reason
    # rather than deleted, which is what makes it a change detector for the day
    # a fixture starts exercising that path.
    ('drop-the-swap-in-the-FLAT-loop', 'fc',
     "                    if at is not None:\n"
     "                        vx, vy = at[j]\n"
     "                    d = _point_to_rect_dist(vx, vy, (bx0, by0, bx1, by1))\n",
     "                    d = _point_to_rect_dist(vx, vy, (bx0, by0, bx1, by1))\n",
     (T779, T725), 'SURVIVED'),

    # ---- the carry-forward, which only a second hop can see ----------------
    ('a-second-hop-overwrites-the-original-seed', 'fc',
     _RECORD,
     "                    self._via_seed_xy[id(moved)] = (\n"
     "                        moved, t[0], t[1])\n",
     (T779,), 'KILLED'),

    # ---- the id-recycling guard --------------------------------------------
    ('the-map-stops-holding-its-tuple', 'fc',
     _RECORD,
     "                    _sd = self._via_seed_xy.get(id(t))\n"
     "                    self._via_seed_xy[id(moved)] = (\n"
     "                        (None, _sd[1], _sd[2]) if _sd is not None\n"
     "                        else (None, t[0], t[1]))\n",
     (T779,), 'KILLED'),

    # ---- the default must stay OFF for every other caller ------------------
    ('seed_pos-defaults-ON', 'fc',
     "    def _via_shortfalls(self, ref, cap, x, y, rot, seed_pos=False):\n",
     "    def _via_shortfalls(self, ref, cap, x, y, rot, seed_pos=True):\n",
     (T779, T775, T725), 'KILLED'),

    # ---- the landing, not the seed -----------------------------------------
    ('record-the-LANDING-instead-of-the-seed', 'fc',
     _RECORD,
     "                    self._via_seed_xy[id(moved)] = (\n"
     "                        moved, moved[0], moved[1])\n",
     (T779,), 'KILLED'),

    # ---- INERT PROBES, which must change nothing ---------------------------
    ('a-trailing-comment-names-the-kwarg', 'fc',
     "        at = None\n",
     "        at = None  # seed_pos=True\n",
     (T779,), 'SURVIVED'),

    ('the-resolve-uses-a-generator-not-a-list', 'fc',
     "            at = [_sx[id(t)][1:] if id(t) in _sx else (t[0], t[1])\n"
     "                  for t in vias]\n",
     "            at = list(_sx[id(t)][1:] if id(t) in _sx else (t[0], t[1])\n"
     "                      for t in vias)\n",
     (T779,), 'SURVIVED'),
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
