#!/usr/bin/env python3
"""The #747 mutation battery, shipped so its numbers can be re-derived.

`tests/test_747_fanout_clearance_via_registrar.py` records what each arm kills.
A count is only checkable if the exact source edit is written down -- two
reviewers of the #746 branch reconstructed its rows from their names and both
got the wrong answer, because a plausible-looking reconstruction of one row was
semantically inert. So the edits live here, as data, next to the numbers they
produced.

Every row carries an EXPECTATION. An inert row recorded as an expected survivor
is a finding; an inert row quietly deleted is a hole. A row whose verdict does
not match its expectation is reported as WRONG.

One target file (`py_placer/placement/fanout_clearance.py`), several grading
test files: #747 moved an invariant that `test_725` used to hold, so several
rows are killed there rather than in this issue's own file, and a row is KILLED
if ANY of its named tests exits non-zero. `test_370` and `test_617` are
`run()`-style scripts rather than unittest, so the FAIL/ERROR scraper tolerates
their prose.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. The file refuses to start on a
dirty engine, because restoring would write the COMMITTED text back over
uncommitted work.

    python3 tests/mutate_747.py
    python3 tests/mutate_747.py --row drop-radius-carry-over

A row is KILLED by a FAILURE **or an ERROR**: several of these mutations make an
arm raise rather than fail, and a battery that counted only failures would call
that a survivor.

An anchor that does not match EXACTLY ONCE is reported as BROKEN rather than
skipped -- a battery that silently applies nothing reports every row as a
survivor, which reads as a catastrophic test failure and is really a stale
anchor. `str.replace(old, new, 1)` of an absent needle returns the file
unchanged, which is why the count is checked BEFORE the write.

Python `str.replace`, never `sed`: commit `bb8f4477` records two rows of
`mutate_761` leaving a `SyntaxError` behind because `sed` ate an unescaped
metacharacter, and a battery that cannot start reports nothing at all.
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

T747 = os.path.join(_TESTS, 'test_747_fanout_clearance_via_registrar.py')
T725 = os.path.join(_TESTS, 'test_725_fanout_clearance_pad_floors.py')
T775 = os.path.join(_TESTS,
                    'test_775_fanout_clearance_via_prune_refresh.py')
T732 = os.path.join(_TESTS, 'test_732_fanout_clearance_via_radius.py')
T736 = os.path.join(_TESTS, 'test_736_fanout_clearance_regrade_view.py')
T746 = os.path.join(_TESTS, 'test_746_fanout_clearance_resolved_credit.py')
T370 = os.path.join(_TESTS, 'test_370_tierb_fixes.py')

# The registrar's match, hoisted because it spans two lines and every row that
# touches it needs the whole condition.
_MATCH = """                if (t[2] == net and abs(t[0] - old_x) < 1e-6
                        and abs(t[1] - old_y) < 1e-6):"""

# The registrar's per-move rebind, likewise: the rows below need to distinguish
# WHERE it sits in the loop nest, and one line of it is not enough.
_REBIND = """                    rebuilt.append(t)
            self.vias = rebuilt
        return n"""

# The pre-#747 block, restored VERBATIM beside the report, for the one row
# that no behavioural arm can catch. Kept as a module constant so the row's
# replacement text is readable and so nothing here is assembled from escapes.
_REVIVED_BLOCK = """            st.vias = [((nx, ny, w2[2], w2[3])
                        if (abs(w2[0] - old[0]) < 1e-6
                            and abs(w2[1] - old[1]) < 1e-6) else w2)
                       for w2 in getattr(st, 'vias', [])]
            nm = pcb_data.nets[v.net_id].name if v.net_id in pcb_data.nets else v.net_id"""

ROWS = [
    # ---- the builder: what a via tuple IS ------------------------------
    ('recompute-the-keepout', 'fc',
     """        t = (x, y, net_id,
             (radius + self._item_reach(self._via_floor_for(net_id)))
             if keepout is _DERIVE_KEEPOUT else keepout)""",
     """        t = (x, y, net_id,
             (radius + self._item_reach(self._via_floor_for(net_id)))
             if radius is not None else keepout)""",
     (T747,), 'KILLED'),

    # The sentinel put back to None, which is the spelling an adversarial
    # review measured as broken: a relocated tuple whose element 3 is itself
    # None either RAISES (absent from the radius map) or has its keep-out
    # silently re-derived (present in it). Nothing in the module builds such a
    # tuple, so only the arm written for it can see this.
    ('the-derive-marker-is-None-again', 'fc',
     [("    def _register_via(self, x, y, net_id, radius=None,\n"
       "                      keepout=_DERIVE_KEEPOUT):",
       "    def _register_via(self, x, y, net_id, radius=None,\n"
       "                      keepout=None):"),
      ("        if radius is None and keepout is _DERIVE_KEEPOUT:",
       "        if radius is None and keepout is None:"),
      ("             if keepout is _DERIVE_KEEPOUT else keepout)",
       "             if keepout is None else keepout)")],
     None,
     (T747,), 'KILLED'),

    ('file-a-radius-for-an-unmapped-tuple', 'fc',
     """        if radius is not None:
            self._via_radius_by_id[id(t)] = (t, radius)""",
     """        self._via_radius_by_id[id(t)] = (t, radius)""",
     (T747,), 'KILLED'),

    ('map-value-drops-the-tuple', 'fc',
     '            self._via_radius_by_id[id(t)] = (t, radius)',
     '            self._via_radius_by_id[id(t)] = (None, radius)',
     (T747, T725, T732), 'KILLED'),

    # `_via_effs` reads the radius out of element 1 of the value, so swapping
    # the pair mis-prices every via on every board rather than only a moved one.
    ('map-value-order-swapped', 'fc',
     '            self._via_radius_by_id[id(t)] = (t, radius)',
     '            self._via_radius_by_id[id(t)] = (radius, t)',
     (T747, T725, T732), 'KILLED'),

    ('the-no-information-guard-is-dropped', 'fc',
     """        if radius is None and keepout is _DERIVE_KEEPOUT:
            raise ValueError('a via tuple needs a radius or a keep-out')
""",
     '',
     (T747,), 'KILLED'),

    # ---- the registrar: the carry-over ---------------------------------
    ('drop-radius-carry-over', 'fc',
     '                        radius=None if rec is None else rec[1],',
     '                        radius=None,',
     (T747, T725), 'KILLED'),

    ('drop-keepout-carry-over', 'fc',
     """                        radius=None if rec is None else rec[1],
                        keepout=t[3]))""",
     """                        radius=0.0 if rec is None else rec[1]))""",
     (T747,), 'KILLED'),

    # ---- the registrar: the match --------------------------------------
    ('match-on-coordinates-only', 'fc',
     _MATCH,
     """                if (abs(t[0] - old_x) < 1e-6
                        and abs(t[1] - old_y) < 1e-6):""",
     (T747,), 'KILLED'),

    ('match-on-net-only', 'fc',
     _MATCH,
     '                if t[2] == net:',
     (T747, T725), 'KILLED'),

    # ---- the registrar: ordering and identity --------------------------
    # The rebind is hoisted OUT of the per-move loop, so every move resolves
    # against the pre-nudge list: the second hop of a twice-relocated via finds
    # nothing. `rebuilt` has to be seeded outside too, or the mutant is a
    # NameError rather than the defect being simulated.
    ('single-pass-against-the-original-list', 'fc',
     """        n = 0
        for old_x, old_y, spec in via_moves:
            net, nx, ny = spec['net_id'], spec['x'], spec['y']
            rebuilt = []
            for t in self.vias:""",
     """        n = 0
        _frozen = list(self.vias)
        for old_x, old_y, spec in via_moves:
            net, nx, ny = spec['net_id'], spec['x'], spec['y']
            rebuilt = []
            for t in _frozen:""",
     (T747,), 'KILLED'),

    ('skip-the-rebind-when-nothing-matched', 'fc',
     _REBIND,
     """                    rebuilt.append(t)
            if n:
                self.vias = rebuilt
        return n""",
     (T747,), 'KILLED'),

    ('mutate-the-list-in-place', 'fc',
     _REBIND,
     """                    rebuilt.append(t)
            self.vias[:] = rebuilt
        return n""",
     (T747,), 'KILLED'),

    ('count-moves-not-tuples', 'fc',
     """            self.vias = rebuilt
        return n""",
     """            self.vias = rebuilt
        return len(via_moves)""",
     (T747,), 'KILLED'),

    # ---- the caller ----------------------------------------------------
    ('skip-relocate-in-repair', 'fc',
     '            st.relocate_vias(via_moves)\n',
     '',
     (T747, T725), 'KILLED'),

    # Moving the call inside the nudger is the #736 lesson applied to this
    # channel: the duck-typed stand-ins carry no such method, so the harnesses
    # raise. Both halves are mutated together -- deleting the caller's line
    # alone is a different row (above).
    ('call-relocate-from-the-nudger', 'fc',
     [('            st.relocate_vias(via_moves)\n', ''),
      ('            via_moves.append((old[0], old[1],',
       '            st.relocate_vias([(old[0], old[1], {})])\n'
       '            via_moves.append((old[0], old[1],')],
     None,
     (T747, T370), 'KILLED'),

    # ---- __init__ ------------------------------------------------------
    # The builder stops being the construction site: __init__ inlines the tuple
    # again, which is precisely the shape #747 removed.
    ('reinline-the-tuple-in-init', 'fc',
     """        for v in pcb_data.vias:
            self.vias.append(self._register_via(v.x, v.y, v.net_id,
                                                radius=self.via_radius(v)))""",
     """        for v in pcb_data.vias:
            _r = self.via_radius(v)
            _t = (v.x, v.y, v.net_id,
                  _r + self._item_reach(self._via_floor_for(v.net_id)))
            self.vias.append(_t)
            self._via_radius_by_id[id(_t)] = (_t, _r)""",
     (T747,), 'KILLED'),

    # ---- the survivors -------------------------------------------------
    # Every via that did NOT move is carried by REFERENCE. Rebuilding it makes
    # a new tuple absent from the radius map, so the map empties itself of
    # every unmoved barrel on the first nudge -- numerically invisible until a
    # cap is graded against one of them.
    ('survivors-rebuilt-instead-of-carried', 'fc',
     """                else:
                    rebuilt.append(t)""",
     """                else:
                    rebuilt.append((t[0], t[1], t[2], t[3]))""",
     (T747,), 'KILLED'),

    # ---- the one-writer property itself --------------------------------
    # THE ANNOTATION HERE WAS WRONG AND A FACT-CHECK REFUTED IT BY READING THE
    # RUN. It said "killed by a source guard alone, no behavioural arm can
    # separate them". Measured: 5 killers, of which FOUR are behavioural. Two
    # reasons, and both were checkable before the claim was written:
    #
    #   * test_747's headline arm asserts the nudger leaves the graded list the
    #     SAME OBJECT holding the SAME TUPLES. Any write from inside the nudger
    #     fails that, whatever it writes -- so a behavioural arm for exactly
    #     this mutation already existed.
    #   * the block below is the PRE-6af3495a rebuild, which carries no radius
    #     across. So the end state is NOT identical either: the moved tuple
    #     loses its map entry and the registrar, finding nothing at the old
    #     position, returns 0 where the arms expect 1.
    #
    # Left as measured rather than re-tuned into the claim it was supposed to
    # make: what this row shows is that the one-writer property is defended
    # behaviourally AND structurally, which is a better answer than the one
    # predicted.
    ('nudger-writes-the-graded-view-again', 'fc',
     "            nm = pcb_data.nets[v.net_id].name "
     "if v.net_id in pcb_data.nets else v.net_id",
     _REVIVED_BLOCK,
     (T747,), 'KILLED'),

    # ---- the two INERT probes, recorded rather than omitted -------------
    # The guard must be reading CODE. A KILLED here would mean a trailing
    # comment can invent a construction site, which is the failure #756
    # recorded twice in one change.
    ('a-trailing-comment-names-a-guarded-literal', 'fc',
     '            self.vias = rebuilt',
     '            self.vias = rebuilt  '
     '# self._via_radius_by_id[id(t)] = (t, radius)',
     (T747,), 'SURVIVED'),

    # HONEST SURVIVOR. No fixture in this family sits within 1e-6 of a via
    # position -- the nudger rounds its landings to four places and every rig
    # places vias at three or fewer -- so `<` and `<=` are the same predicate
    # on every input the battery can build. The tolerance is unchanged from
    # the deleted block and is not this fix's subject; recorded so its survival
    # is not read as a coverage hole.
    ('the-match-tolerance-respelled-at-the-boundary', 'fc',
     '                if (t[2] == net and abs(t[0] - old_x) < 1e-6',
     '                if (t[2] == net and abs(t[0] - old_x) <= 1e-6',
     (T747,), 'SURVIVED'),

    # ---- ordering in the caller ----------------------------------------
    # The per-cap refresh reads the list this repairs, so relocating AFTER it
    # leaves every cap holding the pre-move tuples.
    #
    # RECORDED EXPECTATION CORRECTED BY THE FIRST RUN. This row was written as
    # an expected SURVIVOR on the reasoning that nothing on the tracked corpus
    # relocates a via, so no fixture could reach it. That reasoning was wrong
    # twice over: this file's own real-board arm drives orangecrab at a boxed
    # configuration that relocates nine, and #736's and #746's rigs relocate
    # one each on synthetic boards. Measured: 8 arms across three files, and
    # the note is left here rather than quietly flipped, because a battery
    # whose predictions are edited to match its results measures nothing.
    #
    # RE-ANCHORED BY #775, which replaced the inline per-cap comprehension
    # this row used to move around with a method on _Repair. It is the SAME
    # mutation -- relocate AFTER the refresh -- and it keeps the note above
    # rather than being rewritten as though it had always looked like this.
    # T775 joins the graders: TestTheRefreshSeesTheMOVEDBarrel asserts
    # directly that no cap holds a pre-move tuple, which is exactly what
    # this ordering breaks, and #746's ordering arm now checks it
    # structurally as well as behaviourally.
    ('relocate-after-the-per-cap-refresh', 'fc',
     [('            st.relocate_vias(via_moves)\n', ''),
      ('            st.refresh_cap_vias()\n',
       '            st.refresh_cap_vias()\n'
       '            st.relocate_vias(via_moves)\n')],
     None,
     (T747, T736, T746, T775), 'KILLED'),
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
            print('  ran %-42s %s' % (name, results[-1][1]))
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
            print('%-42s %s' % (r[0], r[5]))
        return 0
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
