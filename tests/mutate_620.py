#!/usr/bin/env python3
"""The #620/#618 mutation battery, shipped so its numbers can be re-derived.

`tests/test_620_pending_via_pairs.py` and `tests/test_618_coupled_via_site_gate.py`
record what each arm kills. A count is only checkable if the exact source edit
is written down -- two reviewers of an earlier branch reconstructed rows from
their names and both got the wrong answer, because a plausible-looking
reconstruction of one row was semantically inert. So the edits live here, as
data, next to the numbers they produced.

Every row carries an EXPECTATION. An inert row recorded as an expected survivor
is a finding; an inert row quietly deleted is a hole.

THREE targets: the pair test and ladder in `bga_fanout/geometry.py`, their
wiring in `bga_fanout/__init__.py`, and #618's coupled-escape gate in
`bga_fanout/underpad.py`.

THE ROWS TO LOOK AT FIRST if this file ever goes red are the four broad-phase
ones. An adversarial review found that the differential test named
`window = max(d + self._h2h,` in its own docstring and could not kill it: the
parameter grid never made the drill term binding, so half the window could be
corrupted freely. `broad-phase-window-drops-the-halving` is that exact
mutation, and it is why the grid now sweeps floors of zero and drills that
exceed every ring.

THIS RUNNER IS A COPY of `mutate_756.py`'s (itself the fourth). Deliberately
not refactored into a shared one: that would rewrite shipped batteries whose
recorded counts are the evidence for merged reviews, which is a change to make
on its own and not inside a fix.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. It refuses to start on a dirty
engine, because restoring would write the COMMITTED text back over uncommitted
work.

    python3 tests/mutate_620.py
    python3 tests/mutate_620.py --row twin-branch-refuses-instead

A row is KILLED by a FAILURE **or an ERROR**: dropping a term makes some arms
raise rather than fail, and a battery counting only failures would call that a
survivor.

An anchor that does not match EXACTLY ONCE is reported as BROKEN rather than
skipped -- a battery that silently applies nothing reports every row as a
survivor, which reads as a catastrophic test failure and is really a stale
anchor.

ANCHORS ARE WRITTEN WITH LF AND TRANSLATED TO THE TARGET'S OWN ENDING. All
three targets are LF in git (`.gitattributes` has `*.py text eol=lf`), so from
a clean checkout the translation is a no-op. It is here because a WORKING TREE
can still be CRLF -- any edit written through Python's text mode on Windows
converts the whole file, which happened twice while building this branch.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

GEO = os.path.join(_ROOT, 'py_router', 'bga_fanout', 'geometry.py')
BGA = os.path.join(_ROOT, 'py_router', 'bga_fanout', '__init__.py')
UP = os.path.join(_ROOT, 'py_router', 'bga_fanout', 'underpad.py')
TARGETS = {'geo': GEO, 'bga': BGA, 'up': UP}

T620 = os.path.join(_TESTS, 'test_620_pending_via_pairs.py')
T618 = os.path.join(_TESTS, 'test_618_coupled_via_site_gate.py')
T370 = os.path.join(_TESTS, 'test_370_tierb_fixes.py')
T756 = os.path.join(_TESTS, 'test_756_fanout_clearance_drill_floors.py')

_WINDOW = ("        window = max(d / 2.0 + self._max_drill / 2.0 + self._h2h,\n"
           "                     s / 2.0 + self._max_size / 2.0 + "
           "self._clearance,\n"
           "                     self._max_size / 2.0 + tw / 2.0)")

# (name, target, old, new, tests, expect)
ROWS = [
    # --- the defect itself: the pair test ----------------------------------
    ('pair-test-never-consulted', 'bga',
     '                _v, _detail = _pending.verdict(pad_x, pad_y, v_size, '
     'v_drill,\n'
     '                                               route.net_id, _bulges,\n'
     '                                               track_width=track_width)',
     "                _v, _detail = 'clear', None",
     (T620,), 'KILLED'),

    ('verdict-always-clear', 'geo',
     '        d = drill or 0.0\n        s = size or 0.0\n'
     '        tw = track_width or 0.0',
     "        return 'clear', None\n"
     '        d = drill or 0.0\n        s = size or 0.0\n'
     '        tw = track_width or 0.0',
     (T620,), 'KILLED'),

    ('pending-never-records-the-committed-via', 'bga',
     '                _pending.add(pad_x, pad_y, v_size, v_drill, '
     'route.net_id,\n                             _bulges)',
     '                pass',
     (T620,), 'KILLED'),

    # --- the DRILL arm ------------------------------------------------------
    ('drill-arm-deleted', 'geo',
     '            need = d / 2.0 + od / 2.0 + self._h2h',
     '            need = 0.0',
     (T620,), 'KILLED'),

    ('drill-arm-drops-the-floor', 'geo',
     '            need = d / 2.0 + od / 2.0 + self._h2h',
     '            need = d / 2.0 + od / 2.0',
     (T620,), 'KILLED'),

    ('drill-arm-reads-the-FAB-floor-not-the-boards', 'bga',
     '    _pending = PendingVias(_h2h, clearance)',
     '    _pending = PendingVias(_h2h_fab, clearance)',
     (T620,), 'KILLED'),

    # --- the RING arm and its bulge condition -------------------------------
    ('ring-arm-deleted', 'geo',
     '            if (bulges or obulges) and onet != net_id:',
     '            if False:',
     (T620,), 'KILLED'),

    ('ring-arm-ignores-the-bulge', 'geo',
     '            if (bulges or obulges) and onet != net_id:',
     '            if onet != net_id:',
     (T620,), 'KILLED'),

    ('ring-arm-ignores-the-net', 'geo',
     '            if (bulges or obulges) and onet != net_id:',
     '            if (bulges or obulges):',
     (T620,), 'KILLED'),

    ('bulge-flag-hardwired-false', 'bga',
     "                _bulges = status == 'floor'",
     '                _bulges = False',
     (T620,), 'KILLED'),

    ('bulge-flag-hardwired-true', 'bga',
     "                _bulges = status == 'floor'",
     '                _bulges = True',
     (T620,), 'KILLED'),

    # --- THE BROAD PHASE. The rows an earlier test could not kill. ----------
    # #854 note: these three replacements used to end in `ahx`, the name the
    # anchor-box rule bound. This PR deleted it, which silently turned all
    # three into CRASH mutants -- `NameError: ahx` -- and the runner counts an
    # ERROR as a kill, so they would have reported KILLED forever while
    # measuring nothing. The BROKEN-anchor guard cannot catch this: it
    # validates the string being REPLACED, not the replacement. Found by a
    # pre-push review, and the reason `_verify_mutants_are_not_crashes` below
    # now exists.
    ('broad-phase-window-drops-the-halving', 'geo',
     _WINDOW,
     '        window = max(d + self._h2h,\n'
     '                     s / 2.0 + self._max_size / 2.0 + self._clearance,\n'
     '                     self._max_size / 2.0 + tw / 2.0)',
     (T620,), 'KILLED'),

    ('broad-phase-window-drops-the-ring-term', 'geo',
     _WINDOW,
     '        window = max(d / 2.0 + self._max_drill / 2.0 + self._h2h,\n'
     '                     self._max_size / 2.0 + tw / 2.0)',
     (T620,), 'KILLED'),

    ('broad-phase-window-drops-the-drill-term', 'geo',
     _WINDOW,
     '        window = max(s / 2.0 + self._max_size / 2.0 + self._clearance,\n'
     '                     self._max_size / 2.0 + tw / 2.0)',
     (T620,), 'KILLED'),

    # #854 RE-POINTED this row rather than retiring it: the window's third
    # term is no longer the candidate pad's half-width but the ANCHOR REACH
    # (widest committed via + half a track), and it is still the term nothing
    # else makes binding -- see test_the_REACH_term_in_the_broad_phase_window.
    ('broad-phase-window-drops-the-reach-term', 'geo',
     _WINDOW,
     '        window = max(d / 2.0 + self._max_drill / 2.0 + self._h2h,\n'
     '                     s / 2.0 + self._max_size / 2.0 + self._clearance)',
     (T620,), 'KILLED'),

    ('broad-phase-max-drill-never-updated', 'geo',
     '        if d > self._max_drill:\n            self._max_drill = d',
     '        pass',
     (T620,), 'KILLED'),

    # --- TWINS --------------------------------------------------------------
    ('twin-branch-refuses-instead', 'geo',
     '            if onet == net_id and via_anchors_route(ox, oy, os_, '
     '(x, y), tw):\n'
     "                return 'twin', (ox, oy, os_, od)",
     '            if onet == net_id and via_anchors_route(ox, oy, os_, '
     '(x, y), tw):\n'
     "                return 'conflict', ('same site', ox, oy)",
     (T620,), 'KILLED'),

    ('twin-branch-ignores-the-net', 'geo',
     '            if onet == net_id and via_anchors_route(ox, oy, os_, '
     '(x, y), tw):',
     '            if True and via_anchors_route(ox, oy, os_, (x, y), tw):',
     (T620,), 'KILLED'),

    ('twin-keyed-on-the-exact-site-again', 'geo',
     '            if onet == net_id and via_anchors_route(ox, oy, os_, '
     '(x, y), tw):',
     '            if onet == net_id and dist <= self._tol:',
     (T620,), 'KILLED'),

    # --- #854: a reach test is not a containment test -----------------------
    # The row the issue asks for by name ("mutate_620.py should get a row that
    # dies when the anchor test is widened back"). 0.5 is the BIG pad's own
    # half-extent in the dissimilar-size arm, so this row IS that arm's defect.
    ('twin-keyed-on-the-candidate-PAD-BOX-again', 'geo',
     '            if onet == net_id and via_anchors_route(ox, oy, os_, '
     '(x, y), tw):',
     '            if (onet == net_id\n'
     '                    and abs(ox - x) <= 0.5 and abs(oy - y) <= 0.5):',
     (T620,), 'KILLED'),

    # The SECOND copy of the same rule, which is what made the stranding
    # invisible downstream: fixing `verdict` alone leaves `_has_copper`
    # agreeing with the old answer, so nothing straps the swallowed ball.
    ('ball-anchor-test-reverted-to-the-pad-box', 'bga',
     "    if any(v['net_id'] == pad.net_id\n"
     "           and via_anchors_route(v['x'], v['y'], v.get('size') or 0.0,\n"
     "                                 (pad.global_x, pad.global_y), "
     "track_width)\n"
     '           for v in vias):',
     "    tol = max(pad.size_x, pad.size_y) / 2 + 0.01\n"
     "    if any(v['net_id'] == pad.net_id\n"
     "           and abs(v['x'] - pad.global_x) < tol\n"
     "           and abs(v['y'] - pad.global_y) < tol\n"
     '           for v in vias):',
     (T620,), 'KILLED'),

    # #854: `tighten` shrinks the survivor, and a smaller barrel reaches less
    # far, so the merge can be justified by a via that no longer reaches once
    # tightened. Impossible under the old pad-BOX rule (a box does not depend
    # on via size), so this row exists because the rule changed.
    ('post-tighten-reach-recheck-deleted', 'bga',
     '                    if not via_anchors_route(_tx, _ty, min(_ts, v_size),\n'
     "                                             (pad_x, pad_y), track_width):\n"
     "                        _v = 'clear'\n"
     '                    else:',
     '                    if False:\n'
     "                        _v = 'clear'\n"
     '                    else:',
     (T620,), 'KILLED'),

    ('twin-appends-a-second-via-anyway', 'bga',
     "                    _twin_shared += 1\n                    continue",
     '                    _twin_shared += 1',
     (T620,), 'KILLED'),

    ('twin-keeps-the-FIRST-via-not-the-tighter', 'bga',
     '                    if _pending.tighten(_tx, _ty, v_size, v_drill):',
     '                    if False:',
     (T620,), 'KILLED'),

    # --- THE LADDER ---------------------------------------------------------
    # NAMED FOR WHAT IT IS, after an audit pointed out the old name asserted a
    # behaviour the edit does not produce: `None or X` is exactly `X`, so this
    # never stops the ladder being tried. It is the battery's OWN control --
    # the row that proves a SURVIVED verdict is reachable and that the runner
    # is not silently killing everything. `ladder-refuses-immediately` is the
    # row that actually removes the ladder.
    ('control-semantically-inert-edit', 'bga',
     '                    _thin = thin_drill_to_clear(',
     '                    _thin = None or thin_drill_to_clear(',
     (T620,), 'SURVIVED'),

    ('ladder-refuses-immediately', 'bga',
     '                    if _thin is None:',
     '                    if True:',
     (T620,), 'KILLED'),

    ('ladder-returns-the-thinnest-rung', 'geo',
     "    cands = sorted({f['via_drill'] for f in floors[rung:]\n"
     "                    if f['via_drill'] < drill - 1e-9}, reverse=True)",
     "    cands = sorted({f['via_drill'] for f in floors[rung:]\n"
     "                    if f['via_drill'] < drill - 1e-9})",
     (T620,), 'KILLED'),

    ('ladder-returns-a-rung-that-does-not-clear', 'geo',
     '    for cand in cands:\n        if clears(cand):\n            return cand',
     '    for cand in cands:\n        return cand',
     (T620,), 'KILLED'),

    # EXPECTED SURVIVOR, with the rationale CORRECTED by an audit. It is not
    # "rung 0 is the top" -- the real reason is the `via_drill < drill - 1e-9`
    # filter: the standard ladder's drills are [0.20, 0.15, 0.15], so widening
    # the slice can only re-admit drills the filter then throws away. The row
    # stays because the slice IS load-bearing for a ladder whose shallow rungs
    # are thinner than a deep one, which no packaged tier is today.
    ('ladder-climbs-above-its-own-rung', 'geo',
     "    cands = sorted({f['via_drill'] for f in floors[rung:]",
     "    cands = sorted({f['via_drill'] for f in floors[0:]",
     (T620,), 'SURVIVED'),

    ('thinned-drill-not-applied', 'bga',
     '                    _thinned.append((v_drill, _thin))\n'
     '                    v_drill = _thin',
     '                    _thinned.append((v_drill, _thin))',
     (T620,), 'KILLED'),

    # --- the second guard's bookkeeping (#620's other half) -----------------
    # RENAMED after an audit: `if False and ...` DELETES the refusal, it does
    # not restore the old drop-via/keep-route shape. Both are live mutations
    # worth having, so the second one is now its own row below.
    ('ring-guard-refusal-deleted', 'bga',
     '                if would_overlap_existing_via(pad_x, pad_y, v_size):',
     '                if False and would_overlap_existing_via(pad_x, pad_y, '
     'v_size):',
     (T620,), 'KILLED'),

    # The actual pre-#620 shape: the via is dropped and the ROUTE is kept, so
    # an inner-layer track ships with nothing connecting it to the ball.
    ('silent-skip-restored', 'bga',
     '                if would_overlap_existing_via(pad_x, pad_y, v_size):\n',
     '                if would_overlap_existing_via(pad_x, pad_y, v_size):\n'
     '                    continue\n',
     (T620,), 'KILLED'),

    # --- the disclosures ----------------------------------------------------
    ('refusal-disclosure-deleted', 'bga',
     '        print(f"  WARNING: {len(_pending_refused)} escape(s) dropped: '
     'this "',
     '        _unused = (f"  WARNING: {len(_pending_refused)} escape(s) '
     'dropped: this "',
     (T620,), 'KILLED'),

    ('thinning-escalation-not-disclosed', 'bga',
     '        warn_fab_escalation(\n'
     '            f"{len(_thinned)} via-in-pad drill(s) thinned to {_to}mm to '
     'hold "',
     '        _unused = (\n'
     '            f"{len(_thinned)} via-in-pad drill(s) thinned to {_to}mm to '
     'hold "',
     (T620,), 'KILLED'),

    # --- #618: the coupled escape's via sites -------------------------------
    # Anchored INSIDE the gate, not at a call site: the call site text is
    # identical at both of them, and a two-match anchor is reported BROKEN
    # (which is right -- a battery that silently applies one of two edits
    # measures half a mutation). Returning True after the locked-SMD check
    # reproduces the pre-#618 behaviour exactly, at both callers at once.
    ('coupled-gate-reverted-to-locked-smd-only', 'up',
     '        sites = [_via_site_geom(q) for q in (pp, nn)]',
     '        return True\n'
     '        sites = [_via_site_geom(q) for q in (pp, nn)]',
     (T618,), 'KILLED'),

    # EXPECTED SURVIVOR, and the reason is worth more than the row.
    # `_via_site_conflict` has two domains: the board's own copper, and the
    # vias THIS RUN has already committed (`_via_ctx` folds `vias_to_add` in).
    # The first is already covered by the occupancy grid, which is built from
    # the board before any escape runs -- measured: planting a foreign via on
    # the exact coordinate a coupled escape chooses still keeps every emitted
    # via clear of it with this check disabled, so the mutation changes
    # nothing observable. The second domain is the one only this check covers,
    # and no in-repo board reaches it: the only rigs that couple are 0.8mm
    # pitch, where a committed via is 0.4mm clear of the floor. So the row is
    # a measured no-op HERE and load-bearing in general. Recorded, not deleted.
    ('coupled-gate-skips-the-site-conflict', 'up',
     '            if why is not None:\n'
     "                if why.startswith('drill hole'):\n"
     "                    h2h_stats['coupled_pairs'].add(_pair_key)\n"
     '                return False',
     '            if False:\n                return False',
     (T618,), 'SURVIVED'),

    # I recorded this as an expected SURVIVOR, reasoning that the only in-repo
    # rigs are 0.8mm-pitch parts whose pair clears by 0.4mm. The battery
    # refuted it: the declared-0.7 arm makes the pair need 0.9mm at 0.8mm
    # pitch, so the pair-vs-itself test is what declines it and the row is
    # KILLED. Kept as the record of a wrong expectation corrected by measuring.
    ('coupled-pair-not-tested-against-itself', 'up',
     '        if math.hypot(ax - bx, ay - by) < ((ad or 0.0) / 2.0\n'
     '                                           + (bd or 0.0) / 2.0\n'
     '                                           + _h2h - 1e-6):',
     '        if False:',
     (T618,), 'KILLED'),

    ('h2h-decline-not-disclosed', 'up',
     '            print(f"  Under-pad: {h2h_stats[\'sites\']} via-in-pad centre '
     'site(s)"',
     '            _unused = (f"  Under-pad: {h2h_stats[\'sites\']} via-in-pad '
     'centre site(s)"',
     (T618,), 'KILLED'),

    # --- #756's arms must still hold (this branch touches the same file) ----
    ('bga-via-arm-reverted-to-the-flat-constant', 'bga',
     '                    + _h2h - 1e-6:',
     '                    + HOLE_TO_HOLE_CLEARANCE - 1e-6:',
     (T756, T620), 'KILLED'),

    ('bga-drops-the-fab-wrap', 'bga',
     '    _h2h = max(_h2h_decl, _h2h_fab)',
     '    _h2h = _h2h_decl',
     (T756, T370), 'KILLED'),
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
            path = TARGETS[tgt]
            base = orig[tgt]
            edits = old if isinstance(old, list) else [(old, new)]
            if '\r\n' in base:
                edits = [(o.replace('\n', '\r\n'), n.replace('\n', '\r\n'))
                         for o, n in edits]
            counts = [base.count(o) for o, _n in edits]
            if counts != [1] * len(edits):
                results.append((name, 'BROKEN', expect,
                                'anchors matched %s times' % counts, []))
                continue
            mutated = base
            for o, nw in edits:
                mutated = mutated.replace(o, nw, 1)
            io.open(path, 'w', encoding='utf-8', newline='').write(mutated)
            killed, failed, crashed = False, [], False
            for t in tests:
                p = subprocess.run([sys.executable, '-X', 'utf8', t],
                                   capture_output=True, text=True,
                                   encoding='utf-8', errors='replace',
                                   timeout=1800, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                # A mutant that makes the ENGINE crash proves nothing about
                # the test's discrimination -- the runner would score the
                # non-zero exit as a kill either way. Three rows here spent a
                # commit in exactly that state (`NameError: ahx`), reporting
                # KILLED while measuring nothing, so it is now its own verdict.
                if 'NameError' in out or 'AttributeError' in out:
                    crashed = True
                if p.returncode:
                    killed = True
                failed += ['%s::%s' % (os.path.basename(t)[5:8],
                                       l.split('(')[0].replace('FAIL: ', '')
                                       .replace('ERROR: ', '').strip())
                           for l in out.splitlines()
                           if l.startswith(('FAIL:', 'ERROR:'))]
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name,
                            'CRASH' if crashed else
                            ('KILLED' if killed else 'SURVIVED'),
                            expect, '%d' % len(failed), failed))
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])

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
    broken = sum(1 for r in results if r[1] in ('BROKEN', 'CRASH'))
    print('\n%d rows: %d killed, %d survived (%d of them expected), %d broken'
          % (len(results), killed, survived,
             sum(1 for r in results if r[1] == r[2] == 'SURVIVED'), broken))
    if wrong or broken:
        print('%d row(s) did not match their expectation' % (wrong + broken))
    return 1 if (wrong or broken) else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row', help='run a single row by name')
    a = ap.parse_args()
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
