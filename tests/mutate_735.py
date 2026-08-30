#!/usr/bin/env python3
"""Mutation battery for #735 -- does `tests/test_735_*` actually catch a
reverted or mis-scoped track-rule channel?

A test file that passes proves nothing on its own. This rewrites the ENGINE one
row at a time, re-runs the graded tests, and reports which rows survive. A
surviving row is a claim the tests do not really make.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run a suite in the same
working tree while this is running, and do not run two batteries at once. The
file refuses to start on a dirty engine, because restoring would write the
COMMITTED text back over uncommitted work.

    python3 tests/mutate_735.py
    python3 tests/mutate_735.py --row revert-the-s2-arm
    python3 tests/mutate_735.py --list

A row is KILLED by a FAILURE **or an ERROR** -- a mutation that makes a test
crash is caught, not skipped. An anchor that does not match EXACTLY ONCE is
reported as BROKEN rather than skipped, because a battery that silently applies
nothing reports every row as a survivor, which reads as a catastrophic test
failure and is really a stale anchor.

Python `str.replace`, never `sed`: commit bb8f4477 records two rows of
`mutate_761` leaving a `SyntaxError` behind because `sed` ate an unescaped
metacharacter. Newlines are read and written with `newline=''` so a CRLF
checkout round-trips byte-for-byte.

FOUR TARGETS, because the fix spans four files and a row that can only reach
one of them cannot test the other three.

WHAT THIS MEASURES -- 23 rows. Counts are recorded FROM THE RUN, in the header
of `tests/test_735_fanout_clearance_track_rules.py`, not predicted here.

(Both numbers above were stale in the first draft -- "three targets", "16
rows" -- in a file whose own rule is that counts come from the run. A
fact-check caught them.)
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
LEG = os.path.join(_ROOT, 'py_placer', 'placement', 'legality.py')
DRU = os.path.join(_ROOT, 'py_router', 'kicad_dru.py')
DRC = os.path.join(_ROOT, 'py_router', 'check_drc.py')
TARGETS = {'fc': FC, 'leg': LEG, 'dru': DRU, 'drc': DRC}

T735 = os.path.join(_TESTS, 'test_735_fanout_clearance_track_rules.py')
T735E = os.path.join(_TESTS, 'test_735_fanout_clearance_track_rules_e2e.py')
TDRU = os.path.join(_TESTS, 'test_735_dru_track_clearance.py')
TDRUC = os.path.join(_TESTS, 'test_735_dru_rule_pair_classification.py')
T747 = os.path.join(_TESTS, 'test_747_fanout_clearance_via_registrar.py')
T725 = os.path.join(_TESTS, 'test_725_fanout_clearance_pad_floors.py')

# The gate this whole issue is about, hoisted because it spans two lines.
_S2 = """            pair_req = (track_req(cfl, sfl, net_id, s2.net_id)
                        if honour_track_rules else req(cfl, sfl))"""
_S2_OLD = '            pair_req = req(cfl, sfl)'
_S2_SWAP = """            pair_req = (track_req(cfl, sfl, s2.net_id, net_id)
                        if honour_track_rules else req(cfl, sfl))"""
_LADDER = '            trk_ladder = [True, False] if _trk_on else [True]'

ROWS = [
    # --- the defect itself -------------------------------------------------
    ('revert-the-s2-arm', 'fc', _S2, _S2_OLD, (T735, T735E), 'KILLED'),

    # the plausible-looking "tidy" that swaps the pair's two nets. Symmetric
    # today, so this SHOULD survive -- and a row that survives for a reason
    # the code guarantees is worth keeping as a change detector for the day
    # the predicate stops being symmetric.
    ('swap-the-two-nets-at-the-s2-arm', 'fc', _S2, _S2_SWAP,
     (T735, T735E), 'SURVIVED'),

    # --- the LADDER: the rule is a preference, and losing that is the
    # regression this change was rewritten to avoid ------------------------
    ('the-ladder-becomes-an-all-or-nothing-gate', 'fc', _LADDER,
     '            trk_ladder = [True]',
     (T735, T735E), 'KILLED'),

    ('the-relaxed-rung-runs-FIRST', 'fc', _LADDER,
     '            trk_ladder = [False, True] if _trk_on else [True]',
     (T735, T735E), 'KILLED'),

    ('the-fallback-is-taken-SILENTLY', 'fc',
     '            if trk_relaxed:',
     '            if False:',
     (T735, T735E), 'KILLED'),

    ('the-second-rung-runs-on-every-board', 'fc', _LADDER,
     '            trk_ladder = [True, False]',
     (T735,), 'SURVIVED'),

    # --- the scope: the rule must NOT reach the other arms -----------------
    ('via-vs-track-arm-acquires-the-rule', 'fc',
     """            if _point_to_seg_dist(nx, ny, s.start_x, s.start_y, s.end_x, s.end_y) \\
                    < vr + s.width / 2.0 + req(vfl, seg_fl(s.net_id, s.layer)):""",
     """            if _point_to_seg_dist(nx, ny, s.start_x, s.start_y, s.end_x, s.end_y) \\
                    < vr + s.width / 2.0 + track_req(vfl, seg_fl(s.net_id, s.layer), v.net_id, s.net_id):""",
     (T735,), 'KILLED'),

    # --- the resolver ------------------------------------------------------
    ('track_required-ignores-the-channel', 'fc',
     '        if self._track is None:\n            return base',
     '        if True:\n            return base',
     (T735, T735E), 'KILLED'),

    ('track_required-drops-the-flat-fallback', 'fc',
     '        base = self._pair_or_flat(fa, fb)',
     '        base = self.clearance if self._floors is None else 0.0',
     (T735,), 'KILLED'),

    ('the-two-handles-are-merged', 'fc',
     '        self._track = _model if _model.track_rules else None',
     '        self._track = self._floors',
     (T735,), 'KILLED'),

    ('floors-widened-to-admit-a-track-only-board', 'fc',
     '        self._floors = _model if _model.active else None',
     '        self._floors = (_model if (_model.active or _model.track_rules)\n'
     '                        else None)',
     (T735,), 'KILLED'),

    # --- the duck-typed read ----------------------------------------------
    ('the-resolver-is-read-without-getattr', 'fc',
     "    _trk_req = getattr(st, 'track_required', None)",
     '    _trk_req = st.track_required',
     (T735,), 'KILLED'),

    ('the-shim-falls-back-to-the-flat-scalar', 'fc',
     '        if _trk_req is None:\n            return req(fa, fb)',
     '        if _trk_req is None:\n            return clearance',
     (T735,), 'KILLED'),

    # --- the model ---------------------------------------------------------
    ('for_board-never-reads-the-track-rules', 'leg',
     '                track_rules, net_classes = board_track_rules(pcb_data, path)',
     '                track_rules, net_classes = [], {}',
     (T735, T735E), 'KILLED'),

    ('active-absorbs-the-track-channel', 'leg',
     '        self.active = bool(self.net_floor or self.layer_rules or has_overrides)',
     '        self.active = bool(self.net_floor or self.layer_rules\n'
     '                           or has_overrides or self.track_rules)',
     (T735,), 'KILLED'),

    ('the-model-forgets-the-memberships', 'leg',
     '        self.net_classes = dict(net_classes or {})',
     '        self.net_classes = {}',
     (T735, T735E), 'KILLED'),

    # Renamed from `track_pair-becomes-a-tier-of-pair`, which is what it was
    # called and NOT what it did: it prepended an early return, i.e. it
    # DISABLED the resolver -- a duplicate of `track_required-ignores-the-
    # channel` one layer down -- and left the "did the rule leak into the pad
    # resolver" control, the single most load-bearing claim in legality.py,
    # with no row exercising it. A review caught that. The row below is the
    # leak; this one keeps the disable, honestly named.
    ('track_pair-is-disabled', 'leg',
     '        if not self.track_rules:\n            return resolved, None',
     '        return resolved, None\n        if not self.track_rules:\n'
     '            return resolved, None',
     (T735, T735E), 'KILLED'),

    ('the-rule-LEAKS-into-the-pad-pair-resolver', 'leg',
     '        """The required clearance between two pads (mm)."""\n'
     '        return self.pair_with_source(fa, fb)[0]',
     '        """The required clearance between two pads (mm)."""\n'
     '        return max(self.pair_with_source(fa, fb)[0],\n'
     '                   max((r.clearance_mm for r in self.track_rules),\n'
     '                       default=0.0))',
     (T735,), 'KILLED'),

    # --- the shared predicate ---------------------------------------------
    ('the-predicate-stops-being-raise-only', 'dru',
     '        if binds and r.clearance_mm > eff:',
     '        if binds:',
     (T735, TDRU), 'KILLED'),

    ('other_only-stops-exempting-siblings', 'dru',
     '        binds = ((a_in != b_in) or (a_in and b_in and not r.other_only))',
     '        binds = ((a_in != b_in) or (a_in and b_in))',
     (T735,), 'KILLED'),

    ('the-rule-identity-is-not-reported', 'dru',
     '            eff = r.clearance_mm\n            rule = r',
     '            eff = r.clearance_mm',
     (T735, TDRUC), 'KILLED'),

    # The quiet reader hands back the memberships that are the rules' binding
    # KEY. Dropping them leaves a rule list nothing can ever match, which is
    # the failure mode that looks most like "the feature is just inert".
    #
    # The row this replaced tried to anchor on the except-branch that drops the
    # rules when memberships cannot be read at all. It is NOT reachable from
    # any fixture here -- `net_class_memberships` does not raise on the inputs
    # this file can stage -- so it would have been a permanent survivor
    # claiming coverage it does not have. Said rather than left in.
    ('board_track_rules-hands-back-no-memberships', 'dru',
     '    return rules, {nid: frozenset(cls) for nid, cls in raw.items()}',
     '    return rules, {}',
     (T735, T735E), 'KILLED'),

    # --- the grader's delegation ------------------------------------------
    ('check_drc-stops-delegating', 'drc',
     '        return track_pair_clearance(_track_rules, _cls_of.get(net_a, ()),\n'
     '                                    _cls_of.get(net_b, ()), eff)',
     '        return eff, None',
     (T735, TDRUC), 'KILLED'),
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
                print('  ran %-46s BROKEN' % name)
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
                failed += ['%s::%s' % (os.path.basename(t)[5:8], l.strip()[:70])
                           for l in out.splitlines() if l.startswith('  FAIL')]
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed))
            print('  ran %-46s %s' % (name, results[-1][1]))
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
            print('%-46s %s' % (r[0], r[5]))
        return 0
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
