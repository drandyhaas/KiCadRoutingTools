"""The #1120 / #1121 / #1122 mutation battery: a placement path turns a part
only into the rotation its intent declares.

One row per load-bearing line, each reverting it; every row names the test
case that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore the defect each issue measured, or the reason it went unseen:

  * `poses-ignores-the-declaration` / `generator-reads-no-intent` (#1121) --
    the portfolio's `poses` strategy turned splitflap's U1 270 -> 90 with U1
    declared 270, because nothing handed it the claims the quench is gated
    with;
  * `gate-blind-to-other-callees` -- test_893's standing gate read only
    `_try_place` calls, which is why `perturb_poses` was never asked for its
    declaration;
  * `stage1-set-not-applied` / `stage1-unfit-set-measured-at-the-input`
    (#1120) -- stage 1's edge seat applied a single declared rotation and
    no set, and no later stage re-seats what it seats, so splitflap's J5
    declared `[0, 90]` was written at its input 180, graded clean;
  * `ungated-arm-loses-the-claims` (#1122) -- the cap pass's comparison
    run drops `intent`, and on the U30 crop it is the run KEPT: a hold read
    from `intent` would not be in the board that ships.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1120_1121_1122.py
    python3 tests/mutate_1120_1121_1122.py --row poses-sorts-a-set

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending. A witness is `(test file, case-name substring...)`: the
test files run only the cases whose names contain one of the substrings.

Not covered by a row, and why:
  * `generate()`'s `poses: N free part(s) declare a rotation` line -- a
    disclosure; `place_portfolio_does_not_turn` asserts it, and its mutant
    is that assertion failing;
  * dropping a row from test_893's `_DECLARATION_CALLS` -- that deletes the
    guard rather than weakening it, and no assertion can see its own table
    shrink without restating the table;
  * the via-clear fallback passing `None` for the claim -- no fixture sends
    a DECLARED cap into the fallback with a clear pose only at another
    angle (#1122's verifier measured that mutant SURVIVE); the AST arm
    `rotations_confined` pins that the fallback asks `_cap_rotations` at all;
  * (the escalation guard has a row: `escalation-arms-a-turn-it-cannot-make`.
    It is NOT "when, not whether" -- #1122's verifier measured it flip
    which cap the U30 crop leaves grazing, both ways);
  * `declared_cap_rotations`' `intent is None` return -- its mutant raises
    in every witness that runs the pass without an intent.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
_PL = os.path.join(_ROOT, 'py_placer', 'placement')

TARGETS = {
    'pf': os.path.join(_PL, 'portfolio.py'),
    't893': os.path.join(_TESTS, 'test_893_declared_rotation.py'),
    'sd': os.path.join(_PL, 'seeder.py'),
    'fc': os.path.join(_PL, 'fanout_clearance.py'),
    'pfc': os.path.join(_ROOT, 'py_placer', 'place_fanout_clearance.py'),
    'gui': os.path.join(_ROOT, 'kicad_routing_plugin', 'fanout_gui.py'),
}


def _t(name, *cases):
    return (os.path.join(_TESTS, name),) + cases


T1121 = 'test_1121_portfolio_declared_rotation.py'
DECLARED = _t(T1121, 'declared_angle_is_not')
SLOT = _t(T1121, 'declared_slot_goes')
OFF_ANGLE = _t(T1121, 'off_its_declared_angle')
CAND_SET = _t(T1121, 'candidate_set_bounds')
AUTHOR = _t(T1121, 'author_order')
OFF_LATTICE = _t(T1121, 'off_lattice_member')
E2E = _t(T1121, 'place_portfolio_does_not')
GATE = _t('test_893_declared_rotation.py', 'declaration_taking_call')
GATE_CTL = _t('test_893_declared_rotation.py', 'ladder_gate')
# test_983 is unittest: `-k` selects, and a pattern that matches nothing
# exits 5 (Python 3.12+), so a misspelt witness refuses at the baseline.
T983 = 'test_983_seat_grade_bounds.py'
C5 = _t(T983, '-k', 'test_c5_')
C7 = _t(T983, '-k', 'test_c7_')
C8 = _t(T983, '-k', 'test_c8_')
C9 = _t(T983, '-k', 'test_c9_')
C10 = _t(T983, '-k', 'test_c10_')
C13 = _t(T983, '-k', 'test_c13_')
T1122 = 'test_1122_fanout_declared_rotation.py'
ROT_LIST = _t(T1122, 'rotation_list')
CONFINED = _t(T1122, 'confined_to_the_helper')
HELD = _t(T1122, 'not_turned_in_the_run_that_ships')
UNGATED = _t(T1122, 'ungated_arm_is_handed')
CONTRA = _t(T1122, 'contradiction_writes')
GUARD = _t(T1122, 'escalation_guard')
UNRESOLVED = _t(T1122, 'unresolved_rotation_block')
PARTLY = _t(T1122, 'partly_resolved_block')
OFF_NAMED = _t(T1122, 'off_lattice_member_is_named')
# The GUI gate exits 2 without KiCad's python, which the unmutated baseline
# would report as a refusal before any row runs -- never as a kill.
GUI = _t(os.path.join('gui_parity', 'test_1122_cap_rotation_gui.py'))

# (name, target, old, new, tests, expect)
ROWS = [
    # -- #1121: the portfolio's poses strategy -------------------------------
    ('poses-ignores-the-declaration', 'pf',
     "    if claim is None:",
     "    if True:",
     (DECLARED, CAND_SET), 'KILLED'),
    ('poses-offers-the-current-angle', 'pf',
     "            if not _same_angle(a, part.rot)]",
     "            ]",
     (DECLARED, AUTHOR), 'KILLED'),
    ('poses-ranks-before-it-filters', 'pf',
     "                   and _pose_variants(state.parts[r], declared.get(r))),",
     "                   ),",
     (SLOT,), 'KILLED'),
    ('poses-sorts-a-set', 'pf',
     "    return [a % 360 for a in declared_ladder(claim)",
     "    return [a % 360 for a in sorted(declared_ladder(claim))",
     (AUTHOR,), 'KILLED'),
    ('poses-judges-an-unmaterialised-box', 'pf',
     "                rot = _materialise_rotation(part, rot)",
     "                rot = rot % 360",
     (OFF_LATTICE,), 'KILLED'),
    ('generator-drops-the-declaration', 'pf',
     "                                  declared=_declared)",
     "                                  declared=None)",
     (E2E, GATE), 'KILLED'),
    ('generator-kwarg-dropped-gate-sees-it', 'pf',
     "                                  declared=_declared)",
     "                                  )",
     (GATE,), 'KILLED'),
    ('generator-reads-no-intent', 'pf',
     "    _declared = dict((qkw.get('intent_gate') or {}).get('rotations') or {})",
     "    _declared = {}",
     (E2E,), 'KILLED'),
    ('gate-blind-to-other-callees', 't893',
     "        if name == callee:",
     "        if name == '_try_place':",
     (GATE,), 'KILLED'),
    ('gate-accepts-a-literal-None', 't893',
     "        return '%s=None' % kw",
     "        return None",
     (GATE_CTL,), 'KILLED'),
    # -- #1120: stage 1 applies a candidate set ------------------------------
    ('stage1-set-not-applied', 'sd',
     "    if claim is not None and claim[1]:",
     "    if False:",
     (C5, C7), 'KILLED'),
    ('stage1-set-sorted', 'sd',
     "        set1120 = [c % 360.0 for c in claim[1]]",
     "        set1120 = sorted(c % 360.0 for c in claim[1])",
     (C5,), 'KILLED'),
    ('stage1-set-ignores-fit', 'sd',
     "        fit1120 = [r for r in set1120 if fits is None or fits(r)]",
     "        fit1120 = list(set1120)",
     (C7,), 'KILLED'),
    ('stage1-turns-a-part-inside-its-set', 'sd',
     "        if any(abs((r - part.rot + 180.0) % 360.0 - 180.0) < 1e-6",
     "        if False and any(abs((r - part.rot + 180.0) % 360.0 - 180.0) < 1e-6",
     (C5,), 'KILLED'),
    ('stage1-unfit-set-measured-at-the-input', 'sd',
     "        return fit1120[0] if fit1120 else set1120[0]",
     "        return fit1120[0] if fit1120 else part.rot",
     (C8,), 'KILLED'),
    ('stage1-production-passes-no-fit', 'sd',
     "                fits=lambda r: _stage1_fits(state, part, c, bounds, edge, r))",
     "                fits=None)",
     (C7,), 'KILLED'),
    ('stage1-fits-ignores-width', 'sd',
     "    if lo1120 > hi1120:",
     "    if False:",
     (C7, C10), 'KILLED'),
    ('stage1-fits-judges-the-unturned-box', 'sd',
     "        rot = _materialise_rotation(part, rot)",
     "        pass",
     (C13,), 'KILLED'),
    ('stage1-fits-ignores-window', 'sd',
     "    return max(lo1120, w_lo) <= min(hi1120, w_hi)",
     "    return True",
     (C9, C10), 'KILLED'),
    ('stage1-apply-single-only', 'sd',
     "            if _edge_decl is not None:",
     "            if _edge_decl is not None and _edge_decl[0] is not None:",
     (C5, C7), 'KILLED'),
    ('stage1-apply-first-member', 'sd',
     "                         else _geo_rot) % 360.0",
     "                         else _edge_decl[1][0]) % 360.0",
     (C7,), 'KILLED'),
    ('stage1-set-turn-unnamed', 'sd',
     "                        + (f\", the first of its rotation_candidates \"",
     "                        + (f\", \"",
     (C5,), 'KILLED'),
    ('stage1-unfit-set-unnamed', 'sd',
     "                    f\"edge connector {ref}: none of its declared \"",
     "                    f\"edge connector {ref}: \"",
     (C8,), 'KILLED'),
    # -- #1122: the cap pass ---------------------------------------------------
    ('descent-reads-the-lattice', 'fc',
     "                rots = _cap_rotations(cap, _c1122, allow_rotations, rotate[ref])",
     "                rots = ROTATIONS if (allow_rotations and rotate[ref]) else [cap.rot]",
     (CONFINED, HELD), 'KILLED'),
    ('fallback-reads-the-lattice', 'fc',
     "                rots = _cap_rotations(cap, _c1122, allow_rotations, True)",
     "                rots = ROTATIONS if allow_rotations else [cap.rot]",
     (CONFINED,), 'KILLED'),
    ('helper-ignores-the-claim', 'fc',
     "    if claim is None:",
     "    if True:",
     (ROT_LIST, HELD), 'KILLED'),
    ('helper-drops-the-current-angle', 'fc',
     "    out1122 = [cap.rot]",
     "    out1122 = []",
     (ROT_LIST,), 'KILLED'),
    ('helper-admits-off-lattice', 'fc',
     "    return abs((rot - cap.seed_rot + 45.0) % 90.0 - 45.0) <= 1e-6",
     "    return True",
     (ROT_LIST,), 'KILLED'),
    ('helper-ignores-allow-rotations', 'fc',
     "    if not (allow_rotations and rotate):",
     "    if not rotate:",
     (ROT_LIST,), 'KILLED'),
    ('claims-never-resolved', 'fc',
     "    kw['declared_rotations'] = declared_cap_rotations(intent, pcb_data,",
     "    kw['declared_rotations'] = {} or declared_cap_rotations(None, pcb_data,",
     (HELD, UNGATED), 'KILLED'),
    ('ungated-arm-loses-the-claims', 'fc',
     "                                                    on_move=None))",
     "                                                    on_move=None, declared_rotations=None))",
     (UNGATED, HELD), 'KILLED'),
    ('pass-reads-no-claims', 'fc',
     "    claims1122 = {r: c for r, c in sorted((declared_rotations or {}).items())",
     "    claims1122 = {r: c for r, c in sorted({}.items())",
     (HELD,), 'KILLED'),
    ('disclosure-dropped', 'fc',
     "        print(\"Declared rotations (intent): %d cap(s): %s\" % (",
     "        (\"Declared rotations (intent): %d cap(s): %s\" % (",
     (HELD,), 'KILLED'),
    ('result-key-dropped', 'fc',
     "        out['declared_rotations'] = {",
     "        out['declared_rotations_unused'] = {",
     (HELD,), 'KILLED'),
    ('json-key-dropped', 'pfc',
     "            **({'declared_rotations': result['declared_rotations']}",
     "            **({'declared_rotations_dropped': result['declared_rotations']}",
     (HELD,), 'KILLED'),
    ('cli-refuses-late', 'pfc',
     "            declared_cap_rotations(intent, _pcb1122)",
     "            pass",
     (CONTRA,), 'KILLED'),
    ('helper-first-angle-is-the-seed', 'fc',
     "    out1122 = [cap.rot]",
     "    out1122 = [cap.seed_rot]",
     (ROT_LIST,), 'KILLED'),
    ('helper-turns-a-cap-not-armed', 'fc',
     "    if not (allow_rotations and rotate):",
     "    if not allow_rotations:",
     (ROT_LIST,), 'KILLED'),
    ('escalation-arms-a-turn-it-cannot-make', 'fc',
     "                                                   True, True)) > 1):",
     "                                                   True, True)) > 0):",
     (GUARD,), 'KILLED'),
    ('rotation-block-problems-unsaid', 'fc',
     "            if v.block in rot1122:",
     "            if False:",
     (UNRESOLVED,), 'KILLED'),
    ('warn-every-block', 'fc',
     "            if v.block in rot1122:",
     "            if True:",
     (PARTLY,), 'KILLED'),
    ('holds-no-part-always', 'fc',
     "            held = ('the rotation it declares holds no part' if not",
     "            held = ('the rotation it declares holds no part' if True or not",
     (PARTLY,), 'KILLED'),
    ('one-warning-per-problem', 'fc',
     "                said1122.setdefault(v.block, []).append(v)",
     "                said1122.setdefault((v.block, len(said1122)), []).append(v)",
     (UNRESOLVED, PARTLY), 'KILLED'),
    ('off-lattice-note-dropped', 'fc',
     "              + ''.join(' (%s: %s not offered -- off its quarter-turn '",
     "              + ''.join(' (%s: %s offered -- off its quarter-turn '",
     (OFF_NAMED,), 'KILLED'),
    ('gui-contradiction-runs', 'gui',
     "                    _fc1122.declared_cap_rotations(_cap_intent, pcb_data)",
     "                    pass",
     (GUI,), 'KILLED'),
]

sys.path.insert(0, _TESTS)
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def _run_tests(tests):
    failed = []
    for t in tests:
        p = subprocess.run([sys.executable, '-X', 'utf8', t[0]] + list(t[1:]),
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=2400, cwd=_ROOT)
        if p.returncode != 0:
            failed.append((os.path.basename(t[0]) + ':' + ','.join(t[1:]),
                           p.returncode,
                           [ln.strip()[:90] for ln in
                            ((p.stdout or '') + (p.stderr or '')).splitlines()
                            if 'FAIL' in ln or 'Error' in ln][:2]))
    return failed


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
    # THE UNMUTATED BASELINE: every witness must pass as the code stands,
    # or a row it "kills" proves nothing.
    witnesses = sorted({t for r in rows for t in r[4]})
    base_fail = _run_tests(witnesses)
    if base_fail:
        print('REFUSING: witnesses fail UNMUTATED -- %s' % base_fail)
        return 2
    print('baseline: %d witnesses pass unmutated' % len(witnesses))
    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path = TARGETS[tgt]
            base = orig[tgt]
            o, n = old, new
            if '\r\n' in base:
                o, n = o.replace('\n', '\r\n'), n.replace('\n', '\r\n')
            if base.count(o) != 1 or o == n:
                results.append((name, 'BROKEN', expect,
                                ['anchor matched %d times' % base.count(o)]))
                continue
            io.open(path, 'w', encoding='utf-8', newline='').write(
                base.replace(o, n, 1))
            try:
                failed = _run_tests(tests)
            finally:
                io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if failed else 'SURVIVED',
                            expect, [str(f)[:150] for f in failed[:2]]))
            print('%-40s %s' % (name, results[-1][1]), flush=True)
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])
    wrong = [r for r in results if r[1] != r[2]]
    print('')
    for name, verdict, expect, why in results:
        print('%-40s %-9s%s' % (name, verdict, '' if verdict == expect else
                                '   <-- WRONG, expected %s' % expect))
        for w in why:
            print('      %s' % w)
    print('\n%d rows: %d killed, %d survived, %d broken'
          % (len(results), sum(r[1] == 'KILLED' for r in results),
             sum(r[1] == 'SURVIVED' for r in results),
             sum(r[1] == 'BROKEN' for r in results)))
    return 1 if wrong else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--row', default=None, help='run only this row')
    a = ap.parse_args()
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
