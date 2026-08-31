#!/usr/bin/env python3
"""The #797 mutation battery, shipped so its numbers can be re-derived.

`tests/test_797_zone_exclusive_seating.py` records what each arm kills. A count
is only checkable if the exact source edit is written down, so the edits live
here as data, next to the numbers they produced.

Every row carries an EXPECTATION. An inert row recorded as an expected survivor
is a finding; an inert row quietly deleted is a hole. A row whose verdict does
not match its expectation is reported as WRONG.

WHY THIS BATTERY EXISTS AT ALL, in this issue's own terms. #797 is a gate whose
whole value is that a REFUSAL happens and is NAMED. Both halves are easy to
test vacuously: "the part is out of the rect" is satisfied by any refusal at
all, and "the verdict is zone_exclusive_blocks" is satisfied by a hard-coded
string. Every row below asks whether the arms would notice the corresponding
edit.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. It refuses to start on a dirty
engine, because restoring would write the COMMITTED text back over uncommitted
work.

    python3 -X utf8 tests/mutate_797.py
    python3 -X utf8 tests/mutate_797.py --row the-exclusive-conjunct-is-dropped
    python3 -X utf8 tests/mutate_797.py --list

A row is KILLED by a FAILURE **or an ERROR**: several of these make an arm raise
rather than fail, and a battery that counted only failures would call that a
survivor.

An anchor that does not match EXACTLY ONCE is reported as BROKEN rather than
skipped -- a battery that silently applies nothing reports every row as a
survivor, which reads as a catastrophic test failure and is really a stale
anchor. `str.replace(old, new, 1)` of an absent needle returns the file
unchanged, which is why the count is checked BEFORE the write.

Python `str.replace`, never `sed`: `sed` has eaten an unescaped metacharacter
in this repo before and left a SyntaxError behind, and a battery that cannot
start reports nothing at all.

`_uncache` is carried over from `tests/mutate_703.py` and is NOT hygiene: three
rows here are single-token edits (`elif` -> `if`, `>` -> `>=`) that leave the
file the SAME SIZE, and CPython validates a `.pyc` on (mtime seconds, size)
alone. Mutate, run and restore inside one second and every later import in this
checkout reads the mutant.

THE MEASURED TABLE IS IN THE HEADER OF
`tests/test_797_zone_exclusive_seating.py`, FROM THE RUN -- never predicted
here and never edited afterwards to match.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

SEEDER = os.path.join(_ROOT, 'py_placer', 'placement', 'seeder.py')
QUENCH = os.path.join(_ROOT, 'py_placer', 'placement', 'quench.py')
FLOORPLAN = os.path.join(_ROOT, 'py_placer', 'placement', 'floorplan.py')
PLACE_SEED = os.path.join(_ROOT, 'py_placer', 'place_seed.py')
TARGETS = {'s': SEEDER, 'q': QUENCH, 'fp': FLOORPLAN, 'ps': PLACE_SEED}

T797P = os.path.join(_TESTS, 'test_797_zone_exclusive_predicate.py')
T797S = os.path.join(_TESTS, 'test_797_zone_exclusive_seating.py')
T701P = os.path.join(_TESTS, 'test_701_keepout_predicate.py')
T698 = os.path.join(_TESTS, 'test_698_reseat_acceptance.py')
T702 = os.path.join(_TESTS, 'test_702_quench_intent_gate.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    # ---- the conjunct itself -------------------------------------------
    ('the-exclusive-conjunct-is-dropped', 's',
     "    if not state.exclusive_clear(ref, (r, tht)):\n"
     "        return False\n"
     "    return state.candidate_valid(",
     "    return state.candidate_valid(",
     (T797P, T797S), 'KILLED'),

    ('the-conjunct-is-monotone-instead-of-absolute', 'q',
     "        return all(v <= t.threshold\n"
     "                   for v, t in zip(intent_term_values(spec, rects), spec))",
     "        cand = intent_term_values(spec, rects)\n"
     "        if all(v <= t.threshold for v, t in zip(cand, spec)):\n"
     "            return True\n"
     "        cur = intent_term_values(spec, self.parts[ref].rects())\n"
     "        return all(c <= u + legality.EPS for c, u in zip(cand, cur))",
     (T797S,), 'KILLED'),

    ('the-seed-state-drops-the-exclusive-zones', 's',
     "        exclusive_zones=(floorplan.zone_entries(intent, blocks)\n"
     "                         if intent else ()))\n"
     "    bounds = state.board",
     "        exclusive_zones=())\n"
     "    bounds = state.board",
     (T797S,), 'KILLED'),

    ('the-external-caller-drops-them', 'ps',
     "                    exclusive_zones=floorplan.zone_entries(intent, blocks2))",
     "                    exclusive_zones=())",
     (T701P,), 'KILLED'),

    # ---- the resolution: membership, side, the slice --------------------
    ('members-are-refused-too', 'q',
     "            elif _z['exclusive'] and (not _z['side']",
     "            if _z['exclusive'] and (not _z['side']",
     (T797P, T797S, T702), 'KILLED'),

    ('the-side-filter-is-dropped', 'q',
     "            elif _z['exclusive'] and (not _z['side']\n"
     "                                      or _p.side == _z['side']):",
     "            elif _z['exclusive']:",
     (T797P, T797S), 'KILLED'),

    ('the-gate-side-filter-uses-the-SET-not-the-SCALAR', 'q',
     "                                      or _p.side == _z['side']):",
     "                                      or _z['side'] in _p.sides):",
     (T797P, T797S), 'KILLED'),

    ('the-grade-side-filter-uses-the-SET-not-the-SCALAR', 'fp',
     "            if z.side and part.side != z.side:",
     "            if z.side and z.side not in part.sides:",
     (T797P,), 'KILLED'),

    ('the-grade-drops-the-member-skip', 'fp',
     "            if ref in members:\n                continue",
     "            if False:\n                continue",
     (T797P, T797S), 'KILLED'),

    ('the-exclusive-slice-keeps-every-term', 'q',
     "        _keep = tuple(t for t in _terms if t.rule == 'zone_exclusive')",
     "        _keep = tuple(_terms)",
     (T797P, T698), 'KILLED'),

    ('zone_entries-drops-the-exclusive-flag', 'fp',
     "         'exclusive': bool(z.exclusive)}",
     "         'exclusive': False}",
     (T797P, T797S), 'KILLED'),

    # ---- the measurement: which rects, which threshold ------------------
    ('both-rects-instead-of-courtyard-only', 'q',
     "            out.append(rect_overlap_area(rects[0], t.rect))",
     "            out.append(max(rect_overlap_area(r, t.rect)\n"
     "                           for r in rects if r is not None))",
     (T797P,), 'KILLED'),

    # The two rows below are NECESSARILY blunt, and the reason is the finding
    # that produced them. On real geometry the ONLY value either front can see
    # below `legality.EPS` is exactly 0.0 -- a zero-area touch, which arm X's
    # theorem lands on deliberately. Every threshold in (0, smallest real
    # overlap) therefore behaves identically, which is why the first draft of
    # this row (`v <= t.threshold` -> `v <= 0.0`) SURVIVED: `0.0 <= 0.0` is
    # still clean, so the edit changed nothing a board can produce. Moving the
    # threshold BELOW zero is the only edit that flips the touch verdict, and
    # it necessarily refuses everything else too. Kept because the question it
    # answers -- does a zero-area touch seat -- is the one arm X depends on.
    ('the-gate-refuses-a-zero-area-touch', 'q',
     "        return all(v <= t.threshold\n"
     "                   for v, t in zip(intent_term_values(spec, rects), spec))",
     "        return all(v < 0.0\n"
     "                   for v, t in zip(intent_term_values(spec, rects), spec))",
     (T797S,), 'KILLED'),

    ('the-grade-flags-a-zero-area-touch', 'fp',
     "            area = legality.rect_overlap_area(part.rect, z.rect)\n"
     "            if area > legality.EPS:",
     "            area = legality.rect_overlap_area(part.rect, z.rect)\n"
     "            if area > -1.0:",
     (T797P, T797S), 'KILLED'),

    ('the-zone-rect-is-tolerance-inflated', 'q',
     "                _terms.append(_IntentTerm(\n"
     "                    'zone_exclusive', _z['name'], tuple(_z['rect']),\n"
     "                    legality.EPS, False, None))",
     "                _r = _z['rect']\n"
     "                _terms.append(_IntentTerm(\n"
     "                    'zone_exclusive', _z['name'],\n"
     "                    (_r[0] - _tol, _r[1] - _tol,\n"
     "                     _r[2] + _tol, _r[3] + _tol),\n"
     "                    legality.EPS, False, None))",
     (T797S,), 'KILLED'),

    # ---- the disclosure: the census, the verdict, the prose -------------
    ('the-per-zone-census-returns-an-empty-dict', 's',
     "                    if _n > baseline:\n"
     "                        zx_freeing[_t.name] = _n",
     "                    if False:\n"
     "                        zx_freeing[_t.name] = _n",
     (T797S,), 'KILLED'),

    ('the-census-lift-is-defeated-by-a-frozen-spec', 's',
     "        kept_z = tuple(t for t in saved_z if t.name not in lift_z)",
     "        kept_z = tuple(saved_z)",
     (T797S,), 'KILLED'),

    ('the-joint-census-never-runs', 's',
     "                if not zx_freeing and len(_zb) > 1:",
     "                if False:",
     (T797S,), 'KILLED'),

    ('the-verdict-is-never-produced', 's',
     "    elif (census.get('zone_exclusive_freeing')\n"
     "          or census.get('zone_exclusive_joint')):",
     "    elif False and (census.get('zone_exclusive_freeing')\n"
     "                    or census.get('zone_exclusive_joint')):",
     (T797S,), 'KILLED'),

    ('the-verdict-is-misnamed', 's',
     "    'zone_exclusive_blocks',\n)",
     "    'zone_exclusive_block',\n)",
     (T797S, T701P), 'KILLED'),

    ('the-note-reuses-the-keepout-prose', 's',
     "        return (f\"{ref}: no legal pose, and {what}. Add {ref} to the block \"\n"
     "                f\"that owns the zone, move the zone, or drop its `exclusive` \"\n"
     "                f\"flag\")",
     "        return (f\"{ref}: no legal pose, and {what}. Move a keep-out, or \"\n"
     "                f\"add {ref} to an `allow` list\")",
     (T797S,), 'KILLED'),

    # ---- the paths that bypass pose_ok, and the net under the polish ----
    ('the-edge-seat-has-no-exclusive-conjunct', 's',
     "    _zblockers = state.exclusive_blockers(part.ref, (r, tht))\n"
     "    if _zblockers:\n"
     "        if reasons is not None:\n"
     "            reasons.extend(f\"exclusive zone of block {n!r}\" "
     "for n in _zblockers)\n"
     "        return False\n",
     "",
     (T797S,), 'KILLED'),

    ('the-repair-list-drops-zone_exclusive', 'ps',
     "            _repairable = ('zone_containment', 'keepout', "
     "'zone_exclusive')",
     "            _repairable = ('zone_containment', 'keepout')",
     (T797S,), 'KILLED'),

    ('the-seat-enforced-tuple-drops-the-rule', 's',
     "SEAT_ENFORCED_RULES = ('zone_containment', 'zone_exclusive', 'keepout')",
     "SEAT_ENFORCED_RULES = ('zone_containment', 'keepout')",
     (T797P,), 'KILLED'),

    # ---- the two guards a blind review added ---------------------------
    ('the-unresolved-zone-guard-is-removed', 'q',
     "    zones = tuple(z for z in (zones or ())\n"
     "                  if z.get('refs') or not z.get('exclusive'))\n",
     "",
     (T797P,), 'KILLED'),

    ('the-edge-slide-is-armed-for-keepouts-only', 's',
     "            _slide = ((0.0,) if not (state.keepouts_for.get(ref)\n"
     "                                     or state.exclusive_for.get(ref)) else",
     "            _slide = ((0.0,) if not state.keepouts_for.get(ref) else",
     (T797S,), 'KILLED'),

    # ---- rows kept as EXPECTED SURVIVORS, with the reason ---------------
    #
    # Recorded rather than deleted, per this file's own header: an inert row
    # kept with its reason is a detector for the day the reason stops holding.

    # The threshold DIRECTION at exactly EPS. `> EPS` and `>= EPS` differ on
    # exactly one input -- an overlap of 1e-6 mm2 -- and no board-level fixture
    # can produce one: the smallest intrusion a footprint rect can make is
    # bounded by the file's own coordinate resolution, and the zero-area touch
    # arm X lands on measures 0.0, where both spellings agree.
    # `test_797_zone_exclusive_predicate.py`'s EPS arm pins the direction
    # ARITHMETICALLY (it asserts what `> EPS` and `<= EPS` answer at that
    # input) but does not import this branch, so it cannot observe the edit.
    # The `zero-area-touch` rows above are the killable form of the same
    # question, and they are what actually guards the boundary.
    #
    # The anchor is the two-line form: `if area > legality.EPS:` alone appears
    # TWICE in floorplan.py, and the battery reported the one-line version
    # BROKEN rather than applying it to whichever site came first -- which is
    # the whole point of checking the count before the write.
    ('the-grade-threshold-is-ge-not-gt', 'fp',
     "            area = legality.rect_overlap_area(part.rect, z.rect)\n"
     "            if area > legality.EPS:",
     "            area = legality.rect_overlap_area(part.rect, z.rect)\n"
     "            if area >= legality.EPS:",
     (T797P, T797S), 'SURVIVED'),

    # The GATE's threshold at the same input, and inert for the same reason:
    # `0.0 <= 0.0` is clean, so replacing EPS with zero changes nothing a
    # board can produce. Recorded from the run rather than predicted -- this
    # row was written expecting KILLED and the battery said otherwise.
    ('the-gate-threshold-is-zero-not-EPS', 'q',
     "        return all(v <= t.threshold\n"
     "                   for v, t in zip(intent_term_values(spec, rects), spec))",
     "        return all(v <= 0.0\n"
     "                   for v, t in zip(intent_term_values(spec, rects), spec))",
     (T797P, T797S), 'SURVIVED'),

]


def _uncache(path):
    """Delete the target's cached bytecode. MEASURED HAZARD, not hygiene --
    see this module's docstring."""
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
