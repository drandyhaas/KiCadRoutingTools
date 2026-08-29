#!/usr/bin/env python3
"""The #702 mutation battery, shipped so its numbers can be re-derived.

`tests/test_702_quench_intent_gate.py` records what each arm kills. A count is
only checkable if the exact source edit is written down, so the edits live here
as data, next to the numbers they produced.

Every row carries an EXPECTATION. An inert row recorded as an expected survivor
is a finding; an inert row quietly deleted is a hole. A row whose verdict does
not match its expectation is reported as WRONG.

WHY THIS BATTERY EXISTS AT ALL. The first version of arm F asserted "with the
gate A1 never lands in the keep-out" and PASSED -- while `by_site` came back
`{}`, i.e. the swap conjunct it claimed to test was never reached and the nudge
gate had refused the pose instead. The behavioural assertion was true for a
reason the arm was not testing. Every row below exists to ask the same question
of every other arm.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. It refuses to start on a dirty
engine, because restoring would write the COMMITTED text back over uncommitted
work.

    python3 tests/mutate_702.py
    python3 tests/mutate_702.py --row the-gate-always-says-yes
    python3 tests/mutate_702.py --list

A row is KILLED by a FAILURE **or an ERROR**: several of these make an arm
raise rather than fail, and a battery that counted only failures would call
that a survivor.

An anchor that does not match EXACTLY ONCE is reported as BROKEN rather than
skipped -- a battery that silently applies nothing reports every row as a
survivor, which reads as a catastrophic test failure and is really a stale
anchor. `str.replace(old, new, 1)` of an absent needle returns the file
unchanged, which is why the count is checked BEFORE the write.

Python `str.replace`, never `sed`: `sed` has eaten an unescaped metacharacter
in this repo before and left a SyntaxError behind, and a battery that cannot
start reports nothing at all.

THE MEASURED TABLE IS IN THE HEADER OF `test_702_quench_intent_gate.py`, FROM
THE RUN -- never predicted here and never edited afterwards to match.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

QUENCH = os.path.join(_ROOT, 'py_placer', 'placement', 'quench.py')
FLOORPLAN = os.path.join(_ROOT, 'py_placer', 'placement', 'floorplan.py')
TARGETS = {'q': QUENCH, 'fp': FLOORPLAN}

T702 = os.path.join(_TESTS, 'test_702_quench_intent_gate.py')
T701P = os.path.join(_TESTS, 'test_701_keepout_predicate.py')
T701S = os.path.join(_TESTS, 'test_701_keepout_seating.py')
T549 = os.path.join(_TESTS, 'test_549_floorplan_grade.py')

ROWS = [
    # ---- the gate itself ---------------------------------------------------
    ('the-gate-always-says-yes', 'q',
     "        if rects is None:\n"
     "            rects = self.parts[ref].rects(x, y, rot)\n"
     "        cand = self.intent_terms(ref, rects)\n"
     "        if all(v <= t.threshold for v, t in zip(cand, spec)):\n"
     "            return True\n"
     "        cur = self._incumbent_intent(ref)\n",
     "        return True\n"
     "        if rects is None:\n"
     "            rects = self.parts[ref].rects(x, y, rot)\n"
     "        cand = self.intent_terms(ref, rects)\n"
     "        if all(v <= t.threshold for v, t in zip(cand, spec)):\n"
     "            return True\n"
     "        cur = self._incumbent_intent(ref)\n",
     (T702,), 'KILLED'),

    # ---- each enforcement site, deleted independently -----------------------
    # Three rows, not one, because the whole claim of #702 is that these are
    # THREE sites and the other two are not reachable from the first.
    ('delete-the-conjunct-from-candidate_valid', 'q',
     "        if self._intent_active and not self.intent_ok(ref, x, y, rot, rects):\n"
     "            self._note_intent_refusal(ref, 'candidate_valid', rects)\n"
     "            return False\n",
     "",
     (T702,), 'KILLED'),

    ('delete-the-conjunct-from-the-SWAP-phase', 'q',
     "                        if (state._intent_active\n"
     "                                and not state.swap_intent_ok(ra, rb)):\n",
     "                        if False:\n",
     (T702,), 'KILLED'),

    ('the-swap-conjunct-tests-only-one-half', 'q',
     "        return (self.intent_ok(ra, pb.x, pb.y, pb.rot)\n"
     "                and self.intent_ok(rb, pa.x, pa.y, pa.rot))\n",
     "        return self.intent_ok(rb, pa.x, pa.y, pa.rot)\n",
     (T702,), 'KILLED'),

    # ---- the monotone rule --------------------------------------------------
    ('strict-instead-of-non-strict', 'q',
     "        return all(c <= u + legality.EPS for c, u in zip(cand, cur))\n",
     "        return all(c < u - legality.EPS for c, u in zip(cand, cur))\n",
     (T702,), 'KILLED'),

    ('the-incumbent-is-read-at-the-SEED-pose', 'q',
     "            v = self.intent_terms(ref, self.parts[ref].rects())\n",
     "            _p = self.parts[ref]\n"
     "            v = self.intent_terms(ref, _p.rects(_p.seed_x, _p.seed_y,\n"
     "                                               _p.orig_rot))\n",
     (T702,), 'KILLED'),

    ('the-vector-is-summed-into-a-scalar', 'q',
     "        cand = self.intent_terms(ref, rects)\n"
     "        if all(v <= t.threshold for v, t in zip(cand, spec)):\n"
     "            return True\n"
     "        cur = self._incumbent_intent(ref)\n"
     "        return all(c <= u + legality.EPS for c, u in zip(cand, cur))\n",
     "        cand = self.intent_terms(ref, rects)\n"
     "        if all(v <= t.threshold for v, t in zip(cand, spec)):\n"
     "            return True\n"
     "        cur = self._incumbent_intent(ref)\n"
     "        return sum(cand) <= sum(cur) + legality.EPS\n",
     (T702,), 'KILLED'),

    # ---- per-ENTRY indexing -------------------------------------------------
    # The whole point of _IntentTerm: aggregate a rule's entries to one scalar
    # per ref and a part hops between two keep-outs at 1.0 -> 1.0.
    ('keepout-terms-collapse-to-one-per-ref', 'q',
     "        return zones + tuple(\n"
     "            _IntentTerm('keepout', str(k.get('name') or '<unnamed>'),\n"
     "                        None, 0.0, False, k)\n"
     "            for k in kos)\n",
     "        return zones + (_IntentTerm('keepout', 'all', None, 0.0,\n"
     "                                    False, kos[0]),)\n",
     (T702,), 'KILLED'),

    # The keep-out slice must stay LIVE, or #701's census lift is defeated:
    # `count_legal_poses` removes an entry from `keepouts_for` and recounts,
    # and a frozen copy makes the lift invisible. Measured before the fix:
    # the census went lifted=49 -> lifted=0 on arm Q's fixture, and the
    # verdict degraded from
    # `keepout_blocks` to `no_movable_neighbour`.
    ('the-keepout-slice-stops-honouring-the-lift', 'q',
     "        kos = self.keepouts_for.get(ref, ())\n",
     "        kos = (self.keepouts if self.keepouts_for.get(ref) else ())\n",
     (T702,), 'KILLED'),

    # ---- what the terms MEASURE --------------------------------------------
    ('the-keepout-test-forgets-the-through-hole-rect', 'q',
     "                out.append(_fp.keepout_hit(t.entry, rects))\n",
     "                out.append(_fp.keepout_hit(t.entry, (rects[0],)))\n",
     (T702, T701P), 'KILLED'),

    ('the-anchor-branch-is-never-taken', 'q',
     "                        _anchor = not any(\n"
     "                            _fp.zone_fits_courtyard(\n"
     "                                _z['rect'], _p.rect(0.0, 0.0, _r), _tol)\n"
     "                            for _r in (_p.rot % 360, (_p.rot + 90) % 360))\n",
     "                        _anchor = False\n",
     (T702,), 'KILLED'),

    ('the-zone-tolerance-is-ignored', 'q',
     "                    _tol = float(_z['tolerance_mm'])\n",
     "                    _tol = 0.0\n",
     (T702,), 'KILLED'),

    ('the-exclusive-zone-term-is-dropped', 'q',
     "                        _terms.append(_IntentTerm(\n"
     "                            'zone_exclusive', _z['name'], tuple(_z['rect']),\n"
     "                            legality.EPS, False, None))\n",
     "                        pass\n",
     (T702,), 'KILLED'),

    # ---- the shared measurement, which the GRADE also calls -----------------
    ('zone_escape-grades-the-origin-not-the-centre', 'fp',
     "    if anchor:\n"
     "        cx = (part_rect[0] + part_rect[2]) / 2.0\n"
     "        cy = (part_rect[1] + part_rect[3]) / 2.0\n"
     "        return _rect_escape(zone_rect, (cx, cy, cx, cy))\n",
     "    if anchor:\n"
     "        return _rect_escape(zone_rect, part_rect)\n",
     (T702, T549), 'KILLED'),

    # ---- the load-time contradiction ---------------------------------------
    ('the-crossed-claim-check-never-fires', 'fp',
     "        if not _swallows(k, reach):\n"
     "            continue\n",
     "        if True:\n"
     "            continue\n",
     (T702,), 'KILLED'),

    ('the-crossed-claim-check-fires-on-ANY-overlap', 'fp',
     "    r = entry.get('rect')\n"
     "    if r is not None:\n"
     "        return (r[0] <= rect[0] and r[1] <= rect[1]\n"
     "                and r[2] >= rect[2] and r[3] >= rect[3])\n",
     "    r = entry.get('rect')\n"
     "    if r is not None:\n"
     "        return legality.rect_overlap_area(r, rect) > legality.EPS\n",
     (T702,), 'KILLED'),

    # `allow` and `sides` are the two filters `keepouts_for_ref` exists to
    # centralize, and the contradiction check read neither. Measured before
    # the fix: a `sides: ["B"]` keep-out over an F-side block, and the
    # mounting-hole `allow: ["MH1"]` pattern over MH1's own zone, were each
    # reported as an ERROR contradiction while the grade raised no `keepout`
    # finding at all.
    ('the-contradiction-check-ignores-allow-and-sides', 'fp',
     "        for ref, sides in member_sides.items():\n"
     "            if keepouts_for_ref((k,), ref, sides):\n"
     "                return str(k.get('name') or '<unnamed>')\n",
     "        return str(k.get('name') or '<unnamed>')\n",
     (T702,), 'KILLED'),

    # ---- the metrics, which a keep-out-only intent made self-contradictory --
    ('the-metrics-read-zone-terms-only', 'q',
     "                'refs_bound': len(set(state._intent_spec)\n"
     "                                  | set(state.keepouts_for)),\n",
     "                'refs_bound': len(state._intent_spec),\n",
     (T702,), 'KILLED'),

    # ---- the contradiction check must respect the zone TOLERANCE -----------
    # A zone whose tolerance band reaches outside the keep-out still has legal
    # poses, so reporting it as a contradiction is a false ERROR. Measured on a
    # 4x4 zone at tolerance 2.0 with the keep-out equal to the rect: 5 poses
    # satisfy both rules. Recorded SURVIVED: no arm builds that fixture, and
    # the row is here so the day one does, the guard is already written.
    ('the-swallow-test-ignores-the-tolerance', 'fp',
     "    reach = _inflate(zone.rect, tolerance)\n",
     "    reach = zone.rect\n",
     (T702,), 'SURVIVED'),

    # ---- inertness ----------------------------------------------------------
    ('the-gate-is-built-even-with-no-intent', 'q',
     "        if self.intent_zones or self.keepouts_for:\n",
     "        if True:\n",
     (T702,), 'SURVIVED'),

    # ---- expected survivors, recorded rather than deleted -------------------
    # `_intent_active` is what candidate_valid guards on; flipping it ON with
    # an empty spec changes nothing, because every lookup misses and
    # `intent_ok` returns True on `if not spec`. Recorded so it becomes a
    # detector the day the guard starts meaning something else.
    ('the-active-flag-ignores-whether-anything-bound', 'q',
     "            self._intent_active = bool(self._intent_spec\n"
     "                                       or self.keepouts_for)\n",
     "            self._intent_active = True\n",
     (T702,), 'SURVIVED'),

    # MEASURED KILLED, and I had expected SURVIVED. Arm A asserts
    # `by_site['candidate_valid'] > 0`, so the tally is load-bearing after all:
    # it is what stops an arm passing while the site it names was never
    # reached, which is exactly what arm F did before the counter caught it.
    ('the-refusal-tally-is-never-incremented', 'q',
     "        self.intent_rejected_by_site[site] = (\n"
     "            self.intent_rejected_by_site.get(site, 0) + 1)\n",
     "        pass\n",
     (T702,), 'KILLED'),
]


def _git_clean(paths):
    r = subprocess.run(['git', 'diff', '--quiet', '--'] + list(paths),
                       cwd=_ROOT)
    return r.returncode == 0


def _run(tests):
    for t in tests:
        r = subprocess.run([sys.executable, '-X', 'utf8', t],
                           cwd=_ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        if r.returncode != 0:
            return True, f"{os.path.basename(t)} exit {r.returncode}"
    return False, "all named tests passed"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--row', action='append', default=None)
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()

    if a.list:
        for name, tgt, _o, _n, tests, exp in ROWS:
            print(f"  {exp:9} {name}  [{tgt}] "
                  f"-> {', '.join(os.path.basename(t) for t in tests)}")
        return 0

    rows = ROWS
    if a.row:
        unknown = [n for n in a.row if n not in {r[0] for r in ROWS}]
        if unknown:
            print(f"no such row: {', '.join(unknown)}; try --list",
                  file=sys.stderr)
            return 2
        rows = [r for r in ROWS if r[0] in set(a.row)]

    if not _git_clean(TARGETS.values()):
        print("REFUSED: the engine files are dirty. Restoring would write the "
              "COMMITTED text back over uncommitted work.", file=sys.stderr)
        return 2

    originals = {k: io.open(p, encoding='utf-8').read()
                 for k, p in TARGETS.items()}
    verdicts = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            src = originals[tgt]
            n = src.count(old)
            if n != 1:
                verdicts.append((name, 'BROKEN', f"anchor matched {n} times"))
                print(f"  BROKEN   {name} -- anchor matched {n} times")
                continue
            io.open(TARGETS[tgt], 'w', encoding='utf-8', newline='').write(
                src.replace(old, new, 1))
            killed, why = _run(tests)
            io.open(TARGETS[tgt], 'w', encoding='utf-8',
                    newline='').write(src)
            got = 'KILLED' if killed else 'SURVIVED'
            mark = 'ok' if got == expect else 'WRONG'
            verdicts.append((name, got, why))
            print(f"  {got:9}{'' if mark == 'ok' else ' WRONG'} {name} -- {why}")
    finally:
        for k, p in TARGETS.items():
            io.open(p, 'w', encoding='utf-8', newline='').write(originals[k])

    wrong = [v for v, (name, got, _w) in zip(rows, verdicts)
             if got != v[5]]
    broken = [n for n, g, _w in verdicts if g == 'BROKEN']
    print(f"\n{len(verdicts)} row(s): "
          f"{sum(1 for _n, g, _w in verdicts if g == 'KILLED')} killed, "
          f"{sum(1 for _n, g, _w in verdicts if g == 'SURVIVED')} survived, "
          f"{len(broken)} broken, {len(wrong)} disagreeing with expectation")
    return 1 if (wrong or broken) else 0


if __name__ == '__main__':
    sys.exit(main())
