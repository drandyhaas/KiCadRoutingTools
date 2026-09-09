#!/usr/bin/env python3
"""Mutation battery for #893 (declared rotation, facing term) and #916 (body model).

Every row below breaks ONE claim the new gates make, and names the test that
must notice. A row that SURVIVES is a hole in the tests, not a curiosity: it
means the engine can be wrong in that exact way and the suite stays green.

Two of these rows exist because the mistake was made for real during the work:

* `facing-total-cost-sums-per-ref` -- the first draft of `total_cost` summed
  `ref_inversions` over every ref, which double-counts every pair (measured on
  esp_prog: 8 unordered, 16 per-ref).
* `pair-order-pads-always-side-a` -- a draft of the `pad_globals` hoist always
  passed the queried ref's pads as side A. It ran 6.7x faster and reported
  ulx3s U2 as 358 inversions where the truth is 26, and it passed every test in
  the tree at the time.

Run: python3 -X utf8 tests/mutate_893_916.py [--verify-anchors]
"""
import os
import shutil
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
sys.path.insert(0, _TESTS)

QUENCH = os.path.join(_ROOT, 'py_placer', 'placement', 'quench.py')
PAIR_ORDER = os.path.join(_ROOT, 'py_placer', 'placement', 'pair_order.py')
FLOORPLAN = os.path.join(_ROOT, 'py_placer', 'placement', 'floorplan.py')
SEEDER = os.path.join(_ROOT, 'py_placer', 'placement', 'seeder.py')

TARGETS = {'q': QUENCH, 'po': PAIR_ORDER, 'fp': FLOORPLAN, 'sd': SEEDER}

T_EQUIV = os.path.join(_TESTS, 'test_893_pair_order_equivalence.py')
T_FACING = os.path.join(_TESTS, 'test_893_facing_weight.py')
T_ROT = os.path.join(_TESTS, 'test_893_declared_rotation.py')
T_BODY = os.path.join(_TESTS, 'test_916_search_body_model.py')

KILLED = 'KILLED'

ROWS = [
    # ---- #893 pair_order hot path ------------------------------------------
    # The 6.7x-faster WRONG draft, reproduced exactly: hand the queried ref's
    # pads to side A regardless of which slot it occupies.
    ('pair-order-pads-always-side-a', 'po',
     """        m = (pair_metrics(state, ref, other, pose_a=pose, pads_a=pads)
             if ref < other
             else pair_metrics(state, other, ref, pose_b=pose, pads_b=pads))""",
     """        m = (pair_metrics(state, ref, other, pose_a=pose, pads_a=pads)
             if ref < other
             else pair_metrics(state, other, ref, pose_b=pose, pads_a=pads))""",
     (T_EQUIV,), KILLED),

    # The early-out must reproduce pair_metrics' own test. Off by one and it
    # drops real scoring pairs.
    ('pair-order-early-out-drops-two-net-pairs', 'po',
     "        if len(own_nets & _part_net_set(state, other, state.parts[other])) < 2:",
     "        if len(own_nets & _part_net_set(state, other, state.parts[other])) < 3:",
     (T_EQUIV,), KILLED),

    # SURVIVED, and the row is kept because the SURVIVAL is the finding: the
    # intersection is a NO-OP by construction. `net_refs` is built FROM every
    # part's nets (quench.py:985-989) and `part.nets` is filtered by
    # `ignore_net_ids` earlier in that same constructor, so `part.nets` is
    # always a subset of `net_refs`. Measured on esp_prog, tigard and ulx3s:
    # 0 parts of 333 hold a net outside `net_refs`. The original expression
    # this replaced -- `set(pa.nets) & set(pb.nets) & set(state.net_refs)` --
    # was therefore doing a whole-board set build per call for nothing. The
    # intersection is KEPT anyway, because `pair_metrics` accepts any
    # duck-typed state and a caller's stand-in need not honour the invariant;
    # the row records that no test can tell the difference on a real board.
    ('pair-order-part-net-set-ignores-scoring-nets', 'po',
     "        got = set(part.nets) & _scoring_net_ids(state)",
     "        got = set(part.nets)",
     (T_EQUIV,), 'SURVIVED'),

    # ---- #893 facing term --------------------------------------------------
    # The early return is what makes a default run pay nothing AND keeps the
    # A/B's OFF arm a true control.
    ('facing-no-early-return-at-zero', 'q',
     """        if self.facing_weight <= 0.0:
            return 0.0
        return self.facing_weight * ref_inversions(self, ref, x, y, rot)""",
     """        return self.facing_weight * ref_inversions(self, ref, x, y, rot)""",
     (T_FACING,), KILLED),

    # The real bug from the first draft: per-ref summation double-counts.
    ('facing-total-cost-sums-per-ref', 'q',
     """        facing = (self.facing_weight
                  * sum(m['inversions']
                        for m in pair_inversions(self).values())
                  if self.facing_weight > 0.0 else 0.0)""",
     """        facing = (self.facing_weight
                  * sum(ref_inversions(self, r) for r in self.parts)
                  if self.facing_weight > 0.0 else 0.0)""",
     (T_FACING,), KILLED),

    # Dropping the term from the objective entirely.
    ('facing-not-in-part-geometry-cost', 'q',
     "        pen += self._facing_cost(ref, x, y, rot)\n",
     "",
     (T_FACING,), KILLED),

    # ---- #916 body model ---------------------------------------------------
    # The OFF arm must be the inlined ladder. Arming it unconditionally is the
    # "flip the default without measuring" mistake.
    ('body-model-armed-unconditionally', 'q',
     "        self.body_model = bool(body_model)",
     "        self.body_model = True",
     (T_BODY,), KILLED),

    # The seat box must be OCCUPANCY, not the bare body -- taking body_local
    # re-opens the grader/enforcer divergence in the SHRINKING direction.
    ('body-model-uses-bare-body-not-occupancy', 'q',
     "                if _geom.occupancy_local is not None:\n"
     "                    body_locals[_ref] = _geom.occupancy_local",
     "                if _geom.body_local is not None:\n"
     "                    body_locals[_ref] = _geom.body_local",
     (T_BODY,), KILLED),

    # The declared body must actually win over the inlined ladder.
    ('body-local-ignored-by-part', 'q',
     "        lb = body_local\n        if lb is None:",
     "        lb = None\n        if lb is None:",
     (T_BODY,), KILLED),

    # ---- #893 declared rotation -------------------------------------------
    # Declaring both keys must be refused, not silently resolved.
    ('rotation-both-keys-allowed', 'fp',
     "        if b.get('rotation') is not None and b.get('rotation_candidates') is not None:",
     "        if False:",
     (T_ROT,), KILLED),

    # An empty candidate set must be refused, not read as "no constraint".
    ('rotation-empty-candidates-allowed', 'fp',
     """    if not raw:
        raise IntentError(
            f"{where}: rotation_candidates is empty. An empty candidate set "
            f"admits no pose at all; omit the key to leave the rotation free")""",
     """    if not raw:
        return ()""",
     (T_ROT,), KILLED),

    # A bool is an int subclass; letting it through makes `rotation: true` 1.0.
    ('rotation-accepts-a-bool', 'fp',
     "    if isinstance(raw, bool) or not isinstance(raw, (int, float)):",
     "    if not isinstance(raw, (int, float)):",
     (T_ROT,), KILLED),

    # Angles must be normalised, or -90 never equals a declared 270.
    ('rotation-not-normalised', 'fp',
     "    return float(raw) % 360.0",
     "    return float(raw)",
     (T_ROT,), KILLED),

    # Two blocks claiming one ref at different angles must not be resolved by
    # last-writer-wins.
    ('rotation-contradiction-silently-resolved', 'fp',
     """            if prev is not None and prev != claim:
                raise IntentError(""",
     """            if False:
                raise IntentError(""",
     (T_ROT,), KILLED),

    # The declared ladder must reach the seat search, or the claim is inert.
    ('rotation-ladder-not-honoured', 'sd',
     """            for rot in (list(rotations) if rotations is not None
                        else [part.rot] + [(part.rot + d) % 360
                                           for d in (90.0, 180.0, 270.0)]):""",
     """            for rot in [part.rot] + [(part.rot + d) % 360
                                     for d in (90.0, 180.0, 270.0)]:""",
     (T_ROT,), KILLED),

    # A SECOND seating stage. The first version of this work threaded the
    # declared ladder through 2 of 13 `_try_place` sites, so `must_lock`,
    # decap and eviction seats silently used the fallback -- the very failure
    # #893 removes, reintroduced by its own fix. Found in pre-push review;
    # verified by reverting this one site, which places U1 at 0 despite a
    # declared 90.
    ('rotation-ladder-skipped-by-the-must-lock-stage', 'sd',
     """        clr = _try_place(state, ref, part.x, part.y, unplaced - {ref},
                         constraint=rect, tol=tol, info=info,
                         rotations=_rot_ladder(ref))""",
     """        clr = _try_place(state, ref, part.x, part.y, unplaced - {ref},
                         constraint=rect, tol=tol, info=info)""",
     (T_ROT,), KILLED),

    # The candidate ORDER is load-bearing: the search keeps the first that fits.
    ('rotation-candidate-order-sorted-away', 'fp',
     '''            f"{where}: rotation_candidates has repeated angles {out!r}")
    return tuple(out)''',
     '''            f"{where}: rotation_candidates has repeated angles {out!r}")
    return tuple(sorted(out))''',
     (T_ROT,), KILLED),
]

from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _git_clean(paths):
    r = subprocess.run(['git', 'status', '--porcelain', '--'] + list(paths),
                       cwd=_ROOT, capture_output=True, text=True)
    return not r.stdout.strip()


def main():
    if not _git_clean(list(TARGETS.values())):
        print('REFUSED: engine files are dirty; commit before mutating '
              '(a battery restores from ITS OWN snapshot, and an interrupted '
              'run has eaten uncommitted work here before)')
        return 2

    originals = {k: open(v, encoding='utf-8', newline='').read()
                 for k, v in TARGETS.items()}
    results = []
    try:
        for name, key, old, new, witnesses, expect in ROWS:
            path = TARGETS[key]
            src = originals[key]
            if src.count(old) != 1:
                results.append((name, 'BROKEN', 'anchor matches %d times'
                                % src.count(old)))
                continue
            with open(path, 'w', encoding='utf-8', newline='') as fh:
                fh.write(src.replace(old, new, 1))
            got = 'SURVIVED'
            for w in witnesses:
                r = subprocess.run([sys.executable, '-X', 'utf8', w],
                                   cwd=_ROOT, capture_output=True, text=True)
                if r.returncode != 0:
                    got = KILLED
                    break
            with open(path, 'w', encoding='utf-8', newline='') as fh:
                fh.write(src)
            results.append((name, got, 'expected %s' % expect))
            print('%-46s %s' % (name, got))
    finally:
        for k, v in TARGETS.items():
            with open(v, 'w', encoding='utf-8', newline='') as fh:
                fh.write(originals[k])

    bad = [r for r in results
           if r[1] == 'BROKEN' or r[1] != dict(
               (n, e) for n, _k, _o, _n2, _w, e in
               [(x[0], x[1], x[2], x[3], x[4], x[5]) for x in ROWS])[r[0]]]
    print()
    print('%d row(s), %d killed, %d survived, %d broken'
          % (len(results),
             sum(1 for r in results if r[1] == KILLED),
             sum(1 for r in results if r[1] == 'SURVIVED'),
             sum(1 for r in results if r[1] == 'BROKEN')))
    if bad:
        for n, got, note in bad:
            print('  DISAGREES %-42s got %s (%s)' % (n, got, note))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
