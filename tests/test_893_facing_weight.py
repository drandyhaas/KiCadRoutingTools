#!/usr/bin/env python3
"""#893: the pin-order facing term, and the two things that make it safe.

`--facing-weight` prices the pin ORDER a pose forces, via
`pair_order.ref_inversions` -- the same lower bound `placement_score` reports,
CALLED rather than re-derived.

Two assertions carry this file, and they are opposites:

* **At 0.0 the objective is BIT-IDENTICAL.** This is not a nicety. The weight is
  the OFF arm of every A/B that will ever judge the term, so a leak at 0.0 makes
  the control arm not a control, and every number measured against it is void.
* **At a non-zero weight it actually BITES**, and specifically it must see
  something `--orient-weight` cannot. Otherwise the honest thing is to delete it
  and tell people to use the term that already exists.

The second is the interesting one. `_orient_cost` already "rewards a pose whose
pads FACE the nets they serve", so a reviewer's first question is what this buys.
The answer is ORDER: `_orient_cost` sums a direction per pad against a net
centroid and is blind to whether the nets cross. Two parts can point their pads
straight at each other with every net inverted -- run 5's U3, where a 180-degree
rotation took the same nets from 4/7 routed to 7/7 while airwire lengths barely
moved. `test_pair_order.py` pins that case; this file pins that the COST built
on it inherits the discrimination.

Run: python3 -X utf8 tests/test_893_facing_weight.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 900
RUN_ALL_FAST_OK = True

BOARD = 'esp_prog.kicad_pcb'

TESTS = []


def _state(**kw):
    from kicad_parser import parse_kicad_pcb
    from placement.quench import QuenchState
    path = run_utils.evidence(os.path.join(ROOT, 'kicad_files', BOARD),
                              'corpus board')
    pcb = parse_kicad_pcb(path)
    return QuenchState(pcb, path, 0.2, 0.55, 30.0, 0.5, 0.15, 2.0, 2.0, 2.0,
                       0.1, 0.3, **kw)


def test_weight_zero_is_bit_identical():
    """Every cost component, and every per-part geometry cost."""
    off = _state()
    on = _state(facing_weight=0.0)
    a, b = off.total_cost(), on.total_cost()
    assert set(a) == set(b), 'the cost dict keys differ: %r vs %r' % (
        sorted(a), sorted(b))
    for k in sorted(a):
        assert a[k] == b[k], (
            'total_cost[%r] differs at facing_weight=0.0: %r != %r' % (k, a[k], b[k]))
    for ref in sorted(off.parts):
        ga = off.part_geometry_cost(ref)
        gb = on.part_geometry_cost(ref)
        assert ga == gb, (
            'part_geometry_cost(%s) differs at facing_weight=0.0: %r != %r'
            % (ref, ga, gb))
    assert a.get('facing') == 0.0, (
        'the facing component must be exactly 0.0 when the weight is 0, not %r'
        % (a.get('facing'),))
    print('  total_cost and %d per-part costs bit-identical at 0.0'
          % len(off.parts))


TESTS.append(test_weight_zero_is_bit_identical)


def test_the_term_returns_before_touching_geometry_at_zero():
    """The early return is what makes a default run pay nothing.

    Asserted by CALLING it on a state whose parts dict would raise if walked,
    rather than by reading the source -- a source grep is satisfiable by a
    comment (and this repo has been bitten by exactly that).
    """
    st = _state()
    ref = sorted(st.parts)[0]
    saved = st.net_refs
    st.net_refs = None          # any real walk raises TypeError
    try:
        got = st._facing_cost(ref)
    finally:
        st.net_refs = saved
    assert got == 0.0, 'expected 0.0 from the early return, got %r' % (got,)
    print('  _facing_cost returns 0.0 without walking anything')


TESTS.append(test_the_term_returns_before_touching_geometry_at_zero)


def test_a_nonzero_weight_changes_the_objective():
    st = _state(facing_weight=1.0)
    cost = st.total_cost()
    assert cost.get('facing', 0.0) > 0.0, (
        'facing_weight=1.0 priced 0.0 on %s -- the term is inert, so no A/B on '
        'it could measure anything' % BOARD)
    base = _state().total_cost()
    assert cost['total'] > base['total'], (
        'the armed total (%r) is not above the unarmed one (%r)'
        % (cost['total'], base['total']))
    print('  facing component %.1f on %s' % (cost['facing'], BOARD))


TESTS.append(test_a_nonzero_weight_changes_the_objective)


def test_it_sees_order_where_orient_weight_cannot():
    """The U3 case, as a COST rather than as a metric.

    A part is rotated 180 degrees. `_facing_cost` must change; `_orient_cost`
    is allowed to change too, but the point is that a rotation exists where
    facing discriminates. Without this, `--facing-weight` is a slower spelling
    of `--orient-weight`.
    """
    st = _state(facing_weight=1.0, orient_weight=1.0)
    discriminating = []
    for ref in sorted(st.parts):
        p = st.parts[ref]
        rot = (p.rot + 180.0) % 360.0
        f0 = st._facing_cost(ref, p.x, p.y, p.rot)
        f1 = st._facing_cost(ref, p.x, p.y, rot)
        if abs(f1 - f0) > 1e-9:
            discriminating.append((ref, f0, f1))
    assert discriminating, (
        'no part on %s changes its facing cost under a 180-degree rotation, so '
        'this board cannot show the term works and the test asserts nothing'
        % BOARD)
    print('  %d part(s) change facing cost under rot+180, e.g. %s %.1f -> %.1f'
          % (len(discriminating), *discriminating[0]))


TESTS.append(test_it_sees_order_where_orient_weight_cannot)


def test_part_geometry_cost_carries_the_term():
    """The hook the MOVE LOOP reads, which `total_cost` does not prove.

    A mutation that deleted `pen += self._facing_cost(...)` from
    `part_geometry_cost` SURVIVED the first run of this file: `total_cost`
    computes its own facing component from `pair_inversions`, and
    `_facing_cost` was only ever called directly. So every assertion here
    passed while the term reached no move the optimizer actually makes -- the
    term would have been inert exactly where it is supposed to act.
    """
    off = _state()
    on = _state(facing_weight=1.0)
    moved = []
    for ref in sorted(off.parts):
        a = off.part_geometry_cost(ref)
        b = on.part_geometry_cost(ref)
        if abs(a - b) > 1e-9:
            moved.append(ref)
    assert moved, (
        'no part geometry cost changed with facing_weight=1.0, so the term '
        'does not reach part_geometry_cost -- the one function the nudge, '
        'group and swap evaluators all call')
    for ref in moved:
        gap = on.part_geometry_cost(ref) - off.part_geometry_cost(ref)
        assert abs(gap - on._facing_cost(ref)) < 1e-9, (
            '%s: part_geometry_cost moved by %r but _facing_cost is %r'
            % (ref, gap, on._facing_cost(ref)))
    print('  part_geometry_cost carries the term on %d part(s)' % len(moved))


TESTS.append(test_part_geometry_cost_carries_the_term)


def test_total_cost_counts_each_pair_once():
    """`ref_inversions` summed over refs would double every pair.

    `total_cost` must use `pair_inversions` (unordered). The check is that the
    reported component equals the unordered sum and NOT twice it -- the exact
    bug the `align` block at the same site documents having avoided.
    """
    from placement.pair_order import pair_inversions, ref_inversions
    st = _state(facing_weight=1.0)
    unordered = sum(m['inversions'] for m in pair_inversions(st).values())
    per_ref = sum(ref_inversions(st, r) for r in st.parts)
    reported = st.total_cost()['facing']
    assert abs(reported - unordered) < 1e-9, (
        'total_cost reported %r; the unordered pair sum is %r'
        % (reported, unordered))
    if unordered:
        assert abs(per_ref - 2 * unordered) < 1e-9, (
            'the per-ref sum (%r) is not twice the unordered sum (%r), so this '
            'test is not actually discriminating the double count'
            % (per_ref, unordered))
        assert abs(reported - per_ref) > 1e-9, (
            'reported == the per-ref sum, i.e. every pair is counted twice')
    print('  facing counts each pair once (%d unordered, %d per-ref)'
          % (unordered, per_ref))


TESTS.append(test_total_cost_counts_each_pair_once)


def main():
    failures = 0
    for fn in TESTS:
        print('%s ...' % fn.__name__)
        try:
            fn()
        except AssertionError as exc:
            failures += 1
            print('FAIL %s: %s' % (fn.__name__, exc))
    if failures:
        print('%d/%d checks FAILED' % (failures, len(TESTS)))
        return 1
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
