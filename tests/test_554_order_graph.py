#!/usr/bin/env python3
"""The #554 order graph and reach envelope, on geometry small enough to solve by hand.

`placement/relocate.py` turns "relocate this block, keep everyone else's relative
order" into a difference-constraint system and reads the block's travel off the
envelope. Every number this repo will publish about #554 rests on that system being
right, so it is tested here against arithmetic a reader can check, with fakes only:
no board file, no router, no `pose_score`, nothing shelled out.

THE FIXTURE, AND WHY THESE NUMBERS
-----------------------------------
Three unit squares in a row on one side, `clearance = 0.25`, usable box
`(-10, -10, 10, 10)`::

      A [0,1]      B [2,3]      C [4,5]        (all y in [0,1])

B is the block. Both gaps are 1.0 mm, so each pair's requirement is
`min(0.25, 1.0) = 0.25` and each slack is `-0.75`.

  * **C pinned** -- B may close its own gap to C and no more: `s_B <= 0.75`.
  * **C free** -- C may run to the wall (`s_C <= 10 - 5 = 5`), and B follows it:
    `s_B <= s_C + 0.75 = 5.75`. B's own wall allows 7, so C is what binds.

So the frozen arm reads **0.75** and the yielding arm **5.75**, and the whole
mechanism #554 rests on is that 7.67x, computed by hand before any board was
measured. A test that only asserted `reach >= frozen` would be asserting a theorem
(pinning only removes variables), which is why the exact values are pinned instead.

Run: python3 -X utf8 tests/test_554_order_graph.py
"""
import math
import os
import sys

# Fakes only -- milliseconds, no board file, no router, no shelling out. The
# marker regex anchors on end of line, so this assignment must carry no trailing
# comment: with one, the opt-out silently does not fire and a pure unit test
# drops out of `--fast` for a word that appears in its own prose.
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from placement import relocate as RL           # noqa: E402

_fails = []


def check(cond, msg):
    if not cond:
        _fails.append(msg)
    return cond


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
# fakes
# ---------------------------------------------------------------------------

def _rect_gap(ra, rb):
    dx = max(rb[0] - ra[2], ra[0] - rb[2], 0.0)
    dy = max(rb[1] - ra[3], ra[1] - rb[3], 0.0)
    return math.hypot(dx, dy)


class _Part:
    def __init__(self, ref, x0, y0, x1, y1, locked=False, side='F', isolated=False):
        self.ref = ref
        self._box = (x0, y0, x1, y1)
        self.locked = locked
        self.side = side
        self.isolated = isolated       # stands in for "shares no board side"
        self.x = (x0 + x1) / 2.0
        self.y = (y0 + y1) / 2.0
        self.rot = 0.0

    def rect(self, x=None, y=None, rot=None):
        return self._box

    def rects(self, x=None, y=None, rot=None):
        return (self._box, None)

    def gap_to(self, other, self_rects=None, other_rects=None):
        # `None` is the real contract for "these two share no board side and
        # cannot interact at all" -- every consumer must skip the pair.
        if self.isolated or other.isolated:
            return None
        a = (self_rects or self.rects())[0]
        b = (other_rects or other.rects())[0]
        return _rect_gap(a, b)


class _State:
    def __init__(self, parts, clearance=0.25, usable=(-10.0, -10.0, 10.0, 10.0)):
        self.parts = {p.ref: p for p in parts}
        self.clearance = clearance
        self.usable = usable


def row_fixture(**kw):
    return _State([_Part('A', 0, 0, 1, 1), _Part('B', 2, 0, 3, 1),
                   _Part('C', 4, 0, 5, 1)], **kw)


def _reach(state, blocks, unit, direction, frozen=False):
    units = RL.rigid_units(state, blocks)
    edges = RL.order_graph(state, units)
    if frozen:
        units = RL.pin_all_but(units, unit)
    return RL.reach(state, units, edges, unit, direction), edges


# ---------------------------------------------------------------------------
# the arithmetic
# ---------------------------------------------------------------------------

def t_frozen_arm_closes_its_own_gap_and_no_more():
    r, _ = _reach(row_fixture(), None, 'B', (1.0, 0.0), frozen=True)
    check(close(r.reach_mm, 0.75),
          'frozen reach %.4f, want 0.75 (B closes its 1.0mm gap to C down to the '
          '0.25mm clearance and stops)' % r.reach_mm)


def t_yielding_arm_pushes_the_neighbour_to_the_wall():
    r, _ = _reach(row_fixture(), None, 'B', (1.0, 0.0))
    check(close(r.reach_mm, 5.75),
          'yielding reach %.4f, want 5.75 (C runs to the wall at s=5, B follows '
          'at +0.75)' % r.reach_mm)


def t_the_gain_is_the_mechanism_and_it_is_not_a_theorem():
    """7.67x on geometry with a hand-checkable answer."""
    fr, _ = _reach(row_fixture(), None, 'B', (1.0, 0.0), frozen=True)
    yi, _ = _reach(row_fixture(), None, 'B', (1.0, 0.0))
    check(close(yi.reach_mm / fr.reach_mm, 5.75 / 0.75),
          'ratio %.4f, want %.4f' % (yi.reach_mm / fr.reach_mm, 5.75 / 0.75))


def t_a_tighter_wall_binds_instead_of_the_neighbour():
    """Change ONE thing -- the wall -- and the binding party changes with it."""
    st = row_fixture(usable=(-10.0, -10.0, 6.0, 10.0))
    r, _ = _reach(st, None, 'B', (1.0, 0.0))
    # C may now travel only 6 - 5 = 1.0, so B gets 1.0 + 0.75.
    check(close(r.reach_mm, 1.75),
          'reach %.4f with the wall at x=6, want 1.75' % r.reach_mm)
    check(any(b[2] == 'wall' for b in r.binding['x']),
          'the binding chain does not name the wall that actually bound it: %r'
          % (r.binding['x'],))


def t_travel_in_the_NEGATIVE_direction_reads_the_lower_envelope():
    """Leftward travel is bounded by `lo`, not by `hi`.

    A pass that read `hi` for both directions still returns a float and still
    looks like an answer: the cap comes out negative, gets clamped to 0, and the
    block silently reports that it cannot move left at all. Here it can move
    10.75 -- A retreats to the wall and B follows.
    """
    r, _ = _reach(row_fixture(), None, 'B', (-1.0, 0.0))
    check(close(r.reach_mm, 10.75, tol=5e-4),
          'leftward reach %.4f, want 10.75 (A runs to the low wall at -10 and B '
          'follows at 0.75 behind it)' % r.reach_mm)
    f, _ = _reach(row_fixture(), None, 'B', (-1.0, 0.0), frozen=True)
    check(close(f.reach_mm, 0.75),
          'frozen leftward reach %.4f, want 0.75' % f.reach_mm)
    check(r.binding_axis == 'x',
          'the binding axis is %r, want x' % r.binding_axis)


def t_the_binding_chain_names_the_parts_not_a_number():
    r, _ = _reach(row_fixture(), None, 'B', (1.0, 0.0))
    named = [b[0] for b in r.binding['x']]
    check('C' in named,
          'the chain that stopped B does not name C: %r' % (named,))
    check(all(isinstance(b[1], float) for b in r.binding['x']),
          'a binding row carries no gap in mm: %r' % (r.binding['x'],))


# ---------------------------------------------------------------------------
# the properties everything else rests on
# ---------------------------------------------------------------------------

def t_identity_is_feasible_on_every_edge():
    for st in (row_fixture(),
               # overlapping deeply, and a part hanging outside the usable box
               _State([_Part('A', 0, 0, 3, 1), _Part('B', 1, 0, 4, 1),
                       _Part('C', -12, 0, -11, 1)])):
        units = RL.rigid_units(st, None)
        edges = RL.order_graph(st, units)
        bad = RL.identity_violations(edges)
        check(not bad,
              'identity violated by %d edge(s), first %r' % (len(bad), bad[:1]))


def t_identity_violations_can_actually_fire():
    """Not dead code: hand it a positive slack and it must say so."""
    bad = RL.identity_violations([RL.OrderEdge(axis='x', lo='A', hi='B',
                                               gap_mm=0.25, sep_mm=0.1,
                                               slack=0.15, source='clearance')])
    check(len(bad) == 1, 'identity_violations missed a positive slack')
    st = row_fixture()
    units = RL.rigid_units(st, None)
    edges = RL.order_graph(st, units) + [
        RL.OrderEdge(axis='x', lo='A', hi='B', gap_mm=9.0, sep_mm=1.0,
                     slack=8.0, source='clearance')]
    r = RL.reach(st, units, edges, 'B', (1.0, 0.0))
    check(r.refusal.startswith('order_graph_infeasible'),
          'an edge refusing the incumbent did not produce a named refusal: %r'
          % r.refusal)


def t_an_existing_violation_is_held_not_healed_and_not_deepened():
    """A deeply overlapping pair gets gap == separation, so slack is exactly 0."""
    st = _State([_Part('A', 0, 0, 3, 1), _Part('B', 1, 0, 4, 1)])
    units = RL.rigid_units(st, None)
    edges = [e for e in RL.order_graph(st, units) if e.source != 'wall']
    check(edges, 'the overlapping pair produced no constraint at all')
    e = edges[0]
    check(close(e.gap_mm, e.sep_mm) and close(e.slack, 0.0),
          'overlapping pair got gap %.4f vs sep %.4f (slack %.4f): the rule must '
          'hold the violation constant, neither healing nor deepening it'
          % (e.gap_mm, e.sep_mm, e.slack))
    check(e.source == 'baseline',
          'an overlapping pair was labelled %r, not "baseline"' % e.source)


def t_a_pair_sharing_no_side_gets_no_edge():
    st = _State([_Part('A', 0, 0, 1, 1), _Part('B', 2, 0, 3, 1),
                 _Part('C', 4, 0, 5, 1, isolated=True)])
    units = RL.rigid_units(st, None)
    edges = [e for e in RL.order_graph(st, units) if e.source != 'wall']
    check(all('C' not in (e.lo_ref, e.hi_ref) for e in edges),
          'a pair whose gap_to() returned None still produced an edge: %r'
          % [e.to_dict() for e in edges if 'C' in (e.lo_ref, e.hi_ref)])
    # ... and with nothing between it and the wall, B now reaches further.
    r, _ = _reach(st, None, 'B', (1.0, 0.0))
    check(close(r.reach_mm, 7.0),
          'with C not interacting, B should reach its own wall at 7.0, got %.4f'
          % r.reach_mm)


def t_a_block_moves_as_one_body():
    """Rigidity is variable sharing: both members take the same shift."""
    st = row_fixture()
    units = RL.rigid_units(st, {'blk': ['A', 'B']})
    check(units.of_ref['A'] == units.of_ref['B'] == 'blk',
          'block members did not share a unit: %r' % (units.of_ref,))
    edges = RL.order_graph(st, units)
    check(all(not (e.lo == 'blk' and e.hi == 'blk') for e in edges),
          'an intra-block pair produced a constraint; a rigid translate leaves '
          'that geometry invariant and constraining it vetoes every candidate')
    r = RL.reach(st, units, edges, 'blk', (1.0, 0.0))
    check(close(r.reach_mm, 5.75),
          'block reach %.4f, want 5.75 -- same answer as B alone, because A is '
          'behind B and does not bind rightward travel' % r.reach_mm)


def t_a_block_with_a_locked_member_is_refused_by_name():
    st = _State([_Part('A', 0, 0, 1, 1), _Part('B', 2, 0, 3, 1, locked=True),
                 _Part('C', 4, 0, 5, 1)])
    units = RL.rigid_units(st, {'blk': ['A', 'B']})
    check('blk' in units.pinned, 'a block containing a locked part was not pinned')
    r = RL.reach(st, units, RL.order_graph(st, units), 'blk', (1.0, 0.0))
    check(r.refusal.startswith('block_member_locked'),
          'expected a named block_member_locked refusal, got %r' % r.refusal)
    check(r.reach_mm == 0.0, 'a refused block reported travel %.4f' % r.reach_mm)


def t_a_locked_neighbour_stays_exactly_put():
    """A fixed node takes no incoming edge, so it cannot be relaxed off zero."""
    st = _State([_Part('A', 0, 0, 1, 1), _Part('B', 2, 0, 3, 1),
                 _Part('C', 4, 0, 5, 1, locked=True)])
    units = RL.rigid_units(st, None)
    edges = RL.order_graph(st, units)
    hi, lo, _bind, refusal = RL.envelope(st, units, edges)
    check(not refusal, 'envelope refused: %s' % refusal)
    for axis in RL.AXES:
        check(close(hi[axis]['C'], 0.0) and close(lo[axis]['C'], 0.0),
              'locked C got envelope [%.4f, %.4f] on %s, must be exactly [0, 0]'
              % (lo[axis]['C'], hi[axis]['C'], axis))
    r = RL.reach(st, units, edges, 'B', (1.0, 0.0))
    check(close(r.reach_mm, 0.75),
          'with C locked, B must read the frozen answer 0.75, got %.4f'
          % r.reach_mm)


def t_pinning_only_ever_removes_travel():
    """The ordering is a theorem; assert it so a broken pairing is caught."""
    st = row_fixture()
    units = RL.rigid_units(st, None)
    edges = RL.order_graph(st, units)
    for unit in ('A', 'B', 'C'):
        for d in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.7, -0.7)):
            y = RL.reach(st, units, edges, unit, d)
            f = RL.reach(st, RL.pin_all_but(units, unit), edges, unit, d)
            check(y.reach_mm >= f.reach_mm - 1e-9,
                  '%s toward %r: yielding %.4f < frozen %.4f' %
                  (unit, d, y.reach_mm, f.reach_mm))


def t_the_two_arms_share_one_graph():
    """They must differ in pinning ALONE, including on the wall edges."""
    st = row_fixture()
    units = RL.rigid_units(st, None)
    edges = RL.order_graph(st, units)
    frozen_units = RL.pin_all_but(units, 'B')
    check(units.members is frozen_units.members and
          units.of_ref is frozen_units.of_ref,
          'pin_all_but rebuilt the partition instead of only changing `pinned`')
    walls = {(e.axis, e.lo, e.hi) for e in edges if e.source == 'wall'}
    check(('x', 'C', RL.WALL_HI) in walls,
          "the shared graph lost C's wall edge, so the two arms would not be "
          'measuring the same board: %r' % (sorted(walls)[:4],))


def t_reach_is_jointly_achievable_not_a_per_axis_fantasy():
    """The envelope is a lattice point, so the whole assignment must be legal."""
    st = row_fixture()
    units = RL.rigid_units(st, None)
    edges = RL.order_graph(st, units)
    hi, _lo, _b, refusal = RL.envelope(st, units, edges)
    check(not refusal, 'envelope refused: %s' % refusal)
    s = {u: {a: hi[a][u] for a in RL.AXES} for u in units.members}
    for e in edges:
        shi = 0.0 if e.hi in (RL.WALL_LO, RL.WALL_HI) else s[e.hi][e.axis]
        slo = 0.0 if e.lo in (RL.WALL_LO, RL.WALL_HI) else s[e.lo][e.axis]
        check(shi - slo >= e.slack - 1e-6,
              'the componentwise MAXIMUM is not feasible: %s %s->%s needs %.4f, '
              'assignment gives %.4f. The lattice argument the module rests on is '
              'false for this system.' % (e.axis, e.lo, e.hi, e.slack, shi - slo))


def t_the_direction_is_used_not_just_its_axis():
    st = row_fixture()
    units = RL.rigid_units(st, None)
    edges = RL.order_graph(st, units)
    right = RL.reach(st, units, edges, 'B', (1.0, 0.0)).reach_mm
    diag = RL.reach(st, units, edges, 'B', (1.0, 1.0)).reach_mm
    # Travelling at 45 degrees, the x component is d/sqrt(2), so the same x bound
    # admits sqrt(2) times the path length -- unless y binds first, which here it
    # does: B's own top wall is 10 - 1 = 9, so y is not binding and x is.
    # Tolerance is the module's own output rounding (`round(..., 4)`), not a
    # fudge: comparing an exact irrational against a deliberately quantised
    # result at 1e-6 tests the quantiser, not the envelope.
    check(close(diag, right * math.sqrt(2.0), tol=5e-5),
          'diagonal reach %.6f, want %.6f (the x envelope divided by the x '
          'component)' % (diag, right * math.sqrt(2.0)))
    check(RL.reach(st, units, edges, 'B', (0.0, 0.0)).refusal.startswith(
        'block_has_no_target'),
        'a zero direction did not produce a named refusal')


def t_a_state_with_nothing_movable_is_refused():
    st = _State([_Part('A', 0, 0, 1, 1, locked=True),
                 _Part('B', 2, 0, 3, 1)])
    try:
        RL.assert_relocatable_state(st)
    except RL.RelocateError as e:
        check('move_refs' in str(e),
              'the refusal does not name move_refs, which is the cause a caller '
              'will actually have hit: %s' % e)
        return
    _fails.append('assert_relocatable_state accepted a board with 1 movable part; '
                  'a state built with move_refs= reports "no room" everywhere and '
                  'looks exactly like a correct negative')


def t_results_do_not_depend_on_input_order():
    a = row_fixture()
    b = _State([_Part('C', 4, 0, 5, 1), _Part('B', 2, 0, 3, 1),
                _Part('A', 0, 0, 1, 1)])
    ra, ea = _reach(a, None, 'B', (1.0, 0.0))
    rb, eb = _reach(b, None, 'B', (1.0, 0.0))
    check(close(ra.reach_mm, rb.reach_mm),
          'reach depends on part insertion order: %.6f vs %.6f'
          % (ra.reach_mm, rb.reach_mm))
    check([e.to_dict() for e in ea] == [e.to_dict() for e in eb],
          'the edge list depends on part insertion order')
    check(ra.to_dict() == rb.to_dict(),
          'the serialised result depends on part insertion order')


def main():
    for name, fn in sorted(globals().items()):
        if name.startswith('t_') and callable(fn):
            fn()
    for f in _fails:
        print('FAIL: %s' % f)
    n = sum(1 for k in globals() if k.startswith('t_'))
    print('test_554_order_graph: %s (%d tests, %d checks failed)'
          % ('FAIL' if _fails else 'PASS', n, len(_fails)))
    return 1 if _fails else 0


if __name__ == '__main__':
    sys.exit(main())
