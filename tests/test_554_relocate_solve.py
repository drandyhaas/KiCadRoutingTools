#!/usr/bin/env python3
"""The #554 minimum-perturbation solve, the exact re-check, and the dose ladder.

`test_554_order_graph.py` pins the constraint system. This pins what is built on
top of it: given a dose, which neighbours yield and by how much; whether the
result survives a gate the LP cannot represent; and what happens when it does not.

Fakes only -- the same three-squares-in-a-row fixture, so the answers stay
arithmetic a reader can check.

TWO DEFECTS THIS FILE EXISTS TO HOLD DOWN, both shipped and both caught by RUNNING
the pass on real boards rather than by reading it:

  1. **The ladder did not fall.** On the first exact-check refusal `relocate_block`
     returned instead of trying a shorter dose, so `splitflap_driver` reported "no
     room at any dose" with a measured reach of 11.02 mm -- the 0.75 rung never
     ran. A ladder that stops at the first rung is not a ladder.
  2. **The exact re-check was absolute.** It gated on `candidate_valid`, which
     asks "is this pose fully legal" -- and measured on this corpus that is FALSE
     AT THE INCUMBENT POSE for 35% of esp_prog's parts, 61% of tigard's, 49% of
     splitflap's and 99% of watchy's, because human placements sit below the
     0.25 mm courtyard clearance the optimizer asks for. Gated absolutely, the
     pass refuses every shift on every real board for a reason that has nothing
     to do with the shift. It is the same artefact that made the first version of
     `tests/stress/relocation_reach.py` report a phantom null, in a second place.

     The geometry conjunct is therefore baseline-relative ("no worse than the
     board we were handed"), while DECLARED constraints -- keep-outs and intent
     zones -- stay absolute. `t_a_declared_keepout_still_refuses` is what stops
     the relaxation leaking into the half that must not move.

Run: python3 -X utf8 tests/test_554_relocate_solve.py
"""
import math
import os
import sys

# Fakes only -- milliseconds. The marker regex anchors on end of line, so this
# assignment must carry no trailing comment.
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


def close(a, b, tol=1e-4):
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
# fakes
# ---------------------------------------------------------------------------

def _rect_gap(ra, rb):
    dx = max(rb[0] - ra[2], ra[0] - rb[2], 0.0)
    dy = max(rb[1] - ra[3], ra[1] - rb[3], 0.0)
    return math.hypot(dx, dy)


class _Part:
    def __init__(self, ref, x0, y0, x1, y1, locked=False):
        self.ref = ref
        self._w = x1 - x0
        self._h = y1 - y0
        self.x = (x0 + x1) / 2.0
        self.y = (y0 + y1) / 2.0
        self.rot = 0.0
        self.locked = locked

    def rect(self, x=None, y=None, rot=None):
        cx = self.x if x is None else x
        cy = self.y if y is None else y
        return (cx - self._w / 2, cy - self._h / 2,
                cx + self._w / 2, cy + self._h / 2)

    def rects(self, x=None, y=None, rot=None):
        return (self.rect(x, y, rot), None)

    def gap_to(self, other, self_rects=None, other_rects=None):
        a = (self_rects or self.rects())[0]
        b = (other_rects or other.rects())[0]
        return _rect_gap(a, b)


class _State:
    """A QuenchState stand-in with the surface `relocate` actually consumes."""

    def __init__(self, parts, clearance=0.25, usable=(-10.0, -10.0, 10.0, 10.0),
                 keepouts=(), intent_bad=(), pad_bad=()):
        self.parts = {p.ref: p for p in parts}
        self.clearance = clearance
        self.usable = usable
        self._keepouts = keepouts        # rects a part may not enter
        self._intent_bad = set(intent_bad)
        self.legality_ctx = None
        if pad_bad:
            self.legality_ctx = _Ctx(set(pad_bad))

    def violation_parts(self, ref, x=None, y=None, rot=None, exclude=None):
        """(board, overlap) -- the real signature, and the real semantics."""
        p = self.parts[ref]
        r = p.rect(x, y, rot)
        board = max(0.0, self.usable[0] - r[0]) + max(0.0, r[2] - self.usable[2]) \
            + max(0.0, self.usable[1] - r[1]) + max(0.0, r[3] - self.usable[3])
        ex = set(exclude or ())
        overlap = 0.0
        for o, q in self.parts.items():
            if o == ref or o in ex:
                continue
            g = _rect_gap(r, q.rect())
            if g < self.clearance:
                overlap += self.clearance - g
        return board, overlap

    def keepout_clear(self, ref, rects):
        r = rects[0]
        for k in self._keepouts:
            if _rect_gap(r, k) <= 0.0:
                return False
        return True

    def keepout_blockers(self, ref, rects):
        return ['zone'] if not self.keepout_clear(ref, rects) else []

    def intent_ok(self, ref, x, y, rot, rects=None):
        return ref not in self._intent_bad or (x, y) == (self.parts[ref].x,
                                                         self.parts[ref].y)


class _Ctx:
    def __init__(self, bad):
        self.bad = bad

    def pads_ok(self, ref, x, y, rot, neighbors, exclude=None):
        return ref not in self.bad


def row(**kw):
    return _State([_Part('A', 0, 0, 1, 1), _Part('B', 2, 0, 3, 1),
                   _Part('C', 4, 0, 5, 1)], **kw)


# ---------------------------------------------------------------------------
# min_perturbation
# ---------------------------------------------------------------------------

def t_the_neighbour_yields_exactly_what_is_needed_and_no_more():
    """B by 3.0 forces C right by 3.0 - 0.75 = 2.25. A must not move at all."""
    st = row()
    units = RL.rigid_units(st, None)
    edges = RL.order_graph(st, units)
    moves, solver = RL.min_perturbation(st, units, edges, 'B', (3.0, 0.0))
    check(moves is not None, 'min_perturbation refused a reachable dose: %s' % solver)
    if moves is None:
        return
    check(close(moves['B'][0], 3.0), 'B took %r, want 3.0' % (moves['B'],))
    check(close(moves['C'][0], 2.25),
          'C yielded %r, want exactly 2.25 -- the minimum that keeps the 0.25mm '
          'clearance' % (moves['C'],))
    check(close(moves.get('A', (0.0, 0.0))[0], 0.0),
          'A moved %r; nothing required it to, and "minimize displacement" means '
          'the corridor is the parts that HAD to yield' % (moves.get('A'),))


def t_a_short_dose_needs_no_corridor_at_all():
    st = row()
    units = RL.rigid_units(st, None)
    edges = RL.order_graph(st, units)
    moves, _s = RL.min_perturbation(st, units, edges, 'B', (0.5, 0.0))
    check(moves is not None and close(moves['C'][0], 0.0),
          'C yielded for a 0.5mm move that fits inside the existing 1.0mm gap: %r'
          % (moves,))


def t_a_dose_past_the_envelope_is_refused_by_name():
    st = row()
    units = RL.rigid_units(st, None)
    edges = RL.order_graph(st, units)
    moves, why = RL.min_perturbation(st, units, edges, 'B', (99.0, 0.0))
    check(moves is None, 'a 99mm dose on a 20mm board was accepted')
    check('no_room_at_any_dose' in (why or ''),
          'refusal is not named: %r' % why)


def t_a_pinned_neighbour_cannot_be_asked_to_yield():
    st = row()
    units = RL.rigid_units(st, None)
    edges = RL.order_graph(st, units)
    moves, _s = RL.min_perturbation(st, units, edges, 'B', (0.75, 0.0),
                                    pinned=('C',))
    check(moves is not None and close(moves['C'][0], 0.0),
          'a pinned unit moved: %r' % (moves,))
    m2, why = RL.min_perturbation(st, units, edges, 'B', (3.0, 0.0),
                                  pinned=('C',))
    check(m2 is None, 'a dose needing a pinned unit to yield was accepted: %r' % (m2,))


# ---------------------------------------------------------------------------
# the exact re-check
# ---------------------------------------------------------------------------

def t_the_geometry_conjunct_is_baseline_relative():
    """THE trap. A board whose incumbent already overlaps must still be movable."""
    st = _State([_Part('A', 0, 0, 1, 1), _Part('B', 0.9, 0, 1.9, 1),
                 _Part('C', 6, 0, 7, 1)])
    units = RL.rigid_units(st, None)
    before = sum(st.violation_parts('B', exclude=set()))
    check(before > 0, 'the fixture is not actually violating, so it tests nothing')
    # Move B AWAY from A: the overlap improves, so a no-worse rule must accept.
    why = RL.exact_refusal(st, units, {'B': (1.0, 0.0)})
    check(why == '',
          'a shift that IMPROVES an existing overlap was refused: %r. An absolute '
          'gate refuses every shift on 35-99%% of real parts.' % why)
    # ... and the case that actually distinguishes the two rules: a shift that
    # improves the violation but does NOT clear it. An absolute gate refuses this
    # (the pose is still not legal); a no-worse gate accepts it. The test above
    # passes under BOTH rules, because it lands at zero violation -- which is
    # exactly why the `exact-check-absolute` mutation survived until this check
    # existed.
    small = RL.exact_refusal(st, units, {'B': (0.2, 0.0)})
    after = sum(st.violation_parts('B', st.parts['B'].x + 0.2,
                                   st.parts['B'].y, 0.0, exclude={'B'}))
    check(0 < after < before,
          'the fixture must leave a REDUCED but non-zero violation for this to '
          'separate the two rules: before %.4f, after %.4f' % (before, after))
    check(small == '',
          'a shift that reduces a violation without clearing it was refused: %r. '
          'That is the absolute rule, and it refuses every shift on a board whose '
          'incumbent already violates -- which is most of them.' % small)


def t_a_shift_that_worsens_geometry_is_refused_by_name():
    st = _State([_Part('A', 0, 0, 1, 1), _Part('B', 3, 0, 4, 1)])
    why = RL.exact_refusal(st, RL.rigid_units(st, None), {'B': (-1.8, 0.0)})
    check(why.startswith('geometry_worsened:'),
          'moving B onto A was not refused by name: %r' % why)


def t_a_declared_keepout_still_refuses():
    """The relaxation must NOT leak into declared constraints."""
    st = row(keepouts=((3.5, -1.0, 6.0, 2.0),))
    why = RL.exact_refusal(st, RL.rigid_units(st, None), {'B': (0.75, 0.0)})
    check(why.startswith('declared_keepout_refused_a_shift:B'),
          'a declared keep-out did not stop a shift into it: %r' % why)


def t_a_keepout_does_not_freeze_a_part_already_inside_it():
    """Monotone, like the declared-intent gate: it stops a part being walked IN.

    The absolute reading looks stricter and is worse. C already overlaps this
    keep-out, so under an absolute rule C could never yield and one declaration
    would freeze the entire corridor -- which is what a fixture measured before
    the rule was made monotone.
    """
    st = row(keepouts=((4.6, -1.0, 9.0, 2.0),))
    check(not st.keepout_clear('C', st.parts['C'].rects()),
          'the fixture C is not actually inside the keep-out, so this tests '
          'nothing')
    check(RL.exact_refusal(st, RL.rigid_units(st, None), {'C': (0.2, 0.0)}) == '',
          'a part already inside a declared keep-out was frozen by it')
    # ... while a part that is CLEAR is still stopped from entering. On a
    # fixture with no neighbour in the way, so the geometry conjunct (which runs
    # first) cannot be what refuses -- otherwise this would pass without the
    # keep-out ever being consulted.
    solo = _State([_Part('A', 0, 0, 1, 1), _Part('B', 2, 0, 3, 1)],
                  keepouts=((4.6, -1.0, 9.0, 2.0),))
    why = RL.exact_refusal(solo, RL.rigid_units(solo, None), {'B': (2.0, 0.0)})
    check(why.startswith('declared_keepout_refused_a_shift:B'),
          'the monotone rule let a clear part walk into the keep-out: %r' % why)


def t_a_declared_intent_claim_still_refuses():
    st = row(intent_bad=('B',))
    why = RL.exact_refusal(st, RL.rigid_units(st, None), {'B': (0.5, 0.0)})
    check(why.startswith('declared_claim_refused_a_shift:B'),
          'a declared zone did not stop a shift out of it: %r' % why)


def t_the_pad_gate_still_refuses():
    st = row(pad_bad=('B',))
    why = RL.exact_refusal(st, RL.rigid_units(st, None), {'B': (0.5, 0.0)})
    check(why.startswith('pad_gate_refused_a_shift:B'),
          'the pad/hole gate was not consulted: %r' % why)


def t_two_parts_that_both_move_are_checked_against_each_other():
    """The per-part check EXCLUDES every moved ref, so this pair is invisible to it.

    That exclusion is necessary -- `state.parts` still holds the old poses, so
    testing a new pose against stale neighbours tests a board that will never
    exist -- and it leaves exactly one hole: two parts that both move onto the
    same spot. The LP should never propose it; the check is the belt.
    """
    st = _State([_Part('A', 0, 0, 1, 1), _Part('C', 4, 0, 5, 1)])
    units = RL.rigid_units(st, None)
    solo = RL.exact_refusal(st, units, {'A': (2.0, 0.0)})
    check(solo == '', 'moving A alone into empty space was refused: %r' % solo)
    both = RL.exact_refusal(st, units, {'A': (2.0, 0.0), 'C': (-2.0, 0.0)})
    check(both.startswith('moved_pair_worsened:'),
          'two parts moved onto the same spot were not caught: %r' % both)


def t_the_exact_check_catches_what_the_GRAPH_permits():
    """The load-bearing safety claim, on the case that falsified the old one.

    The graph constrains ONE axis per pair, while a real gap is Euclidean. A pair
    separated (0.24, 0.24) has a true gap of 0.339 -- legal at 0.25 -- so the
    x-axis edge only requires 0.24, and shrinking y to 0 satisfies every edge
    while driving the pair to 0.240 and into violation. Measured live on watchy,
    glasgow_revC and ulx3s before this test existed.

    So the module's guarantee is NOT "the graph cannot propose an illegal board".
    It is "nothing is applied on the graph's word". This asserts the second.
    """
    st = _State([_Part('A', 0.0, 0.0, 1.0, 1.0),
                 _Part('B', 1.24, 1.24, 2.24, 2.24)])
    units = RL.rigid_units(st, None)
    edges = [e for e in RL.order_graph(st, units) if e.source != 'wall']
    check(len(edges) == 1, 'expected one pair edge, got %d' % len(edges))
    if edges:
        e = edges[0]
        # The graph asks only for the axis separation it already has ...
        check(close(e.gap_mm, 0.24, tol=1e-3),
              'the pair edge requires %.4f; the diagonal case needs it to be the '
              '0.24 axis separation, not the 0.339 true gap' % e.gap_mm)
    # ... and this shift satisfies it while destroying the true gap.
    ok_by_graph = RL.min_perturbation(st, units, edges, 'B', (0.0, -1.24))[0]
    check(ok_by_graph is not None,
          'the graph refused a shift it is supposed to permit, so this test no '
          'longer exercises the hole it was written for')
    why = RL.exact_refusal(st, units, {'B': (0.0, -1.24)})
    check(why.startswith('geometry_worsened:'),
          'THE SAFETY CLAIM FAILED: a shift the graph permits drove a legal pair '
          'sub-clearance and the exact check let it through: %r' % why)


def t_moving_nothing_is_never_a_refusal():
    st = row()
    check(RL.exact_refusal(st, RL.rigid_units(st, None), {'B': (0.0, 0.0)}) == '',
          'an empty move set produced a refusal')


# ---------------------------------------------------------------------------
# the pass
# ---------------------------------------------------------------------------

def t_the_ladder_falls_to_a_shorter_dose():
    """Defect 1: a keep-out 2mm out must not kill the whole ladder."""
    st = row(keepouts=((4.6, -1.0, 9.0, 2.0),))
    rel = RL.relocate_block(st, None, 'B', (1.0, 0.0), want_mm=5.75)
    check(not rel.refusal,
          'the ladder gave up instead of trying a shorter rung: %r' % rel.refusal)
    check(0 < rel.dose_mm < 5.75,
          'expected a partial dose, got %.4f' % rel.dose_mm)


def t_a_pin_from_one_rung_does_not_follow_the_ladder_down():
    """Pins are per-RUNG. Carrying them makes every later rung fail for the first
    rung's reason -- and the failure looks exactly like "there is no room".

    The keep-out sits 1.0 mm to C's right, so:
      * long doses need C to yield more than 1.0 mm  -> refused -> C pinned;
      * the 0.25 rung needs C to yield only 0.6875   -> legal, and must succeed.
    With pins carried forward, C is still frozen at the 0.25 rung, B is capped at
    its own 0.75 mm gap, and the pass reports no_room_at_any_dose on a board that
    plainly has room.
    """
    st = row(keepouts=((6.0, -1.0, 9.0, 2.0),))
    rel = RL.relocate_block(st, None, 'B', (1.0, 0.0), want_mm=5.75)
    check(not rel.refusal,
          'the 0.25 rung was refused for the 1.0 rung\'s reason: %r' % rel.refusal)
    if not rel.refusal:
        check(close(rel.dose_mm, 1.4375, tol=1e-3),
              'expected the 0.25 rung (1.4375 mm), got %.4f' % rel.dose_mm)
        check('C' in rel.corridor,
              'C had to yield 0.6875 mm for this dose but is not in the corridor: '
              '%r' % (rel.corridor,))


def t_a_reachable_block_reports_its_corridor_and_its_binding_chain():
    st = row()
    rel = RL.relocate_block(st, None, 'B', (1.0, 0.0), want_mm=5.75)
    check(not rel.refusal, 'refused: %s' % rel.refusal)
    check(close(rel.reach_mm, 5.75) and close(rel.frozen_reach_mm, 0.75),
          'reach %.4f / frozen %.4f, want 5.75 / 0.75'
          % (rel.reach_mm, rel.frozen_reach_mm))
    check('C' in rel.corridor,
          'C yielded but is not named in the corridor: %r' % (rel.corridor,))
    check(rel.corridor_mm > 0, 'a corridor with zero cost')
    check(any(b[0] == 'C' for b in rel.binding_path),
          'the binding chain does not name C: %r' % (rel.binding_path,))
    check(rel.moves and all(
        set(m) == {'reference', 'new_x', 'new_y', 'new_rotation'}
        for m in rel.moves),
        'moves are not writer-shaped: %r' % (rel.moves[:1],))


def t_the_corridor_budget_refuses_a_disturbing_move():
    st = row()
    free = RL.relocate_block(st, None, 'B', (1.0, 0.0), want_mm=5.75)
    check(free.corridor_mm > 0.5, 'fixture corridor too cheap to test a budget')
    tight = RL.relocate_block(st, None, 'B', (1.0, 0.0), want_mm=5.75,
                              max_corridor_mm=0.01)
    check(tight.refusal or tight.corridor_mm <= 0.01,
          'the corridor budget did not bind: %.4f mm moved with a 0.01 mm budget'
          % tight.corridor_mm)
    if tight.refusal:
        check('corridor_over_budget' in tight.refusal or
              'no_room_at_any_dose' in tight.refusal,
              'budget refusal is not named: %r' % tight.refusal)


def t_a_block_that_cannot_move_says_so_rather_than_proposing_nothing():
    # Gaps EXACTLY at the clearance, C locked, and the walls flush against A
    # and C: every direction is already at its bound, so no dose is available.
    st = _State([_Part('A', 0, 0, 1, 1), _Part('B', 1.25, 0, 2.25, 1),
                 _Part('C', 2.5, 0, 3.5, 1, locked=True)],
                usable=(0.0, 0.0, 3.5, 1.0))
    rel = RL.relocate_block(st, None, 'B', (1.0, 0.0), want_mm=5.0)
    check(rel.refusal.startswith('no_room_at_any_dose'),
          'a boxed-in block did not report no_room_at_any_dose: %r' % rel.refusal)
    check(not rel.moves, 'a refused relocation still emitted moves')


def t_every_proposal_carries_the_no_efficacy_disclosure():
    from placement.diagnosis import NO_EFFICACY_CLAIM
    for kw in ({}, {'max_corridor_mm': 0.0}):
        rel = RL.relocate_block(row(), None, 'B', (1.0, 0.0), want_mm=5.75, **kw)
        check(NO_EFFICACY_CLAIM in rel.disclosures,
              'a relocation dropped the diagnosis disclosure it inherits')
        check(NO_EFFICACY_CLAIM in (rel.to_dict()['disclosures']),
              'to_dict() dropped the disclosure, so a machine consumer never '
              'sees it')
        check(RL.NO_EFFICACY_CLAIM in rel.disclosures,
              "the relocation's OWN disclosure is missing; the selector's is "
              'about a different experiment and does not substitute for it')
        check('relocate-on vs relocate-off' in RL.NO_EFFICACY_CLAIM,
              'the relocation disclosure names the wrong comparison: %r. A '
              'sentence that says NOT MEASURED about someone else\'s experiment '
              'reads like a disclosure and is not one.' % RL.NO_EFFICACY_CLAIM)


def t_an_unknown_block_is_refused_by_name():
    rel = RL.relocate_block(row(), None, 'nope', (1.0, 0.0), want_mm=1.0)
    check(rel.refusal.startswith('no_diagnosed_block'),
          'unknown block refusal: %r' % rel.refusal)


def t_the_result_serialises_to_strict_json():
    """`inf` would serialise as the literal Infinity and poison any baseline."""
    import json
    rel = RL.relocate_block(row(), None, 'B', (1.0, 0.0), want_mm=5.75)
    txt = json.dumps(rel.to_dict(), sort_keys=True)
    try:
        json.loads(txt, parse_constant=lambda c: (_ for _ in ()).throw(
            ValueError('non-finite %s' % c)))
    except ValueError as e:
        _fails.append('to_dict() is not strict JSON: %s' % e)


def t_results_do_not_depend_on_input_order():
    a = RL.relocate_block(row(), None, 'B', (1.0, 0.0), want_mm=5.75)
    st = _State([_Part('C', 4, 0, 5, 1), _Part('B', 2, 0, 3, 1),
                 _Part('A', 0, 0, 1, 1)])
    b = RL.relocate_block(st, None, 'B', (1.0, 0.0), want_mm=5.75)
    check(a.to_dict() == b.to_dict(),
          'the proposal depends on part insertion order')


def main():
    for name, fn in sorted(globals().items()):
        if name.startswith('t_') and callable(fn):
            fn()
    for f in _fails:
        print('FAIL: %s' % f)
    n = sum(1 for k in globals() if k.startswith('t_'))
    print('test_554_relocate_solve: %s (%d tests, %d checks failed)'
          % ('FAIL' if _fails else 'PASS', n, len(_fails)))
    return 1 if _fails else 0


if __name__ == '__main__':
    sys.exit(main())
