#!/usr/bin/env python3
"""#893: an intent can DECLARE a rotation, and the seeder honours or refuses it.

Before this, a rotation could not be written down anywhere the toolchain reads.
`seeder.py`'s own module docstring said so -- "The intent schema cannot express a
rotation, so a part whose rotation is a DECISION ... must be locked" -- and that
advice costs the part its POSITION too, because the lock flag is one boolean
covering both. Run 25 worked around it by rewriting the board before seeding.

Two keys, and they mean different things on purpose:

* `rotation` -- a DECISION. Honoured exactly. If no legal pose exists at it the
  part is reported UNSEATED with the angle named, never quietly turned.
* `rotation_candidates` -- a SET the search may choose from, in the author's
  order, because the seat search keeps the FIRST pose that fits.

Run: python3 -X utf8 tests/test_893_declared_rotation.py
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


def _intent(blocks, **top):
    from placement import floorplan
    doc = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
           'blocks': blocks}
    doc.update(top)
    return floorplan.intent_from_dict(doc)


def test_the_schema_refuses_each_bad_shape_for_its_own_reason():
    """A refusal that fires for the wrong reason is not a guard.

    Each case asserts the MESSAGE, not merely that something raised -- an
    unknown-key refusal firing on a typo would otherwise look like a working
    validator.
    """
    from placement.floorplan import IntentError
    cases = [
        ({'name': 'x', 'refs': ['U1'], 'rotation': 0,
          'rotation_candidates': [90]}, 'BOTH rotation and rotation_candidates'),
        ({'name': 'x', 'refs': ['U1'], 'rotation': 'ninety'},
         'expected a number of degrees'),
        ({'name': 'x', 'refs': ['U1'], 'rotation': True},
         'expected a number of degrees'),
        ({'name': 'x', 'refs': ['U1'], 'rotation_candidates': []},
         'admits no pose at all'),
        ({'name': 'x', 'refs': ['U1'], 'rotation_candidates': [0, 360]},
         'repeated angles'),
        ({'name': 'x', 'refs': ['U1'], 'rotation_candidates': 90},
         'expected a list of degrees'),
    ]
    for block, needle in cases:
        try:
            _intent([block])
        except IntentError as exc:
            assert needle in str(exc), (
                'refused, but NOT for the stated reason. Expected %r in %r'
                % (needle, str(exc)))
        else:
            raise AssertionError('no refusal for %r' % (block,))
    print('  %d bad shapes each refused for their own reason' % len(cases))


TESTS.append(test_the_schema_refuses_each_bad_shape_for_its_own_reason)


def test_an_angle_is_normalised_and_order_is_preserved():
    """KiCad writes -90 where this tool writes 270."""
    i = _intent([{'name': 'a', 'refs': ['U1'], 'rotation': -90},
                 {'name': 'b', 'refs': ['C*'],
                  'rotation_candidates': [270, 0, 90]}])
    assert i.blocks[0].rotation == 270.0, i.blocks[0].rotation
    assert i.blocks[1].rotation_candidates == (270.0, 0.0, 90.0), (
        'the author order was not preserved: %r'
        % (i.blocks[1].rotation_candidates,))
    print('  -90 -> 270.0, candidate order preserved')


TESTS.append(test_an_angle_is_normalised_and_order_is_preserved)


def test_two_blocks_claiming_one_ref_differently_is_refused():
    """Globs overlap legitimately; contradictory angles do not."""
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    from placement.floorplan import IntentError
    path = run_utils.evidence(os.path.join(ROOT, 'kicad_files', BOARD), 'board')
    pcb = parse_kicad_pcb(path)
    same = _intent([{'name': 'a', 'refs': ['U1'], 'rotation': 90},
                    {'name': 'b', 'refs': ['U*'], 'rotation': 90}])
    blocks, _ = floorplan.resolve_blocks(same, pcb, ('kicad', 'sheet'))
    got = floorplan.rotations_for_ref(same, blocks)
    assert got.get('U1') == (90.0, None), got.get('U1')

    clash = _intent([{'name': 'a', 'refs': ['U1'], 'rotation': 90},
                     {'name': 'b', 'refs': ['U*'], 'rotation': 180}])
    blocks, _ = floorplan.resolve_blocks(clash, pcb, ('kicad', 'sheet'))
    try:
        floorplan.rotations_for_ref(clash, blocks)
    except IntentError as exc:
        assert 'different rotations' in str(exc), str(exc)
    else:
        raise AssertionError('two blocks claiming U1 at 90 and 180 was allowed')
    print('  agreeing globs allowed; contradictory ones refused')


TESTS.append(test_two_blocks_claiming_one_ref_differently_is_refused)


def _seed(blocks):
    import random
    from kicad_parser import parse_kicad_pcb
    from placement import seeder
    path = run_utils.evidence(os.path.join(ROOT, 'kicad_files', BOARD), 'board')
    pcb = parse_kicad_pcb(path)
    return seeder.seed_from_intent(pcb, path, _intent(blocks),
                                   random.Random('893'),
                                   group_sources=('kicad', 'sheet')), pcb


def test_a_declared_rotation_is_the_angle_that_is_placed():
    """The whole point: the pose written carries the declared angle."""
    ref = 'U1'
    res, pcb = _seed([{'name': 'r', 'refs': [ref], 'rotation': 90}])
    got = {p['reference']: p for p in res['placements']}
    if ref in got:
        assert abs((got[ref]['new_rotation'] % 360) - 90.0) < 1e-6, (
            '%s was placed at %r, not the declared 90'
            % (ref, got[ref]['new_rotation']))
        print('  %s placed at the declared 90 deg' % ref)
    else:
        # Not placed by this stage is only acceptable if it was REFUSED, which
        # is the other half of the contract -- never "kept its input angle".
        assert ref in (res.get('rotation_unseated') or {}), (
            '%s neither placed at the declared angle nor reported unseated -- '
            'that is the silent-fallback failure #893 exists to remove' % ref)
        print('  %s refused rather than silently kept (declared 90)' % ref)


TESTS.append(test_a_declared_rotation_is_the_angle_that_is_placed)


def test_an_unfittable_declaration_is_refused_by_name():
    """A declared angle nothing can fit must surface AS that claim.

    SELF-CALIBRATING, because the first version of this test was VACUOUS: it
    declared 37 degrees on every part and every part fitted, so it asserted
    nothing and said so. Here the zone is sized from the part's own geometry to
    admit it at its input angle and refuse it turned 90 degrees, and the test
    refuses to run if that separation does not exist on this board.
    """
    import random
    from kicad_parser import parse_kicad_pcb
    from placement import seeder
    import pose_score

    path = run_utils.evidence(os.path.join(ROOT, 'kicad_files', BOARD), 'board')
    pcb = parse_kicad_pcb(path)
    st = pose_score.make_state(pcb, path)

    # Pick the part with the most lopsided body: the one where a 90-degree turn
    # changes the footprint of the box the most.
    best = None
    for ref, part in st.parts.items():
        b0 = part.bounds_by_rot.get(0.0)
        b90 = part.bounds_by_rot.get(90.0)
        if b0 is None or b90 is None:
            continue
        w0, h0 = b0[2] - b0[0], b0[3] - b0[1]
        if w0 <= 0 or h0 <= 0:
            continue
        ratio = max(w0 / h0, h0 / w0)
        if best is None or ratio > best[1]:
            best = (ref, ratio, b0, b90)
    assert best is not None, 'no part on %s has a usable body box' % BOARD
    ref, ratio, b0, _ = best
    assert ratio > 1.5, (
        'the most lopsided part on %s is only %.2f:1, so no zone can admit it '
        'at one angle and refuse it at another -- this board cannot exercise '
        'the refusal' % (BOARD, ratio))

    # A zone that fits the part's INPUT box with room, but is narrower than its
    # long side, so the 90-degree pose cannot be contained.
    part = st.parts[ref]
    w, h = b0[2] - b0[0], b0[3] - b0[1]
    short, long_ = min(w, h), max(w, h)
    cx, cy = part.x, part.y
    half_w, half_h = long_ * 0.6, short * 0.6
    zone = [round(cx - half_w, 3), round(cy - half_h, 3),
            round(cx + half_w, 3), round(cy + half_h, 3)]

    fits = _intent([{'name': 'z', 'refs': [ref], 'zone': zone,
                     'rotation': part.rot}])
    turned = _intent([{'name': 'z', 'refs': [ref], 'zone': zone,
                       'rotation': (part.rot + 90.0) % 360.0}])
    ok = seeder.seed_from_intent(pcb, path, fits, random.Random('893'),
                                 group_sources=('kicad', 'sheet'))
    bad = seeder.seed_from_intent(pcb, path, turned, random.Random('893'),
                                  group_sources=('kicad', 'sheet'))

    assert ref not in (ok.get('rotation_unseated') or {}), (
        '%s could not be seated even at its own angle in a zone sized for it, '
        'so the turned arm proves nothing' % ref)
    unseated = bad.get('rotation_unseated') or {}
    assert ref in unseated, (
        '%s was NOT refused at %g degrees in a zone too narrow for it -- it '
        'was either turned silently or seated outside the claim, which is the '
        'failure #893 exists to remove' % (ref, (part.rot + 90.0) % 360.0))
    assert unseated[ref] == (part.rot + 90.0) % 360.0, unseated
    placed = {p['reference']: p['new_rotation'] % 360
              for p in bad['placements']}
    assert abs(placed.get(ref, (part.rot + 90.0) % 360.0)
               - (part.rot + 90.0) % 360.0) < 1e-6, (
        '%s was placed at %r despite declaring %g'
        % (ref, placed.get(ref), (part.rot + 90.0) % 360.0))
    print('  %s (%.1f:1) fits its zone at %g deg, refused by name at %g'
          % (ref, ratio, part.rot, (part.rot + 90.0) % 360.0))


TESTS.append(test_an_unfittable_declaration_is_refused_by_name)


def test_candidates_restrict_the_ladder():
    """A candidate set must not admit an angle outside it."""
    res, _ = _seed([{'name': 'r', 'refs': ['*'],
                     'rotation_candidates': [0, 180]}])
    bad = {p['reference']: p['new_rotation'] % 360
           for p in res['placements']
           if abs(p['new_rotation'] % 360) > 1e-6
           and abs((p['new_rotation'] % 360) - 180.0) > 1e-6}
    assert not bad, (
        'placed outside the declared candidate set {0, 180}: %r'
        % sorted(bad.items())[:5])
    print('  every placed part took a declared candidate angle')


TESTS.append(test_candidates_restrict_the_ladder)


def test_every_seating_stage_honours_the_declaration():
    """The claim is about the SEEDER, not about one stage of it.

    `seed_from_intent` seats parts from several stages, and `_try_place` is
    called from 13 sites. The first version of this work threaded the declared
    ladder through TWO of them, so a part seated by stage 1.5 (`must_lock`),
    stage 2.5 (the decap seats) or the eviction rung took the FALLBACK ladder
    and could be turned silently -- the exact failure #893 exists to remove,
    reintroduced by the fix for it. The tests above did not catch it because
    their intents declared no locks and no decap rules, so those stages never
    ran.

    This exercises them together: a `must_lock` part, a decap rule, and a zone,
    all with one declared angle over every ref. Nothing may be placed at any
    other angle.
    """
    import random
    from kicad_parser import parse_kicad_pcb
    from placement import seeder

    path = run_utils.evidence(os.path.join(ROOT, 'kicad_files', BOARD), 'board')
    pcb = parse_kicad_pcb(path)
    angle = 90.0
    intent = _intent(
        [{'name': 'all', 'refs': ['*'], 'rotation': angle}],
        must_lock=['U1'],
        decaps={'max_distance_mm': 3.0},
    )
    res = seeder.seed_from_intent(pcb, path, intent, random.Random('893'),
                                  group_sources=('kicad', 'sheet'),
                                  decap_owner_chips=True)
    wrong = {p['reference']: p['new_rotation'] % 360
             for p in res['placements']
             if abs((p['new_rotation'] % 360) - angle) > 1e-6}
    assert not wrong, (
        'placed at an angle other than the declared %g with must_lock and '
        'decap stages live: %r -- a seating stage is not honouring the '
        'declared ladder' % (angle, sorted(wrong.items())[:5]))
    # And the stages must actually have RUN, or this proves nothing.
    assert res['placements'] or res['unseated'], (
        'the seeder neither placed nor refused anything, so no stage ran')
    print('  %d placed, %d unseated, none turned away from %g deg'
          % (len(res['placements']), len(res['unseated']), angle))


TESTS.append(test_every_seating_stage_honours_the_declaration)


def test_no_declaration_leaves_the_seeder_unchanged():
    """`rotations=None` must reproduce the pre-#893 ladder exactly."""
    a, _ = _seed([{'name': 'z', 'refs': ['U1']}])
    b, _ = _seed([{'name': 'z', 'refs': ['U1']}])
    pa = {p['reference']: (round(p['new_x'], 6), round(p['new_y'], 6),
                           p['new_rotation'] % 360) for p in a['placements']}
    pb = {p['reference']: (round(p['new_x'], 6), round(p['new_y'], 6),
                           p['new_rotation'] % 360) for p in b['placements']}
    assert pa == pb, 'an undeclared seed is not deterministic'
    assert not (a.get('rotation_unseated') or {}), a.get('rotation_unseated')
    print('  an undeclared seed is deterministic and claims nothing')


TESTS.append(test_no_declaration_leaves_the_seeder_unchanged)


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
