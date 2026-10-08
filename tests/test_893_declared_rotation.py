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
import ast
import io
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
    called from 15 sites in seeder.py (one more, outside it, in place_seed's
    post-polish re-seat). The first version of this work threaded the declared
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


def test_a_declared_rotation_survives_the_quench():
    """The seeder honouring it once is not enough -- the next step must too.

    `place_seed` is followed by `place_optimize` / `place_route_loop` in every
    chain, and the quench turns unlocked parts freely. Before this the module
    docstring told authors to declare a rotation INSTEAD of locking the part,
    while locking was the only thing that had ever protected the angle -- so
    the replacement was strictly WEAKER than the advice it replaced, against
    the very U3 case it cites. Found in pre-push review.

    The declaration reaches the quench through the intent gate, so this drives
    the real `resolve_intent_gate` rather than hand-building a map.
    """
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    from placement.quench import quench, _candidate_rotations, QuenchState

    path = run_utils.evidence(os.path.join(ROOT, 'kicad_files', BOARD), 'board')
    pcb = parse_kicad_pcb(path)
    intent = _intent([{'name': 'r', 'refs': ['R*'], 'rotation': 90}])
    gate, _problems = floorplan.resolve_intent_gate(intent, pcb,
                                                    ('kicad', 'sheet'))
    assert gate.get('rotations'), (
        'resolve_intent_gate carried no rotations, so the quench cannot see '
        'the declaration however well it handles one')

    st = QuenchState(pcb, path, 0.2, 0.55, 30.0, 0.5, 0.15, 2.0, 2.0, 2.0,
                     0.1, 0.3, declared_rotations=gate['rotations'])
    declared = [r for r in sorted(st.parts) if r in gate['rotations']]
    assert declared, 'no part on %s matched R*, so nothing is declared' % BOARD
    for ref in declared:
        got = _candidate_rotations(st.parts[ref], True,
                                   st.declared_rotations.get(ref))
        assert got == [90.0], (
            '%s: the quench offers %r, so it can still turn a part whose '
            'rotation was declared' % (ref, got))
    # And an UNDECLARED part must keep its full lattice, or this has broken
    # the optimizer for every board that declares nothing.
    other = [r for r in sorted(st.parts) if r not in gate['rotations']]
    assert other, 'every part is declared; cannot check the undeclared arm'
    free = _candidate_rotations(st.parts[other[0]], True,
                                st.declared_rotations.get(other[0]))
    assert len(free) >= 4, (
        '%s is undeclared but was offered only %r -- the declaration leaked '
        'onto parts nobody claimed' % (other[0], free))
    print('  %d declared part(s) pinned to 90 deg; %s keeps %d rotations'
          % (len(declared), other[0], len(free)))


TESTS.append(test_a_declared_rotation_survives_the_quench)


def test_every_try_place_site_passes_a_rotation_ladder():
    """A STANDING gate: no seating site may quietly use the fallback.

    The behavioural tests above can only cover the stages a fixture happens to
    reach, and that is exactly how this was missed twice -- first 2 of 13 sites
    were threaded, then 8 of 13, and both times the suite was green because no
    intent in it declared a lock, a decap rule, or an eviction. A new stage
    added next year would repeat it.

    So this reads the SOURCE and requires every `_try_place` call to pass
    `rotations=`. It is a shape assertion, not a grep for a comment: it parses
    the file and checks the keyword is present in each call's arguments, so a
    mention in prose cannot satisfy it.

    It missed a THIRD time, and the reason is the gate's own scope (#1117):
    it read only seeder.py and only bare-name calls, and the one site that
    really did omit the ladder -- `place_seed`'s post-polish re-seat -- is
    `seeder._try_place(...)` in another file. So it now reads every source
    tree, matches attribute calls too, and refuses a literal `rotations=None`
    (a site that "passes" the keyword and still searches the fallback).
    `tests/` is not read: test_run26_rotate_by_facing calls the fallback on
    purpose, as a unit test of it.
    """
    missing, per_file = [], {}
    for path in _ladder_source_files():
        rel = os.path.relpath(path, ROOT).replace(os.sep, '/')
        try:
            tree = ast.parse(io.open(path, encoding='utf-8').read(), path)
        except SyntaxError as exc:
            raise AssertionError('%s does not parse (%s): the gate cannot '
                                 'say what it does not read' % (rel, exc))
        for call in _try_place_calls(tree):
            per_file[rel] = per_file.get(rel, 0) + 1
            why = _ladder_missing(call)
            if why:
                missing.append('%s:%d (%s)' % (rel, call.lineno, why))
    total = sum(per_file.values())
    assert total >= 16 and 'py_placer/place_seed.py' in per_file, (
        'only %d `_try_place` call(s) found (%r) -- this gate is not looking '
        'at what it thinks it is' % (total, per_file))
    assert not missing, (
        '%d of %d `_try_place` call(s) do not pass a rotation ladder: %r. '
        'A seating site that omits it uses the FALLBACK ladder and can turn a '
        'part whose rotation was DECLARED -- silently, which is the failure '
        '#893 exists to remove.' % (len(missing), total, missing))
    print('  all %d _try_place call sites pass a rotation ladder (%s)'
          % (total, ', '.join('%s %d' % kv for kv in sorted(per_file.items()))))


TESTS.append(test_every_try_place_site_passes_a_rotation_ladder)


#: The trees the standing gate reads (#1117). Production source only.
_LADDER_TREES = ('py_placer', 'py_router', 'py_tools', 'kicad_routing_plugin')


def _ladder_source_files():
    out = []
    for tree in _LADDER_TREES:
        for dirpath, dirnames, filenames in os.walk(os.path.join(ROOT, tree)):
            dirnames[:] = [d for d in dirnames if d != '__pycache__']
            out.extend(os.path.join(dirpath, f) for f in sorted(filenames)
                       if f.endswith('.py'))
    return sorted(out)


def _try_place_calls(tree, callee='_try_place'):
    """Every call to `callee` (default `_try_place`), spelled bare or as an
    attribute (`seeder._try_place`)."""
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        name = (fn.id if isinstance(fn, ast.Name)
                else fn.attr if isinstance(fn, ast.Attribute) else None)
        if name == callee:
            yield node


def _ladder_missing(call, kw='rotations'):
    """Why `call` does not hand over its declaration keyword `kw` (default
    `rotations`, the ladder), or None if it does. A `**kw` call cannot be
    read, so it counts as missing."""
    kws = {k.arg: k.value for k in call.keywords}
    if kw not in kws:
        return 'no %s=' % kw + (' (**kw cannot be read)' if None in kws
                                else '')
    v = kws[kw]
    if isinstance(v, ast.Constant) and v.value is None:
        return '%s=None' % kw
    return None


#: The other functions that take a rotation DECLARATION, the keyword they
#: take it by, and the file whose production call must pass it (#1121).
#: `perturb_poses` is the portfolio's `poses` strategy, which turned a
#: declared part because nothing handed it the claims.
_DECLARATION_CALLS = (
    ('_seat_edge', 'rotations', 'py_placer/placement/seeder.py'),
    ('perturb_poses', 'declared', 'py_placer/placement/portfolio.py'),
)


def test_every_declaration_taking_call_passes_it():
    """The standing gate's shape, for every function in `_DECLARATION_CALLS`:
    each production call passes the declaration, and the file that must
    call it does."""
    for callee, kw, home in _DECLARATION_CALLS:
        missing, per_file = [], {}
        for path in _ladder_source_files():
            rel = os.path.relpath(path, ROOT).replace(os.sep, '/')
            tree = ast.parse(io.open(path, encoding='utf-8').read(), path)
            for call in _try_place_calls(tree, callee):
                per_file[rel] = per_file.get(rel, 0) + 1
                why = _ladder_missing(call, kw)
                if why:
                    missing.append('%s:%d (%s)' % (rel, call.lineno, why))
        assert home in per_file, (
            'no `%s` call in %s (%r) -- this gate is not looking at what it '
            'thinks it is' % (callee, home, per_file))
        assert not missing, (
            '`%s` call(s) without %s=: %r. A caller that drops it lets the '
            'function turn a part whose rotation was DECLARED.'
            % (callee, kw, missing))
        print('  every %s call passes %s= (%s)' % (
            callee, kw, ', '.join('%s %d' % kv
                                  for kv in sorted(per_file.items()))))


TESTS.append(test_every_declaration_taking_call_passes_it)


def test_the_ladder_gate_sees_what_slipped_past_it():
    """The control for the gate above, on source it is handed: the #1117
    call shape (an attribute call with no ladder) and a literal None must be
    reported; a call that passes a ladder must not."""
    src = ("seeder._try_place(st, 'U1', 0, 0, set(), tol=0.5)\n"
           "_try_place(st, 'U1', 0, 0, set(), rotations=None)\n"
           "_try_place(st, 'U1', 0, 0, set(), **kw)\n"
           "seeder._try_place(st, 'U1', 0, 0, set(), rotations=lad)\n")
    got = [(c.lineno, _ladder_missing(c))
           for c in _try_place_calls(ast.parse(src))]
    assert [ln for ln, _ in got] == [1, 2, 3, 4], got
    assert got[0][1] == 'no rotations=' and got[1][1] == 'rotations=None', got
    assert got[2][1] and '**kw' in got[2][1] and got[3][1] is None, got
    # #1121: the same gate, for the portfolio's declaration keyword.
    src2 = ("portfolio.perturb_poses(st, free, 0)\n"
            "perturb_poses(st, free, 0, declared=None)\n"
            "perturb_poses(st, free, 0, declared=_declared)\n")
    got2 = [_ladder_missing(c, 'declared')
            for c in _try_place_calls(ast.parse(src2), 'perturb_poses')]
    assert got2 == ['no declared=', 'declared=None', None], got2
    print('  the gate reports %d of 4 shapes and passes the good one'
          % sum(1 for _, w in got if w))


TESTS.append(test_the_ladder_gate_sees_what_slipped_past_it)


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


def _edge_seat_probe(board_name, declared_delta, ladder=None):
    """Drive `_seat_edge` directly on every unlocked connector of a board.

    Returns [(ref, edge, seated?, angle it ended at, angle(s) declared)].

    Direct rather than through `repair_placement` because repair only reaches
    `_seat_edge` for a ref its own violator census picks, and that census is
    the thing most likely to change underneath this test -- a fixture that
    stops reaching the code under test reports PASS for the wrong reason.
    """
    from kicad_parser import parse_kicad_pcb
    import pose_score
    from placement import seeder

    path = os.path.join(ROOT, 'kicad_files', board_name)
    pcb = parse_kicad_pcb(path)
    st = pose_score.make_state(pcb, path)
    out = []
    for ref in sorted(st.parts):
        if not ref.startswith('J') or st.parts[ref].locked:
            continue
        part = st.parts[ref]
        for edge in ('north', 'south', 'east', 'west'):
            x0, y0, rot0 = part.x, part.y, part.rot
            rots = (ladder if ladder is not None
                    else [(rot0 + declared_delta) % 360.0])
            ok = seeder._seat_edge(
                st, ref, {'ref': ref, 'edge': edge, 'class': 'edge_receptacle'},
                set(), [], rotations=rots)
            out.append((ref, edge, ok, st.parts[ref].rot % 360.0,
                        tuple(r % 360.0 for r in rots)))
            st.apply_move(ref, x0, y0, rot0)
    return out


def test_seat_edge_never_ships_an_undeclared_angle():
    """The repair path's `_seat_edge` must not seat at the INPUT angle.

    THE BUG THIS PINS, because it is invisible from the call site: `_seat_edge`
    took a `rotations=` argument and threaded it into the #706 fallback ladder
    -- which is reached only when no seat exists at the part's own angle. In
    the ordinary case `seat = try_rot(part.rot)` succeeded first and the
    function returned True, so the declaration was dropped in silence.
    Measured on splitflap_driver with an angle declared 90deg off the board's:
    17 of 17 connectors seated at the input angle, 0 honoured.

    `_try_place` had it right (the ladder REPLACES the fallback), so before
    this a declared NON-edge part was corrected by repair and a declared EDGE
    part was not -- which is why this asserts on BOTH arms of the same board
    rather than on a count.
    """
    rows = _edge_seat_probe('splitflap_driver.kicad_pcb', 90.0)
    assert len(rows) >= 20, (
        'only %d probe row(s) -- this gate is not looking at what it thinks '
        'it is' % len(rows))
    seated = [r for r in rows if r[2]]
    assert seated, 'no row seated at all; the probe proves nothing'
    bad = [(ref, edge, got, want) for ref, edge, ok, got, want in rows
           if ok and abs((got - want[0]) % 360.0) > 1e-6]
    assert not bad, (
        '%d of %d seated row(s) shipped an UNDECLARED angle, e.g. %r. A seat '
        'at the part\'s incoming rotation is exactly the silent drop #893 '
        'exists to remove.' % (len(bad), len(seated), bad[:3]))
    print('  %d seated row(s), all at the declared angle (%d refused rather '
          'than turned)' % (len(seated), len(rows) - len(seated)))


TESTS.append(test_seat_edge_never_ships_an_undeclared_angle)


def test_seat_edge_keeps_a_declared_angle_it_already_has():
    """The NEGATIVE control: declaring the angle the part already has must
    seat it, unchanged. Without this arm the assertion above is satisfied by
    an implementation that refuses everything."""
    rows = _edge_seat_probe('splitflap_driver.kicad_pcb', 0.0)
    seated = [r for r in rows if r[2]]
    assert len(seated) >= 20, (
        'only %d of %d row(s) seated at the angle the part ALREADY has -- a '
        'declared ladder must not make an in-place seat harder'
        % (len(seated), len(rows)))
    bad = [r for r in seated if abs((r[3] - r[4][0]) % 360.0) > 1e-6]
    assert not bad, bad[:3]
    print('  %d row(s) seated unchanged at their own declared angle'
          % len(seated))


TESTS.append(test_seat_edge_keeps_a_declared_angle_it_already_has)


def test_seat_edge_walks_a_candidate_set_in_author_order():
    """`rotation_candidates` is a SET, and the FIRST that seats wins.

    Order is the assertion: a ladder that sorted, or that fell back to the
    part's own angle, would still land on a legal pose and pass a
    "did it seat" check.
    """
    rows = _edge_seat_probe('splitflap_driver.kicad_pcb', None,
                            ladder=[45.0, 90.0, 0.0])
    seated = [r for r in rows if r[2]]
    assert seated, 'no row seated; the probe proves nothing'
    bad = [r for r in seated if r[3] not in (45.0, 90.0, 0.0)]
    assert not bad, 'seated outside the declared candidate set: %r' % bad[:3]
    first = [r for r in seated if abs(r[3] - 45.0) < 1e-6]
    assert first, (
        'no row took the FIRST candidate (45deg) -- a ladder that reordered '
        'the author\'s set would look like this')
    print('  %d seated row(s) inside the candidate set, %d on the first '
          'candidate' % (len(seated), len(first)))


TESTS.append(test_seat_edge_walks_a_candidate_set_in_author_order)


def test_seat_edge_is_unchanged_without_a_declaration():
    """`rotations=None` must reproduce the pre-#893 `_seat_edge` exactly:
    minimal move, at the part's own angle."""
    from kicad_parser import parse_kicad_pcb
    import pose_score
    from placement import seeder

    path = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
    pcb = parse_kicad_pcb(path)
    st = pose_score.make_state(pcb, path)
    n = 0
    for ref in sorted(st.parts):
        if not ref.startswith('J') or st.parts[ref].locked:
            continue
        part = st.parts[ref]
        x0, y0, rot0 = part.x, part.y, part.rot
        ok = seeder._seat_edge(
            st, ref, {'ref': ref, 'edge': 'south', 'class': 'edge_receptacle'},
            set(), [], rotations=None)
        if ok:
            n += 1
            assert abs((st.parts[ref].rot - rot0) % 360.0) < 1e-6, (
                '%s turned with no declaration: %g -> %g'
                % (ref, rot0, st.parts[ref].rot))
        st.apply_move(ref, x0, y0, rot0)
    assert n >= 5, 'only %d undeclared row(s) seated' % n
    print('  %d undeclared row(s) seated at their own angle, none turned' % n)


TESTS.append(test_seat_edge_is_unchanged_without_a_declaration)


def main():
    failures = 0
    only = sys.argv[1:]
    run = [fn for fn in TESTS
           if not only or any(o in fn.__name__ for o in only)]
    if only and not run:
        # A filter that names no case passes nothing: a mutation battery
        # witness spelled wrong would otherwise read every row as SURVIVED.
        print('NO TEST matches %r' % (only,))
        return 2
    for fn in run:
        print('%s ...' % fn.__name__)
        try:
            fn()
        except AssertionError as exc:
            failures += 1
            print('FAIL %s: %s' % (fn.__name__, exc))
    if failures:
        print('%d/%d checks FAILED' % (failures, len(run)))
        return 1
    print('all %d checks passed' % len(run))
    return 0


if __name__ == '__main__':
    sys.exit(main())
