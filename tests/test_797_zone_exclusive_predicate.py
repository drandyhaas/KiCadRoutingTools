#!/usr/bin/env python3
"""#797: a declared EXCLUSIVE zone reaches the SEAT predicate.

`rule_zone_exclusive` graded a reserved rectangle and the seat search had no
exclusive-zone concept at all (`grep -c exclusive py_placer/placement/seeder.py`
returned 0), so `place_seed` could seat a stranger in the middle of a region the
intent reserved and then exit 4 against the intent it was built from. That is
#701's shape one rule over, and `docs/floorplan-intent.md` said so in its own
"Which rules the SEARCH can see" table.

WHAT THIS FILE TESTS, and what it deliberately cannot.

The seat gate and the QUENCH gate are ONE implementation by construction: both
resolve through `quench.build_zone_spec` (membership, the `z.side` filter, and
the load-bearing `elif` that exempts a member) and both measure through
`quench.intent_term_values`. So there is nothing to test between those two.

The GRADER still has its own copy -- `floorplan.rule_zone_exclusive` -- and
that copy was left alone on purpose: moving its `> EPS` threshold into a shared
hit function would change the floats `intent_term_values` returns, which feed
`intent_ok`'s monotone compare, i.e. a #702 behaviour change shipped inside
#797. The price of that decision is that grade-vs-gate agreement is a claim
about TWO implementations, and it is what the pose lattice below is for. The
lattice is therefore a real detector, not a tautology: it can catch which
membership set is consulted, which side attribute, which rect, whether the zone
is tolerance-inflated, and the threshold direction.

What no agreement test can catch is a bug INSIDE a value both sides compute the
same way. That is what `test_the_EPS_boundary_is_a_strict_gt_on_BOTH_sides`
exists for, and it is the only assertion here that a `>` -> `>=` mutation fails.

Every accepting arm is PAIRED with a refusing one over the same geometry. An
assertion like "the B-side stranger is seated" is satisfied by a gate that is
simply never armed, and then the arm passes in both directions.

No subprocess and no `run_utils`, so this stays in `run_all.py --fast`.
"""
import inspect
import os
import sys
import tempfile

RUN_ALL_FAST_OK = True

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO,):
    if _p not in sys.path:
        sys.path.insert(0, _p)
        sys.path.insert(0, os.path.join(_p, 'py_router'))   # placement split
        sys.path.insert(0, os.path.join(_p, 'py_tools'))    # placement split
        sys.path.insert(0, os.path.join(_p, 'py_placer'))   # placement split

from kicad_parser import parse_kicad_pcb                        # noqa: E402
from placement import floorplan, legality, seeder               # noqa: E402
from placement import quench as q                               # noqa: E402
from placement.quench import QuenchState                        # noqa: E402
import routing_defaults as defaults                             # noqa: E402

# The fixture vocabulary is test_702's, imported rather than copied: a second
# `_part` would be a second definition of what a courtyard is, which is the
# drift this whole area of the tree is organised against.
from test_702_quench_intent_gate import (_part, board,          # noqa: E402
                                         intent_doc, load)

_QKW = dict(clearance=defaults.CLEARANCE, board_edge_clearance=0.55,
            crossing_penalty=10.0, halo_base=0.5, halo_coef=0.25,
            halo_weight=2.0, edge_halo=2.0, edge_weight=2.0,
            grid_step=defaults.GRID_STEP, length_weight=1.0)

#: The name of #797's channel. Probed rather than assumed, so that on an engine
#: that does not have it the arms below fail on their OWN assertions -- "the
#: stranger was seated in the reserved zone" -- instead of dying on a TypeError
#: three frames from anything they claim. A test that dies before it checks
#: anything exits non-zero exactly like a satisfied guard.
CHANNEL = 'exclusive_zones'
HAS_CHANNEL = CHANNEL in inspect.signature(QuenchState.__init__).parameters

SIZE = (40.0, 24.0)


def seat_state(bpath, intent, sources=()):
    """`(state, gate, pcb)` for the state a SEAT search builds.

    Keep-outs (#701) and, since #797, the exclusive zones -- and never
    `intent_zones`, which `tests/test_698_reseat_acceptance.py` arm H forbids
    on this path because a MONOTONE containment gate would make a repair refuse
    its own target.

    Built from the same `resolve_intent_gate` bundle `seed_from_intent` uses,
    so no arm below can pass against zones resolved some other way.
    """
    pcb = parse_kicad_pcb(bpath)
    gate, _probs = floorplan.resolve_intent_gate(intent, pcb, sources)
    kw = dict(_QKW, keepouts=gate['keepouts'])
    if HAS_CHANNEL:
        kw[CHANNEL] = gate['zones']
    return QuenchState(pcb, bpath, **kw), gate, pcb


def refused(st, ref, x, y, rot=0.0):
    """Does THE seat predicate refuse this pose?

    `seeder.pose_ok` itself, never a private helper: `count_legal_poses`'s
    whole contract is that a counted pose is one the search would really have
    taken, and an arm asserting against a second predicate proves nothing about
    the first.
    """
    return not seeder.pose_ok(st, ref, x, y, rot, set())


def graded_exclusive(pcb, bpath, intent, ref=None, sources=()):
    """`rule_zone_exclusive`'s verdict on the poses CURRENTLY in `pcb`.

    In-memory: `floorplan.grade` builds its own QuenchState from `pcb_data`
    (floorplan.py:1608) and reads `pcb_file` only for the outline and the
    locked refs, so moving a footprint and re-grading needs no file write.
    """
    res = floorplan.grade(intent, pcb, bpath, group_sources=sources)
    return [v for v in res.errors
            if v.rule == 'zone_exclusive' and (ref is None or v.ref == ref)]


def terms_for(st, ref):
    """The exclusive terms binding `ref`, or `()` on an engine with no channel."""
    return tuple(getattr(st, 'exclusive_for', {}).get(ref, ()) or ())


# --------------------------------------------------------------------------
# 1. the channel itself: what it carries, and what it must NOT arm
# --------------------------------------------------------------------------

def test_the_channel_exists_and_carries_only_zone_exclusive_terms(wd):
    assert HAS_CHANNEL, (
        f"QuenchState has no `{CHANNEL}` parameter: the seat state has no way "
        f"to learn which rects a stranger must avoid, so #797's conjunct "
        f"cannot exist")
    b = board(os.path.join(wd, 'chan.kicad_pcb'), [
        _part('U1', 6.0, 4.0, 1.0, 1.0),
        _part('M1', 34.0, 20.0, 1.0, 1.0),
    ])
    it = load(intent_doc(blocks=[
        {"name": "rf", "refs": ["M1"], "zone": [16.0, 8.0, 24.0, 16.0],
         "exclusive": True, "tolerance_mm": 0.5}]), wd, 'chan')
    st, _gate, _pcb = seat_state(b, it)

    u = terms_for(st, 'U1')
    assert len(u) == 1, f"the stranger U1 has {len(u)} exclusive term(s), want 1"
    assert u[0].rule == 'zone_exclusive', f"term rule is {u[0].rule!r}"
    assert u[0].name == 'rf', f"term name is {u[0].name!r}, want the BLOCK name"
    assert tuple(u[0].rect) == (16.0, 8.0, 24.0, 16.0), \
        f"term rect {u[0].rect} is not the declared zone"
    assert u[0].threshold == legality.EPS, \
        (f"term threshold {u[0].threshold} is not legality.EPS -- "
         f"rule_zone_exclusive flags `area > legality.EPS`")

    m = terms_for(st, 'M1')
    assert not m, (
        f"the MEMBER M1 carries {len(m)} exclusive term(s): "
        f"`build_zone_spec`'s `elif` is what exempts a block's own members, "
        f"and rule_zone_exclusive skips them too")

    # The load-bearing invariant. Folding these terms into `_intent_spec` would
    # arm `candidate_valid`'s MONOTONE `intent_ok`, which admits any pose
    # termwise no worse than the one the part is in -- and before it is seated
    # that is its generator-default pile coordinate. A stranger whose pile
    # coordinate is already inside the reserved zone would then be admitted to
    # every pose no worse than that, i.e. seated inside it: #797's bug back
    # through the door `pose_ok`'s own docstring names for keep-outs.
    assert st._intent_spec == {}, (
        f"the exclusive channel armed the CONTAINMENT spec: "
        f"{ {k: len(v) for k, v in st._intent_spec.items()} }")
    assert st._intent_active is False, \
        "the exclusive channel armed candidate_valid's monotone intent gate"
    print(f"  PASS: the stranger binds exactly 1 zone_exclusive term named "
          f"'rf' at threshold EPS, the member binds none, and _intent_spec / "
          f"_intent_active are untouched")


# --------------------------------------------------------------------------
# 2. membership -- the pair, over identical geometry
# --------------------------------------------------------------------------

RF = (16.0, 8.0, 24.0, 16.0)
RF_CENTRE = (20.0, 12.0)


def test_a_stranger_is_refused_where_a_member_is_seated(wd):
    """The pair is the point. Asserting only the refusal is satisfied by a gate
    that refuses everybody, and asserting only the member's seat is satisfied
    by a gate that is never armed."""
    b = board(os.path.join(wd, 'mem.kicad_pcb'), [
        _part('U1', 6.0, 4.0, 1.0, 1.0),
        _part('M1', 34.0, 20.0, 1.0, 1.0),
    ])
    it = load(intent_doc(blocks=[
        {"name": "rf", "refs": ["M1"], "zone": list(RF),
         "exclusive": True, "tolerance_mm": 0.5}]), wd, 'mem')
    st, _gate, pcb = seat_state(b, it)

    # Identical geometry, so the ONLY difference between the two verdicts is
    # membership. Without this the pair could be explained by a size.
    x, y = RF_CENTRE
    a_u = legality.rect_overlap_area(st.parts['U1'].rect(x, y, 0.0), RF)
    a_m = legality.rect_overlap_area(st.parts['M1'].rect(x, y, 0.0), RF)
    assert a_u == a_m > 0.0, \
        f"the fixture parts differ in geometry: U1 {a_u}mm2 vs M1 {a_m}mm2"

    assert refused(st, 'U1', x, y), (
        f"the STRANGER U1 was seated at the centre of the exclusive zone "
        f"{RF}, overlapping it by {a_u}mm2 -- the seat search cannot see the "
        f"rule the grade will flag it with")
    assert not refused(st, 'M1', x, y), (
        f"the MEMBER M1 was refused its own block's zone: the `elif` in "
        f"build_zone_spec, or rule_zone_exclusive's member skip, is gone")
    print(f"  PASS: at {RF_CENTRE} both parts overlap the zone by {a_u}mm2; "
          f"the stranger is refused and the member is seated")


# --------------------------------------------------------------------------
# 3. the side filter, and the SCALAR/SET trap
# --------------------------------------------------------------------------

def test_the_side_filter_reads_the_SCALAR_side_like_the_grade_does(wd):
    """`rule_keepout` passes `part.sides` (a through-hole part's leads pierce a
    keep-out from either face). `rule_zone_exclusive` reads the SCALAR
    `part.side` and never `sides`. An implementation that "harmonises" the two
    refuses a through-hole stranger whose BODY is on the far side -- a pose the
    grade calls clean, which is the round trip inverted."""
    b = board(os.path.join(wd, 'side.kicad_pcb'), [
        _part('M1', 34.0, 20.0, 1.0, 1.0),                    # owner, F
        _part('SF', 4.0, 4.0, 1.0, 1.0),                      # stranger, F
        _part('SB', 8.0, 4.0, 1.0, 1.0, layer='B.Cu'),        # stranger, B
        _part('ST', 12.0, 4.0, 1.0, 1.0, layer='B.Cu', thru=True),
    ])
    it = load(intent_doc(blocks=[
        {"name": "rf", "refs": ["M1"], "zone": list(RF), "side": "F",
         "exclusive": True, "tolerance_mm": 0.5}]), wd, 'side')
    st, _gate, pcb = seat_state(b, it)

    # Fixture self-check: without it, "the through-hole stranger is seated"
    # is satisfied by a part that is not through-hole at all.
    assert st.parts['ST'].side == 'B', f"ST side is {st.parts['ST'].side!r}"
    assert st.parts['ST'].sides == frozenset({'F', 'B'}), \
        (f"ST occupies {set(st.parts['ST'].sides)}, so this arm is not "
         f"exercising the scalar-vs-set distinction it names")

    x, y = RF_CENTRE
    assert refused(st, 'SF', x, y), \
        "the F-side stranger was seated in an F-side exclusive zone"
    assert not refused(st, 'SB', x, y), \
        ("the B-side stranger was refused an F-side exclusive zone -- "
         "rule_zone_exclusive skips it, so the grade will call this clean")
    assert not refused(st, 'ST', x, y), \
        ("the THROUGH-HOLE stranger whose BODY is on B was refused an F-side "
         "exclusive zone: the filter is reading `part.sides` where "
         "rule_zone_exclusive reads the scalar `part.side`")

    # And the grade agrees on all three, at the same poses.
    for ref, want in (('SF', 1), ('SB', 0), ('ST', 0)):
        pcb.footprints[ref].x, pcb.footprints[ref].y = x, y
    got = {v.ref for v in graded_exclusive(pcb, b, it)}
    assert got == {'SF'}, \
        f"the grade flags {sorted(got)} at the shared pose, want ['SF']"
    print("  PASS: F-side stranger refused; B-side SMD and B-side THROUGH-HOLE "
          "strangers both seated, and the grade flags exactly SF")


# --------------------------------------------------------------------------
# 4. the boundary -- the one assertion a `>` -> `>=` mutation fails
# --------------------------------------------------------------------------

def test_the_EPS_boundary_is_a_strict_gt_on_BOTH_sides(wd):
    """An overlap of EXACTLY `legality.EPS` is the only input on which `> EPS`
    and `>= EPS` differ, and therefore the only assertion that pins which side
    of it each front falls on.

    The construction is exact in binary: the overlap is [0, EPS] x [0, 1], so
    the width subtraction is `EPS - 0.0` and the area is `EPS * 1.0`.

    This arm pins behaviour that EXISTS before #797 -- it is the mutation
    detector the agreement sweep cannot be, because a bug inside a value both
    fronts compute the same way is invisible to any agreement test.
    """
    eps = legality.EPS
    unit = (0.0, 0.0, 1.0, 1.0)
    exact = (-5.0, 0.0, eps, 1.0)
    assert legality.rect_overlap_area(unit, exact) == eps, \
        "the exactly-EPS fixture is not exact on this platform"

    t = q._IntentTerm('zone_exclusive', 'z', exact, eps, False, None)
    v = q.intent_term_values((t,), (unit, None))[0]
    assert v == eps, f"the gate measures {v!r} where the grade measures {eps!r}"
    # The GATE calls a term clean at `v <= threshold`; the GRADE flags at
    # `area > EPS`. At exactly EPS both must say CLEAN.
    assert v <= t.threshold, \
        "the gate refuses an overlap of exactly EPS; the grade does not flag it"
    assert not (v > eps), \
        "the grade flags an overlap of exactly EPS; the gate calls it clean"

    over = (-5.0, 0.0, 2.0 * eps, 1.0)
    v2 = q.intent_term_values(
        (q._IntentTerm('zone_exclusive', 'z', over, eps, False, None),),
        (unit, None))[0]
    assert v2 > eps and not (v2 <= eps), \
        f"an overlap of 2*EPS ({v2!r}) is not over the threshold on both sides"

    # Zero-area contact -- a corner touch and an edge touch -- is CLEAN.
    for tag, rect in (('corner', (1.0, 1.0, 2.0, 2.0)),
                      ('edge', (1.0, 0.0, 2.0, 1.0))):
        z = q.intent_term_values(
            (q._IntentTerm('zone_exclusive', 'z', rect, eps, False, None),),
            (unit, None))[0]
        assert z == 0.0, f"a {tag} touch measures {z!r}, want 0.0"
    print(f"  PASS: an overlap of exactly EPS ({eps}) is CLEAN on both fronts, "
          f"2*EPS is dirty on both, and corner/edge touches measure 0.0")


# --------------------------------------------------------------------------
# 5. COURTYARD ONLY -- the inverse of the keep-out rule
# --------------------------------------------------------------------------

_THT_BOARD = os.path.join(REPO, 'kicad_files', 'ulx3s.kicad_pcb')
_THT_REF = 'LCD1'


def test_the_measurement_is_courtyard_only_unlike_a_keepout(wd):
    """`rule_zone_exclusive` reads `part.rect` and never `tht_rect`, so a
    stranger whose LEADS cross a reserved zone while its body does not is a
    pose the grade calls clean. A seat gate passing both rects would refuse it
    -- the round trip broken in the direction nobody looks, because the tool
    would be stricter than its own grade and strand a part for nothing.

    Same geometry, opposite answers, in one arm: the identical sliver declared
    as a KEEP-OUT must refuse, because rule_keepout does grade both rects.
    """
    if not os.path.exists(_THT_BOARD):
        print(f"  SKIP: {_THT_BOARD} not present")
        return
    pcb = parse_kicad_pcb(_THT_BOARD)
    st0 = QuenchState(pcb, _THT_BOARD, **_QKW)
    p = st0.parts[_THT_REF]
    body, tht = p.rects()
    gaps = {'w': body[0] - tht[0], 'e': tht[2] - body[2],
            'n': body[1] - tht[1], 's': tht[3] - body[3]}
    side = max(gaps, key=gaps.get)
    assert gaps[side] > 1e-6, \
        f"{_THT_REF}'s lead rect no longer exceeds its courtyard: {gaps}"
    m = gaps[side] / 3.0
    sliver = {'w': (tht[0] + m, tht[1], body[0] - m, tht[3]),
              'e': (body[2] + m, tht[1], tht[2] - m, tht[3]),
              'n': (tht[0], tht[1] + m, tht[2], body[1] - m),
              's': (tht[0], body[3] + m, tht[2], tht[3] - m)}[side]
    assert legality.rect_overlap_area(body, sliver) <= legality.EPS, \
        "the fixture sliver touches the courtyard; it must hit the leads only"
    assert legality.rect_overlap_area(tht, sliver) > legality.EPS, \
        "the fixture sliver does not hit the lead rect either"

    owner = sorted(r for r in st0.parts if r != _THT_REF)[0]
    doc = {"schema": 1, "kind": "floorplan-intent", "units": "mm",
           "blocks": [{"name": "sliver", "refs": [owner],
                       "zone": [round(v, 6) for v in sliver],
                       "exclusive": True, "tolerance_mm": 0.5}]}
    it = load(doc, wd, 'tht')
    gate, _probs = floorplan.resolve_intent_gate(it, pcb, ())
    kw = dict(_QKW, keepouts=gate['keepouts'])
    if HAS_CHANNEL:
        kw[CHANNEL] = gate['zones']
    st = QuenchState(pcb, _THT_BOARD, **kw)
    assert terms_for(st, _THT_REF), \
        f"{_THT_REF} is not bound by the sliver zone; the arm tests nothing"

    assert not refused(st, _THT_REF, p.x, p.y, p.rot), (
        f"{_THT_REF} was REFUSED a pose whose only intrusion is its LEADS -- "
        f"the seat gate is passing both rects where rule_zone_exclusive reads "
        f"the courtyard alone, so the tool is stricter than its own grade")
    assert not graded_exclusive(pcb, _THT_BOARD, it, ref=_THT_REF), \
        f"the grade flags {_THT_REF} on a lead-only intrusion; the premise moved"

    # The control, same geometry: as a KEEP-OUT the identical sliver refuses.
    ko = {'name': 'lead-sliver', 'sides': ('F', 'B'), 'allow': (),
          'rect': sliver}
    st_ko = QuenchState(pcb, _THT_BOARD, keepouts=[ko], **_QKW)
    assert refused(st_ko, _THT_REF, p.x, p.y, p.rot), \
        ("the identical sliver as a KEEP-OUT did not refuse, so this arm's "
         "'opposite answers' claim is untested")
    print(f"  PASS: {_THT_REF}'s leads are {gaps[side]:.4f}mm proud on the "
          f"{side} side; that sliver as an exclusive zone SEATS it (as the "
          f"grade does) and as a keep-out REFUSES it")


# --------------------------------------------------------------------------
# 6. the inert cases -- skipped, not passing
# --------------------------------------------------------------------------

def test_a_zone_with_no_rect_and_exclusive_false_arm_nothing(wd):
    b = board(os.path.join(wd, 'inert.kicad_pcb'), [
        _part('U1', 6.0, 4.0, 1.0, 1.0),
        _part('M1', 34.0, 20.0, 1.0, 1.0),
    ])
    x, y = RF_CENTRE

    # (a) exclusive, but no rect: the rule is SKIPPED, not passed.
    it_a = load(intent_doc(blocks=[
        {"name": "rf", "refs": ["M1"], "exclusive": True}]), wd, 'inert_a')
    st_a, _g, _p = seat_state(b, it_a)
    assert not terms_for(st_a, 'U1'), "a zone with no rect bound a stranger"
    assert not refused(st_a, 'U1', x, y), "a zone with no rect refused a pose"
    res_a = floorplan.grade(it_a, parse_kicad_pcb(b), b)
    assert 'zone_exclusive' in res_a.rules_skipped, (
        f"rule_zone_exclusive RAN on a block with no rect: it must be SKIPPED "
        f"with a stated reason, never silently pass -- "
        f"rules_run={res_a.rules_run}")

    # (b) a rect, flag off -- paired with the flag ON over the same geometry,
    # or "nothing was refused" is satisfied by a gate that never fires.
    it_b = load(intent_doc(blocks=[
        {"name": "rf", "refs": ["M1"], "zone": list(RF),
         "exclusive": False, "tolerance_mm": 0.5}]), wd, 'inert_b')
    st_b, _g, _p = seat_state(b, it_b)
    assert not terms_for(st_b, 'U1'), "`exclusive: false` still bound a stranger"
    assert not refused(st_b, 'U1', x, y), "`exclusive: false` refused a pose"

    it_c = load(intent_doc(blocks=[
        {"name": "rf", "refs": ["M1"], "zone": list(RF),
         "exclusive": True, "tolerance_mm": 0.5}]), wd, 'inert_c')
    st_c, _g, _p = seat_state(b, it_c)
    assert refused(st_c, 'U1', x, y), \
        ("the same zone with `exclusive: true` did not refuse either, so the "
         "two negatives above are vacuous")
    print("  PASS: a rect-less block arms nothing and its rule is SKIPPED "
          "with a reason; `exclusive: false` binds nothing, and the same zone "
          "flagged true does refuse")


# --------------------------------------------------------------------------
# 7. several zones -- one term each, never one aggregate
# --------------------------------------------------------------------------

RF2 = (4.0, 16.0, 12.0, 22.0)


def test_two_zones_give_two_named_terms_never_one_aggregate(wd):
    """`_IntentTerm` is one term per (ref, ENTRY), never per (ref, rule): an
    aggregate lets a part buy its way into zone B by leaving zone A, and it
    leaves the verdict unable to name WHICH zone refuses."""
    b = board(os.path.join(wd, 'two.kicad_pcb'), [
        _part('U1', 30.0, 4.0, 1.0, 1.0),
        _part('M1', 20.0, 12.0, 0.5, 0.5),
        _part('M2', 8.0, 19.0, 0.5, 0.5),
    ])
    it = load(intent_doc(blocks=[
        {"name": "rf", "refs": ["M1"], "zone": list(RF),
         "exclusive": True, "tolerance_mm": 0.5},
        {"name": "ant", "refs": ["M2"], "zone": list(RF2),
         "exclusive": True, "tolerance_mm": 0.5}]), wd, 'two')
    st, _gate, _pcb = seat_state(b, it)

    names = sorted(t.name for t in terms_for(st, 'U1'))
    assert names == ['ant', 'rf'], \
        f"U1 binds {names}, want one NAMED term per declared zone"
    assert refused(st, 'U1', *RF_CENTRE), "the rf zone did not refuse"
    assert refused(st, 'U1', 8.0, 19.0), "the ant zone did not refuse"
    assert not refused(st, 'U1', 30.0, 4.0), \
        "a pose outside BOTH zones was refused; something else is refusing"
    print(f"  PASS: the stranger binds two separately named terms {names}, is "
          f"refused inside each, and seated outside both")


# --------------------------------------------------------------------------
# 8. the pose lattice -- gate vs GRADE, two implementations, every pose
# --------------------------------------------------------------------------

LAT_CENTRE = (10.0, 12.0)
LAT_ZONE = (7.0, 9.0, 13.0, 15.0)


def _lattice_case(wd, tag, part_kw, block):
    """`(clean, dirty, disagreements)` over the 13x9 integer-mm lattice.

    Integer millimetres are load-bearing. With a half-extent of 1.0 against a
    zone spanning x in [7, 13], x = 6 is a ZERO-AREA touch (which must read
    CLEAN, since both fronts threshold at `> EPS`) and x = 7 is an area of 1.0
    (which must read DIRTY). The sweep straddles both boundaries exactly, on
    both axes, which a fractional step would miss.
    """
    b = board(os.path.join(wd, f'lat_{tag}.kicad_pcb'), [
        _part('U1', LAT_CENTRE[0], LAT_CENTRE[1], 1.0, 1.0, **part_kw),
        _part('M1', 34.0, 20.0, 0.5, 0.5),
    ])
    it = load(intent_doc(blocks=block), wd, f'lat_{tag}')
    st, _gate, pcb = seat_state(b, it)
    p = st.parts['U1']
    fp = pcb.footprints['U1']
    clean = dirty = 0
    disagree = []
    for dx in range(-6, 7):
        for dy in range(-4, 5):
            x, y = LAT_CENTRE[0] + dx, LAT_CENTRE[1] + dy
            spec = terms_for(st, 'U1')
            vals = q.intent_term_values(spec, p.rects(x, y, 0.0)) if spec else ()
            gate_clean = all(v <= t.threshold for v, t in zip(vals, spec))
            fp.x, fp.y = x, y
            grade_clean = not graded_exclusive(pcb, b, it, ref='U1')
            if gate_clean:
                clean += 1
            else:
                dirty += 1
            if gate_clean != grade_clean:
                disagree.append((x, y, gate_clean, grade_clean))
    return clean, dirty, disagree


def test_gate_and_grade_agree_over_a_pose_lattice(wd):
    stranger = [{"name": "rf", "refs": ["M1"], "zone": list(LAT_ZONE),
                 "exclusive": True, "tolerance_mm": 0.5}]
    member = [{"name": "rf", "refs": ["M1", "U1"], "zone": list(LAT_ZONE),
               "exclusive": True, "tolerance_mm": 0.5}]
    sided = [{"name": "rf", "refs": ["M1"], "zone": list(LAT_ZONE),
              "side": "F", "exclusive": True, "tolerance_mm": 0.5}]
    off = [{"name": "rf", "refs": ["M1"], "zone": list(LAT_ZONE),
            "exclusive": False, "tolerance_mm": 0.5}]
    two = stranger + [{"name": "ant", "refs": ["M1"], "zone": [3.0, 3.0,
                                                              6.0, 21.0],
                       "exclusive": True, "tolerance_mm": 0.5}]

    cases = (
        ('stranger-unsided', {}, stranger, 'mixed'),
        ('member', {}, member, 'all-clean'),
        ('sided-F-stranger-on-B', {'layer': 'B.Cu'}, sided, 'all-clean'),
        ('sided-F-stranger-on-B-THT', {'layer': 'B.Cu', 'thru': True},
         sided, 'all-clean'),
        ('offset-courtyard', {'cy_off': (1.5, 0.0)}, stranger, 'mixed'),
        ('exclusive-false', {}, off, 'all-clean'),
        ('two-zones', {}, two, 'mixed'),
    )
    rows = {}
    problems = []
    for tag, part_kw, block, shape in cases:
        clean, dirty, disagree = _lattice_case(wd, tag, part_kw, block)
        rows[tag] = (clean, dirty, len(disagree))
        print(f"      {tag:<28} {clean:>3} clean / {dirty:>3} dirty, "
              f"{len(disagree)} disagreement(s)")
        if disagree:
            problems.append(f"{tag}: {len(disagree)} disagreement(s), "
                            f"first {disagree[:4]}")
        if shape == 'all-clean' and dirty:
            problems.append(f"{tag}: {dirty} pose(s) refused where the grade "
                            f"is silent")
        if shape == 'mixed' and not dirty:
            problems.append(f"{tag}: the gate refused NOTHING over 117 poses")

    # ANTI-VACUITY, as an assertion rather than a comment. Every `all-clean`
    # case above is satisfied by a gate that is simply never armed; what
    # separates "correctly silent" from "inert" is that the SAME lattice with a
    # stranger produces plenty of refusals.
    base_dirty = rows['stranger-unsided'][1]
    assert base_dirty >= 20, (
        f"the stranger case refused only {base_dirty} of 117 poses -- the "
        f"lattice barely enters the zone, so every all-clean case above is "
        f"vacuous")
    assert not problems, "; ".join(problems)
    print(f"  PASS: 7 cases x 117 poses; the gate and rule_zone_exclusive "
          f"agree on every one, with {base_dirty} refusals in the stranger "
          f"case")


TESTS = [
    test_the_channel_exists_and_carries_only_zone_exclusive_terms,
    test_a_stranger_is_refused_where_a_member_is_seated,
    test_the_side_filter_reads_the_SCALAR_side_like_the_grade_does,
    test_the_EPS_boundary_is_a_strict_gt_on_BOTH_sides,
    test_the_measurement_is_courtyard_only_unlike_a_keepout,
    test_a_zone_with_no_rect_and_exclusive_false_arm_nothing,
    test_two_zones_give_two_named_terms_never_one_aggregate,
    test_gate_and_grade_agree_over_a_pose_lattice,
]


def main():
    """Runs EVERY test and reports each, rather than aborting on the first.

    A BROKEN TEST -- one that dies before it checks anything -- is reported
    apart from a failed assertion, because the two produce the same non-zero
    exit and must never read alike (`tests/run_utils.py` makes the same
    distinction for subprocess arms).
    """
    fails, broken = [], []
    with tempfile.TemporaryDirectory() as wd:
        for t in TESTS:
            print(f"--- {t.__name__}")
            try:
                t(wd)
            except AssertionError as exc:
                msg = (str(exc).strip().splitlines() or [''])[0][:220]
                fails.append((t.__name__, msg))
                print(f"  FAIL: {msg}")
            except Exception as exc:                        # noqa: BLE001
                msg = f"{type(exc).__name__}: {exc}"
                broken.append((t.__name__, msg))
                print(f"  BROKEN TEST, not a satisfied guard: {msg}")
    print(f"\n{len(TESTS) - len(fails) - len(broken)} passed, "
          f"{len(fails)} failed, {len(broken)} broken")
    for n, m in fails:
        print(f"  FAILED  {n}: {m}")
    for n, m in broken:
        print(f"  BROKEN  {n}: {m}")
    return 1 if (fails or broken) else 0


if __name__ == '__main__':
    sys.exit(main())
