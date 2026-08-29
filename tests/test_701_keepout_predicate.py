#!/usr/bin/env python3
"""#701: the seat predicate and the grader share ONE keep-out implementation.

`rule_keepout` graded a declared keep-out and the seat search had no keep-out
concept at all (`grep -c "forbid\\|keepout" py_placer/placement/seeder.py`
returned 0), so a keep-out was a region `place_seed` walked parts into and
`check_floorplan` complained about forever. The fix makes both fronts call
`floorplan.keepouts_for_ref` and `floorplan.keepout_hit`.

That claim -- ONE implementation, not two agreeing ones -- is what this file
tests, and it is not testable by reading. Four ways, weakest to strongest:

  * IDENTITY, behaviourally: monkeypatch `floorplan.keepout_hit` and show the
    seat predicate observes the patch. A copy-pasted geometry in `seeder.py`
    would not, and would still pass every agreement check written against
    equivalent-but-separate code.
  * AGREEMENT: sweep keep-outs x parts and require the seat verdict and the
    GRADE's verdict to be identical on every combination, with the hit/miss
    split printed -- a sweep that only ever asked about clear poses proves
    nothing.
  * BOUNDARIES: the cases where two independent implementations drift first
    (an exact corner touch, an overlap at +/-EPS, a tangent circle).
  * A STATIC GATE on the seat-function call sites, because "enforced on one
    path" is invisible to any behavioural test that does not happen to
    exercise the missed path.

No subprocess and no `run_utils`, so this stays in `run_all.py --fast`.
"""
import ast
import io
import os
import sys

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
from placement.quench import QuenchState                        # noqa: E402
import routing_defaults as defaults                             # noqa: E402

BOARD = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')

_QKW = dict(clearance=defaults.CLEARANCE, board_edge_clearance=0.55,
            crossing_penalty=10.0, halo_base=0.5, halo_coef=0.25,
            halo_weight=2.0, edge_halo=2.0, edge_weight=2.0,
            grid_step=defaults.GRID_STEP, length_weight=1.0)


def _state(pcb, keepouts=None):
    return QuenchState(pcb, BOARD, keepouts=keepouts, **_QKW)


def _entry(**over):
    """A keep-out entry in the NORMALIZED form `intent_from_dict` produces."""
    e = {'name': 'k', 'sides': ('F', 'B'), 'allow': ()}
    e.update(over)
    return e


# --------------------------------------------------------------------------
# 1. identity -- the seat predicate calls THE function, not a copy of it
# --------------------------------------------------------------------------

def test_the_seat_predicate_calls_the_graders_own_hit_test():
    """Behavioural, not structural: a `grep` for the name would be satisfied
    by a shadowing local, and an agreement sweep would be satisfied by a
    second implementation that happens to agree today. Patching the object
    and watching the seat predicate change its answer is the only check that
    distinguishes ONE implementation from TWO."""
    pcb = parse_kicad_pcb(BOARD)
    st = _state(pcb)
    ref = sorted(st.parts)[0]
    p = st.parts[ref]
    r = p.rect()
    # A keep-out squarely over the part: refused while the real test runs.
    st_ko = _state(pcb, [_entry(rect=(r[0] - 1, r[1] - 1, r[2] + 1, r[3] + 1))])
    assert not seeder.pose_ok(st_ko, ref, p.x, p.y, p.rot, set()), \
        f"{ref} was seated inside a keep-out covering its whole courtyard"

    real = floorplan.keepout_hit
    seen = []
    try:
        floorplan.keepout_hit = lambda e, rects: (seen.append(e), 0.0)[1]
        assert seeder.pose_ok(st_ko, ref, p.x, p.y, p.rot, set()), \
            ("pose_ok did not observe the patched floorplan.keepout_hit -- "
             "it is calling a SECOND implementation, which is the drift this "
             "whole change exists to prevent")
    finally:
        floorplan.keepout_hit = real
    assert seen, "the patched hit test was never called"
    print(f"  PASS: pose_ok observes floorplan.keepout_hit "
          f"({len(seen)} call(s)); it is the same object the grader uses")


def test_the_edge_seat_calls_it_too():
    """`edge_seat_ok` deliberately bypasses `pose_ok` (an edge connector
    overhangs by design, so full containment is the wrong predicate), so it
    is a SECOND place the shared test has to be reached from. Both edge
    paths -- `_seat_edge`'s `on_board` and stage 1 of `seed_from_intent`,
    which runs no legality gate at all by design -- come through here."""
    pcb = parse_kicad_pcb(BOARD)
    st = _state(pcb)
    ref = sorted(st.parts)[0]
    p = st.parts[ref]
    r = p.rect()
    st_ko = _state(pcb, [_entry(rect=(r[0] - 1, r[1] - 1, r[2] + 1, r[3] + 1))])
    # A band wide enough that the overhang conjunct cannot be what refuses.
    seen = []
    real = floorplan.keepout_hit
    try:
        floorplan.keepout_hit = lambda e, rects: (seen.append(e), 0.0)[1]
        seeder.edge_seat_ok(st_ko, p, p.x, p.y, 'north', 0.0, 1000.0)
    finally:
        floorplan.keepout_hit = real
    assert seen, ("edge_seat_ok never called floorplan.keepout_hit -- the "
                  "edge path is unguarded, and no pose_ok test can see it")
    print(f"  PASS: edge_seat_ok observes floorplan.keepout_hit "
          f"({len(seen)} call(s))")


def test_the_edge_seat_names_the_keepout_that_refused_it():
    """A refusal that says only 'no conflict-free seat on the north edge'
    sends the reader to look at the outline and the neighbours, which are not
    the problem. The `reasons` out-param is what lets `_seat_edge` and stage 1
    say which keep-out it was."""
    pcb = parse_kicad_pcb(BOARD)
    st = _state(pcb)
    ref = sorted(st.parts)[0]
    p = st.parts[ref]
    r = p.rect()
    st_ko = _state(pcb, [_entry(name='enclosure-rib',
                                rect=(r[0] - 1, r[1] - 1, r[2] + 1, r[3] + 1))])
    why = []
    ok = seeder.edge_seat_ok(st_ko, p, p.x, p.y, 'north', 0.0, 1000.0,
                             reasons=why)
    assert not ok, "the keep-out did not refuse the edge seat"
    assert any('enclosure-rib' in w for w in why), \
        f"the refusal did not name the keep-out: {why}"
    print(f"  PASS: the edge refusal names the keep-out -- {why}")


# --------------------------------------------------------------------------
# 2. agreement -- the seat verdict and the GRADE verdict, over a sweep
# --------------------------------------------------------------------------

def _sweep_entries(st):
    """Keep-outs chosen to produce BOTH hits and misses, over rect/circle,
    every `sides` spelling, and `allow` absent/exact/glob/universal."""
    refs = sorted(st.parts)
    a, b, c = refs[0], refs[len(refs) // 2], refs[-1]
    ra, rb, rc = st.parts[a].rect(), st.parts[b].rect(), st.parts[c].rect()
    board = st.board
    out = [
        _entry(name='over-a', rect=(ra[0] - .5, ra[1] - .5, ra[2] + .5, ra[3] + .5)),
        _entry(name='over-b-F', sides=('F',),
               rect=(rb[0] - .5, rb[1] - .5, rb[2] + .5, rb[3] + .5)),
        _entry(name='over-b-B', sides=('B',),
               rect=(rb[0] - .5, rb[1] - .5, rb[2] + .5, rb[3] + .5)),
        # `sides` absent entirely: the loader defaults it, and a consumer that
        # re-derives the default differently is what this catches.
        {'name': 'over-c-nosides', 'allow': (),
         'rect': (rc[0] - .5, rc[1] - .5, rc[2] + .5, rc[3] + .5)},
        _entry(name='over-a-allow-exact',
               rect=(ra[0] - .5, ra[1] - .5, ra[2] + .5, ra[3] + .5),
               allow=(a,)),
        _entry(name='over-all-allow-glob',
               rect=(board[0] - 1, board[1] - 1, board[2] + 1, board[3] + 1),
               allow=('C*',)),
        _entry(name='over-all-allow-none',
               rect=(board[0] - 1, board[1] - 1, board[2] + 1, board[3] + 1),
               allow=('ZZZ_NO_SUCH_REF*',)),
        _entry(name='over-all-allow-star',
               rect=(board[0] - 1, board[1] - 1, board[2] + 1, board[3] + 1),
               allow=('*',)),
        _entry(name='circle-on-a',
               circle=[(ra[0] + ra[2]) / 2.0, (ra[1] + ra[3]) / 2.0, 2.0]),
        _entry(name='circle-far', circle=[board[0] - 50.0, board[1] - 50.0, 1.0]),
        # A whole-board circle: every part hit, so `allow`/`sides` are the
        # only thing that can differ between the two fronts.
        _entry(name='circle-all',
               circle=[(board[0] + board[2]) / 2.0, (board[1] + board[3]) / 2.0,
                       1e4], sides=('F',)),
    ]
    return out


def _graded_keepout_refs(pcb, entries):
    """The refs `rule_keepout` flags, via the real `grade`."""
    raw = floorplan.emit_intent(pcb, BOARD)
    raw['keepouts'] = [
        {k: (list(v) if isinstance(v, tuple) else v) for k, v in e.items()}
        for e in entries]
    intent = floorplan.intent_from_dict(raw, BOARD)
    res = floorplan.grade(intent, pcb, BOARD)
    return {(v.ref, v.measured['keepout']) for v in res.violations
            if v.rule == 'keepout'}


def test_the_seat_verdict_and_the_grade_verdict_agree_on_every_combination():
    pcb = parse_kicad_pcb(BOARD)
    st = _state(pcb)
    entries = _sweep_entries(st)
    graded = _graded_keepout_refs(pcb, entries)

    st_ko = _state(pcb, entries)
    hits = misses = 0
    disagree = []
    for ref in sorted(st_ko.parts):
        p = st_ko.parts[ref]
        rects = p.rects()
        for e in st_ko.keepouts_for.get(ref, ()):
            seat_hit = bool(floorplan.keepout_hit(e, rects))
            grade_hit = (ref, e['name']) in graded
            if seat_hit:
                hits += 1
            else:
                misses += 1
            if seat_hit != grade_hit:
                disagree.append((ref, e['name'], seat_hit, grade_hit))
        # A keep-out the RESOLVER excluded must also be absent from the grade:
        # an `allow` the seat honours and the grade ignores would strand a
        # part that the grade calls clean.
        bound = {e['name'] for e in st_ko.keepouts_for.get(ref, ())}
        for e in entries:
            if e['name'] in bound:
                continue
            misses += 1
            if (ref, e['name']) in graded:
                disagree.append((ref, e['name'], False, True))

    assert not disagree, ("seat/grade disagreement on "
                          f"{len(disagree)}: {disagree[:8]}")
    # A sweep that produced no hits proves nothing; so does one with no
    # misses. Both bounds, not just a total.
    assert hits >= 20, f"only {hits} hit(s) -- the sweep is vacuous"
    assert misses >= 20, f"only {misses} miss(es) -- the sweep is vacuous"
    print(f"  PASS: {hits + misses} (part, keep-out) combination(s) agree "
          f"exactly -- {hits} hit, {misses} miss, 0 disagreement")


# --------------------------------------------------------------------------
# 3. boundaries -- where two implementations drift first
# --------------------------------------------------------------------------

def test_the_hit_test_boundaries():
    r = (0.0, 0.0, 10.0, 10.0)
    eps = legality.EPS

    # Corner touch: overlap area exactly 0. Clear, and NOT a hit -- the
    # threshold lives inside keepout_hit precisely so both fronts agree here.
    assert floorplan.keepout_hit(
        _entry(rect=(10.0, 10.0, 20.0, 20.0)), (r,)) == 0.0
    # Edge touch, zero-area: same answer.
    assert floorplan.keepout_hit(
        _entry(rect=(10.0, 0.0, 20.0, 10.0)), (r,)) == 0.0
    # An overlap a hair under EPS is clear; a hair over is a hit. 10mm wide
    # strip x d deep => area 10*d, so d = eps/100 is well under and
    # d = eps is well over.
    under = floorplan.keepout_hit(
        _entry(rect=(10.0 - eps / 100.0, 0.0, 20.0, 10.0)), (r,))
    over = floorplan.keepout_hit(
        _entry(rect=(10.0 - eps, 0.0, 20.0, 10.0)), (r,))
    assert under == 0.0, f"sub-EPS overlap reported as a hit: {under}"
    assert over > 0.0, f"supra-EPS overlap reported as clear: {over}"

    # EXACTLY EPS, which is the only input on which `> EPS` and `>= EPS`
    # differ -- and therefore the only assertion that pins which side of the
    # boundary the SHARED threshold falls on. Without it a mutation from `>`
    # to `>=` survives the whole file, which is the "passes in both
    # directions" failure this repo names by hand. The construction is exact
    # in binary: the overlap is [0, EPS] x [0, 1], so the width subtraction
    # is `EPS - 0.0` and the area is `EPS * 1.0`.
    unit = (0.0, 0.0, 1.0, 1.0)
    assert legality.rect_overlap_area(unit, (-5.0, 0.0, eps, 1.0)) == eps, \
        "the exactly-EPS fixture is not exact on this platform"
    assert floorplan.keepout_hit(
        _entry(rect=(-5.0, 0.0, eps, 1.0)), (unit,)) == 0.0, \
        ("an overlap of exactly EPS must be CLEAR (the threshold is a strict "
         "`> EPS`); if this fires, the seat predicate and rule_keepout have "
         "moved apart at the boundary")

    # Circle exactly tangent to the rect: `_circle_hits_rect` uses a strict
    # `<`, so tangency is clear. Just inside is a hit.
    assert floorplan.keepout_hit(_entry(circle=[15.0, 5.0, 5.0]), (r,)) == 0.0
    assert floorplan.keepout_hit(
        _entry(circle=[15.0, 5.0, 5.0001]), (r,)) == 1.0

    # A None rect is skipped, not crashed on -- quench._Part.rects() returns
    # (courtyard, None) for the majority of parts.
    assert floorplan.keepout_hit(_entry(rect=(0, 0, 1, 1)), (None,)) == 0.0
    assert floorplan.keepout_hit(_entry(rect=(0, 0, 1, 1)), (None, r)) > 0.0

    # The circle branch returns a MARKER, never a fabricated area.
    assert floorplan.keepout_hit(_entry(circle=[5.0, 5.0, 1.0]), (r,)) == 1.0
    print("  PASS: corner touch, +/-EPS, tangent circle, None rect, "
          "circle marker")


def test_the_resolver_boundaries():
    # `allow` is an fnmatch, not an equality test.
    assert not floorplan.keepouts_for_ref(
        (_entry(allow=('MH*',)),), 'MH1', ('F',))
    assert floorplan.keepouts_for_ref(
        (_entry(allow=('MH*',)),), 'MH', ('F',)) == ()
    assert floorplan.keepouts_for_ref(
        (_entry(allow=('MH1',)),), 'MH12', ('F',))
    # A part shares no face with the keep-out => not bound.
    assert not floorplan.keepouts_for_ref(
        (_entry(sides=('B',)),), 'U1', ('F',))
    # A through-hole part occupies both faces, so a one-sided keep-out binds.
    assert floorplan.keepouts_for_ref(
        (_entry(sides=('B',)),), 'U1', ('F', 'B'))
    # `sides` absent or empty means BOTH -- the loader's default, re-derived
    # in exactly one place.
    assert floorplan.keepouts_for_ref(({'name': 'k'},), 'U1', ('B',))
    assert floorplan.keepouts_for_ref(({'name': 'k', 'sides': None},),
                                      'U1', ('B',))
    print("  PASS: allow globs, side intersection, the sides default")


# --------------------------------------------------------------------------
# 4. the grader must NOT inherit the seat gate
# --------------------------------------------------------------------------

#: A tracked board carrying a part whose THROUGH-HOLE rect sticks out of its
#: own courtyard, which is what makes the tht term independently observable.
#: On most footprints tht_rect is a subset of the courtyard, so a keep-out
#: that hits the leads hits the body too and a seat test cannot tell whether
#: the second rect was passed at all. Measured over the tracked corpus, only
#: ulx3s (LCD1, 1.6436mm proud) and glasgow_revC (SW1, 0.35mm) have one.
_THT_BOARD = os.path.join(REPO, 'kicad_files', 'ulx3s.kicad_pcb')
_THT_REF = 'LCD1'


def test_the_seat_predicate_sees_the_through_hole_rect_not_just_the_body():
    """`rule_keepout` grades the courtyard AND the drilled-pad rect, because
    a through-hole part's leads pass through a keep-out even when its body
    sits on the far side. If the seat predicate looked at the courtyard only,
    it would SEAT a part the grade then flags -- exit 4 on a board the seeder
    placed correctly, which is the round-trip break this change exists to
    close, one level down."""
    if not os.path.exists(_THT_BOARD):
        print(f"  SKIP: {_THT_BOARD} not present")
        return
    pcb = parse_kicad_pcb(_THT_BOARD)
    st = QuenchState(pcb, _THT_BOARD, **_QKW)
    p = st.parts[_THT_REF]
    body, tht = p.rects()
    # The sliver of the lead rect that lies OUTSIDE the courtyard, on
    # whichever side is proudest. A keep-out here hits `tht` and misses
    # `body`, so it is refused only by a predicate that passes both.
    gaps = {'w': body[0] - tht[0], 'e': tht[2] - body[2],
            'n': body[1] - tht[1], 's': tht[3] - body[3]}
    side = max(gaps, key=gaps.get)
    assert gaps[side] > 1e-6, \
        f"{_THT_REF}'s lead rect no longer exceeds its courtyard: {gaps}"
    m = gaps[side] / 3.0
    ko = {'w': (tht[0] + m, tht[1], body[0] - m, tht[3]),
          'e': (body[2] + m, tht[1], tht[2] - m, tht[3]),
          'n': (tht[0], tht[1] + m, tht[2], body[1] - m),
          's': (tht[0], body[3] + m, tht[2], tht[3] - m)}[side]
    e = _entry(name='lead-sliver', rect=ko)
    assert floorplan.keepout_hit(e, (body,)) == 0.0, \
        "the fixture rect touches the courtyard; it must hit the leads only"
    assert floorplan.keepout_hit(e, (body, tht)) > 0.0, \
        "the fixture rect does not hit the lead rect either"

    st_ko = QuenchState(pcb, _THT_BOARD, keepouts=[e], **_QKW)
    assert not seeder.pose_ok(st_ko, _THT_REF, p.x, p.y, p.rot, set()), \
        (f"{_THT_REF} was seated with its LEADS in a keep-out -- pose_ok is "
         f"passing only the courtyard, so the grade will flag a seat it "
         f"accepted")
    # BOTH seat predicates, separately. `edge_seat_ok` bypasses `pose_ok` by
    # design, so it carries its own copy of the two-rect call and its own way
    # to lose it. A band of [0, 1e4] makes the overhang conjunct unable to be
    # what refuses.
    p2 = st_ko.parts[_THT_REF]
    assert not seeder.edge_seat_ok(st_ko, p2, p2.x, p2.y, 'north',
                                   0.0, 1e4), \
        (f"edge_seat_ok seated {_THT_REF} with its LEADS in a keep-out -- it "
         f"is passing only the courtyard")
    print(f"  PASS: {_THT_REF}'s lead rect is {gaps[side]:.4f}mm proud of its "
          f"courtyard on the {side} side; a keep-out in that sliver refuses "
          f"BOTH pose_ok and edge_seat_ok")


def test_the_grader_builds_a_state_that_carries_no_keepouts():
    """A grader that inherited the seat gate would be a tautology grading the
    seat predicate against itself, and every agreement test above would pass
    for the wrong reason. `grade` builds its own QuenchState and must not
    pass `keepouts`; `rule_keepout` reads `ctx.intent.keepouts`.

    Asserted on the EFFECTIVE state, not on the constructor argument. A check
    that read only the kwarg would be satisfied by any code that attaches
    keep-outs after construction -- which is a check that reads a reported
    field rather than the value actually used."""
    pcb = parse_kicad_pcb(BOARD)
    raw = floorplan.emit_intent(pcb, BOARD)
    st = sorted(_state(pcb).parts)[0]
    r = _state(pcb).parts[st].rect()
    raw['keepouts'] = [{'name': 'k', 'rect': [r[0] - 1, r[1] - 1,
                                              r[2] + 1, r[3] + 1]}]
    intent = floorplan.intent_from_dict(raw, BOARD)

    built = []
    real = QuenchState.__init__

    def spy(self, *a, **kw):
        built.append(self)
        return real(self, *a, **kw)

    try:
        QuenchState.__init__ = spy
        res = floorplan.grade(intent, pcb, BOARD)
    finally:
        QuenchState.__init__ = real

    assert built, "grade built no QuenchState"
    leaked = [(i, len(s.keepouts_for), len(s.keepouts))
              for i, s in enumerate(built)
              if s.keepouts_for or s.keepouts]
    assert not leaked, (
        f"grade's own state(s) carry the SEAT gate: {leaked}. The grader "
        f"would then be grading the seat predicate against itself")
    # ... and it still finds the violation, from the INTENT.
    assert any(v.rule == 'keepout' and v.ref == st for v in res.violations), \
        "the grader stopped grading keep-outs"
    print(f"  PASS: grade built {len(built)} state(s), none carrying a seat "
          f"gate, and still flagged {st}")


# --------------------------------------------------------------------------
# 5. the vocabulary is documented -- a static change detector
# --------------------------------------------------------------------------

def test_every_no_pose_verdict_has_a_README_row():
    """The README table is the only place a reader learns this vocabulary,
    and nothing guarded it before. A verdict shipped without its row is a
    string consumers switch on and humans cannot look up."""
    readme = io.open(os.path.join(REPO, 'py_placer', 'placement', 'README.md'),
                     encoding='utf-8').read()
    missing = [v for v in seeder.NO_POSE_VERDICTS
               if f"`{v}`" not in readme]
    assert not missing, f"verdict(s) with no README row: {missing}"
    # And the census keys, for the same reason.
    keys = sorted(seeder._empty_census())
    missing_k = [k for k in keys if f"`{k}`" not in readme]
    assert not missing_k, f"census key(s) with no README mention: {missing_k}"
    print(f"  PASS: {len(seeder.NO_POSE_VERDICTS)} verdict(s) and "
          f"{len(keys)} census key(s) all documented")


# --------------------------------------------------------------------------
# 6. the static gate -- "enforced on one path" is invisible behaviourally
# --------------------------------------------------------------------------

#: Modules allowed to call a seat predicate. Every entry is a module that
#: builds its state with `keepouts=`; a NEW caller must be added here AND
#: attach, which is the whole point of listing them.
_SEAT_FUNCS = ('_try_place', 'pose_ok', 'count_legal_poses', 'edge_seat_ok')


def test_every_seat_predicate_caller_outside_the_seeder_attaches_keepouts():
    """`pose_ok` is the chokepoint, but only for code that goes through the
    seeder's own state. A module that builds its own state and calls
    `seeder._try_place` on it enforces nothing, and no behavioural test can
    see that unless it happens to exercise that path -- which is exactly how
    a rule ends up enforced on some paths and silently absent on others.

    So: enumerate the callers mechanically, and require each one's enclosing
    function to build its state with `keepouts=`."""
    offenders = []
    checked = []
    for root, _dirs, files in os.walk(os.path.join(REPO, 'py_placer')):
        for fn in files:
            if not fn.endswith('.py'):
                continue
            path = os.path.join(root, fn)
            if os.path.basename(path) == 'seeder.py':
                continue          # the definitions live here
            tree = ast.parse(io.open(path, encoding='utf-8').read())
            for node in ast.walk(tree):
                if not isinstance(node, (ast.FunctionDef,
                                         ast.AsyncFunctionDef)):
                    continue
                body = ast.dump(node)
                if not any(f"attr='{f}'" in body or f"id='{f}'" in body
                           for f in _SEAT_FUNCS):
                    continue
                rel = os.path.relpath(path, REPO).replace('\\', '/')
                checked.append(f"{rel}:{node.lineno} {node.name}")
                if "keepouts" not in body:
                    offenders.append(f"{rel}:{node.lineno} {node.name}")
    assert checked, ("the scan found NO seat-predicate caller outside "
                     "seeder.py -- the gate is vacuous, fix the scan")
    assert not offenders, (
        "seat-predicate caller(s) whose state carries no keep-outs, so the "
        f"rule is unenforced there: {offenders}")
    print(f"  PASS: {len(checked)} external seat-predicate caller(s), all "
          f"attaching keep-outs -- {', '.join(checked)}")


TESTS = [
    test_the_seat_predicate_calls_the_graders_own_hit_test,
    test_the_edge_seat_calls_it_too,
    test_the_edge_seat_names_the_keepout_that_refused_it,
    test_the_seat_verdict_and_the_grade_verdict_agree_on_every_combination,
    test_the_hit_test_boundaries,
    test_the_resolver_boundaries,
    test_the_seat_predicate_sees_the_through_hole_rect_not_just_the_body,
    test_the_grader_builds_a_state_that_carries_no_keepouts,
    test_every_no_pose_verdict_has_a_README_row,
    test_every_seat_predicate_caller_outside_the_seeder_attaches_keepouts,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
