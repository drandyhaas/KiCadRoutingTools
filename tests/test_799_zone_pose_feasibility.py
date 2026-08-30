"""A zone a keep-out leaves no room in is refused, not only one it covers (#799).

#702 refuses total coverage. The question underneath is "does the zone MINUS the
binding keep-outs still hold this part at some rotation", and the two coincide
only at total coverage. Measured on the pre-#799 tree: zone `[10,10,20,20]` at
tolerance 0 against a keep-out `[10,10,19.9,20]` -- 99% of it -- raised nothing
while the member had zero satisfying poses, and two keep-outs covering half each
were missed the same way.

MOST OF THIS FILE IS CONTROLS. The check refuses an intent at ERROR severity, so
a false positive is worse than a miss, and the arms below are the ones a
brainstorm pass predicted would be wrong while every naturally-written test
passed. Four of them WERE wrong in the first implementation and are fixed here:

  * a DEGENERATE keep-out rect (`_rect` normalises `[15,20,15,10]` into a
    zero-width rect rather than refusing it) forbade a band of origins while
    `rect_overlap_area` can never report a positive area for it;
  * an ANCHOR-mode zone with an off-centre courtyard was refused by any keep-out
    anywhere, because an earlier draft intersected the seeder's origin
    convention with the grade's centre convention -- and on 234 of 1316 corpus
    parts those boxes are disjoint (worst 36.825mm);
  * a corner overlap under `keepout_hit`'s EPS-of-AREA threshold is not a hit,
    but the open-rect algebra excluded it;
  * one decorative circle anywhere switched the whole check off for a part.

The fix that makes most of these disappear at once: the rect algebra only
PROPOSES candidate origins, and every candidate is judged by `zone_escape` and
`keepout_hit` -- the same two functions the grade judges with.

THE MUTATION TABLE, RECORDED FROM THE RUN (`python3 tests/mutate_799.py`),
never predicted and never edited afterwards to match:

    20 rows: 16 killed, 4 survived, 0 broken, 0 disagreeing with expectation

The first run was 14 killed, 6 survived, 4 WRONG. TWO were defects -- one in
this file, one dead line in the engine -- and two were rows whose EXPECTATION
was wrong, now recorded as survivors with their reasons:

  * `total-coverage-no-longer-runs-first` measured nothing -- its anchor was a
    `continue` that was already the last statement of its loop body. The dead
    line is gone and the row targets the real dedupe.
  * `the-binding-resolver-is-bypassed` survived because both whole-block
    exemption controls exempt EVERY member, so the per-member loop is never
    entered and they passed for a reason that was not the resolver. The mixed
    block in `arm_reporting` covers it now.

The four SURVIVORS are recorded with their reasons, not deleted:

  * `the-degenerate-keepout-still-forbids` and
    `the-through-hole-rect-is-forgotten-by-the-GENERATOR` -- both drop a hole
    from candidate GENERATION only, and verification still decides every
    candidate with `keepout_hit`. The guards are belt-and-braces; the rows are
    the change detector for the day verification stops backing the generator.
  * `the-zone-side-eps-slack-is-dropped` -- the EPS slack on the zone side
    guards a float-scale case (`(z0-tol-b0)+b0 < z0-tol` on a few percent of
    random triples -- 6.3% and 7.4% in two independent probes, sampling
    distribution unpinned) that no arm here constructs. The row exists so the
    guard is already written the day one does.
  * `synthetic-parts-are-graded-too` -- no fixture here carries a footprint with
    neither courtyard nor pads.
"""
import json
import os
import sys
import tempfile

RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import floorplan  # noqa: E402

RULE = 'intent_zone_in_keepout'
SIZE = (40.0, 30.0)

passed = 0
failed = 0


def check(name, ok, detail=""):
    """Detail must read as a MEASUREMENT: it prints on OK and on FAIL alike."""
    global passed, failed
    if ok:
        passed += 1
        print(f"  OK   {name}" + (f" -- {detail}" if detail else ""))
    else:
        failed += 1
        print(f"  FAIL {name}" + (f" -- {detail}" if detail else ""))


def P(local, rot=0.0, tht=None):
    return floorplan._LocalPart(rot, local, tht)


def ko(name, rect=None, circle=None, allow=(), sides=('F', 'B')):
    e = {'name': name, 'allow': tuple(allow), 'sides': tuple(sides)}
    if rect is not None:
        e['rect'] = tuple(float(v) for v in rect)
    else:
        e['circle'] = tuple(float(v) for v in circle)
    return e


F = floorplan.zone_pose_feasibility


def arm_the_issues_own_cases():
    print("--- the counterexamples #799 was filed with")
    check("99% coverage refuses where total coverage did not",
          not F((10, 10, 20, 20), 0.0, P((-1, -1, 1, 1)),
                [ko('wall', [10, 10, 19.9, 20])])['feasible'],
          "zone [10,10,20,20] tol 0 vs keep-out [10,10,19.9,20]")
    # The old test must still say nothing, or the widening is untested: this is
    # what proves the new verdict came from the new code.
    z = type('Z', (), {'rect': (10.0, 10.0, 20.0, 20.0)})()
    check("... and `zone_covered_by_keepout` STILL returns None on it",
          floorplan.zone_covered_by_keepout(
              z, [ko('wall', [10, 10, 19.9, 20])], {'U1': frozenset('F')},
              0.0) is None,
          "so the refusal is the widened check, not a changed old one")
    check("CONTROL: a 50% partial overlap is NOT refused",
          F((10, 10, 20, 20), 0.0, P((-1, -1, 1, 1)),
            [ko('half', [10, 10, 15, 20])])['feasible'],
          "a zone with a bite out of it still has room")

    r = F((10, 10, 20, 20), 0.0, P((-1, -1, 1, 1)),
          [ko('a', [10, 10, 15, 20]), ko('b', [15, 10, 20, 20])])
    check("two keep-outs covering half each refuse together",
          not r['feasible'], f"reason={r['reason']}")
    check("... and the blame says each is individually necessary",
          r['reason'] == 'keepout_any_of'
          and set(r['keepouts_freeing']) == {'a', 'b'},
          f"freeing={r['keepouts_freeing']} joint={r['keepouts_joint']}")
    r2 = F((10, 10, 20, 20), 0.0, P((-1, -1, 1, 1)),
           [ko('a', [10, 10, 19.9, 20]), ko('b', [10, 10, 19.8, 20])])
    check("nested keep-outs where no single lift frees say JOINT",
          r2['reason'] == 'keepout_joint'
          and set(r2['keepouts_joint']) == {'a', 'b'},
          f"freeing={r2['keepouts_freeing']} joint={r2['keepouts_joint']}")
    r3 = F((10, 10, 20, 20), 0.0, P((-1, -1, 1, 1)),
           [ko('wall', [10, 10, 19.9, 20]), ko('far', [900, 900, 901, 901])])
    check("one culprit beside an innocent bystander names only the culprit",
          r3['reason'] == 'keepout_alone'
          and r3['keepouts_freeing'] == ('wall',),
          f"freeing={r3['keepouts_freeing']}")


def arm_false_error_guards():
    print("--- the four false-ERROR sources, each with its control")
    for tag, rect in (('zero-width', [15, 10, 15, 20]),
                      ('zero-height', [10, 15, 20, 15]),
                      ('point', [15, 15, 15, 15])):
        check(f"a DEGENERATE ({tag}) keep-out refuses nothing",
              F((10, 10, 20, 20), 0.0, P((-5, -5, 5, 5)),
                [ko('k', rect)])['feasible'],
              "rect_overlap_area is 0 for it, so keepout_hit can never fire")
    check("CONTROL: a thin but REAL keep-out still refuses",
          not F((10, 10, 20, 20), 0.0, P((-5, -5, 5, 5)),
                [ko('k', [14.995, 10, 15.005, 20])])['feasible'],
          "so the arms above are the degeneracy, not the thinness")

    # tigard J2/J3's real courtyard: centre offset 10.15mm from the origin.
    tig = (-1.8, -1.8, 1.8, 22.1)
    check("an ANCHOR zone is not refused by a far-away keep-out",
          F((50.0, 30.0, 50.4, 30.4), 0.5, P(tig, rot=90),
            [ko('far', [0, 0, 1, 1])])['feasible'],
          "234 of 1316 corpus parts have an off-centre courtyard")
    check("CONTROL: an anchor zone IS refused by a keep-out over it",
          not F((50.0, 30.0, 50.4, 30.4), 0.5, P(tig, rot=90),
                [ko('on', [30, 20, 70, 45])])['feasible'],
          "so the arm above is the convention, not an inert anchor branch")

    check("a corner overlap under EPS of AREA is not a hit, so not refused",
          F((0, 0, 10, 10), 0.0, P((-1, -1, 1, 1)),
            [ko('nw', [1.999, 1.999, 100, 100])])['feasible'],
          "1e-6 mm2 of overlap; keepout_hit thresholds on AREA")
    check("CONTROL: a real corner bite refuses",
          not F((0, 0, 10, 10), 0.0, P((-1, -1, 1, 1)),
                [ko('nw', [1.5, 1.5, 100, 100])])['feasible'])

    check("a decorative far-away CIRCLE does not disarm the rect verdict",
          not F((0, 0, 10, 10), 0.0, P((-1, -1, 1, 1)),
                [ko('a', [0, 0, 10, 6]), ko('b', [0, 4, 10, 10]),
                 ko('c', circle=(500, 500, 1.0))])['feasible'],
          "the rects alone still refuse, so the answer is sound")
    rc = F((4, 4, 6, 6), 0.0, P((-0.2, -0.2, 0.2, 0.2)),
           [ko('hole', circle=(5.0, 5.0, 4.0))])
    check("a circle that IS the refusal abstains instead of refusing",
          rc['feasible'] and rc['reason'] == 'circle_undecided'
          and rc['undecided_circles'] == ('hole',),
          f"reason={rc['reason']} -- no disc/rect free-area kernel exists")

    # THE MIDDLE CASE, and the one both arms above miss: rects that are
    # satisfiable ON THEIR OWN, plus a disc that closes the last of the room.
    # The two arms above cover only "no rects at all" and "the rects already
    # refuse". Weakening the guard to `if discs and not rects` leaves both of
    # them passing and turns this case into an ERROR that blames the RECT for a
    # refusal the CIRCLE caused -- exactly the thing the docstring says cannot
    # happen.
    mid = F((0, 0, 10, 10), 0.0, P((-1, -1, 1, 1)),
            [ko('a', [0, 0, 8, 10]), ko('disc', circle=(9.0, 5.0, 10.0))])
    only_rect = F((0, 0, 10, 10), 0.0, P((-1, -1, 1, 1)),
                  [ko('a', [0, 0, 8, 10])])
    check("a disc closing the room left by a SATISFIABLE rect abstains",
          mid['feasible'] and mid['reason'] == 'circle_undecided'
          and mid['undecided_circles'] == ('disc',),
          f"reason={mid['reason']} freeing={mid['keepouts_freeing']} "
          f"-- refusing here would blame the rect for the disc's refusal")
    check("CONTROL: that rect alone really is satisfiable",
          only_rect['feasible'] and only_rect['reason'] == 'seated',
          f"reason={only_rect['reason']} witness={only_rect['witness']} "
          f"-- so the arm above is the disc, not an already-doomed zone")


def arm_boundary_and_rotation():
    print("--- the boundary, the rotations, and the tolerance band")
    check("a gap EXACTLY the courtyard width is feasible",
          F((0, 0, 10, 10), 0.0, P((-1, -1, 1, 1)),
            [ko('a', [0, 0, 4, 10]), ko('b', [6, 0, 10, 10])])['feasible'],
          "the witness is a measure-zero line ON a keep-out edge, so the "
          "candidate set must contain the edges and not only midpoints")
    check("... and one epsilon narrower refuses",
          not F((0, 0, 10, 10), 0.0, P((-1, -1, 1, 1)),
                [ko('a', [0, 0, 4, 10]),
                 ko('b', [5.999, 0, 10, 10])])['feasible'])

    # A RECTANGULAR part: a square one proves nothing about the rotation loop,
    # because every rotation of a square is the same box.
    check("a part that fits only at 90 degrees is feasible",
          F((0, 0, 3, 9), 0.0, P((-4, -1, 4, 1)),
            [ko('x', [100, 100, 101, 101])])['feasible'],
          "8x2 courtyard in a 3x9 zone")
    check("CONTROL: a SQUARE part gives the same answer at every rotation",
          F((0, 0, 6, 6), 0.0, P((-1, -1, 1, 1)),
            [ko('x', [0, 0, 2.5, 6])])['feasible']
          == F((0, 0, 6, 6), 0.0, P((-1, -1, 1, 1), rot=90),
               [ko('x', [0, 0, 2.5, 6])])['feasible'],
          "so a rotation arm written on a square subject is vacuous")
    r45 = F((0, 0, 4.1, 1.1), 0.0, P((-2, -0.5, 2, 0.5), rot=45),
            [ko('x', [100, 100, 101, 101])])
    check("a 45-degree part is scored on its OWN rotation lattice",
          set(round(v, 1) for v in r45['rotations']) == {45.0, 135.0,
                                                        225.0, 315.0},
          f"rotations={[round(v, 1) for v in r45['rotations']]}")

    check("the TOLERANCE BAND can be the only feasible region",
          F((10, 10, 14, 14), 2.0, P((-0.5, -0.5, 0.5, 0.5)),
            [ko('k', [10, 10, 14, 14])])['feasible'],
          "keep-out == the bare zone rect, tolerance 2.0")
    check("... and the same intent at tolerance 0 refuses",
          not F((10, 10, 14, 14), 0.0, P((-0.5, -0.5, 0.5, 0.5)),
                [ko('k', [10, 10, 14, 14])])['feasible'],
          "so the arm above is the band, not an inert keep-out")

    # The through-hole rect is graded too (`rule_keepout` tests both), and on
    # the corpus it only ever matters for parts whose pads reach outside the
    # courtyard -- ulx3s LCD1 is one, +1.64mm east.
    b = (-9.37, -1.75, 9.38, 1.75)
    t = (-8.484, -1.016, 11.024, 1.016)
    zone = (0, 0, 18.8, 3.6)
    with_t = F(zone, 0.0, P(b, tht=t), [ko('east', [19, -10, 30, 10])])
    without = F(zone, 0.0, P(b), [ko('east', [19, -10, 30, 10])])
    # The detail is built with `.get`-safe accessors: an eager
    # `with_t['witness'][2]` raises when the assertion is False BECAUSE the
    # witness is None, and the file then dies mid-run -- turning a genuine FAIL
    # into a broken test, which is the same signal.
    wt = with_t['witness']
    wo = without['witness']
    check("the THROUGH-HOLE rect is priced, and moves the witness rotation",
          bool(wt) and bool(wo) and wt[2] != wo[2],
          f"with tht -> {wt}, courtyard only -> {wo}")


def _part(ref, x, y, half=1.0, thru=False):
    pad = ('\t\t(pad "1" thru_hole circle\n\t\t\t(at 0 0)\n'
           '\t\t\t(size 0.6 0.6)\n\t\t\t(drill 0.35)\n'
           '\t\t\t(layers "*.Cu" "*.Mask")\n\t\t\t(net 1 "N1")\n'
           f'\t\t\t(uuid "p0-{ref}")\n\t\t)\n') if thru else (
        '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 0.3 0.3)\n'
        '\t\t\t(layers "F.Cu")\n\t\t\t(net 1 "N1")\n'
        f'\t\t\t(uuid "p0-{ref}")\n\t\t)\n')
    return (f'\t(footprint "test:P{ref}"\n\t\t(layer "F.Cu")\n'
            f'\t\t(uuid "fp-{ref}")\n\t\t(at {x} {y})\n'
            f'\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0)\n\t\t)\n'
            f'\t\t(fp_rect\n\t\t\t(start {-half} {-half})\n'
            f'\t\t\t(end {half} {half})\n\t\t\t(layer "F.CrtYd")\n'
            f'\t\t\t(uuid "cy-{ref}")\n\t\t)\n' + pad + '\t)\n')


def board(path, parts):
    body = ('(kicad_pcb\n\t(version 20241229)\n\t(net 0 "")\n\t(net 1 "N1")\n'
            f'\t(gr_rect\n\t\t(start 0 0)\n\t\t(end {SIZE[0]} {SIZE[1]})\n'
            '\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n'
            + ''.join(parts) + ')\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(body)
    return path


def intent_doc(blocks=(), keepouts=()):
    d = {"schema": 1, "kind": "floorplan-intent", "units": "mm",
         "envelope": {"rect": [0.0, 0.0, SIZE[0], SIZE[1]],
                      "tolerance_mm": 0.5},
         "blocks": [dict(b) for b in blocks]}
    if keepouts:
        d["keepouts"] = [dict(k) for k in keepouts]
    return d


def load(doc, wd, tag):
    p = os.path.join(wd, f'{tag}.json')
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(doc, f)
    return floorplan.load_intent(p)


def arm_the_eps_limitation():
    """The ONE case where this check refuses a pose the grade would accept.

    Recorded, not fixed. `keepout_hit` fires on overlap AREA above EPS, so the
    true satisfying set is slightly LARGER than the open-rect complement the
    candidates are drawn from, and a free window can fall strictly between two
    sampled coordinates. Found by an adversarial review, derived analytically
    (the sweep's 33696 cases and ~7200 fuzz cases both missed it -- the window
    here is 3e-7mm wide).

    It is bounded: a missed pose must graze EVERY binding keep-out by under one
    square micrometre, i.e. far below the 0.05mm floor of any lattice the seat
    search sweeps, so no authored intent reaches it. It is deliberately NOT
    modelled -- widening the candidate set to cover the slack would invent
    tolerance the grade does not have.

    The arm asserts the CURRENT (wrong) answer on purpose, so this is a change
    detector: if someone closes the gap, this arm fails and says so.
    """
    print("--- the recorded EPS-of-area limitation (asserts the WRONG answer)")
    zone = (0.0, 0.0, 10.0, 2.0)
    part = P((-1.0, -1.0, 1.0, 1.0))
    kos = [ko('left', [-99.0, 0.5, 4.0, 1.0]),
           ko('right', [5.9999978, -10.0, 99.0, 10.0])]
    v = F(zone, 0.0, part, kos)
    x, y, rot = 4.99999815, 1.0, 0.0
    r = part.rect(x, y, rot)
    esc = floorplan.zone_escape(zone, r, False)[0]
    hits = [floorplan.keepout_hit(k, (r, None)) for k in kos]
    check("the missed pose satisfies BOTH graded predicates",
          esc == 0.0 and hits == [0.0, 0.0],
          f"zone_escape={esc} keepout_hit={hits} at ({x}, {y}, {rot})")
    check("...and the check refuses it anyway -- KNOWN, bounded, unfixed",
          not v['feasible'],
          f"reason={v['reason']} -- the free window is ~3e-7mm wide and every "
          f"pose in it grazes both keep-outs by under 1e-6 mm2")


def arm_feasible_means_seated():
    """`feasible=True` is also what every ABSTENTION returns.

    Five reasons yield it -- `seated`, `no_keepout_binds`, `zone_too_small`,
    `circle_undecided`, `too_many_keepouts` -- and only `seated` means "there
    is room". So a bare `['feasible']` assertion passes when the search found
    NOTHING, which is the opposite of what such an arm claims. Measured: with
    `_search` stubbed to always return None, 13 of 39 checks in this file still
    passed, including every arm in `arm_false_error_guards`. Two of them are
    precisely the regressions their siblings exist to catch: the 90-degree arm
    reads True under `only-one-rotation-is-tried`, and the anchor arm reads True
    under the origin/centre intersection bug.

    This arm pins the REASON for every positive fixture in the file at once,
    which closes that hole without weakening the refusal arms (`feasible=False`
    has only ever meant a real refusal).
    """
    print("--- a positive verdict must be `seated`, never an abstention")
    tig = (-1.8, -1.8, 1.8, 22.1)
    cases = [
        ("50% partial overlap", (10, 10, 20, 20), 0.0, P((-1, -1, 1, 1)),
         [ko('half', [10, 10, 15, 20])]),
        ("degenerate keep-out", (10, 10, 20, 20), 0.0, P((-5, -5, 5, 5)),
         [ko('k', [15, 10, 15, 20])]),
        ("anchor zone, far keep-out", (50.0, 30.0, 50.4, 30.4), 0.5,
         P(tig, rot=90), [ko('far', [0, 0, 1, 1])]),
        ("corner nick under EPS", (0, 0, 10, 10), 0.0, P((-1, -1, 1, 1)),
         [ko('nw', [1.999, 1.999, 100, 100])]),
        ("gap exactly the courtyard", (0, 0, 10, 10), 0.0, P((-1, -1, 1, 1)),
         [ko('a', [0, 0, 4, 10]), ko('b', [6, 0, 10, 10])]),
        ("fits only at 90", (0, 0, 3, 9), 0.0, P((-4, -1, 4, 1)),
         [ko('x', [100, 100, 101, 101])]),
        ("tolerance band", (10, 10, 14, 14), 2.0, P((-0.5, -0.5, 0.5, 0.5)),
         [ko('k', [10, 10, 14, 14])]),
    ]
    bad = []
    for name, z, tol, part, kos in cases:
        v = F(z, tol, part, kos)
        if not (v['feasible'] and v['reason'] == 'seated'
                and v['witness'] is not None):
            bad.append(f"{name}: feasible={v['feasible']} "
                       f"reason={v['reason']} witness={v['witness']}")
    check("every positive fixture reports `seated` with a real witness",
          not bad, f"{len(cases)} fixture(s) checked" if not bad
          else '; '.join(bad))

    # And the discriminating half: the 90-degree fixture must be seated AT 90,
    # or the arm is satisfied by a part that fits at 0 too and proves nothing
    # about the rotation loop.
    v90 = F((0, 0, 3, 9), 0.0, P((-4, -1, 4, 1)),
            [ko('x', [100, 100, 101, 101])])
    check("... and the 90-degree fixture is seated AT 90",
          v90['witness'] is not None and v90['witness'][2] == 90.0,
          f"witness={v90['witness']} -- an 8x2 courtyard cannot fit a 3x9 zone "
          f"at rot 0, so this is what makes the SQUARE control's point true")


def arm_reporting(wd):
    print("--- how it is reported, and where")
    b = board(os.path.join(wd, 'r.kicad_pcb'),
              [_part('U1', 15.0, 15.0, half=1.0),
               _part('U2', 15.0, 25.0, half=4.0)])
    pcb = parse_kicad_pcb(b)
    zone = [10.0, 10.0, 20.0, 20.0]
    blocks = [{"name": "z", "refs": ["U1", "U2"], "zone": zone,
               "tolerance_mm": 0.0}]
    # Covers all but a 0.1mm strip: too small for U2's 8x8 courtyard, roomy
    # for U1's 2x2. THE SAME ZONE AND KEEP-OUT, two answers.
    kos = [{"name": "wall", "rect": [10.0, 10.0, 17.0, 20.0]}]
    it = load(intent_doc(blocks, kos), wd, 'r')

    g = floorplan.grade(it, pcb, b)
    gr = [v for v in g.violations if v.rule == RULE]
    _bundle, probs = floorplan.resolve_intent_gate(it, pcb, ())
    pr = [v for v in probs if v.rule == RULE]

    check("it is reported PER MEMBER, not per block",
          len(gr) == 1 and gr[0].ref == 'U2',
          f"{len(gr)} finding(s), refs={[v.ref for v in gr]} "
          f"-- U1 (2x2) fits the strip, U2 (8x8) does not")
    check("BOTH grade() and resolve_intent_gate() report it",
          len(pr) == 1 and pr[0].ref == 'U2'
          and pr[0].message == gr[0].message,
          f"grade={len(gr)} gate={len(pr)}, messages "
          f"{'identical' if pr and pr[0].message == gr[0].message else 'DIFFER'}")
    check("check_floorplan can now see it: it is an ERROR, so `passed` is False",
          bool(gr) and not g.passed and gr[0].severity == floorplan.ERROR,
          f"passed={g.passed} severity="
          f"{gr[0].severity if gr else 'no finding raised'}")

    # The wording was measured in #702 and must not drift: 6 of 8 probed
    # alternative poses inside a swallowed zone were ADMITTED, so the member is
    # CONFINED, not frozen.
    m = gr[0].message if gr else ''
    check("the message says the member cannot LEAVE its zone",
          'never LEAVE its zone' in m, m[:80] or 'no finding raised')
    check("... and does not claim it cannot move",
          bool(m) and 'cannot move' not in m.lower(),
          "confined, not frozen -- every pose in the zone scores identically")
    meas = gr[0].measured if gr else {}
    check("the measured payload names the keep-out and the rotations",
          meas.get('keepouts_freeing') == ['wall']
          and len(meas.get('rotations') or []) == 4,
          f"freeing={meas.get('keepouts_freeing')} "
          f"rotations={meas.get('rotations')}")

    # CONTROLS: the two filters #702 was fixed to honour, and total coverage.
    for tag, extra in (('allow', {"allow": ["U1", "U2"]}),
                       ('sides', {"sides": ["B"]})):
        itx = load(intent_doc(blocks, [dict(kos[0], **extra)]), wd, f'r_{tag}')
        _bx, px = floorplan.resolve_intent_gate(itx, pcb, ())
        check(f"CONTROL: a keep-out exempt by `{tag}` raises nothing",
              not [v for v in px if v.rule == RULE],
              "the same resolver the seat and the grade use")

    # A MIXED block: the keep-out would refuse both members, and only one is
    # exempt. The two whole-block controls above cannot catch a per-member loop
    # that ignores the resolver, because when EVERY member is exempt the loop
    # is never entered at all -- measured: bypassing `keepouts_for_ref` inside
    # the loop survived both of them.
    wide = [{"name": "wide", "rect": [10.0, 10.0, 19.5, 20.0],
             "allow": ["U1"]}]
    itm = load(intent_doc(blocks, wide), wd, 'r_mixed')
    _bm, pm = floorplan.resolve_intent_gate(itm, pcb, ())
    mixed = [v for v in pm if v.rule == RULE]
    check("an exempt member is skipped while its block-mate is reported",
          len(mixed) == 1 and mixed[0].ref == 'U2',
          f"{len(mixed)} finding(s), refs={[v.ref for v in mixed]} "
          f"-- the keep-out refuses both; only U1 is in `allow`")
    itm2 = load(intent_doc(blocks, [dict(wide[0], allow=[])]), wd, 'r_mixed2')
    _bm2, pm2 = floorplan.resolve_intent_gate(itm2, pcb, ())
    check("CONTROL: without the exemption BOTH members are reported",
          {v.ref for v in pm2 if v.rule == RULE} == {'U1', 'U2'},
          f"refs={sorted(v.ref for v in pm2 if v.rule == RULE)} "
          f"-- so the arm above is the exemption, not an unreachable member")

    itc = load(intent_doc(blocks, [{"name": "all", "rect": [0, 0, 40, 30]}]),
               wd, 'r_cover')
    _bc, pc = floorplan.resolve_intent_gate(itc, pcb, ())
    cov = [v for v in pc if v.rule == RULE]
    check("total coverage keeps the #702 message, once, per BLOCK",
          len(cov) == 1 and cov[0].ref is None
          and 'covers entirely' in cov[0].message,
          f"{len(cov)} finding(s), ref={cov[0].ref if cov else None}")

    empty = load(intent_doc(blocks, []), wd, 'r_none')
    _be, pe = floorplan.resolve_intent_gate(empty, pcb, ())
    check("CONTROL: no keep-outs at all -> nothing, and no file read",
          not [v for v in pe if v.rule == RULE],
          "this is what keeps every emitted intent inert")


def arm_identity():
    print("--- the kernel is the GRADE's, not a copy of it")
    zone, part = (10, 10, 20, 20), P((-1, -1, 1, 1))
    kos = [ko('wall', [10, 10, 19.9, 20])]
    base = F(zone, 0.0, part, kos)

    real = floorplan.keepout_hit
    seen = []

    def never(entry, rects):
        seen.append(1)
        return 0.0
    floorplan.keepout_hit = never
    try:
        patched = F(zone, 0.0, part, kos)
    finally:
        floorplan.keepout_hit = real
    check("patching `keepout_hit` flips the verdict",
          not base['feasible'] and patched['feasible'] and seen,
          f"called {len(seen)} time(s); a private hit test would not move")

    real_z = floorplan.zone_escape
    seen_z = []

    def far(zone_rect, part_rect, anchor):
        seen_z.append(1)
        return (1e9, 'north')
    far_ko = [ko('far', [900, 900, 901, 901])]
    clean = F(zone, 0.0, part, far_ko)
    floorplan.zone_escape = far
    try:
        pz = F(zone, 0.0, part, far_ko)
    finally:
        floorplan.zone_escape = real_z
    # Poisoning the zone term does not flip this to a REFUSAL, and should not:
    # with no candidate satisfying the zone even with every keep-out dropped,
    # the honest answer is `zone_too_small` -- which is
    # `rule_zone_containment`'s finding, not a keep-out contradiction. That the
    # reason moves is the proof the term was consulted.
    check("patching `zone_escape` is observed too",
          seen_z and clean['reason'] == 'seated'
          and pz['reason'] == 'zone_too_small' and pz['feasible'],
          f"called {len(seen_z)} time(s); reason {clean['reason']} -> "
          f"{pz['reason']} -- and it does NOT become a refusal, because a "
          f"zone nothing fits is not a keep-out's fault")


def main():
    with tempfile.TemporaryDirectory() as wd:
        arm_the_issues_own_cases()
        arm_false_error_guards()
        arm_feasible_means_seated()
        arm_the_eps_limitation()
        arm_boundary_and_rotation()
        arm_reporting(wd)
        arm_identity()
    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
