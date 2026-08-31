#!/usr/bin/env python3
"""#797: `place_seed` honours a declared EXCLUSIVE zone, end to end.

The boards are HAND-WRITTEN, for the reason `tests/test_701_keepout_seating.py`
gives for its own: on a corpus board "where would this part have gone" is only
answerable by re-running the seeder, which makes the control circular. Here a
single-ref zone gets `jx, jy = (0.0, 0.0)` (seeder.py:1547 -- "a single-member
zone is the spec-coordinate pattern, so it gets the exact center"), so the seat
target is the zone centre exactly and the nearest clear pose can be written
down.

WHY EVERY STRANGER HERE SITS IN AN *ANCHOR* ZONE, measured rather than chosen.
The obvious fixture -- give the stranger its own zone inside the reserved rect
-- is rejected by the engine before it places anything:

    GRADE ERROR [intent_zone_overlap] zones 'rf' and 's' overlap by 28.00mm2
    on the same side; no placement can satisfy both

So two zones may not overlap at all, and a stranger with an ordinary zone can
never be aimed into someone else's reserved area. What CAN, and is the real
shape of this bug, is a zone SMALLER than the courtyard: `seeder.zone_gate`
switches to its ANCHOR branch and constrains the footprint ORIGIN, leaving the
courtyard free to spill across the zone boundary into the reserved rect next
door. The zones stay disjoint, the intent is satisfiable, and the intrusion is
real -- confirmed on the unmodified engine, which seats R1 at (9.0, 6.0) with
4.00mm2 inside 'rf' and then exits 4 against its own intent.

EVERY ARM'S SEAT IS A THEOREM, and the arms assert the NUMBER. "the part is out
of the rect" is satisfied by any refusal at all; "it moved exactly 1.000mm, to
(8.000, 6.000)" is satisfied only by a search that found the nearest clear
pose. A fallback-sweep implementation lands on the 2.0mm `FALLBACK_STEP_MM`
lattice; a tolerance-inflated zone rect overshoots.

Arm X's theorem lands on the ZERO-AREA boundary on purpose: at cx = 8.0 the
courtyard's east edge is exactly the zone's west edge, so the overlap area is
exactly 0.0 and the pose is legal only under a strict `> EPS` threshold.

EVERY ACCEPTING ARM HAS A CONTROL. An assertion like "the part is not in the
reserved rect" is satisfied by a board where the part was never going there,
and then the arm passes in both directions. Each control also RE-GRADES its own
written board against the arm's EXCLUSIVE intent, which is what proves the
arm's clean grade is a consequence of the change rather than of the board.

Every accepting arm asserts on the RE-PARSED WRITTEN BOARD, never on the JSON
summary alone -- test_630_seeder_eviction.py records that its own first version
passed while one part sat 100% inside another's courtyard, because it believed
a count. The JSON is checked too, for what it claims.

MEASURED MUTATION TABLE, from `python3 -X utf8 tests/mutate_797.py`. The edits
are in that file; these are the verdicts, recorded FROM THE RUN and not
predicted here:

    24 rows: 21 killed, 3 survived, 0 broken, 0 disagreeing with expectation

    the three survivors, recorded rather than deleted:
      the-grade-threshold-is-ge-not-gt     `> EPS` and `>= EPS` differ on
      the-gate-threshold-is-zero-not-EPS   exactly one input -- an overlap of
                                           1e-6 mm2 -- and no board can produce
                                           one. The only sub-EPS value real
                                           geometry yields is exactly 0.0, a
                                           zero-area touch, where every
                                           threshold in [0, EPS] agrees. The
                                           EPS arm in the predicate file pins
                                           the direction arithmetically but
                                           does not import either branch. The
                                           `zero-area-touch` rows are the
                                           killable form and DO guard it.
      the-reseat-resolves-blocks-at-bare-empty-sources
                                           no fixture here puts a `group:`-
                                           shaped block on the re-seat path --
                                           the synthetic boards have no KiCad
                                           groups and no sheets, so 'auto' and
                                           '()' resolve identically on them.
                                           The guard is by construction.

Two rows exist because the RUN corrected a prediction, not the engine:
`the-gate-threshold-is-zero-not-EPS` was written expecting a kill and survived
(see above), and `the-grade-threshold-is-ge-not-gt` was reported BROKEN rather
than applied, because its one-line anchor matched twice in floorplan.py.

NOT COVERED HERE, and why rather than silently: the courtyard-only measurement
(a stranger whose LEADS cross a reserved zone is a pose the grade calls clean).
An end-to-end version needs the zone's owner seated inside a lead-width rect,
where courtyard-to-lead clearance rather than `zone_exclusive` decides the
outcome -- a vacuous arm. It is tested instead in
`tests/test_797_zone_exclusive_predicate.py`, through the same `seeder.pose_ok`,
on ulx3s/LCD1 where the sliver is derived from measured geometry.
"""
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO,):
    if _p not in sys.path:
        sys.path.insert(0, _p)
        sys.path.insert(0, os.path.join(_p, 'py_router'))   # placement split
        sys.path.insert(0, os.path.join(_p, 'py_tools'))    # placement split
        sys.path.insert(0, os.path.join(_p, 'py_placer'))   # placement split

from kicad_parser import parse_kicad_pcb                        # noqa: E402
from placement import floorplan                                 # noqa: E402

# The fixtures are test_701_keepout_seating's, imported rather than copied:
# a second `_part` would be a second definition of what a courtyard is.
from test_701_keepout_seating import (_part, board, run, poses,  # noqa: E402
                                      overlap, pairwise_legal, SIZE)

RUN_ALL_TIMEOUT = 1200

passed = failed = 0


def check(name, ok, detail=""):
    """Prints `detail` on OK and FAIL alike, so every detail must read as a
    MEASUREMENT rather than as a failure explanation."""
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}"
          + (f" -- {detail}" if detail else ""))


def graded_exclusive_errors(board_path, intent):
    pcb = parse_kicad_pcb(board_path)
    res = floorplan.grade(floorplan.intent_from_dict(dict(intent), board_path),
                          pcb, board_path, clearance=0.2,
                          board_edge_clearance=0.5)
    return [v for v in res.errors if v.rule == 'zone_exclusive']


def graded_errors(board_path, intent):
    pcb = parse_kicad_pcb(board_path)
    res = floorplan.grade(floorplan.intent_from_dict(dict(intent), board_path),
                          pcb, board_path, clearance=0.2,
                          board_edge_clearance=0.5)
    return res.errors


def intent(blocks):
    return {"schema": 1, "kind": "floorplan-intent", "units": "mm",
            "envelope": {"rect": [0.0, 0.0, SIZE[0], SIZE[1]],
                         "tolerance_mm": 0.5},
            "blocks": [dict(b) for b in blocks]}


def blk(name, refs, zone, exclusive=False, side=None):
    b = {"name": name, "refs": list(refs), "zone": list(zone),
         "tolerance_mm": 0.5}
    if exclusive:
        b["exclusive"] = True
    if side:
        b["side"] = side
    return b


def census(summary, ref):
    return (summary.get('no_pose_census') or {}).get(ref) or {}


def verdict(summary, ref):
    return (summary.get('no_pose_verdict') or {}).get(ref)


def notes(stdout, *must):
    """The seeder's own NOTE lines containing every fragment in `must`.

    Restricted to `NOTE:` lines on purpose. A first draft of this matched any
    line naming the ref and the block, and was satisfied by the run's
    `GRADE ERROR [zone_exclusive] R1 (not a member of 'rf') ...` -- i.e. it
    passed on the UNFIXED engine, reporting the grade's sentence as though it
    were the seeder's disclosure.
    """
    return [ln.strip() for ln in stdout.splitlines()
            if 'NOTE:' in ln and all(m in ln for m in must)]


def dist(a, b):
    return round(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5, 3)


# --------------------------------------------------------------------------
# X -- displaced OUT of the reserved zone, and MINIMALLY, and its control
# --------------------------------------------------------------------------

X_RF = (10.0, 2.0, 18.0, 10.0)      # the reserved zone, owned by X1
X_S = (8.5, 5.5, 9.5, 6.5)          # R1's ANCHOR zone; centre (9.0, 6.0)
X_TARGET = (9.0, 6.0)
X_SEAT = (8.0, 6.0)                 # the theorem, below
X_DISP = 1.0


def _x_board(path):
    # R1: a 4x4 SQUARE courtyard, so the four rotations the search tries are
    # indistinguishable and the pose arithmetic cannot be disturbed by one.
    board(path, [_part('R1', 9.0, 6.0, 2.0, 2.0, 4),
                 _part('X1', 14.0, 6.0, 0.5, 0.5, 2)], size=SIZE)


def arm_X_displaced_and_its_control(wd):
    """THE THEOREM. Block names sort 'rf' < 's', so X1 seats first, at its own
    zone's exact centre (14.0, 6.0).

    R1's zone is 1x1 against a 4x4 courtyard, so `zone_gate` takes its ANCHOR
    branch and constrains the ORIGIN to X_S inflated by tol 0.5, i.e.
    cx in [8, 10] and cy in [5, 7]. The target is the zone centre (9.0, 6.0),
    whose courtyard [7, 4, 11, 8] reaches 1mm into the reserved rect.

    R1 clears X_RF only by leaving it on some axis. `cx - 2 >= 18`,
    `cy + 2 <= 2` and `cy - 2 >= 10` are all outside the anchor box, so the
    ONLY escape is `cx + 2 <= 10`, i.e. cx <= 8.0. The nearest such pose to
    (9.0, 6.0) is (8.0, 6.0), at exactly 1.00mm, and it is unique because the
    constraint does not involve cy at all.

    At cx = 8.0 the courtyard's east edge is EXACTLY the zone's west edge, so
    the overlap area is exactly 0.0. The pose is therefore legal only under a
    strict `> EPS` threshold -- a `>=`, or a tolerance-inflated rect, moves the
    answer and this arm says by how much.
    """
    print("--- X: displaced out of the reserved zone, and minimally")
    it = intent([blk('rf', ['X1'], X_RF, exclusive=True),
                 blk('s', ['R1'], X_S)])
    r, s, out = run(wd, _x_board, it, tag='X')
    check("X: place_seed exits 0 with nothing unseated",
          r.returncode == 0 and s and s.get('unseated') == 0,
          f"rc={r.returncode} unseated={s and s.get('unseated')} "
          f"grade_errors={s and s.get('grade_errors')}")
    if not s:
        return
    p = poses(out)
    ov = overlap(out, 'R1', X_RF)
    check("X: R1's courtyard is clear of the reserved zone",
          ov == 0.0, f"overlap {ov}mm2 with {X_RF}")
    check("X: and it moved EXACTLY the minimum -- 1.000mm, to (8.000, 6.000)",
          p['R1'][:2] == X_SEAT and dist(p['R1'], X_TARGET) == X_DISP,
          f"seat {p['R1'][:2]}, displacement {dist(p['R1'], X_TARGET)}mm "
          f"(theorem: {X_SEAT}, {X_DISP}mm)")
    # The exemption, end to end: the zone's OWN member is still inside it.
    ov_x = overlap(out, 'X1', X_RF)
    check("X: the MEMBER X1 is still seated inside its own exclusive zone",
          ov_x > 0.0, f"X1 overlap {ov_x}mm2")
    check("X: the written board grades clean on zone_exclusive",
          not graded_exclusive_errors(out, it),
          f"{[(v.ref, v.block) for v in graded_exclusive_errors(out, it)]}")
    ok, why = pairwise_legal(out)
    check("X: and the board is pairwise legal", ok, why)

    # ---- the control: the same board, the flag off ----------------------
    it2 = intent([blk('rf', ['X1'], X_RF), blk('s', ['R1'], X_S)])
    r2, s2, out2 = run(wd, _x_board, it2, tag='Xctrl')
    check("X-control: with `exclusive` absent R1 IS seated at its zone centre",
          r2.returncode == 0 and s2 and s2.get('unseated') == 0,
          f"rc={r2.returncode} unseated={s2 and s2.get('unseated')}")
    if s2:
        p2 = poses(out2)
        ov2 = overlap(out2, 'R1', X_RF)
        check("X-control: at (9.000, 6.000), 4.0mm2 INSIDE the rect -- so "
              "arm X is not a board where R1 was never going there",
              p2['R1'][:2] == X_TARGET and ov2 == 4.0,
              f"seat {p2['R1'][:2]}, overlap {ov2}mm2")
        # The sharpest assertion in the file: re-grade the CONTROL board
        # against the arm's EXCLUSIVE intent. If that yields no error, arm X's
        # clean grade was a property of the board and not of the change.
        errs = graded_exclusive_errors(out2, it)
        check("X-control: and that board graded against the EXCLUSIVE intent "
              "yields exactly one zone_exclusive error, naming R1 and 'rf'",
              len(errs) == 1 and errs[0].ref == 'R1' and errs[0].block == 'rf',
              f"{[(v.ref, v.block) for v in errs]}")


# --------------------------------------------------------------------------
# Y -- stranded, and the verdict NAMES the zone
# --------------------------------------------------------------------------

Y_RF = (3.0, 0.0, 17.0, 12.0)       # leaves a 4x4 courtyard nowhere to go
Y_COLD = (17.5, 0.2, 19.5, 2.5)     # binds R1, frees nothing
Y_S = (1.5, 5.5, 2.5, 6.5)          # R1's ANCHOR zone; origin box [1, 5, 3, 7]


def _y_board(path):
    board(path, [_part('R1', 2.0, 6.0, 2.0, 2.0, 4),
                 _part('X1', 10.0, 6.0, 0.5, 0.5, 2),
                 _part('X2', 18.5, 1.35, 0.5, 0.5, 2)], size=SIZE)


def arm_Y_stranded_and_its_controls(wd):
    """A reserved zone that leaves R1 no legal pose ANYWHERE, so it is
    genuinely stranded rather than merely pushed out of its own zone. Y_RF
    spans x in [3, 17]; a 4x4 courtyard clears it only at cx <= 1 or cx >= 19,
    and both put it off the usable board (the inset is
    [0.5, 0.5, 19.5, 11.5]). So the zone stage refuses it AND stage 3's
    unzoned retry refuses it too.

    A SECOND exclusive zone is declared in the far corner. It BINDS R1 -- R1 is
    a stranger to it and it declares no side -- but lifting it frees nothing,
    because Y_RF still refuses every pose. So "name them all" and "name the
    first one" are both wrong answers and the assertion can be on the exact
    set. That is what makes the naming a MEASUREMENT rather than a lookup of
    what was declared.
    """
    print("--- Y: stranded, and the verdict names the zone")
    it = intent([blk('cold', ['X2'], Y_COLD, exclusive=True),
                 blk('rf', ['X1'], Y_RF, exclusive=True),
                 blk('s', ['R1'], Y_S)])
    r, s, out = run(wd, _y_board, it, tag='Y')
    check("Y: R1 is reported unseated, not seated in the reserved zone",
          bool(s) and s.get('unseated_refs') == ['R1'],
          f"unseated_refs={s and s.get('unseated_refs')}")
    if not s:
        return
    # The VERDICT first, and the exit code after it: a run that died for an
    # unrelated reason also exits non-zero, and the two must not read alike.
    v = verdict(s, 'R1')
    check("Y: the verdict is zone_exclusive_blocks, not no_movable_neighbour",
          v == 'zone_exclusive_blocks', f"verdict={v!r}")
    cen = census(s, 'R1')
    freeing = cen.get('zone_exclusive_freeing')
    check("Y: exactly the RESPONSIBLE zone is named",
          sorted(freeing or {}) == ['rf'],
          f"zone_exclusive_freeing={freeing!r}")
    check("Y: the census says 0 poses with it and MORE without it -- a count "
          "from the seat predicate, not a static zone-vs-rect test",
          cen.get('baseline') == 0 and (freeing or {}).get('rf', 0) > 0,
          f"baseline={cen.get('baseline')} freeing={freeing!r}")
    check("Y: and the JOINT channel is present and correctly zero here",
          cen.get('zone_exclusive_joint') == 0,
          f"zone_exclusive_joint={cen.get('zone_exclusive_joint')!r}")
    named = notes(r.stdout, 'R1', 'rf')
    check("Y: the seeder's own NOTE names R1 and 'rf' -- the JSON verdict and "
          "the prose come from one function, so they cannot drift",
          bool(named), named[0] if named else "matching NOTE lines: 0")
    check("Y: and the NOTE does not call it a keep-out (there is no `allow` "
          "list for an exclusive zone -- membership IS the allow list)",
          bool(named) and not any('keep-out' in ln.lower() for ln in named),
          named[0] if named else "no NOTE to check")
    check("Y: place_seed exits 4 -- the seed does not satisfy its intent",
          r.returncode == 4, f"rc={r.returncode}")

    # ---- control 1: the same board, the flags off ------------------------
    it2 = intent([blk('cold', ['X2'], Y_COLD), blk('rf', ['X1'], Y_RF),
                  blk('s', ['R1'], Y_S)])
    r2, s2, out2 = run(wd, _y_board, it2, tag='Yctrl1')
    check("Y-control-1: with the flags off R1 IS seated -- so arm Y is not "
          "just an impossible board",
          bool(s2) and s2.get('unseated') == 0,
          f"rc={r2.returncode} unseated={s2 and s2.get('unseated')}")
    if s2 and s2.get('unseated') == 0:
        ov = overlap(out2, 'R1', Y_RF)
        check("Y-control-1: and it sits inside the rect", ov > 0.0,
              f"overlap {ov}mm2 at {poses(out2)['R1'][:2]}")
        errs = graded_exclusive_errors(out2, it)
        check("Y-control-1: that board graded against the EXCLUSIVE intent "
              "does flag R1",
              any(v.ref == 'R1' for v in errs),
              f"{[(v.ref, v.block) for v in errs]}")

    # ---- control 2: R1 made a MEMBER of the reserved block --------------
    # Without this, "the stranger is refused" is satisfied by a gate that
    # refuses everybody. R1 joins 'rf' and gives up its own block; that block
    # now has two members and therefore jitters, so this asserts SEATED AND
    # INSIDE rather than a pose.
    it3 = intent([blk('cold', ['X2'], Y_COLD, exclusive=True),
                  blk('rf', ['X1', 'R1'], Y_RF, exclusive=True)])
    r3, s3, out3 = run(wd, _y_board, it3, tag='Yctrl2')
    check("Y-control-2: made a MEMBER of 'rf', R1 is seated",
          bool(s3) and s3.get('unseated') == 0,
          f"rc={r3.returncode} unseated={s3 and s3.get('unseated')}")
    if s3 and s3.get('unseated') == 0:
        ov3 = overlap(out3, 'R1', Y_RF)
        check("Y-control-2: inside the very rect that stranded it as a "
              "stranger, and grading clean on zone_exclusive",
              ov3 > 0.0 and not graded_exclusive_errors(out3, it3),
              f"overlap {ov3}mm2, "
              f"{[(v.ref, v.block) for v in graded_exclusive_errors(out3, it3)]}")


# --------------------------------------------------------------------------
# Z -- JOINTLY blocked: neither zone frees a pose alone
# --------------------------------------------------------------------------

Z_LOW = (0.0, 0.0, 20.0, 5.5)       # everything below the gap
Z_HIGH = (0.0, 6.5, 20.0, 12.0)     # everything above it; the gap is 1.0 tall
Z_S = (9.5, 5.5, 10.5, 6.5)         # R1's ANCHOR zone; origin box [9, 5, 11, 7]
Z_SEAT = (10.0, 6.0)


def _z_board(path):
    board(path, [_part('R1', 10.0, 6.0, 2.0, 2.0, 4),
                 _part('OL', 10.0, 2.75, 0.2, 0.2, 2),
                 _part('OH', 10.0, 9.25, 0.2, 0.2, 2)], size=SIZE)


def arm_Z_jointly_blocked(wd):
    """Two DISJOINT exclusive zones that each free NOTHING when lifted alone.

    Nesting them -- #701's own joint fixture -- is impossible here: overlapping
    zones are refused outright as `intent_zone_overlap`. The shape that works
    instead is the whole board split into two bands with a 1.0mm gap between
    them. Two things follow, and BOTH are needed:

      * R1's 4x4 courtyard cannot fit in a 1.0mm gap, so EVERY pose on the
        board touches at least one band. That is what makes R1 genuinely
        stranded rather than merely pushed out of its zone -- stage 3's
        unzoned retry has nowhere to put it either. (A first draft used two
        thin strips over the anchor box alone; the unzoned retry then seated
        R1 elsewhere on the board and the arm tested nothing.)
      * R1's origin is confined to [9, 5, 11, 7], so its courtyard always
        spans y in [cy - 2, cy + 2] with cy in [5, 7] -- which reaches below
        5.5 and above 6.5 at every one of those poses. So each band ALONE
        intrudes on every candidate pose.

    Lifting Z_LOW alone therefore frees nothing (Z_HIGH still crosses every
    pose) and lifting Z_HIGH alone frees nothing either, so the per-zone sweep
    reports `{}`. Without a joint channel the verdict would fall back to
    `no_movable_neighbour`, whose prose blames the outline, the zone or the
    part's own size -- exactly the misleading answer this disclosure exists to
    replace, one rule over from `test_701_keepout_seating.py`'s own joint arm.

    Each band is single-member, so its owner seats at the band's exact centre
    -- (10.0, 2.75) and (10.0, 9.25). Both clear every R1 courtyard by 1.05mm,
    so what refuses R1 is the ZONES and never their owners' courtyards.
    """
    print("--- Z: jointly blocked by two disjoint exclusive strips")
    it = intent([blk('low', ['OL'], Z_LOW, exclusive=True),
                 blk('high', ['OH'], Z_HIGH, exclusive=True),
                 blk('s', ['R1'], Z_S)])
    r, s, out = run(wd, _z_board, it, tag='Z')
    check("Z: R1 is reported unseated",
          bool(s) and s.get('unseated_refs') == ['R1'],
          f"unseated_refs={s and s.get('unseated_refs')}")
    if not s:
        return
    v = verdict(s, 'R1')
    check("Z: the verdict is still zone_exclusive_blocks",
          v == 'zone_exclusive_blocks', f"verdict={v!r}")
    cen = census(s, 'R1')
    check("Z: NO single zone frees a pose -- the per-zone sweep is empty, "
          "asserted exactly rather than as a falsy value",
          cen.get('zone_exclusive_freeing') == {},
          f"zone_exclusive_freeing={cen.get('zone_exclusive_freeing')!r}")
    check("Z: and lifting BOTH frees some -- which is the whole reason the "
          "joint channel exists",
          (cen.get('zone_exclusive_joint') or 0) > 0,
          f"zone_exclusive_joint={cen.get('zone_exclusive_joint')!r}")
    jointly = notes(r.stdout, 'R1', 'JOINTLY')
    check("Z: the NOTE says JOINTLY, and names no single zone falsely",
          bool(jointly), jointly[0] if jointly else "matching NOTE lines: 0")

    # ---- the control ----------------------------------------------------
    it2 = intent([blk('low', ['OL'], Z_LOW), blk('high', ['OH'], Z_HIGH),
                  blk('s', ['R1'], Z_S)])
    r2, s2, out2 = run(wd, _z_board, it2, tag='Zctrl')
    check("Z-control: with both flags off R1 is seated at its zone centre",
          bool(s2) and s2.get('unseated') == 0
          and poses(out2)['R1'][:2] == Z_SEAT,
          f"rc={r2.returncode} unseated={s2 and s2.get('unseated')}, "
          f"seat={poses(out2)['R1'][:2] if s2 else None}")
    if s2 and s2.get('unseated') == 0:
        check("Z-control: and it overlaps BOTH strips",
              overlap(out2, 'R1', Z_LOW) > 0.0
              and overlap(out2, 'R1', Z_HIGH) > 0.0,
              f"low {overlap(out2, 'R1', Z_LOW)}mm2, "
              f"high {overlap(out2, 'R1', Z_HIGH)}mm2")


# --------------------------------------------------------------------------
# S -- the side filter, end to end, with its control
# --------------------------------------------------------------------------

S_RF = (10.0, 1.0, 18.0, 11.0)
S_ZF = (9.3, 3.8, 9.7, 4.2)         # anchor zones, all WEST of S_RF
S_ZB = (9.3, 6.8, 9.7, 7.2)
S_ZT = (9.3, 9.8, 9.7, 10.2)


def _s_board(path):
    board(path, [_part('M1', 14.0, 6.0, 0.5, 0.5, 2),
                 _part('SF', 9.5, 4.0, 1.0, 1.0, 2),
                 _part('SB', 9.5, 7.0, 1.0, 1.0, 2, layer='B.Cu'),
                 _part('ST', 9.5, 10.0, 1.0, 1.0, 2, layer='B.Cu',
                       thru=True)], size=SIZE)


def arm_S_side_filter_and_its_control(wd):
    """`rule_zone_exclusive` reads the SCALAR `part.side`; `rule_keepout` reads
    the SET `part.sides`. So an F-side reserved zone does not bind a B-side
    stranger, and -- the trap -- does not bind a THROUGH-HOLE stranger whose
    BODY is on B either, even though its leads reach F.

    All three strangers get a 0.4x0.4 anchor zone just west of the reserved
    rect, against a 2x2 courtyard: each is aimed at (9.5, y), whose courtyard
    [8.5, 10.5] reaches 0.5mm into S_RF, i.e. 1.00mm2. The zones themselves end
    at x = 9.7 and the rect begins at x = 10.0, so they stay disjoint and the
    intent is satisfiable.

    SF is the control: without it, the two accepts are satisfied by a gate that
    never fires at all. Its escape is `cx + 1 <= 10`, i.e. cx <= 9.0, and the
    anchor box allows cx in [8.8, 10.2] -- so it moves exactly 0.50mm.
    """
    print("--- S: the side filter reads the scalar side, and its control")
    it = intent([blk('rf', ['M1'], S_RF, exclusive=True, side='F'),
                 blk('z_sf', ['SF'], S_ZF),
                 blk('z_sb', ['SB'], S_ZB),
                 blk('z_st', ['ST'], S_ZT)])
    r, s, out = run(wd, _s_board, it, tag='S')
    check("S: place_seed exits 0 with nothing unseated",
          r.returncode == 0 and s and s.get('unseated') == 0,
          f"rc={r.returncode} unseated={s and s.get('unseated')} "
          f"grade_errors={s and s.get('grade_errors')}")
    if not s:
        return
    p = poses(out)
    ov_sb = overlap(out, 'SB', S_RF)
    check("S: the B-side SMD stranger is seated at its own zone centre, "
          "INSIDE the F-side reserved rect",
          p['SB'][:2] == (9.5, 7.0) and ov_sb > 0.0,
          f"seat {p['SB'][:2]}, overlap {ov_sb}mm2")
    ov_st = overlap(out, 'ST', S_RF)
    check("S: and so is the THROUGH-HOLE stranger whose body is on B -- the "
          "filter is the scalar side, not the occupied set",
          p['ST'][:2] == (9.5, 10.0) and ov_st > 0.0,
          f"seat {p['ST'][:2]}, overlap {ov_st}mm2")
    ov_sf = overlap(out, 'SF', S_RF)
    check("S-control: the F-side stranger IS refused and moved exactly "
          "0.500mm to (9.000, 4.000) -- without this the two accepts above "
          "are satisfied by an unarmed gate",
          ov_sf == 0.0 and p['SF'][:2] == (9.0, 4.0),
          f"seat {p['SF'][:2]}, overlap {ov_sf}mm2, "
          f"displacement {dist(p['SF'], (9.5, 4.0))}mm")
    check("S: the written board grades clean on zone_exclusive",
          not graded_exclusive_errors(out, it),
          f"{[(v.ref, v.block) for v in graded_exclusive_errors(out, it)]}")
    ok, why = pairwise_legal(out)
    check("S: and the board is pairwise legal", ok, why)


# --------------------------------------------------------------------------
# E -- the EDGE path, which bypasses pose_ok by design
# --------------------------------------------------------------------------

E_SIZE = (20.0, 14.0)
E_RF = (7.0, 10.0, 13.0, 14.0)      # a reserved band across the south edge


def _e_board(path):
    board(path, [_part('J1', 10.0, 7.0, 2.0, 1.5, 2, pad_y=-1.0),
                 _part('M1', 3.0, 12.5, 0.4, 0.4, 2)], size=E_SIZE)


def _e_intent(exclusive):
    return {"schema": 1, "kind": "floorplan-intent", "units": "mm",
            "envelope": {"rect": [0.0, 0.0, E_SIZE[0], E_SIZE[1]],
                         "tolerance_mm": 0.5},
            "blocks": [{"name": "b", "refs": ["J1"]},
                       dict({"name": "rf", "refs": ["M1"],
                             "zone": list(E_RF), "tolerance_mm": 0.5},
                            **({"exclusive": True} if exclusive else {}))],
            "edge_connectors": [{"ref": "J1", "edge": "south",
                                 "overhang_mm": {"min": 0.0, "max": 1.0}}]}


def arm_E_edge_seat_and_its_control(wd):
    """`_seat_edge` and stage 1 use `edge_seat_ok`, NOT `pose_ok` -- an edge
    connector overhangs by design, so full containment is the wrong predicate
    and that path deliberately bypasses the one #797's conjunct sits in.

    A change that guarded only `pose_ok` leaves this red, and nothing else in
    either #797 file catches it. That is the whole reason this arm exists, and
    it is the same argument `tests/test_701_keepout_seating.py`'s own edge arm
    makes for keep-outs.

    The control is the identical board with the flag off, showing J1 landing
    INSIDE the rect the exclusive zone will cover -- without it, "J1 is out of
    the rect" is satisfied by a board where it was never going there.
    """
    print("--- E: the edge path bypasses pose_ok, so it needs its own conjunct")
    r0, s0, out0 = _run_e(wd, False, 'Ectrl')
    check("E-control: the edge connector is seated with the flag off",
          bool(s0) and s0.get('unseated') == 0,
          f"rc={r0.returncode} unseated={s0 and s0.get('unseated')}")
    if not (s0 and s0.get('unseated') == 0):
        return
    ov0 = overlap(out0, 'J1', E_RF)
    check("E-control: and it lands INSIDE the rect the zone will reserve, so "
          "arm E's assertion is not vacuous",
          ov0 > 0.0, f"overlap {ov0}mm2 at {poses(out0)['J1']}")

    r1, s1, out1 = _run_e(wd, True, 'E')
    check("E: with the zone reserved the edge seat is refused there",
          bool(s1), f"rc={r1.returncode}")
    if not s1:
        return
    ov1 = overlap(out1, 'J1', E_RF)
    check("E: J1's courtyard is clear of the reserved band",
          ov1 == 0.0, f"overlap {ov1}mm2 at {poses(out1)['J1']}")
    check("E: and the written board grades clean on zone_exclusive",
          not graded_exclusive_errors(out1, _e_intent(True)),
          f"{[(v.ref, v.block) for v in graded_exclusive_errors(out1, _e_intent(True))]}")


def _run_e(wd, exclusive, tag):
    return run(wd, _e_board, _e_intent(exclusive), tag=tag)


# --------------------------------------------------------------------------
# P -- the post-polish repair, which is what `_repairable` is for
# --------------------------------------------------------------------------

#: Fault injection for arm P-a, the rig `tests/test_701_keepout_seating.py`
#: uses for the same reason. The quench on a fixture this small happens to
#: leave the part alone, so a plain end-to-end run never reaches the repair and
#: would pass with `zone_exclusive` deleted from `_repairable` -- a test that
#: passes in both directions. This forces the polish to return the one move
#: that walks R1 back into the reserved zone, which is what a MONOTONE gate
#: cannot prevent for a part that is already clean only because the seeder put
#: it there.
_INJECT = '''
import placement.quench as _q
_real = _q.quench


def _walk_it_in(*a, **kw):
    _real(*a, **kw)
    return [{'reference': 'R1', 'new_x': 9.0, 'new_y': 6.0,
             'new_rotation': 0.0}]


_q.quench = _walk_it_in
'''


def arm_P_polish_repair(wd):
    """`place_seed`'s polish runs a quench, and #702 gates it per move -- but
    MONOTONICALLY, so it holds a clean seed clean and never REPAIRS a breach
    that reaches the written board by another route. `_repairable` is the net
    under that, and #797 adds `zone_exclusive` to it.

    P-a injects exactly that fault and requires the repair to undo it. P-b is
    the ordinary polished run, which the `--no-polish` arms above cannot see
    because they never run the quench at all.
    """
    print("--- P: the post-polish repair, and the ordinary polished run")
    it = intent([blk('rf', ['X1'], X_RF, exclusive=True),
                 blk('s', ['R1'], X_S)])

    # ---- P-a: the polish IS forced to break it --------------------------
    inj = os.path.join(wd, 'inj797')
    os.makedirs(inj, exist_ok=True)
    with open(os.path.join(inj, 'sitecustomize.py'), 'w',
              encoding='utf-8') as f:
        f.write(_INJECT)
    import json as _json
    bpath = os.path.join(wd, 'Pa-in.kicad_pcb')
    ipath = os.path.join(wd, 'Pa-fp.json')
    out_a = os.path.join(wd, 'Pa-out.kicad_pcb')
    _x_board(bpath)
    with open(ipath, 'w', encoding='utf-8') as f:
        _json.dump(it, f)
    env = dict(os.environ, PYTHONHASHSEED='0', PYTHONIOENCODING='utf-8',
               PYTHONPATH=inj + os.pathsep
               + os.pathsep.join(os.path.join(REPO, d) for d in
                                 ('py_placer', 'py_router', 'py_tools')))
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join('py_placer', 'place_seed.py'), bpath, out_a,
         '--intent', ipath, '--clearance', '0.2',
         '--board-edge-clearance', '0.5', '--force'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=REPO, timeout=900, env=env)
    injected = 'polish walked' in r.stdout
    check("P-a: the injected polish really did break the board, so this arm "
          "is not vacuous", injected, f"repair line present: {injected}")
    if injected:
        check("P-a: the repair names `zone_exclusive` as what it repaired -- "
              "this is the assertion that reverting `_repairable` turns red",
              'zone_exclusive' in r.stdout,
              f"names zone_exclusive: {'zone_exclusive' in r.stdout}")
        ov = overlap(out_a, 'R1', X_RF)
        check("P-a: and R1 is back out of the reserved zone on the WRITTEN "
              "board", ov == 0.0,
              f"overlap {ov}mm2 at {poses(out_a)['R1'][:2]}")
        check("P-a: place_seed exits 0 after repairing itself",
              r.returncode == 0, f"rc={r.returncode}")

    # ---- P-b: the ordinary polished run ---------------------------------
    r2, s2, out2 = run(wd, _x_board, it, tag='Pb', polish=True)
    check("P-b: the polished run exits 0 with no grade errors",
          r2.returncode == 0 and s2 and s2.get('grade_errors') == 0,
          f"rc={r2.returncode} grade_errors={s2 and s2.get('grade_errors')}")
    if not s2:
        return
    ov2 = overlap(out2, 'R1', X_RF)
    check("P-b: and R1 is still clear of the reserved zone after the quench",
          ov2 == 0.0, f"overlap {ov2}mm2 at {poses(out2)['R1'][:2]}")


def main():
    with tempfile.TemporaryDirectory() as wd:
        arm_X_displaced_and_its_control(wd)
        arm_Y_stranded_and_its_controls(wd)
        arm_Z_jointly_blocked(wd)
        arm_S_side_filter_and_its_control(wd)
        arm_E_edge_seat_and_its_control(wd)
        arm_P_polish_repair(wd)
    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
