#!/usr/bin/env python3
"""Generate an initial placement for an UNPLACED board from its floorplan intent.

Usage:
  python place_seed.py input.kicad_pcb output.kicad_pcb --intent floorplan.json

The placement stack refines and deliberately does not place from scratch
UNAIDED -- but a declared intent carries the constraints a from-scratch run
lacks: zones, edge bands, locks, decap rules. This tool turns that intent into
a legal starting placement (see placement/seeder.py for exactly what each
construct becomes), stamps the intent's must_lock refs `(locked yes)` into the
output, runs a quench polish over the free parts, and then GRADES its own
output against the same intent -- a seed that fails the intent it was built
from is a defect, not a result.

Different --seed values produce genuinely different legal seeds (packing order
and target jitter); the same seed reproduces byte for byte. Compose with
place_portfolio.py to diversify and rank what this emits.

Exit codes: 0 seeded and graded clean; 2 bad arguments; 3 the board cannot be
seeded (no Edge.Cuts outline -- the outline is spec-owned and will not be
invented -- or the board is already placed / carries copper); 4 the seed was
written but parts could not be seated or the intent grade has errors ON
PARTS THE SEED PLACED. A grade error on a part the seed was told not to move
-- `(locked yes)` in the file, or matched by the intent's `must_lock` -- is
printed and counted in `grade_errors_pinned`, and does not fail the gate:
it is a contradiction between the board and the intent, which only their
author can settle. Measured, run 27: a fixed USB socket declared
`along_edge: center` within 0.6 mm sits 1.75 mm off centre, and every one of
ten seeds failed on it, so nothing the seeder did could ever be ranked.

Every JSON_SUMMARY also carries `connector_requirements` (#974): which declared
edge-connector requirements were graded and on what basis, which were not
measured, and the connector errors on each side of the pinned split. It
reports; it never withholds the board and never changes an exit code.

The pad conflicts in the written board are reported in three buckets that
PARTITION `pad_conflicts_after` (#982): `pad_conflicts_seeded` is the seed's
own, a pair with a part it moved on at least one side; `pad_conflicts_unseated`
is a pair against a part it could NOT seat, which was written at the pose it
came in with, so whether anything lands on it is incidental to the seed;
`pad_conflicts_inherited` is the board's own. Only the first fails the gate,
and only when nothing is unseated -- an unseated part already fails it for a
better reason. All three are named on the console, because every one of them
is copper a fab will see.

It also carries `edge_floor_fallback` (#975): the declared edge connectors whose
seat leaves pad copper inside the board-edge floor because no in-band seat
clears it, by ref, with the pads and why the seat could not move. An edge seat
prefers a pose that clears the floor and otherwise keeps the one the pad-centre
test accepts, since an unseated connector is an unrouted one. Like
`connector_requirements`, it never changes an exit code.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement'], 'kind': 'actor'}

import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
import argparse
import json
import os
import sys


def _split_pinned(graded, output_file, intent):
    """(own, pinned): the grade errors the seed is answerable for, and the
    ones that sit on a part it was told not to move -- `(locked yes)` in the
    written file (which is where `must_lock` lands after stamping) or matched
    by a `must_lock` pattern. Block-level findings (no `ref`) are always own.
    """
    import fnmatch
    from kicad_parser import parse_kicad_pcb
    locked = {r for r, f in parse_kicad_pcb(output_file).footprints.items()
              if getattr(f, 'locked', False)}
    pats = tuple(getattr(intent, 'must_lock', ()) or ())

    def pinned(v):
        ref = getattr(v, 'ref', None)
        return bool(ref) and (ref in locked
                              or any(fnmatch.fnmatch(ref, p) for p in pats))
    return ([v for v in graded.errors if not pinned(v)],
            [v for v in graded.errors if pinned(v)])


def _print_grade(own, pinned):
    """The gate's errors, then the set-aside ones NAMED -- never silent."""
    for v in own[:10]:
        print(f"  GRADE ERROR [{v.rule}] {v.message}")
    if pinned:
        by = {}
        for v in pinned:
            by.setdefault(v.ref, set()).add(v.rule)
        print(f"  {len(pinned)} grade error(s) sit on locked part(s) the seed "
              f"did not place -- "
              + '; '.join(f"{r}: {', '.join(sorted(rs))}"
                          for r, rs in sorted(by.items()))
              + ". Reported, not the seed's failure: a pinned pose that "
              "breaks a declared clause is a contradiction between the board "
              "and the intent, and only their author can say which is wrong.")
        for v in pinned[:10]:
            print(f"  GRADE ERROR (pinned) [{v.rule}] {v.message}")


def split_pad_pairs(worst, seeded, unseated):
    """Split the written board's pad pairs into the seed's and the unseated's.

    Returns `(mine, against_unseated)`, both in `worst` order. A pair is the
    seed's when either member is a ref it MOVED, and lands in the second list
    instead when the other member is a ref it could not seat. Pairs with
    neither member moved are in neither list: they are the board's own, counted
    from the total by the caller -- and a pair between TWO unseated parts is one
    of those, since the seed moved neither of them.

    #982. A part the seed cannot seat keeps the pose it came in with, and THAT
    pose is what gets written -- `placements` has no row for it and the writer
    leaves its block alone. Later stages pass the pile as `exclude`
    (`seeder._try_place`: "the pile they still form at their meaningless input
    coordinates must not veto real poses"), so they pack onto that copper, and
    the pair then reaches the count through the partner the seed DID move. It
    is real copper -- on ulx3s SEED 1, `py_tools/check_assembly.py` grades
    `H4 <-> J1` a 1.5089 mm2 `pad_intersection`, BLOCKING, and the board NOT
    BUILDABLE -- so it stays named and counted. A pair in this bucket is not
    always that severe: ulx3s seed 0's H4-J2 is a 0.191 mm graze on a board
    `check_assembly` still grades buildable. What it is not is the seed's answer
    for the parts it placed: whether a hole lands on an unseated part is
    incidental to the seed, and the count swung 0 -> 1 at seed 0 and 5 -> 8 over
    seeds 0..9 when AUDIO1 was seated 0.386 mm further inward for a board-edge
    copper fix (#975's A/B; a sham nudge of the same size on the unmodified
    engine reproduces it, so the cause is the displacement, not the fix). Its
    own bucket keeps `pad_conflicts_seeded` a number the seed can be held to.

    How widespread this is, measured over seven boards at seed 0 (19 charged
    pairs, 17 of them against a part that could not be seated) plus ulx3s at ten
    seeds (5 of 5): ulx3s 5 of 5, rp2350 15 of 16, orangecrab 2 of 3, and
    nothing at all on the four boards that seat everything. On rp2350 those
    fifteen are ONE unseated part, U6, whose 75 pads sit at the designer's pose.
    Every number here is re-measured by `tests/measure_982_unseated_pairs.py`,
    which prepares its boards the way the issue does; read that file's
    docstring for the recipe, because these numbers do NOT reproduce from a
    board prepared some other way.

    The exit code cannot move by this split: `gate_reason` returns the intent
    arm whenever anything is unseated, and when nothing is unseated the second
    list is empty by construction. A ref that is somehow BOTH moved and
    unseated is charged to the seed, the direction that keeps the gate honest;
    the seeder keeps the two disjoint, so this is a tie-break nothing reaches.

    The alternative reading -- treat the written pose as an obstacle for later
    stages -- was prototyped and MEASURED WORSE. It unseats parts that were
    seated: ulx3s 20 -> 25 unseated over ten seeds (H4 at seeds 1 and 4, H1, H2
    and H3 at seed 7), rp2350 1 -> 3, and hole conflicts up on two boards. It
    does not even buy stability, because at ulx3s seed 7 it swapped three pairs
    against J1/J2 for three against the H1/H2/H3 it had just unseated -- the
    seat predicate is courtyard-level, so an unseated part's whole courtyard
    (14.5 x 52 mm for ulx3s J1/J2) becomes a keep-out. It also ran about 20%
    slower. Trading seats for a cleaner count is the wrong way round when an
    unseated part's nets cannot be routed at all, so the search is left alone
    here; this judgement is the branch's, not a rule quoted from CLAUDE.md.
    """
    if isinstance(unseated, str) or isinstance(seeded, str):
        # A bare ref would split by CHARACTER and mis-sort every pair in
        # silence, which is the kind of thing a summary key hides for a year.
        raise TypeError('seeded and unseated are collections of refs, not a ref')
    unseated, seeded = set(unseated), set(seeded)
    mine, against_unseated = [], []
    for w in worst:
        if not (w[0] in seeded or w[1] in seeded):
            continue
        charged = {w[0], w[1]} & seeded
        (against_unseated if (unseated & {w[0], w[1]}) - charged
         else mine).append(w)
    return mine, against_unseated


def gate_reason(unseated, own, my_pads, hole_delta):
    """The one stderr line that says WHY this seed did not pass its gate.

    Two failures reach exit 4 and they are not the same failure, so the line
    names which: a seed that misses the document it was built from, and a
    seed that satisfies that document and still leaves copper that cannot be
    assembled. `None` when nothing fired -- the caller returns 0.

    The second arm names ONLY the channel that actually fired. A hole
    conflict has no pair to look at (`grade_pad_legality` counts holes
    without recording the pair, which is exactly why it is judged on the
    DELTA), so a hole-only refusal that said "pads ... see the pairs above"
    named the wrong channel and pointed at output that is not printed in
    that case.
    """
    if not (unseated or own or my_pads or hole_delta):
        return None
    tail = " It was still written, for inspection."
    if unseated or own:
        return ("place_seed: the seed does NOT satisfy its intent -- see the "
                "errors above." + tail)
    ch = []
    if my_pads:
        ch.append("pads closer than their clearance (the pairs are named "
                  "above)")
    if hole_delta:
        ch.append(f"{hole_delta} hole conflict(s) the board did not come in "
                  f"with")
    return ("place_seed: the seed satisfies its intent but leaves "
            + " and ".join(ch) + "." + tail)


def main():
    import routing_defaults as defaults
    from placement.cli_gates import add_intent_arg

    p = argparse.ArgumentParser(
        description="Intent-driven initial placement for an unplaced board.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog="""
Examples:
  python place_seed.py board.kicad_pcb seed.kicad_pcb --intent floorplan.json
  python place_seed.py board.kicad_pcb seed.kicad_pcb --intent fp.json --seed 3
""")
    p.add_argument("input_file", help="Input KiCad PCB (unplaced parts + outline)")
    p.add_argument("output_file", help="Output board with the seeded placement")
    add_intent_arg(p, required=True, extra=(
        "Here it is also the CONSTRUCTION source for the seed and the "
        "acceptance gate the emitted seed is graded against."))
    p.add_argument("--seed", type=int, default=0,
                   help="Packing-order/jitter seed; same seed reproduces byte "
                        "for byte (default: 0)")
    p.add_argument("--group-by", default="auto",
                   help="Block sources for resolving the intent's `group` "
                        "references (default: auto = kicad,sheet)")
    p.add_argument("--ignore-nets", nargs="+", default=None, metavar="NET",
                   help="Net patterns excluded from the polish's airwire "
                        "scoring (plane-routed rails)")
    p.add_argument("--clearance", type=float, default=None)
    p.add_argument("--board-edge-clearance", type=float, default=None)
    p.add_argument("--grid-step", type=float, default=defaults.GRID_STEP)
    p.add_argument("--max-displacement", type=float, default=3.0,
                   help="Polish displacement cap in mm (default: 3.0)")
    p.add_argument("--no-polish", action="store_true",
                   help="Skip the quench polish; emit the raw packed seed")
    p.add_argument("--corridor-weight", type=float, default=0.0, metavar="W",
                   help="Price the length each foreign airwire cuts through "
                        "the intent's health.bus_corridors during the polish, "
                        "at W per mm. 0 = off (default). See "
                        "place_portfolio.py --corridor-weight")
    p.add_argument("--force", action="store_true",
                   help="Re-seed a board that already looks placed. The "
                        "existing placement is DISCARDED; to explore around "
                        "it instead, use place_portfolio.py")
    p.add_argument("--anchors-first", action="store_true",
                   help="Seed the ANCHOR tier (pad-extent >= the P75 "
                        "threshold, the same tiering reconstruct uses) by "
                        "descending extent BEFORE any small part -- the "
                        "default queue is pin-count descending, which seeds "
                        "a large low-pin connector late, after the smalls "
                        "claimed its space. The smalls are parked as "
                        "non-obstacles either way (the existing exclude "
                        "mechanism); this changes only WHO goes first "
                        "(run-4 C)")
    p.add_argument("--rotate-by-facing", action="store_true",
                   help="Among the rotations that fit, seat the one that leaves "
                        "the fewest connected pads on a row facing the board "
                        "outline with nothing beyond (placement.edge_facing, "
                        "the same number placement_score reports). OFF by "
                        "default, and measured: tests/test_placement_ab.py "
                        "REJECTED it as a default on three boards (fewer "
                        "pads face the edge, more crossings and pin-order "
                        "inversions). Opt in when that trade is the one you "
                        "want; a tie keeps the input rotation first.")
    p.add_argument("--evict-depth", type=int, default=0, choices=(0, 1, 2),
                   metavar="N",
                   help="Eviction rung (#630, #699). At every depth a part "
                        "with NO legal pose gets a census of its seated "
                        "neighbours (JSON_SUMMARY no_pose_blockers: how many "
                        "poses lifting each one would free). 0 (default) "
                        "moves nothing. 1 evicts the neighbour that frees "
                        "the most, seats the part, then re-seats the "
                        "neighbour (inside its own zone) with the part in "
                        "place; the trade is kept only if both seats are "
                        "legal against every seated part and the seated "
                        "board's overlap count did not rise, else both parts "
                        "are put back and the revert is recorded. 2 also "
                        "censuses PAIRS when no single lift frees a pose, "
                        "and trades the best pair under the same rule -- at "
                        "most 16 of the pairs its 8 candidates form, so it "
                        "costs nothing on a part a single lift already "
                        "solves. APPLIES TO --reseat TOO, and there it "
                        "relaxes that pass's contract: --reseat normally "
                        "holds every part outside its scope fixed, and a "
                        "depth >= 1 lets it trade one out. Those parts are "
                        "named in the JSON's `evicted` and in a NOTE, and "
                        "the pass additionally refuses any trade that raised "
                        "the board's stack count or overlap area. Parts "
                        "locked in the file or by the intent's must_lock, "
                        "and declared edge connectors, are never evicted; a "
                        "blocker's own blocker is not chased, at either "
                        "depth, and there is one trade per part. Only fires "
                        "on a part that was going to be reported unseated. "
                        "Opt-in until an A/B row on three boards exists")
    p.add_argument("--anchor-rounds", type=int, default=1,
                   help="With --anchors-first: gated re-seat passes after "
                        "the first full placement (default 1 = none). Each "
                        "round re-seats anchors then smalls at their partner "
                        "centroids over the FULL placement and keeps the "
                        "round only if the legality/hpwl gate tuple does not "
                        "worsen; stops early when a round moves nothing")
    p.add_argument("--repair", action="store_true",
                   help="Violation-driven minimal-move repair of a PLACED "
                        "board: only parts violating the intent or pad/hole "
                        "legality move, worst first, each seated nearest its "
                        "current pose with an escalating displacement cap. "
                        "The opposite contract of --force")
    p.add_argument("--reseat", nargs="*", default=None, metavar="REF",
                   help="LIFT the named parts and re-seat them FROM SCRATCH "
                        "at their net centroids, holding every other part "
                        "fixed as an obstacle. With no REF the scope is the "
                        "off-outline pad-CENTRE census "
                        "(reconstruct.damage_witnesses), which is zero on all "
                        "33 corpus boards -- so a bare --reseat on a healthy "
                        "board is a no-op that exits 0. Unlike --repair, the "
                        "part's CURRENT POSE IS NOT THE SEARCH CENTRE: a part "
                        "30mm from where it belongs carries no information "
                        "about where it belongs, and --repair's cap ladder "
                        "tops out at 5mm from the wrong centre. REF accepts "
                        "fnmatch globs. Any edge_connector declaration on a "
                        "scope ref is DROPPED (the band was measured off the "
                        "pose being discarded). Composes with --repair and "
                        "runs BEFORE it. Judge it on witnesses_after, not on "
                        "how far anything moved")
    p.add_argument("--reseat-region", nargs=4, type=float, action="append",
                   default=None, metavar=("X0", "Y0", "X1", "Y1"),
                   help="Name the reseat scope by GEOMETRY instead of by ref: "
                        "every part with a pad in the rectangle X0 Y0 X1 Y1 "
                        "(board mm, half-open on X1/Y1) joins the scope. "
                        "Repeatable; unions with any --reseat REF. This is "
                        "#459's 'a way to name a REGION rather than a ref "
                        "list', whose other half -- a gate that can accept an "
                        "on-board part -- landed as #698. "
                        "`check_pockets` prints a ready-made rectangle for "
                        "its top cold region. IT IS SCOPE NAMING, NOT AIMING: "
                        "nothing in the seeder aims at the rectangle you pass "
                        "-- a lifted part goes to its declared zone, its edge "
                        "band, its owner's pin cluster if it is a decap, else "
                        "its net centroid (else the board centre when it has "
                        "no placed partner) -- so passing an EMPTY region "
                        "moves nothing INTO it. Declaring the rectangle as an "
                        "intent block `zone` is what makes it a destination. "
                        "Resolves "
                        "through placement.utility.refs_in_rect, the same "
                        "call check_pockets names its windows with, so the "
                        "rectangle cannot mean two different sets of parts")
    p.add_argument("--reseat-min-gain", type=float, default=0.0, metavar="MM",
                   help="With --reseat REF (an EXPLICIT scope): the smallest "
                        "wirelength win, in mm, that counts as a re-seat. 0 "
                        "(the default) means any strict win. It gates the "
                        "MILLIMETRE basis only -- the scope's own HPWL, which "
                        "is where a sideways shuffle can score. The "
                        "intent-violation, blocking-pair and stack bases are "
                        "COUNTS, whose smallest meaningful gain is one whole "
                        "defect: one number compared against both currencies "
                        "would be asserting an exchange rate between half a "
                        "millimetre of wire and half a keep-out violation. "
                        "Inert on the AUTO scope (bare --reseat), whose rule "
                        "is unchanged. JSON_SUMMARY accept_basis reports every "
                        "basis, whether this applied to it, and by how much "
                        "each one missed")
    p.add_argument("--dry-run", action="store_true",
                   help="With --repair/--reseat: print the move list and "
                        "grades, write nothing")
    args = p.parse_args()

    # Run-7 S1 / run-13 F6: unset floors come from the BOARD, not a constant.
    # A fixed 0.25 on a board declaring 0.2 measured 34% more shortfall and
    # double the oob count -- and these tools VETO candidate moves on it, so a
    # wrong floor steers the search, it does not merely mis-report.
    from list_nets import board_floor_knobs
    args.clearance, args.board_edge_clearance, _knobs = board_floor_knobs(
        args.input_file, args.clearance, args.board_edge_clearance)
    print(f"legality at clearance {args.clearance} "
          f"({_knobs['clearance']['source']}), edge {args.board_edge_clearance} "
          f"({_knobs['board_edge_clearance']['source']})")
    # #459: a region is an EXPLICIT scope that stands on its own, so it turns
    # --reseat on without the flag being typed. Normalised before every check
    # below, so `--reseat-region ... --dry-run` is not rejected by a rule that
    # only knows about --reseat.
    if args.reseat_region:
        for _r in args.reseat_region:
            if _r[2] <= _r[0] or _r[3] <= _r[1]:
                p.error("--reseat-region takes X0 Y0 X1 Y1 with X1>X0 and "
                        "Y1>Y0 (board mm); got %g %g %g %g" % tuple(_r))
        if args.reseat == []:
            # #698 gave the two scopes DIFFERENT acceptance policies: an
            # explicit scope is graded on its own terms, the AUTO damage-
            # witness scope on 'the off-board amount strictly improved'.
            # Silently picking one of the two is how that distinction gets
            # lost, so a bare --reseat beside a region is a usage error rather
            # than a quiet merge.
            p.error("--reseat-region is an EXPLICIT scope and a bare --reseat "
                    "is the AUTO damage-witness scope; #698 grades them by "
                    "different rules, so they do not combine. Use "
                    "--reseat-region on its own (it implies an explicit "
                    "--reseat), or name refs: --reseat REF ... "
                    "--reseat-region X0 Y0 X1 Y1")
        if args.reseat is None:
            args.reseat = []          # explicit; the refs come from the rects
    if (args.repair or args.reseat is not None) and args.force:
        p.error("--repair/--reseat and --force are mutually exclusive (they "
                "move only the parts that need it; force re-derives "
                "everything)")
    if args.dry_run and not (args.repair or args.reseat is not None):
        p.error("--dry-run only applies to --repair / --reseat")
    if args.reseat_min_gain and args.reseat is None:
        p.error("--reseat-min-gain only applies to --reseat")
    if args.reseat_min_gain < 0:
        p.error("--reseat-min-gain is a magnitude in mm; negative is not a "
                "looser threshold, it is a typo")
    if args.reseat_min_gain and args.reseat == [] and not args.reseat_region:
        # An inert knob says so, rather than reading as a threshold that
        # happened to find nothing wrong (cli_gates' own disclosure rule).
        print("--reseat-min-gain is inert on the AUTO scope: that rule is "
              "'the off-board amount strictly improved', which this does not "
              "gate", file=sys.stderr)

    try:
        from redo_record import record_invocation
        record_invocation()
    except Exception:
        pass

    import random
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan, seeder
    from placement.groups import GroupError, parse_sources
    from placement.placement_state import UNPLACED_EXIT, assess_placement
    from placement.portfolio import copy_siblings
    from placement.writer import write_placed_output

    try:
        sources = parse_sources(args.group_by)
    except GroupError as exc:
        p.error(str(exc))
    try:
        intent = floorplan.load_intent(args.intent)
    except (OSError, ValueError) as exc:
        print(f"cannot load intent {args.intent}: {exc}", file=sys.stderr)
        return 2

    print(f"Loading {args.input_file}...")
    pcb = parse_kicad_pcb(args.input_file)
    if pcb.board_info.board_bounds is None:
        print("place_seed: this board has no Edge.Cuts outline. The outline "
              "is spec-owned -- draw it (or have the repo's seeder write it "
              "from the spec) before seeding a placement.", file=sys.stderr)
        return UNPLACED_EXIT
    st = assess_placement(pcb, args.input_file)
    if st.has_copper:
        print(f"place_seed: this board carries {st.segments} segment(s) and "
              f"{st.vias} via(s); seeding moves footprints and would strand "
              f"every track. Seed the unrouted board.", file=sys.stderr)
        return UNPLACED_EXIT
    if args.repair or args.reseat is not None:
        import math as _math
        import tempfile
        if st.unplaced:
            print("place_seed: --repair/--reseat need a PLACED board (this "
                  "one is unplaced -- seed it instead).", file=sys.stderr)
            return UNPLACED_EXIT
        summary = {'dry_run': args.dry_run,
                   'output': None if args.dry_run else args.output_file,
                   # #975: filled by whichever pass seats an edge part below.
                   'edge_floor_fallback': {}}

        # Both passes stage into a temp dir and the finished board is copied
        # to the output path once, at the end. That keeps --dry-run honest
        # (--reseat then --repair previews the repair against the RE-SEATED
        # board, which is what the real run would do) and keeps the output
        # path from ever holding a half-finished result (run-7 A11).
        _stage = tempfile.TemporaryDirectory()
        cur, cur_pcb = args.input_file, pcb
        exit_rc = 0
        # Every move either pass applied, keyed by ref, later passes winning
        # key by key. The staged writes happen outside any regime, so this --
        # not the temp files -- is what the delivery row claims (#973).
        delivered_moves = {}

        def _advance(moves, tag):
            """Apply `moves` onto a fresh staged board; advances cur/cur_pcb."""
            nonlocal cur, cur_pcb
            if not moves:
                return
            nxt = os.path.join(_stage.name, f'{tag}.kicad_pcb')
            write_placed_output(cur, nxt, moves)
            copy_siblings(cur, nxt)
            cur = nxt
            cur_pcb = parse_kicad_pcb(cur)
            from placement.provenance import accumulate_moves
            accumulate_moves(delivered_moves, moves)

        reseat = None
        if args.reseat is not None:
            # `nargs='*'`: bare --reseat is [] and means AUTO scope; None is
            # the flag being absent.
            _refs = list(args.reseat or ())
            _selector = None
            if args.reseat_region:
                # #459's "a way to name a REGION rather than a ref list".
                # Resolved HERE, at the CLI layer, into the ordinary `refs`
                # argument -- never by inventing a new scope_source. The
                # acceptance policy in seeder.reseat_accept switches on
                # `scope_source != 'explicit'` by string equality, so a
                # 'region:...' source would drop every region re-seat into the
                # auto:oob-strict policy, which a legal on-board part can
                # never satisfy: the pass would refuse everything and look
                # broken rather than wrong.
                #
                # refs_in_rect is the same call check_pockets names its own
                # windows with, so the rectangle it prints and the rectangle
                # this lifts cannot mean two different sets of parts.
                from placement.utility import refs_in_rect
                _hits = []
                #: A typed --reseat opts into glob semantics; a GEOMETRIC
                #: selection did not. `reseat_scope` runs every ref through
                #: fnmatch, so a literal reference carrying glob
                #: metacharacters -- `D[1]` -- would resolve to `D1` and then
                #: report "matches no reference on this board". Escape them.
                def _literal(ref):
                    return ''.join('[%s]' % c if c in '*?[]' else c
                                   for c in ref)
                for _r in args.reseat_region:
                    _in = refs_in_rect(cur_pcb, tuple(_r))
                    print("  region [%g,%g]-[%g,%g]: %d part(s)%s"
                          % (_r[0], _r[1], _r[2], _r[3], len(_in),
                             ('  ' + ', '.join(_in[:12])
                              + (' ...' if len(_in) > 12 else ''))
                             if _in else ''))
                    _hits.extend(_in)
                _region_refs = sorted(set(_hits))
                if not _region_refs:
                    # A result, not a failure. It goes THROUGH reseat_scope on
                    # the explicit branch rather than returning here: an early
                    # return skipped the output board, the JSON summary and the
                    # whole --repair pass, so `--repair --reseat-region <empty>`
                    # exited 0 having produced nothing. `--reseat ZZ99` is the
                    # same event -- an explicit scope that resolved to zero
                    # refs -- and seeder's own `_empty()` already handles it,
                    # returning the full schema so a reader can tell "the
                    # census found nothing" from "no census ran".
                    print("  region(s) contain no part; the scope is empty. "
                          "This is a RESULT, not a failure -- an empty region "
                          "is exactly what check_pockets ranks.")
                _refs.extend(_literal(r) for r in _region_refs)
                _selector = 'region:' + ';'.join(
                    # Full precision, not %g: this string is the audit trail
                    # for which rectangle was resolved, and 6 significant
                    # digits turns 142.8125 into 142.812, which need not
                    # re-resolve the same parts.
                    '%r,%r,%r,%r' % tuple(_r) for _r in args.reseat_region)
                _refs = sorted(set(_refs))
            reseat = seeder.reseat_scope(
                cur_pcb, cur, intent,
                # `[]` is NOT `None`: an explicit scope that resolved to
                # nothing must stay explicit, because `reseat_accept` switches
                # policy on `scope_source != 'explicit'` and `None` is the AUTO
                # damage-witness scope under a different acceptance rule.
                refs=(_refs if (args.reseat_region or _refs) else None),
                group_sources=sources,
                clearance=args.clearance,
                board_edge_clearance=args.board_edge_clearance,
                grid_step=args.grid_step, seed=args.seed,
                # The same flag, not a second one: it was parsed and
                # silently ignored on this path (#699).
                evict_depth=args.evict_depth,
                min_gain=args.reseat_min_gain)
            for note in reseat['notes']:
                print(f"  NOTE: {note}")
            # Over the SCOPE only: the line prints it as "{n} re-seated
            # (max X mm)", and `n` deliberately excludes the parts the
            # eviction rung moved, so measuring over both mixes the two
            # counts the engine went to the trouble of separating.
            _scope = set(reseat['scope'])
            _rmax = 0.0
            for mv in reseat['moves']:
                if mv['reference'] not in _scope:
                    continue
                fp = cur_pcb.footprints.get(mv['reference'])
                if fp is not None:
                    _rmax = max(_rmax, _math.hypot(mv['new_x'] - fp.x,
                                                  mv['new_y'] - fp.y))
            if _selector:
                # A SEPARATE key. Overloading scope_source is what would have
                # broken the acceptance policy switch, so the provenance of
                # the scope is reported beside it rather than inside it.
                #
                # The split is over the RESOLVED scope, not over the argument
                # lists: `len(args.reseat)` counts PATTERNS (`--reseat 'U*'`
                # is one), and a named ref that also lies in the region would
                # be counted twice, so the two numbers did not add up to the
                # scope they described.
                _named = set(reseat['scope']) - set(_region_refs)
                _both = set(reseat['scope']) & set(_region_refs)
                print(f"  scope_selector: {_selector} "
                      f"({len(_both)} of {len(reseat['scope'])} in scope came "
                      f"from the region, {len(_named)} from --reseat)")
            print(f"Reseat ({reseat['scope_source']}): "
                  f"{len(reseat['scope'])} in scope, "
                  f"{len(reseat['reseated'])} re-seated "
                  f"(max {_rmax:.2f}mm), {len(reseat['unseated'])} unseated, "
                  f"{len(reseat['refused'])} refused, "
                  f"{len(reseat.get('evicted') or [])} evicted; "
                  f"OFF-OUTLINE PARTS {len(reseat['witnesses_before'])} -> "
                  f"{len(reseat['witnesses_after'])}"
                  + ('' if reseat['accepted'] else '  [GATE REFUSED]'))
            print(f"  gate {reseat['gate_before']} -> {reseat['gate_after']}")
            # #698: WHICH scope-relevant term carried the pass, or -- on a
            # refusal -- what every basis measured. A verdict with no basis is
            # how this pass came to refuse on a term the operator never asked
            # about for a whole release.
            _ab = reseat.get('accept_basis') or {}
            if _ab.get('fired'):
                _t = next((t for t in _ab.get('terms') or []
                           if t['term'] == _ab['fired']), {})
                print(f"  accepted on {_ab['fired']}: "
                      f"{_t.get('before')} -> {_t.get('after')} "
                      f"({_t.get('units')}); {_ab.get('policy')}")
            elif _ab.get('policy') == 'explicit:one-term-strict':
                # "would have fired", never "no basis fired": on a safety or
                # licence refusal the top-level `fired` is None by design while
                # a term still carries `first`, and printing "no basis fired"
                # then contradicts the record right beside it.
                _first = next((t['term'] for t in (_ab.get('terms') or [])
                               if t.get('first')), None)
                print(("  refused despite " + _first + " improving: "
                       if _first else "  no basis improved: ") + ", ".join(
                    f"{t['term']} {t['before']}->{t['after']}"
                    for t in (_ab.get('terms') or [])))
            summary.update({
                'reseat': True,
                'scope': reseat['scope'],
                'scope_source': reseat['scope_source'],
                # #459: how the scope was NAMED, beside (never inside) what
                # policy graded it. None when refs were typed directly.
                'scope_selector': _selector,
                'reseated': len(reseat['reseated']),
                'reseated_refs': reseat['reseated'],
                'unseated': reseat['unseated'],
                'no_pose_blockers': reseat.get('no_pose_blockers') or {},
                'no_pose_verdict': reseat.get('no_pose_verdict') or {},
                # Parts moved OUTSIDE the declared scope by the eviction
                # rung. Empty at depth 0, which is the default.
                'evicted': reseat.get('evicted') or [],
                'no_pose_census': reseat.get('no_pose_census') or {},
                'evictions': reseat.get('evictions', 0),
                'evictions_reverted': reseat.get('evictions_reverted', 0),
                'refused': reseat['refused'],
                'edge_bands_dropped': reseat['edge_bands_dropped'],
                # THE load-bearing number: it is the one that predicts
                # routability. `reseated` counts moves, which is effort.
                'witnesses_before': len(reseat['witnesses_before']),
                'witnesses_after': len(reseat['witnesses_after']),
                'witnesses_after_refs': reseat['witnesses_after'],
                'gate_before': reseat['gate_before'],
                'gate_after': reseat['gate_after'],
                'accepted': reseat['accepted'],
                # #698: the whole basis record, not just the winner -- a basis
                # that measured nothing and a basis that measured no change
                # must not look alike to a reader of the summary either.
                'accept_basis': reseat.get('accept_basis'),
                'reseat_min_gain': args.reseat_min_gain,
                'reseat_max_move_mm': round(_rmax, 3),
            })
            summary['edge_floor_fallback'].update(
                reseat.get('edge_floor_fallback') or {})
            _advance(reseat['moves'], 'reseat')
            if reseat['edge_bands_dropped']:
                # Grade against what the pass actually honoured. Keeping the
                # dropped declarations would charge the repair for repairing:
                # a part brought home from 160mm out then reads "sits nearest
                # the west edge but is declared on the east edge".
                intent = reseat['intent_used']
                print(f"  (final grade excludes the "
                      f"{len(reseat['edge_bands_dropped'])} dropped edge "
                      f"declaration(s): a band measured off a discarded pose "
                      f"is not a spec to grade the homecoming against)")
            if reseat['unseated'] or reseat['refused'] \
                    or not reseat['accepted']:
                exit_rc = 4

        result = None
        if args.repair:
            result = seeder.repair_placement(
                cur_pcb, cur, intent, group_sources=sources,
                clearance=args.clearance,
                board_edge_clearance=args.board_edge_clearance,
                grid_step=args.grid_step)
            for note in result['notes']:
                print(f"  NOTE: {note}")
            max_move = 0.0
            for mv in result['moves']:
                fp = cur_pcb.footprints.get(mv['reference'])
                if fp is not None:
                    max_move = max(max_move, _math.hypot(mv['new_x'] - fp.x,
                                                         mv['new_y'] - fp.y))
            print(f"Repair: {len(result['violators'])} violator(s), "
                  f"{len(result['repaired'])} repaired "
                  f"({len(result['moves'])} moved, max {max_move:.2f}mm), "
                  f"{len(result.get('unresolved') or [])} unresolved, "
                  f"{len(result['unrepairable'])} unrepairable")
            summary.update({
                'repaired': len(result['repaired']),
                'unrepairable': len(result['unrepairable']),
                'moved_refs': [m['reference'] for m in result['moves']],
                'max_move_mm': round(max_move, 3),
            })
            summary['edge_floor_fallback'].update(
                result.get('edge_floor_fallback') or {})
            summary.update({f'{k}_before': v
                            for k, v in result['pad_report_before'].items()})
            _advance(result['moves'], 'repair')
            if result['unrepairable']:
                exit_rc = 4

        if not args.dry_run:
            # A no-op still writes a board: the next step in a chain is handed
            # a path, and "nothing needed doing" must not look like "the tool
            # produced nothing".
            #
            # Written in the stage, then COPIED onto the output inside a
            # recorded delivery (#973). An empty writer call straight to the
            # output recorded a row that claimed nothing and read a temp board
            # no row produced, so every part the passes moved came back
            # unclaimed under an armed regime.
            import shutil
            from placement import provenance
            _final = os.path.join(_stage.name, 'delivered.kicad_pcb')
            write_placed_output(cur, _final, [])
            with provenance.recorded_delivery(
                    args.input_file, args.output_file,
                    list(delivered_moves.values())):
                shutil.copyfile(_final, args.output_file)
            copy_siblings(cur, args.output_file)
            print(f"Delivered {args.output_file}")
            from placement.legality import grade_pad_legality
            pcb_out = parse_kicad_pcb(args.output_file)
            pads_after = grade_pad_legality(pcb_out, args.clearance,
                                            edge_margin=args.board_edge_clearance,
                                            pcb_file=args.output_file)
            graded = floorplan.grade(intent, pcb_out, args.output_file,
                                     group_sources=sources,
                                     clearance=args.clearance,
                                     board_edge_clearance=args.board_edge_clearance)
            own, pinned = _split_pinned(graded, args.output_file, intent)
            _print_grade(own, pinned)
            summary['grade_errors'] = len(own)
            summary['grade_errors_pinned'] = len(pinned)
            # #974: the same own/pinned lists that decide exit_rc below.
            summary['connector_requirements'] = floorplan.connector_requirements(
                graded, own, pinned,
                bands_dropped=(reseat['edge_bands_dropped']
                               if reseat is not None else None))
            summary['pad_conflicts_after'] = pads_after['pad_conflicts']
            # #697: the requirement each counted pair was graded at, when it
            # sits above args.clearance, so the count is explainable.
            summary['pad_clearance_required'] = pads_after.get('required') or []
            from placement.legality import format_required_clause as _req_cl
            if _req_cl(pads_after):
                print(f"  above the {args.clearance}mm floor: "
                      f"{_req_cl(pads_after)}")
            summary['hole_conflicts_after'] = pads_after['hole_conflicts']
            summary['oob_pad_count_after'] = pads_after['oob_pad_count']
            summary['pad_edge_after'] = pads_after['pad_edge']
            if own:
                exit_rc = 4
        else:
            summary['connector_requirements'] = (
                floorplan.connector_requirements_ungraded('dry-run'))
        _stage.cleanup()
        summary.setdefault('complete', True)
        summary.setdefault('status', 'ok')
        print('JSON_SUMMARY: ' + json.dumps(summary, sort_keys=True,
                                            default=str), flush=True)
        return exit_rc

    if not st.unplaced and not st.partially_unplaced and not args.force:
        # PARTIALLY unplaced boards (a stacked pile beside real placements --
        # a netlist re-import, or a seeder that pinned only the spec-fixed
        # parts) are a legitimate seeding target, not a refusal: locked parts
        # are treated as authoritative and the pile is what gets placed.
        print("place_seed: this board already looks PLACED. Seeding would "
              "discard that placement; use place_portfolio.py to explore "
              "variations of it, or --force to re-seed anyway.",
              file=sys.stderr)
        return UNPLACED_EXIT

    # Partially unplaced without --force: seed ONLY the stacked pile. The
    # genuinely-placed unlocked parts are someone's work, not this tool's to
    # re-derive; --force widens the scope back to everything unlocked.
    seed_refs = None
    if st.partially_unplaced and not st.unplaced and not args.force:
        # SCOPE FOLLOWS THE GATE. `partially_unplaced` is now decided on the
        # SUSPECT subset -- co-located parts that are not markers and not on
        # opposite sides -- so scoping the seed from the full `stacked_refs`
        # would re-seed the very parts the gate had just exonerated. A
        # front/back fiducial pair shares a coordinate BY DESIGN; a run that
        # tripped over one genuine pile would have moved every such pair on the
        # board as a side effect, and nothing downstream would attribute it.
        seed_refs = set(st.stacked_suspect_refs)
        _benign = len(set(st.stacked_refs) - seed_refs)
        print(f"place_seed: partially unplaced -- seeding only the "
              f"{len(seed_refs)} stacked part(s) that look like a pile; the "
              f"rest stand as placed (--force re-seeds everything unlocked)"
              + (f". {_benign} other co-located part(s) are left alone "
                 f"(markers, or opposite sides of the board)" if _benign else ""))
        # ...unless every one of them is (locked yes) IN THE FILE, in which
        # case `seed_from_intent` treats them as authoritatively placed
        # (seeder.py:396-400) and THE PILE CANNOT BE SEEDED AT ALL. Not exotic:
        # this toolchain STAMPS its own locks (seeder.stamp_locked, on
        # place_seed's own output), so a pile created by an earlier seeding run
        # arrives here locked and gets announced as the scope of a run that
        # then cannot touch it.
        #
        # Care with the claim: this is NOT "the run does nothing". The polish
        # pass is on by default and moves plenty -- measured on a 65-part
        # fixture, 41 parts moved while all three piled refs stayed put. That
        # is exactly why refusing is right rather than pedantic: continuing
        # would exit 0 having rearranged two thirds of the board and left the
        # one thing this branch exists to fix untouched.
        _movable = [r for r in sorted(seed_refs)
                    if not getattr(pcb.footprints.get(r), 'locked', False)]
        if not _movable:
            print(f"place_seed: all {len(seed_refs)} of those part(s) are "
                  f"(locked yes) in the file, so the pile CANNOT be seeded -- "
                  f"seed_from_intent treats a file-locked ref as already "
                  f"placed. Continuing would still move other parts (the "
                  f"polish pass is on by default) and exit 0 with the pile "
                  f"exactly as it is, so this refuses instead. Unlock those "
                  f"refs to seed them; --force re-seeds the WHOLE board and "
                  f"discards the existing placement; place_optimize.py is the "
                  f"tool if polish was all you wanted.", file=sys.stderr)
            return UNPLACED_EXIT

    rng = random.Random(f"{args.seed}")
    result = seeder.seed_from_intent(
        pcb, args.input_file, intent, rng, group_sources=sources,
        clearance=args.clearance,
        board_edge_clearance=args.board_edge_clearance,
        grid_step=args.grid_step, seed_refs=seed_refs,
        anchors_first=args.anchors_first,
        anchor_rounds=args.anchor_rounds,
        evict_depth=args.evict_depth,
        rotate_by_facing=args.rotate_by_facing)
    for note in result['notes']:
        print(f"  NOTE: {note}")
    print(f"Seeded {len(result['placements'])} part(s); "
          f"{len(result['unseated'])} unseated; "
          f"{len(result['lock_refs'])} to lock")
    # #893. A DECLARED rotation that could not be seated is a different fact
    # from a part that merely found no pose, and it is the one the author can
    # act on -- their claim is the reason. Without this the operator saw a
    # generic "no legal pose within any cap" and had no way to know which
    # declaration caused it.
    _rot_unseated = result.get('rotation_unseated') or {}
    if _rot_unseated:
        print(f"  {len(_rot_unseated)} declared rotation(s) could not be "
              f"seated -- the angle is the claim, not a fallback:")
        for _r in sorted(_rot_unseated):
            print(f"    {_r}: declared {_rot_unseated[_r]}")

    write_placed_output(args.input_file, args.output_file,
                        result['placements'])
    n_locked = seeder.stamp_locked(args.output_file, result['lock_refs'])
    copy_siblings(args.input_file, args.output_file)
    print(f"Stamped (locked yes) on {n_locked} part(s)")

    def _replace_output(moves, suffix):
        """Write `moves` beside the output, then rename it into place.

        The staged write records a row naming `<out><suffix>`, a file that is
        gone a moment later; the rename is what delivers, so the rename is
        recorded against the output itself (#973), before it happens.
        """
        from placement import provenance
        tmp = args.output_file + suffix
        write_placed_output(args.output_file, tmp, moves)
        with provenance.recorded_delivery(args.output_file, args.output_file,
                                          moves):
            os.replace(tmp, args.output_file)

    ratsnest = {}
    if not args.no_polish:
        from placement.quench import quench
        pcb_seeded = parse_kicad_pcb(args.output_file)
        # Guidance weights, same as place_portfolio: the seed should be
        # polished by the objective the later steps rank with. Locks ride in
        # from the file (must_lock was just stamped); edge connectors are
        # locked per-call so the polish cannot walk them off their band.
        # #702: edge_claims(), must_lock and the declared zones all now
        # ride in through `intent_gate`, resolved by the ONE function
        # every quenching CLI uses. The edge_claims()-not-edge_connectors
        # filter that used to live here moved with it -- see
        # `floorplan.resolve_intent_gate`, which is the point: a filter
        # that must be remembered at each call site is one that will be
        # forgotten at the next.
        from placement.cli_gates import resolve_intent_gate_for_cli
        _gate, _ = resolve_intent_gate_for_cli(
            intent, pcb_seeded, sources, args.intent)
        placements = quench(
            pcb_seeded, pcb_file=args.output_file,
            max_displacement=args.max_displacement,
            step=1.0, grid_step=args.grid_step, clearance=args.clearance,
            board_edge_clearance=args.board_edge_clearance,
            crossing_penalty=30.0, length_weight=0.3, halo_base=0.5,
            halo_coef=0.15, halo_weight=2.0, edge_halo=2.0, edge_weight=2.0,
            ignore_nets=args.ignore_nets,
            metrics_out=ratsnest, intent_gate=_gate,
            corridor_weight=args.corridor_weight,
            corridor_specs=list((intent.health or {}).get('bus_corridors')
                                or ()) or None)
        if placements:
            _replace_output(placements, '.polish')

    # ---- self-check: the seed must grade clean against its own intent ------
    def _grade():
        pcb_out = parse_kicad_pcb(args.output_file)
        return floorplan.grade(intent, pcb_out, args.output_file,
                               group_sources=sources,
                               clearance=args.clearance,
                               board_edge_clearance=args.board_edge_clearance)

    try:
        graded = _grade()
        # The quench has no zone term, so a polish nudge can walk a declared
        # zone member past its tolerance (measured: a crystal load cap,
        # 0.86mm past its zone's edge at one seed). A plain revert to the
        # seeded pose is not enough -- the polish moved NEIGHBORS into that
        # space too (measured: 0.784mm2 of new overlap) -- so the part is
        # RE-SEATED: the seeder's own search, targeted at its seeded pose,
        # constrained to its zone, against the post-polish board.
        # #701: and it has no keep-out term either, so the same nudge can walk
        # a part into a declared keep-out -- on a board this tool's own seeder
        # placed correctly, which then exits 4 against its own intent. Same
        # repair, same reason, one more rule name.
        #
        # #797 adds the third, and it only WORKS because of #797: before the
        # seat predicate had an exclusive-zone conjunct, this re-seat was free
        # to put the stranger straight back into the zone it was moved out of,
        # so listing the rule here would have been a repair that cannot
        # repair. The quench does gate `zone_exclusive` per move (#702), and
        # monotonically, so it can hold a clean seed clean but never fixes a
        # breach that reaches the written board by any other route -- which is
        # what this net is for.
        if not args.no_polish:
            _repairable = ('zone_containment', 'keepout', 'zone_exclusive')
            broke = sorted({v.ref for v in graded.errors
                            if v.rule in _repairable and v.ref})
            _rules = sorted({v.rule for v in graded.errors
                             if v.rule in _repairable and v.ref})
            if broke:
                import pose_score
                pcb_cur = parse_kicad_pcb(args.output_file)
                blocks2, _p = floorplan.resolve_blocks(intent, pcb_cur,
                                                       sources)
                st = pose_score.make_state(
                    pcb_cur, args.output_file, clearance=args.clearance,
                    board_edge_clearance=args.board_edge_clearance,
                    grid_step=args.grid_step,
                    # #701: the ONLY seat-predicate call site outside
                    # seeder.py. Without this the re-seat below would be free
                    # to put the part back into a declared keep-out while
                    # fixing its zone.
                    keepouts=intent.keepouts,
                    # #797: and the same for a declared exclusive zone --
                    # without it this repair could move a part out of its
                    # containment breach and straight into somebody's reserved
                    # region, so `place_seed` would exit 4 on a board it had
                    # just repaired. Resolved with `sources`, the same blocks
                    # the grade below uses.
                    exclusive_zones=floorplan.zone_entries(intent, blocks2))
                zone_of = {}
                for z in intent.blocks:
                    if z.rect is None:
                        continue
                    for r in blocks2.get(z.name, ()):
                        zone_of.setdefault(r, z)
                seeded_pose = {p['reference']: p for p in result['placements']}
                fixes = []
                pinned = []
                for ref in broke:
                    z = zone_of.get(ref)
                    sp = seeded_pose.get(ref)
                    # #701: a keep-out violation does NOT imply a zone. The
                    # zone-only version required one and skipped a zone-less
                    # part entirely, which would have left the keep-out half
                    # of this repair silently inert on most boards.
                    if sp is None or ref not in st.parts:
                        continue
                    # A KiCad-LOCKED part is not this repair's to move (#797).
                    # `_try_place` does not consult `locked` -- it is a seat
                    # search, and every OTHER caller filters its candidates
                    # first -- so without this the repair silently relocates a
                    # part the user pinned. Measured on glasgow_revC, which is
                    # what found it: adding `zone_exclusive` to `_repairable`
                    # made the loop reach two locked FIDUCIALS and move them,
                    # FID3 from (122.000, 118.500) to (128.000, 92.500) -- a
                    # 26mm move of an optical alignment target, which is a
                    # manufacturing fact and not a placement opinion.
                    #
                    # The violation is REPORTED instead. A locked part inside
                    # a declared zone is a contradiction between the file and
                    # the intent, and only its author can say which one is
                    # wrong; silently moving the part picks for them.
                    if getattr(st.parts[ref], 'locked', False):
                        pinned.append(ref)
                        continue
                    clr = seeder._try_place(
                        st, ref, sp['new_x'], sp['new_y'], set(),
                        constraint=z.rect if z is not None else None,
                        tol=intent.zone_tolerance(z) if z is not None else 0.5)
                    if clr is not None:
                        p2 = st.parts[ref]
                        fixes.append({'reference': ref, 'new_x': p2.x,
                                      'new_y': p2.y, 'new_rotation': p2.rot})
                if pinned:
                    # Named, never silent: a reader who sees the grade error
                    # and no repair line would otherwise conclude the repair
                    # is broken, when it declined on purpose.
                    print(f"  NOT repaired, {', '.join(pinned)} "
                          f"{'is' if len(pinned) == 1 else 'are'} "
                          f"(locked yes) in the file: a pinned part inside a "
                          f"declared {' / '.join(_rules)} is a contradiction "
                          f"between the board and the intent, and only its "
                          f"author can say which is wrong. Unlock it, or move "
                          f"the claim off it")
                if fixes:
                    print(f"  polish walked "
                          f"{', '.join(f['reference'] for f in fixes)} out of "
                          f"a declared {' / '.join(_rules)}; re-seated "
                          f"against the polished board")
                    _replace_output(fixes, '.reseat')
                    graded = _grade()
    except floorplan.UntrustworthyOutline as exc:
        print(f"place_seed: outline cannot be trusted for grading: {exc}",
              file=sys.stderr)
        return UNPLACED_EXIT
    own, pinned = _split_pinned(graded, args.output_file, intent)
    _print_grade(own, pinned)
    # THE GATE LOOKS AT THE COPPER IT JUST ARRANGED (run 27).
    #
    # The intent grade above answers "does this satisfy the document it was
    # built from", and a pad conflict is not in that document: `legality_budget`
    # carries `oob_count` (the emitter bakes that one) and, on an emitted
    # intent, withholds `overlap_area`, while pad and hole conflicts are not a
    # budgeted channel at all. So a seed could put one part's pin through
    # another's pad, grade clean against its own intent, and be RANKED clean
    # by compare_seeds -- and `check_assembly` would then call the board NOT
    # BUILDABLE for the pad_intersection nobody upstream had looked for.
    # Measured on esp_prog: ten seeds of ten, all passing, all unbuildable.
    #
    # Attribution, so the seed answers for its own work and not the board's
    # (the same split `_split_pinned` makes for the intent grade): a PAD pair
    # is the seed's when either member is a part it placed -- precise, by ref
    # -- and not when the other member is a part it could not seat (#982,
    # below). A hole conflict cannot be attributed that way, because
    # `grade_pad_legality` counts holes without recording the pair, so it is
    # judged on the DELTA against the input board: a count that rose is the
    # seed's, one that was already there is not.
    from placement.legality import grade_pad_legality
    _pads_in = grade_pad_legality(pcb, args.clearance,
                                  edge_margin=args.board_edge_clearance,
                                  pcb_file=args.input_file)
    _pads_out = grade_pad_legality(parse_kicad_pcb(args.output_file),
                                   args.clearance,
                                   edge_margin=args.board_edge_clearance,
                                   pcb_file=args.output_file,
                                   worst_n=0)
    # THE PARTS IT MOVED, not `placements`. That list carries every part the
    # seeder wrote, locked and out-of-scope ones included at the pose they
    # came in with -- so reading it as "what the seed placed" charges the seed
    # for a short between two parts it never touched. Measured on a fixture
    # with two locked, already-overlapping parts: 2 seeded pairs reported,
    # both of them the board's.
    def _moved(p):
        fp_in = pcb.footprints.get(p['reference'])
        return fp_in is None or (
            abs(p['new_x'] - fp_in.x) > 1e-6
            or abs(p['new_y'] - fp_in.y) > 1e-6
            or abs((p['new_rotation'] - fp_in.rotation) % 360.0) > 1e-6)
    _seeded = {p['reference'] for p in result['placements'] if _moved(p)}
    # #982: the pairs against a part that could not be seated are the seed's
    # doing only incidentally -- see `split_pad_pairs`.
    _my_pads, _unseated_pads = split_pad_pairs(
        _pads_out.get('worst') or (), _seeded, result['unseated'])
    # From the COUNT, not from `len(worst)`: `worst` is capped by `worst_n`
    # (10 by default, 0 above meaning uncapped), and subtracting a capped list
    # from itself would report 0 inherited on a board with 50 shorts. This way
    # the three numbers always sum to `pad_conflicts` whatever the cap is, and a
    # cap that ever came back would cost detail in the NAMES rather than
    # silence in the totals.
    _their_pads = max(0, (_pads_out.get('pad_conflicts') or 0)
                      - len(_my_pads) - len(_unseated_pads))
    _hole_delta = max(0, (_pads_out.get('hole_conflicts') or 0)
                      - (_pads_in.get('hole_conflicts') or 0))
    if _my_pads:
        print(f"  {len(_my_pads)} pad conflict(s) among the parts this seed "
              f"placed: "
              + '; '.join(f"{a} <-> {b} ({mm:.3f}mm)"
                          for a, b, mm in _my_pads[:10])
              + ("" if len(_my_pads) <= 10 else
                 f" ... and {len(_my_pads) - 10} more"))
    if _unseated_pads:
        print(f"  {len(_unseated_pads)} pad conflict(s) against a part this "
              f"seed could NOT seat, which was written at the pose it came in "
              f"with: "
              + '; '.join(f"{a} <-> {b} ({mm:.3f}mm)"
                          for a, b, mm in _unseated_pads[:10])
              + ("" if len(_unseated_pads) <= 10 else
                 f" ... and {len(_unseated_pads) - 10} more")
              + " -- reported not charged; seat the part and they go with it")
    if _their_pads:
        print(f"  {_their_pads} further pad conflict(s) between parts this seed "
              f"did not place -- the board's own, reported not charged")
    if _hole_delta:
        print(f"  hole conflicts rose {_pads_in.get('hole_conflicts')} -> "
              f"{_pads_out.get('hole_conflicts')} across this seed")
    after = ratsnest.get('after', {})
    summary = {'placed': len(result['placements']),
               'unseated': len(result['unseated']),
               # NAMES, not just a count. #629's complaint is that a verdict
               # you cannot act on is a dead end, and a count names nobody.
               'unseated_refs': list(result['unseated']),
               # #893: WHICH declared angle was refused, by ref. A caller that
               # sees only `unseated_refs` cannot tell a declaration it must
               # revisit from a board that is simply full.
               'rotation_unseated': result.get('rotation_unseated') or {},
               # #975: declared edge connectors seated with pad copper inside
               # the board-edge floor because no in-band seat clears it -- the
               # alternative was not seating them. `pad_edge_after` grades the
               # copper; this says which seats chose it, and why.
               'edge_floor_fallback': result.get('edge_floor_fallback') or {},
               'no_pose_blockers': result.get('no_pose_blockers') or {},
               # WHY each of them has no pose, not just who is nearby (#699).
               # "nothing is near it" and "everything near it is locked" were
               # the same empty dict, and they need different answers.
               'no_pose_verdict': result.get('no_pose_verdict') or {},
               'no_pose_census': result.get('no_pose_census') or {},
               # Trades KEPT, and trades REVERTED, separately: a reader who
               # sees `evictions: 1` must not have to guess whether the board
               # changed. The records themselves are in the NOTE lines.
               'evictions': sum(1 for e in (result.get('evictions') or [])
                                if e.get('accepted')),
               'evictions_reverted': sum(
                   1 for e in (result.get('evictions') or [])
                   if not e.get('accepted')),
               'locked': n_locked,
               'grade_errors': len(own),
               'grade_errors_pinned': len(pinned),
               # run 27: the copper this seed arranged, graded. `_seeded`
               # names the pairs; `_inherited` is the board's own and is
               # reported rather than charged.
               'pad_conflicts_seeded': len(_my_pads),
               'pad_conflicts_seeded_pairs': [[a, b, mm]
                                              for a, b, mm in _my_pads],
               # #982: against a part it could not seat, at that part's input
               # pose. Real copper, reported apart so the seeded count does
               # not swing with poses that have nothing to do with it.
               'pad_conflicts_unseated': len(_unseated_pads),
               'pad_conflicts_unseated_pairs': [[a, b, mm]
                                                for a, b, mm in _unseated_pads],
               'pad_conflicts_inherited': _their_pads,
               # The total the three buckets partition, so a reader can check
               # the arithmetic instead of trusting it. The repair path
               # publishes the same key from the same grade.
               'pad_conflicts_after': _pads_out.get('pad_conflicts') or 0,
               'hole_conflicts_added': _hole_delta,
               'grade_warnings': len(graded.warnings),
               'crossings': after.get('crossings'),
               'hpwl': (round(after['hpwl'], 3)
                        if after.get('hpwl') is not None else None),
               'output': args.output_file}
    summary['pad_edge_before'] = _pads_in['pad_edge']
    summary['pad_edge_after'] = _pads_out['pad_edge']
    # #974: after the split above, from the lists gate_reason reads below.
    summary['connector_requirements'] = floorplan.connector_requirements(
        graded, own, pinned)
    print("JSON_SUMMARY: " + json.dumps(summary, sort_keys=True))
    _reason = gate_reason(result['unseated'], own, _my_pads, _hole_delta)
    if _reason is not None:
        print(_reason, file=sys.stderr)
        return 4
    if pinned:
        print(f"place_seed: {len(pinned)} grade error(s) on locked part(s) set "
              f"aside (named above); the seed's own work grades clean.",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    # Declare the lever for the WHOLE run, so every pose this CLI writes
    # carries its name in the provenance record -- place_reconstruct.py
    # does the same. Without it a seeded board carries no lever at all, and
    # the provenance instrument is silent about where its poses came from.
    from placement.provenance import declare_lever
    with declare_lever('place_seed.py', sys.argv):
        import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
        sys.exit(main())
