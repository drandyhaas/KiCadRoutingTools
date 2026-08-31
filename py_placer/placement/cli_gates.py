"""Argparse flags shared by the placement CLIs (#431, widened by #702).

Defined once so the CLIs cannot drift: a lock advisor that reports on one
tool but not the other, or an `--allow-unplaced` that spells itself differently,
is exactly the kind of divergence CLAUDE.md's CLI/GUI section warns about --
here between two CLIs rather than between a CLI and the GUI.

`add_intent_arg` raises the stakes a level, which is why it lives here rather
than four times over: since #702 the same resolved intent becomes the same hard
gate inside the same engine for `place_optimize`, `place_route_loop`,
`place_seed` and `place_portfolio`. Four spellings of that flag, or four
descriptions of what it buys, is how one of them ends up gating on something
the others do not -- and that divergence IS #702, which was `grep -ci intent`
returning 36 on place_seed.py and 0 on the two CLIs that run the most quench
iterations in a real chain.
"""
from __future__ import annotations


def add_intent_arg(parser, *, required: bool = False, extra: str = "") -> None:
    """`--intent`: the floorplan intent, for every CLI that quenches (#702).

    `required` because `place_seed` cannot run without one -- there the intent
    is the INPUT the placement is derived from, not a constraint on a placement
    that already exists. `extra` appends a per-tool clause, for a real
    difference in what the flag buys rather than a wording preference: the
    portfolio's intent is ALSO its hard rank gate and the source of its health
    signals.
    """
    parser.add_argument(
        "--intent", metavar="JSON", required=required, default=None,
        help="Floorplan intent JSON (`check_floorplan.py --emit-intent` writes "
             "a starter). Its declared zones, keep-outs and exclusive zones "
             "become HARD per-move gates inside the quench, and its must_lock "
             "globs and edge_connectors edge claims are locked, so a declared "
             "constraint no longer decays under optimization (#702). "
             "MONOTONE, not repairing: a part that arrives already violating "
             "may improve or hold, never worsen -- this prevents a walk-out, "
             "it does not undo one. WARNING: an intent that --emit-intent "
             "wrote FROM the board you are repairing records that board's "
             "damage as the requirement, and now STEERS the search toward it "
             "rather than merely mis-grading it. Omitted (the default), the "
             "run is bit-identical to one built before this flag existed"
             + ((" " + extra) if extra else ""))


def load_intent_or_exit(args):
    """(intent, exit_code). `exit_code` is 2 when the intent is unreadable.

    Shared because the two things it gets right are the two things a per-CLI
    copy got wrong:

    * it is called BEFORE `record_invocation`, so an exit 2 never records a
      manifest command that touched no file (place_route_loop recorded one);
    * `--suggest-locks` warns and continues rather than erroring, and warns on
      BOTH CLIs -- place_optimize printed the note and place_route_loop
      returned before reaching it, which is a small instance of exactly the
      drift this module exists to prevent.
    """
    import sys
    from placement import floorplan
    if not getattr(args, 'intent', None):
        return None, 0
    if getattr(args, 'suggest_locks', False):
        print("--intent is ignored with --suggest-locks: no quench runs, and "
              "the lock advisor does not read an intent", file=sys.stderr)
        return None, 0
    try:
        return floorplan.load_intent(args.intent), 0
    except (OSError, ValueError) as exc:
        print(f"cannot load intent {args.intent}: {exc}", file=sys.stderr)
        return None, 2


def resolve_intent_gate_for_cli(intent, pcb_data, sources, path):
    """(bundle, problems) plus the one report every quenching CLI must print.

    Shared so the four CLIs cannot describe the same gate differently. Two
    things it does that a hand-rolled copy at each site kept getting wrong:

    * **Resolves at `sources or auto`.** `place_optimize` and
      `place_route_loop` default `--group-by none`, while `place_seed`,
      `place_portfolio` and `check_floorplan` default `auto`. Resolving at a
      bare `()` makes every `group:`-shaped block resolve to NOTHING on exactly
      the two tools that run the most quench iterations -- and silently, since
      neither runs a grader. That is #702 one level down. Deriving groups is
      read-only (`check_floorplan.py` says so in its own `--group-by` help), so
      it is safe to derive here even when the caller asked not to MOVE blocks;
      `sources` still governs the write side, which stays off.
    * **Prints `resolve_blocks`' problems instead of dropping them.**
      `block_unresolved` is ERROR severity by design, and the argument is
      stronger for a gate than for a grade: a block that resolves to nothing
      gates nobody, and looks identical to a gate that is working.
    """
    import sys
    from placement.groups import parse_sources
    from placement import floorplan
    resolve_sources = tuple(sources) or parse_sources('auto')
    bundle, problems = floorplan.resolve_intent_gate(
        intent, pcb_data, resolve_sources)
    # Print the severity the finding CARRIES. This loop labelled everything
    # `INTENT ERROR`, which was harmless while every finding reaching it
    # defaulted to error -- and became a lie with #793, whose whole design is a
    # finding at WARN so it can be loud without being fatal. Relabelling it
    # ERROR at the four CLIs the issue was written for would have undone that.
    # Default-severity output is unchanged: every pre-#793 finding here is an
    # error, so only a demoted one now reads differently, which is correct and
    # was previously wrong.
    n_err = 0
    for v in problems:
        is_err = v.severity == floorplan.ERROR
        n_err += is_err
        label = 'INTENT ERROR' if is_err else 'INTENT WARN '
        print(f"  {label} [{v.rule}] {v.message}", file=sys.stderr)
    if problems:
        # A tally, because these four CLIs PRINT the problems and act on none
        # of them: without a count, "nothing was wrong" and "several things
        # were wrong and scrolled past" look the same to a reader.
        print(f"  ({n_err} error(s), {len(problems) - n_err} warning(s) from "
              f"the intent; this gate reports, it does not exit)",
              file=sys.stderr)
    zoned = [z for z in bundle['zones'] if z['refs']]
    bound = len({r for z in zoned for r in z['refs']})
    if not (zoned or bundle['keepouts'] or bundle['lock_refs']):
        # place_portfolio's --corridor-weight warning, same shape: an
        # intent-derived knob with nothing to bite on says so, rather than
        # reading as enforcement that happened to find nothing wrong.
        print(f"--intent {path} declares nothing this quench can gate on "
              f"(no block with a resolved zone rect, no keep-out, no "
              f"must_lock, no edge claim): the gate is inert", file=sys.stderr)
    else:
        print(f"intent: {len(zoned)} zoned block(s) over {bound} part(s), "
              f"{len(bundle['keepouts'])} keep-out(s), "
              f"{len(bundle['lock_refs'])} locked ref(s); blocks resolved from "
              f"{','.join(resolve_sources) or 'refs only'}")
    return bundle, problems


def add_board_state_args(parser) -> None:
    """`--allow-unplaced` / `--allow-routed` overrides for the two gates."""
    parser.add_argument("--allow-unplaced", action="store_true",
                        help="Run even when the board does not look placed "
                             "(parts stacked at one coordinate). Off by default: "
                             "this toolchain REFINES a placement, so on a pile "
                             "every candidate pose is illegal and the run prints "
                             "'0 parts moved' plus a legality block that looks "
                             "like a result")
    parser.add_argument("--allow-routed", action="store_true",
                        help="Run even when the board already carries copper. "
                             "Off by default: placement moves FOOTPRINTS and "
                             "not tracks, so every segment would be left behind "
                             "detached from its pad")


def add_lock_advisor_args(parser) -> None:
    """`--suggest-locks` and friends. Report-only; nothing is ever auto-locked."""
    parser.add_argument("--suggest-locks", action="store_true",
                        help="Report which parts look position-critical "
                             "(mounting holes, board-edge overhang, connectors) "
                             "with a reason each, print a paste-ready --lock "
                             "list, and exit. Writes NO board and locks nothing "
                             "-- a wrong auto-lock silently freezes a part that "
                             "needed to move, and that failure is invisible")
    parser.add_argument("--suggest-locks-json", metavar="PATH",
                        help="With --suggest-locks, also write the findings as "
                             "JSON (every measurement, fired or not)")
    parser.add_argument("--suggest-locks-globs", action="store_true",
                        help="With --suggest-locks, collapse the suggestion to "
                             "globs (J*) instead of exact refs, printing each "
                             "glob's blast radius. Exact refs are the default: "
                             "a glob you did not inspect freezes parts you "
                             "never looked at")
    parser.add_argument("--lock-confidence", default="medium",
                        choices=("high", "medium", "low"),
                        help="Minimum confidence to include in the suggested "
                             "--lock list (default: medium)")
    parser.add_argument("--lock-edge-margin", type=float, default=1.0,
                        metavar="MM",
                        help="Distance from the board edge under which a part "
                             "is flagged as possibly position-critical "
                             "(default: 1.0mm)")


def add_tidiness_args(parser) -> None:
    """The #548 alignment and orientation cost terms. BOTH OFF by default.

    Off because they are heuristic and aesthetic, and the router -- not a
    tidiness score -- is the judge of a placement. Turning them on by default
    would silently change every user's board to buy legibility, and would also
    corrupt the isolated fixtures in `test_458_*`, which zero every geometry
    knob they know about so the objective is clean enough to assert an exact
    total. At 0.0 both return before touching any geometry, so a default run is
    bit-identical rather than merely close.
    """
    parser.add_argument("--align-weight", type=float, default=0.0,
                        help="Reward same-footprint parts that share an axis, "
                             "so a row of decoupling caps comes out ON a line "
                             "rather than within a few tenths of one. 0 = off "
                             "(default). Try 5")
    parser.add_argument("--align-radius", type=float, default=0.5,
                        metavar="MM",
                        help="Off-axis distance past which two peers are "
                             "simply not a row, so the penalty stops growing "
                             "(default: 0.5mm). The penalty is continuous "
                             "here: a cliff would pay a part to flee the row")
    parser.add_argument("--align-span", type=float, default=20.0,
                        metavar="MM",
                        help="How far apart two same-footprint parts can SEED "
                             "and still be considered peers (default: 20.0mm)")
    parser.add_argument("--orient-weight", type=float, default=0.0,
                        help="Reward a rotation that puts a part's pads on the "
                             "side its nets leave from. 0 = off (default). The "
                             "airwire cost already uses exact pad positions, "
                             "but a ~1mm pad offset is noise against a ~20mm "
                             "net, so the signal needs its own weight. Try 1")
