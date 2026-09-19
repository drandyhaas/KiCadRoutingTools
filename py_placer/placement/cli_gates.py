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


def add_mechanical_arg(parser) -> None:
    """`--mechanical` / `--no-mechanical`: the RECORDED mechanical facts (#959).

    `mechanical.json` is what `stage_unaided` writes next to the staged board:
    the poses carried over from the source because they are mechanical facts
    a real new board would already know. Nothing read it until #959 -- run 29
    moved a fiducial off its declared pose and no gate objected. Discovered in
    the board's own directory, like the design brief; `--no-mechanical` is
    the OFF arm.
    """
    parser.add_argument(
        "--mechanical", metavar="JSON", default=None,
        help="mechanical.json: poses (and, in the declaration form, edges) a "
             "run did not choose. Auto-discovered in the board's directory "
             "when omitted. Reconciled against the brief and the board, "
             "compiled into grade-only anchor blocks on --emit-intent, and "
             "graded as mechanical_drift (#959)")
    parser.add_argument(
        "--no-mechanical", action="store_true",
        help="Do not read mechanical.json, even if one sits beside the board. "
             "The OFF arm for an auto-discovered input (#959)")


def _same(a: str, b: str) -> bool:
    """Two spellings of one path, compared the way the filesystem does
    (case-folded on Windows)."""
    import os
    return (os.path.normcase(os.path.abspath(a))
            == os.path.normcase(os.path.abspath(b)))


def _relocated(recorded: str, sha: str, board_path: str):
    """The recorded mechanical file under its new home, if the run dir
    moved: the same basename beside the regime manifest or beside the board,
    with the recorded sha. None otherwise -- a file with other bytes is not
    the declaration, whatever it is called."""
    import os
    from placement import provenance as PV
    from placement import reconcile
    if not sha:
        return None
    wd = PV.regime_for(board_path)
    cands = [os.path.join(wd, os.path.basename(recorded)) if wd else None,
             reconcile.discover_mechanical(board_path) or None]
    for c in cands:
        if c and os.path.isfile(c) and reconcile._sha256(c) == sha:
            return c
    return None


def load_mechanical_or_exit(args, board_path: str):
    """(mechanical, path, exit_code). `exit_code` is 2 when an explicit
    `--mechanical` names nothing, or the file found is not a mechanical
    declaration this build reads -- a file by that name that reads as "no
    mechanical facts" would be the silent absence #959 is about.

    UNDER AN UNAIDED REGIME whose manifest recorded a `mechanical.json`, that
    file is an input the run was handed, and the run cannot make it
    disappear: `--no-mechanical`, another `--mechanical`, a deleted file and
    a rewritten one (its sha no longer the recorded one) all exit 2. The
    Phase-3 verifier cleared two undispositioned contradictions each way --
    the flag, deleting the file, a lap board copied to a directory without
    it, and a rewrite -- and the recorded path is read wherever the board
    now lives, so a copied board keeps it."""
    import os
    import sys
    from placement import reconcile
    man = reconcile.regime_manifest(board_path)
    recorded = man.get('mechanical') if isinstance(man, dict) else None
    if recorded:
        asked = getattr(args, 'mechanical', None)
        why = None
        if getattr(args, 'no_mechanical', False):
            why = ("--no-mechanical: the unaided regime governing this board "
                   f"recorded {recorded} as an input at staging, and a "
                   "recorded input cannot be switched off")
        elif asked and _same(asked, recorded) is False:
            why = (f"--mechanical {asked}: the unaided regime recorded "
                   f"{recorded} at staging; another file is not it")
        elif not os.path.isfile(recorded):
            # A run dir that was MOVED keeps its bytes: the file beside the
            # regime manifest, or beside the board, with the recorded sha is
            # the same declaration (round-2 verifier: an archived run exited
            # 2 with the file sitting right there).
            found = _relocated(recorded, man.get('mechanical_sha256'),
                               board_path)
            if found:
                recorded = found
            else:
                why = (f"the mechanical declaration the unaided regime "
                       f"recorded at staging, {recorded}, is gone -- "
                       f"restore it")
        if why:
            print(f"cannot use the mechanical declaration: {why}",
                  file=sys.stderr)
            return None, recorded, 2
        try:
            mech = reconcile.load_mechanical(recorded)
        except reconcile.MechanicalError as exc:
            print(f"{exc}", file=sys.stderr)
            return None, recorded, 2
        msha = man.get('mechanical_sha256')
        if msha and mech['sha256'] != msha:
            print(f"cannot use the mechanical declaration: {recorded} "
                  f"changed after staging (sha {mech['sha256'][:12]}, "
                  f"recorded {msha[:12]}). It is an input the run was "
                  f"handed, not the run's to rewrite -- restore it",
                  file=sys.stderr)
            return None, recorded, 2
        return mech, recorded, 0
    if getattr(args, 'no_mechanical', False):
        return None, '', 0
    path = getattr(args, 'mechanical', None)
    if path:
        if not os.path.isfile(path):
            print(f"cannot read mechanical declaration {path}: no such file",
                  file=sys.stderr)
            return None, path, 2
    else:
        path = reconcile.discover_mechanical(board_path)
        if not path:
            return None, '', 0
    try:
        return reconcile.load_mechanical(path), path, 0
    except reconcile.MechanicalError as exc:
        print(f"{exc}", file=sys.stderr)
        return None, path, 2


def add_brief_arg(parser) -> None:
    """`--brief` / `--no-brief`: the DECLARED design intent (#711).

    Defined here for the same reason `--intent` is: one spelling, one
    description of what it buys. The sibling is auto-discovered, following the
    `.kicad_dru` precedent -- the artifact is the source of truth and a board
    that carries one should not need a flag to be believed.

    `--no-brief` exists because an auto-discovered input that changes placement
    needs an OFF arm: an A/B with no way to turn the variable off is not an
    A/B, and a board whose sibling brief you deliberately want ignored has no
    other way to say so.
    """
    parser.add_argument(
        "--brief", metavar="JSON", default=None,
        help="Design brief JSON: what the board file cannot know -- which "
             "connectors are user-facing, which edge each belongs on and "
             "where along it, what the enclosure forbids. Auto-discovered as "
             "the sibling <board>.design-brief.json when this is omitted; "
             "pass a path to override. A DECLARED claim outranks the edge "
             "this toolchain would otherwise infer from the part's current "
             "pose (#711)")
    parser.add_argument(
        "--no-brief", action="store_true",
        help="Do not read the sibling design brief, even if one exists. The "
             "OFF arm: without it there is no way to measure what declaring "
             "changed")


def load_brief_or_exit(args, board_path: str):
    """(brief, path, exit_code). `exit_code` is 2 when a brief is unreadable.

    Called BEFORE `record_invocation`, as `load_intent_or_exit` is, so an exit
    2 never records a manifest command that touched no file.

    An EXPLICIT `--brief` that does not exist is an error; a MISSING sibling
    is not. Those are different acts: naming a file that is not there is a
    mistake worth stopping for, and not having written one is the ordinary
    case this whole channel is designed around.
    """
    import os
    import sys
    from placement import design_brief
    if getattr(args, 'no_brief', False):
        return None, '', 0
    path = getattr(args, 'brief', None)
    if path:
        if not os.path.isfile(path):
            print(f"cannot read design brief {path}: no such file",
                  file=sys.stderr)
            return None, path, 2
    else:
        path = design_brief.discover_brief(board_path)
        if not path:
            return None, '', 0
    try:
        return design_brief.load_brief(path), path, 0
    except (OSError, ValueError) as exc:
        print(f"{exc}", file=sys.stderr)
        return None, path, 2


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
    parser.add_argument("--facing-weight", type=float, default=0.0,
                        help="Price the pin ORDER a pose forces: for every "
                             "part pair sharing 2+ nets, the count of net "
                             "pairs whose pad order is crossed (a proven lower "
                             "bound on the crossings any router must pay). "
                             "0 = off (default). Distinct from "
                             "--orient-weight, which scores DIRECTION and is "
                             "blind to order: two parts can point straight at "
                             "each other with every net crossed. Costly -- it "
                             "is the most expensive term in the objective. "
                             "Try 1")
