"""Argparse flags shared by the placement CLIs (#431, widened by #702).

Defined once so the CLIs cannot drift: an `--allow-unplaced` that spells
itself differently, or a `--brief` one tool discovers and another does not,
is exactly the kind of divergence CLAUDE.md's CLI/GUI section warns about --
here between two CLIs rather than between a CLI and the GUI.
"""
from __future__ import annotations


def add_intent_arg(parser, *, summary: str) -> None:
    """`--intent`: the floorplan intent, with `summary` saying what this CLI
    reads of it (#1067: place_fanout_clearance holds the decap limits and the
    declared rotations, and nothing else), so one flag spelling, metavar and
    default serve every CLI without a help text that lies about one.
    """
    parser.add_argument(
        "--intent", metavar="JSON", default=None,
        help="Floorplan intent JSON (`check_floorplan.py --emit-intent` "
             "writes a starter). " + summary)


def load_intent_or_exit(args):
    """(intent, exit_code). `exit_code` is 2 when the intent is unreadable.

    Call it BEFORE `record_invocation`, so an exit 2 never records a manifest
    command that touched no file.
    """
    import sys
    from placement import floorplan
    if not getattr(args, 'intent', None):
        return None, 0
    try:
        return floorplan.load_intent(args.intent), 0
    except (OSError, ValueError) as exc:
        print(f"cannot load intent {args.intent}: {exc}", file=sys.stderr)
        return None, 2


def add_mechanical_arg(parser) -> None:
    """`--mechanical` / `--no-mechanical`: the RECORDED mechanical facts.

    A `mechanical.json` beside the board records the parts whose pose the
    enclosure fixes (`interfaces`, `fixed`, `project.floors`). Discovered in
    the board's own directory, like the design brief; `--no-mechanical` is
    the OFF arm.
    """
    parser.add_argument(
        "--mechanical", metavar="JSON", default=None,
        help="mechanical.json: poses and edges a run did not choose. "
             "Auto-discovered in the board's directory when omitted. "
             "Reconciled against the brief and the board, compiled at GRADE "
             "time into grade-only anchor blocks (never written into a plan), "
             "and graded as mechanical_drift")
    parser.add_argument(
        "--no-mechanical", action="store_true",
        help="Do not read mechanical.json, even if one sits beside the board. "
             "The OFF arm for an auto-discovered input")


def load_mechanical_or_exit(args, board_path: str):
    """(mechanical, path, exit_code). `exit_code` is 2 when an explicit
    `--mechanical` names nothing, or the file found is not a mechanical
    declaration this build reads -- a file by that name that reads as "no
    mechanical facts" would be a silent absence."""
    import os
    import sys
    from placement import reconcile
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


def add_board_state_args(parser) -> None:
    """`--allow-unplaced` / `--allow-routed` overrides for the two gates.
    check_floorplan is the one caller, and it never applies the routed gate."""
    parser.add_argument("--allow-unplaced", action="store_true",
                        help="Run even when the board does not look placed "
                             "(parts stacked at one coordinate). Off by default: "
                             "graded, a pile fires every rule at once, which is "
                             "noise rather than a verdict. On a pile that "
                             "place_simple.py is about to place, pass it to "
                             "emit or check the intent (--plan-only implies it)")
    parser.add_argument("--allow-routed", action="store_true",
                        help="Accepted and ignored: check_floorplan never "
                             "refuses a board for carrying copper (copper is "
                             "irrelevant to a floorplan)")


