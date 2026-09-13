#!/usr/bin/env python3
"""Apply a pose the MODEL chose, and let the engine grade it (#892).

    place_pose.py BOARD OUT set U1 129.9 98.3 --rot 270
    place_pose.py BOARD OUT set U1 --near 130 98 --rot 270      # snap to legal
    place_pose.py BOARD OUT rotate CON2 180
    place_pose.py BOARD OUT face U1 W USB1                      # aim a pad row
    place_pose.py BOARD OUT lock U1 CON1 CON2
    place_pose.py BOARD OUT unlock U1
    place_pose.py BOARD OUT set U1 129.9 98.3 --rot 270 rotate CON2 180 lock U1

Every verb writes through `placement.writer.write_placed_output` -- the same
writer `place_seed` uses -- carries the sibling `.kicad_pro` / `.kicad_dru`
(#441), and runs inside `declare_lever('place_pose.py')`, so an armed unaided
regime accepts this tool and still refuses the hand script it replaces.

SEVERAL VERBS IN ONE CALL describe ONE arrangement: every op is resolved
against the INPUT board and written in a single pass, so no op sees another's
effect and the result is one board state rather than a replayed sequence.

EXIT CODES. 0 written; 2 the request does not name a thing on this board (bad
arguments, an unreadable board, an unknown ref, a face with no pad row) -- a
typo you fix by rewriting the command; 3 the board carries copper (moving a
footprint would strand its tracks -- `--allow-routed` to override); 4 the
request is WELL FORMED and the board said no (it grades worse, or the part is
locked) and nothing was written -- a measurement you act on. Note that 4
departs from `place_seed`, where it means "written, but the grade found
errors": here a refusal writes nothing at all, which is what #892 asks for.

Every exit THIS TOOL decides prints exactly one `JSON_SUMMARY:` line, refusals
included -- the board gate and the unreadable-input path as well as the pose
refusals. The exception is argparse's own usage errors, which exit 2 from
inside argparse before there is a board, a grade or a summary to print; if you
are parsing output, treat "exit 2 with no summary line" as a usage error.

WHAT IS GRADED, and what "illegal" means. The verdict is
`placement.legality.grade_pad_legality` -- the same numbers `place_seed` and
the review sheet print, netclass- and `.kicad_dru`-aware (#697) -- taken on the
candidate board and compared with the SAME grade on the input. A request is
refused when it makes a category worse -- the counts (pad conflicts, hole
conflicts, pads off-board) and their MAGNITUDES (`pad_shortfall`,
`oob_pad_amount`) -- and never for damage the board already had. The magnitude
arms are not a nicety: on counts alone, and measured on flat_hierarchy, a part
already 2.0 mm off the board could be moved to 204.66 mm off it, exit 0, with
nothing in the summary saying so. The two verdicts are reported apart: `no_worse` is what this verb refuses
on, `legal` is whether the board is CLEAN at the resulting pose.
That is deliberate: an absolute gate is False for a large share of parts on a
real board before anything moves, so it would refuse poses no worse than where
the part already sits -- and it would make this tool useless on exactly the
unplaced pile it exists to arrange. `--strict-legal` is the absolute arm for a
caller who wants it; `--force` writes anyway and says so in the summary (the
cheats watcher labels `--force` a WAIVER, which is the correct reading).

There is no `--allow-unplaced`: this tool has no unplaced gate, because placing
the parts of a pile one decision at a time is what it is for. An unplaced board
is NOTED, not refused.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'actor'}

import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
import argparse
import json
import os
import sys

VERBS = ('set', 'rotate', 'face', 'lock', 'unlock')

#: The verbs and THEIR flags. `main()` parses with the verb-less parser, so
#: what `--help` renders is THIS TEXT -- the subparsers exist for a reader who
#: goes looking through the parser object (and for
#: `tests/test_431_skill_commands.py`, which does exactly that), not for the
#: help screen. (`place_pose.py` is deliberately absent from
#: `krt_capabilities.FLAG_SCRIPTS` for the same reason -- that contract is
#: "every flag this script accepts is visible in --help as an option and
#: accepted at the top level", and a per-verb flag is neither.)
VERB_HELP = """verbs (several in one call describe ONE arrangement):

  set REF [X Y] [--rot DEG] [--near X Y]
            --near X Y   the same point read as APPROXIMATE; implies --snap
            --rot DEG    absolute rotation; omitted, the part keeps its own
  rotate REF DEG [--relative]
            --relative   add to the current rotation instead of replacing it
  face REF FACE PARTNER      FACE names the row by the face it is on NOW
  lock REF [REF ...]
  unlock REF [REF ...]       the only way to move a KiCad-locked part
"""


def build_parser(with_verbs=True):
    """The CLI's parser.

    `with_verbs` registers the verb parsers as subparsers -- which is how
    `tests/test_431_skill_commands.py` finds `--rot` / `--near` / `--relative`
    when it checks that a documented flag exists. `main()` asks for the parser
    WITHOUT them, because a subparsers action consumes the rest of the line at
    the first verb and this tool takes several in one call; so what `--help`
    actually renders is the epilog, not a subparser section.
    """
    p = argparse.ArgumentParser(
        description=__doc__,
        epilog=VERB_HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('input_file', help='board to read')
    p.add_argument('output_file',
                   help='board to write (still required with --dry-run, '
                        'which writes nothing and reports the path under '
                        '`would_write` -- the same shape place_seed uses, and '
                        'it keeps a verb token from being read as the output '
                        'path)')
    p.add_argument('--clearance', type=float, default=None, metavar='MM',
                   help="Pad clearance the grade runs at. Omitted, it comes "
                        "from the BOARD (its Default netclass / constraints) "
                        "-- a fixed 0.25 on a board routed to 0.15 vetoes "
                        "poses the router would happily route")
    p.add_argument('--board-edge-clearance', type=float, default=None,
                   metavar='MM',
                   help="Edge clearance for the pose ranking. Omitted, from "
                        "the board")
    p.add_argument('--track-width', type=float, default=None, metavar='MM',
                   help="Track width the FACE rule prices a lane at. "
                        "Omitted, the board's Default netclass width")
    p.add_argument('--snap', action='store_true',
                   help="If the requested pose is refused, take the best "
                        "legal pose within --radius instead (ranked by "
                        "pose_score, then RE-GRADED here -- the ranker's "
                        "legality is an AABB gate and this verb's verdict is "
                        "exact geometry, so a candidate is verified, never "
                        "trusted). Implied by --near")
    p.add_argument('--radius', type=float, default=2.0, metavar='MM',
                   help='How far --snap may move the part, as a straight-line '
                        "distance (default: 2.0mm). The underlying sweep is a "
                        'square lattice, so its corner reaches 1.41x further; '
                        'the bound is applied here, where the flag reads as a '
                        'distance')
    p.add_argument('--snap-step', type=float, default=0.25, metavar='MM',
                   help='Lattice step of the snap sweep (default: 0.25mm)')
    p.add_argument('--snap-tries', type=int, default=6, metavar='N',
                   help='How many ranked poses to re-grade before giving up '
                        '(default: 6)')
    p.add_argument('--strict-legal', action='store_true',
                   help="Refuse unless the RESULT is clean, not merely no "
                        "worse than the input. Off by default: a board under "
                        "repair is rarely clean, and refusing every pose on "
                        "it would stop the work this tool exists for")
    p.add_argument('--force', action='store_true',
                   help="Write the pose even when it grades worse. A WAIVER: "
                        "the summary records `forced` and the run_watch "
                        "cheats scan labels this flag for what it is")
    p.add_argument('--allow-routed', action='store_true',
                   help="Run even when the board already carries copper. Off "
                        "by default: placement moves FOOTPRINTS and not "
                        "tracks, so every segment would be left behind "
                        "detached from its pad")
    p.add_argument('--dry-run', action='store_true',
                   help='Grade the request and print the summary; write '
                        'nothing')
    if with_verbs:
        _verb_parsers(p.add_subparsers(dest='verb', metavar='VERB'))
    return p


def _verb_parsers(sub=None):
    """One parser per verb, so a verb's own flags cannot drift.

    They are REGISTERED as subparsers when `build_parser` is asked for them,
    even though `main()` parses each segment by hand (several verbs in one
    call, which a subparsers action cannot express -- it consumes the rest of
    the line at the first one). The registration is not decoration: it is how
    `--rot` / `--near` / `--relative` are discoverable, and both
    `tests/test_431_skill_commands.py` (documented flags must exist) goes
    looking for them through the subparsers action -- `--help` itself renders
    the epilog, since `main()` builds the verb-less parser. The
    parsers registered there are the SAME objects the split-segment path
    parses with, so a flag cannot be documented in one and honoured by the
    other.
    """
    if sub is None:
        # ONE construction path. Asked for the map without a subparsers action
        # to hang it on, build the documented parser and read the map back out
        # of it -- so the parsers `main()` parses each segment with are the
        # very objects `--help` and the documented-flag gate looked at, and no
        # second definition can drift from the first.
        for action in build_parser(with_verbs=True)._actions:
            if isinstance(action, argparse._SubParsersAction):
                return dict(action.choices)
        raise RuntimeError('build_parser() registered no verbs')

    def _new(name, **kw):
        return sub.add_parser(name, add_help=False, **kw)

    out = {}

    q = _new('set', help='place a part at a pose')
    q.add_argument('ref')
    q.add_argument('x', nargs='?', type=float)
    q.add_argument('y', nargs='?', type=float)
    q.add_argument('--near', nargs=2, type=float, metavar=('X', 'Y'),
                   help='the same coordinates read as APPROXIMATE: implies '
                        '--snap, so the engine seats the part at the best '
                        'legal pose near the point you aimed at')
    q.add_argument('--rot', type=float, default=None,
                   help='absolute rotation in degrees; omitted, the part '
                        'keeps the rotation it has')
    out['set'] = q

    q = _new('rotate', help='turn a part, absolutely by default')
    q.add_argument('ref')
    q.add_argument('degrees', type=float)
    q.add_argument('--relative', action='store_true',
                   help='add to the current rotation instead of replacing it')
    out['rotate'] = q

    q = _new('face', help='turn a part so a named pad row faces a partner')
    q.add_argument('ref')
    q.add_argument('face', help='which pad row, named by the face it is on '
                                'NOW: north/south/east/west (or N/S/E/W)')
    q.add_argument('partner', help='the part that row should face')
    out['face'] = q

    for verb in ('lock', 'unlock'):
        q = _new(verb, help="stamp / strip KiCad's `(locked yes)`")
        q.add_argument('refs', nargs='+')
        out[verb] = q
    return out


def split_segments(rest):
    """Cut the leftover argv into one segment per verb, in order.

    A verb token starts a segment; everything up to the next verb token
    belongs to it. This is what lets several verbs ride in one call without
    argparse subparsers, which stop at the first one.

    KNOWN LIMIT, stated rather than papered over: a token is a verb by exact
    lowercase match, so a footprint actually REFERENCED `set` or `lock` would
    start a segment instead of being a name. KiCad references are conventionally
    uppercase and no board in this repo has one, but if you meet one, place it
    with `place_seed`/`place_optimize` rather than here -- a quoting escape
    would be a second syntax to learn for a case nobody has.
    """
    segments = []
    cur = None
    for tok in rest:
        if tok in VERBS:
            cur = [tok]
            segments.append(cur)
        elif cur is None:
            raise ValueError(
                "%r comes before any verb; the shape is "
                "`place_pose.py BOARD OUT <verb> ...` with a verb from %s"
                % (tok, '/'.join(VERBS)))
        else:
            cur.append(tok)
    return segments


def parse_segments(segments, parsers, error):
    """(ops, lock_refs, unlock_refs, snap_requested)."""
    ops, locks, unlocks = [], [], []
    snap = False
    for seg in segments:
        verb, argv = seg[0], seg[1:]
        try:
            a = parsers[verb].parse_args(argv)
        except SystemExit:
            error("cannot read the `%s` verb from %r -- see --help"
                  % (verb, ' '.join(argv)))
            raise
        if verb == 'set':
            x, y = a.x, a.y
            if a.near:
                if x is not None or y is not None:
                    error("`set %s` was given both positional coordinates and "
                          "--near; they are the same point spelled two ways, "
                          "so pass one (positional = exact, --near = "
                          "approximate and snapped)" % a.ref)
                x, y = a.near
                snap = True
            if x is None and y is None and a.rot is None:
                error("`set %s` asks for nothing: give X Y (or --near X Y) "
                      "and/or --rot" % a.ref)
            if (x is None) != (y is None):
                error("`set %s` needs both X and Y, or neither" % a.ref)
            ops.append({'kind': 'set', 'ref': a.ref, 'x': x, 'y': y,
                        'rot': a.rot})
        elif verb == 'rotate':
            ops.append({'kind': 'rotate', 'ref': a.ref, 'rot': a.degrees,
                        'relative': a.relative})
        elif verb == 'face':
            ops.append({'kind': 'face', 'ref': a.ref, 'face': a.face,
                        'partner': a.partner})
        elif verb == 'lock':
            locks.extend(a.refs)
        elif verb == 'unlock':
            unlocks.extend(a.refs)
    return ops, locks, unlocks, snap


def main(argv=None):
    p = build_parser(with_verbs=False)
    args, rest = p.parse_known_args(argv)
    # BEFORE the split, or the verb this swallowed makes the NEXT token look
    # like the error ("'C3' comes before any verb"), which sends the caller
    # looking at the wrong end of their command line.
    # Validated HERE, where a bad value is a usage error with a message,
    # rather than inside `pose_score._offsets`, which divides by the step:
    # `--snap-step 0` was a ZeroDivisionError traceback, exit 1, and ZERO
    # JSON_SUMMARY lines -- the third crash of that family.
    if args.snap_step <= 0:
        p.error("--snap-step is a lattice pitch in mm and must be positive; "
                "%g would divide the sweep by zero" % args.snap_step)
    if args.radius < 0:
        p.error("--radius is a distance in mm; %g is not one" % args.radius)
    if args.snap_tries < 0:
        p.error("--snap-tries counts candidates to re-grade; %d is not a "
                "count (0 means 'do not try any')" % args.snap_tries)
    if args.output_file in VERBS:
        p.error("%r reads as the OUTPUT PATH here, not a verb -- the shape is "
                "`place_pose.py BOARD OUT %s ...`. Pass an output board (with "
                "--dry-run too; it reports the path it would have written)"
                % (args.output_file, args.output_file))
    parsers = _verb_parsers()
    try:
        segments = split_segments(rest)
    except ValueError as exc:
        p.error(str(exc))
    if not segments:
        p.error("no verb given: %s" % '/'.join(VERBS))
    ops, lock_refs, unlock_refs, near_snap = parse_segments(
        segments, parsers, p.error)

    try:
        from redo_record import record_invocation
        record_invocation()
    except Exception:                                        # noqa: BLE001
        pass

    from kicad_parser import parse_kicad_pcb
    from placement import pose_ops
    from placement.placement_state import assess_placement, UNPLACED_EXIT

    if not os.path.isfile(args.input_file):
        return _refuse(args, "%s is not a file" % args.input_file, 2)
    try:
        pcb = parse_kicad_pcb(args.input_file)
    except Exception as exc:                                 # noqa: BLE001
        return _refuse(args, "cannot read %s: %s"
                       % (args.input_file, exc), 2)

    st = assess_placement(pcb, args.input_file)
    if st.has_copper and not args.allow_routed:
        return _refuse(
            args,
            "this board carries %d segment(s) and %d via(s); moving a "
            "footprint strands every track attached to it. Pose the unrouted "
            "board, or pass --allow-routed if you mean to."
            % (st.segments, st.vias), UNPLACED_EXIT,
            segments=st.segments, vias=st.vias)
    if st.unplaced:
        # A NOTE, not a gate (see the module docstring): arranging a pile one
        # decision at a time is what this tool is for, and the relative
        # legality rule lets a part leave the pile without being refused for
        # the pile's own overlaps.
        print("note: this board looks unplaced (%s) -- that is not a refusal "
              "here; place_pose exists to arrange it."
              % '; '.join(st.reasons[:2]), file=sys.stderr)

    try:
        summary = pose_ops.apply_poses(
            args.input_file, args.output_file, ops,
            pcb_data=pcb,
            clearance=args.clearance,
            board_edge_clearance=args.board_edge_clearance,
            track_width=args.track_width,
            lock_refs=lock_refs, unlock_refs=unlock_refs,
            snap=args.snap or near_snap, snap_radius=args.radius,
            snap_step=args.snap_step, snap_tries=args.snap_tries,
            strict=args.strict_legal, force=args.force,
            dry_run=args.dry_run)
    except pose_ops.PoseRefusal as exc:
        # The VERB is the caller's to print, not the finding's: the reason
        # text is reprinted verbatim by the --force path on a run that WROTE,
        # so a "refused rather than written" inside it contradicted the
        # outcome in its own last clause.
        print("place_pose REFUSED, nothing written: %s" % exc.reason,
              file=sys.stderr)
        # EVERY exit carries a summary, including the refusals raised before
        # one was built (an unknown ref, a locked part, a face with no pads):
        # a machine caller that has to parse stderr for those and JSON for the
        # rest will parse stderr for none of them.
        refused = exc.extra.get('summary')
        if refused is None:
            refused = {'input': args.input_file,
                       'output': None,
                       'dry_run': bool(args.dry_run),
                       'refused': exc.reason,
                       'moved': [], 'ops': [], 'legal': None}
            refused.update({k: v for k, v in exc.extra.items()
                            if k != 'summary'})
        refused.setdefault('exit_code', exc.code)
        _sc = (refused.get('snap_census') or {})
        if _sc.get('skipped'):
            # `_report` runs only on the success path, so without this a
            # refused run carried the "did nothing" note in the JSON alone.
            print("note: --snap/--near did not apply -- %s" % _sc['skipped'],
                  file=sys.stderr)
        print('JSON_SUMMARY: ' + json.dumps(refused, sort_keys=True,
                                            default=str), flush=True)
        # 2 = the request names something that is not there (a typo the caller
        # rewrites); 4 = the request is well formed and the BOARD said no (a
        # measurement the caller acts on). One code for both would make them
        # indistinguishable to anything reading the exit status.
        return exc.code

    _report(summary)
    print('JSON_SUMMARY: ' + json.dumps(summary, sort_keys=True, default=str),
          flush=True)
    return 0


def _refuse(args, reason, code, **extra):
    """Print the refusal AND a summary, then hand back the exit code.

    Every exit this tool decides carries a summary: a caller that has to parse
    stderr for the board gate and JSON for a pose refusal will parse stderr for
    neither.
    """
    print("place_pose REFUSED, nothing written: %s" % reason,
          file=sys.stderr)
    doc = {'input': args.input_file, 'output': None,
           'dry_run': bool(args.dry_run), 'refused': reason,
           'exit_code': code, 'moved': [], 'ops': [], 'legal': None}
    doc.update(extra)
    print('JSON_SUMMARY: ' + json.dumps(doc, sort_keys=True, default=str),
          flush=True)
    return code


def _report(summary):
    """The human half of the summary -- the numbers, not a verdict."""
    print("legality at clearance %g (%s), edge %g (%s)" % (
        summary['clearance'], summary['knobs']['clearance']['source'],
        summary['board_edge_clearance'],
        summary['knobs']['board_edge_clearance']['source']))
    for n in summary['ops']:
        arrow = ('%s -> %s' % (n['from'], n['to'])) if n['moved'] else 'unmoved'
        extra = ''
        if n['kind'] == 'face':
            extra = (" [%s row aimed %s, landed %d/%d]"
                     % (n['face'], n['target_face'],
                        n['row_on_target'][0], n['row_on_target'][1])
                     if 'row_on_target' in n else
                     " [%s row aimed %s]" % (n['face'], n['target_face']))
        if n.get('snapped'):
            extra += ' [snapped]'
        print("  %-6s %-8s %s%s" % (n['kind'], n['ref'], arrow, extra))
    print("pad legality: conflicts %s -> %s, holes %s -> %s, off-board "
          "%s -> %s" % (summary['pad_conflicts_before'],
                        summary['pad_conflicts_after'],
                        summary['hole_conflicts_before'],
                        summary['hole_conflicts_after'],
                        summary['oob_pad_count_before'],
                        summary['oob_pad_count_after']))
    if summary['knobs']['clearance']['source'] == 'cli':
        # A refusing tool whose threshold is a flag has to say when the
        # threshold came from the caller: measured on esp_prog (no netclass),
        # sweeping U1 over the 81 positions of `pose_score._offsets(2.0, 0.5)`,
        # 7 were refused at the board-resolved 0.25 and accepted at
        # --clearance 0.01. Per-part, not board-wide -- CON2 flips 10 of 81 on
        # the same sweep and C1 none -- so the part and the sweep are named
        # rather than the ratio alone. run_watch's FLOOR scope does not cover
        # this tool, so this line is the disclosure.
        print("note: the verdict ran at --clearance %g, which YOU supplied; "
              "the board's own floor was not used"
              % summary['clearance'], file=sys.stderr)
    _sc = summary.get('snap_census') or {}
    if _sc.get('skipped'):
        # On stderr and in the summary both: a flag that did nothing has to
        # say so where the operator is looking, not only in the JSON.
        print("note: --snap/--near did not apply -- %s" % _sc['skipped'],
              file=sys.stderr)
    if summary.get('forced'):
        # NOT the refusal sentence verbatim: it ends "Refused rather than
        # written", which is false on a run that wrote.
        print("WARNING: WRITTEN under --force, over this finding -- %s"
              % summary['refused'])
    if summary.get('locked_count') is not None:
        print("locked %d, unlocked %d" % (summary.get('locked_count') or 0,
                                          summary.get('unlocked_count') or 0))
    if summary['dry_run']:
        print("--dry-run: nothing was written")


if __name__ == "__main__":
    # Declare the lever for the WHOLE run, so every pose this CLI writes
    # carries its name (#903). place_pose.py is in
    # placement.provenance.LEVER_REGISTRY: an armed unaided regime accepts
    # this tool and still refuses the hand script it replaces.
    from placement.provenance import declare_lever
    with declare_lever('place_pose.py', sys.argv):
        import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
        sys.exit(main())
