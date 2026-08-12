#!/usr/bin/env python3
"""Is this pad reachable, or is the router being blamed for the geometry?

    python3 check_reachability.py board.kicad_pcb --net GND --at 142.5,88.1
    python3 check_reachability.py board.kicad_pcb --pad U3.23
    python3 check_reachability.py board.kicad_pcb --pad U3.23 --track 0.15 --json

Measures the widest track that can reach the rest of the net from that pad, by
Euclidean distance transform plus an exact Kruskal widest path (see
`placement/reachability.py`). Two verdicts, and they mean different things:

    PASSABLE -- a route EXISTS at this track width. If the router failed, that
                is a finding about the ROUTER (grid, ordering, ripup budget),
                not about the board.
    CAGED    -- no route exists at ANY grid. This one IS geometry, and the fix
                is placement or a spec change.

Why it exists: across four recorded runs, 9 of 14 "no placement can fix this"
claims were later refuted, and every claim that a pad was unroutable in
principle was wrong. The rule this tool enforces is simple -- **no impossibility
claim without a numeric field.**

Clearances are read from the BOARD (netclasses via the sibling `.kicad_pro`,
then `--clearance` as the base), so the verdict is graded at what the board
would actually route at rather than at a guessed round number.

Exit codes: 0 PASSABLE, 1 CAGED, 2 usage/geometry error.

**1 is reserved for the geometry verdict and nothing else.** An argument that
names nothing on this board -- a misspelled ref, a pad number that is not there,
an unreadable board file -- exits 2 with what it could not resolve. It used to
exit 1, because `raise SystemExit(msg)` does, and a caller acting on exit 1
re-enters placement and throws away every routed board.
"""
import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)
import argparse
import json
import os
import sys


class Unresolvable(Exception):
    """The arguments do not name anything on this board.

    Its own exception type because the ALTERNATIVE was `raise SystemExit(msg)`,
    and SystemExit with a message exits **1** -- which in this tool is the CAGED
    verdict, the most expensive answer it can give. Measured:

        --pad NOSUCH.9  -> exit 1   a typo
        --pad RM1.1.1   -> exit 1   a real pad, named "1.1"
        --pad U1.1      -> exit 2   the honest "nothing to reach here"

    The loop's L3 stage treats "a single genuine CAGED on the copper-free board"
    as sufficient to re-enter placement, and re-entering placement discards every
    routed board. So a misspelling could buy the most expensive misclassification
    in the loop, and nearly did. `target_cells == 0` was already guarded against
    a false PASSABLE (:224); this is the same guard on the other side, and the
    false CAGED is the costlier of the two.
    """


def _resolve_pad(pcb, spec):
    """'U3.23' -> (x, y, net_id, layer). The layer is the pad's own.

    Splits at EVERY dot and keeps what the BOARD actually has, rather than
    splitting on the last one positionally. KiCad pad numbers are strings and
    are allowed to contain dots -- RM1 on the measured board has a pad literally
    named "1.1", so `RM1.1.1` rsplit to ref `RM1.1` (no such footprint) and the
    tool reported the geometry verdict CAGED for a pad that exists and is fine.
    """
    if '.' not in spec:
        raise Unresolvable(f"--pad wants REF.PADNUM, got {spec!r}")
    # Every place the dot could be, scored against the board.
    cands, tried_refs = [], []
    for i, ch in enumerate(spec):
        if ch != '.':
            continue
        ref, num = spec[:i], spec[i + 1:]
        tried_refs.append(ref)
        fp = pcb.footprints.get(ref)
        if fp is None or not num:
            continue
        for p in fp.pads:
            if str(p.pad_number) == num:
                cands.append((ref, num, p))
                break                            # first pad of that number wins
    if not cands:
        known = [r for r in tried_refs if r in pcb.footprints]
        if known:
            fp = pcb.footprints[known[-1]]
            have = ', '.join(str(p.pad_number) for p in fp.pads[:12])
            more = '' if len(fp.pads) <= 12 else f' ... (+{len(fp.pads) - 12})'
            raise Unresolvable(
                f"footprint {known[-1]!r} exists but has no pad "
                f"{spec[len(known[-1]) + 1:]!r} (has: {have}{more})")
        raise Unresolvable(
            f"no footprint on this board matches any prefix of {spec!r} "
            f"(tried: {', '.join(repr(r) for r in tried_refs)})")
    if len(cands) > 1:
        # Genuinely ambiguous -- e.g. footprint "A" with pad "1.1" AND footprint
        # "A.1" with pad "1". Guessing would be a silent wrong answer about
        # which pad was graded, which is the whole failure mode being fixed.
        where = '; '.join(f"{r} pad {n!r}" for r, n, _ in cands)
        raise Unresolvable(f"{spec!r} is ambiguous on this board -- it could be "
                           f"{where}. Nothing here can decide which you meant.")
    _ref, _num, p = cands[0]
    layers = [l for l in (p.layers or []) if l.endswith('.Cu')]
    layer = None
    if p.drill and p.drill > 0:
        layer = None                              # any; caller picks the first
    elif layers and layers[0] != '*.Cu':
        layer = layers[0]
    return p.global_x, p.global_y, p.net_id, layer


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('board')
    p.add_argument('--pad', help="REF.PADNUM, e.g. U3.23 -- sets --at and "
                                 "--net from the pad itself")
    p.add_argument('--at', help="x,y of the stranded pad (mm)")
    p.add_argument('--net', help="net name; inferred from --pad when given")
    p.add_argument('--layers', default=None,
                   help="comma list, the seed pad's layer FIRST "
                        "(default: the board's copper layers)")
    p.add_argument('--track', type=float, default=None,
                   help="track width to test (default: the board's Default "
                        "netclass track width)")
    p.add_argument('--via', type=float, default=None,
                   help="via diameter for layer changes (default: the board's "
                        "Default netclass via)")
    p.add_argument('--clearance', type=float, default=None,
                   help="base clearance (default: the board's Default "
                        "netclass clearance). Per-net classes are read from "
                        "the sibling .kicad_pro on top of this")
    p.add_argument('--step', type=float, default=0.01,
                   help="raster step in mm (default 0.01). A throat is only "
                        "resolved to +/- one step, so halve it if the margin "
                        "comes out inside a step of zero")
    p.add_argument('--margin', type=float, default=4.0,
                   help="half-size of the view around the seed, mm "
                        "(default 4.0). Reachability is LOCAL; a whole board "
                        "at 10um costs memory for nothing")
    p.add_argument('--view', default=None, help="x0,y0,x1,y1 to override")
    p.add_argument('--json', action='store_true')
    args = p.parse_args(argv)

    from kicad_parser import parse_kicad_pcb  # _path put py_router on sys.path
    from placement import reachability

    # An unreadable board is exit 2 for the same reason a bad --pad is: an
    # uncaught exception here exits 1, which reads as CAGED to every caller.
    if not os.path.isfile(args.board):
        print(f"cannot resolve: no such board file: {args.board}",
              file=sys.stderr)
        return 2
    try:
        pcb = parse_kicad_pcb(args.board)
    except Exception as exc:                                    # noqa: BLE001
        print(f"cannot resolve: {args.board} did not parse: "
              f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    # EVERY argument-resolution failure below leaves here as exit 2. None of
    # them is a statement about the board's geometry, and 1 is reserved for the
    # one statement that is (CAGED). See Unresolvable.
    layer_first = None
    try:
        if args.pad:
            x, y, nid, layer_first = _resolve_pad(pcb, args.pad)
            seed = (x, y)
            net_id = nid
            if not net_id:
                raise Unresolvable(f"{args.pad} has no net -- nothing to reach")
        else:
            if not (args.at and args.net):
                raise Unresolvable(
                    "give --pad REF.NUM, or both --at x,y and --net")
            try:
                seed = tuple(float(v) for v in args.at.split(','))
            except ValueError:
                raise Unresolvable(f"--at wants x,y in mm, got {args.at!r}")
            if len(seed) != 2:
                raise Unresolvable(f"--at wants x,y in mm, got {args.at!r}")
            net_id = None
    except Unresolvable as exc:
        print(f"cannot resolve: {exc}", file=sys.stderr)
        print("(exit 2 -- this is an argument that names nothing on this board, "
              "NOT the CAGED geometry verdict, which is exit 1)", file=sys.stderr)
        return 2

    # Knobs from the BOARD, not from round numbers.
    import list_nets
    try:
        dr = list_nets.read_design_rules(args.board)
    except Exception:                             # noqa: BLE001
        dr = None
    def _board(key, fallback):
        v = list_nets.board_default_netclass_param(args.board, key, dr)
        return float(v) if v else fallback
    clearance = args.clearance if args.clearance is not None else _board(
        'clearance', 0.2)
    track = args.track if args.track is not None else _board('track_width',
                                                             0.15)
    via = args.via if args.via is not None else _board('via_diameter', 0.6)
    try:
        net_clearances = list_nets.net_clearance_map_by_id(
            args.board, {i: n.name for i, n in pcb.nets.items()}, dr)
    except Exception:                             # noqa: BLE001
        net_clearances = {}

    layers = (args.layers.split(',') if args.layers
              else list(pcb.board_info.copper_layers or ('F.Cu', 'B.Cu')))
    if layer_first and layer_first in layers:     # the seed's own layer first
        layers = [layer_first] + [l for l in layers if l != layer_first]
    # --view gets the same treatment as --at, and for the same reason: a
    # traceback here exits 1, which is the CAGED verdict. Garbage floats raised
    # ValueError out of this line; the wrong COUNT of them survived to raise
    # IndexError deep inside reachability.slack_field. Both read as "no route
    # exists at any grid" to a caller branching on the exit code.
    view = None
    if args.view:
        try:
            view = [float(v) for v in args.view.split(',')]
        except ValueError:
            print(f"cannot resolve: --view wants x0,y0,x1,y1 in mm, got "
                  f"{args.view!r}", file=sys.stderr)
            return 2
        if len(view) != 4:
            print(f"cannot resolve: --view wants FOUR values x0,y0,x1,y1, got "
                  f"{len(view)} in {args.view!r}", file=sys.stderr)
            return 2
        if view[2] <= view[0] or view[3] <= view[1]:
            print(f"cannot resolve: --view must have x1>x0 and y1>y0, got "
                  f"{args.view!r}", file=sys.stderr)
            return 2

    try:
        r = reachability.pad_reachability(
            pcb, seed, net_name=args.net, net_id=net_id, layers=layers,
            view=view, track_mm=track, via_mm=via, base_clearance=clearance,
            net_clearances=net_clearances, step=args.step,
            margin_mm=args.margin)
    except reachability.ScipyRequired as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    source = 'from --flags' if args.clearance is not None else 'from the board'
    if args.json:
        print(json.dumps(r.to_dict(), indent=2))
    else:
        print(f"board      {os.path.basename(args.board)}")
        print(f"knobs      clearance {clearance}mm, track {track}mm, "
              f"via {via}mm ({source})")
        print(r.format_text())
        if r.target_cells and not r.caged:
            print("\nPASSABLE means a route EXISTS at this width. If the "
                  "router failed here, that is a finding about the router "
                  "(grid, ordering, ripup budget) -- not about the board.")
    # "nothing to reach inside the view" is not a verdict, and must not be
    # reported as one: exit 2 so a script cannot read it as PASSABLE.
    if r.target_cells == 0:
        return 2
    return 1 if r.caged else 0


if __name__ == '__main__':
    sys.exit(main())
