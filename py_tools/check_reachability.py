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
then `--clearance` as the base), so the verdict is measured at what the board
would actually route at rather than at a guessed round number -- and then
pinned UP to the fab floor, because this tool PREDICTS routability and a lane
finer than any fab can etch is capacity nobody can build (see `_fab_wrap`).

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


def _relief_for(pcb, r, track_mm, clearance):
    """"So what do I DO?" -- how far the blocking part must move, or nothing.

    Deliberately narrow. It answers only when the two nearest things at the
    throat are pads belonging to two DIFFERENT footprints, because that is the
    only case where "move this part" is the shape of the answer. A throat formed
    by a segment, a via, or two pads of the same part is a different question
    (rip that copper, re-fan that part) and returning a plausible-looking
    distance for it would be a wrong instruction, which is worse than none.

    The distance is between the two FOOTPRINT pad bounding boxes, and the need
    is in GAP space: `track + 2*clearance`, not the track width. Reporting a
    track-space number here would ask for a move roughly 2*clearance too small.
    """
    near = getattr(r, 'near', ()) or ()
    if len(near) < 2:
        return []
    a, b = near[0], near[1]
    if a.get('kind') != 'pad' or b.get('kind') != 'pad':
        return []
    if not a.get('ref') or not b.get('ref') or a['ref'] == b['ref']:
        return []

    def _bbox(ref):
        fp = pcb.footprints.get(ref)
        if not fp or not fp.pads:
            return None
        xs0 = [p.global_x - p.size_x / 2.0 for p in fp.pads]
        ys0 = [p.global_y - p.size_y / 2.0 for p in fp.pads]
        xs1 = [p.global_x + p.size_x / 2.0 for p in fp.pads]
        ys1 = [p.global_y + p.size_y / 2.0 for p in fp.pads]
        return (min(xs0), min(ys0), max(xs1), max(ys1))

    ra, rb = _bbox(a['ref']), _bbox(b['ref'])
    if not ra or not rb:
        return []
    from placement.routability import relief_move
    need = track_mm + 2.0 * clearance
    # BOTH movers, not one. The pair is symmetric as geometry but not as a
    # decision: these are whole-footprint bounding boxes, so a 100-pin QFN and
    # the 0402 beside it give different distances for the same throat, and
    # naming only the first-listed one hands the operator "move the QFN" when
    # nudging the resistor is the cheap answer. Sorted so the smallest move
    # leads, and each entry says WHICH part it is asking to move.
    out = [dict(m, ref=b['ref'], against=a['ref'], need_mm=round(need, 5))
           for m in relief_move(ra, rb, need)]
    out += [dict(m, ref=a['ref'], against=b['ref'], need_mm=round(need, 5))
            for m in relief_move(rb, ra, need)]
    out.sort(key=lambda m: m['min_mm'])
    return out


#: floors key -> `fab_tiers` floor key. Via diameter is in here for the same
#: reason as track width: a barrel narrower than the fab drills is capacity
#: that cannot be built.
_FAB_KEYS = {'clearance': 'clearance', 'track_width': 'track_width',
             'via_diameter': 'via_diameter'}


def _fab_wrap(board, floors, tier=None, overrides=None):
    """Pin each resolved floor UP to what a fab can actually make.

    `list_nets.board_floor` is board-authoritative, NOT raise-only, and its own
    docstring says why that cuts both ways: a tool that GRADES existing copper
    must measure at the board's own value or it manufactures phantom
    violations, while a tool that PREDICTS routability must wrap at the fab
    floor or it promises capacity nobody can etch. `check_channels` learned
    this the expensive way -- on tigard --refs U3, a declared 0.05/0.05 took
    its deficit faces from 3 to 1 and U3's supply from 29 to 120, hiding two
    real deficits.

    This tool predicts. Its whole output is a claim about whether a route
    EXISTS, and its CAGED verdict is the most expensive answer in the loop, so
    a sub-fab clearance here buys a confident PASSABLE for a lane no fab can
    make -- the phantom-supply direction, which hides a defect rather than
    wasting effort on one.

    Pinned regardless of SOURCE, exactly as `enforce_fab_floors` treats an
    explicit flag: a `--clearance 0.05` typed on the command line is as
    unetchable as a declared one. The declared value is not discarded -- it is
    kept under `declared` in the floors block, so the defect record shows both
    what the board asked for and what the measurement actually used.

    `tier` / `overrides` come from the shared `--fab-tier` / `--fab-overrides`
    flags, and they are NOT decoration: a floor a tool enforces has to be one
    the operator can tell it about. A board deliberately built at an advanced
    tier, or with the documented `--fab-overrides` escape hatch, would
    otherwise be measured against a floor its fab beats -- and the answer that
    buys is CAGED, the most expensive verdict in the loop.

    Returns `(values, fab)` -- the wrapped floors and the fab floor dict, which
    the caller needs to wrap the PER-NET clearance map at the same minimum.
    """
    # #857: the PHYSICAL fab floor (override file, else the advanced rung),
    # not the selected tier's -- the tier bounds automatic descents, and a
    # reachability prediction is not a descent.
    from fab_tiers import physical_fab_floor, count_copper_layers_in_file
    try:
        fab = physical_fab_floor(count_copper_layers_in_file(board), overrides)
    except Exception:                                          # noqa: BLE001
        fab = {}
    for name, key in _FAB_KEYS.items():
        rec = floors.get(name)
        f = fab.get(key)
        if not rec or f is None or rec['value'] >= f - 1e-9:
            continue
        print(f"  {name.replace('_', ' ')} {rec['value']:g}mm "
              f"[{rec['source']}] is below the {f:g}mm fab floor; measuring "
              f"at {f:g}mm (no fab can make it narrower).", file=sys.stderr)
        floors[name] = {'value': f, 'source': 'fab floor',
                        'declared': {'value': rec['value'],
                                     'source': rec['source']}}
    return {n: floors[n]['value'] for n in _FAB_KEYS if n in floors}, fab


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
    p.add_argument('--json', action='store_true',
                   help="print the result as JSON on stdout, and nothing else "
                        "-- notices go to stderr. --json-out writes the same "
                        "document to a file and keeps the text report")
    p.add_argument('--defect-json', metavar='PATH', default=None,
                   help="when the verdict is CAGED, write a `defect-record` "
                        "JSON document to PATH: the throat coordinates, the "
                        "nearest foreign refs/pads, the shortfall in both "
                        "track-width and gap space, the floors graded at (with "
                        "their sources) and, when the two nearest blockers are "
                        "pads of two different parts, a per-direction relief "
                        "distance. Nothing is written for a PASSABLE or "
                        "NO-TARGET measurement. Schema: "
                        "placement.reachability.Reachability.defect_record")
    p.add_argument('--json-out', metavar='PATH', default=None,
                   help="also write the result dict to PATH (the same document "
                        "--json prints); the text report is still printed")
    # The same two flags every routing and DRC CLI carries, from the same
    # helper, so the fab floor this tool measures against is the one the rest
    # of the chain routed to.
    from fab_tiers import (add_fab_tier_args, fab_tier_from_args,
                           set_default_fab_tier)
    add_fab_tier_args(p)
    args = p.parse_args(argv)
    try:
        _tier, _tier_over = fab_tier_from_args(args)
    except SystemExit as exc:                     # a missing override FILE
        print(f"cannot resolve: {exc}", file=sys.stderr)
        return 2
    # Set the PROCESS default too, as route.py:5713 and check_drc.py do.
    # Nothing else in this tool's call graph reads a fab floor today, so this
    # changes no number here -- it is so a future nested consumer cannot
    # silently measure against the stock tier while this run was told another.
    set_default_fab_tier(_tier, _tier_over)

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
    # `board_floor` rather than a local closure: it returns the SOURCE beside
    # the value ('cli' | 'board netclass' | 'board constraint' | 'fixed
    # default' | 'fab floor'), which is what lets the defect record say where
    # each floor came from instead of publishing three bare numbers a reader
    # cannot audit.
    # One precedence step the old closure did not have: on a board with no
    # Default-netclass value, track and via now fall through to the board's
    # `min_track_width` / `min_via_diameter` constraint before the fixed
    # default (clearance has no constraint key, so it is unchanged). And one
    # step NEITHER had: the fab floor on top of all of it. The knobs line names
    # each source so both differences are visible in the text.
    floors = {}

    def _board(key, explicit, fallback):
        v, src = list_nets.board_floor(args.board, key, explicit=explicit,
                                       fallback=fallback, design_rules=dr)
        floors[key] = {'value': v, 'source': src}
        return v

    _board('clearance', args.clearance, 0.2)
    _board('track_width', args.track, 0.15)
    _board('via_diameter', args.via, 0.6)
    # ...and then wrapped at the fab floor, because this tool PREDICTS.
    _floors, _fab = _fab_wrap(args.board, floors, _tier, _tier_over)
    clearance = _floors['clearance']
    track = _floors['track_width']
    via = _floors['via_diameter']
    try:
        net_clearances = list_nets.net_clearance_map_by_id(
            args.board, {i: n.name for i, n in pcb.nets.items()}, dr)
    except Exception:                             # noqa: BLE001
        net_clearances = {}
    # The per-net map gets the same wrap as the base. Pinning only the base
    # would leave the hole open on exactly the boards this map exists for: a
    # net class declaring 0.05 would still build its part of the field at a
    # spacing no fab can etch, and the field is a min() over the classes, so
    # one unwrapped class sets the answer.
    _fc = _fab.get('clearance')
    if _fc is not None and net_clearances:
        _pinned = sum(1 for c in net_clearances.values() if c < _fc - 1e-9)
        if _pinned:
            print(f"  {_pinned} net-class clearance(s) below the {_fc:g}mm "
                  f"fab floor; measuring those nets at {_fc:g}mm.",
                  file=sys.stderr)
            net_clearances = {i: max(c, _fc) for i, c in net_clearances.items()}

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

    auto_widened = None
    try:
        r = reachability.pad_reachability(
            pcb, seed, net_name=args.net, net_id=net_id, layers=layers,
            view=view, track_mm=track, via_mm=via, base_clearance=clearance,
            net_clearances=net_clearances, step=args.step,
            margin_mm=args.margin)
        # run-23: NO-TARGET at the default view means the net's other island
        # sits beyond --margin, NOT that the question is unanswerable -- and
        # the manual retry ladder measured badly (--margin 12 took 128s,
        # --margin 18 306s, --margin 25 timed out at 10 minutes with NO
        # data, because cells scale as (span/step)^2 at the fixed step).
        # Locate the nearest other island by VECTOR union-find (no raster),
        # widen the view to hold seed + its closest point, and coarsen the
        # step so the grid stays ~1200^2 whatever the span. Coarse-locate;
        # the step used rides in the result so readings stay comparable --
        # re-measure a found throat on a small box at native step.
        if (r.target_cells == 0 and view is None
                and net_id is not None):
            import math as _math
            # `list_nets.net_islands`, NOT `net_forensics._components`: that
            # was a sibling CLI's private name imported from a directory
            # `_path` does not put on sys.path, so the ImportError would have
            # escaped as exit 1 -- the code this file reserves for the CAGED
            # geometry verdict, and the most expensive answer it can give.
            sx, sy = seed
            best = None
            own_d, own_i = None, None
            comp_pts = []
            for comp in list_nets.net_islands(pcb, net_id):
                pts = [p for _k, _o, ps in comp for p in ps]
                if not pts:
                    continue
                d = min(_math.hypot(px - sx, py - sy) for px, py in pts)
                comp_pts.append((d, pts))
                if own_d is None or d < own_d:
                    own_d, own_i = d, len(comp_pts) - 1
            for i, (_d, pts) in enumerate(comp_pts):
                if i == own_i:
                    continue
                for px, py in pts:
                    dd = _math.hypot(px - sx, py - sy)
                    if best is None or dd < best[0]:
                        best = (dd, px, py)
            if best is not None:
                _d, px, py = best
                x0 = min(sx, px) - args.margin
                x1 = max(sx, px) + args.margin
                y0 = min(sy, py) - args.margin
                y1 = max(sy, py) + args.margin
                span = max(x1 - x0, y1 - y0)
                # Rounded UP to 10 nm so the step the field is built at is
                # the step the result discloses (one number, not a rounded
                # copy beside a raw one), and the grid stays <= ~1200^2.
                step2 = max(args.step,
                            _math.ceil(span / 1200.0 * 1e5) / 1e5)
                # stderr: --json makes stdout a machine channel (the loop
                # driver documents it as one), and a notice on it is a
                # JSONDecodeError at character 0.
                print(f"  NO-TARGET at the default +/-{args.margin:g}mm "
                      f"view; nearest other island of this net is "
                      f"{best[0]:.2f}mm away -- auto-widening to "
                      f"[{x0:.1f},{y0:.1f},{x1:.1f},{y1:.1f}] at step "
                      f"{step2:g}mm", file=sys.stderr)
                r = reachability.pad_reachability(
                    pcb, seed, net_name=args.net, net_id=net_id,
                    layers=layers, view=[x0, y0, x1, y1], track_mm=track,
                    via_mm=via, base_clearance=clearance,
                    net_clearances=net_clearances, step=step2,
                    margin_mm=args.margin)
                # The widened step grows with the ISLAND DISTANCE (span/1200),
                # and nothing about that distance says the answer is still
                # resolvable. A bottleneck is only resolved to +/- one step, so
                # once a step is a large fraction of the track width the
                # verdict is a coin toss dressed as a measurement. Measured,
                # tigard U3.30 at one view: native step 0.01 gave bottleneck
                # 0.4403, the widened 0.02021 gave 0.46904 -- 29 um
                # OPTIMISTIC, more than one whole step, and optimistic is the
                # direction that hides a cage.
                coarse = step2 > track / 4.0 + 1e-12
                auto_widened = {
                    'view': [round(v, 2) for v in (x0, y0, x1, y1)],
                    'step_mm': step2,
                    'nearest_island_mm': round(best[0], 2),
                    'coarse': coarse}
                if coarse:
                    _msg = (f"COARSE: the widened step {step2:g}mm is more "
                            f"than a quarter of the {track:g}mm track, and a "
                            f"bottleneck is resolved to +/- one step -- treat "
                            f"this verdict as a locate, then re-measure the "
                            f"throat on a small --view at --step {args.step:g}")
                    auto_widened['coarse_note'] = _msg
                    r.note = f"{r.note} | {_msg}" if r.note else _msg
                    print(f"  {_msg}", file=sys.stderr)
    except reachability.ScipyRequired as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    # `declared -> pinned` in the report, not only on the stderr notice: the
    # record carries both numbers and the text should say the same thing, or a
    # reader of the printed report cannot tell a wrapped floor from a declared
    # one that happens to equal the fab minimum.
    source = ', '.join(
        f"{k.replace('_', ' ')}: {v['source']}"
        + (f" ({v['declared']['value']:g} declared [{v['declared']['source']}]"
           f" -> {v['value']:g})" if v.get('declared') else '')
        for k, v in floors.items())
    if args.defect_json:
        _sha = None
        try:
            import hashlib
            with open(args.board, 'rb') as _fh:
                _sha = hashlib.sha256(_fh.read()).hexdigest()
        except OSError:
            pass
        _relief = []
        # A relief distance needs two RECTS, and this tool measures a raster
        # throat -- so it is offered only when the two nearest things are pads
        # whose footprints we can bound. Absent rather than guessed: a wrong
        # "move R7 east by X" is worse than no instruction.
        try:
            _relief = _relief_for(pcb, r, track_mm=track, clearance=clearance)
        except Exception:                                   # noqa: BLE001
            _relief = []
        doc = r.defect_record(board=args.board, board_sha=_sha,
                              relief=_relief, floors=floors,
                              instrument='check_reachability')
        if doc is None:
            print(f"  no defect record written to {args.defect_json}: this "
                  f"measurement is {r.to_dict()['verdict']}, and a record "
                  f"describes a DEFECT", file=sys.stderr)
        else:
            try:
                with open(args.defect_json, 'w', encoding='utf-8') as fh:
                    json.dump(doc, fh, indent=1)
                print(f"  DEFECT RECORD -> {args.defect_json}",
                      file=sys.stderr)
            except OSError as exc:
                print(f"  could not write {args.defect_json}: {exc}",
                      file=sys.stderr)
    _doc = r.to_dict()
    if auto_widened:
        # The reading is only comparable at its own step -- say which.
        _doc['auto_widened'] = auto_widened
    if args.json_out:
        try:
            with open(args.json_out, 'w', encoding='utf-8') as fh:
                json.dump(_doc, fh, indent=2)
            print(f"  JSON -> {args.json_out}", file=sys.stderr)
        except OSError as exc:
            print(f"  could not write {args.json_out}: {exc}", file=sys.stderr)
    if args.json:
        print(json.dumps(_doc, indent=2))
    else:
        print(f"board      {os.path.basename(args.board)}")
        print(f"knobs      clearance {clearance}mm, track {track}mm, "
              f"via {via}mm ({source})")
        print(r.format_text())
        # WHERE, beside how much. The number alone sent run 20 to two more
        # tools and a hand-assembled paragraph. Not on a wide-open result:
        # there is no throat to locate (`throat` is already None there, and
        # this guard keeps it that way if that ever changes).
        _t = None if r.wide_open else r.throat
        if _t:
            _who = ', '.join(n.get('pad') or f"{n['kind']}@{n['net']}"
                             for n in (r.near or ()))
            print(f"throat     ({_t['x']}, {_t['y']}) on {_t['layer']}"
                  + (f"  between {_who}" if _who else ''))
            if r.gap_mm is not None:
                _ca, _cb = r.binding_clearances
                print(f"           gap {r.gap_mm:.4f}mm vs "
                      f"{r.gap_need_mm:.4f}mm needed "
                      f"(gap space); track {r.bottleneck_mm:.4f} vs "
                      f"{r.track_mm:.4f} (track space) -- same throat, two "
                      f"spaces, related by the clearance at each blocker "
                      f"({_ca:g} + {_cb:g}mm)")
                if r.gap_blockers < 2:
                    # Say it, rather than print a two-object number derived
                    # from one object and let the label carry the lie.
                    print(f"           NOTE only {r.gap_blockers} blocker(s) "
                          f"identified at this throat, so the far side is "
                          f"ASSUMED at the same clearance: the gap above is "
                          f"twice the room on the known side, not a distance "
                          f"between two named objects")
            if r.caged:
                _side = max(0.5, 4.0 * (r.track_mm - (r.bottleneck_mm or 0.0)))
                print(f"look at it:\n"
                      f"  python3 -X utf8 py_tools/render_placement.py "
                      f"{os.path.basename(args.board)} --focus \\\n"
                      f"      --view {_t['x'] - _side / 2:.3f},"
                      f"{_t['y'] - _side / 2:.3f},{_t['x'] + _side / 2:.3f},"
                      f"{_t['y'] + _side / 2:.3f} --size 1600")
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
