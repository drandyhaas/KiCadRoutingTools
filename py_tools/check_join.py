#!/usr/bin/env python3
"""Verify a candidate hand-join (polyline + vias) against ALL board copper,
BEFORE the first segment is committed.

Promoted from run 7's wk7/join_check.py, built mid-run after hand-authored
copper shipped 42 invisible shorts: connectivity gates said "connected", and
KiCad's DRC was only consulted at the end. The prototype checked AABB pads at
two hardcoded constants; this version stages the candidate onto a copy of the
board and lets the REAL check_drc engine grade it -- rotated pads, custom pad
polygons, netclass pairwise clearances, per-layer .kicad_dru rules, board
edge, hole-to-hole, all for free -- then adds the join-specific checks the
DRC deliberately does not do: layer transitions without a via, and same-net
via stacks (KiCad permits those, so only this tool will ever flag them).

    python check_join.py BOARD NET x,y,layer x,y,layer ... [via:x,y ...]

The polyline is the join: consecutive points on the SAME layer become track
segments; a layer change between consecutive points must happen at the SAME
coordinate AND carry a via: token there, or it is a missing-via violation.
Track width and via geometry default from the board's own Default netclass.

Exit 0 clean, 1 violations, 2 usage errors.
"""
import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)
import argparse
import json
import math
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

COORD_TOL = 1e-3   # mm: polyline vertices this close are the same point


def build_parser():
    p = argparse.ArgumentParser(
        description="Verify a candidate hand-join against ALL board copper "
                    "before committing it.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog="""
Examples:
  python check_join.py b.kicad_pcb SWDIO 132.1,75.4,F.Cu 133.0,75.4,F.Cu
  python check_join.py b.kicad_pcb GPIO1 130.0,70.0,F.Cu 130.0,70.0,B.Cu \\
      130.5,72.0,B.Cu via:130.0,70.0
""")
    p.add_argument('board', help='the .kicad_pcb the join would be added to')
    p.add_argument('net', help='EXACT net name the join belongs to')
    p.add_argument('tokens', nargs='+', metavar='x,y,layer|via:x,y',
                   help='polyline points (x,y,layer) and via positions '
                        '(via:x,y), in order')
    p.add_argument('--width', type=float, default=None,
                   help="track width in mm (default: the board's Default "
                        "netclass track_width, else 0.3)")
    p.add_argument('--via-size', type=float, default=None,
                   help="via outer diameter (default: board netclass, "
                        "else 0.5)")
    p.add_argument('--via-drill', type=float, default=None,
                   help="via drill (default: board netclass, else 0.3)")
    p.add_argument('--keep-staged', metavar='PATH', default=None,
                   help='also write the staged board (input + candidate) '
                        'here, for inspection in KiCad')
    p.add_argument('-q', '--quiet', action='store_true',
                   help='print only the verdict and the violations')
    return p


def _parse_tokens(tokens, parser):
    pts, vias = [], []
    for tok in tokens:
        try:
            if tok.startswith('via:'):
                x, y = tok[4:].split(',')
                vias.append((float(x), float(y)))
            else:
                x, y, layer = tok.split(',')
                pts.append((float(x), float(y), layer))
        except ValueError:
            parser.error(f"cannot parse token {tok!r} (want x,y,layer or "
                         f"via:x,y)")
    if len(pts) < 2 and not vias:
        parser.error("a join needs at least two polyline points (or a via)")
    return pts, vias


def _drc_knobs(board, quiet):
    """The same board-first knob resolution check_drc's own CLI performs, so
    the staged grade equals what plain check_drc.py would say."""
    clearance = 0.2
    try:
        from fix_kicad_drc_settings import find_project, project_copper_clearance
        pro = find_project(board)
        if os.path.isfile(pro):
            with open(pro, encoding='utf-8') as f:
                pc = project_copper_clearance(json.load(f))
            if pc:
                clearance = pc
    except Exception:
        pass
    net_clearances = None
    edge = 0.0
    h2h = None
    try:
        import routing_defaults as defaults
        h2h = defaults.HOLE_TO_HOLE_CLEARANCE
        from list_nets import read_design_rules, net_clearance_map
        from kicad_parser import extract_nets, detect_kicad_version
        rules = read_design_rules(board)
        if rules.get('classes'):
            with open(board, encoding='utf-8', errors='replace') as f:
                content = f.read()
            net_objs, _ = extract_nets(content, detect_kicad_version(content))
            del content
            net_clearances = net_clearance_map(
                board, [n.name for n in net_objs.values()], rules=rules) or None
        edge = float(rules.get('constraints', {})
                     .get('min_copper_edge_clearance') or 0.0)
        pro_h2h = float(rules.get('constraints', {})
                        .get('min_hole_to_hole') or 0.0)
        if pro_h2h > h2h:
            h2h = pro_h2h
    except Exception as e:
        if not quiet:
            print(f"  (netclass/edge rules not read: {e})")
    return clearance, net_clearances, edge, h2h


def _violation_sig(v):
    def _r(loc):
        try:
            return tuple(round(float(x), 3) for x in loc)
        except (TypeError, ValueError):
            return (str(loc),)
    locs = sorted(_r(v.get(k)) for k in ('loc1', 'loc2') if v.get(k))
    return (v.get('type'), tuple(sorted((str(v.get('net1')),
                                         str(v.get('net2'))))),
            v.get('layer'), tuple(locs))


def _stage_board(board, out_path, pcb, net_id, cand_segs, cand_vias):
    """input board text + the candidate copper appended as real s-exprs."""
    from kicad_parser import board_uses_name_nets
    from kicad_writer import generate_segment_sexpr, generate_via_sexpr
    with open(board, encoding='utf-8', errors='replace') as f:
        content = f.read()
    # Match the net-token format the file ACTUALLY uses (the output_writer
    # rule): a name token in a numbered-net file double-parses the copper.
    net_name = ((pcb.net_id_to_name or {}).get(net_id)
                if board_uses_name_nets(content) else None)
    blocks = []
    for s in cand_segs:
        blocks.append(generate_segment_sexpr(
            (s.start_x, s.start_y), (s.end_x, s.end_y), s.width, s.layer,
            net_id, net_name=net_name))
    for v in cand_vias:
        blocks.append(generate_via_sexpr(v.x, v.y, v.size, v.drill,
                                         v.layers, net_id,
                                         net_name=net_name))
    tail = content.rstrip()
    if not tail.endswith(')'):
        raise ValueError(f"{board} does not end with a closing paren")
    idx = content.rindex(')')
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(content[:idx] + '\n'.join(blocks) + '\n' + content[idx:])
    from placement.portfolio import copy_siblings
    copy_siblings(board, out_path)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    args.board = os.path.abspath(args.board)
    if not os.path.isfile(args.board):
        parser.error(f"{args.board}: no such file")
    pts, via_pts = _parse_tokens(args.tokens, parser)

    from kicad_parser import Segment, Via, parse_kicad_pcb
    import routing_defaults as defaults
    from list_nets import board_default_netclass_param

    pcb = parse_kicad_pcb(args.board)
    net_id = next((nid for nid, n in pcb.nets.items()
                   if n.name == args.net), None)
    if net_id is None:
        print(f"check_join: no net named {args.net!r} on this board "
              f"(names are exact; check with list_nets.py)", file=sys.stderr)
        return 2
    copper = list(pcb.board_info.copper_layers or ['F.Cu', 'B.Cu'])
    bad_layers = sorted({p[2] for p in pts} - set(copper))
    if bad_layers:
        print(f"check_join: {', '.join(bad_layers)} is not a copper layer "
              f"of this board ({', '.join(copper)})", file=sys.stderr)
        return 2

    width = (args.width
             or board_default_netclass_param(args.board, 'track_width')
             or defaults.TRACK_WIDTH)
    via_size = (args.via_size
                or board_default_netclass_param(args.board, 'via_diameter')
                or defaults.VIA_SIZE)
    via_drill = (args.via_drill
                 or board_default_netclass_param(args.board, 'via_drill')
                 or defaults.VIA_DRILL)

    # ---- candidate geometry + the join-shape checks ------------------------
    violations = []
    cand_segs = []
    for p, q in zip(pts, pts[1:]):
        if p[2] == q[2]:
            if math.hypot(q[0] - p[0], q[1] - p[1]) > COORD_TOL:
                cand_segs.append(Segment(
                    start_x=p[0], start_y=p[1], end_x=q[0], end_y=q[1],
                    width=width, layer=p[2], net_id=net_id))
            continue
        # A layer change: must happen in place, over a via. The prototype
        # silently DROPPED these pairs, so a join relying on a transition
        # was never checked as drawn.
        if math.hypot(q[0] - p[0], q[1] - p[1]) > COORD_TOL:
            violations.append(
                f"missing-via: layer change {p[2]} -> {q[2]} between "
                f"({p[0]:.3f},{p[1]:.3f}) and ({q[0]:.3f},{q[1]:.3f}) is "
                f"also a MOVE -- a transition happens at one coordinate; "
                f"add the point twice (once per layer) plus via:x,y")
        elif not any(math.hypot(vx - p[0], vy - p[1]) <= COORD_TOL
                     for vx, vy in via_pts):
            violations.append(
                f"missing-via: layer change {p[2]} -> {q[2]} at "
                f"({p[0]:.3f},{p[1]:.3f}) has no via: token there")
    cand_vias = [Via(x=vx, y=vy, size=via_size, drill=via_drill,
                     layers=[copper[0], copper[-1]], net_id=net_id)
                 for vx, vy in via_pts]

    # ---- same-net stacks (the DRC will NEVER flag these) -------------------
    from check_weird import stacked_copper_over_model
    own_segs = [s for s in pcb.segments if s.net_id == net_id]
    own_vias = [v for v in pcb.vias if v.net_id == net_id]
    pre = {_violation_sig_stub(f) for f in stacked_copper_over_model(
        {net_id: own_segs}, {net_id: own_vias}, lambda n: args.net)}
    for f in stacked_copper_over_model(
            {net_id: own_segs + cand_segs}, {net_id: own_vias + cand_vias},
            lambda n: args.net):
        if _violation_sig_stub(f) not in pre:
            violations.append(f"stacked-copper: {f['detail']} "
                              f"[{f['layer']}]")

    # ---- the real DRC engine, staged-vs-baseline ---------------------------
    if cand_segs or cand_vias:
        from check_drc import run_drc
        clearance, net_clearances, edge, h2h = _drc_knobs(args.board,
                                                          args.quiet)
        if not args.quiet:
            print(f"grading at clearance {clearance} "
                  f"(+netclasses/.kicad_dru as check_drc resolves them)")
        tmp = tempfile.mkdtemp(prefix='check_join_')
        try:
            staged = os.path.join(tmp, 'staged.kicad_pcb')
            _stage_board(args.board, staged, pcb, net_id, cand_segs,
                         cand_vias)
            if args.keep_staged:
                shutil.copy(staged, args.keep_staged)
                from copy_board import SIBLING_EXTS   # ONE list (#711)
                for ext in SIBLING_EXTS:
                    sib = os.path.splitext(staged)[0] + ext
                    if os.path.isfile(sib):
                        shutil.copy(sib, os.path.splitext(
                            os.path.abspath(args.keep_staged))[0] + ext)
            common = dict(clearance=clearance, net_patterns=[args.net],
                          quiet=True, hole_to_hole_clearance=h2h,
                          board_edge_clearance=edge, max_print=0,
                          check_sizes=False, print_summary=False,
                          net_clearances=net_clearances)
            base_v = {(_violation_sig(v))
                      for v in run_drc(args.board, **common)
                      if not v.get('accepted')}
            for v in run_drc(staged, **common):
                if v.get('accepted') or _violation_sig(v) in base_v:
                    continue
                loc = v.get('loc1') or v.get('loc2') or ()
                loc_s = ','.join(f"{float(x):.3f}" for x in loc[:2]) if loc else '?'
                violations.append(
                    f"{v.get('type')}: {v.get('net1')} vs {v.get('net2')}"
                    f" [{v.get('layer') or '*'}] near ({loc_s})"
                    + (f" -- {v.get('distance'):.3f}mm"
                       if isinstance(v.get('distance'), float) else ''))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    for msg in violations:
        print(f"  VIOLATION {msg}")
    print('CLEAN' if not violations else f'{len(violations)} violation(s)')
    return 0 if not violations else 1


def _violation_sig_stub(f):
    return (f['category'], f['layer'], round(f['x'], 3), round(f['y'], 3))


if __name__ == '__main__':
    from console_encoding import enable_utf8_console
    enable_utf8_console()
    sys.exit(main())
