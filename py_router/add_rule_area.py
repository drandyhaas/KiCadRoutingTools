#!/usr/bin/env python3
"""add_rule_area.py -- write a copper keep-out rule area onto a board (#1200).

A module's PCB antenna needs a copper keep-out on every layer, and KiCad draws
that as a rule area: `(zone ... (keepout (tracks not_allowed) ...))`. The
toolchain honours one when the board has it -- the router stamps it
(`obstacle_map.add_rule_area_keepout_obstacles`) and placement grades it --
but nothing could CREATE one: route_planes writes pour-only keep-outs around
NPTH slots, and a design brief's `keepouts[]` compiles into the placement
intent only. On a board placed from scratch the keep-out belongs to the
module, so it must follow the part's final pose, which only a tool can do
after placement.

    python3 py_router/add_rule_area.py in.kicad_pcb out.kicad_pcb \\
        --name ANT_KEEPOUT --ref U1 --rect -9 -18 9 -12

The area is a rectangle (`--rect X0 Y0 X1 Y1`) or a polygon (`--polygon X,Y
X,Y X,Y ...`), in board millimetres -- or, with `--ref`, in that footprint's
LOCAL frame as the file stores it (the frame its own pads' `(at)` positions
are written in, already mirrored for a part on the back), so re-running after
the part moves puts the area where the part now is. It goes on every copper
layer unless `--layers` names some, and forbids tracks, vias and copper pour
unless `--forbid` says otherwise (pads and footprints stay allowed: KiCad
flags a footprint's own pads inside its own antenna keep-out otherwise).

A board-level rule area of the same `--name` is replaced, so the command is
idempotent. The output gets the input's siblings (`copy_board`), and the run
says what it wrote.

Exit 0 written; 2 for a usage error or a `--ref` the board does not have.
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

KRT_TOOL = {'scope': ['placement', 'routing', 'combined'], 'kind': 'actor'}


def _top_level_zone_spans(text):
    """(start, end) of each board-level `(zone ...)` block, end exclusive
    and including its trailing newline. String-aware paren matching."""
    spans = []
    for m in re.finditer(r'\n\t\(zone(?=[\s(])', text):
        i, depth, in_str = m.start() + 1, 0, False
        while i < len(text):
            c = text[i]
            if in_str:
                if c == '\\':
                    i += 1
                elif c == '"':
                    in_str = False
            elif c == '"':
                in_str = True
            elif c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    end = i + 1
                    if end < len(text) and text[end] == '\n':
                        end += 1
                    spans.append((m.start() + 1, end))
                    break
            i += 1
    return spans


def remove_named_rule_areas(text, name):
    """`text` without its board-level rule areas named `name`, and how many."""
    out, n, last = [], 0, 0
    for a, b in _top_level_zone_spans(text):
        block = text[a:b]
        nm = re.search(r'\(name\s+"((?:[^"\\]|\\.)*)"\)', block)
        if '(keepout' in block and nm and nm.group(1) == name:
            out.append(text[last:a])
            last, n = b, n + 1
    out.append(text[last:])
    return ''.join(out), n


def area_points(args, pcb):
    """The polygon in board mm, plus a description of where it came from."""
    if args.rect:
        x0, y0, x1, y1 = args.rect
        pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    else:
        pts = []
        for tok in args.polygon:
            try:
                x, y = (float(v) for v in tok.split(','))
            except ValueError:
                raise SystemExit(f"add_rule_area: error: --polygon point {tok!r} "
                                 f"is not X,Y")
            pts.append((x, y))
        if len(pts) < 3:
            raise SystemExit("add_rule_area: error: --polygon needs at least "
                             "3 points")
    if not args.ref:
        return pts, 'board coordinates'
    fp = pcb.footprints.get(args.ref)
    if fp is None:
        print(f"add_rule_area: error: {args.ref} names no footprint on this "
              f"board", file=sys.stderr)
        raise SystemExit(2)
    from kicad_parser import local_to_global
    rot = fp.rotation or 0.0
    return ([local_to_global(fp.x, fp.y, rot, x, y) for x, y in pts],
            f"{args.ref}'s local frame ({fp.x:g}, {fp.y:g}, rot {rot:g}, "
            f"{fp.layer})")


def main():
    try:
        from redo_record import record_invocation
        record_invocation()  # stress-test redo manifest; no-op unless REDO_MANIFEST set
    except Exception:                                          # noqa: BLE001
        pass
    from kicad_writer import RULE_AREA_FLAGS
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('input_file', help='input .kicad_pcb')
    p.add_argument('output_file', help='output .kicad_pcb (may equal the input)')
    p.add_argument('--name', required=True,
                   help='the rule area name; a board-level rule area of this '
                        'name is replaced')
    shape = p.add_mutually_exclusive_group(required=True)
    shape.add_argument('--rect', type=float, nargs=4,
                       metavar=('X0', 'Y0', 'X1', 'Y1'),
                       help='rectangle corners, mm')
    shape.add_argument('--polygon', nargs='+', metavar='X,Y',
                       help='polygon vertices, mm, at least 3')
    p.add_argument('--ref', default=None,
                   help="coordinates are in this footprint's local frame as "
                        "the file stores it (its pads' frame), so the area "
                        "follows the part")
    p.add_argument('--layers', nargs='+', default=None,
                   help='copper layers (default: every copper layer)')
    p.add_argument('--forbid', nargs='+', default=['tracks', 'vias', 'copperpour'],
                   choices=list(RULE_AREA_FLAGS),
                   help='what the area does not allow (default: tracks vias '
                        'copperpour)')
    args = p.parse_args()

    from kicad_parser import parse_kicad_pcb, pcb_uses_name_nets
    from kicad_writer import generate_rule_area_sexpr
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(args.input_file)
    copper = list(getattr(pcb.board_info, 'copper_layers', None) or [])
    layers = args.layers or copper
    bad = [l for l in layers if copper and l not in copper]
    if bad:
        print(f"add_rule_area: error: {', '.join(bad)} not a copper layer of "
              f"this board ({', '.join(copper)})", file=sys.stderr)
        return 2
    pts, frame = area_points(args, pcb)

    if os.path.abspath(args.output_file) != os.path.abspath(args.input_file):
        from copy_board import copy_board
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(args.input_file, args.output_file)
    with open(args.output_file, encoding='utf-8', newline='') as f:
        text = f.read()
    text, replaced = remove_named_rule_areas(text, args.name)
    zone = generate_rule_area_sexpr(layers, pts, args.name,
                                    not_allowed=tuple(args.forbid),
                                    use_net_name=pcb_uses_name_nets(pcb))
    close = text.rstrip().rfind(')')
    text = text[:close].rstrip('\n') + '\n' + zone + '\n' + text[close:]
    with open(args.output_file, 'w', encoding='utf-8', newline='') as f:
        f.write(text)

    xs, ys = [x for x, _ in pts], [y for _, y in pts]
    print(f"add_rule_area: wrote rule area {args.name!r} on {', '.join(layers)}: "
          f"{', '.join(args.forbid)} not allowed; {len(pts)} points from "
          f"{frame}, bbox ({min(xs):.3f}, {min(ys):.3f})-({max(xs):.3f}, "
          f"{max(ys):.3f})"
          + (f"; replaced {replaced} earlier area(s) of that name" if replaced
             else ""))
    print(f"Wrote {args.output_file}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
