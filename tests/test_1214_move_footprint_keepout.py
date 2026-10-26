#!/usr/bin/env python3
"""#1214: moving or rotating a footprint must carry its own keep-out zone.

A footprint-internal `(zone ...)` stores its `(polygon (pts (xy ...)))` in
ABSOLUTE board coordinates (the same fact the flip in
`py_placer/placement/writer.py` mirrors it about), and so does a footprint
`(point ...)`, which each case also carries on one of the pads. The move and rotate
path of `write_placed_output` rewrites the footprint's `(at ...)`, rotates
pad and text angles, but leaves the zone's `(pts ...)` untouched, so moving a
footprint leaves its keep-out zone behind at the old absolute position.

This test injects a footprint-internal zone whose corners sit ON the
footprint's own pads, moves and rotates the footprint, and asserts every
corner is still on its pad as the parser reads the output back. The pads are
the reference because `kicad_parser.local_to_global` places them the way
pcbnew does (the GUI parity gates hold the two bit-identical), so they carry
KiCad's rotation sense. A test that re-derives the rotation itself would
share any sign mistake in the writer and pass it: the first version of this
fix turned zones the opposite way to their pads, and its test agreed.

No case rotates by 180 degrees, because both rotation senses give the same
answer there.

    python3 tests/test_1214_move_footprint_keepout.py
"""
from __future__ import annotations

import math
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)

BOARD = os.path.join(REPO, 'kicad_files', 'tigard.kicad_pcb')

# (ref, dx, dy, delta_rot). J7 sits at 0 degrees, J4 at -90, so the last case
# composes a turn with a pose that was already turned.
CASES = [
    ('J7', 5.0, 3.0, 90.0),
    ('J7', 5.0, 3.0, 0.0),
    ('J7', -2.0, 4.0, 30.0),
    ('J4', -4.0, 2.0, 270.0),
]

# Zone corner to pad centre. Pads are snapped to KiCad's nanometre grid and
# the writer prints six decimals, so a correct move lands within ~1e-6 mm.
TOL = 1e-5


def _zone_sexpr(pts):
    """A footprint-internal keep-out zone with the given absolute (x, y) pts."""
    body = " ".join(f"(xy {x:.6f} {y:.6f})" for x, y in pts)
    return (
        "(zone\n"
        "\t\t\t(layer \"B.Cu\")\n"
        "\t\t\t(uuid \"12140000-0000-0000-0000-000000000000\")\n"
        "\t\t\t(hatch full 0.508)\n"
        "\t\t\t(connect_pads (clearance 0))\n"
        "\t\t\t(min_thickness 0.254)\n"
        "\t\t\t(keepout (tracks not_allowed) (vias not_allowed) "
        "(pads not_allowed) (copperpour not_allowed) (footprints not_allowed))\n"
        "\t\t\t(polygon (pts " + body + "))\n"
        "\t\t)"
    )


def _inject_zone(src, dst, ref, pts):
    """Copy `src` to `dst`, adding a keep-out zone to the footprint `ref`."""
    from kicad_parser import iter_footprint_blocks
    text = open(src, encoding='utf-8').read()
    out = None
    for s, e, fp_text, _raw, key in iter_footprint_blocks(text):
        if key != ref:
            continue
        close = fp_text.rindex(')')
        point = (f'(point (at {pts[0][0]:.6f} {pts[0][1]:.6f}) (size 1) '
                 f'(layer "F.Fab") (uuid "12140000-0000-0000-0000-000000000001"))')
        body = (fp_text[:close] + '\n\t\t' + _zone_sexpr(pts) + '\n\t\t'
                + point + '\n\t' + fp_text[close:])
        out = text[:s] + body + text[e:]
        break
    if out is None:
        raise AssertionError(f"{ref} not found in {src} -- fixture stale")
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(out)
    return dst


def _zone_pts(path, ref):
    """Return the absolute (x, y) pts of the first zone in footprint `ref`."""
    from kicad_parser import iter_footprint_blocks
    text = open(path, encoding='utf-8').read()
    for _s, _e, fp_text, _raw, key in iter_footprint_blocks(text):
        if key != ref:
            continue
        m = re.search(r'\(pts\s+((?:\(xy\s+[-\d.]+\s+[-\d.]+\)\s*)+)\)', fp_text)
        if not m:
            return None
        pts = []
        for x, y in re.findall(r'\(xy\s+([-\d.]+)\s+([-\d.]+)\)', m.group(1)):
            pts.append((float(x), float(y)))
        return pts
    return None


def _anchor_pads(fp):
    """Indices into `fp.pads`, ordered by angle about their centroid.

    Every pad, so the polygon stays a real area even where a row of signal
    pads is collinear (J7's are; its mounting pads are not), and ordered by
    angle so the polygon does not cross itself.
    """
    cx = sum(p.global_x for p in fp.pads) / len(fp.pads)
    cy = sum(p.global_y for p in fp.pads) / len(fp.pads)
    return sorted(range(len(fp.pads)), key=lambda i: math.atan2(
        fp.pads[i].global_y - cy, fp.pads[i].global_x - cx))


def _run_case(base, tmp, ref, dx, dy, delta_rot):
    """Move `ref` with a pad-anchored zone; return (problems, input board)."""
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output

    fp = base.footprints[ref]
    old_rot = fp.rotation or 0.0
    order = _anchor_pads(fp)
    anchors = [(fp.pads[i].global_x, fp.pads[i].global_y) for i in order]
    reach = max(math.hypot(x - fp.x, y - fp.y) for x, y in anchors)
    if reach < 1.0:
        raise AssertionError(
            f"{ref}: its pads lie within {reach:.3f} mm of the origin, so a "
            f"rotation barely moves them and this case tests nothing -- "
            f"fixture stale")

    tag = f"{ref}_{delta_rot:g}"
    src = _inject_zone(BOARD, os.path.join(tmp, f'in_{tag}.kicad_pcb'), ref,
                       anchors)
    out = os.path.join(tmp, f'out_{tag}.kicad_pcb')
    write_placed_output(src, out, [{
        'reference': ref, 'new_x': fp.x + dx, 'new_y': fp.y + dy,
        'new_rotation': (old_rot + delta_rot) % 360}])

    case = (f"{ref} moved by ({dx}, {dy}) and turned {delta_rot:g} deg "
            f"from {old_rot:g}")
    got = _zone_pts(out, ref)
    if got is None:
        return [f"{case}: the output footprint lost its keep-out zone"], src
    moved = parse_kicad_pcb(out)
    pads = moved.footprints[ref].pads
    if len(pads) != len(fp.pads) or len(got) != len(order):
        return [f"{case}: {len(fp.pads)} pads / {len(order)} corners went in, "
                f"{len(pads)} pads / {len(got)} corners came out"], src

    problems = []
    worst = max(math.hypot(zx - pads[i].global_x, zy - pads[i].global_y)
                for (zx, zy), i in zip(got, order))
    if worst > TOL:
        problems.append(
            f"{case}: a zone corner ended {worst:.4f} mm from the pad it was "
            f"drawn on -- the zone did not move with its pads")
    pm = None
    for _s, _e, fp_text, _raw, key in __import__('kicad_parser').iter_footprint_blocks(
            open(out, encoding='utf-8').read()):
        if key == ref:
            pm = re.search(r'\(point\s+\(at\s+([-\d.]+)\s+([-\d.]+)\)', fp_text)
    if pm is None:
        problems.append(f"{case}: the output footprint lost its (point ...)")
    else:
        off = math.hypot(float(pm.group(1)) - pads[order[0]].global_x,
                         float(pm.group(2)) - pads[order[0]].global_y)
        if off > TOL:
            problems.append(f"{case}: the (point ...) ended {off:.4f} mm from "
                            f"the pad it was drawn on")
    # The parser must serve the moved zone as the footprint's keep-out.
    want = [(round(x, 6), round(y, 6)) for x, y in got]
    seen = [[(round(x, 6), round(y, 6)) for x, y in k['polygon']]
            for k in moved.board_info.keepouts if k.get('in_footprint')]
    if want not in seen:
        problems.append(f"{case}: the parser does not read the moved "
                        f"keep-out back")
    return problems, src


def main():
    from kicad_parser import parse_kicad_pcb
    from placement.writer import SideFlipUnsupported, write_placed_output

    if not os.path.isfile(BOARD):
        print(f"ERROR: board not found: {BOARD}")
        return 2

    tmp = tempfile.mkdtemp(prefix='krt1214')
    problems = []
    try:
        base = parse_kicad_pcb(BOARD)
        srcs = {}
        for ref, dx, dy, delta_rot in CASES:
            got, src = _run_case(base, tmp, ref, dx, dy, delta_rot)
            problems.extend(got)
            srcs.setdefault(ref, src)

        # A fill is absolute too and this move does not carry it, so a zone
        # with one must refuse and say why rather than leave the fill behind.
        fp = base.footprints['J7']
        filled = os.path.join(tmp, 'filled.kicad_pcb')
        text = open(srcs['J7'], encoding='utf-8').read().replace(
            "\t\t\t(polygon (pts ",
            "\t\t\t(filled_polygon (layer \"B.Cu\") (pts (xy 0 0) (xy 1 0) (xy 1 1)))\n"
            "\t\t\t(polygon (pts ", 1)
        with open(filled, 'w', encoding='utf-8') as fh:
            fh.write(text)
        try:
            write_placed_output(filled, os.path.join(tmp, 'filled-out.kicad_pcb'), [{
                'reference': 'J7', 'new_x': fp.x + 5.0, 'new_y': fp.y + 3.0,
                'new_rotation': ((fp.rotation or 0.0) + 90.0) % 360}])
            problems.append("a zone with a (filled_polygon ...) moved without refusing")
        except SideFlipUnsupported as e:
            if 'filled_polygon' not in str(e):
                problems.append(f"the fill refusal does not name its construct: {e}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if problems:
        print(f"FAIL: {len(problems)} problem(s)")
        for p in problems:
            print("  " + p)
        return 1
    print(f"PASS: {len(CASES)} moves and rotations carry the keep-out zone "
          f"and the point with the footprint's pads")
    return 0


if __name__ == '__main__':
    sys.exit(main())
