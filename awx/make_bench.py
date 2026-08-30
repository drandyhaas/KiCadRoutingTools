#!/usr/bin/env python3
"""Write a copy of a bench with ONE footprint rotated about its centre
and shifted -- the test article for a corridor that has no axis to
lean on: a destination at 45 degrees to the source, offset diagonally,
so the source is neither due west of it nor square to it.

Only the footprint's own `(at x y angle)` changes; everything nested
inside it is local and rides along (rotate_board.py's rule). The pads
of the moved part must be bare on the input (the chain fans them out
afterwards), because copper already attached to them is not moved.
Self-verifies by re-parsing: every pad of the moved footprint must land
where the transform says, and every other pad must not move.

usage: make_bench.py IN.kicad_pcb OUT.kicad_pcb REF --rot DEG [--dx MM --dy MM]
"""
import argparse
import math
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('ref')
    ap.add_argument('--rot', type=float, required=True,
                    help='degrees to ADD to the footprint rotation (KiCad '
                         'sense: counter-clockwise on screen)')
    ap.add_argument('--dx', type=float, default=0.0)
    ap.add_argument('--dy', type=float, default=0.0)
    ap.add_argument('--drop-colliding', action='store_true',
                    help='remove every other footprint a pad of which '
                         'lands within 0.3 mm of a moved pad (the bench '
                         'has decoupling caps under the array; a rotated '
                         'array sweeps over them)')
    a = ap.parse_args()
    pcb0 = parse_kicad_pcb(a.src)
    fp0 = pcb0.footprints[a.ref]
    attached = [s for s in pcb0.segments
                if any(abs(s.start_x - p.global_x) + abs(s.start_y - p.global_y)
                       < 0.01 or abs(s.end_x - p.global_x)
                       + abs(s.end_y - p.global_y) < 0.01 for p in fp0.pads)]
    if attached:
        print(f'refusing: {len(attached)} segment(s) already attach to '
              f'{a.ref} pads; move a bare part')
        return 2
    cx, cy = fp0.x, fp0.y
    txt = open(a.src, encoding='utf-8').read()

    def blocks(txt):
        """(ref, block_start, block_end, at_span) for every footprint,
        walking depth: a footprint's own (at ...) is the first level-2
        `at` after its header, before any pad."""
        out = []
        i = 0
        n = len(txt)
        stack = []
        in_string = False
        block_start = None
        at_span = None
        ref = None
        while i < n:
            ch = txt[i]
            if in_string:
                if ch == '"' and txt[i - 1] != '\\':
                    in_string = False
                i += 1
                continue
            if ch == '"':
                in_string = True
                i += 1
                continue
            if ch == '(':
                j = i + 1
                while j < n and txt[j] not in ' \t\r\n()':
                    j += 1
                tok = txt[i + 1:j]
                stack.append((tok, i))
                if tok == 'footprint' and len(stack) == 2:
                    block_start = i
                    at_span = None
                    ref = None
                if tok == 'at' and len(stack) == 3 and block_start is not None \
                        and at_span is None:
                    k = txt.find(')', i)
                    at_span = (i, k + 1)
                i = j
                continue
            if ch == ')':
                tok, s0 = stack.pop()
                if tok == 'property' and len(stack) == 2 and block_start is not None:
                    body = txt[s0:i]
                    if '"Reference"' in body:
                        q = body.split('"Reference"', 1)[1].split('"')
                        ref = q[1] if len(q) > 1 else None
                if tok == 'footprint' and len(stack) == 1 and block_start is not None:
                    e = i + 1
                    if e < n and txt[e] == '\n':
                        e += 1
                    out.append((ref, block_start, e, at_span))
                    block_start = None
                i += 1
                continue
            i += 1
        return out
    mine = [b for b in blocks(txt) if b[0] == a.ref]
    if not mine or mine[0][3] is None:
        print(f'footprint {a.ref} not found')
        return 2
    at_span = mine[0][3]
    old = txt[at_span[0]:at_span[1]]
    parts = old[1:-1].split()
    rot0 = float(parts[3]) if len(parts) > 3 else 0.0
    new_rot = (rot0 + a.rot) % 360
    new_at = f'(at {cx + a.dx:.6f} {cy + a.dy:.6f} {new_rot:g})'
    out = txt[:at_span[0]] + new_at + txt[at_span[1]:]
    dropped = []
    if a.drop_colliding:
        # where the moved pads will be, from the same transform the
        # verification below checks
        r = math.radians(a.rot)
        c, s = math.cos(r), math.sin(r)
        moved = []
        for p0 in fp0.pads:
            dx, dy = p0.global_x - cx, p0.global_y - cy
            moved.append((cx + c * dx + s * dy + a.dx, cy - s * dx + c * dy + a.dy,
                          max(p0.size_x, p0.size_y) / 2))
        for ref, f0 in pcb0.footprints.items():
            if ref == a.ref:
                continue
            hit = any(math.hypot(p.global_x - mx, p.global_y - my)
                      < mr + max(p.size_x, p.size_y) / 2 + 0.3
                      for p in f0.pads for (mx, my, mr) in moved)
            if hit:
                dropped.append(ref)
        for (ref, b0, b1, _at) in sorted(blocks(out), key=lambda b: -b[1]):
            if ref in dropped:
                out = out[:b0] + out[b1:]
    with open(a.dst, 'w', encoding='utf-8') as f:
        f.write(out)
    pro = os.path.splitext(a.src)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(a.dst)[0] + '.kicad_pro')
    # verify: the moved part's pads rotate about (cx, cy) by -rot in the
    # y-down frame KiCad's angle convention gives, then shift; nothing
    # else moves
    pcb1 = parse_kicad_pcb(a.dst)
    r = math.radians(a.rot)
    c, s = math.cos(r), math.sin(r)
    bad = 0
    for p0, p1 in zip(fp0.pads, pcb1.footprints[a.ref].pads):
        dx, dy = p0.global_x - cx, p0.global_y - cy
        ex = cx + c * dx + s * dy + a.dx
        ey = cy - s * dx + c * dy + a.dy
        if math.hypot(p1.global_x - ex, p1.global_y - ey) > 0.002:
            bad += 1
    for ref, f0 in pcb0.footprints.items():
        if ref == a.ref:
            continue
        if ref in dropped:
            if ref in pcb1.footprints:
                bad += 1
            continue
        for p0, p1 in zip(f0.pads, pcb1.footprints[ref].pads):
            if math.hypot(p1.global_x - p0.global_x, p1.global_y - p0.global_y) > 0.002:
                bad += 1
    if bad:
        os.remove(a.dst)
        print(f'VERIFY FAILED: {bad} pad(s) off -- output removed')
        return 1
    xs = [p.global_x for p in pcb1.footprints[a.ref].pads]
    ys = [p.global_y for p in pcb1.footprints[a.ref].pads]
    print(f'wrote {a.dst}: {a.ref} {old} -> {new_at}; pads now span '
          f'x [{min(xs):.2f},{max(xs):.2f}] y [{min(ys):.2f},{max(ys):.2f}]; '
          f'verified {len(fp0.pads)} moved + every other pad unmoved'
          + (f'; dropped {len(dropped)} colliding part(s): {dropped}'
             if dropped else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
