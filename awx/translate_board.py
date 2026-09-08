#!/usr/bin/env python3
"""Write a .kicad_pcb moved by (dx, dy): the third isometry of the pose gate.

A board moved on the sheet is the same board, so a chain that leans on
absolute coordinates anywhere -- a truncation onto a grid, a sort on a
raw distance, a memo keyed on position -- grades the moved board
differently. The fanout engine did (#622 pose gate, 2026-09-08: the
same array shifted 1 mm routed one ball out of the other face and the
rescue re-assigned five nets), and this makes the test an article.

Move by a multiple of 0.1 mm, so every routing grid (0.1, 0.05, 0.025)
is its own image and the only thing tested is the chain's arithmetic.
Footprints go through the placement writer at their moved position
(rotation and side unchanged); board-level items (segment, via, zone,
graphics) have their points moved by a depth-aware walk over the
s-expression, like rotate_board.py. Self-verifies by re-parsing.

usage: translate_board.py IN.kicad_pcb OUT.kicad_pcb DX DY
"""
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, os.path.join(HERE, '..', 'py_placer'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402
from mirror_board import POINT_TOKENS, _fmt  # noqa: E402


def move_items(txt, DX, DY):
    """Board-level items only: every global point moved. Footprint blocks
    pass untouched (the writer has already moved them)."""
    out, i, n, stack = [], 0, len(txt), []
    while i < n:
        c = txt[i]
        if c == '(':
            j = i + 1
            while j < n and txt[j] not in ' \t\n)(':
                j += 1
            name = txt[i + 1:j]
            depth = len(stack)
            in_fp = 'footprint' in stack
            if depth >= 2 and not in_fp and name in POINT_TOKENS:
                k = txt.index(')', j)
                parts = txt[j:k].split()
                try:
                    nums = [float(v) for v in parts]
                except ValueError:
                    nums = None
                if nums and len(nums) >= 2:
                    rest = ' ' + ' '.join(parts[2:]) if len(parts) > 2 else ''
                    out.append(f'({name} {_fmt(nums[0] + DX)} {_fmt(nums[1] + DY)}{rest})')
                    i = k + 1
                    continue
            stack.append(name)
            out.append(txt[i:j])
            i = j
            continue
        if c == ')':
            if stack:
                stack.pop()
        out.append(c)
        i += 1
    return ''.join(out)


def main():
    src, dst = sys.argv[1], sys.argv[2]
    DX, DY = float(sys.argv[3]), float(sys.argv[4])
    for v in (DX, DY):
        if abs(v / 0.1 - round(v / 0.1)) > 1e-9:
            print(f'move by a multiple of 0.1 mm (got {v}), so the routing grids stay their own image')
            return 2
    pcb0 = parse_kicad_pcb(src)
    pl = [{'reference': ref, 'new_x': fp.x + DX, 'new_y': fp.y + DY,
           'new_rotation': fp.rotation} for ref, fp in pcb0.footprints.items()]
    tmp = dst + '.fp'
    write_placed_output(src, tmp, pl)
    txt = move_items(open(tmp, encoding='utf-8').read(), DX, DY)
    os.remove(tmp)
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(txt)
    pro = os.path.splitext(src)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(dst)[0] + '.kicad_pro')
    pcb1 = parse_kicad_pcb(dst)
    bad = []
    for ref, f0 in pcb0.footprints.items():
        f1 = pcb1.footprints.get(ref)
        if f1 is None:
            bad.append(f'{ref} missing')
            continue
        p1 = {p.pad_number: p for p in f1.pads}
        for p in f0.pads:
            q = p1.get(p.pad_number)
            if q is None or abs(q.global_x - (p.global_x + DX)) > 0.002 or abs(q.global_y - (p.global_y + DY)) > 0.002:
                bad.append(f'{ref}.{p.pad_number}')
    if len(pcb0.segments) != len(pcb1.segments) or len(pcb0.vias) != len(pcb1.vias):
        bad.append(f'counts {len(pcb0.segments)}/{len(pcb0.vias)} -> {len(pcb1.segments)}/{len(pcb1.vias)}')
    for s0, s1 in zip(pcb0.segments, pcb1.segments):
        if abs(s1.start_x - (s0.start_x + DX)) > 0.002 or abs(s1.start_y - (s0.start_y + DY)) > 0.002 \
                or abs(s1.end_x - (s0.end_x + DX)) > 0.002 or abs(s1.end_y - (s0.end_y + DY)) > 0.002:
            bad.append(f'segment ({s0.start_x:.3f},{s0.start_y:.3f})')
            if len(bad) > 8:
                break
    for v0, v1 in zip(pcb0.vias, pcb1.vias):
        if abs(v1.x - (v0.x + DX)) > 0.002 or abs(v1.y - (v0.y + DY)) > 0.002:
            bad.append(f'via ({v0.x:.3f},{v0.y:.3f})')
    if bad:
        os.remove(dst)
        print('TRANSLATE FAILED:\n  ' + '\n  '.join(bad[:12]))
        return 2
    print(f'wrote {dst}: faithful move by ({DX}, {DY}) -- {len(pcb1.footprints)} footprints, '
          f'{len(pcb1.segments)} segments, {len(pcb1.vias)} vias verified')
    return 0


if __name__ == '__main__':
    sys.exit(main())
