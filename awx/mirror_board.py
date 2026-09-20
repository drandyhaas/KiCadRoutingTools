#!/usr/bin/env python3
"""Write the MIRROR of a .kicad_pcb: the board turned over.

A board seen from its other face is the same board -- every part on the
other side, every track on the other layer, y mirrored -- so a chain
that names no face must grade the mirror exactly as the original. That
is the reflection twin of rotate_board.py's isometry gate: the pose gate
already puts an array on the other face with a FRESH fanout there, and
the fanout engine's own face asymmetry (a back-side BGA fans out with a
different structure) then hides whether the chain itself leans on F.
The mirror of a FANNED article keeps the copper, so the chain's plan and
braid are tested alone.

The transform is a half turn about the board's horizontal centre line in
3D: (x, y, F) -> (x, 2*CY - y, B). Footprints go through the placement
writer's mirror (the #714 path: side swapped, local y negated, pad
layers and angles folded) at the mirrored position with the negated
angle; board-level items (segment, via, zone, graphics) have their y
mirrored and their F.* / B.* layers swapped by a depth-aware walk over
the s-expression, like rotate_board.py. Self-verifies by re-parsing:
every pad, segment and via must land where the transform says, on the
swapped layer, or no file is left behind.

usage: mirror_board.py IN.kicad_pcb OUT.kicad_pcb
"""
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, os.path.join(HERE, '..', 'py_placer'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402

POINT_TOKENS = ('start', 'end', 'center', 'mid', 'xy', 'at')
LAYER_RE = re.compile(r'"([FB])\.([A-Za-z_]+)"')


def _fmt(v: float) -> str:
    """A coordinate as KiCad writes it: six decimals, trailing zeros
    dropped -- full precision, so a mirror mirrored back is the board."""
    t = f'{v:.6f}'.rstrip('0').rstrip('.')
    return '0' if t in ('', '-0') else t


def swap_layer_names(s):
    return LAYER_RE.sub(lambda m: f'"{"B" if m.group(1) == "F" else "F"}.{m.group(2)}"', s)


def mirror_items(txt, CY):
    """Board-level items only: y -> 2*CY - y on their global points,
    F.* <-> B.* on their layer tokens. Footprint blocks pass untouched
    (the writer has already mirrored them)."""
    out, i, n, stack = [], 0, len(txt), []
    while i < n:
        c = txt[i]
        if c == '(':
            j = i + 1
            while j < n and txt[j] not in ' \t\n)(':
                j += 1
            name = txt[i + 1:j]
            depth = len(stack)
            parent = stack[-1] if stack else None
            in_fp = 'footprint' in stack
            if depth == 2 and not in_fp and name in POINT_TOKENS:
                k = txt.index(')', j)
                parts = txt[j:k].split()
                try:
                    nums = [float(v) for v in parts]
                except ValueError:
                    nums = None
                if nums and len(nums) >= 2:
                    rest = ' ' + ' '.join(parts[2:]) if len(parts) > 2 else ''
                    out.append(f'({name} {parts[0]} {_fmt(2 * CY - nums[1])}{rest})')
                    i = k + 1
                    continue
            if depth == 2 and not in_fp and name in ('layer', 'layers'):
                k = txt.index(')', j)
                out.append('(' + name + swap_layer_names(txt[j:k]) + ')')
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
    pcb0 = parse_kicad_pcb(src)
    ys = [p.global_y for f in pcb0.footprints.values() for p in f.pads]
    CY = round((min(ys) + max(ys)) / 2 / 0.025) * 0.025    # on the lattice
    # 1. every footprint to the other face at its mirrored position
    pl = []
    for ref, fp in pcb0.footprints.items():
        pl.append({'reference': ref, 'new_x': fp.x, 'new_y': 2 * CY - fp.y,
                   'new_rotation': (-fp.rotation) % 360,
                   'new_side': 'F' if fp.layer.startswith('B') else 'B'})
    tmp = dst + '.fp'
    write_placed_output(src, tmp, pl)
    # 2. the board-level items
    txt = mirror_items(open(tmp, encoding='utf-8').read(), CY)
    os.remove(tmp)
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(txt)
    pro = os.path.splitext(src)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(dst)[0] + '.kicad_pro')
    # 3. verify against the parsed original
    pcb1 = parse_kicad_pcb(dst)
    bad = []

    def other(L):
        return ('B' if L.startswith('F') else 'F') + L[1:]
    for ref, f0 in pcb0.footprints.items():
        f1 = pcb1.footprints.get(ref)
        if f1 is None:
            bad.append(f'{ref} missing')
            continue
        p1 = {p.pad_number: p for p in f1.pads}
        for p in f0.pads:
            q = p1.get(p.pad_number)
            if q is None:
                bad.append(f'{ref}.{p.pad_number} missing')
                continue
            if abs(q.global_x - p.global_x) > 0.002 or abs(q.global_y - (2 * CY - p.global_y)) > 0.002:
                bad.append(f'{ref}.{p.pad_number} at ({q.global_x:.3f},{q.global_y:.3f}) '
                           f'!= ({p.global_x:.3f},{2 * CY - p.global_y:.3f})')
            want = sorted(other(L) if L[:2] in ('F.', 'B.') else L for L in p.layers)
            if sorted(q.layers) != want:
                bad.append(f'{ref}.{p.pad_number} layers {q.layers} != {want}')
    if len(pcb0.segments) != len(pcb1.segments) or len(pcb0.vias) != len(pcb1.vias):
        bad.append(f'counts: {len(pcb0.segments)}/{len(pcb0.vias)} -> '
                   f'{len(pcb1.segments)}/{len(pcb1.vias)}')
    for s0, s1 in zip(pcb0.segments, pcb1.segments):
        if (abs(s1.start_x - s0.start_x) > 0.002 or abs(s1.start_y - (2 * CY - s0.start_y)) > 0.002
                or abs(s1.end_x - s0.end_x) > 0.002 or abs(s1.end_y - (2 * CY - s0.end_y)) > 0.002
                or s1.layer != other(s0.layer)):
            bad.append(f'segment {s0.layer} ({s0.start_x:.3f},{s0.start_y:.3f}) -> '
                       f'{s1.layer} ({s1.start_x:.3f},{s1.start_y:.3f})')
            if len(bad) > 8:
                break
    for v0, v1 in zip(pcb0.vias, pcb1.vias):
        if abs(v1.x - v0.x) > 0.002 or abs(v1.y - (2 * CY - v0.y)) > 0.002:
            bad.append(f'via ({v0.x:.3f},{v0.y:.3f}) -> ({v1.x:.3f},{v1.y:.3f})')
    if bad:
        os.remove(dst)
        print('MIRROR FAILED:\n  ' + '\n  '.join(bad[:12]))
        return 2
    print(f'wrote {dst}: faithful mirror about y = {CY:.3f} -- {len(pcb1.footprints)} footprints, '
          f'{len(pcb1.segments)} segments, {len(pcb1.vias)} vias verified')
    return 0


if __name__ == '__main__':
    sys.exit(main())
