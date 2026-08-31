#!/usr/bin/env python3
"""Write a rotated copy of a .kicad_pcb, to test the flow frame against
a board whose source is NOT due west of its destination.

Coordinates in a .kicad_pcb are NOT all global: a footprint's own
`(at x y angle)` is a board position, but everything nested inside it --
pad `(at ...)`, graphic `(start ...)` -- is LOCAL to the footprint and
must be left alone, since rotating the footprint's placement angle
already carries it. A flat regex over every `(at ...)` gets this wrong;
this walks the s-expression tracking depth.

Rotating a board file is only trustworthy if it is checked, so this
SELF-VERIFIES: it re-parses its own output and compares every pad,
segment and via against the same rotation applied to the parsed
original, and refuses to leave a bad file behind.

usage: rotate_board.py IN.kicad_pcb OUT.kicad_pcb DEGREES
"""
import math
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

src, dst, deg = sys.argv[1], sys.argv[2], float(sys.argv[3])
pcb0 = parse_kicad_pcb(src)
xs = [p.global_x for f in pcb0.footprints.values() for p in f.pads]
ys = [p.global_y for f in pcb0.footprints.values() for p in f.pads]
CX, CY = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
# Snap the rotation centre to the 0.025 routing lattice, so an
# orthogonal rotation maps lattice-aligned geometry to lattice-aligned
# geometry. The gate tests isometry invariance; without the snap every
# coordinate also shifts grid PHASE by a half-cell-ish offset, and any
# quantisation sensitivity in the engines confounds the reading. (Not
# claimed as the cap-graze cause -- the bench's own cap pads sit
# off-lattice unrotated and grade clean; the grazes track which nets
# the per-rotation plan leaves to the engine's under-pad rescue.)
CX, CY = round(CX / 0.025) * 0.025, round(CY / 0.025) * 0.025
r = math.radians(deg)
C, S = math.cos(r), math.sin(r)


def rot(x, y):
    dx, dy = x - CX, y - CY
    return (CX + C * dx - S * dy, CY + S * dx + C * dy)


txt = open(src, encoding='utf-8').read()

# Walk the s-expression. `stack` holds the token name opening each level.
# Level 0 is (kicad_pcb ...); its children (segment, via, gr_line,
# footprint, zone, ...) are level 1; their fields are level 2.
POINT_TOKENS = ('start', 'end', 'center', 'mid', 'xy', 'at')
out = []
i = 0
n = len(txt)
stack = []
in_string = False
while i < n:
    ch = txt[i]
    if in_string:
        out.append(ch)
        if ch == '"' and txt[i - 1] != '\\':
            in_string = False
        i += 1
        continue
    if ch == '"':
        in_string = True
        out.append(ch)
        i += 1
        continue
    if ch == ')':
        if stack:
            stack.pop()
        out.append(ch)
        i += 1
        continue
    if ch != '(':
        out.append(ch)
        i += 1
        continue
    # at an open paren: read the token name
    j = i + 1
    while j < n and txt[j] not in ' \t\n)(':
        j += 1
    name = txt[i + 1:j]
    depth = len(stack)
    parent = stack[-1] if stack else None
    if (depth == 3 and name == 'at' and parent == 'pad'
            and len(stack) >= 2 and stack[-2] == 'footprint'):
        # A pad's stored angle is ABSOLUTE (the footprint's angle is
        # included -- the same convention as ref_label). Rotating the
        # footprint's placement angle without touching these leaves
        # every rectangular pad's stored ORIENTATION stale: the pad
        # centres verify (position is local + fp angle) while the
        # rectangle does not, and the obstacle models then price 0402
        # cap pads ~30 um thin on one axis -- the rotated-bench-only
        # cap-graze DRC class (9/22/17 at 90/180/270, found 0831).
        # Local x y are kept verbatim; only the angle turns.
        k = txt.index(')', j)
        parts = txt[j:k].split()
        try:
            nums = [float(v) for v in parts]
        except ValueError:
            nums = None
        if nums and len(nums) >= 2:
            a0 = nums[2] if len(nums) > 2 else 0.0
            out.append(f'({name} {parts[0]} {parts[1]} '
                       f'{(a0 - deg) % 360:g})')
            i = k + 1
            continue
    # A point is global only at level 2 whose parent is a top-level item
    # that is NOT a footprint, plus the footprint's own placement `at`.
    is_global = (
        depth == 2 and name in POINT_TOKENS and parent != 'footprint'
    ) or (depth == 2 and name == 'at' and parent == 'footprint')
    if is_global:
        k = txt.index(')', j)
        parts = txt[j:k].split()
        try:
            nums = [float(v) for v in parts]
        except ValueError:
            nums = None
        if nums and len(nums) >= 2:
            x, y = rot(nums[0], nums[1])
            rest = ''
            if name == 'at' and parent == 'footprint' and len(nums) == 2:
                # A footprint at rotation 0 stores NO angle. Turning the
                # world still turns the part, so the angle has to be
                # ADDED -- without this, every unrotated footprint kept
                # its original pad orientation while its centre moved.
                rest = f' {(-deg) % 360:g}'
            elif len(nums) > 2:
                if name == 'at':
                    # KiCad applies R(-angle) to a footprint's locals, so
                    # turning the WORLD by +deg means the stored angle
                    # goes DOWN. With +deg the pads of every 2-pad part
                    # landed in each other's places -- which is what the
                    # self-verify caught.
                    rest = f' {(nums[2] - deg) % 360:g}'
                else:
                    rest = ' ' + ' '.join(f'{v:g}' for v in nums[2:])
            out.append(f'({name} {x:.6f} {y:.6f}{rest})')
            stack.append(name)
            stack.pop()
            i = k + 1
            continue
    out.append('(')
    stack.append(name)
    i += 1

open(dst, 'w', encoding='utf-8').write(''.join(out))
pro = os.path.splitext(src)[0] + '.kicad_pro'
if os.path.exists(pro):
    shutil.copy(pro, os.path.splitext(dst)[0] + '.kicad_pro')

# --- self-verify against the same rotation applied to the parsed input
pcb1 = parse_kicad_pcb(dst)
bad = []


def near(a, b, tol=0.002):
    return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol


for ref, f0 in pcb0.footprints.items():
    f1 = pcb1.footprints.get(ref)
    if f1 is None or len(f1.pads) != len(f0.pads):
        bad.append(f'footprint {ref} missing or pad count differs')
        continue
    for p0, p1 in zip(f0.pads, f1.pads):
        if not near(rot(p0.global_x, p0.global_y),
                    (p1.global_x, p1.global_y)):
            bad.append(f'pad {ref}.{p0.pad_number} '
                       f'{rot(p0.global_x, p0.global_y)} != '
                       f'({p1.global_x:.3f},{p1.global_y:.3f})')
        # the rectangle must turn WITH the board, not just its centre
        # -- centres alone let the stale-pad-angle bug through
        if deg % 90 == 0:
            exp = ((p0.size_y, p0.size_x) if deg % 180 == 90
                   else (p0.size_x, p0.size_y))
            if (abs(exp[0] - p1.size_x) > 1e-3
                    or abs(exp[1] - p1.size_y) > 1e-3):
                bad.append(f'pad {ref}.{p0.pad_number} extents '
                           f'({p1.size_x:.3f},{p1.size_y:.3f}) != '
                           f'rotated ({exp[0]:.3f},{exp[1]:.3f})')
for a, b, what in ((pcb0.segments, pcb1.segments, 'segment'),
                   (pcb0.vias, pcb1.vias, 'via')):
    if len(a) != len(b):
        bad.append(f'{what} count {len(a)} != {len(b)}')
for s0, s1 in zip(pcb0.segments, pcb1.segments):
    if not near(rot(s0.start_x, s0.start_y), (s1.start_x, s1.start_y)) \
            or not near(rot(s0.end_x, s0.end_y), (s1.end_x, s1.end_y)):
        bad.append(f'segment {rot(s0.start_x, s0.start_y)} != '
                   f'({s1.start_x:.3f},{s1.start_y:.3f})')
for v0, v1 in zip(pcb0.vias, pcb1.vias):
    if not near(rot(v0.x, v0.y), (v1.x, v1.y)):
        bad.append(f'via {rot(v0.x, v0.y)} != ({v1.x:.3f},{v1.y:.3f})')
if bad:
    print(f'REFUSED: {len(bad)} geometry mismatches -- {dst} is NOT a '
          f'faithful {deg:g} deg rotation')
    for m in bad[:6]:
        print('   ', m)
    sys.exit(2)
print(f'wrote {dst}: faithful {deg:g} deg rotation about '
      f'({CX:.2f},{CY:.2f}) -- {len(pcb1.footprints)} footprints, '
      f'{len(pcb1.segments)} segments, {len(pcb1.vias)} vias verified')
