#!/usr/bin/env python3
"""The pair's FLOW FRAME: the chain runs every array pair in one pose.

The routing lattice's symmetries are the eight poses of a square --
four quarter turns, each with or without a mirror. The mirror half is
the chirality frame (select_moves.PairFrame, braid.setup's turn-over);
this is the quarter-turn half. A board is turned, as a FILE, by the
exact quarter turn that points the pair's source-to-destination
direction along +x, every stage of the chain runs on that file
unchanged, and the result is turned back. A pair dropped at any of the
four angles is then the identical computation to the letter -- there is
nothing to hunt stage by stage, because every stage sees one board.
Measured before this (tmp/frame/rot11.out, the engine's own quarter-turn
frame on): the fanout identical in all four poses, R0 = R180 through
the chain to the segment, R90 = R270 to the segment, and the two pairs
differing from each other by the plan's compass faces and the braid's
octilinear search leaning on one axis.

Exactness: the turn is (dx, dy) -> (-dy, dx) about a point on the 0.1
mm lattice, no trigonometry, so every routing grid (0.1 / 0.05 / 0.025,
all anchored at the origin) is its own image and a turned file turned
back is the file. Coordinates are written as KiCad writes them (six
decimals). Footprints turn by their placement (the stored angle runs
the other way from the geometric turn: it goes DOWN by the turn, the
pads' absolute angles with it), everything else by its global points --
the same depth-aware walk as rotate_board.py, whose pose gate this
replaces the trigonometry of. Self-verifies by re-parsing.

Which pose: the centroid of the run's source pads to the centroid of
its destination pads; the quarter turn k that makes that vector's
dominant component +x (a tie between |dx| and |dy| goes to x). Near a
diagonal a small move of a part flips k -- either frame is a legitimate
run and both are translation-invariant; it is a knife edge like K41's,
not a defect.

usage: flow_frame.py quarter BOARD DEST NET[,NET...]   -> "k cx cy"
       flow_frame.py turn IN OUT K CX CY                 (K may be negative)
"""
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

LATTICE = 0.1          # the turn's centre: a multiple of every routing grid
POINT_TOKENS = ('start', 'end', 'center', 'mid', 'xy', 'at')


def _q(dx, dy, k):
    """R(+90 k) in the parser's convention: (dx, dy) -> (-dy, dx)."""
    for _ in range(k % 4):
        dx, dy = -dy, dx
    return dx, dy


def _fmt(v):
    t = f'{v:.6f}'.rstrip('0').rstrip('.')
    return '0' if t in ('', '-0') else t


def quarter_of(pcb, dest, names):
    """(k, cx, cy): the quarter turn that points the run's source-to-
    destination centroid vector along +x, and the lattice point nearest
    the destination array's centre to turn about."""
    src, dst = [], []
    for nid, net in pcb.nets.items():
        nm = net.name.split('/')[-1]
        if nm not in names:
            continue
        for p in net.pads:
            (dst if p.component_ref == dest else src).append((p.global_x, p.global_y))
    if not src or not dst:
        return 0, 0.0, 0.0
    sx = sum(p[0] for p in src) / len(src)
    sy = sum(p[1] for p in src) / len(src)
    dx = sum(p[0] for p in dst) / len(dst) - sx
    dy = sum(p[1] for p in dst) / len(dst) - sy
    # the turn k for which the turned vector has the largest x (ties: the
    # smaller k), keys to a nanometre so a diagonal is a deterministic tie
    best = max(range(4), key=lambda k: (round(_q(dx, dy, k)[0], 6), -k))
    fp = pcb.footprints[dest]
    cx = round(fp.x / LATTICE) * LATTICE
    cy = round(fp.y / LATTICE) * LATTICE
    return best, cx, cy


def turn_text(txt, k, cx, cy):
    """The board file turned by k quarter turns about (cx, cy)."""
    deg = (90.0 * k) % 360.0

    def rot(x, y):
        dx, dy = _q(x - cx, y - cy, k)
        return cx + dx, cy + dy

    out = []
    i, n = 0, len(txt)
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
        j = i + 1
        while j < n and txt[j] not in ' \t\n)(':
            j += 1
        name = txt[i + 1:j]
        depth = len(stack)
        parent = stack[-1] if stack else None
        if (depth == 3 and name == 'at' and parent == 'pad'
                and len(stack) >= 2 and stack[-2] == 'footprint'):
            # a pad's stored angle is ABSOLUTE: it turns with the part
            kk = txt.index(')', j)
            parts = txt[j:kk].split()
            try:
                nums = [float(v) for v in parts]
            except ValueError:
                nums = None
            if nums and len(nums) >= 2:
                a0 = nums[2] if len(nums) > 2 else 0.0
                out.append(f'({name} {parts[0]} {parts[1]} {(a0 - deg) % 360:g})')
                i = kk + 1
                continue
        is_global = (depth == 2 and name in POINT_TOKENS and parent != 'footprint') \
            or (depth == 2 and name == 'at' and parent == 'footprint')
        if is_global:
            kk = txt.index(')', j)
            parts = txt[j:kk].split()
            try:
                nums = [float(v) for v in parts]
            except ValueError:
                nums = None
            if nums and len(nums) >= 2:
                x, y = rot(nums[0], nums[1])
                rest = ''
                if name == 'at' and parent == 'footprint' and len(nums) == 2:
                    rest = f' {(-deg) % 360:g}'
                elif len(nums) > 2:
                    if name == 'at':
                        rest = f' {(nums[2] - deg) % 360:g}'
                    else:
                        rest = ' ' + ' '.join(f'{v:g}' for v in nums[2:])
                out.append(f'({name} {_fmt(x)} {_fmt(y)}{rest})')
                stack.append(name)
                stack.pop()
                i = kk + 1
                continue
        out.append('(')
        stack.append(name)
        i += 1
    return ''.join(out)


SIBLINGS = ('.kicad_pro', '.ladder.txt', '.kicad_dru')


def turn_file(src, dst, k, cx, cy):
    """Write `src` turned by k quarter turns about (cx, cy) to `dst`,
    its siblings copied, and verify the pads, segments and vias against
    the same turn of the parsed original. Returns the list of faults."""
    k %= 4
    pcb0 = parse_kicad_pcb(src)
    txt = turn_text(open(src, encoding='utf-8').read(), k, cx, cy)
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(txt)
    for ext in SIBLINGS:
        side = os.path.splitext(src)[0] + ext
        if os.path.exists(side):
            shutil.copy(side, os.path.splitext(dst)[0] + ext)

    def rot(x, y):
        dx, dy = _q(x - cx, y - cy, k)
        return cx + dx, cy + dy

    def near(a, b, tol=0.002):
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

    pcb1 = parse_kicad_pcb(dst)
    bad = []
    for ref, f0 in pcb0.footprints.items():
        f1 = pcb1.footprints.get(ref)
        if f1 is None or len(f1.pads) != len(f0.pads):
            bad.append(f'footprint {ref} missing or pad count differs')
            continue
        for p0, p1 in zip(f0.pads, f1.pads):
            if not near(rot(p0.global_x, p0.global_y), (p1.global_x, p1.global_y)):
                bad.append(f'pad {ref}.{p0.pad_number}')
            exp = (p0.size_y, p0.size_x) if k % 2 else (p0.size_x, p0.size_y)
            if abs(exp[0] - p1.size_x) > 1e-3 or abs(exp[1] - p1.size_y) > 1e-3:
                bad.append(f'pad {ref}.{p0.pad_number} extents')
    if len(pcb0.segments) != len(pcb1.segments) or len(pcb0.vias) != len(pcb1.vias):
        bad.append('segment/via counts differ')
    for s0, s1 in zip(pcb0.segments, pcb1.segments):
        if not near(rot(s0.start_x, s0.start_y), (s1.start_x, s1.start_y)) \
                or not near(rot(s0.end_x, s0.end_y), (s1.end_x, s1.end_y)):
            bad.append('segment')
            break
    for v0, v1 in zip(pcb0.vias, pcb1.vias):
        if not near(rot(v0.x, v0.y), (v1.x, v1.y)):
            bad.append('via')
            break
    if bad:
        os.remove(dst)
    return bad


def main():
    if len(sys.argv) >= 5 and sys.argv[1] == 'quarter':
        board, dest, names = sys.argv[2], sys.argv[3], set(sys.argv[4].split(','))
        k, cx, cy = quarter_of(parse_kicad_pcb(board), dest, names)
        print(f'{k} {_fmt(cx)} {_fmt(cy)}')
        return 0
    if len(sys.argv) >= 7 and sys.argv[1] == 'turn':
        src, dst = sys.argv[2], sys.argv[3]
        k, cx, cy = int(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])
        bad = turn_file(src, dst, k, cx, cy)
        if bad:
            print('TURN FAILED: ' + '; '.join(bad[:6]))
            return 2
        print(f'wrote {dst}: {k % 4} quarter turn(s) about ({_fmt(cx)}, {_fmt(cy)})')
        return 0
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main())
