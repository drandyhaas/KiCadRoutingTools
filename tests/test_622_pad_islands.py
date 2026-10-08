#!/usr/bin/env python3
"""Islands made of pads (whole_ctx.part_islands): a lane may pass between two pads of one part when it fits.

  python3 tests/test_622_pad_islands.py

The whole route's geometry holds each lane to one side of every island. Pads -- of one part or of several -- closer on
a shared layer than a lane can surely pass between (a track, a clearance and the router's corner buffer either side,
and a grid step: 0.387 mm at a 0.127 track, 0.105 clearance and 0.025 grid) are one island; a part whole in an island
is named by its reference, a part split between islands by its reference and its pads. On parts of its own this pins:

1. an 0402 whose pads stand 0.42 mm apart is two islands ('C1:1', 'C1:2'); one whose pads stand 0.30 apart is one
   ('C2');
2. two parts 0.20 mm apart are one island ('C3+C4'), and one 0.40 apart two;
3. a part split, one of its pads joined to a neighbour: 'C7:1' and 'C7:2+C8';
4. pads on different layers are never joined, and a through-hole pad is on both;
5. the skipped parts (the arrays) carry no island;
6. a part of more than two pads (a pin header, 0.84 mm between pins) is ONE island: decided pin by pin, a lane wove
   through the row;
7. a two-pad part splits only where `split` allows it (the geometry: its pads across the lanes): with none given,
   every part is one island.
"""
import os
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import whole_ctx  # noqa: E402


def pad(num, x, y, sx, sy, layer='F.Cu', drill=0.0):
    return NS(pad_number=num, global_x=x, global_y=y, size_x=sx, size_y=sy, drill=drill,
              pad_type='thru_hole' if drill else 'smd', layers=['*.Cu'] if drill else [layer])


def two(x, y, gap, layer='F.Cu'):
    """a two-pad part on end: pads 0.64 wide, 0.54 tall, `gap` apart along y"""
    d = (gap + 0.54) / 2
    return NS(pads=[pad('1', x, y - d, 0.64, 0.54, layer), pad('2', x, y + d, 0.64, 0.54, layer)])


def main():
    print('=' * 60)
    print('islands made of pads: between two pads of one part when a lane fits')
    print('=' * 60)
    fps = {'C1': two(0, 0, 0.42), 'C2': two(5, 0, 0.30),
           'C3': two(10, 0, 0.30), 'C4': two(10.84, 0, 0.30),               # 0.84 - 0.64 = 0.20 apart in x
           'C5': two(15, 0, 0.30), 'C6': two(16.04, 0, 0.30),               # 0.40 apart
           'C7': two(20, 0, 0.60),
           'C8': NS(pads=[pad('1', 20, 0.60 / 2 + 0.54 + 0.2 + 0.27, 0.64, 0.54)]),   # 0.20 below C7's pad 2
           'C9': two(25, 0, 0.30, 'F.Cu'), 'C10': two(25, 0.9, 0.30, 'B.Cu'),       # overlapping, other layers
           'J1': NS(pads=[pad('1', 30, 0, 1.0, 1.0, drill=0.6)]),
           'C11': two(30, 0.9, 0.30, 'B.Cu'),                               # 0.1 below J1's barrel on B
           'U1': two(40, 0, 0.30),
           'J2': NS(pads=[pad(str(i + 1), 50 + 2.54 * i, 0, 1.7, 1.7, drill=1.0) for i in range(3)])}
    ctx = NS(cfg=NS(track_width=0.127, clearance=0.105, grid_step=0.025), pcb=NS(footprints=fps))
    isl = whole_ctx.part_islands(ctx, skip=('U1',), split=lambda ref: True)
    lab = lambda ref, i: isl.get((ref, i))
    want = {('C1', 0): 'C1:1', ('C1', 1): 'C1:2', ('C2', 0): 'C2', ('C2', 1): 'C2',
            ('C3', 0): 'C3+C4', ('C4', 1): 'C3+C4', ('C5', 0): 'C5', ('C6', 0): 'C6',
            ('C7', 0): 'C7:1', ('C7', 1): 'C7:2+C8', ('C8', 0): 'C7:2+C8',
            ('C9', 0): 'C9', ('C10', 0): 'C10', ('J1', 0): 'C11+J1', ('C11', 0): 'C11+J1',
            ('J2', 0): 'J2', ('J2', 1): 'J2', ('J2', 2): 'J2'}
    fails = [f'{k}: {lab(*k)!r}, want {v!r}' for k, v in want.items() if lab(*k) != v]
    if ('U1', 0) in isl:
        fails.append('a skipped part (an array) got an island')
    whole = whole_ctx.part_islands(ctx, skip=('U1',))
    if whole.get(('C1', 0)) != 'C1' or whole.get(('C1', 1)) != 'C1' or whole.get(('C7', 1)) != 'C7+C8':
        fails.append(f"no split given: C1 {whole.get(('C1', 0))!r}/{whole.get(('C1', 1))!r}, C7's pad 2 "
                     f"{whole.get(('C7', 1))!r} (want C1 whole, C7+C8)")
    only = whole_ctx.part_islands(ctx, skip=('U1',), split=lambda ref: ref == 'C7')
    if only.get(('C1', 0)) != 'C1' or only.get(('C7', 0)) != 'C7:1':
        fails.append(f"split for C7 only: C1 {only.get(('C1', 0))!r}, C7's pad 1 {only.get(('C7', 0))!r}")
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: a part split where a lane fits between its pads, joined where it does not; parts joined below the '
          'bar on a shared layer, never across layers; a barrel on both; a header one island; none split unless asked')
    return 0


if __name__ == '__main__':
    sys.exit(main())
