#!/usr/bin/env python3
"""A bus's ROW PART end -- a QFN (escape_moves.row_part, grid_of, enumerate_moves) -- and a shared source's escapes
kept (route_bus.escape_blocks).

  python3 tests/test_622_row_part.py

On parts stood in for (pads alone):

1. row_part: a QFN -- long pads in rows round its edge, an exposed pad inside -- is one; a ball array (round balls on
   a full grid) is not, nor a part with two pads inside its rows;
2. grid_of a row part: its pitch the pads' own along a row (the exposed pad's offset is no pitch), its box the rows'
   COPPER, its exposed pad left out of its columns and rows; a ball array's as before;
3. enumerate_moves at a row part: ONE move per pad, a surface escape straight out of its row on its own layer, from
   the pad's centre to half a pitch past the pads' outer ends; its clearance asked from the pad's outer end on (a
   model's disc round the exposed pad that reaches a pad's inner end refuses nothing), and none where that is not clear;
4. escape_blocks: the segments and vias of the named nets inside the array's box grown 1.5 mm, in either net dialect,
   and nothing outside it or of another net.
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import escape_moves as em  # noqa: E402

fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print('FAIL', msg)


def pad(n, x, y, sx, sy, net=''):
    return types.SimpleNamespace(pad_number=str(n), global_x=x, global_y=y, size_x=sx, size_y=sy, pad_type='smd',
                                 layers=['F.Cu', 'F.Mask'], net_name=net, net_id=1 if net else 0)


# a QFN-32 at 0.5 mm: eight pads a side, 0.9 x 0.28, their centres 2.45 mm from the middle, a 3.3 mm exposed pad
def qfn(inner=1):
    ps, n = [], 1
    for i in range(8):
        a = -1.75 + 0.5 * i
        ps += [pad(n, -2.45, a, 0.9, 0.28, f'L{i}'), pad(n + 1, a, 2.45, 0.28, 0.9, f'B{i}'),
               pad(n + 2, 2.45, -a, 0.9, 0.28, f'R{i}'), pad(n + 3, -a, -2.45, 0.28, 0.9, f'T{i}')]
        n += 4
    ps += [pad(n + i, 0.3 * i, 0.0, 3.3, 3.3, 'GND') for i in range(inner)]
    return types.SimpleNamespace(pads=ps)


bga = types.SimpleNamespace(pads=[pad(f'{r}{c}', 0.8 * c, 0.8 * r, 0.4, 0.4, 'N') for r in range(6) for c in range(6)])

# 1. row_part
check(em.row_part(qfn()), '1: a QFN is a row part')
check(not em.row_part(bga), '1: a ball array is not')
check(not em.row_part(qfn(inner=2)), '1: two pads inside the rows: not a row part')

# 2. grid_of
g = em.grid_of(qfn())
check(g.row and abs(g.pitch_x - 0.5) < 1e-9 and abs(g.pitch_y - 0.5) < 1e-9, f'2: the pads\' own pitch: {g}')
check(all(abs(a - b) < 1e-9 for a, b in zip(g.bbox, (-2.9, -2.9, 2.9, 2.9))), f'2: the rows\' copper: {g.bbox}')
check(0.0 not in g.xs, f'2: the exposed pad is no column: {g.xs}')
gb = em.grid_of(bga)
check(not gb.row and abs(gb.pitch_x - 0.8) < 1e-9 and gb.bbox == (0.0, 0.0, 4.0, 4.0), f'2: a ball array as before: {gb}')

# 3. enumerate_moves
fp = qfn()
EP_R = 2.64                                            # a model's circumscribed disc round the 3.3 mm exposed pad


def clear(p, q, L):
    """free of a disc of EP_R round the middle (sampled along p-q)"""
    for i in range(21):
        t = i / 20
        x, y = p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t
        if (x * x + y * y) ** 0.5 < EP_R:
            return False
    return True


want = {'L': ('left', (-3.15, None)), 'B': ('down', (None, 3.15)), 'R': ('right', (3.15, None)), 'T': ('up', (None, -3.15))}
for p in fp.pads[:-1]:
    mv = em.enumerate_moves(p, g, ['F.Cu', 'B.Cu'], clear)
    d, (ex, ey) = want[p.net_name[0]]
    e = (ex if ex is not None else p.global_x, ey if ey is not None else p.global_y)
    ok = (len(mv) == 1 and mv[0].kind == 'surface' and mv[0].direction == d and mv[0].layer == 'F.Cu'
          and mv[0].vias == 0 and abs(mv[0].exit_pt[0] - e[0]) < 1e-9 and abs(mv[0].exit_pt[1] - e[1]) < 1e-9
          and mv[0].legs == [((p.global_x, p.global_y), mv[0].exit_pt, 'F.Cu')])
    check(ok, f'3: pad {p.pad_number} ({p.net_name}): one escape {d} to {e}: {mv}')
blocked = em.enumerate_moves(fp.pads[0], g, ['F.Cu', 'B.Cu'], lambda p, q, L: False)
check(blocked == [], f'3: no move where its way out is not clear: {blocked}')

# 4. escape_blocks
import route_bus as rb  # noqa: E402
TXT = '''(kicad_pcb
\t(segment (start 0.5 0.5) (end 1.2 0.5) (width 0.12) (layer "F.Cu") (net "A") (uuid "s1"))
\t(segment (start 4.0 0.5) (end 6.0 0.5) (width 0.12) (layer "F.Cu") (net "A") (uuid "s2"))
\t(segment (start 0.5 1.0) (end 1.0 1.0) (width 0.12) (layer "B.Cu") (net 7) (uuid "s3"))
\t(segment (start 0.5 1.5) (end 1.0 1.5) (width 0.12) (layer "F.Cu") (net "C") (uuid "s4"))
\t(via (at 1.2 0.5) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") (net "A") (uuid "v1"))
\t(via (at 9.0 9.0) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") (net "A") (uuid "v2"))
)
'''
arr = types.SimpleNamespace(pads=[pad(1, 0.0, 0.0, 0.4, 0.4), pad(2, 2.0, 2.0, 0.4, 0.4)])
pcb = types.SimpleNamespace(footprints={'U1': arr}, nets={5: types.SimpleNamespace(name='A'),
                                                         7: types.SimpleNamespace(name='B'),
                                                         8: types.SimpleNamespace(name='C')})
got = rb.escape_blocks(TXT, pcb, 'U1', ['A', 'B'])
uu = sorted(b.split('(uuid "')[1][:2] for b in got)
check(uu == ['s1', 's3', 'v1'], f'4: the nets\' copper inside the box (+1.5 mm), either dialect: {uu}')

print('row part: ' + ('FAIL' if fails else 'PASS: a QFN found, its grid on its rows\' copper, one straight escape a pad '
                                          'checked from its outer end; a shared source\'s escapes picked out'))
sys.exit(1 if fails else 0)
