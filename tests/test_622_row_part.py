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
   and nothing outside it or of another net;
5. a ONE-ROW part whose body's pads are under the mounting-tab size (a card socket): its pin line is its row, out away
   from its body, a two-row pack not read so; with Inward, vias in the strip behind its pins, inside its body, and a
   via IN a pin the via fits in, out on another layer toward the body's side -- never in a pin narrower than the via,
   nor in a QFN's pad, and neither for a row facing its source;
6. whole_frame.group_of_one: a single row turned from the source (its lanes under the body to their pins) is a group
   of one, one facing the source is not, and a QFN is whatever the source;
7. a card's own EDGE FINGERS (connect pads, no other copper, the origin on their line, one finger shorter and its
   centre off the others'): one row, its way out AWAY from the board's edge they stand at -- told with the board
   alone, as the router reads the finger strip; none without the board, nor for the part inside the board -- and each
   finger's one escape out of its inner end.
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

# 5. a ONE-ROW part whose other copper is not all mounting tabs (a card socket: eight 1.1 x 0.75 mm pins at 1.1 mm in a
# column, its card-detect pin and tabs about its body to the +x side, none four times a pin's area)
def socket():
    ps = [pad(i + 1, 0.0, 1.1 * i, 1.1, 0.75, f'S{i}') for i in range(8)]
    ps += [pad(9, 6.2, -2.6, 1.0, 1.2, 'CD'), pad('MP', 0.35, 8.9, 1.8, 1.17), pad('MP', 10.9, 6.8, 1.35, 1.9),
           pad('MP', 10.9, 0.9, 1.35, 1.9), pad('MP', 0.1, -2.4, 1.35, 1.55), pad('MP', 9.9, -2.6, 1.0, 1.2)]
    return types.SimpleNamespace(pads=ps)


sk = socket()
gs = em.grid_of(sk)
check(em.row_part(sk) and gs.row and gs.out == 'left', f'5: a socket\'s pin column is its row, out away from its body: {gs}')
check(abs(gs.pitch_x - 1.1) < 1e-9 and abs(gs.bbox[0] + 0.55) < 1e-9 and abs(gs.bbox[2] - 0.55) < 1e-9,
      f'5: its pitch the pins\', its box their copper: {gs}')
# (a two-row pack of short pads, four a side: a row part of BOTH its rows -- every pad one of its row's, each its way
# out straight out of its row, so the part has no one way out -- and never one row with a body behind it)
pack = types.SimpleNamespace(pads=[pad(i + 1, 0.0, 0.5 * i, 0.5, 0.4) for i in range(4)]
                             + [pad(i + 5, 1.6, 0.5 * i, 0.5, 0.4) for i in range(4)])
gp = em.grid_of(pack)
check(em.row_part(pack) and gp.row and gp.out is None and len(em._row_pads(pack)[0]) == 8
      and abs(gp.pitch_y - 0.5) < 1e-9,
      f'5: a two-row pack of short pads is a row part of both its rows, with no one way out: {gp}')
# (...its pads square, none says which way its row runs: no row part)
square = types.SimpleNamespace(pads=[pad(i + 1, 0.0, 0.5 * i, 0.4, 0.4) for i in range(4)]
                               + [pad(i + 5, 1.6, 0.5 * i, 0.4, 0.4) for i in range(4)])
check(not em.row_part(square), '5: a two-row pack of square pads is no row part')
# (...its two rows nearer than a pad's length, or of unlike counts: not two rows)
near = types.SimpleNamespace(pads=[pad(i + 1, 0.0, 0.5 * i, 0.5, 0.4) for i in range(4)]
                             + [pad(i + 5, 0.45, 0.5 * i, 0.5, 0.4) for i in range(4)])
odd = types.SimpleNamespace(pads=[pad(i + 1, 0.0, 0.5 * i, 0.5, 0.4) for i in range(4)]
                            + [pad(i + 5, 1.6, 0.5 * i, 0.5, 0.4) for i in range(3)])
check(em._two_rows(near.pads) is None and em._two_rows(odd.pads) is None and em._two_rows(pack.pads) is not None,
      '5: two rows are two lines a pad\'s length apart with as many pads on each')
check(em.turned_from(sk, (25.0, 3.0)) and not em.turned_from(sk, (-20.0, 3.0))
      and not em.turned_from(sk, (0.5, 30.0)) and not em.turned_from(qfn(), (25.0, 3.0)),
      '5: turned from a source behind its body; not one in front, not one beside its line, not a QFN')
iw = em.Inward('right', 0.25, 0.09, 0.0762, turned=True)
pin = sk.pads[3]
ms = em.enumerate_moves(pin, gs, ['F.Cu', 'In1.Cu', 'B.Cu'], lambda p, q, L: True, lambda s, L: True, row_inward=iw)
inner = [m for m in ms if m.inner and m.kind == 'dogbone']
check(inner and all(m.site[0] > 0.55 for m in inner), f'5: vias inside the body, behind the pins: {inner[:2]}')
vip = [m for m in ms if m.kind == 'via_in_pad']
check(vip and all(m.site == (pin.global_x, pin.global_y) and m.layer != 'F.Cu' and m.direction == 'right'
                  for m in vip), f'5: the via in the pin, out on another layer toward the body\'s side: {vip}')
big = em.enumerate_moves(pin, gs, ['F.Cu', 'In1.Cu', 'B.Cu'], lambda p, q, L: True, lambda s, L: True,
                         row_inward=em.Inward('right', 0.8, 0.09, 0.0762, turned=True))
check(not [m for m in big if m.kind == 'via_in_pad'], '5: no via in a pin narrower than the via')
qm = em.enumerate_moves(fp.pads[0], g, ['F.Cu', 'B.Cu'], clear, lambda s, L: True,
                        row_inward=em.Inward('right', 0.25, 0.09, 0.0762, turned=True))
check(not [m for m in qm if m.kind == 'via_in_pad'], '5: a QFN (rows round its edge) takes no via in a pad')
fac = em.enumerate_moves(pin, gs, ['F.Cu', 'In1.Cu', 'B.Cu'], lambda p, q, L: True, lambda s, L: True,
                         row_inward=em.Inward('left', 0.25, 0.09, 0.0762, turned=False))
check(not [m for m in fac if m.inner or m.kind == 'via_in_pad'],
      f'5: a row FACING its source (its lanes leave by its own face) takes neither: {fac}')

# 6. the frame's group of one: a single row TURNED FROM the source (the source on its body's side) -- its lanes under
# the body to their pins -- and not one facing it
import whole_frame as wf  # noqa: E402
src_east = types.SimpleNamespace(pads=[pad('A1', 25.0, 3.0, 0.3, 0.3)])
src_west = types.SimpleNamespace(pads=[pad('A1', -20.0, 3.0, 0.3, 0.3)])
check(wf.group_of_one(types.SimpleNamespace(footprints={'J': sk, 'U': src_east}), ['J'], 'U'),
      '6: a row turned from its source is a group of one')
check(not wf.group_of_one(types.SimpleNamespace(footprints={'J': sk, 'U': src_west}), ['J'], 'U'),
      '6: a row facing its source is not')
src_side = types.SimpleNamespace(pads=[pad('A1', 0.5, 30.0, 0.3, 0.3)])
check(not wf.group_of_one(types.SimpleNamespace(footprints={'J': sk, 'U': src_side}), ['J'], 'U'),
      '6: nor one whose source stands beside its line (the same test as the menu\'s, escape_moves.turned_from)')
check(wf.group_of_one(types.SimpleNamespace(footprints={'Q': qfn(), 'U': src_west}), ['Q'], 'U'),
      '6: a QFN (rows on more than one line) is, whatever the source')

# 7. a card's own edge fingers, on a board written from s-expressions (the finger-strip test's: four fingers on the
# bottom edge, the fourth shorter, its inner end in line)
import tempfile  # noqa: E402
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import test_2022_finger_band as fb  # noqa: E402
with tempfile.TemporaryDirectory() as tmp:
    pe = fb.parse(fb.write_board(os.path.join(tmp, 'edge.kicad_pcb')))
    je = pe.footprints['J1']
    check(len({round(p.global_y, 3) for p in je.pads}) == 2 and em.row_part(je),
          '7: the fingers a row part, the short one\'s centre off the others\'')
    ge = em.grid_of(je, pe)
    check(ge.row and ge.out == 'up', f'7: one row, out away from the board\'s bottom edge: {ge.out}')
    check(em.grid_of(je).out is None, '7: no way out told without the board')
    ms7 = em.enumerate_moves(je.pads[0], ge, ['F.Cu', 'B.Cu'], lambda p, q, L: True)
    check(len(ms7) == 1 and ms7[0].direction == 'up' and ms7[0].exit_pt[1] < fb.INNER,
          f'7: a finger\'s one escape, out of its inner end: {ms7}')
    short = em.enumerate_moves(je.pads[3], ge, ['F.Cu', 'B.Cu'], lambda p, q, L: True)
    check(len(short) == 1 and short[0].direction == 'up', f'7: the short finger\'s the same way: {short}')
    pi = fb.parse(fb.write_board(os.path.join(tmp, 'inside.kicad_pcb'), plug_at=(15.0, 8.0)))
    check(em.grid_of(pi.footprints['J1'], pi).out is None, '7: the same part inside the board: no edge, no way out told')

print('row part: ' + ('FAIL' if fails else 'PASS: a QFN found, its grid on its rows\' copper, one straight escape a pad '
                                          'checked from its outer end; a shared source\'s escapes picked out; a '
                                          'socket\'s one row read past its body\'s pads, with vias behind and in its '
                                          'pins, a group of one turned from its source; a '
                                          'card\'s edge fingers out away from the board\'s edge'))
sys.exit(1 if fails else 0)
