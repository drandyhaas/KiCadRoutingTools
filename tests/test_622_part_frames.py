#!/usr/bin/env python3
"""A part seen from several frames of the whole route's geometry (corridor.line_extent / part_home / side_carry /
decide_sides).

  python3 tests/test_622_part_frames.py

The whole route lays each lane in frames -- the trunk, a ring round the destination -- each a spine whose offset runs
across it. A part a lane passes is one thing on the board: its side is decided once (decide_sides) and carried into every
frame (side_carry), and each frame measures the part along each column's own offset line (line_extent), not as the box
of its four corners projected -- far larger than its copper in a slanted or bent frame. On spines of its own this pins:

1. line_extent: on a straight spine, a rectangle the column's line crosses, grown by g, exactly; one it misses, None;
2. line_extent on a slanted spine: near a square part's end along the spine its extent is far narrower than the box of
   its projected corners (the over-blocking it removes);
3. side_carry: a part's +1 side in the trunk is the ring's +1 side where both travel the same way round it, and -1 for a
   ring laid the other way;
4. part_home: the frame that sees the part whole, before a nearer one that cuts it at its end;
5. decide_sides: the home frame's split where the lane is in it; else the frame it meets the part in over most columns
   (its split, else its mean offset); carried into the home frame's terms; a lane meeting the part in one frame only,
   left to that frame;
6. Spine.lane_line: at a corner a lane's two legs meet where its own two lines cross -- at the mitre for a fixed
   offset, before it for a lane moving into the corner -- with no column past the crossing drawn; outside a corner
   nothing is left out; a fixed point (a via's column) always drawn; a straight spine draws every column.
7. corridor.round_cover: a round pad or hole covered by a cross of two rectangles, reaching at most 0.23 of its
   radius past it (its box's square, 0.41);
8. corridor.pads_across: a two-pad part's pads across the lanes of its home frame (one beside the other in offset) or
   along them -- judged in the frame that sees it whole, nearest its spine.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import corridor as cor  # noqa: E402


def close(a, b, tol=1e-9):
    return a is not None and b is not None and all(abs(x - y) <= tol for x, y in zip(a, b))


def main():
    print('=' * 60)
    print('a part seen from several frames: its extent per column, its home, its one side')
    print('=' * 60)
    fails = []

    # 1. a straight spine east along y = 0: its normal (-dy, dx) = (0, 1), the column at s = 5 is the line x = 5
    sp = cor.Spine([(0.0, 0.0), (10.0, 0.0)])
    got = cor.line_extent(sp, 5.0, [(4.0, 2.0, 6.0, 3.0)], 0.1)
    if not close(got, (1.9, 3.1)):
        fails.append(f'line_extent, straight, crossed: {got}, want (1.9, 3.1)')
    if cor.line_extent(sp, 5.0, [(8.0, 2.0, 9.0, 3.0)], 0.1) is not None:
        fails.append('line_extent, straight: a rectangle the column misses gave an extent')
    if not close(cor.line_extent(sp, 5.0, [(4.0, 2.0, 6.0, 3.0), (4.5, -3.0, 5.5, -2.0)], 0.0), (-3.0, 3.0)):
        fails.append('line_extent: two rectangles on the line are not one extent from the lowest to the highest')

    # 2. a spine at 45 degrees and a square part beside it: the box of its projected corners vs the column's extent
    sl = cor.Spine([(0.0, 0.0), (10.0, 10.0)])
    sq = (5.0, 3.0, 6.0, 4.0)
    so = [sl.project_pt(p) for p in ((sq[0], sq[1]), (sq[0], sq[3]), (sq[2], sq[1]), (sq[2], sq[3]))]
    sa, sb = min(q[0] for q in so), max(q[0] for q in so)
    oa, ob = min(q[1] for q in so), max(q[1] for q in so)
    near_end = sa + 0.15 * (sb - sa)
    ext = cor.line_extent(sl, near_end, [sq], 0.0)
    if ext is None or not (ext[1] - ext[0]) < 0.5 * (ob - oa):
        fails.append(f'line_extent, slanted: near the part\'s end {ext} is not far narrower than its box\'s {oa:.3f}..{ob:.3f}')

    # 3. the trunk east along y = 0 (normal toward +y) and a ring leaving it south-east, a part between them
    trunk = cor.Spine([(0.0, 0.0), (10.0, 0.0)])
    ring = cor.Spine([(9.0, 0.0), (14.0, 5.0)])
    back = cor.Spine([(14.0, 5.0), (9.0, 0.0)])          # the same ring laid the other way
    part = (9.6, 1.6, 10.4, 2.4)
    c = ((part[0] + part[2]) / 2, (part[1] + part[3]) / 2)
    o_hi = max(trunk.project_pt(p)[1] for p in ((part[0], part[1]), (part[0], part[3]), (part[2], part[1]), (part[2], part[3])))
    if cor.side_carry(trunk, ring, c, o_hi, 0.2) != 1:
        fails.append('side_carry: the trunk\'s +1 side is not the ring\'s +1, both travelling one way round the part')
    if cor.side_carry(trunk, back, c, o_hi, 0.2) != -1:
        fails.append('side_carry: a ring laid the other way kept the side\'s sign')

    # 4. the home: a frame whose spine stops inside the part's span (its far end) is not the home, though nearer
    far = cor.Spine([(0.0, 5.0), (20.0, 5.0)])          # sees the part whole, 3 mm off
    cut = cor.Spine([(0.0, 2.0), (10.0, 2.0)])          # nearer, but the part runs past its end
    spans = {'far': (9.6, 10.4), 'cut': (9.6, 10.0)}
    if cor.part_home({'far': far, 'cut': cut}, spans, c) != 'far':
        fails.append('part_home: chose the frame that cuts the part at its end')
    spans2 = {'far': (9.6, 10.4), 'cut': (4.0, 5.0)}
    if cor.part_home({'far': far, 'cut': cut}, spans2, c) != 'cut':
        fails.append('part_home: two frames that both see the part whole -- not the nearer')

    # 5. one side per lane and part
    home = {0: 'T'}
    carry = {(0, 'T'): 1, (0, 'S'): -1}
    mid = {('T', 0): 6.0, ('S', 0): -1.5}
    meets = {('A', 0): {'T': [5.9] * 4, 'S': [-1.3] * 13},      # in the home's split
             ('B', 0): {'T': [5.9] * 4, 'S': [-1.3] * 13},      # not in the home's split, in the ring's
             ('C', 0): {'T': [5.9] * 4, 'S': [-1.3] * 13},      # in no split: the ring's mean offset (above its middle)
             ('D', 0): {'S': [-1.3] * 13}}                      # one frame only
    split = {('T', 'A', 0): -1, ('S', 'A', 0): -1, ('S', 'B', 0): -1}   # (A's ring split, carried, would say +1)
    got = cor.decide_sides(meets, home, split, mid, carry)
    want = {('A', 0): -1, ('B', 0): 1, ('C', 0): -1}
    if got != want:
        fails.append(f'decide_sides: {got}, want {want} (home split; else the ring\'s split; else its offset; '
                     f'carried by the ring\'s -1; D left to its one frame)')

    # 6. a lane at a corner: east 10 mm, then a right turn south (toward +o, its inner side)
    L_ = cor.Spine([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)])

    def steps_back(xy):
        return [i for i in range(len(xy) - 2)
                if (xy[i + 1][0] - xy[i][0]) * (xy[i + 2][0] - xy[i + 1][0])
                + (xy[i + 1][1] - xy[i][1]) * (xy[i + 2][1] - xy[i + 1][1]) < -1e-12]

    def has(xy, p):
        return any(abs(q[0] - p[0]) < 1e-6 and abs(q[1] - p[1]) < 1e-6 for q in xy)
    ss = [round(8.0 + 0.1 * i, 6) for i in range(41)]
    # (a) 0.5 mm inside at a fixed offset: its two lines cross at the mitre (9.5, 0.5); drawn through every column
    # (lane_xy) it steps past it and back -- the control that this test sees the fold
    fixed_o = [(s_, 0.5) for s_ in ss]
    if not steps_back(L_.lane_xy(fixed_o)):
        fails.append('lane_line: drawn through every column, the lane did not step back (the test sees no fold)')
    got = L_.lane_line(fixed_o)
    if steps_back(got) or not has(got, (9.5, 0.5)):
        fails.append(f'lane_line, inside at a fixed offset: steps back at {steps_back(got)}, or misses the mitre (9.5, 0.5)')
    # (b) inside and moving further in (0.3 at s 8 to 0.9 at s 12, as K44's DQ3): its lines y = 0.3 + 0.15 (x - 8) and
    # x = 9.4 - 0.15 y cross at (9.32518, 0.49878) -- before the fixed-offset fold, so the mitre of its interpolated
    # offset lies inside its line (a dip) and a column past the crossing is a hook
    sloped = [(s_, 0.3 + 0.15 * (s_ - 8.0)) for s_ in ss]
    y_x = 0.51 / 1.0225
    X = (9.4 - 0.15 * y_x, y_x)
    got = L_.lane_line(sloped)
    if steps_back(got) or not has(got, X):
        fails.append(f'lane_line, inside and moving in: steps back at {steps_back(got)}, or misses its lines\' crossing {X}')
    # (c) outside at a fixed offset: nothing left out, the corner its mitre (10.5, -0.5)
    outer = [(s_, -0.5) for s_ in ss]
    got = L_.lane_line(outer)
    if len(got) != len(ss) + 1 or not has(got, (10.5, -0.5)) or steps_back(got):
        fails.append(f'lane_line, outside: {len(got)} points (want {len(ss) + 1}), the mitre (10.5, -0.5) in: '
                     f'{has(got, (10.5, -0.5))}')
    # (d) a FIXED point past the crossing (a via's column) is drawn all the same
    k_fix = ss.index(9.8)
    got = L_.lane_line(fixed_o, fixed={k_fix})
    if not any(abs(q[2] - 9.8) < 1e-9 for q in got):
        fails.append('lane_line: a fixed point past the crossing was left out')
    # (e) a straight spine: every column, as it is
    st = cor.Spine([(0.0, 0.0), (20.0, 0.0)])
    if [(round(q[0], 9), round(q[1], 9)) for q in st.lane_line(sloped)] != [(round(s_, 9), round(o_, 9)) for s_, o_ in sloped]:
        fails.append('lane_line: a straight spine did not draw every column where it is')

    # 7. a round pad or hole as the cross of two rectangles: every point of its circle inside one of them, and no
    # corner of them farther out than 0.23 of the radius (its box's square reached 0.41)
    import math as _m
    rc = cor.round_cover(3.0, -2.0, 1.1, 1.1)
    outside = [a for a in range(720) if not any(
        r_[0] - 1e-9 <= 3.0 + 1.1 * _m.cos(_m.radians(a / 2)) <= r_[2] + 1e-9
        and r_[1] - 1e-9 <= -2.0 + 1.1 * _m.sin(_m.radians(a / 2)) <= r_[3] + 1e-9 for r_ in rc)]
    reach = max(_m.hypot(cx - 3.0, cy + 2.0) for r_ in rc for cx in (r_[0], r_[2]) for cy in (r_[1], r_[3])) / 1.1 - 1.0
    if outside or reach > 0.23:
        fails.append(f'round_cover: {len(outside)} points of the circle uncovered, its corners {reach:.3f} r out (want '
                     f'0 and at most 0.23)')

    # 8. a trunk east along y = 0 and a ring north along x = 20: a part beside the trunk, its pads stacked in y, is across
    # (the trunk its home); turned, along. Beside the ring (its home, nearer), pads stacked in y are ALONG the ring
    tr8 = cor.Spine([(0.0, 0.0), (20.0, 0.0)])
    rg8 = cor.Spine([(20.0, 0.0), (20.0, -20.0)])
    sp8 = {'T': tr8, 'N': rg8}
    got8 = (cor.pads_across(sp8, [(5.0, 1.0), (5.0, 2.0)]), cor.pads_across(sp8, [(5.0, 1.5), (6.0, 1.5)]),
            cor.pads_across(sp8, [(21.0, -10.0), (21.0, -11.0)]), cor.pads_across(sp8, [(21.0, -10.0), (22.0, -10.0)]))
    if got8 != (True, False, False, True):
        fails.append(f'pads_across: {got8}, want (True, False, False, True) (across / along the trunk; along / across '
                     f'the ring that is the part\'s home)')

    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: a column\'s extent exact and narrower than the projected box where the frame slants; the side carried '
          'by the travel; the home whole before near; one side per lane and part, from the home split first; a lane '
          'at a corner drawn where its two lines cross, never past it; a round pad covered by its cross; two pads across or along their home frame')
    return 0


if __name__ == '__main__':
    sys.exit(main())
