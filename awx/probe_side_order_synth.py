#!/usr/bin/env python3
"""Which exit order means "not crossing", per side -- decided on a
synthetic board where the answer is not confounded.

The real bench mixes two things: which way a leg wraps the array (the
shortest way round is not the same way for every net) and which exit
order avoids a crossing. This isolates the second. One box, launches on
a line due west of it, exits spread along one side, and for every
assignment of launches to exits the question asked is the only one that
matters: can the two legs be stacked so they do not cross?

Prints, per side, the sign that predicts the geometry with no
disagreements. That sign is what SIDE_ORDER must hold.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import select_moves as sm  # noqa: E402

BOX = (10.0, 0.0, 30.0, 20.0)          # x0, y0, x1, y1  (y grows DOWN)
LAUNCH_X = -5.0
N = 5
# escape_moves.edge() puts the exit HALF A PITCH outside the bbox (the
# bbox is pad CENTRES), so an exit sits just outside the padded box the
# corridor routes around. Model that: an exit exactly ON the box is
# inside every padding, and around_box_path then finds no legal corner
# pair and falls back to the straight line STRAIGHT THROUGH the array --
# which silently turns this whole experiment into a straight-line one.
OUT = 1.0                              # ~ half pitch
PADS = (0.30, 0.55)                    # both < OUT, or same degeneracy


def cross(p1, p2, p3, p4):
    def d(a, b, c):
        return ((b[0] - a[0]) * (c[1] - a[1])
                - (b[1] - a[1]) * (c[0] - a[0]))
    d1, d2 = d(p3, p4, p1), d(p3, p4, p2)
    d3, d4 = d(p1, p2, p3), d(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def must_cross(la, ea, lb, eb):
    """Do these two legs cross under EVERY stacking?"""
    for pa, pb in (PADS, PADS[::-1]):
        A = sm.around_box_path(la, ea, BOX, pad=pa)
        B = sm.around_box_path(lb, eb, BOX, pad=pb)
        if not any(cross(p, q, r, s)
                   for p, q in zip(A, A[1:])
                   for r, s in zip(B, B[1:])):
            return False
    return True


x0, y0, x1, y1 = BOX
# launches: due west, spread over the box's own y range (the flow frame
# puts the source west of the destination, so this is the real shape)
launches = [(LAUNCH_X, y0 + (y1 - y0) * (i + 0.5) / N) for i in range(N)]
EXITS = {
    'left':  [(x0 - OUT, y0 + (y1 - y0) * (i + 0.5) / N) for i in range(N)],
    'right': [(x1 + OUT, y0 + (y1 - y0) * (i + 0.5) / N) for i in range(N)],
    'up':    [(x0 + (x1 - x0) * (i + 0.5) / N, y0 - OUT) for i in range(N)],
    'down':  [(x0 + (x1 - x0) * (i + 0.5) / N, y1 + OUT) for i in range(N)],
}

print(f'box {BOX}, {N} launches at x={LAUNCH_X} (y grows DOWN: '
      f'"up" is the y0 edge)\n')
for side, exits in EXITS.items():
    axis = 0 if side in ('up', 'down') else 1
    # every launch paired with every exit; ask the geometry directly
    real = {}
    for i, la in enumerate(launches):
        for j, lb in enumerate(launches):
            if j <= i:
                continue
            for ea in exits:
                for eb in exits:
                    if ea == eb:
                        continue
                    real[(i, j, ea, eb)] = must_cross(la, ea, lb, eb)
    for sign, tag in ((1.0, '+'), (-1.0, '-')):
        wrong = sum(1 for (i, j, ea, eb), r in real.items()
                    if ((sign * ea[axis] > sign * eb[axis]) != r))
        held = sm.SIDE_ORDER[side][1] == sign
        print(f'  {side:5s} sign {tag}: {wrong:4d} of {len(real)} '
              f'assignments disagree with the geometry'
              + ('   <- SIDE_ORDER holds this' if held else ''))
    print()
