#!/usr/bin/env python3
"""Patch 2: flank entrants ride a lane just outside the W block through
the trunk (clears the corridor cap band, e.g. C5), then drop/rise
vertically at the splice column to the outer N/S run."""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
p = os.path.join(HERE, 'topo_emit.py')
t = open(p).read()

old = """    # flank entries (#622 take-3 task 2): run along the field's N/S edge,
    # then the vertical inter-column street west of the ball, half-pitch
    # jog. Run ys stack outward, nearest turn innermost (non-crossing).
    for flank in ('N', 'S'):
        movers = sorted((nm for nm, f in moves.items() if f == flank),
                        key=lambda nm: ends[nm][1][0])
        for i, nm in enumerate(movers):
            bx, by = ends[nm][1][0], ends[nm][1][1]
            sx = bx - half
            if flank == 'N':
                run_y = rows_all[0] - 0.4 - i * 0.3
            else:
                run_y = rows_all[-1] + 0.4 + i * 0.3
            entry[nm] = (flank, (sx, run_y))
            placed_f.append(((x1, run_y), (sx, run_y)))
            placed_f.append(((sx, run_y), (sx, by)))
            placed_f.append(((sx, by), (bx, by)))"""
new = """    # flank entries (#622 take-3 task 2): trunk lane just outside the W
    # block (clears the corridor cap band), vertical drop at the splice
    # column to the outer N/S run along the field edge, then the
    # vertical inter-column street west of the ball, half-pitch jog.
    f_ys = [e[1] for e in entry.values() if e[0] == 'F']
    for flank in ('N', 'S'):
        movers = sorted((nm for nm, f in moves.items() if f == flank),
                        key=lambda nm: ends[nm][1][0])
        for i, nm in enumerate(movers):
            bx, by = ends[nm][1][0], ends[nm][1][1]
            sx = bx - half
            if flank == 'N':
                run_y = rows_all[0] - 0.4 - i * 0.3
                ly = (min(f_ys) if f_ys else rows_all[0]) - 0.35 - i * 0.3
                dx_ = x1 - 0.3 * i
            else:
                run_y = rows_all[-1] + 0.4 + i * 0.3
                ly = (max(f_ys) if f_ys else rows_all[-1]) + 0.35 + i * 0.3
                dx_ = x1 - 0.3 * i
            entry[nm] = (flank, (sx, run_y, ly, dx_))
            placed_f.append(((dx_, ly), (dx_, run_y)))
            placed_f.append(((dx_, run_y), (sx, run_y)))
            placed_f.append(((sx, run_y), (sx, by)))
            placed_f.append(((sx, by), (bx, by)))"""
assert old in t
t = t.replace(old, new)

old = """    for nm, e in entry.items():
        if e[0] in ('N', 'S'):
            lane_y_flank = e[1][1]
    f_sorted = sorted([(e[1], nm) for nm, e in entry.items()
                       if e[0] == 'F']
                      + [(e[1][1], nm) for nm, e in entry.items()
                         if e[0] in ('N', 'S')])"""
new = """    f_sorted = sorted([(e[1], nm) for nm, e in entry.items()
                       if e[0] == 'F']
                      + [(e[1][2], nm) for nm, e in entry.items()
                         if e[0] in ('N', 'S')])"""
assert old in t
t = t.replace(old, new)

old = """        elif mode in ('N', 'S'):
            sx, ry = v
            path = trunk + [(x1, ry), (sx, ry), (sx, by), (bx, by)]"""
new = """        elif mode in ('N', 'S'):
            sx, ry, ly, dx_ = v
            path = trunk + [(dx_, ly), (dx_, ry), (sx, ry),
                            (sx, by), (bx, by)]"""
assert old in t
t = t.replace(old, new)

open(p, 'w').write(t)
print('patched 2')
