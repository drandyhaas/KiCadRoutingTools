#!/usr/bin/env python3
"""One-shot patch: N/S flank entries in topo_emit (--moves NET=N|S)."""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
p = os.path.join(HERE, 'topo_emit.py')
t = open(p).read()

t = t.replace("""    ap.add_argument('--no-smooth', action='store_true',
                    help='skip the repo #536 octolinear smoothing pass')""",
              """    ap.add_argument('--no-smooth', action='store_true',
                    help='skip the repo #536 octolinear smoothing pass')
    ap.add_argument('--moves', default='',
                    help='homotopy entry moves, e.g. SDQ0=S,SDQ11=N')""")

old = """    ball_order = sorted(names, key=lambda nm: (ends[nm][1][1],
                                               ends[nm][1][0]))
    for nm in ball_order:
        bx, by = ends[nm][1][0], ends[nm][1][1]
        cands = []"""
new = """    moves = {}
    for tok in a.moves.split(','):
        if '=' in tok:
            k, v = tok.split('=')
            moves[k.strip()] = v.strip().upper()
    comps_pads = [p for c in comps for p in pcb.footprints[c].pads]
    rows_all = sorted({round(p.global_y, 3) for p in comps_pads})

    ball_order = sorted(names, key=lambda nm: (ends[nm][1][1],
                                               ends[nm][1][0]))
    for nm in ball_order:
        if nm in moves:
            continue
        bx, by = ends[nm][1][0], ends[nm][1][1]
        cands = []"""
assert old in t
t = t.replace(old, new)

old = """    for nm in ball_order:
        if nm in entry:
            continue
        bx, by = ends[nm][1][0], ends[nm][1][1]
        sx = bx - half"""
new = """    # flank entries (#622 take-3 task 2): run along the field's N/S edge,
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
            placed_f.append(((sx, by), (bx, by)))

    for nm in ball_order:
        if nm in entry or nm in moves:
            continue
        bx, by = ends[nm][1][0], ends[nm][1][1]
        sx = bx - half"""
assert old in t
t = t.replace(old, new)

old = """    f_sorted = sorted((e[1], nm) for nm, e in entry.items() if e[0] == 'F')"""
new = """    for nm, e in entry.items():
        if e[0] in ('N', 'S'):
            lane_y_flank = e[1][1]
    f_sorted = sorted([(e[1], nm) for nm, e in entry.items()
                       if e[0] == 'F']
                      + [(e[1][1], nm) for nm, e in entry.items()
                         if e[0] in ('N', 'S')])"""
assert old in t
t = t.replace(old, new)

old = """        bx, by = ends[nm][1][0], ends[nm][1][1]
        if mode == 'F':
            sy = v
            path = trunk + [(x1, sy), (bx, sy), (bx, by)]
        else:
            sx, sy = v
            path = trunk + [(x1, lane_y[nm]), (sx, sy)]"""
new = """        bx, by = ends[nm][1][0], ends[nm][1][1]
        if mode == 'F':
            sy = v
            path = trunk + [(x1, sy), (bx, sy), (bx, by)]
        elif mode in ('N', 'S'):
            sx, ry = v
            path = trunk + [(x1, ry), (sx, ry), (sx, by), (bx, by)]
        else:
            sx, sy = v
            path = trunk + [(x1, lane_y[nm]), (sx, sy)]"""
assert old in t
t = t.replace(old, new)

open(p, 'w').write(t)
print('moves patched')
