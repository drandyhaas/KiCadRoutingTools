#!/usr/bin/env python3
"""One-shot patch: emitter-matching W capacity + in-field crossing
pricing (B-descent) in plan_flanks.py."""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
p = os.path.join(HERE, 'plan_flanks.py')
t = open(p).read()

t = t.replace('''    def w_capacity(w_nets):
        """Greedy W street chain (emitter's rule, geometry-only
        approximation): direct for column-A balls, else north/south
        street, strictly >prev+0.45; leftovers are dogbones."""
        prev = -1e9
        dogbone = set()
        for nm in sorted(w_nets, key=lambda n: (ball[n][1], ball[n][0])):
            bx, by = ball[nm]
            got = False
            opts = ([by] if bx <= fx0 + 0.01 else []) + \\
                [by - half, by + half]
            for ey in opts:
                if ey > prev + 0.45:
                    prev = ey
                    got = True
                    break
            if not got:
                dogbone.add(nm)
        return dogbone''',
'''    def w_capacity(w_nets):
        """Greedy W entry assignment (matches the emitter's 0.3 pairwise
        rule): direct for column-A balls, else north/south street; a
        candidate needs >=0.3 from every taken entry y. Returns
        (dogbones, {nm: entry_y})."""
        taken = []
        entry_y = {}
        dogbone = set()
        for nm in sorted(w_nets, key=lambda n: (ball[n][1], ball[n][0])):
            bx, by = ball[nm]
            opts = ([by] if bx <= fx0 + 0.01 else []) + \\
                [by - half, by + half]
            for ey in opts:
                if all(abs(ey - v) >= 0.3 for v in taken):
                    taken.append(ey)
                    entry_y[nm] = ey
                    break
            else:
                dogbone.add(nm)
        return dogbone, entry_y''')
assert 'entry_y = {}' in t

t = t.replace('''        order = (sorted(nblock, key=lambda n: -assign[n][1])
                 + sorted(wblock, key=lambda n: (ball[n][1], ball[n][0]))
                 + sorted(sblock, key=lambda n: assign[n][1]))
        rank = {nm: i for i, nm in enumerate(order)}
        seq = [rank[nm] for nm in inside]
        keep = te.lis_keep(seq) if seq else set()
        divers = {inside[i] for i in range(len(inside)) if i not in keep}
        dogbone = w_capacity(wblock)
        promoted = dogbone - divers
        vias = 2 * len(divers) + 2 * len(promoted)
        extra = sum(assign[nm][2] for nm in names)
        return vias, extra, divers, promoted''',
'''        order = (sorted(nblock, key=lambda n: -assign[n][1])
                 + sorted(wblock, key=lambda n: (ball[n][1], ball[n][0]))
                 + sorted(sblock, key=lambda n: assign[n][1]))
        rank = {nm: i for i, nm in enumerate(order)}
        seq = [rank[nm] for nm in inside]
        keep = te.lis_keep(seq) if seq else set()
        divers = {inside[i] for i in range(len(inside)) if i not in keep}
        dogbone, entry_y = w_capacity(wblock)
        promoted = dogbone - divers
        # in-field crossings: a flank descent at sx spanning a y-range
        # crosses a W street run (entry_y, x1..bx) F-on-F -> the descent
        # must go on B.Cu (empty under the field), dogbone surface: +2
        b_descent = set()
        for nm in names:
            fl, sx, _e = assign[nm]
            if fl == 'W':
                continue
            by = ball[nm][1]
            if fl in ('N', 'AS'):
                ylo, yhi = north_y, by
            else:
                ylo, yhi = by, south_y
            for wm in wblock:
                if wm in dogbone:
                    continue
                ey = entry_y[wm]
                if ylo < ey < yhi and x1 < sx < ball[wm][0]:
                    b_descent.add(nm)
                    break
        vias = 0
        for nm in names:
            if nm in divers or nm in b_descent or nm in promoted:
                vias += 2
        extra = sum(assign[nm][2] for nm in names)
        return vias, extra, divers, promoted, b_descent''')
assert 'b_descent' in t

t = t.replace('''                r = evaluate(assign)
                if r is None:
                    continue
                vias, extra, divers, promoted = r
                results.append((vias, extra, assign, divers, promoted))''',
'''                r = evaluate(assign)
                if r is None:
                    continue
                vias, extra, divers, promoted, b_desc = r
                results.append((vias, extra, assign, divers, promoted,
                                b_desc))''')

t = t.replace('''    for vias, extra, assign, divers, promoted in frontier:
        moves = {nm: assign[nm][0] for nm in names
                 if assign[nm][0] != 'W'}
        print(f'  {vias:2d} vias  +{extra:6.2f} mm   moves={moves or "none"}'
              f'  divers={sorted(divers)}'
              + (f' promoted={sorted(promoted)}' if promoted else ''))''',
'''    for vias, extra, assign, divers, promoted, b_desc in frontier:
        moves = {nm: assign[nm][0] for nm in names
                 if assign[nm][0] != 'W'}
        print(f'  {vias:2d} vias  +{extra:6.2f} mm   moves={moves or "none"}'
              f'  divers={sorted(divers)}'
              + (f' promoted={sorted(promoted)}' if promoted else '')
              + (f' B-descent={sorted(b_desc)}' if b_desc else ''))''')

open(p, 'w').write(t)
print('patched')
