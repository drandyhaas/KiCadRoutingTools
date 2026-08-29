#!/usr/bin/env python3
"""Patch 3: evaluator ranks W nets by ASSIGNED entry y (launch-aware
street assignment within a row), dogbones slotted at their real lane."""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
p = os.path.join(HERE, 'plan_flanks.py')
t = open(p).read()

old = '''    def w_capacity(w_nets):
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
        return dogbone, entry_y'''
new = '''    launch_idx = {nm: i for i, nm in enumerate(launch)}

    def w_capacity(w_nets):
        """Greedy W entry assignment: iterate by (ball row, LAUNCH
        position) and try north street / direct (column A) / south
        street -- launch-aware so same-row groups take streets in
        launch order (fewer manufactured inversions). 0.3 pairwise
        min like the emitter. Returns (dogbones, {nm: entry_y}) with
        dogbones slotted at their real between-lanes y."""
        taken = []
        entry_y = {}
        dogbone = set()
        for nm in sorted(w_nets, key=lambda n: (round(ball[n][1], 2),
                                                launch_idx[n])):
            bx, by = ball[nm]
            opts = [by - half] + ([by] if bx <= fx0 + 0.01 else []) + \\
                [by + half]
            for ey in opts:
                if all(abs(ey - v) >= 0.3 for v in taken):
                    taken.append(ey)
                    entry_y[nm] = ey
                    break
            else:
                dogbone.add(nm)
        for nm in dogbone:      # real lane: midway between flanking Fs
            sy = ball[nm][1] + half
            lo = max([v for v in taken if v <= sy], default=sy - 0.4)
            hi = min([v for v in taken if v > sy], default=sy + 0.4)
            entry_y[nm] = (lo + hi) / 2
        return dogbone, entry_y'''
assert old in t
t = t.replace(old, new)

old = '''        order = (sorted(nblock, key=lambda n: -assign[n][1])
                 + sorted(wblock, key=lambda n: (ball[n][1], ball[n][0]))
                 + sorted(sblock, key=lambda n: assign[n][1]))
        rank = {nm: i for i, nm in enumerate(order)}
        seq = [rank[nm] for nm in inside]
        keep = te.lis_keep(seq) if seq else set()
        divers = {inside[i] for i in range(len(inside)) if i not in keep}
        dogbone, entry_y = w_capacity(wblock)
        promoted = dogbone - divers'''
new = '''        dogbone, entry_y = w_capacity(wblock)
        order = (sorted(nblock, key=lambda n: -assign[n][1])
                 + sorted(wblock, key=lambda n: entry_y[n])
                 + sorted(sblock, key=lambda n: assign[n][1]))
        rank = {nm: i for i, nm in enumerate(order)}
        seq = [rank[nm] for nm in inside]
        keep = te.lis_keep(seq) if seq else set()
        divers = {inside[i] for i in range(len(inside)) if i not in keep}
        promoted = dogbone - divers'''
assert old in t
t = t.replace(old, new)

open(p, 'w').write(t)
print('patched 3')
