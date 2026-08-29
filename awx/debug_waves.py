#!/usr/bin/env python3
"""Standalone reproduction of the K11 wave-schedule stall."""
launch = ['SDQM1', 'SDQ9', 'SDQ10', 'SDQ11', 'SDQ8', 'SDQ12', 'SDQ13',
          'SDQM0', 'SDQ14', 'SDQ15', 'SDQ0']
target = ['SDQ15', 'SDQ0', 'SDQ10', 'SDQ8', 'SDQ13', 'SDQM0', 'SDQ14',
          'SDQ9', 'SDQM1', 'SDQ12', 'SDQ11']
trank = {nm: i for i, nm in enumerate(target)}
lidx = {nm: i for i, nm in enumerate(launch)}
keepset = None


def inverted(a, b):
    return (lidx[a] < lidx[b]) != (trank[a] < trank[b])


# divers = complement of LIS on ranks in launch order
ranks = [trank[nm] for nm in launch]
n = len(ranks)
best = [1] * n
prev = [-1] * n
for i in range(n):
    for j in range(i):
        if ranks[j] < ranks[i] and best[j] + 1 > best[i]:
            best[i] = best[j] + 1
            prev[i] = j
i = max(range(n), key=lambda k: best[k])
keep = set()
while i >= 0:
    keep.add(i)
    i = prev[i]
divers = {launch[i] for i in range(n) if i not in keep}
print('divers:', sorted(divers, key=lambda d: trank[d]))

ups = sorted((d for d in divers if trank[d] < lidx[d]),
             key=lambda nm: trank[nm])
downs = sorted((d for d in divers if trank[d] >= lidx[d]),
               key=lambda nm: -trank[nm])
dirs = {d: (-1 if trank[d] < lidx[d] else 1) for d in divers}
waves = []
for group in (ups, downs):
    gw = []
    for d in group:
        for w in gw:
            if all(not inverted(d, e) for e in w):
                w.append(d)
                break
        else:
            gw.append([d])
    waves.extend(gw)
print('waves:', waves, 'dirs:', dirs)

seq = list(launch)
for wi, wave in enumerate(waves):
    print(f'wave {wi} {wave}: start seq={seq}')
    while True:
        used = set()
        col = []
        for d in wave:
            if d in used:
                continue
            i = seq.index(d)
            j = i + dirs[d]
            if 0 <= j < len(seq):
                p = seq[j]
                if p not in used and inverted(d, p):
                    seq[i], seq[j] = seq[j], seq[i]
                    col.append((d, p))
                    used.add(d)
                    used.add(p)
        if not col:
            break
        print('   col:', col)
print('final:', seq)
print('target:', target)
print('MATCH' if seq == target else 'STALL')
for a in launch:
    for b in launch:
        if lidx[a] < lidx[b] and inverted(a, b) and \
                seq.index(a) < seq.index(b):
            print(f'  unresolved inversion: {a} x {b} '
                  f'(divers: {a in divers}/{b in divers})')
