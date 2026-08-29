#!/usr/bin/env python3
"""Standalone repro of the gated round scheduler deadlock."""
launch = ['SDQM1', 'SDQ9', 'SDQ10', 'SDQ11', 'SDQ8', 'SDQ12', 'SDQ13',
          'SDQM0', 'SDQ14', 'SDQ15', 'SDQ0']
target = ['SDQ15', 'SDQ0', 'SDQ10', 'SDQ8', 'SDQ13', 'SDQM0', 'SDQ14',
          'SDQ9', 'SDQM1', 'SDQ12', 'SDQ11']
trank = {nm: i for i, nm in enumerate(target)}
lidx = {nm: i for i, nm in enumerate(launch)}


def inverted(a, b):
    return (lidx[a] < lidx[b]) != (trank[a] < trank[b])


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

ups = sorted((d for d in divers if trank[d] < lidx[d]),
             key=lambda nm: trank[nm])
downs = sorted((d for d in divers if trank[d] >= lidx[d]),
               key=lambda nm: -trank[nm])
dirs = {d: (-1 if trank[d] < lidx[d] else 1) for d in divers}
priority = ups + downs
need_pass = {d: {p for p in priority[:priority.index(d)]
                 if inverted(p, d)} for d in priority}
print('priority:', priority)
print('need_pass:', {k: sorted(v) for k, v in need_pass.items()})
passed = set()
seq = list(launch)
for rnd in range(50):
    used = set()
    col = []
    stall_info = []
    for d in priority:
        if d in used:
            continue
        waiting = [p for p in need_pass[d] if (p, d) not in passed]
        if waiting:
            stall_info.append(f'{d} waits {waiting}')
            continue
        i = seq.index(d)
        j = i + dirs[d]
        if not (0 <= j < len(seq)):
            continue
        p = seq[j]
        correcting = (trank[d] > trank[p]) if dirs[d] > 0 \
            else (trank[d] < trank[p])
        if p in used or not correcting:
            stall_info.append(f'{d} blocked at {p} (used={p in used}, '
                              f'correcting={correcting})')
            continue
        seq[i], seq[j] = seq[j], seq[i]
        col.append((d, p))
        used.add(d)
        used.add(p)
        if p in dirs:
            passed.add((d, p))
    print(f'round {rnd}: {col}')
    if not col:
        print('DEADLOCK; stalls:', stall_info)
        break
    if seq == target:
        print('DONE in', rnd + 1, 'columns')
        break
print('final:', seq)
