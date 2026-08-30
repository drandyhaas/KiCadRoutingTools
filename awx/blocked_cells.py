#!/usr/bin/env python3
"""Summarise the blocked-cell frontier a CONNECT_DEBUG failure printed:
per layer, the cells in mm (0.025 grid) as a compact list of x-columns
with their y-ranges. usage: blocked_cells.py PROBE_FILE [which]"""
import ast
import re
import sys

txt = open(sys.argv[1]).read()
which = int(sys.argv[2]) if len(sys.argv) > 2 else 0
xlo = float(sys.argv[3]) if len(sys.argv) > 3 else -1e9
xhi = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9
hits = list(re.finditer(r"router failed: (\{.*)$", txt, re.M))
if not hits:
    sys.exit('no failure record')
info = ast.literal_eval(hits[which].group(1))
G = 0.025
for key in ('blocked_cells_forward', 'blocked_cells_backward'):
    cells = info.get(key) or []
    if not cells:
        continue
    print(f'{key}: {len(cells)} cells')
    for L in (0, 1):
        cols = {}
        for (x, y, l) in cells:
            if l == L and xlo <= x * G <= xhi:
                cols.setdefault(x, []).append(y)
        if not cols:
            continue
        print(f'  layer {"F" if L == 0 else "B"}:')
        for x in sorted(cols):
            ys = sorted(cols[x])
            runs, a, b = [], ys[0], ys[0]
            for y in ys[1:]:
                if y == b + 1:
                    b = y
                else:
                    runs.append((a, b))
                    a = b = y
            runs.append((a, b))
            print(f'    x={x * G:.3f}: ' + ' '.join(
                f'y[{a * G:.3f},{b * G:.3f}]' for a, b in runs))
others = {k: v for k, v in info.items() if not k.startswith('blocked_cells')}
print(others)
