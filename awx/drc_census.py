#!/usr/bin/env python3
"""Run check_drc on a board and print a per-pair violation census plus
the first few violation blocks (the braid campaign's grading lens)."""
import os
import re
import subprocess
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
board = sys.argv[1]
r = subprocess.run(
    [sys.executable, os.path.join(HERE, '..', 'py_router', 'check_drc.py'),
     board, '--clearance', '0.1', '--clearance-margin', '0.1',
     '--max-print', '0'],
    capture_output=True, text=True)
out = r.stdout + r.stderr
pairs = Counter()
for m in re.finditer(r'/([A-Za-z0-9_ ]+)/(\S+) <-> /([A-Za-z0-9_ ]+)/(\S+)',
                     out):
    pairs[tuple(sorted((m.group(2), m.group(4))))] += 1
for line in out.splitlines():
    if 'FOUND' in line or 'violations (' in line or 'PASS' in line \
            or 'no violations' in line.lower():
        print(line)
print('pair census:')
for (a, b), n in pairs.most_common(15):
    print(f'  {n:4d}  {a} <-> {b}')
blocks = out.split('----------------------------------------')
for b in blocks[1:3]:
    print(b.rstrip()[:400])
