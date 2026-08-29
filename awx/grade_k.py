#!/usr/bin/env python3
"""Net-scoped grade for a K-board: disconnected count AMONG THE RUN'S
NETS (grade_all.sh counts every open net on the board, including the
51-K DDR nets a smaller rung never routed), DRC violation count at the
routed floor, and the via/segment census. Usage: grade_k.py board.kicad_pcb
NET,NET,..."""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
board = sys.argv[1]
nets = sys.argv[2].split(',')
# A check whose INPUT is missing tests nothing, and check_connected /
# check_drc on a nonexistent path print no matches -- which reads as
# "0 open, 0 DRC". Refuse loudly instead (run_utils.evidence's rule).
if not os.path.isfile(board) or os.path.getsize(board) < 1000:
    print(f'GRADE {os.path.basename(board)} BROKEN: not a real board '
          f'({"missing" if not os.path.exists(board) else "empty"})')
    sys.exit(2)
r = subprocess.run([PY, os.path.join(HERE, '..', 'py_router',
                                     'check_connected.py'), board],
                   capture_output=True, text=True)
opens = []
for line in (r.stdout + r.stderr).splitlines():
    m = re.search(r'(\S+) \(net \d+\):', line)
    if m and m.group(1).split('/')[-1] in nets:
        opens.append(m.group(1).split('/')[-1])
r = subprocess.run([PY, os.path.join(HERE, '..', 'py_router',
                                     'check_drc.py'), board,
                    '--clearance', '0.1', '--clearance-margin', '0.1'],
                   capture_output=True, text=True)
m = re.search(r'FOUND (\d+) DRC VIOLATIONS', r.stdout + r.stderr)
ndrc = int(m.group(1)) if m else 0
r = subprocess.run([PY, os.path.join(HERE, 'via_census.py'), board,
                    ','.join(nets)], capture_output=True, text=True)
m = re.search(r'TOTAL vias=(\d+) segs=(\d+)', r.stdout)
if m is None:
    print(f'GRADE {os.path.basename(board)} BROKEN: via census failed:\n'
          + (r.stdout + r.stderr).strip()[-300:])
    sys.exit(2)
print(f'GRADE {os.path.basename(board)} K={len(nets)} '
      f'open={len(opens)} drc={ndrc} vias={m.group(1) if m else "?"} '
      f'segs={m.group(2) if m else "?"}'
      + (f'  open: {",".join(sorted(opens))}' if opens else ''))
