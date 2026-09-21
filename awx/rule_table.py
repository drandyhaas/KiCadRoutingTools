#!/usr/bin/env python3
"""rule_table.py [DIR ...] [--ks 28,35,41,51] [--tags a,b,c] -- every arm's
routed boards graded under THE RULE (Andy, 2026-09-15: length at 7.5 mm per
via equivalent everywhere): vias on the run's nets, copper mm, the score
vias + mm / VIA_MM, and the open nets (grade_k's two shapes). One row per
arm and K, arms as columns of the score table at the end. Default DIR:
tmp/s10 (+ the s9 control d40)."""
import glob
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

VIA_MM = 7.5


def arg(name, default):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


def coherent(K):
    r = subprocess.run([sys.executable, 'coherent_nets.py', str(K), '--board=fb_t2q_fresh.kicad_pcb'],
                       capture_output=True, text=True)
    return [n for n in r.stdout.strip().split(',') if n]


def opens_of(board, names):
    r = subprocess.run([sys.executable, '../py_router/check_connected.py', board],
                       capture_output=True, text=True)
    out = []
    for line in (r.stdout + r.stderr).splitlines():
        m = re.search(r'(\S+) \(net \d+\):', line) or re.match(r'\s+(\S+) \(\d+ pads?\)\s*$', line)
        if m and m.group(1).split('/')[-1] in names:
            out.append(m.group(1).split('/')[-1])
    return sorted(set(out))


def main():
    dirs = [a for a in sys.argv[1:] if not a.startswith('--') and os.path.isdir(a)] or ['tmp/s10', 'tmp/s9']
    ks = [int(k) for k in arg('--ks', '28,35,41,51').split(',')]
    only = {t for t in arg('--tags', '').split(',') if t}
    rows = {}
    for K in ks:
        names = coherent(K)
        for d in dirs:
            for p in sorted(glob.glob(os.path.join(d, f'*_k{K}.kicad_pcb'))):
                tag = os.path.basename(p)[:-len(f'_k{K}.kicad_pcb')]
                if '_fo' in tag or '_frame' in tag or 'srcres' in tag or (only and tag not in only):
                    continue
                if d == 'tmp/s9' and tag != 'd40':
                    continue
                pcb = parse_kicad_pcb(p)
                ids = {i for i, n in pcb.nets.items() if n.name.split('/')[-1] in names}
                mm = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in pcb.segments if s.net_id in ids)
                v = sum(1 for x in pcb.vias if x.net_id in ids)
                op = opens_of(p, names)
                rows[(tag, K)] = (v, mm, v + mm / VIA_MM, op)
    tags = sorted({t for t, _ in rows}, key=lambda t: (t != 'd40', t))
    print(f'{"arm":8s} ' + ' '.join(f'{"K" + str(K) + " v/mm/rule/open":>28s}' for K in ks))
    for t in tags:
        cells = []
        for K in ks:
            r = rows.get((t, K))
            cells.append(f'{r[0]:3d} {r[1]:6.0f} {r[2]:6.1f} {len(r[3]):2d}' if r else f'{"--":>20s}')
        print(f'{t:8s} ' + ' '.join(f'{c:>28s}' for c in cells))
    print('\nrule = vias + mm / 7.5; open = open nets among the run\'s (an open net is a failure, not a score)')


if __name__ == '__main__':
    main()
