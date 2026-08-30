#!/usr/bin/env python3
"""Per net: the braid's two free ends (tooth, stub) and their layers, and
the corridor grouping -- to diff two fanout boards of the same K.
usage: ends_of.py FANOUT_BOARD K [--dest REF]"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import braid as te  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('board')
ap.add_argument('k', type=int)
ap.add_argument('--dest', default='DU1')
a = ap.parse_args()
nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                       str(a.k)], capture_output=True, text=True).stdout.strip()
names = [n for n in nets.split(',') if n]
ctx, groups = te.setup(a.board, names, a.dest, lambda *x: None)
print('corridors: ' + '  '.join(f'[{len(g)}] {",".join(g)}' for g in groups))
for nm in names:
    (tx, ty), (sx, sy), _ = ctx.ends[nm]
    print(f'{nm:6} tooth ({tx:.3f},{ty:.3f}){ctx.tooth_layer[nm][0]} '
          f'stub ({sx:.3f},{sy:.3f}){ctx.dest_layer[nm][0]}')
