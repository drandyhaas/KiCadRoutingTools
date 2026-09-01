#!/usr/bin/env python3
"""#622 the composed driver: COMPLETE first, then CHEAPEN.

retry_chain closes opens (directed iteration on refusal: the braid's
own refused nets forced through the source-B / dest-B split arms,
best board kept). improve_k cuts vias (ledger diagnosis -> serial
pages flips and relay trials, strictly-better only). This driver runs
them in that order on one tag, so a single command turns a K into the
best board the current machinery can produce:

  1. retry_chain TAG K          -> tmp/TAGr*_..., winner recorded
  2. improve_k on the WINNING ARM's fanout board -> evolved best

The composition is monotone: improve_k's free-braid baseline of the
winner's fanout board reproduces the winner exactly (the chain is
bit-deterministic), and every later accept is lexicographically
strictly better on (open, drc, vias).

usage: drive_k.py TAG K [--rounds N] [--num-improve N] [-- fanout opts]
env: BASE, DEST, DIRS, PLAN_OPTS, TWO_PAGE, TP_SCOPE as chain_k.sh.
The recorded arm wants `-- --no-plane-drop` exactly like chain_k.sh:
dropping it silently grades a different (plane-drop) world -- measured
69v/3drc where the chain draws 58v/0/0 at K28.
"""
import argparse
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)

argv = sys.argv[1:]
extra = []
if '--' in argv:
    i = argv.index('--')
    extra = argv[i + 1:]
    argv = argv[:i]
ap = argparse.ArgumentParser()
ap.add_argument('tag')
ap.add_argument('k')
ap.add_argument('--rounds', type=int, default=2)
ap.add_argument('--num-improve', type=int, default=3)
a = ap.parse_args(argv)
tag = a.tag if '/' in a.tag else os.path.join('tmp', a.tag)

# the recorded arm, unless the caller overrides: retry_chain runs its
# braids under THIS process's env, and a driver that forgets TWO_PAGE
# grades a different machine (measured: same fanout board, 31v/1 open
# without vs 25v/0 with)
os.environ.setdefault('TWO_PAGE', '1')
os.environ.setdefault('TP_SCOPE', 'split')
os.environ.setdefault('PLAN_OPTS', '--two-page')

r = subprocess.run(
    [sys.executable, 'retry_chain.py', tag, a.k,
     '--rounds', str(a.rounds)]
    + (['--'] + extra if extra else []),
    capture_output=True, text=True)
sys.stdout.write(r.stdout)
m = re.search(r'^KEPT (\S+) -> (\S+): open=(\d+) drc=(\d+) '
              r'vias=(\d+)', r.stdout, re.M)
if not m:
    print('retry_chain kept nothing; stopping')
    sys.exit(2)
wtag, board = m.group(1), m.group(2)
fo = f'{wtag}_fo_k{a.k}.kicad_pcb'
if not os.path.exists(fo):
    print(f'winning arm fanout missing: {fo}; stopping at retry best')
    sys.exit(0)
print(f'\n== improve_k on the winner\'s fanout ({fo})')
r2 = subprocess.run(
    [sys.executable, 'improve_k.py', fo, a.k,
     os.path.basename(tag), '--num-improve', str(a.num_improve)],
    capture_output=True, text=True)
sys.stdout.write(r2.stdout)
