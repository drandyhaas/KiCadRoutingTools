#!/usr/bin/env python3
"""The bus step imports the chain's modules under the chain's own policy (route_bus.step_policy).

  python3 tests/test_622_route_bus_policy.py

An awx module reads its policy when it is first imported in a process (awx_settings), and the chain's in-process
stages run on the modules the step imported before them. Imported under the shell's environment, braid read
BRAID_PAIRS off and every stage planned and laid a pair as two single lanes. In a fresh process whose environment says
nothing of the policy, this pins:

1. importing route_bus imports no chain module (braid, make_bench, fanout_from_plan);
2. the buses found (find_buses) import the chain's modules with a pair one lane (braid.PAIRS 1) and the caches off;
3. the step (route_bus, here refused at once for a part the board does not have) likewise.
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'qfn_interior_pads.kicad_pcb')

PROBE = r'''
import contextlib, os, sys, tempfile
sys.path.insert(0, os.path.join(sys.argv[1], 'awx'))
sys.path.insert(0, os.path.join(sys.argv[1], 'py_router'))
import route_bus as rb
chain = sorted(m for m in ('braid', 'make_bench', 'fanout_from_plan', 'pairs') if m in sys.modules)
print('LOADED', ','.join(chain) or '-')
if sys.argv[3] == 'find':
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(sys.stderr):
        rb.find_buses(parse_kicad_pcb(sys.argv[2]))
else:
    with tempfile.TemporaryDirectory() as td:
        print('EXIT', rb.route_bus(sys.argv[2], os.path.join(td, 'o.kicad_pcb'), 'NOPE1', 'NOPE2',
                                   log=lambda *a: None)[0])
import braid, pairs
print('PAIRS', braid.PAIRS)
'''


def probe(mode):
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(('BRAID_', 'PLAN_', 'TAUT_', 'PROBE_', 'STAGE_'))}
    r = subprocess.run([sys.executable, '-c', PROBE, ROOT, BOARD, mode], capture_output=True, text=True, env=env)
    got = dict(ln.split(' ', 1) for ln in r.stdout.splitlines() if ln.split(' ', 1)[0] in ('LOADED', 'EXIT', 'PAIRS'))
    if 'PAIRS' not in got:
        raise SystemExit(f'BROKEN TEST: the probe ({mode}) did not finish: {(r.stderr.strip().splitlines() or [""])[-1]}')
    return got


def main():
    print('=' * 60)
    print('the bus step imports the chain under its own policy')
    if not os.path.isfile(BOARD):
        raise SystemExit(f'BROKEN TEST: no board {BOARD}')
    fails = []
    f = probe('find')
    if f['LOADED'] != '-':
        fails.append(f'importing route_bus loaded chain modules: {f["LOADED"]}')
    if f['PAIRS'] != '1':
        fails.append(f'after find_buses braid.PAIRS is {f["PAIRS"]}: the chain imported under the shell\'s environment')
    s = probe('step')
    if s.get('EXIT') != '2':
        fails.append(f'route_bus with no such parts: exit {s.get("EXIT")}, want 2')
    if s['PAIRS'] != '1':
        fails.append(f'after route_bus braid.PAIRS is {s["PAIRS"]}: its stages would lay a pair as two single lanes')
    for x in fails:
        print(f'  FAIL: {x}')
    if fails:
        return 1
    print('PASS: route_bus loads no chain module; the buses found and the step import the chain with a pair one lane')
    return 0


if __name__ == '__main__':
    sys.exit(main())
