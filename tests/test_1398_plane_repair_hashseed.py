#!/usr/bin/env python3
"""#1398: the plane-repair engine returns the same board whatever the process's
PYTHONHASHSEED.

Fill-model region discovery was handed a SET of layer-name strings and
attributed each anchor to the first layer whose island it touched, so the
number of regions found (and the joins routed from them) followed the string
hash seed: one board for some seeds, another for the rest. Determinism is
proved the way CLAUDE.md asks -- by comparing the outcome of two processes
that differ only in PYTHONHASHSEED (copper line counts and the region count the
engine prints), never by hashing files (UUIDs differ per run).

Chain: strip a corpus board, route its signals, pour GND/+3V3 planes, lay a
foreign B.Cu stub under an F.Cu GND pad (so the B.Cu-primary fill query misses
there), then run `repair_planes.py` under two seeds known to order the layer
set differently.

    python3 tests/test_1398_plane_repair_hashseed.py
"""
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
from run_utils import check, evidence, tool        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
SEEDS = ('0', '1')
PY = [sys.executable, '-X', 'utf8']


def step(argv, log=None, **kw):
    r = check(PY + argv, accept=True, **kw)
    if log:
        with open(log, 'w', encoding='utf-8') as f:
            f.write((r.stdout or '') + (r.stderr or ''))
    return r


def copper_lines(path):
    with open(path, encoding='utf-8') as f:
        return sum(1 for l in f if re.match(r'\s*\((segment|via)', l))


def main():
    evidence(BOARD, 'tigard board')
    tmp = tempfile.mkdtemp(prefix='t1398_')
    try:
        bare = os.path.join(tmp, 'bare.kicad_pcb')
        sig = os.path.join(tmp, 'sig.kicad_pcb')
        pour = os.path.join(tmp, 'pour.kicad_pcb')
        stub = os.path.join(tmp, 'stub.kicad_pcb')
        step([os.path.join(ROOT, 'tests', 'stress', 'strip_copper_only.py'),
              BOARD, bare])
        step([tool('route.py'), bare, sig, '--nets', '*', '!GND', '!+3V3',
              '--clearance-ceiling', '0.15', '--track-width', '0.15',
              '--via-size', '0.5', '--via-drill', '0.3',
              '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'])
        step([tool('route_planes.py'), sig, pour, '--nets', 'GND', 'GND',
              'GND', '+3V3', '--plane-layers', 'F.Cu', 'In1.Cu', 'B.Cu',
              'In2.Cu'])
        step([tool('check_join.py'), pour, 'VREG', '57.2,64.15,B.Cu',
              '57.8,64.15,B.Cu', '--width', '0.15', '--keep-staged', stub,
              '-q'])
        result = {}
        for seed in SEEDS:
            out = os.path.join(tmp, f'out_{seed}.kicad_pcb')
            env = dict(os.environ, PYTHONHASHSEED=seed)
            r = step([tool('repair_planes.py'), stub, out, '--nets', 'GND',
                      'GND', 'GND', '--plane-layers', 'F.Cu', 'In1.Cu',
                      'B.Cu', '--no-repair-pads'], env=env)
            regions = re.findall(r'Region discovery from fill model: (\d+)',
                                 (r.stdout or '') + (r.stderr or ''))
            result[seed] = (copper_lines(evidence(out, f'seed {seed} output')),
                            regions)
            print(f'  PYTHONHASHSEED={seed}: {result[seed]}')
        assert result[SEEDS[0]][1], 'engine printed no region discovery line'
        assert result[SEEDS[0]] == result[SEEDS[1]], (
            'plane repair differs between PYTHONHASHSEED values: '
            f'{result}')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('PASS')


if __name__ == '__main__':
    main()
