#!/usr/bin/env python3
"""place_portfolio's determinism contract, test_457-style.

Same --seed + same input must be byte-identical ACROSS PYTHONHASHSEED values
(the #457 hazard: set-of-strings iteration reaching anything that decides a
move), a different --seed must actually change the portfolio, and --only N
must regenerate candidate N byte-identically -- that is the property the
ledger's replay command rests on.
"""

#: Measured ALONE at 366 s. It was killed at the runner's 600 s
#: default under -j contention and reported as a timeout, which reads
#: exactly like a hang; it passes 11/11 every time it is given room.
RUN_ALL_TIMEOUT = 1200

import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(REPO, "kicad_files", "splitflap_driver.kicad_pcb")

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


def run(out_dir, *extra, hashseed="0"):
    env = dict(os.environ, PYTHONHASHSEED=hashseed, PYTHONIOENCODING='utf-8')
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(REPO, 'py_placer', 'place_portfolio.py'), BOARD,
            '--out-dir', out_dir, '--seed', '0', '--candidates', '3',
            '--keep', '2', '--route-top', '0', '--no-render'] + list(extra)
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', env=env, cwd=REPO, timeout=1200)
    return r


def canon_json(out_dir):
    """portfolio.json with the run's own out-dir path neutralized, so runs
    into different directories compare equal iff everything else is."""
    with open(os.path.join(out_dir, 'portfolio.json'), encoding='utf-8') as f:
        text = f.read()
    return text.replace(json.dumps(out_dir)[1:-1], '@OUT@')


def board_bytes(out_dir, name):
    with open(os.path.join(out_dir, name), 'rb') as f:
        return f.read()


if not os.path.isfile(BOARD):
    print("SKIP: fixture missing")
    sys.exit(0)

with tempfile.TemporaryDirectory() as d:
    dirs = {k: os.path.join(d, k) for k in
            ('h0', 'h1', 'h12345', 'seed1', 'only')}

    # 1. identical across PYTHONHASHSEED (and across processes)
    for key, hs in (('h0', '0'), ('h1', '1'), ('h12345', '12345')):
        r = run(dirs[key], hashseed=hs)
        check(f"run under PYTHONHASHSEED={hs} exits 0", r.returncode == 0,
              (r.stdout + r.stderr)[-400:])
    j0, j1, j2 = (canon_json(dirs[k]) for k in ('h0', 'h1', 'h12345'))
    check("portfolio.json identical across hash seeds",
          j0 == j1 == j2)
    b0 = board_bytes(dirs['h0'], 'cand_01.kicad_pcb')
    check("cand_01 board identical across hash seeds",
          b0 == board_bytes(dirs['h1'], 'cand_01.kicad_pcb')
          == board_bytes(dirs['h12345'], 'cand_01.kicad_pcb'))
    check("baseline board identical across hash seeds",
          board_bytes(dirs['h0'], 'baseline_quenched.kicad_pcb')
          == board_bytes(dirs['h1'], 'baseline_quenched.kicad_pcb'))

    # 2. a different --seed actually changes the perturbation
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(REPO, 'py_placer', 'place_portfolio.py'), BOARD,
         '--out-dir', dirs['seed1'], '--seed', '1', '--candidates', '3',
         '--keep', '2', '--route-top', '0', '--no-render'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=REPO, timeout=1200)
    check("seed 1 exits 0/4", r.returncode in (0, 4), f"rc={r.returncode}")
    check("seed 1 cand_01 differs from seed 0's",
          board_bytes(dirs['seed1'], 'cand_01.kicad_pcb') != b0)

    # 3. --only N regenerates candidate N byte-identically
    r = run(dirs['only'], '--only', '1')
    check("--only 1 exits 0", r.returncode == 0,
          (r.stdout + r.stderr)[-400:])
    check("--only 1 board matches the full run's cand_01",
          board_bytes(dirs['only'], 'cand_01.kicad_pcb') == b0)
    check("--only 1 skipped the baseline", not os.path.isfile(
        os.path.join(dirs['only'], 'baseline_quenched.kicad_pcb')))

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
