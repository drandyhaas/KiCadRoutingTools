"""#1541: a seed's s<S>.log carries its progress while the seed runs.

With `{seed}` in OUT, place_simple runs each seed as a child process whose
stdout is `s<S>.log`. That stdout was block-buffered, so the anneal's
progress lines and its estimate (#1483) -- which the placement skill says to
read before waiting on a large board -- reached the file only after 8 kB had
piled up or when the seed exited. On a small board the file stayed empty for
the whole run.

The check: launch one seed the `{seed}` way and poll its log. The estimate
must be in the file BEFORE the seed has written its placed board, which it
does only at the end. Also pinned: every tenth of the anneal prints its
progress line, including a tenth whose last move proposed nothing.
"""

import json
import os
import re
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import evidence  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def run(*args):
    return subprocess.run([sys.executable, '-X', 'utf8'] + list(args), capture_output=True,
                          text=True, cwd=ROOT)


def main():
    evidence(BOARD, 'the fixture board')
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'b.kicad_pcb')
        r = run(os.path.join(ROOT, 'py_router', 'copy_board.py'), BOARD, board)
        evidence(board, f'the copied board ({r.stderr[-200:]})')
        intent = os.path.join(td, 'intent.json')
        r = run(os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), board,
                '--emit-intent', intent)
        evidence(intent, f'the emitted intent ({r.stderr[-200:]})')
        groups = os.path.join(td, 'groups.json')
        json.dump({'schema': 1, 'groups': [{'name': 'u1', 'refs': ['U1']}]},
                  open(groups, 'w'))
        out = os.path.join(td, 's{seed}.kicad_pcb')
        seed_log = os.path.join(td, 's0.log')
        seed_board = os.path.join(td, 's0.kicad_pcb')
        proc = subprocess.Popen(
            [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_placer', 'place_simple.py'),
             board, out, '--groups', groups, '--intent', intent, '--seeds', '0',
             '--json', os.path.join(td, 's{seed}.json')],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        live = None         # did the estimate reach the log before the seed's board?
        deadline = time.time() + 600
        while proc.poll() is None and time.time() < deadline:
            if live is None and os.path.exists(seed_log):
                with open(seed_log, encoding='utf-8') as f:
                    if 'estimate:' in f.read():
                        live = not os.path.exists(seed_board)
            time.sleep(0.05)
        if proc.poll() is None:
            proc.kill()
        parent = proc.communicate()[0]
        ok('the seed ran and wrote its board', proc.returncode in (0, 4)
           and os.path.exists(seed_board), parent[-400:])
        if not os.path.exists(seed_log):
            ok('the seed wrote its log', False)
            return 1
        text = open(seed_log, encoding='utf-8').read()
        ok('the seed log carries an estimate', 'estimate:' in text, text[-400:])
        ok('the estimate reached the log while the seed was still annealing', live is True,
           'it appeared only once the seed had written its board' if live is False
           else 'never seen while the parent ran')
        # each anneal logs ten tenths; group the progress lines per anneal by
        # the move counter starting over
        tenths, runs = [], []
        for m in re.finditer(r'move +(\d+):', text):
            k = int(m.group(1))
            if tenths and k <= tenths[-1]:
                runs.append(tenths)
                tenths = []
            tenths.append(k)
        if tenths:
            runs.append(tenths)
        ok('every anneal prints all ten progress lines', runs and all(len(t) == 10 for t in runs),
           [len(t) for t in runs])
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
