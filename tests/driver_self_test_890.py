#!/usr/bin/env python3
"""Run `loop_driver.py --self-test` as a gate file, for `tests/mutate_890.py`.

NOT named `test_*`: `run_all.py` already covers the driver's self-test through
`tests/test_run8_skill_drivers.py`, and a second collector running the same
thing would double a two-minute cost for nothing.

It exists because the runtime L5 assertion -- that the end-to-end verifier is
spawned FRESH and never forked, in both `--delegate-mode` arms -- lives in the
self-test rather than in `test_890_delegation_handoff.py`. It has to: reaching
L5's verifier prompt needs a ledger, a score, a routing close-out and a board
whose sha matches, and the self-test is where that fixture already exists.
Every cheaper fixture makes L5 return `<error>`, against which "the tag is not
fork" is true for free -- which is exactly the tautology an earlier draft of
this PR shipped and a test caught.

So the battery needs the self-test as something it can name in a row's gate
list, and a gate list holds files it runs. This is that file.

Exit 0 pass, 1 fail. Never 77.
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRIVER = os.path.join(ROOT, '.claude', 'skills',
                      'plan-pcb-placement-and-routing', 'scripts',
                      'loop_driver.py')

if not os.path.isfile(DRIVER):
    # A FAILURE, not a skip. The battery reads a skip as "nothing asserted"
    # and would report every row it guards as unevidenced; a missing driver is
    # a broken tree, which is a different thing and must look different.
    print(f'the driver is missing: {DRIVER}')
    sys.exit(1)

r = subprocess.run([sys.executable, '-X', 'utf8', DRIVER, '--self-test'],
                   cwd=ROOT, capture_output=True, text=True,
                   encoding='utf-8', errors='replace', timeout=1800)
out = (r.stdout or '') + (r.stderr or '')
for line in out.splitlines():
    if line.strip().startswith('FAIL') or line.strip() in ('OK', 'FAILED'):
        print(line)
# The self-test prints one PASS/FAIL line per check and `OK` at the end. Both
# are asserted: a run that exits 0 without ever printing `OK` did not finish,
# and reading only the exit code would call that a pass.
if r.returncode != 0 or 'OK' not in out.splitlines()[-1:]:
    print(f'loop_driver --self-test did not pass (exit {r.returncode})')
    print(out[-2000:])
    sys.exit(1)
print('loop_driver --self-test OK')
sys.exit(0)
