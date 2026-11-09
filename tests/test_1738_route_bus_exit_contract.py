#!/usr/bin/env python3
"""#1738: route_bus.py exits 0 whenever it writes OUT.

A plan file runs under `set -e` (route_plan_check R01, #1735), and the bus
step used to exit 1 when it handed OUT on as the board as it came -- which
the routing skill calls "not a chain failure": the rest of the chain routes
the whole bus. That exit stopped the plan at the bus step.

Checks:
  1. the CLI on a board with no bus: OUT is the board as it came, exit 0, the
     summary keeping the step's own status (exit 1, none found);
  2. a bus named with a part the board does not have is a usage error: exit
     2, naming it, and no OUT written;
  3. --src/--dest with a part the board does not have: exit 2;
  4. `cli_exit`: 0 for a status 0 or 1 with OUT written by the run, the
     status otherwise (2 stays 2), 1 for a run with no status and no OUT;
  5. route_buses with a bus left as it came (the step stood in for): status
     1 in the summary, OUT written, and the process exit 0.

    python3 tests/test_1738_route_bus_exit_contract.py
"""
import json
import os
import shutil
import sys
import tempfile

try:
    import ortools  # noqa: F401  (route_bus plans with CP-SAT)
except ImportError as exc:
    # ipc-migration lists no ortools in requirements.txt (it cannot share an
    # environment with kicad-python), so the suite image has none, and
    # route_bus refuses every arm at its dependency gate.
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'tests'), os.path.join(ROOT, 'awx'),
                os.path.join(ROOT, 'py_router')]
from run_utils import check, evidence  # noqa: E402

TOOL = os.path.join(ROOT, 'awx', 'route_bus.py')
BOARD = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def cli(td):
    evidence(BOARD, 'fixture board')
    print('1. no bus on the board: OUT is the board as it came, exit 0')
    out = os.path.join(td, 'none.kicad_pcb')
    r = check([sys.executable, '-X', 'utf8', TOOL, BOARD, out], accept=True, timeout=600)
    summ = os.path.join(td, 'none.bus', 'summary.json')
    s = json.load(open(evidence(summ, 'summary'), encoding='utf-8'))
    ok('exit 0 with OUT written', os.path.isfile(out), r.stdout[-300:])
    ok("...the summary keeps the step's status: none found, exit 1",
       s.get('exit') == 1 and s.get('buses') == [] and s.get('found') == [], str(s)[:300])
    ok('...and the console says OUT is the board as it came',
       'OUT is the board as it came' in r.stdout + r.stderr, (r.stdout + r.stderr)[-300:])

    print('2-3. a part the board does not have is a usage error')
    out = os.path.join(td, 'bad.kicad_pcb')
    check([sys.executable, '-X', 'utf8', TOOL, BOARD, out, '--bus', 'U1:NOPE9'],
          refuse='no footprint NOPE9', code=2, timeout=300)
    ok('--bus U1:NOPE9: exit 2, naming the part, and no OUT', not os.path.exists(out))
    check([sys.executable, '-X', 'utf8', TOOL, BOARD, out, '--src', 'NOPE8', '--dest', 'U1'],
          refuse='no footprint NOPE8', code=2, timeout=300)
    ok('--src NOPE8: exit 2, naming the part, and no OUT', not os.path.exists(out))


def unit(td):
    import route_bus as rb
    print('4. cli_exit')
    out = os.path.join(td, 'u.kicad_pcb')
    rb.HANDED_ON.discard(os.path.abspath(out))
    ok('status 1 with no OUT written: 1', rb.cli_exit(1, out) == 1)
    ok('status 2 with no OUT written: 2', rb.cli_exit(2, out) == 2)
    ok('no status, no OUT: 1 (nothing usable)', rb.cli_exit(0, out) == 1)
    rb.HANDED_ON.add(os.path.abspath(out))
    ok('status 1 with OUT written (the board as it came): 0', rb.cli_exit(1, out) == 0)
    ok('status 0 with OUT written: 0', rb.cli_exit(0, out[:-len('.kicad_pcb')]) == 0)

    print('5. route_buses with a bus left as it came')

    def step(board, out_, src, dest, *a, **kw):
        shutil.copy(board, out_)
        shutil.copy(board[:-len('.kicad_pcb')] + '.kicad_pro', out_[:-len('.kicad_pcb')] + '.kicad_pro')
        rb.HANDED_ON.add(os.path.abspath(out_))
        return 1, f'BUS {src}->{dest}'
    real = rb.route_bus
    rb.route_bus = step
    try:
        b0 = os.path.join(td, 'b.kicad_pcb')
        open(b0, 'w').write('(kicad_pcb)\n')
        open(os.path.join(td, 'b.kicad_pro'), 'w').write('{}\n')
        o = os.path.join(td, 'o.kicad_pcb')
        code, _g = rb.route_buses(b0, o, [('U1', 'U2', 'plain', None)], log=lambda *a: None)
    finally:
        rb.route_bus = real
    s = json.load(open(os.path.join(td, 'o.bus', 'summary.json'), encoding='utf-8'))
    ok('the step status is 1 (as it came), in the summary too', code == 1 and s['exit'] == 1, f'{code} {s}')
    ok('...OUT written, and the process exit 0', os.path.isfile(o) and rb.cli_exit(code, o) == 0,
       f'{rb.cli_exit(code, o)}')


def main():
    with tempfile.TemporaryDirectory() as td:
        cli(td)
    with tempfile.TemporaryDirectory() as td:
        unit(td)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
