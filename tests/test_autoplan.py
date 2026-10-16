#!/usr/bin/env python3
"""autoplan.py: the plan it reads off a board, the pass gating, and a run.

1. `_glob` turns every net name into an fnmatch pattern that matches exactly
   that net and that argparse will not read as an option ('-BATT' ->
   '[-]BATT'; a bare '-BATT' crashed route.py's parser).
2. `analyze` on in-repo boards: planes on B.Cu for 2 layers and on the first
   two inner layers for 4+, plane layers priced up, fine-pitch parts fanned
   out on the fine grid.
3. `route_passes` with a fake Chain: passes stop when one closes no nets, at
   zero open, and when the iteration budget (--effort-factor) would be
   exceeded -- all on counts, never on time.
4. A real run on splitflap_driver, end to end and with --finish, graded by
   check_connected rather than by autoplan's own summary.
"""
import fnmatch
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import autoplan as AP                                        # noqa: E402
from run_utils import evidence                               # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
FAILS = []


def expect(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILS.append(what)


def test_glob():
    print("_glob / _exact_ok")
    names = ['GND', '/+3V3', '-BATT', '-5V', 'D[0]', 'A*B', 'Q?', '/USB_D-']
    others = names + ['GNDA', '/+3V31', 'BATT', 'D0', 'AxB', 'QQ', '[-]BATT']
    for n in names:
        pat = AP._glob(n)
        hits = [o for o in others if fnmatch.fnmatchcase(o, pat)]
        expect(hits == [n], f"{n!r} -> {pat!r} matches only itself ({hits})")
        expect(AP._exact_ok(pat), f"{pat!r} is safe to pass to argparse")
    expect(not AP._exact_ok('-BATT') and AP._exact_ok('-5V'),
           "'-BATT' is not an exact value argparse accepts, '-5V' is")


def test_analyze():
    print("analyze")
    a = AP.analyze(BOARD)
    expect(a['layers'] == ['F.Cu', 'B.Cu'], "splitflap: 2 layers")
    expect(a['planes'] == [('GND', 'B.Cu')], f"splitflap: GND on B.Cu ({a['planes']})")
    expect(a['parts'] == [] and a['grid'] == 0.1, "splitflap: no fanout, 0.1 grid")
    expect(a['plane_nets'][0] in a['power_nets'], "plane net is a power net")
    expect(a['clearance'] >= a['fab_clearance'] > 0, "clearance at or above the fab floor")

    b = AP.analyze(os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb'))
    L = b['layers']
    expect([l for _, l in b['planes']] == L[1:3],
           f"orangecrab: planes on the first two inner layers ({b['planes']})")
    expect(b['layer_costs'][1:3] == [3.0, 3.0] and b['layer_costs'][0] == 1.0,
           f"orangecrab: plane layers priced up ({b['layer_costs']})")
    tools = {r: t for r, t, _ in b['parts']}
    expect(tools.get('U1') == 'bga' and 'qfn' in tools.values(),
           f"orangecrab: BGAs and fine-pitch QFNs fanned out ({tools})")
    expect(b['grid'] == 0.05 and b['congested'], "orangecrab: fine grid, congested")
    excl = {f'!{AP._glob(n)}' for n in b['plane_nets']}
    expect(len(excl) == len(b['plane_nets']), "plane nets excluded from fanout")


class FakeChain:
    """Chain.run replaced by a script of (open, iterations) per pass."""

    def __init__(self, script):
        self.script = list(script)
        self.last_iterations = 0
        self.runs = 0
        self.cur = 'in'

    def run(self, kind, args, tag):
        n_open, it = self.script[self.runs]
        self.runs += 1
        self.last_iterations = it
        return f'out{self.runs}', {}, {'failed_single': ['x'] * n_open}

    def accept(self, out):
        self.cur = out


def _passes(script, effort_factor=2.0):
    a = dict(clearance=0.1, track=0.1, via=0.45, drill=0.2, grid=0.1,
             power_nets=[], congested=False)
    ch = FakeChain(script)
    _, best, n_open = AP.route_passes(ch, a, ['F.Cu', 'B.Cu'], ['1', '1.5'],
                                      effort_factor)
    return ch.runs, best, n_open


def test_passes():
    print("route_passes")
    expect(_passes([(0, 100)] * 3) == (1, 'out1', 0), "zero open: one pass")
    expect(_passes([(5, 100), (3, 50), (2, 40)]) == (3, 'out3', 2),
           "progress within budget: three passes")
    expect(_passes([(5, 100), (5, 50), (1, 10)]) == (2, 'out2', 5),
           "a pass that closes nothing ends the passes")
    expect(_passes([(5, 100), (3, 120), (1, 10)]) == (2, 'out2', 3),
           "100 + 120 + 120 > 2 x 100: pass 3 not started")
    expect(_passes([(5, 100), (3, 120), (1, 10)], effort_factor=3.5) == (3, 'out3', 1),
           "a larger --effort-factor lets pass 3 run")
    expect(_passes([(5, 100), (6, 50), (6, 40)]) == (2, 'out1', 5),
           "a worse pass never replaces the best board")


def _connected(board):
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'check_connected.py'),
                        board], capture_output=True, text=True, timeout=600)
    return r.returncode, r.stdout + r.stderr


def test_run():
    print("end to end on splitflap_driver")
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'routed.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'autoplan.py'),
                            BOARD, out], capture_output=True, text=True,
                           timeout=1800)
        expect(r.returncode == 0, f"autoplan exits 0 (rc={r.returncode})")
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
            return
        evidence(out, 'routed board')
        expect(os.path.exists(out[:-len('.kicad_pcb')] + '.kicad_pro'),
               "the routed board ships with its .kicad_pro")
        plan = json.load(open(out + '.autoplan/plan.json'))
        tags = [s['tag'] for s in plan['steps']]
        expect(tags[0] == 'planes' and tags[1] == 'route',
               f"planes first, then route ({tags})")
        expect(all(isinstance(s['iterations'], int) for s in plan['steps']),
               "every step records its router iterations")
        rc, text = _connected(out)
        expect(rc == 0, "check_connected: fully routed")
        if rc:
            print(text[-2000:])

        fin = os.path.join(tmp, 'finished.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'autoplan.py'),
                            out, fin, '--finish'], capture_output=True,
                           text=True, timeout=1800)
        expect(r.returncode == 0, f"--finish exits 0 (rc={r.returncode})")
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
            return
        evidence(fin, 'finished board')
        fplan = json.load(open(fin + '.autoplan/plan.json'))
        expect([s['tool'] for s in fplan['steps']] == ['route'] * len(fplan['steps']),
               "--finish runs route passes only")
        expect(('GND', 'B.Cu') in [tuple(p) for p in fplan['analysis']['planes']],
               "--finish reads the poured plane from the board's zones")
        expect(_connected(fin)[0] == 0, "check_connected: still fully routed")


def main():
    test_glob()
    test_analyze()
    test_passes()
    test_run()
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS: autoplan")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
