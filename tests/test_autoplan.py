#!/usr/bin/env python3
"""autoplan.py: the plan it reads off a board, how it picks a board, and a run.

1. `_glob` gives every net a pattern that selects that net alone through the
   selectors the chain's tools use -- `expand_net_patterns` (route.py
   --nets), `matches_net_filter` with '!' exclusions (the fanouts) and
   `identify_power_nets` (--power-nets) -- and never starts with '!' or '-'.
   A net no pattern isolates on the board is left out of the analysis.
2. `analyze` on in-repo boards: planes on B.Cu for 2 layers and on the first
   two inner layers for 4+, plane layers priced up, fine-pitch parts fanned
   out on the fine grid.
3. `route_passes` with a scripted chain: a pass is kept only when it is no
   worse on open nets and DRC and better on one; --finish keeps the input
   unless a pass beats it; the iteration budget is a forecast from reported
   counts, and a pass that reports none starts no other.
4. --effort-factor refuses a non-finite or negative value.
5. Real runs on splitflap_driver: end to end, --finish, and --finish on a
   board inside the output's own work directory with a stale sibling at the
   output -- graded by check_connected, not by autoplan's own summary.
"""
import json
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import autoplan as AP                                        # noqa: E402
from net_queries import (expand_net_patterns, identify_power_nets,  # noqa: E402
                         matches_net_filter)
from run_utils import check, evidence                        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
SCRIPT = os.path.join(ROOT, 'py_router', 'autoplan.py')
FAILS = []


def expect(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILS.append(what)


def _stub_board(names):
    nets = {i + 1: SimpleNamespace(name=n) for i, n in enumerate(names)}
    pads = {i: [SimpleNamespace(net_name=n.name)] for i, n in nets.items()}
    return SimpleNamespace(nets=nets, pads_by_net=pads)


def test_glob():
    print("_glob through the real selectors")
    names = ['GND', '/GND', '/Analog/GND', 'GNDA', '-BATT', 'BATT', '!D[0]',
             'D[0]', 'D0', '/D[1]', 'A*B', 'AxB', 'Q?', 'QQ', '-5V', ']x', '[x',
             '+5V', '/5V', 'GN', 'x]', 'x^', '!RESET', 'RESET', '/Sheet/!RESET', 'A']
    pcb = _stub_board(names)
    ids = {n.name: i for i, n in pcb.nets.items()}
    for n in names:
        pat = AP._glob(n)
        others = sorted(set(names) - {n})
        expect(not pat.startswith(('!', '-')), f"{n!r} -> {pat!r} is not an option or an exclusion")
        expect(AP._selects_exactly(pat, n, names), f"{pat!r} isolates {n!r}")
        expect(expand_net_patterns(pcb, [pat], quiet=True) == [n],
               f"--nets {pat!r} selects only {n!r}")
        expect(sorted(expand_net_patterns(pcb, ['*', '!' + pat], quiet=True)) == others,
               f"--nets '*' '!{pat}' drops only {n!r}")
        expect([m for m in names if not matches_net_filter(m, ['*', '!' + pat])] == [n],
               f"fanout filter '!{pat}' excludes only {n!r}")
        expect(set(identify_power_nets(pcb, [pat], [0.3])) == {ids[n]},
               f"--power-nets {pat!r} widens only {n!r}")
    expect(not AP._selects_exactly(AP._glob('!D[0]'), '!D[0]', ['!D[0]', ' D[0]']),
           "'!D[0]' beside ' D[0]' cannot be isolated, and that is detected")


def test_analyze():
    print("analyze")
    a = AP.analyze(BOARD)
    expect(a['layers'] == ['F.Cu', 'B.Cu'], "splitflap: 2 layers")
    expect(a['planes'] == [('GND', 'B.Cu')], f"splitflap: GND on B.Cu ({a['planes']})")
    expect(a['parts'] == [] and a['grid'] == 0.1, "splitflap: no fanout, 0.1 grid")
    expect(a['plane_nets'][0] in a['power_nets'], "plane net is a power net")
    expect(all(n in a['globs'] for n in a['power_nets'] + a['pair_nets']),
           "every net the chain names has a verified pattern")
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


class ScriptedChain:
    """Chain.run and Chain.grade replaced by a script: per pass
    (grade or None, iterations or None); `input` grades the starting board."""

    def __init__(self, passes, input_grade=None):
        self.passes = list(passes)
        self.input_grade = input_grade
        self.last_iterations = None
        self.runs = 0
        self.cur = 'in'
        self.steps = []

    def grade(self, board):
        if board == 'in':
            return self.input_grade
        return self.passes[int(board[3:]) - 1][0]

    def run(self, kind, args, tag):
        self.runs += 1
        self.last_iterations = self.passes[self.runs - 1][1]
        self.steps.append({'tag': tag, 'ok': True})
        return f'out{self.runs}', {}, {}

    def accept(self, out):
        self.cur = out


def G(n_open, drc=0):
    return {'open': n_open, 'drc': drc, 'open_nets': []}


def _passes(script, effort_factor=2.0, finish=False, input_grade=None):
    a = dict(clearance=0.1, track=0.1, via=0.45, drill=0.2, grid=0.1,
             power_nets=[], congested=False, globs={})
    ch = ScriptedChain(script, input_grade)
    _, best, g = AP.route_passes(ch, a, ['F.Cu', 'B.Cu'], ['1', '1.5'],
                                 effort_factor, finish=finish)
    return ch.runs, best, g and (g['open'], g['drc'])


def test_passes():
    print("route_passes")
    expect(_passes([(G(0), 100)] * 3) == (1, 'out1', (0, 0)), "zero open: one pass")
    expect(_passes([(G(5), 100), (G(3), 50), (G(2), 40)]) == (3, 'out3', (2, 0)),
           "improving within budget: three passes")
    expect(_passes([(G(5), 100), (G(5), 50), (G(1), 10)]) == (2, 'out1', (5, 0)),
           "a pass no better than the best is not kept and ends the passes")
    expect(_passes([(G(5, 2), 100), (G(4, 3), 50), (G(1), 10)]) == (2, 'out1', (5, 2)),
           "fewer open nets bought with more DRC is not kept")
    expect(_passes([(G(5, 3), 100), (G(5, 1), 50), (G(5, 1), 10)]) == (3, 'out2', (5, 1)),
           "fewer DRC at the same open count is kept")
    expect(_passes([(G(5), 100), (G(3), 120), (G(1), 10)]) == (2, 'out2', (3, 0)),
           "100 + 120 + 120 > 2 x 100: pass 3 not started")
    expect(_passes([(G(5), 100), (G(3), 120), (G(1), 10)], effort_factor=3.4) == (3, 'out3', (1, 0)),
           "100 + 120 + 120 = 340 <= 3.4 x 100: the forecast admits equality")
    expect(_passes([(G(5), 100), (G(3), None), (G(1), 10)]) == (2, 'out2', (3, 0)),
           "a pass that reports no iterations starts no further pass")
    expect(_passes([(G(5), None), (G(3), 10)]) == (1, 'out1', (5, 0)),
           "a first pass with no iterations gives no budget")
    expect(_passes([(G(5), 0), (G(3), 0), (G(1), 0)]) == (3, 'out3', (1, 0)),
           "zero reported iterations: 0 + 0 <= 2 x 0, the passes run")
    expect(_passes([(None, 100)]) == (1, 'out1', None),
           "an ungraded first pass still ships (full chain)")

    print("route_passes --finish")
    expect(_passes([(G(30, 71), 100)], finish=True, input_grade=G(30, 53)) == (1, 'in', (30, 53)),
           "a pass adding DRC without closing a net loses to the input")
    expect(_passes([(G(31, 0), 100)], finish=True, input_grade=G(30, 5)) == (1, 'in', (30, 5)),
           "a pass opening a net loses to the input")
    expect(_passes([(G(2, 1), 100), (G(1, 1), 50), (G(1, 1), 50)], finish=True,
                   input_grade=G(4, 1)) == (3, 'out2', (1, 1)),
           "passes that beat the input are kept, a tie is not")
    expect(_passes([(G(2), 100)], finish=True, input_grade=G(0, 3)) == (0, 'in', (0, 3)),
           "a fully connected input runs no pass")
    expect(_passes([(G(2), 100)], finish=True, input_grade=None) == (0, None, None),
           "an input that cannot be graded runs no pass")


def test_effort_factor():
    print("--effort-factor")
    for bad in ('nan', 'inf', '-1'):
        check([sys.executable, '-X', 'utf8', SCRIPT, BOARD, os.devnull + '.kicad_pcb',
               '--effort-factor', bad], refuse='must be a finite number >= 0', code=2,
              allow=('error: argument',))
        expect(True, f"--effort-factor {bad} refused for its reason")


def _connected(board):
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'check_connected.py'),
                        board], capture_output=True, text=True, timeout=600)
    return r.returncode, r.stdout + r.stderr


def _autoplan(*argv):
    return subprocess.run([sys.executable, '-X', 'utf8', SCRIPT, *argv],
                          capture_output=True, text=True, timeout=1800)


def test_run():
    print("end to end on splitflap_driver")
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'routed.kicad_pcb')
        r = _autoplan(BOARD, out)
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
        expect(plan['grade'] is not None and plan['grade']['open'] == 0,
               f"the shipped board's grade is recorded ({plan['grade']})")
        rc, text = _connected(out)
        expect(rc == 0, "check_connected: fully routed")
        if rc:
            print(text[-2000:])

        fin = os.path.join(tmp, 'finished.kicad_pcb')
        r = _autoplan(out, fin, '--finish')
        expect(r.returncode == 0, f"--finish exits 0 (rc={r.returncode})")
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
            return
        evidence(fin, 'finished board')
        fplan = json.load(open(fin + '.autoplan/plan.json'))
        expect([s['tag'] for s in fplan['steps']] == ['input'],
               "--finish on a fully connected board grades it and runs no pass")
        expect(('GND', 'B.Cu') in [tuple(p) for p in fplan['analysis']['planes']],
               "--finish reads the poured plane from the board's pours")
        expect(_connected(fin)[0] == 0, "check_connected: still fully routed")

        print("input inside the output's work directory, stale sibling at the output")
        inner = [s for s in plan['steps'] if s['tag'] == 'route'][0]['log']
        inner = os.path.join(out + '.autoplan', inner[:-len('.log')] + '.kicad_pcb')
        evidence(inner, 'step board inside the work directory')
        stale = out[:-len('.kicad_pcb')] + '.kicad_dru'
        open(stale, 'w').write('(version 1)\n(rule stale (constraint clearance (min 5mm)))\n')
        r = _autoplan(inner, out, '--finish')
        expect(r.returncode == 0 and 'Traceback' not in r.stdout + r.stderr,
               f"a board inside <output>.autoplan/ is staged before the directory is cleared (rc={r.returncode})")
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
            return
        expect(not os.path.exists(stale), "a sibling the shipped board lacks is removed")
        expect(_connected(out)[0] == 0, "check_connected: the re-shipped board is fully routed")


def main():
    test_glob()
    test_analyze()
    test_passes()
    test_effort_factor()
    test_run()
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS: autoplan")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
