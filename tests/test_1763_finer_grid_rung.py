"""#1763: the final reconciliation's finer-grid rung.

A fine-pitch net left boxed in at the run's grid was never retried at a finer
grid WITH rip-up: the per-net rescue's finer grid routes through free space
only, the terminal escalation and the reconciliation laps keep the run's grid.
The fixture is #1763's: a 0.5 mm-pitch QFN whose pads sit off the 0.1 grid,
eight nets joining its top row to its bottom row, a 0.28 clearance rule. At
grid 0.1 the step used to ship 4/8; the rung re-routes the boxed-in four at
grid 0.05 with rip authority over the nets walling them.

Also pins the rung's planning (who qualifies, the cell budget) without
routing.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_utils import ROOT_DIR, tool, tool_env, evidence  # noqa: E402

sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
import finer_grid_rung as fgr  # noqa: E402
import routing_defaults as defaults  # noqa: E402


def test_plan_selects_boxed_in_only():
    tlo = {'A': {'boxed_in': True, 'blockers': ['X', 'Y']},
           'B': {'boxed_in': False, 'blockers': ['Z']},
           'C': {'boxed_in': False, 'blockers': []}}
    names, fine, cells, why = fgr.plan(tlo, {'C'}, 0.1, (0, 0, 10, 10), 2)
    assert names == ['A', 'C'], names
    assert abs(fine - 0.05) < 1e-12 and why is None
    assert abs(cells - 2 * 200 * 200) < 1e-6, cells
    assert fgr.rung_blockers(tlo, names) == ['X', 'Y']
    # a target is never its own blocker
    tlo2 = {'A': {'boxed_in': True, 'blockers': ['C']},
            'C': {'boxed_in': True, 'blockers': ['A', 'W']}}
    assert fgr.rung_blockers(tlo2, ['A', 'C']) == ['W']


def test_plan_nothing_to_do():
    assert fgr.plan({'B': {'boxed_in': False}}, set(), 0.1,
                    (0, 0, 10, 10), 2) == ([], None, 0.0, None)


def test_plan_respects_cell_budget():
    side = (defaults.FINER_GRID_RUNG_MAX_CELLS / 4) ** 0.5 * 0.05 * 1.01
    names, fine, cells, why = fgr.plan({'A': {'boxed_in': True}}, set(), 0.1,
                                       (0, 0, side, side), 4)
    assert names == [] and why and 'budget' in why, (names, why)
    assert cells > defaults.FINER_GRID_RUNG_MAX_CELLS


def _make_fixture(d):
    base = os.path.join(d, 'base.kicad_pcb')
    subprocess.run([sys.executable, '-B', tool('copy_board.py'),
                    os.path.join(ROOT_DIR, 'kicad_files',
                                 'qfn_underpad_coupling.kicad_pcb'), base],
                   check=True, capture_output=True, env=tool_env())
    s = open(base, encoding='utf-8').read()
    net_of = {}
    for k, top in enumerate(range(25, 33)):   # top pad p sits above 41 - p
        net_of[str(top)] = net_of[str(41 - top)] = f'N{k}'

    def add_net(m):
        if m.group(2) not in net_of:
            return m.group(0)
        return (m.group(1) + '\t\t\t(net "' + net_of[m.group(2)] + '")\n'
                + m.group(3))
    s = re.sub(r'(\(pad "(\d+)" smd roundrect\n(?:\t\t\t(?!\(uuid)[^\n]*\n)*)'
               r'(\t\t\t\(uuid)', add_net, s)
    assert s.count('(net "N') == 16, s.count('(net "N')
    qfn = os.path.join(d, 'qfn.kicad_pcb')
    open(qfn, 'w', encoding='utf-8').write(s)
    open(os.path.join(d, 'qfn.kicad_dru'), 'w').write(
        '(version 1)\n(rule "pin_clearance"\n'
        '  (constraint clearance (min 0.28mm)))\n')
    return qfn


def test_rung_recovers_boxed_in_nets():
    d = tempfile.mkdtemp(prefix='t1763_')
    try:
        qfn = _make_fixture(d)
        out = os.path.join(d, 'out.kicad_pcb')
        js = os.path.join(d, 'out.json')
        r = subprocess.run(
            [sys.executable, '-B', '-X', 'utf8', tool('route.py'), qfn, out,
             '--nets', 'N*', '--track-width', '0.1', '--clearance', '0.28',
             '--grid-step', '0.1', '--via-size', '0.3', '--via-drill', '0.15',
             '--json-out', js],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', env=tool_env(), cwd=ROOT_DIR)
        log = r.stdout + r.stderr
        assert r.returncode == 0, log[-3000:]
        evidence(js)
        assert 'Finer-grid rung (#1763)' in log, log[-3000:]
        assert 'at grid 0.05' in log, log[-3000:]
        summ = json.load(open(js, encoding='utf-8'))
        failed = (set(summ.get('failed_single') or [])
                  | set(summ.get('open_single') or [])
                  | {m['net_name'] for m in summ.get('failed_multipoint') or []})
        # 4 open before the rung; it recovers at least 3 of them
        assert len(failed) <= 1, failed
        assert summ.get('min_clearance_used') == 0.28, summ.get(
            'min_clearance_used')
    finally:
        shutil.rmtree(d, ignore_errors=True)


if __name__ == '__main__':
    for fn in (test_plan_selects_boxed_in_only, test_plan_nothing_to_do,
               test_plan_respects_cell_budget,
               test_rung_recovers_boxed_in_nets):
        fn()
        print(f'PASS {fn.__name__}')
