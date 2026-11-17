#!/usr/bin/env python3
"""autoroute.py's bus step: what it plans, how it decides, what follows it.

1. The plan, read off in-repo boards with no routing: a board with no ball
   array plans no bus step and its analysis carries no bus record; a board
   with a ball array and no bus records why each pair was not taken and plans
   no bus step; a board with a bus plans one `bus` row after the planes and
   before every fanout, at the fine via when one of its arrays is fine-pitch,
   and the fanouts of the arrays it serves become optional rows.
2. Without ortools (a stub package that will not import, first on the path,
   in a child) the plan still lists the bus row, says it will not run and how
   to install the library, and the arrays' fanouts are ordinary rows again.
3. `bus_step` with a scripted chain: not run without ortools; a failed step,
   a step that laid no copper on its arrays, and one whose board ranks below
   the board before it are not kept and the chain stays on that board; a step
   that ranks no lower is kept. The command is the plan skill's: --rounds 2,
   --inproc, the chain's clearance ceiling and track, the bus's via.
4. `run_plan` with a scripted chain: after a kept bus step no fanout runs on
   an array it served, every other fanout excludes the nets it handed on
   protected, and a pair it handed on is not given to route_diff; after a
   failed bus step every array is fanned out as usual.
5. `bus_handoff` on real board files: an array with a new track end on one of
   its pads is served, one without is not, and the nets protected after and
   not before are the ones handed on.
6. `bus_grade_fields` / `step_summary('bus', ...)` read route_bus's summary.
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import autoroute as AP                                        # noqa: E402
from run_utils import evidence                               # noqa: E402

SCRIPT = os.path.join(ROOT, 'py_router', 'autoroute.py')
NO_ARRAY = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
ARRAY_NO_BUS = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
BUS = os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
FAILS = []


def expect(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILS.append(what)


def _plan(board, env=None):
    """(AUTOROUTE_PLAN payload, the run's prose lines) of a --dry-run."""
    with tempfile.TemporaryDirectory() as tmp:
        r = subprocess.run([sys.executable, '-X', 'utf8', SCRIPT, board,
                            os.path.join(tmp, 'out.kicad_pcb'), '--dry-run'],
                           capture_output=True, text=True, timeout=600, env=env)
    expect(r.returncode == 0 and 'Traceback' not in r.stdout + r.stderr,
           f"--dry-run on {os.path.basename(board)} exits 0 (rc={r.returncode})")
    plan = next((json.loads(ln[len('AUTOROUTE_PLAN: '):]) for ln in r.stdout.splitlines()
                 if ln.startswith('AUTOROUTE_PLAN: ')), None)
    prose = [ln for ln in r.stdout.splitlines() if not ln.startswith('AUTOROUTE_')]
    return plan or {}, prose


def test_plan():
    print("the plan")
    p, _ = _plan(NO_ARRAY)
    expect('bus' not in p and all(r['tool'] != 'bus' for r in p.get('steps', [])),
           "no ball array: no bus record, no bus step")
    p, _ = _plan(ARRAY_NO_BUS)
    expect(p.get('bus', {}).get('buses') == [] and p['bus'].get('not_taken')
           and all(b.get('why') for b in p['bus']['not_taken']),
           "a ball array and no bus: the pairs not taken, each with why")
    expect(all(r['tool'] != 'bus' for r in p.get('steps', [])), "...and no bus step")
    p, _ = _plan(BUS)
    bus = p.get('bus', {})
    tags = [r['tag'] for r in p.get('steps', [])]
    expect([(b['src'], b['dest']) for b in bus.get('buses', [])] == [('U3', 'U4')]
           and bus.get('arrays') == ['U3', 'U4'],
           f"a bus board: the bus U3 -> U4, serving both arrays ({bus.get('buses')})")
    expect(tags[:2] == ['planes', 'bus'] and all(
        tags.index('bus') < i for i, t in enumerate(tags) if t.startswith('fanout_')),
        f"the bus step after the planes, before every fanout ({tags})")
    expect((bus.get('via'), bus.get('drill')) == (p['rules']['fine_via'], p['rules']['fine_drill']),
           "the fine via: one of its arrays is at 0.65 mm or finer")
    row = {r['tag']: r for r in p['steps']}
    expect('EXPERIMENTAL' in row['bus']['why'], "its row says it is experimental")
    expect(row['fanout_U3']['optional'] and row['fanout_U4']['optional']
           and not row['fanout_U1']['optional'],
           "the served arrays' fanouts are optional rows, the others are not")


def test_no_ortools():
    print("without ortools")
    with tempfile.TemporaryDirectory() as stub:
        os.makedirs(os.path.join(stub, 'ortools'))
        with open(os.path.join(stub, 'ortools', '__init__.py'), 'w') as fh:
            fh.write("raise ImportError('No module named ortools (hidden by the test)')\n")
        env = dict(os.environ, PYTHONPATH=stub + os.pathsep + os.environ.get('PYTHONPATH', ''))
        p, prose = _plan(BUS, env)
    row = {r['tag']: r for r in p.get('steps', [])}
    missing = (p.get('bus') or {}).get('missing') or ''
    expect('ortools' in missing and 'pip install' in missing,
           f"the plan's bus record names ortools and how to install it ({missing!r})")
    expect('NOT RUN' in row.get('bus', {}).get('why', '') and 'pip install' in row['bus']['why'],
           "the bus row says it will not run, and how to install the library")
    expect(not row.get('fanout_U3', {}).get('optional', True),
           "the arrays' fanouts are ordinary rows")
    expect(any('ortools' in ln for ln in prose), "the run says so in its prose too")


class Scripted(AP.ChainEvents):
    """Chain.run / grade / drc_nets / check_drc replaced by a script: a tag in
    FAIL produces no board, a fanout escapes 10 balls, GRADES maps a board to
    its grade."""

    def __init__(self, fail=(), grades=None):
        self.cur = 'in'
        self.steps = []
        self.calls = []
        self.fail = set(fail)
        self.grades = grades or {}
        self.last_iterations = 100

    def run(self, kind, args, tag, parent=None, hold=False):
        self.end_step()
        self.calls.append((kind, tag, list(args)))
        out = None if tag in self.fail else f'{tag}.kicad_pcb'
        s = {'escaped': 10, 'failed': 0} if kind in ('bga', 'qfn') else {}
        self.steps.append({'tag': tag, 'ok': bool(out), 'result': 'ran'})
        if hold and out:
            self.held = {'tag': tag, 'result': 'ran', '_index': len(self.steps) - 1}
        return out, s, {}

    def accept(self, out):
        if out:
            self.cur = out

    def grade(self, board):
        return self.grades.get(board, {'open': 0, 'drc': 0, 'open_nets': []})

    def drc_nets(self, board):
        return set()

    def check_drc(self, board):
        return {'violations': 0, 'items': []}


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def _bus_a(**bus):
    return dict(clearance=0.09, track=0.1, bus=dict(
        {'buses': [{'src': 'U3', 'dest': 'U4', 'nets': 23, 'why': ''}], 'arrays': ['U3', 'U4'],
         'via': 0.25, 'drill': 0.15, 'missing': None, 'error': None}, **bus))


def test_bus_step():
    print("bus_step with a scripted chain")
    saved = AP.bus_handoff
    try:
        ch = Scripted()
        h = _quiet(AP.bus_step, ch, _bus_a(missing='the bus step needs ortools: not installed'))
        expect(h is None and not ch.calls and 'ortools' in ch.skipped.get('bus', ''),
               "without ortools: not run, skipped with why")

        ch = Scripted(fail={'bus'})
        h = _quiet(AP.bus_step, ch, _bus_a())
        expect(h is None and ch.cur == 'in' and ch.decisions[-1]['kept'] is False,
               "a failed step: not kept, the chain stays on the board before it")
        args = ch.calls[0][2]
        want = ['--rounds', '2', '--inproc', '--clearance-ceiling', '0.09', '--track-width', '0.1',
                '--fanout-track-width', '0.1', '--via-size', '0.25', '--via-drill', '0.15']
        expect(args[:2] == ['{IN}', '{OUT}'] and args[2:] == want,
               f"the command is the plan skill's bus step ({args})")

        AP.bus_handoff = lambda before, after, arrays: {'served': [], 'nets': [], 'names': []}
        ch = Scripted()
        h = _quiet(AP.bus_step, ch, _bus_a())
        expect(h is None and ch.cur == 'in', "no copper on its arrays: not kept")

        AP.bus_handoff = lambda before, after, arrays: {'served': ['U3', 'U4'], 'nets': ['D0'],
                                                        'names': ['D0']}
        ch = Scripted(grades={'in': {'open': 40, 'drc': 0}, 'bus.kicad_pcb': {'open': 41, 'drc': 0}})
        h = _quiet(AP.bus_step, ch, _bus_a())
        expect(h is None and ch.cur == 'in', "a board ranking below the one before it: not kept")
        ch = Scripted(grades={'in': {'open': 40, 'drc': 0}, 'bus.kicad_pcb': {'open': 20, 'drc': 1}})
        h = _quiet(AP.bus_step, ch, _bus_a())
        expect(h and h['served'] == ['U3', 'U4'] and ch.cur == 'bus.kicad_pcb'
               and ch.decisions[-1]['kept'] is True,
               "fewer open nets, if with a violation more: kept (completion first)")
    finally:
        AP.bus_handoff = saved


def test_run_plan():
    print("run_plan after the bus step, with a scripted chain")
    a = _quiet(AP.analyze, BUS)
    a['planes'] = []
    pair = a['pair_nets'][:2]
    saved = (AP.Chain, AP.bus_handoff, AP.route_passes)
    made = []

    def chain(*args, **kw):
        made.append(Scripted(fail=made_fail))
        return made[-1]
    try:
        AP.Chain = chain
        AP.route_passes = lambda ch, *a_, **k_: (ch, ch.cur, None)
        names = sorted(set(a['globs']) | {'RAM_D13'})
        AP.bus_handoff = lambda before, after, arrays: {
            'served': ['U3', 'U4'], 'nets': ['RAM_D13'] + pair, 'names': names}
        made_fail = set()
        _quiet(AP.run_plan, 'in', 'out', a, '.', effort_factor=2.0)
        ch = made[-1]
        tags = [t for _, t, _ in ch.calls]
        expect('bus' in tags and not any(t.startswith(('fanout_U3', 'fanout_U4')) for t in tags),
               f"no fanout of a served array ({tags})")
        expect('served' in ch.skipped.get('fanout_U3', ''), "...each skipped with why")
        fan = next(args for kind, t, args in ch.calls if t == 'fanout_U1')
        expect('!' + AP._glob('RAM_D13') in fan, "another fanout excludes a net the bus handed on")
        diff = next((args for kind, t, args in ch.calls if t == 'diffpairs'), [])
        sel = diff[diff.index('--nets') + 1:diff.index('--track-width')] if diff else []
        expect(sel and not {a['globs'][n] for n in pair} & set(sel)
               and len(sel) == len(a['pair_nets']) - 2,
               f"route_diff is not given the pair the bus handed on ({len(sel)} nets)")

        made_fail = {'bus'}
        _quiet(AP.run_plan, 'in', 'out', a, '.', effort_factor=2.0)
        tags = [t for _, t, _ in made[-1].calls]
        expect('fanout_U3' in tags and 'fanout_U4' in tags,
               f"after a failed bus step every array is fanned out ({tags})")
    finally:
        AP.Chain, AP.bus_handoff, AP.route_passes = saved


def test_handoff():
    print("bus_handoff on board files")
    from kicad_parser import parse_kicad_pcb
    from protected_nets import persist_protected_nets
    with tempfile.TemporaryDirectory() as tmp:
        before, after = os.path.join(tmp, 'before.kicad_pcb'), os.path.join(tmp, 'after.kicad_pcb')
        shutil.copyfile(BUS, before)
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(before)
        pad = next(p for p in pcb.footprints['U3'].pads if p.net_name and 'GND' not in p.net_name)
        text = open(before, encoding='utf-8').read().rstrip()
        assert text.endswith(')')
        seg = (f'\t(segment (start {pad.global_x:.4f} {pad.global_y:.4f}) '
               f'(end {pad.global_x + 0.5:.4f} {pad.global_y:.4f}) (width 0.1) (layer "F.Cu") '
               f'(net "{pad.net_name}") (uuid "00000000-0000-0000-0000-000000001234"))\n')
        open(after, 'w', encoding='utf-8').write(text[:-1] + seg + ')\n')
        evidence(after, 'board with a track end on a U3 pad')
        pro = after[:-len('.kicad_pcb')] + '.kicad_pro'
        with open(pro, 'w', encoding='utf-8') as fh:
            fh.write('{}\n')            # the records go only into an existing project
        persist_protected_nets(pro, {pad.net_name: 'bus'}, verbose=False)
        h = _quiet(AP.bus_handoff, before, after, ['U3', 'U4'])
    expect(h['served'] == ['U3'], f"the array with a new track end on a pad is served ({h['served']})")
    expect(h['nets'] == [pad.net_name], f"the nets protected after and not before ({h['nets']})")


def test_summary():
    print("route_bus's summary read")
    line = ('BUS U3->U4 K=45 round=1 lanes=45/45 vias=95 copper=984mm connected=0 drc=0 | on the '
            'board: vias=94 copper=974mm connected=1 drc=0 (bus 0, board 0 vs 0 in) planes=107 vs '
            '105 in walled=2 secs=855')
    g = AP.bus_grade_fields(line)
    expect(g == {'lanes': '45/45', 'vias': '94', 'copper': '974mm', 'connected': '1', 'drc': '0',
                 'walled': '2'}, f"the BUS line's fields, as graded on the board ({g})")
    d, res = AP.step_summary('bus', '', {'exit': 0, 'buses': [
        {'src': 'U3', 'dest': 'U4', 'exit': 0, 'grade': line}]}, None)
    expect(re.match(r'1/1 bus\(es\) handed on routed, U3->U4 lanes 45/45, 2 ball', res),
           f"its result line ({res})")
    expect(AP.step_summary('bus', '', None, None)[1] == 'no bus summary written',
           "no summary: said so")


def main():
    test_plan()
    test_no_ortools()
    test_bus_step()
    test_run_plan()
    test_handoff()
    test_summary()
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS: autoroute bus step")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
