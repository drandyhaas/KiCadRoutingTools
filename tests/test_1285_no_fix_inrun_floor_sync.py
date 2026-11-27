#!/usr/bin/env python3
"""#1285: --no-fix-drc-settings leaves the output project as authored, the
in-run floor sync included.

route.py's #650 in-run floor sync lowers the output's sibling .kicad_pro to
the routed copper floors before the in-run audits grade the pours, so the
audit sees what main()'s writeback will ship. It ran on its own env knob
alone, so with the writeback turned off it still shipped a lowered project:
the reporter's board declared min_hole_clearance 0.25 over a 0.2 Default
class and came back at 0.2 under --no-fix-drc-settings. The sync now runs
only when the writeback will (batch_route's fix_drc_settings, which the CLI
sets from --no-fix-drc-settings and the GUI from its Fix DRC settings box).

The board: two pads joined by one net, and a project declaring
min_hole_clearance 0.25 above a 0.2 Default class and min_clearance.

Checks:
  1. --no-fix-drc-settings, the sync knob at its default: the net routes,
     the run says nothing about in-run floors, and the output .kicad_pro is
     the input's, every key
  2. control, the writeback on, routed at --clearance 0.15: the in-run
     sync fires and lowers min_clearance 0.2 -> 0.15 -- so check 1
     measured a writer that exists -- and (#1493) the copper-to-hole floor
     stays at the 0.25 the step held, since none of its copper needs less
  3. every apply_routed_floors call in route.py sits under a test of
     fix_drc_settings (the staged-board one runs only with
     KICAD_ORACLE_SUMMARY=1 and kicad-cli, so check 1 cannot reach it), and
     main() and the GUI both pass the kwarg

    python3 tests/test_1285_no_fix_inrun_floor_sync.py
"""
import ast
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool        # noqa: E402
from synth import RoutingBoard                                  # noqa: E402

FAILS = []
SYNC_LINE = 'In-run DRC floors (#650): set'


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def project():
    return {'board': {'design_settings': {'rules': {
                'min_clearance': 0.2, 'min_hole_clearance': 0.25,
                'min_track_width': 0.2, 'min_via_diameter': 0.5,
                'min_through_hole_diameter': 0.3, 'min_hole_to_hole': 0.25,
                'min_copper_edge_clearance': 0.3},
                'rule_severities': {}}},
            'meta': {'filename': 'in.kicad_pro', 'version': 3},
            'net_settings': {'classes': [{'name': 'Default', 'clearance': 0.2,
                                          'track_width': 0.2, 'via_diameter': 0.6,
                                          'via_drill': 0.3}],
                             'meta': {'version': 4}}}


def stage(d):
    b = RoutingBoard()
    b.pad('J1', 8, 15, 'CLK', sx=1.0, sy=1.0)
    b.pad('J2', 32, 15, 'CLK', sx=1.0, sy=1.0)
    pcb = os.path.join(d, 'in.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(b.text())
    with open(os.path.join(d, 'in.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump(project(), f, indent=2)
    return pcb


def route(inp, out, extra):
    env = dict(os.environ, KICAD_PLANE_FINALIZE='0')
    env.pop('KICAD_INRUN_FLOOR_SYNC', None)       # the default, as reported
    r = run_check([sys.executable, '-X', 'utf8', tool('route.py'), inp, out,
                   '--nets', 'CLK', '--escalation', 'off'] + list(extra),
                  accept=True, env=env)
    return (r.stdout or '') + (r.stderr or '')


def read_pro(pcb):
    with open(evidence(os.path.splitext(pcb)[0] + '.kicad_pro', 'project'),
              encoding='utf-8') as f:
        return json.load(f)


def routed_segments(pcb):
    with open(evidence(pcb, 'routed board'), encoding='utf-8') as f:
        return f.read().count('(segment')


def hole_clearance(proj):
    return proj['board']['design_settings']['rules'].get('min_hole_clearance')


def t_opt_out(tmp):
    d = os.path.join(tmp, 'optout')
    os.makedirs(d)
    inp = stage(d)
    out = os.path.join(d, 'out.kicad_pcb')
    log = route(inp, out, ['--no-fix-drc-settings'])
    check('opt-out: the net routed', routed_segments(out) > 0,
          f'{routed_segments(out)} segment(s)')
    check('opt-out: no in-run floor sync', SYNC_LINE not in log,
          next((l.strip() for l in log.splitlines() if SYNC_LINE in l), ''))
    got = read_pro(out)
    check('opt-out: min_hole_clearance kept at 0.25', hole_clearance(got) == 0.25,
          f'got {hole_clearance(got)}')
    check('opt-out: the output project is the input project, every key',
          got == read_pro(inp))


def t_writeback_on(tmp):
    d = os.path.join(tmp, 'fix')
    os.makedirs(d)
    inp = stage(d)
    out = os.path.join(d, 'out.kicad_pcb')
    log = route(inp, out, ['--clearance', '0.15'])
    sync = [l.strip() for l in log.splitlines() if SYNC_LINE in l]
    check('control: the in-run sync fires with the writeback on', bool(sync),
          sync[0] if sync else 'no sync line')
    check('control: it lowers min_clearance on this board',
          'rules.min_clearance: 0.2 -> 0.15' in log)
    check('control: the shipped hole floor is the 0.25 held (#1493)',
          hole_clearance(read_pro(out)) == 0.25,
          f'got {hole_clearance(read_pro(out))}')


def _guarded(tree, call_name, flag):
    """[(lineno, guarded)] for every call to `call_name`: whether an
    enclosing `if` tests the name `flag`."""
    parents = {}
    for node in ast.walk(tree):
        for child in ast.iter_child_nodes(node):
            parents[child] = node
    out = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call)
                and getattr(node.func, 'id', None) == call_name):
            continue
        guarded, p = False, parents.get(node)
        while p is not None:
            if isinstance(p, ast.If) and any(
                    isinstance(n, ast.Name) and n.id == flag
                    for n in ast.walk(p.test)):
                guarded = True
                break
            p = parents.get(p)
        out.append((node.lineno, guarded))
    return out


def t_static():
    with open(tool('route.py'), encoding='utf-8') as f:
        src = f.read()
    tree = ast.parse(src)
    calls = _guarded(tree, 'apply_routed_floors', 'fix_drc_settings')
    check('route.py: both in-run sync calls found', len(calls) == 2,
          f'{len(calls)} call(s)')
    for line, ok in calls:
        check(f'route.py:{line}: apply_routed_floors is gated on fix_drc_settings', ok)
    check('route.py main(): passes fix_drc_settings from --no-fix-drc-settings',
          'fix_drc_settings=not args.no_fix_drc_settings' in src)
    with open(os.path.join(ROOT, 'kicad_routing_plugin', 'swig_gui.py'),
              encoding='utf-8') as f:
        gui = f.read()
    check('swig_gui: the batch_route call passes the Fix DRC settings box',
          "fix_drc_settings=config.get('fix_drc_settings', True)" in gui)


def main():
    print('#1285: --no-fix-drc-settings and the in-run floor sync')
    tmp = tempfile.mkdtemp(prefix='t1285_')
    try:
        t_opt_out(tmp)
        t_writeback_on(tmp)
        t_static()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f'\nFAILED: {len(FAILS)} check(s): {FAILS}')
        return 1
    print('\nAll checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
