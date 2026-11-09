#!/usr/bin/env python3
"""#1691: --clearance-ceiling holds the physical fab floor, for every net class.

The Default class was pinned up to the floor with the run's clearance, while the
ceiling itself was not, so on a board with a wider second class the map and the
project writeback lowered that class below what the fab can make: a 2-layer board
with Default 0.2 / Wide 0.4 under --clearance-ceiling 0.0889 wrote Default 0.1 and
Wide 0.0889. The ceiling now goes through the same pin as the clearance.

  A. the pin: the floor the Default class gets, by layer count and overrides
     file, not moved by the tier.
  B. every routing main hands its ceiling to that pin.
  C. the routed runs: the written project and the log.
  D. what the pin leaves alone: a ceiling above the floor, a --fab-overrides
     floor below the ceiling, a class the board declares below the floor, and
     an explicit --net-clearances map.

    python3 tests/test_1691_ceiling_physical_floor.py
"""
import ast
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

import fab_tiers  # noqa: E402

BOARD = os.path.join('kicad_files', 'flat_hierarchy.kicad_pcb')
MAINS = ('route.py', 'route_diff.py', 'route_planes.py', 'repair_planes.py',
         os.path.join('qfn_fanout', '__init__.py'), os.path.join('bga_fanout', '__init__.py'))
failures = []


def check(label, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'} {label}{('  ' + str(detail)) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def pin(*a, **k):
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        out = fab_tiers.enforce_fab_floors(*a, **k)
    return out, buf.getvalue()


def the_pin():
    print('A. the pin')
    got, said = pin(2, clearance_ceiling=0.0889)
    check('2 layers: pinned at the floor the Default class gets',
          got == {'clearance_ceiling': pin(2, clearance=0.0889)[0]['clearance']} and
          got['clearance_ceiling'] == 0.1, got)
    check('it says so', '--clearance-ceiling 0.0889 is below the physical fab floor 0.1' in said, said)
    check('4 layers: its own floor', pin(4, clearance_ceiling=0.08)[0] == {'clearance_ceiling': 0.09})
    check('at the floor: not pinned', pin(2, clearance_ceiling=0.1)[0] == {})
    check('above the floor: not pinned', pin(2, clearance_ceiling=0.15)[0] == {})
    check('omitted: not pinned', pin(2, clearance_ceiling=None)[0] == {})
    check('an overrides file moves the floor',
          pin(2, overrides={'clearance': 0.08}, clearance_ceiling=0.0889)[0] == {})
    for tier in ('standard', 'advanced', 'auto'):
        check(f'the {tier} tier does not move it',
              pin(2, tier=tier, overrides={}, clearance_ceiling=0.0889)[0] == {'clearance_ceiling': 0.1})


def the_mains():
    print('B. every routing main hands its ceiling to the pin')
    for rel in MAINS:
        path = run_utils.evidence(os.path.join(REPO, 'py_router', rel))
        with open(path, encoding='utf-8') as f:
            tree = ast.parse(f.read())
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and getattr(n.func, 'id', getattr(n.func, 'attr', '')) == 'enforce_fab_floors']
        check(f'{rel}: enforce_fab_floors is called once, with clearance_ceiling',
              len(calls) == 1 and any(k.arg == 'clearance_ceiling' for k in calls[0].keywords),
              f'{len(calls)} call(s)')


def stage(td, name, wide=None, default=None):
    d = os.path.join(td, name)
    os.makedirs(d)
    src = os.path.join(d, 'in.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'copy_board.py'), BOARD, src],
                   capture_output=True, cwd=REPO)
    run_utils.evidence(src)
    pro = os.path.splitext(src)[0] + '.kicad_pro'
    with open(pro, encoding='utf-8') as f:
        doc = json.load(f)
    for c in doc['net_settings']['classes']:
        if c['name'] == 'Wide' and wide is not None:
            c['clearance'] = wide
        if c['name'] == 'Default' and default is not None:
            c['clearance'] = default
    with open(pro, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=1)
    return src


def classes(out):
    with open(run_utils.evidence(os.path.splitext(out)[0] + '.kicad_pro'), encoding='utf-8') as f:
        return {c['name']: c['clearance'] for c in json.load(f)['net_settings']['classes']}


def route(src, *flags, tool='route.py', nets=('VCC',)):
    out = os.path.join(os.path.dirname(src), 'out.kicad_pcb')
    argv = [sys.executable, '-X', 'utf8', os.path.join('py_router', tool), src, out, '--nets', *nets,
            '--via-size', '0.25', '--via-drill', '0.15', *flags]
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=REPO)
    if not os.path.isfile(os.path.splitext(out)[0] + '.kicad_pro'):
        raise AssertionError(f'BROKEN TEST: {tool} wrote no project: exit {r.returncode} '
                             f'{(r.stdout + r.stderr)[-800:]}')
    return classes(out), r.stdout + r.stderr


def the_runs(td):
    print('C. the routed runs')
    got, log = route(stage(td, 'route'), '--track-width', '0.1', '--clearance-ceiling', '0.0889')
    check('route.py: precondition, the board has a wider class', got.get('Wide', 0) > 0, got)
    check('route.py: Wide written at the floor', got.get('Wide') == 0.1, got)
    check('route.py: Default written at the floor', got.get('Default') == 0.1, got)
    check('route.py: the log says the ceiling was pinned',
          '--clearance-ceiling 0.0889 is below the physical fab floor 0.1' in log)
    check('route.py: the map is capped at the pinned value',
          'capped at --clearance-ceiling 0.1 (mm: [0.1])' in log)
    check('route.py: the class disclosure names the value it routes at',
          'LOWERS NET CLASS Wide 4.0x -- 0.4 -> 0.1 mm' in log)
    for tier in ('advanced', 'standard'):
        got, _ = route(stage(td, f'tier_{tier}'), '--track-width', '0.1', '--clearance-ceiling', '0.0889',
                       '--fab-tier', tier)
        check(f'route.py --fab-tier {tier}: the same floor', got.get('Wide') == 0.1, got)
    out = os.path.join(td, 'pour.kicad_pcb')
    src = stage(td, 'pour')
    r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'route_planes.py'), src, out,
                        '--nets', 'GND', '--plane-layers', 'B.Cu', '--clearance-ceiling', '0.0889'],
                       capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=REPO)
    got = classes(out)
    check('route_planes.py: Wide written at the floor', got.get('Wide') == 0.1, got)
    check('route_planes.py: the log says the ceiling was pinned',
          '--clearance-ceiling 0.0889 is below the physical fab floor 0.1' in r.stdout)


def left_alone(td):
    print('D. what the pin leaves alone')
    got, log = route(stage(td, 'above'), '--track-width', '0.1', '--clearance-ceiling', '0.15')
    check('a ceiling above the floor: Wide at the ceiling', got.get('Wide') == 0.15, got)
    check('...and no pin line', '--clearance-ceiling 0.15 is below' not in log)

    fab = os.path.join(td, 'fine.fab')
    with open(fab, 'w', encoding='utf-8') as f:
        f.write('clearance = 0.08\n')
    got, log = route(stage(td, 'overrides'), '--track-width', '0.1', '--clearance-ceiling', '0.0889',
                     '--fab-overrides', fab)
    check('an overrides file below the ceiling: Wide at the ceiling', got.get('Wide') == 0.0889, got)
    check('...and no pin line', '--clearance-ceiling 0.0889 is below' not in log)

    got, _ = route(stage(td, 'declared', wide=0.05), '--track-width', '0.1', '--clearance-ceiling', '0.0889')
    check('a class the board declares below the floor stays as declared', got.get('Wide') == 0.05, got)

    got, _ = route(stage(td, 'declared_default', default=0.05), '--track-width', '0.1',
                   '--clearance-ceiling', '0.0889')
    check('as the Default class does', got.get('Default') == 0.05, got)

    nc = os.path.join(td, 'nc.json')
    with open(nc, 'w', encoding='utf-8') as f:
        json.dump({'VCC': 0.3, 'VPP': 0.05}, f)
    got, log = route(stage(td, 'netclr'), '--track-width', '0.1', '--clearance-ceiling', '0.0889',
                     '--net-clearances', nc)
    check('--net-clearances: read as given, no cap on its map',
          'Loaded per-net clearances for 2/' in log and 'capped at --clearance-ceiling' not in log, log[-300:])
    check('...and the class still lands at the pinned ceiling', got.get('Wide') == 0.1, got)


if __name__ == '__main__':
    td = tempfile.mkdtemp(prefix='krt1691_')
    try:
        the_pin()
        the_mains()
        the_runs(td)
        left_alone(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print()
    if failures:
        print(f'FAILED: {len(failures)} check(s)')
        for f_ in failures:
            print(f'  {f_}')
        sys.exit(1)
    print('PASS: #1691 the ceiling holds the physical fab floor for every class')
