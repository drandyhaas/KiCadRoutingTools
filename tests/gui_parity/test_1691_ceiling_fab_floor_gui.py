#!/usr/bin/env python3
"""#1691: the GUI holds a Clearance ceiling at the physical fab floor.

    python3 tests/gui_parity/test_1691_ceiling_fab_floor_gui.py

Needs KiCad's python (re-execs into it); exits 2 without it. A plan sets the Min
Clearance spin without the event that pins a typed value, so a ceiling below the
floor reached the writeback as given: the map, built from the floored effective
clearance, was right and the class written back was not; a plan's end-of-run
project write took the Default class and the board minimum from the plan's own
number in the same way. On a 2-layer board with
a wider second class, this drives a route step through the real dialog and plan
executor with the CLI's arguments, and asserts each class the plan's project
write ends up with (a ceiling-only plan, and one that also carries `clearance`)
equals the CLI's own, and that the run says it pinned the ceiling. The dialog's
tab configs carry the floored value, which is what the differential tab (through
the route config), planes and fanout tabs write back.

The wx-free half is tests/test_1691_ceiling_physical_floor.py.

ipc-migration: the dialog runs over the fake-IPC board
(`tests/gui_parity/fake_ipc_board.py`, which `headless_plan.run_plan` installs).
kipy cannot write live design settings, so this front's class writeback is the
sibling .kicad_pro -- each tab's `gui_utils.apply_drc_settings_fix` and the plan
executor's end-of-plan `fix_project_for_output`. Where the SWIG gate read the
live board's net settings, this one reads that project file; a ceiling-only plan
now writes it too (#1698), so both runs are graded there.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
for sub in ('py_router', 'py_tools', 'py_placer'):
    sys.path.insert(0, os.path.join(REPO, sub))
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

BOARD = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
HOST_PYTHON = os.environ.get('KRT_HOST_PYTHON', sys.executable)
CEILING = 0.0889
FLOOR = 0.1
ARGS = ('--track-width', '0.1', '--via-size', '0.25', '--via-drill', '0.15',
        '--clearance-ceiling', str(CEILING))
NETS = ['VCC']


def reexec_into_kicad():
    from kicad_locate import kicad_python_candidates
    for py in kicad_python_candidates():
        if py == sys.executable or not os.path.exists(py):
            continue
        if subprocess.run([py, '-c', 'import wx, kipy'], capture_output=True).returncode == 0:
            env = dict(os.environ, KRT_HOST_PYTHON=sys.executable)
            sys.exit(subprocess.run([py, '-X', 'utf8', os.path.abspath(__file__)],
                                    env=env).returncode)
    print('NOT RUN: no python with wx + kipy found')
    sys.exit(2)


def project_classes(board):
    with open(os.path.splitext(board)[0] + '.kicad_pro', encoding='utf-8') as f:
        return {c['name']: c['clearance'] for c in json.load(f)['net_settings']['classes']}


def cli_arm(td):
    from copy_board import copy_board
    src = os.path.join(td, 'cli_in.kicad_pcb')
    out = os.path.join(td, 'cli_out.kicad_pcb')
    copy_board(BOARD, src)
    env = {k: v for k, v in os.environ.items() if not k.startswith('PYTHON')}
    r = subprocess.run([HOST_PYTHON, '-X', 'utf8', os.path.join(REPO, 'py_router', 'route.py'),
                        src, out, '--nets', *NETS, *ARGS],
                       capture_output=True, text=True, encoding='utf-8', errors='replace',
                       cwd=REPO, env=env)
    if not os.path.isfile(os.path.splitext(out)[0] + '.kicad_pro'):
        raise AssertionError(f'BROKEN TEST: the CLI arm wrote no project: exit '
                             f'{r.returncode} {(r.stdout + r.stderr)[-600:]}')
    return project_classes(out)


def plan_run(td, name, params):
    """One route step through headless_plan: (the classes its project holds,
    what it printed). The project is this front's only class store (kipy has no
    live net-settings write), so it stands where the SWIG gate read the board."""
    import contextlib
    import io
    from copy_board import copy_board
    from headless_plan import run_plan
    path = os.path.join(td, f'{name}.kicad_pcb')
    copy_board(BOARD, path)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        result = run_plan(path, [{'action': 'route', 'nets': NETS, 'params': params}], timeout=900)
    output = buf.getvalue() + '\n'.join(str(x) for x in result.get('log') or ())
    statuses = [s.get('status') for s in result.get('steps') or ()]
    if statuses != ['done']:
        raise AssertionError(f'BROKEN TEST: the step did not run: {statuses} {output[-800:]}')
    return project_classes(path), output


def main():
    try:
        import wx  # noqa: F401
        import kipy  # noqa: F401
    except ImportError:
        reexec_into_kicad()
    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

    failures = []

    def check(name, ok, detail=''):
        if not ok:
            failures.append(name)
        print(('  PASS ' if ok else '  FAIL ') + name + (f'  {detail}' if detail else ''))

    td = tempfile.mkdtemp(prefix='krt1691_')
    try:
        print('A. a route step, through the real dialog and executor')
        cli = cli_arm(td)
        check('precondition: the CLI held Wide at the floor',
              abs(cli.get('Wide', 9) - FLOOR) < 1e-9, f'{cli}')
        common = {'track_width': 0.1, 'via_size': 0.25, 'via_drill': 0.15}
        only, output = plan_run(td, 'ceiling', {'clearance_ceiling': CEILING, **common})
        check('the project a ceiling-only plan writes holds Wide at the floor, as the CLI',
              abs(only.get('Wide', 9) - cli['Wide']) < 1e-9, f'plan {only} cli {cli}')
        check('the project a ceiling-only plan writes holds Default at the floor, as the CLI',
              abs(only.get('Default', 9) - cli['Default']) < 1e-9, f'plan {only} cli {cli}')
        check('the run says it pinned the ceiling',
              f'--clearance-ceiling {CEILING} is below the physical fab floor {FLOOR}' in output,
              output[-400:])
        # A plan's end-of-run project write runs when a step carries `clearance`.
        path = os.path.join(td, 'both.kicad_pcb')
        plan_run(td, 'both', {'clearance': CEILING, 'clearance_ceiling': CEILING, **common})
        written = project_classes(path)
        check('the project the plan writes holds Wide at the floor, as the CLI',
              abs(written.get('Wide', 9) - cli['Wide']) < 1e-9, f'written {written} cli {cli}')
        check('the project the plan writes holds Default at the floor, as the CLI',
              abs(written.get('Default', 9) - cli['Default']) < 1e-9, f'written {written} cli {cli}')
        with open(os.path.splitext(path)[0] + '.kicad_pro', encoding='utf-8') as f:
            rules = json.load(f)['board']['design_settings']['rules']
        check('...and the board minimum', abs(rules.get('min_clearance', 9) - FLOOR) < 1e-9,
              f"{rules.get('min_clearance')}")
        print('B. every tab config carries the floored ceiling')
        tab_configs(check, td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print()
    if failures:
        print(f'FAILED: {len(failures)} check(s)')
        return 1
    print('PASS: #1691 the GUI holds the ceiling at the physical fab floor')
    return 0


def tab_configs(check, td):
    import wx
    from copy_board import copy_board
    from fake_ipc_board import install as install_fake_board, build_pcb_data_like_ipc
    from kicad_routing_plugin import ai_plan, routing_dialog

    path = os.path.join(td, 'tabs.kicad_pcb')
    copy_board(BOARD, path)
    app = wx.App.Get() or wx.App(False)
    wx.Log.SetActiveTarget(wx.LogStderr())
    assert app is not None
    install_fake_board(path)
    fine = os.path.join(td, 'fine.fab')
    with open(fine, 'w', encoding='utf-8') as f:
        f.write('clearance = 0.08\n')
    dlg = routing_dialog.RoutingDialog(None, build_pcb_data_like_ipc(path), path)
    try:
        for label, ceiling, overrides, expected in (
                ('below the floor', CEILING, '', FLOOR),
                ('above the floor', 0.15, '', 0.15),
                ('below a floor an overrides file lowered', CEILING, fine, CEILING)):
            dlg.reset_params_to_defaults()
            dlg.fab_overrides_path.SetValue(overrides)
            ai_plan.apply_step_params(
                {'action': 'route', 'params': {'clearance_ceiling': ceiling}}, dlg)
            got = {
                'route': dlg._build_routing_config([], ['F.Cu', 'B.Cu'])['clearance_ceiling'],
                'planes': dlg.planes_tab.get_shared_params()['clearance_ceiling'],
                'fanout': dlg.fanout_tab.get_shared_params()['clearance_ceiling'],
            }
            check(f'{label}: {ceiling} -> {expected}',
                  all(abs(v - expected) < 1e-9 for v in got.values()), f'{got}')
        dlg.reset_params_to_defaults()
        dlg.fab_overrides_path.SetValue('')
        check('no ceiling box: no ceiling',
              dlg.fanout_tab.get_shared_params()['clearance_ceiling'] is None)
    finally:
        dlg.Destroy()


if __name__ == '__main__':
    sys.exit(main())
