#!/usr/bin/env python3
"""#1699: the cap pass's ceiling holds the physical fab floor in the GUI.

    python3 tests/gui_parity/test_1699_cap_ceiling_fab_floor_gui.py

Needs KiCad's python (re-execs into it); exits 2 without it. A plan sets the Min
Clearance spin without the event that pins a typed value, so a value below the
floor reached the cap pass and its class clamp as given. This drives an
`optimize_caps` step through the real dialog and plan executor with the arguments
of place_fanout_clearance.py, spies what the engine is handed, and asserts each
class the project ends up with equals the CLI's own, and that the run says it
pinned. Then the shared key itself, which the inline path reads too.

The wx-free half is tests/test_1699_cap_ceiling_fab_floor.py.

ipc-migration: the dialog runs over the fake-IPC board
(`tests/gui_parity/fake_ipc_board.py`, which `headless_plan.run_plan` installs).
kipy cannot write live design settings, so this front's class writeback is the
sibling .kicad_pro only (the tab's `gui_utils.apply_drc_settings_fix` and the
plan executor's end-of-plan `fix_project_for_output`). The SWIG gate also
asserted the live board's net settings; there is no live class store on this
front, so that arm is dropped and the project file is graded.
"""
import contextlib
import io
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
CLI = os.path.join(REPO, 'py_placer', 'place_fanout_clearance.py')
FLOOR = 0.1
PINNED = '--clearance 0.0889 is below the physical fab floor 0.1'
failures = []


def check(name, ok, detail=''):
    if not ok:
        failures.append(name)
    print(('  PASS ' if ok else '  FAIL ') + name + (f'  {detail}' if detail else ''))


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


def cli_arm(td, name, value):
    from copy_board import copy_board
    src = os.path.join(td, f'{name}_in.kicad_pcb')
    out = os.path.join(td, f'{name}_out.kicad_pcb')
    copy_board(BOARD, src)
    env = {k: v for k, v in os.environ.items() if not k.startswith('PYTHON')}
    r = subprocess.run([HOST_PYTHON, '-X', 'utf8', CLI, src, out, '--clearance', str(value)],
                       capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=REPO, env=env)
    if not os.path.isfile(os.path.splitext(out)[0] + '.kicad_pro'):
        raise AssertionError(f'BROKEN TEST: the CLI arm wrote no project: exit {r.returncode} '
                             f'{(r.stdout + r.stderr)[-600:]}')
    return project_classes(out), r.stdout + r.stderr


def plan_run(td, name, value):
    """One optimize_caps step through headless_plan, with a spy on the engine:
    (board, what the engine was handed, what it printed)."""
    from copy_board import copy_board
    from headless_plan import run_plan
    from placement import fanout_clearance as fc
    path = os.path.join(td, f'{name}.kicad_pcb')
    copy_board(BOARD, path)
    seen = {}
    real = fc.repair_fanout_clearance

    def spy(pcb_data, **kw):
        seen.update(kw)
        return real(pcb_data, **kw)

    fc.repair_fanout_clearance = spy
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            result = run_plan(path, [{'action': 'optimize_caps', 'params': {'clearance': value}}], timeout=900)
    finally:
        fc.repair_fanout_clearance = real
    output = buf.getvalue() + '\n'.join(str(x) for x in result.get('log') or ())
    statuses = [s.get('status') for s in result.get('steps') or ()]
    if statuses != ['done'] or not seen:
        raise AssertionError(f'BROKEN TEST: the step did not run: {statuses} {output[-800:]}')
    return path, seen, output

def shared_key(check_fn, td):
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
        for label, value, overrides, expected in (
                ('below the floor', 0.0889, '', FLOOR),
                ('above the floor', 0.15, '', 0.15),
                ('above the Default class: the raw value, not min(Default, value)', 0.3, '', 0.3),
                ('below a floor an overrides file lowered', 0.0889, fine, 0.0889)):
            dlg.reset_params_to_defaults()
            dlg.fab_overrides_path.SetValue(overrides)
            ai_plan.apply_step_params({'action': 'optimize_caps', 'params': {'clearance': value}}, dlg)
            got = dlg.fanout_tab.get_shared_params()['placement_clearance_ceiling']
            check_fn(f'{label}: {value} -> {expected}', abs(got - expected) < 1e-9, f'{got}')
        dlg.reset_params_to_defaults()
        dlg.fab_overrides_path.SetValue('')
        dlg.clearance_check.SetValue(False)
        check_fn('no override: no ceiling',
                 dlg.fanout_tab.get_shared_params()['placement_clearance_ceiling'] is None)
    finally:
        dlg.Destroy()


def main():
    try:
        import wx  # noqa: F401
        import kipy  # noqa: F401
    except ImportError:
        reexec_into_kicad()
    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

    td = tempfile.mkdtemp(prefix='krt1699_')
    try:
        print('A. an optimize_caps step, through the real dialog and executor')
        cli, cli_log = cli_arm(td, 'cli', 0.0889)
        check('precondition: the CLI holds both classes at the floor',
              cli == {'Default': FLOOR, 'Wide': FLOOR}, f'{cli}')
        check('precondition: the CLI says so', PINNED in cli_log)
        path, seen, output = plan_run(td, 'plan', 0.0889)
        check('the engine is handed the floor as its ceiling',
              abs((seen.get('netclass_ceiling') or 9) - FLOOR) < 1e-9, f"{seen.get('netclass_ceiling')}")
        # The SWIG gate also read the live board's net settings here. kipy has no
        # live class store or writer, so on this front the project below is the
        # only place the classes land.
        written = project_classes(path)
        check('the project the plan writes holds them there', written == cli, f'written {written} cli {cli}')
        check('the run says it pinned, as the CLI does', PINNED in output, output[-500:])

        print('B. a value above the floor, and above the Default class')
        cli, _ = cli_arm(td, 'cli2', 0.3)
        path, seen, _ = plan_run(td, 'plan2', 0.3)
        check('the engine is handed it as given', abs((seen.get('netclass_ceiling') or 9) - 0.3) < 1e-9,
              f"{seen.get('netclass_ceiling')}")
        check('the classes land where the CLI lands them',
              project_classes(path) == cli == {'Default': 0.2, 'Wide': 0.3},
              f'written {project_classes(path)} cli {cli}')

        print('C. the shared key every cap path reads')
        shared_key(check, td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print()
    if failures:
        print(f'FAILED: {len(failures)} check(s)')
        for f_ in failures:
            print(f'  {f_}')
        return 1
    print('PASS: #1699 the GUI holds the cap ceiling at the physical fab floor')
    return 0


if __name__ == '__main__':
    sys.exit(main())
