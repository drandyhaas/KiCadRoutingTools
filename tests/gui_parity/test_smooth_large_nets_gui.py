#!/usr/bin/env python3
"""--smooth-large-nets in the route tab: the "Smooth large nets" checkbox
against the REAL headless dialog.

  1. a fresh dialog and a reset leave it off;
  2. settings save and restore it, and settings saved before the checkbox
     existed load with it off;
  3. a recorded route.py command with the flag converts to a plan step that
     ticks it, and the route config carries it;
  4. the route tab's batch_route call (KICAD_DUMP_BATCH_KWARGS) gets False
     unticked and True ticked.

Run: python3 tests/gui_parity/test_smooth_large_nets_gui.py
(re-execs into KiCad's bundled python automatically, like its siblings)
"""
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
from test_settings_roundtrip import KICAD_PYTHONS  # noqa: E402
del sys.path[0]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'], capture_output=True).returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)] + sys.argv[1:]).returncode)
    print("SKIP: no python with pcbnew+wx found")
    sys.exit(77)


fails = []


def check(name, cond, detail=""):
    print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))
    sys.path.insert(0, os.path.join(REPO, 'tests', 'stress'))
    sys.path.insert(0, os.path.dirname(REPO))

    app = wx.App(False)  # noqa: F841
    wx.Log.SetActiveTarget(wx.LogStderr())
    board = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.swig_gui import RoutingDialog
    from kicad_routing_plugin.settings_persistence import (
        get_dialog_settings, restore_dialog_settings)
    from kicad_routing_plugin import ai_plan
    import manifest_to_plan as m2p
    import env_knobs

    dlg = RoutingDialog(None, parse_kicad_pcb(board), board)
    nets, layers = ['*'], ['F.Cu', 'B.Cu']

    print("1. default and reset")
    check("fresh dialog: off", dlg.smooth_large_nets.GetValue() is False)
    check("fresh dialog: the route config carries False",
          dlg._build_routing_config(nets, layers).get('smooth_large_nets') is False)
    dlg.smooth_large_nets.SetValue(True)
    dlg.reset_params_to_defaults()
    check("reset turns it off", dlg.smooth_large_nets.GetValue() is False)

    print("2. settings")
    dlg.smooth_large_nets.SetValue(True)
    saved = get_dialog_settings(dlg)
    check("saved", saved.get('smooth_large_nets') is True)
    dlg.reset_params_to_defaults()
    restore_dialog_settings(dlg, saved)
    check("restored", dlg.smooth_large_nets.GetValue() is True)
    legacy = {k: v for k, v in saved.items() if k != 'smooth_large_nets'}
    dlg.reset_params_to_defaults()
    try:
        restore_dialog_settings(dlg, legacy)
        ok = True
    except Exception as e:                               # noqa: BLE001
        ok = False
        print(f"    {type(e).__name__}: {e}")
    check("settings from before the checkbox load, and it stays off",
          ok and dlg.smooth_large_nets.GetValue() is False)

    print("3. recorded command -> plan step -> checkbox")
    step = m2p.parse_command(['python3', 'route.py', 'a.kicad_pcb', 'b.kicad_pcb', '--nets', 'X*',
                              '--smooth-large-nets'])
    params = (step or {}).get('params') or {}
    check("the step carries smooth_large_nets", params.get('smooth_large_nets') is True, f"{params!r}")
    dlg.reset_params_to_defaults()
    ai_plan.apply_step_params(step, dlg)
    check("the plan step ticks the checkbox", dlg.smooth_large_nets.GetValue() is True)
    check("... and the route config carries True",
          dlg._build_routing_config(nets, layers).get('smooth_large_nets') is True)

    print("4. the route tab's batch_route call (KICAD_DUMP_BATCH_KWARGS)")
    td = tempfile.mkdtemp()
    saved_knobs = env_knobs.DUMP_BATCH_KWARGS, env_knobs.DUMP_BATCH_KWARGS_CONTINUE
    try:
        for ticked in (False, True):
            dump = os.path.join(td, f'gui_{ticked}.json')
            env_knobs.DUMP_BATCH_KWARGS, env_knobs.DUMP_BATCH_KWARGS_CONTINUE = dump, False
            dlg.reset_params_to_defaults()
            dlg.smooth_large_nets.SetValue(ticked)
            dlg._run_routing(dlg._build_routing_config(nets, layers))
            with open(dump, encoding='utf-8') as fh:
                kw = json.load(fh)
            check(f"{'ticked' if ticked else 'unticked'}: batch_route gets smooth_large_nets={ticked}",
                  kw.get('smooth_large_nets', '<absent>') is ticked,
                  f"{kw.get('smooth_large_nets', '<absent>')!r}")
    finally:
        env_knobs.DUMP_BATCH_KWARGS, env_knobs.DUMP_BATCH_KWARGS_CONTINUE = saved_knobs

    dlg.Destroy()
    print("OK" if not fails else f"FAILED: {len(fails)} check(s): {fails}")
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
