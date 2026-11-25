#!/usr/bin/env python3
"""#2049 in the route tab: Via Cost Nets / Via Costs against the REAL headless
dialog.

  1. empty fields (a fresh dialog, and after reset): the route config carries
     no per-net via cost, so the engine runs as it did before the fields;
  2. filled fields reach the route config as route.py's lists;
  3. a count mismatch and a non-integer cost are refused before the run with
     the message route.py's own argument check prints for the same values;
  4. settings save and restore the fields, and settings saved before the
     fields existed load with them empty;
  5. a recorded route.py command with the flags converts to a plan step whose
     params fill the two fields;
  6. with both fields empty the route tab hands batch_route None for both, the
     engine's defaults (what it got before the fields existed).
A zero or negative cost is accepted on both fronts: route.py's flag is
type=int with no bound (3b).

Run: python3 tests/gui_parity/test_via_cost_nets_gui.py
(re-execs into KiCad's bundled python automatically, like its siblings)
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
from test_settings_roundtrip import KICAD_PYTHONS  # noqa: E402
del sys.path[0]
# route.py runs under the python that launched the gate (it has the router's
# dependencies); KiCad's bundled python runs the dialog.
HOST_PYTHON = os.environ.get('KRT_HOST_PYTHON') or sys.executable


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'], capture_output=True).returncode == 0:
            env = dict(os.environ, KRT_HOST_PYTHON=sys.executable)
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)] + sys.argv[1:],
                                    env=env).returncode)
    print("SKIP: no python with pcbnew+wx found")
    sys.exit(77)


fails = []


def check(name, cond, detail=""):
    print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


def cli_kwargs(board, nets, costs, dump):
    """route.py's exit code and the batch_route kwargs it dumps
    (KICAD_DUMP_BATCH_KWARGS: the engine returns before routing)."""
    env = dict(os.environ, KICAD_DUMP_BATCH_KWARGS=dump)
    env.pop('KICAD_DUMP_BATCH_KWARGS_CONTINUE', None)
    r = subprocess.run([HOST_PYTHON, '-X', 'utf8', os.path.join(REPO, 'py_router', 'route.py'),
                        board, os.devnull, '--nets', '*', '--via-cost-nets', *nets,
                        '--via-cost-nets-costs', *costs], capture_output=True, text=True,
                       timeout=600, env=env)
    import json
    with open(dump, encoding='utf-8') as fh:
        return r.returncode, json.load(fh)


def cli_error(board, nets, costs):
    """route.py's usage error for --via-cost-nets/--via-cost-nets-costs."""
    r = subprocess.run([HOST_PYTHON, '-X', 'utf8', os.path.join(REPO, 'py_router', 'route.py'),
                        board, os.devnull, '--nets', '*', '--via-cost-nets', *nets,
                        '--via-cost-nets-costs', *costs], capture_output=True, text=True, timeout=600)
    line = next((ln for ln in r.stderr.splitlines() if 'error: ' in ln), '')
    return r.returncode, line.split('error: ', 1)[-1]


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
    from kicad_routing_plugin.swig_gui import RoutingDialog, _split_net_list
    from kicad_routing_plugin.settings_persistence import (
        get_dialog_settings, restore_dialog_settings)
    from kicad_routing_plugin import ai_plan
    import manifest_to_plan as m2p

    dlg = RoutingDialog(None, parse_kicad_pcb(board), board)
    nets, layers = ['*'], ['F.Cu', 'B.Cu']

    print("1. empty fields")
    cfg = dlg._build_routing_config(nets, layers)
    check("fresh dialog: no per-net via cost in the route config",
          not cfg['via_cost_nets'] and not cfg['via_cost_nets_costs'],
          f"{cfg['via_cost_nets']!r} {cfg['via_cost_nets_costs']!r}")
    dlg.via_cost_nets_ctrl.SetValue("XTAL*")
    dlg.via_cost_nets_costs_ctrl.SetValue("40")
    dlg.reset_params_to_defaults()
    check("reset empties both fields",
          dlg.via_cost_nets_ctrl.GetValue() == "" and dlg.via_cost_nets_costs_ctrl.GetValue() == "")

    print("2. filled fields")
    dlg.via_cost_nets_ctrl.SetValue("XTAL* '/OSC IN'")
    dlg.via_cost_nets_costs_ctrl.SetValue("40 30")
    cfg = dlg._build_routing_config(nets, layers)
    check("patterns reach the config (a quoted name with a space stays whole)",
          cfg['via_cost_nets'] == ['XTAL*', '/OSC IN'], f"{cfg['via_cost_nets']!r}")
    check("costs reach the config as integers", cfg['via_cost_nets_costs'] == [40, 30],
          f"{cfg['via_cost_nets_costs']!r}")

    print("3. refused before the run, with route.py's message")
    for label, n, c in (("count mismatch", ["XTAL*", "OSC*"], ["40"]),
                        ("costs without patterns", [], ["40"]),
                        ("a cost that is not an integer", ["XTAL*"], ["4x"])):
        dlg.via_cost_nets_ctrl.SetValue(" ".join(n))
        dlg.via_cost_nets_costs_ctrl.SetValue(" ".join(c))
        try:
            dlg._via_cost_nets_fields()
            gui = None
        except ValueError as e:
            gui = str(e)
        rc, cli = cli_error(board, n, c)
        check(f"{label}: the GUI refuses", gui is not None, repr(gui))
        check(f"{label}: route.py refuses (exit 2)", rc == 2, f"exit {rc}")
        check(f"{label}: same message on both fronts", gui == cli, f"GUI {gui!r} / CLI {cli!r}")

    print("3b. a zero or negative cost is accepted, as route.py accepts it (type=int, no bound)")
    import tempfile
    td = tempfile.mkdtemp()
    for cost in ("0", "-5"):
        dlg.via_cost_nets_ctrl.SetValue("XTAL*")
        dlg.via_cost_nets_costs_ctrl.SetValue(cost)
        try:
            gui = dlg._via_cost_nets_fields()
        except ValueError as e:
            gui = str(e)
        check(f"cost {cost}: the GUI accepts it", gui == (['XTAL*'], [int(cost)]), repr(gui))
        rc, kw = cli_kwargs(board, ["XTAL*"], [cost], os.path.join(td, f"cli{cost}.json"))
        check(f"cost {cost}: route.py accepts it and hands it to the engine",
              rc == 0 and kw.get('via_cost_nets_costs') == [int(cost)],
              f"exit {rc}, {kw.get('via_cost_nets_costs')!r}")

    print("4. settings")
    dlg.via_cost_nets_ctrl.SetValue("XTAL*")
    dlg.via_cost_nets_costs_ctrl.SetValue("40")
    saved = get_dialog_settings(dlg)
    check("saved", saved.get('via_cost_nets') == "XTAL*" and saved.get('via_cost_nets_costs') == "40")
    dlg.reset_params_to_defaults()
    restore_dialog_settings(dlg, saved)
    check("restored", dlg.via_cost_nets_ctrl.GetValue() == "XTAL*"
          and dlg.via_cost_nets_costs_ctrl.GetValue() == "40")
    legacy = {k: v for k, v in saved.items() if not k.startswith('via_cost_nets')}
    dlg.reset_params_to_defaults()
    try:
        restore_dialog_settings(dlg, legacy)
        ok = True
    except Exception as e:                               # noqa: BLE001
        ok = False
        print(f"    {type(e).__name__}: {e}")
    check("settings from before the fields load, the fields stay empty",
          ok and dlg.via_cost_nets_ctrl.GetValue() == "" and dlg.via_cost_nets_costs_ctrl.GetValue() == "")

    print("5. recorded command -> plan step -> dialog")
    step = m2p.parse_command(['python3', 'route.py', 'a.kicad_pcb', 'b.kicad_pcb', '--nets', 'X*',
                              '--via-cost-nets', 'XTAL*', 'OSC*', '--via-cost-nets-costs', '40', '30'])
    params = (step or {}).get('params') or {}
    check("the step carries both lists",
          [str(v) for v in params.get('via_cost_nets', [])] == ['XTAL*', 'OSC*']
          and [int(v) for v in params.get('via_cost_nets_costs', [])] == [40, 30], f"{params!r}")
    dlg.reset_params_to_defaults()
    ai_plan.apply_step_params(step, dlg)
    check("the plan step fills the fields (patterns quoted as in Power Nets)",
          _split_net_list(dlg.via_cost_nets_ctrl.GetValue()) == ['XTAL*', 'OSC*']
          and dlg.via_cost_nets_costs_ctrl.GetValue() == "40 30",
          f"{dlg.via_cost_nets_ctrl.GetValue()!r} {dlg.via_cost_nets_costs_ctrl.GetValue()!r}")
    cfg = dlg._build_routing_config(nets, layers)
    check("... and reach the route config",
          cfg['via_cost_nets'] == ['XTAL*', 'OSC*'] and cfg['via_cost_nets_costs'] == [40, 30])

    print("6. empty fields: the route tab's batch_route call (KICAD_DUMP_BATCH_KWARGS)")
    import env_knobs
    dump = os.path.join(td, 'gui.json')
    saved_knobs = env_knobs.DUMP_BATCH_KWARGS, env_knobs.DUMP_BATCH_KWARGS_CONTINUE
    env_knobs.DUMP_BATCH_KWARGS, env_knobs.DUMP_BATCH_KWARGS_CONTINUE = dump, False
    try:
        dlg.reset_params_to_defaults()
        dlg._run_routing(dlg._build_routing_config(nets, layers))
    finally:
        env_knobs.DUMP_BATCH_KWARGS, env_knobs.DUMP_BATCH_KWARGS_CONTINUE = saved_knobs
    import json
    with open(dump, encoding='utf-8') as fh:
        kw = json.load(fh)
    check("empty fields: via_cost_nets and via_cost_nets_costs reach the engine as None "
          "(its defaults, as when the dialog did not pass them)",
          'via_cost_nets' in kw and kw['via_cost_nets'] is None and kw['via_cost_nets_costs'] is None,
          f"{kw.get('via_cost_nets', '<absent>')!r} {kw.get('via_cost_nets_costs', '<absent>')!r}")

    dlg.Destroy()
    print("OK" if not fails else f"FAILED: {len(fails)} check(s): {fails}")
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
