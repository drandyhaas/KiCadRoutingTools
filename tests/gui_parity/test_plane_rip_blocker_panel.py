#!/usr/bin/env python3
"""Regression: the Claude-plan executor must set rip_blocker_nets on the CORRECT
plane options panel per action.

`rip_blocker_check` exists on BOTH the Create and Repair plane options panels.
The generic param loop in apply_step_params resolves rip_blocker_nets ->
'rip_blocker_check' and picks the FIRST owner (create_options), so a
repair_planes step used to flip the CREATE panel's checkbox instead of the
Repair panel's. It was masked by an explicit per-action block that also set the
right panel, but the wrong-panel write was fragile and misleading; a reorder or
removal of the explicit block would silently drop --rip-blocker-nets on repairs.

Fix: rip_blocker_nets is skipped in the generic loop for both plane actions, so
only the explicit, panel-correct blocks handle it. This test pins that:
- repair_planes -> repair panel True, create panel untouched (False)
- route_planes  -> create panel True, repair panel untouched (False)

Needs wx (KiCad python). Skips if absent.
"""
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\\Program Files\\KiCad\\bin\\python.exe"),
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand != sys.executable and os.path.exists(cand):
            if subprocess.run([cand, '-c', 'import wx, pcbnew'],
                              capture_output=True).returncode == 0:
                os.execv(cand, [cand, os.path.abspath(__file__)] + sys.argv[1:])
    print("SKIP: no python with wx found")
    sys.exit(0)


def main():
    try:
        import wx  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    import wx
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'kicad_routing_plugin'))
    from kicad_routing_plugin.planes_gui import (CreatePlanesOptionsPanel,
                                                 RepairPlanesOptionsPanel)
    from kicad_routing_plugin import claude_plan

    app = wx.App(False)
    frame = wx.Frame(None)

    def mk():
        co = CreatePlanesOptionsPanel(frame)
        ro = RepairPlanesOptionsPanel(frame)

        class T:
            pass

        class D:
            pass
        t = T(); t.create_options = co; t.repair_options = ro
        d = D(); d.planes_tab = t
        d.via_size = wx.SpinCtrlDouble(frame)
        d.via_drill = wx.SpinCtrlDouble(frame)
        d.grid_step = wx.SpinCtrlDouble(frame)
        co.rip_blocker_check.SetValue(False)
        ro.rip_blocker_check.SetValue(False)
        return co, ro, d

    co, ro, d = mk()
    claude_plan.apply_step_params(
        {'action': 'repair_planes',
         'params': {'rip_blocker_nets': True, 'via_size': 0.25,
                    'via_drill': 0.15, 'grid_step': 0.05}}, d)
    assert ro.rip_blocker_check.GetValue() is True, \
        "repair_planes did not set the Repair panel's rip_blocker_check"
    assert co.rip_blocker_check.GetValue() is False, \
        "repair_planes wrongly set the Create panel's rip_blocker_check"

    co, ro, d = mk()
    claude_plan.apply_step_params(
        {'action': 'route_planes',
         'params': {'rip_blocker_nets': True, 'add_gnd_vias': False}}, d)
    assert co.rip_blocker_check.GetValue() is True, \
        "route_planes did not set the Create panel's rip_blocker_check"
    assert ro.rip_blocker_check.GetValue() is False, \
        "route_planes wrongly set the Repair panel's rip_blocker_check"

    print("PASS: rip_blocker_nets targets the correct plane options panel per action")


if __name__ == '__main__':
    main()
