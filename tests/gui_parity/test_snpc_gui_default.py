#!/usr/bin/env python3
"""The dialog's via-in-pad policy follows the CLI default (SAME_NET_PAD_CLEARANCE_DEFAULT).

Every step tab passes `_same_net_pad_clearance_value()` to its engine and records
it in the board's project (`persist_live_tool_records`), where every later CLI
step reads it. So the dialog's default is the board's default from then on:

  1. a board that records nothing, opened as the plugin opens it (constructor +
     `_deferred_init`): 'Allow via-in-pad' unchecked, the default clearance;
  2. after 'reset to defaults' the same, and a run then records the default
     (it used to tick the box, record -1 and turn via-in-pad back on for every
     later CLI step on that board);
  3. a board that records -1 (built with filled vias): checked, -1, kept.

    python3 tests/gui_parity/test_snpc_gui_default.py
"""
import glob
import os
import shutil
import subprocess
import sys
import tempfile

# The real dialog builds about_tab, whose sizer flags trip a fatal assert on wx
# debug builds. Must be set before wx is imported.
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\\Program Files\\KiCad\\bin\\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand != sys.executable and os.path.exists(cand):
            if subprocess.run([cand, '-c', 'import wx, pcbnew'],
                              capture_output=True).returncode == 0:
                argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
                if os.name == 'nt':
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("SKIP: no python with wx + pcbnew found")
    sys.exit(0)


def _ok(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    return bool(cond)


def main():
    try:
        import wx, pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import wx
    import pcbnew

    for d in (REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_tools'),
              os.path.dirname(os.path.abspath(__file__))):
        sys.path.insert(0, d)
    import routing_defaults
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui
    from kicad_routing_plugin.gui_utils import persist_live_tool_records
    from protected_nets import (persist_same_net_pad_clearance, pro_path_for_board,
                                read_same_net_pad_clearance)

    want = routing_defaults.SAME_NET_PAD_CLEARANCE_DEFAULT
    td = tempfile.mkdtemp()
    path = os.path.join(td, 'b.kicad_pcb')
    shutil.copyfile(SRC, path)
    shutil.copyfile(os.path.splitext(SRC)[0] + '.kicad_pro', os.path.join(td, 'b.kicad_pro'))
    pro = pro_path_for_board(path)

    app = wx.App(False)  # noqa: F841 - must outlive the dialogs
    wx.Log.SetActiveTarget(wx.LogStderr())
    wx.MessageBox = lambda *a, **k: wx.OK
    board = pcbnew.LoadBoard(path)
    pcbnew.GetBoard = lambda: board

    def opened():
        d = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board), path)
        d._deferred_init()
        return d

    r = []
    print("\n[1] a board that records nothing, as the plugin opens it")
    d = opened()
    r.append(_ok("'Allow via-in-pad' unchecked", not d.via_in_pad_check.GetValue()))
    r.append(_ok(f"policy value {d._same_net_pad_clearance_value()!r} == {want!r}",
                 abs(d._same_net_pad_clearance_value() - want) < 1e-9))

    print("\n[2] reset to defaults, then a run")
    d.reset_params_to_defaults()
    v = d._same_net_pad_clearance_value()
    r.append(_ok("'Allow via-in-pad' unchecked", not d.via_in_pad_check.GetValue()))
    r.append(_ok(f"policy value {v!r} == {want!r}", abs(v - want) < 1e-9))
    persist_live_tool_records(board, v)
    got = read_same_net_pad_clearance(pro)
    r.append(_ok(f"a later CLI step reads {got!r} (want {want!r})", abs(got - want) < 1e-9))
    d.Destroy()

    print("\n[3] a board that records -1 (filled vias)")
    persist_same_net_pad_clearance(pro, -1.0)
    d = opened()
    r.append(_ok("'Allow via-in-pad' checked", d.via_in_pad_check.GetValue()))
    r.append(_ok(f"policy value {d._same_net_pad_clearance_value()!r} == -1.0",
                 d._same_net_pad_clearance_value() == -1.0))
    d.Destroy()

    ok = all(r)
    print(f"\n{'OK' if ok else 'FAILED'}: {sum(r)}/{len(r)} checks")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
