#!/usr/bin/env python3
"""Regression: the GUI plane CREATE step must pass the same all_layers the CLI
route_planes.py defaults to.

route_planes.py, when --layers is omitted, sets all_layers = ['F.Cu'] +
plane_layers + ['B.Cu'] (outer layers + the pour layers), NOT every copper
layer -- so plane-connection traces stay off the inner SIGNAL layers. The GUI's
_run_create_planes used to pass _get_all_copper_layers() (all 6 copper layers),
handing the router 2 extra inner layers and diverging from the CLI on the same
board. This pins the GUI to the CLI default.

Drives the REAL PlanesTab on a REAL headless RoutingDialog (#493). It used to
bind _run_create_planes onto a hand-built `Shim` carrying just the few
attributes the method read at the time -- and then the method grew a
`self._cancel_requested` read, so this gate spent its life dying with
`AttributeError: 'Shim' object has no attribute '_cancel_requested'` instead of
checking anything. A mirrored interface has to be maintained in lockstep with
the real one; instantiating the real dialog costs a board load and cannot rot
that way.

create_plane is still mocked, so no plane is actually routed and this stays
fast. Needs KiCad python (wx + pcbnew); skips cleanly without it.

Run:  python3 tests/gui_parity/test_plane_all_layers_parity.py
"""

# ---------------------------------------------------------------------------
# macOS: if this HANGS at ~0% CPU, it is NOT wx, machine load, or a deadlock.
#
# After any wx process here is killed (a pkill, a timeout, a crash), macOS
# decides the app "quit unexpectedly", and the NEXT headless launch stops inside
# NSApplication bootstrap showing the restore-windows alert you cannot see:
#     -[NSPersistentUIRestorer promptToIgnorePersistentState]
#         -> -[NSAlert runModal]
# Headless, nobody can click it, so it waits forever: process state SN accruing
# ~0.3s of CPU over many minutes, which reads exactly like a hang. This cost a
# full session of ".gui-parity-checked" markers recording "wx blocked, gate NOT
# RUN" -- the gates were fine the whole time.
#
#   diagnose:  sample <pid> 3 -mayDie | grep -E "NSAlert|PersistentUI"
#   fix:       defaults write -g ApplePersistenceIgnoreState -bool YES
#
# A sandboxed HOME does NOT help -- cfprefsd serves that pref per-user
# regardless of HOME. With the default set, test_gui_engine_parity.py runs ~90s.
# ---------------------------------------------------------------------------
import glob
import os
import subprocess
import sys

# The real dialog builds about_tab, whose wxEXPAND|wxALIGN_* sizer flags trip a
# fatal assert on wx debug builds. Must be set before wx is imported.
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BOARD = os.path.join(REPO, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
# Every versioned install, newest first by NUMERIC version (a string sort
# puts KiCad\9.0 above KiCad\10.0).
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]    # this file orders its own sys.path further down
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
                    # os.execv re-splits argv on spaces on Windows, and the
                    # interpreter lives under "Program Files".
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("SKIP: no python with wx + pcbnew found")
    sys.exit(0)


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    import wx
    import pcbnew
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522

    if not os.path.exists(BOARD):
        print(f"SKIP: {os.path.relpath(BOARD, REPO)} not found")
        return 0

    app = wx.App(False)          # noqa: F841 - must outlive the dialog
    wx.MessageBox = lambda *a, **k: wx.OK

    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui

    board = pcbnew.LoadBoard(BOARD)
    pcbnew.GetBoard = lambda: board
    layers = build_pcb_data_from_board(board).board_info.copper_layers
    if len(layers) < 6:
        print(f"SKIP: need a 6-layer board, {os.path.basename(BOARD)} has {layers}")
        return 0

    dialog = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board), BOARD)
    tab = dialog.planes_tab

    captured = {}

    class _Stop(BaseException):
        """BaseException so _run_create_planes' `except Exception` won't swallow
        it -- we only need the kwargs create_plane was handed, then bail out."""

    def fake_create_plane(*a, **k):
        captured['all_layers'] = k.get('all_layers')
        raise _Stop()

    import route_planes
    orig = route_planes.create_plane
    # _run_create_planes imports create_plane via `from route_planes import
    # create_plane` at call time, so patch the source module.
    route_planes.create_plane = fake_create_plane
    try:
        config = {
            'assignments': [(['GND', '+3V3'], ['In1.Cu', 'In4.Cu'])],
            'via_size': 0.45, 'via_drill': 0.2, 'clearance': 0.10,
            'track_width': 0.09, 'grid_step': 0.05,
            'hole_to_hole_clearance': 0.2, 'power_nets': ['VIN'],
            'power_nets_widths': [0.3],
        }
        try:
            tab._run_create_planes(config)
        except _Stop:
            pass
    finally:
        route_planes.create_plane = orig

    got = captured.get('all_layers')
    want = ['F.Cu', 'In1.Cu', 'In4.Cu', 'B.Cu']
    if got is None:
        print("FAIL: create_plane was never called -- _run_create_planes bailed "
              "out before reaching the engine (check the log above)")
        return 1
    if got != want:
        print(f"FAIL: GUI create passed all_layers={got}, expected the CLI "
              f"default {want} (outer + pour layers, not all copper layers)")
        return 1
    print(f"PASS: GUI create all_layers={got} matches the CLI route_planes default")
    return 0


if __name__ == '__main__':
    sys.exit(main())
