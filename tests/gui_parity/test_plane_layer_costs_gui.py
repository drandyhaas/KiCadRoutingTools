#!/usr/bin/env python3
"""Regression: the GUI plane CREATE step hands create_plane one layer cost per
entry of ITS layer list, placed by name (found reviewing #1269).

The Planes tab routes on F.Cu + its pour layers + B.Cu (route_planes.py's
--layers default, test_plane_all_layers_parity.py), but it took its layer
costs from the Basic tab aligned to the SELECTED layers -- by default every
copper layer. create_plane reads them positionally against its own list and
then appends the unrequested copper layers at FORBIDDEN cost after them, so on
a 6-layer board the default six values priced the wrong layers and the -1
entries fell off the end: the unrequested inner layers were routable.

Drives the REAL PlanesTab on a REAL headless RoutingDialog with the shared
parameters the tab really merges in (get_shared_params), create_plane mocked to
capture its kwargs:
  1. Layer Costs typed one per copper layer (six distinct values) -> the
     engine gets exactly the values of F.Cu, the pour layers and B.Cu, in
     create_plane's layer order.
  2. Control: the route tab still hands batch_route one value per selected
     layer for the same field.
  3. Values written per selected layer, with a pour layer left unselected ->
     the tab refuses before the engine, naming the layer.

Needs KiCad python (wx + pcbnew); re-execs into it, skips cleanly without it.

Run:  python3 tests/gui_parity/test_plane_layer_costs_gui.py
"""
import glob
import os
import subprocess
import sys

# The real dialog builds about_tab, whose wxEXPAND|wxALIGN_* sizer flags trip a
# fatal assert on wx debug builds. Must be set before wx is imported.
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BOARD = os.path.join(REPO, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
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


failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + str(detail)) if detail else ''}")
    if not ok:
        failures.append(label)


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
    copper = list(build_pcb_data_from_board(board).board_info.copper_layers)
    if len(copper) < 6:
        print(f"SKIP: need a 6-layer board, {os.path.basename(BOARD)} has {copper}")
        return 0

    dialog = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board), BOARD)
    tab = dialog.planes_tab
    pours = ['In1.Cu', 'In4.Cu']
    assert all(p in copper for p in pours), copper
    want_layers = ['F.Cu'] + pours + ['B.Cu']

    class _Stop(BaseException):
        """BaseException so _run_create_planes' `except Exception` won't swallow
        it -- only the kwargs create_plane was handed are needed."""

    def run(costs_text, selected):
        for layer, cb in dialog.layer_checks.items():
            cb.SetValue(layer in selected)
        dialog.layer_costs_ctrl.SetValue(costs_text)
        captured = {}

        def fake_create_plane(*a, **k):
            captured.update(k)
            raise _Stop()

        import route_planes
        orig = route_planes.create_plane
        route_planes.create_plane = fake_create_plane
        try:
            config = {
                'assignments': [(['GND', '+3V3'], pours)],
                'via_size': 0.45, 'via_drill': 0.2, 'clearance': 0.10,
                'track_width': 0.09, 'grid_step': 0.05,
                'hole_to_hole_clearance': 0.2,
            }
            config.update(tab.get_shared_params())
            try:
                tab._run_create_planes(config)
            except _Stop:
                pass
        finally:
            route_planes.create_plane = orig
        return captured

    print("1. one value per copper layer, placed on the planes step's layers")
    typed = [float(i + 1) for i in range(len(copper))]
    got = run(' '.join(f"{c:g}" for c in typed), copper)
    if not got:
        check("create_plane was called", False,
              "_run_create_planes bailed out before reaching the engine")
    else:
        want = [typed[copper.index(l)] for l in want_layers]
        check("all_layers is the route_planes default",
              got.get('all_layers') == want_layers, got.get('all_layers'))
        check("layer_costs are those layers' values, by name",
              got.get('layer_costs') == want,
              f"got {got.get('layer_costs')} for {got.get('all_layers')}, want {want}")

    print("2. control: the route tab still gets one value per selected layer")
    for layer, cb in dialog.layer_checks.items():
        cb.SetValue(layer in ('F.Cu', 'B.Cu'))
    sel = dialog._selected_layer_costs()
    check("copper-ordered field -> the selected layers' values",
          sel == [typed[copper.index('F.Cu')], typed[copper.index('B.Cu')]], sel)

    print("3. values per selected layer cannot price an unselected pour layer")
    try:
        got = run('1 5', ['F.Cu', 'B.Cu'])
        check("refused before the engine", False,
              f"create_plane got layer_costs={got.get('layer_costs')} "
              f"for all_layers={got.get('all_layers')}")
    except ValueError as e:
        check("refused before the engine, naming the pour layers",
              all(p in str(e) for p in pours), e)

    dialog.Destroy()
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
