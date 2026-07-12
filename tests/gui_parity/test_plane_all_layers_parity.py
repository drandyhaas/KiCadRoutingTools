#!/usr/bin/env python3
"""Regression: the GUI plane CREATE step must pass the same all_layers the CLI
route_planes.py defaults to.

route_planes.py, when --layers is omitted, sets all_layers = ['F.Cu'] +
plane_layers + ['B.Cu'] (outer layers + the pour layers), NOT every copper
layer -- so plane-connection traces stay off the inner SIGNAL layers. The GUI's
_run_create_planes used to pass _get_all_copper_layers() (all 6 copper layers),
handing the router 2 extra inner layers and diverging from the CLI on the same
board. This pins the GUI to the CLI default.

Mocks create_plane to capture the all_layers kwarg -- no actual plane routing,
so it's fast. Needs wx (KiCad python). Skips if absent.
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
            if subprocess.run([cand, '-c', 'import wx'],
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
    import types
    from kicad_routing_plugin import planes_gui

    app = wx.App(False)

    # A shim carrying just what _run_create_planes reads before create_plane.
    class Shim:
        pass
    tab = Shim()
    tab.board_filename = "dummy.kicad_pcb"

    class BI:
        copper_layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'B.Cu']

    class PD:
        board_info = BI()
        nets = {}
    tab.pcb_data = PD()
    tab._get_all_copper_layers = types.MethodType(
        planes_gui.PlanesTab._get_all_copper_layers, tab)

    captured = {}

    class _Stop(BaseException):
        """BaseException so _run_create_planes' `except Exception` won't swallow
        it -- we only need the kwargs create_plane was handed, then bail out."""

    def fake_create_plane(*a, **k):
        captured['all_layers'] = k.get('all_layers')
        raise _Stop()

    import route_planes
    orig = route_planes.create_plane
    route_planes.create_plane = fake_create_plane
    # _run_create_planes imports create_plane via `from route_planes import
    # create_plane` at call time, so patch the source module.
    try:
        run = types.MethodType(planes_gui.PlanesTab._run_create_planes, tab)
        config = {
            'assignments': [(['GND', '+3V3'], ['In1.Cu', 'In4.Cu'])],
            'via_size': 0.45, 'via_drill': 0.2, 'clearance': 0.10,
            'track_width': 0.09, 'grid_step': 0.05,
            'hole_to_hole_clearance': 0.2, 'power_nets': ['VIN'],
            'power_nets_widths': [0.3],
        }
        try:
            run(config)
        except _Stop:
            pass
    finally:
        route_planes.create_plane = orig

    got = captured.get('all_layers')
    want = ['F.Cu', 'In1.Cu', 'In4.Cu', 'B.Cu']
    assert got == want, (
        f"GUI create passed all_layers={got}, expected the CLI default {want} "
        "(outer + pour layers, not all copper layers)")
    print(f"PASS: GUI create all_layers={got} matches the CLI route_planes default")


if __name__ == '__main__':
    main()
