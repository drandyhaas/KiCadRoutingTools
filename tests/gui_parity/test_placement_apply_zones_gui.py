#!/usr/bin/env python3
"""The AI Placement tab's Apply of a Place + Route result brings its zones.

A Place + Route run ends with a board that is placed, poured and routed.
Its Apply used to lay the run's tracks and vias, refill the live board's own
zones and import none of the result's ("new zones are NOT imported"). It now
goes through result_apply, as the Autorouter tab's does: poses and locks,
tracks and vias, the poured zones with their fills (not refilled), and the
result project's rules.

The gate builds a Place + Route-shaped result cheaply and for real: the
unrouted splitflap_driver is poured and routed by autoroute.py (a GND pour
on B.Cu), then place_pose turns C5 and locks it -- a placed, poured, routed
board whose parts the live board does not have where it has them. It loads
the unrouted board as the live board, drives the REAL PlacementTab's Apply
with that result pending in Place + Route mode (the confirmation answered
Yes), saves the live board as KiCad saves it, and grades it against the
result file with test_autoroute_tab_gui's comparison: check_connected and
check_drc, the project floors, the zones and their fill, the poses, tracks
and vias to the nanometre, the uuids and track order, and the fill after a
KiCad save and a refill.

Needs KiCad python (wx + pcbnew); re-execs into it, and exits 77 with a
SKIP: reason when there is none.

    python3 tests/gui_parity/test_placement_apply_zones_gui.py
"""
import glob
import os
import subprocess
import sys
import tempfile

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/"
    "Versions/Current/bin/python3",
    "/usr/bin/python3",
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]
BOARD = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
PART = 'C5'


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        r = subprocess.run([cand, '-c', 'import pcbnew, wx'], capture_output=True)
        if r.returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("SKIP: no python with pcbnew + wx found (this gate needs KiCad's)")
    sys.exit(77)


def _run(argv, what):
    r = subprocess.run(argv, capture_output=True, text=True, timeout=1800)
    if r.returncode != 0:
        raise AssertionError(f"BROKEN TEST: {what} failed (rc={r.returncode}):\n"
                             f"{r.stdout[-1500:]}{r.stderr[-1500:]}")
    return r


def run():
    import pcbnew
    import wx
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, HERE)
    from copy_board import copy_board
    from kicad_parser import build_pcb_data_from_board, parse_kicad_pcb
    import test_autoroute_tab_gui as T

    tmp = tempfile.mkdtemp(prefix='krt_placement_zones_')
    live_in = os.path.join(tmp, 'in', 'board.kicad_pcb')
    os.makedirs(os.path.dirname(live_in))
    copy_board(BOARD, live_in)

    # The Place + Route-shaped result: poured and routed, then a part turned
    # and locked.
    routed = os.path.join(tmp, 'run', 'routed.kicad_pcb')
    os.makedirs(os.path.dirname(routed))
    _run([sys.executable, '-X', 'utf8', os.path.join(REPO, 'py_router', 'autoroute.py'),
          live_in, routed], 'autoroute.py')
    result = os.path.join(tmp, 'run', 'final.kicad_pcb')
    _run([sys.executable, '-X', 'utf8', os.path.join(REPO, 'py_placer', 'place_pose.py'),
          routed, result, 'rotate', PART, '180', 'lock', PART,
          '--allow-routed', '--force'], 'place_pose.py')
    res = parse_kicad_pcb(result)
    before = parse_kicad_pcb(live_in)
    T.check("fixture: the result turned and locked the part, and has a zone",
            res.footprints[PART].rotation != before.footprints[PART].rotation
            and res.footprints[PART].locked and len(res.zones) > 0,
            f"{PART} {before.footprints[PART].rotation} -> {res.footprints[PART].rotation}, "
            f"locked {res.footprints[PART].locked}, zones {len(res.zones)}")

    app = wx.App(False)  # noqa: F841
    wx.Log.SetActiveTarget(wx.LogStderr())
    asked = []

    def _yes(msg, *a, **k):
        asked.append(msg)
        return wx.YES
    wx.MessageBox = _yes
    board = pcbnew.LoadBoard(live_in)
    pcbnew.GetBoard = lambda: board
    from kicad_routing_plugin.swig_gui import RoutingDialog
    dlg = RoutingDialog(None, build_pcb_data_from_board(board), live_in)
    try:
        tab = dlg.placement_tab
        tab._pending_result = {'board': result}
        tab._mode = 'place_route'
        T.check("the live board has no zone before Apply", len(list(board.Zones())) == 0)
        tab._on_apply(None)
        msg = asked[-1] if asked else ''
        T.check("Apply asked first, naming the zones, the copper and the moved part",
                'Zones:' in msg and 'Tracks and vias:' in msg and PART in msg, msg[:300])
        T.check("the old 'zones are NOT imported' warning is gone",
                'NOT imported' not in tab.status_text.GetLabel())
        T.check("Apply brought the result's zone", len(list(board.Zones())) == len(res.zones),
                f"{len(list(board.Zones()))} vs {len(res.zones)}")
        T.check(f"{PART} is locked on the live board, as the result has it",
                board.FindFootprintByReference(PART).IsLocked())
        T.compare_saved(board, result, os.path.join(tmp, 'gui', 'applied.kicad_pcb'))
    finally:
        dlg.Destroy()
        try:
            wx.GetApp().ProcessPendingEvents()
        except Exception:                                       # noqa: BLE001
            pass
    print(f"\nwork: {tmp}")
    if T.FAILS:
        print(f"VERDICT: FAIL ({len(T.FAILS)}): " + "; ".join(T.FAILS))
        return 1
    print("VERDICT: a Place + Route Apply lands its zones and grades as the result file")
    return 0


def main():
    try:
        import pcbnew  # noqa: F401
        import wx  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(run())


if __name__ == '__main__':
    main()
