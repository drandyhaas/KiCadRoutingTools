#!/usr/bin/env python3
"""CLI/GUI parity for a BGA fanout inside a rule area that relaxes a
class-pair rule (bga_fanout/area_view.py).

The fanout reads the room from the board's .kicad_dru and its rule areas:
`PCBData.source_path` finds the rules file and `board_info.keepouts` the area.
The GUI builds its PCBData from the live board (`build_pcb_data_from_board`),
so this gate checks the room reaches the engine on that path too. On the IPC
branch the live board is tests/gui_parity/fake_ipc_board.py's, whose PCBData
is the file's: the tab -> engine half is driven, the kipy read is not.

Fixture: tests/fixture_rule_area_bga.py, 'room' (a 0.5 mm class-pair rule,
then a 0.1 mm rule in a rule area around the BGA). With the room read, every
ball escapes; without it, 42 of 64 do.

What it pins, driving the REAL FanoutTab on a REAL headless RoutingDialog
(the Basic tab's track, clearance and via set to the CLI's values) against
the CLI's text-parsed engine call with the tab's OWN kwargs:

  1. both fronts read the room over U1's ball field (a spy on
     `area_view.part_view`);
  2. the GUI front escapes all 64 balls;
  3. the emitted copper (tracks, vias, failed nets) is IDENTICAL across the
     two fronts.

Needs KiCad python (wx + pcbnew); skips cleanly without it. Two 64-ball
fanouts, well under a minute.

Run:  python3 tests/gui_parity/test_bga_fanout_rule_area_gui.py
"""
# macOS: a ~0% CPU hang here is the restore-windows alert, not wx --
# `defaults write -g ApplePersistenceIgnoreState -bool YES` (see
# test_fanout_rotated_gui.py's header for the diagnosis).
import glob
import os
import subprocess
import sys
import tempfile

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REF = 'U1'
SHARED = {'track_width': 0.1, 'clearance': 0.1, 'via_size': 0.3, 'via_drill': 0.15,
          'layers': ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']}
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
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("SKIP: no python with wx + pcbnew found")
    sys.exit(77)


def _ok(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    return bool(cond)


def _track_key(tracks):
    return sorted((round(t['start'][0], 6), round(t['start'][1], 6),
                   round(t['end'][0], 6), round(t['end'][1], 6),
                   t['net_id'], t['layer'], round(t['width'], 6))
                  for t in tracks)


def _via_key(vias):
    return sorted((round(v['x'], 6), round(v['y'], 6), v['net_id'],
                   tuple(v.get('layers') or ()), round(v.get('size', 0), 6))
                  for v in vias)


class _Spy:
    """Records what `area_view.part_view` answers for the part."""
    def __init__(self):
        import bga_fanout.area_view as av
        self.av, self.real, self.found = av, av.part_view, []

    def __enter__(self):
        def wrapped(pcb_data, footprint):
            got = self.real(pcb_data, footprint)
            self.found.append(got is not None)
            return got
        self.av.part_view = wrapped
        return self

    def __exit__(self, *exc):
        self.av.part_view = self.real


def run_gui(pcbnew, board_path):
    import bga_fanout
    # routing_dialog.py is this branch's swig_gui.py (renamed by the IPC port).
    from kicad_routing_plugin import routing_dialog as swig_gui
    # build_pcb_data_from_board takes a KIPY board on this branch, so the live
    # board is the fake-IPC harness the other ported gates use. It serves
    # PCBData from the file, as a text parse: on this front the gate pins the
    # tab -> engine half; the kipy builder's own source_path / keepouts read
    # (`_ipc_board_path`, extract_keepouts) is not driven here.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from fake_ipc_board import (install as _install_fake_board,
                                build_pcb_data_like_ipc)

    _install_fake_board(board_path)
    pcb_data = build_pcb_data_like_ipc(board_path)
    dialog = swig_gui.RoutingDialog(None, pcb_data, board_path)

    real_engine = bga_fanout.generate_bga_fanout
    seen, captured = {}, {}

    def _spy(footprint, pcb, **kwargs):
        seen['kwargs'] = dict(kwargs)
        return real_engine(footprint, pcb, **kwargs)

    def _capture(tracks, vias, failed_nets=None, **kw):
        captured['tracks'] = tracks
        captured['vias'] = vias
        captured['failed'] = failed_nets or []

    try:
        bga_fanout.generate_bga_fanout = _spy
        tab = dialog.fanout_tab
        tab._apply_fanout_results = _capture
        base_shared = tab.get_shared_params
        tab.get_shared_params = lambda: dict(base_shared() if base_shared else {}, **SHARED)
        fp = pcb_data.footprints[REF]
        nets = sorted({p.net_name for p in fp.pads if p.net_id and p.net_name})
        with _Spy() as views:
            tab._run_bga_fanout(fp, nets, tab.bga_options.get_config())
            # the tab runs on a worker thread: a real MainLoop (wx_pump.py)
            from wx_pump import run_until
            run_until(lambda: not getattr(tab, '_running', False), 600)
        kwargs = seen.get('kwargs')
        if kwargs is not None:
            kwargs.pop('progress_callback', None)
            kwargs.pop('cancel_check', None)
        return ({'tracks': captured.get('tracks') or [], 'vias': captured.get('vias') or [],
                 'failed': captured.get('failed') or [], 'views': views.found},
                kwargs)
    finally:
        bga_fanout.generate_bga_fanout = real_engine
        dialog.Destroy()


def run_cli(board_path, kwargs):
    from kicad_parser import parse_kicad_pcb
    from bga_fanout import generate_bga_fanout
    pcb = parse_kicad_pcb(board_path)
    fp = pcb.footprints[REF]
    with _Spy() as views:
        tracks, vias, _rm, failed = generate_bga_fanout(fp, pcb, **kwargs)
    return {'tracks': tracks, 'vias': vias, 'failed': failed, 'views': views.found}


def main():
    try:
        import wx, pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import wx
    import pcbnew

    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))
    sys.path.insert(0, os.path.join(REPO, 'tests'))
    sys.path.insert(0, os.path.join(REPO, 'tests', 'gui_parity'))
    import fixture_rule_area_bga as fx

    app = wx.App(False)  # noqa: F841
    wx.Log.SetActiveTarget(wx.LogStderr())
    wx.MessageBox = lambda *a, **k: wx.OK
    r = []
    with tempfile.TemporaryDirectory() as td:
        board_path = fx.write(td, 'room', fx.WIDE + fx.ROOM)
        print(f"\nGUI front: real FanoutTab on {REF}")
        gui, kwargs = run_gui(pcbnew, board_path)
        r.append(_ok("captured the tab's engine kwargs (the CLI leg replays these)",
                     bool(kwargs)))
        if not kwargs:
            print("\nThe tab never reached generate_bga_fanout -- nothing to compare")
            return 1
        print(f"\nCLI front: text-parsed PCBData, replaying {len(kwargs)} kwargs")
        cli = run_cli(board_path, kwargs)

        print("\n[1] both fronts read the room")
        r.append(_ok(f"GUI front read it ({gui['views']})", any(gui['views'])))
        r.append(_ok(f"CLI front read it ({cli['views']})", any(cli['views'])))

        print("\n[2] the GUI front escapes every ball")
        r.append(_ok(f"failed nets on the GUI front: {gui['failed']}",
                     not gui['failed'] and len(gui['tracks']) > 0))

        print("\n[3] emitted copper parity")
        print(f"      CLI: {len(cli['tracks'])} tracks, {len(cli['vias'])} vias, {len(cli['failed'])} failed")
        print(f"      GUI: {len(gui['tracks'])} tracks, {len(gui['vias'])} vias, {len(gui['failed'])} failed")
        r.append(_ok("track sets identical across fronts",
                     _track_key(cli['tracks']) == _track_key(gui['tracks'])))
        r.append(_ok("via sets identical across fronts",
                     _via_key(cli['vias']) == _via_key(gui['vias'])))
        r.append(_ok("failed-net lists identical across fronts",
                     sorted(cli['failed']) == sorted(gui['failed'])))

    passed = sum(r)
    print(f"\n{passed}/{len(r)} rule-area BGA fanout parity checks passed")
    print("=" * 60)
    return 0 if passed == len(r) else 1


if __name__ == "__main__":
    sys.exit(main())
