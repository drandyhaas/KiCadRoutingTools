#!/usr/bin/env python3
"""CLI/GUI parity for BGA fanout on a BACK-SIDE part (bus622-take5's
`bga_fanout/flip_frame.py`, 2026-09-19).

A BGA on B.Cu now fans out as the mirror of the same BGA on F.Cu: the engine
turns the whole board over in memory, runs on the part now on the front, and
mirrors the copper back. That transform re-derives every pad's LOCAL
coordinates under the parser's convention -- and pad locals are the one
PCBData field the GUI front COMPUTES (`build_pcb_data_from_board`) rather than
reads, which is exactly how `_global_to_local`'s transposed rotation once
survived every gate (see test_fanout_rotated_gui.py). The rotated gate pins a
rotated QFN; nothing pinned a flipped BGA until this file.

Fixture: kicad_files/glasgow_revC.kicad_pcb with U30 (BGA-121, -90 deg)
FLIPPED to the back through pcbnew's own FOOTPRINT.Flip and saved to a temp
dir with its .kicad_pro -- a legitimate back-side BGA on a real 4-layer board,
built the way the #714 gate builds its reference, so no in-repo fixture and
no awx tool is needed.

What it pins, driving the REAL FanoutTab on a REAL headless RoutingDialog
against the CLI's text-parsed engine call with the tab's OWN kwargs:

  1. both parse paths see U30 on B.Cu at the same pads (the premise);
  2. the mirror frame is actually TAKEN on both fronts (a spy on
     flip_frame.to_front_frame) -- the change detector: if the engine ever
     stops flipping back-side parts, this says so instead of passing;
  3. the emitted copper (tracks, vias, failed nets) is IDENTICAL across
     the two fronts;
  4. every SURFACE escape (a track leaving a ball with no via at the ball)
     lies on the part's own face, B.Cu, on both fronts.

Needs KiCad python (wx + pcbnew); skips cleanly without it. Costs two
BGA-121 fanouts, well under a minute.

Run:  python3 tests/gui_parity/test_fanout_backside_gui.py
"""
# macOS: a ~0% CPU hang here is the restore-windows alert, not wx --
# `defaults write -g ApplePersistenceIgnoreState -bool YES` (see the
# rotated gate's header for the diagnosis).
import math
import glob
import os
import shutil
import subprocess
import sys
import tempfile

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(REPO, 'kicad_files', 'glasgow_revC.kicad_pcb')
REF = 'U30'
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


def _surface_escapes_off_face(tracks, vias, footprint):
    """(surface escapes, those NOT on the part's layer). A surface escape is
    a track with an endpoint at a ball that carries no via."""
    via_at = {(round(v['x'], 3), round(v['y'], 3)) for v in vias}
    on = off = 0
    for p in footprint.pads:
        if not p.net_id:
            continue
        key = (round(p.global_x, 3), round(p.global_y, 3))
        if key in via_at:
            continue
        for t in tracks:
            if t['net_id'] != p.net_id:
                continue
            for e in (t['start'], t['end']):
                if math.hypot(e[0] - p.global_x, e[1] - p.global_y) < 1e-3:
                    if t['layer'] == footprint.layer:
                        on += 1
                    else:
                        off += 1
                    break
    return on, off


def make_fixture(pcbnew, tmpdir):
    """glasgow with U30 turned over in place, through pcbnew's own flip."""
    board = pcbnew.LoadBoard(SRC)
    fp = board.FindFootprintByReference(REF)
    fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
    dst = os.path.join(tmpdir, 'glasgow_U30_back.kicad_pcb')
    pcbnew.SaveBoard(dst, board)
    pro = os.path.splitext(SRC)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(dst)[0] + '.kicad_pro')
    return dst


class _Spy:
    """Counts the calls a front makes to flip_frame.to_front_frame."""
    def __init__(self):
        import bga_fanout.flip_frame as ff
        self.ff, self.real, self.calls = ff, ff.to_front_frame, 0

    def __enter__(self):
        def wrapped(*a, **k):
            self.calls += 1
            return self.real(*a, **k)
        self.ff.to_front_frame = wrapped
        return self

    def __exit__(self, *exc):
        self.ff.to_front_frame = self.real


def run_gui(pcbnew, board_path):
    import bga_fanout
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui

    board = pcbnew.LoadBoard(board_path)
    pcbnew.GetBoard = lambda: board
    pcb_data = build_pcb_data_from_board(board)
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
        fp = pcb_data.footprints[REF]
        nets = sorted({p.net_name for p in fp.pads if p.net_id and p.net_name})
        with _Spy() as flips:
            tab._run_bga_fanout(fp, nets, tab.bga_options.get_config())
            # #621: the tab runs on a worker thread. A real MainLoop, not a
            # Yield loop -- Yield never fires the wx.CallLater that collects
            # the result on Windows (see wx_pump.py).
            from wx_pump import run_until
            run_until(lambda: not getattr(tab, '_running', False), 600)
        kwargs = seen.get('kwargs')
        if kwargs is not None:
            kwargs.pop('progress_callback', None)
            kwargs.pop('cancel_check', None)
        return ({'footprint': fp, 'tracks': captured.get('tracks') or [],
                 'vias': captured.get('vias') or [],
                 'failed': captured.get('failed') or [], 'flips': flips.calls},
                nets, kwargs)
    finally:
        bga_fanout.generate_bga_fanout = real_engine
        dialog.Destroy()


def run_cli(board_path, kwargs):
    from kicad_parser import parse_kicad_pcb
    from bga_fanout import generate_bga_fanout
    pcb = parse_kicad_pcb(board_path)
    fp = pcb.footprints[REF]
    with _Spy() as flips:
        tracks, vias, _rm, failed = generate_bga_fanout(fp, pcb, **kwargs)
    return {'footprint': fp, 'tracks': tracks, 'vias': vias, 'failed': failed,
            'flips': flips.calls}


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
    if not os.path.exists(SRC):
        print(f"SKIP: {os.path.relpath(SRC, REPO)} not found")
        return 0

    app = wx.App(False)  # noqa: F841
    wx.MessageBox = lambda *a, **k: wx.OK
    r = []
    with tempfile.TemporaryDirectory() as td:
        board_path = make_fixture(pcbnew, td)
        print(f"\nfixture: {os.path.basename(SRC)} with {REF} flipped to the back "
              f"-> {os.path.basename(board_path)}")

        print(f"\nGUI front: real FanoutTab on {REF}")
        gui, nets, kwargs = run_gui(pcbnew, board_path)
        r.append(_ok("captured the tab's engine kwargs (the CLI leg replays these)",
                     bool(kwargs)))
        if not kwargs:
            print("\nThe tab never reached generate_bga_fanout -- nothing to compare")
            return 1
        print(f"\nCLI front: text-parsed PCBData, same {len(nets)} nets, "
              f"replaying {len(kwargs)} kwargs from the tab")
        cli = run_cli(board_path, kwargs)

        print("\n[1] premise")
        r.append(_ok(f"{REF} is on B.Cu on both fronts "
                     f"(GUI {gui['footprint'].layer}, CLI {cli['footprint'].layer})",
                     gui['footprint'].layer == 'B.Cu' == cli['footprint'].layer))
        g = sorted(gui['footprint'].pads, key=lambda p: p.pad_number)
        c = sorted(cli['footprint'].pads, key=lambda p: p.pad_number)
        dloc = max(abs(a.local_x - b.local_x) + abs(a.local_y - b.local_y)
                   for a, b in zip(g, c)) if g and c else 1e9
        r.append(_ok(f"pad locals agree across fronts to 1e-6 (max {dloc:.2e})",
                     len(g) == len(c) and dloc < 1e-6))

        print("\n[2] the mirror frame is taken on both fronts")
        r.append(_ok(f"GUI front flipped the board ({gui['flips']} call(s))", gui['flips'] >= 1))
        r.append(_ok(f"CLI front flipped the board ({cli['flips']} call(s))", cli['flips'] >= 1))

        print("\n[3] emitted copper parity")
        print(f"      CLI: {len(cli['tracks'])} tracks, {len(cli['vias'])} vias, {len(cli['failed'])} failed")
        print(f"      GUI: {len(gui['tracks'])} tracks, {len(gui['vias'])} vias, {len(gui['failed'])} failed")
        r.append(_ok("fanout actually produced copper", len(cli['tracks']) > 0))
        r.append(_ok("track sets identical across fronts",
                     _track_key(cli['tracks']) == _track_key(gui['tracks'])))
        r.append(_ok("via sets identical across fronts",
                     _via_key(cli['vias']) == _via_key(gui['vias'])))
        r.append(_ok("failed-net lists identical across fronts",
                     sorted(cli['failed']) == sorted(gui['failed'])))

        print("\n[4] surface escapes lie on the part's own face")
        for label, res in (("CLI", cli), ("GUI", gui)):
            on, off = _surface_escapes_off_face(res['tracks'], res['vias'], res['footprint'])
            r.append(_ok(f"{label}: {on} surface escape(s) on {res['footprint'].layer}, "
                         f"{off} on another face", on > 0 and off == 0))

    passed = sum(r)
    print(f"\n{passed}/{len(r)} back-side BGA fanout parity checks passed")
    print("=" * 60)
    return 0 if passed == len(r) else 1


if __name__ == "__main__":
    sys.exit(main())
