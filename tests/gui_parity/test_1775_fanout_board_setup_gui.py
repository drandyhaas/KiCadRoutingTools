#!/usr/bin/env python3
"""#1775, GUI half: the fanout tab's self-sized via meets the board's Board
Setup via minimums under Escalation 'board', and is named when it does not.

The tab starts its via from the Default net class (via override unticked);
the board here declares that class via at 0.45/0.25 (ring 0.10) under a Board
Setup ring of 0.13 (the CLI test's socket board, tests/test_1775_*). Drives
the REAL FanoutTab of a headless RoutingDialog, "Fix DRC settings" unticked:

  1. Escalation 'board': the shared resolver raises the via to 0.51/0.25, and
     the vias reaching the apply step hold the ring; nothing is named;
  2. Escalation 'fab': the via stays 0.45/0.25, the engine's
     `board_setup_vias` reaches the apply step and the completion message
     lists the Board Setup minimum, as qfn_fanout.py's JSON_SUMMARY does.

On ipc-migration the plugin has no in-process board: the dialog reaches it
through kicad_ipc_adapter, which this gate points at
tests/gui_parity/fake_ipc_board (the board FILE, commits flushed back), and the
Board Setup minimums reach it from the sibling .kicad_pro. Each run therefore
works on a fresh copy of the board and its siblings, so the first run's vias
never reach the second -- what a fresh pcbnew.LoadBoard per run gave the SWIG
gate.

Needs KiCad python (wx + kipy); re-execs into it, skips without it.
Seconds to run.

Run:  python3 tests/gui_parity/test_1775_fanout_board_setup_gui.py
"""
import glob
import os
import shutil
import subprocess
import sys
import tempfile

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REF = 'J9'
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
            if subprocess.run([cand, '-c', 'import wx, kipy'],
                              capture_output=True).returncode == 0:
                argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
                if os.name == 'nt':
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("SKIP: no python with wx + kipy found")
    sys.exit(0)


def _ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{(': ' + detail) if detail else ''}")
    return bool(cond)


def run_tab(wx, board_path, escalation):
    """The tab's QFN fanout of SIG* on J9; (apply kwargs, message)."""
    from copy_board import copy_board
    from fake_ipc_board import install as install_fake_board, build_pcb_data_like_ipc
    # routing_dialog.py is this branch's swig_gui.py (renamed by the IPC port).
    from kicad_routing_plugin import routing_dialog

    # A fresh copy per run: the fake live board flushes the apply's commit
    # back into its file.
    tmp = tempfile.mkdtemp(prefix='t1775gui_run_')
    live = os.path.join(tmp, os.path.basename(board_path))
    copy_board(board_path, live)
    install_fake_board(live)
    pcb_data = build_pcb_data_like_ipc(live)
    dialog = routing_dialog.RoutingDialog(None, pcb_data, live)
    messages = []
    wx.MessageBox = lambda msg, *a, **k: messages.append(msg) or wx.OK
    seen = {}
    try:
        dialog.escalation.SetStringSelection(escalation)
        dialog.fix_drc_check.SetValue(False)
        tab = dialog.fanout_tab
        real_apply = tab._apply_fanout_results

        def _spy(tracks, vias, **kw):
            seen.update(kw, tracks=tracks, vias=vias)
            return real_apply(tracks, vias, **kw)

        tab._apply_fanout_results = _spy
        nets = sorted({p.net_name for p in pcb_data.footprints[REF].pads if p.net_name})
        tab._run_qfn_fanout(pcb_data.footprints[REF], nets, tab.qfn_options.get_config())
        from wx_pump import run_until
        run_until(lambda: not getattr(tab, '_running', False), 300)
    finally:
        dialog.Destroy()
        shutil.rmtree(tmp, ignore_errors=True)
    return seen, '\n'.join(messages)


def main():
    try:
        import wx, kipy  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import wx

    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))
    sys.path.insert(0, os.path.join(REPO, 'tests'))
    sys.path.insert(0, os.path.join(REPO, 'tests', 'gui_parity'))
    app = wx.App(False)  # noqa: F841
    wx.Log.SetActiveTarget(wx.LogStderr())
    from test_1775_self_sized_via_board_setup import make_board

    r = []
    tmp = tempfile.mkdtemp(prefix='t1775gui_')
    try:
        board = make_board(os.path.join(tmp, 'b'), class_via=(0.45, 0.25))
        print("[1] Escalation 'board', Fix DRC settings off")
        seen, msg = run_tab(wx, board, 'board')
        sizes = sorted({(round(v['size'], 3), round(v['drill'], 3))
                        for v in seen.get('vias') or []})
        r.append(_ok("the via-drop's vias hold the Board Setup ring", sizes == [(0.51, 0.25)],
                     repr(sizes)))
        r.append(_ok("nothing named", not seen.get('board_setup_vias'),
                     repr(seen.get('board_setup_vias'))))
        print("[2] Escalation 'fab', Fix DRC settings off")
        seen, msg = run_tab(wx, board, 'fab')
        sizes = sorted({(round(v['size'], 3), round(v['drill'], 3))
                        for v in seen.get('vias') or []})
        r.append(_ok("the via keeps the class size", sizes == [(0.45, 0.25)], repr(sizes)))
        rows = seen.get('board_setup_vias') or []
        r.append(_ok("board_setup_vias reaches the apply step",
                     len(rows) == 1 and rows[0]['rule'] == 'Board Setup min_via_annular_width'
                     and rows[0]['below'] == len(seen.get('vias') or []) > 0, repr(rows)))
        r.append(_ok("the completion message names it",
                     "below the board's Board Setup minimums" in msg
                     and 'min_via_annular_width' in msg, msg.replace('\n', ' | ')[:400]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print("\nALL PASS" if all(r) else "\nFAILED")
    return 0 if all(r) else 1


if __name__ == '__main__':
    sys.exit(main())
