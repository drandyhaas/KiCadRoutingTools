#!/usr/bin/env python3
"""#1804, GUI half: the fanout tab's QFN via-drop gives no second via to a
pad that has its escape, and says how many it skipped, as qfn_fanout.py's
JSON_SUMMARY `pads_already_served` does.

Drives the REAL FanoutTab of a headless RoutingDialog, underpad escape with
via-in-pad, GND of kicad_files/tigard.kicad_pcb's U3:

  1. on the tracked board: every one of U3's 28 GND pads is escaped, none
     dropped, and the per-pad ledger reaches the apply step;
  2. on the board qfn_fanout.py wrote for the same step (the CLI's lap 1):
     the tab lays no track and no via, the ledger names all 28 pads served,
     as the CLI's own second lap does, and the completion message says so.

Needs KiCad python (wx + pcbnew); re-execs into it, skips without it.
Under a minute.

Run:  python3 tests/gui_parity/test_1804_fanout_underpad_rerun_gui.py
"""
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BOARD = os.path.join(REPO, 'kicad_files', 'tigard.kicad_pcb')
REF = 'U3'
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


def _ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{(': ' + detail) if detail else ''}")
    return bool(cond)


def run_tab(pcbnew, wx, board_path):
    """The tab's underpad via-drop of U3's GND; (apply kwargs, message)."""
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui

    board = pcbnew.LoadBoard(board_path)
    pcbnew.GetBoard = lambda: board
    pcb_data = build_pcb_data_from_board(board)
    dialog = swig_gui.RoutingDialog(None, pcb_data, board_path)
    messages = []
    wx.MessageBox = lambda msg, *a, **k: messages.append(msg) or wx.OK
    seen = {}
    try:
        tab = dialog.fanout_tab
        real_apply = tab._apply_fanout_results

        def _spy(tracks, vias, **kw):
            seen.update(kw, tracks=tracks, vias=vias)
            return real_apply(tracks, vias, **kw)

        tab._apply_fanout_results = _spy
        cfg = dict(tab.qfn_options.get_config(), escape_method='underpad',
                   allow_via_in_pad=True)
        tab._run_qfn_fanout(pcb_data.footprints[REF], ['GND'], cfg)
        from wx_pump import run_until
        run_until(lambda: not getattr(tab, '_running', False), 300)
    finally:
        dialog.Destroy()
    return seen, '\n'.join(messages)


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
    sys.path.insert(0, os.path.join(REPO, 'tests', 'gui_parity'))
    app = wx.App(False)  # noqa: F841
    wx.Log.SetActiveTarget(wx.LogStderr())

    r = []
    tmp = tempfile.mkdtemp(prefix='t1804gui_')
    try:
        print("[1] the tracked board")
        seen, _msg = run_tab(pcbnew, wx, BOARD)
        led = seen.get('underpad_pads') or {}
        r.append(_ok("28 pads escaped, none dropped, the ledger reaches the apply step",
                     led.get('pads_requested') == led.get('pads_escaped') == 28
                     and led.get('dropped_pads') == [] and len(seen.get('vias') or []) > 0,
                     f"{led}, {len(seen.get('vias') or [])} via(s)"))

        print("[2] the CLI's lap-1 output")
        b0 = os.path.join(tmp, 't0.kicad_pcb')
        b1 = os.path.join(tmp, 't1.kicad_pcb')
        b2 = os.path.join(tmp, 't2.kicad_pcb')
        py = shutil.which('python3') or 'python3'
        subprocess.run([py, os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, b0],
                       check=True, capture_output=True)
        cli = [py, '-X', 'utf8', os.path.join(REPO, 'py_router', 'qfn_fanout.py')]
        tail = ['--component', REF, '--nets', 'GND', '--escape-method', 'underpad',
                '--allow-via-in-pad']
        subprocess.run(cli + [b0, '--output', b1] + tail, check=True, capture_output=True)
        out = subprocess.run(cli + [b1, '--output', b2] + tail, check=True,
                             capture_output=True, text=True).stdout
        m = re.search(r'^JSON_SUMMARY: (.*)$', out, re.M)
        cli_served = json.loads(m.group(1)).get('pads_already_served') if m else None
        seen, msg = run_tab(pcbnew, wx, b1)
        led = seen.get('underpad_pads') or {}
        r.append(_ok("no track and no via", not seen.get('tracks') and not seen.get('vias'),
                     f"{len(seen.get('tracks') or [])} track(s), {len(seen.get('vias') or [])} via(s)"))
        r.append(_ok("every pad served, as the CLI's second lap says",
                     led.get('pads_already_served') == cli_served == 28,
                     f"tab {led.get('pads_already_served')}, CLI {cli_served}"))
        r.append(_ok("the completion message says so",
                     'Pads that already had a via escape, skipped: 28' in msg,
                     msg.replace('\n', ' | ')[:300]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if all(r):
        print("\nALL PASS")
        return 0
    print(f"\nFAIL: {r.count(False)} check(s)")
    return 1


if __name__ == '__main__':
    sys.exit(main())
