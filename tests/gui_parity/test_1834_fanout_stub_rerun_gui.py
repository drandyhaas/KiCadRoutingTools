#!/usr/bin/env python3
"""#1834, GUI half: the fanout tab's QFN stub escape lays no second stub on
a pad that already has its surface escape, and says how many it skipped, as
qfn_fanout.py's JSON_SUMMARY `pads_already_served` does.

Drives the REAL FanoutTab of a headless RoutingDialog, stub escape, every
net of kicad_files/tigard.kicad_pcb's U3, on the board qfn_fanout.py wrote
for the same part (the CLI's lap 1): the tab lays no track and no via, its
surface-fan ledger names all 48 pads served, as the CLI's own second lap
does, and the completion message says so.

Needs KiCad python (wx + pcbnew); re-execs into it, skips without it.
Under a minute.

Run:  python3 tests/gui_parity/test_1834_fanout_stub_rerun_gui.py
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
    """The tab's stub escape of every U3 net; (apply kwargs, message)."""
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
        cfg = dict(tab.qfn_options.get_config(), escape_method='stub')
        tab._run_qfn_fanout(pcb_data.footprints[REF], None, cfg)
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
    tmp = tempfile.mkdtemp(prefix='t1834gui_')
    try:
        print("[1] the CLI's lap-1 output")
        b0 = os.path.join(tmp, 't0.kicad_pcb')
        b1 = os.path.join(tmp, 't1.kicad_pcb')
        b2 = os.path.join(tmp, 't2.kicad_pcb')
        py = shutil.which('python3') or 'python3'
        subprocess.run([py, os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, b0],
                       check=True, capture_output=True)
        cli = [py, '-X', 'utf8', os.path.join(REPO, 'py_router', 'qfn_fanout.py')]
        tail = ['--component', REF]
        subprocess.run(cli + [b0, '--output', b1] + tail, check=True, capture_output=True)
        out = subprocess.run(cli + [b1, '--output', b2] + tail, check=True,
                             capture_output=True, text=True).stdout
        m = re.search(r'^JSON_SUMMARY: (.*)$', out, re.M)
        cli_served = json.loads(m.group(1)).get('pads_already_served') if m else None
        seen, msg = run_tab(pcbnew, wx, b1)
        led = seen.get('stub_pads') or {}
        r.append(_ok("no track and no via", not seen.get('tracks') and not seen.get('vias'),
                     f"{len(seen.get('tracks') or [])} track(s), {len(seen.get('vias') or [])} via(s)"))
        r.append(_ok("every pad served, as the CLI's second lap says",
                     led.get('pads_already_served') == cli_served == 48
                     and led.get('pads_escaped') == 0,
                     f"tab {led}, CLI {cli_served}"))
        r.append(_ok("the completion message says so",
                     'Pads that already had their surface escape, skipped: 48' in msg,
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
