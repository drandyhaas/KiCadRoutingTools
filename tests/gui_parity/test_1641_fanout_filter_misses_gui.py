#!/usr/bin/env python3
"""#1641, GUI half: the fanout tab says which selected nets had no pad to fan
out, as qfn_fanout.py's JSON_SUMMARY `unmatched` does.

Drives the REAL FanoutTab of a headless RoutingDialog on
kicad_files/qfn_interior_pads.kicad_pcb (U1 has three interior GND pads):

  1. GND selected, stub escape: the engine's LAST_FILTER_MISSES reaches the
     apply step as `filter_misses` (reason `interior`, the three pads), and
     the completion message lists GND and names --escape-method underpad.
     Before #1641 the tab said only "Fanout complete! 0 tracks".
  2. A perimeter net renamed with a bus index ('D[3]') and selected in the
     tab is fanned out: the tab passes the literal name, which fnmatch read
     as 'D' plus a class, so the engine selected nothing.
  3. #1717: GND and FOREIGN selected (interior pads plus a perimeter net):
     the engine's LAST_SKIPPED_INTERIOR reaches the apply step as
     `skipped_interior` and the completion message lists the pads, as
     qfn_fanout.py's JSON_SUMMARY `skipped_interior` does.

Needs KiCad python (wx + pcbnew); re-execs into it, skips without it.
Seconds to run.

Run:  python3 tests/gui_parity/test_1641_fanout_filter_misses_gui.py
"""
import glob
import os
import shutil
import subprocess
import sys
import tempfile

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BOARD = os.path.join(REPO, 'kicad_files', 'qfn_interior_pads.kicad_pcb')
REF = 'U1'
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


def run_tab(pcbnew, wx, board_path, nets):
    """The tab's QFN fanout of NETS on U1; (apply kwargs, tracks, message)."""
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
            seen.update(kw, tracks=tracks)
            return real_apply(tracks, vias, **kw)

        tab._apply_fanout_results = _spy
        tab._run_qfn_fanout(pcb_data.footprints[REF], nets,
                            tab.qfn_options.get_config())
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
    print("[1] GND (three interior pads), stub escape")
    seen, msg = run_tab(pcbnew, wx, BOARD, ['GND'])
    miss = seen.get('filter_misses') or []
    r.append(_ok("the apply step got the engine's miss: interior, U1.33/36/37",
                 len(miss) == 1 and miss[0]['reason'] == 'interior'
                 and miss[0]['pads'] == ['U1.33', 'U1.36', 'U1.37'], repr(miss)))
    r.append(_ok("the completion message lists it and names underpad",
                 'Selected nets with no pad to fan out' in msg
                 and '--escape-method underpad' in msg, msg.replace('\n', ' | ')[:300]))

    print("[2] a net named with a bus index, selected by its name")
    tmp = tempfile.mkdtemp(prefix='t1641gui_')
    try:
        br = os.path.join(tmp, 'br.kicad_pcb')
        with open(BOARD, encoding='utf-8') as f:
            text = f.read()
        with open(br, 'w', encoding='utf-8') as f:
            f.write(text.replace('"FOREIGN"', '"D[3]"'))
        seen, _msg = run_tab(pcbnew, wx, br, ['D[3]'])
        tracks = seen.get('tracks') or []
        r.append(_ok("the tab fans out 'D[3]'", len(tracks) > 0
                     and not seen.get('filter_misses'),
                     f"{len(tracks)} track(s), misses {seen.get('filter_misses')}"))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print("[3] #1717: GND and the perimeter net FOREIGN, stub escape")
    seen, msg = run_tab(pcbnew, wx, BOARD, ['GND', 'FOREIGN'])
    r.append(_ok("the apply step got the skipped interior pads",
                 seen.get('skipped_interior') == ['U1.33', 'U1.36', 'U1.37'],
                 repr(seen.get('skipped_interior'))))
    r.append(_ok("the completion message lists them",
                 'Selected interior pads given no copper' in msg
                 and 'U1.36' in msg, msg.replace('\n', ' | ')[:300]))

    if all(r):
        print("\nALL PASS")
        return 0
    print(f"\nFAIL: {r.count(False)} check(s)")
    return 1


if __name__ == '__main__':
    sys.exit(main())
