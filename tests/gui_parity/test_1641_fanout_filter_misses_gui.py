#!/usr/bin/env python3
"""#1641, GUI half: the fanout tab says which selected nets had no pad to fan
out, as qfn_fanout.py's JSON_SUMMARY `unmatched` does.

Drives the REAL FanoutTab of a headless RoutingDialog on
kicad_files/qfn_interior_pads.kicad_pcb (U1 has three interior GND pads):

  1. GND selected, stub escape: since #1717 the engine via-drops GND's three
     interior pads, so there is no filter miss; the via it places reaches
     the apply step, and so do the two pads it cannot place, as
     `skipped_interior`, which the completion message lists. #1743: the
     placed pad's via-drop stub, drawn at the track width on a ground net,
     reaches it as `power_stubs_at_width`, and the message lists it too.
  2. A perimeter net renamed with a bus index ('D[3]') and selected in the
     tab is fanned out: the tab passes the literal name, which fnmatch read
     as 'D' plus a class, so the engine selected nothing.
  3. #1717: GND and FOREIGN selected (interior pads plus a perimeter net):
     FOREIGN's stubs and the same `skipped_interior` reach the apply step,
     as qfn_fanout.py's JSON_SUMMARY `skipped_interior` has them.

On ipc-migration the plugin has no in-process board: the dialog reaches it
through kicad_ipc_adapter, which this gate points at
tests/gui_parity/fake_ipc_board (the board FILE, commits flushed back). Each
run therefore works on a fresh temp copy of the board and its siblings, so the
tab's apply (and the .kicad_pro its DRC-settings fix writes) never touches the
input -- what a fresh pcbnew.LoadBoard per run gave the SWIG gate.

Needs KiCad python (wx + kipy); re-execs into it, skips without it.
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


def run_tab(wx, board_path, nets):
    """The tab's QFN fanout of NETS on U1; (apply kwargs, tracks, message)."""
    from copy_board import copy_siblings
    from fake_ipc_board import install as install_fake_board, build_pcb_data_like_ipc
    # routing_dialog.py is this branch's swig_gui.py (renamed by the IPC port).
    from kicad_routing_plugin import routing_dialog

    # A fresh copy per run: the fake live board flushes the apply's commit
    # back into its file.
    tmp = tempfile.mkdtemp(prefix='t1641gui_run_')
    live = os.path.join(tmp, os.path.basename(board_path))
    shutil.copyfile(board_path, live)
    copy_siblings(board_path, live)
    install_fake_board(live)
    pcb_data = build_pcb_data_like_ipc(live)
    dialog = routing_dialog.RoutingDialog(None, pcb_data, live)
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
        tab._run_qfn_fanout(pcb_data.footprints[REF], nets,
                            tab.qfn_options.get_config())
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
    sys.path.insert(0, os.path.join(REPO, 'tests', 'gui_parity'))
    app = wx.App(False)  # noqa: F841
    wx.Log.SetActiveTarget(wx.LogStderr())

    r = []
    print("[1] GND (three interior pads), stub escape")
    seen, msg = run_tab(wx, BOARD, ['GND'])
    r.append(_ok("no filter miss, and the via-drop's via reaches the apply step",
                 not seen.get('filter_misses') and len(seen.get('vias') or []) == 1,
                 f"misses {seen.get('filter_misses')}, {len(seen.get('vias') or [])} via(s)"))
    r.append(_ok("the unplaced pads reach it and the message lists them",
                 seen.get('skipped_interior') == ['U1.36', 'U1.37']
                 and 'Selected interior pads given no copper' in msg,
                 msg.replace('\n', ' | ')[:300]))
    # #1743: the placed pad's via-drop stub is drawn at the track width on a
    # ground net, as the CLI's `power_stubs_at_width` names it
    r.append(_ok("the power stub at the track width reaches it and the message lists it",
                 [p.get('pad') for p in seen.get('power_stubs_at_width') or []] == ['U1.33']
                 and 'Power/ground stubs drawn at the track width' in msg
                 and 'only a wider --width widens it' in msg,
                 f"{seen.get('power_stubs_at_width')} | "
                 + msg.replace('\n', ' | ')[:300]))

    print("[2] a net named with a bus index, selected by its name")
    tmp = tempfile.mkdtemp(prefix='t1641gui_')
    try:
        br = os.path.join(tmp, 'br.kicad_pcb')
        with open(BOARD, encoding='utf-8') as f:
            text = f.read()
        with open(br, 'w', encoding='utf-8') as f:
            f.write(text.replace('"FOREIGN"', '"D[3]"'))
        seen, _msg = run_tab(wx, br, ['D[3]'])
        tracks = seen.get('tracks') or []
        r.append(_ok("the tab fans out 'D[3]'", len(tracks) > 0
                     and not seen.get('filter_misses'),
                     f"{len(tracks)} track(s), misses {seen.get('filter_misses')}"))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print("[3] #1717: GND and the perimeter net FOREIGN, stub escape")
    seen, msg = run_tab(wx, BOARD, ['GND', 'FOREIGN'])
    r.append(_ok("FOREIGN's stubs and the unplaced pads reach the apply step",
                 len(seen.get('tracks') or []) > 0
                 and seen.get('skipped_interior') == ['U1.36', 'U1.37'],
                 f"{len(seen.get('tracks') or [])} track(s), {seen.get('skipped_interior')}"))
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
