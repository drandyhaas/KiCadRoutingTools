#!/usr/bin/env python3
"""The Auto tab's Labels sub-tab against py_placer/beautify_labels.py.

Beautify Labels moved off the AI Placement sub-tab onto its own. This gate
drives the REAL button on the REAL Labels tab of a headless RoutingDialog on a
live board (pcbnew.LoadBoard, reached through pcbnew.GetBoard as inside
KiCad), saves the board, and compares every footprint's Reference label --
position, angle, size, thickness -- with the CLI's output on the same file,
at the same options (the tab's defaults are the CLI's, and one option is set
off its default on both fronts). It also checks the AI Placement tab carries
no label control any more.

Needs KiCad python (wx + pcbnew); re-execs into it, and exits 77 with a SKIP:
reason when there is none.

ipc-migration: the live board is the fake-IPC board (tests/gui_parity/
fake_ipc_board.py) over a COPY of the input, so the gate needs kipy too. The
tab writes each label onto the footprint's REAL kipy Reference field
(`labels_gui.apply_label_results`: world position, angle, size, stroke,
alignment) in one commit; the fake writes the changed fields back into its
file -- world position into the footprint frame by the parser's own inverse
transform, through the CLI's label writer, which writes a centred label of
one size. So this gate grades that the button runs the engine and hands its
values to the kipy fields, and that they reach the file. It does NOT grade
how KiCad itself stores a field set over IPC (the angle, the alignment, a
B-side mirror): only a live KiCad proves that. "Saved" is kipy's save_as.

    python3 tests/gui_parity/test_labels_tab_gui.py
"""
import glob
import os
import subprocess
import sys
import tempfile

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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
SIZE = 0.9                     # off the 1.0 default, on both fronts
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f" -- {detail}" if detail else ''),
          flush=True)
    if not ok:
        FAILS.append(name)


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        r = subprocess.run([cand, '-c', 'import pcbnew, wx, kipy'], capture_output=True)
        if r.returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("SKIP: no python with pcbnew + wx + kipy found (this gate needs KiCad's, "
          "with kicad-python installed)")
    sys.exit(77)


def labels(path):
    """{footprint key: (x, y, angle, size, thickness)} of every Reference
    label, in board coordinates to 1 um."""
    import pcbnew
    b = pcbnew.LoadBoard(path)
    out = {}
    for key, fp in _pcbnew_footprints_by_key(b).items():
        t = fp.Reference()
        p = t.GetPosition()
        out[key] = (round(pcbnew.ToMM(p.x), 3), round(pcbnew.ToMM(p.y), 3),
                    round(t.GetTextAngleDegrees() % 360, 2),
                    round(pcbnew.ToMM(t.GetTextSize().y), 3),
                    round(pcbnew.ToMM(t.GetTextThickness()), 3))
    return out


def _pcbnew_footprints_by_key(b):
    """{PCBData key: pcbnew footprint} of pcbnew board B (#726): the bare
    reference, `TP4~2` for a repeat -- main's gui_utils.live_footprints_by_key,
    which this branch's IPC plugin does not have (its twin,
    result_apply.live_footprints_by_key, walks a kipy board)."""
    from kicad_parser import disambiguate_references
    live = list(b.GetFootprints())
    raw = [f.GetReference() or ('#' + f.m_Uuid.AsString()) for f in live]
    return dict(zip(disambiguate_references(raw), live))


def run():
    import wx
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, os.path.join(REPO, 'tests', 'gui_parity'))
    from copy_board import copy_board
    from fake_ipc_board import (install as install_fake_board,
                                build_pcb_data_like_ipc)
    from wx_pump import run_until

    tmp = tempfile.mkdtemp(prefix='krt_labels_gate_')
    src = os.path.join(tmp, 'board.kicad_pcb')
    copy_board(BOARD, src)
    before = labels(src)

    cli_out = os.path.join(tmp, 'cli.kicad_pcb')
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(REPO, 'py_placer', 'beautify_labels.py'),
                        src, cli_out, '--size', str(SIZE)],
                       capture_output=True, text=True, timeout=600)
    if r.returncode != 0 or not os.path.isfile(cli_out):
        raise AssertionError(f"BROKEN TEST: beautify_labels.py failed (rc={r.returncode}):"
                             f"\n{r.stdout[-1500:]}{r.stderr[-1500:]}")

    app = wx.App(False)  # noqa: F841
    wx.Log.SetActiveTarget(wx.LogStderr())
    wx.MessageBox = lambda *a, **k: wx.OK
    # ipc-migration: the open board is the fake-IPC board over a COPY of the
    # input (every commit is written back to its file).
    live = os.path.join(tmp, 'live', 'board.kicad_pcb')
    os.makedirs(os.path.dirname(live))
    copy_board(src, live)
    board = install_fake_board(live)
    from kicad_routing_plugin.routing_dialog import RoutingDialog
    dlg = RoutingDialog(None, build_pcb_data_like_ipc(live), live)
    try:
        sub = [dlg.ai_notebook.GetPageText(i) for i in range(dlg.ai_notebook.GetPageCount())]
        check("Labels is the Auto page's fourth sub-tab",
              sub[3:4] == ['Labels'] and dlg.ai_notebook.GetPage(3) is dlg.labels_tab, sub)
        pt = dlg.placement_tab
        check("the AI Placement tab carries no label control",
              not hasattr(pt, 'labels_options') and not hasattr(pt, 'action_btn')
              and not hasattr(pt, 'run_beautify_labels'))
        tab = dlg.labels_tab
        for name in ('label_size', 'label_thickness', 'label_min_size', 'label_orientation',
                     'label_max_distance', 'label_include_hidden', 'label_skip_locked'):
            check(f"control {name} on the Labels tab",
                  getattr(tab.labels_options, name, None) is not None)
        tab.labels_options.label_size.SetValue(SIZE)
        evt = wx.CommandEvent(wx.wxEVT_BUTTON, tab.action_btn.GetId())
        evt.SetEventObject(tab.action_btn)
        tab.action_btn.GetEventHandler().ProcessEvent(evt)
        if tab._routing_thread is None:
            raise AssertionError(f"BROKEN TEST: the button started nothing: "
                                 f"{tab.status_text.GetValue()}")
        check("the job finished", run_until(lambda: tab._routing_thread is None, 300))
        status = tab.status_text.GetValue()
        check("its status line is the engine's summary",
              status.startswith('Labels:') and 'applied' in status, status)
        check("the button is armed again", tab.action_btn.IsEnabled())
        gui_out = os.path.join(tmp, 'gui.kicad_pcb')
        board.save_as(gui_out, overwrite=True, include_project=False)  # kipy's save
    finally:
        dlg.Destroy()
        try:
            wx.GetApp().ProcessPendingEvents()
        except Exception:                                       # noqa: BLE001
            pass
    cli, gui = labels(cli_out), labels(gui_out)
    changed = [k for k in cli if cli[k] != before.get(k)]
    check("the CLI moved labels (the comparison grades something)", len(changed) > 10,
          f"{len(changed)} of {len(cli)}")
    differ = [k for k in cli if cli[k] != gui.get(k)]
    check("every Reference label is the CLI's", not differ and len(cli) == len(gui),
          f"{len(differ)} differ, e.g. " + "; ".join(
              f"{k}: cli {cli[k]} gui {gui.get(k)}" for k in differ[:4]))
    print(f"\nwork: {tmp}")
    if FAILS:
        print(f"VERDICT: FAIL ({len(FAILS)}): " + "; ".join(FAILS))
        return 1
    print(f"VERDICT: the Labels tab matches beautify_labels.py ({len(changed)} labels moved)")
    return 0


def main():
    try:
        import pcbnew  # noqa: F401
        import wx  # noqa: F401
        import kipy  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(run())


if __name__ == '__main__':
    main()
