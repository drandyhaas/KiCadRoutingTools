#!/usr/bin/env python3
"""#581 on the fanout tab: the Basic tab's via-in-pad policy reaches BOTH escape engines.

`bga_fanout.py` / `qfn_fanout.py` pass `same_net_pad_clearance` to their
engines (> 0: BGA under-pad escapes run dog-bone, QFN places no via-in-pad).
The dialog has ONE control for it, on the Basic tab ("Allow via-in-pad" plus
its clearance spin), and the fanout tab's shared params carry its value. #621
moved the two engine calls into a worker-thread kwargs dict and the forwarding
line did not come along, so from 2026-08-16 the fanout tab ignored the control:
with via-in-pad unticked the CLI ran dog-bone and the GUI laid vias in pads.

That went unseen because `test_engine_kwarg_parity` could not read a call made
with `**kwargs` and printed SKIP under an OK verdict. That gate now follows the
dict; this one drives the REAL headless dialog and observes the VALUE arrive at
each engine (a wiring fix can be inert -- the value is the test):

  1. via-in-pad UNTICKED, spin set: both engines receive the dialog's own
     `_same_net_pad_clearance_value()`, and it is > 0;
  2. control: via-in-pad TICKED: both receive -1.0 (allowed).

The engines are spied, not run -- what is under test is the tab's hand-off,
and the escape itself is pinned by the rotated/backside fanout gates.

    python3 tests/gui_parity/test_581_fanout_via_in_pad_gui.py
"""
import glob
import os
import subprocess
import sys

# The real dialog builds about_tab, whose wxEXPAND|wxALIGN_* sizer flags trip a
# fatal assert on wx debug builds. Must be set before wx is imported.
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BOARD = os.path.join(REPO, 'kicad_files', 'glasgow_revC.kicad_pcb')
BGA_REF, QFN_REF = 'U30', 'U1'
SPIN = 0.15
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
            if subprocess.run([cand, '-c', 'import wx, kipy, pcbnew'],
                              capture_output=True).returncode == 0:
                argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
                if os.name == 'nt':
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("SKIP: no python with wx + kipy + pcbnew found")
    sys.exit(0)


def _ok(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    return bool(cond)


def _run_both(dialog, pcb_data):
    """Run the REAL fanout tab on the BGA and on the QFN; return the kwargs
    each spied engine received ({} when the tab never reached it)."""
    import bga_fanout
    import qfn_fanout
    from wx_pump import run_until

    seen = {}
    real_bga, real_qfn = bga_fanout.generate_bga_fanout, qfn_fanout.generate_qfn_fanout

    def _spy_bga(footprint, pcb, **kwargs):
        seen['bga'] = dict(kwargs)
        return [], [], [], []

    def _spy_qfn(footprint, pcb, **kwargs):
        seen['qfn'] = dict(kwargs)
        return [], [], []

    tab = dialog.fanout_tab
    tab._apply_fanout_results = lambda *a, **k: None
    try:
        bga_fanout.generate_bga_fanout = _spy_bga
        qfn_fanout.generate_qfn_fanout = _spy_qfn
        for kind, ref, run, opts in (
                ('bga', BGA_REF, tab._run_bga_fanout, tab.bga_options),
                ('qfn', QFN_REF, tab._run_qfn_fanout, tab.qfn_options)):
            fp = pcb_data.footprints[ref]
            nets = sorted({p.net_name for p in fp.pads if p.net_id and p.net_name})
            run(fp, nets, opts.get_config())
            run_until(lambda: not getattr(tab, '_running', False), 120)
    finally:
        bga_fanout.generate_bga_fanout, qfn_fanout.generate_qfn_fanout = real_bga, real_qfn
    return seen.get('bga', {}), seen.get('qfn', {})


def main():
    try:
        import wx, kipy, pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import wx

    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # wx_pump

    # ipc-migration: no swig_gui (renamed routing_dialog.py), and the dialog's
    # PCBData comes from a kipy board -- through the branch's fake-IPC harness,
    # like every other ported gate here.
    from fake_ipc_board import install as _install_fake_board, build_pcb_data_like_ipc
    from kicad_routing_plugin import routing_dialog

    app = wx.App(False)  # noqa: F841 - must outlive the dialog
    wx.MessageBox = lambda *a, **k: wx.OK

    _install_fake_board(BOARD)
    pcb_data = build_pcb_data_like_ipc(BOARD)
    dialog = routing_dialog.RoutingDialog(None, pcb_data, BOARD)
    r = []
    try:
        print(f"\n[1] via-in-pad UNTICKED, clearance spin {SPIN}")
        dialog.via_in_pad_check.SetValue(False)
        dialog.same_net_pad_clearance.Enable(True)
        dialog.same_net_pad_clearance.SetValue(SPIN)
        want = dialog._same_net_pad_clearance_value()   # post-clamp, so a range clamp cannot lie
        r.append(_ok(f"the dialog's policy value is a clearance ({want!r} > 0)", want > 0))
        bga_kw, qfn_kw = _run_both(dialog, pcb_data)
        r.append(_ok("the tab reached both engines", bool(bga_kw) and bool(qfn_kw)))
        for kind, kw in (('generate_bga_fanout', bga_kw), ('generate_qfn_fanout', qfn_kw)):
            got = kw.get('same_net_pad_clearance', '<ABSENT>')
            r.append(_ok(f"{kind} received same_net_pad_clearance={got!r} (want {want!r})",
                         got == want))

        print("\n[2] control: via-in-pad TICKED (allowed)")
        dialog.via_in_pad_check.SetValue(True)
        want = dialog._same_net_pad_clearance_value()
        r.append(_ok(f"the dialog's policy value is 'allowed' ({want!r})", want == -1.0))
        bga_kw, qfn_kw = _run_both(dialog, pcb_data)
        r.append(_ok("the tab reached both engines", bool(bga_kw) and bool(qfn_kw)))
        for kind, kw in (('generate_bga_fanout', bga_kw), ('generate_qfn_fanout', qfn_kw)):
            got = kw.get('same_net_pad_clearance', '<ABSENT>')
            r.append(_ok(f"{kind} received same_net_pad_clearance={got!r} (want {want!r})",
                         got == want))
    finally:
        dialog.Destroy()

    failed = r.count(False)
    print(f"\n{'PASS' if not failed else 'FAIL'}: {len(r) - failed}/{len(r)} checks -- "
          f"the fanout tab hands the Basic tab's via-in-pad policy to both escape engines (#581)")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
