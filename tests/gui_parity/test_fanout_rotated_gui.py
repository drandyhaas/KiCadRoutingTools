#!/usr/bin/env python3
"""CLI/GUI parity for QFN fanout on a ROTATED footprint (PR 633 / 301ee1b3).

This fills a hole the ledger has named twice: **no gate runs a fanout step**.
Fanout is the only routing consumer of `pad.local_x/local_y`, and those are the
one PCBData field the GUI front COMPUTES (`build_pcb_data_from_board`'s
pad-local fallback, live since KiCad 7+ removed `pad.GetPos0()`) rather than
reading from the file. So a bug there is invisible to every existing gate:
test_gui_engine_parity's plan is signal + plane, and the CLI text parser reads
pad locals verbatim and never executes the code at all.

That is exactly how `_global_to_local`'s transposed rotation survived -- wrong
for 620/1079 pads on this very board, and nothing went red.

What this pins, driving the REAL FanoutTab on a REAL headless RoutingDialog
against the CLI's text-parsed engine call, same board, same component:

  1. side classification is IDENTICAL across the two fronts. This is the
     assertion that catches a local-frame bug: the transposed body reported
     bottom 17 / top 3 on this 90 deg QFN-76 where the CLI says bottom 3 /
     top 17.
  2. the emitted copper is IDENTICAL across the two fronts. Note this alone
     would NOT have caught the bug -- QFN escape geometry is built from pad
     GLOBALS, so a 180 deg relabel of a centre-symmetric package moves nothing
     (measured: 104 tracks both ways, byte-identical). It is here because
     fanout copper parity is worth pinning on its own, and because an
     asymmetric or non-orthogonal part would break it.
  3. every escape leaves its pad OUTWARD, on both fronts.

Board: kicad_files/haasoscope_pro_max_test.kicad_pcb, U2 = QFN-76 @ 90 deg --
the in-repo part whose rotation makes the two candidate local frames disagree
(they coincide only at 0/180).

Needs KiCad python (wx + pcbnew); skips cleanly without it. Costs a few
seconds -- no routing, just the fanout engine twice.

Run:  python3 tests/gui_parity/test_fanout_rotated_gui.py
"""

# ---------------------------------------------------------------------------
# macOS: if this HANGS at ~0% CPU, it is NOT wx, machine load, or a deadlock.
#
# After any wx process here is killed (a pkill, a timeout, a crash), macOS
# decides the app "quit unexpectedly", and the NEXT headless launch stops inside
# NSApplication bootstrap showing the restore-windows alert you cannot see:
#     -[NSPersistentUIRestorer promptToIgnorePersistentState]
#         -> -[NSAlert runModal]
# Headless, nobody can click it, so it waits forever: process state SN accruing
# ~0.3s of CPU over many minutes, which reads exactly like a hang.
#
#   diagnose:  sample <pid> 3 -mayDie | grep -E "NSAlert|PersistentUI"
#   fix:       defaults write -g ApplePersistenceIgnoreState -bool YES
# ---------------------------------------------------------------------------
import math
import glob
import os
import subprocess
import sys

# The real dialog builds about_tab, whose wxEXPAND|wxALIGN_* sizer flags trip a
# fatal assert on wx debug builds. Must be set before wx is imported.
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BOARD = os.path.join(REPO, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')
REF = 'U2'
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


def _sides(footprint):
    """{pad_number: side} from the QFN layout analyser, or None."""
    from qfn_fanout.layout import analyze_qfn_layout, analyze_pad
    layout = analyze_qfn_layout(footprint)
    if layout is None:
        return None
    return {p.pad_number: analyze_pad(p, layout).side for p in footprint.pads}


def _side_histogram(sides):
    h = {}
    for s in (sides or {}).values():
        h[s] = h.get(s, 0) + 1
    return dict(sorted(h.items()))


def _copper_key(tracks):
    """Order- and identity-independent view of the emitted copper."""
    return sorted((round(t['start'][0], 6), round(t['start'][1], 6),
                   round(t['end'][0], 6), round(t['end'][1], 6),
                   t['net_id'], t['layer'], round(t['width'], 6))
                  for t in tracks)


def _outward_counts(tracks, footprint):
    """(outward, inward) escapes, judged in BOARD space against the package
    centre -- front-independent by construction."""
    gxs = [p.global_x for p in footprint.pads]
    gys = [p.global_y for p in footprint.pads]
    cx, cy = (min(gxs) + max(gxs)) / 2, (min(gys) + max(gys)) / 2
    by_net = {}
    for p in footprint.pads:
        if p.net_id:
            by_net.setdefault(p.net_id, []).append(p)

    outward = inward = 0
    for t in tracks:
        pads = by_net.get(t['net_id']) or []
        best, bd, far = None, 1e18, None
        for p in pads:
            for near, other in ((t['start'], t['end']), (t['end'], t['start'])):
                d = math.hypot(near[0] - p.global_x, near[1] - p.global_y)
                if d < bd:
                    best, bd, far = p, d, other
        if best is None or bd > 0.6:
            continue
        rx, ry = best.global_x - cx, best.global_y - cy
        n = math.hypot(rx, ry) or 1.0
        tx, ty = far[0] - best.global_x, far[1] - best.global_y
        if math.hypot(tx, ty) < 1e-9:
            continue
        if (tx * rx / n + ty * ry / n) > 0:
            outward += 1
        else:
            inward += 1
    return outward, inward


def run_gui(pcbnew):
    """The GUI front: pcbnew-built PCBData through the REAL FanoutTab.

    Also records the exact kwargs the tab hands `generate_qfn_fanout`, so the
    CLI leg can replay THOSE rather than a hand-rebuilt parameter set. The tab
    resolves its arguments from three places (the QFN options panel, the
    Basic-tab shared params, routing_defaults), and mirroring that here is
    precisely the kind of parallel interface that drifts silently -- the plane
    gate learned this the hard way with its `Shim`.
    """
    import qfn_fanout
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui

    board = pcbnew.LoadBoard(BOARD)
    pcbnew.GetBoard = lambda: board
    pcb_data = build_pcb_data_from_board(board)
    dialog = swig_gui.RoutingDialog(None, pcb_data, BOARD)

    real_engine = qfn_fanout.generate_qfn_fanout
    seen = {}

    def _spy(footprint, pcb, **kwargs):
        seen['kwargs'] = dict(kwargs)
        return real_engine(footprint, pcb, **kwargs)

    try:
        qfn_fanout.generate_qfn_fanout = _spy
        tab = dialog.fanout_tab
        fp = pcb_data.footprints[REF]

        captured = {}

        def _capture(tracks, vias, failed_nets=None, **kw):
            captured['tracks'] = tracks
            captured['vias'] = vias
            captured['failed'] = failed_nets or []

        # Stop at the engine boundary: applying would mutate the live board and
        # is not what this gate is about.
        tab._apply_fanout_results = _capture

        nets = sorted({p.net_name for p in fp.pads if p.net_id and p.net_name})
        tab._run_qfn_fanout(fp, nets, tab.qfn_options.get_config())

        # #621: the tab runs its escape on a WORKER THREAD now, so the call
        # above returns before the engine has done anything. Pump the real
        # event loop until the tab reports idle -- the same condition
        # ai_plan._poll_until_idle waits on. Reading `captured` without this
        # gets an empty dict and the gate "passes" by comparing nothing.
        # A real MainLoop, not a Yield loop: the tab collects the worker's
        # result through wx.CallLater, and Yield-pumping never fires wx timers
        # on Windows (see wx_pump.py).
        from wx_pump import run_until
        run_until(lambda: not getattr(tab, '_running', False), 300)

        kwargs = seen.get('kwargs')
        if kwargs is not None:
            # Bound to the tab's status label; the dialog is about to die and
            # the CLI leg has no UI to drive anyway.
            kwargs.pop('progress_callback', None)
            # Same reason, for #621's cancel predicate: it is a live closure
            # over the (about to be destroyed) tab, not a routing parameter,
            # and the CLI leg has no cancel source anyway.
            kwargs.pop('cancel_check', None)

        return ({'footprint': fp, 'sides': _sides(fp),
                 'tracks': captured.get('tracks') or [],
                 'vias': captured.get('vias') or [],
                 'failed': captured.get('failed') or []},
                nets, kwargs)
    finally:
        qfn_fanout.generate_qfn_fanout = real_engine
        dialog.Destroy()


def run_cli(kwargs):
    """The CLI front: text-parsed PCBData into the shared engine, with the
    GUI's own resolved kwargs."""
    from kicad_parser import parse_kicad_pcb
    from qfn_fanout import generate_qfn_fanout

    pcb = parse_kicad_pcb(BOARD)
    fp = pcb.footprints[REF]
    tracks, vias, failed = generate_qfn_fanout(fp, pcb, **kwargs)
    return {'footprint': fp, 'sides': _sides(fp), 'tracks': tracks,
            'vias': vias, 'failed': failed}


def main():
    try:
        import wx, pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import wx
    import pcbnew

    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522

    if not os.path.exists(BOARD):
        print(f"SKIP: {os.path.relpath(BOARD, REPO)} not found")
        return 0

    app = wx.App(False)  # noqa: F841 - must outlive the dialog
    wx.MessageBox = lambda *a, **k: wx.OK

    r = []

    print(f"\nGUI front: real FanoutTab on {os.path.basename(BOARD)} {REF}")
    gui, nets, kwargs = run_gui(pcbnew)
    r.append(_ok("captured the tab's engine kwargs (the CLI leg replays these, "
                 "so the two fronts cannot differ by a parameter)",
                 bool(kwargs)))
    if not kwargs:
        print("\nThe tab never reached generate_qfn_fanout -- nothing to compare")
        return 1
    print(f"\nCLI front: text-parsed PCBData, same {len(nets)} nets, "
          f"replaying {len(kwargs)} kwargs from the tab")
    cli = run_cli(kwargs)

    # The premise: if this part ever stops being rotated off-axis, the gate is
    # no longer testing anything and must say so rather than pass.
    rot = cli['footprint'].rotation
    r.append(_ok(f"{REF} is rotated off-axis (rotation={rot}) -- the two local "
                 f"frames only disagree away from 0/180",
                 abs(rot % 180) > 1e-9))
    r.append(_ok(f"both fronts analysed {REF} as a QFN",
                 bool(cli['sides']) and bool(gui['sides'])))
    if not (cli['sides'] and gui['sides']):
        print("\nCannot continue without a QFN layout on both fronts")
        return 1

    # --- 1: side classification parity (the assertion that catches the bug) ---
    print("\n[1] side classification parity")
    h_cli, h_gui = _side_histogram(cli['sides']), _side_histogram(gui['sides'])
    print(f"      CLI sides: {h_cli}")
    print(f"      GUI sides: {h_gui}")
    differing = [k for k in cli['sides']
                 if gui['sides'].get(k) != cli['sides'][k]]
    r.append(_ok(f"every pad lands on the same side on both fronts "
                 f"({len(cli['sides'])} pads, {len(differing)} differing)",
                 not differing))
    if differing:
        for k in differing[:8]:
            print(f"        pad {k}: CLI={cli['sides'][k]} GUI={gui['sides'][k]}")

    # --- 2: emitted copper parity ---
    print("\n[2] emitted copper parity")
    print(f"      CLI: {len(cli['tracks'])} tracks, {len(cli['vias'])} vias, "
          f"{len(cli['failed'])} failed")
    print(f"      GUI: {len(gui['tracks'])} tracks, {len(gui['vias'])} vias, "
          f"{len(gui['failed'])} failed")
    r.append(_ok("fanout actually produced copper (a silent no-op would pass "
                 "every parity check below)", len(cli['tracks']) > 0))
    r.append(_ok("track sets identical across fronts",
                 _copper_key(cli['tracks']) == _copper_key(gui['tracks'])))
    r.append(_ok("via counts identical across fronts",
                 len(cli['vias']) == len(gui['vias'])))
    r.append(_ok("failed-net counts identical across fronts",
                 len(cli['failed']) == len(gui['failed'])))

    # --- 3: escapes leave the package on both fronts ---
    print("\n[3] escape direction")
    for label, res in (("CLI", cli), ("GUI", gui)):
        out, inw = _outward_counts(res['tracks'], res['footprint'])
        r.append(_ok(f"{label}: all {out} matched escapes travel outward "
                     f"({inw} inward)", out > 0 and inw == 0))

    passed = sum(r)
    print(f"\n{passed}/{len(r)} rotated-QFN fanout parity checks passed")
    print("=" * 60)
    return 0 if passed == len(r) else 1


if __name__ == "__main__":
    sys.exit(main())
