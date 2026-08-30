#!/usr/bin/env python3
"""#733 follow-up, on the REAL headless dialog: the cap edge margin has its own
control, and the SHARED signal control cannot move it.

    python3 tests/gui_parity/test_733_cap_edge_control_real_dialog.py

(re-execs into KiCad's bundled python automatically, like its siblings)

The source-text half of this lives in tests/test_733_cap_edge_control_decoupled.py.
This half is the one that matters, because the defect it guards was invisible to
source text: PR743 read the dialog's shared "Min Edge Clearance (mm)" control --
the SIGNAL copper-to-edge keep-out -- for the CAP PLACEMENT margin. Both are
spelled `--board-edge-clearance` on the CLI, but on two independent tools
(route.py vs place_fanout_clearance.py) that can be set independently.

`resolve_cap_edge_clearance` honours an explicit positive value in BOTH
directions, so ticking the shared override at a normal signal 0.20-0.25 dropped
the cap margin from 0.55 to that value -- the 0.35mm relaxation #733 exists to
close, arriving through the operator rather than through the key. The zero case
was guarded; the realistic one was not.
"""
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

KICAD_PYTHONS = [
    '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/'
    'Versions/Current/bin/python3',
    '/usr/bin/python3',
    r'C:\Program Files\KiCad\10.0\bin\python.exe',
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with pcbnew+wx found")
    sys.exit(0)


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, os.path.join(REPO, 'py_placer'))
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))
    sys.path.insert(0, os.path.dirname(REPO))

    app = wx.App(False)  # noqa: F841
    board = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.swig_gui import RoutingDialog
    from placement.fanout_clearance import resolve_cap_edge_clearance

    dlg = RoutingDialog(None, parse_kicad_pcb(board), board)
    opts = dlg.fanout_tab.bga_options
    failures = []

    def check(name, cond, detail=""):
        if not cond:
            failures.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    # 1. The control exists on the cap panel and defaults to "engine resolves".
    check("the cap panel has its own cap_board_edge_clearance control",
          hasattr(opts, 'cap_board_edge_clearance'))
    if failures:
        print("\nFAILED: no control to test")
        return 1

    cfg = opts.get_config()
    check("default is None (0 in the spin = engine resolves)",
          cfg.get('cap_board_edge_clearance') is None,
          repr(cfg.get('cap_board_edge_clearance')))
    check("and the engine then resolves it to the 0.55 default",
          abs(resolve_cap_edge_clearance(board, cfg.get('cap_board_edge_clearance'))[0]
              - 0.55) < 1e-9)

    # 2. THE DECOUPLING. Tick the shared SIGNAL edge control at a realistic
    #    signal value; the cap margin must not follow it.
    dlg.edge_clearance_check.SetValue(True)
    dlg.board_edge_clearance.SetValue(0.20)
    cfg2 = opts.get_config()
    check("ticking the SHARED signal edge control leaves the cap margin unset",
          cfg2.get('cap_board_edge_clearance') is None,
          f"cap margin became {cfg2.get('cap_board_edge_clearance')!r} "
          f"when Min Edge Clearance was set to 0.20")
    check("so the cap margin still resolves to 0.55, not 0.20",
          abs(resolve_cap_edge_clearance(board, cfg2.get('cap_board_edge_clearance'))[0]
              - 0.55) < 1e-9)

    # 3. The operator CAN still set it -- on its own knob.
    opts.cap_board_edge_clearance.SetValue(0.80)
    cfg3 = opts.get_config()
    check("the cap knob itself sets the margin",
          cfg3.get('cap_board_edge_clearance') == 0.80,
          repr(cfg3.get('cap_board_edge_clearance')))
    check("and the engine honours it",
          abs(resolve_cap_edge_clearance(board, cfg3.get('cap_board_edge_clearance'))[0]
              - 0.80) < 1e-9)

    # 4. The SIGNAL keep-out still works and is a DIFFERENT number -- otherwise
    #    arm 2 could pass because the shared control does nothing at all.
    sig = dlg._effective_board_edge_clearance()
    check("negative control: the shared control still drives the SIGNAL margin",
          sig is not None and abs(sig - 0.20) < 1e-9,
          f"signal edge clearance reads {sig!r} (expected 0.20)")

    print()
    if failures:
        print(f"VERDICT: FAILED ({len(failures)}): " + ", ".join(failures))
        return 1
    print("VERDICT: cap edge margin is independent of the signal edge control")
    return 0


if __name__ == '__main__':
    sys.exit(main())
