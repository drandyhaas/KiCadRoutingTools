#!/usr/bin/env python3
"""A QFN fanout step's `escape_method` must reach the QFN panel (#860 follow-up).

THE DEFECT. `ai_plan._apply_special` handled `escape_method` by reaching
`dialog.fanout_tab.bga_options.escape_method_choice` unconditionally and
returning True. The two fanout panels do not model the escape the same way:

  * BGAOptionsPanel has a 4-way `wx.Choice` (`ESCAPE_METHODS`).
  * QFNOptionsPanel has NO dropdown -- it has a BOOLEAN `underpad_escape`
    checkbox, and its config emits 'underpad' when set, 'stub' when not.

So a QFN step set the BGA panel's dropdown, logged "set
escape_method=underpad", and left the QFN panel untouched. The under-pad
escape stayed OFF, which in turn made `allow_via_in_pad` inert -- it is
under-pad-only. A plan carrying `--escape-method underpad
--allow-via-in-pad` replayed a STUB fanout while reporting both params applied.

That flag stopped being cosmetic with #846: a via that OVERLAPS its pad is now
classified and clamped as one, so a replay that silently drops the under-pad
escape ships different copper from the run it is replaying.

WHY THE REAL DIALOG. The wx-free `test_manifest_plan_parity` gate cannot see
this: it checks that the flag survives conversion INTO the plan, and it does --
`--allow-via-in-pad` reaches the plan on the unknown-flag fallthrough even
unregistered (measured: removing its BOOL_FLAGS row leaves that gate green at
7400 checks, 0 mismatches). The break is downstream, in which CONTROL the
executor puts the value on, and only a real dialog has controls.

Run: python3 tests/gui_parity/test_860_qfn_escape_method_panel.py
(re-execs into KiCad's bundled python automatically, like its siblings)
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

CHECKS = []


def check(ok, label, detail=''):
    CHECKS.append((bool(ok), label, detail))
    print(f"  {'PASS' if ok else 'FAIL'}  {label}"
          + (f"   [{detail}]" if detail and not ok else ''))


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
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))
    sys.path.insert(0, os.path.join(REPO, 'tests', 'stress'))
    sys.path.insert(0, os.path.dirname(REPO))

    app = wx.App(False)  # noqa: F841  (before any wx object)
    board = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
    from kicad_parser import parse_kicad_pcb
    # routing_dialog is this branch's swig_gui (renamed by the IPC port).
    from kicad_routing_plugin.routing_dialog import RoutingDialog
    from kicad_routing_plugin.ai_plan import apply_step_params
    import manifest_to_plan

    dlg = RoutingDialog(None, parse_kicad_pcb(board), board)
    qfn = dlg.fanout_tab.qfn_options
    bga = dlg.fanout_tab.bga_options

    # --- 0. The rig must be able to detect the defect ------------------------
    # If the panels ever converge on one control shape this test is moot, and
    # it must say so rather than pass vacuously.
    check(getattr(qfn, 'underpad_escape', None) is not None,
          "PRECONDITION: QFN panel has an underpad_escape checkbox")
    check(getattr(qfn, 'escape_method_choice', None) is None,
          "PRECONDITION: QFN panel has NO escape_method dropdown "
          "(so the BGA control is the wrong target)")
    check(getattr(bga, 'escape_method_choice', None) is not None,
          "PRECONDITION: BGA panel has an escape_method dropdown")

    # --- 1. The CONVERTER carries both params, with kind=qfn -----------------
    step = manifest_to_plan.parse_command(
        ['python3', 'qfn_fanout.py', 'in.kicad_pcb', 'out.kicad_pcb',
         '--component', 'U2', '--escape-method', 'underpad',
         '--allow-via-in-pad'])
    check(step.get('kind') == 'qfn', "converter marks the step kind=qfn",
          f"kind={step.get('kind')!r}")
    check(step['params'].get('escape_method') == 'underpad',
          "converter carries escape_method=underpad")
    check(step['params'].get('allow_via_in_pad') is True,
          "converter carries allow_via_in_pad=True")

    # --- 2. A QFN step reaches the QFN panel, not the BGA one ----------------
    qfn.underpad_escape.SetValue(False)
    qfn.allow_via_in_pad.SetValue(False)
    bga.escape_method_choice.SetSelection(0)

    apply_step_params(step, dlg)

    check(qfn.underpad_escape.GetValue() is True,
          "QFN step SETS the QFN panel's underpad_escape",
          f"underpad_escape={qfn.underpad_escape.GetValue()}")
    check(qfn.allow_via_in_pad.GetValue() is True,
          "QFN step SETS the QFN panel's allow_via_in_pad",
          f"allow_via_in_pad={qfn.allow_via_in_pad.GetValue()}")
    # The regression itself: the BGA dropdown must be UNTOUCHED. Index 2 is
    # 'underpad' in ESCAPE_METHODS, which is what the old code selected.
    check(bga.escape_method_choice.GetSelection() == 0,
          "QFN step does NOT touch the BGA escape dropdown",
          f"bga selection={bga.escape_method_choice.GetSelection()} "
          f"(2 == the old bug writing 'underpad' onto the BGA panel)")

    # And the engine token the panel would emit is the one asked for.
    cfg = dlg.fanout_tab.qfn_options.get_config() \
        if hasattr(dlg.fanout_tab.qfn_options, 'get_config') else None
    if isinstance(cfg, dict) and 'escape_method' in cfg:
        check(cfg['escape_method'] == 'underpad',
              "the QFN panel now EMITS escape_method='underpad'",
              f"emitted {cfg.get('escape_method')!r}")

    # --- 3. 'stub' must turn the checkbox back OFF ---------------------------
    step_stub = manifest_to_plan.parse_command(
        ['python3', 'qfn_fanout.py', 'in.kicad_pcb', 'out.kicad_pcb',
         '--component', 'U2', '--escape-method', 'stub'])
    apply_step_params(step_stub, dlg)
    check(qfn.underpad_escape.GetValue() is False,
          "escape_method='stub' clears underpad_escape "
          "(the mapping is reversible, not set-only)")

    # --- 4. The BGA path is not broken by the fix ----------------------------
    bga_step = manifest_to_plan.parse_command(
        ['python3', 'bga_fanout.py', 'in.kicad_pcb', 'out.kicad_pcb',
         '--component', 'U1', '--escape-method', 'underpad'])
    check(bga_step.get('kind') == 'bga', "converter marks bga_fanout kind=bga")
    qfn.underpad_escape.SetValue(False)
    apply_step_params(bga_step, dlg)
    methods = type(bga).ESCAPE_METHODS
    check(bga.escape_method_choice.GetSelection() == methods.index('underpad'),
          "BGA step still SETS the BGA escape dropdown",
          f"selection={bga.escape_method_choice.GetSelection()}, "
          f"want {methods.index('underpad')}")
    check(qfn.underpad_escape.GetValue() is False,
          "BGA step does NOT touch the QFN checkbox")

    bad = [c for c in CHECKS if not c[0]]
    print(f"\n{len(CHECKS) - len(bad)}/{len(CHECKS)} checks passed")
    if bad:
        print("FAILED:")
        for _ok, label, detail in bad:
            print(f"  - {label}   [{detail}]")
        return 1
    print("OK -- a QFN fanout step's escape_method reaches the QFN panel")
    return 0


if __name__ == '__main__':
    sys.exit(main())
