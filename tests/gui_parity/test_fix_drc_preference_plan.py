#!/usr/bin/env python3
"""A plan run keeps the user's "Fix DRC settings after routing" choice.

The box is two things: a per-step plan parameter (a step replaying route.py's
--no-fix-drc-settings unticks it; the per-step reset re-ticks it -- the CLI's
per-step semantics) and the user's own saved preference (#693: unticked,
NOTHING may rewrite the board's DRC floors). A plan must not trade one for
the other, so the user's value is a VETO (unticked stays unticked for every
step and for the end-of-plan writeback) and is put back when the plan ends,
however it ends -- or settings_persistence saves the last step's value.

This drives the REAL PlanExecutor on the REAL headless RoutingDialog. Only
the step's action is stubbed -- it records the fix_drc_settings the route tab
would hand its engine, instead of routing -- and every end-of-plan floor
writer is spied. On ipc-migration that is ONE writer: the executor's
fix_project_for_output .kicad_pro write (kipy cannot write live design
settings, so main's live Board Setup and Default-netclass diff-pair writers do
not exist here, and the IPC tabs' per-step writer is apply_drc_settings_fix,
which the stubbed step never reaches). The board is a temp copy served by the
fake-IPC harness, so a spy that failed to intercept could not write into the
repository.

  1. user UNTICKED + a plan with no flag -> every step False, no floor
     written anywhere, the box still unticked after (and saved unticked);
  2. user TICKED + [flag, none, flag] -> [False, True, False] as before,
     the box ticked after; [flag, none] ends on a writing step, so the
     end-of-plan writers DO run -- the gate is not simply dead;
  3. Stop pressed during a flagged step -> the box is ticked again;
  4. loading a plan (the AI tab's reset + pre-fill) leaves the box as the
     user had it.

Run: python3 tests/gui_parity/test_fix_drc_preference_plan.py
(re-execs into KiCad's bundled python automatically, like its siblings)
"""
import glob
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/'
    'Versions/Current/bin/python3',
    '/usr/bin/python3',
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import wx, kipy'],
                          capture_output=True).returncode == 0:
            argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with wx + kipy found")
    sys.exit(77)


def main():
    try:
        import wx  # noqa: F401
        import kipy  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    for p in (REPO, os.path.join(REPO, 'py_router'),
              os.path.join(REPO, 'py_tools'),
              os.path.join(REPO, 'tests', 'stress'),
              os.path.join(REPO, 'tests', 'gui_parity')):
        sys.path.insert(0, p)
    app = wx.App(False)  # noqa: F841  (before any wx object)
    wx.Log.SetActiveTarget(wx.LogStderr())
    wx.MessageBox = lambda *a, **k: wx.OK

    # routing_dialog.py is this branch's swig_gui.py (renamed by the IPC port),
    # and the board comes through the fake-IPC harness like every ported gate.
    from kicad_routing_plugin.routing_dialog import RoutingDialog
    from kicad_routing_plugin import ai_plan, gui_utils
    from fake_ipc_board import (install as _install_fake_board,
                                build_pcb_data_like_ipc)
    from kicad_routing_plugin.settings_persistence import get_dialog_settings
    import fix_kicad_drc_settings
    import manifest_to_plan as m2p
    from wx_pump import run_until

    tmp = tempfile.mkdtemp(prefix='fixdrc_plan_')
    board_path = os.path.join(tmp, 'board.kicad_pcb')
    shutil.copy(os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb'),
                board_path)
    _install_fake_board(board_path)
    dlg = RoutingDialog(None, build_pcb_data_like_ipc(board_path), board_path)

    writes = []
    fix_kicad_drc_settings.fix_project_for_output = \
        lambda *a, **k: writes.append('project file floors')

    def step(*flags):
        s = m2p.parse_command(['python3', 'py_router/route.py', 'in.kicad_pcb',
                               'out.kicad_pcb', '--nets', '*', '--clearance',
                               '0.15', '--diff-pair-gap', '0.12', *flags])
        s.pop('_files', None)
        return s

    def run(steps, user, stop_at=None):
        """Run a plan; return (per-step fix_drc_settings, writers, box after,
        saved preference)."""
        dlg.fix_drc_check.SetValue(user)
        seen, done = [], []
        del writes[:]
        ex = ai_plan.PlanExecutor(dlg, steps, range(len(steps)),
                                  on_status=lambda i, s: None,
                                  on_finished=lambda n, r: done.append(r))
        ex.POLL_MS = 10

        def invoke():
            seen.append(dlg._build_routing_config(
                [], ['F.Cu', 'B.Cu'])['fix_drc_settings'])
            # Report a produced result, as the route tab's apply does: a step
            # that marks none stops the plan (ai_plan._step_refusal).
            gui_utils.mark_plan_step_produced(dlg)
            if stop_at is not None and len(seen) == stop_at:
                ex.stop()
        ex._action_parts = lambda action: (invoke, lambda: False)
        ex.start()
        run_until(lambda: bool(done), 120)
        return (seen, list(writes), dlg.fix_drc_check.GetValue(),
                get_dialog_settings(dlg)['fix_drc_settings'], done)

    flag = '--no-fix-drc-settings'
    results = []

    def check(label, ok, detail):
        results.append(ok)
        print(f"  {'PASS' if ok else 'FAIL'}  {label}: {detail}")

    print("\n[1] user UNTICKED, plan with no flag")
    seen, w, after, saved, done = run([step(), step()], user=False)
    check("every step runs with the writers off", seen == [False, False],
          f"per-step fix_drc_settings {seen}")
    check("no floor written anywhere", w == [], f"writers {w or 'none'}")
    check("box still unticked after, and saved unticked",
          after is False and saved is False, f"box {after}, saved {saved}")

    print("\n[2] user TICKED")
    seen, w, after, saved, done = run([step(flag), step(), step(flag)],
                                      user=True)
    check("[flag, none, flag] -> [False, True, False]",
          seen == [False, True, False], f"per-step {seen}")
    check("box ticked after, and saved ticked",
          after is True and saved is True, f"box {after}, saved {saved}")
    check("ends on a flagged step -> no end-of-plan write (as before)",
          w == [], f"writers {w or 'none'}")
    seen, w, after, saved, done = run([step(flag), step()], user=True)
    check("[flag, none] ends on a writing step -> the end-of-plan writers run",
          seen == [False, True] and 'project file floors' in w,
          f"per-step {seen}, writers {w}")

    print("\n[3] Stop pressed during a flagged step")
    seen, w, after, saved, done = run([step(flag), step(), step()],
                                      user=True, stop_at=1)
    check("the plan stopped after step 1", done == ['stopped by user']
          and seen == [False], f"finished {done}, per-step {seen}")
    check("the box is ticked again", after is True and saved is True,
          f"box {after}, saved {saved}")

    print("\n[4] loading a plan keeps the user's value")
    for user in (False, True):
        dlg.fix_drc_check.SetValue(user)
        dlg.ai_tab._install_plan_steps([step(flag), step()])
        after = dlg.fix_drc_check.GetValue()
        check(f"user {'ticked' if user else 'unticked'}: box after load",
              after is user, f"{after}")

    dlg.Destroy()
    shutil.rmtree(tmp, ignore_errors=True)
    n_ok = sum(results)
    print(f"\n{n_ok}/{len(results)} checks passed")
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
