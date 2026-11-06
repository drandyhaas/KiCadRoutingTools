#!/usr/bin/env python3
"""A plan step that produces no result STOPS the plan, as a CLI chain stops.

A CLI step that refuses, fails or is interrupted writes no output board, so
the next command fails on the missing file and nothing after it lays copper.
The GUI plan used to mark any step "done" once its tab went idle, so a
refused optimize_caps (#1067: an intent that does not load) or a declined
route step (its nets matched nothing -- route.py exits 1 there) let the plan
route on over the unchanged board (recorded open in the 108cac8b parity
entry). Now each tab marks the one place it applies a result, and the
executor stops on a step that ends without that mark.

Drives the REAL PlanExecutor on the REAL headless RoutingDialog. Only the
SECOND step of each plan is a spy (it records that it ran instead of
routing), and only where the case does not need it real:

  1. a REFUSED optimize_caps (real fanout tab, missing intent) then a step
     -> the second step never runs; the plan reports step 1 refused;
  2. a DECLINED route step (real route tab; its nets match nothing) then a
     step -> the second step never runs;
  3. a step whose tab marks nothing (fail closed) then a step -> stops;
  4. CONTROL: a real optimize_caps that runs, then a step -> both run and the
     plan completes, so the gate is not "every plan stops";
  5. CONTROL: a route step the plan's own plane-net safety net empties
     (wildcard GND before the planes) is a no-op, not a refusal -> the plan
     goes on, as it did before (the CLI would have routed those nets);
  6. each tab's REAL completion handler (route, route_diff, route_planes,
     fanout; only the apply call stubbed) marks a result on its success
     ending and on neither its error nor its cancel ending.

Run: python3 tests/gui_parity/test_plan_refused_step_stops.py
(re-execs into KiCad's bundled python automatically; exits 2 without it --
this gate does not self-skip)
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
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with pcbnew+wx found; this gate does not self-skip")
    sys.exit(2)


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    import pcbnew
    for p in (REPO, os.path.join(REPO, 'py_router'),
              os.path.join(REPO, 'py_tools'),
              os.path.join(REPO, 'tests', 'gui_parity')):
        sys.path.insert(0, p)
    app = wx.App(False)  # noqa: F841  (before any wx object)
    wx.Log.SetActiveTarget(wx.LogStderr())
    popups = []
    wx.MessageBox = lambda msg, *a, **k: popups.append(msg) or wx.OK

    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.swig_gui import RoutingDialog
    from kicad_routing_plugin import ai_plan, gui_utils
    from wx_pump import run_until

    tmp = tempfile.mkdtemp(prefix='plan_refused_')
    failures = []

    def check(label, ok, detail=""):
        print(f"  [{'ok' if ok else 'FAIL'}] {label}"
              + (f" -- {detail}" if detail and not ok else ""))
        if not ok:
            failures.append(label)

    try:
        board_path = os.path.join(tmp, 'board.kicad_pcb')
        shutil.copy(os.path.join(REPO, 'kicad_files',
                                 'splitflap_driver.kicad_pcb'), board_path)
        board = pcbnew.LoadBoard(board_path)       # in memory; never saved
        pcbnew.GetBoard = lambda: board
        dlg = RoutingDialog(None, parse_kicad_pcb(board_path), board_path)
        fanout_tab = dlg.fanout_tab

        def run(steps, indices=None, spy=None):
            """Run a plan; return (statuses, finished, ran, log)."""
            statuses, finished, ran, log = {}, [], [], []
            ex = ai_plan.PlanExecutor(
                dlg, steps, list(range(len(steps))) if indices is None
                else indices,
                on_status=lambda i, s: statuses.setdefault(i, []).append(s),
                on_finished=lambda n, r: finished.append((n, r)),
                log=log.append, quiet=True)
            ex.POLL_MS = 10
            real_parts = ex._action_parts
            if spy is not None:
                def parts(action, _real=real_parts):
                    if action != spy:
                        return _real(action)
                    owner = ex._action_owner(action)

                    def invoke():
                        ran.append(action)
                        # getattr: the spy must still RUN on a tree without
                        # the protocol, or the pre-fix evidence is a crash.
                        getattr(gui_utils, 'mark_plan_step_produced',
                                lambda o: None)(owner)
                    return invoke, (lambda: False)
                ex._action_parts = parts
            ex.start()
            run_until(lambda: bool(finished), 300)
            return statuses, finished, ran, '\n'.join(log)

        missing = os.path.join(tmp, 'missing.intent.json')
        route_step = {'action': 'route', 'nets': ['*'], 'params': {}}
        cap_step = {'action': 'optimize_caps', 'params': {}}

        print("\n[1] refused optimize_caps, then a route step")
        st, fin, ran, log = run(
            [{'action': 'optimize_caps',
              'params': {'cap_intent_path': missing, 'clearance': 0.2}},
             route_step], spy='route')
        check("the cap step really refused (real tab)",
              'Cap optimization NOT run' in log, log[-400:])
        check("the route step NEVER ran", ran == [] and 1 not in st,
              f"ran {ran}, statuses {st}")
        check("step 1 is marked failed", st.get(0, [])[-1:] == ['failed'],
              str(st))
        check("the plan ends with step 1 named as refused",
              fin and fin[0][1] and 'step 1' in fin[0][1]
              and 'refused' in fin[0][1], str(fin))
        print(f"      plan reports: {fin[0][1] if fin else None!r}")

        print("\n[2] declined route step (its nets match nothing), then a step")
        del popups[:]
        st, fin, ran, log = run(
            [{'action': 'route', 'nets': ['NO_SUCH_NET_*'], 'params': {}},
             cap_step], spy='optimize_caps')
        check("the route tab really declined (real tab, its own popup)",
              any('select at least one net' in str(p) for p in popups),
              str(popups))
        check("the next step NEVER ran", ran == [] and 1 not in st,
              f"ran {ran}, statuses {st}")
        check("the plan ends with step 1 named as refused",
              fin and fin[0][1] and 'step 1' in fin[0][1], str(fin))
        print(f"      plan reports: {fin[0][1] if fin else None!r}")

        print("\n[3] a step whose tab marks no result (fail closed)")
        def run_unmarked():
            statuses, finished, ran, log = {}, [], [], []
            ex = ai_plan.PlanExecutor(
                dlg, [route_step, cap_step], [0, 1],
                on_status=lambda i, s: statuses.setdefault(i, []).append(s),
                on_finished=lambda n, r: finished.append((n, r)),
                log=log.append, quiet=True)
            ex.POLL_MS = 10
            ex._action_parts = lambda action: (
                (lambda: ran.append(action)), (lambda: False))
            ex.start()
            run_until(lambda: bool(finished), 120)
            return statuses, finished, ran
        st, fin, ran = run_unmarked()
        check("only the unmarked step ran", ran == ['route'], f"ran {ran}")
        check("the plan stopped on it", fin and fin[0][1]
              and 'step 1' in fin[0][1], str(fin))

        print("\n[4] CONTROL: a real optimize_caps that runs, then a step")
        st, fin, ran, log = run([cap_step, route_step], spy='route')
        check("the cap step really ran (real tab)",
              'Cap optimization NOT run' not in log
              and 'step 1 finished' in log, log[-400:])
        check("the route step ran after it", ran == ['route'], f"ran {ran}")
        check("the plan completed (2 steps, no abort)",
              fin == [(2, None)], str(fin))

        print("\n[5] CONTROL: a plane-net-only route step before the planes")
        del popups[:]
        steps = [{'action': 'route', 'nets': ['GND*'], 'params': {}},
                 {'action': 'route_planes',
                  'assignments': [{'nets': ['GND'], 'layers': ['B.Cu']}]},
                 cap_step]
        st, fin, ran, log = run(steps, indices=[0, 2], spy='optimize_caps')
        check("the route step is a no-op, said so in the log",
              'runs nothing' in log, log[-400:])
        check("...with no 'select a net' popup", not popups, str(popups))
        check("the plan went on to the next step",
              ran == ['optimize_caps'] and fin == [(2, None)],
              f"ran {ran}, finished {fin}")

        print("\n[6] each tab marks its SUCCESS ending, never its error or "
              "cancel ending")
        # The real completion handlers, with only the apply call stubbed: the
        # question is where each tab's mark sits relative to its own early
        # returns, and a plan that ran no real fanout / diff route could not
        # otherwise see a misplaced one. (route + planes also run for real in
        # test_gui_engine_parity's chain.)
        attr = getattr(gui_utils, 'PLAN_STEP_PRODUCED', '_plan_step_produced')

        def ending(owner, setup):
            setattr(owner, attr, False)
            setup()
            return getattr(owner, attr, False) is True

        diff, planes = dlg.differential_tab, dlg.planes_tab
        noop_apply = (lambda *a, **k: (0, 0, 0))
        dlg._apply_results_to_board = noop_apply
        diff._apply_results_to_board = noop_apply
        planes._apply_results_to_board = noop_apply
        fanout_tab._apply_fanout_results = noop_apply

        def diff_end(result, cancel=False):
            diff._cancel_requested = cancel
            diff._routing_result = result
            diff._on_routing_complete()

        def planes_end(result, cancel=False):
            planes._cancel_requested = cancel
            planes._operation_result = result
            planes._on_operation_complete()

        def fanout_end(result, error=None, cancel=False):
            fanout_tab._cancel_requested = cancel
            fanout_tab._operation_error = error
            fanout_tab._operation_result = result
            fanout_tab._on_operation_complete({}, 'bga')

        ok_diff = {'successful': 0, 'failed': 0, 'total_time': 0.0,
                   'results_data': {}}
        ok_fan = {'tracks': [], 'vias': [], 'failed': [], 'skipped': []}
        rows = [
            ('route', 'success', True,
             lambda: dlg._routing_finished(dlg._apply_results_to_board)),
            ('route', 'error', False,
             lambda: dlg._routing_finished(dlg._routing_error, 'boom')),
            ('route', 'cancel', False,
             lambda: dlg._routing_finished(dlg._routing_cancelled)),
            ('route_diff', 'success', True, lambda: diff_end(ok_diff)),
            ('route_diff', 'error', False, lambda: diff_end({'error': 'boom'})),
            ('route_diff', 'cancel', False,
             lambda: diff_end(ok_diff, cancel=True)),
            ('route_planes', 'success', True,
             lambda: planes_end({'affected_nets': []})),
            ('route_planes', 'error', False,
             lambda: planes_end({'error': 'boom'})),
            ('route_planes', 'cancel', False,
             lambda: planes_end({'affected_nets': []}, cancel=True)),
            ('fanout', 'success', True, lambda: fanout_end(ok_fan)),
            ('fanout', 'error', False,
             lambda: fanout_end(None, error=RuntimeError('boom'))),
            ('fanout', 'cancel', False,
             lambda: fanout_end(ok_fan, cancel=True)),
        ]
        owners = {'route': dlg, 'route_diff': diff, 'route_planes': planes,
                  'fanout': fanout_tab}
        for action, how, want, setup in rows:
            got = ending(owners[action], setup)
            check(f"{action} {how} ending marks a result: {want}",
                  got is want, f"marked {got}")
        dlg.Destroy()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nVERDICT: FAIL ({len(failures)}): " + "; ".join(failures))
        return 1
    print("\nVERDICT: a step that produces no result stops the plan")
    return 0


if __name__ == '__main__':
    sys.exit(main())
