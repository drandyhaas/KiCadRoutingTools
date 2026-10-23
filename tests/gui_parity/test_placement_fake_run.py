#!/usr/bin/env python3
"""Placement tab end-to-end against a FAKE claude CLI (no real agent).

Proves the whole runner -> monitor -> completion loop headless: the tab
launches the CLI with the placement contract (write-capable allowlist, one
--add-dir, staged board), streams the transcript, derives the progress line from
the converge ledger row a milestone writes, renders a live preview frame from a mid-run lap board,
parses the terminal RESULT= JSON, surfaces the REPORT.md button, and
re-arms its buttons. The fake CLI is a real .cmd shim on Windows, so the
npm-install path is under test end to end: the runner must strip the
quote-laden prompt from argv (cmd.exe mangles list2cmdline's \" escaping)
and hand it to the CLI on stdin.

Run: python3 tests/gui_parity/test_placement_fake_run.py
(re-execs into KiCad's bundled python automatically, like its siblings)
"""
import glob
import os
import subprocess
import sys
import tempfile
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
FAKE_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'fixtures', 'fake_claude.py')

# Every versioned install, newest first by NUMERIC version (a string sort
# puts KiCad\9.0 above KiCad\10.0).
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]    # this file orders its own sys.path further down
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
    sys.path.insert(0, os.path.dirname(REPO))

    app = wx.App(False)  # noqa: F841
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.ai_backend import BACKENDS
    from kicad_routing_plugin.placement_gui import PlacementTab

    try:
        import PIL  # noqa: F401
        pil_ok = True
    except ImportError:
        pil_ok = False

    # Work on a COPY of the in-repo board so krt_placement/ lands in a temp
    # dir, not in kicad_files/.
    tmp = tempfile.mkdtemp(prefix='krt_fake_run_')
    board = os.path.join(tmp, 'splitflap_driver.kicad_pcb')
    with open(os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb'),
              'rb') as src, open(board, 'wb') as dst:
        dst.write(src.read())
    pcb_data = parse_kicad_pcb(board)

    # Fake CLI: a REAL .cmd shim on Windows - this exercises the npm-shim
    # path end to end (AISkillRunner must strip the quote-laden prompt from
    # argv and pipe it on stdin, because cmd.exe mangles embedded quotes).
    # POSIX gets an executable sh shim; argv construction is the real
    # build_cmd either way.
    claude = BACKENDS['claude']
    if os.name == 'nt':
        shim = os.path.join(tmp, 'fake_claude.cmd')
        with open(shim, 'w') as f:
            f.write('@echo off\r\n"%s" "%s" %%*\r\n' % (sys.executable, FAKE_PY))
    else:
        shim = os.path.join(tmp, 'fake_claude.sh')
        with open(shim, 'w') as f:
            f.write('#!/bin/sh\nexec "%s" "%s" "$@"\n' % (sys.executable, FAKE_PY))
        os.chmod(shim, 0o755)
    claude.find_cli = lambda: shim

    frame = wx.Frame(None)
    logged = []
    tab = PlacementTab(frame, pcb_data, board,
                       append_log=lambda t: logged.append(t))

    failures = []

    def check(name, cond, detail=""):
        status = "ok" if cond else "FAIL"
        print(f"  [{status}] {name}" + (f" -- {detail}" if detail and not cond else ""))
        if not cond:
            failures.append(name)

    check("CLI gate: buttons enabled with fake CLI", tab.place_btn.IsEnabled())

    # Backend dropdown: every harness listed, unsupported picks revert.
    from kicad_routing_plugin.ai_backend import BACKEND_IDS
    check("backend dropdown lists all harnesses",
          tab.backend_choice.GetCount() == len(BACKEND_IDS))
    check("Claude selected by default",
          tab.backend_choice.GetSelection() == BACKEND_IDS.index('claude')
          and tab.get_backend_value() == 'claude')
    tab.backend_choice.SetSelection(BACKEND_IDS.index('opencode'))
    tab._on_backend_choice(None)
    check("unsupported backend reverts to Claude",
          tab.backend_choice.GetSelection() == BACKEND_IDS.index('claude')
          and tab.backend.id == 'claude'
          and "cannot drive placement yet"
          in tab.status_text.GetLabel().replace("\n", " "),
          tab.status_text.GetLabel())
    tab.set_backend_value('opencode')  # persistence restore path, same rule
    check("restore of unsupported backend reverts too",
          tab.get_backend_value() == 'claude')

    # Model/effort dropdowns: suggestions loaded, Default means no flag,
    # a chosen value must reach the CLI argv (asserted post-run below).
    check("model/effort suggestions loaded",
          tab.model_choice.GetCount() > 1 and tab.effort_choice.GetCount() > 1)
    check("Default resolves to no flag",
          tab.get_model_value() is None and tab.get_effort_value() is None)
    tab.model_choice.SetValue('opus')
    tab.effort_choice.SetValue('high')

    tab._start_ai_run("place")
    check("run accepted (runner live)", tab._runner is not None)
    workdir = tab.last_workdir
    check("workdir created", bool(workdir) and os.path.isdir(workdir), workdir)
    check("board staged",
          os.path.isfile(os.path.join(workdir, "input.kicad_pcb")))

    statuses = set()

    def pump_until(pred, timeout_s, grace_ticks=0):
        """Run a real MainLoop until pred() holds (+ grace ticks), or timeout.

        Yield()-pumping does not deliver wx.Timer events (the monitor's 1 s
        tick) outside a running loop on MSW - headless_plan.py runs
        PlanExecutor inside a real MainLoop for the same reason. Returns
        False on timeout."""
        state = {'timed_out': False, 'grace': None}
        timer = wx.Timer(frame)

        def _on_poll(event):
            # set_status wraps long lines with \n; normalize for substrings.
            statuses.add(tab.status_text.GetLabel().replace("\n", " "))
            if pred():
                if state['grace'] is None:
                    state['grace'] = grace_ticks
                else:
                    state['grace'] -= 1
                if state['grace'] <= 0:
                    app.ExitMainLoop()

        frame.Bind(wx.EVT_TIMER, _on_poll, timer)
        timer.Start(100)

        def _on_timeout():
            state['timed_out'] = True
            app.ExitMainLoop()

        watchdog = wx.CallLater(int(timeout_s * 1000), _on_timeout)
        app.MainLoop()
        timer.Stop()
        watchdog.Stop()
        return not state['timed_out']

    # grace: ~2 s extra so the final board's preview render can land.
    ok = pump_until(lambda: tab._runner is None, 90, grace_ticks=20)
    check("run finished before timeout", ok, str(statuses))

    transcript = tab.transcript_ctrl.GetValue()
    check("transcript flowed (init line)", "fake-model" in transcript)
    check("transcript shows the tool line (description preferred)",
          "Record the milestone board" in transcript, transcript[:400])
    # The run records milestones in the converge ledger; the newest LEDGER
    # ROW is the status line's last milestone, after the current activity.
    check("progress derived from the ledger row",
          any("lap 1: quench/nudge" in s for s in statuses), str(statuses))
    check("elapsed/mode in status",
          any(s.startswith("Place —") for s in statuses), str(statuses))
    # The candidates table: seed 0 from the run folder, kept at the end (the
    # final board is that seed's placement), selected, and previewed.
    rows = {r["seed"]: r for r in tab._cand_rows}
    check("candidates table filled from the run folder's seed files",
          0 in rows and rows[0]["vias"] == 12, str(list(rows)))
    check("the kept candidate is marked and selected",
          rows.get(0, {}).get("status") == "KEPT"
          and tab._sel_key == 0
          and tab.cand_list.GetFirstSelected() == 0, str(tab._sel_key))
    if pil_ok:
        png = os.path.join(workdir, "_gui_preview", "s0_fast.png")
        check("the selected row's fast-routed board rendered",
              os.path.isfile(png), png)
        check("the preview caption names the row",
              "seed 0" in tab.preview.caption.GetLabel(),
              tab.preview.caption.GetLabel())
    else:
        print("  [skip] preview render (Pillow not importable)")
    try:
        import json as _json
        with open(os.path.join(workdir, "argv.json")) as f:
            argv_seen = _json.load(f)
    except OSError:
        argv_seen = []
    check("model dropdown value reached the CLI",
          "--model" in argv_seen
          and argv_seen[argv_seen.index("--model") + 1] == "opus", str(argv_seen))
    check("effort dropdown value reached the CLI",
          "--effort" in argv_seen
          and argv_seen[argv_seen.index("--effort") + 1] == "high", str(argv_seen))
    check("RESULT parsed into pending result",
          tab._pending_result is not None
          and os.path.basename(tab._pending_result.get("board") or "")
          == "final.kicad_pcb", str(tab._pending_result))
    check("report surfaced in transcript", "=== REPORT" in transcript
          and "Fake placement report" in transcript)
    check("status shows finished + blocking",
          any("finished" in s and "blocking 0" in s for s in statuses),
          str(statuses))
    check("report button armed", tab.open_report_btn.IsEnabled())
    check("apply armed", tab.apply_btn.IsEnabled())
    check("buttons re-armed", tab.place_btn.IsEnabled()
          and tab.place_route_btn.IsEnabled() and tab.action_btn.IsEnabled())
    check("cancel back to Close", tab.cancel_btn.GetLabel() == "Close")

    # --- Beautify Labels through the tab: pins the engine's summary/result
    # contract against the GUI's completion handler (a field rename in
    # placement.labels must fail HERE, not silently in a user session).
    tab._on_action(None)
    if tab._routing_thread is None:
        check("beautify engine importable (#481 phase 1 present)", False,
              tab.status_text.GetLabel())
    else:
        check("beautify finished before timeout",
              pump_until(lambda: tab._routing_thread is None, 120))
        label = tab.status_text.GetLabel().replace("\n", " ")
        check("beautify summary contract", label.startswith("Labels:")
              and "unplaceable" in label, label)
        check("beautify applied 0 (no live board headless)",
              " 0 applied" in label, label)
        check("beautify re-armed", tab.action_btn.IsEnabled())

    # --- KEEP_RUNS (#1057): a second run through the tab prunes this
    # board's oldest run folders, keeps the newest, and says so in the log.
    # With KEEP_RUNS older runs beside the first run, exactly the oldest two
    # go: the new run plus the KEEP_RUNS - 1 newest others stay.
    import json as _json
    from kicad_routing_plugin.placement_run import KEEP_RUNS
    runs_root = os.path.dirname(workdir)
    check("krt_placement carries a .gitignore",
          os.path.isfile(os.path.join(runs_root, ".gitignore")))
    old_runs = []                                   # oldest first
    for i in range(1, KEEP_RUNS + 1):
        d = os.path.join(runs_root, f"20000101_{i:06d}_place")
        os.makedirs(os.path.join(d, "boards"))
        with open(os.path.join(d, ".krt_run.json"), "w") as f:
            _json.dump({"board": board, "mode": "place"}, f)
        old_runs.append(d)
    tab._start_ai_run("place")
    second = tab.last_workdir
    check("second run accepted", tab._runner is not None and second != workdir)
    check("candidates and outputs cleared for the new run",
          not tab._cand_rows and tab._report is None)
    check("second run finished before timeout",
          pump_until(lambda: tab._runner is None
                     and any("older run(s)" in t for t in logged), 90))
    check("KEEP_RUNS pruned the board's two oldest run folders",
          not any(os.path.exists(d) for d in old_runs[:2]))
    check("KEEP_RUNS kept the newest runs, the new one included",
          all(os.path.isdir(d) for d in old_runs[2:] + [workdir, second]))
    check("the prune is disclosed in the log",
          any("removed 2 older run(s)" in t for t in logged),
          "".join(t for t in logged if "Placement:" in t))

    # Deterministic wx teardown (headless_plan.py precedent: leaving pending
    # timers/windows to interpreter shutdown corrupts the heap and crashes
    # AFTER the verdict). Stop everything, destroy while the App lives, then
    # drain pending events.
    try:
        tab.shutdown()
        frame.Destroy()
        app.ProcessPendingEvents()
    except Exception:
        pass
    print()
    rc = 0
    if failures:
        print(f"VERDICT: {len(failures)} FAILURES: {failures}")
        rc = 1
    else:
        print("VERDICT: placement fake-run clean")
    # os._exit: even after the deterministic teardown above, this KiCad-python
    # + wx combination corrupts the heap during interpreter shutdown
    # (0xC0000374 AFTER the verdict, run threads all joined). The verdict is
    # printed and flushed; skip CPython finalization entirely so the exit
    # code reports the checks, not the teardown.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)


if __name__ == "__main__":
    sys.exit(main())
