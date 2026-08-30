"""
KiCad Routing Tools - AI (Claude Code / opencode) integration

Runs an agent CLI headless to drive the project's AI skills from the GUI
(GitHub issues #40, #34, #39; configurable backend: #503).

Building blocks:
- ai_backend.AIBackend: which CLI, its argv, and its stream-event format
  (Claude Code by default; opencode as the configurable alternative)
- AISkillRunner: spawn the CLI, stream events to main-thread callbacks
- AISkillDialog: modal dialog that runs one skill with a live transcript
  and returns the machine-readable RESULT=<value> last line
- run_skill_dialog(): the one-call helper the other tabs use (CLI-missing
  guard + prompt phrasing + modal run)
- AITab: the AI tab in the routing dialog (backend/model/effort
  selection, planning, plan execution, review, diagnosis)
"""

import os
import json
import sys
import threading
import subprocess

import wx

from .ai_backend import (
    ANALYSIS_CONSTRAINT, BACKENDS, BACKEND_IDS, DEFAULT_BACKEND_ID,
    get_backend, extract_result_line,
)

PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)
if ROOT_DIR not in sys.path:      # top-level modules
    sys.path.insert(0, ROOT_DIR)

# #522 layout: engine/tool modules live in py_router/ (flat-install safe)
_ENGINE_DIR = os.path.join(ROOT_DIR, 'py_router')
if os.path.isdir(_ENGINE_DIR) and _ENGINE_DIR not in sys.path:
    sys.path.insert(0, _ENGINE_DIR)
# py_placer/ holds the placement package (placement.groups / .fanout_clearance
# are imported from here) and py_tools/ the instruments. Same exists() guard so
# a FLAT installed layout (PCM zip) keeps working.
for _sib in ('py_placer', 'py_tools'):
    _d = os.path.join(ROOT_DIR, _sib)
    if os.path.isdir(_d) and _d not in sys.path:
        sys.path.append(_d)

# "Default" combo entry = don't pass the model/effort flag at all.
DEFAULT_CHOICE = "Default"


def board_path_for_analysis(board_filename):
    """Return the board file path for a headless analysis run.

    Inside pcbnew the user edits an in-memory board that may be newer than
    the file on disk, and the routing dialog is modal so they can't save
    first. Saving over the editor's open file from the plugin crashed
    pcbnew at close (the frame's document state diverges), so instead the
    live board is snapshotted to a temp file and the analysis runs on
    that - the user's file is never touched and no prompt is needed.

    Returns the path to analyze, or None if the run should not proceed.
    """
    board = None
    try:
        import pcbnew
        board = pcbnew.GetBoard()
    except Exception:
        pass  # not running inside pcbnew (e.g. tests) - use the file as-is
    if board is not None:
        import tempfile
        base = os.path.basename(board_filename) if board_filename else "board.kicad_pcb"
        snapshot = os.path.join(tempfile.gettempdir(), f"kicadrt_analysis_{base}")
        try:
            # aSkipSettings: analysis snapshot; writing a .kicad_pro besides
            # would crash on pre-KiCad-10 projects (uncaught C++ type_error
            # in the .kicad_pro merge; see _stage_live_board in swig_gui.py).
            pcbnew.SaveBoard(snapshot, board, aSkipSettings=True)
            # Stage the on-disk project siblings beside the snapshot so the
            # analysis sees the real netclasses and layer rules (#498) --
            # the implicit settings save this replaced never carried the
            # .kicad_dru at all. The snapshot path is deterministic, so a
            # sibling left by an earlier snapshot of a DIFFERENT project
            # must be removed, not inherited.
            import shutil
            for ext in ('.kicad_pro', '.kicad_dru'):
                sib = (os.path.splitext(board_filename)[0] + ext
                       if board_filename else None)
                stale = os.path.splitext(snapshot)[0] + ext
                try:
                    if sib and os.path.isfile(sib):
                        shutil.copy(sib, stale)
                    elif os.path.isfile(stale):
                        os.remove(stale)
                except OSError:
                    pass
            return snapshot
        except Exception as e:
            wx.MessageBox(
                f"Could not snapshot the board for analysis: {e}\n\n"
                "Falling back to the last saved file - unsaved changes "
                "will not be visible to the analysis.",
                "AI", wx.OK | wx.ICON_WARNING)
    if not board_filename or not os.path.isfile(board_filename):
        wx.MessageBox(
            "Board file not found on disk. Save the board first so the "
            f"analysis sees the current state.\n\nLooked for: {board_filename}",
            "AI", wx.OK | wx.ICON_WARNING)
        return None
    return os.path.abspath(board_filename)


class AISkillRunner:
    """Runs the agent CLI headless on a background thread, streaming progress.

    Callbacks are invoked on the wx main thread:
      on_transcript(text)        - formatted transcript chunk to append
      on_done(result_text, error) - exactly one is non-None; cancel reports
                                    error="Cancelled."
    """

    def __init__(self, cli_path, on_transcript, on_done, backend=None,
                 on_event=None):
        self.backend = backend or BACKENDS[DEFAULT_BACKEND_ID]
        self.cli_path = cli_path  # the backend's CLI path
        self.on_transcript = on_transcript
        self.on_done = on_done
        # Optional raw-event tap (wx main thread): the parsed stream-json
        # dict BEFORE formatting/truncation. The Placement tab's monitor
        # derives its stage from full Bash commands this way - the formatted
        # transcript prefers the tool's short description and truncates.
        self.on_event = on_event
        self._process = None
        self._thread = None
        self._stream_state = None
        self._stdin_payload = None
        self._cancel_requested = False

    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    def run(self, prompt, model=None, effort=None, **cmd_kwargs):
        if self.is_running():
            raise RuntimeError(f"a {self.backend.label} run is already in progress")
        cmd = self.backend.build_cmd(self.cli_path, prompt,
                                     model=model, effort=effort, **cmd_kwargs)
        # npm installs resolve `claude` to a .cmd shim, which Windows launches
        # through an implicit cmd.exe -- and cmd's tokenizer does not
        # understand list2cmdline's \" escaping, so the first embedded quote
        # in the prompt ends the quoted region and any | in it becomes a
        # shell pipe. Claude Code reads the prompt from stdin in -p mode, so
        # hand it over that way and keep the argv quote-free.
        self._stdin_payload = None
        if os.name == "nt" and cmd and cmd[0].lower().endswith((".cmd", ".bat")):
            try:
                i = cmd.index("-p")
                if i + 1 < len(cmd) and cmd[i + 1] == prompt:
                    self._stdin_payload = cmd.pop(i + 1)
            except ValueError:
                pass
        self._stream_state = self.backend.stream_state()
        self._cancel_requested = False
        self._thread = threading.Thread(target=self._work, args=(cmd,), daemon=True)
        self._thread.start()

    def cancel(self):
        self._cancel_requested = True
        proc = self._process
        if proc is not None:
            try:
                proc.terminate()
            except OSError:
                pass

    def _work(self, cmd):
        state = self._stream_state
        try:
            kwargs = {}
            if os.name == "nt":
                kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW
            stdin_payload = self._stdin_payload
            self._process = subprocess.Popen(
                cmd,
                cwd=ROOT_DIR,  # skill discovery is working-directory based
                stdin=subprocess.PIPE if stdin_payload is not None else None,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                # utf-8, not the locale codec: the CLI emits UTF-8, and the
                # RESULT= payload can carry file paths that must round-trip.
                encoding="utf-8",
                errors="replace",
                bufsize=1,  # line-buffered: one stream-json event per line
                **kwargs,
            )
            if stdin_payload is not None:
                try:
                    self._process.stdin.write(stdin_payload)
                finally:
                    self._process.stdin.close()
            # Drain stderr concurrently so a chatty stderr can't fill its
            # pipe buffer and deadlock the stdout loop below.
            stderr_chunks = []
            stderr_thread = threading.Thread(
                target=lambda p: stderr_chunks.append(p.stderr.read()),
                args=(self._process,), daemon=True)
            stderr_thread.start()
            for line in self._process.stdout:
                line = line.strip()
                if not line:
                    continue
                try:
                    event = json.loads(line)
                except (json.JSONDecodeError, ValueError):
                    continue
                if self.on_event is not None:
                    wx.CallAfter(self.on_event, event)
                text = state.feed(event)
                if text:
                    wx.CallAfter(self.on_transcript, text)
            self._process.wait()
            stderr_thread.join(timeout=5)
            stderr = "".join(stderr_chunks)
            returncode = self._process.returncode
        except Exception as e:
            wx.CallAfter(self.on_done, None,
                         f"Failed to launch {self.backend.cli_name}: {e}")
            return
        finally:
            self._process = None
        wx.CallAfter(self._finish, state, stderr, returncode)

    def _finish(self, state, stderr, returncode):
        if self._cancel_requested:
            self.on_done(None, "Cancelled.")
            return
        result_text, error = state.finish(returncode, stderr)
        self.on_done(result_text, error)


class AISkillDialog(wx.Dialog):
    """Modal dialog that runs one AI skill and shows the live transcript.

    After ShowModal() returns, `result_value` holds the parsed RESULT=<value>
    string (None if the run failed, was cancelled, or had no RESULT= line),
    and `result_text` holds the AI's full final reply.
    """

    def __init__(self, parent, title, prompt, cli_path=None,
                 intro=None, model=None, effort=None, backend=None):
        super().__init__(parent, title=title, size=(720, 520),
                         style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self._backend = backend or BACKENDS[DEFAULT_BACKEND_ID]
        self.result_value = None
        self.result_text = None
        self._elapsed_seconds = 0
        self._done = False

        sizer = wx.BoxSizer(wx.VERTICAL)
        self.output_ctrl = wx.TextCtrl(
            self, style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_RICH2)
        self.output_ctrl.SetFont(
            wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        if intro:
            self.output_ctrl.SetValue(intro + "\n\n")
        sizer.Add(self.output_ctrl, 1, wx.EXPAND | wx.ALL, 8)

        self.gauge = wx.Gauge(self, range=100)
        sizer.Add(self.gauge, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 8)

        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.elapsed_label = wx.StaticText(self, label="0s")
        btn_sizer.Add(self.elapsed_label, 1, wx.ALIGN_CENTER_VERTICAL)
        self.action_btn = wx.Button(self, label="Cancel")
        self.action_btn.Bind(wx.EVT_BUTTON, self._on_action)
        btn_sizer.Add(self.action_btn, 0)
        sizer.Add(btn_sizer, 0, wx.EXPAND | wx.ALL, 8)
        self.SetSizer(sizer)

        self.Bind(wx.EVT_CLOSE, self._on_close)
        self._elapsed_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_elapsed_tick, self._elapsed_timer)
        self._elapsed_timer.Start(1000)

        self._runner = AISkillRunner(
            cli_path or self._backend.find_cli(), self._append, self._on_done,
            backend=self._backend)
        self._runner.run(prompt, model=model, effort=effort)

    def _append(self, text):
        if not self:  # dialog destroyed while the run streamed
            return
        self.output_ctrl.AppendText(text)

    def _on_done(self, result_text, error):
        if not self:
            return
        self._done = True
        self._elapsed_timer.Stop()
        self.gauge.SetValue(0)
        self.action_btn.SetLabel("Close")
        if error:
            hint = self._backend.auth_hint(error)
            if self.output_ctrl.GetValue().rstrip().endswith(error.strip()):
                appended = hint
            else:
                appended = f"\n{error}{hint}"
            if appended:
                self.output_ctrl.AppendText(appended + "\n")
            return
        self.result_text = result_text
        self.result_value = extract_result_line(result_text)
        self.output_ctrl.AppendText(
            f"\n--- done in {self.elapsed_label.GetLabel()}"
            + (f" | RESULT={self.result_value}" if self.result_value is not None
               else " | no RESULT= line found")
            + " ---\n")

    def _on_action(self, event):
        if self._done:
            self.EndModal(wx.ID_OK)
        else:
            self._runner.cancel()
            self.action_btn.Disable()

    def _on_close(self, event):
        if not self._done:
            self._runner.cancel()
        self.EndModal(wx.ID_CANCEL)

    def _on_elapsed_tick(self, event):
        if not self:
            return
        self._elapsed_seconds += 1
        mins, secs = divmod(self._elapsed_seconds, 60)
        self.elapsed_label.SetLabel(f"{mins}m {secs:02d}s" if mins else f"{secs}s")
        self.gauge.Pulse()


def run_skill_dialog(parent, title, skill, skill_args, instructions, intro,
                     ai_params=None):
    """Run one skill modally and return its parsed RESULT= value (or None).

    The one-call helper for the other tabs' "Ask AI" buttons: resolves the
    backend selected on the AI tab (ai_params from get_ai_params),
    guards the CLI-not-installed case with a message box, phrases the skill
    invocation for that backend, and shows the live-transcript dialog.
    """
    params = ai_params or {}
    backend = get_backend(params.get('backend') or DEFAULT_BACKEND_ID)
    cli_path = backend.find_cli()
    if cli_path is None:
        wx.MessageBox(backend.not_found_message(), "AI",
                      wx.OK | wx.ICON_WARNING)
        return None
    prompt = backend.skill_prompt(skill, skill_args, instructions)
    dlg = AISkillDialog(
        parent, title, prompt, cli_path=cli_path, backend=backend,
        model=params.get('model'), effort=params.get('effort'), intro=intro)
    dlg.ShowModal()
    value = dlg.result_value
    dlg.Destroy()
    return value


class AITab(wx.Panel):
    """AI tab: run AI skills headless and bring results into the GUI."""

    def __init__(self, parent, board_filename, log_callback=None, routing_dialog=None):
        super().__init__(parent)
        self.board_filename = board_filename
        self.log_callback = log_callback
        self.routing_dialog = routing_dialog
        self._elapsed_timer = wx.Timer(self)
        self._elapsed_seconds = 0
        self.Bind(wx.EVT_TIMER, self._on_elapsed_tick, self._elapsed_timer)
        self._runner = None  # created per run for the selected backend
        self._pending_kind = None  # 'test' or 'plan' - what the runner is doing
        self._plan_steps = []
        self._plan_executor = None
        # Model/effort entries are remembered per backend so switching back
        # and forth doesn't lose e.g. an opencode provider/model string.
        self._backend_params = {bid: {'model': DEFAULT_CHOICE,
                                      'effort': DEFAULT_CHOICE}
                                for bid in BACKEND_IDS}
        self._create_ui()
        # The runner streams via wx.CallAfter from a background thread; if the
        # dialog is destroyed mid-run those callbacks would land on dead
        # widgets (crashing pcbnew). Shut the run down with the window.
        self.Bind(wx.EVT_WINDOW_DESTROY, self._on_destroy)

    def _on_destroy(self, event):
        if event.GetEventObject() is self:
            self.shutdown()
        event.Skip()

    def shutdown(self):
        """Stop background work before the dialog goes away: cancel the
        agent CLI subprocess, stop the plan executor and the elapsed timer."""
        if self._runner is not None:
            self._runner.cancel()
        if self._plan_executor is not None:
            self._plan_executor.stop()
        self._elapsed_timer.Stop()

    def _create_ui(self):
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Top area: planned steps on the left, controls box on the right
        top_sizer = wx.BoxSizer(wx.HORIZONTAL)

        steps_box = wx.StaticBox(self, label="Planned Steps")
        steps_sizer = wx.StaticBoxSizer(steps_box, wx.VERTICAL)
        self.plan_list = wx.CheckListBox(self, choices=[])
        self.plan_list.SetToolTip(
            "The routing plan from the AI. Check the steps to run; review or "
            "tweak each step's parameters on its own tab before running.")
        steps_sizer.Add(self.plan_list, 1, wx.EXPAND | wx.ALL, 3)
        top_sizer.Add(steps_sizer, 1, wx.EXPAND | wx.RIGHT, 8)

        ctrl_box = wx.StaticBox(self, label="AI")
        ctrl_sizer = wx.StaticBoxSizer(ctrl_box, wx.VERTICAL)

        # Availability status (kept current by _refresh_backend_ui)
        self.status_label = wx.StaticText(self, label="")
        self.status_label.Wrap(280)
        ctrl_sizer.Add(self.status_label, 0, wx.ALL, 5)

        # Backend / model / effort selection
        sel_grid = wx.FlexGridSizer(cols=2, hgap=5, vgap=5)
        sel_grid.AddGrowableCol(1)
        sel_grid.Add(wx.StaticText(self, label="Backend:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.backend_choice = wx.Choice(
            self, choices=[BACKENDS[bid].label for bid in BACKEND_IDS])
        self.backend_choice.SetSelection(BACKEND_IDS.index(DEFAULT_BACKEND_ID))
        self.backend_choice.SetToolTip(
            "Agent CLI used for all AI features (this tab and the other tabs' "
            "'Ask AI' buttons). Claude Code runs Anthropic models; opencode "
            "(https://opencode.ai) supports many model providers. Both use the "
            "same skills (.claude/skills/) and need their CLI installed and "
            "logged in.")
        self.backend_choice.Bind(wx.EVT_CHOICE, self._on_backend_change)
        sel_grid.Add(self.backend_choice, 0, wx.EXPAND)
        sel_grid.Add(wx.StaticText(self, label="Model:"), 0, wx.ALIGN_CENTER_VERTICAL)
        # Editable: Claude Code takes tier aliases (suggestions), opencode
        # takes free-form provider/model strings.
        self.model_choice = wx.ComboBox(self, value=DEFAULT_CHOICE, choices=[])
        sel_grid.Add(self.model_choice, 0, wx.EXPAND)
        sel_grid.Add(wx.StaticText(self, label="Effort:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.effort_choice = wx.ComboBox(self, value=DEFAULT_CHOICE, choices=[])
        sel_grid.Add(self.effort_choice, 0, wx.EXPAND)
        ctrl_sizer.Add(sel_grid, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # Visual break between the model/effort selectors and the actions
        ctrl_sizer.AddSpacer(6)
        ctrl_sizer.Add(wx.StaticLine(self), 0,
                       wx.EXPAND | wx.LEFT | wx.RIGHT, 5)
        ctrl_sizer.AddSpacer(6)

        # Planning: the headline action with its Cancel right under it
        self.plan_btn = wx.Button(self, label="Plan Routing")
        self.plan_btn.SetToolTip(
            "Run the plan-pcb-routing skill headless on the current board. The "
            "plan fills the tabs' parameter fields and appears in the step list "
            "you can review, edit (in each tab), select, and run.")
        self.plan_btn.Bind(wx.EVT_BUTTON, self._on_plan)
        ctrl_sizer.Add(self.plan_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.cancel_btn = wx.Button(self, label="Cancel")
        self.cancel_btn.SetToolTip("Cancel the running AI analysis")
        self.cancel_btn.Bind(wx.EVT_BUTTON, self._on_cancel)
        self.cancel_btn.Disable()
        ctrl_sizer.Add(self.cancel_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        ctrl_sizer.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # Execution: run/stop the planned steps
        self.run_plan_btn = wx.Button(self, label="Run Selected Steps")
        self.run_plan_btn.SetToolTip(
            "Execute the checked steps in order through the tabs' own routing "
            "machinery, updating the list with check marks as each completes.")
        self.run_plan_btn.Bind(wx.EVT_BUTTON, self._on_run_selected)
        self.run_plan_btn.Disable()
        ctrl_sizer.Add(self.run_plan_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # Same as above but unattended: no per-step "Routing Complete" OK popup.
        self.run_all_plan_btn = wx.Button(self, label="Run All Selected Steps")
        self.run_all_plan_btn.SetToolTip(
            "Run the checked steps end-to-end WITHOUT the per-step completion "
            "popup you normally have to OK. Each step's summary is printed to "
            "the output/log instead.")
        self.run_all_plan_btn.Bind(wx.EVT_BUTTON, self._on_run_all_selected)
        self.run_all_plan_btn.Disable()
        ctrl_sizer.Add(self.run_all_plan_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.stop_plan_btn = wx.Button(self, label="Stop")
        self.stop_plan_btn.SetToolTip(
            "Cancel the currently running step (it aborts at its next safe "
            "point and its partial results are discarded) and stop the plan; "
            "remaining steps are not run")
        self.stop_plan_btn.Bind(wx.EVT_BUTTON, self._on_stop_plan)
        self.stop_plan_btn.Disable()
        ctrl_sizer.Add(self.stop_plan_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        ctrl_sizer.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # Post-route skills: QA sign-off and failure diagnosis
        self.review_btn = wx.Button(self, label="Review Routed Board")
        self.review_btn.SetToolTip(
            "Run the review-routed-board skill: DRC, connectivity, and orphan-stub "
            "checks, match-group and GND-via coverage review. The report shows in "
            "the transcript with a PASS/FAIL verdict.")
        self.review_btn.Bind(wx.EVT_BUTTON, self._on_review)
        ctrl_sizer.Add(self.review_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.diagnose_btn = wx.Button(self, label="Diagnose Routing Failures")
        self.diagnose_btn.SetToolTip(
            "Run the diagnose-routing-failures skill on the board plus this "
            "session's Log tab content: root-causes failed routes and recommends "
            "a targeted retry. Run a routing operation first.")
        self.diagnose_btn.Bind(wx.EVT_BUTTON, self._on_diagnose)
        ctrl_sizer.Add(self.diagnose_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        ctrl_sizer.AddStretchSpacer(1)

        # Close pinned at the column bottom (same behavior as the other
        # tabs' Close buttons).
        ctrl_sizer.AddSpacer(6)
        ctrl_sizer.Add(wx.StaticLine(self), 0,
                       wx.EXPAND | wx.LEFT | wx.RIGHT, 5)
        ctrl_sizer.AddSpacer(6)
        self.close_btn = wx.Button(self, label="Close")
        self.close_btn.SetToolTip("Close dialog")
        self.close_btn.Bind(wx.EVT_BUTTON, self._on_close)
        ctrl_sizer.Add(self.close_btn, 0,
                       wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        top_sizer.Add(ctrl_sizer, 0, wx.EXPAND)

        sizer.Add(top_sizer, 1, wx.EXPAND | wx.ALL, 8)

        # Status bar: elapsed/step text + progress gauge, full width between
        # the Planned Steps box and the parsed-result line -- it mirrors the
        # working tab's status during plan steps, so give it room.
        activity_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.elapsed_label = wx.StaticText(self, label="")
        activity_sizer.Add(self.elapsed_label, 0,
                           wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 16)
        self.gauge = wx.Gauge(self, range=100)
        activity_sizer.Add(self.gauge, 1, wx.EXPAND)
        self._activity_sizer = activity_sizer
        sizer.Add(activity_sizer, 0,
                  wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)

        # Parsed machine-readable result (RESULT= line of the last run)
        parsed_sizer = wx.BoxSizer(wx.HORIZONTAL)
        parsed_sizer.Add(wx.StaticText(self, label="Parsed result:"), 0,
                         wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.parsed_ctrl = wx.TextCtrl(self, style=wx.TE_READONLY)
        self.parsed_ctrl.SetToolTip(
            "The machine-readable last line of the AI's reply (RESULT=...), "
            "demonstrating how skill output will populate GUI fields.")
        parsed_sizer.Add(self.parsed_ctrl, 1, wx.EXPAND)
        self.save_plan_btn = wx.Button(self, label="Save...")
        self.save_plan_btn.SetToolTip(
            "Save the current plan steps to a JSON file (replay later with "
            "Load - no AI run needed)")
        self.save_plan_btn.Bind(wx.EVT_BUTTON, self._on_save_plan)
        parsed_sizer.Add(self.save_plan_btn, 0, wx.LEFT, 5)
        self.load_plan_btn = wx.Button(self, label="Load...")
        self.load_plan_btn.SetToolTip(
            "Load plan steps from a JSON file (a saved plan, or one exported "
            "by tests/stress/manifest_to_plan.py from a stress-test manifest)")
        self.load_plan_btn.Bind(wx.EVT_BUTTON, self._on_load_plan)
        parsed_sizer.Add(self.load_plan_btn, 0, wx.LEFT, 5)
        sizer.Add(parsed_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)

        # Full output across the whole width at the bottom
        self.output_ctrl = wx.TextCtrl(
            self, style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_RICH2)
        self.output_ctrl.SetFont(
            wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        sizer.Add(self.output_ctrl, 1, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)

        self.SetSizer(sizer)
        self._refresh_backend_ui()

    # ---------------------------------------------- backend / model / effort

    def _current_backend(self):
        return BACKENDS[BACKEND_IDS[self.backend_choice.GetSelection()]]

    def _on_backend_change(self, event):
        # Remember this backend's entries before switching the combos over.
        for bid, params in self._backend_params.items():
            if BACKENDS[bid] is self._last_backend:
                params['model'] = self.model_choice.GetValue()
                params['effort'] = self.effort_choice.GetValue()
        self._refresh_backend_ui()

    def _refresh_backend_ui(self):
        """Point the status label, combo suggestions, tooltips, and button
        enablement at the selected backend."""
        backend = self._current_backend()
        self._last_backend = backend
        cli_path = backend.find_cli()
        if cli_path:
            self.status_label.SetLabel(f"{backend.label} CLI found: {cli_path}")
        else:
            self.status_label.SetLabel(
                backend.not_found_message() + " Then reopen this dialog.")
        self.status_label.Wrap(280)
        params = self._backend_params[backend.id]
        self.model_choice.Set(list(backend.model_suggestions))
        self.model_choice.SetValue(params['model'] or DEFAULT_CHOICE)
        self.model_choice.SetToolTip(backend.model_tooltip)
        self.effort_choice.Set(list(backend.effort_suggestions))
        self.effort_choice.SetValue(params['effort'] or DEFAULT_CHOICE)
        self.effort_choice.SetToolTip(backend.effort_tooltip)
        available = cli_path is not None
        self.plan_btn.Enable(available and self.routing_dialog is not None)
        self.review_btn.Enable(available)
        self.diagnose_btn.Enable(available)
        self.Layout()

    def get_backend_value(self):
        """The selected backend id (settings key, e.g. 'claude')."""
        return self._current_backend().id

    def set_backend_value(self, value):
        """Select the saved backend; unknown ids revert to the default."""
        bid = value if value in BACKEND_IDS else DEFAULT_BACKEND_ID
        self.backend_choice.SetSelection(BACKEND_IDS.index(bid))
        self._refresh_backend_ui()

    @staticmethod
    def _combo_value(ctrl):
        value = ctrl.GetValue().strip()
        return None if not value or value == DEFAULT_CHOICE else value

    def get_model_value(self):
        """The model flag value for the selected backend (None = CLI default)."""
        return self._combo_value(self.model_choice)

    def set_model_value(self, value, backend_id=None):
        """Restore a saved model entry (for the given backend, default current)."""
        self._set_backend_param('model', self.model_choice, value, backend_id)

    def get_effort_value(self):
        """The effort/variant flag value (None = CLI default)."""
        return self._combo_value(self.effort_choice)

    def set_effort_value(self, value, backend_id=None):
        """Restore a saved effort entry (for the given backend, default current)."""
        self._set_backend_param('effort', self.effort_choice, value, backend_id)

    def _set_backend_param(self, key, ctrl, value, backend_id):
        value = value or DEFAULT_CHOICE
        if backend_id is None or backend_id == self._current_backend().id:
            ctrl.SetValue(value)
        if backend_id is None:
            backend_id = self._current_backend().id
        if backend_id in self._backend_params:
            self._backend_params[backend_id][key] = value

    def _get_backend_param(self, key, ctrl, backend_id):
        if backend_id == self._current_backend().id:
            return self._combo_value(ctrl)
        value = self._backend_params.get(backend_id, {}).get(key)
        return None if not value or value == DEFAULT_CHOICE else value

    def get_model_value_for(self, backend_id):
        """The remembered model entry for a backend (settings persistence)."""
        return self._get_backend_param('model', self.model_choice, backend_id)

    def get_effort_value_for(self, backend_id):
        """The remembered effort entry for a backend (settings persistence)."""
        return self._get_backend_param('effort', self.effort_choice, backend_id)

    # ---------------------------------------------------------- persistence

    def get_plan_state(self):
        """Transcript + plan list state for settings persistence."""
        return {
            'output': self.output_ctrl.GetValue(),
            'steps': self._plan_steps,
            'items': [self.plan_list.GetString(i)
                      for i in range(self.plan_list.GetCount())],
            'checked': list(self.plan_list.GetCheckedItems()),
        }

    def restore_plan_state(self, state):
        """Restore a previously saved transcript and plan list."""
        if not isinstance(state, dict):
            return
        if state.get('output'):
            self.output_ctrl.SetValue(state['output'])
        steps = state.get('steps') or []
        items = state.get('items') or []
        if steps and len(items) == len(steps):
            self._plan_steps = steps
            self.plan_list.Set(items)
            self.plan_list.SetCheckedItems(
                [i for i in state.get('checked', []) if 0 <= i < len(steps)])
            # Replaying a restored plan is a no-LLM operation: it does not
            # need any agent CLI installed, only the routing dialog.
            self.run_plan_btn.Enable(self.routing_dialog is not None)
            self.run_all_plan_btn.Enable(self.run_plan_btn.IsEnabled())

    # ------------------------------------------------------------------ run

    def _board_path_or_warn(self):
        return board_path_for_analysis(self.board_filename)

    def _start_run(self, skill, skill_args, instructions, kind, intro):
        """Start a headless skill run (kind names what the result is for)
        on the selected backend, with shared UI state."""
        backend = self._current_backend()
        cli_path = backend.find_cli()
        if cli_path is None:
            wx.MessageBox(backend.not_found_message(), "AI",
                          wx.OK | wx.ICON_WARNING)
            return
        prompt = backend.skill_prompt(skill, skill_args, instructions)
        self._pending_kind = kind
        self.plan_btn.Disable()
        self.review_btn.Disable()
        self.diagnose_btn.Disable()
        self.run_plan_btn.Disable()
        self.run_all_plan_btn.Disable()
        self.cancel_btn.Enable()
        self.parsed_ctrl.SetValue("")
        # The transcript and step list persist across runs and dialog reopens;
        # only a fresh plan run clears them. Other runs append.
        if kind == "plan":
            self._plan_steps = []
            self.plan_list.Set([])
            self.output_ctrl.SetValue(intro + "\n\n")
        else:
            if self.output_ctrl.GetValue().strip():
                self.output_ctrl.AppendText("\n" + "=" * 60 + "\n\n")
            self.output_ctrl.AppendText(intro + "\n\n")
        self._elapsed_seconds = 0
        self.elapsed_label.SetLabel("0s")
        self._elapsed_timer.Start(1000)
        model = self.get_model_value()
        effort = self.get_effort_value()
        self._log(f"{backend.label}: {intro.splitlines()[0]}"
                  + (f" | model={model}" if model else "")
                  + (f" | effort={effort}" if effort else ""))
        self._runner = AISkillRunner(
            cli_path, self._append_transcript, self._on_done, backend=backend)
        self._runner.run(prompt, model=model, effort=effort)

    def _on_review(self, event):
        if self._runner is not None and self._runner.is_running():
            return
        board = self._board_path_or_warn()
        if board is None:
            return
        self._start_run(
            "review-routed-board", board,
            ANALYSIS_CONSTRAINT
            + " After the report, end your reply with exactly one line of the "
              "form RESULT=PASS or RESULT=FAIL (the overall sign-off verdict)",
            "review",
            f"Running review-routed-board on {os.path.basename(board)} ...\n"
            "(DRC + connectivity checkers + review; typically a few minutes)")

    def _on_diagnose(self, event):
        if self._runner is not None and self._runner.is_running():
            return
        board = self._board_path_or_warn()
        if board is None:
            return
        log_text = ""
        if self.routing_dialog is not None and hasattr(self.routing_dialog, "log_text"):
            log_text = self.routing_dialog.log_text.GetValue()
        if not log_text.strip():
            wx.MessageBox(
                "The Log tab is empty. Run a routing operation first so there "
                "is a log to diagnose.", "AI", wx.OK | wx.ICON_WARNING)
            return
        import tempfile
        fd, log_path = tempfile.mkstemp(prefix="kicadrt_gui_log_", suffix=".txt")
        with os.fdopen(fd, "w", encoding="utf-8", errors="replace") as f:
            f.write(log_text)
        self._start_run(
            "diagnose-routing-failures", board,
            ANALYSIS_CONSTRAINT
            + f" The routing log from this GUI session is at {log_path}."
            + " After the report, end your reply with exactly one line of the "
              "form RESULT=<one-line recommended fix, or 'no failures "
              "found'>",
            "diagnose",
            f"Running diagnose-routing-failures on {os.path.basename(board)} "
            "+ the Log tab content ...\n(log analysis; typically a few minutes)")

    def _on_plan(self, event):
        if self._runner is not None and self._runner.is_running():
            return
        if self._plan_executor is not None:
            wx.MessageBox("A plan is currently executing. Stop it first.",
                          "AI", wx.OK | wx.ICON_WARNING)
            return
        board = self._board_path_or_warn()
        if board is None:
            return
        from .ai_plan import PLAN_RESULT_SCHEMA
        self._start_run(
            "plan-pcb-routing", board,
            ANALYSIS_CONSTRAINT
            + " After the report, end your reply with exactly one line of the "
              f"form {PLAN_RESULT_SCHEMA}",
            "plan",
            f"Running plan-pcb-routing on {os.path.basename(board)} ...\n"
            "(board analysis + datasheet lookups; typically several minutes)")

    def _append_transcript(self, text):
        if not self:  # dialog destroyed while the run streamed
            return
        self.output_ctrl.AppendText(text)

    def _on_done(self, result_text, error):
        if not self:  # dialog destroyed while the run finished
            return
        self._elapsed_timer.Stop()
        self.gauge.SetValue(0)
        self.plan_btn.Enable(self.routing_dialog is not None)
        self.review_btn.Enable()
        self.diagnose_btn.Enable()
        self.run_plan_btn.Enable(bool(self._plan_steps))
        self.run_all_plan_btn.Enable(self.run_plan_btn.IsEnabled())
        self.cancel_btn.Disable()
        kind, self._pending_kind = self._pending_kind, None

        if error:
            # The CLI often streams the failure text (e.g. 'Not logged in')
            # as an assistant message before the error result repeats it -
            # don't print the same line twice.
            backend = self._runner.backend if self._runner else self._current_backend()
            hint = backend.auth_hint(error)
            if self.output_ctrl.GetValue().rstrip().endswith(error.strip()):
                appended = hint
            else:
                appended = f"\n{error}{hint}"
            if appended:
                self.output_ctrl.AppendText(appended + "\n")
            self._log(f"{backend.label}: {error}")
            return

        # The streamed transcript already shows the full report; just close out.
        self.output_ctrl.AppendText(f"\n--- done in {self.elapsed_label.GetLabel()} ---\n")
        parsed = extract_result_line(result_text)
        if parsed is not None:
            self.parsed_ctrl.SetValue(parsed if len(parsed) < 200 else parsed[:200] + "...")
            self._log(f"AI: done in {self._elapsed_seconds}s")
        else:
            self.parsed_ctrl.SetValue("(no RESULT= line found)")
            self._log(f"AI: done in {self._elapsed_seconds}s, no RESULT= line")
        if kind == "plan":
            self._handle_plan_result(parsed)

    # ------------------------------------------------------------- planning

    def _handle_plan_result(self, value):
        """Parse the plan JSON, fill the tabs, and populate the step list."""
        from .ai_plan import parse_plan_result, step_label, \
            apply_step_params, apply_step_selection

        if value is None:
            self.output_ctrl.AppendText("\nNo RESULT= plan line found - nothing to apply.\n")
            return
        steps, errors = parse_plan_result(value)
        for message in errors:
            self.output_ctrl.AppendText(f"plan: {message}\n")
            self._log(f"AI plan: {message}")
        if steps is None:
            self.output_ctrl.AppendText("\nPlan was unusable - nothing applied.\n")
            return

        self._install_plan_steps(steps)

    def _install_plan_steps(self, steps):
        """Adopt a validated step list (from a fresh plan OR a loaded plan
        file): populate the checklist and pre-fill the tabs."""
        from .ai_plan import step_label, apply_step_params, \
            apply_step_selection
        # A new plan supersedes the session's panel tweaks: reset every
        # routing parameter to defaults BEFORE applying the plan's values,
        # so options the plan does not specify run at CLI-default-
        # equivalent state (the add_gnd_vias leak, generalized). Applies to
        # both a fresh AI plan and a Load...-ed file.
        try:
            if self.routing_dialog is not None and \
                    hasattr(self.routing_dialog, 'reset_params_to_defaults'):
                self.routing_dialog.reset_params_to_defaults()
                self._log("AI plan: options reset to defaults before "
                          "applying the plan")
        except Exception as e:
            self._log(f"AI plan: defaults reset skipped: {e}")
        self._plan_steps = steps
        self.plan_list.Set([step_label(i + 1, s) for i, s in enumerate(steps)])
        self.plan_list.SetCheckedItems(range(len(steps)))
        self.run_plan_btn.Enable(bool(steps) and self.routing_dialog is not None)
        self.run_all_plan_btn.Enable(self.run_plan_btn.IsEnabled())

        # Fill the tabs so each step can be reviewed in its native controls.
        # (Selections of same-action steps overwrite each other here; they are
        # re-applied per step at execution time.)
        notes = []
        for step in steps:
            try:
                notes += apply_step_params(step, self.routing_dialog)
                notes += apply_step_selection(step, self.routing_dialog)
            except Exception as e:
                notes.append(f"applying {step['action']}: {e}")
        for note in notes:
            self.output_ctrl.AppendText(f"plan: {note}\n")
            self._log(f"AI plan: {note}")
        self.output_ctrl.AppendText(
            f"\nPlan loaded: {len(steps)} step(s). Parameters were applied to the "
            "tabs - review/tweak them there, uncheck steps you don't want, then "
            "press 'Run Selected Steps' (or 'Run All Selected Steps' to run "
            "unattended, without the per-step completion popup).\n")
        self.run_plan_btn.Enable()
        self.run_all_plan_btn.Enable(self.run_plan_btn.IsEnabled())
        self._log(f"AI plan: {len(steps)} steps loaded")

    def _on_save_plan(self, event):
        if not self._plan_steps:
            wx.MessageBox("No plan steps to save. Run Plan Routing first "
                          "(or Load a plan file).", "AI",
                          wx.OK | wx.ICON_WARNING)
            return
        # Default to <boardname>_plan.json next to the board.
        default_dir = ""
        default_file = "plan.json"
        if self.board_filename:
            default_dir = os.path.dirname(os.path.abspath(self.board_filename))
            stem = os.path.splitext(os.path.basename(self.board_filename))[0]
            if stem:
                default_file = f"{stem}_plan.json"
        with wx.FileDialog(self, "Save plan (.json)", wildcard="*.json",
                           defaultDir=default_dir, defaultFile=default_file,
                           style=wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT) as dlg:
            if dlg.ShowModal() != wx.ID_OK:
                return
            path = dlg.GetPath()
        try:
            import json
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"steps": self._plan_steps}, f, indent=2)
            self._log(f"AI plan: saved {len(self._plan_steps)} step(s) to {path}")
            self.output_ctrl.AppendText(f"\nPlan saved to {path}\n")
        except Exception as e:
            wx.MessageBox(f"Could not save plan:\n{e}", "AI",
                          wx.OK | wx.ICON_ERROR)

    def _on_load_plan(self, event):
        from .ai_plan import parse_plan_result
        with wx.FileDialog(self, "Load plan (.json - other files are shown greyed by macOS)", wildcard="*.json",
                           style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST) as dlg:
            if dlg.ShowModal() != wx.ID_OK:
                return
            path = dlg.GetPath()
        try:
            with open(path, encoding="utf-8") as f:
                text = f.read()
        except Exception as e:
            wx.MessageBox(f"Could not read plan file:\n{e}", "AI",
                          wx.OK | wx.ICON_ERROR)
            return
        steps, errors = parse_plan_result(text)
        for message in errors:
            self.output_ctrl.AppendText(f"plan: {message}\n")
        if steps is None:
            wx.MessageBox("The file is not a usable plan (see the transcript "
                          "for details).", "AI", wx.OK | wx.ICON_ERROR)
            return
        self.parsed_ctrl.SetValue(f"Loaded from {path}")
        self._log(f"AI plan: loaded {len(steps)} step(s) from {path}")
        self._install_plan_steps(steps)

    def _on_run_all_selected(self, event):
        """Run the checked steps unattended (no per-step completion popups)."""
        self._on_run_selected(event, quiet=True)

    def _on_run_selected(self, event, quiet=False):
        from .ai_plan import PlanExecutor

        if self._plan_executor is not None or not self._plan_steps:
            return
        if self._runner is not None and self._runner.is_running():
            return
        indices = list(self.plan_list.GetCheckedItems())
        if not indices:
            wx.MessageBox("No steps are checked.", "AI", wx.OK | wx.ICON_WARNING)
            return
        self.run_plan_btn.Disable()
        self.run_all_plan_btn.Disable()
        self.plan_btn.Disable()
        self.stop_plan_btn.Enable()
        # Routing movie (#506): a plan run is ONE movie covering every step it
        # runs, not one per step -- bracket it (no-op unless recording is on).
        recorder = self._movie_recorder()
        if recorder is not None:
            recorder.begin_group()
        self._plan_executor = PlanExecutor(
            self.routing_dialog, self._plan_steps, indices,
            on_status=self._on_plan_step_status,
            on_finished=self._on_plan_finished,
            log=self._log,
            on_progress=self._on_plan_step_progress,
            quiet=quiet)
        self._plan_executor.start()

    def _on_stop_plan(self, event):
        if self._plan_executor is not None:
            self._plan_executor.stop()
            self.stop_plan_btn.Disable()
            self._log("AI plan: stop requested (cancelling current step)")

    def _on_plan_step_progress(self, index, step, label, value, rng,
                               elapsed, is_busy, force_repaint=False):
        """Mirror the working tab's status bar here: same text, same gauge,
        plus which step and its elapsed time -- a route_diff step reads
        exactly like the differential tab while it runs.

        Two feeds land here: the executor's POLL (main loop alive, normal
        paint) and the ui_thread_status PUSH-mirror (`force_repaint=True`) for
        steps that run ON the main loop -- fanout, cap optimize, the apply
        phases -- where only a forced Refresh+Update can make the text visible
        (same narrow repaint as gui_utils.ui_thread_status: label only, never
        Gauge.Pulse, inside an action plugin)."""
        if not self:
            return
        mins, secs = divmod(int(elapsed), 60)
        action = step.get('action', '?')
        text = f"Step {index + 1} ({action}) {mins}:{secs:02d}"
        if label and label != 'Ready':
            text += f" - {label}"
        self.elapsed_label.SetLabel(text)
        self.Layout()
        if force_repaint:
            try:
                self.elapsed_label.Refresh()
                self.elapsed_label.Update()
            except Exception:
                pass
            return  # gauge untouched: the value is meaningless mid-block
        try:
            if self.gauge.GetRange() != rng and rng > 0:
                self.gauge.SetRange(rng)
            self.gauge.SetValue(min(max(0, int(value)), self.gauge.GetRange()))
        except Exception:
            pass

    def _on_plan_step_status(self, index, status):
        if not self:
            return
        from .ai_plan import step_label
        mark = {"running": "> ", "done": "[ok] ", "failed": "[FAIL] ",
                "stopped": "[stopped] "}[status]
        self.plan_list.SetString(index, mark + step_label(index + 1, self._plan_steps[index]))
        if status == "done":
            # Stopped/failed steps stay checked so a re-run picks them up.
            self.plan_list.Check(index, False)

    def _movie_recorder(self):
        """The dialog's routing-movie recorder, or None (tab used standalone)."""
        dialog = self.routing_dialog
        if dialog is None and hasattr(self, 'GetTopLevelParent'):
            dialog = self.GetTopLevelParent()
        return getattr(dialog, 'movie_recorder', None)

    def _on_plan_finished(self, completed, aborted_reason):
        self._plan_executor = None
        # Close the movie group: renders one movie for the steps that ran (a
        # stopped/failed plan still gets the movie of what it did do).
        recorder = self._movie_recorder()
        if recorder is not None:
            try:
                recorder.end_group()
            except Exception as e:
                self._log(f"Routing movie: not rendered ({e})")
        if self:
            try:
                self.elapsed_label.SetLabel("")
                self.gauge.SetValue(0)
            except Exception:
                pass
        if not self:  # dialog destroyed while a step was running
            return
        self.stop_plan_btn.Disable()
        self.run_plan_btn.Enable(bool(self._plan_steps))
        self.run_all_plan_btn.Enable(self.run_plan_btn.IsEnabled())
        self.plan_btn.Enable()
        if aborted_reason:
            message = f"AI plan: stopped after {completed} step(s): {aborted_reason}"
        else:
            message = f"AI plan: all {completed} selected step(s) completed"
        self.output_ctrl.AppendText(f"\n{message}\n")
        self._log(message)

    # -------------------------------------------------------------- helpers

    def _on_cancel(self, event):
        if self._runner is not None:
            self._runner.cancel()
        self.cancel_btn.Disable()
        self._log("AI: cancel requested")

    def _on_elapsed_tick(self, event):
        if not self:
            return
        self._elapsed_seconds += 1
        mins, secs = divmod(self._elapsed_seconds, 60)
        self.elapsed_label.SetLabel(f"{mins}m {secs:02d}s" if mins else f"{secs}s")
        self.Layout()
        self.gauge.Pulse()

    def _on_close(self, event):
        """Close the parent dialog (matches the other tabs' Close button)."""
        self.GetTopLevelParent().EndModal(wx.ID_CANCEL)

    def _log(self, message):
        if self.log_callback:
            # Plan progress lines arrived without terminators and ran
            # together in the Log tab ('step 4 finishedAI plan: ...').
            if not message.endswith(chr(10)):
                message += chr(10)
            self.log_callback(message)
