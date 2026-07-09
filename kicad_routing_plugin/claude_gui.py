"""
KiCad Routing Tools - Claude integration

Runs Claude Code headless to drive the project's AI skills from the GUI
(GitHub issues #40, #34, #39).

Building blocks:
- ClaudeSkillRunner: spawn `claude -p`, stream events to main-thread callbacks
- ClaudeSkillDialog: modal dialog that runs one skill with a live transcript
  and returns the machine-readable RESULT=<value> last line
- ClaudeTab: the Claude tab in the routing dialog (currently a smoke-test
  button that runs /recommend-stackup)
"""

import os
import json
import shutil
import threading
import subprocess

import wx

PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)

# KiCad launched from Finder/desktop doesn't inherit the shell PATH, so
# shutil.which() alone often misses `claude`. Check common install spots.
_CLAUDE_CANDIDATES = [
    os.path.expanduser("~/.claude/local/claude"),
    os.path.expanduser("~/.local/bin/claude"),
    "/opt/homebrew/bin/claude",
    "/usr/local/bin/claude",
]

# Read-only analysis tools: the skills never need write access to the board.
DEFAULT_ALLOWED_TOOLS = "Read,Glob,Grep,Bash,WebSearch"

# Main models offered in the model dropdown: (label, --model value).
# None = let the CLI use the user's configured default. ALIASES, not pinned
# version IDs: the CLI resolves 'fable'/'opus'/'sonnet'/'haiku' to the
# newest model of each tier, so this list never goes stale ('Sonnet 4.6'
# style entries were outdated the day a new Sonnet shipped).
MODEL_CHOICES = [
    ("Default", None),
    ("Fable (latest)", "fable"),
    ("Opus (latest)", "opus"),
    ("Sonnet (latest)", "sonnet"),
    ("Haiku (latest)", "haiku"),
]

# --effort levels accepted by the CLI ("Default" = don't pass the flag).
EFFORT_CHOICES = ["Default", "low", "medium", "high", "xhigh", "max"]


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
            pcbnew.SaveBoard(snapshot, board)
            return snapshot
        except Exception as e:
            wx.MessageBox(
                f"Could not snapshot the board for analysis: {e}\n\n"
                "Falling back to the last saved file - unsaved changes "
                "will not be visible to the analysis.",
                "Claude", wx.OK | wx.ICON_WARNING)
    if not board_filename or not os.path.isfile(board_filename):
        wx.MessageBox(
            "Board file not found on disk. Save the board first so the "
            f"analysis sees the current state.\n\nLooked for: {board_filename}",
            "Claude", wx.OK | wx.ICON_WARNING)
        return None
    return os.path.abspath(board_filename)


def find_claude():
    """Return the path to the claude CLI, or None if not installed."""
    path = shutil.which("claude")
    if path:
        return path
    for candidate in _CLAUDE_CANDIDATES:
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return None


def extract_result_line(text):
    """Return the value of the last RESULT=<value> line, or None."""
    for line in reversed(text.strip().splitlines()):
        line = line.strip()
        if line.startswith("RESULT="):
            return line[len("RESULT="):].strip()
    return None


def auth_error_hint(error):
    """Extra guidance when a run failed because the claude CLI isn't
    logged in - the CLI's own message ("Please run /login") assumes the
    user knows /login is a command inside the claude terminal app."""
    markers = ("invalid api key", "/login", "not logged in", "authentication", "oauth")
    if error and any(m in error.lower() for m in markers):
        return ("\nClaude Code is installed but not logged in: open a "
                "terminal, run `claude`, complete /login, then retry.")
    return ""


def summarize_tool_use(name, tool_input):
    """One-line human-readable summary of a tool call."""
    if name == "Bash":
        detail = tool_input.get("description") or tool_input.get("command", "")
    elif name in ("Read", "Write", "Edit"):
        detail = tool_input.get("file_path", "")
    elif name in ("Glob", "Grep"):
        detail = tool_input.get("pattern", "")
    elif name == "WebSearch":
        detail = tool_input.get("query", "")
    elif name == "WebFetch":
        detail = tool_input.get("url", "")
    else:
        detail = json.dumps(tool_input)
    detail = " ".join(str(detail).split())
    if len(detail) > 120:
        detail = detail[:120] + "..."
    return f"{name}: {detail}"


def tool_result_text(block, max_len=120):
    """First line of a tool result, truncated."""
    content = block.get("content", "")
    if isinstance(content, list):
        content = " ".join(
            c.get("text", "") for c in content
            if isinstance(c, dict) and c.get("type") == "text")
    first_line = str(content).strip().splitlines()[0] if str(content).strip() else "(no output)"
    if len(first_line) > max_len:
        first_line = first_line[:max_len] + "..."
    return first_line


def format_stream_event(event):
    """Format one stream-json event as transcript text, or None to skip it."""
    etype = event.get("type")
    if etype == "system" and event.get("subtype") == "init":
        model = event.get("model", "unknown")
        version = event.get("claude_code_version", "unknown")
        lines = [f"Claude Code {version} | model: {model}",
                 f"cwd: {event.get('cwd', '?')}"]
        skills = event.get("skills", [])
        if skills:
            shown = ", ".join(skills[:8]) + (", ..." if len(skills) > 8 else "")
            lines.append(f"skills discovered: {len(skills)} ({shown})")
        return "\n".join(lines) + "\n\n"
    if etype == "assistant":
        lines = []
        for block in event.get("message", {}).get("content", []):
            btype = block.get("type")
            if btype == "text" and block.get("text", "").strip():
                lines.append(block["text"].rstrip())
            elif btype == "tool_use":
                summary = summarize_tool_use(block.get("name", "?"), block.get("input", {}))
                lines.append(f"  -> {summary}")
        return "\n".join(lines) + "\n" if lines else None
    if etype == "user":
        content = event.get("message", {}).get("content", [])
        lines = []
        if isinstance(content, list):
            for block in content:
                if isinstance(block, dict) and block.get("type") == "tool_result":
                    mark = "x" if block.get("is_error") else "ok"
                    lines.append(f"     [{mark}] {tool_result_text(block)}")
        return "\n".join(lines) + "\n" if lines else None
    return None


class ClaudeSkillRunner:
    """Runs `claude -p` headless on a background thread, streaming progress.

    Callbacks are invoked on the wx main thread:
      on_transcript(text)        - formatted transcript chunk to append
      on_done(result_text, error) - exactly one is non-None; cancel reports
                                    error="Cancelled."
    """

    def __init__(self, claude_path, on_transcript, on_done):
        self.claude_path = claude_path
        self.on_transcript = on_transcript
        self.on_done = on_done
        self._process = None
        self._thread = None
        self._cancel_requested = False

    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    def run(self, prompt, allowed_tools=DEFAULT_ALLOWED_TOOLS, model=None, effort=None):
        if self.is_running():
            raise RuntimeError("a Claude run is already in progress")
        cmd = [
            self.claude_path, "-p", prompt,
            # stream-json (requires --verbose in -p mode) emits one JSON event
            # per line as Claude works, so we can show live progress.
            "--output-format", "stream-json", "--verbose",
            "--allowedTools", allowed_tools,
        ]
        if model:
            cmd += ["--model", model]
        if effort:
            cmd += ["--effort", effort]
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
        final_event = None
        try:
            kwargs = {}
            if os.name == "nt":
                kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW
            self._process = subprocess.Popen(
                cmd,
                cwd=ROOT_DIR,  # skill discovery is working-directory based
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                errors="replace",
                bufsize=1,  # line-buffered: one stream-json event per line
                **kwargs,
            )
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
                if event.get("type") == "result":
                    final_event = event
                else:
                    text = format_stream_event(event)
                    if text:
                        wx.CallAfter(self.on_transcript, text)
            self._process.wait()
            stderr_thread.join(timeout=5)
            stderr = "".join(stderr_chunks)
            returncode = self._process.returncode
        except Exception as e:
            wx.CallAfter(self.on_done, None, f"Failed to launch claude: {e}")
            return
        finally:
            self._process = None
        wx.CallAfter(self._finish, final_event, stderr, returncode)

    def _finish(self, final_event, stderr, returncode):
        if self._cancel_requested:
            self.on_done(None, "Cancelled.")
        elif final_event is None:
            # claude died before emitting a result event
            self.on_done(None, (stderr or "").strip() or f"claude exited with code {returncode}")
        elif final_event.get("is_error"):
            self.on_done(None, str(final_event.get("result", "unknown error from claude")))
        else:
            self.on_done(str(final_event.get("result", "")), None)


class ClaudeSkillDialog(wx.Dialog):
    """Modal dialog that runs one Claude skill and shows the live transcript.

    After ShowModal() returns, `result_value` holds the parsed RESULT=<value>
    string (None if the run failed, was cancelled, or had no RESULT= line),
    and `result_text` holds Claude's full final reply.
    """

    def __init__(self, parent, title, prompt, claude_path=None,
                 allowed_tools=DEFAULT_ALLOWED_TOOLS, intro=None,
                 model=None, effort=None):
        super().__init__(parent, title=title, size=(720, 520),
                         style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
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

        self._runner = ClaudeSkillRunner(
            claude_path or find_claude(), self._append, self._on_done)
        self._runner.run(prompt, allowed_tools=allowed_tools, model=model, effort=effort)

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
            if self.output_ctrl.GetValue().rstrip().endswith(error.strip()):
                appended = auth_error_hint(error)
            else:
                appended = f"\n{error}{auth_error_hint(error)}"
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


class ClaudeTab(wx.Panel):
    """Claude tab: run AI skills headless and bring results into the GUI."""

    def __init__(self, parent, board_filename, log_callback=None, routing_dialog=None):
        super().__init__(parent)
        self.board_filename = board_filename
        self.log_callback = log_callback
        self.routing_dialog = routing_dialog
        self._elapsed_timer = wx.Timer(self)
        self._elapsed_seconds = 0
        self.Bind(wx.EVT_TIMER, self._on_elapsed_tick, self._elapsed_timer)
        self._claude_path = find_claude()
        self._runner = None
        self._pending_kind = None  # 'test' or 'plan' - what the runner is doing
        self._plan_steps = []
        self._plan_executor = None
        if self._claude_path:
            self._runner = ClaudeSkillRunner(
                self._claude_path, self._append_transcript, self._on_done)
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
        claude subprocess, stop the plan executor and the elapsed timer."""
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
            "The routing plan from Claude. Check the steps to run; review or "
            "tweak each step's parameters on its own tab before running.")
        steps_sizer.Add(self.plan_list, 1, wx.EXPAND | wx.ALL, 3)
        top_sizer.Add(steps_sizer, 1, wx.EXPAND | wx.RIGHT, 8)

        ctrl_box = wx.StaticBox(self, label="Claude")
        ctrl_sizer = wx.StaticBoxSizer(ctrl_box, wx.VERTICAL)

        # Availability status
        if self._claude_path:
            status = f"Claude Code CLI found: {self._claude_path}"
        else:
            status = ("Claude Code CLI not found. Install it (https://claude.com/claude-code) "
                      "and make sure `claude` is on your PATH, then reopen this dialog.")
        self.status_label = wx.StaticText(self, label=status)
        self.status_label.Wrap(280)
        ctrl_sizer.Add(self.status_label, 0, wx.ALL, 5)

        # Model / effort selection
        sel_grid = wx.FlexGridSizer(cols=2, hgap=5, vgap=5)
        sel_grid.AddGrowableCol(1)
        sel_grid.Add(wx.StaticText(self, label="Model:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.model_choice = wx.Choice(self, choices=[label for label, _ in MODEL_CHOICES])
        self.model_choice.SetSelection(0)
        self.model_choice.SetToolTip(
            "Model for the headless run (--model). Default = your claude CLI default. "
            "Bigger models give deeper analysis; Haiku is fastest/cheapest.")
        sel_grid.Add(self.model_choice, 0, wx.EXPAND)
        sel_grid.Add(wx.StaticText(self, label="Effort:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.effort_choice = wx.Choice(self, choices=EFFORT_CHOICES)
        self.effort_choice.SetSelection(0)
        self.effort_choice.SetToolTip(
            "Reasoning effort (--effort): low/medium/high/xhigh/max. Higher = more "
            "thorough but slower and costlier. Not supported on Haiku.")
        sel_grid.Add(self.effort_choice, 0, wx.EXPAND)
        ctrl_sizer.Add(sel_grid, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # Planning: the headline action (bold) with its Cancel right under it
        self.plan_btn = wx.Button(self, label="Plan Routing")
        self.plan_btn.SetFont(self.plan_btn.GetFont().Bold())
        self.plan_btn.SetToolTip(
            "Run the /plan-pcb-routing skill headless on the current board. The "
            "plan fills the tabs' parameter fields and appears in the step list "
            "you can review, edit (in each tab), select, and run.")
        self.plan_btn.Bind(wx.EVT_BUTTON, self._on_plan)
        self.plan_btn.Enable(self._claude_path is not None and self.routing_dialog is not None)
        ctrl_sizer.Add(self.plan_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.cancel_btn = wx.Button(self, label="Cancel")
        self.cancel_btn.SetToolTip("Cancel the running Claude analysis")
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

        self.stop_plan_btn = wx.Button(self, label="Stop")
        self.stop_plan_btn.SetToolTip("Stop after the currently running step finishes")
        self.stop_plan_btn.Bind(wx.EVT_BUTTON, self._on_stop_plan)
        self.stop_plan_btn.Disable()
        ctrl_sizer.Add(self.stop_plan_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        ctrl_sizer.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # Post-route skills: QA sign-off and failure diagnosis
        self.review_btn = wx.Button(self, label="Review Routed Board")
        self.review_btn.SetToolTip(
            "Run the /review-routed-board skill: DRC, connectivity, and orphan-stub "
            "checks, match-group and GND-via coverage review. The report shows in "
            "the transcript with a PASS/FAIL verdict.")
        self.review_btn.Bind(wx.EVT_BUTTON, self._on_review)
        self.review_btn.Enable(self._claude_path is not None)
        ctrl_sizer.Add(self.review_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.diagnose_btn = wx.Button(self, label="Diagnose Routing Failures")
        self.diagnose_btn.SetToolTip(
            "Run the /diagnose-routing-failures skill on the board plus this "
            "session's Log tab content: root-causes failed routes and recommends "
            "a targeted retry. Run a routing operation first.")
        self.diagnose_btn.Bind(wx.EVT_BUTTON, self._on_diagnose)
        self.diagnose_btn.Enable(self._claude_path is not None)
        ctrl_sizer.Add(self.diagnose_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # Activity: elapsed time + pulsing gauge while Claude runs
        self.elapsed_label = wx.StaticText(self, label="")
        ctrl_sizer.Add(self.elapsed_label, 0, wx.LEFT | wx.RIGHT, 5)
        self.gauge = wx.Gauge(self, range=100)
        ctrl_sizer.Add(self.gauge, 0, wx.EXPAND | wx.ALL, 5)

        ctrl_sizer.AddStretchSpacer(1)
        top_sizer.Add(ctrl_sizer, 0, wx.EXPAND)

        sizer.Add(top_sizer, 1, wx.EXPAND | wx.ALL, 8)

        # Parsed machine-readable result (RESULT= line of the last run)
        parsed_sizer = wx.BoxSizer(wx.HORIZONTAL)
        parsed_sizer.Add(wx.StaticText(self, label="Parsed result:"), 0,
                         wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.parsed_ctrl = wx.TextCtrl(self, style=wx.TE_READONLY)
        self.parsed_ctrl.SetToolTip(
            "The machine-readable last line of Claude's reply (RESULT=...), "
            "demonstrating how skill output will populate GUI fields.")
        parsed_sizer.Add(self.parsed_ctrl, 1, wx.EXPAND)
        sizer.Add(parsed_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)

        # Full output across the whole width at the bottom
        self.output_ctrl = wx.TextCtrl(
            self, style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_RICH2)
        self.output_ctrl.SetFont(
            wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        sizer.Add(self.output_ctrl, 1, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)

        self.SetSizer(sizer)

    # ------------------------------------------------------- model/effort

    def get_model_value(self):
        """The selected --model value (None = CLI default)."""
        return MODEL_CHOICES[self.model_choice.GetSelection()][1]

    def set_model_value(self, value):
        """Select the saved model; unknown/retired values revert to Default."""
        for i, (_label, v) in enumerate(MODEL_CHOICES):
            if v == value:
                self.model_choice.SetSelection(i)
                return
        self.model_choice.SetSelection(0)

    def get_effort_value(self):
        """The selected --effort level (None = CLI default)."""
        label = EFFORT_CHOICES[self.effort_choice.GetSelection()]
        return None if label == "Default" else label

    def set_effort_value(self, value):
        """Select the saved effort; unknown levels revert to Default."""
        if value in EFFORT_CHOICES[1:]:
            self.effort_choice.SetSelection(EFFORT_CHOICES.index(value))
        else:
            self.effort_choice.SetSelection(0)

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
            self.run_plan_btn.Enable(self.routing_dialog is not None
                                     and self._claude_path is not None)

    # ------------------------------------------------------------------ run

    def _board_path_or_warn(self):
        return board_path_for_analysis(self.board_filename)

    def _start_run(self, prompt, kind, intro):
        """Start a headless run (kind names what the result is for) with shared UI state."""
        self._pending_kind = kind
        self.plan_btn.Disable()
        self.review_btn.Disable()
        self.diagnose_btn.Disable()
        self.run_plan_btn.Disable()
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
        self._log(f"Claude: {intro.splitlines()[0]}"
                  + (f" | model={model}" if model else "")
                  + (f" | effort={effort}" if effort else ""))
        self._runner.run(prompt, model=model, effort=effort)

    def _on_review(self, event):
        if self._runner is None or self._runner.is_running():
            return
        board = self._board_path_or_warn()
        if board is None:
            return
        prompt = (
            f"/review-routed-board {board} — analysis only, do not modify any "
            "files. After the report, end your reply with exactly one line of the "
            "form RESULT=PASS or RESULT=FAIL (the overall sign-off verdict)"
        )
        self._start_run(prompt, "review",
                        f"Running /review-routed-board on {os.path.basename(board)} ...\n"
                        "(DRC + connectivity checkers + review; typically a few minutes)")

    def _on_diagnose(self, event):
        if self._runner is None or self._runner.is_running():
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
                "is a log to diagnose.", "Claude", wx.OK | wx.ICON_WARNING)
            return
        import tempfile
        fd, log_path = tempfile.mkstemp(prefix="kicadrt_gui_log_", suffix=".txt")
        with os.fdopen(fd, "w", encoding="utf-8", errors="replace") as f:
            f.write(log_text)
        prompt = (
            f"/diagnose-routing-failures {board} — the routing log from this GUI "
            f"session is at {log_path}. Analysis only, do not modify any files. "
            "After the report, end your reply with exactly one line of the form "
            "RESULT=<one-line recommended fix, or 'no failures found'>"
        )
        self._start_run(prompt, "diagnose",
                        f"Running /diagnose-routing-failures on {os.path.basename(board)} "
                        "+ the Log tab content ...\n(log analysis; typically a few minutes)")

    def _on_plan(self, event):
        if self._runner is None or self._runner.is_running():
            return
        if self._plan_executor is not None:
            wx.MessageBox("A plan is currently executing. Stop it first.",
                          "Claude", wx.OK | wx.ICON_WARNING)
            return
        board = self._board_path_or_warn()
        if board is None:
            return
        from .claude_plan import PLAN_RESULT_SCHEMA
        prompt = (
            f"/plan-pcb-routing {board} — analysis and planning only: do not "
            "execute any routing commands and do not modify any files. After the "
            f"report, end your reply with exactly one line of the form {PLAN_RESULT_SCHEMA}"
        )
        self._start_run(prompt, "plan",
                        f"Running /plan-pcb-routing on {os.path.basename(board)} ...\n"
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
        self.cancel_btn.Disable()
        kind, self._pending_kind = self._pending_kind, None

        if error:
            # The CLI often streams the failure text (e.g. 'Not logged in')
            # as an assistant message before the error result repeats it -
            # don't print the same line twice.
            if self.output_ctrl.GetValue().rstrip().endswith(error.strip()):
                appended = auth_error_hint(error)
            else:
                appended = f"\n{error}{auth_error_hint(error)}"
            if appended:
                self.output_ctrl.AppendText(appended + "\n")
            self._log(f"Claude: {error}")
            return

        # The streamed transcript already shows the full report; just close out.
        self.output_ctrl.AppendText(f"\n--- done in {self.elapsed_label.GetLabel()} ---\n")
        parsed = extract_result_line(result_text)
        if parsed is not None:
            self.parsed_ctrl.SetValue(parsed if len(parsed) < 200 else parsed[:200] + "...")
            self._log(f"Claude: done in {self._elapsed_seconds}s")
        else:
            self.parsed_ctrl.SetValue("(no RESULT= line found)")
            self._log(f"Claude: done in {self._elapsed_seconds}s, no RESULT= line")
        if kind == "plan":
            self._handle_plan_result(parsed)

    # ------------------------------------------------------------- planning

    def _handle_plan_result(self, value):
        """Parse the plan JSON, fill the tabs, and populate the step list."""
        from .claude_plan import parse_plan_result, step_label, \
            apply_step_params, apply_step_selection

        if value is None:
            self.output_ctrl.AppendText("\nNo RESULT= plan line found - nothing to apply.\n")
            return
        steps, errors = parse_plan_result(value)
        for message in errors:
            self.output_ctrl.AppendText(f"plan: {message}\n")
            self._log(f"Claude plan: {message}")
        if steps is None:
            self.output_ctrl.AppendText("\nPlan was unusable - nothing applied.\n")
            return

        self._plan_steps = steps
        self.plan_list.Set([step_label(i + 1, s) for i, s in enumerate(steps)])
        self.plan_list.SetCheckedItems(range(len(steps)))

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
            self._log(f"Claude plan: {note}")
        self.output_ctrl.AppendText(
            f"\nPlan loaded: {len(steps)} step(s). Parameters were applied to the "
            "tabs - review/tweak them there, uncheck steps you don't want, then "
            "press 'Run Selected Steps'.\n")
        self.run_plan_btn.Enable()
        self._log(f"Claude plan: {len(steps)} steps loaded")

    def _on_run_selected(self, event):
        from .claude_plan import PlanExecutor

        if self._plan_executor is not None or not self._plan_steps:
            return
        if self._runner is not None and self._runner.is_running():
            return
        indices = list(self.plan_list.GetCheckedItems())
        if not indices:
            wx.MessageBox("No steps are checked.", "Claude", wx.OK | wx.ICON_WARNING)
            return
        self.run_plan_btn.Disable()
        self.plan_btn.Disable()
        self.stop_plan_btn.Enable()
        self._plan_executor = PlanExecutor(
            self.routing_dialog, self._plan_steps, indices,
            on_status=self._on_plan_step_status,
            on_finished=self._on_plan_finished,
            log=self._log,
            on_progress=self._on_plan_step_progress)
        self._plan_executor.start()

    def _on_stop_plan(self, event):
        if self._plan_executor is not None:
            self._plan_executor.stop()
            self.stop_plan_btn.Disable()
            self._log("Claude plan: stop requested (after current step)")

    def _on_plan_step_progress(self, index, step, label, value, rng,
                               elapsed, is_busy):
        """Mirror the working tab's status bar here: same text, same gauge,
        plus which step and its elapsed time -- a route_diff step reads
        exactly like the differential tab while it runs."""
        if not self:
            return
        mins, secs = divmod(int(elapsed), 60)
        action = step.get('action', '?')
        text = f"Step {index + 1} ({action}) {mins}:{secs:02d}"
        if label and label != 'Ready':
            text += f" - {label}"
        self.elapsed_label.SetLabel(text)
        try:
            if self.gauge.GetRange() != rng and rng > 0:
                self.gauge.SetRange(rng)
            self.gauge.SetValue(min(max(0, int(value)), self.gauge.GetRange()))
        except Exception:
            pass

    def _on_plan_step_status(self, index, status):
        if not self:
            return
        from .claude_plan import step_label
        mark = {"running": "> ", "done": "[ok] ", "failed": "[FAIL] "}[status]
        self.plan_list.SetString(index, mark + step_label(index + 1, self._plan_steps[index]))
        if status == "done":
            self.plan_list.Check(index, False)

    def _on_plan_finished(self, completed, aborted_reason):
        self._plan_executor = None
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
        self.plan_btn.Enable()
        if aborted_reason:
            message = f"Claude plan: stopped after {completed} step(s): {aborted_reason}"
        else:
            message = f"Claude plan: all {completed} selected step(s) completed"
        self.output_ctrl.AppendText(f"\n{message}\n")
        self._log(message)

    # -------------------------------------------------------------- helpers

    def _on_cancel(self, event):
        if self._runner is not None:
            self._runner.cancel()
        self.cancel_btn.Disable()
        self._log("Claude: cancel requested")

    def _on_elapsed_tick(self, event):
        if not self:
            return
        self._elapsed_seconds += 1
        mins, secs = divmod(self._elapsed_seconds, 60)
        self.elapsed_label.SetLabel(f"{mins}m {secs:02d}s" if mins else f"{secs}s")
        self.gauge.Pulse()

    def _log(self, message):
        if self.log_callback:
            self.log_callback(message)
