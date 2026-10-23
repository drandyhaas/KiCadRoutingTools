"""
KiCad Routing Tools - Placement sub-tab (AI tab notebook).

Two Claude-Code-driven actions ("Place" via /plan-pcb-placement place, "Place +
Route" via /plan-pcb-placement full) plus
the non-AI "Beautify Labels" action. The layout follows the Routing sub-tab:
a large pane on the left (here the run's candidates table and the preview,
with the labels options under them), the "AI" control column on the right,
then the progress gauge, a status row and the transcript across the full
width.

The AI runs are headless, write into a persistent workdir
(placement_run.create_workdir), take minutes to hours, and are observed from
outside: a 1 s monitor timer turns the newest tool command, the converge
ledger tail and newly-appearing board files into an elapsed/stage status
line, and renders a preview frame of each new board while the run is going.
At the end REPORT.md and the run folder are surfaced in the tab and the
result can be explicitly applied back to the live board.

Claude Code is REQUIRED for the two AI actions: the tab's own backend
dropdown lists every harness but only lets Claude Code be chosen (the opencode
analysis agent denies edits, and these skills must write), and the buttons are
disabled with an install hint when the CLI is missing.
"""

import os
import sys
import threading
import time
from collections import deque

import wx

from .ai_backend import BACKENDS, BACKEND_IDS
from .placement_run import (
    KEEP_RUNS, PLACEMENT_ALLOWED_TOOLS, PLACEMENT_SUPPORTED_BACKENDS,
    active_candidates, build_placement_prompt, create_workdir, derive_stage,
    format_prune_note, mark_kept, newest_run_board, parse_placement_result,
    prune_runs, read_ledger_tail, scan_candidates, scan_workdir_outputs,
    stage_inputs,
)

# Same path setup as ai_gui/swig_gui: the engine modules live in py_router/,
# the placement package in py_placer/ (with a flat fallback for the PCM zip).
PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)
_ENGINE_DIR = os.path.join(ROOT_DIR, 'py_router')
if os.path.isdir(_ENGINE_DIR) and _ENGINE_DIR not in sys.path:
    sys.path.insert(0, _ENGINE_DIR)
for _sib in ('py_placer', 'py_tools'):
    _d = os.path.join(ROOT_DIR, _sib)
    if os.path.isdir(_d) and _d not in sys.path:
        sys.path.append(_d)

MODE_LABELS = {"place": "Place", "place_route": "Place + Route"}

# "Default" combo entry = don't pass the model/effort flag (ai_gui convention).
DEFAULT_CHOICE = "Default"

# Transcript control cap: an hours-long stream-json run grows without bound;
# when the text exceeds the cap the oldest quarter is dropped.
_TRANSCRIPT_CAP = 1024 * 1024
_TRANSCRIPT_TRIM = 256 * 1024


def _safe_mtime(path):
    """Sort key tolerant of files vanishing between listdir and stat."""
    try:
        return os.path.getmtime(path)
    except OSError:
        return 0


def _fmt_elapsed(seconds):
    seconds = int(seconds)
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    if h:
        return f"{h}h {m:02d}m {s:02d}s"
    return f"{m}m {s:02d}s"


class LabelsOptionsPanel(wx.Panel):
    """Options for Beautify Labels: two rows of label/control pairs with the
    two checkboxes beside them, a strip under the run preview.

    Control attribute names deliberately equal the future plan-step param
    names (label_size, label_orientation, ...) - the AI plan executor applies
    snake_case params by control attribute name.
    """

    def __init__(self, parent):
        super().__init__(parent)
        grid = wx.FlexGridSizer(cols=6, vgap=4, hgap=6)

        def row(label, ctrl, tooltip):
            grid.Add(wx.StaticText(self, label=label), 0,
                     wx.ALIGN_CENTER_VERTICAL | wx.LEFT, 4)
            ctrl.SetToolTip(tooltip)
            grid.Add(ctrl, 0, wx.EXPAND)
            return ctrl

        self.label_size = row(
            "Text size:",
            wx.SpinCtrlDouble(self, min=0.3, max=3.0, initial=1.0, inc=0.05, size=(78, -1)),
            "Uniform reference-label font height (and width).")
        self.label_thickness = row(
            "Thickness:",
            wx.SpinCtrlDouble(self, min=0.05, max=0.5, initial=0.15, inc=0.01, size=(78, -1)),
            "Uniform stroke thickness.")
        self.label_min_size = row(
            "Min size:",
            wx.SpinCtrlDouble(self, min=0.3, max=3.0, initial=0.7, inc=0.05, size=(78, -1)),
            "Shrink floor when the uniform size does not fit; equal to the "
            "text size disables shrinking.")
        self.label_orientation = row(
            "Orientation:", wx.Choice(self, choices=["horizontal", "vertical"], size=(118, -1)),
            "Preferred label direction; the other direction is a fallback "
            "when the preferred one does not fit.")
        self.label_orientation.SetSelection(0)
        self.label_max_distance = row(
            "Max dist:",
            wx.SpinCtrlDouble(self, min=0.5, max=10.0, initial=2.0, inc=0.25, size=(78, -1)),
            "How far beyond the part's courtyard a label may be pushed.")
        self.label_include_hidden = wx.CheckBox(self, label="Include hidden labels")
        self.label_include_hidden.SetToolTip(
            "Also place labels that are currently hidden (they stay hidden; "
            "only their pose and size are tidied).")
        self.label_skip_locked = wx.CheckBox(self, label="Skip locked parts")
        self.label_skip_locked.SetToolTip(
            "Leave the labels of KiCad-locked footprints untouched.")
        checks = wx.BoxSizer(wx.VERTICAL)
        checks.Add(self.label_include_hidden, 0, wx.BOTTOM, 4)
        checks.Add(self.label_skip_locked, 0)
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        sizer.Add(grid, 0, wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(checks, 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT, 16)
        self.SetSizer(sizer)

    def get_config(self):
        """Engine LabelConfig kwargs (keys = LabelConfig field names)."""
        return {
            'size': self.label_size.GetValue(),
            'thickness': self.label_thickness.GetValue(),
            'min_size': self.label_min_size.GetValue(),
            'orientation': self.label_orientation.GetStringSelection() or 'horizontal',
            'max_distance': self.label_max_distance.GetValue(),
            'include_hidden': self.label_include_hidden.GetValue(),
            'skip_locked': self.label_skip_locked.GetValue(),
        }

    def set_config(self, config):
        try:
            if 'size' in config:
                self.label_size.SetValue(float(config['size']))
            if 'thickness' in config:
                self.label_thickness.SetValue(float(config['thickness']))
            if 'min_size' in config:
                self.label_min_size.SetValue(float(config['min_size']))
            if 'orientation' in config:
                idx = self.label_orientation.FindString(str(config['orientation']))
                if idx != wx.NOT_FOUND:
                    self.label_orientation.SetSelection(idx)
            if 'max_distance' in config:
                self.label_max_distance.SetValue(float(config['max_distance']))
            if 'include_hidden' in config:
                self.label_include_hidden.SetValue(bool(config['include_hidden']))
            if 'skip_locked' in config:
                self.label_skip_locked.SetValue(bool(config['skip_locked']))
        except (TypeError, ValueError):
            pass  # a malformed saved dict must not break dialog restore

    def reset_to_defaults(self):
        self.set_config({'size': 1.0, 'thickness': 0.15, 'min_size': 0.7,
                         'orientation': 'horizontal', 'max_distance': 2.0,
                         'include_hidden': False, 'skip_locked': False})


class BoardPreview(wx.Panel):
    """One board's render, fitted to the pane, with a caption under it.

    The bitmap is placed by hand, not by a sizer: a sizer would let the
    image's size feed back into the pane's, and every fit would resize the
    pane and ask for the next fit.
    """

    def __init__(self, parent):
        super().__init__(parent)
        self._png = None
        self._fit_size = None
        sizer = wx.BoxSizer(wx.VERTICAL)
        # An inset panel in the LISTBOX colour, so the empty pane reads as a
        # frame the way the candidates list beside it does.
        self.frame_panel = wx.Panel(self, style=wx.BORDER_SUNKEN)
        self.frame_panel.SetBackgroundColour(
            wx.SystemSettings.GetColour(wx.SYS_COLOUR_LISTBOX))
        self.frame_panel.SetMinSize((240, 180))
        self.bitmap = wx.StaticBitmap(self.frame_panel)
        sizer.Add(self.frame_panel, 1, wx.EXPAND)
        self.caption = wx.StaticText(self, label="No run yet.")
        sizer.Add(self.caption, 0, wx.EXPAND | wx.TOP, 4)
        self.SetSizer(sizer)
        self.frame_panel.Bind(wx.EVT_SIZE, self._on_size)

    def show_png(self, path, caption=None):
        """Display one PNG scaled to the pane (aspect preserved)."""
        try:
            img = wx.Image(path, wx.BITMAP_TYPE_PNG)
            if not img.IsOk():
                return False
        except Exception:
            return False
        self._png = path
        w, h = self.frame_panel.GetClientSize()
        self._fit_size = (w, h)
        w, h = max(w - 6, 60), max(h - 6, 45)
        iw, ih = img.GetWidth(), img.GetHeight()
        scale = min(w / iw, h / ih)
        bw, bh = max(1, int(iw * scale)), max(1, int(ih * scale))
        # IMAGE_QUALITY_NORMAL: HIGH is very slow (about_tab precedent).
        img = img.Scale(bw, bh, wx.IMAGE_QUALITY_NORMAL)
        self.bitmap.SetBitmap(wx.Bitmap(img))
        self.bitmap.SetSize((bw, bh))
        self.bitmap.SetPosition(((w + 6 - bw) // 2, (h + 6 - bh) // 2))
        if caption is not None:
            self.caption.SetLabel(caption)
        return True

    def clear(self, caption=""):
        self._png = None
        self.bitmap.SetBitmap(wx.NullBitmap)
        self.caption.SetLabel(caption)

    def _on_size(self, event):
        event.Skip()
        if self._png and tuple(event.GetSize()) != self._fit_size:
            wx.CallAfter(self._refit)

    def _refit(self):
        if self and self._png and \
                tuple(self.frame_panel.GetClientSize()) != self._fit_size:
            self.show_png(self._png)


class PlacementRunMonitor:
    """Polls a placement run's workdir + transcript into the status line and
    the candidates table.

    One 1000 ms wx.Timer owned by the tab. Every second tick it rescans the
    run's seeds (stat + small JSON reads: placement_run.scan_candidates)
    and the newest board; every tick it reads the converge ledger's new rows
    and rewrites the status line.
    """

    TICK_MS = 1000
    QUIET_AFTER_S = 120

    def __init__(self, tab, workdir, mode):
        self.tab = tab
        self.workdir = workdir
        self.mode = mode
        self._start = time.monotonic()
        self._timer = wx.Timer(tab)
        tab.Bind(wx.EVT_TIMER, self._on_tick, self._timer)
        self._ticks = 0
        self._transcript_tail = deque(maxlen=40)
        self._last_event = time.monotonic()
        self._ledger_offset = 0
        self._last_ledger_row = None
        self._newest = None

    def start(self):
        self._timer.Start(self.TICK_MS)

    def stop(self):
        self._timer.Stop()

    def note_transcript(self, text):
        for line in text.splitlines():
            line = line.strip()
            if line:
                self._transcript_tail.append(line)
        self._last_event = time.monotonic()

    def _on_tick(self, event):
        tab = self.tab
        if not tab:
            self._timer.Stop()
            return
        self._ticks += 1
        if self._ticks % 2 == 0:
            tab.refresh_candidates(
                active=active_candidates(self._transcript_tail))
            self._newest = newest_run_board(self.workdir)
        rows, self._ledger_offset = read_ledger_tail(
            os.path.join(self.workdir, "ledger.jsonl"), self._ledger_offset)
        if rows:
            self._last_ledger_row = rows[-1]
        stage = derive_stage(self._transcript_tail, self._last_ledger_row,
                             self._newest)
        status = (f"{MODE_LABELS.get(self.mode, self.mode)} — "
                  f"{_fmt_elapsed(time.monotonic() - self._start)} — {stage}")
        quiet = time.monotonic() - self._last_event
        if quiet > self.QUIET_AFTER_S:
            status += f" (last agent event {int(quiet // 60)}m ago)"
        tab.set_status(status)
        tab.progress_bar.Pulse()


# The candidates table: (heading, width) per column.
_CAND_COLUMNS = (("Seed", 44), ("Blocking", 66), ("Vias", 48),
                 ("Copper (mm)", 88), ("Segments", 72), ("Status", 64))


def _cand_cells(row):
    """One candidates-table row's cell texts, in _CAND_COLUMNS order."""
    def num(v, fmt="{}"):
        return "" if v is None else fmt.format(v)
    return (str(row["seed"]), num(row["blocking"]),
            num(row["vias"]), num(row["copper_mm"], "{:.0f}"),
            num(row["segments"]), row["status"])


def _cand_caption(row):
    """The preview caption for a candidate."""
    head = f"seed {row['seed']}"
    if row["blocking"] is not None:
        text = (f"{head} — fast route: {row['blocking']} blocking, "
                f"{row['vias']} vias, {row['copper_mm']:.0f} mm")
    elif row["placed"]:
        text = f"{head} — placed; fast route pending"
    else:
        text = f"{head} — placing"
    tag = {"KEPT": "kept", "best": "best fast route"}.get(row["status"])
    return f"{text} ({tag})" if tag else text


class _StatusField(wx.TextCtrl):
    """The status row's read-only field (the Routing tab's "Parsed result"
    box). Label-style accessors so callers set and read it like the
    StaticText it replaces."""

    def __init__(self, parent, text=""):
        super().__init__(parent, value=text, style=wx.TE_READONLY)

    def SetLabel(self, text):
        self.ChangeValue(text)

    def GetLabel(self):
        return self.GetValue()


class PlacementTab(wx.Panel):
    """Placement sub-tab of the AI notebook (see module docstring)."""

    def __init__(self, parent, pcb_data, board_filename, on_complete=None,
                 append_log=None, sync_pcb_data_callback=None):
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self.on_complete = on_complete
        self.append_log = append_log or (lambda text: None)
        self.sync_pcb_data_callback = sync_pcb_data_callback
        # The backend dropdown shows every harness but only
        # PLACEMENT_SUPPORTED_BACKENDS can actually be selected (today:
        # Claude Code - the skills must write, and e.g. opencode's analysis
        # agent denies edits). self.backend always holds a SUPPORTED backend.
        self.backend = BACKENDS[PLACEMENT_SUPPORTED_BACKENDS[0]]
        # Model/effort entries remembered PER BACKEND, exactly as the
        # Routing tab does: without this, switching backend carried the
        # previous one's model over, so opencode could sit there with a
        # Claude-only model selected.
        self._backend_params = {bid: {'model': DEFAULT_CHOICE,
                                      'effort': DEFAULT_CHOICE}
                                for bid in BACKEND_IDS}
        self._last_backend = self.backend

        # Plan-executor contract (ai_plan.PlanExecutor): the beautify action
        # is the plan-drivable non-AI action on this tab.
        self._cancel_requested = False
        self._routing_thread = None

        self._runner = None
        self._monitor = None
        self._mode = None
        self._pending_result = None
        self.last_workdir = ""
        self._report = None            # the run's REPORT.md, when it exists
        self._run_dir = None           # the run's folder, when it exists
        self._cand_rows = []           # placement_run.scan_candidates rows
        self._user_picked = False      # a click stops the auto-selection
        self._selecting = False        # programmatic selection in progress
        self._sel_key = None           # the seed shown in the preview
        self._shown = None             # (png, png mtime, caption) on screen
        self._render_busy = False
        self._render_next = None       # (board, png) queued behind a render

        self._create_ui()
        self._refresh_cli_status()
        self.Bind(wx.EVT_WINDOW_DESTROY, self._on_destroy)

    # ----------------------------------------------------------------- UI

    def _create_ui(self):
        """The Routing sub-tab's layout (ai_gui.AITab._create_ui): a large
        pane on the left, the "AI" control column on the right, then the
        gauge, a status row and the transcript across the full width."""
        sizer = wx.BoxSizer(wx.VERTICAL)
        top = wx.BoxSizer(wx.HORIZONTAL)

        # Left: the run's candidates (where the Routing tab lists its planned
        # steps) with the selected one's board beside them, and the Beautify
        # Labels options as a strip under both.
        left = wx.BoxSizer(wx.VERTICAL)
        cand_box = wx.StaticBox(self, label="Candidates")
        cand_sizer = wx.StaticBoxSizer(cand_box, wx.HORIZONTAL)
        self.cand_list = wx.ListCtrl(
            self, style=wx.LC_REPORT | wx.LC_SINGLE_SEL)
        for i, (name, width) in enumerate(_CAND_COLUMNS):
            self.cand_list.InsertColumn(
                i, name, wx.LIST_FORMAT_LEFT if i == len(_CAND_COLUMNS) - 1
                else wx.LIST_FORMAT_RIGHT, width)
        self.cand_list.SetToolTip(
            "One row per seed: its fast route's blocking nets, vias, "
            "copper and segments. 'best' is the best fast route, 'clean' a "
            "fast route with no blocking net, 'KEPT' the board the run "
            "returned. Select a row to see its board.")
        self.cand_list.Bind(wx.EVT_LIST_ITEM_SELECTED, self._on_cand_selected)
        cand_sizer.Add(self.cand_list, 1, wx.EXPAND | wx.ALL, 3)
        self.preview = BoardPreview(self)
        cand_sizer.Add(self.preview, 1, wx.EXPAND | wx.ALL, 3)
        left.Add(cand_sizer, 1, wx.EXPAND)

        labels_box = wx.StaticBox(self, label="Beautify Labels (no AI)")
        labels_sizer = wx.StaticBoxSizer(labels_box, wx.HORIZONTAL)
        self.labels_options = LabelsOptionsPanel(self)
        labels_sizer.Add(self.labels_options, 0,
                         wx.ALIGN_CENTER_VERTICAL | wx.ALL, 3)
        labels_sizer.AddStretchSpacer(1)
        self.action_btn = wx.Button(self, label="Beautify Labels")
        self.action_btn.SetToolTip(
            "Uniform, non-overlapping reference-designator silkscreen. "
            "Moves and restyles only the labels, never the parts.")
        labels_sizer.Add(self.action_btn, 0,
                         wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)
        left.Add(labels_sizer, 0, wx.EXPAND | wx.TOP, 6)
        top.Add(left, 1, wx.EXPAND | wx.RIGHT, 8)

        # Right: the "AI" column, built the way the Routing tab builds its.
        ctrl_box = wx.StaticBox(self, label="AI")
        ctrl_sizer = wx.StaticBoxSizer(ctrl_box, wx.VERTICAL)
        sel_grid = wx.FlexGridSizer(cols=2, hgap=5, vgap=5)
        sel_grid.AddGrowableCol(1)
        sel_grid.Add(wx.StaticText(self, label="Backend:"), 0,
                     wx.ALIGN_CENTER_VERTICAL)
        self.backend_choice = wx.Choice(
            self, choices=[BACKENDS[bid].label for bid in BACKEND_IDS])
        self.backend_choice.SetSelection(BACKEND_IDS.index(self.backend.id))
        self.backend_choice.SetToolTip(
            "Harness that runs the /plan-pcb-placement skill. Only Claude Code "
            "can run it (placement runs must WRITE, and the other harnesses' "
            "agents deny edits); picking another one reverts.")
        self.backend_choice.Bind(wx.EVT_CHOICE, self._on_backend_choice)
        sel_grid.Add(self.backend_choice, 0, wx.EXPAND)
        sel_grid.Add(wx.StaticText(self, label="Model:"), 0,
                     wx.ALIGN_CENTER_VERTICAL)
        self.model_choice = wx.ComboBox(self, value=DEFAULT_CHOICE, choices=[])
        sel_grid.Add(self.model_choice, 0, wx.EXPAND)
        sel_grid.Add(wx.StaticText(self, label="Effort:"), 0,
                     wx.ALIGN_CENTER_VERTICAL)
        self.effort_choice = wx.ComboBox(self, value=DEFAULT_CHOICE, choices=[])
        sel_grid.Add(self.effort_choice, 0, wx.EXPAND)
        ctrl_sizer.Add(sel_grid, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        # No combo population here: __init__ calls _refresh_cli_status() once
        # AFTER _create_ui, like the Routing tab.

        ctrl_sizer.Add(wx.StaticText(self, label="Extra instructions:"), 0,
                       wx.LEFT | wx.RIGHT | wx.TOP, 5)
        self.extra_instructions = wx.TextCtrl(
            self, style=wx.TE_MULTILINE, size=(-1, 54))
        self.extra_instructions.SetToolTip(
            "Optional extra instructions passed to the agent (constraints, "
            "focus areas, budget hints).")
        ctrl_sizer.Add(self.extra_instructions, 0,
                       wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        ctrl_sizer.AddSpacer(6)
        ctrl_sizer.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 5)
        ctrl_sizer.AddSpacer(6)

        # The runs, with their Cancel right under them (Routing: Plan/Cancel).
        self.place_btn = wx.Button(self, label="Place (AI)")
        self.place_btn.SetToolTip(
            "Claude Code runs /plan-pcb-placement place headless on a "
            "snapshot of this board: it groups the parts, places 12 seeds "
            "(6 started scattered, 6 clustered), fast-routes the 6 its cost "
            "rates best and keeps the one whose fast route is best. "
            "Artifacts land in a krt_placement folder next to the board file, "
            f"which keeps this board's newest {KEEP_RUNS} runs.")
        ctrl_sizer.Add(self.place_btn, 0,
                       wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        self.place_route_btn = wx.Button(self, label="Place + Route (AI)")
        self.place_route_btn.SetToolTip(
            "Claude Code runs /plan-pcb-placement full headless: the same "
            "placement as Place, then the kept placement is routed "
            "with the full chain and graded. Artifacts land in a krt_placement "
            f"folder next to the board file, which keeps this board's newest "
            f"{KEEP_RUNS} runs.")
        ctrl_sizer.Add(self.place_route_btn, 0,
                       wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        self.stop_btn = wx.Button(self, label="Cancel")
        self.stop_btn.SetToolTip(
            "Cancel the running placement or label job; a placement run's "
            "partial artifacts are kept in its folder.")
        self.stop_btn.Disable()
        ctrl_sizer.Add(self.stop_btn, 0,
                       wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        ctrl_sizer.Add(wx.StaticLine(self), 0,
                       wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # After a run (Routing: Review / Diagnose).
        self.apply_btn = wx.Button(self, label="Apply Result to Board")
        self.apply_btn.Enable(False)
        self.apply_btn.SetToolTip(
            "Apply the finished run's board back to the live board "
            "(footprint poses; Place + Route also replaces all tracks/vias). "
            "Asks for confirmation first.")
        ctrl_sizer.Add(self.apply_btn, 0,
                       wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        ctrl_sizer.AddStretchSpacer(1)
        ctrl_sizer.AddSpacer(6)
        ctrl_sizer.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 5)
        ctrl_sizer.AddSpacer(6)
        self.close_btn = wx.Button(self, label="Close")
        self.close_btn.SetToolTip("Close dialog")
        ctrl_sizer.Add(self.close_btn, 0,
                       wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        # The GUI gates read the close button under its old name.
        self.cancel_btn = self.close_btn
        top.Add(ctrl_sizer, 0, wx.EXPAND)
        sizer.Add(top, 1, wx.EXPAND | wx.ALL, 8)

        # Gauge, full width (Routing: the activity row).
        self.progress_bar = wx.Gauge(self, range=100)
        sizer.Add(self.progress_bar, 0,
                  wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)

        # Status row (Routing: "Parsed result:" with Save / Load).
        status_row = wx.BoxSizer(wx.HORIZONTAL)
        status_row.Add(wx.StaticText(self, label="Status:"), 0,
                       wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.status_text = _StatusField(self, "Ready")
        status_row.Add(self.status_text, 1, wx.EXPAND)
        self.open_report_btn = wx.Button(self, label="Open Report")
        self.open_report_btn.SetToolTip("Open the run's REPORT.md")
        self.open_workdir_btn = wx.Button(self, label="Open Folder")
        self.open_workdir_btn.SetToolTip("Open the run's working folder")
        status_row.Add(self.open_report_btn, 0, wx.LEFT, 5)
        status_row.Add(self.open_workdir_btn, 0, wx.LEFT, 5)
        self.open_report_btn.Bind(
            wx.EVT_BUTTON, lambda e: self._launch(self._report))
        self.open_workdir_btn.Bind(
            wx.EVT_BUTTON, lambda e: self._launch(self._run_dir))
        self._set_outputs()
        sizer.Add(status_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)

        # Transcript across the full width at the bottom.
        self.transcript_ctrl = wx.TextCtrl(
            self, style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_RICH2)
        self.transcript_ctrl.SetFont(
            wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL,
                    wx.FONTWEIGHT_NORMAL))
        sizer.Add(self.transcript_ctrl, 1,
                  wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)
        self.SetSizer(sizer)

        self.place_btn.Bind(wx.EVT_BUTTON, lambda e: self._start_ai_run("place"))
        self.place_route_btn.Bind(wx.EVT_BUTTON,
                                  lambda e: self._start_ai_run("place_route"))
        self.action_btn.Bind(wx.EVT_BUTTON, self._on_action)
        self.apply_btn.Bind(wx.EVT_BUTTON, self._on_apply)
        self.stop_btn.Bind(wx.EVT_BUTTON, self._on_stop)
        self.close_btn.Bind(wx.EVT_BUTTON, self._on_close)

    def set_status(self, text):
        """Show the run's status line; the full text is also the tooltip, for
        a line longer than the field."""
        self.status_text.SetLabel(text)
        self.status_text.SetToolTip(text)

    def _populate_ai_combos(self):
        """Kept as the old entry point; the refresh owns the combos now
        so there is ONE place that decides what a backend offers."""
        self._refresh_cli_status()

    @staticmethod
    def _combo_value(ctrl):
        value = ctrl.GetValue().strip()
        return None if not value or value == DEFAULT_CHOICE else value

    def get_model_value(self):
        """The model flag value for placement runs (None = CLI default)."""
        return self._combo_value(self.model_choice)

    def set_model_value(self, value):
        self.model_choice.SetValue(value or DEFAULT_CHOICE)

    def get_effort_value(self):
        """The effort flag value for placement runs (None = CLI default)."""
        return self._combo_value(self.effort_choice)

    def set_effort_value(self, value):
        self.effort_choice.SetValue(value or DEFAULT_CHOICE)

    def _on_backend_choice(self, event):
        bid = BACKEND_IDS[self.backend_choice.GetSelection()]
        if bid not in PLACEMENT_SUPPORTED_BACKENDS:
            # Revert: shown (greyed) so the roadmap is visible, but placement
            # runs must WRITE and only this backend's agent may edit.
            self.backend_choice.SetSelection(BACKEND_IDS.index(self.backend.id))
            self.set_status(
                f"{BACKENDS[bid].label} cannot drive placement yet - these "
                f"runs must write, and only "
                f"{BACKENDS[PLACEMENT_SUPPORTED_BACKENDS[0]].label} supports "
                "that today.")
            return
        # Remember the OUTGOING backend's entries first (ai_gui parity).
        self._backend_params[self._last_backend.id] = {
            'model': self.model_choice.GetValue(),
            'effort': self.effort_choice.GetValue()}
        self.backend = BACKENDS[bid]
        self._last_backend = self.backend
        self._refresh_cli_status()

    def get_backend_value(self):
        """The selected backend id (settings persistence)."""
        return self.backend.id

    def set_backend_value(self, value):
        """Restore a saved backend id; unsupported/unknown ids revert to the
        default supported backend (Claude Code)."""
        bid = value if value in PLACEMENT_SUPPORTED_BACKENDS else \
            PLACEMENT_SUPPORTED_BACKENDS[0]
        self.backend = BACKENDS[bid]
        self._last_backend = self.backend
        self.backend_choice.SetSelection(BACKEND_IDS.index(bid))
        self._refresh_cli_status()

    def _refresh_cli_status(self):
        """The Routing tab's _refresh_backend_ui, for this tab's controls:
        the CLI status note in the transcript, then the selected backend's own
        model/effort entries and tooltips."""
        cli = self.backend.find_cli()
        note = self.backend.cli_status(cli)
        if note not in self.transcript_ctrl.GetValue():
            self._append_transcript(note + "\n")
        params = self._backend_params[self.backend.id]
        self.model_choice.Set(list(self.backend.model_suggestions))
        self.model_choice.SetValue(params['model'] or DEFAULT_CHOICE)
        self.model_choice.SetToolTip(self.backend.model_tooltip)
        self.effort_choice.Set(list(self.backend.effort_suggestions))
        self.effort_choice.SetValue(params['effort'] or DEFAULT_CHOICE)
        self.effort_choice.SetToolTip(self.backend.effort_tooltip)
        self.place_btn.Enable(cli is not None)
        self.place_route_btn.Enable(cli is not None)
        self.Layout()

    # ------------------------------------------------------------- AI runs

    def _busy(self):
        return ((self._runner is not None and self._runner.is_running())
                or (self._routing_thread is not None
                    and self._routing_thread.is_alive()))

    def _start_ai_run(self, mode):
        if self._busy():
            wx.MessageBox("A placement operation is already running.",
                          "Placement", wx.OK | wx.ICON_WARNING)
            return
        cli = self.backend.find_cli()
        if cli is None:
            wx.MessageBox(self.backend.not_found_message(), "Placement",
                          wx.OK | wx.ICON_WARNING)
            return
        from .ai_gui import AISkillRunner, board_path_for_analysis
        snapshot = board_path_for_analysis(self.board_filename)
        if snapshot is None:
            return
        try:
            workdir = create_workdir(self.board_filename, mode)
            staged = stage_inputs(workdir, snapshot, self.board_filename)
        except OSError as e:
            wx.MessageBox(f"Could not create the run's working directory: {e}",
                          "Placement", wx.OK | wx.ICON_ERROR)
            return
        self.last_workdir = workdir
        self._mode = mode
        self._pending_result = None
        # This run's frames and outputs only: the previous run's files may be
        # pruned (KEEP_RUNS) the moment this one launches.
        self._clear_candidates()
        self._set_outputs()
        prompt = build_placement_prompt(
            self.backend, workdir, staged, mode,
            extra=self.extra_instructions.GetValue())

        model = self.get_model_value()
        effort = self.get_effort_value()

        label = MODE_LABELS[mode]
        self._set_running_ui(True)
        self.set_status(f"{label} — starting Claude Code...")
        self._append_transcript(
            f"=== {label} run ===\nworkdir: {workdir}\n"
            f"model: {model or 'default'}  effort: {effort or 'default'}\n\n")
        self.append_log(f"Placement: {label} run started, workdir {workdir}\n")

        self._runner = AISkillRunner(cli, self._on_transcript,
                                     self._on_run_done, backend=self.backend,
                                     on_event=self._on_stream_event)
        try:
            self._runner.run(prompt, model=model, effort=effort,
                             allowed_tools=PLACEMENT_ALLOWED_TOOLS,
                             add_dirs=(workdir,))
        except ValueError as e:
            # #552: a backend can now REFUSE an allowlist it cannot serve
            # (opencode's pinned agent denies edits, and placement writes).
            # PLACEMENT_SUPPORTED_BACKENDS keeps that unreachable today, but
            # placement_run.py invites growing that tuple -- and without this
            # handler the refusal would escape into the wx event handler with
            # _set_running_ui(True) already latched: buttons disabled forever,
            # an orphan workdir, and the carefully worded message never shown.
            self._runner = None
            self._set_running_ui(False)
            self.set_status(f"{label} — refused")
            self._append_transcript(f"REFUSED: {e}\n")
            self.append_log(f"Placement: {label} refused -- {e}\n")
            wx.MessageBox(str(e), "Placement", wx.OK | wx.ICON_WARNING)
            return
        self._monitor = PlacementRunMonitor(self, workdir, mode)
        self._monitor.start()
        self._start_prune(workdir)

    def _start_prune(self, workdir):
        """Delete this board's run folders beyond KEEP_RUNS (#1057).

        Called only once the new run has launched, so a run that never
        started cannot cost the user an old one. Off the UI thread: a run
        folder's converge board store can hold thousands of files.
        """
        board = self.board_filename

        def work():
            try:
                note = format_prune_note(prune_runs(workdir, board, KEEP_RUNS))
            except Exception as e:                         # noqa: BLE001
                note = f"Placement: pruning old runs failed: {e}\n"
            if note:
                wx.CallAfter(self._prune_done, note)

        threading.Thread(target=work, daemon=True).start()

    def _prune_done(self, note):
        if not self:
            return
        self._append_transcript(note)
        self.append_log(note)

    def _set_running_ui(self, running):
        self.place_btn.Enable(not running and self.backend.find_cli() is not None)
        self.place_route_btn.Enable(
            not running and self.backend.find_cli() is not None)
        self.action_btn.Enable(not running)
        self.apply_btn.Enable(not running and self._pending_result is not None
                              and self._pending_result.get("board") is not None)
        self.stop_btn.Enable(running)

    def _append_transcript(self, text):
        ctrl = self.transcript_ctrl
        if ctrl.GetLastPosition() > _TRANSCRIPT_CAP:
            ctrl.Remove(0, _TRANSCRIPT_TRIM)
        ctrl.AppendText(text)

    def _on_transcript(self, text):
        if not self:
            return
        self._append_transcript(text)
        if self._monitor is not None:
            self._monitor.note_transcript(text)

    def _on_stream_event(self, event):
        """Raw stream-json tap: feed FULL Bash commands to the monitor.

        The formatted transcript prefers the Bash tool's short description
        and truncates at ~120 chars, which usually loses the `s{seed}`
        and `s<S>` arguments that stage derivation keys on - the raw
        event still carries them.
        """
        if not self or self._monitor is None:
            return
        try:
            for block in event.get("message", {}).get("content", []):
                if (isinstance(block, dict) and block.get("type") == "tool_use"
                        and str(block.get("name", "")).lower() == "bash"):
                    cmd = (block.get("input") or {}).get("command")
                    if cmd:
                        self._monitor.note_transcript(str(cmd))
        except (AttributeError, TypeError):
            pass

    def _on_run_done(self, result_text, error):
        if not self:
            return
        if self._monitor is not None:
            self._monitor.stop()
        self.progress_bar.SetValue(0)
        workdir = self.last_workdir
        label = MODE_LABELS.get(self._mode, "Placement")

        if error is not None:
            hint = self.backend.auth_hint(error)
            self._append_transcript(f"\n=== ERROR ===\n{error}{hint}\n")
            # Even a failed/cancelled run may have left useful partials.
            outputs = scan_workdir_outputs(workdir) if workdir else None
            self._finish_run(outputs,
                             f"{label} failed — partial artifacts in {workdir}"
                             if error != "Cancelled." else
                             f"{label} cancelled — partial artifacts kept in {workdir}")
            return

        from .ai_backend import extract_result_line
        value = extract_result_line(result_text or "")
        outputs, errs = parse_placement_result(value)
        if outputs is None:
            self._append_transcript(
                "\n(no usable RESULT line: " + "; ".join(errs)
                + " — scanning the workdir instead)\n")
            outputs = scan_workdir_outputs(workdir)
        elif errs:
            self._append_transcript("\n(RESULT notes: " + "; ".join(errs) + ")\n")

        if outputs.get("status") == "refused":
            self._finish_run(
                outputs, f"{label}: skill refused (see transcript) — "
                "e.g. the board already carries routed copper")
            return
        blocking = outputs.get("blocking")
        status = f"{label} finished"
        if blocking is not None:
            status += f" — blocking {blocking}"
        if outputs.get("summary"):
            status += f" — {outputs['summary']}"
        self._finish_run(outputs, status)

    def _finish_run(self, outputs, status):
        """Common tail for success/failure/cancel: surface artifacts + UI."""
        self._runner = None
        self._pending_result = outputs if outputs and outputs.get("board") else None
        workdir = self.last_workdir
        if outputs:
            report = outputs.get("report")
            if report:
                self._load_report(report)
            self._set_outputs(report=report, workdir=workdir)
            for key in ("board", "report"):
                if outputs.get(key):
                    self._append_transcript(f"{key}: {outputs[key]}\n")
                    self.append_log(f"Placement {key}: {outputs[key]}\n")
        self._monitor = None
        self.refresh_candidates(kept_board=(outputs or {}).get("board"),
                                final=True)
        self.set_status(status)
        self._set_running_ui(False)

    # ------------------------------------------------------- candidates

    def _set_outputs(self, report=None, workdir=None):
        """Arm Open Report / Open Folder for the run's report and folder."""
        self._report = report if report and os.path.isfile(report) else None
        self._run_dir = workdir if workdir and os.path.isdir(workdir) else None
        self.open_report_btn.Enable(self._report is not None)
        self.open_workdir_btn.Enable(self._run_dir is not None)

    @staticmethod
    def _launch(path):
        if path:
            wx.LaunchDefaultApplication(path)

    def _clear_candidates(self):
        self._cand_rows = []
        self._user_picked = False
        self._sel_key = None
        self._shown = None
        self.cand_list.DeleteAllItems()
        self.preview.clear("No candidates yet.")

    def refresh_candidates(self, active=None, kept_board=None, final=False):
        """Rescan the run's seeds into the table (monitor tick; run end).

        During a run the newest row is selected until the user clicks one;
        at the end the row the run kept is marked KEPT and selected.
        """
        workdir = self.last_workdir
        if not workdir or not os.path.isdir(workdir):
            return
        rows = scan_candidates(workdir, active=None if final else active)
        kept = mark_kept(rows, workdir, kept_board) if final else None
        keys = [r["seed"] for r in rows]
        if keys != [r["seed"] for r in self._cand_rows]:
            self.cand_list.DeleteAllItems()
            for i, _k in enumerate(keys):
                self.cand_list.InsertItem(i, "")
        for i, row in enumerate(rows):
            for col, text in enumerate(_cand_cells(row)):
                if self.cand_list.GetItemText(i, col) != text:
                    self.cand_list.SetItem(i, col, text)
        self._cand_rows = rows
        if kept is not None:
            pick = kept["seed"]
        elif not self._user_picked and rows:
            newest = max(rows, key=lambda r: (r["mtime"], r["seed"]))
            pick = newest["seed"]
        else:
            pick = self._sel_key
        if pick in keys:
            i = keys.index(pick)
            if self.cand_list.GetFirstSelected() != i:
                self._selecting = True
                try:
                    self.cand_list.Select(i)
                    self.cand_list.EnsureVisible(i)
                finally:
                    self._selecting = False
            self._sel_key = pick
        self._show_selected()

    def _on_cand_selected(self, event):
        i = event.GetIndex()
        if 0 <= i < len(self._cand_rows):
            if not self._selecting:
                self._user_picked = True
            row = self._cand_rows[i]
            self._sel_key = row["seed"]
            self._show_selected()

    def _show_selected(self):
        """Show the selected row's fast-routed board (else its placement)."""
        row = next((r for r in self._cand_rows
                    if r["seed"] == self._sel_key), None)
        if row is None:
            return
        caption = _cand_caption(row)
        board = row["fast"] or row["placed"]
        if board is None:
            self._shown = None
            self.preview.clear(caption)
            return
        png = os.path.join(
            self.last_workdir, "_gui_preview",
            f"s{row['seed']}{'_fast' if row['fast'] else ''}.png")
        if os.path.isfile(png) and _safe_mtime(png) >= _safe_mtime(board):
            shown = (png, _safe_mtime(png), caption)
            if shown != self._shown:
                self._shown = shown
                self.preview.show_png(png, caption)
            return
        self.preview.caption.SetLabel(caption + " (rendering...)")
        self._request_render(board, png)

    def _request_render(self, board, png):
        """Render one board on ONE daemon thread at a time; the newest
        request waits behind a render in flight."""
        if self._render_busy:
            self._render_next = (board, png)
            return
        self._render_busy = True
        threading.Thread(target=self._render_work, args=(board, png),
                         daemon=True).start()

    def _render_work(self, board, png):
        ok = False
        try:
            from route_render import render_board_file
            os.makedirs(os.path.dirname(png), exist_ok=True)
            ok = bool(render_board_file(board, png, size=900, quiet=True))
        except ImportError:
            ok = None   # Pillow/renderer missing: previews off
        except Exception:
            ok = False  # torn or unparseable board mid-write: retried later
        wx.CallAfter(self._render_done, ok)

    def _render_done(self, ok):
        if not self:
            return
        self._render_busy = False
        if ok is None:
            self.preview.caption.SetLabel(
                "preview unavailable (renderer/Pillow not importable)")
            return
        nxt, self._render_next = self._render_next, None
        if nxt is not None:
            self._request_render(*nxt)
        else:
            self._show_selected()

    def _load_report(self, report_path):
        try:
            with open(report_path, "r", encoding="utf-8", errors="replace") as f:
                text = f.read()
        except OSError as e:
            self._append_transcript(f"\n(could not read report: {e})\n")
            return
        if len(text) > 200 * 1024:
            text = text[:200 * 1024] + "\n... (report truncated; open the file)"
        self._append_transcript(f"\n=== REPORT ({report_path}) ===\n{text}\n")

    # ---------------------------------------------------------- apply-back

    def _on_apply(self, event):
        outputs = self._pending_result
        if not outputs or not outputs.get("board"):
            return
        try:
            import pcbnew
        except ImportError:
            wx.MessageBox("Not running inside pcbnew.", "Placement",
                          wx.OK | wx.ICON_ERROR)
            return
        board = pcbnew.GetBoard()
        if board is None:
            wx.MessageBox("Board is no longer open.", "Placement",
                          wx.OK | wx.ICON_ERROR)
            return
        from kicad_parser import parse_kicad_pcb
        try:
            result_pcb = parse_kicad_pcb(outputs["board"])
        except Exception as e:
            wx.MessageBox(f"Could not parse the result board: {e}",
                          "Placement", wx.OK | wx.ICON_ERROR)
            return

        moves = self._pose_moves(board, result_pcb, pcbnew)
        locks = self._lock_changes(board, result_pcb)
        with_copper = self._mode == "place_route"

        def _refs(refs):
            return ", ".join(refs[:30]) + (", ..." if len(refs) > 30 else "")
        lines = [f"{len(moves)} footprint(s) will move."]
        if moves:
            lines.append(_refs([r for r, _fp, _p in moves]))
        # #1054: the run's locks are part of its result (`place_pose
        # lock/unlock` stamps and strips them), so Apply mirrors them -- and
        # NAMES them first, an unlock above all.
        to_lock = [r for r, _fp, want in locks if want]
        to_unlock = [r for r, _fp, want in locks if not want]
        if to_lock:
            lines.append(f"{len(to_lock)} footprint(s) will be LOCKED, as the "
                         f"result board has them: {_refs(to_lock)}")
        if to_unlock:
            lines.append(f"{len(to_unlock)} footprint(s) will be UNLOCKED, as "
                         f"the result board has them: {_refs(to_unlock)}")
        # #829: say what was NOT applied. A skip the dialog does not mention is
        # a live board that silently disagrees with the result board.
        _skipped = getattr(self, '_outline_skipped', None)
        if _skipped:
            lines.append(
                f"NOT applied: {', '.join(_skipped)} -- these draw the board "
                f"outline, so moving them would resize the board. Their poses "
                f"in the result board are left alone here; edit part and "
                f"outline together in KiCad if one really must move (#829).")
        if with_copper:
            lines.append("Place + Route additionally makes the live board's "
                         "tracks and vias the run's routing: copper the run "
                         "kept unchanged stays as it is (groups included), "
                         "the rest is replaced. Replaced arc tracks come "
                         "back as short straight segments (the parser "
                         "linearizes arcs) - if the run changed any, open "
                         "the result board file instead.")
        lines.append("The live board may have changed since the run started - "
                     "this diff is against its state right now. Continue?")
        if wx.MessageBox("\n\n".join(lines), "Apply Placement Result",
                         wx.YES_NO | wx.ICON_WARNING) != wx.YES:
            return

        for _ref, fp, parsed in moves:
            self._apply_pose(fp, parsed, pcbnew)
        for _ref, fp, want in locks:
            fp.SetLocked(want)
        tracks_added = vias_added = 0
        if with_copper:
            tracks_added, vias_added = self._apply_copper(board, result_pcb, pcbnew)

        board.BuildConnectivity()
        from .gui_utils import refill_all_zones
        refill_all_zones(board)
        pcbnew.Refresh()
        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()
        if self.on_complete:
            self.on_complete()
        parsed_zones = len(getattr(result_pcb, "zones", []) or [])
        live_zones = board.Zones().size() if hasattr(board.Zones(), "size") else len(list(board.Zones()))
        if parsed_zones > live_zones:
            self.append_log(
                f"WARNING: result board has {parsed_zones} zones, live board "
                f"{live_zones} - new zones are NOT imported by Apply; open "
                "the result board file to get them.\n")
        msg = f"Applied: {len(moves)} footprint pose(s)"
        if locks:
            msg += (f", {len(to_lock)} lock(s) set and {len(to_unlock)} "
                    f"cleared")
        if with_copper:
            msg += (f", {tracks_added} track(s), {vias_added} via(s) added "
                    f"(unchanged copper kept)")
        msg += ". Use Edit -> Undo in pcbnew to revert."
        self.set_status(msg)
        self.append_log("Placement apply: " + msg + "\n")

    def _pose_moves(self, board, result_pcb, pcbnew):
        """[(ref, live_fp, parsed_fp)] for every footprint whose pose differs.

        #829: a footprint that draws the board's own outline is skipped here.
        This applier is a SECOND mover -- it writes poses onto the live board
        with SetPosition/SetOrientationDegrees/Flip and never goes through
        `placement.writer.write_placed_output`, so the CLI-side guard does not
        reach it. And it cannot be assumed safe because the CLI already
        refused: the move list is a diff of the RESULT board against the LIVE
        board, not a replay of what the CLI decided, so a live board that has
        drifted from the staged input (the user nudged something in KiCad while
        the run was going) produces moves the CLI never proposed. `Flip` is the
        worst of them -- it MIRRORS the owned geometry, which no amount of
        inspection recovers.
        """
        moves = []
        # Resolve by the PCBData key, not by the bare reference (#726): two
        # blocks sharing a reference are `TP4` and `TP4~2`, and
        # FindFootprintByReference would hand both of them the first block.
        from .gui_utils import live_footprints_by_key
        live = live_footprints_by_key(board)
        unreachable = []
        self._outline_skipped = []
        for ref, parsed in sorted(result_pcb.footprints.items()):
            if getattr(parsed, 'owns_board_outline', False):
                # #829, and it goes BEFORE the lookup: an outline owner is not
                # ours to move, so whether the live board can be made to find
                # it is beside the point.
                #
                # DISCLOSED, not silent. The confirmation dialog is built from
                # `moves`, so a footprint dropped here would otherwise appear
                # nowhere at all and the live board would diverge from the
                # result board the user can open beside it. The CLI writer
                # raises for the same reason; this front cannot raise mid-apply
                # without leaving the board half-updated, so it reports.
                self._outline_skipped.append(ref)
                continue
            fp = live.get(ref) or board.FindFootprintByReference(ref)
            if fp is None:
                # Part exists only in the result; never invent parts. Collected
                # rather than dropped in silence, for the same reason the CLI
                # writer reports an unapplied placement (#726): a pose the run
                # produced and the board never received is the failure nobody
                # looks at.
                unreachable.append(ref)
                continue
            pos = fp.GetPosition()
            live_x = pcbnew.ToMM(pos.x)
            live_y = pcbnew.ToMM(pos.y)
            live_rot = fp.GetOrientationDegrees() % 360
            live_back = fp.IsFlipped()
            parsed_back = str(parsed.layer or "F.Cu").startswith("B")
            if (abs(live_x - parsed.x) > 1e-4 or abs(live_y - parsed.y) > 1e-4
                    or abs((live_rot - (parsed.rotation or 0) % 360 + 180) % 360 - 180) > 1e-3
                    or live_back != parsed_back):
                moves.append((ref, fp, parsed))
        if unreachable:
            print("WARNING: %d part(s) in the result are not on the live "
                  "board, so their poses were not applied: %s"
                  % (len(unreachable), ', '.join(sorted(unreachable)[:12])))
        return moves

    def _lock_changes(self, board, result_pcb):
        """[(ref, live_fp, want_locked)] for every footprint whose KiCad lock
        differs from the result board's (#1054).

        A SEPARATE pass from `_pose_moves`, because a lock is not a pose:
        `place_pose lock` stamps `(locked yes)` on a part and changes nothing
        else, so a pose diff never sees it. The result board is mirrored in
        BOTH directions. Placement
        unlocks on purpose -- `place_pose unlock` is the only way to move a
        KiCad-locked part, and make_unplaced strips each moved part's lock --
        so keeping the live lock would ship a part locked at a pose nobody
        locked it at. The caller names every change before applying any.

        An outline owner (#829) is included: `_pose_moves` skips it because
        moving it would resize the board, and a lock moves nothing. A part
        missing from the live board was already reported by `_pose_moves`.
        """
        from .gui_utils import live_footprints_by_key
        live = live_footprints_by_key(board)
        changes = []
        for ref, parsed in sorted(result_pcb.footprints.items()):
            fp = live.get(ref) or board.FindFootprintByReference(ref)
            if fp is None:
                continue
            want = bool(getattr(parsed, 'locked', False))
            if bool(fp.IsLocked()) != want:
                changes.append((ref, fp, want))
        return changes

    def _apply_pose(self, fp, parsed, pcbnew):
        from kicad_parser import mm_to_iu
        parsed_back = str(parsed.layer or "F.Cu").startswith("B")
        if fp.IsFlipped() != parsed_back:
            fp.Flip(fp.GetPosition(), False)
        fp.SetPosition(pcbnew.VECTOR2I(mm_to_iu(parsed.x), mm_to_iu(parsed.y)))
        fp.SetOrientationDegrees(parsed.rotation or 0)

    def _apply_copper(self, board, result_pcb, pcbnew):
        """Make the live board's tracks and vias the result board's copper.

        The result board is the authority: a Place + Route run's contract is
        that placement invalidates prior copper, so live copper the result
        does not carry is removed and copper it adds is created. A live
        track, arc or via the result carries UNCHANGED -- same geometry,
        layer, width and net, and for a via the same size, drill, layer pair
        and type -- is KEPT rather than removed and recreated (#1279).
        Recreating gave every item a new uuid, which took it out of any KiCad
        group: a `sense:<REF>.<PAD>` group lost its members, KiCad dropped
        the emptied group on save, and the Kelvin branch it declared was no
        longer protected; an unchanged arc also came back as chords. A kept
        via takes the result's protection spec. A created item takes its
        KiCad lock from the result's `locked`, which is the file's own
        `(locked ...)` token (#521) -- a sense hold is `held_by`, never a lock,
        so Apply never writes one. RemoveNative, never Remove (see the
        SWIG-registry warning in swig_gui). Returns (tracks_added,
        vias_added).
        """
        from kicad_parser import mm_to_iu, pcbnew_track_copper
        from .swig_gui import _build_layer_mappings
        from .gui_utils import apply_via_protection, set_via_layers
        name_to_id, _ = _build_layer_mappings()

        # Result-board net ids can differ from live net codes; map by name.
        def live_net_code(net_id):
            net = result_pcb.nets.get(net_id)
            if net is not None:
                live = board.FindNet(net.name)
                if live is not None:
                    return live.GetNetCode()
            return net_id

        def result_net_name(net_id):
            net = result_pcb.nets.get(net_id)
            return net.name if net is not None else ''

        # Keys compare in internal units, the resolution both boards hold.
        def seg_key(s, net_name):
            a = (mm_to_iu(s.start_x), mm_to_iu(s.start_y))
            b = (mm_to_iu(s.end_x), mm_to_iu(s.end_y))
            return (min(a, b), max(a, b), mm_to_iu(s.width), s.layer, net_name)

        def via_key(v, net_name):
            return (mm_to_iu(v.x), mm_to_iu(v.y), mm_to_iu(v.size),
                    mm_to_iu(v.drill), tuple(v.layers or ()),
                    getattr(v, 'via_type', 'through'), net_name)

        # The result's copper, by key. Copper GRAPHICS parse into segments too
        # (#337 board gr_*, #908 footprint fp_*): they are drawings, not
        # tracks. The live board keeps its own (a footprint's travel with its
        # pose), and as a PCB_TRACK each would be a net-0 track over its own
        # pads.
        want_segs, want_vias = {}, {}
        for seg in result_pcb.segments:
            if getattr(seg, "graphic", False):
                continue
            want_segs.setdefault(seg_key(seg, result_net_name(seg.net_id)),
                                 []).append(seg)
        for via in result_pcb.vias:
            want_vias.setdefault(via_key(via, result_net_name(via.net_id)),
                                 []).append(via)

        # The live copper through the extractor every live read uses, so an
        # arc's pieces are chorded exactly as the result's parse chords them;
        # `owners` maps each piece back to its one live item.
        owners = {}
        live_segs, live_vias = pcbnew_track_copper(board, owners=owners)
        items = {}
        for s in live_segs:
            it = owners[id(s)]
            items.setdefault(id(it), (it, 'seg', []))[2].append(
                seg_key(s, it.GetNetname()))
        for v in live_vias:
            it = owners[id(v)]
            items[id(it)] = (it, 'via', [via_key(v, it.GetNetname())])

        kept = []
        for it, kind, keys in items.values():
            pool = want_segs if kind == 'seg' else want_vias
            need = {}
            for k in keys:
                need[k] = need.get(k, 0) + 1
            if all(len(pool.get(k, ())) >= n for k, n in need.items()):
                taken = [pool[k].pop() for k in keys]
                kept.append((it, kind, taken))
            else:
                board.RemoveNative(it)
        for it, kind, taken in kept:
            if kind == 'via':
                # Keep the result's tenting/plugging spec (#489 §8); an empty
                # spec leaves the via inheriting the board's, as it was.
                apply_via_protection(it, getattr(taken[0], "tenting_attrs", None))

        tracks_added = vias_added = 0
        for seg in (s for segs in want_segs.values() for s in segs):
            track = pcbnew.PCB_TRACK(board)
            # mm_to_iu, never FromMM(round(...)) (#493 item 5 / #362).
            track.SetStart(pcbnew.VECTOR2I(mm_to_iu(seg.start_x),
                                           mm_to_iu(seg.start_y)))
            track.SetEnd(pcbnew.VECTOR2I(mm_to_iu(seg.end_x),
                                         mm_to_iu(seg.end_y)))
            track.SetWidth(mm_to_iu(seg.width))
            track.SetLayer(name_to_id.get(seg.layer, pcbnew.F_Cu))
            track.SetNetCode(live_net_code(seg.net_id))
            # #521: user-pinned copper must come back pinned - the whole
            # protection chain (never-rippable, no override) rests on it.
            track.SetLocked(bool(getattr(seg, "locked", False)))
            board.Add(track)
            tracks_added += 1
        for via in (v for vias in want_vias.values() for v in vias):
            pcb_via = pcbnew.PCB_VIA(board)
            pcb_via.SetPosition(pcbnew.VECTOR2I(mm_to_iu(via.x), mm_to_iu(via.y)))
            pcb_via.SetWidth(mm_to_iu(via.size))
            pcb_via.SetDrill(mm_to_iu(via.drill))
            pcb_via.SetNetCode(live_net_code(via.net_id))
            if getattr(via, "layers", None) and len(via.layers) >= 2:
                set_via_layers(pcb_via, name_to_id.get(via.layers[0], pcbnew.F_Cu),
                               name_to_id.get(via.layers[-1], pcbnew.B_Cu),
                               getattr(via, "via_type", None))
            pcb_via.SetLocked(bool(getattr(via, "locked", False)))  # #521
            # Keep a via's own tenting/plugging spec (#489 §8).
            apply_via_protection(pcb_via, getattr(via, "tenting_attrs", None))
            board.Add(pcb_via)
            vias_added += 1
        print(f"Placement apply: {len(kept)} unchanged track(s)/via(s) kept, "
              f"{len(items) - len(kept)} removed, {tracks_added} track(s) and "
              f"{vias_added} via(s) added")
        return tracks_added, vias_added

    # ------------------------------------------------------ beautify labels

    def _on_action(self, event):
        """Beautify Labels: canonical worker-thread pattern (plan-drivable)."""
        if self._busy():
            wx.MessageBox("A placement operation is already running.",
                          "Placement", wx.OK | wx.ICON_WARNING)
            return
        try:
            from placement.labels import beautify_labels, LabelConfig  # noqa: F401
        except ImportError:
            self.set_status(
                "Beautify Labels: the label engine (placement.labels) could "
                "not be imported.")
            return
        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()
        self._cancel_requested = False
        self.action_btn.Disable()
        self.stop_btn.Enable()
        self.set_status("Beautifying labels...")
        config = self.labels_options.get_config()
        self._routing_thread = threading.Thread(
            target=self._run_beautify, args=(config,), daemon=True)
        self._routing_thread.start()

    def run_beautify_labels(self, log=None):
        """Programmatic entry. NOT synchronous (unlike run_cap_optimization):
        it starts the worker thread and returns - a future plan action must
        use a busy predicate that polls action_btn, never `lambda: False`."""
        self._on_action(None)

    def _run_beautify(self, config):
        from .gui_utils import StdoutRedirector
        original_stdout = sys.stdout
        sys.stdout = StdoutRedirector(self.append_log, original_stdout)
        result = None
        error = None
        try:
            from placement.labels import beautify_labels, LabelConfig
            import pcbnew
            from kicad_parser import build_pcb_data_from_board
            board = pcbnew.GetBoard()
            if board is not None:
                pcb_data = build_pcb_data_from_board(board)
            else:
                pcb_data = self.pcb_data  # headless tests
            def progress(current, total, label=""):
                wx.CallAfter(self._update_progress, current, total, label)
                time.sleep(0.01)  # let the main thread paint
            result = beautify_labels(
                pcb_data, self.board_filename,
                LabelConfig(**config),
                cancel_check=lambda: self._cancel_requested,
                progress_callback=progress)
        except Exception as e:
            error = str(e)
        finally:
            sys.stdout = original_stdout
        wx.CallAfter(self._beautify_finished, result, error)

    def _update_progress(self, current, total, label):
        if not self:
            return
        if total:
            self.progress_bar.SetValue(min(100, int(current * 100 / total)))
        else:
            self.progress_bar.Pulse()
        if label:
            self.set_status(f"Beautify: {label}")

    def _beautify_finished(self, result, error):
        if not self:
            return
        self._routing_thread = None
        self.progress_bar.SetValue(0)
        self.action_btn.Enable()
        self.stop_btn.Disable()
        if self._cancel_requested:
            self.set_status("Beautify cancelled.")
            return
        if error is not None:
            self.set_status(f"Beautify failed: {error}")
            self.append_log(f"Beautify labels failed: {error}\n")
            return
        applied = self._apply_labels_to_board(result)
        summary = result.get("summary", {})
        counts = {k: summary.get(k, 0)
                  for k in ("placed", "shrunk", "rotated", "shrunk_rotated")}
        unchanged_refs = summary.get("unchanged_refs", [])
        status = (f"Labels: {sum(counts.values())} placed "
                  f"({counts['shrunk'] + counts['shrunk_rotated']} shrunk, "
                  f"{counts['rotated'] + counts['shrunk_rotated']} rotated), "
                  f"{len(unchanged_refs)} unplaceable, {applied} applied")
        self.set_status(status)
        self.append_log("Beautify labels: " + status + "\n")
        for ref in unchanged_refs[:20]:
            self.append_log(f"  unplaceable: {ref}\n")
        if self.on_complete:
            self.on_complete()

    def _apply_labels_to_board(self, result):
        """Apply LabelResults to the live board's Reference fields.

        Writes the FILE-frame values the CLI writer would emit (position is
        absolute; the angle is the node's own (at) rotation, which pcbnew
        stores verbatim on the text item) - so CLI and GUI outputs match by
        construction, whatever the angle-storage semantics turn out to be.
        """
        try:
            import pcbnew
        except ImportError:
            return 0
        board = pcbnew.GetBoard()
        if board is None:
            return 0
        from kicad_parser import mm_to_iu
        from .gui_utils import live_footprints_by_key
        _live_labels = live_footprints_by_key(board)
        applied = 0
        for r in result.get("results", []):
            action = getattr(r, "action", None) or (
                r.get("action") if isinstance(r, dict) else None)
            if action not in ("placed", "shrunk", "rotated", "shrunk_rotated"):
                continue
            get = (lambda k, _r=r: getattr(_r, k, None)) \
                if not isinstance(r, dict) else r.get
            # Same #726 resolution as _pose_moves: a label result names a
            # PCBData key, which for a duplicated reference is `TP4~2`.
            _k = get("reference")
            fp = _live_labels.get(_k) or board.FindFootprintByReference(_k)
            if fp is None:
                continue
            ref = fp.Reference()
            size = mm_to_iu(get("size"))
            ref.SetTextSize(pcbnew.VECTOR2I(size, size))
            ref.SetTextThickness(mm_to_iu(get("thickness")))
            ref.SetPosition(pcbnew.VECTOR2I(mm_to_iu(get("world_x")),
                                            mm_to_iu(get("world_y"))))
            # file_rotation is the stored (at ...) angle, which the KiCad 10
            # probe showed pcbnew's GetTextAngle/SetTextAngle round-trips
            # verbatim (ABSOLUTE, not footprint-relative).
            angle = float(get("file_rotation") or 0)
            try:
                ref.SetTextAngle(pcbnew.EDA_ANGLE(angle, pcbnew.DEGREES_T))
            except (AttributeError, TypeError):
                ref.SetTextAngleDegrees(angle)
            if hasattr(ref, "SetHorizJustify"):
                ref.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
                ref.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_CENTER)
            applied += 1
        if applied:
            pcbnew.Refresh()
        return applied

    # -------------------------------------------------------- cancel/close

    def _on_stop(self, event):
        """Cancel the running placement run or label job (Routing: Cancel)."""
        if self._runner is not None and self._runner.is_running():
            self._runner.cancel()
            self.set_status(
                "Cancelling — partial artifacts are kept in the workdir...")
        elif self._routing_thread is not None and self._routing_thread.is_alive():
            self._cancel_requested = True
            self.set_status("Cancelling...")
        self.stop_btn.Disable()

    def _on_close(self, event):
        """Close the dialog like the other tabs. A placement run in progress
        is cancelled by the close (shutdown), so ask first."""
        if self._busy() and wx.MessageBox(
                "A placement job is still running. Closing the dialog cancels "
                "it (a run's partial artifacts are kept in its folder).\n\n"
                "Close anyway?", "Placement",
                wx.YES_NO | wx.NO_DEFAULT | wx.ICON_WARNING) != wx.YES:
            return
        top = wx.GetTopLevelParent(self)
        if isinstance(top, wx.Dialog) and top.IsModal():
            top.EndModal(wx.ID_CANCEL)
        else:
            top.Close()

    def _on_destroy(self, event):
        if event.GetEventObject() is self:
            self.shutdown()
        event.Skip()

    def shutdown(self):
        """Stop background work before the dialog goes away (AITab pattern)."""
        if self._runner is not None:
            self._runner.cancel()
        if self._monitor is not None:
            self._monitor.stop()
        self._cancel_requested = True

    # ------------------------------------------------- session restore

    def restore_last_workdir(self, workdir):
        """Re-surface a previous session's run (settings restore)."""
        if not workdir or not os.path.isdir(workdir):
            return
        self.last_workdir = workdir
        outputs = scan_workdir_outputs(workdir)
        self._set_outputs(report=outputs.get("report"), workdir=workdir)
        self.refresh_candidates(kept_board=outputs.get("board"), final=True)
        if outputs.get("board"):
            self._pending_result = outputs
            self._mode = ("place_route"
                          if os.path.basename(workdir).endswith("place_route")
                          else "place")
            self.apply_btn.Enable(True)
            self.set_status(
                f"Previous run's artifacts loaded from {workdir}")
