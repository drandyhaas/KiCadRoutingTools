"""
KiCad Routing Tools - Placement sub-tab (AI tab notebook).

Two Claude-Code-driven actions ("Place" via /plan-pcb-placement, "Place +
Route" via /plan-pcb-placement-and-routing) plus the non-AI "Beautify Labels"
action (issue #481). The AI runs are headless, write into a persistent
workdir (placement_run.create_workdir), take minutes to hours, and are
observed from outside: a 1 s monitor timer merges the stream-json transcript,
the converge ledger tail, and newly-appearing board artifacts into an
elapsed/stage status line, and renders preview frames from new boards while
the run is going. At the end the movie and REPORT.md are surfaced in the tab
and the result can be explicitly applied back to the live board.

Claude Code is REQUIRED for the two AI actions: they are pinned to the Claude
backend regardless of the AI tab's backend selection (the opencode analysis
agent denies edits, and these skills must write), and the buttons are disabled
with an install hint when the CLI is missing.
"""

import os
import re
import sys
import threading
import time
from collections import deque

import wx

from .ai_backend import BACKENDS, BACKEND_IDS
from . import placement_run
from .placement_run import (
    PLACEMENT_ALLOWED_TOOLS, PLACEMENT_SUPPORTED_BACKENDS,
    build_placement_prompt, create_workdir, derive_stage,
    list_board_artifacts, newest_stable_board, parse_placement_result,
    read_ledger_tail, scan_workdir_outputs, stage_inputs,
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
    """Options for Beautify Labels (#481).

    Control attribute names deliberately equal the future plan-step param
    names (label_size, label_orientation, ...) - the AI plan executor applies
    snake_case params by control attribute name.
    """

    def __init__(self, parent):
        super().__init__(parent)
        grid = wx.FlexGridSizer(cols=2, vgap=3, hgap=6)
        grid.AddGrowableCol(1)

        def row(label, ctrl, tooltip):
            grid.Add(wx.StaticText(self, label=label), 0,
                     wx.ALIGN_CENTER_VERTICAL)
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
            "Orientation:", wx.Choice(self, choices=["horizontal", "vertical"], size=(78, -1)),
            "Preferred label direction; the other direction is a fallback "
            "when the preferred one does not fit.")
        self.label_orientation.SetSelection(0)
        self.label_max_distance = row(
            "Max dist:",
            wx.SpinCtrlDouble(self, min=0.5, max=10.0, initial=2.0, inc=0.25, size=(78, -1)),
            "How far beyond the part's courtyard a label may be pushed.")
        self.label_include_hidden = wx.CheckBox(self, label="Include hidden labels")
        self.label_include_hidden.SetToolTip(
            "Also place (and un-hide) labels that are currently hidden.")
        self.label_skip_locked = wx.CheckBox(self, label="Skip locked parts")
        self.label_skip_locked.SetToolTip(
            "Leave the labels of KiCad-locked footprints untouched.")
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(grid, 0, wx.EXPAND)
        sizer.Add(self.label_include_hidden, 0, wx.TOP, 3)
        sizer.Add(self.label_skip_locked, 0, wx.TOP, 3)
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


class FramePreviewPanel(wx.Panel):
    """Board preview: live frames during a run, a scrubber afterwards."""

    PLAY_MS = 500

    def __init__(self, parent):
        super().__init__(parent)
        self._frames = []       # PNG paths, oldest first
        self._index = -1
        self._movie = None
        self._report = None
        self._workdir = None
        self._play_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_play_tick, self._play_timer)

        sizer = wx.BoxSizer(wx.VERTICAL)
        # The frame area gets the LISTBOX colour and a sunken border so it
        # reads as an inset panel, the way the Routing tab's Planned Steps
        # does. That box is a wx.CheckListBox, which paints its own dark
        # background; a StaticBitmap with no image paints nothing, so before
        # a run the whole pane was flat dialog grey with no visible frame.
        self.frame_panel = wx.Panel(self, style=wx.BORDER_SUNKEN)
        self.frame_panel.SetBackgroundColour(
            wx.SystemSettings.GetColour(wx.SYS_COLOUR_LISTBOX))
        _fp = wx.BoxSizer(wx.VERTICAL)
        self.bitmap = wx.StaticBitmap(self.frame_panel)
        _fp.Add(self.bitmap, 1, wx.EXPAND)
        self.frame_panel.SetSizer(_fp)
        self.frame_panel.SetMinSize((480, 360))
        sizer.Add(self.frame_panel, 1, wx.EXPAND | wx.ALL, 2)
        self.caption = wx.StaticText(self, label="No run yet.")
        sizer.Add(self.caption, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 4)

        btns = wx.BoxSizer(wx.HORIZONTAL)
        self.prev_btn = wx.Button(self, label="<", size=(32, -1))
        self.play_btn = wx.Button(self, label="Play", size=(56, -1))
        self.next_btn = wx.Button(self, label=">", size=(32, -1))
        self.open_movie_btn = wx.Button(self, label="Open Movie")
        self.open_report_btn = wx.Button(self, label="Open Report")
        self.open_workdir_btn = wx.Button(self, label="Open Folder")
        for b in (self.prev_btn, self.play_btn, self.next_btn):
            btns.Add(b, 0, wx.RIGHT, 3)
        btns.AddStretchSpacer()
        for b in (self.open_movie_btn, self.open_report_btn, self.open_workdir_btn):
            btns.Add(b, 0, wx.LEFT, 3)
        sizer.Add(btns, 0, wx.EXPAND | wx.ALL, 3)
        self.SetSizer(sizer)

        self.prev_btn.Bind(wx.EVT_BUTTON, lambda e: self._step(-1))
        self.next_btn.Bind(wx.EVT_BUTTON, lambda e: self._step(+1))
        self.play_btn.Bind(wx.EVT_BUTTON, self._on_play)
        self.open_movie_btn.Bind(wx.EVT_BUTTON, lambda e: self._launch(self._movie))
        self.open_report_btn.Bind(wx.EVT_BUTTON, lambda e: self._launch(self._report))
        self.open_workdir_btn.Bind(wx.EVT_BUTTON, lambda e: self._launch(self._workdir))
        self._refresh_buttons()

    # ------------------------------------------------------------- frames

    def show_png(self, path, caption=None):
        """Display one PNG scaled to the panel (aspect preserved)."""
        try:
            img = wx.Image(path, wx.BITMAP_TYPE_PNG)
            if not img.IsOk():
                return False
        except Exception:
            return False
        # Scale against the FRAME's client area, not the StaticBitmap:
        # the bitmap is now a child of the inset panel and reports 0x0
        # until it holds an image.
        w, h = self.frame_panel.GetClientSize()
        w = max(w, 200)
        h = max(h, 150)
        iw, ih = img.GetWidth(), img.GetHeight()
        scale = min(w / iw, h / ih)
        # IMAGE_QUALITY_NORMAL: HIGH is very slow (about_tab precedent).
        img = img.Scale(max(1, int(iw * scale)), max(1, int(ih * scale)),
                        wx.IMAGE_QUALITY_NORMAL)
        self.bitmap.SetBitmap(wx.Bitmap(img))
        if caption is not None:
            self.caption.SetLabel(caption)
        self.frame_panel.Layout()
        self.Layout()
        return True

    def add_frame(self, path, caption=None):
        """A new live frame: append and jump to it."""
        if path not in self._frames:
            self._frames.append(path)
        self._index = len(self._frames) - 1
        self.show_png(path, caption)
        self._refresh_buttons()

    def set_frames(self, paths):
        """Post-run scrubber source; shows the newest frame."""
        self._frames = [p for p in paths if os.path.isfile(p)]
        self._index = len(self._frames) - 1
        if self._index >= 0:
            self.show_png(self._frames[self._index],
                          os.path.basename(self._frames[self._index]))
        self._refresh_buttons()

    def set_outputs(self, movie=None, report=None, workdir=None):
        self._movie = movie if movie and os.path.isfile(movie) else None
        self._report = report if report and os.path.isfile(report) else None
        self._workdir = workdir if workdir and os.path.isdir(workdir) else None
        self._refresh_buttons()

    def shutdown(self):
        self._play_timer.Stop()

    # ------------------------------------------------------------ internal

    def _refresh_buttons(self):
        has = len(self._frames) > 1
        self.prev_btn.Enable(has)
        self.next_btn.Enable(has)
        self.play_btn.Enable(has)
        self.open_movie_btn.Enable(self._movie is not None)
        self.open_report_btn.Enable(self._report is not None)
        self.open_workdir_btn.Enable(self._workdir is not None)

    def _step(self, delta):
        if not self._frames:
            return
        self._index = (self._index + delta) % len(self._frames)
        self.show_png(self._frames[self._index],
                      os.path.basename(self._frames[self._index]))

    def _on_play(self, event):
        if self._play_timer.IsRunning():
            self._play_timer.Stop()
            self.play_btn.SetLabel("Play")
        elif self._frames:
            self._play_timer.Start(self.PLAY_MS)
            self.play_btn.SetLabel("Stop")

    def _on_play_tick(self, event):
        self._step(+1)

    def _launch(self, path):
        if path:
            wx.LaunchDefaultApplication(path)


class PlacementRunMonitor:
    """Polls a placement run's workdir + transcript into status/preview.

    One 1000 ms wx.Timer owned by the tab; the artifact scan runs every
    second tick. Preview rendering happens on ONE daemon thread at a time
    (skip-to-newest by construction: the next scan picks whatever is newest).
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
        self._artifacts = []
        self._last_rendered = None   # (path, mtime) of the last render
        self._render_busy = False
        self._final_pending = None   # board queued while a render is in flight
        self._preview_dir = os.path.join(workdir, "_gui_preview")
        self._preview_ok = True

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

    # ------------------------------------------------------------ internal

    def _on_tick(self, event):
        tab = self.tab
        if not tab:
            self._timer.Stop()
            return
        self._ticks += 1
        if self._ticks % 2 == 0:
            arts = list_board_artifacts(self.workdir)
            # Compare against the PREVIOUS scan (self._artifacts) - a board
            # is previewable one scan (~2 s) after its size stops changing.
            self._maybe_render(newest_stable_board(arts, self._artifacts))
            self._artifacts = arts
        rows, self._ledger_offset = read_ledger_tail(
            os.path.join(self.workdir, "ledger.jsonl"), self._ledger_offset)
        if rows:
            self._last_ledger_row = rows[-1]
        newest = os.path.basename(self._artifacts[-1][0]) if self._artifacts else None
        stage = derive_stage(self._transcript_tail, self._last_ledger_row, newest)
        status = (f"{MODE_LABELS.get(self.mode, self.mode)} — "
                  f"{_fmt_elapsed(time.monotonic() - self._start)} — {stage}")
        quiet = time.monotonic() - self._last_event
        if quiet > self.QUIET_AFTER_S:
            status += f" (last agent event {int(quiet // 60)}m ago)"
        tab.set_status(status)
        tab.progress_bar.Pulse()

    def _maybe_render(self, board_path, final=False):
        if not board_path or not self._preview_ok:
            return
        # Keyed on (path, mtime): fixed-name artifacts (placed.kicad_pcb is
        # rewritten every loop cycle) must re-render when their content does.
        key = (board_path, _safe_mtime(board_path))
        if key == self._last_rendered:
            return
        if self._render_busy:
            if final:
                self._final_pending = board_path  # rendered when the slot frees
            return
        self._render_busy = True
        self._last_rendered = key
        threading.Thread(target=self._render_work, args=(board_path,),
                         daemon=True).start()

    def _render_work(self, board_path):
        png = None
        try:
            from make_movie import board_snapshot
            os.makedirs(self._preview_dir, exist_ok=True)
            stem = os.path.splitext(os.path.basename(board_path))[0]
            out = os.path.join(self._preview_dir, stem + ".png")
            png = board_snapshot(board_path, out=out, size=900, quiet=True)
        except ImportError:
            self._preview_ok = False  # Pillow/renderer missing: previews off
        except Exception:
            png = None  # torn/unparseable board mid-write: skip silently
        wx.CallAfter(self._render_done, board_path, png)

    def _render_done(self, board_path, png):
        self._render_busy = False
        tab = self.tab
        if not tab:
            return
        if not self._preview_ok:
            tab.preview.caption.SetLabel(
                "live preview unavailable (renderer/Pillow not importable)")
            return
        if png and os.path.isfile(png):
            tab.preview.add_frame(png, os.path.basename(board_path))
        else:
            # Failed render (e.g. torn-but-size-stable read): un-blacklist so
            # the completed file gets another chance on a later scan.
            self._last_rendered = None
        if self._final_pending:
            pending, self._final_pending = self._final_pending, None
            self._maybe_render(pending, final=True)


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

        self._create_ui()
        self._refresh_cli_status()
        self.Bind(wx.EVT_WINDOW_DESTROY, self._on_destroy)

    # ----------------------------------------------------------------- UI

    def _create_ui(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        top = wx.BoxSizer(wx.HORIZONTAL)

        preview_box = wx.StaticBox(self, label="Run Preview")
        preview_sizer = wx.StaticBoxSizer(preview_box, wx.VERTICAL)
        self.preview = FramePreviewPanel(self)
        preview_sizer.Add(self.preview, 1, wx.EXPAND | wx.ALL, 2)
        top.Add(preview_sizer, 1, wx.EXPAND | wx.RIGHT, 8)

        # #GUI: the right column SCROLLS. Both boxes sit at proportion 0,
        # so the column demands its full natural height; without a scroller
        # a short dialog clipped the last rows of the Beautify Labels box
        # (the 'Skip locked parts' checkbox vanished under the action
        # button). Matches the Routing tab's shape -- fixed-width control
        # column beside a proportion-1 left pane -- but survives being
        # taller than the frame.
        sw = wx.ScrolledWindow(self, style=wx.VSCROLL)
        sw.SetScrollRate(0, 8)
        self.right_scroll = sw
        right = wx.BoxSizer(wx.VERTICAL)

        ai_box = wx.StaticBox(sw, label="AI")
        ai_sizer = wx.StaticBoxSizer(ai_box, wx.VERTICAL)

        self.cli_status_label = wx.StaticText(sw, label="")
        self.cli_status_label.Wrap(280)          # the Routing tab's wrap width
        ai_sizer.Add(self.cli_status_label, 0, wx.ALL, 5)

        # STACKED selectors in a 2-column grid -- ai_gui's sel_grid exactly.
        sel_grid = wx.FlexGridSizer(cols=2, hgap=5, vgap=5)
        sel_grid.AddGrowableCol(1)
        sel_grid.Add(wx.StaticText(sw, label="Backend:"), 0,
                     wx.ALIGN_CENTER_VERTICAL)
        # Plain labels, exactly like the Routing tab's backend choice. The
        # "(coming soon)" suffix widened this Choice to 184px against Routing's
        # ~110, and since it is the widest child it set the WHOLE column's
        # width (297 vs ~230). The same information is in the tooltip below,
        # and _on_backend_choice refuses an unsupported pick anyway, so the
        # suffix was costing layout without carrying its weight.
        self.backend_choice = wx.Choice(
            sw, choices=[BACKENDS[bid].label for bid in BACKEND_IDS])
        self.backend_choice.SetSelection(BACKEND_IDS.index(self.backend.id))
        self.backend_choice.SetToolTip(
            "Harness that drives the placement skills. Only Claude Code is "
            "supported today (placement runs must WRITE, and the other "
            "harnesses' agents deny edits); more harnesses later.")
        self.backend_choice.Bind(wx.EVT_CHOICE, self._on_backend_choice)
        sel_grid.Add(self.backend_choice, 0, wx.EXPAND)
        sel_grid.Add(wx.StaticText(sw, label="Model:"), 0,
                     wx.ALIGN_CENTER_VERTICAL)
        self.model_choice = wx.ComboBox(sw, value=DEFAULT_CHOICE, choices=[])
        sel_grid.Add(self.model_choice, 0, wx.EXPAND)
        sel_grid.Add(wx.StaticText(sw, label="Effort:"), 0,
                     wx.ALIGN_CENTER_VERTICAL)
        self.effort_choice = wx.ComboBox(sw, value=DEFAULT_CHOICE, choices=[])
        sel_grid.Add(self.effort_choice, 0, wx.EXPAND)
        ai_sizer.Add(sel_grid, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        # No combo population here: __init__ calls _refresh_cli_status() once
        # AFTER _create_ui, like the Routing tab. Doing it mid-construction
        # now touches place_btn, which does not exist yet.

        ai_sizer.Add(wx.StaticText(sw, label="Extra instructions:"), 0,
                     wx.LEFT | wx.TOP, 5)
        self.extra_instructions = wx.TextCtrl(
            sw, style=wx.TE_MULTILINE, size=(-1, 40))
        self.extra_instructions.SetToolTip(
            "Optional extra instructions passed to the agent (constraints, "
            "focus areas, budget hints).")
        ai_sizer.Add(self.extra_instructions, 0,
                     wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.place_btn = wx.Button(sw, label="Place (AI)")
        self.place_btn.SetToolTip(
            "Claude Code runs the /plan-pcb-placement skill headless on a "
            "snapshot of this board. Runs minutes to hours; artifacts land "
            "in a krt_placement folder next to the board file.")
        ai_sizer.Add(self.place_btn, 0,
                     wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        self.place_route_btn = wx.Button(sw, label="Place + Route (AI)")
        self.place_route_btn.SetToolTip(
            "Claude Code runs the /plan-pcb-placement-and-routing skill "
            "headless (place, then route, looping on the outcome). Runs "
            "minutes to hours; artifacts land in a krt_placement folder "
            "next to the board file.")
        ai_sizer.Add(self.place_route_btn, 0,
                     wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        right.Add(ai_sizer, 0, wx.EXPAND | wx.BOTTOM, 6)

        labels_box = wx.StaticBox(sw, label="Beautify Labels (no AI)")
        labels_sizer = wx.StaticBoxSizer(labels_box, wx.VERTICAL)
        self.labels_options = LabelsOptionsPanel(sw)
        labels_sizer.Add(self.labels_options, 0, wx.EXPAND | wx.ALL, 3)
        self.action_btn = wx.Button(sw, label="Beautify Labels")
        self.action_btn.SetToolTip(
            "Uniform, non-overlapping reference-designator silkscreen "
            "(#481). Moves/restyles only the labels, never the parts.")
        labels_sizer.Add(self.action_btn, 0, wx.EXPAND | wx.ALL, 3)
        right.Add(labels_sizer, 0, wx.EXPAND | wx.BOTTOM, 6)

        sw.SetSizer(right)
        right.Fit(sw)
        sw.SetMinSize((sw.GetBestSize().width + 20, 260))
        top.Add(sw, 0, wx.EXPAND)
        sizer.Add(top, 1, wx.EXPAND | wx.ALL, 5)

        # Status spans the FULL dialog width (not the narrow right column):
        # run statuses (elapsed + stage + summary) are long, and the column
        # both clipped them and ran out of height for the buttons below.
        # Apply/Close live here too - they are run-level controls, and moving
        # them out keeps the right column short enough to never clip.
        status_box = wx.StaticBox(self, label="Status")
        status_sizer = wx.StaticBoxSizer(status_box, wx.VERTICAL)
        status_row = wx.BoxSizer(wx.HORIZONTAL)
        self.status_text = wx.StaticText(self, label="Ready")
        # Two lines reserved so a rare wrap doesn't re-flow the page.
        self.status_text.SetMinSize(
            (-1, self.status_text.GetCharHeight() * 2 + 4))
        status_row.Add(self.status_text, 1, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 3)
        self.apply_btn = wx.Button(self, label="Apply Result to Board")
        self.apply_btn.Enable(False)
        self.apply_btn.SetToolTip(
            "Apply the finished run's board back to the live board "
            "(footprint poses; Place + Route also replaces all tracks/vias). "
            "Asks for confirmation first.")
        self.cancel_btn = wx.Button(self, label="Close")
        status_row.Add(self.apply_btn, 0, wx.ALL, 3)
        status_row.Add(self.cancel_btn, 0, wx.ALL, 3)
        status_sizer.Add(status_row, 0, wx.EXPAND)
        self.progress_bar = wx.Gauge(self, range=100)
        status_sizer.Add(self.progress_bar, 0, wx.EXPAND | wx.ALL, 3)
        sizer.Add(status_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 5)

        self.transcript_ctrl = wx.TextCtrl(
            self, style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_RICH2)
        self.transcript_ctrl.SetFont(
            wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL,
                    wx.FONTWEIGHT_NORMAL))
        self.transcript_ctrl.SetMinSize((-1, 140))
        sizer.Add(self.transcript_ctrl, 0, wx.EXPAND | wx.ALL, 5)
        self.SetSizer(sizer)

        self.place_btn.Bind(wx.EVT_BUTTON, lambda e: self._start_ai_run("place"))
        self.place_route_btn.Bind(wx.EVT_BUTTON,
                                  lambda e: self._start_ai_run("place_route"))
        self.action_btn.Bind(wx.EVT_BUTTON, self._on_action)
        self.apply_btn.Bind(wx.EVT_BUTTON, self._on_apply)
        self.cancel_btn.Bind(wx.EVT_BUTTON, self._on_cancel_or_close)

    def set_status(self, text):
        """Set the status line, wrapped to the tab's current width.

        StaticText does not wrap on its own; the box is full-width so this
        only matters for unusually long summaries.
        """
        self.status_text.SetLabel(text)
        self.status_text.Wrap(max(500, self.GetClientSize()[0] - 60))
        self.Layout()

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
        status line WITH the CLI path, then the selected backend's own
        model/effort entries and tooltips."""
        cli = self.backend.find_cli()
        if cli:
            self.cli_status_label.SetLabel(
                f"{self.backend.label} CLI found: {cli}")
        else:
            self.cli_status_label.SetLabel(
                self.backend.not_found_message() + " Then reopen this dialog.")
        self.cli_status_label.Wrap(280)
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

    def _set_running_ui(self, running):
        self.place_btn.Enable(not running and self.backend.find_cli() is not None)
        self.place_route_btn.Enable(
            not running and self.backend.find_cli() is not None)
        self.action_btn.Enable(not running)
        self.apply_btn.Enable(not running and self._pending_result is not None
                              and self._pending_result.get("board") is not None)
        self.cancel_btn.SetLabel("Cancel" if running else "Close")

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
        and truncates at ~120 chars, which usually loses the `--stage P4`
        that stage derivation keys on - the raw event still carries it.
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
            self.preview.set_outputs(movie=outputs.get("movie"), report=report,
                                     workdir=workdir)
            frames = []
            preview_dir = os.path.join(workdir, "_gui_preview") if workdir else None
            if preview_dir and os.path.isdir(preview_dir):
                frames = sorted(
                    (os.path.join(preview_dir, n)
                     for n in os.listdir(preview_dir) if n.endswith(".png")),
                    key=_safe_mtime)
            self.preview.set_frames(frames)
            board = outputs.get("board")
            if board and self._monitor is not None:
                # One final frame of the result board itself (queued behind
                # any render still in flight).
                self._monitor._maybe_render(board, final=True)
            for key in ("board", "movie", "report"):
                if outputs.get(key):
                    self._append_transcript(f"{key}: {outputs[key]}\n")
                    self.append_log(f"Placement {key}: {outputs[key]}\n")
        self._monitor = None
        self.set_status(status)
        self._set_running_ui(False)

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
            from kicad_ipc_adapter import get_board
            board = get_board()
        except Exception as e:
            wx.MessageBox(f"Not connected to KiCad (IPC): {e}", "Placement",
                          wx.OK | wx.ICON_ERROR)
            return
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

        moves = self._pose_moves(board, result_pcb)
        with_copper = self._mode == "place_route"
        lines = [f"{len(moves)} footprint(s) will move."]
        if moves:
            refs = ", ".join(r for r, _fp, _p in moves[:30])
            if len(moves) > 30:
                refs += ", ..."
            lines.append(refs)
        if with_copper:
            lines.append("Place + Route additionally REPLACES ALL tracks and "
                         "vias on the live board with the run's routing. "
                         "Arc tracks come back as short straight segments "
                         "(the parser linearizes arcs) and blind/buried vias "
                         "as through vias - if the input board carried "
                         "either, open the result board file instead. "
                         "Per-via tenting/plugging specs (#489 s8) are NOT "
                         "carried: kipy has no via-protection writer, so "
                         "re-placed vias take the board's default.")
        lines.append("The live board may have changed since the run started - "
                     "this diff is against its state right now. Continue?")
        if wx.MessageBox("\n\n".join(lines), "Apply Placement Result",
                         wx.YES_NO | wx.ICON_WARNING) != wx.YES:
            return

        tracks_added, vias_added = self._apply_ipc(
            board, result_pcb, moves, with_copper)

        # BuildConnectivity / refill / Refresh are NOT needed under IPC:
        # KiCad rebuilds and fills server-side on each commit (same doctrine
        # as routing_dialog's apply and planes_gui's).
        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()
        if self.on_complete:
            self.on_complete()
        parsed_zones = len(getattr(result_pcb, "zones", []) or [])
        live_zones = len(list(board.get_zones()))
        if parsed_zones > live_zones:
            self.append_log(
                f"WARNING: result board has {parsed_zones} zones, live board "
                f"{live_zones} - new zones are NOT imported by Apply; open "
                "the result board file to get them.\n")
        msg = f"Applied: {len(moves)} footprint pose(s)"
        if with_copper:
            msg += f", {tracks_added} track(s), {vias_added} via(s) (replaced)"
        msg += ". Use Edit -> Undo in KiCad to revert."
        self.set_status(msg)
        self.append_log("Placement apply: " + msg + "\n")

    @staticmethod
    def _fp_is_back(fp) -> bool:
        """True when the live footprint sits on the back side.

        kipy models the side as the footprint's `layer` (BL_F_Cu / BL_B_Cu),
        the IPC analogue of SWIG's IsFlipped().
        """
        from kicad_ipc_adapter import _ensure_kipy
        try:
            return fp.layer == _ensure_kipy().BoardLayer.BL_B_Cu
        except Exception:
            return False

    def _pose_moves(self, board, result_pcb):
        """[(ref, live_fp, parsed_fp)] for every footprint whose pose differs."""
        from kicad_parser import _fp_reference
        from kicad_ipc_adapter import _vec_xy_mm

        # One enumeration, not a lookup per part: kipy has no
        # FindFootprintByReference and each get_footprints() is a round trip.
        live = {}
        for fp in board.get_footprints():
            ref = _fp_reference(fp)
            if ref:
                live.setdefault(ref, fp)

        moves = []
        for ref, parsed in sorted(result_pcb.footprints.items()):
            fp = live.get(ref)
            if fp is None:
                continue  # part exists only in the result; never invent parts
            live_x, live_y = _vec_xy_mm(fp.position)
            live_rot = float(getattr(fp.orientation, "degrees", 0.0)) % 360
            live_back = self._fp_is_back(fp)
            parsed_back = str(parsed.layer or "F.Cu").startswith("B")
            if (abs(live_x - parsed.x) > 1e-4 or abs(live_y - parsed.y) > 1e-4
                    or abs((live_rot - (parsed.rotation or 0) % 360 + 180) % 360 - 180) > 1e-3
                    or live_back != parsed_back):
                moves.append((ref, fp, parsed))
        return moves

    def _apply_ipc(self, board, result_pcb, moves, with_copper):
        """Apply poses (and optionally ALL copper) in ONE IPC commit.

        Wholesale copper replacement is correct here: the place+route loop's
        contract is that placement invalidates all prior copper. Poses and
        copper ride the SAME commit so the board is never left half-applied
        and the whole apply is one undo step (parity with
        kicad_ipc_adapter.apply_footprint_moves).
        """
        from kicad_ipc_adapter import (begin_commit, make_track, make_via,
                                       vec_mm, _ensure_kipy)
        try:
            from kipy.geometry import Angle
        except Exception:
            Angle = None

        # Result-board net ids are PCBData's synthetic ids; kipy resolves nets
        # by NAME (net_map), so translate here rather than passing ids through.
        def name_for(net_id):
            net = result_pcb.nets.get(net_id)
            return net.name if net is not None else None

        tracks_added = vias_added = 0
        with begin_commit(board, "KiCadRoutingTools: apply placement") as commit:
            BoardLayer = _ensure_kipy().BoardLayer
            for _ref, fp, parsed in moves:
                parsed_back = str(parsed.layer or "F.Cu").startswith("B")
                if self._fp_is_back(fp) != parsed_back:
                    # kipy has no Flip(); the side IS the footprint's layer.
                    fp.layer = (BoardLayer.BL_B_Cu if parsed_back
                                else BoardLayer.BL_F_Cu)
                fp.position = vec_mm(parsed.x, parsed.y)
                if Angle is not None:
                    try:
                        fp.orientation = Angle.from_degrees(
                            float(parsed.rotation or 0))
                    except Exception:
                        pass  # leave orientation if the API shape differs
                commit.update(fp)

            if not with_copper:
                return 0, 0

            for item in list(board.get_tracks()) + list(board.get_vias()):
                commit.remove(item)
            for seg in result_pcb.segments:
                t = make_track(commit.net_map, seg.start_x, seg.start_y,
                               seg.end_x, seg.end_y, seg.width, seg.layer,
                               net_name=name_for(seg.net_id))
                # #521: user-pinned copper must come back pinned - the whole
                # protection chain (never-rippable, no override) rests on it.
                try:
                    t.locked = bool(getattr(seg, "locked", False))
                except Exception:
                    pass
                commit.add(t)
                tracks_added += 1
            for via in result_pcb.vias:
                layers = getattr(via, "layers", None) or ["F.Cu", "B.Cu"]
                v = make_via(commit.net_map, via.x, via.y, via.size, via.drill,
                             top_layer=layers[0], bottom_layer=layers[-1],
                             net_name=name_for(via.net_id))
                try:
                    v.locked = bool(getattr(via, "locked", False))  # #521
                except Exception:
                    pass
                # NOTE: the via's own tenting/plugging spec (#489 s8) is NOT
                # carried - kipy exposes no via-protection writer, the same
                # IPC gap planes_gui records for teardrops. Disclosed in the
                # Apply confirmation rather than dropped silently.
                commit.add(v)
                vias_added += 1
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
                "Label engine not available yet (issue #481).")
            return
        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()
        self._cancel_requested = False
        self.action_btn.Disable()
        self.cancel_btn.SetLabel("Cancel")
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
            from kicad_ipc_adapter import get_board
            from kicad_parser import build_pcb_data_from_board
            board = get_board()
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
        self.cancel_btn.SetLabel("Close")
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
        absolute; the angle is the node's own (at) rotation, which KiCad
        stores verbatim on the text item) - so CLI and GUI outputs match by
        construction, whatever the angle-storage semantics turn out to be.
        """
        try:
            from kicad_ipc_adapter import (get_board, begin_commit, vec_mm,
                                           _mm_to_nm)
            from kicad_parser import _fp_reference
            from kipy.geometry import Vector2
            from kipy.proto.common import types as ktypes
        except Exception:
            return 0
        board = get_board()
        if board is None:
            return 0
        live = {}
        for fp in board.get_footprints():
            _r = _fp_reference(fp)
            if _r:
                live.setdefault(_r, fp)
        applied = 0
        with begin_commit(board, "KiCadRoutingTools: beautify labels") as commit:
            for r in result.get("results", []):
                action = getattr(r, "action", None) or (
                    r.get("action") if isinstance(r, dict) else None)
                if action not in ("placed", "shrunk", "rotated", "shrunk_rotated"):
                    continue
                get = (lambda k, _r=r: getattr(_r, k, None)) \
                    if not isinstance(r, dict) else r.get
                fp = live.get(get("reference"))
                if fp is None:
                    continue
                text = fp.reference_field.text
                attrs = text.attributes          # write-through proto ref
                size_nm = _mm_to_nm(get("size"))
                attrs.size = Vector2.from_xy(size_nm, size_nm)
                attrs.stroke_width = _mm_to_nm(get("thickness"))
                # file_rotation is the stored (at ...) angle, ABSOLUTE rather
                # than footprint-relative (the KiCad 10 probe behind #481).
                attrs.angle = float(get("file_rotation") or 0)
                attrs.horizontal_alignment = \
                    ktypes.HorizontalAlignment.HA_CENTER
                attrs.vertical_alignment = ktypes.VerticalAlignment.VA_CENTER
                text.position = vec_mm(get("world_x"), get("world_y"))
                commit.update(fp)
                applied += 1
        # No Refresh() under IPC: KiCad redraws server-side on commit.
        return applied

    # -------------------------------------------------------- cancel/close

    def _on_cancel_or_close(self, event):
        if self._runner is not None and self._runner.is_running():
            self._runner.cancel()
            self.set_status(
                "Cancelling — partial artifacts are kept in the workdir...")
            return
        if self._routing_thread is not None and self._routing_thread.is_alive():
            self._cancel_requested = True
            self.set_status("Cancelling...")
            return
        # Nothing running: close the dialog like the other tabs.
        top = wx.GetTopLevelParent(self)
        if isinstance(top, wx.Dialog):
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
        self.preview.shutdown()

    # ------------------------------------------------- session restore

    def restore_last_workdir(self, workdir):
        """Re-surface a previous session's artifacts (settings restore)."""
        if not workdir or not os.path.isdir(workdir):
            return
        self.last_workdir = workdir
        outputs = scan_workdir_outputs(workdir)
        self.preview.set_outputs(movie=outputs.get("movie"),
                                 report=outputs.get("report"), workdir=workdir)
        preview_dir = os.path.join(workdir, "_gui_preview")
        if os.path.isdir(preview_dir):
            self.preview.set_frames(sorted(
                (os.path.join(preview_dir, n) for n in os.listdir(preview_dir)
                 if n.endswith(".png")),
                key=_safe_mtime))
        if outputs.get("board"):
            self._pending_result = outputs
            self._mode = ("place_route"
                          if os.path.basename(workdir).endswith("place_route")
                          else "place")
            self.apply_btn.Enable(True)
            self.set_status(
                f"Previous run's artifacts loaded from {workdir}")
