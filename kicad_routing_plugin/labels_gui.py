"""
KiCad Routing Tools - Labels sub-tab (Auto tab notebook).

The non-AI "Beautify Labels" action (#481): uniform, non-overlapping
reference-designator silkscreen on the open board. It moves and restyles only
the labels, never the parts, through the same engine the CLI's
`py_placer/beautify_labels.py` runs (`placement.labels.beautify_labels`), and
writes the engine's FILE-frame values onto the live board's Reference fields,
so the two fronts agree by construction.
"""

import os
import sys
import threading
import time

import wx

# Same path setup as placement_gui: the engine lives in py_placer/ (with the
# py_router modules beside it).
PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)
_ENGINE_DIR = os.path.join(ROOT_DIR, 'py_router')
if os.path.isdir(_ENGINE_DIR) and _ENGINE_DIR not in sys.path:
    sys.path.insert(0, _ENGINE_DIR)
for _sib in ('py_placer', 'py_tools'):
    _d = os.path.join(ROOT_DIR, _sib)
    if os.path.isdir(_d) and _d not in sys.path:
        sys.path.append(_d)

#: The LabelConfig values a fresh dialog shows (the CLI's own defaults).
LABEL_DEFAULTS = {'size': 1.0, 'thickness': 0.15, 'min_size': 0.7,
                  'orientation': 'horizontal', 'max_distance': 2.0,
                  'include_hidden': False, 'skip_locked': False}


class LabelsOptionsPanel(wx.Panel):
    """Options for Beautify Labels: label/control pairs with the two
    checkboxes beside them.

    Control attribute names deliberately equal the plan-step param names
    (label_size, label_orientation, ...) - the AI plan executor applies
    snake_case params by control attribute name.
    """

    def __init__(self, parent):
        super().__init__(parent)
        grid = wx.FlexGridSizer(cols=2, vgap=6, hgap=8)

        def row(label, ctrl, tooltip):
            grid.Add(wx.StaticText(self, label=label), 0,
                     wx.ALIGN_CENTER_VERTICAL | wx.LEFT, 4)
            ctrl.SetToolTip(tooltip)
            grid.Add(ctrl, 0, wx.EXPAND)
            return ctrl

        self.label_size = row(
            "Text size (mm):",
            wx.SpinCtrlDouble(self, min=0.3, max=3.0, initial=1.0, inc=0.05, size=(90, -1)),
            "Uniform reference-label font height (and width).")
        self.label_thickness = row(
            "Thickness (mm):",
            wx.SpinCtrlDouble(self, min=0.05, max=0.5, initial=0.15, inc=0.01, size=(90, -1)),
            "Uniform stroke thickness.")
        self.label_min_size = row(
            "Min size (mm):",
            wx.SpinCtrlDouble(self, min=0.3, max=3.0, initial=0.7, inc=0.05, size=(90, -1)),
            "Shrink floor when the uniform size does not fit; equal to the "
            "text size disables shrinking.")
        self.label_orientation = row(
            "Orientation:", wx.Choice(self, choices=["horizontal", "vertical"], size=(130, -1)),
            "Preferred label direction; the other direction is a fallback "
            "when the preferred one does not fit.")
        self.label_orientation.SetSelection(0)
        self.label_max_distance = row(
            "Max distance (mm):",
            wx.SpinCtrlDouble(self, min=0.5, max=10.0, initial=2.0, inc=0.25, size=(90, -1)),
            "How far beyond the part's courtyard a label may be pushed.")
        self.label_include_hidden = wx.CheckBox(self, label="Include hidden labels")
        self.label_include_hidden.SetToolTip(
            "Also place labels that are currently hidden (they stay hidden; "
            "only their pose and size are tidied).")
        self.label_skip_locked = wx.CheckBox(self, label="Skip locked parts")
        self.label_skip_locked.SetToolTip(
            "Leave the labels of KiCad-locked footprints untouched.")
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(grid, 0, wx.ALL, 4)
        sizer.Add(self.label_include_hidden, 0, wx.LEFT | wx.TOP, 8)
        sizer.Add(self.label_skip_locked, 0, wx.LEFT | wx.TOP, 8)
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
        self.set_config(LABEL_DEFAULTS)


class LabelsTab(wx.Panel):
    """The Labels sub-tab: Beautify Labels' options, its button, Cancel and
    Close, a gauge and a status line."""

    def __init__(self, parent, pcb_data, board_filename, on_complete=None,
                 append_log=None, sync_pcb_data_callback=None):
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self.on_complete = on_complete
        self.append_log = append_log or (lambda text: None)
        self.sync_pcb_data_callback = sync_pcb_data_callback
        # Plan-executor contract (ai_plan.PlanExecutor): a worker thread the
        # executor joins, and a cancel flag the engine polls.
        self._cancel_requested = False
        self._routing_thread = None
        self._create_ui()
        self.Bind(wx.EVT_WINDOW_DESTROY, self._on_destroy)

    def _create_ui(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        top = wx.BoxSizer(wx.HORIZONTAL)

        box = wx.StaticBox(self, label="Beautify Labels (no AI)")
        box_sizer = wx.StaticBoxSizer(box, wx.VERTICAL)
        box_sizer.Add(wx.StaticText(
            self, label="Uniform, non-overlapping reference-designator "
                        "silkscreen. Moves and restyles only the labels, "
                        "never the parts."), 0, wx.ALL, 5)
        self.labels_options = LabelsOptionsPanel(self)
        box_sizer.Add(self.labels_options, 0, wx.ALL, 3)
        top.Add(box_sizer, 1, wx.EXPAND | wx.RIGHT, 8)

        ctrl_box = wx.StaticBox(self, label="Run")
        ctrl_sizer = wx.StaticBoxSizer(ctrl_box, wx.VERTICAL)
        self.action_btn = wx.Button(self, label="Beautify Labels")
        self.action_btn.SetToolTip(
            "Uniform, non-overlapping reference-designator silkscreen. "
            "Moves and restyles only the labels, never the parts.")
        ctrl_sizer.Add(self.action_btn, 0, wx.EXPAND | wx.ALL, 5)
        self.stop_btn = wx.Button(self, label="Cancel")
        self.stop_btn.SetToolTip("Cancel the running label job.")
        self.stop_btn.Disable()
        ctrl_sizer.Add(self.stop_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        ctrl_sizer.AddStretchSpacer(1)
        ctrl_sizer.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.ALL, 5)
        self.close_btn = wx.Button(self, label="Close")
        self.close_btn.SetToolTip("Close dialog")
        ctrl_sizer.Add(self.close_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        top.Add(ctrl_sizer, 0, wx.EXPAND)
        sizer.Add(top, 0, wx.EXPAND | wx.ALL, 8)

        self.progress_bar = wx.Gauge(self, range=100)
        sizer.Add(self.progress_bar, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)
        status_row = wx.BoxSizer(wx.HORIZONTAL)
        status_row.Add(wx.StaticText(self, label="Status:"), 0,
                       wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.status_text = wx.TextCtrl(self, value="Ready", style=wx.TE_READONLY)
        status_row.Add(self.status_text, 1, wx.EXPAND)
        sizer.Add(status_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)
        sizer.AddStretchSpacer(1)
        self.SetSizer(sizer)

        self.action_btn.Bind(wx.EVT_BUTTON, self._on_action)
        self.stop_btn.Bind(wx.EVT_BUTTON, self._on_stop)
        self.close_btn.Bind(wx.EVT_BUTTON, self._on_close)

    def set_status(self, text):
        self.status_text.SetValue(text)
        self.status_text.SetToolTip(text)

    def _busy(self):
        return self._routing_thread is not None and self._routing_thread.is_alive()

    # ------------------------------------------------------ beautify labels

    def _on_action(self, event):
        """Beautify Labels: canonical worker-thread pattern (plan-drivable)."""
        if self._busy():
            wx.MessageBox("A label job is already running.",
                          "Labels", wx.OK | wx.ICON_WARNING)
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
        """Programmatic entry. NOT synchronous: it starts the worker thread and
        returns - a plan action must use a busy predicate that polls
        action_btn, never `lambda: False`."""
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
        return apply_label_results(board, result, pcbnew)

    # -------------------------------------------------------- cancel/close

    def _on_stop(self, event):
        if self._busy():
            self._cancel_requested = True
            self.set_status("Cancelling...")
        self.stop_btn.Disable()

    def _on_close(self, event):
        if self._busy() and wx.MessageBox(
                "A label job is still running. Closing the dialog cancels it."
                "\n\nClose anyway?", "Labels",
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
        self._cancel_requested = True


def apply_label_results(board, result, pcbnew):
    """Write beautify_labels' RESULT onto BOARD's Reference fields; returns
    how many labels were applied. Shared by the tab and its parity gate, so
    the gate grades the code the button runs."""
    from kicad_parser import mm_to_iu
    from .gui_utils import live_footprints_by_key
    live = live_footprints_by_key(board)
    applied = 0
    for r in result.get("results", []):
        action = getattr(r, "action", None) or (
            r.get("action") if isinstance(r, dict) else None)
        if action not in ("placed", "shrunk", "rotated", "shrunk_rotated"):
            continue
        get = (lambda k, _r=r: getattr(_r, k, None)) \
            if not isinstance(r, dict) else r.get
        # #726: a label result names a PCBData key, which for a duplicated
        # reference is `TP4~2`.
        key = get("reference")
        fp = live.get(key) or board.FindFootprintByReference(key)
        if fp is None:
            continue
        ref = fp.Reference()
        size = mm_to_iu(get("size"))
        ref.SetTextSize(pcbnew.VECTOR2I(size, size))
        ref.SetTextThickness(mm_to_iu(get("thickness")))
        ref.SetPosition(pcbnew.VECTOR2I(mm_to_iu(get("world_x")),
                                        mm_to_iu(get("world_y"))))
        # file_rotation is the stored (at ...) angle, which the KiCad 10 probe
        # showed pcbnew's GetTextAngle/SetTextAngle round-trips verbatim
        # (ABSOLUTE, not footprint-relative).
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
