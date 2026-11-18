"""
KiCad Routing Tools - the Bus tab: the bus step (awx/route_bus.py).

The bus step plans and lays the buses between a ball array and the parts it
talks to (another ball array, or row parts no track passes between the pins
of) -- every lane's order, layer and layer changes, and the arrays' fanout --
in one call that takes minutes to over an hour. The tab runs it the way the
Autorouter tab runs autoroute.py: as a CHILD PROCESS on a snapshot of the open
board, in a work folder of its own, because

  * the step's solver, OR-Tools, is never imported in KiCad's process
    (deps_check.CLI_ONLY_PACKAGES: on Windows its DLLs clash with KiCad's);
  * the step runs each stage in a process of its own, and a solver's crash is
    a signal that must end a child, not KiCad;
  * the command the tab runs is the CLI's own (bus_progress.build_argv), so a
    GUI run and a terminal run of the same arguments are the same run -- and,
    with REDO_MANIFEST in KiCad's environment, the same redo-manifest line.

What it shows comes from the step's events file (awx/bus_events.py,
<OUT stem>.bus/events.jsonl, written by every process of the step): a row
for each group, each bus and each of their stages, with its state, its time
and its result, read into rows by bus_progress.RunModel. The step's own
console goes to the Log tab.

Buses: on first showing the tab runs `route_bus.py --find` at the current
sizes and lists what it finds, each checked, and what it does not take, with
why. Unchecking, reordering, adding or editing a bus makes the run name each
checked one with --bus, in order (the routing order); otherwise the run
passes none and the step finds the same buses itself.

Apply Result to Board makes the open board match the step's output
(result_apply, as the Autorouter tab): its tracks and vias (the lanes are
KiCad-locked), its poses (a passive the step moved off an escape), its zones,
the design rules it was graded against and its records -- the bus nets'
protection (#521) among them.
"""

import os
import shlex
import subprocess
import sys
import tempfile
import time

import wx

from . import bus_progress as bp
from .autoroute_gui import _fmt_secs, _prune_workdirs, _tail_file, python_for_subprocess
from .child_run import ChildProcess, snapshot_board

#: Each run's work folder is a temp folder with this prefix; a new run keeps
#: the newest KEEP_RUNS - 1 earlier ones.
WORKDIR_PREFIX = 'krt_bus_'
KEEP_RUNS = 5

_STATE_COLOURS = {
    'running': wx.Colour(0, 90, 200),
    'done': wx.Colour(0, 120, 0),
    'failed': wx.Colour(190, 0, 0),
    'not routed': wx.Colour(160, 90, 0),
    'ended': wx.Colour(120, 120, 120),
    'pending': wx.Colour(80, 80, 80),
}

_COLUMNS = (("Group / bus / stage", 230), ("State", 90), ("Time", 56), ("Result", 420))

#: The environment a python of the user's own choosing must not inherit from
#: KiCad's: these point an interpreter at KiCad's own library.
_FOREIGN_PYTHON_VARS = ('PYTHONHOME', 'PYTHONPATH', 'PYTHONEXECUTABLE', '__PYVENV_LAUNCHER__',
                        'PYTHONSTARTUP')


def _defaults():
    import routing_defaults as d
    return d


class BusOptionsPanel(wx.Panel):
    """route_bus.py's own options, each control named after its plan param,
    and the python the step runs under (a setting of this machine, not of a
    run)."""

    def __init__(self, parent):
        super().__init__(parent)
        d = _defaults()
        grid = wx.FlexGridSizer(cols=2, vgap=6, hgap=8)
        grid.Add(wx.StaticText(self, label="Rounds:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.rounds = wx.SpinCtrl(self, min=1, max=50, initial=d.BUS_ROUNDS, size=(80, -1))
        self.rounds.SetToolTip(
            "--rounds: the group's rounds at most. A round that leaves the board "
            "short of clean (a net open, a violation, a ball walled in) sends what "
            "its buses found back to the group's ends model, and the buses are "
            "routed again; the best round's board is kept.")
        grid.Add(self.rounds, 0)
        grid.Add(wx.StaticText(self, label="Feedback rounds:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.feedback_rounds = wx.SpinCtrl(self, min=0, max=50, initial=d.BUS_FEEDBACK_ROUNDS,
                                           size=(80, -1))
        self.feedback_rounds.SetToolTip(
            "--feedback-rounds: a bus's own whole route, after a draw that leaves "
            "nets open, runs up to this many rounds under its seed, the fanout fed "
            "what the round before found (0: every round a draw).")
        grid.Add(self.feedback_rounds, 0)
        self.fanout_track_width_check = wx.CheckBox(self, label="Fanout track width:")
        self.fanout_track_width_check.SetToolTip(
            "--fanout-track-width: the escapes' track under the arrays. Unticked: "
            "the lanes' track (the shared Track Width), as the CLI does without "
            "the flag.")
        grid.Add(self.fanout_track_width_check, 0, wx.ALIGN_CENTER_VERTICAL)
        self.fanout_track_width = wx.SpinCtrlDouble(self, min=0.03, max=2.0, initial=0.1,
                                                    inc=0.005, size=(80, -1))
        self.fanout_track_width.SetDigits(4)
        self.fanout_track_width.Enable(False)
        grid.Add(self.fanout_track_width, 0)
        self.fanout_track_width_check.Bind(
            wx.EVT_CHECKBOX, lambda e: self.fanout_track_width.Enable(
                self.fanout_track_width_check.GetValue()))
        grid.Add(wx.StaticText(self, label="Ladder nets (K):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.k = wx.SpinCtrl(self, min=0, max=100000, initial=0, size=(80, -1))
        self.k.SetToolTip("--k: route only each bus's first K nets of its ladder "
                          "(0: the whole ladder, the CLI's default). A debugging aid.")
        grid.Add(self.k, 0)
        self.inproc = wx.CheckBox(self, label="Stages in one process")
        self.inproc.SetToolTip("--inproc: each bus's stages run in its own process rather "
                               "than each stage in a process of its own.")
        grid.Add(self.inproc, 0, wx.ALIGN_CENTER_VERTICAL)
        grid.AddSpacer(0)
        self.pack = wx.CheckBox(self, label="Pack the laid lanes")
        self.pack.SetToolTip("--pack: after a round's lay each single lane is pulled taut "
                             "against its neighbour and laid again as packed copper; the "
                             "packed board is kept only where it has no more nets open and "
                             "no more violations than the lay. Unticked: the lay stands, as "
                             "the CLI does without the flag.")
        grid.Add(self.pack, 0, wx.ALIGN_CENTER_VERTICAL)
        grid.AddSpacer(0)
        grid.Add(wx.StaticText(self, label="Round edits:"), 0, wx.ALIGN_CENTER_VERTICAL)
        edits = wx.BoxSizer(wx.HORIZONTAL)
        self.round_edits = wx.TextCtrl(self, size=(150, -1))
        self.round_edits.SetToolTip(
            "--round-edits EDITS.json: what to ask of each bus's rounds (ends banned, "
            "pinned or avoided, cuts, areas), per bus and round. Empty: none.")
        edits.Add(self.round_edits, 1, wx.EXPAND)
        self.round_edits_btn = wx.Button(self, label="...", size=(30, -1))
        self.round_edits_btn.Bind(wx.EVT_BUTTON, self._browse_edits)
        edits.Add(self.round_edits_btn, 0, wx.LEFT, 2)
        grid.Add(edits, 0, wx.EXPAND)
        grid.Add(wx.StaticText(self, label="Python:"), 0, wx.ALIGN_CENTER_VERTICAL)
        py = wx.BoxSizer(wx.HORIZONTAL)
        self.bus_python = wx.TextCtrl(self, size=(150, -1))
        self.bus_python.SetHint("KiCad's python")
        self.bus_python.SetToolTip(
            "The Python the bus step runs under (it needs OR-Tools). Empty: the "
            "Python KiCad runs plugins with. The step never runs in KiCad's own "
            "process.")
        py.Add(self.bus_python, 1, wx.EXPAND)
        self.bus_python_btn = wx.Button(self, label="...", size=(30, -1))
        self.bus_python_btn.Bind(wx.EVT_BUTTON, self._browse_python)
        py.Add(self.bus_python_btn, 0, wx.LEFT, 2)
        grid.Add(py, 0, wx.EXPAND)
        grid.AddGrowableCol(1, 1)
        self.SetSizer(grid)

    def _browse_edits(self, event):
        dlg = wx.FileDialog(self, "Round edits file", wildcard="JSON (*.json)|*.json|All files|*",
                            style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST)
        if dlg.ShowModal() == wx.ID_OK:
            self.round_edits.SetValue(dlg.GetPath())
        dlg.Destroy()

    def _browse_python(self, event):
        dlg = wx.FileDialog(self, "Python for the bus step", style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST)
        if dlg.ShowModal() == wx.ID_OK:
            self.bus_python.SetValue(dlg.GetPath())
        dlg.Destroy()

    def options(self):
        """The options as bus_progress.build_argv takes them."""
        return {'rounds': self.rounds.GetValue(), 'feedback_rounds': self.feedback_rounds.GetValue(),
                'k': self.k.GetValue(), 'inproc': self.inproc.GetValue(),
                'pack': self.pack.GetValue(),
                'round_edits': self.round_edits.GetValue().strip()}

    def fanout_width(self):
        return (self.fanout_track_width.GetValue() if self.fanout_track_width_check.GetValue()
                else None)

    def get_config(self):
        return dict(self.options(), fanout_track_width_check=self.fanout_track_width_check.GetValue(),
                    fanout_track_width=self.fanout_track_width.GetValue(),
                    bus_python=self.bus_python.GetValue().strip())

    def set_config(self, cfg):
        try:
            for name in ('rounds', 'feedback_rounds', 'k'):
                if name in cfg:
                    getattr(self, name).SetValue(int(cfg[name]))
            if 'inproc' in cfg:
                self.inproc.SetValue(bool(cfg['inproc']))
            if 'pack' in cfg:
                self.pack.SetValue(bool(cfg['pack']))
            if 'round_edits' in cfg:
                self.round_edits.SetValue(str(cfg['round_edits'] or ''))
            if 'fanout_track_width' in cfg:
                self.fanout_track_width.SetValue(float(cfg['fanout_track_width']))
            if 'fanout_track_width_check' in cfg:
                self.fanout_track_width_check.SetValue(bool(cfg['fanout_track_width_check']))
                self.fanout_track_width.Enable(bool(cfg['fanout_track_width_check']))
            if 'bus_python' in cfg:
                self.bus_python.SetValue(str(cfg['bus_python'] or ''))
        except (TypeError, ValueError):
            pass    # a malformed saved dict must not break dialog restore

    def reset_to_defaults(self):
        """Every run option to the CLI's default; the python is kept (it is
        this machine's, not a step's)."""
        d = _defaults()
        self.set_config({'rounds': d.BUS_ROUNDS, 'feedback_rounds': d.BUS_FEEDBACK_ROUNDS, 'k': 0,
                         'inproc': False, 'pack': False, 'round_edits': '',
                         'fanout_track_width_check': False})


class BusDialog(wx.Dialog):
    """Add or edit a bus: a source (the board's ball arrays) and a SET of
    destination parts, in the order checked -- the first is the trunk's, a
    branch leaves it for each other (U4+U3)."""

    def __init__(self, parent, sources, partners, spec=None):
        super().__init__(parent, title="Bus", style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self.partners = partners            # src -> [(ref, shared nets)], most first
        self.chosen = []
        s = wx.BoxSizer(wx.VERTICAL)
        s.Add(wx.StaticText(self, label="Source (a ball array):"), 0, wx.ALL, 5)
        self.src = wx.Choice(self, choices=list(sources))
        s.Add(self.src, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 5)
        s.Add(wx.StaticText(self, label="Destinations, in the order checked (the first is the "
                                        "trunk's):"), 0, wx.ALL, 5)
        self.dest = wx.CheckListBox(self, size=(260, 200))
        s.Add(self.dest, 1, wx.EXPAND | wx.LEFT | wx.RIGHT, 5)
        self.preview = wx.StaticText(self, label="")
        s.Add(self.preview, 0, wx.ALL, 5)
        s.Add(self.CreateButtonSizer(wx.OK | wx.CANCEL), 0, wx.EXPAND | wx.ALL, 5)
        self.SetSizerAndFit(s)
        self.src.Bind(wx.EVT_CHOICE, lambda e: self._fill())
        self.dest.Bind(wx.EVT_CHECKLISTBOX, self._on_check)
        if spec:
            src, dest = bp.parse_bus_spec(spec)
            if src not in sources:
                self.src.Append(src)
            self.src.SetStringSelection(src)
            self.chosen = dest.split('+')
        elif sources:
            self.src.SetSelection(0)
        self._fill()

    def _fill(self):
        src = self.src.GetStringSelection()
        rows = self.partners.get(src, [])
        refs = [r for r, _n in rows]
        for r in self.chosen:
            if r not in refs:
                rows = rows + [(r, 0)]
        self._refs = [r for r, _n in rows]
        self.dest.Set([f"{r} ({n} shared net{'s' if n != 1 else ''})" for r, n in rows])
        self.chosen = [r for r in self.chosen if r in self._refs]
        for i, r in enumerate(self._refs):
            self.dest.Check(i, r in self.chosen)
        self._preview()

    def _on_check(self, event):
        r = self._refs[event.GetInt()]
        if self.dest.IsChecked(event.GetInt()):
            if r not in self.chosen:
                self.chosen.append(r)
        elif r in self.chosen:
            self.chosen.remove(r)
        self._preview()

    def _preview(self):
        spec = self.spec()
        self.preview.SetLabel(f"--bus {spec}" if spec else "(check one destination or more)")

    def spec(self):
        src = self.src.GetStringSelection()
        return f"{src}:{'+'.join(self.chosen)}" if src and self.chosen else None


class BusTab(wx.Panel):
    """The Bus tab (see module docstring)."""

    POLL_MS = 500

    def __init__(self, parent, pcb_data, board_filename, get_sizes=None, on_complete=None,
                 append_log=None, sync_pcb_data_callback=None, get_fix_drc=None):
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.board_filename = board_filename
        # get_sizes(): {route_bus size name: mm} for the sizes the shared
        # panel's override boxes give; one not ticked is omitted, which the
        # step resolves as route.py does (the board's Default class).
        self.get_sizes = get_sizes or (lambda: {})
        self.get_fix_drc = get_fix_drc or (lambda: True)
        self.on_complete = on_complete
        self.append_log = append_log or (lambda text: None)
        self.sync_pcb_data_callback = sync_pcb_data_callback
        self.buses = bp.BusList()
        self.model = bp.RunModel()
        self._child = None
        self._job = None             # 'find' | 'run'
        self._probed = {}            # python -> (ok, text)
        self.suggested_python = None
        self._offset = 0
        self._shown_key = None
        self._rows = []
        self._plan_run = False
        self._apply_pending = False
        self._cancel_requested = False
        self.find_dir = None
        self.workdir = None
        self.output = None
        self.events_path = None
        self.argv = None
        self.returncode = None
        self.t_start = None
        self.last_apply_stats = None
        self._create_ui()
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_tick, self._timer)
        self.Bind(wx.EVT_WINDOW_DESTROY, self._on_destroy)

    # ----------------------------------------------------------------- UI

    def _create_ui(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        top = wx.BoxSizer(wx.HORIZONTAL)
        left = wx.BoxSizer(wx.VERTICAL)

        bus_box = wx.StaticBoxSizer(wx.StaticBox(self, label="Buses (routing order)"), wx.VERTICAL)
        self.bus_list = wx.CheckListBox(self, size=(-1, 110))
        self.bus_list.SetToolTip(
            "The buses the step routes, in order. Found by route_bus.py --find at "
            "the shared sizes; uncheck, reorder, add or edit to name them yourself "
            "(the run then passes --bus for each checked row, in this order).")
        bus_box.Add(self.bus_list, 1, wx.EXPAND | wx.ALL, 3)
        btns = wx.BoxSizer(wx.HORIZONTAL)
        self.find_btn = wx.Button(self, label="Find Buses")
        self.find_btn.SetToolTip("Run route_bus.py --find on a snapshot of the board at the "
                                 "shared sizes and list what it finds")
        self.add_btn = wx.Button(self, label="Add...")
        self.edit_btn = wx.Button(self, label="Edit...")
        self.remove_btn = wx.Button(self, label="Remove")
        self.up_btn = wx.Button(self, label="Up")
        self.down_btn = wx.Button(self, label="Down")
        self.use_found_btn = wx.Button(self, label="As Found")
        self.use_found_btn.SetToolTip("Back to the buses --find found, all checked, in its order")
        for b in (self.find_btn, self.add_btn, self.edit_btn, self.remove_btn, self.up_btn,
                  self.down_btn, self.use_found_btn):
            btns.Add(b, 0, wx.RIGHT, 3)
        bus_box.Add(btns, 0, wx.ALL, 3)
        self.bus_mode_text = wx.StaticText(self, label="")
        bus_box.Add(self.bus_mode_text, 0, wx.EXPAND | wx.ALL, 3)
        self.not_taken_text = wx.TextCtrl(self, style=wx.TE_MULTILINE | wx.TE_READONLY, size=(-1, 52))
        self.not_taken_text.SetToolTip("The buses --find does not take, each with why "
                                       "(left to the router)")
        bus_box.Add(self.not_taken_text, 0, wx.EXPAND | wx.ALL, 3)
        left.Add(bus_box, 0, wx.EXPAND)

        prog_box = wx.StaticBoxSizer(wx.StaticBox(self, label="Progress"), wx.VERTICAL)
        self.progress_list = wx.ListCtrl(self, style=wx.LC_REPORT | wx.LC_SINGLE_SEL)
        for i, (name, width) in enumerate(_COLUMNS):
            self.progress_list.InsertColumn(i, name, width=width)
        self.progress_list.SetToolTip(
            "One row per group (a source and its buses), per bus and per stage, from "
            "the step's events file. Select a row for its details and log.")
        self.progress_list.Bind(wx.EVT_LIST_ITEM_SELECTED, self._on_row_selected)
        prog_box.Add(self.progress_list, 3, wx.EXPAND | wx.ALL, 3)
        self.info_text = wx.TextCtrl(self, style=wx.TE_MULTILINE | wx.TE_READONLY | wx.HSCROLL)
        self.info_text.SetFont(wx.Font(11, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL,
                                       wx.FONTWEIGHT_NORMAL))
        prog_box.Add(self.info_text, 2, wx.EXPAND | wx.ALL, 3)
        row = wx.BoxSizer(wx.HORIZONTAL)
        self.open_log_btn = wx.Button(self, label="Open Log")
        self.open_log_btn.SetToolTip("Open the selected row's log (the step's console when "
                                     "no row is selected)")
        self.open_log_btn.Disable()
        row.Add(self.open_log_btn, 0)
        prog_box.Add(row, 0, wx.ALL, 3)
        left.Add(prog_box, 1, wx.EXPAND | wx.TOP, 6)
        top.Add(left, 1, wx.EXPAND | wx.RIGHT, 8)

        ctrl = wx.StaticBoxSizer(wx.StaticBox(self, label="Bus step"), wx.VERTICAL)
        self.options = BusOptionsPanel(self)
        ctrl.Add(self.options, 0, wx.EXPAND | wx.ALL, 5)
        prow = wx.BoxSizer(wx.HORIZONTAL)
        self.check_python_btn = wx.Button(self, label="Check Python")
        self.check_python_btn.SetToolTip("Check, in a child process, that the step runs under "
                                         "this Python: it parses, and OR-Tools and the Rust "
                                         "router load")
        prow.Add(self.check_python_btn, 0, wx.RIGHT, 3)
        self.install_btn = wx.Button(self, label="Install OR-Tools")
        self.install_btn.SetToolTip("pip install OR-Tools into the step's Python")
        self.install_btn.Hide()
        prow.Add(self.install_btn, 0, wx.RIGHT, 3)
        self.use_python_btn = wx.Button(self, label="Use It")
        self.use_python_btn.SetToolTip("Run the step under the python named beside, which is ready")
        self.use_python_btn.Hide()
        prow.Add(self.use_python_btn, 0)
        ctrl.Add(prow, 0, wx.LEFT | wx.RIGHT, 5)
        self.python_status = wx.StaticText(self, label="", size=(220, -1),
                                           style=wx.ST_ELLIPSIZE_END)
        ctrl.Add(self.python_status, 0, wx.EXPAND | wx.ALL, 5)
        ctrl.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.ALL, 5)
        self.run_btn = wx.Button(self, label="Run Bus Step")
        self.run_btn.SetToolTip("Run awx/route_bus.py on a snapshot of this board in a temp "
                                "folder; the open board is not touched until Apply.")
        ctrl.Add(self.run_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        self.stop_btn = wx.Button(self, label="Stop")
        self.stop_btn.SetToolTip("Stop the step (kills it and every stage it runs)")
        self.stop_btn.Disable()
        ctrl.Add(self.stop_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        ctrl.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.ALL, 5)
        self.apply_btn = wx.Button(self, label="Apply Result to Board")
        self.apply_btn.SetToolTip("Make the open board match the step's output: tracks and "
                                  "vias (the lanes locked), poses, zones, the design rules "
                                  "and its records (the bus nets protected). Lists every "
                                  "change and asks first.")
        self.apply_btn.Disable()
        ctrl.Add(self.apply_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        self.open_folder_btn = wx.Button(self, label="Open Folder")
        self.open_folder_btn.SetToolTip("Open the run's work folder (the step's output, its "
                                        "work directory <OUT>.bus/ with every stage's log, and "
                                        "events.jsonl)")
        self.open_folder_btn.Disable()
        ctrl.Add(self.open_folder_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        ctrl.AddStretchSpacer(1)
        ctrl.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.ALL, 5)
        self.close_btn = wx.Button(self, label="Close")
        ctrl.Add(self.close_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        top.Add(ctrl, 0, wx.EXPAND)
        sizer.Add(top, 1, wx.EXPAND | wx.ALL, 8)

        gauges = wx.FlexGridSizer(cols=3, vgap=4, hgap=6)
        gauges.AddGrowableCol(1, 1)
        gauges.Add(wx.StaticText(self, label="Group stages:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.progress_bar = wx.Gauge(self, range=100)
        self.progress_bar.SetToolTip("The groups' stages ended, of those that will run "
                                     "(capture, ends model, routing, merge, grade per group "
                                     "and round)")
        gauges.Add(self.progress_bar, 1, wx.EXPAND)
        self.stage_count = wx.StaticText(self, label="", size=(110, -1))
        gauges.Add(self.stage_count, 0, wx.ALIGN_CENTER_VERTICAL)
        gauges.Add(wx.StaticText(self, label="Buses finished:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.bus_gauge = wx.Gauge(self, range=100)
        self.bus_gauge.SetToolTip("The buses of the group being routed whose process has "
                                  "ended, of its buses")
        gauges.Add(self.bus_gauge, 1, wx.EXPAND)
        self.bus_count = wx.StaticText(self, label="", size=(110, -1))
        gauges.Add(self.bus_count, 0, wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(gauges, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)
        status_row = wx.BoxSizer(wx.HORIZONTAL)
        status_row.Add(wx.StaticText(self, label="Status:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.status_text = wx.StaticText(self, label="Ready", style=wx.ST_ELLIPSIZE_END)
        status_row.Add(self.status_text, 1, wx.EXPAND)
        sizer.Add(status_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)
        self.SetSizer(sizer)

        self.find_btn.Bind(wx.EVT_BUTTON, lambda e: self.find())
        self.add_btn.Bind(wx.EVT_BUTTON, self._on_add)
        self.edit_btn.Bind(wx.EVT_BUTTON, self._on_edit)
        self.remove_btn.Bind(wx.EVT_BUTTON, self._on_remove)
        self.up_btn.Bind(wx.EVT_BUTTON, lambda e: self._move(-1))
        self.down_btn.Bind(wx.EVT_BUTTON, lambda e: self._move(1))
        self.use_found_btn.Bind(wx.EVT_BUTTON, self._on_use_found)
        self.bus_list.Bind(wx.EVT_CHECKLISTBOX, self._on_bus_checked)
        self.check_python_btn.Bind(wx.EVT_BUTTON, lambda e: self.check_python(force=True))
        self.install_btn.Bind(wx.EVT_BUTTON, self._on_install)
        self.use_python_btn.Bind(wx.EVT_BUTTON, self._on_use_python)
        self.run_btn.Bind(wx.EVT_BUTTON, lambda e: self.run())
        self.stop_btn.Bind(wx.EVT_BUTTON, self._on_stop)
        self.apply_btn.Bind(wx.EVT_BUTTON, lambda e: self.apply(confirm=True))
        self.open_folder_btn.Bind(wx.EVT_BUTTON, lambda e: self._launch(self.workdir))
        self.open_log_btn.Bind(wx.EVT_BUTTON, self._on_open_log)
        self.close_btn.Bind(wx.EVT_BUTTON, self._on_close)
        self._refresh_bus_list()

    def set_status(self, text):
        self.status_text.SetLabel(text)
        self.status_text.SetToolTip(text)

    @staticmethod
    def _launch(path):
        if path and os.path.exists(path):
            wx.LaunchDefaultApplication(path)

    # Settings persistence / resets (the options panel's).
    def get_config(self):
        return self.options.get_config()

    def set_config(self, cfg):
        self.options.set_config(cfg)

    def reset_to_defaults(self):
        self.options.reset_to_defaults()

    # ------------------------------------------------------------ the buses

    def _refresh_bus_list(self):
        rows = self.buses.rows
        self.bus_list.Set([r['spec'].replace(':', ' -> ')
                           + (f"  ({r['nets']} nets)" if r.get('nets') else '')
                           + (f" -- {r['why']}" if r.get('why') else '') for r in rows])
        for i, r in enumerate(rows):
            self.bus_list.Check(i, r['checked'])
        named = self.buses.argv_buses()
        if named is None and self.buses.found is None:
            self.bus_mode_text.SetLabel("The step finds the buses itself (no --bus); "
                                        "Find Buses lists them first.")
        elif named is None:
            self.bus_mode_text.SetLabel(f"As found: {len(rows)} bus(es); the run passes no "
                                        f"--bus and the step finds the same.")
        else:
            self.bus_mode_text.SetLabel("Named: --bus " + " --bus ".join(named) if named
                                        else "Named: none checked")
        self.not_taken_text.SetValue("\n".join(
            f"not taken {b['src']} -> {b['dest']}, {b['nets']} nets -- {b['why']}"
            for b in self.buses.left))

    def _on_bus_checked(self, event):
        i = event.GetInt()
        self.buses.rows[i]['checked'] = self.bus_list.IsChecked(i)
        self._refresh_bus_list()

    def _move(self, d):
        i = self.bus_list.GetSelection()
        if i == wx.NOT_FOUND:
            return
        j = self.buses.move(i, d)
        self._refresh_bus_list()
        self.bus_list.SetSelection(j)

    def _on_remove(self, event):
        i = self.bus_list.GetSelection()
        if i != wx.NOT_FOUND:
            self.buses.remove(i)
            self._refresh_bus_list()

    def _on_use_found(self, event):
        self.buses.use_found()
        self._refresh_bus_list()

    def _sources_and_partners(self):
        """(the source choices -- --find's ball arrays, else every part with
        more than 8 pads -- and {source: [(part, shared nets)]}, most first)."""
        fps = getattr(self.pcb_data, 'footprints', {}) or {}
        sources = list(self.buses.arrays) or sorted(
            (r for r, f in fps.items() if len(f.pads) > 8), key=lambda r: -len(fps[r].pads))
        refs_of = []
        for net in (getattr(self.pcb_data, 'nets', {}) or {}).values():
            refs = {p.component_ref for p in net.pads if p.component_ref}
            if 1 < len(refs) <= 6:
                refs_of.append(refs)
        partners = {}
        for s in sources:
            count = {}
            for refs in refs_of:
                if s in refs:
                    for r in refs - {s}:
                        if r in fps and len(fps[r].pads) > 2:
                            count[r] = count.get(r, 0) + 1
            partners[s] = sorted(count.items(), key=lambda x: (-x[1], x[0]))
        return sources, partners

    def _on_add(self, event):
        sources, partners = self._sources_and_partners()
        dlg = BusDialog(self, sources, partners)
        if dlg.ShowModal() == wx.ID_OK and dlg.spec():
            i = self.bus_list.GetSelection()
            self.buses.add(dlg.spec(), None if i == wx.NOT_FOUND else i + 1)
            self._refresh_bus_list()
        dlg.Destroy()

    def _on_edit(self, event):
        i = self.bus_list.GetSelection()
        if i == wx.NOT_FOUND:
            return
        sources, partners = self._sources_and_partners()
        dlg = BusDialog(self, sources, partners, spec=self.buses.rows[i]['spec'])
        if dlg.ShowModal() == wx.ID_OK and dlg.spec():
            self.buses.replace(i, dlg.spec())
            self._refresh_bus_list()
        dlg.Destroy()

    def set_plan_buses(self, specs):
        """A plan step's buses: its list, named in its order; None for the
        step's own (no --bus)."""
        if specs is None:
            self.buses.use_found()
        else:
            self.buses.set_named([str(s) for s in specs])
        self._refresh_bus_list()

    def on_shown(self):
        """The tab shown: list the buses the first time."""
        if self.buses.found is None and not self.is_busy():
            self.find()

    # ------------------------------------------------------------ the python

    def child_python(self):
        return self.options.bus_python.GetValue().strip() or python_for_subprocess()

    def _child_env(self, python):
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        # (the step's events go to its default file, <OUT stem>.bus/events.jsonl)
        env.pop('BUS_EVENTS', None)
        env.pop('BUS_EVENTS_BUS', None)
        if os.path.normcase(os.path.abspath(python)) != os.path.normcase(
                os.path.abspath(python_for_subprocess())):
            for k in _FOREIGN_PYTHON_VARS:
                env.pop(k, None)
        return env

    def check_python(self, force=False):
        """(ok, what): the step's python probed in a child (bus_progress.PROBE):
        route_bus.py parses under it, and OR-Tools and the Rust router load with
        pcbnew kept out, as the step keeps it out of its processes -- so KiCad's
        own python is ready once OR-Tools is installed in it. Once per python
        unless FORCE. With the Python field empty and KiCad's python not ready,
        a python3 on PATH that is ready is named, for Use It to fill in."""
        python = self.child_python()
        ok, what, installable = self._probe(python, force)
        self.suggested_python = None
        if not ok and not self.options.bus_python.GetValue().strip():
            import shutil
            alt = shutil.which('python3') or shutil.which('python')
            if alt and os.path.normcase(os.path.realpath(alt)) != os.path.normcase(
                    os.path.realpath(python)):
                if self._probe(alt, force)[0]:
                    self.suggested_python = alt
        self.python_status.SetLabel(("Python ready: " if ok else "Python NOT ready: ") + what
                                    + (f" -- {self.suggested_python} is: Use It"
                                       if self.suggested_python else ''))
        self.python_status.SetToolTip(what)
        self.install_btn.Show(installable)
        self.use_python_btn.Show(bool(self.suggested_python))
        self.Layout()
        return ok, what

    def _probe(self, python, force):
        if force or python not in self._probed:
            try:
                with wx.BusyCursor():
                    r = subprocess.run(bp.probe_argv(python), capture_output=True, text=True,
                                       timeout=180, env=self._child_env(python))
                v = bp.probe_verdict(r.returncode, (r.stdout or '') + (r.stderr or ''))
                what = f"{python} ({v['version'] or 'no version printed'}): " + (
                    'ready' if v['ok'] else '; '.join(v['problems']))
                ok, installable = v['ok'], v['installable']
            except (OSError, subprocess.TimeoutExpired) as e:
                ok, what, installable = False, f"{python}: could not run it ({e})", False
            self._probed[python] = (ok, what, installable)
        return self._probed[python]

    def _on_use_python(self, event):
        if self.suggested_python:
            self.options.bus_python.SetValue(self.suggested_python)
            self.check_python()

    def _on_install(self, event):
        python = self.child_python()
        from startup_checks import parse_requirements, requirements_path
        req = next((n + (s or '') for n, s in parse_requirements(requirements_path())
                    if n == 'ortools'), 'ortools')
        if wx.MessageBox(f"Install {req} into the bus step's Python?\n\n{python}\n\n"
                         f"It is installed there only: KiCad's own process never loads it.",
                         "Install OR-Tools", wx.YES_NO | wx.ICON_QUESTION) != wx.YES:
            return
        from .deps_check import _pip_install_threaded
        progress = wx.ProgressDialog("Installing OR-Tools", f"pip install {req}...", maximum=100,
                                     parent=self, style=wx.PD_APP_MODAL | wx.PD_CAN_ABORT | wx.PD_AUTO_HIDE)
        ok, log = _pip_install_threaded([req], progress, python_exe=python)
        progress.Destroy()
        if not ok:
            wx.MessageBox(f"pip install failed.\n\nBy hand:\n  \"{python}\" -m pip install --upgrade "
                          f"\"{req}\"\n\n{log[-1500:]}", "Install failed", wx.OK | wx.ICON_ERROR)
        self.check_python(force=True)

    # ------------------------------------------------------------ --find

    def is_busy(self):
        return self._child is not None or self._apply_pending

    def find(self):
        """route_bus.py --find on a snapshot at the shared sizes, as a child."""
        if self.is_busy():
            return False
        python = self.child_python()
        try:
            if self.find_dir is None or not os.path.isdir(self.find_dir):
                _prune_workdirs(1, prefix=WORKDIR_PREFIX + 'find_')
                self.find_dir = tempfile.mkdtemp(prefix=WORKDIR_PREFIX + 'find_')
            staged = snapshot_board(self.find_dir, self.board_filename)
        except Exception as e:                                  # noqa: BLE001
            self.set_status(f"Could not snapshot the board: {e}")
            return False
        self._found_json = os.path.join(self.find_dir, 'found.json')
        if os.path.exists(self._found_json):
            os.remove(self._found_json)
        argv = bp.find_argv(python, staged, self._found_json, self.get_sizes())
        return self._start('find', argv, self.find_dir, os.path.join(self.find_dir, 'find.log'),
                           f"Finding the buses: {' '.join(shlex.quote(a) for a in argv[4:])}")

    # ------------------------------------------------------------ the run

    def command(self, python, board, out):
        """The step's command line for BOARD -> OUT, as the CLI types it."""
        sizes = dict(self.get_sizes(), fanout_track_width=self.options.fanout_width())
        return bp.build_argv(python, board, out, self.buses.argv_buses(), self.options.options(), sizes)

    def run(self):
        """Run the step on a snapshot; returns whether it started."""
        if self.is_busy():
            return False
        self._cancel_requested = False
        if not os.path.isfile(bp.ROUTE_BUS_SCRIPT):
            self.set_status(f"The bus step is not here: {bp.ROUTE_BUS_SCRIPT}")
            return False
        try:
            argv_buses = self.buses.argv_buses()
            if argv_buses == []:
                self.set_status("No bus checked: check one, or As Found for the step's own")
                return False
        except ValueError as e:
            self.set_status(str(e))
            return False
        ok, what = self.check_python()
        if not ok:
            self.set_status("The bus step's Python is not ready: " + what)
            return False
        python = self.child_python()
        _prune_workdirs(KEEP_RUNS - 1, prefix=WORKDIR_PREFIX + 'run_')
        self.workdir = tempfile.mkdtemp(prefix=WORKDIR_PREFIX + 'run_')
        try:
            staged = snapshot_board(self.workdir, self.board_filename)
        except Exception as e:                                  # noqa: BLE001
            self.set_status(f"Could not snapshot the board: {e}")
            return False
        self.output = os.path.join(self.workdir, 'bus.kicad_pcb')
        self.events_path = bp.bus_events().default_path(self.output)
        self.argv = self.command(python, staged, self.output)
        self.model = bp.RunModel()
        self._offset = 0
        self._shown_key = None
        self.returncode = None
        self.last_apply_stats = None
        self.progress_list.DeleteAllItems()
        self.info_text.SetValue("")
        self.apply_btn.Disable()
        self.progress_bar.SetValue(0)
        self.bus_gauge.SetValue(0)
        self.stage_count.SetLabel("")
        self.bus_count.SetLabel("")
        self.append_log("Bus step: " + " ".join(shlex.quote(a) for a in self.argv) + "\n")
        return self._start('run', self.argv, self.workdir, os.path.join(self.workdir, 'bus.log'),
                           f"The bus step started on a snapshot in {self.workdir}")

    def run_for_plan(self):
        """The plan executor's invoke: run, and when the step ends, apply its
        output with no question asked and mark the step produced. A --find
        still listing is stopped first: the run finds or names its own."""
        from .gui_utils import PLAN_STEP_PRODUCED
        setattr(self, PLAN_STEP_PRODUCED, False)
        if self._job == 'find' and self._child is not None:
            self._abort_child()
        self._plan_run = True
        if not self.run():
            self._plan_run = False

    def _abort_child(self):
        """The running child killed and its state cleared, its output read
        to its end (a few seconds at most)."""
        child, self._child, self._job = self._child, None, None
        self._timer.Stop()
        child.kill()
        t0 = time.time()
        while not child.drain()[1] and time.time() - t0 < 5:
            time.sleep(0.05)
        self.run_btn.Enable()
        self.find_btn.Enable()
        self.stop_btn.Disable()

    def _start(self, job, argv, cwd, log_path, status):
        python = argv[0]
        try:
            self._child = ChildProcess(argv, cwd, env=self._child_env(python), log_path=log_path).start()
        except OSError as e:
            self._child = None
            self.set_status(f"Could not start {python}: {e}")
            return False
        self._job = job
        self.t_start = time.time()
        self._lines = []
        self.run_btn.Disable()
        self.find_btn.Disable()
        self.stop_btn.Enable()
        self.open_folder_btn.Enable(job == 'run')
        self.set_status(status)
        self._timer.Start(self.POLL_MS)
        return True

    def _on_stop(self, event):
        if self._child is not None:
            self._child.stop()
            self.set_status("Stopping the bus step...")
            self.stop_btn.Disable()

    def _on_tick(self, event):
        if not self:
            return
        child = self._child
        if child is None:
            self._timer.Stop()
            return
        if self._cancel_requested and not child.stopped:
            child.stop()
        child.tick()
        lines, done = child.drain()
        if self._job == 'run':
            for ln in lines:
                self.append_log(ln + "\n")
            self._read_events()
        self._lines.extend(lines)
        if done:
            self._timer.Stop()
            self._child = None
            job, self._job = self._job, None
            self.returncode = child.returncode
            self.run_btn.Enable()
            self.find_btn.Enable()
            self.stop_btn.Disable()
            if job == 'find':
                self._found(child)
            else:
                self._finished(child)

    # ------------------------------------------------------------ the events

    def _read_events(self):
        if not self.events_path:
            return
        evs, self._offset = bp.bus_events().read(self.events_path, self._offset)
        if evs:
            self.model.feed_all(evs)
        self._refresh_progress(bool(evs))

    def _refresh_progress(self, changed):
        now = time.time()
        rows = self.model.flat()
        lst = self.progress_list
        if changed or len(rows) != lst.GetItemCount():
            lst.Freeze()
            try:
                lst.DeleteAllItems()
                for i, r in enumerate(rows):
                    lst.InsertItem(i, '   ' * r['level'] + r['label'])
                    self._set_row(i, r, now)
                    colour = _STATE_COLOURS.get(r['state'])
                    if colour is not None:
                        lst.SetItemTextColour(i, colour)
                sel = next((i for i, r in enumerate(rows) if r['key'] == self._shown_key), None)
                if sel is not None:
                    lst.SetItemState(sel, wx.LIST_STATE_SELECTED, wx.LIST_STATE_SELECTED)
            finally:
                lst.Thaw()
        else:
            for i, r in enumerate(rows):
                if r['state'] == 'running':
                    lst.SetItem(i, 2, _fmt_secs(self.model.elapsed(r, now)))
        self._rows = rows
        fr = self.model.fractions()
        done, total = fr['stages']
        self.progress_bar.SetValue(int(100 * done / total) if total else 0)
        self.stage_count.SetLabel(f"{done} of {total}" if total else "")
        fin, buses = fr['buses']
        self.bus_gauge.SetValue(int(100 * fin / buses) if buses else 0)
        self.bus_count.SetLabel(f"{fin} of {buses}" if buses else "")
        if self._child is not None:
            el = _fmt_secs(now - (self.t_start or now))
            line = self.model.status_line(now)
            self.set_status(f"{el} running -- {line}" if line else f"{el} running -- starting")
        if self._shown_key is not None:
            self._show_row(self._shown_key)

    def _set_row(self, i, r, now):
        lst = self.progress_list
        lst.SetItem(i, 1, r['state'])
        lst.SetItem(i, 2, _fmt_secs(self.model.elapsed(r, now)))
        lst.SetItem(i, 3, r.get('result') or '')

    def _on_row_selected(self, event):
        i = event.GetIndex()
        if 0 <= i < len(self._rows):
            self._shown_key = self._rows[i]['key']
            self._show_row(self._shown_key)

    def _show_row(self, key):
        r = self.model.rows.get(key)
        if r is None:
            return
        el = self.model.elapsed(r, time.time())
        out = [r['label'], f"State: {r['state']}" + (f", {el:.0f} s" if el is not None else '')]
        if r.get('result'):
            out.append(f"Result: {r['result']}")
        out += [f"  {d}" for d in r.get('details') or ()]
        log = r.get('log')
        if log:
            out.append(f"Log: {r['log']}")
            if os.path.isfile(r['log']):
                out.append("--- log tail ---")
                out += _tail_file(r['log'])
        text = "\n".join(out)
        if self.info_text.GetValue() != text:
            self.info_text.SetValue(text)
        self.open_log_btn.Enable(bool(log and os.path.exists(log)))

    def _on_open_log(self, event):
        r = self.model.rows.get(self._shown_key) if self._shown_key is not None else None
        path = (r or {}).get('log') or (self.workdir and os.path.join(self.workdir, 'bus.log'))
        self._launch(path)

    # ------------------------------------------------------------ the end

    def _found(self, child):
        listing = bp.read_find(self._found_json)
        if listing is None:
            tail = [ln for ln in self._lines if ln.strip()][-4:]
            # (why, when it is the python: the probe says so beside it)
            self.check_python()
            self.set_status(f"--find gave no listing (exit {child.returncode}): "
                            + (" | ".join(tail) or 'no output'))
            return
        self.buses.set_found(listing, keep_rows=self.buses.names_buses())
        self._refresh_bus_list()
        self.set_status(f"Found {len(listing.get('buses') or [])} bus(es), "
                        f"{len(listing.get('left') or [])} not taken, at the shared sizes")

    def _finished(self, child):
        self._read_events()
        self.model.close(time.time(), 'stopped' if child.stopped else
                         f'the step ended (exit {child.returncode}) while it ran')
        self._refresh_progress(True)
        el = _fmt_secs(time.time() - (self.t_start or time.time()))
        ok = child.returncode == 0 and self.output and os.path.isfile(self.output)
        if child.stopped:
            self.set_status(f"Stopped after {el}; the work folder keeps what ran.")
        elif ok:
            s = self.model.summary()
            self.set_status(f"Finished in {el}: " + ({0: 'every bus handed on routed',
                                                      1: 'a bus left as it came (or none found)',
                                                      2: 'a bus refused'}.get(self.model.status, 'done'))
                            + (f" -- {'; '.join(f'{k}: {v}' for k, v in s.items())}" if s else '')
                            + ". Review, then Apply Result to Board.")
        else:
            tail = [ln for ln in self._lines if ln.strip()][-3:]
            self.set_status(f"The bus step FAILED (exit {child.returncode}) after {el}: "
                            + (" | ".join(tail) or 'no output'))
        self.apply_btn.Enable(bool(ok and not child.stopped))
        self.append_log(f"Bus step finished (exit code {child.returncode}) in {el}\n")
        if self._plan_run:
            self._plan_run = False
            if ok and not child.stopped:
                self._apply_pending = True
                wx.CallAfter(self._plan_apply)

    def _plan_apply(self):
        try:
            if self.apply(confirm=False):
                from .gui_utils import mark_plan_step_produced
                mark_plan_step_produced(self)
        finally:
            self._apply_pending = False

    def apply(self, confirm=True):
        """Make the open board match the step's output (result_apply).
        CONFIRM: list the changes and ask first. Returns whether it applied
        (an output that changes nothing counts)."""
        res = self.output
        if not res or not os.path.isfile(res):
            return False
        try:
            import pcbnew
        except ImportError:
            if confirm:
                wx.MessageBox("Not running inside pcbnew.", "Bus step", wx.OK | wx.ICON_ERROR)
            return False
        board = pcbnew.GetBoard()
        if board is None:
            return False
        from .result_apply import plan_apply, run_apply
        try:
            plan = plan_apply(board, res, pcbnew, fix_drc=bool(self.get_fix_drc()))
        except Exception as e:                                  # noqa: BLE001
            self.set_status(f"Could not read the step's output: {e}")
            return False
        lines = plan.lines()
        self.last_apply_message = "\n\n".join(lines)
        if not plan.has_changes:
            if confirm:
                wx.MessageBox(self.last_apply_message, "Apply Bus Step Result",
                              wx.OK | wx.ICON_INFORMATION)
            return True
        if confirm and wx.MessageBox(
                self.last_apply_message + "\n\nThe open board is made to match the step's output; "
                "it may have changed since the run started, and this list is against its state "
                "right now. Continue?", "Apply Bus Step Result",
                wx.YES_NO | wx.ICON_WARNING) != wx.YES:
            return False
        stats = run_apply(board, plan, pcbnew)
        self.last_apply_stats = stats
        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()
        if self.on_complete:
            self.on_complete()
        c = stats.get('copper') or {}
        msg = (f"Applied: {c.get('tracks_added', 0)} track(s) and {c.get('vias_added', 0)} via(s) "
               f"added, {c.get('removed', 0)} removed, {stats['moved']} footprint pose(s). "
               f"Use Edit -> Undo in pcbnew to revert.")
        self.set_status(msg)
        self.append_log("Bus step apply: " + msg + "\n")
        return True

    # -------------------------------------------------------- close/destroy

    def _on_close(self, event):
        if self.is_busy() and wx.MessageBox(
                "The bus step is still running. Closing the dialog stops it.\n\nClose anyway?",
                "Bus step", wx.YES_NO | wx.NO_DEFAULT | wx.ICON_WARNING) != wx.YES:
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
        """Stop the child before the dialog goes away."""
        try:
            self._timer.Stop()
        except Exception:                                       # noqa: BLE001
            pass
        if self._child is not None:
            self._child.kill()
