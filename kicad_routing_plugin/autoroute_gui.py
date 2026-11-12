"""
KiCad Routing Tools - Autorouter sub-tab (Auto tab notebook).

Runs the CLI's `py_router/autoroute.py` -- the deterministic chain it builds
from the board itself (planes, fanouts, decap clearance, diff pairs, route
passes) -- as a subprocess on a SNAPSHOT of the open board, and shows what it
is doing from the machine-readable lines it prints:

  AUTOROUTE_PLAN      -> the Plan pane (design rules and where they came
                         from, planes, the parts fanned out and the ones not,
                         with why) and one checklist row per planned step
  AUTOROUTE_STEP      -> a row's state (pending / running with its time
                         ticking / done / failed / skipped), its result, and
                         the rows the run adds (a fanout's next rung, a re-fan)
  AUTOROUTE_DECISION  -> what the chain chose about a step and why (the rung
                         kept, a route pass kept or not)
  AUTOROUTE_SUMMARY   -> the shipped board, its open nets and DRC

Selecting a row shows its command line, decisions, summary and log tail. The
run is the CLI's own, so the tab and a terminal get the same board; "Apply
Result to Board" then makes the open board match it (result_apply): footprint
poses, tracks and vias, zones with their fills, and the design rules.

The UI thread never blocks: a reader thread queues the child's stdout, a
250 ms wx timer drains it. Stop kills the child's whole process group (every
step runs as its own process under it).
"""

import json
import os
import queue
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

import wx

PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)
_ENGINE_DIR = os.path.join(ROOT_DIR, 'py_router')
if os.path.isdir(_ENGINE_DIR) and _ENGINE_DIR not in sys.path:
    sys.path.insert(0, _ENGINE_DIR)

AUTOROUTE_SCRIPT = os.path.join(_ENGINE_DIR, 'autoroute.py')

#: The CLI's defaults, read from the CLI so the two cannot drift.
try:
    import autoroute as _AR
    DEFAULT_EFFORT_FACTOR = _AR.EFFORT_FACTOR
    DEFAULT_PLANE_LAYER_COST = _AR.PLANE_LAYER_COST
except Exception:                                               # noqa: BLE001
    DEFAULT_EFFORT_FACTOR, DEFAULT_PLANE_LAYER_COST = 2.0, -1.0

EVENT_TOKENS = ('AUTOROUTE_PLAN', 'AUTOROUTE_STEP', 'AUTOROUTE_DECISION',
                'AUTOROUTE_SUMMARY', 'AUTOROUTE_ANALYSIS')

_STATE_COLOURS = {
    'running': wx.Colour(0, 90, 200),
    'done': wx.Colour(0, 120, 0),
    'failed': wx.Colour(190, 0, 0),
    'skipped': wx.Colour(120, 120, 120),
    'pending': wx.Colour(80, 80, 80),
}

_COLUMNS = (("#", 34), ("Step", 150), ("State", 96), ("Time", 56), ("Result", 420))

_LOG_TAIL_LINES = 14


def _fmt_secs(s):
    if s is None:
        return ""
    s = int(round(s))
    return f"{s // 60}:{s % 60:02d}"


def parse_event(line):
    """(token, payload) for one AUTOROUTE_* JSON line, else None."""
    for tok in EVENT_TOKENS:
        if line.startswith(tok + ': '):
            try:
                return tok, json.loads(line[len(tok) + 2:])
            except ValueError:
                return None
    return None


def python_for_subprocess():
    """A python that runs a CLI script: inside KiCad `sys.executable` is the
    pcbnew binary (deps_check solves this the same way)."""
    from .deps_check import _find_python_executable
    return _find_python_executable()


def _tail_file(path, n=_LOG_TAIL_LINES):
    try:
        with open(path, errors='replace') as fh:
            lines = [ln.rstrip('\n') for ln in fh.readlines()[-400:]]
    except OSError:
        return []
    return [ln for ln in lines if ln.strip()][-n:]


class AutorouteOptionsPanel(wx.Panel):
    """autoroute.py's options, the controls named after its params."""

    def __init__(self, parent):
        super().__init__(parent)
        grid = wx.FlexGridSizer(cols=2, vgap=6, hgap=8)
        self.finish = wx.CheckBox(self, label="Finish only (--finish)")
        self.finish.SetToolTip(
            "Only the final route passes, on a board another chain has already "
            "prepared and routed: its pours are read as the planes, and the board "
            "itself is a candidate, so a pass ships only if it beats it.")
        grid.Add(self.finish, 0, wx.ALIGN_CENTER_VERTICAL)
        grid.AddSpacer(0)
        grid.Add(wx.StaticText(self, label="Effort factor:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.effort_factor = wx.SpinCtrlDouble(
            self, min=0.0, max=100.0, initial=DEFAULT_EFFORT_FACTOR, inc=0.5, size=(80, -1))
        self.effort_factor.SetDigits(2)
        self.effort_factor.SetToolTip(
            "--effort-factor: a route pass after the first starts only while the "
            "router iterations spent, plus the last pass's again, stay within "
            "this many times the first pass's. A forecast, not a cap.")
        grid.Add(self.effort_factor, 0)
        grid.Add(wx.StaticText(self, label="Plane layer cost:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.plane_layer_cost = wx.SpinCtrlDouble(
            self, min=-10.0, max=10.0, initial=DEFAULT_PLANE_LAYER_COST, inc=0.5,
            size=(80, -1))
        self.plane_layer_cost.SetDigits(2)
        self.plane_layer_cost.SetToolTip(
            "--plane-layer-cost: the layer cost of every inner plane layer in the "
            "fanout, diff-pair and first route steps. Negative keeps signals off "
            "the planes (each stays a solid reference; later passes may cross at "
            "3.0); positive lets every step cross them. Not 0.")
        grid.Add(self.plane_layer_cost, 0)
        self.SetSizer(grid)

    def get_config(self):
        return {'finish': self.finish.GetValue(),
                'effort_factor': self.effort_factor.GetValue(),
                'plane_layer_cost': self.plane_layer_cost.GetValue()}

    def set_config(self, cfg):
        try:
            if 'finish' in cfg:
                self.finish.SetValue(bool(cfg['finish']))
            if 'effort_factor' in cfg:
                self.effort_factor.SetValue(float(cfg['effort_factor']))
            if 'plane_layer_cost' in cfg:
                self.plane_layer_cost.SetValue(float(cfg['plane_layer_cost']))
        except (TypeError, ValueError):
            pass  # a malformed saved dict must not break dialog restore

    def reset_to_defaults(self):
        self.set_config({'finish': False, 'effort_factor': DEFAULT_EFFORT_FACTOR,
                         'plane_layer_cost': DEFAULT_PLANE_LAYER_COST})

    def cli_args(self):
        """The options as autoroute.py arguments; raises ValueError for one the
        CLI would refuse."""
        cfg = self.get_config()
        if cfg['plane_layer_cost'] == 0:
            raise ValueError("Plane layer cost must not be 0 (the CLI refuses it).")
        args = ['--effort-factor', f"{cfg['effort_factor']:g}",
                '--plane-layer-cost', f"{cfg['plane_layer_cost']:g}"]
        if cfg['finish']:
            args.append('--finish')
        return args


class AutorouteTab(wx.Panel):
    """The Autorouter sub-tab (see module docstring)."""

    def __init__(self, parent, board_filename, on_complete=None, append_log=None,
                 sync_pcb_data_callback=None):
        super().__init__(parent)
        self.board_filename = board_filename
        self.on_complete = on_complete
        self.append_log = append_log or (lambda text: None)
        self.sync_pcb_data_callback = sync_pcb_data_callback
        self._proc = None
        self._queue = None
        self._reader = None
        self._kill_at = None
        self._stopped = False
        self.workdir = None
        self.output = None
        self._reset_run_state()
        self._create_ui()
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_tick, self._timer)
        self.Bind(wx.EVT_WINDOW_DESTROY, self._on_destroy)

    def _reset_run_state(self):
        self.rows = []              # one dict per checklist row
        self.plan = None
        self.summary = None
        self.decisions = []
        self.other_lines = []       # the child's prose (and any traceback)
        self.returncode = None
        self.t_start = None
        self.t_end = None
        self._shown_row = None

    # ----------------------------------------------------------------- UI

    def _create_ui(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        top = wx.BoxSizer(wx.HORIZONTAL)
        left = wx.BoxSizer(wx.VERTICAL)

        plan_box = wx.StaticBox(self, label="Plan")
        plan_sizer = wx.StaticBoxSizer(plan_box, wx.VERTICAL)
        self.plan_text = wx.TextCtrl(self, style=wx.TE_MULTILINE | wx.TE_READONLY,
                                     size=(-1, 136))
        self.plan_text.SetValue(
            "Run Autorouter routes a snapshot of this board with py_router/"
            "autoroute.py: it reads the board, plans its chain (planes, fanouts, "
            "decap clearance, diff pairs, route passes) and shows it here.")
        plan_sizer.Add(self.plan_text, 1, wx.EXPAND | wx.ALL, 3)
        left.Add(plan_sizer, 0, wx.EXPAND)

        steps_box = wx.StaticBox(self, label="Steps")
        steps_sizer = wx.StaticBoxSizer(steps_box, wx.VERTICAL)
        self.step_list = wx.ListCtrl(self, style=wx.LC_REPORT | wx.LC_SINGLE_SEL)
        for i, (name, width) in enumerate(_COLUMNS):
            self.step_list.InsertColumn(i, name, width=width)
        self.step_list.SetToolTip(
            "One row per step autoroute planned, and each step it added while "
            "running. Select a row for its command line, decisions and log.")
        self.step_list.Bind(wx.EVT_LIST_ITEM_SELECTED, self._on_row_selected)
        self.step_list.Bind(wx.EVT_SIZE, self._on_list_size)
        steps_sizer.Add(self.step_list, 1, wx.EXPAND | wx.ALL, 3)
        self.summary_text = wx.TextCtrl(self, style=wx.TE_MULTILINE | wx.TE_READONLY,
                                        size=(-1, 44))
        steps_sizer.Add(self.summary_text, 0, wx.EXPAND | wx.ALL, 3)
        left.Add(steps_sizer, 3, wx.EXPAND | wx.TOP, 6)

        detail_box = wx.StaticBox(self, label="Details")
        detail_sizer = wx.StaticBoxSizer(detail_box, wx.VERTICAL)
        self.detail_text = wx.TextCtrl(
            self, style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_RICH2 | wx.HSCROLL)
        self.detail_text.SetFont(wx.Font(11, wx.FONTFAMILY_TELETYPE,
                                         wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        detail_sizer.Add(self.detail_text, 1, wx.EXPAND | wx.ALL, 3)
        drow = wx.BoxSizer(wx.HORIZONTAL)
        self.open_log_btn = wx.Button(self, label="Open Step Log")
        self.open_log_btn.SetToolTip("Open the selected step's full log")
        self.open_log_btn.Disable()
        drow.Add(self.open_log_btn, 0)
        detail_sizer.Add(drow, 0, wx.ALL, 3)
        left.Add(detail_sizer, 2, wx.EXPAND | wx.TOP, 6)
        top.Add(left, 1, wx.EXPAND | wx.RIGHT, 8)

        ctrl_box = wx.StaticBox(self, label="Autorouter")
        ctrl_sizer = wx.StaticBoxSizer(ctrl_box, wx.VERTICAL)
        self.options = AutorouteOptionsPanel(self)
        ctrl_sizer.Add(self.options, 0, wx.ALL, 5)
        ctrl_sizer.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.ALL, 5)
        self.run_btn = wx.Button(self, label="Run Autorouter")
        self.run_btn.SetToolTip(
            "Run py_router/autoroute.py on a snapshot of this board, in a temp "
            "folder; the open board is not touched until Apply.")
        ctrl_sizer.Add(self.run_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        self.stop_btn = wx.Button(self, label="Stop")
        self.stop_btn.SetToolTip("Stop the run (kills the autorouter and its steps).")
        self.stop_btn.Disable()
        ctrl_sizer.Add(self.stop_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        ctrl_sizer.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.ALL, 5)
        self.apply_btn = wx.Button(self, label="Apply Result to Board")
        self.apply_btn.SetToolTip(
            "Make the open board match the shipped result: footprint poses, "
            "tracks and vias, zones with their fills, and the design rules. "
            "Lists every change and asks first.")
        self.apply_btn.Disable()
        ctrl_sizer.Add(self.apply_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        self.open_folder_btn = wx.Button(self, label="Open Folder")
        self.open_folder_btn.SetToolTip("Open the run's work folder (every step's "
                                        "board and log, and plan.json)")
        self.open_folder_btn.Disable()
        ctrl_sizer.Add(self.open_folder_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        ctrl_sizer.AddStretchSpacer(1)
        ctrl_sizer.Add(wx.StaticLine(self), 0, wx.EXPAND | wx.ALL, 5)
        self.close_btn = wx.Button(self, label="Close")
        self.close_btn.SetToolTip("Close dialog")
        ctrl_sizer.Add(self.close_btn, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        top.Add(ctrl_sizer, 0, wx.EXPAND)
        sizer.Add(top, 1, wx.EXPAND | wx.ALL, 8)

        self.progress_bar = wx.Gauge(self, range=100)
        sizer.Add(self.progress_bar, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)
        status_row = wx.BoxSizer(wx.HORIZONTAL)
        status_row.Add(wx.StaticText(self, label="Status:"), 0,
                       wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.status_text = wx.TextCtrl(self, value="Ready", style=wx.TE_READONLY)
        status_row.Add(self.status_text, 1, wx.EXPAND)
        sizer.Add(status_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)
        self.SetSizer(sizer)

        self.run_btn.Bind(wx.EVT_BUTTON, self._on_run)
        self.stop_btn.Bind(wx.EVT_BUTTON, self._on_stop)
        self.apply_btn.Bind(wx.EVT_BUTTON, self._on_apply)
        self.open_folder_btn.Bind(wx.EVT_BUTTON, lambda e: self._launch(self.workdir))
        self.open_log_btn.Bind(wx.EVT_BUTTON, self._on_open_log)
        self.close_btn.Bind(wx.EVT_BUTTON, self._on_close)

    def set_status(self, text):
        self.status_text.SetValue(text)
        self.status_text.SetToolTip(text)

    # Settings persistence / resets (the options panel's).
    def get_config(self):
        return self.options.get_config()

    def set_config(self, cfg):
        self.options.set_config(cfg)

    def reset_to_defaults(self):
        self.options.reset_to_defaults()

    @staticmethod
    def _launch(path):
        if path and os.path.exists(path):
            wx.LaunchDefaultApplication(path)

    # ------------------------------------------------------------- the run

    def is_running(self):
        return self._proc is not None

    def _snapshot(self, workdir):
        """Write the board to WORKDIR/input.kicad_pcb with its project
        siblings; returns the path, or raises."""
        staged = os.path.join(workdir, 'input.kicad_pcb')
        try:
            from copy_board import SIBLING_EXTS
        except Exception:                                       # noqa: BLE001
            SIBLING_EXTS = ('.kicad_pro', '.kicad_prl', '.kicad_dru',
                            '.design-brief.json')
        board = None
        try:
            import pcbnew
            board = pcbnew.GetBoard()
        except Exception:                                       # noqa: BLE001
            board = None
        if board is not None:
            # aSkipSettings: a snapshot; writing a .kicad_pro beside it crashes
            # on pre-KiCad-10 projects (see swig_gui._stage_live_board). The
            # siblings come from the board's own files, as the AI tabs stage.
            import pcbnew
            pcbnew.SaveBoard(staged, board, aSkipSettings=True)
        elif self.board_filename and os.path.isfile(self.board_filename):
            shutil.copyfile(self.board_filename, staged)
        else:
            raise RuntimeError("no open board and no board file to route")
        if self.board_filename:
            stem = os.path.splitext(os.path.abspath(self.board_filename))[0]
            for ext in SIBLING_EXTS:
                if os.path.isfile(stem + ext):
                    shutil.copyfile(stem + ext, os.path.join(workdir, 'input' + ext))
        return staged

    def _on_run(self, event):
        if self.is_running():
            return
        try:
            extra = self.options.cli_args()
        except ValueError as e:
            self.set_status(str(e))
            return
        if not os.path.isfile(AUTOROUTE_SCRIPT):
            self.set_status(f"Autorouter not found: {AUTOROUTE_SCRIPT}")
            return
        self._reset_run_state()
        self.step_list.DeleteAllItems()
        self.detail_text.SetValue("")
        self.summary_text.SetValue("")
        self.apply_btn.Disable()
        self.open_log_btn.Disable()
        self.progress_bar.SetValue(0)
        self.workdir = tempfile.mkdtemp(prefix='krt_autoroute_')
        try:
            staged = self._snapshot(self.workdir)
        except Exception as e:                                  # noqa: BLE001
            self.set_status(f"Could not snapshot the board: {e}")
            return
        self.output = os.path.join(self.workdir, 'routed.kicad_pcb')
        self.argv = [python_for_subprocess(), '-u', '-X', 'utf8', AUTOROUTE_SCRIPT,
                     staged, self.output] + extra
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        kw = {}
        if os.name == 'nt':
            kw['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kw['start_new_session'] = True
        try:
            self._proc = subprocess.Popen(
                self.argv, cwd=self.workdir, env=env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                errors='replace', bufsize=1, **kw)
        except OSError as e:
            self._proc = None
            self.set_status(f"Could not start the autorouter: {e}")
            return
        self._stopped = False
        self._kill_at = None
        self.t_start = time.time()
        self._queue = queue.Queue()
        self._reader = threading.Thread(target=self._read, args=(self._proc, self._queue),
                                        daemon=True)
        self._reader.start()
        self.run_btn.Disable()
        self.stop_btn.Enable()
        self.open_folder_btn.Enable()
        self.plan_text.SetValue("Analysing the board...")
        self.set_status(f"Autorouter started on a snapshot in {self.workdir}")
        self.append_log("Autorouter: " + " ".join(shlex.quote(a) for a in self.argv) + "\n")
        self._timer.Start(250)

    def _read(self, proc, q):
        """Reader thread: the child's stdout line by line into Q, then None.
        The whole stream is kept in the work folder as autoroute.log."""
        log = None
        try:
            log = open(os.path.join(self.workdir, 'autoroute.log'), 'w',
                       encoding='utf-8')
        except OSError:
            pass
        try:
            for line in proc.stdout:
                if log:
                    log.write(line)
                q.put(line.rstrip('\n'))
        finally:
            if log:
                log.close()
            proc.wait()
            q.put(None)

    def _on_stop(self, event):
        if not self.is_running():
            return
        self._stopped = True
        self.set_status("Stopping the autorouter...")
        self._signal(signal.SIGTERM if os.name != 'nt' else None)
        self._kill_at = time.time() + 3.0
        self.stop_btn.Disable()

    def _signal(self, sig):
        p = self._proc
        if p is None or p.poll() is not None:
            return
        try:
            if os.name == 'nt':
                subprocess.run(['taskkill', '/T', '/F', '/PID', str(p.pid)],
                               capture_output=True)
            else:
                os.killpg(os.getpgid(p.pid), sig)
        except (OSError, ProcessLookupError):
            pass

    def _on_tick(self, event):
        if not self:
            return
        if self._kill_at is not None and time.time() > self._kill_at:
            self._kill_at = None
            if os.name != 'nt':
                self._signal(signal.SIGKILL)
        done = False
        changed = False
        q = self._queue
        while q is not None:
            try:
                line = q.get_nowait()
            except queue.Empty:
                break
            if line is None:
                done = True
                break
            changed |= self._handle_line(line)
        if changed:
            self._refresh_list()
        else:
            self._refresh_times()
        if self._shown_row is not None and self._shown_row < len(self.rows) and \
                self.rows[self._shown_row]['state'] == 'running':
            self._show_row(self._shown_row)
        self._update_status()
        if done:
            self._finished()

    # ------------------------------------------------------------ events

    def _handle_line(self, line):
        ev = parse_event(line)
        if ev is None:
            if line.strip():
                self.other_lines.append(line)
                self.append_log(line + "\n")
            return False
        tok, p = ev
        if tok == 'AUTOROUTE_PLAN':
            self.plan = p
            self.plan_text.SetValue(self._plan_text(p))
            self.rows = [dict(r, state='pending', planned=True, parent=None)
                         for r in p.get('steps') or []]
            return True
        if tok == 'AUTOROUTE_STEP':
            return self._on_step(p)
        if tok == 'AUTOROUTE_DECISION':
            self.decisions.append(p)
            row = self._row(p.get('tag'))
            if row is not None:
                row.setdefault('decisions', []).append(p)
            return True
        if tok == 'AUTOROUTE_SUMMARY':
            self.summary = p
            return True
        return False

    def _row(self, tag):
        return next((r for r in self.rows if r.get('tag') == tag), None)

    def _on_step(self, p):
        tag = p.get('tag')
        row = self._row(tag)
        if row is None:
            # A step the plan did not list: placed after its parent's rows.
            row = {'tag': tag, 'tool': p.get('tool'), 'what': '', 'why': '',
                   'optional': False, 'planned': False, 'parent': p.get('parent'),
                   'state': 'pending'}
            parent = p.get('parent')
            at = len(self.rows)
            if parent:
                idx = [i for i, r in enumerate(self.rows)
                       if r.get('tag') == parent or r.get('parent') == parent]
                if idx:
                    at = idx[-1] + 1
            self.rows.insert(at, row)
        state = p.get('state')
        row['state'] = state
        for k in ('n', 'cmd', 'log', 'board', 'input', 'rc', 'seconds', 'summary',
                  'result', 'error_tail', 'grade', 'kept', 'reason', 'iterations'):
            if k in p:
                row[k] = p[k]
        if state == 'running':
            row['t0'] = time.time()
        if state == 'skipped':
            row['result'] = 'skipped: ' + (p.get('reason') or '')
        return True

    # ----------------------------------------------------------- display

    def _plan_text(self, p):
        r = p.get('rules') or {}
        lines = [
            f"{'Finish only (--finish)' if p.get('mode') == 'finish' else 'Full chain'} "
            f"on {p.get('n_nets', '?')} nets, {len(p.get('layers') or [])} copper layers"
            + ("; congested: deeper rip-up" if p.get('congested') else ""),
            f"Design rules: clearance {r.get('clearance')} mm, track {r.get('track')} mm, "
            f"via {r.get('via')}/{r.get('drill')} mm (fine {r.get('fine_via')}/"
            f"{r.get('fine_drill')}), grid {r.get('grid')} mm -- {p.get('rules_why', '')}",
        ]
        planes = p.get('planes') or []
        lines.append("Planes: " + (", ".join(f"{n} on {lay}" for n, lay in planes)
                                   if planes else "none"))
        parts = p.get('parts') or []
        line = "Fan out: " + (", ".join(f"{ref} ({tool} {pitch:g} mm)"
                                        for ref, tool, pitch in parts) if parts else "none")
        nf = p.get('not_fanned') or []
        if nf:
            line += "; not fanned out: " + ", ".join(
                f"{d['ref']} ({d['pads']} pads: {d['why']})" for d in nf[:6]) + \
                (" ..." if len(nf) > 6 else "")
        lines.append(line)
        pairs = p.get('pair_nets') or []
        if pairs:
            lines.append(f"Diff pairs: {len(pairs) // 2} -- " + ", ".join(
                f"{pairs[i]}/{pairs[i + 1]}" for i in range(0, min(len(pairs), 8) - 1, 2))
                + (" ..." if len(pairs) > 8 else ""))
        pn = p.get('power_nets') or []
        if pn:
            lines.append("Power nets: " + ", ".join(pn[:10]) + (" ..." if len(pn) > 10 else ""))
        if p.get('unselectable'):
            lines.append("Left out (no pattern selects them alone): "
                         + ", ".join(p['unselectable']))
        return "\n".join(lines)

    def _state_label(self, row):
        st = row.get('state')
        if st == 'pending':
            return 'optional' if row.get('optional') else 'pending'
        if st == 'failed':
            return 'FAILED'
        if st == 'done' and self.summary and self.summary.get('final_tag') == row.get('tag'):
            return 'done, shipped'
        return st or ''

    def _elapsed(self, row):
        if row.get('state') == 'running' and row.get('t0'):
            return time.time() - row['t0']
        return row.get('seconds')

    def _row_result(self, row):
        res = row.get('result') or ''
        if row.get('state') == 'pending' and not res:
            res = row.get('what') or ''
        for d in row.get('decisions') or ():
            res = (res + '; ' if res else '') + 'decision: ' + d.get('why', '')
        return res

    def _on_list_size(self, event):
        """The Result column takes the width the others leave."""
        event.Skip()
        lst = self.step_list
        rest = sum(lst.GetColumnWidth(i) for i in range(lst.GetColumnCount() - 1))
        lst.SetColumnWidth(lst.GetColumnCount() - 1,
                           max(_COLUMNS[-1][1], lst.GetClientSize().width - rest - 4))

    def _refresh_list(self):
        lst = self.step_list
        lst.Freeze()
        try:
            lst.DeleteAllItems()
            for i, row in enumerate(self.rows):
                label = row.get('tag') or ''
                if row.get('parent'):
                    label = '   ' + label
                lst.InsertItem(i, str(row.get('n') or ''))
                lst.SetItem(i, 1, label)
                lst.SetItem(i, 2, self._state_label(row))
                lst.SetItem(i, 3, _fmt_secs(self._elapsed(row)))
                lst.SetItem(i, 4, self._row_result(row))
                colour = _STATE_COLOURS.get(row.get('state'))
                if colour is not None:
                    lst.SetItemTextColour(i, colour)
            if self._shown_row is not None and self._shown_row < len(self.rows):
                lst.SetItemState(self._shown_row, wx.LIST_STATE_SELECTED,
                                 wx.LIST_STATE_SELECTED)
        finally:
            lst.Thaw()

    def _refresh_times(self):
        for i, row in enumerate(self.rows):
            if row.get('state') == 'running' and i < self.step_list.GetItemCount():
                self.step_list.SetItem(i, 3, _fmt_secs(self._elapsed(row)))

    def _counts(self):
        c = {'done': 0, 'failed': 0, 'skipped': 0, 'running': 0, 'pending': 0}
        for r in self.rows:
            c[r.get('state', 'pending')] = c.get(r.get('state', 'pending'), 0) + 1
        return c

    def _update_status(self):
        if not self.is_running():
            return
        c = self._counts()
        total = len(self.rows) or 1
        self.progress_bar.SetValue(int(100 * (c['done'] + c['failed'] + c['skipped']) / total))
        running = [r for r in self.rows if r.get('state') == 'running']
        el = _fmt_secs(time.time() - (self.t_start or time.time()))
        if running:
            r = running[0]
            what = f"running {r['tag']} ({_fmt_secs(self._elapsed(r))})"
        elif self.plan is None:
            what = "analysing the board"
        else:
            what = "grading / between steps"
        self.set_status(f"{el} -- {what}; {c['done']} done, {c['failed']} failed, "
                        f"{c['skipped']} skipped, {c['pending']} to go")

    def _on_row_selected(self, event):
        self._show_row(event.GetIndex())

    def _show_row(self, i):
        if i is None or i >= len(self.rows):
            return
        self._shown_row = i
        row = self.rows[i]
        out = [f"{row.get('tag')}" + (f" -- {row['what']}" if row.get('what') else "")]
        st = row.get('state')
        line = f"State: {self._state_label(row)}"
        if self._elapsed(row) is not None:
            line += f", {self._elapsed(row):.1f} s"
        if row.get('rc') is not None:
            line += f", exit code {row['rc']}"
        out.append(line)
        if row.get('why'):
            out.append(f"Why: {row['why']}")
        if row.get('parent'):
            out.append(f"Added by the run under: {row['parent']}")
        if row.get('result'):
            out.append(f"Result: {row['result']}")
        for d in row.get('decisions') or ():
            out.append(f"Decision: {'kept' if d.get('kept') else 'not kept'} -- {d.get('why')}")
        g = row.get('grade')
        if g:
            out.append(f"Grade: {g.get('open')} open net(s), {g.get('drc')} real DRC"
                       + (f"; open: {', '.join(g.get('open_nets')[:12])}"
                          if g.get('open_nets') else ""))
            for item in (g.get('drc_items') or [])[:10]:
                out.append(f"  DRC: {item}")
        if row.get('summary'):
            out.append("Summary: " + ", ".join(f"{k}={v}" for k, v in row['summary'].items()))
        if row.get('cmd'):
            out.append("Command: " + " ".join(shlex.quote(str(a)) for a in row['cmd']))
        if row.get('log'):
            out.append(f"Log: {row['log']}")
        if row.get('error_tail'):
            out.append("--- error: the last lines of its log ---")
            out += row['error_tail']
        elif row.get('log') and st in ('running', 'done'):
            tail = _tail_file(row['log'])
            if tail:
                out.append("--- log tail ---")
                out += tail
        text = "\n".join(out)
        if self.detail_text.GetValue() != text:
            self.detail_text.SetValue(text)
            if row.get('error_tail'):
                self.detail_text.SetStyle(0, len(out[0]), wx.TextAttr(wx.Colour(190, 0, 0)))
        self.open_log_btn.Enable(bool(row.get('log')) and os.path.isfile(row.get('log') or ''))

    def _on_open_log(self, event):
        if self._shown_row is not None and self._shown_row < len(self.rows):
            self._launch(self.rows[self._shown_row].get('log'))

    # ------------------------------------------------------------ finish

    def _finished(self):
        self._timer.Stop()
        proc, self._proc = self._proc, None
        self.returncode = proc.returncode if proc is not None else None
        self.t_end = time.time()
        self._queue = None
        self.run_btn.Enable()
        self.stop_btn.Disable()
        self.progress_bar.SetValue(100 if self.returncode == 0 else 0)
        # A row still running when the process ended was cut off.
        for r in self.rows:
            if r.get('state') == 'running':
                r['state'] = 'failed'
                r['result'] = ('stopped' if self._stopped else
                               'the autorouter ended while the step ran')
        self._refresh_list()
        el = _fmt_secs(self.t_end - (self.t_start or self.t_end))
        s = self.summary or {}
        ok = (self.returncode == 0 and s.get('output') and os.path.isfile(s['output']))
        if self._stopped:
            self.set_status(f"Stopped after {el}; the work folder keeps what ran.")
            self.summary_text.SetValue("Stopped: no result to apply.")
        elif ok:
            self.set_status(
                f"Finished in {el}: shipped {s.get('final_tag')} -- "
                f"{s.get('open')} open net(s), {s.get('drc')} real DRC. "
                f"Review, then Apply Result to Board.")
            self.summary_text.SetValue(self._final_summary(s))
        else:
            # Outright failure: say so plainly, with the child's last words.
            why = s.get('error') or ''
            tail = [ln for ln in self.other_lines if ln.strip()][-12:]
            if self.plan is None:
                why = why or "it stopped before it planned anything"
            self.set_status(f"Autorouter FAILED (exit code {self.returncode}) after "
                            f"{el}: {why or 'see the details pane'}")
            self.summary_text.SetValue("Failed: no result to apply.")
            self.detail_text.SetValue(
                f"The autorouter exited with code {self.returncode}.\n"
                + (f"{why}\n" if why else "")
                + "Command: " + " ".join(shlex.quote(a) for a in self.argv) + "\n"
                + "--- its last output ---\n" + "\n".join(tail))
            self._shown_row = None
        self.apply_btn.Enable(bool(ok and not self._stopped))
        self.append_log(f"Autorouter finished (exit code {self.returncode}) in {el}\n")

    def _final_summary(self, s):
        """What ships and what Apply will bring back, before Apply."""
        res = s.get('output')
        counts = {}
        try:
            with open(res, encoding='utf-8', errors='replace') as fh:
                text = fh.read()
            import re
            # Board-level items: one indent step (a tab, or two spaces).
            counts = {k: len(re.findall(r'^(?:\t| {2})\(' + tok + r'\b', text, re.M))
                      for k, tok in (('tracks', 'segment'), ('arcs', 'arc'),
                                     ('vias', 'via'), ('zones', 'zone'))}
        except OSError:
            pass
        moved = sum(int((r.get('summary') or {}).get('moved') or 0)
                    for r in self.rows if r.get('tool') == 'decap' and r.get('state') == 'done')
        open_nets = s.get('open_nets') or []
        lines = [f"Ships {s.get('final_step')} ({s.get('final_tag')}): "
                 f"{s.get('open')} open net(s)"
                 + (f" ({', '.join(open_nets[:8])}{' ...' if len(open_nets) > 8 else ''})"
                    if open_nets else "")
                 + f", {s.get('drc')} real DRC" + (f"; {s['note']}" if s.get('note') else "")]
        if counts:
            lines.append(
                f"Apply makes the open board match it: {counts['tracks'] + counts['arcs']} "
                f"track(s), {counts['vias']} via(s), {counts['zones']} zone(s); the decap "
                f"step moved {moved} part(s). The confirmation lists every change.")
        return "\n".join(lines)

    # ------------------------------------------------------------- apply

    def _on_apply(self, event):
        s = self.summary or {}
        res = s.get('output')
        if not res or not os.path.isfile(res):
            return
        try:
            import pcbnew
        except ImportError:
            wx.MessageBox("Not running inside pcbnew.", "Autorouter", wx.OK | wx.ICON_ERROR)
            return
        board = pcbnew.GetBoard()
        if board is None:
            wx.MessageBox("Board is no longer open.", "Autorouter", wx.OK | wx.ICON_ERROR)
            return
        from .result_apply import plan_apply, run_apply
        try:
            plan = plan_apply(board, res, pcbnew)
        except Exception as e:                                  # noqa: BLE001
            wx.MessageBox(f"Could not read the result board: {e}", "Autorouter",
                          wx.OK | wx.ICON_ERROR)
            return
        lines = plan.lines()
        lines.append("The open board is made to match the result board; it may "
                     "have changed since the run started, and this list is against "
                     "its state right now. Continue?")
        self.last_apply_message = "\n\n".join(lines)
        if wx.MessageBox(self.last_apply_message, "Apply Autorouter Result",
                         wx.YES_NO | wx.ICON_WARNING) != wx.YES:
            return
        stats = run_apply(board, plan, pcbnew)
        self.last_apply_stats = stats
        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()
        if self.on_complete:
            self.on_complete()
        c = stats.get('copper') or {}
        z = stats.get('zones') or {}
        msg = (f"Applied: {stats['moved']} footprint pose(s), {c.get('tracks_added', 0)} "
               f"track(s) and {c.get('vias_added', 0)} via(s) added, {c.get('removed', 0)} "
               f"removed, {z.get('added', 0)} zone(s) added and {z.get('removed', 0)} "
               f"removed. Use Edit -> Undo in pcbnew to revert.")
        self.set_status(msg)
        self.append_log("Autorouter apply: " + msg + "\n")

    # -------------------------------------------------------- close/destroy

    def _on_close(self, event):
        if self.is_running() and wx.MessageBox(
                "The autorouter is still running. Closing the dialog stops it.\n\n"
                "Close anyway?", "Autorouter",
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
        """Stop the child before the dialog goes away."""
        try:
            self._timer.Stop()
        except Exception:                                       # noqa: BLE001
            pass
        if self._proc is not None:
            self._signal(signal.SIGKILL if os.name != 'nt' else None)
