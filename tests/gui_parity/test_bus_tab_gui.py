#!/usr/bin/env python3
"""The Bus tab (kicad_routing_plugin/bus_gui.py) against awx/route_bus.py run by the CLI.

The tab runs the bus step as a CHILD PROCESS on a snapshot of the open board, follows its events file, and "Apply
Result to Board" makes the open board match its output (result_apply). This gate drives the REAL tab on a REAL
headless RoutingDialog (nothing mocked but the confirmation box) on a small synthetic bus -- a 12-net pair of ball
arrays on four copper layers, awx/synth_bus.py, about 30 s a run -- and checks:

  layout   the main notebook's pages are tab_names.MAIN_PAGES, "Bus" right after "Fanout"
  python   the tab probes the step's python in a child: one that runs the step is "ready"; KiCad's own is reported
           with the reason it is not, where it is not (the step needs OR-Tools; its source needs a newer Python)
  find     --find at the shared sizes lists the bus (SD1 -> SU1, 12 nets) checked, and the run then names none
  run      the tab's command is the CLI's command, argument for argument, and with REDO_MANIFEST in KiCad's
           environment the child records that command in it, as the CLI does; the events file the step writes fills
           the tab's rows (the group, its stages, the bus and its stages) and its gauges end full; the run's
           output grades as the CLI's own run of that command does -- the step's summary grade, check_connected
           and check_drc on both outputs, the bus's vias and copper
  apply    after Apply the live board, saved, grades as the CLI's output; its lanes are KiCad-locked and the bus
           nets protected in its records, as the CLI's output has them
  plan     a one-step AI plan, {"action": "bus", "params": {...}}, run by the real PlanExecutor: it drives the tab,
           waits for the child, applies, and the step is "done" -- the same command, the same grade
  stop     Stop mid-run ends the child and its stages, says so, and arms no Apply

Needs KiCad python (wx + pcbnew) for the dialog, and a python with OR-Tools for the step (--python, else
$KRT_BUS_PYTHON, else the first python3 on PATH that runs it). Re-execs into KiCad's python like its siblings;
exits 77 with a SKIP: reason when either is missing -- never 0.

    python3 tests/gui_parity/test_bus_tab_gui.py [--keep DIR] [--python PY]
"""
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"), key=path_version_key, reverse=True),
]
SCRIPT = os.path.join(REPO, 'awx', 'route_bus.py')
SIZES = ['--clearance-ceiling', '0.1', '--track-width', '0.1', '--via-size', '0.35', '--via-drill', '0.15']
RUN_TIMEOUT = 1800
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f" -- {detail}" if detail and not ok else ''), flush=True)
    if not ok:
        FAILS.append(name)


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'], capture_output=True).returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)] + sys.argv[1:]).returncode)
    print("SKIP: no python with pcbnew + wx found (this gate needs KiCad's)")
    sys.exit(77)


def _arg(flag):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else None


def step_python():
    """A python that runs the bus step (route_bus.py parses under it, OR-Tools and the Rust router load), or None"""
    sys.path.insert(0, REPO)
    from kicad_routing_plugin import bus_progress as bp
    cands = [_arg('--python'), os.environ.get('KRT_BUS_PYTHON'), shutil.which('python3'), shutil.which('python')]
    for c in cands:
        if not c or not os.path.exists(c):
            continue
        r = subprocess.run(bp.probe_argv(c), capture_output=True, text=True, timeout=180,
                           env={k: v for k, v in os.environ.items() if k not in ('PYTHONHOME', 'PYTHONPATH')})
        if r.returncode == 0 and 'OK' in r.stdout.split():
            return c
    return None


def summary_grade(out):
    """(the step's status, [(bus, its grade line's numbers)], the group's grade) from OUT's summary"""
    import re
    work = out[:-len('.kicad_pcb')] + '.bus'
    s = json.load(open(os.path.join(work, 'summary.json'), encoding='utf-8'))
    buses = []
    for b in s.get('buses') or []:
        g = b.get('grade') or ''
        on = g.split('| on the board:', 1)[-1]
        nums = {k: v for k, v in re.findall(r'(lanes|vias|copper|connected|drc|walled)=(\S+)', on)}
        lanes = re.search(r'lanes=(\S+)', g)
        buses.append((f"{b['src']}:{b['dest']}", lanes.group(1) if lanes else None,
                      {k: nums.get(k) for k in ('vias', 'copper', 'connected', 'drc', 'walled')}))
    gj = json.load(open(b['group'], encoding='utf-8')).get('grade') if s.get('buses') else None
    group = (len(gj['open']), gj['drc']['n'], len(gj['walled']['walled'])) if gj else None
    return s.get('exit'), buses, group


def grade(board):
    """(check_connected issue nets, check_drc violations) of BOARD at its own project's floors, each checker a
    fresh process of this (KiCad's) python"""
    env = dict(os.environ, KICAD_NO_GRADE_RECONCILE='1')
    probe = ("import contextlib, io, json, sys\n"
             "sys.path.insert(0, sys.argv[1])\n"
             "from check_connected import run_connectivity_check\n"
             "with contextlib.redirect_stdout(io.StringIO()):\n"
             "    issues = run_connectivity_check(sys.argv[2], quiet=True)\n"
             "print('ISSUES: ' + json.dumps(sorted(str(i.get('net_name')) for i in issues)))\n")
    r = subprocess.run([sys.executable, '-X', 'utf8', '-c', probe, os.path.join(REPO, 'py_router'), board],
                       capture_output=True, text=True, env=env)
    line = next((ln for ln in r.stdout.splitlines() if ln.startswith('ISSUES: ')), None)
    if r.returncode or line is None:
        raise AssertionError(f"BROKEN TEST: check_connected failed on {board}:\n{r.stdout[-1500:]}{r.stderr[-1500:]}")
    jpath = board + '.drc.json'
    r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(REPO, 'py_router', 'check_drc.py'), board,
                        '--quiet', '--clearance-margin', '0.1', '--json', jpath], capture_output=True, text=True,
                       env=env)
    try:
        drc = int(json.load(open(jpath))['violations'])
    except (OSError, ValueError, KeyError) as e:
        raise AssertionError(f"BROKEN TEST: check_drc failed on {board} ({e}):\n{r.stdout[-1500:]}{r.stderr[-1500:]}")
    return json.loads(line[len('ISSUES: '):]), drc


def copper(board, nets=None):
    """(tracks, vias, locked items) of BOARD's copper, on NETS when given: tracks and vias as multisets in nm"""
    from collections import Counter
    from kicad_parser import parse_kicad_pcb, mm_to_iu as nm
    p = parse_kicad_pcb(board)
    name = {i: n.name for i, n in p.nets.items()}
    keep = (lambda i: name.get(i) in nets) if nets else (lambda i: True)
    segs = Counter()
    locked = 0
    for s in p.segments:
        if getattr(s, 'graphic', False) or not keep(s.net_id):
            continue
        a, b = (nm(s.start_x), nm(s.start_y)), (nm(s.end_x), nm(s.end_y))
        segs[(min(a, b), max(a, b), nm(s.width), s.layer, name.get(s.net_id))] += 1
        locked += bool(getattr(s, 'locked', False))
    vias = Counter()
    for v in p.vias:
        if keep(v.net_id):
            vias[(nm(v.x), nm(v.y), nm(v.size), nm(v.drill), name.get(v.net_id))] += 1
            locked += bool(getattr(v, 'locked', False))
    return segs, vias, locked


def protected(board):
    """the nets BOARD's records protect (#521): the bus step's routed nets"""
    import tool_records
    from protected_nets import PRO_KEY
    return set((tool_records.effective_namespace(board) or {}).get(PRO_KEY) or {})


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import pcbnew
    import wx
    sys.path[:0] = [REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'tests', 'gui_parity')]
    py = step_python()
    if py is None:
        print("SKIP: no python here runs the bus step (route_bus.py must parse under it, and OR-Tools and the "
              "Rust router load); pass --python PY or set KRT_BUS_PYTHON")
        return 77
    print(f"the step's python: {py}", flush=True)
    keep = _arg('--keep')
    tmp = keep or tempfile.mkdtemp(prefix='krt_bus_gate_')
    os.makedirs(os.path.join(tmp, 'in'), exist_ok=True)
    src = os.path.join(tmp, 'in', 'board.kicad_pcb')
    r = subprocess.run([py, os.path.join(REPO, 'awx', 'synth_bus.py'), src, '--pattern', 'sorted', '--k', '12',
                        '--cols', '7', '--depth', '4', '--copper', '4'], capture_output=True, text=True)
    if r.returncode or not os.path.isfile(src):
        raise AssertionError(f"BROKEN TEST: synth_bus.py failed: {r.stdout[-800:]}{r.stderr[-800:]}")

    # ---- the CLI, on the same file the dialog opens, from a terminal's environment
    print("CLI: route_bus.py", flush=True)
    cli_dir = os.path.join(tmp, 'cli')
    os.makedirs(cli_dir, exist_ok=True)
    cli_out = os.path.join(cli_dir, 'bus.kicad_pcb')
    cli_argv = [py, '-u', '-X', 'utf8', SCRIPT, src, cli_out, '--rounds', '1'] + SIZES
    env = {k: v for k, v in os.environ.items() if k not in ('PYTHONHOME', 'PYTHONPATH', 'BUS_EVENTS', 'REDO_MANIFEST')}
    t0 = time.time()
    r = subprocess.run(cli_argv, capture_output=True, text=True, timeout=RUN_TIMEOUT, cwd=cli_dir, env=env)
    cli_secs = time.time() - t0
    open(os.path.join(cli_dir, 'run.log'), 'w').write(r.stdout + r.stderr)
    if r.returncode != 0 or not os.path.isfile(cli_out):
        raise AssertionError(f"BROKEN TEST: the CLI run failed (rc={r.returncode}):\n{r.stdout[-2000:]}")
    cli_sum = summary_grade(cli_out)
    print(f"  CLI: {cli_secs:.0f} s, status {cli_sum[0]}, buses {cli_sum[1]}, group {cli_sum[2]}", flush=True)

    # ---- the dialog
    app = wx.App(False)  # noqa: F841
    wx.Log.SetActiveTarget(wx.LogStderr())
    wx.MessageBox = lambda *a, **k: wx.YES
    board = pcbnew.LoadBoard(src)
    pcbnew.GetBoard = lambda: board
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin.swig_gui import RoutingDialog
    from kicad_routing_plugin.tab_names import MAIN_PAGES
    from kicad_routing_plugin import bus_progress as bp
    from wx_pump import run_until
    dlg = RoutingDialog(None, build_pcb_data_from_board(board), src)
    try:
        print("layout", flush=True)
        pages = [dlg.notebook.GetPageText(i) for i in range(dlg.notebook.GetPageCount())]
        check("main pages, left to right, are MAIN_PAGES", tuple(pages) == MAIN_PAGES, pages)
        check("'Bus' right after 'Fanout'", pages.index('Bus') == pages.index('Fanout') + 1, pages)
        tab = dlg.bus_tab
        check("the Bus page is the Bus tab", dlg.notebook.GetPage(pages.index('Bus')) is tab)

        print("python", flush=True)
        kicad_ok, kicad_what = tab.check_python(force=True)
        print(f"  KiCad's own python: {'ready' if kicad_ok else 'not ready'}: {kicad_what}", flush=True)
        check("KiCad's python is probed with a reason when not ready",
              kicad_ok or ('does not parse' in kicad_what or 'ortools' in kicad_what.lower()), kicad_what)
        tab.options.bus_python.SetValue(py)
        ok, what = tab.check_python(force=True)
        check("the step's python is ready", ok, what)

        # the shared sizes: the CLI's flags (Min Clearance as a ceiling, track, via, drill)
        dlg.clearance.SetValue(0.1)
        dlg.clearance_check.SetValue(True)
        dlg.clearance_ceiling_check.SetValue(True)
        for name, v in (('track_width', 0.1), ('via_size', 0.35), ('via_drill', 0.15)):
            getattr(dlg, name).SetValue(v)
            getattr(dlg, name + '_check').SetValue(True)
        tab.options.rounds.SetValue(1)

        print("find", flush=True)
        check("--find starts", tab.find(), tab.status_text.GetLabel())
        run_until(lambda: not tab.is_busy(), 300)
        rows = tab.buses.rows
        check("--find lists SD1 -> SU1, 12 nets, checked",
              [(r_['spec'], r_['nets'], r_['checked']) for r_ in rows] == [('SD1:SU1', 12, True)],
              f"{rows}; {tab.status_text.GetLabel()}")
        check("...the board's arrays", tab.buses.arrays == ['SD1', 'SU1'], tab.buses.arrays)
        check("...and the run names none (the step finds the same)", tab.buses.argv_buses() is None)

        def drive(stop_at=None):
            if not tab.run():
                raise AssertionError(f"BROKEN TEST: the tab did not start: {tab.status_text.GetLabel()}")
            if stop_at:
                run_until(lambda: any(e_ for e_ in [tab.model.latest] if e_ and e_.get('stage') == stop_at)
                          or not tab.is_busy(), RUN_TIMEOUT)
                tab._on_stop(None)
            if not run_until(lambda: not tab.is_busy(), RUN_TIMEOUT):
                tab._on_stop(None)
                run_until(lambda: not tab.is_busy(), 30)
                raise AssertionError("BROKEN TEST: the tab's run timed out")

        print("run", flush=True)
        # (a REDO_MANIFEST in KiCad's environment: the child records itself, as the CLI does)
        man = os.path.join(tmp, 'gui', 'redo_commands.sh')
        os.makedirs(os.path.dirname(man), exist_ok=True)
        os.environ.update(REDO_MANIFEST=man, REDO_RUNDIR=os.path.dirname(man))
        t0 = time.time()
        try:
            drive()
        finally:
            os.environ.pop('REDO_MANIFEST', None)
            os.environ.pop('REDO_RUNDIR', None)
        gui_secs = time.time() - t0
        tail = lambda a: a[1:5] + a[7:]                                  # noqa: E731  (the python and the paths)
        check("the tab's command is the CLI's, argument for argument",
              tail(tab.argv) == tail(cli_argv) and tab.argv[4] == SCRIPT, f"tab {tab.argv} cli {cli_argv}")
        check("the run exits 0 with its output", tab.returncode == 0 and os.path.isfile(tab.output),
              tab.status_text.GetLabel())
        import shlex
        rec = [ln for ln in (open(man, encoding='utf-8').read().splitlines() if os.path.isfile(man) else [])
               if ln and not ln.startswith('#') and ln != 'set -e']
        check("the GUI run is in the redo manifest: one line, the tab's command",
              len(rec) == 1 and shlex.split(rec[0]) == ['python3'] + tab.argv[1:], rec)
        evs = [json.loads(ln) for ln in open(tab.events_path, encoding='utf-8')]
        kinds = {(e['stage'], e['kind']) for e in evs}
        check("the events file names every stage", {('capture', 'end'), ('ends', 'solve'), ('route', 'start'),
                                                    ('fanout', 'end'), ('solve', 'end'), ('loop', 'end'),
                                                    ('lay', 'end'), ('bus', 'graded'), ('step', 'end')} <= kinds,
              sorted(kinds))
        m = tab.model
        check("the model read every event", m.events == len(evs), f"{m.events} of {len(evs)}")
        flat = m.flat()
        g = m.rows.get(('g', 'SD1'))
        b = m.rows.get(('b', 'SD1:SU1'))
        check("a group row and a bus row, both done", g and b and g['state'] == 'done' and b['state'] == 'done',
              [(r_['label'], r_['state']) for r_ in flat])
        check("the bus's stages: capture, fanout, solve, loop, lay",
              [r_['stage'] for r_ in flat if r_['parent'] == ('b', 'SD1:SU1')]
              == ['capture', 'fanout', 'solve', 'loop', 'lay'],
              [r_['stage'] for r_ in flat if r_['parent'] == ('b', 'SD1:SU1')])
        check("the bus row's result: lanes 12/12, open 0, DRC 0", b and b['result'].startswith('lanes 12/12; ')
              and 'open 0, DRC 0' in b['result'], b and b['result'])
        check("the list shows every row", tab.progress_list.GetItemCount() == len(flat),
              f"{tab.progress_list.GetItemCount()} of {len(flat)}")
        check("the gauges end full", tab.progress_bar.GetValue() == 100 and tab.bus_gauge.GetValue() == 100,
              f"{tab.stage_count.GetLabel()}, {tab.bus_count.GetLabel()}")
        check("the status says it finished", tab.status_text.GetLabel().startswith('Finished in'),
              tab.status_text.GetLabel())
        check("Apply is armed", tab.apply_btn.IsEnabled())
        gui_sum = summary_grade(tab.output)
        print(f"  tab: {gui_secs:.0f} s, status {gui_sum[0]}, buses {gui_sum[1]}, group {gui_sum[2]}", flush=True)
        check("the step's own grade is the CLI run's (status, each bus's lanes/vias/copper/connected/drc/walled, "
              "the group's open/drc/walled)", gui_sum == cli_sum, f"tab {gui_sum} cli {cli_sum}")
        cli_g, gui_g = grade(cli_out), grade(tab.output)
        print(f"  check_connected / check_drc: CLI {len(cli_g[0])} issue(s), {cli_g[1]} DRC; "
              f"tab {len(gui_g[0])} issue(s), {gui_g[1]} DRC", flush=True)
        check("check_connected and check_drc: the same on both outputs", cli_g == gui_g, f"cli {cli_g} gui {gui_g}")
        bus_nets = protected(cli_out)
        cs, cv, cl = copper(cli_out, bus_nets)
        gs, gv, gl = copper(tab.output, protected(tab.output))
        print(f"  bus copper: CLI {sum(cs.values())} tracks, {sum(cv.values())} vias, {cl} locked; "
              f"tab {sum(gs.values())} tracks, {sum(gv.values())} vias, {gl} locked", flush=True)
        check("the same bus nets protected on both outputs", bus_nets and protected(tab.output) == bus_nets,
              f"cli {sorted(bus_nets)} tab {sorted(protected(tab.output))}")
        check("the same bus copper, to the nm (tracks, vias, locks)", (cs, cv, cl) == (gs, gv, gl),
              f"tracks {sum(cs.values())}/{sum(gs.values())} vias {sum(cv.values())}/{sum(gv.values())}")

        print("apply", flush=True)
        check("Apply applies", tab.apply(confirm=True), tab.status_text.GetLabel())
        saved = os.path.join(tmp, 'gui', 'applied.kicad_pcb')
        os.makedirs(os.path.dirname(saved), exist_ok=True)
        pcbnew.SaveBoard(saved, board)
        sg = grade(saved)
        check("the live board, saved, grades as the CLI's output", sg == cli_g, f"saved {sg} cli {cli_g}")
        ss, sv, sl = copper(saved, bus_nets)
        check("...its bus copper the tab's output's, the lanes locked", (ss, sv, sl) == (gs, gv, gl),
              f"tracks {sum(ss.values())}/{sum(gs.values())} locked {sl}/{gl}")
        check("...the bus nets protected in its records", protected(saved) == bus_nets,
              f"{sorted(protected(saved))}")

        print("plan", flush=True)
        board2 = pcbnew.LoadBoard(src)
        pcbnew.GetBoard = lambda: board2
        dlg._sync_pcb_data_from_board()
        from kicad_routing_plugin.ai_plan import PlanExecutor, parse_plan_result, step_label
        steps, errs = parse_plan_result(json.dumps({'steps': [{'action': 'bus', 'params': {
            'rounds': 1, 'clearance_ceiling': 0.1, 'track_width': 0.1, 'via_size': 0.35, 'via_drill': 0.15}}]}))
        check("the plan parses: one bus step", steps and [s['action'] for s in steps] == ['bus'], errs)
        check("its label", steps and step_label(1, steps[0]) == '1. Bus step: the buses it finds (--find)',
              steps and step_label(1, steps[0]))
        status, done = {}, {}
        ex = PlanExecutor(dlg, steps, [0], on_status=lambda i, s: status.__setitem__(i, s),
                          on_finished=lambda n, why: done.update(n=n, why=why), log=lambda m_: None, quiet=True)
        ex.start()
        if not run_until(lambda: 'n' in done, RUN_TIMEOUT):
            raise AssertionError("BROKEN TEST: the plan did not finish")
        check("the plan's bus step is done", status.get(0) == 'done' and done.get('why') is None,
              f"{status} {done}; {tab.status_text.GetLabel()}")
        check("...it ran the CLI's command", tail(tab.argv) == tail(cli_argv), f"{tab.argv}")
        plan_saved = os.path.join(tmp, 'plan', 'applied.kicad_pcb')
        os.makedirs(os.path.dirname(plan_saved), exist_ok=True)
        pcbnew.SaveBoard(plan_saved, board2)
        pg = grade(plan_saved)
        check("...and the live board after it grades as the CLI's output", pg == cli_g, f"plan {pg} cli {cli_g}")
        check("...its bus nets protected", protected(plan_saved) == bus_nets, f"{sorted(protected(plan_saved))}")

        print("stop", flush=True)
        drive(stop_at='capture')
        check("Stop ends the run and says so", tab.status_text.GetLabel().startswith('Stopped after'),
              tab.status_text.GetLabel())
        check("...no Apply", not tab.apply_btn.IsEnabled())
        check("...no row left running", not any(r_['state'] == 'running' for r_ in tab.model.flat()),
              [(r_['label'], r_['state']) for r_ in tab.model.flat()])
        check("...the step's process group is gone", tab._child is None)
    finally:
        try:
            dlg.Destroy()
        except Exception:                                         # noqa: BLE001
            pass
    print(f"\nwork folder: {tmp}")
    if FAILS:
        print(f"VERDICT: FAIL ({len(FAILS)}): " + "; ".join(FAILS))
        return 1
    print(f"VERDICT: the Bus tab runs the CLI's command and its result grades as the CLI's "
          f"(CLI {cli_secs:.0f} s, tab {gui_secs:.0f} s)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
