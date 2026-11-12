#!/usr/bin/env python3
"""The Auto tab's Autorouter sub-tab against autoroute.py run by the CLI.

The tab runs py_router/autoroute.py as a subprocess on a snapshot of the
open board, shows its plan and steps from the AUTOROUTE_* lines, and "Apply
Result to Board" makes the open board match the result (result_apply). This
gate drives the REAL tab on a REAL headless RoutingDialog (nothing mocked but
the confirmation box, answered Yes) and checks:

  layout   the main notebook opens on page 0, "Auto", and the pages are
           tab_names.MAIN_PAGES ("Advanced", not "Advanced options"); the
           Auto sub-tabs are Autorouter, AI Routing, AI Placement, Labels
  run      the tab's checklist ends with every row terminal, in the states
           the CLI's own run reports for the same steps, and ships the same
           step with the same open-net and DRC grade
  apply    after Apply the live board, saved as KiCad saves it (SaveBoard,
           project included), grades as the CLI's output does -- the same
           check_connected issues and check_drc violations -- with the same
           zones (net, layers, outline, fill), footprint poses, and tracks
           and vias
  failure  a step made to fail (KRT_AUTOROUTE_FAIL_STEPS) shows FAILED in
           its row, and its exit code and error in the detail pane; a run
           whose every step fails says FAILED in the status and arms no
           Apply; Stop kills a run mid-route and says so

Board: kicad_files/splitflap_driver.kicad_pcb (a GND pour on B.Cu, then one
route pass; about 20 s a run). Needs KiCad python (wx + pcbnew); re-execs
into it like its siblings, and exits 77 with a SKIP: reason when there is
none -- never 0.

    python3 tests/gui_parity/test_autoroute_tab_gui.py [--keep DIR]
    python3 tests/gui_parity/test_autoroute_tab_gui.py --apply-check INPUT CLI_OUTPUT

--apply-check skips the runs: it applies a CLI output to its input loaded as
the live board and compares as above, for a board too slow to run twice.
"""
import glob
import json
import os
import subprocess
import sys
import tempfile
from collections import Counter

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/"
    "Versions/Current/bin/python3",
    "/usr/bin/python3",
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]
BOARD = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
RUN_TIMEOUT = 900
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f" -- {detail}" if detail else ''),
          flush=True)
    if not ok:
        FAILS.append(name)


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        r = subprocess.run([cand, '-c', 'import pcbnew, wx'], capture_output=True)
        if r.returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("SKIP: no python with pcbnew + wx found (this gate needs KiCad's)")
    sys.exit(77)


# ------------------------------------------------------------------ grading

def grade(board):
    """(check_connected issue count, its issue nets, check_drc violations)
    for BOARD, each checker as a fresh process, at the board's own recorded
    floors (its .kicad_pro), the KiCad cross-check off."""
    env = dict(os.environ, KICAD_NO_GRADE_RECONCILE='1')
    probe = ("import contextlib, io, json, sys\n"
             "sys.path.insert(0, sys.argv[1])\n"
             "from check_connected import run_connectivity_check\n"
             "with contextlib.redirect_stdout(io.StringIO()):\n"
             "    issues = run_connectivity_check(sys.argv[2], quiet=True)\n"
             "print('ISSUES: ' + json.dumps(sorted(str(i.get('net_name')) for i in issues)))\n")
    r = subprocess.run([sys.executable, '-X', 'utf8', '-c', probe,
                        os.path.join(REPO, 'py_router'), board],
                       capture_output=True, text=True, env=env)
    line = next((ln for ln in r.stdout.splitlines() if ln.startswith('ISSUES: ')), None)
    if r.returncode or line is None:
        raise AssertionError(f"BROKEN TEST: check_connected failed on {board}:\n"
                             f"{r.stdout[-1500:]}{r.stderr[-1500:]}")
    issues = json.loads(line[len('ISSUES: '):])
    jpath = board + '.drc.json'
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(REPO, 'py_router', 'check_drc.py'), board,
                        '--quiet', '--json', jpath], capture_output=True, text=True, env=env)
    try:
        with open(jpath) as fh:
            drc = int(json.load(fh)['violations'])
    except (OSError, ValueError, KeyError) as e:
        raise AssertionError(f"BROKEN TEST: check_drc failed on {board} ({e}):\n"
                             f"{r.stdout[-1500:]}{r.stderr[-1500:]}")
    return len(issues), issues, drc


def board_facts(path):
    """Zones, poses, tracks and vias of the board at PATH, as comparable
    multisets (coordinates to 1 um)."""
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(path)
    name = {nid: n.name for nid, n in pcb.nets.items()}
    r = lambda v: round(float(v), 3)
    zones = Counter()
    for z in pcb.zones:
        if z.in_footprint:
            continue
        zones[(z.net_name, z.layer, tuple((r(x), r(y)) for x, y in z.polygon),
               len(z.holes), z.priority, z.clearance, z.min_thickness,
               z.pad_connection)] += 1
    # The fill: the parser keeps no fill polygons, so count the file's.
    import re
    with open(path, encoding='utf-8', errors='replace') as fh:
        text = fh.read()
    zones[('filled_polygon blocks', len(re.findall(r'\(filled_polygon\b', text)))] += 1
    poses = {ref: (r(fp.x), r(fp.y), round(float(fp.rotation or 0) % 360, 2), fp.layer)
             for ref, fp in pcb.footprints.items()}
    segs = Counter()
    for s in pcb.segments:
        if getattr(s, 'graphic', False):
            continue
        a, b = (r(s.start_x), r(s.start_y)), (r(s.end_x), r(s.end_y))
        segs[(min(a, b), max(a, b), r(s.width), s.layer, name.get(s.net_id))] += 1
    vias = Counter((r(v.x), r(v.y), r(v.size), r(v.drill), tuple(v.layers or ()),
                    name.get(v.net_id)) for v in pcb.vias)
    return {'zones': zones, 'poses': poses, 'tracks': segs, 'vias': vias}


def project_rules(board):
    """{rule: mm} the board's sibling .kicad_pro declares -- the floors check_drc
    and KiCad grade at -- plus its Default class clearance as 'class:Default'."""
    with open(os.path.splitext(board)[0] + '.kicad_pro', encoding='utf-8') as fh:
        proj = json.load(fh)
    rules = (proj.get('board') or {}).get('design_settings', {}).get('rules') or {}
    out = {k: float(v) for k, v in rules.items()
           if k in ('min_clearance', 'min_track_width', 'min_via_diameter',
                    'min_through_hole_diameter', 'min_hole_to_hole',
                    'min_copper_edge_clearance', 'min_hole_clearance')
           and isinstance(v, (int, float))}
    for nc in (proj.get('net_settings') or {}).get('classes') or []:
        if nc.get('name') == 'Default' and isinstance(nc.get('clearance'), (int, float)):
            out['class:Default'] = float(nc['clearance'])
    return out


def events(text):
    out = []
    for line in text.splitlines():
        for tok in ('AUTOROUTE_PLAN', 'AUTOROUTE_STEP', 'AUTOROUTE_SUMMARY'):
            if line.startswith(tok + ': '):
                out.append((tok, json.loads(line[len(tok) + 2:])))
    return out


def terminal_states(evs):
    """{tag: terminal state} from AUTOROUTE_STEP payloads."""
    return {p['tag']: p['state'] for tok, p in evs
            if tok == 'AUTOROUTE_STEP' and p['state'] != 'running'}


def compare_saved(board, cli_out, gui_out):
    """Save the live BOARD as KiCad saves it (project included) to GUI_OUT and
    grade it against the CLI's CLI_OUT: project floors, check_connected,
    check_drc, zones, poses, tracks and vias."""
    import pcbnew
    os.makedirs(os.path.dirname(gui_out), exist_ok=True)
    pcbnew.SaveBoard(gui_out, board)        # as KiCad saves: project too
    check("the saved board has its project",
          os.path.isfile(gui_out[:-len('.kicad_pcb')] + '.kicad_pro'))
    cli_rules, gui_rules = project_rules(cli_out), project_rules(gui_out)
    drift = {k: (v, gui_rules.get(k)) for k, v in cli_rules.items()
             if gui_rules.get(k) is None or abs(gui_rules[k] - v) > 1e-6}
    check("the saved project carries the result's rule floors and Default class",
          cli_rules and not drift, f"cli {cli_rules}; differ {drift}")
    cli_g, gui_g = grade(cli_out), grade(gui_out)
    print(f"  CLI: {cli_g[0]} connectivity issue(s), {cli_g[2]} DRC; "
          f"GUI: {gui_g[0]} issue(s), {gui_g[2]} DRC", flush=True)
    check("check_connected: the same issues", cli_g[:2] == gui_g[:2],
          f"cli {cli_g[1]} gui {gui_g[1]}")
    check("check_drc: the same violation count", cli_g[2] == gui_g[2],
          f"cli {cli_g[2]} gui {gui_g[2]}")
    cf, gf = board_facts(cli_out), board_facts(gui_out)
    check("the same zones (net, layer, outline, fill)", cf['zones'] == gf['zones'],
          f"cli {sorted(cf['zones'].items())[:3]} gui {sorted(gf['zones'].items())[:3]}")
    moved = [k for k in cf['poses'] if cf['poses'][k] != gf['poses'].get(k)]
    check("the same footprint poses", not moved and len(cf['poses']) == len(gf['poses']),
          f"{len(moved)} differ: {moved[:5]}")
    check("the same tracks", cf['tracks'] == gf['tracks'],
          f"cli {sum(cf['tracks'].values())} gui {sum(gf['tracks'].values())}, "
          f"{sum((cf['tracks'] - gf['tracks']).values())} only in the CLI's, "
          f"{sum((gf['tracks'] - cf['tracks']).values())} only in the GUI's")
    check("the same vias", cf['vias'] == gf['vias'],
          f"cli {sum(cf['vias'].values())} gui {sum(gf['vias'].values())}")
    return cli_g, gui_g


def apply_check(src, cli_out, tmp):
    """--apply-check: no run, Apply alone. Load SRC as the live board, apply
    the CLI's CLI_OUT to it through result_apply (what the tab's Apply
    calls), save and compare -- for a board too slow to run here twice."""
    import pcbnew
    import wx
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    app = wx.App(False)  # noqa: F841
    wx.Log.SetActiveTarget(wx.LogStderr())
    board = pcbnew.LoadBoard(src)
    pcbnew.GetBoard = lambda: board
    from kicad_routing_plugin.result_apply import plan_apply, run_apply
    plan = plan_apply(board, cli_out, pcbnew)
    print("\n\n".join(plan.lines()), flush=True)
    stats = run_apply(board, plan, pcbnew)
    print(f"  applied: {stats}", flush=True)
    compare_saved(board, cli_out, os.path.join(tmp, 'gui', 'applied.kicad_pcb'))
    if FAILS:
        print(f"VERDICT: FAIL ({len(FAILS)}): " + "; ".join(FAILS))
        return 1
    print("VERDICT: Apply makes the live board grade as the CLI's output")
    return 0


# --------------------------------------------------------------------- run

def run(keep):
    import pcbnew
    import wx
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, os.path.join(REPO, 'tests', 'gui_parity'))
    from copy_board import copy_board
    from kicad_parser import build_pcb_data_from_board
    import autoroute as AR

    tmp = keep or tempfile.mkdtemp(prefix='krt_autoroute_gate_')
    os.makedirs(tmp, exist_ok=True)
    src = os.path.join(tmp, 'in', 'board.kicad_pcb')
    os.makedirs(os.path.dirname(src), exist_ok=True)
    copy_board(BOARD, src)

    # ---- the CLI, on the same file the dialog opens
    print("CLI: autoroute.py", flush=True)
    cli_out = os.path.join(tmp, 'cli', 'routed.kicad_pcb')
    os.makedirs(os.path.dirname(cli_out), exist_ok=True)
    env = {k: v for k, v in os.environ.items() if k != AR.FAIL_STEPS_ENV}
    r = subprocess.run([sys.executable, '-u', '-X', 'utf8',
                        os.path.join(REPO, 'py_router', 'autoroute.py'), src, cli_out],
                       capture_output=True, text=True, timeout=RUN_TIMEOUT, env=env)
    if r.returncode != 0 or not os.path.isfile(cli_out):
        raise AssertionError(f"BROKEN TEST: the CLI run failed (rc={r.returncode}):\n"
                             f"{r.stdout[-2000:]}{r.stderr[-2000:]}")
    cli_ev = events(r.stdout)
    cli_summary = [p for tok, p in cli_ev if tok == 'AUTOROUTE_SUMMARY'][-1]

    # ---- the dialog
    app = wx.App(False)  # noqa: F841
    wx.Log.SetActiveTarget(wx.LogStderr())
    asked = []

    def _yes(msg, *a, **k):
        asked.append(msg)
        return wx.YES
    wx.MessageBox = _yes
    board = pcbnew.LoadBoard(src)
    pcbnew.GetBoard = lambda: board
    from kicad_routing_plugin.swig_gui import RoutingDialog
    from kicad_routing_plugin.tab_names import MAIN_PAGES, AUTO_SUBTABS
    from wx_pump import run_until
    dlg = RoutingDialog(None, build_pcb_data_from_board(board), src)
    try:
        print("layout", flush=True)
        nb = dlg.notebook
        pages = [nb.GetPageText(i) for i in range(nb.GetPageCount())]
        check("main page 0 is Auto", pages[:1] == ['Auto'], pages)
        check("main pages, left to right", tuple(pages) == MAIN_PAGES, pages)
        check("'Advanced options' is now 'Advanced'",
              'Advanced' in pages and 'Advanced options' not in pages, pages)
        check("a fresh dialog opens on the Auto page", nb.GetSelection() == 0,
              f"selection {nb.GetSelection()}")
        sub = [dlg.ai_notebook.GetPageText(i) for i in range(dlg.ai_notebook.GetPageCount())]
        check("Auto sub-tabs: Autorouter, AI Routing, AI Placement, Labels",
              tuple(sub) == AUTO_SUBTABS == ('Autorouter', 'AI Routing', 'AI Placement',
                                             'Labels'), sub)
        check("the sub-tab panels are the dialog's tabs",
              [dlg.ai_notebook.GetPage(i) for i in range(4)]
              == [dlg.autoroute_tab, dlg.ai_tab, dlg.placement_tab, dlg.labels_tab])

        tab = dlg.autoroute_tab

        def drive(fail_steps=None, stop_at=None):
            if fail_steps:
                os.environ[AR.FAIL_STEPS_ENV] = fail_steps
            else:
                os.environ.pop(AR.FAIL_STEPS_ENV, None)
            try:
                tab._on_run(None)
                if not tab.is_running():
                    raise AssertionError(f"BROKEN TEST: the tab did not start: "
                                         f"{tab.status_text.GetValue()}")
                if stop_at:
                    run_until(lambda: any(rw.get('tag') == stop_at
                                          and rw.get('state') == 'running'
                                          for rw in tab.rows) or not tab.is_running(),
                              RUN_TIMEOUT)
                    tab._on_stop(None)
                ok = run_until(lambda: not tab.is_running(), RUN_TIMEOUT)
                if not ok:
                    tab._on_stop(None)
                    run_until(lambda: not tab.is_running(), 30)
                    raise AssertionError("BROKEN TEST: the tab's run timed out")
            finally:
                os.environ.pop(AR.FAIL_STEPS_ENV, None)

        # ---- a clean run
        print("tab: run", flush=True)
        drive()
        gui_ev = events(open(os.path.join(tab.workdir, 'autoroute.log')).read())
        check("the run exits 0", tab.returncode == 0,
              f"rc={tab.returncode}; {tab.status_text.GetValue()}")
        check("the plan came first and filled the Plan pane",
              tab.plan is not None and 'Design rules' in tab.plan_text.GetValue())
        states = {rw['tag']: rw['state'] for rw in tab.rows}
        check("every checklist row ends terminal",
              all(st in ('done', 'failed', 'skipped') for st in states.values()), states)
        check("the rows' states are the CLI's",
              states == terminal_states(cli_ev), f"tab {states} cli {terminal_states(cli_ev)}")
        check("the rows' states are the tab's own events",
              states == terminal_states(gui_ev), f"{terminal_states(gui_ev)}")
        s = tab.summary or {}
        check("ships the step the CLI ships, at its grade",
              (s.get('final_tag'), s.get('open'), s.get('drc'))
              == (cli_summary.get('final_tag'), cli_summary.get('open'), cli_summary.get('drc')),
              f"tab {s.get('final_tag'), s.get('open'), s.get('drc')} cli "
              f"{cli_summary.get('final_tag'), cli_summary.get('open'), cli_summary.get('drc')}")
        shipped = [i for i, rw in enumerate(tab.rows) if rw['tag'] == s.get('final_tag')]
        check("the shipped row says so",
              shipped and tab.step_list.GetItemText(shipped[0], 2) == 'done, shipped',
              tab.step_list.GetItemText(shipped[0], 2) if shipped else 'no row')
        check("the summary says what Apply brings",
              'Apply makes the open board match it' in tab.summary_text.GetValue(),
              tab.summary_text.GetValue())
        skipped = [rw for rw in tab.rows if rw['state'] == 'skipped']
        check("a skipped row says why",
              all(rw.get('result', '').startswith('skipped: ') and len(rw['result']) > 9
                  for rw in skipped), [rw.get('result') for rw in skipped])
        check("Apply armed", tab.apply_btn.IsEnabled())

        print("tab: apply", flush=True)
        tab._on_apply(None)
        check("Apply asked first, naming the zones and copper",
              asked and 'Zones:' in asked[-1] and 'Tracks and vias:' in asked[-1],
              asked[-1][:300] if asked else 'not asked')
        compare_saved(board, cli_out, os.path.join(tmp, 'gui', 'routed.kicad_pcb'))

        # ---- one step made to fail
        print("tab: a failing step", flush=True)
        drive(fail_steps='planes')
        rows = {rw['tag']: (i, rw) for i, rw in enumerate(tab.rows)}
        i, planes = rows.get('planes', (None, {}))
        check("the failed step's row says FAILED",
              planes.get('state') == 'failed' and i is not None
              and tab.step_list.GetItemText(i, 2) == 'FAILED',
              f"{planes.get('state')}")
        check("its row names the exit code", 'exit code 3' in (planes.get('result') or ''),
              planes.get('result'))
        if i is not None:
            tab.step_list.SetItemState(i, wx.LIST_STATE_SELECTED, wx.LIST_STATE_SELECTED)
            tab._show_row(i)
        detail = tab.detail_text.GetValue()
        check("the detail pane shows its exit code and error",
              'exit code 3' in detail and AR.FAIL_STEP_MESSAGE in detail, detail[:400])
        check("the chain went on: route done", rows.get('route', (0, {}))[1].get('state') == 'done')

        # ---- every step fails
        print("tab: a run that fails outright", flush=True)
        drive(fail_steps='planes,route')
        st = tab.status_text.GetValue()
        check("the status says FAILED with the exit code",
              'FAILED' in st and 'exit code 1' in st, st)
        check("no Apply after a failed run", not tab.apply_btn.IsEnabled())

        # ---- Stop
        print("tab: stop mid-route", flush=True)
        drive(stop_at='route')
        check("Stop ends the run and says so",
              not tab.is_running() and tab.status_text.GetValue().startswith('Stopped'),
              tab.status_text.GetValue())
        check("no Apply after a stopped run", not tab.apply_btn.IsEnabled())
    finally:
        dlg.Destroy()
        try:
            wx.GetApp().ProcessPendingEvents()
        except Exception:                                       # noqa: BLE001
            pass
    print(f"\nwork: {tmp}")
    if FAILS:
        print(f"VERDICT: FAIL ({len(FAILS)}): " + "; ".join(FAILS))
        return 1
    print("VERDICT: the Autorouter tab runs autoroute.py and Apply matches the CLI")
    return 0


def main():
    try:
        import pcbnew  # noqa: F401
        import wx  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    keep = None
    if '--keep' in sys.argv:
        keep = os.path.abspath(sys.argv[sys.argv.index('--keep') + 1])
    if '--apply-check' in sys.argv:
        i = sys.argv.index('--apply-check')
        src, out = (os.path.abspath(a) for a in sys.argv[i + 1:i + 3])
        sys.exit(apply_check(src, out, keep or tempfile.mkdtemp(prefix='krt_apply_')))
    sys.exit(run(keep))


if __name__ == '__main__':
    main()
