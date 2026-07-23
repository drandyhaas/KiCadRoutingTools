#!/usr/bin/env python3
"""Headless GUI-vs-CLI engine parity harness (claude-tab plan execution).

The claude tab's promise: "run selected steps" in the GUI should leave the
live board in the same state as running the stress chain's CLI steps on the
board FILE and loading the final output. This harness measures that promise
on a real board without pressing buttons:

  CLI leg   -- runs the recorded chain's commands file->file (subprocesses),
               exactly like a stress replay.
  GUI leg   -- inside KiCad's bundled python: loads the board with pcbnew,
               builds PCBData via build_pcb_data_from_board, invokes the SAME
               engine calls the tabs make (batch_route with the swig_gui
               kwargs and pcb_data=..., create_plane / repair via the planes
               tab's own worker + _apply_results_to_board on a shimmed tab
               instance), rebuilding PCBData from the live board between
               steps, then saves the board.

Both finals are then graded (check_connected / check_drc / kicad-cli DRC
unconnected) and the divergence is printed. This harness REPORTS the gap; it
is not (yet) a pass/fail gate -- known deliberate divergences exist (the
CLI mains' kicad-oracle recheck, clean_plane_copper, end-of-run
reconciliation, .kicad_pro floor carryover, and plan-parameter whitelist).

Needs pcbnew; re-execs into KiCad's python automatically.

    python3 tests/test_gui_engine_parity.py [board.kicad_pcb] [--workdir DIR]
"""
import os
import shutil
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)


_WX_APP = None

KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\\Program Files\\KiCad\\bin\\python.exe"),
]

DEFAULT_BOARD = os.path.join(REPO, "kicad_files", "splitflap_driver.kicad_pcb")

# The splitflap chain, expressed twice:
#  - CLI: the recorded stress commands (what a stress replay runs);
#  - GUI: what the claude plan CAN set on the tabs (track/clearance/via/
#    power via apply_step_params) -- everything else stays at the GUI
#    panels' defaults, exactly as a real "run selected steps" would.
SIGNAL = dict(nets=['*', '!GND'], clearance=0.15, track_width=0.127,
              via_size=0.45, via_drill=0.2,
              power_nets=['+3V3', '+12V'], power_nets_widths=[0.4, 0.4],
              max_iterations=1000000, max_ripup=10)
GND_FIX = dict(nets=['GND'], clearance=0.127, track_width=0.127,
               via_size=0.45, via_drill=0.2,
               max_iterations=1000000, max_ripup=10)
PLANES = dict(nets=['GND'], layers=['B.Cu'], clearance=0.15,
              via_size=0.45, via_drill=0.2)
REPAIR = dict(clearance=0.15, via_size=0.45, via_drill=0.2,
              track_width=0.127, grid_step=0.1,
              power_nets=['+3V3', '+12V'], power_nets_widths=[0.4, 0.4])


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable:
            continue
        if os.path.exists(cand):
            r = subprocess.run([cand, '-c', 'import pcbnew'],
                               capture_output=True)
            if r.returncode == 0:
                os.execv(cand, [cand, os.path.abspath(__file__)] + sys.argv[1:])
    print("ERROR: no python with pcbnew found")
    sys.exit(2)


def run_cli_leg(board, workdir):
    py = shutil.which('python3') or sys.executable
    steps = []
    s1 = os.path.join(workdir, 'cli_step1.kicad_pcb')
    steps.append([py, '-X', 'utf8', os.path.join(REPO, 'route.py'), board, s1,
                  '--nets', '*', '!GND', '--clearance', '0.15',
                  '--track-width', '0.127', '--via-size', '0.45',
                  '--via-drill', '0.2', '--power-nets', '+3V3', '+12V',
                  '--power-nets-widths', '0.4', '0.4',
                  '--max-ripup', '10', '--max-iterations', '1000000'])
    s2 = os.path.join(workdir, 'cli_step2.kicad_pcb')
    steps.append([py, '-X', 'utf8', os.path.join(REPO, 'route_planes.py'), s1, s2,
                  '--nets', 'GND', '--plane-layers', 'B.Cu',
                  '--clearance', '0.15', '--via-size', '0.45',
                  '--via-drill', '0.2'])
    s3 = os.path.join(workdir, 'cli_step3.kicad_pcb')
    steps.append([py, '-X', 'utf8',
                  os.path.join(REPO, 'route_disconnected_planes.py'), s2, s3,
                  '--clearance', '0.15', '--via-size', '0.45',
                  '--via-drill', '0.2', '--track-width', '0.127',
                  '--grid-step', '0.1', '--power-nets', '+3V3', '+12V',
                  '--power-nets-widths', '0.4', '0.4', '--rip-blocker-nets'])
    s4 = os.path.join(workdir, 'cli_final.kicad_pcb')
    steps.append([py, '-X', 'utf8', os.path.join(REPO, 'route.py'), s3, s4,
                  '--nets', 'GND', '--clearance', '0.127',
                  '--track-width', '0.127', '--via-size', '0.45',
                  '--via-drill', '0.2', '--max-ripup', '10',
                  '--max-iterations', '1000000'])
    for i, cmd in enumerate(steps):
        print(f"[cli] step {i + 1}/4 ...", flush=True)
        r = subprocess.run(cmd, capture_output=True, text=True, cwd=workdir)
        if r.returncode != 0:
            print(r.stdout[-3000:])
            print(r.stderr[-2000:])
            raise RuntimeError(f"CLI step {i + 1} failed")
    return s4


class _Stub:
    """Absorbs any UI attribute/method access with no-ops. Numeric/iteration
    protocols return empty/zero so code like range(panel.GetCount()) or
    iterating a list control degrades to a no-op."""
    def __getattr__(self, name):
        return _Stub()

    def __call__(self, *a, **k):
        return _Stub()

    def __bool__(self):
        return False

    def __index__(self):
        return 0

    def __int__(self):
        return 0

    def __len__(self):
        return 0

    def __iter__(self):
        return iter(())


class _Shim:
    pass


def _borrow(shim, cls, names):
    """Bind the REAL methods of the wx dialog class onto a plain shim (py3
    functions have no isinstance check), so the harness runs the exact
    board-apply code the buttons run -- without constructing a wx object."""
    import types
    for n in names:
        setattr(shim, n, types.MethodType(getattr(cls, n), shim))


def _make_route_shim(swig_gui, board, board_path):
    d = _Shim()
    d.board = board
    d.board_filename = board_path
    d.pcb_data = None
    for name in ('net_panel', 'status_text', 'progress_bar', 'route_btn'):
        setattr(d, name, _Stub())
    _borrow(d, swig_gui.RoutingDialog,
            [n for n in ('_add_via_to_board', '_sync_pcb_data_from_board',
                         '_clear_user_layer_graphics',
                         '_move_copper_text_to_silkscreen')
             if hasattr(swig_gui.RoutingDialog, n)])
    d._update_status_bar = lambda *a, **k: None
    d._check_connectivity_with_progress = lambda *a, **k: None
    d._add_debug_lines = lambda *a, **k: None
    return d


def _gui_route_config(step):
    """The swig_gui _on_route config, at GUI-panel defaults except the
    fields a plan step can set. Every default sourced from routing_defaults
    exactly as the real panels initialize -- hardcoded copies here already
    caused two phantom divergences (ordering 'inside_out', track proximity
    cost 0.2)."""
    import routing_defaults as defaults
    return {
        'layers': None,  # filled by caller from the board
        'grid_step': step.get('grid_step', defaults.GRID_STEP),
        'via_cost': defaults.VIA_COST,
        'impedance': None,
        'max_iterations': step.get('max_iterations',
                                    defaults.MAX_ITERATIONS),
        'max_probe_iterations': defaults.MAX_PROBE_ITERATIONS,
        'heuristic_weight': defaults.HEURISTIC_WEIGHT,
        'proximity_heuristic_factor': defaults.PROXIMITY_HEURISTIC_FACTOR,
        'turn_cost': defaults.TURN_COST,
        'direction_preference_cost': defaults.DIRECTION_PREFERENCE_COST,
        'max_ripup': step.get('max_ripup', defaults.MAX_RIPUP),
        'ripup_abandon_metric': step.get('ripup_abandon_metric',
                                          defaults.RIPUP_ABANDON_METRIC),
        'ripup_blocker_select': step.get('ripup_blocker_select', 'count'),
        'ordering_strategy': step.get('ordering_strategy',
                                      defaults.DEFAULT_ORDERING_STRATEGY),
        'direction': None,
        'stub_proximity_radius': defaults.STUB_PROXIMITY_RADIUS,
        'stub_proximity_cost': defaults.STUB_PROXIMITY_COST,
        'via_proximity_cost': defaults.VIA_PROXIMITY_COST,
        'track_proximity_distance': defaults.TRACK_PROXIMITY_DISTANCE,
        'track_proximity_cost': defaults.TRACK_PROXIMITY_COST,
        'routing_clearance_margin': defaults.ROUTING_CLEARANCE_MARGIN,
        'hole_to_hole_clearance': defaults.HOLE_TO_HOLE_CLEARANCE,
        'board_edge_clearance': defaults.BOARD_EDGE_CLEARANCE,
        'enable_layer_switch': True,
        'keep_input_copper': step.get('keep_input_copper', False),
        'debug_lines': False,
        'verbose': False,
        'power_nets': step.get('power_nets'),
        'power_nets_widths': step.get('power_nets_widths'),
    }


def _resolve_nets(pcb_data, globs):
    """Resolve the plan step's net globs to concrete names, the same way
    apply_step_selection does before pressing the button."""
    from kicad_routing_plugin.claude_plan import _match_net_names
    return _match_net_names(pcb_data, globs)


GUI_INPUT_MODE = os.environ.get('GUI_PARITY_INPUT', 'builder')


def _gui_pcb_data(board, board_path):
    """builder = the real GUI path (build_pcb_data_from_board).
    parser = DIAGNOSTIC: temp-save the board and text-parse it, isolating
    the input-representation fork from everything else."""
    from kicad_parser import build_pcb_data_from_board, parse_kicad_pcb
    if GUI_INPUT_MODE == 'parser':
        import pcbnew
        import tempfile
        with tempfile.NamedTemporaryFile(suffix='.kicad_pcb',
                                         delete=False) as f:
            tmp = f.name
        pcbnew.SaveBoard(tmp, board)
        data = parse_kicad_pcb(tmp)
        os.unlink(tmp)
        return data
    return build_pcb_data_from_board(board)


def _gui_route_step(swig_gui, board, board_path, step):
    from route import batch_route
    pcb_data = _gui_pcb_data(board, board_path)
    config = _gui_route_config(step)
    config['layers'] = list(pcb_data.board_info.copper_layers)
    step = dict(step, nets=_resolve_nets(pcb_data, step['nets']))
    successful, failed, total_time, results_data = batch_route(
        input_file=board_path,
        output_file="",
        net_names=step['nets'],
        layers=config['layers'],
        track_width=step['track_width'],
        clearance=step['clearance'],
        via_size=step['via_size'],
        via_drill=step['via_drill'],
        grid_step=config['grid_step'],
        via_cost=config['via_cost'],
        max_iterations=config['max_iterations'],
        max_probe_iterations=config['max_probe_iterations'],
        heuristic_weight=config['heuristic_weight'],
        proximity_heuristic_factor=config['proximity_heuristic_factor'],
        turn_cost=config['turn_cost'],
        direction_preference_cost=config['direction_preference_cost'],
        max_rip_up_count=config['max_ripup'],
        ripup_abandon_metric=config['ripup_abandon_metric'],
        ripup_blocker_select=config['ripup_blocker_select'],
        ordering_strategy=config['ordering_strategy'],
        direction_order=config['direction'],
        stub_proximity_radius=config['stub_proximity_radius'],
        stub_proximity_cost=config['stub_proximity_cost'],
        via_proximity_cost=config['via_proximity_cost'],
        track_proximity_distance=config['track_proximity_distance'],
        track_proximity_cost=config['track_proximity_cost'],
        routing_clearance_margin=config['routing_clearance_margin'],
        hole_to_hole_clearance=config['hole_to_hole_clearance'],
        board_edge_clearance=config['board_edge_clearance'],
        enable_layer_switch=config['enable_layer_switch'],
        keep_input_copper=config['keep_input_copper'],
        power_nets=config['power_nets'],
        power_nets_widths=config['power_nets_widths'],
        return_results=True,
        pcb_data=pcb_data,
    )
    shim = _make_route_shim(swig_gui, board, board_path)
    shim.pcb_data = pcb_data
    swig_gui.RoutingDialog._apply_results_to_board(
        shim, results_data, successful, failed, total_time, config)
    return successful, failed


def _make_planes_shim(planes_gui, board, board_path, pcb_data):
    tab = _Shim()
    tab.board = board
    tab.board_filename = board_path
    tab.pcb_data = pcb_data
    for name in ('status_text', 'progress_bar', 'action_btn', 'cancel_btn',
                 'options_panel', 'repair_options', 'assignments_panel'):
        setattr(tab, name, _Stub())
    tab._cancel_requested = False
    tab.sync_pcb_data_callback = None
    cls = planes_gui.PlanesTab
    # Borrow every underscore method the worker bodies might call -- the
    # shim only stubs the UI widgets, the logic is all real.
    import inspect
    _borrow(tab, cls,
            [n for n, f in vars(cls).items()
             if callable(f) and not isinstance(f, (staticmethod, classmethod))
             and n not in ('__init__', '_create_ui')])
    return tab


def _gui_planes_step(planes_gui, board, board_path, step, mode):
    pcb_data = _gui_pcb_data(board, board_path)
    tab = _make_planes_shim(planes_gui, board, board_path, pcb_data)
    if mode == 'create':
        config = {
            'mode': 'create',
            'assignments': [(list(step['nets']), list(step['layers']))],
            'via_size': step['via_size'], 'via_drill': step['via_drill'],
            'clearance': step['clearance'],
            'add_gnd_vias': False,
        }
        tab._run_create_planes(config)
    else:
        config = {
            'mode': 'repair',
            'assignments': [(['GND'], ['B.Cu'])],
            'clearance': step['clearance'], 'via_size': step['via_size'],
            'via_drill': step['via_drill'], 'track_width': step['track_width'],
            'grid_step': step.get('grid_step', 0.1),
            'power_nets': step.get('power_nets'),
            'power_nets_widths': step.get('power_nets_widths'),
            'rip_blocker_nets': True,
        }
        tab._run_repair_planes(config)
    tab._apply_results_to_board()


def run_gui_leg(board_path, workdir):
    import pcbnew
    import wx
    from kicad_routing_plugin import swig_gui
    from kicad_routing_plugin import planes_gui
    gui_src = os.path.join(workdir, 'gui_input.kicad_pcb')
    shutil.copy(board_path, gui_src)
    board = pcbnew.LoadBoard(gui_src)
    # Standalone harness stand-ins for the live-GUI environment: the apply
    # methods fetch the open board via pcbnew.GetBoard() (None when
    # headless), and error paths raise wx.MessageBox (needs an App).
    global _WX_APP
    try:
        _WX_APP = wx.App(False)
    except Exception:
        _WX_APP = None
    wx.MessageBox = lambda *a, **k: None  # headless: no popups
    pcbnew.GetBoard = lambda: board

    print("[gui] step 1/4 route signals ...", flush=True)
    _gui_route_step(swig_gui, board, gui_src, SIGNAL)
    print("[gui] step 2/4 create planes ...", flush=True)
    _gui_planes_step(planes_gui, board, gui_src, PLANES, 'create')
    print("[gui] step 3/4 repair planes ...", flush=True)
    _gui_planes_step(planes_gui, board, gui_src, REPAIR, 'repair')
    print("[gui] step 4/4 route GND fix ...", flush=True)
    _gui_route_step(swig_gui, board, gui_src, GND_FIX)

    out = os.path.join(workdir, 'gui_final.kicad_pcb')
    pcbnew.SaveBoard(out, board)
    return out


def grade(pcb, label):
    py = shutil.which('python3') or sys.executable
    conn = subprocess.run([py, '-X', 'utf8',
                           os.path.join(REPO, 'check_connected.py'), pcb],
                          capture_output=True, text=True)
    conn_full = 'ALL NETS FULLY CONNECTED' in conn.stdout
    import re
    m = re.search(r'FOUND (\d+) ISSUES', conn.stdout)
    conn_issues = int(m.group(1)) if m else 0
    drc = subprocess.run([py, '-X', 'utf8', os.path.join(REPO, 'check_drc.py'),
                          pcb, '--clearance-margin', '0.1', '-c', '0.127'],
                         capture_output=True, text=True)
    m = re.search(r'FOUND (\d+) DRC', drc.stdout)
    drc_n = 0 if 'NO DRC' in drc.stdout else (int(m.group(1)) if m else -1)
    kicad_n = -1
    for cand in KICAD_PYTHONS[:1]:
        cli = cand.replace(
            'Frameworks/Python.framework/Versions/Current/bin/python3',
            'MacOS/kicad-cli')
        if os.path.exists(cli):
            out = pcb + '.drc.json'
            subprocess.run([cli, 'pcb', 'drc', pcb, '--format', 'json', '-o',
                            out, '--severity-all', '--refill-zones'],
                           capture_output=True)
            try:
                import json
                kicad_n = len(json.load(open(out)).get('unconnected_items', []))
            except Exception:
                pass
    print(f"  {label:10s} conn_full={conn_full} conn_issues={conn_issues} "
          f"drc={drc_n} kicad_unconnected={kicad_n}")
    return dict(conn_full=conn_full, conn_issues=conn_issues, drc=drc_n,
                kicad=kicad_n)


def compare_copper(cli_pcb, gui_pcb):
    """Canonical copper-set comparison: segments as (net NAME, layer,
    sorted rounded endpoints, width), vias as (net, pos, size, drill).
    UUIDs are per-run random, so byte comparison is meaningless by design;
    set equality is the strongest meaningful identity bar."""
    from kicad_parser import parse_kicad_pcb

    def canon(path):
        pcb = parse_kicad_pcb(path)
        names = {nid: net.name for nid, net in pcb.nets.items()}
        segs = set()
        for s in pcb.segments:
            a = (round(s.start_x, 3), round(s.start_y, 3))
            b = (round(s.end_x, 3), round(s.end_y, 3))
            segs.add((names.get(s.net_id, s.net_id), s.layer,
                      min(a, b), max(a, b), round(s.width, 3)))
        vias = set()
        for v in pcb.vias:
            vias.add((names.get(v.net_id, v.net_id), round(v.x, 3),
                      round(v.y, 3), round(v.size, 3), round(v.drill, 3)))
        return segs, vias

    s1, v1 = canon(cli_pcb)
    s2, v2 = canon(gui_pcb)
    print(f"\n=== COPPER-SET COMPARISON (UUID-independent) ===")
    print(f"  segments: CLI={len(s1)} GUI={len(s2)} common={len(s1 & s2)} "
          f"cli-only={len(s1 - s2)} gui-only={len(s2 - s1)}")
    print(f"  vias:     CLI={len(v1)} GUI={len(v2)} common={len(v1 & v2)} "
          f"cli-only={len(v1 - v2)} gui-only={len(v2 - v1)}")
    for tag, diff in (('cli-only seg', sorted(s1 - s2)[:3]),
                      ('gui-only seg', sorted(s2 - s1)[:3])):
        for d in diff:
            print(f"    {tag}: {d}")
    identical = (s1 == s2 and v1 == v2)
    print(f"  copper sets identical: {identical}")
    return identical


def main():
    board = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('--') \
        else DEFAULT_BOARD
    workdir = None
    if '--workdir' in sys.argv:
        workdir = sys.argv[sys.argv.index('--workdir') + 1]
    if workdir is None:
        workdir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'work')
    workdir = os.path.abspath(workdir)
    os.makedirs(workdir, exist_ok=True)

    cli_final = run_cli_leg(board, workdir)
    gui_final = run_gui_leg(board, workdir)

    print("\n=== PARITY REPORT ===")
    a = grade(cli_final, 'CLI')
    b = grade(gui_final, 'GUI')
    same = (a['conn_full'] == b['conn_full'] and a['kicad'] == b['kicad']
            and a['drc'] == b['drc'])
    identical = compare_copper(cli_final, gui_final)
    print(f"\nVERDICT: {'PARITY' if same else 'DIVERGENT'} "
          f"(kicad unconnected CLI={a['kicad']} GUI={b['kicad']}, "
          f"drc CLI={a['drc']} GUI={b['drc']}); "
          f"copper sets {'IDENTICAL' if identical else 'DIFFER'}")
    return 0


if __name__ == '__main__':
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(main())
