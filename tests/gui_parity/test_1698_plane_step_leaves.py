#!/usr/bin/env python3
"""What a Planes step leaves for the next step matches the CLI's (#1698).

On the CLI each step is a fresh process that reads the project the last one
wrote. The GUI is one process holding one board and one PCBData, so anything a
step leaves there is read by every later step. A recorded chain's plan differed
from its CLI by 7 segments, first at the route step after its planes, from two
such leftovers:

  edge   `route_planes.py` records the PROJECT's copper-to-edge rule pinned up to
         the fab floor (#338). The Planes tab wrote its own plane-zone INSET
         there, 0.5 on a board declaring no rule, so the later route step held
         copper 0.5 off the edge and refused a pad entry at the 0.2 the CLI
         routes to. The castellated retract read the same inset.
  map    `create_plane` publishes the class map to the fill model, on the
         PCBData it is handed. On the CLI that object dies with the step; on the
         GUI the next step's fill model was stamped at the plane step's classes
         and offered different pour-launch cells.

Real dialog and plan executor (headless_plan.run_plan) against route_planes.py
on the same files, for a project declaring 0, one declaring 0.3 (a rule above
the floor stands), no project at all, and an explicit --board-edge-clearance,
which outranks the rule as the pour's inset on both fronts. Each arm runs in its own process, under
a wall limit, so a stuck wx loop fails rather than hangs.

Needs KiCad python (wx + pcbnew); re-execs into it, and does NOT self-skip
without it (exit 2).

    python3 tests/gui_parity/test_1698_plane_step_leaves.py
"""
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

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
ARM_WALL_S = 180
CLASS_CLEARANCE = 0.3

BOARD = (
    '(kicad_pcb (version 20240108) (generator "test")\n'
    '  (general (thickness 1.6))\n'
    '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
    '  (setup (pad_to_mask_clearance 0))\n'
    '  (net 0 "")\n  (net 1 "GND")\n  (net 2 "HV")\n'
    '  (footprint "t:TH" (layer "F.Cu") (at 18 18)\n'
    '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))\n'
    '    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8)\n'
    '      (layers "*.Cu") (net 1 "GND")))\n'
    '  (footprint "t:TH" (layer "F.Cu") (at 5 5)\n'
    '    (property "Reference" "J2" (at 0 0) (layer "F.SilkS"))\n'
    '    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8)\n'
    '      (layers "*.Cu") (net 2 "HV")))\n'
    '  (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1) (type default))\n'
    '    (fill none) (layer "Edge.Cuts"))\n)\n')

STEP = {'action': 'route_planes',
        'params': {'via_size': 0.6, 'via_drill': 0.3},
        'assignments': [{'nets': ['GND'], 'layer': 'B.Cu'}]}

#: (name, the project's edge rule, an explicit --board-edge-clearance)
ARMS = (('declares 0', 0.0, None), ('declares 0.3', 0.3, None), ('no project', None, None),
        ('declares 0, explicit 1.0', 0.0, 1.0), ('declares 0.3, explicit 1.0', 0.3, 1.0))
BOARD_SIDE = 30.0

FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f" -- {detail}" if detail else ''))
    if not ok:
        FAILS.append(name)


def _stage(d, rule):
    path = os.path.join(d, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    if rule is not None:
        project = {
            'board': {'design_settings': {'rules': {'min_copper_edge_clearance': rule}}},
            'net_settings': {'meta': {'version': 3},
                             'classes': [{'name': 'Default', 'clearance': 0.2},
                                         {'name': 'HV', 'clearance': CLASS_CLEARANCE}],
                             'netclass_assignments': {'HV': 'HV'}},
            'meta': {'version': 1}}
        with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
            json.dump(project, f, indent=2)
    return path


def _zone_inset(xs_ys):
    """How far a pour's outline stands inside the board's 0..BOARD_SIDE square."""
    xs, ys = xs_ys
    return round(min(min(xs), min(ys), BOARD_SIDE - max(xs), BOARD_SIDE - max(ys)), 4)


def _gui_arm(board_path, out_path, explicit=None):
    """Child process: run the step through the real dialog, read what it left."""
    import pcbnew
    for p in (REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_tools')):
        sys.path.insert(0, p)
    import plane_fill_model
    from headless_plan import run_plan
    from kicad_routing_plugin import gui_utils

    seen = {'published': None, 'retract_edge': None, 'at_done': None}
    boards = []
    real_publish = plane_fill_model.set_board_net_clearances

    def publish(pcb_data, net_clearances):
        if net_clearances:
            boards.append(pcb_data)
            seen['published'] = sorted(float(v) for v in net_clearances.values())
        return real_publish(pcb_data, net_clearances)
    plane_fill_model.set_board_net_clearances = publish

    def retract(board, edge):
        seen['retract_edge'] = edge
    gui_utils.apply_castellated_landing_retract = retract

    def on_status(index, status):
        if status == 'done':
            seen['at_done'] = {
                'edge': pcbnew.GetBoard().GetDesignSettings().m_CopperEdgeClearance / 1e6,
                'map': [getattr(b, plane_fill_model._NC_ATTR, None) or {} for b in boards],
                'inset': [_zone_inset((
                    [pcbnew.ToMM(z.Outline().CVertex(i).x) for i in range(z.Outline().TotalVertices())],
                    [pcbnew.ToMM(z.Outline().CVertex(i).y) for i in range(z.Outline().TotalVertices())]))
                          for z in pcbnew.GetBoard().Zones()]}

    step = json.loads(json.dumps(STEP))
    if explicit is not None:
        step['params']['board_edge_clearance'] = explicit
    res = run_plan(board_path, [step], on_status=on_status,
                   quiet=True, timeout=ARM_WALL_S - 30)
    seen['completed'] = res['completed']
    seen['aborted'] = res['aborted']
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(seen, f)
    sys.stdout.flush()
    os._exit(0)     # the arm's verdict is on disk; no wx teardown to wait for


def _gui(board_path, d, explicit=None):
    out = os.path.join(d, 'gui.json')
    r = subprocess.run([sys.executable, os.path.abspath(__file__), '--arm', board_path, out,
                        *([] if explicit is None else [str(explicit)])],
                       capture_output=True, text=True, errors='replace',
                       timeout=ARM_WALL_S, cwd=REPO)
    if not os.path.exists(out):
        print(r.stdout[-1500:] + r.stderr[-1500:])
        return None
    with open(out, encoding='utf-8') as f:
        return json.load(f)


def _cli(board_path, d, explicit=None):
    """(The edge rule route_planes.py records, its pours' inset from the edge.)"""
    out = os.path.join(d, 'cli.kicad_pcb')
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join(REPO, 'py_router', 'route_planes.py'),
         board_path, out, '--nets', 'GND', '--plane-layers', 'B.Cu',
         '--via-size', '0.6', '--via-drill', '0.3',
         *([] if explicit is None else ['--board-edge-clearance', str(explicit)])],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=REPO, timeout=ARM_WALL_S)
    pro = os.path.join(d, 'cli.kicad_pro')
    if r.returncode != 0 or not os.path.exists(pro):
        print(r.stdout[-1500:] + r.stderr[-1500:])
        return None
    with open(out, encoding='utf-8') as f:
        text = f.read()
    outlines = re.findall(r'\(polygon\s*\(pts(.*?)\)\s*\)', text, re.S)
    insets = []
    for o in outlines:
        pts = [(float(a), float(b)) for a, b in re.findall(r'\(xy ([-\d.]+) ([-\d.]+)\)', o)]
        insets.append(_zone_inset(([a for a, _ in pts], [b for _, b in pts])))
    with open(pro, encoding='utf-8') as f:
        rule = json.load(f)['board']['design_settings']['rules']['min_copper_edge_clearance']
    return rule, insets


def main():
    if len(sys.argv) in (4, 5) and sys.argv[1] == '--arm':
        return _gui_arm(sys.argv[2], sys.argv[3],
                        float(sys.argv[4]) if len(sys.argv) == 5 else None)
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        for cand in KICAD_PYTHONS:
            if cand == sys.executable or not os.path.exists(cand):
                continue
            if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                              capture_output=True).returncode == 0:
                sys.exit(subprocess.run([cand, os.path.abspath(__file__)]).returncode)
        print("ERROR: no python with pcbnew + wx found; this gate does not self-skip.")
        sys.exit(2)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    from fix_kicad_drc_settings import effective_board_edge_clearance

    for name, rule, explicit in ARMS:
        print(f'--- {name}')
        d = tempfile.mkdtemp(prefix='plane1698_')
        try:
            board = _stage(d, rule)
            cli_input_floor = effective_board_edge_clearance(board, 0.0)
            got = _cli(board, d, explicit)
            check(f'[{name}] route_planes.py recorded an edge rule', got is not None)
            cli, cli_insets = got or (None, None)
            # After the CLI leg: the GUI leg writes the same board and project.
            gui = _gui(board, d, explicit)
            check(f'[{name}] the plan ran its step',
                  gui is not None and gui['completed'] == 1 and not gui['aborted'],
                  f"{gui and (gui['completed'], gui['aborted'])}")
            if gui is None or cli is None or not gui['at_done']:
                continue
            check(f"[{name}] the tab insets its pour as the CLI does",
                  bool(cli_insets) and gui['at_done']['inset'] == cli_insets,
                  f"live {gui['at_done']['inset']} vs CLI {cli_insets}")
            if explicit is not None:
                check(f'[{name}] the inset is the explicit value',
                      gui['at_done']['inset'] == [explicit], f"{gui['at_done']['inset']}")
            check(f"[{name}] the tab leaves the CLI's copper-to-edge rule on the board",
                  abs(gui['at_done']['edge'] - cli) < 1e-6,
                  f"live {gui['at_done']['edge']} vs project {cli}")
            check(f"[{name}] its castellated retract reads the CLI's floor, not the inset",
                  gui['retract_edge'] is not None
                  and abs(gui['retract_edge'] - cli_input_floor) < 1e-6,
                  f"{gui['retract_edge']} vs {cli_input_floor}")
            if rule is not None:
                check(f'[{name}] the step did publish a class map (the case this arm tests)',
                      gui['published'] and CLASS_CLEARANCE in gui['published'],
                      f"{gui['published']}")
                check(f'[{name}] the step leaves none for the next one',
                      gui['at_done']['map'] and not any(gui['at_done']['map']),
                      f"{gui['at_done']['map']}")
        finally:
            shutil.rmtree(d, ignore_errors=True)

    if FAILS:
        print(f'\nFAILED {len(FAILS)}:')
        for f in FAILS:
            print(' ', f)
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
