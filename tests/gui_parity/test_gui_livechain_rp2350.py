#!/usr/bin/env python3
"""GRADE-parity gate on a set11-class board (rp2350_fpga_eensy).

The copper-identity harness (test_gui_engine_parity.py) proved GUI-vs-CLI on a
route/planes/repair chain but measures copper-overlap %, which on a chaotic
rip-up router diverges even when both fronts grade clean (#362). The invariant
that actually matters is GRADE parity: the GUI must not introduce DRC the CLI
doesn't.

This gate chains the rp2350 PLANE sub-chain (create -> repair -> reconnect
route -> repair2) on ONE live pcbnew board -- exactly as the Claude-tab plan
executor does, in-memory across steps -- starting from the recorded CLI
pre-plane board, and asserts every stage grades 0 DRC like the CLI file chain.

It caught the swig_gui route-apply width-rounding bug (0.0762 -> 0.076 fab-floor
violations, 42 of them at the reconnect route step; #362). Per-step isolation
on CLI inputs did NOT catch it -- only chaining on a live board did, because
the bug rides the GUI's in-memory apply path.

The pre-plane input board (rp2350_fpga_eensy_prePlane.kicad_pcb, the recorded
step4b_retry) is checked into kicad_files/, so the gate is self-contained.
Needs KiCad's python (pcbnew); skips (exit 0) if pcbnew is absent.
Run: python3 tests/gui_parity/test_gui_livechain_rp2350.py
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'tests', 'gui_parity'))
START_BOARD = os.path.join(REPO, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')

KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\\Program Files\\KiCad\\bin\\python.exe"),
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand != sys.executable and os.path.exists(cand):
            if subprocess.run([cand, '-c', 'import pcbnew'],
                              capture_output=True).returncode == 0:
                os.execv(cand, [cand, os.path.abspath(__file__)] + sys.argv[1:])
    print("SKIP: no python with pcbnew found")
    sys.exit(0)


def _grade(pcb, clr=0.09):
    r = subprocess.run(['python3', os.path.join(REPO, 'check_drc.py'), pcb,
                        '--clearance', str(clr), '--hole-to-hole-clearance', '0.2',
                        '--clearance-margin', '0.1'], capture_output=True, text=True)
    m = re.search(r'FOUND (\d+) DRC', r.stdout)
    return 0 if 'NO DRC' in r.stdout else (int(m.group(1)) if m else -1)


def main():
    start_board = START_BOARD
    if not os.path.exists(start_board):
        print(f"SKIP: checked-in board not found at {start_board}")
        return 0

    import pcbnew
    import wx
    from kicad_routing_plugin import planes_gui, swig_gui
    import test_gui_engine_parity as H
    try:
        H._WX_APP = wx.App(False)
    except Exception:
        pass
    wx.MessageBox = lambda *a, **k: None
    from kicad_parser import build_pcb_data_from_board
    from route import batch_route

    work = tempfile.mkdtemp(prefix='rp2350_livechain_')
    src = os.path.join(work, 'live.kicad_pcb')
    shutil.copy(start_board, src)
    board = pcbnew.LoadBoard(src)
    pcbnew.GetBoard = lambda: board

    def planes(cfg):
        tab = H._make_planes_shim(planes_gui, board, src, build_pcb_data_from_board(board))
        (tab._run_create_planes if cfg['mode'] == 'create' else tab._run_repair_planes)(cfg)
        tab._apply_results_to_board()

    def route(nets, clr, tw, vs, vd, gs):
        pd = build_pcb_data_from_board(board)
        c = H._gui_route_config(dict(nets=nets, clearance=clr, track_width=tw,
            via_size=vs, via_drill=vd, grid_step=gs, max_iterations=1000000, max_ripup=10))
        c['layers'] = list(pd.board_info.copper_layers)
        s, f, t, rd = batch_route(
            input_file=src, output_file="", net_names=H._resolve_nets(pd, nets),
            layers=c['layers'], track_width=tw, clearance=clr, via_size=vs, via_drill=vd,
            grid_step=gs, via_cost=c['via_cost'], max_iterations=1000000,
            max_probe_iterations=c['max_probe_iterations'], heuristic_weight=c['heuristic_weight'],
            proximity_heuristic_factor=c['proximity_heuristic_factor'], turn_cost=c['turn_cost'],
            direction_preference_cost=c['direction_preference_cost'], max_rip_up_count=10,
            ripup_abandon_metric=c['ripup_abandon_metric'], ordering_strategy=c['ordering_strategy'],
            direction_order=c['direction'], stub_proximity_radius=c['stub_proximity_radius'],
            stub_proximity_cost=c['stub_proximity_cost'], via_proximity_cost=c['via_proximity_cost'],
            track_proximity_distance=c['track_proximity_distance'],
            track_proximity_cost=c['track_proximity_cost'],
            routing_clearance_margin=c['routing_clearance_margin'], hole_to_hole_clearance=0.2,
            board_edge_clearance=c['board_edge_clearance'],
            enable_layer_switch=c['enable_layer_switch'], return_results=True, pcb_data=pd)
        shim = H._make_route_shim(swig_gui, board, src)
        shim.pcb_data = pd
        swig_gui.RoutingDialog._apply_results_to_board(shim, rd, s, f, t, c)

    def stage(tag):
        p = os.path.join(work, f'{tag}.kicad_pcb')
        pcbnew.SaveBoard(p, board)
        return _grade(p)

    GP = dict(power_nets=['VIN'], power_nets_widths=[0.3], hole_to_hole_clearance=0.2)
    stages = {}
    planes(dict(mode='create', assignments=[(['GND', '+3V3'], ['In1.Cu', 'In4.Cu'])],
                via_size=0.45, via_drill=0.2, clearance=0.10, track_width=0.09,
                grid_step=0.05, add_gnd_vias=False, **GP))
    stages['create'] = stage('create')
    planes(dict(mode='repair', assignments=[(['GND', '+3V3'], ['In1.Cu', 'In4.Cu'])],
                clearance=0.09, via_size=0.25, via_drill=0.15, track_width=0.0762,
                grid_step=0.05, rip_blocker_nets=True, **GP))
    stages['repair'] = stage('repair')
    route(['+1V1', '/T8F49I2X/PIN.5'], 0.09, 0.0762, 0.25, 0.15, 0.025)
    stages['reconnect'] = stage('reconnect')
    planes(dict(mode='repair', assignments=[(['GND', '+3V3'], ['In1.Cu', 'In4.Cu'])],
                clearance=0.09, via_size=0.25, via_drill=0.15, track_width=0.0762,
                grid_step=0.025, rip_blocker_nets=True, **GP))
    stages['final'] = stage('final')

    print("rp2350 live-chain grade parity (GUI, DRC @ 0.09):")
    bad = []
    for tag, n in stages.items():
        flag = 'OK' if n == 0 else 'FAIL'
        print(f"  {tag:<12} DRC={n}  [{flag}]")
        if n != 0:
            bad.append(tag)
    shutil.rmtree(work, ignore_errors=True)
    if bad:
        print(f"\nFAIL: GUI live-chain introduced DRC at stage(s) {bad} that the "
              f"CLI file chain does not (#362).")
        return 1
    print("\nPASS: GUI live-chain grades clean at every stage, matching the CLI.")
    return 0


if __name__ == "__main__":
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(main())
