#!/usr/bin/env python3
"""Full-chain GUI-carry reproduction for the rp2350 GUI/CLI connectivity gap
(#362 follow-on). Reproduces the REAL GUI plan-executor lifecycle headless: ONE
board + ONE shared pcb_data carried across all 10 plan steps (fanout -> caps ->
fanout -> diff -> signal -> retry -> planes create/repair -> reconnect ->
repair2), synced only where the GUI syncs. After each stage it grades unconnected
(kicad-cli --refill-zones) and compares to the CLI's recorded stepN board,
flagging where the GUI diverges.

Findings (see the issue): steps 0-3 match CLI connectivity; the divergence
accumulates from the signal route on. Root mechanism = pcbnew apply quantizes
segment endpoints (sub-um) vs the CLI text writer, plus per-step param-default
mismatches between the CLI-argparse / GUI-panel / engine-default sources, all
amplified by the chaotic rip-up router + plane repair. SYNC_PLANES=1 syncs
pcb_data before plane steps too (the missing half of the #362 sync) -> harness
reaches CLI parity (6==6); default (faithful) leaves 7.

Requires KiCad python (pcbnew) + the set11 stress corpus. Skips if absent.
Run: <kicad-python> tests/gui_parity/diag_fullchain_carry_rp2350.py
"""
import os, subprocess, sys
_KPY=["/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3"]
def _reexec():
    for c in _KPY:
        if c!=sys.executable and os.path.exists(c) and subprocess.run([c,'-c','import pcbnew'],capture_output=True).returncode==0:
            os.execv(c,[c,os.path.abspath(__file__)]+sys.argv[1:])
    print("SKIP: no python with pcbnew"); sys.exit(0)
try:
    import pcbnew  # noqa
except ImportError:
    _reexec()

import os, sys, shutil, tempfile, subprocess, json, types
REPO='/Users/andy/Documents/KiCadRoutingTools'
sys.path.insert(0, REPO); sys.path.insert(0, os.path.join(REPO,'kicad_routing_plugin'))
sys.path.insert(0, os.path.join(REPO,'tests','gui_parity'))
import pcbnew, wx
from kicad_parser import build_pcb_data_from_board
from kicad_routing_plugin import swig_gui, fanout_gui, planes_gui, differential_gui
import test_gui_engine_parity as H
import routing_defaults as defaults

STRESS=os.environ.get('STRESS_DIR', os.path.expanduser('~/Documents/kicad_stress_test'))
UNROUTED=os.path.join(STRESS,'boards_unrouted_set11','rp2350_fpga_eensy.kicad_pcb')
RUN=os.path.join(STRESS,'runs_set11','rp2350_fpga_eensy')
KCLI='/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
LAYERS=['F.Cu','In1.Cu','In2.Cu','In3.Cu','In4.Cu','B.Cu']

if not (os.path.exists(UNROUTED) and os.path.isdir(RUN) and os.path.exists(KCLI)):
    print("SKIP: needs the set11 stress corpus (boards_unrouted_set11 + runs_set11) "
          "and kicad-cli; set STRESS_DIR if it lives elsewhere.")
    sys.exit(0)

try: wx.App(False)
except Exception: pass
wx.MessageBox=lambda *a,**k: None
wx.Yield=lambda *a,**k: None

work=tempfile.mkdtemp(prefix='fullchain_')
src=os.path.join(work,'live.kicad_pcb'); shutil.copy(UNROUTED,src)
board=pcbnew.LoadBoard(src)
pcbnew.GetBoard=lambda: board
PD=build_pcb_data_from_board(board)   # dialog-open pcb_data (SHARED, carried)

def geomdiff(tag, cli_board):
    """Copper-set geometry diff (net,layer,endpoints,width) GUI vs CLI stepN."""
    from kicad_parser import parse_kicad_pcb
    p=os.path.join(work,f'{tag}.kicad_pcb')
    cb=os.path.join(RUN,cli_board)
    if not os.path.exists(cb): return
    def canon(path):
        pcb=parse_kicad_pcb(path); idn={n.net_id:n.name for n in pcb.nets.values()}
        segs=set(); vias=set()
        for s in pcb.segments:
            a=(round(s.start_x,3),round(s.start_y,3)); b=(round(s.end_x,3),round(s.end_y,3))
            segs.add((idn.get(s.net_id,s.net_id),s.layer,frozenset((a,b)),round(s.width,4)))
        for v in pcb.vias:
            vias.add((idn.get(v.net_id,v.net_id),round(v.x,3),round(v.y,3),round(v.size,4)))
        return segs,vias
    gs,gv=canon(p); cs,cv=canon(cb)
    print(f"       geom: segs GUI-only={len(gs-cs)} CLI-only={len(cs-gs)} common={len(gs&cs)} | "
          f"vias GUI-only={len(gv-cv)} CLI-only={len(cv-gv)}")

def grade(tag, cli_board):
    p=os.path.join(work,f'{tag}.kicad_pcb'); pcbnew.SaveBoard(p,board)
    o=p+'.drc.json'
    subprocess.run([KCLI,'pcb','drc',p,'--format','json','-o',o,'--severity-error','--refill-zones'],capture_output=True)
    g=len(json.load(open(o)).get('unconnected_items',[])) if os.path.exists(o) else -1
    c=-1
    cb=os.path.join(RUN,cli_board)
    if os.path.exists(cb):
        oc=os.path.join(work,tag+'.cli.json')
        subprocess.run([KCLI,'pcb','drc',cb,'--format','json','-o',oc,'--severity-error','--refill-zones'],capture_output=True)
        c=len(json.load(open(oc)).get('unconnected_items',[])) if os.path.exists(oc) else -1
    flag='' if g==c else '   <<< DIVERGES'
    print(f"  {tag:16s} GUI_unconn={g:3d}  CLI({cli_board})={c:3d}{flag}")
    return g,c

# ---- shims sharing board + PD ----
def route_shim():
    d=H._make_route_shim(swig_gui, board, src); d.pcb_data=PD; return d
def diff_shim():
    d=H._Shim(); d.board=board; d.board_filename=src; d.pcb_data=PD
    for n in ('status_text','progress_bar','route_btn'): setattr(d,n,H._Stub())
    d.sync_pcb_data_callback=None; d._diff_drc_config=None
    H._borrow(d, differential_gui.DifferentialTab,
              [x for x in ('_apply_results_to_board','_add_via_to_board') if hasattr(differential_gui.DifferentialTab,x)])
    H._borrow(d, swig_gui.RoutingDialog, [x for x in ('_add_via_to_board',) if not hasattr(d,'_add_via_to_board')])
    return d
def fanout_shim():
    d=H._Shim(); d.board=board; d.board_filename=src; d.pcb_data=PD
    for n in ('status_text','progress_bar','fanout_btn'): setattr(d,n,H._Stub())
    d.on_fanout_complete=None
    H._borrow(d, fanout_gui.FanoutTab, ['_apply_fanout_results'])
    return d

def sync():   # what _on_route/_on_diff do first (my #362 fix)
    rs=route_shim(); rs._sync_pcb_data_from_board()

# ================= CHAIN =================
print("Full-chain GUI-carry reproduction (unconnected @ kicad-cli refill):")

# step0: BGA fanout U2 (uses carried PD == fresh unrouted here)
from fab_tiers import set_fab_tier_from_config
set_fab_tier_from_config({'fab_tier':'advanced'})
from bga_fanout import generate_bga_fanout
fpU2=PD.footprints['U2']
tr,va,vr,fn=generate_bga_fanout(fpU2, PD, net_filter=['*','!GND','!+3V3'],
    diff_pair_patterns=None, layers=LAYERS, track_width=0.0762, clearance=0.09,
    diff_pair_gap=defaults.DIFF_PAIR_GAP, exit_margin=defaults.BGA_EXIT_MARGIN,
    primary_escape='horizontal', force_escape_direction=False, rebalance_escape=False,
    via_size=0.25, via_drill=0.15, check_for_previous=False, no_inner_top_layer=False,
    escape_method='auto', grid_step=0.05, layer_costs=None)
fanout_shim()._apply_fanout_results(tr,va,failed_nets=fn)
grade('step0_bga_u2','step1_u2_fanout.kicad_pcb'); geomdiff('step0_bga_u2','step1_u2_fanout.kicad_pcb')

# step1: optimize_caps (build fresh from board, apply moves) -- shared engine
from placement.fanout_clearance import repair_fanout_clearance
pcf=build_pcb_data_from_board(board)
res=repair_fanout_clearance(pcf, pcb_file=src, clearance=0.09, grid_step=0.05, default_via_size=0.25)
for p in res.get('placements',[]):
    fp=board.FindFootprintByReference(p['reference'])
    if fp: fp.SetOrientationDegrees(p['new_rotation']); fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(p['new_x']),pcbnew.FromMM(p['new_y'])))
board.BuildConnectivity()
grade('step1_caps','step1b_capclear.kicad_pcb')

# step2: QFN fanout U6 (uses carried PD -- STALE: no U2 fanout, orig caps!)
from qfn_fanout import generate_qfn_fanout
fpU6=PD.footprints['U6']
tr,va,fn=generate_qfn_fanout(fpU6, PD, net_filter=['*','!GND','!+3V3'],
    layer=getattr(fpU6,'layer','F.Cu'), track_width=0.0762, extension=defaults.QFN_EXTENSION,
    clearance=0.09, grid_step=0.05, escape_method='stub', via_size=0.25, via_drill=0.15,
    allow_via_in_pad=False, board_edge_clearance=0.0)
fanout_shim()._apply_fanout_results(tr,va,failed_nets=fn,fanout_kind='qfn')
grade('step2_qfn_u6','step2_u6_fanout.kicad_pcb'); geomdiff('step2_qfn_u6','step2_u6_fanout.kicad_pcb')

# step3: route_diff (sync first, then route with PD)
sync()
from route_diff import batch_route_diff_pairs
dnets=['/Power Supply and USB/D+','/Power Supply and USB/D-','/RP2354A/USB_D+','/RP2354A/USB_D-']
s,f,t,rd=batch_route_diff_pairs(input_file=src, output_file="", net_names=dnets, layers=LAYERS,
    track_width=0.1, clearance=0.10, via_size=0.45, via_drill=0.2, grid_step=0.05,
    diff_pair_gap=0.1, gnd_via_enabled=False, return_results=True, pcb_data=PD)
diff_shim()._apply_results_to_board(rd)
board.BuildConnectivity()
grade('step3_diff','step3_diffpairs.kicad_pcb'); geomdiff('step3_diff','step3_diffpairs.kicad_pcb')

# step4: route signal (sync first)
sync()
from route import batch_route
cfg=H._gui_route_config({'grid_step':0.05,'max_ripup':10,'max_iterations':1000000,'power_nets':['VIN'],'power_nets_widths':[0.3]})
nets=H._resolve_nets(PD, ['*','!GND','!+3V3'])
s,f,t,rd=batch_route(input_file=src, output_file="", net_names=nets, layers=LAYERS,
    track_width=0.09, clearance=0.10, via_size=0.45, via_drill=0.2, grid_step=0.05,
    hole_to_hole_clearance=0.2, max_rip_up_count=10, max_iterations=1000000,
    power_nets=['VIN'], power_nets_widths=[0.3], return_results=True, pcb_data=PD,
    **{k:cfg[k] for k in ('via_cost','max_probe_iterations','heuristic_weight','proximity_heuristic_factor','turn_cost','direction_preference_cost','ripup_abandon_metric','ordering_strategy','stub_proximity_radius','stub_proximity_cost','via_proximity_cost','track_proximity_distance','track_proximity_cost','routing_clearance_margin','board_edge_clearance','enable_layer_switch')})
rs=route_shim(); rs.pcb_data=PD
swig_gui.RoutingDialog._apply_results_to_board(rs, rd, s, f, t, cfg)
print(f"    [signal route: routed={s} failed={f}]")
grade('step4_signal','step4_signal.kicad_pcb')

def route_step(nets_glob, tw, clr, vs, vd, gs, powr=None):
    sync()
    cf=H._gui_route_config({'grid_step':gs,'max_ripup':10,'max_iterations':1000000,
                            'power_nets':powr,'power_nets_widths':[0.3] if powr else None})
    nn=H._resolve_nets(PD, nets_glob)
    S,F,T,RD=batch_route(input_file=src, output_file="", net_names=nn, layers=LAYERS,
        track_width=tw, clearance=clr, via_size=vs, via_drill=vd, grid_step=gs,
        hole_to_hole_clearance=0.2, max_rip_up_count=10, max_iterations=1000000,
        power_nets=powr, power_nets_widths=[0.3] if powr else None, return_results=True, pcb_data=PD,
        **{k:cf[k] for k in ('via_cost','max_probe_iterations','heuristic_weight','proximity_heuristic_factor','turn_cost','direction_preference_cost','ripup_abandon_metric','ordering_strategy','stub_proximity_radius','stub_proximity_cost','via_proximity_cost','track_proximity_distance','track_proximity_cost','routing_clearance_margin','board_edge_clearance','enable_layer_switch')})
    r=route_shim(); r.pcb_data=PD
    swig_gui.RoutingDialog._apply_results_to_board(r, RD, S, F, T, cf)
    return S,F

SYNC_BEFORE_PLANES=(os.environ.get('SYNC_PLANES','0')=='1')
def planes_step(mode, nets, plane_layers, tw, clr, vs, vd, gs, rip=False, powr=None):
    if SYNC_BEFORE_PLANES: sync()   # candidate fix: give planes a board-fresh PD
    # faithful default: real GUI planes _on_action does NOT sync PD first (#362 fix
    # only covered route/diff) -> planes run on stale carried PD
    tab=H._make_planes_shim(planes_gui, board, src, PD)
    if mode=='create':
        cfg={'mode':'create','assignments':[(list(nets),list(plane_layers))],
             'via_size':vs,'via_drill':vd,'clearance':clr,'track_width':tw,'grid_step':gs,
             'add_gnd_vias':False,'power_nets':powr,'power_nets_widths':[0.3] if powr else None}
        tab._run_create_planes(cfg)
    else:
        cfg={'mode':'repair','assignments':[(list(nets),list(plane_layers or ['In1.Cu','In4.Cu']))],
             'clearance':clr,'via_size':vs,'via_drill':vd,'track_width':tw,'grid_step':gs,
             'rip_blocker_nets':rip,'power_nets':powr,'power_nets_widths':[0.3] if powr else None}
        tab._run_repair_planes(cfg)
    tab._apply_results_to_board()
    board.BuildConnectivity()

# step5: route retry /T8F49I2X/R
S,F=route_step(['/T8F49I2X/R'], 0.0762, 0.09, 0.25, 0.15, 0.025)
print(f"    [retry route: routed={S} failed={F}]")
grade('step5_retry','step4b_retry.kicad_pcb')
# step6: create planes
planes_step('create', ['GND','+3V3'], ['In1.Cu','In4.Cu'], 0.09, 0.10, 0.45, 0.2, 0.05, powr=['VIN'])
grade('step6_planes','step5_planes.kicad_pcb')
# step7: repair planes
planes_step('repair', ['GND','+3V3'], ['In1.Cu','In4.Cu'], 0.0762, 0.09, 0.25, 0.15, 0.05, rip=True, powr=['VIN'])
grade('step7_repair','step5_repair.kicad_pcb')
# step8: reconnect route
S,F=route_step(['+1V1','/T8F49I2X/PIN.5'], 0.0762, 0.09, 0.25, 0.15, 0.025)
print(f"    [reconnect route: routed={S} failed={F}]")
grade('step8_reconnect','step5c_reconnect.kicad_pcb')
# step9: repair2
planes_step('repair', ['GND','+3V3'], ['In1.Cu','In4.Cu'], 0.0762, 0.09, 0.25, 0.15, 0.025, rip=True, powr=['VIN'])
grade('step9_repair2','step5d_repair2.kicad_pcb')

print(f"\nwork dir: {work}")
