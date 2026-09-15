"""Independent issue966 native dialog matrix; writes evidence, never source fixtures."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

repo = Path(sys.argv[1]).resolve()
dest = Path(sys.argv[2]).resolve()
dest.mkdir(parents=True, exist_ok=False)
sys.path[:0] = [str(repo), str(repo/'py_router'), str(repo/'py_tools'), str(repo/'py_placer')]
os.environ['WXSUPPRESS_SIZER_FLAGS_CHECK'] = '1'
import pcbnew
import wx
from kicad_parser import build_pcb_data_from_board
from kicad_routing_plugin import swig_gui, ai_plan
from copy_board import SIBLING_EXTS

src = repo/'kicad_files'/((sys.argv[3] if len(sys.argv)>3 else 'flat_hierarchy')+'.kicad_pcb')
board_path = dest/'native.kicad_pcb'
shutil.copy2(src, board_path)
source_siblings = {}
for ext in SIBLING_EXTS:
    sibling = src.with_suffix(ext)
    source_siblings[ext] = hashlib.sha256(sibling.read_bytes()).hexdigest() if sibling.exists() else None
    if sibling.exists():
        shutil.copy2(sibling,board_path.with_suffix(ext))
app = wx.App(False)
board = pcbnew.LoadBoard(str(board_path))
pcbnew.GetBoard = lambda: board
dlg = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board), str(board_path))
nc = board.GetDesignSettings().m_NetSettings.GetDefaultNetclass()
physical = 0.1 if board.GetCopperLayerCount()<=2 else 0.09
rows = []
def check(case, expected):
    got = dlg._effective_clearance()
    row = {'case':case, 'expected':expected, 'actual':got, 'pass':abs(got-expected)<1e-9}
    rows.append(row)
    print(json.dumps(row), flush=True)

# The physical floor is the advanced capability (0.1 for two layers), even
# for standard tier: a tier bounds automatic descents, not initial flooring.
# This contract is stated by fab_floor_for_param, not inferred from its result.
for tier, floor in [('standard',physical),('advanced',physical),('auto',physical)]:
    dlg.fab_tier.SetStringSelection(tier)
    for declared in [0.0,physical-0.000001,physical,physical+0.000001,0.2,0.4]:
        nc.SetClearance(round(declared*1e6))
        for typed in [0.25,0.73]:
            dlg.clearance.SetValue(typed)
            dlg.clearance_check.SetValue(False)
            dlg.clearance_ceiling_check.SetValue(False)
            check(f'{tier}/declared={declared}/unchecked-typed={typed}',max(declared,floor))
        for typed in [0.09,0.3]:
            dlg.clearance.SetValue(typed)
            dlg.clearance_check.SetValue(True)
            check(f'{tier}/declared={declared}/explicit={typed}',max(typed,floor))
            dlg.clearance_ceiling_check.SetValue(True)
            check(f'{tier}/declared={declared}/ceiling={typed}',max(min(declared,typed),floor))
            dlg.clearance_ceiling_check.SetValue(False)

# A physical custom capability must be honored, and zero hole defaults stay
# distinct from zero clearance. Both were implicated by the original comment.
override = dest/'capability.fab'
override.write_text('clearance = 0.083\nhole_to_hole = 0.10\n')
nc.SetClearance(0)
dlg.clearance_check.SetValue(False)
dlg.fab_overrides_path.SetValue(str(override))
check('custom physical clearance=0.083, declared zero',0.083)
board.GetDesignSettings().m_HoleToHoleMin = 0
dlg.hole_to_hole_clearance_check.SetValue(False)
dlg.hole_to_hole_clearance.SetValue(0.2)
hole = dlg._effective_hole_to_hole_clearance()
rows.append({'case':'custom hole floor0.10 declared0 keeps CLI fallback0.2','expected':0.2,'actual':hole,'pass':abs(hole-0.2)<1e-9})

# Exercise production plan controls/reset on the actual board, with stale
# preceding explicit override and no clearance parameter in the next step.
dlg.fab_overrides_path.SetValue('')
ai_plan.apply_step_params({'action':'route','params':{'clearance':0.3}},dlg)
check('plan explicit clearance',0.3)
dlg.reset_params_to_defaults()
ai_plan.apply_step_params({'action':'route','params':{}},dlg)
check('plan reset then omitted clearance',physical)

data = {'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
        'production_swig_gui_sha256':hashlib.sha256((repo/'kicad_routing_plugin/swig_gui.py').read_bytes()).hexdigest(),
        'source_board':str(src),'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),
        'source_project_sha256':hashlib.sha256(src.with_suffix('.kicad_pro').read_bytes()).hexdigest(),
        'source_siblings':source_siblings,
        'kicad':pcbnew.GetBuildVersion(),'rows':rows,
        'passed':sum(r['pass'] for r in rows),'failed':sum(not r['pass'] for r in rows)}
(dest/'matrix.json').write_text(json.dumps(data,indent=2))
print(json.dumps({k:data[k] for k in ['revision','passed','failed']}),flush=True)
dlg.Destroy()
sys.exit(bool(data['failed']))
