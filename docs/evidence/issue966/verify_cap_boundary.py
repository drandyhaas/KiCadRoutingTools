"""Independent actual native cap optimization boundary on the public flat board."""
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys

repo=Path(sys.argv[1]).resolve()
out=Path(sys.argv[2]).resolve()
out.mkdir(parents=True,exist_ok=False)
sys.path[:0]=[str(repo),str(repo/'py_router'),str(repo/'py_tools'),str(repo/'py_placer'),str(repo/'kicad_routing_plugin')]
os.environ['WXSUPPRESS_SIZER_FLAGS_CHECK']='1'
import pcbnew
import wx
from copy_board import copy_board
from kicad_parser import build_pcb_data_from_board
from kicad_routing_plugin import swig_gui, ai_plan
from placement.fanout_clearance import resolve_pair_clearance

src=repo/'kicad_files/flat_hierarchy.kicad_pcb'
dst=out/'zero.kicad_pcb'
copy_board(str(src),str(dst))
pro=json.loads(dst.with_suffix('.kicad_pro').read_text())
next(c for c in pro['net_settings']['classes'] if c['name']=='Default')['clearance']=0
dst.with_suffix('.kicad_pro').write_text(json.dumps(pro,indent=2))
app=wx.App(False)
board=pcbnew.LoadBoard(str(dst))
pcbnew.GetBoard=lambda:board
dlg=swig_gui.RoutingDialog(None,build_pcb_data_from_board(board),str(dst))
dlg.reset_params_to_defaults()
shared=dlg.fanout_tab.get_shared_params()
cli_pair=resolve_pair_clearance(str(dst),None)
log=io.StringIO()
with contextlib.redirect_stdout(log):
    result=dlg.fanout_tab._optimize_decoupling_caps(board,pcbnew,shared)
text=log.getvalue()
(out/'cap.log').write_text(text,encoding='utf-8')
data={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),'routed_clearance':shared['clearance'],'cli_omitted_cap_clearance':cli_pair,'returned':result,'engine_log':text}
(out/'cap.json').write_text(json.dumps(data,indent=2))
print(json.dumps(data,indent=2))
assert cli_pair[0]==0.25
assert 'cap pair clearance: 0.25mm' in text,('cap requirement changed',text)
assert 'Error' not in text and 'Traceback' not in text,text
rows=[]
for declared, params, expected in [(0,{'clearance':0.3},0.3),(0,{'clearance':0.05},0.1),(0,{'clearance_ceiling':0.3},0.1),(0.2,{},0.2)]:
    next(c for c in pro['net_settings']['classes'] if c['name']=='Default')['clearance']=declared
    dst.with_suffix('.kicad_pro').write_text(json.dumps(pro,indent=2))
    board.GetDesignSettings().m_NetSettings.GetDefaultNetclass().SetClearance(round(declared*1e6))
    dlg.reset_params_to_defaults()
    ai_plan.apply_step_params({'action':'route','params':params},dlg)
    shared=dlg.fanout_tab.get_shared_params()
    log=io.StringIO()
    with contextlib.redirect_stdout(log):
        result=dlg.fanout_tab._optimize_decoupling_caps(board,pcbnew,shared)
    text=log.getvalue()
    assert 'cap pair clearance: '+str(expected)+'mm' in text,(declared,params,text)
    assert 'Error' not in text and 'Traceback' not in text,text
    rows.append({'declared':declared,'params':params,'expected':expected,'routing_clearance':shared['clearance'],'returned':result,'log':text})
(out/'cap-controls.json').write_text(json.dumps(rows,indent=2))
dlg.Destroy()
