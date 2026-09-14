"""Independent966 actual CLI and GUI plan, native output and project checks."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

repo = Path(sys.argv[1]).resolve()
out = Path(sys.argv[2]).resolve()
out.mkdir(parents=True,exist_ok=False)
sys.path[:0]=[str(repo),str(repo/'py_router'),str(repo/'py_tools')]
import pcbnew
from kicad_parser import parse_kicad_pcb
from copy_board import copy_board

src = repo/'kicad_files/flat_hierarchy.kicad_pcb'
net='Net-(D1-A)'
results=[]
def invoke(argv, stem):
    r=subprocess.run(argv,cwd=repo,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=240)
    (out/(stem+'.log')).write_text(r.stdout+r.stderr,encoding='utf-8')
    results.append({'command':argv,'returncode':r.returncode,'log':stem+'.log'})
    print(stem,r.returncode,flush=True)
    assert r.returncode==0,(stem,r.stdout[-2000:],r.stderr[-2000:])
    return r

for variant in ['zero','positive']:
    inp=out/(variant+'.kicad_pcb')
    copy_board(str(src),str(inp))
    if variant=='zero':
        pro=json.loads(inp.with_suffix('.kicad_pro').read_text())
        next(c for c in pro['net_settings']['classes'] if c['name']=='Default')['clearance']=0.0
        pro['board']['design_settings']['rules']['min_clearance']=0.0889
        inp.with_suffix('.kicad_pro').write_text(json.dumps(pro,indent=2))
    plan={'steps':[{'action':'route','nets':[net], 'layers':['F.Cu','B.Cu'], 'params':{'max_ripup':0}}]}
    plan_path=out/(variant+'-plan.json')
    plan_path.write_text(json.dumps(plan,indent=2))
    for front in ['cli','gui']:
        dst=out/(variant+'-'+front+'.kicad_pcb')
        if front=='cli':
            argv=[sys.executable,'-X','utf8',str(repo/'py_router/route.py'),str(inp),str(dst),'--nets',net,'--layers','F.Cu','B.Cu','--max-ripup','0']
        else:
            argv=[sys.executable,'-X','utf8',str(repo/'py_router/run_plan.py'),str(inp),str(plan_path),'-o',str(dst),'--timeout','180']
        invoke(argv,variant+'-'+front)

(out/'commands.json').write_text(json.dumps(results,indent=2))
print('ALL_COMMANDS_COMPLETE',flush=True)
