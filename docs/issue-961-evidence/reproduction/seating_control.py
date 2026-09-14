"""Independent positive/zero minimum and explicit-seat controls for #961."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

p = argparse.ArgumentParser()
p.add_argument('--root', type=Path, required=True)
p.add_argument('--out', type=Path, required=True)
a = p.parse_args()
root, out = a.root.resolve(), a.out.resolve()
out.mkdir(parents=True, exist_ok=True)
sys.path[:0] = [str(root/x) for x in ('py_router', 'py_placer')]
import pcbnew
from copy_board import copy_board, SIBLING_EXTS
from placement.writer import write_placed_output
from placement import floorplan as fl
from placement.legality import BoardOutlineGate
from kicad_parser import parse_kicad_pcb

def dump(p, x): p.write_text(json.dumps(x, indent=2, default=str)+'\n', encoding='utf8')
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
commands=[]
def run(args, log):
    argv = [sys.executable, '-X', 'utf8', *map(str,args)]
    r=subprocess.run(argv, cwd=root, text=True, encoding='utf8', capture_output=True)
    log.write_text(r.stdout+r.stderr, encoding='utf8')
    assert 'Traceback' not in r.stdout+r.stderr
    assert r.returncode in (0,1,4), (argv, r.stdout, r.stderr)
    commands.append({'argv':argv,'cwd':str(root),'exit':r.returncode,'log':str(log.relative_to(out))})
    return r.returncode

source=root/'kicad_files/esp_prog.kicad_pcb'
identity=sha(source)
board=out/'inboard100.kicad_pcb'
copy_board(str(source),str(board))
native=pcbnew.LoadBoard(str(source))
fp=next(f for f in native.GetFootprints() if f.GetReference()=='USB1')
write_placed_output(str(board),str(board),[{'reference':'USB1','new_x':pcbnew.ToMM(fp.GetPosition().x)+1.,
    'new_y':pcbnew.ToMM(fp.GetPosition().y),'new_rotation':fp.GetOrientationDegrees()}])
for ext in SIBLING_EXTS:
    assert source.with_suffix(ext).exists()==board.with_suffix(ext).exists()
native=pcbnew.LoadBoard(str(board))
fp=next(f for f in native.GetFootprints() if f.GetReference()=='USB1')
fab=[g for g in fp.GraphicalItems() if isinstance(g,pcbnew.PCB_SHAPE) and g.GetLayerName()=='F.Fab']
native_body_west=min(pcbnew.ToMM(v.x) for g in fab for v in (g.GetStart(),g.GetEnd()))
assert abs(native_body_west-115.)<1e-6
cases={
 'zero_no_class': {'overhang_mm':{'min':0.,'max':.65}},
 'zero_class_no_seating': {'class':'edge_receptacle','overhang_mm':{'min':0.,'max':.65}},
 'positive_no_seating': {'overhang_mm':{'min':.05,'max':.20}},
 'seat_020': {'overhang_mm':{'min':0.,'max':.65}, 'max_setback_mm':.2},
 'seat_110': {'overhang_mm':{'min':0.,'max':.65}, 'max_setback_mm':1.1}}
rows=[]
data=parse_kicad_pcb(str(board))
for name, extra in cases.items():
    ip=out/(name+'.intent.json')
    dump(ip,{'schema':fl.SCHEMA_VERSION,'kind':fl.KIND,'units':'mm',
             'edge_connectors':[dict(ref='USB1',edge='west',**extra)]})
    for margin in [.25,.55]:
        tag=f'{name}-{margin}'
        code=run([root/'py_tools/check_floorplan.py',board,'--intent',ip,'--clearance',margin,
                  '--board-edge-clearance',margin,'--json',out/(tag+'.json')],out/(tag+'.log'))
        doc=json.loads((out/(tag+'.json')).read_text(encoding='utf8'))
        rows.append({'case':name,'clearance_and_edge_requested_mm':margin,'cli_exit':code,
                     'pass':doc['pass'],'edge_seating':doc['edge_seating'],
                     'edge_violations':[v for v in doc['violations'] if v['rule']=='edge_connector']})
assert sha(source)==identity
legal_board=out/'legal-overhang130.kicad_pcb'
copy_board(str(source),str(legal_board))
native=pcbnew.LoadBoard(str(source))
fp=next(f for f in native.GetFootprints() if f.GetReference()=='USB1')
write_placed_output(str(legal_board),str(legal_board),[{'reference':'USB1',
    'new_x':pcbnew.ToMM(fp.GetPosition().x)-1.3,'new_y':pcbnew.ToMM(fp.GetPosition().y),
    'new_rotation':fp.GetOrientationDegrees()}])
native=pcbnew.LoadBoard(str(legal_board))
fp=next(f for f in native.GetFootprints() if f.GetReference()=='USB1')
fab=[g for g in fp.GraphicalItems() if isinstance(g,pcbnew.PCB_SHAPE) and g.GetLayerName()=='F.Fab']
legal_body_west=min(pcbnew.ToMM(v.x) for g in fab for v in (g.GetStart(),g.GetEnd()))
legal_pad_west=min(pcbnew.ToMM(p.GetBoundingBox().GetLeft()) for p in fp.Pads())
assert abs(legal_body_west-112.7)<1e-6
assert abs(legal_pad_west-114.3)<1e-6
for ext in SIBLING_EXTS:
    assert source.with_suffix(ext).exists()==legal_board.with_suffix(ext).exists()
ip=out/'legal-overhang130.intent.json'
dump(ip,{'schema':fl.SCHEMA_VERSION,'kind':fl.KIND,'units':'mm','edge_connectors':[
    {'ref':'USB1','edge':'west','overhang_mm':{'min':1.25,'max':1.35},'max_setback_mm':0.}]})
legal_rows=[]
for edge in [.25,.55]:
    tag=f'legal-overhang130-{edge}'
    gc=run([root/'py_tools/check_floorplan.py',legal_board,'--intent',ip,'--clearance',.25,
            '--board-edge-clearance',edge,'--json',out/(tag+'.grade.json')],out/(tag+'.grade.log'))
    dc=run([root/'py_router/check_drc.py',legal_board,'--check-pad-edge','--board-edge-clearance',edge,
            '--clearance-margin',0,'--json',out/(tag+'.drc.json')],out/(tag+'.drc.log'))
    legal_rows.append({'requested_copper_mm':None,'requested_edge_mm':edge,
        'grade_exit':gc,'drc_exit':dc,
        'grade':json.loads((out/(tag+'.grade.json')).read_text(encoding='utf8')),
        'drc':json.loads((out/(tag+'.drc.json')).read_text(encoding='utf8'))})
dump(out/'results.json',{'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
     'source_sha256':identity,'board_sha256':sha(board),'native_body_west_mm':native_body_west,
     'native_body_overhang_mm':0.,'native_body_setback_mm':1.,'rows':rows,'commands':commands,
     'legal_control':{'board_sha256':sha(legal_board),'body_overhang_mm':114.-legal_body_west,
        'pad_gap_mm':legal_pad_west-114.,'rows':legal_rows}})
print(json.dumps([{'case':r['case'],'margin':r['clearance_and_edge_requested_mm'],'pass':r['pass']} for r in rows],indent=2))
