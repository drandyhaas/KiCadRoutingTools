"""Final reviewer native geometry and actual check_floorplan/check_drc controls for #961.
Measurements here are independently obtained from native KiCad drawing vertices and
copper polygons; no production body/edge helper is used for expected values.
"""
import hashlib,json,subprocess,sys,time
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'py_placer').is_dir())
sys.path[:0]=[str(ROOT/'py_router'),str(ROOT/'py_placer')]
import pcbnew
from copy_board import copy_board
from placement.writer import write_placed_output
from kicad_parser import parse_kicad_pcb
OUT=ROOT/'docs/issue-961-evidence/final'/('geometry-'+time.strftime('%Y%m%d-%H%M%S'));OUT.mkdir(parents=True)
SOURCE=ROOT/'kicad_files/esp_prog.kicad_pcb'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def native(p):
 b=pcbnew.LoadBoard(str(p));f=next(f for f in b.GetFootprints() if f.GetReference()=='USB1')
 edges=[d for d in b.GetDrawings() if d.GetLayer()==pcbnew.Edge_Cuts];ep=[q for d in edges for q in (d.GetStart(),d.GetEnd())]
 bounds=[pcbnew.ToMM(min(q.x for q in ep)),pcbnew.ToMM(min(q.y for q in ep)),pcbnew.ToMM(max(q.x for q in ep)),pcbnew.ToMM(max(q.y for q in ep))]
 fab=[g for g in f.GraphicalItems() if g.GetLayer()==pcbnew.F_Fab and isinstance(g,pcbnew.PCB_SHAPE)]
 vertices=[q for g in fab for q in (g.GetStart(),g.GetEnd())];west=pcbnew.ToMM(min(q.x for q in vertices))
 gaps=[]
 for pad in f.Pads():
  box=pad.GetEffectivePolygon(pcbnew.F_Cu).BBox();a,z=box.GetOrigin(),box.GetEnd()
  gaps.append({'pad':pad.GetNumber(),'west_gap_mm':pcbnew.ToMM(a.x)-bounds[0], 'bbox_mm':[pcbnew.ToMM(a.x),pcbnew.ToMM(a.y),pcbnew.ToMM(z.x),pcbnew.ToMM(z.y)]})
 return {'pose':[pcbnew.ToMM(f.GetPosition().x),pcbnew.ToMM(f.GetPosition().y),f.GetOrientationDegrees()], 'body_vertex_west_mm':west,'body_overhang_mm':max(0,bounds[0]-west),'body_setback_mm':max(0,west-bounds[0]),'bounds_mm':bounds,'native_pad_polygons':gaps,'pad_west_gap_mm':min(g['west_gap_mm'] for g in gaps)}
report={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'python':sys.version,'kicad':pcbnew.GetBuildVersion(),'source_sha256':sha(SOURCE),'commands':[],'variants':[]}
def run(label,*args):
 argv=[sys.executable,'-X','utf8',*map(str,args)];r=subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=180)
 (OUT/(label+'.log')).write_text(r.stdout+'\nSTDERR\n'+r.stderr,encoding='utf8')
 row={'label':label,'argv':argv,'exit':r.returncode,'summary':[json.loads(t.split(': ',1)[1]) for t in r.stdout.splitlines() if t.startswith('JSON_SUMMARY: ')]}
 assert 'Traceback' not in r.stdout+r.stderr,(label,r.stdout,r.stderr)
 report['commands'].append(row);return row
fp=parse_kicad_pcb(str(SOURCE)).footprints['USB1']
for dx in (0,-.1,-1.0,-1.45,-2.1,.4):
 name=('dx'+str(dx)).replace('-','minus').replace('.','p');board=OUT/(name+'.kicad_pcb');copy_board(str(SOURCE),str(board))
 if dx:write_placed_output(str(board),str(board),[dict(reference='USB1',new_x=fp.x+dx,new_y=fp.y,new_rotation=fp.rotation)])
 actual=native(board);assert abs(actual['body_overhang_mm']-max(0,-dx))<1e-6,actual
 assert abs(actual['body_setback_mm']-max(0,dx))<1e-6,actual
 assert abs(actual['pad_west_gap_mm']-(1.6+dx))<1e-6,actual
 row={'dx_mm':dx,'board_sha256':sha(board),'native':actual,'floorplan':[]}
 for band,bounds,seat in [('positive',{'min':.05,'max':.20},None),('zero',{'min':0,'max':.65},None),('legal-body',{'min':0,'max':1.5},None),('seat',{'min':0,'max':.65},.1)]:
  entry={'ref':'USB1','edge':'west','overhang_mm':bounds}
  if seat is not None:entry['max_setback_mm']=seat
  intent=OUT/(name+'-'+band+'.intent.json');intent.write_text(json.dumps({'schema':1,'kind':'floorplan-intent','units':'mm','edge_connectors':[entry]}),encoding='utf8');ih=sha(intent)
  for clearance,edge in ((.25,0),(.25,.25),(.25,.55),(.55,.25)):
   label=name+'-'+band+'-'+str(clearance)+'-'+str(edge);dest=OUT/(label+'.json')
   r=run(label,ROOT/'py_tools/check_floorplan.py',board,'--intent',intent,'--clearance',clearance,'--board-edge-clearance',edge,'--json',dest)
   assert r['exit'] in (0,4),(label,r)
   assert sha(intent)==ih
   grade=json.loads(dest.read_text(encoding='utf8'));row['floorplan'].append({'band':band,'requested':{'clearance':clearance,'board_edge_clearance':edge},'intent_sha256':ih,'json_file':dest.name,'grade':grade})
  assert sha(board)==row['board_sha256']
 r=run(name+'-drc',ROOT/'py_router/check_drc.py',board,'--check-pad-edge','--board-edge-clearance','.25','--clearance-margin','0')
 assert r['exit'] in (0,1),(name,r)
 row['drc']=r
 report['variants'].append(row);(OUT/'results.json').write_text(json.dumps(report,indent=2),encoding='utf8')
assert sha(SOURCE)==report['source_sha256'];report['source_preserved']=True
(OUT/'results.json').write_text(json.dumps(report,indent=2),encoding='utf8');print(OUT)

