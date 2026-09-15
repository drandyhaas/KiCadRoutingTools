"""Actual alternate CLI publications on copied public esp_prog; no quality claim."""
import hashlib,json,subprocess,sys,time,math
from pathlib import Path
from collections import defaultdict
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'py_placer').is_dir()); sys.path[:0]=[str(ROOT/p) for p in ('py_placer','py_router','py_tools')]
import pcbnew
from copy_board import copy_board
from placement import provenance as PV
from placement.parser import extract_locked_refs
BASE=ROOT/'review_final'/('alternate-'+time.strftime('%Y%m%d-%H%M%S'));BASE.mkdir(parents=True)
SOURCE=ROOT/'kicad_files/esp_prog.kicad_pcb'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def native(p):
 b=pcbnew.LoadBoard(str(p));return {f.m_Uuid.AsString():{'ref':f.GetReference(),'pose':[pcbnew.ToMM(f.GetPosition().x),pcbnew.ToMM(f.GetPosition().y),f.GetOrientationDegrees()%360],'side':'B' if f.IsFlipped() else 'F','locked':f.IsLocked()} for f in b.GetFootprints()}
DOC={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'source_sha256':sha(SOURCE),'commands':[],'checks':[]}
def check(name,cond,detail=None):
 DOC['checks'].append(dict(name=name,passed=bool(cond),detail=detail));(BASE/'results.json').write_text(json.dumps(DOC,indent=2),encoding='utf8');print('PASS' if cond else 'FAIL',name,flush=True)
def run(name,*args):
 argv=[sys.executable,'-X','utf8',*map(str,args)]
 try:r=subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=180); row=dict(name=name,argv=argv,exit=r.returncode,stdout=r.stdout,stderr=r.stderr)
 except subprocess.TimeoutExpired as e:row=dict(name=name,argv=argv,exit='timeout',stdout=str(e.stdout),stderr=str(e.stderr))
 DOC['commands'].append(row);(BASE/(name+'.log')).write_text(row['stdout']+'\nSTDERR\n'+row['stderr'],encoding='utf8');return row
def verify(name,work,source,out,lever,code):
 rows=PV.read_ledger(str(work)); row=next((r for r in reversed(rows) if Path(r['path'])==out),{});
 check(name+' wrote final output',code in (0,4) and out.is_file(),code)
 if not out.exists():return
 n=native(out); DOC.setdefault('outputs',{})[name]={'sha256':sha(out),'native':n,'ledger':rows}
 check(name+' final ledger identity',row.get('board_sha256')==sha(out) and row.get('applied_by')==lever,row)
 checks=[]
 for f in n.values():
  ref=f['ref'];keys=[k for k in row.get('poses_written',{}) if k==ref or k.startswith(ref+'~')]
  if not ref:continue # anonymous IDs separately recorded by UUID natively
  checks.append(any(all(abs(x-y)<1e-6 for x,y in zip(row['poses_written'][k],f['pose'])) and row.get('locks_written',{}).get(k)==f['locked'] and row.get('sides_written',{}).get(k)==f['side'] for k in keys))
 check(name+' native pose locks sides reconcile',bool(checks) and all(checks))
 r=run(name+'-audit',ROOT/'tests/stress/provenance_audit.py','--workdir',work,'--delivered',out,'--json',BASE/(name+'-audit.json'))
 check(name+' audit CLEAN',r['exit']==0,r['stdout'])
 for ext in ('.kicad_dru','.design-brief.json'):
  if source.with_suffix(ext).is_file(): check(name+' requirements '+ext,out.with_suffix(ext).is_file() and sha(out.with_suffix(ext))==sha(source.with_suffix(ext)))

work=BASE/'seed';truth=BASE/'truth';run('stage',ROOT/'tests/stress/stage_unaided.py',SOURCE,work,truth);b=work/'board.kicad_pcb';out=work/'seeded.kicad_pcb'
intent=work/'intent.json';run('seed-intent',ROOT/'py_tools/check_floorplan.py',b,'--emit-intent',intent)
r=run('seed',ROOT/'py_placer/place_seed.py',b,out,'--intent',intent,'--no-polish','--anchors-first');verify('seed',work,b,out,'place_seed.py',r['exit'])
for name,tool,extra in [('reconstruct','place_reconstruct.py',['--stages','classify']),('seed-repair','place_seed.py',['--repair'])]:
 work=BASE/name;work.mkdir();b=work/'baseline.kicad_pcb';copy_board(str(SOURCE),str(b));b.with_suffix('.kicad_dru').write_text('(version 1)\n');b.with_suffix('.design-brief.json').write_text('{"schema":1,"kind":"design-brief","units":"mm","product":{"held_by":"alternate publication public-copy control"}}\n');PV.start_regime(str(work),str(b));old=sha(b);out=work/'out.kicad_pcb'
 if name=='seed-repair':
  intent=work/'intent.json';run('repair-intent',ROOT/'py_tools/check_floorplan.py',b,'--emit-intent',intent);extra+=['--intent',str(intent)]
 r=run(name,ROOT/'py_placer'/tool,b,out,*extra);verify(name,work,b,out,tool,r['exit']);check(name+' baseline preserved',sha(b)==old)
# A real route.py round, scoped to a shortest two-pad net; rounds=0 avoids placement search.
work=BASE/'route-loop';work.mkdir();b=work/'baseline.kicad_pcb';copy_board(str(SOURCE),str(b));PV.start_regime(str(work),str(b));old=sha(b)
nat=pcbnew.LoadBoard(str(b));nets=defaultdict(list)
for fp in nat.GetFootprints():
 for p in fp.Pads():
  if p.GetNetname():nets[p.GetNetname()].append(p.GetPosition())
choices=[(math.hypot(ps[0].x-ps[1].x,ps[0].y-ps[1].y),n) for n,ps in nets.items() if len(ps)==2];net=min(choices)[1];DOC['selected_route_net']=net
out=work/'out.kicad_pcb';r=run('route-loop',ROOT/'py_placer/place_route_loop.py',b,out,'--rounds','0','--route-args','--nets '+json.dumps(net)+' --max-iterations 1000');verify('route-loop',work,b,out,'place_route_loop.py',r['exit']);check('route-loop baseline preserved',sha(b)==old)
work=BASE/'ordinary';work.mkdir();b=work/'input.kicad_pcb';copy_board(str(SOURCE),str(b));out=work/'out.kicad_pcb';r=run('ordinary',ROOT/'py_placer/place_pose.py',b,out,'rotate','R1','90')
check('ordinary model-assisted CLI accepted',r['exit']==0 and out.is_file())
if out.is_file():
 check('ordinary model rotation exact',next(f for f in native(out).values() if f['ref']=='R1')['pose'][2]==90)
 check('ordinary no benchmark ledger fabricated',PV.regime_for(str(out)) is None and not PV.read_ledger(str(work)))
 r=run('ordinary-audit',ROOT/'tests/stress/provenance_audit.py','--workdir',work,'--delivered',out);check('ordinary benchmark claim UNPROVEN',r['exit']==5 and 'UNPROVEN' in r['stdout'])
check('source unchanged',sha(SOURCE)==DOC['source_sha256']);DOC['passed']=sum(r['passed'] for r in DOC['checks']);DOC['failed']=sum(not r['passed'] for r in DOC['checks']);(BASE/'results.json').write_text(json.dumps(DOC,indent=2),encoding='utf8');print(BASE,DOC['passed'],DOC['failed'])
