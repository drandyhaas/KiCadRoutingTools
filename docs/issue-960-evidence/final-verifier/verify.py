"""Independent final reviewer actual CLI/native KiCad verification; public copies only."""
import hashlib, json, os, shutil, stat, subprocess, sys, time
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'py_placer').is_dir())
sys.path[:0]=[str(ROOT/p) for p in ('py_router','py_placer','tests/stress')]
import pcbnew
from placement import provenance as PV
from placement.writer import write_placed_output
from placement.seeder import stamp_locked
from copy_board import copy_board
from kicad_parser import parse_kicad_pcb
BASE=ROOT/'review_final'/('run-'+time.strftime('%Y%m%d-%H%M%S'))
BASE.mkdir(parents=True)
SOURCE=ROOT/'kicad_files/esp_prog.kicad_pcb'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def files(board): return {e:sha(board.with_suffix(e)) for e in ('.kicad_pcb','.kicad_pro','.kicad_dru','.design-brief.json') if board.with_suffix(e).is_file()}
def native(p):
 b=pcbnew.LoadBoard(str(p))
 return {(f.GetReference() or '#'+f.m_Uuid.AsString()):{'pose':[pcbnew.ToMM(f.GetPosition().x),pcbnew.ToMM(f.GetPosition().y),f.GetOrientationDegrees()%360], 'locked':f.IsLocked(),'side':'B' if f.IsFlipped() else 'F'} for f in b.GetFootprints()}
def native_fixed(p):
 b=pcbnew.LoadBoard(str(p))
 return {f.m_Uuid.AsString():[f.GetReference(),f.GetPosition().x,f.GetPosition().y,f.GetOrientationDegrees(),f.IsLocked(),f.GetLayer()] for f in b.GetFootprints() if f.GetReference() not in ('C2','C4','U1','Y1')}
def native_gap(p):
 b=pcbnew.LoadBoard(str(p)); edges=[d for d in b.GetDrawings() if d.GetLayer()==pcbnew.Edge_Cuts]
 pts=[xy for e in edges for xy in (e.GetStart(),e.GetEnd())]
 x0,x1=min(p.x for p in pts),max(p.x for p in pts); y0,y1=min(p.y for p in pts),max(p.y for p in pts)
 f=next(f for f in b.GetFootprints() if f.GetReference()=='Y1')
 gaps=[]
 for pad in f.Pads():
  box=pad.GetEffectivePolygon(pcbnew.F_Cu).BBox(); a,z=box.GetOrigin(),box.GetEnd()
  gaps.append({'pad':pad.GetNumber(),'gap_mm':pcbnew.ToMM(min(a.x-x0,x1-z.x,a.y-y0,y1-z.y))})
 return gaps
DOC={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'source':str(SOURCE),'source_sha256':sha(SOURCE),'python':sys.version,'kicad':pcbnew.GetBuildVersion(),'commands':[], 'checks':[]}
def check(label, condition, detail=None):
 DOC['checks'].append({'label':label,'pass':bool(condition),'detail':detail})
 (BASE/'results.json').write_text(json.dumps(DOC,indent=2),encoding='utf8')
 if not condition: print('FAIL',label,detail)
def run(label,*args):
 argv=[sys.executable,'-X','utf8',*map(str,args)]
 r=subprocess.run(argv,cwd=ROOT,text=True,encoding='utf8',capture_output=True,timeout=180)
 row={'label':label,'argv':argv,'exit':r.returncode,'stdout':r.stdout,'stderr':r.stderr}
 sums=[json.loads(x.split(': ',1)[1]) for x in r.stdout.splitlines() if x.startswith('JSON_SUMMARY: ')]
 row['summary']=sums[-1] if sums else {}
 DOC['commands'].append(row)
 (BASE/(label+'.log')).write_text(r.stdout+'\nSTDERR\n'+r.stderr,encoding='utf8')
 return row
def pose(label,board,out,*args): return run(label,ROOT/'py_placer/place_pose.py',board,out,*args)
def audit(label,work,out):
 r=run(label,ROOT/'tests/stress/provenance_audit.py','--workdir',work,'--delivered',out,'--json',BASE/(label+'.json'))
 check(label+' audit CLEAN',r['exit']==0 and r['summary'].get('verdict')=='CLEAN',r['summary'])
 return r
def accepted(label,r,board,out,refs,strict=False):
 check(label+' accepted',r['exit']==0 and out.is_file(),r['summary'])
 if r['exit'] or not out.is_file(): return
 n=native(out); rows=PV.read_ledger(str(out.parent)); row=rows[-1] if rows else {}
 DOC.setdefault('outputs',{})[label]={'identities':files(out),'native':n,'ledger':rows}
 check(label+' final SHA',row.get('board_sha256')==sha(out),row)
 check(label+' decision/execution distinct',row.get('applied_by')=='place_pose.py' and row.get('decision_source')=='caller',row)
 for ref in refs:
  p=row.get('poses_written',{}).get(ref)
  check(label+' native pose '+ref,p is not None and all(abs((a-b+180)%360-180) < 1e-5 if i==2 else abs(a-b)<1e-6 for i,(a,b) in enumerate(zip(p or [],n[ref]['pose']))),{'native':n[ref],'row':row})
  check(label+' final lock/face '+ref,row.get('locks_written',{}).get(ref)==n[ref]['locked'] and row.get('sides_written',{}).get(ref)==n[ref]['side'],n[ref])
 for ext,h in files(board).items():
  if ext!='.kicad_pcb': check(label+' sibling '+ext,files(out).get(ext)==h)
 if strict: check(label+' strict legal',r['summary'].get('legal') is True)
 audit(label+'-audit',out.parent,out)

# Real staging, required improving pile move, chaining direct and lock changes.
work=BASE/'pile'; truth=BASE/'truth'
r=run('stage',ROOT/'tests/stress/stage_unaided.py',SOURCE,work,truth)
check('real stage succeeded',r['exit']==0)
board=work/'board.kicad_pcb'; baseline=sha(board); DOC['baseline']={'files':files(board),'manifest':json.loads((work/PV.REGIME_NAME).read_text()),'mechanical':json.loads((work/'mechanical.json').read_text())}
out=work/'pose.kicad_pcb'
r=pose('improving-pile',board,out,'set','R1','136.4','98.8','--rot','270')
accepted('improving-pile',r,board,out,['R1'])
check('pile remains engineering unclean without force',r['summary'].get('no_worse') is True and r['summary'].get('legal') is False and r['summary'].get('forced') is False,r['summary'])
if out.is_file():
 n=native(out); check('model coordinates preserved',n['R1']['pose']==[136.4,98.8,270.0],n['R1'])
 for label,args,refs in [('rotate',['rotate','R1','90'],['R1']),('face',['face','U1','N','R1','--force'],['U1']),('lock',['lock','R1'],['R1']),('unlock',['unlock','R1'],['R1']),('multi',['set','R1','136.4','98.8','--rot','270','rotate','R2','180'],['R1','R2'])]:
  dst=work/(label+'.kicad_pcb'); r=pose(label,out,dst,*args)
  accepted(label,r,out,dst,refs)
  if dst.is_file() and r['exit']==0:
   if label in ('lock','unlock'): check(label+' native lock',native(dst)['R1']['locked']==(label=='lock'))
   if label=='face':
    check('face waiver explicitly disclosed',r['summary'].get('forced') is True)
    nat=pcbnew.LoadBoard(str(dst)); f=next(f for f in nat.GetFootprints() if f.GetReference()=='U1')
    row=[p for p in f.Pads() if p.GetNumber() in [str(i) for i in range(11,21)]]
    offsets=[(pcbnew.ToMM(p.GetPosition().x-f.GetPosition().x),pcbnew.ToMM(p.GetPosition().y-f.GetPosition().y)) for p in row]
    check('face row independently points east',len(row)==10 and sum(p[0] for p in offsets)>abs(sum(p[1] for p in offsets)),offsets)
   else: out=dst
 before=files(out); rows=len(PV.read_ledger(str(work)))
 r=pose('in-place',out,out,'rotate','R1','90'); accepted('in-place',r,out,out,['R1'])
 for label,args in [('dry',['set','R1','136.4','98.8','--rot','270','--dry-run']),('refuse',['set','R1','1000','1000'])]:
  dst=work/(label+'.kicad_pcb'); rows=len(PV.read_ledger(str(work))); before=files(out)
  r=pose(label,out,dst,*args)
  check(label+' no publication/ledger',not dst.exists() and rows==len(PV.read_ledger(str(work))) and before==files(out),r['summary'])
  check(label+' reason',r['exit']==(0 if label=='dry' else 4) and (label=='dry' or 'WORSE' in r['summary'].get('refused','')))
check('baseline immutable',sha(board)==baseline)
rows=len(PV.read_ledger(str(work)))
r=pose('baseline-inplace',board,board,'set','R1','136.4','98.8','--rot','270')
accepted('baseline-inplace',r,board,board,['R1'])
manifest=json.loads((work/PV.REGIME_NAME).read_text()); frozen_baseline=Path(manifest['staged_board'])
check('in-place baseline snapshot identity',frozen_baseline!=board and sha(frozen_baseline)==baseline and manifest['staged_sha256']==baseline and files(frozen_baseline)==DOC['baseline']['files'],manifest)

# Public documented parent edge fixture, 17 fixed mechanics, and requirement siblings.
edge=BASE/'edge'; edge.mkdir(); b=edge/'baseline.kicad_pcb'; copy_board(str(SOURCE),str(b))
poses={'C2':(126.9,103.5,90),'C4':(122.4,102.2,90),'U1':(126.6,97.75,270),'Y1':(124.7,103.2,270)}
write_placed_output(str(b),str(b),[dict(reference=k,new_x=v[0],new_y=v[1],new_rotation=v[2]) for k,v in poses.items()])
fixed=set(parse_kicad_pcb(str(b)).footprints)-set(poses); stamp_locked(str(b),fixed)
b.with_suffix('.kicad_dru').write_text('(version 1)\n(rule "copper" (constraint clearance (min 0.25)))\n')
b.with_suffix('.kicad_pro').write_text(json.dumps({'board':{'design_settings':{'rules':{'min_copper_edge_clearance':.55}}},'net_settings':{'classes':[{'name':'Default','clearance':.25,'track_width':.3}]},'review_identity':'issue960-public-copy'}))
b.with_suffix('.design-brief.json').write_text('{"version":1,"purpose":"independent final verification fixed requirements"}\n')
PV.start_regime(str(edge),str(b)); frozen=files(b); nb=native(b)
knobs=['--clearance','.25','--board-edge-clearance','.55']
dst=edge/'snap.kicad_pcb'; r=pose('strict-snap',b,dst,*knobs,'--strict-legal','--radius','.5','--snap-step','.05','--snap-tries','24','set','Y1','--near','124.7','103.21','--rot','270')
accepted('strict-snap',r,b,dst,['Y1'],strict=True)
if dst.is_file():
 n=native(dst); check('snap final differs from request',n['Y1']['pose']!=[124.7,103.21,270.0] and bool(r['summary'].get('snapped')),n['Y1'])
 gaps=native_gap(dst); check('snap native copper edge gap',min(p['gap_mm'] for p in gaps)>=.55-1e-6,gaps)
 check('fixed mechanics preserved',native_fixed(dst)==native_fixed(b),native_fixed(dst))
 before=files(dst); rows=len(PV.read_ledger(str(edge)))
 r=pose('multi-refuse',dst,dst,*knobs,'set','Y1','124.7','103.2','--rot','270','set','C2','126.9','103.5','--rot','90')
 check('multi atomic refusal',r['exit']==4 and 'pad_edge' in r['summary'].get('refused','') and files(dst)==before and len(PV.read_ledger(str(edge)))==rows,r['summary'])
 out=edge/'direct-multi.kicad_pcb'; r=pose('direct-multi',b,out,*knobs,'set','Y1','124.7','103.15','--rot','270','set','C2','126.9','103.5','--rot','90')
 accepted('direct-multi',r,b,out,['Y1','C2'],strict=True)
 gaps=native_gap(out); check('direct boundary native copper edge gap',abs(min(p['gap_mm'] for p in gaps)-.55)<1e-6,gaps)
 # Real Windows failure after sibling replacements; previous bytes and ledger retained.
 prior=edge/'readonly.kicad_pcb'; copy_board(str(out),str(prior)); prior.with_suffix('.design-brief.json').write_text('{"old_requirement":true}\n'); old=files(prior); rows=len(PV.read_ledger(str(edge)))
 os.chmod(prior,stat.S_IREAD)
 try: r=pose('readonly-rollback',out,prior,*knobs,'set','Y1','124.7','103.15','--rot','270')
 finally: os.chmod(prior,stat.S_IWRITE|stat.S_IREAD)
 check('readonly rollback truthful',r['exit']==2 and r['summary'].get('output_state')=='unchanged' and files(prior)==old and len(PV.read_ledger(str(edge)))==rows,r['summary'])
 for ext in ('.kicad_pro','.kicad_dru','.design-brief.json'):
  minimal=BASE/('minimal-'+ext[1:]); minimal.mkdir(); src=minimal/'src.kicad_pcb'; copy_board(str(SOURCE),str(src))
  target=minimal/'out.kicad_pcb'; target.with_suffix(ext).write_text('original destination requirement\n'); digest=sha(target.with_suffix(ext)); nn=native(src)['R1']['pose']
  r=pose('extra-'+ext[1:],src,target,'set','R1',str(nn[0]),str(nn[1]),'--rot',str(nn[2]))
  check('destination-only requirement '+ext,r['exit']==2 and 'requirement siblings' in r['summary'].get('refused','') and not target.exists() and sha(target.with_suffix(ext))==digest,r['summary'])
check('edge baseline and requirements unchanged',files(b)==frozen)
check('original source unchanged',sha(SOURCE)==DOC['source_sha256'])
DOC['passed']=sum(x['pass'] for x in DOC['checks']); DOC['failed']=sum(not x['pass'] for x in DOC['checks'])
(BASE/'results.json').write_text(json.dumps(DOC,indent=2),encoding='utf8')
print(BASE, DOC['passed'], 'passed,',DOC['failed'],'failed')
