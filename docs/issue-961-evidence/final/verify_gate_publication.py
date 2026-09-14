"""Independent accepted/dry/in-place/failed-write probes of declared connector gate."""
import hashlib,json,subprocess,sys,time,stat,os
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'py_placer').is_dir());sys.path[:0]=[str(ROOT/'py_router'),str(ROOT/'py_placer')]
import pcbnew
from placement import provenance as pv
from copy_board import copy_board
OUT=ROOT/'docs/issue-961-evidence/final'/('gate-'+time.strftime('%Y%m%d-%H%M%S'));OUT.mkdir(parents=True);SOURCE=ROOT/'kicad_files/esp_prog.kicad_pcb'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
report={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'production_files':{p:sha(ROOT/p) for p in ['py_placer/placement/connector_publication.py','py_placer/place_seed.py','py_placer/place_reconstruct.py']},'commands':[]}
for tool,extra in [('place_seed.py',['--repair']),('place_reconstruct.py',['--stages','classify'])]:
 for mode in ('accepted','dry','inplace','readonly'):
  name=tool[:-3]+'-'+mode;work=OUT/name;work.mkdir();src=work/'board.kicad_pcb';copy_board(str(SOURCE),str(src));src.with_suffix('.kicad_dru').write_text('(version 1)\n');src.with_suffix('.design-brief.json').write_text('{"schema":1,"kind":"design-brief","units":"mm","product":{"held_by":"reviewer"}}\n');intent=work/'intent.json';intent.write_text(json.dumps({'schema':1,'kind':'floorplan-intent','units':'mm','edge_connectors':[{'ref':'USB1','edge':'west','overhang_mm':{'min':0,'max':.65},'max_setback_mm':.1}]}));ih=sha(intent);pv.start_regime(str(work),str(src));before=sha(src)
  out=src if mode=='inplace' else work/'out.kicad_pcb'
  if mode=='readonly':copy_board(str(src),str(out));os.chmod(out,stat.S_IREAD)
  argv=[sys.executable,'-X','utf8',str(ROOT/'py_placer'/tool),str(src),str(out),'--intent',str(intent),'--clearance','.2','--board-edge-clearance','.25',*extra,*(['--dry-run'] if mode=='dry' else [])]
  try:r=subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=180)
  finally:
   if mode=='readonly':os.chmod(out,stat.S_IREAD|stat.S_IWRITE)
  (OUT/(name+'.log')).write_text(r.stdout+'\nSTDERR\n'+r.stderr,encoding='utf8');rows=pv.read_ledger(str(work));sums=[json.loads(t.split(': ',1)[1]) for t in r.stdout.splitlines() if t.startswith('JSON_SUMMARY: ')];row={'argv':argv,'exit':r.returncode,'summary':sums[-1] if sums else None,'output_exists':out.exists(),'ledger':rows,'input_before_sha256':before,'input_after_sha256':sha(src),'intent_preserved':sha(intent)==ih,'traceback':'Traceback' in r.stdout+r.stderr};report['commands'].append(row)
  if mode in ('accepted','inplace'):
   assert r.returncode==0 and out.exists() and len(rows)==1,(name,row,r.stderr)
   native=pcbnew.LoadBoard(str(out));pose={f.GetReference():[pcbnew.ToMM(f.GetPosition().x),pcbnew.ToMM(f.GetPosition().y),f.GetOrientationDegrees()%360] for f in native.GetFootprints()};row['native_USB1']=pose['USB1'];assert pose['USB1']==[117.5,100,180];assert rows[-1]['board_sha256']==sha(out)==rows[-1]['candidate_sha256'];assert rows[-1]['poses_written']['USB1']==pose['USB1'];assert row['summary']['connector_requirements']['accepted'];assert sha(out.with_suffix('.kicad_dru'))==sha(src.with_suffix('.kicad_dru')) and sha(out.with_suffix('.design-brief.json'))==sha(src.with_suffix('.design-brief.json'))
   args=[sys.executable,'-X','utf8',str(ROOT/'tests/stress/provenance_audit.py'),'--workdir',str(work),'--delivered',str(out)];a=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=90);row.update(audit_argv=args,audit_exit=a.returncode,audit_stdout=a.stdout);assert a.returncode==0
   if mode=='inplace':manifest=json.loads((work/pv.REGIME_NAME).read_text());row['manifest']=manifest;assert sha(manifest['staged_board'])==before
  if mode=='dry':
   assert r.returncode==0 and not out.exists() and not rows
   assert row['summary']['status']=='dry_run' and not row['summary']['complete'] and not row['summary']['engineering_clean']
   assert row['summary']['connector_requirements']['accepted'] is None and row['summary']['connector_requirements']['reason']
  if mode=='readonly':
   assert not rows and sha(out)==before and r.returncode==4 and not row['traceback']
   assert row['summary']['output_state']=='unchanged' and row['summary']['status']=='error' and not row['summary']['published']
  assert sha(intent)==ih
  (OUT/'results.json').write_text(json.dumps(report,indent=2));print(name,r.returncode,'rows',len(rows),'traceback',row['traceback'],flush=True)
print(OUT)
