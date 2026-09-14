"""Independent candidate publication audit with unsupported body and explicit intent."""
import hashlib,json,subprocess,sys,time
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'py_placer').is_dir());sys.path[:0]=[str(ROOT/'py_router'),str(ROOT/'py_placer')]
from placement import provenance as pv
from copy_board import copy_board
SOURCE=Path(sys.argv[1]);INTENT=Path(sys.argv[2]);OUT=ROOT/'docs/issue-961-evidence/final'/('candidate-'+time.strftime('%Y%m%d-%H%M%S'));OUT.mkdir(parents=True)
report={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'commands':[]}
for case,tool,extra in [('seed-repair','place_seed.py',['--repair']),('reconstruct','place_reconstruct.py',['--stages','classify']),('seed-force','place_seed.py',['--force','--no-polish'])]:
 for mode in ('fresh','existing'):
  name=case+'-'+mode
  work=OUT/name;work.mkdir();src=work/'baseline.kicad_pcb';copy_board(str(SOURCE),str(src));intent=work/'intent.json';intent.write_bytes(INTENT.read_bytes());pv.start_regime(str(work),str(src));out=work/'out.kicad_pcb'
  if mode=='existing':copy_board(str(ROOT/'kicad_files/esp_prog.kicad_pcb'),str(out))
  input_before=hashlib.sha256(src.read_bytes()).hexdigest();intent_before=hashlib.sha256(intent.read_bytes()).hexdigest()
  output_before=hashlib.sha256(out.read_bytes()).hexdigest() if out.exists() else None
  argv=[sys.executable,'-X','utf8',str(ROOT/'py_placer'/tool),str(src),str(out),'--intent',str(intent),'--clearance','.2','--board-edge-clearance','.25',*extra]
  r=subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=180);(OUT/(name+'.log')).write_text(r.stdout+'\nSTDERR\n'+r.stderr,encoding='utf8');sums=[json.loads(t.split(': ',1)[1]) for t in r.stdout.splitlines() if t.startswith('JSON_SUMMARY: ')];row={'argv':argv,'exit':r.returncode,'summary':sums[-1] if sums else None,'output_exists':out.exists(),'output_before_sha256':output_before,'output_after_sha256':hashlib.sha256(out.read_bytes()).hexdigest() if out.exists() else None,'ledger':pv.read_ledger(str(work))};assert 'Traceback' not in r.stdout+r.stderr,(name,r.stdout,r.stderr)
  if out.exists():
   grade=OUT/(name+'-floorplan.json');args=[sys.executable,'-X','utf8',str(ROOT/'py_tools/check_floorplan.py'),str(out),'--intent',str(intent),'--clearance','.2','--board-edge-clearance','.25','--json',str(grade)];g=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=90);(OUT/(name+'-grade.log')).write_text(g.stdout+'\nSTDERR\n'+g.stderr,encoding='utf8');row.update(grade_argv=args,grade_exit=g.returncode,grade=json.loads(grade.read_text()))
   args=[sys.executable,'-X','utf8',str(ROOT/'tests/stress/provenance_audit.py'),'--workdir',str(work),'--delivered',str(out)];a=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=90);row.update(audit_argv=args,audit_exit=a.returncode,audit_stdout=a.stdout)
  row.update(input_sha256=input_before,input_preserved=hashlib.sha256(src.read_bytes()).hexdigest()==input_before,intent_sha256=intent_before,intent_preserved=hashlib.sha256(intent.read_bytes()).hexdigest()==intent_before)
  if '--expect-refused' in sys.argv:
   assert r.returncode==4 and row['summary']['status']=='refused' and row['summary']['published'] is False and row['summary']['output'] is None
   assert row['output_before_sha256']==row['output_after_sha256'] and not row['ledger'] and row['input_preserved'] and row['intent_preserved']
  report['commands'].append(row);(OUT/'results.json').write_text(json.dumps(report,indent=2));print(name,r.returncode,'output',out.exists(),'ledger',len(row['ledger']),flush=True)
print(OUT)
