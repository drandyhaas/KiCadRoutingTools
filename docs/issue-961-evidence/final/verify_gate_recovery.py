"""Fault injection at real seed/reconstruct entrypoints; native writes plus audited recovery."""
import contextlib,hashlib,importlib,io,json,os,subprocess,sys,time
from pathlib import Path
from unittest.mock import patch
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'py_placer').is_dir());sys.path[:0]=[str(ROOT/'py_router'),str(ROOT/'py_placer'),str(ROOT/'tests/stress')]
from placement import provenance as pv
from copy_board import copy_board
OUT=ROOT/'docs/issue-961-evidence/final'/('recovery-'+time.strftime('%Y%m%d-%H%M%S'));OUT.mkdir(parents=True)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
report={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'production_files':{p:sha(ROOT/p) for p in ['py_placer/placement/connector_publication.py','py_placer/place_seed.py','py_placer/place_reconstruct.py']},'cases':[]}
for tool,extra in [('place_seed',['--repair']),('place_reconstruct',['--stages','classify'])]:
 for case in ('partial','committed-cleanup','undeclared'):
  work=OUT/(tool+'-'+case);work.mkdir();src=work/'board.kicad_pcb';copy_board(str(ROOT/'kicad_files/esp_prog.kicad_pcb'),str(src));src.with_suffix('.kicad_pro').write_text('{"net_settings":{"classes":[{"name":"Default","clearance":0.2}]},"new":true}');out=work/'out.kicad_pcb';copy_board(str(src),str(out));pro=out.with_suffix('.kicad_pro');pro.write_text('{"old":true}');before={str(p):sha(p) for p in (src,out,pro)};pv.start_regime(str(work),str(src));intent=work/'intent.json';intent.write_text(json.dumps({'schema':1,'kind':'floorplan-intent','units':'mm','edge_connectors':[{'ref':'USB1','edge':'west','overhang_mm':{'min':0,'max':.65},'max_setback_mm':.1}]}))
  replace,remove=os.replace,os.remove
  def fail_replace(a,b):
   if case=='partial' and Path(b)==out:raise OSError('reviewer injected destination board failure')
   if case=='partial' and Path(b)==pro and Path(a).name.startswith('.krt-backup-'):raise OSError('reviewer injected project restoration denial')
   return replace(a,b)
  def fail_remove(p,*a,**kw):
   if case=='committed-cleanup' and Path(p).name=='journal.json' and work in Path(p).parents:raise OSError('reviewer injected postcommit journal cleanup denial')
   return remove(p,*a,**kw)
  argv=[tool+'.py',str(src),str(out),'--intent',str(intent),'--clearance','.2','--board-edge-clearance','.25',*extra];stream=io.StringIO();err=io.StringIO();module=importlib.import_module(tool)
  with contextlib.ExitStack() as stack:
   if case!='undeclared':stack.enter_context(pv.declare_lever(tool+'.py',argv))
   stack.enter_context(patch.object(sys,'argv',argv));stack.enter_context(patch('os.replace',fail_replace));stack.enter_context(patch('os.remove',fail_remove));stack.enter_context(contextlib.redirect_stdout(stream));stack.enter_context(contextlib.redirect_stderr(err));rc=module.main()
  output=stream.getvalue();(OUT/(tool+'-'+case+'.log')).write_text(output+'\nSTDERR\n'+err.getvalue());summaries=[json.loads(t.split(': ',1)[1]) for t in output.splitlines() if t.startswith('JSON_SUMMARY: ')];assert len(summaries)==1,(case,output);summary=summaries[0];rows=pv.read_ledger(str(work));args=[sys.executable,'-X','utf8',str(ROOT/'tests/stress/provenance_audit.py'),'--workdir',str(work),'--delivered',str(out)];audit=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=90)
  row={'entrypoint':tool+'.main','argv':argv,'case':case,'exit':rc,'summary':summary,'before':before,'after':{str(p):sha(p) for p in (src,out,pro)},'rows':rows,'audit_argv':args,'audit_exit':audit.returncode,'audit_stdout':audit.stdout,'backups':{str(p):sha(p) for p in work.glob('.krt-backup-*')}};report['cases'].append(row);(OUT/'results.json').write_text(json.dumps(report,indent=2));assert rc==4 and summary['status']=='error' and not summary['complete'];assert sha(src)==before[str(src)]
  if case=='partial':assert summary['output_state']=='partial' and not summary['published'] and summary['output']==str(out) and not rows and audit.returncode==5 and sha(out)==before[str(out)] and sha(pro)!=before[str(pro)] and before[str(pro)] in row['backups'].values()
  if case=='committed-cleanup':assert summary['output_state']=='committed' and summary['published'] and len(rows)==1 and rows[-1]['board_sha256']==sha(out) and audit.returncode==5
  if case=='undeclared':assert summary['output_state']=='unchanged' and not summary['published'] and not rows and before==row['after']
  print(tool,case,'PASS',flush=True)
print(OUT)
