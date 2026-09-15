import hashlib,json,pathlib,subprocess,sys
repo=pathlib.Path('.').resolve();out=pathlib.Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=True)
for d in ['py_placer','py_router','py_tools']:sys.path.insert(0,str(repo/d))
import pcbnew
from placement import provenance as pv
result={'revision':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'commands':[]}
def sha(p):return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
def run(name,args):
 cmd=[sys.executable,'-X','utf8']+list(map(str,args));r=subprocess.run(cmd,cwd=repo,capture_output=True,text=True,encoding='utf-8');(out/(name+'.log')).write_text(r.stdout+'\nSTDERR:\n'+r.stderr,encoding='utf-8');result['commands'].append({'name':name,'command':subprocess.list2cmdline(cmd),'exit':r.returncode});return r
work=out/'work';truth=out/'truth';r=run('stage',['tests/stress/stage_unaided.py',repo/'kicad_files/esp_prog.kicad_pcb',work,truth]);assert r.returncode==0
intent=out/'intent.json';intent.write_text(json.dumps({'schema':1,'kind':'floorplan-intent','units':'mm','must_lock':['R1']}),encoding='utf-8');board=work/'board.kicad_pcb';output=work/'seed.kicad_pcb';baseline=sha(board);r=run('seed',['py_placer/place_seed.py',board,output,'--intent',intent,'--seed','0','--no-polish']);assert output.exists(),r.stdout+r.stderr
result['seed_summary']=json.loads(next(x.split(': ',1)[1] for x in r.stdout.splitlines() if x.startswith('JSON_SUMMARY: ')));rows=pv.read_ledger(str(work));b=pcbnew.LoadBoard(str(output));f=next(x for x in b.GetFootprints() if x.GetReference()=='R1');native={'pose':[pcbnew.ToMM(f.GetPosition().x),pcbnew.ToMM(f.GetPosition().y),f.GetOrientationDegrees()%360],'locked':f.IsLocked(),'sha256':sha(output)};assert native['locked'] and rows[-1]['locks_written']['R1'];assert rows[-1]['board_sha256']==native['sha256'];assert rows[-1]['poses_written']['R1']==native['pose'];assert rows[-1]['lever']=='place_seed.py';assert sha(board)==baseline;result.update(native=native,ledger=rows,baseline_sha256=baseline,intent_sha256=sha(intent),source_sha256=sha(repo/'kicad_files/esp_prog.kicad_pcb'));r=run('audit',['tests/stress/provenance_audit.py','--workdir',work,'--delivered',output,'--json',out/'audit.json']);assert r.returncode==0,r.stdout+r.stderr;(out/'results.json').write_text(json.dumps(result,indent=2,sort_keys=True),encoding='utf-8');print(json.dumps({'native':native,'ledger_rows':len(rows),'seed_summary':result['seed_summary'],'commands':result['commands']},indent=2))

