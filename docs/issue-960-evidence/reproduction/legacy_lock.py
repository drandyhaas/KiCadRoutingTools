import hashlib,json,pathlib,subprocess,sys
repo=pathlib.Path('.').resolve();out=pathlib.Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=True);expect=sys.argv[2]
for d in ['py_placer','py_router','py_tools']:sys.path.insert(0,str(repo/d))
import pcbnew
result={'revision':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'commands':[]}
def run(name,args):
 cmd=[sys.executable,'-X','utf8']+list(map(str,args));r=subprocess.run(cmd,cwd=repo,capture_output=True,text=True,encoding='utf-8');(out/(name+'.log')).write_text(r.stdout+'\nSTDERR:\n'+r.stderr,encoding='utf-8');result['commands'].append({'name':name,'command':subprocess.list2cmdline(cmd),'exit':r.returncode});return r
work=out/'work';truth=out/'truth';r=run('stage',['tests/stress/stage_unaided.py',repo/'kicad_files/esp_prog.kicad_pcb',work,truth]);assert r.returncode==0
board=work/'board.kicad_pcb';output=work/'pose.kicad_pcb';r=run('pose',['py_placer/place_pose.py',board,output,'set','R1','136.4','98.8','--rot','270']);assert r.returncode==0
ledger=work/'.pose-provenance.jsonl';row=json.loads(ledger.read_text().strip());result['original_row']=row.copy()
# Deliberately downgrade to historical schema-1 rows without lock/final-snapshot claims.
for key in ['locks_written','final_snapshot','candidate_sha256']:row.pop(key,None)
ledger.write_text(json.dumps(row)+'\n',encoding='utf-8');b=pcbnew.LoadBoard(str(output));f=next(x for x in b.GetFootprints() if x.GetReference()=='R1');f.SetLocked(True);pcbnew.SaveBoard(str(output),b);b=pcbnew.LoadBoard(str(output));f=next(x for x in b.GetFootprints() if x.GetReference()=='R1');assert f.IsLocked();result['legacy_row']=row;result['native_final']={'pose':[pcbnew.ToMM(f.GetPosition().x),pcbnew.ToMM(f.GetPosition().y),f.GetOrientationDegrees()%360],'locked':f.IsLocked(),'sha256':hashlib.sha256(output.read_bytes()).hexdigest()}
r=run('audit',['tests/stress/provenance_audit.py','--workdir',work,'--delivered',output,'--json',out/'audit.json']);audit=json.loads((out/'audit.json').read_text());result['audit']=audit;(out/'results.json').write_text(json.dumps(result,indent=2,sort_keys=True),encoding='utf-8');assert audit['verdict']==expect,audit;print(json.dumps({'revision':result['revision'],'verdict':audit['verdict'],'reason':audit['reason'],'exit':r.returncode},indent=2))

