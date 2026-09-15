import argparse,hashlib,json,pathlib,subprocess,sys
p=argparse.ArgumentParser();p.add_argument('--repo',default='.');p.add_argument('--output',required=True);a=p.parse_args();repo=pathlib.Path(a.repo).resolve();out=pathlib.Path(a.output).resolve();out.mkdir(parents=True,exist_ok=True)
result={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),'cases':[]}
def hash(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def run(name,args):
 cmd=[sys.executable,'-X','utf8']+list(map(str,args));r=subprocess.run(cmd,cwd=repo,capture_output=True,text=True,encoding='utf-8');(out/(name+'.log')).write_text(r.stdout+'\nSTDERR:\n'+r.stderr,encoding='utf-8'); return r,subprocess.list2cmdline(cmd)
work=out/'work';truth=out/'truth';r,cmd=run('stage',['tests/stress/stage_unaided.py',repo/'kicad_files/esp_prog.kicad_pcb',work,truth]);assert r.returncode==0;r0={str(p.relative_to(work)):hash(p) for p in work.rglob('*') if p.is_file()}
for name,ops,reason in [('dry_run',['--dry-run','set','R1','136.4','98.8','--rot','270'],None),('offboard',['set','R1','0','0'],'WORSE'),('strict_improving',['--strict-legal','set','R1','136.4','98.8','--rot','270'],'not clean'),('multi_invalid',['set','R1','136.4','98.8','--rot','270','set','DOES_NOT_EXIST','136','99'],'not a footprint')]:
 target=work/(name+'.kicad_pcb');r,cmd=run(name,['py_placer/place_pose.py',work/'board.kicad_pcb',target]+ops);s=[json.loads(x.split(': ',1)[1]) for x in r.stdout.splitlines() if x.startswith('JSON_SUMMARY: ')];doc={'name':name,'command':cmd,'exit':r.returncode,'summary':s[-1] if s else None,'output_exists':target.exists(),'initial_files_preserved':all(hash(work/n)==h for n,h in r0.items()),'ledger_exists':(work/'.pose-provenance.jsonl').exists()};result['cases'].append(doc)
 assert not target.exists(),doc
 assert doc['initial_files_preserved'],doc
 assert not doc['ledger_exists'],doc
 if reason: assert r.returncode!=0 and reason.lower() in (r.stdout+r.stderr).lower(),doc
 else: assert r.returncode==0 and doc['summary']['dry_run'],doc
(out/'results.json').write_text(json.dumps(result,indent=2,sort_keys=True),encoding='utf-8');print(json.dumps([{k:d[k] for k in ['name','exit','output_exists','initial_files_preserved','ledger_exists']}for d in result['cases']],indent=2))

