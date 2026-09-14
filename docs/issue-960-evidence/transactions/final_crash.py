"""A subprocess dies after actual final-board replacement in the real pose CLI."""
import hashlib,json,os,pathlib,subprocess,sys
ROOT=pathlib.Path.cwd();label=sys.argv[1] if len(sys.argv)>1 else 'final-crash';runtime=ROOT/'reviewer-runtime'/label;work=runtime/'work';truth=runtime/'truth';runtime.mkdir(parents=True,exist_ok=True)
def cli(args):
    r=subprocess.run([sys.executable,'-X','utf8',*map(str,args)],capture_output=True,text=True);return dict(command=subprocess.list2cmdline([sys.executable,'-X','utf8',*map(str,args)]),code=r.returncode,stdout=r.stdout,stderr=r.stderr)
stage=cli(['tests/stress/stage_unaided.py','kicad_files/esp_prog.kicad_pcb',work,truth]);assert stage['code']==0,stage
out=work/'pose.kicad_pcb';normal=cli(['py_placer/place_pose.py',work/'board.kicad_pcb',out,'set','R1','136.4','98.8','--rot','270']);assert normal['code']==0,normal
before=hashlib.sha256(out.read_bytes()).hexdigest()
worker=runtime/'crash_worker.py';worker.write_text('''import os,pathlib,runpy,sys
root=pathlib.Path.cwd()
for p in ('py_placer','py_router','py_tools'):sys.path.insert(0,str(root/p))
target=sys.argv[1];original=os.replace
def crashing(src,dst):
    original(src,dst)
    if os.path.normcase(os.path.abspath(dst))==os.path.normcase(os.path.abspath(target)):
        os._exit(91)
os.replace=crashing
sys.argv=['py_placer/place_pose.py',target,target,'lock','R1']
runpy.run_path('py_placer/place_pose.py',run_name='__main__')
''')
crash=cli([worker,out]);after=hashlib.sha256(out.read_bytes()).hexdigest();audit=cli(['tests/stress/provenance_audit.py','--workdir',work,'--delivered',out]);retry=cli(['py_placer/place_pose.py',out,out,'unlock','R1'])
recovery=[]
for p in work.rglob('*'):
    if p.is_file() and ('krt-backup' in p.name or p.name=='journal.json'):recovery.append(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),content=p.read_text() if p.name=='journal.json' else None))
result=dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),stage=stage,normal=normal,crash=crash,before=before,after=after,audit=audit,retry=retry,recovery=recovery)
(ROOT/'docs/issue-960-evidence/transactions'/f'{label}.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(crash_code=crash['code'],bytes_changed=before!=after,audit_code=audit['code'],retry_code=retry['code'],retained=len(recovery))))
assert crash['code']==91 and before!=after and audit['code']==5 and 'recovery' in audit['stdout'].lower() and retry['code']!=0 and 'recovery' in (retry['stdout']+retry['stderr']).lower() and recovery
