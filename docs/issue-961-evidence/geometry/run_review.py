"""Capture reproducible independent review commands and exact file identities."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parent
code=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else root
out=root/'reviewer-evidence';out.mkdir(exist_ok=True)
commands=[]
for script in ('reviewer_geometry.py','reviewer_consumers.py','reviewer_candidate_copper.py','reviewer_declarations.py','reviewer_publication.py'):
    cmd=[sys.executable,'-X','utf8',str(root/script),str(code)]
    r=subprocess.run(cmd,cwd=root,capture_output=True,text=True,encoding='utf-8')
    (out/(script+'.stdout.txt')).write_text(r.stdout,encoding='utf-8')
    (out/(script+'.stderr.txt')).write_text(r.stderr,encoding='utf-8')
    commands.append(dict(argv=cmd,exit_code=r.returncode))
    print(script,'exit',r.returncode,r.stdout.splitlines()[-1] if r.stdout else '')
    if r.returncode: print(r.stderr[-3000:]);raise SystemExit(r.returncode)
identities={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file() and p.suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru','.json')}
def git(*args): return subprocess.check_output(['git',*args],cwd=code,text=True).strip()
report=dict(revision=git('rev-parse','HEAD'),status=git('status','--short'),trees={p:git('rev-parse','HEAD:'+p) for p in ('py_placer','py_router','py_tools','tests','kicad_files')},commands=commands,file_sha256=identities)
(out/'run_identity.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
