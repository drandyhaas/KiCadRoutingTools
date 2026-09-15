"""Check final help text, actual native CLI tests and explicit native absence."""
import json
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parent
out=root/'reviewer-evidence'
rows=[]
for label,cmd,expected in [
    ('native_publication',[sys.executable,'-X','utf8',str(root/'tests/test_961_connector_publication.py')],0),
    ('no_native_publication',['C:/Python313/python.exe','-X','utf8',str(root/'tests/test_961_connector_publication.py')],77),
    ('floorplan_help',[sys.executable,'-X','utf8',str(root/'py_tools/check_floorplan.py'),'--help'],0)]:
    r=subprocess.run(cmd,cwd=root,capture_output=True,text=True,encoding='utf-8',timeout=180)
    (out/(label+'.stdout.txt')).write_text(r.stdout,encoding='utf-8')
    (out/(label+'.stderr.txt')).write_text(r.stderr,encoding='utf-8')
    assert r.returncode==expected,(label,r.returncode,r.stdout,r.stderr)
    if label=='no_native_publication':
        assert 'SKIP: native KiCad pcbnew is required' in r.stdout and 'Traceback' not in r.stderr
    elif label=='native_publication':
        assert 'OK' in r.stderr and 'SKIP' not in r.stdout
    else:
        text=' '.join(r.stdout.split())
        assert 'independent copper and geometry requirements can still fail' in text,text
        assert 'class-only entries impose no edge, seating or maximum overhang requirement' in text,text
    rows.append(dict(label=label,argv=cmd,exit_code=r.returncode))
    print(label,'exit',r.returncode)
revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
(out/'portability_and_help.json').write_text(json.dumps(dict(revision=revision,commands=rows),indent=2),encoding='utf-8')
