"""Record a reviewer-confirmed documentation-only publication identity."""
import json,subprocess,sys
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'py_placer').is_dir());behavior,final=sys.argv[1:3]
paths=['py_placer','py_router','py_tools','kicad_routing_plugin','rust_router','tests','kicad_files']
def git(*a):return subprocess.check_output(['git',*a],cwd=ROOT,text=True).strip()
rows=[]
for path in paths:
 a=git('rev-parse',behavior+':'+path);b=git('rev-parse',final+':'+path);rows.append({'path':path,'behavior_tree':a,'publication_tree':b,'identical':a==b})
result={'reviewer':'3 final placement/regression','behavior_revision':git('rev-parse',behavior),'publication_revision':git('rev-parse',final),'trees':rows,'production_test_fixture_identity':all(r['identical'] for r in rows),'changed_paths':git('diff','--name-only',behavior,final).splitlines()}
out=ROOT/'docs/issue-961-evidence/final'/'publication-identity.json';out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));assert result['production_test_fixture_identity']
