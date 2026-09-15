"""Independent native regression runs after reviewing the #961 legacy test migration."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys

p=argparse.ArgumentParser()
p.add_argument('--root',type=Path,required=True)
p.add_argument('--out',type=Path,required=True)
a=p.parse_args()
root,out=a.root.resolve(),a.out.resolve()
out.mkdir(parents=True,exist_ok=True)
tests=['test_961_connector_geometry.py','test_961_connector_publication.py',
       'test_549_floorplan_cli.py','test_549_floorplan_grade.py',
       'test_712_edge_centering.py','test_run23_connector_affinity.py',
       'test_run26_edge_seat_body_basis.py','test_run27_edge_seat_clears_placed.py',
       'test_run4_reconstruct.py','test_place_edge_containment.py']
def run(test):
    argv=[sys.executable,'-X','utf8',str(root/'tests'/test)]
    r=subprocess.run(argv,cwd=root,text=True,encoding='utf8',capture_output=True,timeout=600)
    (out/(test+'.log')).write_text(r.stdout+r.stderr,encoding='utf8')
    return {'test':test,'argv':argv,'cwd':str(root),'exit':r.returncode,
            'log':test+'.log','traceback':'Traceback' in r.stdout+r.stderr}
with ThreadPoolExecutor(max_workers=3) as pool:
    rows=list(pool.map(run,tests))
revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
(out/'results.json').write_text(json.dumps({'revision':revision,'tests':rows},indent=2)+'\n',encoding='utf8')
print(json.dumps(rows,indent=2))
assert all(r['exit']==0 and not r['traceback'] for r in rows)
