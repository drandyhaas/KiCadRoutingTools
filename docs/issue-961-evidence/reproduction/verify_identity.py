"""Pin final production/test/fixture identities to independently tested revisions."""
import argparse
import json
from pathlib import Path
import subprocess

p=argparse.ArgumentParser()
p.add_argument('--root',type=Path,required=True)
p.add_argument('--revision',required=True)
p.add_argument('--test-revision',required=True)
p.add_argument('--cli-revision',default='9e65e44b0cb1bc7ab2b8656a48f4848ddbc38100')
p.add_argument('--out',type=Path,required=True)
a=p.parse_args()
root=a.root.resolve()
measurement='bc8071b9ab586dd78629cdf511e4d527b4af33a1'
publication='e182bbfe4c3ec19ad65999de9ff023dc84035128'
def git(*args):return subprocess.check_output(['git',*args],cwd=root,text=True).strip()
def identity(rev,path):return git('rev-parse',rev+':'+path)
measurement_paths=['py_placer/placement/floorplan.py','py_placer/placement/connector_geometry.py',
 'py_placer/placement/legality.py','py_placer/placement/quench.py','py_placer/placement/seeder.py',
 'py_placer/placement/reconstruct.py','py_placer/placement/writer.py','py_placer/placement/publication.py',
 'py_router','kicad_routing_plugin','kicad_files']
publication_paths=['py_placer/place_seed.py','py_placer/place_reconstruct.py',
                   'py_placer/placement/connector_publication.py']
rows=[]
for reference,paths in [(measurement,measurement_paths),(publication,publication_paths),
                         (a.cli_revision,['py_tools']),
                         (a.test_revision,['tests'])]:
    for path in paths:
        old,new=identity(reference,path),identity(a.revision,path)
        assert old==new,(reference,a.revision,path,old,new)
        rows.append({'path':path,'tested_revision':reference,'final_revision':a.revision,
                     'tested_blob_or_tree':old,'final_blob_or_tree':new,'identical':True})
changed=git('diff','--name-only',publication,a.revision).splitlines()
assert all(path.startswith(('docs/','tests/')) or path=='py_tools/check_floorplan.py'
           for path in changed),changed
a.out.write_text(json.dumps({'revision':a.revision,'measurement_behavior_revision':measurement,
    'publication_behavior_revision':publication,'test_reference':a.test_revision,
    'cli_help_behavior_reference':a.cli_revision,
    'changed_from_publication_behavior':changed,'all_identical':True,'identities':rows},indent=2)+'\n',encoding='utf8')
print(f'{len(rows)} production/test/fixture identities confirmed')
