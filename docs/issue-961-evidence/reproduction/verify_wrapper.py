"""Independent real-CLI connector publication controls on fixed public geometry."""
import argparse
import hashlib
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
sys.path[:0]=[str(root/x) for x in ('py_router','py_placer')]
import pcbnew
from copy_board import copy_board,SIBLING_EXTS
from kicad_parser import parse_kicad_pcb
from placement.seeder import stamp_locked
from placement.floorplan import KIND,SCHEMA_VERSION

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(p,x):p.write_text(json.dumps(x,indent=2,default=str)+'\n',encoding='utf8')
def poses(path):
    b=pcbnew.LoadBoard(str(path))
    return {f.m_Uuid.AsString():{'ref':f.GetReference(),'x':pcbnew.ToMM(f.GetPosition().x),
       'y':pcbnew.ToMM(f.GetPosition().y),'rotation':f.GetOrientationDegrees(),
       'face':f.GetLayerName(),'locked':f.IsLocked()} for f in b.GetFootprints()}
def family(p):return {p.name:sha(p),**{p.with_suffix(e).name:sha(p.with_suffix(e))
        for e in SIBLING_EXTS if p.with_suffix(e).exists()}}

source=root/'kicad_files/esp_prog.kicad_pcb'
input=out/'fixed.kicad_pcb'
copy_board(str(source),str(input))
pcb=parse_kicad_pcb(str(input))
stamp_locked(str(input),set(pcb.footprints))
native=poses(input)
assert len(native)==21 and all(f['locked'] for f in native.values())
input_id=family(input)
intents={}
for name,lo in [('zero',0.),('positive',.05)]:
    ip=out/(name+'.intent.json')
    dump(ip,{'schema':SCHEMA_VERSION,'kind':KIND,'units':'mm','edge_connectors':[
       {'ref':'USB1','edge':'west','overhang_mm':{'min':lo,'max':.20},'max_setback_mm':0.}]})
    intents[name]=ip
rows=[]
for tool,flags in [('seed',['--repair']),('reconstruct',['--stages','classify'])]:
    for name in intents:
        for dry in (False,True):
            tag=f'{tool}-{name}-'+('dry' if dry else 'write')
            dest=out/(tag+'.kicad_pcb')
            copy_board(str(input),str(dest))
            prior=family(dest)
            argv=[sys.executable,'-X','utf8',str(root/f'py_placer/place_{tool}.py'),
                str(input),str(dest),'--intent',str(intents[name]),'--clearance','.25',
                '--board-edge-clearance','.25',*flags,*(['--dry-run'] if dry else [])]
            r=subprocess.run(argv,cwd=root,text=True,encoding='utf8',capture_output=True,timeout=120)
            (out/(tag+'.log')).write_text(r.stdout+r.stderr,encoding='utf8')
            assert 'Traceback' not in r.stdout+r.stderr,(tag,r.stderr)
            docs=[json.loads(line.split(': ',1)[1]) for line in r.stdout.splitlines()
                  if line.startswith('JSON_SUMMARY: ')]
            assert len(docs)==1,(tag,docs,r.stdout)
            s=docs[0]
            if dry:
                assert not s['published'] and s['output'] is None,(tag,s)
                assert s['engineering_clean'] is False and s['complete'] is False,(tag,s)
                assert s['connector_requirements']['accepted'] is None,(tag,s)
                assert s['connector_requirements']['complete'] is False,(tag,s)
                assert s['status'] in ('dry_run','refused'),(tag,s)
                assert family(dest)==prior,tag
            elif name=='positive':
                assert r.returncode==4 and s['status']=='refused',(tag,s)
                assert not s['published'] and s['output'] is None,(tag,s)
                assert s['connector_requirements']['accepted'] is False,(tag,s)
                assert 'body overhang' in str(s['connector_requirements']['errors']),s
                assert family(dest)==prior,tag
            else:
                assert s['published'] and s['connector_requirements']['accepted'],(tag,s)
                assert s['engineering_clean'] is False,(tag,s)
                assert poses(dest)==native,(tag,poses(dest),native)
                if r.returncode:
                    assert s['status']=='exploratory',(tag,s)
            assert family(input)==input_id,tag
            rows.append({'case':tag,'argv':argv,'cwd':str(root),'exit':r.returncode,
                         'summary':s,'destination_before':prior,'destination_after':family(dest),
                         'native_output':poses(dest),'pass':True})
            dump(out/'results.json',{'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
                'original_source_sha256':sha(source),'fixed_input_identity':input_id,
                'intent_sha256':{k:sha(v) for k,v in intents.items()},'rows':rows})
print(f'{len(rows)} real CLI publication controls passed')
