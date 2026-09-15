"""Independent final-candidate wrapper controls; real CLI ownership is reviewer 3."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
import reviewer_geometry as g
from placement import floorplan
from placement.connector_publication import run_checked
from copy_board import copy_board
import pcbnew

rows=[]
for name,points,pad_x,position,dry,rc in [
    ('accepted',[(-2,-2),(2,-2),(2,2),(-2,2)],0,(1.9,10),False,0),
    ('body_rejected',[(-2,-2),(2,-2),(2,2),(-2,2)],0,(1.7,10),False,0),
    ('copper_rejected',[(-2,-2),(2,-2),(2,2),(-2,2)],-1.2,(1.9,10),False,0),
    ('body_unmeasured',[],0,(1.9,10),False,0),
    ('dry',[(-2,-2),(2,-2),(2,2),(-2,2)],0,(1.9,10),True,0),
    ('unwritten',[(-2,-2),(2,-2),(2,2),(-2,2)],0,(1.9,10),False,0),
    ('exploratory',[(-2,-2),(2,-2),(2,2),(-2,2)],0,(1.9,10),False,4),
]:
    src,*_=g.fixture('publication_'+name,points,position=position,pad_x=pad_x,netted=True)
    ip=g.OUT/('publication_'+name+'.intent.json')
    ip.write_text(json.dumps(dict(schema=floorplan.SCHEMA_VERSION,kind=floorplan.KIND,units='mm',edge_connectors=[dict(ref='J1',edge='west',overhang_mm=dict(min=.05,max=.2))])),encoding='utf-8')
    destination=g.OUT/('publication_'+name+'.published.kicad_pcb')
    destination.write_bytes(b'existing destination control\n')
    before=hashlib.sha256(destination.read_bytes()).hexdigest()
    intent_before=hashlib.sha256(ip.read_bytes()).hexdigest()
    args=SimpleNamespace(intent=str(ip),input_file=str(src),output_file=str(destination),dry_run=dry,clearance=.25,board_edge_clearance=.55)
    def execute(trial):
        if name!='unwritten': copy_board(str(src),trial.output_file)
        print('JSON_SUMMARY: '+json.dumps(dict(output=trial.output_file,status='ok',complete=True)))
        return rc
    capture=io.StringIO()
    with contextlib.redirect_stdout(capture):
        actual_rc=run_checked(args,execute)
    text=capture.getvalue()
    summaries=[json.loads(line.split(': ',1)[1]) for line in text.splitlines() if line.startswith('JSON_SUMMARY: ')]
    assert len(summaries)==1,(name,text)
    summary=summaries[0]
    expected_publish=name in ('accepted','exploratory')
    assert summary['published']==expected_publish,(name,summary)
    assert hashlib.sha256(ip.read_bytes()).hexdigest()==intent_before,name
    after=hashlib.sha256(destination.read_bytes()).hexdigest()
    native_pose=None
    if expected_publish:
        assert before!=after and summary['engineering_clean'] is False,(name,summary)
        native=pcbnew.LoadBoard(str(destination));f=next(iter(native.GetFootprints()))
        native_pose=[f.GetPosition().x/1e6,f.GetPosition().y/1e6,f.GetOrientationDegrees(),f.GetLayerName()]
        assert native_pose[:2]==list(position),(name,native_pose)
        assert summary['status']==('exploratory' if rc else 'ok'),summary
    else:
        assert before==after,(name,summary)
        assert summary['output'] is None,(name,summary)
        if dry or name=='unwritten':
            assert summary['engineering_clean'] is False and summary['complete'] is False,(name,summary)
            assert summary['status']==('dry_run' if dry else 'refused'),(name,summary)
            assert summary['connector_requirements']=={'accepted':None,'complete':False,'reason':'no written final candidate to measure'},(name,summary)
        else:
            assert actual_rc==4 and summary['status']=='refused',(name,actual_rc,summary)
            assert summary['connector_requirements']['accepted'] is False,(name,summary)
    rows.append(dict(name=name,exit_code=actual_rc,summary=summary,input_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),intent_sha256=intent_before,
                     output_before_sha256=before,output_after_sha256=after,native_written_pose=native_pose))
(g.OUT/'publication_controls.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
print('Independent wrapper native-candidate publication controls passed:',len(rows))
