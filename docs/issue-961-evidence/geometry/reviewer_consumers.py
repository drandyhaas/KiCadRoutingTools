"""Independent consumer parity checks against native-written geometry."""
import contextlib
import io
import json
from pathlib import Path
import sys
import hashlib

import reviewer_geometry as g
from kicad_parser import parse_kicad_pcb
from placement import floorplan, seeder
from pose_score import make_state

rows = []
for pad_x,netted in ((0,True),(-1.2,True),(-1.5,True),(-1.2,False),(-1.5,False)):
    path, pts, angle, side = g.fixture('consumer_'+str(pad_x).replace('.','p')+'_'+str(netted),
        [(-2,-2),(2,-2),(2,2),(-2,2)], position=(1.9,10),pad_x=pad_x,netted=netted,courtyard=True)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(str(path))
        state = make_state(pcb,str(path),clearance=.25,board_edge_clearance=.55)
        part = state.parts['J1']
        reasons = []
        accepted = seeder.edge_seat_ok(state,part,part.x,part.y,'west',.05,.2,reasons=reasons)
        intent = floorplan.intent_from_dict(dict(schema=floorplan.SCHEMA_VERSION,kind=floorplan.KIND,units='mm',edge_connectors=[dict(ref='J1',edge='west',overhang_mm=dict(min=.05,max=.2))]))
        contexts=[]
        real_ctx = floorplan._Ctx
        class Capture(real_ctx):
            def __init__(self,*a,**kw):
                super().__init__(*a,**kw);contexts.append(self)
        floorplan._Ctx=Capture
        try:
            r = floorplan.grade(intent,pcb,str(path),clearance=.25,board_edge_clearance=.55)
        finally:
            floorplan._Ctx=real_ctx
        exemptions=contexts[0].oob_exempt()
        corrected=seeder._edge_correct(state,'J1','west',6,10,.1)
        emitted = floorplan.emit_intent(pcb,str(path),declare_classes=True)
    row = dict(pad_x=pad_x,netted=netted,fixture_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),native_gap=1.9+pad_x-.25,edge_seat_ok=accepted,reasons=reasons,
               edge_seating=r.edge_seating,violations=[v.message for v in r.violations],
               emitted_connectors=emitted['edge_connectors'],exemptions=exemptions,corrected=corrected)
    rows.append(row)
    print('pad',pad_x,'netted',netted,'gap',row['native_gap'],'seat',accepted)
    assert accepted == (row['native_gap'] >= .55),row
    assert abs(exemptions['J1']-.1)<1e-6,row
    assert abs(corrected[0]-1.9)<1e-6 and corrected[2],row
    assert abs(r.edge_seating[0]['body_overhang_mm']-.1)<1e-6,row
    assert r.edge_seating[0]['measurements']['body_overhang']['disposition']=='pass',row
    assert r.edge_seating[0]['measurements']['pad_copper_edge_gap']['disposition']==('pass' if accepted else 'fail'),row
    assert len(emitted['edge_connectors'])==1 and abs(emitted['edge_connectors'][0]['observed_overhang_mm']-.1)<1e-6,row
    assert emitted['edge_connectors'][0]['overhang_mm']==dict(min=0,max=.6),row
(g.OUT/'consumers.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')

# The body-origin offset at arbitrary rotation is verified on a new actual
# KiCad file, not only by calling the production measurement a second time.
path,*_=g.fixture('rotation_start',[(-2,-2),(2,-2),(2,2),(-2,2)],position=(6,10),rotation=37,netted=True)
with contextlib.redirect_stdout(io.StringIO()):
    pcb=parse_kicad_pcb(str(path)); state=make_state(pcb,str(path))
    corrected=seeder._edge_correct(state,'J1','west',6,10,.1)
path,pts,angle,side=g.fixture('rotation_corrected',[(-2,-2),(2,-2),(2,2),(-2,2)],position=corrected[:2],rotation=37,netted=True)
native_overhang=max(0,-min(x for x,y in pts))
assert corrected[2] and abs(native_overhang-.1)<2e-6,(corrected,native_overhang)
(g.OUT/'corrected_rotation.json').write_text(json.dumps(dict(corrected=corrected,native_overhang=native_overhang,native_angle=angle,native_face=side,fixture_sha256=hashlib.sha256(path.read_bytes()).hexdigest()),indent=2),encoding='utf-8')
print('Native written rotation correction:',corrected,native_overhang)

# A courtyard allowance never supplies a positive physical body overhang.
exemption_rows=[]
for label,position,minimum,maximum in [('inboard_positive',(3,10),.05,.2),('outside_band',(1.7,10),.05,.2),('two_edges',(1.9,1.9),.05,.2)]:
    path,*_=g.fixture('exempt_'+label,[(-2,-2),(2,-2),(2,2),(-2,2)],position=position,netted=True,courtyard=True)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb=parse_kicad_pcb(str(path))
        intent=floorplan.intent_from_dict(dict(schema=floorplan.SCHEMA_VERSION,kind=floorplan.KIND,units='mm',edge_connectors=[dict(ref='J1',edge='west',overhang_mm=dict(min=minimum,max=maximum))]))
        contexts=[];floorplan._Ctx=Capture
        try: r=floorplan.grade(intent,pcb,str(path))
        finally: floorplan._Ctx=real_ctx
        ex=contexts[0].oob_exempt()
    assert ex=={},(label,ex)
    assert r.violations,(label,r)
    exemption_rows.append(dict(label=label,exemptions=ex,body_overhang_mm=r.edge_seating[0]['body_overhang_mm'],violations=[v.message for v in r.violations]))
(g.OUT/'exemption_controls.json').write_text(json.dumps(exemption_rows,indent=2),encoding='utf-8')
print('Consumer assertions passed: 48; exemption refusal cases:',len(exemption_rows))

optional_rows=[]
for label,entry,points in [
    ('nonbinding_class',dict(ref='J1',**{'class':'edge_receptacle'},source='auto-class',overhang_mm=dict(min=0)),[]),
    ('edge_only_missing',dict(ref='J1',edge='west'),[]),
    ('along_only',dict(ref='J1',edge='west',center_on_edge=dict(tolerance_mm=.5)),[(-2,-2),(2,-2),(2,2),(-2,2)])]:
    path,*_=g.fixture(label,points,position=(3,10),netted=True)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb=parse_kicad_pcb(str(path))
        intent=floorplan.intent_from_dict(dict(schema=floorplan.SCHEMA_VERSION,kind=floorplan.KIND,units='mm',edge_connectors=[entry]))
        r=floorplan.grade(intent,pcb,str(path))
    optional_rows.append(dict(label=label,passed=r.passed,complete=r.complete,not_graded=r.not_graded,edge_seating=r.edge_seating,violations=[v.message for v in r.violations]))
    print(label,'passed',r.passed,'complete',r.complete,'declared',r.edge_seating[0]['declared'])
    if label=='nonbinding_class':
        assert r.passed and r.complete and r.edge_seating[0]['body_overhang_mm'] is None
    elif label=='edge_only_missing':
        assert not r.passed and not r.complete and r.edge_seating[0]['body_overhang_mm'] is None
    else:
        assert r.passed and r.complete and r.edge_seating[0]['declared']
(g.OUT/'optional_claims.json').write_text(json.dumps(optional_rows,indent=2),encoding='utf-8')
