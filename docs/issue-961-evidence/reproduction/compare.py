"""Independent final assertions, using native fixture geometry as the oracle."""
import argparse
import json
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('--before',type=Path,required=True)
p.add_argument('--after',type=Path,required=True)
p.add_argument('--seating',type=Path,required=True)
p.add_argument('--seating-before',type=Path,required=True)
p.add_argument('--out',type=Path,required=True)
a=p.parse_args()
before=json.loads(a.before.read_text(encoding='utf8'))
after=json.loads(a.after.read_text(encoding='utf8'))
seating=json.loads(a.seating.read_text(encoding='utf8'))
seating_before=json.loads(a.seating_before.read_text(encoding='utf8'))
checks=[]
def check(name,condition):
    checks.append({'name':name,'pass':bool(condition)})
    assert condition,name
def near(x,y): return isinstance(x,(float,int)) and abs(x-y)<1e-6
check('same source and requirement siblings',before['source_identity']==after['source_identity'])
bands={'positive':(.05,.20),'zero':(0.,.65),'legal_body':(1.4,1.5)}
for band in bands:
    name=band+'.intent.json'
    check(name+' declaration bytes preserved',
          (a.before.parent/name).read_bytes()==(a.after.parent/name).read_bytes())
check('supplementary inboard board identity',seating_before['board_sha256']==seating['board_sha256'])
check('supplementary legal body board identity',
      seating_before['legal_control']['board_sha256']==seating['legal_control']['board_sha256'])
for ip in a.seating_before.parent.glob('*.intent.json'):
    check(ip.name+' declaration bytes preserved',ip.read_bytes()==(a.seating.parent/ip.name).read_bytes())
for bv,av in zip(before['variants'],after['variants']):
    name=av['name']
    check(name+' same written fixture bytes',bv['board_sha256']==av['board_sha256'])
    check(name+' native readback unchanged',bv['native_pose']==av['native_pose'])
    amount=av['native_fab_overhang_west_mm']
    native=av['native_pad_bbox_mm']
    bounds=av['outline_bounds']
    native_gap=min(native[0]-bounds[0],native[1]-bounds[1],bounds[2]-native[2],bounds[3]-native[3])
    for idx,r in enumerate(av['matrix']):
        tag=f'{name}/{idx}/{r["band"]}'
        e=next(x for x in r['edge_seating'] if x['ref']=='USB1')
        check(tag+' physical body amount',near(e['body_overhang_mm'],amount))
        check(tag+' same verdict amount',near(e['overhang_mm'],amount))
        check(tag+' actual Fab basis',e['body_overhang_basis']=='F.Fab')
        check(tag+' body setback zero',near(e['body_setback_mm'],0.))
        check(tag+' signed position',near(e['body_signed_position_mm'],amount))
        check(tag+' minimum native copper gap',near(e['pad_copper_edge_gap_mm'],native_gap))
        check(tag+' declared-edge native copper gap',near(e['pad_copper_declared_edge_gap_mm'],av['native_pad_bbox_west_gap_mm']))
        check(tag+' independent required edge floor',near(e['required_copper_edge_gap_mm'],r['requested_edge_clearance_mm']))
        for key in ('body_overhang','body_setback','pad_copper_edge_gap','copper_edge_shortfall'):
            m=e['measurements'][key]
            check(tag+'/'+key+' full measurement contract',all(k in m for k in
                  ('value','units','geometry_basis','declared_limit','requirement_source','disposition'))
                  and m['units']=='mm' and bool(m['geometry_basis']) and bool(m['requirement_source']))
        lo,hi=bands[r['band']]
        wanted='pass' if lo-1e-6<=amount<=hi+1e-6 else 'fail'
        check(tag+' margin-independent mechanical disposition',e['measurements']['body_overhang']['disposition']==wanted)
        copper='pass' if native_gap>=r['requested_edge_clearance_mm']-1e-6 else 'fail'
        check(tag+' independent copper disposition',e['measurements']['pad_copper_edge_gap']['disposition']==copper)
        check(tag+' exact copper shortfall',near(e['copper_edge_shortfall_mm'],max(0.,r['requested_edge_clearance_mm']-native_gap)))
    check(name+' actual DRC unchanged',all(bv['drc'][e]['items']==av['drc'][e]['items']
                                          for e in bv['drc']))
for row in seating['rows']:
    tag='inboard/'+row['case']+'/'+str(row['clearance_and_edge_requested_mm'])
    check(tag+' native zero overhang',near(row['edge_seating'][0]['body_overhang_mm'],0.))
    check(tag+' native 1mm setback',near(row['edge_seating'][0]['body_setback_mm'],1.))
    expected=row['case'] in ('zero_no_class','zero_class_no_seating','seat_110')
    check(tag+' correct independent declared verdict',row['pass']==expected)
for row in seating['legal_control']['rows']:
    tag='legal130/'+str(row['requested_edge_mm'])
    e=row['grade']['edge_seating'][0]
    check(tag+' declared body permitted',e['measurements']['body_overhang']['disposition']=='pass')
    count=row['drc']['by_type'].get('pad-board-edge',0)
    check(tag+' independent actual copper check',count==(0 if row['requested_edge_mm']==.25 else 2))
    check(tag+' all collateral disclosed',row['drc']['by_type'].get('pad-pad')==5)
    check(tag+' correct floorplan verdict',row['grade']['pass']==(row['requested_edge_mm']==.25))
a.out.write_text(json.dumps({'before_revision':before['revision'],'after_revision':after['revision'],
    'seating_revision':seating['revision'],'passed':len(checks),'checks':checks},indent=2)+'\n',encoding='utf8')
print(f'{len(checks)} independent assertions passed')
