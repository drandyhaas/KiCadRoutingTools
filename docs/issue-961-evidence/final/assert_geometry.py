"""Assert final CLI evidence using independent native results and frozen base variants."""
import hashlib,json,sys
from pathlib import Path
final=Path(sys.argv[1]);base=Path(sys.argv[2]);f=json.loads((final/'results.json').read_text());b=json.loads((base/'results.json').read_text());checks=[]
def check(name,ok,detail=None):
 checks.append({'name':name,'pass':bool(ok),'detail':detail})
 if not ok:print('FAIL',name,detail)
for v,bv in zip(f['variants'],b['variants']):
 dx=v['dx_mm'];actual=v['native'];check(str(dx)+' same native geometry and board bytes',v['board_sha256']==bv['board_sha256'] and actual==bv['native'])
 minimum=min(min(p['bbox_mm'][0]-114,145.75-p['bbox_mm'][2],p['bbox_mm'][1]-91,105.5-p['bbox_mm'][3]) for p in actual['native_pad_polygons'])
 for run in v['floorplan']:
  label=f"{dx}/{run['band']}/{run['requested']}";g=run['grade'];row=next((r for r in g['edge_seating'] if r['ref']=='USB1'),{})
  check(label+' actual body value',abs(row.get('body_overhang_mm',-100)-actual['body_overhang_mm'])<1e-6,row)
  check(label+' actual setback value',abs(row.get('body_setback_mm',-100)-actual['body_setback_mm'])<1e-6)
  check(label+' physical evidence same currency',row.get('overhang_mm')==row.get('body_overhang_mm') and row.get('overhang_basis')==row.get('body_overhang_basis'))
  check(label+' explicit Fab support basis',row.get('body_overhang_basis')=='F.Fab' and row.get('body_measured') is True)
  check(label+' native all-edge copper gap',abs(row.get('pad_copper_edge_gap_mm',-100)-minimum)<1e-6,(row.get('pad_copper_edge_gap_mm'),minimum))
  check(label+' native west copper gap',abs(row.get('pad_copper_declared_edge_gap_mm',-100)-actual['pad_west_gap_mm'])<1e-6)
  required=run['requested']['board_edge_clearance'];check(label+' effective copper requirement',row.get('required_copper_edge_gap_mm')==required)
  check(label+' independent shortfall',abs(row.get('copper_edge_shortfall_mm',-100)-max(0,required-minimum))<1e-6)
  params=row.get('clearance_parameters',{});check(label+' explicit physical versus occupancy margin',params.get('physical_body_margin_mm')==0 and abs(params.get('effective_occupancy_margin_mm',-100)-max(run['requested'].values()))<1e-6)
  for name in ('body_overhang','body_setback','pad_copper_edge_gap','copper_edge_shortfall'):
   m=row.get('measurements',{}).get(name,{});check(label+'/'+name+' complete contract',all(k in m for k in ('value','units','geometry_basis','declared_limit','requirement_source','disposition')) and m.get('units')=='mm' and bool(m.get('geometry_basis')) and bool(m.get('requirement_source')))
  band={'positive':(.05,.2),'zero':(0,.65),'legal-body':(0,1.5),'seat':(0,.65)}[run['band']];ov=actual['body_overhang_mm'];expected='pass' if band[0]-1e-6<=ov<=band[1]+1e-6 else 'fail'
  check(label+' overhang disposition independently derived',row.get('measurements',{}).get('body_overhang',{}).get('disposition')==expected)
  expected_seat=('fail' if actual['body_setback_mm']>.1+1e-6 else 'pass') if run['band']=='seat' else 'not_declared'
  check(label+' setback independent of zero minimum',row.get('measurements',{}).get('body_setback',{}).get('disposition')==expected_seat)
  expected_copper='fail' if minimum<required-1e-6 else 'pass';check(label+' copper independent disposition',row.get('measurements',{}).get('pad_copper_edge_gap',{}).get('disposition')==expected_copper)
check('original fixture identity preserved',f.get('source_preserved') and f['source_sha256']==b['source_sha256'])
result={'behavior_revision':f['revision'],'integration_base_revision':b['revision'],'checks':checks,'passed':sum(c['pass'] for c in checks),'failed':sum(not c['pass'] for c in checks)}
(final/'independent-assertions.json').write_text(json.dumps(result,indent=2));print(result['passed'],'passed',result['failed'],'failed');sys.exit(bool(result['failed']))
