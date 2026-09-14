"""Independent malformed requirement refusal plus valid boundary controls."""
import json
import reviewer_geometry as g
from placement import floorplan as f
rows=[]
for field in ('min','max','max_setback_mm'):
    for value in (float('nan'),float('inf'),float('-inf'),-.01,True,False,'0.1'):
        entry={'ref':'J1','edge':'west'}
        if field=='max_setback_mm': entry[field]=value
        else: entry['overhang_mm']={field:value}
        try:
            f.intent_from_dict(dict(schema=f.SCHEMA_VERSION,kind=f.KIND,units='mm',edge_connectors=[entry]))
        except f.IntentError as exc:
            rows.append(dict(field=field,value=repr(value),refused=True,reason=str(exc)))
        else:
            rows.append(dict(field=field,value=repr(value),refused=False))
            raise AssertionError(rows[-1])
try:
    f.intent_from_dict(dict(schema=f.SCHEMA_VERSION,kind=f.KIND,units='mm',edge_connectors=[dict(ref='J1',edge='west',overhang_mm=dict(min=.3,max=.2))]))
except f.IntentError as exc:
    assert 'minimum exceeds maximum' in str(exc),str(exc)
    rows.append(dict(field='band',value='.3>.2',refused=True,reason=str(exc)))
else: raise AssertionError('reversed band accepted')
for band in ({'min':0},{'min':0,'max':0},{'min':.05,'max':.2}):
    entry=dict(ref='J1',edge='west',overhang_mm=band,max_setback_mm=0)
    intent=f.intent_from_dict(dict(schema=f.SCHEMA_VERSION,kind=f.KIND,units='mm',edge_connectors=[entry]))
    assert dict(intent.edge_connectors[0])==entry
    rows.append(dict(field='valid',value=band,refused=False,preserved=True))
(g.OUT/'declarations.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
print('Declaration controls passed:',len(rows))
