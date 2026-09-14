"""Independent exact native identities on the existing full-chain gate outputs."""
import collections
import hashlib
import json
from pathlib import Path
import sys
import pcbnew

root=Path(sys.argv[1]).resolve()
out=Path(sys.argv[2]).resolve()
def inspect(path):
    b=pcbnew.LoadBoard(str(path))
    def xy(p):return(p.x,p.y)
    fp=sorted((f.m_Uuid.AsString(),f.GetReference(),xy(f.GetPosition()),f.GetOrientationDegrees(),f.GetLayer(),f.IsLocked(),sorted((p.m_Uuid.AsString(),p.GetNumber(),xy(p.GetPosition()),xy(p.GetSize()),p.GetNetname(),p.GetShape(),xy(p.GetDrillSize())) for p in f.Pads())) for f in b.GetFootprints())
    edge=sorted((d.m_Uuid.AsString(),d.GetShape(),xy(d.GetStart()),xy(d.GetEnd()),d.GetWidth()) for d in b.GetDrawings() if d.GetLayer()==pcbnew.Edge_Cuts)
    tracks=[]
    vias=[]
    for t in b.GetTracks():
        if isinstance(t,pcbnew.PCB_VIA):
            vias.append((t.GetNetname(),xy(t.GetPosition()),t.GetWidth(t.TopLayer()),t.GetDrillValue(),t.TopLayer(),t.BottomLayer()))
        else:
            tracks.append((t.GetNetname(),t.GetLayer(),sorted([xy(t.GetStart()),xy(t.GetEnd())]),t.GetWidth()))
    return {'footprints':fp,'outline':edge,'tracks':sorted(tracks),'vias':sorted(vias)}

raw={k:inspect(root/f) for k,f in [('input','src_splitflap_driver.kicad_pcb'),('cli','cli_final.kicad_pcb'),('gui','gui_replay.kicad_pcb')]}
report={'kicad':pcbnew.GetBuildVersion(),'paths':str(root),'assertions':{},'drc':{}}
for part in ['footprints','outline']:
    report['assertions'][part+'_preserved']=raw['input'][part]==raw['cli'][part]==raw['gui'][part]
    assert report['assertions'][part+'_preserved'],part
for part in ['tracks','vias']:
    report['assertions'][part+'_equal_exact_native_iu']=raw['cli'][part]==raw['gui'][part]
    report[part+'_count']=len(raw['cli'][part])
    assert report['assertions'][part+'_equal_exact_native_iu'],part
for front in ['input','cli','gui']:
    data=json.loads((out/('full-chain-'+front+'-native-drc.json')).read_text())
    report['drc'][front]={'counts':dict(collections.Counter(v['type'] for v in data['violations'])),'unconnected':len(data['unconnected_items'])}
    edge=sorted(tuple(sorted(i['uuid'] for i in v['items'])) for v in data['violations'] if v['type']=='copper_edge_clearance')
    if front=='input':orig=edge
    else:
        assert edge==orig
        assert not data['unconnected_items']
report['assertions']['native_pad_edge_errors_same_uuids']=True
(out/'full-chain-inspection.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
