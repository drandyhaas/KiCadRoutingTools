"""Independent native KiCad and geometry verification of previously written outputs."""
import collections
import hashlib
import json
from pathlib import Path
import subprocess
import sys

repo=Path(sys.argv[1]).resolve()
out=Path(sys.argv[2]).resolve()
sys.path[:0]=[str(repo),str(repo/'py_router'),str(repo/'py_tools')]
import pcbnew

def native(path):
    b=pcbnew.LoadBoard(str(path))
    def v(pt): return (pt.x,pt.y)
    mech={'footprints':sorted((f.m_Uuid.AsString(), f.GetReference(),v(f.GetPosition()),f.GetOrientationDegrees(),f.GetLayer(),f.IsLocked(), sorted((p.GetNumber(),v(p.GetPosition()),v(p.GetSize()),p.GetShape(),p.GetNetname(),p.GetDrillSize().x,p.GetDrillSize().y) for p in f.Pads())) for f in b.GetFootprints()),
          'outline':sorted((d.m_Uuid.AsString(),d.GetShape(),v(d.GetStart()),v(d.GetEnd()),d.GetWidth()) for d in b.GetDrawings() if d.GetLayer()==pcbnew.Edge_Cuts),
          'layers':[i for i in range(pcbnew.PCB_LAYER_ID_COUNT) if b.IsLayerEnabled(i)]}
    copper=[]
    for t in b.GetTracks():
        copper.append((t.GetNetname(),t.GetLayer(),v(t.GetStart()),v(t.GetEnd()),t.GetWidth()))
    return mech,sorted(copper)

def diff(a,b,prefix=''):
    rows=[]
    if isinstance(a,dict) and isinstance(b,dict):
        for k in sorted(set(a)|set(b)):
            rows+=diff(a.get(k,'<absent>'),b.get(k,'<absent>'),prefix+'/'+k)
    elif a!=b: rows.append({'key':prefix,'before':a,'after':b})
    return rows

report={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),'kicad':pcbnew.GetBuildVersion(),'boards':{}}
for variant in ['zero','positive']:
    src=out/(variant+'.kicad_pcb')
    before,_=native(src)
    pro_before=json.loads(src.with_suffix('.kicad_pro').read_text())
    native_results={}
    for front in ['input','cli','gui']:
        path=src if front=='input' else out/(variant+'-'+front+'.kicad_pcb')
        mech,copper=native(path)
        assert mech==before,(path,'mechanical mutation')
        pro=json.loads(path.with_suffix('.kicad_pro').read_text())
        dflt=next(c for c in pro['net_settings']['classes'] if c['name']=='Default')
        wide=next(c for c in pro['net_settings']['classes'] if c['name']=='Wide')
        assert dflt['clearance']==(0.0 if variant=='zero' else 0.2),(path,dflt)
        assert wide==next(c for c in pro_before['net_settings']['classes'] if c['name']=='Wide'),(path,'Wide mutated')
        report_path=out/(variant+'-'+front+'-native-drc.json')
        cmd=[str(Path(sys.executable).with_name('kicad-cli.exe')),'pcb','drc','--format','json','--all-track-errors','--refill-zones','-o',str(report_path),str(path)]
        r=subprocess.run(cmd,capture_output=True,text=True,timeout=180)
        (out/(variant+'-'+front+'-native-drc.log')).write_text(r.stdout+r.stderr)
        assert r.returncode==0,(cmd,r.returncode,r.stderr)
        drc=json.loads(report_path.read_text())
        violations=collections.Counter(v['type'] for v in drc['violations'])
        target_unconnected=[v for v in drc['unconnected_items'] if any('Net-(D1-A)' in i['description'] for i in v['items'])]
        if front!='input':
            assert copper,(path,'no copper')
            assert not target_unconnected,(path,'native target still unconnected',target_unconnected)
            cmd2=[sys.executable,'-X','utf8',str(repo/'py_router/check_connected.py'),str(path),'--nets','Net-(D1-A)']
            r2=subprocess.run(cmd2,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=180)
            (out/(variant+'-'+front+'-connected.log')).write_text(r2.stdout+r2.stderr)
            assert r2.returncode==0 and 'ALL NETS FULLY CONNECTED' in r2.stdout,(path,r2.stdout,r2.stderr)
        native_results[front]={'board_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'project_sha256':hashlib.sha256(path.with_suffix('.kicad_pro').read_bytes()).hexdigest(),'copper':copper,'mechanics_unchanged':mech==before,'default_clearance':dflt['clearance'],'wide_preserved':True,'project_changes':diff(pro_before,pro),'native_drc_types':dict(violations),'native_unconnected_count':len(drc['unconnected_items']),'native_target_unconnected_count':len(target_unconnected)}
    native_results['copper_equal']=native_results['cli']['copper']==native_results['gui']['copper']
    assert native_results['copper_equal'],(variant,'copper differs')
    report['boards'][variant]=native_results
(out/'inspection.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'revision':report['revision'],'variants':{k:{f:v[f]['native_drc_types'] for f in ['input','cli','gui']} for k,v in report['boards'].items()}}),flush=True)
