"""Native UUID-based preservation audit, retaining duplicate-reference footprints."""
import argparse
import hashlib
import json
from pathlib import Path
import pcbnew

p=argparse.ArgumentParser()
p.add_argument('--source',type=Path,required=True)
p.add_argument('--results',type=Path,required=True)
p.add_argument('--out',type=Path,required=True)
a=p.parse_args()
data=json.loads(a.results.read_text(encoding='utf8'))
def xy(p,dx=0.): return [round(pcbnew.ToMM(p.x)-dx,6),round(pcbnew.ToMM(p.y),6)]
def ident(x): return str(x.m_Uuid.AsString())
def graphic(g,dx=0.):
    row={'type':g.GetClass(),'layer':g.GetLayerName()}
    if isinstance(g,pcbnew.PCB_SHAPE):
        row.update(shape=g.GetShape(),start=xy(g.GetStart(),dx),end=xy(g.GetEnd(),dx),
                   width=pcbnew.ToMM(g.GetWidth()))
    elif isinstance(g,pcbnew.PCB_TEXT):
        row.update(text=g.GetText(),position=xy(g.GetPosition(),dx),size=xy(g.GetTextSize()),
                   angle=g.GetTextAngle().AsDegrees())
    return row
def snapshot(path,dx=0.):
    board=pcbnew.LoadBoard(str(path))
    fps={}
    for f in board.GetFootprints():
        delta=dx if f.GetReference()=='USB1' else 0.
        pads={ident(p):{'number':p.GetNumber(),'position':xy(p.GetPosition(),delta),
           'size':xy(p.GetSize()),'shape':p.GetShape(),'angle':p.GetOrientationDegrees(),
           'net':p.GetNetname(),'drill':xy(p.GetDrillSize()),'layers':p.GetLayerSet().FmtHex()}
           for p in f.Pads()}
        fps[ident(f)]={'ref':f.GetReference(),'position':xy(f.GetPosition(),delta),
           'angle':f.GetOrientationDegrees(),'layer':f.GetLayerName(),'locked':f.IsLocked(),
           'pads':pads,'graphics':{ident(g):graphic(g,delta) for g in f.GraphicalItems()}}
    return {'footprints':fps,'drawings':{ident(g):graphic(g) for g in board.GetDrawings()},
            'copper_layers':board.GetCopperLayerCount()}
baseline=snapshot(a.source)
rows=[]
for v in data['variants']:
    path=a.results.parent/v['name']/'esp_prog.kicad_pcb'
    actual=snapshot(path,v['translation_x_mm'])
    assert actual==baseline, v['name']
    rows.append({'variant':v['name'],'translation_x_mm':v['translation_x_mm'],
      'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
      'footprints_preserved_after_undoing_only_USB1_translation':len(actual['footprints']),
      'pads_preserved':sum(len(f['pads']) for f in actual['footprints'].values()),
      'board_drawings_preserved':len(actual['drawings']),'pass':True})
a.out.write_text(json.dumps({'behavior_revision':data['revision'],'native_version':pcbnew.Version(),
    'rows':rows,'source_native_snapshot':baseline},indent=2)+'\n',encoding='utf8')
print('All native UUID/reference/face/lock/rotation/pad/graphic/outline records preserved')
