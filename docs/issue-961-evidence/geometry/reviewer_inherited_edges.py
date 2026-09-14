"""Verify the frozen legacy-test edge table against the actual integration base."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import subprocess
import reviewer_geometry as g
from kicad_parser import parse_kicad_pcb
from placement import floorplan

table=json.loads((g.OUT/'961-inherited-edge-intents.json').read_text(encoding='utf-8'))
revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=g.CODE,text=True).strip()
assert revision==table['integration_base'],(revision,table['integration_base'])
rows=[]
for name,expected in table['edges'].items():
    path=g.CODE/'kicad_files'/(name+'.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()):
        pcb=parse_kicad_pcb(str(path));doc=floorplan.emit_intent(pcb,str(path),declare_classes=True)
    actual={c['ref']:c.get('edge') for c in doc['edge_connectors']}
    assert actual==expected,(name,actual,expected)
    rows.append(dict(board=name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),edges=actual))
(g.OUT/'inherited_edges_verified.json').write_text(json.dumps(dict(revision=revision,rows=rows),indent=2),encoding='utf-8')
print('Frozen integration-base edge tables independently verified:',len(rows))
