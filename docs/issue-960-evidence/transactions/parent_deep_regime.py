"""Run after parent stager reproduction; descendants must inherit its regime."""
import json,pathlib,sys
root=pathlib.Path.cwd()
for p in ('py_placer','py_router','py_tools'):sys.path.insert(0,str(root/p))
from placement import provenance
from placement.writer import write_placed_output
base=root/'reviewer-runtime/work/board.kicad_pcb'
d=root/'reviewer-runtime/adversarial/deep';d.mkdir(exist_ok=True)
provenance.start_regime(str(d),str(base))
for _ in range(25):d=d/'d'
d.mkdir(parents=True,exist_ok=True);out=d/'out.kicad_pcb';error=None
try:write_placed_output(str(base),str(out),[dict(reference='R1',new_x=136.4,new_y=98.8,new_rotation=270)])
except BaseException as e:error=repr(e)
result=dict(ancestor_depth=25,regime_detected=provenance.regime_for(str(out)),output_exists=out.exists(),error=error)
(root/'docs/issue-960-evidence/transactions/parent-deep-regime.json').write_text(json.dumps(result,indent=2));print(result)
