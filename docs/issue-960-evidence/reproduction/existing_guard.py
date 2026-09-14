import argparse,hashlib,json,pathlib,shutil,subprocess,sys,tempfile
p=argparse.ArgumentParser();p.add_argument('--repo',default='.');p.add_argument('--output',required=True);p.add_argument('--expect-fixed',action='store_true');a=p.parse_args();repo=pathlib.Path(a.repo).resolve();out=pathlib.Path(a.output).resolve();out.mkdir(parents=True,exist_ok=True)
for d in ['py_placer','py_router','py_tools']:sys.path.insert(0,str(repo/d))
from placement.writer import write_placed_output
from placement.pose_ops import _promote
from placement import provenance
from copy_board import copy_board
work=out/'work';truth=out/'truth';cmd=[sys.executable,'-X','utf8','tests/stress/stage_unaided.py',str(repo/'kicad_files/esp_prog.kicad_pcb'),str(work),str(truth)];r=subprocess.run(cmd,cwd=repo,capture_output=True,text=True,encoding='utf-8');(out/'stage.log').write_text(r.stdout+'\nSTDERR:\n'+r.stderr,encoding='utf-8');assert r.returncode==0
board=work/'board.kicad_pcb';dest=work/'delivered.kicad_pcb';copy_board(str(board),str(dest))
# Syntactically valid declaration sentinels exercise publication only, not grading.
for ext,data in [('.kicad_pro','{"net_settings":{"classes":[{"name":"Default","clearance":0.3}]}}\n'),('.kicad_dru','(version 1)\n'),('.design-brief.json','{"reviewer_sentinel":"preserve prior requirement declaration"}\n')]:dest.with_suffix(ext).write_text(data,encoding='utf-8')
def snapshot():return {str(p.relative_to(work)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(work.rglob('*')) if p.is_file()}
before=snapshot()
with tempfile.TemporaryDirectory(prefix='krt960-existing-reviewer-') as td:
 cand=pathlib.Path(td)/'candidate.kicad_pcb';copy_board(str(dest),str(cand));write_placed_output(str(board),str(cand),[{'reference':'R1','new_x':136.4,'new_y':98.8,'new_rotation':270}]);cand.with_suffix('.design-brief.json').write_text('{"reviewer_sentinel":"unauthorized replacement"}\n',encoding='utf-8')
 try:_promote(str(cand),str(dest));exc=None;reason=None
 except Exception as e:exc=type(e).__name__;reason=str(e)
after=snapshot();result={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),'stage_command':subprocess.list2cmdline(cmd),'before':before,'after':after,'unchanged':before==after,'exception':exc,'reason':reason,'ledger':provenance.read_ledger(str(work))};(out/'results.json').write_text(json.dumps(result,indent=2,sort_keys=True),encoding='utf-8');print(json.dumps({'exception':exc,'reason':reason,'unchanged':before==after,'ledger_rows':len(result['ledger'])},indent=2))
if a.expect_fixed:assert exc=='UnaidedViolation' and before==after and result['ledger']==[],result
else:assert exc is None and before!=after and result['ledger']==[],result
