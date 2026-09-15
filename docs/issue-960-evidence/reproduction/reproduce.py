import argparse, hashlib, json, os, pathlib, subprocess, sys, tempfile
p=argparse.ArgumentParser(); p.add_argument('--repo',default='.'); p.add_argument('--output',required=True); a=p.parse_args()
repo=pathlib.Path(a.repo).resolve(); out=pathlib.Path(a.output).resolve(); out.mkdir(parents=True,exist_ok=True)
for d in ['py_placer','py_router','py_tools']: sys.path.insert(0,str(repo/d))
import pcbnew
from placement.writer import write_placed_output
from placement.pose_ops import _promote
from placement import provenance
from copy_board import copy_board, SIBLING_EXTS
results={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),'python':sys.version,'kicad':pcbnew.GetBuildVersion(),'commands':[]}
def sha(p): return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
def hashes(d): return {str(p.relative_to(d)):sha(p) for p in sorted(d.rglob('*')) if p.is_file()}
def run(name,args):
 cmd=[sys.executable,'-X','utf8']+list(map(str,args)); r=subprocess.run(cmd,cwd=repo,text=True,capture_output=True,encoding='utf-8'); (out/(name+'.log')).write_text(r.stdout+'\nSTDERR:\n'+r.stderr,encoding='utf-8'); entry={'name':name,'command':subprocess.list2cmdline(cmd),'exit':r.returncode}; results['commands'].append(entry); return r
fixture=repo/'kicad_files/esp_prog.kicad_pcb'; results['source_sha256']=sha(fixture); results['source_siblings']={ext:sha(fixture.with_suffix(ext)) for ext in SIBLING_EXTS if fixture.with_suffix(ext).exists()}
work=out/'work'; truth=out/'truth'; r=run('stage',['tests/stress/stage_unaided.py',fixture,work,truth]); assert r.returncode==0,r.stderr
board=work/'board.kicad_pcb'; initial=hashes(work); results['initial_work_identities']=initial; results['truth_identities']=hashes(truth); results['manifest']=json.loads((work/provenance.REGIME_NAME).read_text())
pose=work/'pose.kicad_pcb'; r=run('registered_pose',['py_placer/place_pose.py',board,pose,'set','R1','136.4','98.8','--rot','270']); assert r.returncode==0,r.stderr
results['registered_summary']=json.loads(next(x[len('JSON_SUMMARY: '):] for x in r.stdout.splitlines() if x.startswith('JSON_SUMMARY: ')))
def native(path):
 b=pcbnew.LoadBoard(str(path)); fp=next(f for f in b.GetFootprints() if f.GetReference()=='R1'); return {'x':pcbnew.ToMM(fp.GetPosition().x),'y':pcbnew.ToMM(fp.GetPosition().y),'rot':fp.GetOrientationDegrees()%360,'layer':fp.GetLayerName(),'locked':fp.IsLocked(),'sha256':sha(path)}
results['registered_native']=native(pose); results['registered_ledger']=provenance.read_ledger(str(work)); run('registered_audit',['tests/stress/provenance_audit.py','--workdir',work,'--delivered',pose,'--json',out/'registered_audit.json'])
placements=[{'reference':'R1','new_x':136.4,'new_y':98.8,'new_rotation':270}]
try: write_placed_output(str(board),str(work/'undeclared_direct.kicad_pcb'),placements); direct={'exception':None}
except Exception as e: direct={'exception':type(e).__name__,'reason':str(e)}
direct['output_exists']=(work/'undeclared_direct.kicad_pcb').exists(); results['undeclared_direct']=direct
with tempfile.TemporaryDirectory(prefix='krt960-reviewer-') as td:
 candidate=pathlib.Path(td)/'candidate.kicad_pcb'; copy_board(str(board),str(candidate)); write_placed_output(str(board),str(candidate),placements); bypass=work/'undeclared_promoted.kicad_pcb'; results['candidate_regime']=provenance.regime_for(str(candidate))
 try: _promote(str(candidate),str(bypass)); outcome={'exception':None}
 except Exception as e: outcome={'exception':type(e).__name__,'reason':str(e)}
 outcome['output_exists']=bypass.exists(); outcome['native']=native(bypass) if bypass.exists() else None; outcome['ledger']=provenance.read_ledger(str(work)); results['undeclared_promote']=outcome
 if bypass.exists(): run('bypass_audit',['tests/stress/provenance_audit.py','--workdir',work,'--delivered',bypass,'--json',out/'bypass_audit.json'])
control=work/'direct_registered.kicad_pcb'
with provenance.declare_lever('place_pose.py',['reviewer direct writer control']): write_placed_output(str(board),str(control),placements)
# Match the production writer caller's sibling propagation.
for ext in SIBLING_EXTS:
 src=board.with_suffix(ext)
 if src.exists(): import shutil; shutil.copyfile(src,control.with_suffix(ext))
results['direct_registered_native']=native(control); results['direct_registered_ledger']=provenance.read_ledger(str(work)); run('direct_registered_audit',['tests/stress/provenance_audit.py','--workdir',work,'--delivered',control,'--json',out/'direct_registered_audit.json'])
results['initial_files_preserved']={name:sha(work/name)==digest for name,digest in initial.items()}; results['source_preserved']=sha(fixture)==results['source_sha256']; (out/'results.json').write_text(json.dumps(results,indent=2,sort_keys=True),encoding='utf-8'); print(json.dumps({k:results[k] for k in ['revision','registered_native','undeclared_direct','undeclared_promote','source_preserved']},indent=2))
