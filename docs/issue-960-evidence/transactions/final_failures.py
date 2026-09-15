"""Independent recovery, cleanup, and alternate-boundary checks."""
import hashlib,json,os,pathlib,shutil,subprocess,sys,tempfile
from unittest.mock import patch
ROOT=pathlib.Path.cwd()
for p in ('py_placer','py_router','py_tools'):sys.path.insert(0,str(ROOT/p))
from placement import provenance as pv, publication as pub
from placement.writer import write_placed_output
from placement.portfolio import copy_siblings
from place_reconstruct import _promote_staged
LABEL=sys.argv[1] if len(sys.argv)>1 else 'final-failures';runtime=ROOT/'reviewer-runtime'/LABEL;runtime.mkdir(parents=True,exist_ok=True)
def sha(p):return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest() if pathlib.Path(p).is_file() else None
def run(args):
 r=subprocess.run([sys.executable,'-X','utf8',*map(str,args)],capture_output=True,text=True);return dict(command=subprocess.list2cmdline([sys.executable,'-X','utf8',*map(str,args)]),code=r.returncode,stdout=r.stdout,stderr=r.stderr)
def create(n):
 d=runtime/n/'work';r=run(['tests/stress/stage_unaided.py','kicad_files/esp_prog.kicad_pcb',d,d.parent/'truth']);assert r['code']==0,r;return d,d/'board.kicad_pcb'
def catch(fn):
 try:fn();return None
 except BaseException as e:return dict(type=type(e).__name__,error=str(e),details=getattr(e,'details',None))
def cand(b,t):
 c=pathlib.Path(t)/'candidate.kicad_pcb';write_placed_output(str(b),str(c),[dict(reference='R1',new_x=136.4,new_y=98.8,new_rotation=270)]);copy_siblings(str(b),str(c));return c
result=dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),cases={})
# The second registered placement promotion helper must also gate destination.
d,b=create('reconstruct_undeclared');out=d/'out.kicad_pcb'
with tempfile.TemporaryDirectory() as t:
 c=cand(b,t);error=catch(lambda:_promote_staged(str(c),str(out)))
result['cases']['reconstruct_undeclared']=dict(error=error,exists=out.exists(),rows=pv.read_ledger(str(d)))
assert error and error['type']=='UnaidedViolation' and not out.exists(),result['cases']['reconstruct_undeclared']
# Registered helper writes a reconciled actual final board.
d,b=create('reconstruct_registered');out=d/'out.kicad_pcb'
with tempfile.TemporaryDirectory() as t:
 c=cand(b,t)
 with pv.declare_lever('place_reconstruct.py'):_promote_staged(str(c),str(out),input_file=str(b))
audit=run(['tests/stress/provenance_audit.py','--workdir',d,'--delivered',out]);rows=pv.read_ledger(str(d));result['cases']['reconstruct_registered']=dict(audit=audit,rows=rows,sha256=sha(out));assert audit['code']==0 and rows[-1]['board_sha256']==sha(out)
# Prior deliverable exists. Project publishes, board publication fails, and
# project restoration itself fails. Require truthful partial and retained old bytes.
d,b=create('recovery_failure');out=d/'out.kicad_pcb';shutil.copy2(b,out);pro=out.with_suffix('.kicad_pro');pro.write_text('{"old":true}');before={str(p):sha(p) for p in (out,pro)};original=os.replace
with tempfile.TemporaryDirectory() as t:
 c=cand(b,t);c.with_suffix('.kicad_pro').write_text('{"new":true}')
 def fail(src,dst):
  if pathlib.Path(dst)==out:raise OSError('injected final-board publication failure')
  if pathlib.Path(dst)==pro and pathlib.Path(src).name.startswith('.krt-backup-'):raise OSError('injected project restoration failure')
  return original(src,dst)
 with pv.declare_lever('place_pose.py'),patch('os.replace',fail):error=catch(lambda:pub.publish_board(str(c),str(out),input_file=str(b)))
after={str(p):sha(p) for p in (out,pro)};audit=run(['tests/stress/provenance_audit.py','--workdir',d,'--delivered',out]);backups={str(p):sha(p) for p in d.glob('.krt-backup-*')};result['cases']['recovery_failure']=dict(error=error,before=before,after=after,backups=backups,audit=audit,rows=pv.read_ledger(str(d)))
assert error['details']['output_state']=='partial' and before[str(out)]==after[str(out)] and before[str(pro)]!=after[str(pro)] and before[str(pro)] in backups.values() and audit['code']==5 and not pv.read_ledger(str(d))
# Cleanup failure happens after commit. It must say committed and keep valid
# ledger bytes and backups; the audit must not report CLEAN while unresolved.
d,b=create('cleanup_committed');out=d/'out.kicad_pcb';shutil.copy2(b,out);before=sha(out);original_remove=os.remove
with tempfile.TemporaryDirectory() as t:
 c=cand(b,t)
 def remove_fail(p,*a,**kw):
  if pathlib.Path(p).name=='journal.json':raise OSError('injected committed journal cleanup failure')
  return original_remove(p,*a,**kw)
 with pv.declare_lever('place_pose.py'),patch('os.remove',remove_fail):error=catch(lambda:pub.publish_board(str(c),str(out),input_file=str(b)))
rows=pv.read_ledger(str(d));audit=run(['tests/stress/provenance_audit.py','--workdir',d,'--delivered',out]);result['cases']['cleanup_committed']=dict(error=error,before=before,after=sha(out),rows=rows,audit=audit,backups={str(p):sha(p) for p in d.glob('.krt-backup-*')})
assert error['details']['output_state']=='committed' and before!=sha(out) and len(rows)==1 and rows[-1]['board_sha256']==sha(out) and audit['code']==5
# Same cleanup failure after a rollback must disclose unchanged, no success row.
d,b=create('cleanup_rolled_back');out=d/'out.kicad_pcb';shutil.copy2(b,out);before=sha(out);original_replace=os.replace
with tempfile.TemporaryDirectory() as t:
 c=cand(b,t)
 def replace_fail(src,dst):
  if pathlib.Path(dst)==out:raise OSError('injected board failure before replacement')
  return original_replace(src,dst)
 with pv.declare_lever('place_pose.py'),patch('os.remove',remove_fail),patch('os.replace',replace_fail):error=catch(lambda:pub.publish_board(str(c),str(out),input_file=str(b)))
result['cases']['cleanup_rolled_back']=dict(error=error,before=before,after=sha(out),rows=pv.read_ledger(str(d)));assert error['details']['output_state']=='unchanged' and before==sha(out) and not pv.read_ledger(str(d))
(ROOT/'docs/issue-960-evidence/transactions'/f'{LABEL}.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:'PASS' for k in result['cases']}))
# A failed next commit preserves an existing successful row byte-for-byte.
d,b=create('existing_ledger');out=d/'out.kicad_pcb'
first=run(['py_placer/place_pose.py',b,out,'set','R1','136.4','98.8','--rot','270']);assert first['code']==0,first
before_board=sha(out);ledger=d/pv.LEDGER_NAME;before_ledger=sha(ledger);original=os.replace;fired=[]
def ledger_fail(src,dst):
 if pathlib.Path(dst)==ledger and not fired:fired.append(True);original(src,dst);raise OSError('injected replacement after existing-ledger commit')
 return original(src,dst)
with pv.declare_lever('place_pose.py'),patch('os.replace',ledger_fail):error=catch(lambda:write_placed_output(str(out),str(out),[dict(reference='R1',new_x=137.4,new_y=98.8,new_rotation=270)]))
result['cases']['existing_ledger']=dict(error=error,before_board=before_board,after_board=sha(out),before_ledger=before_ledger,after_ledger=sha(ledger),rows=pv.read_ledger(str(d)))
assert error and before_board==sha(out) and before_ledger==sha(ledger) and len(pv.read_ledger(str(d)))==1
# Change ONLY a lock via native KiCad after a successful moved pose. The
# execution ledger has no claim for this independent file mutation.
import pcbnew
d,b=create('lock_drift');out=d/'out.kicad_pcb';first=run(['py_placer/place_pose.py',b,out,'set','R1','136.4','98.8','--rot','270']);assert first['code']==0,first
native=pcbnew.LoadBoard(str(out));part=next(f for f in native.GetFootprints() if f.GetReference()=='R1');before_pose=[pcbnew.ToMM(part.GetPosition().x),pcbnew.ToMM(part.GetPosition().y),part.GetOrientationDegrees()];part.SetLocked(True);pcbnew.SaveBoard(str(out),native)
audit=run(['tests/stress/provenance_audit.py','--workdir',d,'--delivered',out]);result['cases']['lock_drift']=dict(audit=audit,pose_unchanged=before_pose,rows=pv.read_ledger(str(d)));assert audit['code']==4 and 'NOT where' in audit['stdout']
(ROOT/'docs/issue-960-evidence/transactions'/f'{LABEL}.json').write_text(json.dumps(result,indent=2));print('existing_ledger PASS; lock_drift PASS')
# Ledger restoration itself can fail after a successful ledger rename. The
# surviving last row then names candidate bytes while the board was restored.
# This is not a reconciled commit: retained marker must force UNPROVEN.
d,b=create('ledger_restore_failure');out=d/'out.kicad_pcb';first=run(['py_placer/place_pose.py',b,out,'set','R1','136.4','98.8','--rot','270']);assert first['code']==0,first
ledger=d/pv.LEDGER_NAME;before_board=sha(out);before_ledger=sha(ledger);original=os.replace;fired=[]
def restore_denied(src,dst):
 if pathlib.Path(dst)==ledger:
  if pathlib.Path(src).name.startswith('.krt-backup-'):raise OSError('injected old-ledger restoration denied')
  if not fired:fired.append(True);original(src,dst);raise OSError('injected interruption after successful ledger replacement')
 return original(src,dst)
with pv.declare_lever('place_pose.py'),patch('os.replace',restore_denied):error=catch(lambda:write_placed_output(str(out),str(out),[dict(reference='R1',new_x=137.4,new_y=98.8,new_rotation=270)]))
audit=run(['tests/stress/provenance_audit.py','--workdir',d,'--delivered',out]);records=pv.read_ledger(str(d));backups={str(p):sha(p) for p in d.glob('.krt-backup-*')};result['cases']['ledger_restore_failure']=dict(error=error,before_board=before_board,after_board=sha(out),before_ledger=before_ledger,after_ledger=sha(ledger),rows=records,backups=backups,audit=audit)
(ROOT/'docs/issue-960-evidence/transactions'/f'{LABEL}.json').write_text(json.dumps(result,indent=2))
assert error['details']['output_state']=='partial' and before_board==sha(out) and before_ledger!=sha(ledger) and len(records)==2 and records[-1]['board_sha256']!=sha(out) and before_ledger in backups.values() and audit['code']==5 and not pv._PENDING
print('ledger_restore_failure PASS (partial ledger; unresolved candidate row; audit UNPROVEN)')
