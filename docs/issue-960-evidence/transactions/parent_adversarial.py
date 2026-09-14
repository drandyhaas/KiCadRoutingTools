"""Independent transaction witnesses; run from repo using KiCad Python."""
import builtins, hashlib, json, os, pathlib, shutil, subprocess, sys, tempfile, threading
from unittest.mock import patch
ROOT = pathlib.Path.cwd()
for p in ('py_placer','py_router','py_tools'):
    sys.path.insert(0,str(ROOT/p))
from placement import provenance, pose_ops
from placement.writer import write_placed_output
from placement.portfolio import copy_siblings
from kicad_parser import parse_kicad_pcb
BASE=ROOT/'reviewer-runtime'/'work'/'board.kicad_pcb'
OUT=ROOT/'docs'/'issue-960-evidence'/'transactions'
RUN=ROOT/'reviewer-runtime'/'adversarial'
RUN.mkdir(parents=True,exist_ok=True)
MOVE=[dict(reference='R1',new_x=136.4,new_y=98.8,new_rotation=270)]
def sha(p): return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest() if pathlib.Path(p).is_file() else None
def pose(p):
    fp=parse_kicad_pcb(str(p)).footprints['R1']; return [fp.x,fp.y,fp.rotation]
def armed(name):
    d=RUN/name; d.mkdir(exist_ok=True); b=d/'board.kicad_pcb'; shutil.copy2(BASE,b); copy_siblings(str(BASE),str(b)); provenance.start_regime(str(d),str(b)); return d,b
results={'revision':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'source_sha256':sha(ROOT/'kicad_files'/'esp_prog.kicad_pcb'),'baseline_sha256':sha(BASE),'python':sys.version,'cases':{}}
# Real externally staged writer and promote, with no declaration at either boundary.
d,b=armed('undeclared')
with tempfile.TemporaryDirectory(prefix='960-outside-') as tmp:
    cand=pathlib.Path(tmp)/'candidate.kicad_pcb'; write_placed_output(str(b),str(cand),MOVE); copy_siblings(str(b),str(cand)); dest=d/'out.kicad_pcb'
    try: pose_ops._promote(str(cand),str(dest)); error=None
    except BaseException as e: error=f'{type(e).__name__}: {e}'
    results['cases']['undeclared_promotion']={'error':error,'output_exists':dest.exists(),'pose':pose(dest) if dest.exists() else None,'rows':provenance.read_ledger(str(d))}
# Writer commits only AFTER destination mutation; simulate unavailable ledger.
d,b=armed('ledger_failure'); dest=d/'out.kicad_pcb'; shutil.copy2(b,dest); old=sha(dest); real_open=builtins.open
def fail_ledger(file,*args,**kwargs):
    if os.path.basename(os.fspath(file))==provenance.LEDGER_NAME and args and 'a' in args[0]: raise OSError('injected ledger append unavailable')
    return real_open(file,*args,**kwargs)
with provenance.declare_lever('place_pose.py'),patch('builtins.open',fail_ledger):
    try: write_placed_output(str(b),str(dest),MOVE); error=None
    except BaseException as e: error=f'{type(e).__name__}: {e}'
results['cases']['writer_ledger_failure']={'error':error,'before_sha':old,'after_sha':sha(dest),'pose':pose(dest),'rows':provenance.read_ledger(str(d)),'pending':list(provenance._PENDING)}
# Pending token overwritten by another invocation to the same destination.
d,b=armed('pending_collision'); dest=d/'out.kicad_pcb'
with provenance.declare_lever('place_pose.py'):
    provenance.record_write(str(b),str(dest),MOVE,pending=True)
with provenance.declare_lever('place_seed.py'):
    provenance.record_write(str(b),str(dest),[dict(MOVE[0],new_x=130)],pending=True)
results['cases']['pending_collision']={'pending_rows':list(provenance._PENDING.values())}
provenance._PENDING.clear()
# Context attribution leaks across concurrent threads on parent.
event=threading.Event(); done=threading.Event(); observed=[]
def declaring_thread():
    with provenance.declare_lever('place_pose.py'):
        event.set(); done.wait(5)
t=threading.Thread(target=declaring_thread);t.start();event.wait(5); observed.append(provenance.active_lever());done.set();t.join()
results['cases']['thread_undeclared_inherits']={'active_lever_in_other_thread':observed[0]}
# Interrupt between project and PCB publication: observe bytes and recovery metadata.
d,b=armed('interrupt'); dest=d/'out.kicad_pcb'; shutil.copy2(b,dest); copy_siblings(str(b),str(dest)); sibling=dest.with_suffix('.kicad_pro'); sibling.write_text('{"old":true}')
old_files={str(p):sha(p) for p in (dest,sibling)}
real_replace=os.replace
with tempfile.TemporaryDirectory(prefix='960-interrupt-') as tmp:
    cand=pathlib.Path(tmp)/'candidate.kicad_pcb'; write_placed_output(str(b),str(cand),MOVE); cand.with_suffix('.kicad_pro').write_text('{"new":true}'); calls=[]
    def interrupted(src,dst):
        real_replace(src,dst); calls.append(str(dst))
        if len(calls)==1: raise KeyboardInterrupt('injected interrupt immediately after successful replacement')
    with provenance.declare_lever('place_pose.py'),patch('os.replace',interrupted):
        try: pose_ops._promote(str(cand),str(dest)); error=None
        except BaseException as e: error=f'{type(e).__name__}: {e}'
    results['cases']['interrupt_after_replace']={'error':error,'before':old_files,'after':{str(p):sha(p) for p in (dest,sibling)},'remaining_backups':[str(p) for p in d.glob('.krt-backup-*')],'rows':provenance.read_ledger(str(d))}
OUT.mkdir(parents=True,exist_ok=True);(OUT/'parent-adversarial.json').write_text(json.dumps(results,indent=2,default=str))
print(json.dumps(results,indent=2,default=str))
