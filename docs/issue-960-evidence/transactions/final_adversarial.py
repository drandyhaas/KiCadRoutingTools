"""Independent FINAL revision transaction probes, launched from reviewer checkout.
Usage: python final_adversarial.py [result-label]
All cases use copies staged by the real unaided CLI and real publication/writer
functions; fault injection is named at the filesystem boundary being exercised.
"""
import builtins, hashlib, importlib.util, json, os, pathlib, shutil, subprocess, sys, tempfile, threading
from unittest.mock import patch
ROOT=pathlib.Path.cwd()
for p in ('py_placer','py_router','py_tools'):sys.path.insert(0,str(ROOT/p))
from placement import provenance as pv, publication as pub, pose_ops
from placement.writer import write_placed_output
from placement.portfolio import copy_siblings
from kicad_parser import parse_kicad_pcb
LABEL=sys.argv[1] if len(sys.argv)>1 else 'final'
RUNTIME=ROOT/'reviewer-runtime'/LABEL;RUNTIME.mkdir(parents=True,exist_ok=True)
EVIDENCE=ROOT/'docs/issue-960-evidence/transactions'
MOVE=[dict(reference='R1',new_x=136.4,new_y=98.8,new_rotation=270)]
def sha(p):return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest() if pathlib.Path(p).is_file() else None
def pose(p):
    fp=parse_kicad_pcb(str(p)).footprints['R1'];return [fp.x,fp.y,fp.rotation]
def cli(args):
    r=subprocess.run([sys.executable,'-X','utf8',*map(str,args)],capture_output=True,text=True)
    return {'command':subprocess.list2cmdline([sys.executable,'-X','utf8',*map(str,args)]),'code':r.returncode,'stdout':r.stdout,'stderr':r.stderr}
def create(name):
    d=RUNTIME/name;work=d/'work';truth=d/'truth'
    staged=cli(['tests/stress/stage_unaided.py','kicad_files/esp_prog.kicad_pcb',work,truth]);assert staged['code']==0,staged
    return work,work/'board.kicad_pcb',staged
def candidate(b,tmp):
    c=pathlib.Path(tmp)/'candidate.kicad_pcb';write_placed_output(str(b),str(c),MOVE);copy_siblings(str(b),str(c));return c
def catch(fn):
    try:fn();return None
    except BaseException as e:return {'type':type(e).__name__,'error':str(e),'details':getattr(e,'details',None),'summary':getattr(e,'summary',None)}
def rows(d):return pv.read_ledger(str(d))
results={'revision':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'tree':subprocess.check_output(['git','rev-parse','HEAD^{tree}'],text=True).strip(),'cases':{}}
# External staging authorization, source and destination byte preservation.
d,b,st=create('undeclared');dest=d/'out.kicad_pcb'
with tempfile.TemporaryDirectory(prefix='960-final-') as t:
    c=candidate(b,t);error=catch(lambda:pose_ops._promote(str(c),str(dest)))
results['cases']['undeclared']={'stager':st,'error':error,'exists':dest.exists(),'rows':rows(d)}
assert error and error['type']=='UnaidedViolation' and not dest.exists() and not rows(d)
# Ledger publication fails before rename and immediately after successful rename.
for after in (False,True):
    name='ledger_after_replace' if after else 'ledger_before_replace';d,b,st=create(name);dest=d/'out.kicad_pcb';shutil.copy2(b,dest);old=sha(dest);original=os.replace;fired=[]
    def failed(src,dst):
        if pathlib.Path(dst).name==pv.LEDGER_NAME and not fired:
            fired.append(True)
            if after:original(src,dst)
            raise OSError('injected '+name)
        return original(src,dst)
    with pv.declare_lever('place_pose.py'),patch('os.replace',failed):error=catch(lambda:write_placed_output(str(b),str(dest),MOVE))
    doc={'error':error,'before':old,'after':sha(dest),'rows':rows(d),'pending':list(pv._PENDING)};results['cases'][name]=doc
    assert error and old==sha(dest) and not rows(d) and not pv._PENDING,doc
# An interrupt immediately after sibling replacement must restore both files.
d,b,st=create('interrupt');dest=d/'out.kicad_pcb';shutil.copy2(b,dest);sibling=dest.with_suffix('.kicad_pro');sibling.write_text('{"old":true}');before={str(p):sha(p) for p in (dest,sibling)};original=os.replace;fired=[]
with tempfile.TemporaryDirectory(prefix='960-final-interrupt-') as t:
    c=candidate(b,t);c.with_suffix('.kicad_pro').write_text('{"new":true}')
    def interrupted(src,dst):
        original(src,dst)
        if pathlib.Path(dst)==sibling and not fired:fired.append(True);raise KeyboardInterrupt('injected after successful sibling replacement')
    with pv.declare_lever('place_pose.py'),patch('os.replace',interrupted):error=catch(lambda:pub.publish_board(str(c),str(dest),input_file=str(b)))
after={str(p):sha(p) for p in (dest,sibling)}
results['cases']['interrupt_after_replace']={'error':error,'before':before,'after':after,'rows':rows(d),'pending':list(pv._PENDING)}
assert before==after and not rows(d) and not pv._PENDING
# Exclusive regime lock rejects concurrent cooperating publication; it cannot
# inherit the declaration in the other thread. Writer A holds after candidate
# preparation; writer B must refuse rather than replacing any bytes.
d,b,st=create('concurrent');dest=d/'out.kicad_pcb';entered=threading.Event();release=threading.Event();original=shutil.copyfile;outcomes={}
def copying(src,dst,*args,**kw):
    result=original(src,dst,*args,**kw)
    if threading.current_thread().name=='owner' and pathlib.Path(dst).name.startswith('.krt-candidate-'):
        entered.set();release.wait(10)
    return result
def owner():
    with pv.declare_lever('place_pose.py'):outcomes['owner']=catch(lambda:write_placed_output(str(b),str(dest),MOVE))
with patch('shutil.copyfile',copying):
    th=threading.Thread(target=owner,name='owner');th.start();assert entered.wait(10)
    outcomes['other_thread_active']=pv.active_lever()
    with pv.declare_lever('place_pose.py'):outcomes['contender']=catch(lambda:write_placed_output(str(b),str(dest),[dict(MOVE[0],new_x=137.4)]))
    release.set();th.join(10)
outcomes.update(pose=pose(dest),rows=rows(d));results['cases']['concurrent']=outcomes
assert outcomes['owner'] is None and outcomes['other_thread_active'] is None and outcomes['contender'] and outcomes['pose'][0]==136.4 and len(rows(d))==1,outcomes
# Pending duplicate refusal and explicit cancellation.
d,b,st=create('pending');dest=d/'out.kicad_pcb'
with pv.declare_lever('place_pose.py'):
    pv.record_write(str(b),str(dest),MOVE,pending=True)
    error=catch(lambda:pv.record_write(str(b),str(dest),MOVE,pending=True))
cancel=pv.cancel_write(str(dest));results['cases']['pending']={'duplicate_error':error,'cancelled':bool(cancel),'remaining':list(pv._PENDING),'rows':rows(d)}
assert error and cancel and not pv._PENDING and not rows(d)
# Deep real regime discovery must refuse undeclared write.
d,b,st=create('deep');deep=d
for _ in range(25):deep=deep/'d'
deep.mkdir(parents=True);dest=deep/'out.kicad_pcb';error=catch(lambda:write_placed_output(str(b),str(dest),MOVE))
results['cases']['deep']={'error':error,'exists':dest.exists(),'regime':pv.regime_for(str(dest))}
assert error and error['type']=='UnaidedViolation' and not dest.exists()
EVIDENCE.mkdir(parents=True,exist_ok=True);(EVIDENCE/(LABEL+'-adversarial.json')).write_text(json.dumps(results,indent=2,default=str));print(json.dumps({k:'PASS' for k in results['cases']},indent=2))
# Interrupt after a pending row has been installed but before the caller's
# assignment receives it; cancellation must be based on ownership, not a flag.
d,b,st=create('pending_interrupt');dest=d/'out.kicad_pcb';original=pv.record_write;fired=[]
def pending_interrupted(*args,**kwargs):
    row=original(*args,**kwargs)
    if row is not None and not fired:fired.append(True);raise KeyboardInterrupt('injected immediately after pending installed')
    return row
with pv.declare_lever('place_pose.py'),patch.object(pv,'record_write',pending_interrupted):error=catch(lambda:write_placed_output(str(b),str(dest),MOVE))
results['cases']['pending_interrupt']={'error':error,'pending':list(pv._PENDING),'exists':dest.exists(),'rows':rows(d)}
(EVIDENCE/(LABEL+'-adversarial.json')).write_text(json.dumps(results,indent=2,default=str))
assert error and not pv._PENDING and not dest.exists() and not rows(d),results['cases']['pending_interrupt']
print('pending_interrupt PASS')
# Actual back-layer write through alternate shared writer, holding x/y/rotation.
import pcbnew
d,b,st=create('side_flip');dest=d/'out.kicad_pcb';fp=parse_kicad_pcb(str(b)).footprints['R1']
with pv.declare_lever('place_pose.py',decision_source='model'):
    write_placed_output(str(b),str(dest),[dict(reference='R1',new_x=fp.x,new_y=fp.y,new_rotation=fp.rotation,new_side='B')])
native=pcbnew.LoadBoard(str(dest));part=next(f for f in native.GetFootprints() if f.GetReference()=='R1');r=rows(d)[-1];audit=cli(['tests/stress/provenance_audit.py','--workdir',d,'--delivered',dest])
results['cases']['side_flip']={'native':dict(x=pcbnew.ToMM(part.GetPosition().x),y=pcbnew.ToMM(part.GetPosition().y),rotation=part.GetOrientationDegrees(),side=native.GetLayerName(part.GetLayer()),flipped=part.IsFlipped()),'row':r,'audit':audit,'board_sha256':sha(dest)}
(EVIDENCE/(LABEL+'-adversarial.json')).write_text(json.dumps(results,indent=2,default=str))
assert part.IsFlipped() and r['sides_written']['R1']=='B' and r['board_sha256']==sha(dest) and r['decision_source']=='model' and audit['code']==0,results['cases']['side_flip']
print('side_flip PASS')
# Contender starts before owner installs pending, then loses the directory
# lock after installation. It must not cancel a token it does not own.
d,b,st=create('pending_contention');dest=d/'out.kicad_pcb';owner_ready=threading.Event();contender_ready=threading.Event();pending_installed=threading.Event();contender_done=threading.Event();original_record=pv.record_write;original_mkdir=os.mkdir;outcomes={}
def record_pause(*a,**kw):
 if threading.current_thread().name=='pending-owner':
  owner_ready.set();assert contender_ready.wait(10)
  row=original_record(*a,**kw);pending_installed.set();assert contender_done.wait(10);return row
 return original_record(*a,**kw)
def mkdir_pause(path,*a,**kw):
 if threading.current_thread().name=='pending-contender' and pathlib.Path(path).name=='.pose-publication-lock':
  contender_ready.set();assert pending_installed.wait(10)
 return original_mkdir(path,*a,**kw)
def owner_pending():
 with pv.declare_lever('place_pose.py'):outcomes['owner']=catch(lambda:write_placed_output(str(b),str(dest),MOVE))
def contender_pending():
 with pv.declare_lever('place_pose.py'):outcomes['contender']=catch(lambda:write_placed_output(str(b),str(dest),[dict(MOVE[0],new_x=137.4)]))
 outcomes['pending_survived']=pv._key(str(dest)) in pv._PENDING;contender_done.set()
with patch.object(pv,'record_write',record_pause),patch('os.mkdir',mkdir_pause):
 owner=threading.Thread(target=owner_pending,name='pending-owner');owner.start();assert owner_ready.wait(10)
 contender=threading.Thread(target=contender_pending,name='pending-contender');contender.start();contender.join(12);owner.join(12)
outcomes.update(rows=rows(d),pose=pose(dest));results['cases']['pending_contention']=outcomes
assert outcomes['owner'] is None and outcomes['contender'] and outcomes['pending_survived'] and len(rows(d))==1 and pose(dest)[0]==136.4,outcomes
# An audit owns exclusion for its whole read. A concurrent writer must refuse
# and leave the coherent audited board/ledger untouched.
spec=importlib.util.spec_from_file_location('independent_audit',ROOT/'tests/stress/provenance_audit.py');audit_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit_module)
d,b,st=create('audit_contention');dest=d/'out.kicad_pcb'
first=cli(['py_placer/place_pose.py',b,dest,'set','R1','136.4','98.8','--rot','270']);assert first['code']==0,first
reading=threading.Event();read_release=threading.Event();original_poses=audit_module.poses;outcomes={}
def reading_pause(*a,**kw):reading.set();assert read_release.wait(10);return original_poses(*a,**kw)
def audit_owner():outcomes['audit']=audit_module.audit(str(d),str(dest))
with patch.object(audit_module,'poses',reading_pause):
 reader=threading.Thread(target=audit_owner);reader.start();assert reading.wait(10)
 with pv.declare_lever('place_pose.py'):outcomes['writer']=catch(lambda:write_placed_output(str(dest),str(dest),[dict(MOVE[0],new_x=137.4)]))
 read_release.set();reader.join(12)
outcomes.update(pose=pose(dest),rows=rows(d));results['cases']['audit_contention']=outcomes
(EVIDENCE/(LABEL+'-adversarial.json')).write_text(json.dumps(results,indent=2,default=str))
assert outcomes['audit'][0]==0 and outcomes['writer'] and pose(dest)[0]==136.4 and len(rows(d))==1,outcomes
print('pending_contention PASS; audit_contention PASS')
