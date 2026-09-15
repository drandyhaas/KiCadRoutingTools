import hashlib,json,pathlib,shutil,sys,tempfile,threading
from unittest.mock import patch
root=pathlib.Path.cwd()
for p in ('py_placer','py_router','py_tools'):sys.path.insert(0,str(root/p))
from placement.pose_ops import _promote
from placement.writer import write_placed_output
from kicad_parser import parse_kicad_pcb
base=root/'reviewer-runtime/work/board.kicad_pcb';runtime=root/'reviewer-runtime/concurrent';runtime.mkdir(exist_ok=True);dest=runtime/'out.kicad_pcb';a=runtime/'a.kicad_pcb';b=runtime/'b.kicad_pcb'
for path,x in ((a,136.4),(b,137.4)):
    write_placed_output(str(base),str(path),[dict(reference='R1',new_x=x,new_y=98.8,new_rotation=270)])
a_copied=threading.Event();b_copied=threading.Event();a_finished=threading.Event();original=shutil.copyfile;result={}
def copying(src,dst,*args,**kwargs):
    name=threading.current_thread().name
    if str(dst).endswith('.krt-tmp'):
        if name=='B':a_copied.wait(5)
        value=original(src,dst,*args,**kwargs)
        if name=='A':a_copied.set();b_copied.wait(5)
        if name=='B':b_copied.set();a_finished.wait(5)
        return value
    return original(src,dst,*args,**kwargs)
def run(name,source):
    try:
        _promote(str(source),str(dest));fp=parse_kicad_pcb(str(dest)).footprints['R1'];result[name]={'success':True,'observed_R1':[fp.x,fp.y,fp.rotation]}
    except BaseException as e:result[name]={'success':False,'error':str(e)}
    finally:
        if name=='A':a_finished.set()
with patch('shutil.copyfile',copying):
    ta=threading.Thread(name='A',target=run,args=('A',a));tb=threading.Thread(name='B',target=run,args=('B',b));ta.start();tb.start();ta.join(10);tb.join(10)
(root/'docs/issue-960-evidence/transactions/parent-concurrent.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
