"""Independent real publication + declared final geometry control for #961."""
import hashlib,json,subprocess,sys,time
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'py_placer').is_dir());sys.path[:0]=[str(ROOT/'py_router'),str(ROOT/'py_placer')]
import pcbnew
from placement import provenance as pv
from copy_board import copy_board
OUT=ROOT/'docs/issue-961-evidence/final'/('publication-'+time.strftime('%Y%m%d-%H%M%S'));OUT.mkdir(parents=True)
SOURCE=ROOT/'kicad_files/esp_prog.kicad_pcb';board=OUT/'baseline.kicad_pcb';copy_board(str(SOURCE),str(board))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
pv.start_regime(str(OUT),str(board));baseline=sha(board)
intent=OUT/'declared.intent.json';intent.write_text(json.dumps({'schema':1,'kind':'floorplan-intent','units':'mm','edge_connectors':[{'ref':'USB1','edge':'west','overhang_mm':{'min':.05,'max':.2},'max_setback_mm':0}]}));ih=sha(intent)
nat=pcbnew.LoadBoard(str(board));fp=next(f for f in nat.GetFootprints() if f.GetReference()=='USB1');original=[pcbnew.ToMM(fp.GetPosition().x),pcbnew.ToMM(fp.GetPosition().y),fp.GetOrientationDegrees()]
report={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'baseline_sha256':baseline,'intent_sha256':ih,'commands':[],'checks':[]}
def run(label,*args):
 argv=[sys.executable,'-X','utf8',*map(str,args)];r=subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=180);(OUT/(label+'.log')).write_text(r.stdout+'\nSTDERR\n'+r.stderr,encoding='utf8')
 row={'label':label,'argv':argv,'exit':r.returncode,'summaries':[json.loads(x.split(': ',1)[1]) for x in r.stdout.splitlines() if x.startswith('JSON_SUMMARY: ')]};report['commands'].append(row);assert 'Traceback' not in r.stdout+r.stderr;return row
out=OUT/'accepted.kicad_pcb';r=run('accepted',ROOT/'py_placer/place_pose.py',board,out,'--clearance','.2','--board-edge-clearance','.25','--strict-legal','set','USB1',original[0]-.1,original[1],'--rot',original[2]);assert r['exit']==0,r
summary=r['summaries'][-1];assert summary['legal'] and summary['no_worse'] and not summary['forced']
row=pv.read_ledger(str(OUT))[-1];assert row['board_sha256']==sha(out)==row['candidate_sha256'];report['ledger']=row
n=pcbnew.LoadBoard(str(out));f=next(f for f in n.GetFootprints() if f.GetReference()=='USB1');pose=[pcbnew.ToMM(f.GetPosition().x),pcbnew.ToMM(f.GetPosition().y),f.GetOrientationDegrees()];assert pose==[original[0]-.1,original[1],original[2]];report['native_pose']=pose
r=run('accepted-floorplan',ROOT/'py_tools/check_floorplan.py',out,'--intent',intent,'--clearance','.2','--board-edge-clearance','.25','--json',OUT/'accepted-floorplan.json');report['mechanical_exit']=r['exit']
r=run('accepted-drc',ROOT/'py_router/check_drc.py',out,'--check-pad-edge','--board-edge-clearance','.25','--clearance-margin','0');assert r['exit']==0,r
r=run('accepted-audit',ROOT/'tests/stress/provenance_audit.py','--workdir',OUT,'--delivered',out,'--json',OUT/'audit.json');assert r['exit']==0 and r['summaries'][-1]['verdict']=='CLEAN'
for label,dx,extra,expected in [('refused',-1.45,[],4),('dry',-.1,['--dry-run'],0)]:
 dest=OUT/(label+'.kicad_pcb');before=pv.read_ledger(str(OUT));r=run(label,ROOT/'py_placer/place_pose.py',board,dest,'--clearance','.2','--board-edge-clearance','.25','set','USB1',original[0]+dx,original[1],'--rot',original[2],*extra)
 assert r['exit']==expected and not dest.exists() and pv.read_ledger(str(OUT))==before,r
 if label=='refused':assert 'WORSE' in r['summaries'][-1]['refused']
assert sha(board)==baseline and sha(intent)==ih;report['source_and_requirements_preserved']=True
(OUT/'results.json').write_text(json.dumps(report,indent=2));print(OUT)
