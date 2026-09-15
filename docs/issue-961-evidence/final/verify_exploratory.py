"""Binding connector requirements with genuine inherited dirty exploratory placement."""
import hashlib,json,subprocess,sys,time
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'py_placer').is_dir());sys.path[:0]=[str(ROOT/'py_router'),str(ROOT/'py_placer')]
from placement import provenance as pv
OUT=ROOT/'docs/issue-961-evidence/final'/('exploratory-'+time.strftime('%Y%m%d-%H%M%S'));OUT.mkdir(parents=True);work=OUT/'work';truth=OUT/'truth';report={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'commands':[]}
def run(label,*args):
 argv=[sys.executable,'-X','utf8',*map(str,args)];r=subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=180);(OUT/(label+'.log')).write_text(r.stdout+'\nSTDERR\n'+r.stderr,encoding='utf8');sums=[json.loads(t.split(': ',1)[1]) for t in r.stdout.splitlines() if t.startswith('JSON_SUMMARY: ')];row={'argv':argv,'exit':r.returncode,'summary':sums[-1] if sums else None};report['commands'].append(row);assert 'Traceback' not in r.stdout+r.stderr;return row
run('stage',ROOT/'tests/stress/stage_unaided.py',ROOT/'kicad_files/esp_prog.kicad_pcb',work,truth);b=work/'board.kicad_pcb';before=hashlib.sha256(b.read_bytes()).hexdigest();intent=work/'intent.json';intent.write_text(json.dumps({'schema':1,'kind':'floorplan-intent','units':'mm','edge_connectors':[{'ref':'USB1','edge':'west','overhang_mm':{'min':0,'max':.65},'max_setback_mm':.1}]}));out=work/'out.kicad_pcb'
r=run('classify',ROOT/'py_placer/place_reconstruct.py',b,out,'--intent',intent,'--stages','classify','--clearance','.25','--board-edge-clearance','.55');report.update(output_exists=out.exists(),rows=pv.read_ledger(str(work)),baseline_sha256=before,baseline_preserved=hashlib.sha256(b.read_bytes()).hexdigest()==before)
if out.exists():
 report['audit']=run('audit',ROOT/'tests/stress/provenance_audit.py','--workdir',work,'--delivered',out)
 report['grade']=run('grade',ROOT/'py_tools/check_floorplan.py',out,'--intent',intent,'--allow-unplaced','--clearance','.25','--board-edge-clearance','.55','--json',OUT/'grade.json')
(OUT/'results.json').write_text(json.dumps(report,indent=2));print(OUT);print(r['exit'],r['summary'].get('status'),r['summary'].get('engineering_clean'),r['summary'].get('final',{}).get('pad_conflicts'))
