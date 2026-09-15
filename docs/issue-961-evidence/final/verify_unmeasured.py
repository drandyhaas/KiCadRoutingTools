"""Independent actual CLI missing-body certification control, using native KiCad edits."""
import hashlib,json,subprocess,sys,time
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'py_placer').is_dir());sys.path.insert(0,str(ROOT/'py_router'))
import pcbnew
from copy_board import copy_board
OUT=ROOT/'docs/issue-961-evidence/final'/('unmeasured-'+time.strftime('%Y%m%d-%H%M%S'));OUT.mkdir(parents=True)
board=OUT/'missing.kicad_pcb';copy_board(str(ROOT/'kicad_files/esp_prog.kicad_pcb'),str(board));b=pcbnew.LoadBoard(str(board));f=next(f for f in b.GetFootprints() if f.GetReference()=='USB1');before=len(list(f.Pads()))
removed=[]
for g in list(f.GraphicalItems()):
 if g.GetLayer() in (pcbnew.F_Fab,pcbnew.F_SilkS) and isinstance(g,pcbnew.PCB_SHAPE):removed.append(g.GetClass());f.Remove(g)
pcbnew.SaveBoard(str(board),b)
probe="import pcbnew,json,sys; b=pcbnew.LoadBoard(sys.argv[1]); f=next(f for f in b.GetFootprints() if f.GetReference()=='USB1'); print(json.dumps({'pads':len(list(f.Pads())),'front_body_shapes':len([g for g in f.GraphicalItems() if isinstance(g,pcbnew.PCB_SHAPE) and g.GetLayer() in (pcbnew.F_Fab,pcbnew.F_SilkS)])}))"
native=json.loads(subprocess.check_output([sys.executable,'-X','utf8','-c',probe,str(board)],text=True));assert native['pads']==before and native['front_body_shapes']==0
intent=OUT/'required.intent.json';intent.write_text(json.dumps({'schema':1,'kind':'floorplan-intent','units':'mm','edge_connectors':[{'ref':'USB1','edge':'west','overhang_mm':{'min':0,'max':.65},'max_setback_mm':.1}]}));ih=hashlib.sha256(intent.read_bytes()).hexdigest();report={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'native_removed_shapes':removed,'native_pad_count_preserved':before,'commands':[]}
for label,extra,code in [('strict',[],4),('explicit-exit-zero',['--exit-zero'],0)]:
 dest=OUT/(label+'.json');argv=[sys.executable,'-X','utf8',str(ROOT/'py_tools/check_floorplan.py'),str(board),'--intent',str(intent),'--clearance','.2','--board-edge-clearance','.25','--json',str(dest),*extra]
 r=subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=90);(OUT/(label+'.log')).write_text(r.stdout+'\nSTDERR\n'+r.stderr,encoding='utf8');assert r.returncode==code,(r.stdout,r.stderr)
 d=json.loads(dest.read_text());row=next(x for x in d['edge_seating'] if x['ref']=='USB1');assert not d['pass'] and not d['complete'];assert row['body_measured'] is False and row['body_overhang_mm'] is None and row['body_setback_mm'] is None and row['body_unmeasured_reason'];assert row['measurements']['body_overhang']['disposition']=='unmeasured';assert row['measurements']['pad_copper_edge_gap']['disposition']=='pass';assert row['copper_edge_complete']
 report['commands'].append({'argv':argv,'exit':r.returncode,'evidence':d})
assert hashlib.sha256(intent.read_bytes()).hexdigest()==ih;report['intent_sha256_preserved']=ih;report['board_sha256']=hashlib.sha256(board.read_bytes()).hexdigest();(OUT/'results.json').write_text(json.dumps(report,indent=2));print(OUT)

