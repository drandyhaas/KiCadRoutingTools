"""Independent #961 final-review driver. Run using KiCad Python from a clean checkout."""
import hashlib, json, os, subprocess, sys, time
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'py_placer').is_dir())
OUT=ROOT/'docs/issue-961-evidence/final'/('run-'+time.strftime('%Y%m%d-%H%M%S'))
OUT.mkdir(parents=True)
REV=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
COMMANDS=[
 ['tests/test_960_pose_publication.py'],
 ['tests/test_967_edge_floor.py'],
 ['tests/gui_parity/test_967_pad_geometry_coverage.py'],
 ['tests/gui_parity/test_966_unset_clearance.py'],
 ['tests/gui_parity/test_768_cap_ceiling_real_dialog.py'],
 ['tests/gui_parity/test_manifest_plan_parity.py'],
 ['tests/gui_parity/test_cli_postpass_coverage.py'],
 ['docs/issue-960-evidence/final-verifier/verify.py'],
 ['docs/issue-960-evidence/final-verifier/alternate.py'],
 ['docs/evidence/issue966/verify_reader_matrix.py','.',str(OUT/'reader-2'),'flat_hierarchy'],
 ['docs/evidence/issue966/verify_reader_matrix.py','.',str(OUT/'reader-4'),'glasgow_revC'],
 ['docs/evidence/issue966/verify_cap_boundary.py','.',str(OUT/'cap')],
]
report={'revision':REV,'python':sys.version,'commands':[], 'source_identities':{}}
for name in ['esp_prog','flat_hierarchy','glasgow_revC']:
 for p in (ROOT/'kicad_files').glob(name+'.*'):
  if p.is_file():report['source_identities'][str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
for args in COMMANDS:
 argv=[sys.executable,'-X','utf8',*args];print('RUN',args,flush=True)
 r=subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,encoding='utf8',timeout=900)
 logfile=OUT/(Path(args[0]).stem+('-'+args[-1] if 'reader' in args[0] else '')+'.log')
 logfile.write_text(r.stdout+'\nSTDERR\n'+r.stderr,encoding='utf8')
 report['commands'].append({'argv':argv,'exit':r.returncode,'log':str(logfile.relative_to(ROOT))})
 (OUT/'commands.json').write_text(json.dumps(report,indent=2),encoding='utf8')
 print('DONE',r.returncode,args[0],flush=True)
 # Parent scripts print counts but lack failure exit; independently inspect their report JSON.
 if args[0].endswith('/verify.py') or args[0].endswith('/alternate.py'):
  candidates=list((ROOT/'review_final').glob('*/results.json'))
  p=max(candidates,key=lambda x:x.stat().st_mtime);d=json.loads(p.read_text())
  report['commands'][-1]['behavior_passed']=d.get('passed');report['commands'][-1]['behavior_failed']=d.get('failed')
  (OUT/(Path(args[0]).stem+'-results.json')).write_bytes(p.read_bytes())
for path,before in report['source_identities'].items():
 assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==before,path
report['all_source_identities_preserved']=True
report['passed']=all(x['exit']==0 and x.get('behavior_failed',0)==0 for x in report['commands'])
(OUT/'commands.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(OUT,'PASS' if report['passed'] else 'FAIL')
sys.exit(0 if report['passed'] else 1)
