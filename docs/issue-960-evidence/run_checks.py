import concurrent.futures, json, os, pathlib, subprocess, sys, time
root=pathlib.Path(__file__).resolve().parents[2]
revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
out=root/'docs/issue-960-evidence/checks'
out.mkdir(exist_ok=True)
files=['tests/test_960_pose_publication.py','tests/test_provenance_audit.py','tests/test_892_place_pose.py','tests/test_892_registries.py','tests/test_903_stagers_arm_the_regime.py','tests/test_967_edge_floor.py','tests/test_714_perturb_layer_flip.py','tests/test_714_identity_write_unchanged.py','tests/test_714_refusals.py','tests/test_829_edge_cuts_owner.py','tests/test_place_seed.py','tests/test_place_reconstruct.py','tests/test_run4_reconstruct.py','tests/test_run27_seed_gate_pinned.py','tests/test_run27_seed_gate_shorts.py','tests/test_board_store_and_pose_score.py','tests/test_718_static_test_hygiene.py','tests/gui_parity/test_manifest_plan_parity.py','tests/gui_parity/test_cli_postpass_coverage.py','tests/gui_parity/test_967_pad_geometry_coverage.py','tests/run_doc_examples.py']
def run(file):
    interpreter='C:/Python313/python.exe' if 'static_test_hygiene' in file else sys.executable
    if file == 'tests/run_doc_examples.py':
        interpreter=str(root/'.venv-960-docs/Scripts/python.exe')
    command=[interpreter,'-X','utf8',file]; start=time.time()
    env=os.environ.copy()
    if file == 'tests/run_doc_examples.py':
        env['PYTHONPATH']=os.pathsep.join(str(root/p) for p in ('py_router','py_placer','py_tools'))
    try:
        result=subprocess.run(command,cwd=root,capture_output=True,timeout=900,env=env)
        log=result.stdout+b'\nSTDERR:\n'+result.stderr; code=result.returncode
    except subprocess.TimeoutExpired as e:
        log=(e.stdout or b'')+b'\nTIMEOUT\n'+(e.stderr or b'');code=124
    (out/(pathlib.Path(file).stem+'.log')).write_bytes(log)
    row=dict(command=command,exit=code,seconds=round(time.time()-start,2))
    if file == 'tests/run_doc_examples.py': row['PYTHONPATH']=env['PYTHONPATH']
    print(json.dumps(row),flush=True);return row
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool: rows=list(pool.map(run,files))
final_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
assert final_revision == revision, 'revision changed during verification'
doc=dict(revision=revision,python=sys.version,checks=rows)
(out/'results.json').write_text(json.dumps(doc,indent=2))
sys.exit(any(r['exit'] for r in rows))
