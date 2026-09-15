import json,pathlib,sys
out=pathlib.Path(sys.argv[1]);r=json.loads((out/'results.json').read_text());registered=r['registered_ledger'];assert len(registered)==1,registered
row=registered[0];native=r['registered_native'];assert [native['x'],native['y'],native['rot']]==[136.4,98.8,270.0],native
assert row['board_sha256']==native['sha256'],row
assert row['poses_written']['R1']==[136.4,98.8,270.0],row
assert row['lever']=='place_pose.py',row
assert r['registered_summary']['no_worse'] and not r['registered_summary']['legal'] and not r['registered_summary']['forced'],r['registered_summary']
audit=json.loads((out/'registered_audit.json').read_text());assert audit['verdict']=='CLEAN' and audit['unclaimed_refs']==[],audit
assert r['undeclared_direct']['exception']=='UnaidedViolation' and not r['undeclared_direct']['output_exists'],r['undeclared_direct']
bypass=r['undeclared_promote'];assert bypass['exception']=='UnaidedViolation' and not bypass['output_exists'],bypass
assert bypass['ledger']==registered,bypass
assert all(r['initial_files_preserved'].values()) and r['source_preserved'],r['initial_files_preserved']
assert json.loads((out/'direct_registered_audit.json').read_text())['verdict']=='CLEAN'
print('PASS: actual CLI, final native pose/hash, single final row, audit CLEAN, no-worse is not clean, destination guard before output, declared direct control, original file preservation')
