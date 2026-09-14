import json,pathlib,subprocess,sys
import pcbnew
root=pathlib.Path.cwd();label=sys.argv[1] if len(sys.argv)>1 else 'candidate4-legacy-lock';work=root/'reviewer-runtime'/label/'work';truth=work.parent/'truth'
def run(a):
 r=subprocess.run([sys.executable,'-X','utf8',*map(str,a)],capture_output=True,text=True);return dict(command=subprocess.list2cmdline(a),code=r.returncode,stdout=r.stdout,stderr=r.stderr)
stage=run(['tests/stress/stage_unaided.py','kicad_files/esp_prog.kicad_pcb',work,truth]);assert stage['code']==0,stage
out=work/'out.kicad_pcb';placed=run(['py_placer/place_pose.py',work/'board.kicad_pcb',out,'set','R1','136.4','98.8','--rot','270']);assert placed['code']==0,placed
ledger=work/'.pose-provenance.jsonl';row=json.loads(ledger.read_text().splitlines()[-1]);legacy={k:v for k,v in row.items() if k not in ('locks_written','candidate_sha256','final_snapshot','applied_by','decision_source','authorship_policy')};legacy['refs_written']=['R1'];legacy['poses_written']={'R1':row['poses_written']['R1']};legacy['sides_written']={};ledger.write_text(json.dumps(legacy)+'\n')
legacy_control=run(['tests/stress/provenance_audit.py','--workdir',work,'--delivered',out]);assert legacy_control['code']==0,legacy_control
board=pcbnew.LoadBoard(str(out));fp=next(f for f in board.GetFootprints() if f.GetReference()=='R1');fp.SetLocked(True);pcbnew.SaveBoard(str(out),board)
audit=run(['tests/stress/provenance_audit.py','--workdir',work,'--delivered',out]);result=dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),shape='legacy row modeled by dropping unavailable lock/snapshot fields and retaining only moved R1 pose; then native KiCad toggles only lock',legacy_row=legacy,legacy_control=legacy_control,audit=audit)
(root/'docs/issue-960-evidence/transactions'/f'{label}.json').write_text(json.dumps(result,indent=2));print(json.dumps(audit,indent=2))

# Hold x/y/rotation and reset the lock, then change only layer side using
# native KiCad. Legacy sides_written={} carries no claim for this change.
fp.SetLocked(False);fp.Flip(fp.GetPosition(),False);fp.SetOrientationDegrees(270);pcbnew.SaveBoard(str(out),board)
side_audit=run(['tests/stress/provenance_audit.py','--workdir',work,'--delivered',out]);result['side_audit']=side_audit;result['native_side_after']=board.GetLayerName(fp.GetLayer());result['native_rotation_after']=fp.GetOrientationDegrees()
(root/'docs/issue-960-evidence/transactions'/f'{label}.json').write_text(json.dumps(result,indent=2));print(json.dumps(side_audit,indent=2))

assert audit['code']==5 and side_audit['code']==5 and result['native_side_after']=='B.Cu' and result['native_rotation_after']%360==270


