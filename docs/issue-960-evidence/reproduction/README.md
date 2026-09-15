# Reviewer 1: independent reproduction and acceptance verification

Reviewed issue #960 and its complete expanded acceptance comment using GitHub CLI, and independently checked PR #968 was OPEN at published fork head cbc819b6b862f6ec14ecb30b8659819e800fce76. Read repository CLAUDE.md. Reproductions ran in an isolated detached worktree; production edits belonged to the primary agent.

Final behavioral revision: **4d7d2947d4af3d91c66f5680a535e06e75f7d841**. `verified-identity.json` records production/test/fixture tree identities and an empty tracked diff. After each earlier integrated production change, the affected checks were rerun; the committed bundle contains the parent reproduction and latest complete passing evidence.

## Methods and results

The existing public esp_prog board was preserved, SHA256 `165302e6a4f7aacdd64b3174df27ed8ddb19fd5f1f7effadd8d92205e25a120e`. KiCad 10.0.0 / Python 3.11.5 executed the real `tests/stress/stage_unaided.py`, `py_placer/place_pose.py`, and `tests/stress/provenance_audit.py`. `pcbnew.LoadBoard` independently read final written geometry, rotations and locks. JSON results include exact subprocess commands, effective parameters, raw ledger rows, source/baseline identities, native geometry, final hashes and audit outcomes; logs contain full stdout/stderr.

| Probe | Published parent | Final integrated revision |
|---|---|---|
| Real sanctioned `set R1 136.4 98.8 --rot 270` | Exit 0, exact native pose, no ledger; audit 4 UNAIDED VIOLATION, R1 unclaimed | Exit 0, exact native pose, one final snapshot row with final SHA; audit 0 CLEAN |
| Real undeclared writer outside regime, then actual `_promote` into armed fresh output | Published, no ledger, audit 4 | UnaidedViolation before output creation, ledger unchanged |
| Same undeclared promotion over existing board and project/rule/brief sentinels | Existing board and brief overwritten, no ledger | UnaidedViolation; entire armed file SHA map unchanged |
| Direct undeclared writer into regime | UnaidedViolation before output | Same correct refusal |
| Direct declared writer control | One row, audit 0 CLEAN | Correct final row, audit 0 CLEAN |
| Dry run / offboard geometric refusal / strict-clean improving-pile refusal / invalid second multi-op | Exit 0 / 4 / 4 / 2; no output, no ledger; initial files preserved | Same outcomes and reasons; initial files preserved |
| In-place set+lock, then unlock+rotate | Not part of parent reproduction | Exact native final pose/rotation/locks, final/parent SHA reconciliation, original baseline frozen with exact hash, audit 0 CLEAN after each |
| Ordinary undeclared writer + actual promotion outside regime | Not part of parent reproduction | Allowed, exact native requested pose, no benchmark ledger |
| Explicit known-model decision metadata | Not part of parent reproduction | `decision_source=model` preserved separately from `applied_by=place_pose.py`; ordinary CLI default remains truthful `caller` |
| Actual stage + seed CLI with R1 must_lock | Not part of parent reproduction | Seed 0 with `--no-polish`: exit 0, one row, native R1 locked, final SHA reconciles, audit 0 CLEAN |

The documented improving-pile move still uses no force: `forced=false`, `no_worse=true`, **`legal=false`**. Effective defaults remain clearance 0.25 mm and board edge clearance 0.55 mm. Measured pad conflicts remain 83 -> 72, pad shortfall 27.1331 -> 22.5783 mm, with zero hole/pad-edge/out-of-board findings. The source pose is a declared known control, not a newly discovered layout.

The seed control uses a valid minimal intent `{"schema":1,"kind":"floorplan-intent","units":"mm","must_lock":["R1"]}`. Its native R1 is locked at [129.875,98.25,0], and the final board hash agrees with the one ledger row after lock stamping. The exact emitted intent and its identity are included with the final seed evidence.

## Reproduction

From the repository root, with the evidence scripts at this directory (replace SCRIPT_DIR and RUN_DIR with absolute paths):

```
"C:/Program Files/KiCad/10.0/bin/python.exe" -X utf8 SCRIPT_DIR/reproduce.py --repo . --output RUN_DIR/reproduction
"C:/Program Files/KiCad/10.0/bin/python.exe" -X utf8 SCRIPT_DIR/assert_fixed.py RUN_DIR/reproduction
"C:/Program Files/KiCad/10.0/bin/python.exe" -X utf8 SCRIPT_DIR/controls.py --repo . --output RUN_DIR/controls
"C:/Program Files/KiCad/10.0/bin/python.exe" -X utf8 SCRIPT_DIR/existing_guard.py --repo . --output RUN_DIR/existing --expect-fixed
"C:/Program Files/KiCad/10.0/bin/python.exe" -X utf8 SCRIPT_DIR/inplace_ordinary.py --repo . --output RUN_DIR/inplace
"C:/Program Files/KiCad/10.0/bin/python.exe" -X utf8 SCRIPT_DIR/seed_control.py RUN_DIR/seed
```

`reproduce.py` invokes both the direct writer and actual `_promote` inside its own process; those exact API calls are in the script. Run `existing_guard.py` without `--expect-fixed` on the published parent. `assert_fixed.py` is intentionally a final-revision assertion, so its missing-row expectation fails on the parent.

## Boundaries

The esp_prog fixture has no tracked project/rule/brief siblings. Existing-output sentinel siblings exercise authorization and publication preservation only; they are explicitly not a geometric grading claim. Other reviewers cover actual requirement-bearing projects and full snap/face/multi-part behavior. Transaction failure/crash/concurrency conclusions require the separate transaction reviewer's adversarial evidence; this review does not substitute positive/control cases for those probes. Audit CLEAN is an accounting verdict, never an assertion of engineering-clean PCB placement, optimizer coordinate authorship, improved PCB quality or better model performance.

Before implementation, independent acceptance also identified the need to record accepted snapped/locked/sided final bytes, cancel rejected pending writes, refuse external promotion before mutation, preserve baseline identity during in-place writes, and accurately disclose failed recovery. These criteria were communicated to the primary agent; the positive findings above are bounded by the probes actually run.

## Final legacy-audit correction

The final change from 37799d9 to 4d7d294 changes the audit only; production and fixture tree identities are independently equal. Actual registered/direct audit, in-place lock+unlock/rotate audit and seed lock/audit controls were rerun on detached 4d7d294 and remain passing. The unaffected no-output refusal and existing-destination guard evidence remains at 37799d9.

Independently reproduced the reported legacy hole before accepting its correction: create an actual registered CLI output, downgrade its row by removing locks_written/final_snapshot/candidate_sha256 to model historical schema-1 state, then use native KiCad to lock R1 without a declared write. At 37799d9 the real audit returned CLEAN (exit 0) despite that uncovered lock change. At 4d7d294 it returns UNPROVEN (exit 5), naming missing pose/side/lock coverage. Native output pose and lock, the original and downgraded emitted row, exact commands and audit result are in legacy-lock-before-37799d9 and final-legacy-lock-4d7d294. `legacy_lock.py RUN_DIR UNPROVEN` reproduces the final case; use CLEAN for the known prior weakness. This is a truthful limitation of historical rows, not a new successful benchmark claim.
