# Independent final transaction review

Reviewer 2 independently reproduced parent failures before evaluating the implementation and owns no production changes. On integrated revision `4d7d2947d4af3d91c66f5680a535e06e75f7d841`, all independent transaction probes passed. The earlier integrated revision `58c62b1c30fb2c78dc619b53bd8e13309d37566f` still allowed undeclared `place_reconstruct._promote_staged` publication; that concrete discrepancy was reported and resolved before these reruns.

Exact final commands from the isolated reviewer worktree:

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-960-evidence/transactions/final_adversarial.py candidate5
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-960-evidence/transactions/final_crash.py candidate5-crash
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-960-evidence/transactions/final_failures.py candidate5b-failures
```

`candidate5-adversarial.json` records eight core transaction cases plus actual side-flip output:

- Externally staged undeclared `_promote` is refused before output creation, with zero rows.
- Ledger replacement failure both before and immediately after the real rename restores the prior board and emits no successful row; no pending token remains.
- `KeyboardInterrupt` immediately after actual sibling replacement restores both prior files.
- An overlapping cooperating writer refuses while the owner succeeds at its own requested coordinates. Another thread does not inherit the owner's declaration.
- Duplicate pending registration refuses; explicit cancellation clears the entry. Interruption immediately after pending installation also cancels the owned entry.
- An armed regime 25 ancestor directories away is discovered and refuses undeclared writing.
- An actual F-to-B layer flip through the shared writer holds R1 x/y/rotation, has native KiCad `IsFlipped=true` and `B.Cu`, matches `sides_written`, and reconciles to the final SHA. `decision_source=model` is retained; the real audit returns 0. This side-flip probe is an accounting/geometry-identity test, not an engineering-legal placement claim.

`candidate5-crash.json` runs the real `place_pose.py` CLI in a subprocess, kills that process with `os._exit(91)` immediately after actual final-board replacement, and verifies changed output bytes, real audit exit 5 (recovery required), refused subsequent CLI publication (exit 2), and three retained journal/backup files. The process is actually terminated; a power failure is not simulated.

`candidate5b-failures.json` establishes additional boundaries:

- Reconstruct's independent promotion helper refuses undeclared external candidates. Its registered control publishes a final matching SHA and audits CLEAN.
- A failed final-board replacement followed by failed project restoration leaves the board unchanged and project changed, reports `output_state=partial`, retains a backup with the exact old project SHA, and audits UNPROVEN. No successful row is added.
- Cleanup failure after successful board/ledger commit says `output_state=committed`, retains evidence and one accurate successful row, and audits UNPROVEN until recovery is resolved. Cleanup failure after rollback says `unchanged` and emits no row.
- Failure immediately after replacing an already-populated ledger during an in-place update restores both the entire old ledger SHA and old deliverable SHA; the original successful row remains.
- A native KiCad mutation changing only R1's lock after a registered moved pose is detected by the real audit as exit 4 drift, while x/y/rotation are unchanged.

The supported guarantee is recoverable publication by cooperating publishers, with destination authorization before deliverable mutation, final snapshots after accepted candidate transformations, and persistent journal evidence for the tested process-death window. It is not a security boundary against arbitrary raw filesystem edits or fabricated declarations, not an atomic snapshot for readers ignoring publication locks, and not a power-loss durability guarantee. Crash/recovery markers deliberately block subsequent publication and make audit UNPROVEN until an operator reconciles actual files. A postcommit cleanup failure is disclosed as committed; treating its accurate row as a failed promotion would itself be false provenance.

Parent reproduction established the exact improving-pile move remains non-clean in engineering terms. CLEAN in these reports always means provenance reconciliation, never a claim of clean routing, complete PCB engineering validity, better layout quality, or improved Astra performance.


The final rerun also exercises two distinct exclusion races. A contender begins before the owner installs its pending row, waits at the regime-lock syscall until installation, then loses the lock; the owner's pending row survives and commits correctly. An audit pauses inside its actual pose read while owning the regime lock; a concurrent registered writer refuses, and the audit finishes CLEAN on an unchanged coherent board and ledger. These are deterministic interleavings, not timing-probability measurements.

The repository mutation row was independently rerun at test-only revision `929e1945ab11deb2e64134a66ede770086d2adee` with:

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 tests/mutate_892.py --row a-failed-promote-is-not-atomic-again
```

It reports KILLED by `test_960_pose_publication.py`: one killed, zero survived, zero broken, zero disagreements. The battery first passes its unmutated control. `git diff --stat` is empty after restoration. At this revision `py_placer` tree identity is `8bbde66a16fba2686742fa532522adbdbf3a19d8` and `tests` is `1baeead481a7cdc059540d9e51a805400f17bd45`; the test-only commit preserves the fully exercised production tree from `37799d98982b33394dbb2d7cd5ddeb5e3354080b`.


Final audit revision `4d7d2947d4af3d91c66f5680a535e06e75f7d841` was independently exercised with the complete adversarial, subprocess-death, and recovery/cleanup suites again; all passed. The legacy-shaped row witness is captured in `candidate5c-legacy.json`, produced by:

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-960-evidence/transactions/legacy_lock.py candidate5c-legacy
```

The legacy coordinate-only control, with unchanged side and lock, still audits CLEAN. Native KiCad lock-only and then side-only edits, with x/y/rotation held, each audit UNPROVEN (5) because the historical row cannot establish that state. R1 is explicitly listed as unverifiable. An earlier reviewer assertion compared native `-90` directly with `270` and failed although the physical rotation was equivalent; the final assertion correctly compares modulo 360. This was a witness correction, not a production discrepancy. Fully covered new records still detect unauthorized lock changes as a violation, as the separate final failure suite demonstrates.


A further final-revision fault injection denies restoration of an existing ledger after its candidate replacement has already succeeded. In `candidate5b-failures.json` / `ledger_restore_failure`, the old board is successfully restored, but the ledger retains the old row plus an unresolved candidate row whose SHA differs from the restored board. The exact old ledger backup and transaction marker remain; the exception discloses `output_state=partial`, pending state is empty, and real audit returns UNPROVEN (5). Thus the no-added-success-row guarantee applies when rollback succeeds. Failed recovery can leave an unresolved candidate row and must be evaluated with the retained journal and actual files; neither this reviewer nor the implementation treats that row as a reconciled successful commit.

The same mutation row was rerun again after the final audit change, at `4d7d2947d4af3d91c66f5680a535e06e75f7d841`: KILLED by the current `test_960_pose_publication.py`, no survivors/broken/disagreements, and an empty tracked diff after restoration (`candidate5-mutation.log`).
