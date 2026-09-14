# Final pose publication and provenance (#960)

This change preserves caller-authored placement decisions and records the accepted
written board. It does not select new coordinates or weaken placement legality.

Depends on #968. The parent was fetched from the published fork branch
`edgehero:fix/967-resolved-edge-clearance`, independently checked against GitHub,
at **cbc819b6b862f6ec14ecb30b8659819e800fce76**. It remains an ancestor of this new
branch, `fix/960-pose-provenance`. The implementation and final audit tests are
**4d7d2947d4af3d91c66f5680a535e06e75f7d841**. Documentation commits after that
revision preserve the production/test trees; the three publication identity
confirmations accompany the PR. The new upstream PR targets `main`, so it
includes parent commits while #968 remains open.

The isolated primary worktree is `KRT-960`; three reviewers used separate
detached worktrees. Existing worktrees, PR #968's branch and original fixtures
were preserved. [Source identities](source-identities.json) record GitHub state,
the issue/comment URLs, upstream main and the public fixture identity.

## Measured before and after

Both required failures were reproduced on the fetched parent using the real
`tests/stress/stage_unaided.py` and `tests/stress/provenance_audit.py`, with native
KiCad readback of public `kicad_files/esp_prog.kicad_pcb` copies. Its SHA256 is
`165302e6a4f7aacdd64b3174df27ed8ddb19fd5f1f7effadd8d92205e25a120e`.

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 tests/stress/stage_unaided.py kicad_files/esp_prog.kicad_pcb WORK TRUTH
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 py_placer/place_pose.py WORK/board.kicad_pcb WORK/pose.kicad_pcb set R1 136.4 98.8 --rot 270
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 tests/stress/provenance_audit.py --workdir WORK --delivered WORK/pose.kicad_pcb
```

| Case | Published parent | Verified implementation |
| --- | --- | --- |
| Registered exact R1 move | Exit 0, exact native pose; zero rows; audit 4, R1 unclaimed | Exit 0, same pose; one final snapshot with matching SHA; audit 0 |
| Undeclared external stage then real `_promote` | Publishes without refusal or row | `UnaidedViolation` before destination mutation |
| Same bypass over an existing board and declarations | Board and brief changed | Entire prior work-file identity map preserved |
| Registered direct writer control | Row and audit 0 | Final snapshot and audit 0 |
| Dry run, geometric refusal, strict dirty-board refusal, invalid multi-op | Refuse/no publication as appropriate | Same behavior, no false committed row |

The R1 move uses resolved copper clearance **0.25 mm** and edge clearance
**0.55 mm**. Pad conflicts remain **83 → 72**, total shortfall
**27.1331 → 22.5783 mm**; hole, pad-edge and out-of-board channels remain zero.
It is accepted without `--force`, with **no_worse=true, legal=false**. Audit
**CLEAN means provenance reconciles**, not engineering-clean placement, a
completed PCB, optimizer-selected coordinates or improved Astra performance.

The source board has no tracked project/rule/brief siblings. Separate copied
fixtures with explicit declarations exercise sibling preservation; their exact
contents and identities are recorded in the reviewer evidence.

## Publication contract

`placement.publication.publish_board` is shared by `place_pose`'s final promotion,
the direct placement writer, reconstruction promotion, seed's post-lock initial
output and route-loop delivery. Seed/reconstruction internal repair replacements
now use the transactional writer in place. Search algorithms and requested
coordinates/rotations are unchanged. Pose search trials stay outside the armed
destination and only the accepted candidate reaches final publication.

The publisher authorizes the **destination** before any deliverable mutation,
prepares unique candidates and backups, then snapshots the accepted board after
all pose, side and lock stamping. Rows record final poses, faces, locks, parent
SHA, candidate SHA and final written SHA. `applied_by` identifies execution;
`decision_source` is separately declared. `place_pose` says `caller`, since the
caller may be a model, human or harness; an explicit model declaration is
supported without inventing a model identity. Outside a benchmark regime,
undeclared checked adapters remain available and no benchmark ledger is created.

The baseline's SHA is preserved. An in-place write to the original baseline path
first freezes its exact bytes and siblings at a unique hidden path and retargets
the manifest within the same transaction. Later in-place writes retain that
baseline. Final snapshots carry inherited unchanged poses for reconciliation
without claiming that the last tool selected every coordinate. Lock/side drift
is checked. Legacy rows missing a claim for a changed lock or face return
UNPROVEN; the audit does not certify state the old ledger never recorded.

## Supported transaction guarantees and limits

- Cooperating publishers use exclusive directory creation to serialize the
  regime ledger and destination; simultaneous contention refuses rather than
  waiting or overwriting another writer's temporary file. Lever declarations
  are context-local. Pending duplicates refuse and cancellation removes only
  the current publisher's record. Pose and direct writer input identities detect
  changed source bytes/requirements at their publication boundary.
- The audit holds the same regime lock for its entire read. Readers that ignore
  these locks can observe intermediate board/sibling/ledger combinations;
  **multi-file atomic visibility is not claimed**.
- Catchable failures, including the tested KeyboardInterrupt immediately after
  a successful replacement syscall, attempt restoration of board, siblings,
  baseline manifest and ledger. Successful rollback preserves prior bytes and
  adds no successful row. The previous ledger is backed up, and a new complete
  ledger is installed by rename rather than appending a potentially torn row.
- Failed restoration reports `output_state=partial`, affected paths and retained
  recovery evidence. Successful commit followed by cleanup failure explicitly
  reports `output_state=committed`, with its accurate successful row retained;
  rollback followed by cleanup failure reports `unchanged`. These states must
  not be interpreted solely from an exception or exit code.
  A tested denial of ledger restoration leaves an unresolved candidate row
  alongside a restored old board; the previous ledger backup survives and audit
  returns UNPROVEN. No consistent-ledger guarantee is claimed when recovery
  itself is denied, and that stranded row is not a reconciled successful result.
- A process terminated after board replacement leaves the lock, journal and
  backups. Audit returns UNPROVEN and another publisher refuses. Recovery is
  manual; there is no automatic stale-lock takeover or replay. Journals record
  destination paths, backup paths, original SHA256 values and candidate SHA256
  values. A process killed before journaling may leave only an exclusion marker
  and temporary files; destination replacements start only after the journal.
- No power-loss durability guarantee, distributed/network-filesystem locking
  guarantee or security boundary against raw filesystem copying, forged
  declarations or external writers ignoring locks is claimed. Generic board
  storage/copying and live GUI pcbnew edits are not universally intercepted.
  Low-level `record_write`/`commit_write` callers must supply locking/recovery;
  production paths covered here use `publish_board`.

For manual recovery, first establish that no publisher/auditor is still active.
Retain a copy of the journal and all surviving candidates/backups. Compare each
actual target with the recorded original/candidate SHA; restore the complete
board, requirement siblings, baseline manifest and ledger consistently, or
retain and disclose the candidate state. An absent original is recorded as null.
Do not remove a marker merely to obtain a CLEAN verdict. Remove markers only
after reconciliation, then rerun the real audit. Failed recovery can leave
inconsistent files; the marker deliberately prevents certifying that state.

## Independent verification

- [Reviewer 1: reproduction and acceptance](reproduction/README.md): derives
  acceptance independently; reproduces both parent failures and verifies exact
  improving-pile, registered/direct/ordinary controls, overwrite refusal,
  in-place baseline/locks and seed final-lock identity.
- [Reviewer 2: transactions](transactions/final-review.md): exercises actual
  syscall-before/after failures, existing-ledger restoration, pending install
  interruption, competing writers, competing audit reads, deep regimes, native
  F/B flips, lock-only drift, cleanup failure and actual process termination.
  The independently discovered reconstruction bypass was fixed and rerun.
- [Reviewer 3: final placement](final-verifier/report.md) checks actual CLI paths and native final
  boards for direct/rotated/face poses, locks/unlocks, multi-part atomicity,
  strict snap, in-place writes, fixed mechanics and requirement siblings. It
  also checks seed/reconstruction/route-loop final delivery. Its report and
  runnable scripts are included beside the other two reports: **124 assertions
  across 41 actual CLI invocations** on the final behavioral revision.

[Focused check commands and outcomes](checks/results.json) come from
[run_checks.py](run_checks.py): **21 commands pass**, including 32 runnable
documentation examples (78 fragments skipped). KiCad 10.0.0 / bundled Python 3.11.5 is used for
native geometry and placement CLIs; Python 3.13 runs static hygiene because two
existing files use newer Python syntax. Documentation examples use a local
Python 3.13 virtual environment with NumPy, since KiCad's bundled interpreter
does not expose the examples' module imports through PYTHONPATH. Recreate it with
`C:/Python313/python.exe -m venv .venv-960-docs`, then
`.venv-960-docs/Scripts/python.exe -m pip install numpy==2.5.3`.
The local Rust dependency was built
with `build_router.py` from the inherited crate, without Rust source changes.
Native geometry parity and both routing parity gates are included. All 1,108
mutation anchors resolve; the relocated publication-recovery mutation was
independently killed and its worktree restored.

This is focused verification, not a full-suite or full-board DRC/connectivity
claim. The two historical reconstruction files report 4 and 9 skipped tests
because their untracked tigard fixtures are absent; their remaining assertions
ran, and separate public-board reconstruction CLI evidence is provided. Parent
#968's documented geometric/custom-rule coverage limits remain in force. No
layout-quality, routed-quality or model-performance improvement is inferred.

Reviewer scripts were relocated into this committed evidence directory. Their
logs preserve the original commands and paths. Run them from the repository
root using their current paths, for example
`docs/issue-960-evidence/reproduction/reproduce.py --output REVIEW-OUTPUT`.
Use fresh output directories; the fault-injection scripts intentionally retain
failed-recovery artifacts in their private test locations.
