# Independent final placement and regression verification

Reviewer 3 worked in detached isolated worktree `KRT-960-final-verifier`, read issue #960 and its complete expanded comment, PR #968, CONTRIBUTING.md and CLAUDE.md independently, and made no production changes. Parent checked on GitHub: open #968, head `cbc819b6b862f6ec14ecb30b8659819e800fce76`. Final behavioral revision: `4d7d2947d4af3d91c66f5680a535e06e75f7d841`.

Environment: Windows, KiCad 10.0 bundled Python 3.11.5, native `pcbnew`. Public source `kicad_files/esp_prog.kicad_pcb`, SHA256 `165302e6a4f7aacdd64b3174df27ed8ddb19fd5f1f7effadd8d92205e25a120e`. Only copies were changed. Fixture, baseline, project/rule/brief identities, exact argv, stdout/stderr, final boards' native poses, complete emitted ledger rows and audit results are in the JSON evidence. The native snapshots identify anonymous/duplicate footprints by UUID; ledger aliases such as `Ref*~2` are reconciled separately from KiCad's duplicate display references.

## Before and after

The reviewer independently ran the real stager and exact documented `set R1 136.4 98.8 --rot 270` on the published parent. The CLI resolved copper clearance .25 mm and board-edge clearance .55 mm. It wrote R1 at exactly `[136.4,98.8,270]`, with `forced:false`, `no_worse:true`, `legal:false`; pad conflicts fell 83→72 and total shortfall 27.1331→22.5783 mm. There were zero ledger rows and real provenance audit exited 4. `parent-reproduction.json` retains this bounded observation, rather than invalid exploratory control attempts.

On the final revision the same stager and move produce one complete final snapshot with matching board SHA and audit CLEAN. The measured placement remains engineering-unclean; provenance CLEAN makes no engineering claim.

`final-main.json`: **96 assertions passed**, across **28 real CLI invocations**, including staging and auditing. Accepted outputs are inspected with native KiCad, not inferred from process exit or JSON alone.

| Behavior | Independently verified result |
|---|---|
| Explicit set and absolute rotation | Exact requested final coordinates/rotation; final SHA and native lock/side match the ledger. |
| Face-direction application | `face U1 N R1 --force` rotates the selected row toward R1; native pad offsets for pads 11–20 point east. This one orientation-control arm deliberately uses and verifies the existing explicit waiver because the proposed turn worsens the pile. It is not an unforced legality control. |
| Lock and unlock | Native lock state and `locks_written` match; final SHA matches post-stamping bytes; audits CLEAN. |
| Coordinated set+rotate | Both requested parts land in the same written board and reconcile to its final snapshot. |
| In-place delivered-board write | Final board and ledger SHA reconcile; audit CLEAN. |
| In-place original staged-board write | Accepted. Original baseline bytes and all original siblings are copied to the preserved manifest-targeted snapshot; `staged_sha256` retains the original identity; delivered board audit CLEAN. |
| Dry run and geometric refusal | No new output and no new successful row. Off-board refusal names worsening geometry. |
| Strict snap inherited from #968 | At explicit .25/.55 mm, request Y1 `[124.7,103.21,270]` snaps to `[124.7,103.06,270]`; final snapshot records the accepted pose. Native effective F.Cu polygons have minimum .64-mm edge gap. |
| Exact direct multi boundary | Y1 `[124.7,103.15,270]` plus C2's coordinated pose passes; native minimum edge gap is .55 mm. |
| Atomic multi-part refusal | Introducing the .50-mm gap refuses the whole in-place request for pad-edge worsening; existing board, all siblings and ledger stay unchanged. |
| Fixed mechanics and requirements | All 17 fixed footprints are unchanged by native UUID/position/rotation/side/lock; project, copper-rule and brief bytes travel unchanged. The main brief is a byte-preservation sentinel, not an assertion that its contents were compiled; the alternate suite uses a valid brief and real intent compiler. |
| Output-only project/rule/brief | Each separately refuses before publication and preserves the existing requirement bytes. |
| Real Windows readonly final board | Publication fails after sibling processing; rollback preserves prior board and all three siblings and ledger, reporting `output_state:unchanged`. |
| Decision versus execution | `applied_by:place_pose.py` and `decision_source:caller` remain separate. The tool does not claim an optimizer chose the supplied coordinates. |

## Alternate final publication paths

`final-alternate.json`: **28 assertions passed**, across **13 real CLI invocations**. Each accepted armed output has a native final pose/side/lock match, its final SHA in the ledger, and real audit CLEAN:

- Real `stage_unaided.py`, real `check_floorplan.py --emit-intent`, then `place_seed.py --intent ... --no-polish --anchors-first`: seeded output, including final native locks, reconciles.
- `place_reconstruct.py --stages classify`: exercises actual final publication with unchanged geometry. It does not claim coverage of every reconstruct search or repair candidate.
- Real valid brief→intent compilation, then `place_seed.py --repair`: exercises the repair-mode final publication on the copied source board.
- `place_route_loop.py --rounds 0 --route-args '--nets "Net-(Q2-Pad1)" --max-iterations 1000'`: runs actual `route.py` on a selected two-pad net and verifies final loop publication. It does not test placement search rounds or certify routed connectivity/DRC.
- Ordinary unarmed `place_pose.py ... rotate R1 90` remains available with exact rotation and no fabricated benchmark ledger. Audit returns UNPROVEN because no benchmark was armed.

## Regressions and review findings

Independently run `tests/test_892_place_pose.py`: **134 assertions pass**; `tests/test_967_edge_floor.py`: **11 tests pass** on `37799d98982b33394dbb2d7cd5ddeb5e3354080b`, whose complete `py_placer` tree is identical to the final behavioral revision. Final affected audit/transaction regressions were rerun on `4d7d2947`: `tests/test_960_pose_publication.py` **11 tests pass**, `tests/test_provenance_audit.py` **65 assertions pass**. Logs are retained.

Review found two stale regression assumptions during integration: an old fixed `.krt-tmp` collision no longer prevents a publisher using unique temporary files, and an unregistered-lever diagnostic had lost its `LEVER_REGISTRY` wording. Both were resolved in the final candidate and the affected tests rerun. Initial alternate harness controls were corrected for required `--intent`, valid brief schema and duplicate-reference aliases before treating them as behavioral evidence; they were not product failures.

The full main and alternate CLI suites were rerun after the final audit change. Final production/test tree identities are recorded in `identities.json`. No unresolved discrepancy remains in this review's exercised scope. The other independent transaction reviewer separately exercises actual F→B layer flips, concurrent writers, ledger failures and crash/recovery boundaries; this review's `face` row is the CLI's directional verb.

## Reproduction and limits

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-960-evidence/final-verifier/verify.py
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-960-evidence/final-verifier/alternate.py
```

Scripts create unique directories under `review_final/`. The alternate route CLI needs the repository-compatible Rust binary; the review copied the primary worktree's locally built compatible binary into its own untracked binary location. Exact binary identity is recorded. No whole-repository suite, mutation-battery pass, complete PCB DRC, connectivity certification, body/height certification, optimizer quality gain or Astra performance gain is inferred. Recoverable multi-file publication is not simultaneous visibility to arbitrary filesystem readers, tamper-proof authorship, automatic crash recovery or a power-loss durability guarantee.
