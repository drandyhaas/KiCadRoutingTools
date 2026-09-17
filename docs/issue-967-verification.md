# Placement edge-floor enforcement: issue 967

This is the placement-legality slice of [#967](https://github.com/drandyhaas/KiCadRoutingTools/issues/967).
It does not close the issue's broader independent-DRC rule-contract work.
The 2026-09-14 follow-up recorded fresh independent reproduction, additional
coverage defects found and corrected, and the final verified revision; its raw
dumps are not carried in this repository -- they are on
[PR #968](https://github.com/drandyhaas/KiCadRoutingTools/pull/968). The earlier measurements below remain pinned to their
original revisions; they are not a substitute for that final verification.
The original failure was reproduced on current upstream
`5a7fbcb6ee4deebd1d9ec1d5bd094d8681f502f3`, not inferred from an older report.
The issue had no comments when read. Implementation and verification used
isolated worktrees; original fixtures, unrelated user changes and the original
checkout were preserved.

## Rule and geometry contract

`pose_ops` now forwards its resolved edge floor through input, candidate,
both snap rungs, and restored-candidate grading. The staged winner is the
board promoted by the existing atomic writer. Direct poses, rotations and
coordinated multi-part operations remain available.

The shared `grade_pad_legality(edge_margin=...)` interface now assigns that
argument to a separate **per-pad edge-clearance channel**. An omitted argument
resolves through `board_floor_knobs`: explicit placement argument, project
constraint, then the existing .55 mm fallback. The legacy whole-part
`oob_pad_count/amount` diagnostic retains its copper-clearance inset; enlarging
that AABB diagnostic to the edge floor could introduce geometric phantoms.
The physical `oob_pad_copper_*` census remains at margin zero. Pad-pair local,
netclass and custom-layer clearance requirements remain separate and unchanged.

The edge report carries required mm, actual minimum gap, individual shortfalls,
pad index/location (including duplicate pad numbers), numerical tolerance,
coverage and unmeasured geometry. Pose summaries distinguish original requested
values, resolved values and their sources. A lower-level grader receiving an
already resolved argument calls its source `caller argument`, not `cli`.

Rectangular Edge.Cuts with ordinary rect/circle/oval/roundrect pads use analytic copper
extrema, including arbitrary rotation and rounded corners. The absolute
tolerance is **1e-6 mm (1 nm)**; equality passes. A bbox without evidence of a
closed rectangle cannot certify coverage. Nonrectangular outlines retain the
existing DRC sampler's findings and are explicitly partially measured. Custom
pad polygons also retain findings but remain partially measured: the parser can
tessellate curves into inscribed polygons. For example, a radius .5 circle at
half a 32-point sampling step can understate copper reach by .0024076 mm.
Unsupported pad geometry is explicitly unmeasured. Chamfered variants and
per-layer padstacks are also unmeasured: their simplified parser shape is not
the native copper. All source edge segments must cover a closed rectangle,
including when the parsed rings omitted an open internal edge. A live-board
caller must supply its current saved board for this source check.

Custom `edge_clearance` declarations are recorded in `pad_edge.rules_unmeasured`
with the rule name, source and declared values. This scalar edge check does not
evaluate their scope, precedence or effective per-pad requirements. Their
presence prevents complete coverage and `legal:true`; a no-worse operation may
still proceed. Custom copper-clearance rules stay in their separate channel.

`legal` requires clean measured pad/hole/outline channels and complete edge
coverage. `no_worse` remains relative to the unchanged input: inherited defects
can remain or improve without force; a newly increased edge count, shortfall,
or unmeasured count refuses. `--strict-legal` also refuses incomplete coverage.
Neither verdict measures bodies, height, routed connectivity or filled copper.

Call-site audit:

| Consumer | Change or existing enforcement |
|---|---|
| `place_pose` input/direct/multi/in-place/dry-run/snap/near | Same resolved floor in every actual grade; count and magnitude refusal; complete coverage required for `legal` |
| `place_seed` repair and final input/output grades | Forward resolved floor; preserve resulting edge report |
| `place_optimize` input/output | Forward resolved floor; report edge findings, magnitudes and coverage; existing warning policy retained |
| `place_reconstruct` final | Forward resolved floor; preserve edge fields in final report |
| Seeder and floorplan repair census | Carry the engine's edge margin into the shared grader |
| Lock advisor, diagnosis, assembly echo | Resolve from the board path; assembly retains the separate edge report |
| Quench/pose ranker/cap candidate gates | Already use a conservative `max(copper, edge)` inset; unchanged. Snap's lattice still permits a direct no-worse alternative to the ranker's conservative proposals |

Seed/optimize/reconstruct operation completion and assembly's mechanical
`buildable` verdict remain their existing scoped results. An exit 0 or
`complete:true` from those tools is not a statement that their edge report is
clean. In particular, an all-locked inherited defect remains reportable without
turning a no-op into a forbidden placement operation.

## Frozen identities and independent reproduction

The source is the [published immutable evidence bundle](https://github.com/edgehero/KiCadRoutingTools/tree/e65e33da58ecbefb346534f2cad36b1803bfb55b/wk/astra-evidence),
derived from repository `kicad_files/esp_prog.kicad_pcb`.
All existing siblings were carried with repository `copy_board`; no project,
custom-rule or local-state sibling existed on these inputs.

| Identity | SHA256 |
|---|---|
| Shared pilot input / baseline | `662b58af79fbd02a05cd5806f2d502c0081de8e32496611742dd5060a0cd41a2` |
| Frozen `current_skill/final.kicad_pcb` | `509e1f95e7c492410eb913d6ceb4e3458d52a74b135392ae37b4c2ecea3a8325` |
| Published `concise_contract/final.kicad_pcb` control | `ae4194a6879bd986d1c769e0c8f35ae0da8047efe7877b72256a7405f7260e7c` |
| Shared unchanged design brief | `72c0ca2c020cc6a73e7eeb169c433ac7d51bff0818d74c18fdf9397ce54bbf67` |

Both independent verifiers reproduced the original failure using actual CLIs
and native KiCad 10.0.0 / Python 3.11.5. The reproduction verifier replayed the
four exact commands in the published `RETURN.md`. Originally all four exited
0, and the final reported `legal:true` at displayed .55 edge / .25 copper.
Independent DRC at explicit .25 copper / .55 edge / margin 0 / pad-edge enabled
found two .05 mm edge shortfalls. Native rectangular-pad bounds put Y1.2 and
the high-Y Y1.3 at Y105.0, against Edge.Cuts **centreline** Y105.5: gap .50 mm.
The stroke-inclusive native board bbox reaches Y105.525 and is not the boundary.

The .55 audit floor was the actor's displayed fallback, not a fabrication
specification preregistered in the pilot brief. No waiver or changed requirement
was introduced by this fix.

## Before/after results

| Check at .25 copper / .55 edge | Original | Corrected |
|---|---|---|
| Frozen .50-gap no-op | `legal:true`, no edge findings | Accepted `no_worse:true`, `legal:false`, two .05 shortfalls, physical off-outline zero |
| Published control no-op | Accepted, DRC 0 | Accepted and clean, identical explicit-floor DRC 0 |
| Original calls 1–3 | Accepted improving poses | Accepted improving poses; residual defects remain disclosed |
| Original in-place coordinated call 4 | Writes new edge defects | Refuses 4: edge count 0→2, shortfall 0→.10; input and siblings unchanged |
| Call 4 with only requested Y1 Y changed to 103.15 | Valid geometry | Accepted; native gap .55; actual output DRC 0 |
| .50→.52 inherited gap | Accepted | Accepted no-worse, still `legal:false` |
| .55 / .60 gap boundaries | Accepted | Accepted clean; .50 gap remains unclean |
| Worsening at unchanged finding count | Missed edge change | Refuses for increased edge shortfall |
| `--near`, 90° boundary, multi-part repair | Supported | Supported; native recheck of written output |
| Fallback/project/explicit override | Displayed floor omitted from actor grade | Reported resolved floor reaches the actual edge grade |

The reproduction verifier's first integrated replay at `1f84d360` preserved all
17 fixed serialized footprint blocks and locks, all non-footprint board text,
baseline and brief. Call 4's refusal retained board hash
`1155d9508d83ee9f1d3f8d3de14d5c1ea5fa003e2f0d7ba32fe6fa4c0cdaf834`.
The accepted boundary alternative hash was
`f4171fab820983eafc2f880ce0b903ba0f8806460b3403aa41ba0cee597c7d58`.
Hashes establish identity/preservation; native geometry and checker results
establish correctness.

The independent final verifier checked the committed implementation, actual
output boards and alternate CLI paths, and identified the custom-circle
coverage overclaim before finalization. That finding was fixed and covered by
a regression in `39d91f27`. The PR's validation record pins the final revision
and contains the final command/native evidence. The command and geometry record --
kept on [PR #968](https://github.com/drandyhaas/KiCadRoutingTools/pull/968)
rather than in this repository -- includes production-tree identities, 21 core
CLI commands, eight alternate commands and four DRC checks on alternate
outputs. Eight written actor outputs preserve all 17 fixed blocks,
board graphics and brief; the separate project scenario preserves its project.
The final verifier found no remaining discrepancy within the exercised scope.

## Reproduce and regress

In an isolated copy of the public evidence checkout, preserve its siblings and
run these from the implementation checkout (substitute absolute fixture paths):

```powershell
$py = 'C:/Program Files/KiCad/10.0/bin/python.exe'
& $py -X utf8 py_router/copy_board.py PUBLIC/current_skill/final.kicad_pcb SCRATCH/frozen.kicad_pcb
& $py -X utf8 py_placer/place_pose.py SCRATCH/frozen.kicad_pcb SCRATCH/noop.kicad_pcb set Y1 124.7 103.2 --rot 270 --clearance .25 --board-edge-clearance .55
& $py -X utf8 py_router/check_drc.py SCRATCH/noop.kicad_pcb --clearance .25 --board-edge-clearance .55 --clearance-margin 0 --check-pad-edge --json SCRATCH/drc.json
& $py -X utf8 py_placer/place_pose.py SCRATCH/frozen.kicad_pcb SCRATCH/boundary.kicad_pcb set Y1 124.7 103.15 --rot 270 --clearance .25 --board-edge-clearance .55
& $py -X utf8 py_router/check_drc.py SCRATCH/boundary.kicad_pcb --clearance .25 --board-edge-clearance .55 --clearance-margin 0 --check-pad-edge
```

The first actor command exits 0 but reports unclean; its DRC exits 1 with two
edge findings. The boundary actor and DRC exit 0. Repeat the control with its
published Y1 `(123,93.4,0)` pose. Replay all four original commands from the
published `RETURN.md` to exercise the new-violation in-place refusal.

`tests/test_967_edge_floor.py` reconstructs both published pose geometries on
copies of the existing repository board and checks actual CLI outputs,
boundaries, arbitrary-angle support, tolerance, custom approximation coverage,
project/fallback/override resolution, local copper-rule independence, atomic
refusal, snap, dry-run and sibling preservation. Existing tests cover direct
multi-part writes and the other established pose operations.

Focused validation commands:

```powershell
& $py -X utf8 tests/run_all.py 967 937_off_outline 628_milled 697_placement 761_legality 900_class 411_placement_siblings run27_seed_gate --jobs 3
& $py -X utf8 tests/run_all.py 892 placement_pad_legality --jobs 3
& $py -X utf8 tests/gui_parity/test_manifest_plan_parity.py
& $py -X utf8 tests/gui_parity/test_cli_postpass_coverage.py
```

The nine-file focused group passed; the existing pose suite reported 133
passing assertions, and the registry/pad-legality regressions and both parity
gates passed. Static test hygiene also requires a Python that parses the repo's
existing Python-3.12-style f-strings: KiCad's 3.11 stops on two untouched test
files. It was additionally checked using installed Python 3.13. Stale mutation
anchors caused by this change were repaired without changing their intended
mutations; both affected rows were killed in another isolated worktree, with
clean unmutated baselines. Python 3.13 static hygiene passed all nine checks,
including 1108 mutation anchors across 50 batteries. The full repository suite
and full native KiCad DRC were not run.

`tests/run_doc_examples.py` passed 32 runnable examples (78 signature/fragments
skipped) in an isolated Python 3.13 environment with NumPy. KiCad's embedded
Python ignores the runner's `PYTHONPATH`, and the bare system interpreter lacked
NumPy; those initial environment failures were resolved for this check without
changing the repository or global interpreter installations.

## Remaining scope and limits

- Native exact clearance for general curved/nonrectangular outlines and custom
  primitives is not established here; their incomplete coverage is explicit.
- Independent `check_drc` keeps its routing-oriented defaults, project minimum
  clamping, severity handling and fabrication floor. For example, placement's
  supported explicit .50 override on a project declaring .60 differs from the
  independent DRC policy. No-project edge omission also retains its old DRC
  behavior. Compare actual effective rules, not successful exits. The .25/.55
  evidence uses identical explicit parameters where these policies agree.
- No body/height certification, routing-quality improvement, connectivity
  completion, or instruction-variant advantage is claimed. The accepted pilot
  output has no tracks/zones. No skill or workflow instructions were changed.
- Broader declaration/consumer gaps in #964, placement strategy research in
  #965, connector/body geometry in #961, and copper-rule resolution in #966
  remain separate work. This PR should not close those issues or #967's
  remaining rule-contract/geometry acceptance criteria.
