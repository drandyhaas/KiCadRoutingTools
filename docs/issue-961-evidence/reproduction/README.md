# Independent reproduction and acceptance review, issue 961

Reviewer 1 used isolated detached worktree `KRT-961-review-repro`, initially at
integration base `3f343981ae3fdbc82a60c0d41d693a03ccca98de`. The primary owns all
implementation. Conclusions below were derived from the complete issue and its
expanded comment, the live parent PR descriptions, repository instructions,
source inspection and separately executed native/CLI measurements.

At review time GitHub reported all three parents OPEN: #970
`bae72deee7ef90a5e27618c23424c27293c88b72` (which includes #968
`cbc819b6b862f6ec14ecb30b8659819e800fce76`), and #969
`acbcfa898f011f976632625194410d5fae8cccd5`. Their limitations remain relevant:
custom edge rules and some copper geometries are explicitly unmeasured; no-worse
placement does not mean engineering clean; provenance CLEAN concerns authorship
and accepted publication, not PCB quality.

## Commands and fixture controls

```powershell
git worktree add --detach ../KRT-961-review-repro 3f343981ae3fdbc82a60c0d41d693a03ccca98de
gh issue view 961 --repo drandyhaas/KiCadRoutingTools --json body
gh issue view 961 --repo drandyhaas/KiCadRoutingTools --comments
gh pr view 970 --repo drandyhaas/KiCadRoutingTools --json state,headRefName,headRefOid,headRepositoryOwner,body,comments
gh pr view 968 --repo drandyhaas/KiCadRoutingTools --json state,headRefOid,body,comments
gh pr view 969 --repo drandyhaas/KiCadRoutingTools --json state,headRefOid,body,comments
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-961-evidence/reproduction/reproduce.py --root . --out docs/issue-961-evidence/reproduction/baseline-preserved
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-961-evidence/reproduction/seating_control.py --root . --out docs/issue-961-evidence/reproduction/baseline-seating
```

`reproduce.py` records every actual subprocess argv, working directory, exit and
log in results JSON. It performs 45 direct real `floorplan.grade` calls matched
to 45 real `check_floorplan.py` calls and nine `check_drc.py` calls. The matrix
crosses the three required native-readback variants with both requested bands,
an additional declared 1.40..1.50-mm legal-body control, and copper/edge settings
(.25,0), (.25,.25), (.25,.55), (.55,.25), (.55,.55). The effective legacy gate
margin is read from each constructed `QuenchState`, not inferred from flags.

Every source sibling is copied through repository `copy_board`, then checked
against its source identity. On this fixture project, local-state, custom-rule
and design-brief siblings are absent. The generated intent files are explicit
test requirements, retained without post-hoc alterations. Native KiCad 10.0.0
reads the written board; F.Fab segment endpoints measure centerlines without
stroke thickness. Native pad bounding boxes independently verify the west gap
for this cardinal rectangle fixture. This is not an exact-shape oracle for
arbitrary pad shapes. The source SHA256 is
`165302e6a4f7aacdd64b3174df27ed8ddb19fd5f1f7effadd8d92205e25a120e`.

An initial harness attempt used native `SaveBoard`. It silently created a stock
project beside translated copies, causing DRC's requested .25 edge to resolve
to .50. Those DRC results were rejected as the required control. The corrected
reproducer uses the inherited text writer, checks sibling absence/identity,
and still uses independent native readback. The rejected local attempt is not
included as acceptance evidence. Initial script-key errors stopped before any
translated grading and were repaired; nonzero exits alone were not counted.

## Integrated baseline reproduction

The corrected 45 API/CLI pairs reproduced the historical defect. Outline
centerline bounds remain `(114,91,145.75,105.5)`. Native drawn F.Fab bounds are
`(114+dx,96.3,121.12+dx,103.7)`; the old graded pad/courtyard rectangle is
`(115.6+dx,95.8,121.62+dx,104.2)`.

| USB1 X translation | Physical body overhang | Pad west gap | Old clause at .25 margin | Old clause at .55 margin | Zero-margin pad-box control |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 | 0 | +1.60 | 0 | 0 | 0 |
| -1.45 | 1.45 | +0.15 | .10 | .40 | 0 |
| -2.10 | 2.10 | -.50 | .75 | 1.05 | .50 |

All dimensions are mm. At -1.45 the explicit 0.05 to 0.20 band passes at .25 and
fails at .55 despite identical body geometry. Its 0..65 band passes at both
margins. The source's positive minimum correctly rejects zero overhang, but
passing rows expose no overhang measurement, basis, units, source or disposition.
Requested edge zero with copper .25 still uses .25 gate margin. The arithmetic
control measures the old rectangle at explicit zero; it is not a body fix.

Actual requested .25-edge, zero-tolerance DRC on source/-1.45/-2.10 reports
0/2/2 pad-board-edge errors, respectively. Both moved copies introduce five
pad-pad violations. Edge shortfalls are .10 mm each at -1.45 and .75 mm each at
-2.10. At .55 edge the -2.10 copy has four pad-edge errors (.30 twice, 1.05 twice).
The source is DRC-clean under these particular checks; it is not a completed
route or a connectivity claim. Placement's older `oob_pad_count` is 0/1/1
**footprints**, independently of DRC's pad counts.

The DRC commands omit copper clearance, which resolves to its .20 fallback on
this projectless board (placement uses explicitly requested .25/.55). DRC's
requested .10 edge is silently fab-clamped to .20 in the engine, even though
`graded_at.board_edge_clearance` still says .10. Accordingly its .05 shortfalls
on the +.15 gap are not evidence that .10 clearance failed. The script records
the requested, resolved, fab and effective-check values separately. No rule was
altered to make this variant clean.

The +1-mm inboard control has native setback 1 mm and body overhang zero. On
the baseline a zero band without class fails because the pad rectangle selects
south rather than declared west; adding `class: edge_receptacle` switches to
body seating but introduces an undeclared .50-mm setback. A declared 1.10-mm
setback without class still fails on the wrong 1.30-mm courtyard distance.
These independent controls therefore test more than the displayed overhang key.

An additional -1.30-mm control has native body overhang 1.30 and copper gap .30.
The declared legal body band is 1.25 to 1.35 with zero maximum setback. Baseline
grading incorrectly refuses it at both .25/.55 edge settings. Independent actual
DRC reports no pad-edge violation at .25, and two .25-mm pad-edge shortfalls at
.55, while five collateral pad-pad findings remain on both. This gives a valid
passing/failing copper pair without changing manufacturing rules or mistaking
the sub-fab requested-.10 case for a passing control.

## Independently derived acceptance

1. The declared mating edge identifies the measurement boundary. Body overhang
   is a nonnegative physical distance independent of copper-margin knobs. An
   inboard body has zero overhang; signed position or separate setback must
   distinguish its actual distance to the mating boundary.
2. Zero minimum is nonbinding. Positive minimum rejects zero overhang. Explicit
   setback checks use their own physical measurement. Connector classification
   must not quietly turn a zero-overhang minimum into a seating requirement.
3. Passed and failed rows expose actual body overhang, setback and copper gap/
   shortfall, their geometry bases, millimetre units, relevant limits, requirement
   sources and dispositions. The overhang clause must grade that same number.
4. Real body and relevant-face geometry are needed; missing/ambiguous body or
   boundary must be explicitly unmeasured with a reason. Rotations, F.Fab,
   supported silk fallback, concavity and cutouts must be tested or disclosed.
   A sum of containment penalties is not a physical overhang measurement.
5. Legal mechanical overhang and illegal copper may coexist. Copper violation
   count means pads in DRC, whereas the old out-of-board diagnostic aggregates
   footprints; neither count should be relabelled as the other.
6. Emitter observations/default bands, exemptions, grading and downstream final
   gates must apply the same contract. Unsupported declared requirements cannot
   silently pass final certification. Exploratory no-worse candidates may retain
   useful measurements without certifying a dirty or unmeasured final board.
7. Requirements, mechanics, fixed footprint states, source baseline and siblings
   remain unchanged. Parent strict-search, routing-versus-placement defaults and
   accepted-candidate publication/provenance behavior remain in force.

## Candidate behavior verified independently

Behavior-tested commit: `bc8071b9ab586dd78629cdf511e4d527b4af33a1`.

```powershell
git checkout --detach bc8071b9ab586dd78629cdf511e4d527b4af33a1
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-961-evidence/reproduction/reproduce.py --root . --out docs/issue-961-evidence/reproduction/candidate-bc8071
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-961-evidence/reproduction/seating_control.py --root . --out docs/issue-961-evidence/reproduction/candidate-bc8071-seating
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-961-evidence/reproduction/compare.py --before docs/issue-961-evidence/reproduction/baseline-preserved/results.json --after docs/issue-961-evidence/reproduction/candidate-bc8071/results.json --seating docs/issue-961-evidence/reproduction/candidate-bc8071-seating/results.json --seating-before docs/issue-961-evidence/reproduction/baseline-seating/results.json --out docs/issue-961-evidence/reproduction/comparison-bc8071.json
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-961-evidence/reproduction/native_preservation.py --source kicad_files/esp_prog.kicad_pcb --results docs/issue-961-evidence/reproduction/candidate-bc8071/results.json --out docs/issue-961-evidence/reproduction/native-preservation-bc8071.json
```

The complete 45 API/CLI matrix, nine DRC commands, twelve supplementary grader
CLIs and two supplementary DRC commands were rerun in the detached candidate.
**734 independent comparison assertions passed.** Original and translated
fixture bytes match the baseline exactly; so do the explicit overhang intent
declarations and actual DRC item arrays. This comparison's oracle is the
independently read native geometry, not the implementation's new helper.
The separate UUID-keyed native preservation audit retains duplicate-reference
footprints and confirms all footprint positions (undoing only the commanded
USB1 X delta), rotations, faces, locks, pads, body graphics and board drawings
match the original. Its full native source snapshot is committed with the rows.

Physical overhang now reports 0/1.45/2.10 mm at every requested/effective margin;
its published `overhang_mm` equals the value controlling its clause. Basis is
F.Fab. The row reports signed position, setback, all-edge copper gap, declared-
edge copper gap, independent required copper gap and shortfall. Each of the four
measurement objects exposes units, geometry basis, declared limit, requirement
source and disposition. The CLI enriches the direct API's already-resolved
source with its actual requested flags and `source: cli`; these differences
are verified rather than mistaken for a measurement mismatch.

At -1.45 both requested original bands now fail mechanically at both .25/.55
margins, as their declared maxima require. An intentionally legal 1.40-to-1.50
band passes mechanically while insufficient copper clearance fails independently.
Requested edge zero still leaves occupancy margin .25 with copper .25, while
body margin is zero and the separate explicit edge requirement is zero. This
does not claim that final DRC, with its independent fab floor, would accept it.

At +1 mm, zero-minimum controls pass with or without connector class and
without a declared seat. Positive minimum refuses. Native setback remains
1 mm: an explicit .20 maximum refuses, while an explicit 1.10 maximum passes,
at both margins. At -1.30, the legal body band passes; the whole floorplan
passes at .25 edge and refuses for copper at .55. Actual DRC pad-edge findings
are zero and two respectively, with five unchanged collateral pad-pad errors.
Thus floorplan `pass` is its declared scope, not a clean-board certification.

No behavioral discrepancy remains in this review's exercised scope. The
implementation deliberately leaves nonrectangular boundaries, cutouts, curved/
open/concave body drawings and missing/ambiguous geometry unmeasured. The geometry
reviewer tests that coverage boundary independently. This review does not claim
full-board connectivity, model-performance gains or arbitrary connector mechanics.
## Final-candidate wrapper follow-up

At `52db3fb0fa6f3ba94f029c1f2b4de76f9c2d34c1`, only the seed/reconstruction
entrypoints and their new publication wrapper differ from `bc8071b9`. The
independent [identity record](identity-52db3.json) verifies that all geometry,
grading, placement engines, writer/publication machinery, routing/plugin/tools,
test and fixture blobs/trees exercised by the preceding matrix are identical.
The 734-control result therefore remains applicable to those unchanged paths.

```powershell
git checkout --detach 52db3fb0fa6f3ba94f029c1f2b4de76f9c2d34c1
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-961-evidence/reproduction/verify_wrapper.py --root . --out docs/issue-961-evidence/reproduction/wrapper-52db3
```

Eight additional actual CLI controls exercise the changed entrypoints. A copy
of the original public board is deliberately locked for these controls, keeping
the flush USB fixed while testing incompatible positive-minimum requirements.
The original source remains unchanged. Both seed repair and reconstruction's
classify stage accept the explicit zero-minimum/zero-setback control, publish
the written candidate and preserve every native footprint pose/face/lock.
Both refuse a .05 minimum overhang before replacing an existing destination.
All four dry runs preserve existing board families and report no published
output. Inputs and explicit intents retain their recorded identities.

The reviewer reported a scope-disclosure concern: these dry runs retain the
inner operation's `status:ok, complete:true` while omitting any explicit
connector-requirement evaluation status; seed's impossible dry repair also
reports `unrepairable:1` and exit 4. They do explicitly report `dry_run:true`,
`published:false`, and null output, so no rejected board is published. An
explicit not-evaluated connector status would prevent confusing operation
completion with final engineering certification.

The reporting concern was corrected in
`e182bbfe4c3ec19ad65999de9ff023dc84035128`. The reviewer checked the two-file
diff (wrapper reporting and reconstruction help), then reran **all eight actual
CLI controls**, including accepted and refused non-dry controls, with stronger
dry-run assertions. All pass: dry results now report `status:dry_run`,
`complete:false`, `engineering_clean:false`, and connector requirements with
null acceptance, incomplete status and a reason that no written final candidate
exists to measure. No destination mutation occurs. Seed's unrepairable dry
operation still exits 4; the corrected summary no longer implies certification.

```powershell
git checkout --detach e182bbfe4c3ec19ad65999de9ff023dc84035128
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-961-evidence/reproduction/verify_wrapper.py --root . --out docs/issue-961-evidence/reproduction/wrapper-e182bb
```

## Legacy regression review and final behavior reference

The reviewer inspected the complete test-only change from `e182bbfe` to
`2448498c762e0c5a5807ff02554cf4faa72a536d`. The migrated tests retain original
body-less fixtures as explicitly unmeasured controls, add supported drawn-body
controls where actual seating is the test subject, preserve frozen inherited
along-edge declarations, and stop treating class inference or a copper-margin
graze as a mechanical allowance. The original source boards remain unchanged.
The geometry reviewer separately verifies the frozen declarations and test 706.

```powershell
git checkout --detach 2448498c762e0c5a5807ff02554cf4faa72a536d
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-961-evidence/reproduction/run_review_tests.py --root . --out docs/issue-961-evidence/reproduction/tests-2448498
git checkout --detach 9e65e44b0cb1bc7ab2b8656a48f4848ddbc38100
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-961-evidence/reproduction/verify_help_and_availability.py --root . --out docs/issue-961-evidence/reproduction/help-portability-9e65
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-961-evidence/reproduction/verify_identity.py --root . --revision 9e65e44b0cb1bc7ab2b8656a48f4848ddbc38100 --test-revision 9e65e44b0cb1bc7ab2b8656a48f4848ddbc38100 --out docs/issue-961-evidence/reproduction/identity-9e65.json
```

All ten independent regression commands passed: new 961 geometry/publication,
549 grade/CLI, 712 centering, edge containment, and run23/26/27/run4 checks.
The run4 file explicitly skips nine historical untracked-board controls; its
remaining missing-body publication assertion passes. These skips are not passes.
Exact commands, exits and complete logs are retained under `tests-2448498`.

The reviewer caught the new publication test's unconditional native import.
Revision `9e65e44b0cb1bc7ab2b8656a48f4848ddbc38100` adds the repository's explicit
skip-77 guard and corrects stale CLI help. Independent verification runs the
actual test under native KiCad (pass) and system Python 3.13 with no `pcbnew`
(explicit skip 77, not pass). AST comparison proves the CLI source differs from
the behavior-tested grader only in `help` keyword strings; actual `--help`
contains the corrected policy, and a real grader command on the existing
fixture produces byte-equivalent parsed JSON evidence to the previous control.

[The final identity record](identity-9e65.json) pins 16 production/test/fixture
blob or tree identities to the independently exercised revisions. There is no
unresolved discrepancy in this review's scope. The evidence-only publication
commit still needs the final identity check after its SHA exists.
