# Issue #961: drawn connector geometry and truthful final publication

This new branch implements the expanded issue comment against the integrated
published parent revision. It does not assess Astra/model performance or claim
that these controlled boards are improved PCB designs.

## Integration and ownership

| Input | Published revision fetched and rechecked |
|---|---|
| upstream main | `5a7fbcb6ee4deebd1d9ec1d5bd094d8681f502f3` |
| #970, edgehero:fix/960-pose-provenance | `bae72deee7ef90a5e27618c23424c27293c88b72` |
| #968, edgehero:fix/967-resolved-edge-clearance | `cbc819b6b862f6ec14ecb30b8659819e800fce76` |
| #969, edgehero:fix/966-unset-clearance | `acbcfa898f011f976632625194410d5fae8cccd5` |
| **INTEGRATION_BASE_SHA** | **`3f343981ae3fdbc82a60c0d41d693a03ccca98de`** |

All three parents remained OPEN at the publication check. #970 includes #968.
The fresh `fix/961-connector-overhang` branch in sibling worktree `KRT-961`
starts at fetched #970 and merges fetched #969 in the separate integration
commit above, without conflicts. All four input SHAs are ancestors of that
integration base. Parent branches and original worktrees were not repurposed.
The upstream-main-based PR necessarily includes inherited commits; its body
links the exact integration-base-to-final-head comparison isolating #961.

[GitHub snapshots](github/) contain the issue with every comment, all three PRs,
review comments, current ownership/head/state checks and exact repeated fetch,
ancestry and public-fixture identity commands in
[integration-verification.json](github/integration-verification.json).
Evidence files disable Git text normalization so recorded artifact SHA256 values
survive checkout. No original public fixture changed. The original esp_prog SHA256 is
`165302e6a4f7aacdd64b3174df27ed8ddb19fd5f1f7effadd8d92205e25a120e`.

## Contract and observed correction

The [geometry contract](CONTRACT.md) defines the drawn body proxy, declared
compass mating boundary, signed position, nonnegative overhang/setback, geometry
coverage and explicit abstention. Copper and occupancy retain their separate
currencies. Required unsupported body/boundary geometry cannot silently pass.

Native outline bounds are `(114,91,145.75,105.5)` mm. On copies of the same
public USB1 geometry:

| USB1 X translation, mm | Native drawn west overhang, mm | Native pad-box west gap, mm | Inherited clause amount at .25 / .55 effective occupancy margin, mm | New clause amount, every margin, mm |
|---:|---:|---:|---:|---:|
| 0 | 0 | +1.60 | 0 / 0 | 0 |
| -1.45 | 1.45 | +0.15 | .10 / .40 | 1.45 |
| -2.10 | 2.10 | -.50 | .75 / 1.05 | 2.10 |

The inherited -1.45 board passes the positive `0.05?0.20 mm` overhang band at .25
but fails at .55 without any geometry change. The corrected clause fails both
settings on its actual 1.45 mm body overhang. The unchanged flush body fails a
positive minimum and passes a zero minimum. A fully inboard translated control
has zero overhang and a separately measured positive setback; only an explicit
setback limit requires seating. A legal -.10 body control passes `0.05?0.20 mm`.

A separate -1.30 control has body overhang 1.30 within its declared `1.25..1.35`
band and copper gap .30: copper passes .25 and fails .55. This proves allowed
body overhang does not waive copper clearance. The -.10 accepted final pose
is `[117.4,100,180]`; the inherited strict pose publication and separate final
mechanical grader agree with its actual written geometry.

The occupancy margin remains `max(clearance, board_edge_clearance)`: requesting
edge zero with copper .25 does not create a zero-margin occupancy test. The
low-level explicitly zero-margin gate remains an arithmetic control only.
Actual DRC with `--check-pad-edge --board-edge-clearance 0.25
--clearance-margin 0` resolves omitted copper clearance to .20. For -1.45,
it reports two USB1 pad-edge violations of .100 mm each; the footprint aggregate
counts one offending footprint. These are different counting units. The moved
controls also have collateral pad-pad violations (five for -1.45 and -2.10),
fully recorded; they are not otherwise-clean boards. Requested .10 edge can
resolve to the fabrication floor .20, so it is not claimed as a .10 copper pass.

## Independent reviewers and artifacts

Exactly three reviewers used separate owned worktrees. Each read the issue,
expanded comment and parents, established independent results and checked the
publication revision's source/test/fixture identity. Their scripts, commands,
logs, measured values, original declarations, native boards, hashes, ledger
rows and audit outputs are retained in the linked bundles.

1. [Reproduction and acceptance](reproduction/README.md): 45 direct-grade/CLI
   pairs, 12 supplemental grader CLIs, 11 DRC commands and 734 comparison/native
   assertions. Native identity preserves 21 footprints, 77 pads and four outline
   drawings, keyed by UUID where references repeat. The initial native-save
   harness accidentally introduced project defaults; that rejected harness is
   explicitly excluded, and all accepted baseline comparisons preserve sibling
   absence and use the real text writer. Eight additional wrapper CLIs cover
   accepted, refused and dry candidates. This reviewer found the dry summary
   certification ambiguity; final dry results are explicitly unevaluated.
2. [Geometry and requirements](geometry/REVIEW.md): 18 native geometry families
   times five margin combinations, 216 native-written candidate copper rotation
   cases, 25 numeric declaration controls, exemptions/emission/consumer controls
   and seven publication-adapter cases. Independent calculations catch the
   rotated-triangle bounding-box false overhang, both faces and Silk fallback.
   The reviewer reproduced the integration-base emitter on all five boards to
   verify the frozen along-edge declarations used by migrated legacy tests.
3. [Final placement and parent regressions](final/REPORT.md): actual grader/DRC,
   written boards, final pose/lock/face/SHA ledger reconciliation and audits: 96 actual grader CLIs, six DRC commands, 1,543 assertions;
   12 parent commands, including 124 native publication and 224 reader assertions;
   seed/reconstruct accepted/refused/dry/in-place/read-only publication; destination
   authorization and partial/committed recovery; improving-pile exploration;
   strict search and separate routing/placement clearance behavior. This reviewer
   found that seed/reconstruct could publish unmeasured connector candidates.
   The implementation now grades the staged final board before its single
   inherited transactional publication. Refused candidates preserve existing
   destinations and produce no success ledger. Accepted nonconnector-dirty
   exploration remains explicitly exploratory, with `engineering_clean: false`.

## Verification revisions and commands

- Integration-only baseline: `3f343981ae3fdbc82a60c0d41d693a03ccca98de`.
- Initial body implementation tested independently: `bc8071b9ab586dd78629cdf511e4d527b4af33a1`.
- Publication gate: `52db3fb0fa6f3ba94f029c1f2b4de76f9c2d34c1`.
- Final gate/dry behavior: `e182bbfe4c3ec19ad65999de9ff023dc84035128`.
- Migrated test controls: `2448498c762e0c5a5807ff02554cf4faa72a536d`.
- Final production/test revision: `9e65e44b0cb1bc7ab2b8656a48f4848ddbc38100`.
  This last change updates help, documents the contract and adds native-test
  skip handling; grading/publication behavior is identical to e182bbfe.
- Subsequent evidence-only commits preserve the trees recorded in
  [tree-identity.json](verification/tree-identity.json). Final reviewer identity
  confirmations are linked from the publication PR, avoiding a self-referential
  commit hash inside its own evidence.

[Primary focused check logs](verification/) include exact invocation metadata in
`2448498c-results.json`. Native commands use
`C:/Program Files/KiCad/10.0/bin/python.exe -X utf8 tests/<script>.py`.
They cover 549 schema/grade/CLI, 706 seating, 712 along-edge measurements, 961
geometry/publication, edge containment, run22/23/26/27 consumers and run4 repair;
independent reviewer logs cover overlapping cases and inherited regressions.
The new publication regression executes six actual CLIs and reads the output
with pcbnew. Without native KiCad it explicitly exits 77; native execution was
performed here.

Both `tests/gui_parity/test_manifest_plan_parity.py` and
`tests/gui_parity/test_cli_postpass_coverage.py` pass. `tests/mutation_anchors.py`
reports 50 batteries, 1108 anchors, no problems and zero unresolved anchors;
this checks anchor integrity, not execution of every mutation. Static test
hygiene passes with system Python 3.13 (the KiCad Python 3.11 parser cannot parse
two existing newer-syntax test files). Documentation examples pass in an isolated
Python 3.13 environment with NumPy/SciPy/Shapely: 32 executed, 78 signature or
fragment blocks skipped. Early runs with embedded Python ignored PYTHONPATH;
these dependency/import failures were resolved by using that isolated environment.

Nine run4 controls and five run5 emitter controls require historical untracked
`wk/` boards absent here and are explicitly skipped. No full repository suite,
full native PCB DRC, routing connectivity, arbitrary custom copper shape/rule
coverage or model-quality evaluation is claimed. Parent evidence documents its
remaining custom-rule, pad-approximation, transaction/concurrency and coverage
limits. Provenance CLEAN is not engineering-clean placement.
