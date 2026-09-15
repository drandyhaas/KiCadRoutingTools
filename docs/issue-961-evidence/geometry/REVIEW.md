# Independent geometry and requirements review for #961

Reviewer 2 worked in the isolated detached `KRT-961-review-geometry` worktree.
Integration baseline: `3f343981ae3fdbc82a60c0d41d693a03ccca98de`.
Behavior-tested revision: `e182bbfe4c3ec19ad65999de9ff023dc84035128`.
Final source/test review: `9e65e44b0cb1bc7ab2b8656a48f4848ddbc38100`.
The prior body/copper implementation was independently tested at
`bc8071b9ab586dd78629cdf511e4d527b4af33a1`; all affected controls were rerun here.
The reviewer did not edit production or repository tests. The full issue and
expanded comment, repository instructions and all three parent PR descriptions
were read. No PCB-quality or model-performance conclusion follows from this review.

## Independent findings

The integrated baseline still conflated occupancy/clearance with body overhang.
Native KiCad 10.0.0 created and reloaded adversarial boards independently of the
implementation's parser. The baseline reports are retained in
`geometry.baseline.json`:

- A closed Silk body was flush at X=0, but its outboard pad inflated the existing
  `drawn_local` body box to X=-1.25. Using that box as body geometry would be wrong.
- A 45-degree triangular drawing had every native point inboard (minimum X=2.5),
  while rotating its local bounding box produced minimum X=-0.328427.
- A square Fab drawing at 37 degrees extended 0.300901 mm west, on both a front
  footprint and its native flipped back counterpart. The old overhang rule read
  zero from the inboard pad geometry.
- Missing body geometry and an open single Fab line were treated as measured
  pad/box geometry. A cutout generated a summed violation amount, not a physical
  depth.

During independent consumer testing the reviewer found a further defect in the
first implementation: `edge_seat_ok` checked netted pad centres, omitted net-0
copper and could accept a 0.45-mm copper gap at required 0.55 mm. The grader
correctly reported a 0.10-mm shortfall on the same native board. The primary
agent corrected the predicate to use all candidate copper and inherited exact
edge grading. All affected consumer and rotated-candidate checks were rerun.

The reviewer also flagged a nonbinding auto-class `min:0` becoming a blocking
missing-edge requirement. It is now informational. Explicit edge-only missing
geometry cannot certify; along-only claims retain their `declared` evidence bit.

## Verified behavior

`geometry.json` records 18 native-written geometry families, each graded five
times: .25/.55 copper clearance with requested edge clearance zero, positive
overhang band 0.05..0.20, zero-minimum band 0..0.65, and explicit setback max .1 mm.
The physical body measurement is invariant under clearance changes and equals
the value controlling the overhang verdict. Requested parameters, effective
occupancy/body margins, all measurement fields and dispositions are recorded.
These are dimensional arithmetic controls, not claims of clean complete boards.

Supported native bodies include closed Fab and Silk drawings (`fp_line`,
`fp_rect` and `fp_poly` are exercised), a nonrectangular
convex triangular body, arbitrary 37/45-degree rotations and F/B faces. Missing,
open, concave, circular, disconnected and opposite-face-only bodies return a
specific unmeasured reason. Concave/slanted outlines, cutouts and extra open cuts
are explicitly unsupported for physical body measurement. The symmetric corner
case refuses inferred edge selection and reports both physical crossings under
an explicit west declaration. Positive-minimum controls reject fully inboard
bodies; zero minimum does not require flush seating.

`consumers.json` and `exemption_controls.json` verify body/exemption/emitter
agreement: a native 0.1-mm body overhang is measured as 0.1, exempted as 0.1,
and emitted with observed value 0.1 and an explicitly inferred max 0.6. Copper
independently passes at 1.65-mm gap and fails at .45/.15 mm with required .55,
for netted and unconnected pads. The body exemption does not remove those copper
failures. Inboard positive-minimum, outside-band and second-edge-crossing cases
receive no exemption. `corrected_rotation.json` verifies an actual written
37-degree candidate at X=2.7009010664: native body overhang is .1 mm.

`candidate_copper.json` records 216 actual written native candidate boards:
rectangular/oval/roundrect pads, F/B footprints, initial rotations 0/89.5/90.5/37,
and candidate rotations 0/.5/1.0001/37/89.5/90/90.5/179.5/359.5 degrees.
Pads have an offset and no net, deliberately covering the omitted-copper case.
Independent support calculations use the saved native pad's position, angle,
size and roundrect radius, not the candidate parser's normalization. Maximum
gap discrepancy is `0.0000007238261763` mm (below 1 nm); all complete/finding
verdicts agree with the independent requirement calculation.

`declarations.json` records 25 loader controls: NaN, positive/negative infinity,
negative values, booleans, numeric strings and reversed bands refuse with a
named reason. Valid zero and positive bands and zero setback retain their exact
declared values. `optional_claims.json` records nonbinding class, edge-only
missing body and along-only declarations separately.

`publication_controls.json` adds seven independent controls for the new final
candidate wrapper: accepted, overhang-band rejected, copper rejected,
missing-body rejected, dry-run, no-written-candidate and an accepted exploratory candidate whose
underlying exit is 4. These call the real `run_checked` and publication engine
with an adapter that copies a native fixture into the staging destination;
they are not additional seed/reconstruct CLI runs. Rejected/dry candidates
leave an existing byte-sentinel destination unchanged and never publish the staged
success summary. Accepted written boards are read natively and retain the
candidate pose. Both successful publications explicitly say
`engineering_clean:false`, and the exploratory one retains exit 4/status
`exploratory`. Intent SHA256 remains unchanged throughout. Reviewer 3 separately
owns the actual CLI/provenance/fault tests.
Dry and unwritten candidates explicitly report `accepted:null`,
`complete:false`, `engineering_clean:false` and `status:dry_run/refused`.

## Commands and identities

The complete commands, exit codes, code/test/fixture tree IDs and SHA256 of every
written native fixture/project are in `run_identity.json`. Exact invocation:

```powershell
git checkout --detach e182bbfe4c3ec19ad65999de9ff023dc84035128
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 run_review.py
```

All five independent scripts exit 0. From this committed evidence directory,
reproduce against an isolated code checkout by passing its absolute path:

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 run_review.py C:/path/to/isolated/checkout
```

Native-written boards and every generated sibling are preserved in
`native-fixtures.zip`; identity rows refer to the archive's filenames.
The archive also retains the intentionally non-PCB byte-sentinel destination
controls from wrapper refusal tests. Native KiCad may create stock synthetic
project/PRL siblings; they are retained and
hashed, and the dimensional probes pass explicit .25/.55 settings. Public
source fixtures were not modified. Native image-handler warnings are retained
in the stderr logs; all processes return zero.

## Scope and limitations

The reviewed body contract is a two-dimensional drawn-envelope support position
relative to a declared compass edge, not an independently identified physical
socket opening or 3-D connector seating model. Body primitives are closed convex
polygonal envelopes on the footprint's own Fab layer, or supported closed Silk
fallback. The own-face restriction and unsupported curves/concavity/cutouts
are explicit coverage limits; they do not silently pass required certification.
Valid along-edge ring measurements remain available independently of body
abstention. No universal centering, flush seating or no-overhang rule was added.

The review's actual-written-board evidence is synthetic native geometry, not a
new full-board DRC/route result. Reviewer 1 owns the public esp_prog replay and
Reviewer 3 owns actual final CLI/provenance/regression verification. The inherited
#968 custom-rule/complex-pad limitations, #969 routing-vs-placement defaults and
#970 transaction/provenance limitations remain their documented scopes. The
reviewer independently agreed that legacy tests asserting pad-box body semantics,
universal inferred exemptions or implicit class seating must be updated, while
preserving their raw optimizer and valid ring-span checks.

The reviewer independently inspected all nine legacy test updates, then returned
the owned worktree to the actual integration base and reran `emit_intent` on all
five boards in `961-inherited-edge-intents.json`. Every frozen ref/edge entry
matches the real inherited emitter exactly, including absent edge values.
`inherited_edges_verified.json` records the public board SHA256 and mappings.
This confirms the along-edge tests retain their original edge declarations.
The final code checkout was restored afterward. Reproduction command:

```powershell
git checkout --detach 3f343981ae3fdbc82a60c0d41d693a03ccca98de
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 reviewer_inherited_edges.py
git checkout --detach e182bbfe4c3ec19ad65999de9ff023dc84035128
```

At test revision `2448498c762e0c5a5807ff02554cf4faa72a536d`, the reviewer
independently ran `tests/test_706_seat_edge_target.py` to `ALL PASS` and the new
`tests/test_961_connector_publication.py` to success across all six actual
CLI subcases. The max-zero overhang control still checks both the body error
and lost exemption. The J17 rotation-guard counterfactual authors an in-memory
overhanging pose while preserving the original source board and golden data.

Final help/native-availability revision `9e65e44b...` changes no grading
behavior. `reviewer_test_identity.py` verifies identical placement/routing/public
fixture trees and identical AST for `check_floorplan.py` after removing only
`help` keyword text. The actual help CLI displays the updated measured-body
contract. The actual native six-CLI regression passes again, while a real
non-KiCad Python 3.13 invocation exits 77 with the explicit native-required skip
reason. `portability_and_help.json` and its stdout/stderr files record these
commands. `test_revision_identity.json` records final tree IDs and test results;
the 706 test file is unchanged from its passing revision. No test skip is
counted as a pass.

