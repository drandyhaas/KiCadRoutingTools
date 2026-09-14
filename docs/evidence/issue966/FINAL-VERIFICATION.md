# Independent final verification: issue 966

Verified integrated production revision: `e5681428d43a17f663df257b9a94c55c544e786c`.
Independent detached checkout based on main
`5a7fbcb6ee4deebd1d9ec1d5bd094d8681f502f3`; native KiCad 10.0.0 on Windows.
The integrated fix changes `swig_gui.py` and the GUI cap caller in
`fanout_gui.py`: the latter preserves the placement engine's independent omitted
clearance contract. No placement strategy, engine, manufacturing capability,
writer, baseline, or fixture changes are present.

## Acceptance and independent results

- Unchecked declared-zero clearance resolves to the routing CLI's physical
  fabrication floor: 0.10 mm on two layers, 0.09 mm on four layers. A typed but
  unchecked value cannot change it. This implements option 1 in issue 966.
- Native dialog matrices on `flat_hierarchy` (2 layers) and `glasgow_revC`
  (4 layers) each passed 112 assertions on the candidate. They cover zero,
  1 nm below/at/above the floor, positive 0.2/0.4 classes, stale 0.25/0.73 controls,
  explicit overrides, ceilings, all tier choices, custom physical capability
  0.083 mm, and an explicit-plan/reset/omitted-plan sequence. The separate
  hole-spacing zero fallback remains 0.2 mm under a 0.10 capability override.
- On original main, the corrected initial 94-case two-layer matrix had 8
  specifically identified failures: six declared-zero unchecked rows, custom
  capability zero, and the reset/omitted-plan row. The 86 controls passed.
  This is behavior evidence, not a nonzero exit treated as a successful guard.
- Independently reran `tests/gui_parity/test_966_unset_clearance.py` on the final
  candidate: the actual route/differential/planes/shared-fanout configuration
  assertions passed. QFN's dedicated control is a separate interface.
- After the cap-callsite correction, independently reran the actual
  `test_768_cap_ceiling_real_dialog`, `test_772_cap_params_reach_engine`, and
  `test_782_fanout_netclass_clamp` gates on the integrated production revision:
  all assertions passed. Both 112-case matrices and the four actual routing
  front/fixture combinations were also rerun on that revision.

## Actual CLI and GUI output boards

`verify_output_boards.py` stages copies of the public `flat_hierarchy` board
and all supported siblings. Positive input retains the original project
(Default 0.2 mm, Wide 0.4 mm). The zero case changes only Default clearance to
0 and `rules.min_clearance` to 0.0889 mm, matching the issue's low-rule case.
No input geometry changes. Both run `Net-(D1-A)`, F.Cu/B.Cu, max-ripup 0, with
clearance omitted; default fab auto/escalation fab apply.

The script invokes actual `py_router/route.py` and actual
`py_router/run_plan.py` in subprocesses. The latter executes the real native
dialog and plan executor and writes an output board. Both final zero arms
record `min_clearance_used: 0.1`, and both positive arms record 0.2. These are
engine ledger values, **not a measured minimum geometric distance**.

Independent native inspection found identical copper in each matched pair:
two 0.2-mm F.Cu segments `(78.4,75.2) -> (81.9,71.7) -> (81.9,63.0)`.
Native KiCad refill DRC reports zero target-net unconnected items, and
`check_connected.py --nets 'Net-(D1-A)'` reports all connected in each output.
This board does not establish improved routing completion: both before and
after may route an easy net despite different configured clearances.

Footprint UUID/reference/pose/rotation/layer/lock state, pad positions/shapes/
sizes/drills/nets, enabled layers, and outline UUID/geometry are unchanged.
The original Default declarations remain zero or 0.2 respectively, and the
entire Wide 0.4 class remains unchanged. No design brief or custom rules sibling
is supplied by these public fixtures; their absence is not a graded requirement.

All four outputs retain the input's real Q3 pad clearance violation: Wide
requires 0.4 mm; KiCad measures 0.2807 mm between Q3.1 and Q3.2. Item UUIDs match
the baseline. There is no new copper DRC error. Each output also gains one
silkscreen overlap warning from routing's existing default copper-text move.
There are 86 unrelated native unconnected items after this deliberately scoped
one-net route. **These boards are not clean.**

Saved project content is not universally identical between the fronts. CLI
writeback changes the hole-clearance rule from 0.25 to 0.1/0.2 and records
via-drill/fab-origin data; the GUI's saved project retains the input data even
though its live-floor log reports a change. The patch changes no writer code;
this is a limitation of asserting full-front equivalence and not evidence of
full project parity. In particular zero is not stamped with a routed value.

## Existing full-chain control, independently inspected

The primary agent generated splitflap_driver's existing three-step matched
chain; I independently opened its actual `cli_final` and `gui_replay` outputs
with their projects. `verify_full_chain.py` established exact native-integer
copper equality: 1,078 segments and 128 vias (including layer span, diameter
and drill), plus preserved footprint/pad and outline geometry from the staged
input. Native refill DRC found 201 unconnected input items and zero in each
output. All three files have the same five mounting-pad edge errors: declared
0.5 mm, actual 0.452 mm, with identical item UUIDs. They also share 72 library
and silkscreen findings. These native results qualify the primary agent's
zero-violation custom check_drc result, which excluded pad-edge checks.
That chain was generated at `a29856f`; the subsequent integrated correction
changes only the cap-optimization caller and its regression, neither of which
the route/plane/route chain invokes. Its actual routing production tree is
unchanged. The affected cap paths were verified afresh on `e5681428`.

The existing full-chain harness printed parity after both graders returned
`-1`; that was not accepted as a grade. Direct custom checks and the independent
native checks above provide the evidence instead. The board-only `gui_final`
copy is not used; `gui_replay` retains its project sibling.

## Evaluator discrepancies resolved

Independent final review discovered a real regression in the first candidate
`a29856f`: shared routing clearance also reached `_optimize_decoupling_caps` as
an explicit placement override. Native execution of the actual cap engine on a
zero-class copy printed 0.1 mm where original main printed 0.25 mm and the
placement CLI's omitted resolver returns 0.25 mm. The fixture has no BGA/vias,
so the actual method returned zero moves; this proves pricing, not cap quality.
The integrated caller fix preserves omission and lets the cap engine resolve
its own value. Native re-verification gives routing 0.1 versus placement 0.25,
with positive 0.2, explicit 0.3, explicit subfab 0.05 (existing effective 0.1),
and ceiling 0.3 on zero (existing effective 0.1) controls retained. The same
method serves inline BGA optimization and the standalone Optimize Caps action.
The expanded existing real-dialog cap gate verifies both producers.

The first independent matrix incorrectly expected the standard tier's 0.127-mm
nominal floor for initial pinning. Reading the declared API contract established
that **physical** advanced capability floors initial parameters regardless of
tier; tiers bound automatic descents. Correcting that expectation yielded the
specific 8 baseline failures above and 224 final passes, without a production
change. An initial native inspector also compared a SWIG object representation
instead of enabled-layer IDs; it was corrected before accepting any mechanics
result. A via-width call required an explicit native layer argument; the final
full-chain inspector uses it. No implementation discrepancy remains unresolved.

## Reproduction commands

Run scripts with KiCad Python, replacing `REPO` and choosing fresh output dirs:

```text
python verify_reader_matrix.py REPO OUT_MATRIX_2
python verify_reader_matrix.py REPO OUT_MATRIX_4 glasgow_revC
python verify_output_boards.py REPO OUT_BOARDS
python inspect_outputs.py REPO OUT_BOARDS
python verify_cap_boundary.py REPO OUT_CAP
```

`commands.json`, matrix JSON and `inspection.json` pin commands, fixtures,
projects, revisions, actual native copper and DRC counts. For the existing
full-chain outputs, run native KiCad DRC for input/CLI/GUI as shown in the main
README, then `verify_full_chain.py CHAIN_DIR NATIVE_JSON_DIR`.

Full historical DDR3 bench, every alternate routing engine, complete suite,
other KiCad versions, and placement/model quality are not measured here. No
claim that all boards are clean, all projects are identical, or the historical
DDR3 routing failure is solved is justified by this bounded reader verification.
