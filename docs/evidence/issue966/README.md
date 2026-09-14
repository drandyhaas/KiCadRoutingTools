# Issue 966: unset routing clearance

The issue's first proposed reader behavior is implemented: an unchecked GUI
clearance override retains a declared Default-class zero as the routing base,
then applies the same physical fabrication floor as the routing CLIs. On a
two-layer board with no fabrication overrides, that resolves to **0.10 mm**.
An unavailable class value uses `routing_defaults.CLEARANCE` (0.25 mm), independently
of any stale unchecked control value. Explicit overrides and class ceilings keep
their existing semantics.

The shared fanout-to-cap-repair call now preserves an omitted placement override
as `None`, so the placement engine continues resolving its own requirement
(0.25 mm for Default zero). Checked placement overrides retain their existing
values. This necessary boundary fix prevents the routing change from lowering
the capacitor placement requirement. It changes no placement engine or strategy,
writeback policy, board declaration, or fabrication capability table. The
issue's separately described writer fix was not present on the tested main:
an existing Default-class zero can remain zero after routing. The corrected
reader handles that value on subsequent runs too.

## Revision and environment

- Baseline: `5a7fbcb6ee4deebd1d9ec1d5bd094d8681f502f3` (upstream main).
- Final production and regression candidate: `e5681428d43a17f663df257b9a94c55c544e786c`.
- Initial candidate `a29856f0223be2e9173857ce30eaabf24ddcc80e` had the cap-repair
  boundary regression described below; it was corrected before publication.
- Production file Git blob: `bbca6792edc5decb41eb1af0fd23482e76e8a5f9`
  (`kicad_routing_plugin/swig_gui.py`); fanout boundary blob
  `9ba64c64ad9ad33736b1b6f21534e584c72b3953`
  (`kicad_routing_plugin/fanout_gui.py`). Later evidence-only commits must preserve both.
- Windows; KiCad 10.0.0; KiCad Python 3.11.5; wxPython 4.2.2.
- Existing router binary 0.22.0, SHA256
  `92624ea6c8e7581cc96f88dd70a67a0dd777c8f8015fc0fe87e751564407c541`.
- Original fixtures were preserved. Tests use separate worktrees and copies;
  project siblings travel with each board. Hashes identify inputs and frozen
  artifacts, never routing determinism (routing UUIDs are random).

## Parameter regression

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 tests/gui_parity/test_966_unset_clearance.py
```

The real native dialog and plan parameter application exercise zero, positive,
below-floor, exact-floor and 1-nm-above-floor values, all three tiers, custom
fabrication capability, explicit overrides, ceilings, reset, and stale unchecked
controls. Assertions inspect the actual route, differential, plane and shared
fanout configuration callbacks. These are parameter-delivery checks, not proof
that every engine has routed a board. The QFN-specific clearance control is a
separate interface and is not covered by the shared-fanout assertion.

The missing-class fallback is explicitly an injected unavailable read: native
KiCad normally synthesizes a Default class for a projectless live board. This
does not establish universal CLI/GUI parity on projectless boards.

The final verifier found that the initial routing-only patch also reached cap
placement through `_optimize_decoupling_caps`: its real engine printed
`cap pair clearance: 0.1mm (cli)` on a zero-class board, whereas the placement
CLI's omitted value resolves to 0.25 mm. The final call-site fix passes omission
through. The extended `test_768_cap_ceiling_real_dialog.py` drives both inline
BGA and standalone cap producers, then evaluates the delivered value with the
actual placement resolver. It verifies cap 0.25 versus routing 0.10 with the
override unchecked, and preserves checked 0.3 and sub-fab override behavior.
An independent actual-engine cap invocation verifies the same values; this
fixture has no BGA caps to move, so that check establishes pricing, not improved
placement or moving-cap geometry.

## Independent native verification

The reproduction verifier independently ran all four baseline and all four final
CLI/GUI cases against identical copied inputs:

| Copied Default class | Baseline CLI | Baseline GUI | Final CLI | Final GUI |
| --- | ---: | ---: | ---: | ---: |
| 0.0 mm | 0.10 | 0.25 | 0.10 | 0.10 |
| 0.2 mm (positive control) | 0.20 | 0.20 | 0.20 | 0.20 |

These are router-reported rule values in mm, not measured nearest-copper gaps.
All eight runs wrote boards and independently passed target-net connectivity,
native copper equality, 64-footprint/247-pad/five-outline-item preservation and
whole-`net_settings` preservation assertions. See [REPRODUCTION.md](REPRODUCTION.md)
for exact before/after commands and [reproduction-comparison.json](reproduction-comparison.json)
for revision, fixture and frozen output identities. The reproduction verifier's
baseline native matrix had 104 passing controls and eight specific zero-class
failures; the final verifier established 224 final matrix passes.

The final verifier authored the verification scripts in this directory independently of
the production patch. Run from a checkout with KiCad Python and the router binary;
every output directory must be fresh:

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/evidence/issue966/verify_reader_matrix.py . wk/issue966/matrix-2layer flat_hierarchy
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/evidence/issue966/verify_reader_matrix.py . wk/issue966/matrix-4layer glasgow_revC
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/evidence/issue966/verify_output_boards.py . wk/issue966/native-output
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/evidence/issue966/inspect_outputs.py . wk/issue966/native-output
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/evidence/issue966/verify_cap_boundary.py . wk/issue966/cap-boundary
```

For a matched before/after replay, the preceding `verify_output_boards.py` command
creates the frozen `zero`/`positive` inputs and plans. Point the same reproduction
script at a separate baseline checkout of `5a7fbcb6` and then the final checkout,
retaining this one fixture directory:

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/evidence/issue966/reproduce_966.py --repo ../KRT-966-repro --fixture-dir wk/issue966/native-output --output-dir wk/issue966/before --expected-zero-gui .25
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/evidence/issue966/reproduce_966.py --repo . --fixture-dir wk/issue966/native-output --output-dir wk/issue966/after --expected-zero-gui .1
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/evidence/issue966/compare_966.py wk/issue966/before/inspection.json wk/issue966/after/inspection.json wk/issue966/comparison.json
```

The matrix passed **112/112** assertions on each board at the production candidate,
including physical floors of 0.10 mm (two layers) and 0.09 mm (four layers), a
custom 0.083-mm floor, 1-nm boundaries, unchecked control values of 0.25/0.73 mm,
explicit/ceiling semantics, and plan reset. The initial evaluator's assumption
that standard-tier initial pinning used 0.127 mm was corrected after checking
the declared `fab_floor_for_param` contract: initial pinning uses the physical
floor; selected tiers govern later automatic descents.

The actual routing pair uses the unmodified public `flat_hierarchy.kicad_pcb`,
SHA256 `f756cec11151f79a9c8fe7123089130ad166f5abfd6d5f4d59aaf828b08eb05a`, and
its project SHA256 `c994fc09d25007823fefba8c2362b4c93f057d7ffd88b8d2c134e6b7cbb574e0`.
The positive control retains that project. The zero case changes only the copied
project's Default clearance to 0.0 and `rules.min_clearance` to 0.0889, matching
the issue's declared values. This parameterization is necessary because the
tracked public projects surveyed had no Default-zero example. No design brief,
custom-rule or placement-baseline sibling is present on these fixtures.

Both arms route only `Net-(D1-A)` on F.Cu/B.Cu with `max_ripup=0`; neither supplies
clearance. `verify_output_boards.py` records the exact `route.py` and `run_plan.py`
commands and writes both boards. The GUI uses the production plan executor and
real dialog. Final zero resolves to 0.10 mm and positive to 0.20 mm on both fronts.
The inspector's frozen results are in [final-output-inspection.json](final-output-inspection.json).

Native geometry inspection found exactly the same two F.Cu segments on each
front, both 0.20 mm wide: `(78.4,75.2) -> (81.9,71.7) -> (81.9,63)` mm.
Footprint/pad and Edge.Cuts identities and mechanics remain unchanged. The whole
nondefault Wide class remains unchanged at 0.4 mm, and Default zero remains zero
in the written project. These equal copper paths establish that a valid selected
net still routes; they do not demonstrate the historical DDR3 boxed-in/rip symptom.

The selected net passes `check_connected` and native DRC has zero remaining
unconnected items for that net. **The rest of this partial board is not clean:**
86 other native unconnected items remain. The pre-existing Q3 pad1/pad2 clearance
error (Wide requirement 0.4 mm, actual about 0.2807 mm) remains, along with library
and silkscreen findings; routing's existing text relocation adds a silkscreen
overlap warning. No new copper clearance error was found.

Written projects are not claimed identical between fronts. Existing CLI writeback
lowers `min_hole_clearance` and records via-drill/fabrication-origin data; GUI saved
project data remains unchanged in this experiment. This patch does not edit any
writeback path. The report records those differences instead of substituting a
different project or weakening the native audit to hide them.

## Full routing-chain positive control

The existing three-step routing/plane/routing gate ran against the public
`kicad_files/splitflap_driver.kicad_pcb` fixture, SHA256
`a9fd165133aebae844630422dc6ed0d93918c0cd9754ca14074645210af4c74a`.
The fixture has no project; the harness explicitly stages a copy with a native
KiCad-authored project before either arm. It uses the gate's existing matched
explicit parameters (clearance 0.15, 0.15, then 0.127 mm).
This chain was generated at `a29856f`; the final boundary correction affects only
cap repair, which this chain does not execute. Its routing code and dialog
clearance resolver are identical at the final production revision.

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 tests/gui_parity/test_gui_engine_parity.py --workdir wk/issue966/engine-parity
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 py_router/check_drc.py wk/issue966/engine-parity/cli_final.kicad_pcb --clearance .127 --clearance-margin .1
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 py_router/check_drc.py wk/issue966/engine-parity/gui_replay.kicad_pcb --clearance .127 --clearance-margin .1
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 py_router/check_connected.py wk/issue966/engine-parity/cli_final.kicad_pcb
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 py_router/check_connected.py wk/issue966/engine-parity/gui_replay.kicad_pcb
```

Both arms delivered 1,078 segments and 128 vias, with identical canonical copper
sets in the existing gate (0.001-mm rounding), subsequently confirmed equal at
exact native integer coordinates by the independent `verify_full_chain.py`
inspection, including via diameter, drill and layer span. Both explicit DRC invocations
reported zero violations and two permitted same-net crossing warnings. Effective
edge clearance was 0.5 mm, hole-to-hole 0.25 mm and copper-to-hole 0.127 mm;
pad-edge checking was off. Both connectivity invocations checked 83 routed nets
and reported all connected, including a native KiCad refill cross-check with
zero links. These results are limited to the checks and tolerances named here.

The final verifier also ran native KiCad DRC with zone refill on the staged
input and both outputs. All three retained five `copper_edge_clearance` errors
at fixed H1–H5 pads (about 0.452 mm actual gap versus 0.5 mm required), plus
72 library/silkscreen findings. These boards are **not globally DRC-clean**.
Native unconnected items decreased from 201 on the input to zero on each
output. The custom checker's zero above does not cover those pad-edge errors.

To reproduce the independent full-chain audit, use the output directory above
and write native reports to a fresh existing `wk/issue966/native-chain` directory:

```powershell
& 'C:/Program Files/KiCad/10.0/bin/kicad-cli.exe' pcb drc --format json --all-track-errors --refill-zones -o wk/issue966/native-chain/full-chain-input-native-drc.json wk/issue966/engine-parity/src_splitflap_driver.kicad_pcb
& 'C:/Program Files/KiCad/10.0/bin/kicad-cli.exe' pcb drc --format json --all-track-errors --refill-zones -o wk/issue966/native-chain/full-chain-cli-native-drc.json wk/issue966/engine-parity/cli_final.kicad_pcb
& 'C:/Program Files/KiCad/10.0/bin/kicad-cli.exe' pcb drc --format json --all-track-errors --refill-zones -o wk/issue966/native-chain/full-chain-gui-native-drc.json wk/issue966/engine-parity/gui_replay.kicad_pcb
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/evidence/issue966/verify_full_chain.py wk/issue966/engine-parity wk/issue966/native-chain
```

Recorded assertions: [full-chain-inspection.json](full-chain-inspection.json).
Independent findings, including evaluator discrepancies and limitations:
[FINAL-VERIFICATION.md](FINAL-VERIFICATION.md).

The existing gate's own grader returned `drc=-1`, `kicad=-1` while printing
`PARITY`. That sentinel agreement was **not accepted as verification**. The
explicit commands above supply the DRC/connectivity evidence instead. They use
`gui_replay`, which retains the project sibling; the gate's `gui_final` board-only
copy does not. Repairing that general-purpose harness is outside this reader fix.

## Focused regressions

All passed with KiCad Python on the candidate:

- `tests/gui_parity/test_manifest_plan_parity.py`: 37 flag checks across one
  manifest, zero mismatches; 13 control resolutions, zero unresolved.
- `tests/gui_parity/test_cli_postpass_coverage.py`: zero failures or warnings
  (static call-site coverage, not a routing result).
- `tests/gui_parity/test_geometry_floor_leak.py`: zero problems, including the
  distinct declared-zero hole-spacing fallback under a custom fab capability.
- `tests/test_530_class_clearance_floor.py`: nondefault class descent floors.
- `tests/test_530_legacy_ceiling_knob.py`: explicit-clearance versus ceiling behavior.
- `tests/test_493_gui_unit_conversion.py`: exact native-unit conversion.
- `tests/test_fab_tiers.py`: fabrication floor and override behavior.
- `tests/gui_parity/test_768_cap_ceiling_real_dialog.py`: inline/standalone cap
  omission and explicit override preservation, including zero-class boundaries.
- `tests/gui_parity/test_772_cap_params_reach_engine.py`: cap parameter delivery.
- `tests/gui_parity/test_782_fanout_netclass_clamp.py`: class writeback gates.
- `tests/test_768_cap_clearance_ceiling.py`: 45 tests passed.
- `git diff --check`.

The full repository suite and the historical DDR3 bench were not run. No model
trials or changes to skills/workflows are involved, and no PCB placement quality
improvement is inferred from this routing-default correction.
