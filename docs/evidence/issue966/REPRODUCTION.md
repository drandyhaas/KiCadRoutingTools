# Independent reproduction and acceptance verification for issue 966

Verified on 2026-09-14 with native KiCad 10.0.0 Python/wx/pcbnew and shipped Rust router 0.22.0 (independently imported to read `__version__`). Baseline revision: `5a7fbcb6ee4deebd1d9ec1d5bd094d8681f502f3`. Independently rerun final implementation revision: `e5681428d43a17f663df257b9a94c55c544e786c`. No production files were edited by this verifier.

## Fixture and expected outcomes

The issue has no comments. Its preferred reader-side contract is to preserve the CLI's declared-zero base and then apply the physical fab floor. The shared physical minimum on this two-layer board is 0.1 mm. Therefore omitted clearance/ceiling with declared Default 0 must resolve to 0.1 on both fronts; an unchecked GUI control is not a requirement. A genuine Default 0.2 remains 0.2. The GUI's explicit override and ceiling are separate supported operations and must retain their existing contracts.

The public tracked `kicad_files/flat_hierarchy.kicad_pcb` was copied with its project. The **positive** pair is byte-identical to this public source. The **zero** pair preserves board bytes and changes only project Default clearance from 0.2 to 0 and `board.design_settings.rules.min_clearance` to 0.0889. This is a disclosed parameterized public fixture, not a claim that the repository already contains the issue's unavailable `awx` bench. It preserves the non-Default Wide class at 0.4 and every other project requirement. The matched source pairs and plan came from the final verifier's `evidence/final-output` staging; their bytes were independently hashed and matched before/after runs. There is no placement brief, placement baseline or custom `.kicad_dru` in this fixture.

* Board SHA256, both cases: `f756cec11151f79a9c8fe7123089130ad166f5abfd6d5f4d59aaf828b08eb05a`.
* Positive project SHA256: `c994fc09d25007823fefba8c2362b4c93f057d7ffd88b8d2c134e6b7cbb574e0`.
* Zero project SHA256: `9b7172785c408a93bf6fd0669e57468ce662839ba1f9b7401548387e93283c3b`.

Route only `Net-(D1-A)` on F.Cu/B.Cu, `max_ripup=0`, default fab tier `auto` and escalation `fab`, no clearance or ceiling override. The actual GUI path is `run_plan.py` driving the native RoutingDialog route action. The two-target-pad net has an unobstructed short route; it is intentionally a control for accepted legitimate routing while exposing resolution disagreement.

## Commands and results

`reproduce_966.py` requires KiCad's Python and a **new** output directory. Use `--repo` for either checkout; the same frozen fixture directory is used for both. `commands.json` in each generated directory contains the exact executed argv, exit status and logs for all route, plan, connectivity and DRC subprocesses. The executed baseline route/plan commands are exactly those recorded there; the script subsequently gained explicit repository/expected-result arguments for portable reruns.

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 reproduce_966.py --repo C:/Users/rob/Documents/prive/git/KRT-966-repro --fixture-dir C:/Users/rob/Documents/prive/git/KRT-966-verifier/evidence/final-output --output-dir baseline-new --expected-zero-gui 0.25
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 reproduce_966.py --repo C:/Users/rob/Documents/prive/git/KRT-966 --fixture-dir C:/Users/rob/Documents/prive/git/KRT-966-verifier/evidence/final-output --output-dir candidate-new --expected-zero-gui 0.1
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 compare_966.py baseline-new/inspection.json candidate-new/inspection.json comparison.json
```

All eight actual routing invocations exited zero, wrote nonempty boards, passed target-only `check_connected.py`, and had zero native KiCad unconnected entries for the target net. These checks establish behavior beyond the process exit.

| Fixture | Baseline CLI | Baseline GUI | Candidate CLI | Candidate GUI |
|---|---:|---:|---:|---:|
| Default 0 / rules minimum 0.0889 | 0.1 | **0.25 (failure)** | 0.1 | **0.1 (fixed)** |
| Default 0.2, unmodified project | 0.2 | 0.2 | 0.2 | 0.2 |

Values are the router's `min_clearance_used` report in millimetres, **not a nearest-copper distance measurement**. The independently native-parsed delivered geometry is identical in all eight runs: two 0.2 mm F.Cu tracks, `(78.4,75.2) -> (81.9,71.7) -> (81.9,63.0)` mm. Every footprint reference/position/rotation/layer/lock, pad position/size/drill/orientation/layers/net, and Edge.Cuts shape/start/end/width matched the source exactly. The full project `net_settings` subtree, including Wide and all associations, was unchanged in each output.

`reproduction-comparison.json` contains compact source/output SHA256 identities, native geometry, and all native DRC counts. Native DRC leaves 86 board-wide unconnected items, while this scoped target has zero. The inherited copper-clearance violation (Wide-class Q3 pad geometry) remains, alongside library/silk violations; these outputs are **not globally clean**. This reproduction ran `kicad-cli pcb drc --format json`; the final verifier additionally runs `--all-track-errors --refill-zones` and compares exact baseline violation identities. The targeted connectivity utility independently performs its refill cross-check.

## Boundaries and discrepancies

Ran the independent final verifier's native dialog matrix against the baseline:

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 C:/Users/rob/Documents/prive/git/KRT-966-verifier/verify_reader_matrix.py C:/Users/rob/Documents/prive/git/KRT-966-repro evidence/baseline-matrix flat_hierarchy
```

Result: exit 1 for **8 asserted mismatches**, with **104 of 112 cases already passing**. This is an expected-failure result with recorded reasons, not an import/argument accident. Six failures are Default-zero/unchecked controls under standard, advanced and auto: a stale typed 0.25 or 0.73 becomes the resolved value instead of 0.1. The remaining failures are a declared physical custom clearance 0.083 resolving to stale control 0.3, and reset-followed-by-omitted-plan-clearance resolving to 0.25 instead of 0.1. Positive values, values around the physical floor, explicit overrides, ceiling semantics and the distinct zero-hole fallback were already accepted by this baseline matrix. The final verifier owns candidate matrix verification and alternate action coverage.

The sparse public board reproduces **wrong rule resolution**, not a failed route: baseline and candidate both connect the chosen net with identical copper. Thus it does not establish improvement in DDR routing completion, rip-victim survival, or PCB quality. The issue's historical `awx` boxed-in/ripup claims and writer proposal remain unverified hypotheses; this reader fix neither requires nor implements that writer proposal. Output DRC-floor writeback differences between frontends are disclosed by the final verifier and are pre-existing; this patch leaves their code unchanged.

The final verifier discovered that intermediate candidate `a29856f` unintentionally changed Optimize Caps pricing from 0.25 to 0.1 because that placement path consumed the routing helper. This reproduction verifier independently ran `verify_cap_boundary.py` on baseline `5a7fbcb`: native Optimize Caps logged `cap pair clearance: 0.25mm (cli)`, matching the placement CLI's omitted fixed default of 0.25. It returned zero moves because the public fixture has no BGA fanout vias. This establishes the pre-change pricing contract, not placement movement quality. The parent corrected that call site in `e568142` by preserving the omitted argument for the placement resolver; the final verifier owns rerunning its actual boundary. All four actual routing commands and native output inspections in the table were rerun on `e568142` after this correction. The earlier four `a29856f` routing runs produced the same results, but the packaged comparison pins the final candidate, not that intermediate patch.

Acceptance for this issue: zero/omitted and stale-control resolution agrees with CLI physical flooring; positive and explicit controls remain supported; shared GUI route paths consume the same corrected effective clearance; written outputs preserve mechanical facts and requirement siblings; no claim of globally clean output or repair of unrelated DDR failures. This independent reproduction establishes the first, second and artifact portions. Final candidate call-site/alternate-path audit and complete required regressions are separately owned by the final verifier.
