# #959 fixtures: run 29's own declare-stage artifacts

Copied from the run-29 work directory (`wk/run29/esp_prog/` on branch `run29-harness`). `wk/` is gitignored, so the run itself is not in the repo, and these files are the minimum the #959 tests need to replay what that run saw.

| file | what it is | source file | sha256 (of the committed LF bytes) |
|---|---|---|---|
| `run29_pile.kicad_pcb` | the staged pile: every part at the board centre except the mechanical refs, with no locks | `board.kicad_pcb` | `c9ed55ee…828d` |
| `run29_pile.kicad_pro` | its project sibling (carries the DRC floor; never copy a board without it) | `board.kicad_pro` | `9a9ab694…6395` |
| `zone_plan_r1.json` | the run's first zone plan (six zoned blocks, `fiducial-nw` among them, `Ref[*]~2` in `must_lock`) | `zone_plan.json` | `702f8fc0…85b9` |
| `zone_plan_r2_lap5.json` | the lap-5 draft of the second plan, which the run amended within a minute. Reconstructed from the session record, not copied from a file: its two overlaps (1.60 and 0.6325 mm2) are the ones the lap-5 grade reported | (session record) | `81c101c8…2845` |

Readers: `run29_pile.kicad_pcb` (its `.kicad_pro` comes along as the sibling) is the pile in `test_959_decaps_auto`, `test_959_padless_blocks`, `test_959_pose_intent`, `test_959_rule_roster`, `test_1064_place_pose_same_net_stack`, `test_1100_1103_run37_followups`, `test_1178_decaps_from_shared_origin` and `test_check_assembly_pile_baseline`; `zone_plan_r1.json` is read by `test_959_rule_roster`, `zone_plan_r2_lap5.json` by `test_959_pose_intent`.

`asbuilt/` holds the design briefs of five as-built boards (esp_prog, glasgow_revC, splitflap_driver, tigard, ulx3s), written for #1000: the calibration set for how connector declarations compile to clauses (`docs/design-brief.md`, "Connector declarations compile to clauses"). No test reads them; they are the evidence that mapping was measured on.

The hashes are of the bytes git stores (LF line endings). A Windows checkout with `core.autocrlf` writes CRLF and hashes differently; pin against `git show HEAD:<path>`, not the working copy.

What the run's own record says about these files:

- The pile's decap census was measured ON THE PILE: `max_mm 0.0`, population 4, tethers 3, one cap (C1) beyond the radius. A strict `--declare-decaps` there derives a 0.0 limit.
- `zone_plan_r1.json` escapes the reference `Ref*` as `Ref[*]`, so run 29 never hit the fnmatch trap where `Ref*` also matches `Ref*~2`.
- The two `intent_zone_overlap` findings reported at lap 5 came from a later draft of the second plan, which the run amended within a minute. `zone_plan_r2_lap5.json` is that draft; both overlaps are satisfiable (WARNs since #959).
