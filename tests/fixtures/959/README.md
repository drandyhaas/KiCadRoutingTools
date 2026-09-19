# #959 fixtures: run 29's own declare-stage artifacts

Copied from the run-29 work directory (`wk/run29/esp_prog/` on branch `run29-harness`). `wk/` is gitignored, so the run itself is not in the repo, and these files are the minimum the #959 tests need to replay what that run saw.

| file | what it is | source file | sha256 |
|---|---|---|---|
| `run29_pile.kicad_pcb` | the staged pile `stage_unaided` wrote: every part at the board centre except the mechanical refs, with no locks | `board.kicad_pcb` | `e9ec45ea…8034` |
| `run29_pile.kicad_pro` | its project sibling (carries the DRC floor; never copy a board without it) | `board.kicad_pro` | `e11a318c…1963` |
| `zone_plan_r1.json` | the run's first zone plan (six zoned blocks, `fiducial-nw` among them, `Ref[*]~2` in `must_lock`) | `zone_plan.json` | `20bef455…6881` |

What the run's own record says about these files:

- The pile's decap census was measured ON THE PILE: `max_mm 0.0`, population 4, tethers 3, one cap (C1) beyond the radius. A strict `--declare-decaps` there derives a 0.0 limit.
- `zone_plan_r1.json` escapes the reference `Ref*` as `Ref[*]`, so run 29 never hit the fnmatch trap where `Ref*` also matches `Ref*~2`.
- The two `intent_zone_overlap` findings reported at lap 5 came from a later draft of the second plan, which the run amended within a minute. `tests/test_959_plan_check.py` carries that draft, reconstructed from the session record.
