# #959 fixtures: run 29's own declare-stage artifacts

Copied from the run-29 work directory (`wk/run29/esp_prog/` on branch `run29-harness`). `wk/` is gitignored, so the run itself is not in the repo, and these files are the minimum the #959 tests need to replay what that run saw.

| file | what it is | source file | sha256 (of the committed LF bytes) |
|---|---|---|---|
| `run29_pile.kicad_pcb` | the staged pile `stage_unaided` wrote: every part at the board centre except the mechanical refs, with no locks | `board.kicad_pcb` | `c9ed55ee…828d` |
| `run29_pile.kicad_pro` | its project sibling (carries the DRC floor; never copy a board without it) | `board.kicad_pro` | `9a9ab694…6395` |
| `zone_plan_r1.json` | the run's first zone plan (six zoned blocks, `fiducial-nw` among them, `Ref[*]~2` in `must_lock`) | `zone_plan.json` | `702f8fc0…85b9` |
| `run29_mechanical.json` | the mechanical declaration `stage_unaided` wrote beside the pile: `Ref*`, `Ref*~2` and `USB1` at their source poses (USB1 on the WEST edge, which the brief contradicts), plus the staging floors | `mechanical.json` | `1d99b10d…19af` |
| `zone_plan_r2_lap5.json` | the lap-5 draft of the second plan, which the run amended within a minute. Reconstructed from the session record, not copied from a file: its two overlaps (1.60 and 0.6325 mm2) are the ones the lap-5 grade reported | (session record) | `81c101c8…2845` |

The hashes are of the bytes git stores (LF line endings). A Windows checkout with `core.autocrlf` writes CRLF and hashes differently; pin against `git show HEAD:<path>`, not the working copy.

What the run's own record says about these files:

- The pile's decap census was measured ON THE PILE: `max_mm 0.0`, population 4, tethers 3, one cap (C1) beyond the radius. A strict `--declare-decaps` there derives a 0.0 limit.
- `zone_plan_r1.json` escapes the reference `Ref*` as `Ref[*]`, so run 29 never hit the fnmatch trap where `Ref*` also matches `Ref*~2`.
- The two `intent_zone_overlap` findings reported at lap 5 came from a later draft of the second plan, which the run amended within a minute. `zone_plan_r2_lap5.json` is that draft, and `tests/test_959_plan_check.py` shows both overlaps are satisfiable (WARNs since #959).
