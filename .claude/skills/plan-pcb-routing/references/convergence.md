# Converging a board

The loop that turns "the chain ran" into "the board is done". Read this with
Step 9 of `SKILL.md`; the evidence keys are in
[`evidence-map.md`](evidence-map.md) and the verifier contract is in
[`verifier-prompts.md`](verifier-prompts.md).

## Why this exists

A full chain ran end to end against a real board and every tool reported success.
The board it produced:

| gate | actual |
|---|---|
| `check_connected` | **39 of 44 nets** — one net unrouted, four partial |
| KiCad DRC | **762 errors**, 83 warnings |
| vias vs the board's own spec (0.6 mm ⌀ / 0.15 mm ring) | **141 of 141 violated** |
| segments vs the spec's 0.15 mm floor | 267 routed at 0.127 mm |
| the USB pair vs its HARD 0.8 mm width | no 0.8 mm segment at all |
| declared length rules | 15 failures |

Nothing stopped it, because **nothing had measured the board**. Every number
above was available from a tool that already shipped; none was asked for. This
loop asks.

## The invariant

> A board is done when `blocking == 0` **and** every verifier lens passes.
> Anything else is reported with its measurement and its stop condition —
> never as "finished".

## 0. Before the first iteration

1. **Step 0a locks.** Everything the spec fixes in place is locked and recorded
   in the intent as `must_lock` (plus a tight `blocks` zone for exact XY). If you
   skip this, the loop will happily optimise a connector out of its footprint.
2. **The intent exists.** Without `--intent` the floorplan component is
   `ungraded`, and `ungraded` is not `passed`.
3. **The size floors are known.** If the spec states a track width, via diameter
   or annular ring, they go on every `board_score.py` call. `check_drc`'s default
   is the *fab* floor for the layer count, which is looser than most specs — that
   gap is exactly how 141 spec-violating vias graded clean.
4. **The budget is 100 per board.** Not 20 — that figure assumed every iteration
   meant a full chain re-run, and it does not (SKILL 9.3a: re-enter at the failing
   step, which takes seconds). Count **completion** iterations (the copper
   changed) separately from **systemic** ones (the chain's floors, classes, zone
   fill or a checker changed). Three consecutive systemic iterations means you are
   tuning the instrument, not the board — go and look at what is unrouted.
5. **The lever is chosen by CONNECTIVITY, not by the largest number.** Work
   `unrouted` → `broken` → widths → floorplan → `drc`, in that order, whatever
   their sizes. A run that let the biggest `blocking_by` entry pick spent eleven
   iterations on clearances — 16 of the 18 of which were grading artifacts — while
   five nets carried no copper at all.

## 1. The iteration

```
  score  ->  classify the top blocker  ->  pull the cheapest lever
     ^                                              |
     |                                              v
  ledger  <-  accept (blocking fell) or revert (it did not)
```

### Score

```bash
python3 -X utf8 .claude/skills/plan-pcb-routing/scripts/board_score.py \
    wk/iter03.kicad_pcb \
    --intent wk/floorplan.json \
    --min-track-width 0.15 --min-via-diameter 0.6 --min-via-drill 0.3 \
    --impedance-nets '*USB*' \
    --length-groups wk/length_groups.json \
    --json wk/score_iter03.json
```

Omit `--clearance`. `check_drc` then reads the sibling `.kicad_pro`, which is the
floor the board was actually routed to; a guessed round number manufactures
phantom violations on legitimately tight copper.

Exit codes: `0` blocking is zero · `4` graded with blockers · `3` board state ·
`2` bad arguments · `1` crash.

### The lever ladder — cheapest first

| # | lever | cost | when |
|---|---|---|---|
| 1 | **re-run one chain step** with different parameters (width, grid, ripup, layer costs) | seconds–minutes | `blockers` empty, or `undersized` non-zero, or "boxed in by static obstacles" |
| 2 | **`place_route_loop`** with the Step 0a locks, then re-run the chain | minutes×rounds | `blockers` non-empty, failures scattered, failing refs are ≤40-pin passives |
| 3 | **revise the intent zones**, re-place, re-run the chain | the whole chain | failures cluster into ≤2 pockets sharing one block — a 3 mm nudge cannot move a block 80 mm |

**Never skip to 3 because it feels thorough.** The default answer is 1.

### Accept or revert

Accept **only** if `blocking` strictly decreased, or `blocking` is unchanged and
`quality` improved. Otherwise record the entry `--rejected` and step back to the
parent (`converge.py step-back` checks out the last accepted board byte-exact).

`quality` is `(vias, copper_mm, segments)` and is compared **only at
`blocking == 0`**. Comparing it earlier lets a router trade a disconnected net
for a lower via count.

## 2. The ledger — `wk/ledger.jsonl`

One record per iteration, appended **before** the next iteration starts.

**Write it with `converge.py record`, not by hand.** The tools that read a ledger
— `converge.py step-back` / `replay` / `status`, and `make_film.py --from-ledger`
— read append-only **JSONL** through `board_store.Ledger`. A hand-written single
JSON document is readable by a person and by nothing else, so the byte-exact step
back and the replay are both unreachable from it. It also loses the content
addressing: `record` stores the board by SHA, which is what makes stepping back
exact after three iterations have overwritten the same path.

```bash
python3 -X utf8 py_placer/converge.py record --ledger wk/ledger.jsonl \
    --board wk/iter03.kicad_pcb --kind completion \
    --lever 'rip lever: --rip-existing-nets QSPI_SD2 + --grid-step 0.025' \
    --score-file wk/score_iter03.json \
    --argv python3 -X utf8 py_router/route.py wk/iter02.kicad_pcb wk/iter03.kicad_pcb --nets QSPI_SD1

# `--argv` is a REMAINDER: everything after it is the command. Do NOT write
# `--argv -- python3 ...` -- the bare `--` lands inside the captured argv and
# argparse rejects the line.
```

**Then run `converge.py status --ledger wk/ledger.jsonl` every iteration and read
what it prints.** It is the alarm for the failure this whole section exists to
prevent: it splits the budget into completion vs systemic and warns when at least
half went to the instrument. A run once spent nine of eleven iterations on how the
chain measures itself and finished with five nets carrying no copper — `status`
says that out loud, and nothing else in the loop does.

The shape below is what `record` ACTUALLY writes — one JSONL line per
iteration (there is no wrapper object, no `convergence.json`; the ledger IS
the `.jsonl` file):

```jsonc
{"iteration": 3,                       // position in the ledger
 "kind": "completion",                 // or "systemic": budget went to the instrument
 "parent_sha": "9c41f0...",            // result_sha of the last ACCEPTED entry
 "result_sha": "2ab77e...",            // content hash; step-back checks it out byte-exact
 "lever": "rip lever: --rip-existing-nets GPIO7, width pinned",
 "lever_argv": ["python3", "-X", "utf8", "route.py", "..."],  // what makes replay possible
 "score": {"blocking": 16, "blocking_by": {"unrouted": 5, "drc": 11}},
 "accepted": true}
```

Fields that carry weight:

- **`parent_sha` / `result_sha`** — boards live in the content store, not at
  paths; `converge.py step-back --to <sha>` checks one out byte-exact. The
  parent is the last *accepted* board, **not** iteration N−1 — it is what
  `render_placement --before` takes; using N−1 renders a delta that never
  existed.
- **`lever` + `lever_argv`** — `lever` is the one-line intent; `lever_argv` is
  the reproducible command (`replay` refuses prose-only entries, exit 4).
  Anything the schema has no field for — the verdict list, a stop-condition
  claim — goes **into the `--lever` text by name** so `status`/the report can
  quote it; do not invent fields the reader will never see.
- **`kind`** — `systemic` marks iterations spent on the instrument (grader
  fixes, reconciliation); `status` warns when at least half the budget went
  there.
- **`accepted`** (`--rejected` at record time) — a rejected iteration is data,
  not a mistake. Keeping it is what makes stop condition 3 detectable.

## 3. Stop conditions

| # | condition | what to report |
|---|---|---|
| 1 | `blocking == 0` **and** every lens passes | done — quote the score and the lens list |
| 2 | budget exhausted — **100 ledger entries actually written** | the best-scoring board **and** every remaining blocker, itemised with measurements |
| 3 | **5** consecutive iterations with `unrouted` AND `broken` both unchanged, after trying the rip lever, a finer grid and a layer change on the failing nets | floorplan-limited or spec-limited — say which, with the number |
| 4 | a blocker is geometrically unsatisfiable | a finding **about the requirement**, with the measurement that proves it |

Stop condition 4, worked: a requirement asked for 2.4 mm edge-to-edge clearance
from unrelated nets on a USB pair. Written as a **netclass** it also applies
pad-to-pad, and on that receptacle the measured pad gaps were **0.500 mm** (VBUS)
and **1.300 mm** (GND) — nearer than 2.4 mm before any track exists. With it:
23/44 nets. Without: 38/44. It is unsatisfiable as written, it took one
measurement to prove, and the honest output is *"this requirement needs a
keepout or a `.kicad_dru` track-scoped rule, not a netclass"* — not a quietly
relaxed class, and not a budget spent grinding.

**Ending on 2, 3 or 4 is legitimate. Ending on them while calling the board
finished is not. Ending on none of them is not an ending at all.**

### What is NOT a stop condition

Wall-clock, fatigue, "the score stopped moving", "the remaining work is hard", or
"the findings are written up". A run once stopped at **11 of 20**, called it
"budget exhausted", and said in its own ledger that the levers were not exhausted.
Before invoking 2 or 3, answer in writing: how many nets are unrouted, what is the
router's own hint for each, and which rip rule has not been tried on them.

## 4. The movie

Every accepted board, in order, is the frame list — that is what the ledger is
for. `place_route_loop` renders its own per-round movie by default; this is the
film of the whole convergence:

```bash
python3 -X utf8 py_router/make_movie.py \
    wk/iter00.kicad_pcb wk/iter01.kicad_pcb wk/iter04.kicad_pcb wk/iter07.kicad_pcb \
    -o wk/convergence.gif --size 1600 --fps 12 --chunks 30 --end-hold 12
```

- `.mp4` needs `imageio` + `imageio-ffmpeg` and silently falls back to a sibling
  `.gif`. Ask for `.gif` directly when you know they are missing.
- Hand it over with `SendUserFile`. **Do not `Read` it** — show-without-reading,
  and its frames would spend the ≤3-image budget for nothing.

## 5. What the final report must contain

1. The **stop condition** that fired, by number.
2. The final `blocking` and its `blocking_by` breakdown.
3. Everything in `ungraded` — named as **unexamined**, never as clean.
4. Iterations used, out of budget.
5. The convergence movie.
6. If `blocking > 0`: every remaining blocker with its measurement, and what
   would have to change to fix it.

A board handed over with `blocking > 0` and no itemised list is the failure this
whole loop exists to prevent.
