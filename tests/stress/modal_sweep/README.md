# Modal parameter sweep

Fan a routing-parameter sweep across the stress corpus on rented cores, so
"what does this knob do across 150 boards?" takes ~an hour instead of a day.

No LLM is involved. Every recorded board leaves a `redo_commands.sh` manifest
that replays its whole chain deterministically, so a sweep is pure CPU.

## Why one task per (arm, board)

The wall-clock floor of the whole sweep is **the single longest board**. An arm
run as one task is ~9 CPU-hours *serial*, so 100 of those in parallel still take
9 hours. Fanning out to board granularity makes the floor a board (~50 min on
sets 1-10, `daisho`) instead of an arm.

Tasks are dispatched **longest-first** from recorded timings. Round-robin order
lets a 50-minute board start near the end and add 50 minutes to the tail.

## Measured sizing (sets 1-10, 150 boards)

| | |
|---|---|
| One arm | ~8.6 CPU-hours (12.7 CPU-h of LLM tool time x 0.68 replay ratio) |
| 100 arms | ~864 CPU-hours, 15,000 tasks |
| Floor | ~50 min (`daisho`) |
| Cores to reach the floor | ~1,730 |
| 500 cores | ~1.7 h |
| CPU cost | ~$41 at Modal's $0.0000131 / core / sec |
| RAM cost | ~= actual usage, since the 1 GB reservation sits below it (see below) |

Cost is not the constraint; **burst concurrency quota is**. 1,700 concurrent
containers is above the default limit on every platform — plan on 500-1,000
for the first run and raise the limit if the tail matters.

## Memory: one 1 GB tier — a reservation buys OOM-protection, not capacity

Everything rides a **1 GB** tier (`DEFAULT_MEM_MB`). The **8 GB** tier
(`BIG_MEM_MB`) is opt-in OOM insurance, not a capacity requirement.

That is counter-intuitive, so the measurement matters. **Modal bills
`max(reserved, used)` per second, and containers BURST above the reservation.**
Resized from **414 measured cloud tasks** (real Linux RSS, `d56ef8b`):

| reservation scheme | cost vs the pure-usage floor |
|---|---|
| **flat 1 GB** | **+0.3%** (673 vs 671 GB-h) |
| 4 GB + 12 GB tiers | +47% |
| per-board known peak | +78% — a board's max bills against every light arm's runtime |

Real Linux peaks on that sample: **p50 2.1 GB, p90 5.2 GB, max 8.6 GB**
(`hackrf_one`, every arm) — and **all of them completed in 2 GB containers via
burst**. Reserving to the peak buys nothing except the bill.

**Do not size these containers from macOS `peak_footprint_mb`.** That metric is
darwin-only, and an earlier version of this file recommended exactly that: it
produced the 4/12 tiering above, i.e. +47% for no benefit. `peak_footprint` is
still the right number for sizing a *local* macOS run — where RSS under-reports
by up to 5x (mimalloc-retained + IOAccelerator-tagged pages, issue #419) — but it
does not transfer to a cloud reservation. Measure on the target OS.

Raise a board to the 8 GB tier only if it actually OOMs.

## Three ways to express an arm

See `arms.example.json`. Always include a **baseline arm with no overrides** —
without an anchor, corpus or commit drift reads as "every arm regressed".

| mechanism | for | how |
|---|---|---|
| `env` | `KICAD_*` knobs the engine already reads | passed to the subprocess |
| `defaults` | `routing_defaults.py` module constants | the file is patched in the container's own repo copy |
| `manifest_sed` | parameters that are CLI **flags** | regex over `redo_commands.sh` |

`defaults` patching is what the documented local A/B recipe does with
`git stash`. Doing it per container needs no shared git state — which is exactly
what lets 100 arms run **concurrently** rather than sequentially, the constraint
called out in `ab_replay_grade.py`'s docstring.

An unknown constant name raises rather than being ignored: a typo that silently
tests nothing would look like "the parameter has no effect".

Anchor `manifest_sed` patterns tightly — a loose one rewrites more of the chain
than you intended.

## The files

| file | what it is |
|---|---|
| `modal_app.py` | the app: fan-out, staging, parameter application, aggregation |
| `sweep_lib.py` | pure-python helpers (no `modal` import) — cost table, discovery, patching |
| `upload_corpus.py` | populate the corpus volume (`--boards` for a small slice) |
| `arms.example.json` | template showing all three arm mechanisms |
| `arms.smoke.json` | the 2-arm config the smoke test uses |
| `smoke_test.sh` | **run this first** — end-to-end, self-verifying |
| `run_sweep.sh` | a full sweep: plan → run → verify, with guard rails |
| `arms.track_proximity.json` | worked example of a real parameter sweep |
| `check_sweep.py` | validates a sweep result; exits non-zero if untrustworthy |

## Start here: the 2-board smoke test

Do this BEFORE the full sweep. A few cents, and it exercises every moving part:
image build, corpus volume, the absolute-path remap, parameter patching,
container reuse, grading, result harvesting.

```bash
pip install modal && modal setup          # interactive browser login, once
bash tests/stress/modal_sweep/smoke_test.sh
```

That uploads a ~37-file slice (two of the cheapest boards), runs 2 arms × 2
boards, and then **checks the result** with `check_sweep.py`. Override with
`SMOKE_SET`, `SMOKE_BOARDS`, `SMOKE_OUT`. First run pays a cold image build of a
few minutes; later runs reuse it.

**Why the run is CHECKED rather than eyeballed.** A sweep can exit green and be
meaningless. During bring-up one run reported `chain_complete` on every row,
100% completion and 0 DRC — while both arms had secretly executed with the same
routing constants, because the image shipped an uncommitted working-tree edit
and the "baseline" was not the baseline. At 100 arms that reads as "the
parameter has no effect", and nothing in the output contradicts it.

`check_sweep.py` tests four things:

1. **chain_complete** — a broken chain is not a result. On failure it prints the
   captured `_replay.log` tail, since the container is gone by then.
2. **arm spec applied** — an arm declaring `defaults` must report non-empty
   `patched_defaults`; inconsistency across boards means container reuse is
   leaking arms.
3. **arms actually differ** — separating routing *outcomes* from *runtime*.
   Runtime alone is noisy: the contaminated run above still differed 5% and
   0.06% on `total_seconds`, so a naive "do they differ?" passes it. Anything
   under 10% is treated as jitter, not proof.
4. **provenance** — every row carries a source commit; a `+dirty` sha is
   reported.

Run it on any sweep, not just the smoke test:

```bash
python3 tests/stress/modal_sweep/check_sweep.py <sweep.json>
# --require-all-complete  : any broken chain fails (right for the smoke test;
#                           a real sweep legitimately has some failing boards)
```

## A full sweep with a parameter change

```bash
# first time only: populate the corpus volume (~2.2 GB)
SWEEP_UPLOAD=1 SWEEP_DRY=1 \
  bash tests/stress/modal_sweep/run_sweep.sh \
       tests/stress/modal_sweep/arms.track_proximity.json

# then run it
bash tests/stress/modal_sweep/run_sweep.sh \
     tests/stress/modal_sweep/arms.track_proximity.json
```

`arms.track_proximity.json` is a worked example: a baseline plus
`TRACK_PROXIMITY_COST` at 0.5 / 2.0 / 4.0, and one arm pairing it with
`RIPPED_ROUTE_AVOIDANCE_COST`. Copy it and edit. Bracketing a value like that is
deliberate — a monotone trend across three points is far stronger evidence than
one arm beating baseline once.

Its plan over sets 1-10:

```
5 arms x 150 boards = 750 tasks
  estimated 64 CPU-hours; floor (longest board) 50 min
  0 tasks routed to the big tier
```

`run_sweep.sh` **always prints that plan before spending anything**, then runs,
then pipes the result through `check_sweep.py`. It refuses an arms file with no
baseline arm, with duplicate names, or that isn't a non-empty list.

| env | effect |
|---|---|
| `SWEEP_SETS=set1,set2` | override the sets (or pass as `$2`) |
| `SWEEP_UPLOAD=1` | populate the corpus volume first (one-off) |
| `SWEEP_DRY=1` | stop after the plan, spend nothing |
| `SWEEP_OUT=/path.json` | where to write the result |
| `KICAD_SWEEP_DIRTY=1` | ship the working tree instead of a clean `HEAD` |

Note the verification here does **not** use `--require-all-complete`: a real
sweep legitimately contains boards whose chain breaks. Those are reported; the
hard failures are "all arms identical" and "no provenance", which mean the sweep
answered nothing.

For a quick look, `modal_app.py --limit N` keeps the N *cheapest* boards — the
default order is longest-first, which is right for throughput but would hand you
the 50-minute board first.

Output: a per-arm table (boards, chain-complete, mean completion %, total real
DRC, CPU-hours) plus every raw row, written to `sweep_<ts>.json`. Each
`(arm, board)` row uses the **same schema `ab_replay_grade.py` emits**, because
each task literally calls it on a one-board set dir — so no grading logic is
duplicated, and `ab_replay_grade.py --compare` still works on the results.

## Verified vs not

Tested here, against real repo files and the real corpus:

- cost table, board discovery (150/150 across sets 1-10), longest-first order
- `routing_defaults.py` patching: values land, the patched module imports, inline
  comments survive, no line drift, unknown names rejected
- recorded-repo-prefix detection from a real manifest
- all three files byte-compile

**Not verified — no Modal account or Linux box was available:**

- **The Modal API calls.** Image-builder method names have moved across client
  versions (`add_local_dir(..., copy=True)` vs the older `copy_local_dir`). Pin
  `modal` and expect to adjust if you are on an older client.
- **The Linux `grid_router` build.** Resolved as of v0.20.2 (2026-08-09): that
  release publishes `grid_router-linux-x86_64.so` built from crate 0.20.1, so the
  image just runs `build_router.py` and keeps the prebuilt — no Rust toolchain,
  ~10 min off the cold build. Verified on the macos-arm64 asset (`__version__`
  reports 0.20.1, matching `Cargo.toml`); the *linux* asset has not been imported
  on a linux box yet, which the smoke test covers. The image deliberately fails
  at build time if a future crate bump ships without binaries — see the comment
  in `modal_app.py` for the two-line fallback.
- **An end-to-end replay in a container.** Manifests bake absolute tool paths;
  `recorded_repo_prefix` detects each manifest's own prefix and remaps it, and
  that logic is tested — but the remap has not been exercised inside a container.
  Run one board, one arm before launching 15,000 tasks.

## Deliberately out of scope

- **No KiCad in the image.** Replay and grading are pure python
  (`check_drc.py` imports numpy/scipy/shapely only), so KiCad buys nothing here
  and costs a large image. The price is `ab_replay_grade`'s optional
  `kicad-cli` cross-check degrading to `None`. Cross-check the *winning* arm
  locally, where KiCad already lives — that check caught #487, where
  `clearance: warning` blinded the grader.
- **Building new corpus sets.** Still needs `pcbnew` for prep; a local one-time
  step, unrelated to sweeping.

## Screen before you sweep

100 arms x 150 boards is ~864 CPU-hours. Running all 100 arms against a ~30-board
subset costs ~10 minutes and tells you which ~20 arms deserve the full corpus.
Same answer about the winners, roughly a fifth of the compute, first results in
minutes.
