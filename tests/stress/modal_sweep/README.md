# Modal parameter sweep

Fan a routing-parameter sweep across the stress corpus on rented cores, so
"what does this knob do across 150 boards?" takes ~an hour instead of a day.

No LLM is involved. Every recorded board leaves a `redo_commands.sh` manifest
that replays its whole chain deterministically, so a sweep is pure CPU.

This directory is also the cloud engine behind `tests/stress/cloud_replay_sets.py`
(a no-knob replay of whole sets that keeps the routed boards) and
`tests/stress/corpus_bisect.sh`; both drive `modal_app.py`. Their recipes are in
`tests/stress/RUNBOOK.md` → "Corpus-scale A/B and bisect on the cloud".

## Why one task per (arm, board)

The wall-clock floor of the whole sweep is **the single longest board**. An arm
run as one task is ~9 CPU-hours *serial*, so 100 of those in parallel still take
9 hours. Fanning out to board granularity makes the floor a board (~50 min on
sets 1-10, `daisho`) instead of an arm.

Tasks are dispatched **longest-first** from recorded timings. Round-robin order
lets a 50-minute board start near the end and add 50 minutes to the tail.

## Measured sizing (sets 1-10, 150 boards, August 2026)

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

Raise a board to the 8 GB tier only if it actually OOMs, by naming it in
`KICAD_SWEEP_BIG_BOARDS=board1,board2` for a top-up run.

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

The control arm is the first one declaring none of the three, whatever its
name. Give it a name unique to the campaign (`baseline590`, `base2127`): the
results volume banks rows **by arm name** (see "Results volume and resume"), so
a plain `baseline` would resume a previous campaign's rows from another commit.

An arm may also carry `"keep_artifacts": true` to keep each final board — with
its `.kicad_pro`/`.kicad_dru`/`.kicad_prl` siblings and the replayed manifest —
under `<arm>/artifacts/<set>__<board>/` on the results volume.
`cloud_replay_sets.py` sets it; a sweep that only needs the numbers should not.

## The files

| file | what it is |
|---|---|
| `modal_app.py` | the app: fan-out, staging, parameter application, resume, aggregation |
| `sweep_lib.py` | pure-python helpers (no `modal` import) — recorded-cost table, discovery, patching |
| `upload_corpus.py` | populate the corpus volume (`--boards` for a small slice, `--dry-run` for the size) |
| `arms.example.json` | template showing all three arm mechanisms |
| `arms.smoke.json` | the 2-arm config the smoke test uses |
| `smoke_test.sh` | **run this first** — end-to-end, self-verifying |
| `run_sweep.sh` | a full sweep: plan → run → verify, with guard rails |
| `arms.track_proximity.json` | worked example of a real parameter sweep |
| `check_sweep.py` | validates a sweep result; exits non-zero if untrustworthy |
| `rank_arms.py` | scores a sweep's arms by PAIRED connectivity verdict |
| `prescreen.py` | local dose-range prescreen of one knob on two boards, before any cloud spend |
| `curate_boards.py` | writes `board_value.json` from a results-volume snapshot |
| `board_value.json` | the 12-board `screen` set and the zero-information `exclude` list |
| `regrade_arm.py` | re-grades an existing arm's kept boards on Modal with one checkout's grader |
| `arms.history590*.json`, `retry_shape_2127.json`, `PREREG_590_sets21_27.md` | the #590 corpus A/B: its arm files, board list and pre-registration |

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

1. **chain_complete** — a broken chain is not a result. A real sweep
   legitimately has some, so broken rows are a warning until their RATE passes
   `--max-broken-frac` (default 0.25), which reads as systemic and fails. It
   prints the captured `_replay.log` tail, since the container is gone by then.
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
baseline arm, with duplicate names, or that isn't a non-empty list. It names the
Modal app after the arms file (`arms.586.json` → `kicad-sweep-586`) so runs are
distinguishable on the dashboard.

| env | effect |
|---|---|
| `SWEEP_SETS=set1,set2` | override the sets (or pass as `$2`); default sets 1-10 |
| `SWEEP_UPLOAD=1` | populate the corpus volume first (one-off) |
| `SWEEP_DRY=1` | stop after the plan, spend nothing |
| `SWEEP_OUT=/path.json` | where to write the result |
| `SWEEP_EXCLUDE=a,b` | skip these boards this run — the wall-clock lever for multi-hour monsters |
| `KICAD_SWEEP_DIRTY=1` | ship the working tree instead of a clean `HEAD` |

Note the verification here does **not** use `--require-all-complete`: a real
sweep legitimately contains boards whose chain breaks. Those are reported; the
hard failures are "all arms identical" and "no provenance", which mean the sweep
answered nothing.

For a quick look, `modal_app.py --limit N` keeps the N *cheapest* boards — the
default order is longest-first, which is right for throughput but would hand you
the 50-minute board first.

Output: a per-arm table (boards, chain-complete, mean completion %, total real
DRC, CPU-hours) plus every raw row, written to `--out` or to
`<stress dir>/sweep_<ts>_<arm names>.json`. Each `(arm, board)` row uses the
**same schema `ab_replay_grade.py` emits**, because each task literally calls it
on a one-board set dir — so no grading logic is duplicated — plus the sweep's own
fields (`arm`, `git`, `deps`, `patched_defaults`, `env_overrides`,
`sweep_wall_s`, and `replay_log_tail` on a broken chain).

## Scoring the arms: `rank_arms.py`

The run's own table (mean completion %) is not the verdict. `rank_arms.py`
scores every arm against the control on `nets_incomplete`, paired on boards
where every arm replayed the same chain, and reports W/L per arm with DRC beside
it rather than folded in:

```bash
python3 tests/stress/modal_sweep/rank_arms.py <sweep.json>
#   --drop-rescue-clean   drop boards whose recorded rescue step biases them
#   --hard N              score only boards where the control leaves >=N nets incomplete
#   --exclude B1,B2       sensitivity check (the headline stays unfiltered)
#   --perf                also report wall / CPU / iterations / peak RSS
```

The rules behind it (pair on identical chains, score `nets_incomplete` alone,
disclose the rescue cell, never trust a two-board result) are
`tests/stress/RUNBOOK.md` → "Rules that make these trustworthy".

## `modal_app.py` options

`run_sweep.sh` and `cloud_replay_sets.py` call it for you; run it directly with
`modal run tests/stress/modal_sweep/modal_app.py --arms <arms.json> ...`.

| option | effect |
|---|---|
| `--sets` | comma-separated set names (default `set1`..`set10`) |
| `--boards a,b` | only these boards |
| `--limit N` | only the N cheapest boards |
| `--exclude a,b` | skip these boards this run |
| `--ignore-excludes` | do not drop `board_value.json`'s `exclude` list (a new knob family's first sweep should not inherit it) |
| `--stage screened` | successive halving: the `screen` boards first, then the full set for surviving arms (see "Screen before you sweep") |
| `--gate-margin F` | screened-stage kill threshold (default 0.25) |
| `--dry-run` | print the plan, spend nothing |
| `--out PATH`, `--stress-dir DIR` | result path; local corpus root (default `~/Documents/kicad_stress_test`) |

| env (read on the launching machine) | effect |
|---|---|
| `KICAD_SWEEP_NAME` | the Modal app name |
| `KICAD_SWEEP_DIRTY=1` | ship the working tree; rows are stamped `<sha>+dirty` |
| `KICAD_SWEEP_BIG_BOARDS=a,b` | put these boards on the 8 GB tier |
| `KICAD_SWEEP_WITH_KICAD=1` | build the KiCad image (see "The image") |
| `KICAD_SWEEP_KICAD_IMAGE` | which KiCad image (default `kicad/kicad:10.0.6`) |
| `KICAD_SWEEP_BUILD_TAG` | pin the release the prebuilt router comes from (for an image at an old engine commit) |

## Results volume and resume

Every task writes its row to the `kicad-sweep-results` volume as
`<arm>/<set>__<board>.json` (a broken chain also leaves the whole replay log as
`<set>__<board>.log`). A relaunch **skips every (arm, board) that already has a
row** and merges the banked rows into its result, so a stopped sweep resumes for
free — and an arm name reused at a different commit resumes the OLD rows, which is
why `cloud_replay_sets.py` puts the commit in its arm names. Pull an arm down with
`modal volume get kicad-sweep-results /<arm> <dir>`.

The corpus lives on the `kicad-corpus` volume, staged by `upload_corpus.py`:
manifests, the boards they read, and `boards_<set>/` + `boards_unrouted_<set>/`
— not prior outputs or transcripts. A board edited locally after its set went up
is only refreshed by re-uploading it (`upload_corpus.py --sets S --boards b`);
`cloud_replay_sets.py`'s upload stage does that check for you.

## The image

- **Source**: a clean `git archive HEAD` by default, so a sweep is reproducible
  and you can keep editing while it runs; `KICAD_SWEEP_DIRTY=1` ships the working
  tree instead.
- **Numeric stack pinned exactly** (`numpy`/`scipy`/`shapely`, `_PY_PINS`): an
  unpinned rebuild once routed 28 of 96 boards differently at identical config.
  Every row records the versions it ran (`deps`). Bump only deliberately,
  expecting a new baseline era.
- **Router**: `build_router.py` keeps the release's prebuilt Linux binary when it
  matches `rust_router/Cargo.toml`, retried three times against flaky downloads;
  when the crate is ahead of every release it builds from source (rustup is in
  the image — a ~10 min cold build, then cached). `KICAD_SWEEP_BUILD_TAG` pins
  the release for an image built at an old engine commit (`corpus_bisect.sh`
  sets it). The build is proved by importing `grid_router` in a fresh
  interpreter before any task runs.
- **KiCad is opt-in here** (`KICAD_SWEEP_WITH_KICAD=1`, which
  `cloud_replay_sets.py` sets by default). The image is then
  `KICAD_SWEEP_KICAD_IMAGE` (default `kicad/kicad:10.0.6`) on its own system
  python, with the same pins, and the build fails unless both `import pcbnew` and
  `kicad-cli version` work. Without KiCad every oracle leg of the route step is
  **dead, not degraded** (`oracle_reconnect` reports `available=False`), so a
  change acting only through the finalize audit or the oracle recheck A/Bs as a
  perfect null, and `ab_replay_grade`'s `kicad-cli` cross-check is skipped, so
  `drc_real` falls back to raw check_drc minus accepted edge items. Plain
  `modal_app.py` stays without KiCad because a different base image is a new
  baseline era for every comparison made on the old one.
- **Building new corpus sets** is out of scope: it needs `pcbnew` for prep, a
  local one-time step unrelated to sweeping.

## Screen before you sweep

100 arms x 150 boards is ~864 CPU-hours. Most arms can be judged far cheaper:

```
prescreen (2 boards, local, $0)  ->  screen (12 boards)  ->  full corpus  ->  holdout (untouched sets)
```

- **`prescreen.py`** runs ONE knob at several values on two challenging-but-fast
  boards on this machine and prints verdict / wall per value. It catches
  blow-ups (quality collapse, wall explosion), not mild damage:

  ```bash
  python3 tests/stress/modal_sweep/prescreen.py HEURISTIC_WEIGHT 1.0 1.5 2.5 3.0
  python3 tests/stress/modal_sweep/prescreen.py --env KICAD_PROXIMITY_SUM 0 1 softcap
  ```

- **`--stage screened`** runs every arm on `board_value.json`'s 12-board
  `screen` set first, then KILLS each arm whose summed `nets_incomplete` on it
  exceeds the control arm's by more than `--gate-margin` (fractional, at least
  +3) before the full set. The gate only saves money; final rankings still come
  from the full set.
- **`curate_boards.py <results snapshot>`** regenerates `board_value.json` from a
  downloaded results-volume snapshot: boards scored by how much they
  discriminate between arms per CPU-hour, the cheapest discriminating ones as
  the `screen`, and boards where every arm ties as `exclude`. Regenerate it after
  a big sweep. The exclude list is fitted to the arm family it was measured on,
  so a new family's first sweep passes `--ignore-excludes`.
