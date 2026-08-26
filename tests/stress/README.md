# Router stress-test harness

Stress-tests the router against real-world open-source KiCad boards of
varying complexity, measuring routing completion rates and DRC violations.
The boards are NOT checked into the repo — they are downloaded from their
upstream GitHub projects each time.

All artifacts live outside the repo in `$STRESS_DIR`
(default: `~/Documents/kicad_stress_test`).

## Pipeline

```bash
# 1. Download .kicad_pcb files from the curated repo list (needs `gh` auth)
python3 fetch_boards.py                      # -> $STRESS_DIR/sources/github_set1/

# 2. Normalize to current KiCad format via pcbnew round-trip
#    (uses KiCad's bundled Python; rescues old KiCad 4-7 format boards)
KIPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
$KIPY normalize_boards.py                    # -> $STRESS_DIR/boards_set1/

# 3. Strip routing -> unrouted test corpus
#    Removes tracks/vias/pour zones (keeps rule areas), regenerates Edge.Cuts
#    as plain chained lines, drops non-copper board graphics, and retypes all
#    copper layers as 'signal'. The last three steps work around known
#    kicad_parser limitations (see "Parser workarounds" below).
#    NOTE: run one board per process (pcbnew segfaults on multi-board runs):
for f in $STRESS_DIR/boards_set1/*.kicad_pcb; do
  $KIPY strip_routing.py "$(basename ${f%.kicad_pcb})"
done                                         # -> $STRESS_DIR/boards_unrouted_set1/

# 4. Sanity-check the corpus parses with the repo parser
python3 validate_boards.py
```

## Running the stress test

Drive the whole corpus with the queue manager:

```bash
bash run_queue.sh [max_concurrency=8] [model=sonnet]   # from tests/stress/
bash stress_status.sh                              # monitor: DONE/RUNNING/TODO + free slots
```

`run_queue.sh` keeps N headless agent workers (`claude -p` by default; `STRESS_AI_BACKEND=opencode` switches to `opencode run`, #503 - model arg is then `provider/model`) in flight until every board
has a results JSON, deriving all state from disk (safe to stop and restart — it
skips finished boards and won't double-launch running ones). Each worker
(`run_board.sh <board> <set> [model]`) routes one board per `RUNBOOK.md`
(analysis -> fanout -> diff pairs -> signal routing -> power planes -> plane
repair -> DRC/connectivity/orphan verification -> compare-to-original), writing
`$STRESS_DIR/results_set<N>/<board>.json` (schema in the runbook) plus a
`FINDINGS.md`, and an `agent_narrative.md` routing decision trail derived from the
captured `transcript.jsonl` by `extract_narrative.py`. The full mechanism, fresh-machine prereqs (including the one-time
permission authorization the headless workers need), and a manual fallback are
in `RUNBOOK.md` → "Orchestration".

Operational limits (baked into the scripts / learned the hard way):

- Every routing/fanout/plane/check command is wrapped in `run_limited.sh`
  (kills the job at ~4 GB RSS, overridable with `LIMIT_KB`). It also **records**
  each command to a replay manifest — see "Deterministic replay" below.
- 4 boards run concurrently — most jobs stay well under the 4 GB cap, so
  4-in-flight works on an 8 GB machine and the per-job watchdog backstops any
  spike. (Lower the concurrency arg if you see swapping.)
- Each worker runs its routing commands in the foreground under a hard
  3-hour/command cap and ~3.5-hour board budget (RUNBOOK rule 12). The queue
  manager and `stress_status.sh` track liveness from disk (results JSON +
  run-dir activity), so dropped/stale notifications can't mislead them.

## Deterministic replay (redo without an LLM) — issue #132

The LLM agent routes each board once, with judgment. That run is slow (tens of
minutes, mostly API latency), can die mid-pipeline on a transient API error, and
picks parameters afresh each time — so two runs of the same board aren't
identical, which makes it impossible to cleanly A/B an engine change.

To fix this, each board-mutating tool (`route.py`, `route_diff.py`,
`route_planes.py`, `repair_planes.py`, `bga_fanout.py`) **self-records**
its invocation (fully quoted argv + cwd) to a manifest via
`redo_record.record_invocation()` at the top of `main()`. Recording is gated on
the `REDO_MANIFEST` env var (a no-op when unset); `run_board.sh` sets it to
`<run-dir>/redo_commands.sh` for every board run. Self-recording is reliable even
when a command isn't routed through `run_limited.sh` — which the agent does
inconsistently — so the manifest is a complete, replayable transcript of the run.

`redo_stress_test.py` replays a manifest — no LLM, no API calls,
seconds-to-minutes instead of tens of minutes, and immune to API outages. The
router is deterministic, so a replay reproduces the recorded board exactly (only
the freshly-assigned track/via UUIDs differ; geometry and nets are identical).

By default the replay is **pruned to the file-dependency chain** that produces
the final board (issue #231): the agent's superseded retries (a route command it
ran 5× — only the last feeds downstream) and dead-end branches (an output nothing
consumes) are skipped. Each kept command re-reads its own input board, so dropping
the overwritten writes can't change the result — the final board is identical, just
reached in far fewer steps (e.g. ottercast 24 → 11 commands, same DRC). Pass
`--verbatim` to replay every recorded command literally.

```bash
# Replay a recorded run in place (re-runs the exact recorded sequence):
python3 tests/stress/redo_stress_test.py <run-dir>/redo_commands.sh

# A/B an engine change: replay the SAME manifest into two fresh dirs with
# --remap (rewrites the original run-dir path prefix so absolute intermediate
# paths land in the new dir; the source board's own absolute path still resolves):
python3 tests/stress/redo_stress_test.py <run-dir>/redo_commands.sh \
    --remap /…/runs_set1/<board>:/tmp/redo_baseline      # with your change reverted
python3 tests/stress/redo_stress_test.py <run-dir>/redo_commands.sh \
    --remap /…/runs_set1/<board>:/tmp/redo_change         # with your change applied
# then diff the two final boards (ignore uuid lines) / re-grade DRC + connectivity.
```

Flags: `--skip-checks` (omit the non-mutating `check_*` commands for speed),
`--dry-run` (print the plan), `--continue-on-error` (push past a failing command
instead of stopping — the recorded sequence already contains the agent's retries,
so a recorded failure is normal and is followed by its successful retry).

Recording is opt-in: set `REDO_MANIFEST=<path>` to capture a manual run (only the
board-mutating tools record; re-run `check_*` separately for grading), or leave it
unset / `=/dev/null` to disable.

### Whole-set A/B (`ab_replay_grade.py`)

`ab_replay_grade.py` is the set-level layer on top of `redo_stress_test.py`: it
replays every board in a set in parallel, auto-detects each board's final board
and the route step's actual `--clearance`, grades DRC + connectivity, and writes
a `summary.json`. Run it once per code version ("wave"), then `--compare` the two
summaries for a per-board regression table (chains broken in either wave are
excluded). DRC is graded at each board's routed clearance, not a guessed one.

**Connectivity is graded on total incomplete nets** (`incompl` column =
unrouted + connectivity-issue nets), *not* the raw connectivity-issue count: a
net that loses its copper entirely leaves the connectivity-issue bucket for the
unrouted bucket, so the issue count can *drop* while the board got worse. The
per-board flag and the net verdict both key off this incomplete-net delta.

```bash
# the engine change is uncommitted in the working tree:
git stash push file1.py file2.py            # baseline = HEAD
python3 tests/stress/ab_replay_grade.py --set ~/…/runs_set3 --out ab/old --label old
git stash pop                               # candidate = HEAD + change
python3 tests/stress/ab_replay_grade.py --set ~/…/runs_set3 --out ab/new --label new
python3 tests/stress/ab_replay_grade.py --compare ab/old/summary.json ab/new/summary.json
```

The two waves must be sequential (they share the live repo's git state), but the
boards within a wave run in parallel (`--jobs`, default 4).

**Provenance — every output dir records its checkout.** Each wave dir gets a
`git_version.txt` (+ `git_version.json`) naming the exact commit that produced it
(`describe --tags --dirty`, full hash, branch, subject, capture time), so a wave
is never ambiguous after the fact. `--compare` reads both waves' `git_version.json`
and prints an `old = … new = …` header. `redo_stress_test.py` writes the same
file into its `--workdir` (a single-board replay), and the live stress worker
`run_board.sh` writes one into each board's run dir. A `-dirty` describe means the
tree had uncommitted changes when the wave ran.

**Grade at the routed clearance.** Each board is graded at the **minimum**
`--clearance` across its manifest's routing steps — the copper was built to that
tightest value, and a looser retry/fanout step's copper still satisfies it.
Grading at a *looser* clearance invents hundreds of phantom violations (e.g.
tigard routes at 0.1 but has one 0.15 signal retry; graded at 0.15 it shows ~640
"violations", at its real 0.1 it is clean). The A/B *delta* is unaffected (same
clearance both waves), but absolute counts are only meaningful at the routed
clearance, so trust the min that `route_clearance()` picks (or pass the board's
real clearance when grading a single board with `check_drc.py -c`).

To re-grade an existing wave's boards in place after a grading fix (no
re-routing), or to reuse a prior wave as a baseline:
```bash
python3 tests/stress/ab_replay_grade.py --regrade ab/old --set ~/…/runs_set3
```

### Multi-set waves & release sign-off (`ab_wave_driver.py`, `ab_wave_report.py`)

`ab_replay_grade.py` handles **one set**. Deciding whether a HEAD should become a
release tag spans the whole corpus against more than one baseline:

```bash
# 1. candidate wave, sets 1-11, 5 boards in flight AT ALL TIMES (across sets)
nohup caffeinate -s python3 -u tests/stress/ab_wave_driver.py wave \
    --out ~/…/ab_main_0728a --label head --jobs 5 \
    --cost-baseline ~/…/ab_main_0726a > ~/wave.log 2>&1 &

# 2. re-grade each baseline with TODAY's grader (back up summary.json first!)
python3 tests/stress/ab_wave_driver.py regrade --out ~/…/ab_main_0726a --jobs 5

# 3. roll it all up
python3 tests/stress/ab_wave_report.py --new ~/…/ab_main_0728a \
    --base 0726a=~/…/ab_main_0726a --base dp250=~/…/ab_dp250
```

`ab_wave_driver.py` runs one **global queue** over every set (so a set's slow tail
no longer drains the pool to idle before the next set starts), in plain corpus
order by default — by set, then board name — which keeps runs reproducible and
comparable board-for-board. `--order lpt` opts into longest-job-first for a
shorter makespan at the cost of a data-dependent order, and `--heavy-slots 1`
opts into memory admission on a small-RAM box (off by default, since it reorders).
It calls `ab_replay_grade.do_board`, so grading and the `summary.json` schema are
identical.
`ab_wave_report.py` aggregates every set against each baseline arm on
`drc_real` / `nets_incomplete` / `kicad_connection_width` / `diff_pairs_coupled`,
ranks per-board regressions, and flags broken chains separately (a release blocker
that can never appear as a DRC delta).

**`diff_pairs_coupled` is a TRUNK measure, not a pad-connectivity one** (#602).
A pair whose coupled trunk routed but whose terminals were peeled to the
single-ended follow-up counts as coupled by design — its trunk *is* coupled and
the follow-up closes the terminals — so the coupled count alone never tells you
a pair's member pads are open. Gate on **`diff_pairs_member_incomplete`** for
that: it is route_diff's own member audit, carried through as
`member_incomplete_pairs` in `JSON_SUMMARY`, and it names every pair with a
disconnected member pad whatever bucket the pair landed in. `diff_pairs_partial`
counts the pairs route_diff demoted out of `coupled` because they claimed a
finished coupled route the audit contradicted (`outcome: "incomplete"`).

Both are documented in depth in `RUNBOOK.md` → "Multi-set waves & release
sign-off", including the run-for-hours checklist: detach with `nohup`, wrap in
`caffeinate -s`, attach a monitor that greps failures as well as progress, freeze
the working tree for the whole wave (manifests bake tool paths absolutely), and
**re-grade baselines rather than diffing stored numbers** — graders here drift
within days.

### Per-command timing (#132)

Both the original run and a replay record per-command wall-clock so routing
performance can be compared across code versions:

- **Original run** — alongside `redo_commands.sh`, `record_invocation()` writes a
  sibling `redo_timings.jsonl` (one JSON line per command: `seconds`, `cwd`,
  `argv`, appended on process exit via `atexit`). The manifest itself stays a
  clean, replayable script.
- **Replay** — `redo_stress_test.py` times each command, prints a slowest-first
  breakdown, and with `--timings-out PATH` writes the per-command timings as JSON.
  Timing the deterministic replay (not the LLM run, whose wall time is interleaved
  with model thinking) is the apples-to-apples measurement.

The two files together also **identify attempts that DIED** (#599): the manifest
line is written when a command *starts*, but the timings record only on a clean
exit, so a manifest command with no matching `redo_timings.jsonl` entry was
killed part-way — by the caller's command timeout, the memory watchdog, or a
crash. That is the cheapest way to tell a wave's wasted attempts from its real
work. Such an attempt does not corrupt a replay: it wrote no output, and
`minimize_manifest.py`'s backward slice binds each read to the file's *current*
producer, so the successful rerun's write supersedes it and the dead line drops
out. Check `ORPHANED_STEP.txt` / `KILLED_STEP.txt` in the run dir for the
wrapper's own account of what killed it.

### Minimizing a manifest (#132)

A recorded manifest contains the agent's trial-and-error — `--help` probes,
retries that overwrite an output with different parameters, dead-end attempts
whose output is never consumed. `minimize_manifest.py` reduces it to the minimal
set of commands that still reproduces the final board, via a data-flow backward
slice (each read binds to the file's current producer, so superseded writes drop
out). It is **read-only on the input** — writes only to `-o` or stdout, and
refuses `-o` equal to the input, so a recorded artifact is never clobbered:

```bash
# Emit a minimal replay alongside the recorded one (never overwrites it):
python3 tests/stress/minimize_manifest.py <run-dir>/redo_commands.sh \
    -o <run-dir>/redo_commands.min.sh
python3 tests/stress/redo_stress_test.py <run-dir>/redo_commands.min.sh \
    --timings-out <run-dir>/redo_timings.replay.json
```

## Routing-constraint validation (what params to route with)

`measure_routing.py <routed.kicad_pcb>...` reports the geometry actually used
in a board's real (downloaded) routing — track widths, via sizes, per-class
usage — next to its net-class definitions. Validated findings:

- **Two tiers of rules** (`list_nets --design-rules`, issues #111/#115): KiCad
  net-class `track_width`/`via_diameter` are *drawing defaults*, NOT DRC minima —
  only the Board Constraints (`design_settings.rules`) are DRC-enforced. The tool
  reads both and combines them with the JLCPCB fab minimum (layer-aware) into a
  **manufacturing floor**.
- **Via**: emit the small **working** via the floor prints
  (`--via-size`/`--via-drill` = `max(min_via_diameter, fab min)`), NOT the
  net-class `via_diameter` — that nominal is a max-like default, too big for
  fine-pitch escape (butterstick 0.8 vs the original's 0.45; lily58/crkbd QFN
  escapes need ~0.45–0.6, unroutable at 0.8).
- **Clearance**: route at the net-class clearance; a fine-pitch escape may drop
  toward the manufacturing floor (never below). Route non-Default-class nets
  separately with their own clearance.
- **Track width**: the netclass `track_width` is a *minimum*. Real boards route
  signals at/above it and widen power/high-current nets to several distinct
  widths (2–4 mm power buses are common). One uniform `--track-width` under-builds
  power nets; widen them explicitly via `--power-nets`.
- **Net classes**: `list_nets --design-rules` reads all classes plus the Board
  Constraints (from the `.kicad_pro` for KiCad 8+, or `(net_class)` blocks for
  KiCad 6/7), and falls back to the JLC fab floor for the board's layer count when
  neither is present. On the test corpus most boards have only Default.
- **DRC at the manufacturing floor**, NOT the net-class clearance and NOT a
  hardcoded 0.25: `check_drc --clearance <floor> --hole-to-hole-clearance <floor>`.
  Escapes are routed down to the floor, so checking there stops them reading as
  violations (the dominant set-1/set-2 DRC source) while still flagging anything
  genuinely sub-manufacturable. `min_clearance` is deliberately not used as the
  floor — it is an unreliable edit-floor (often 0, sometimes a stale large value).
- **DRC baseline** for judging our routing is the **original routed board's**
  violation count *at the same floor* (a few clearance-independent hole/pad
  violations remain, e.g. megadesk 1, piantor 6), not 0.

## Compare-to-original (final step of every board)

`compare_to_original.py --ours <our_final.kicad_pcb> --orig <boards_setN/<board>.kicad_pcb>
--json` contrasts our routing with the human-routed original (via count, total
copper length, distinct track widths, layer balance, nets-with-copper) and emits
SUGGESTIONS for what to change in our routing or approach. The original is ground
truth for a manufacturable board; large gaps are router-improvement findings.
Wired into RUNBOOK step 11b; output goes into the results JSON `comparison` /
`suggestions` fields.

## Board sets

Sets 1–3 are exactly **15 boards** each; set 4 starts smaller and grows:

- **Set 1** — the original 15 (curated in `fetch_boards.py` / `normalize_boards.py`).
  `spirit_cm5` (6-layer) was dropped and replaced by `lpddr4_testbed`
  (antmicro/lpddr4-testbed, 6-layer BGA LPDDR4). Boards live in
  `boards_unrouted_set1/`; results in `results_set1/`; run-1 baseline in `results_baseline/`.
- **Set 2** — 15 newer boards listed in `manifest_set2.json`, downloaded to
  `$STRESS_DIR/sources/github_set2/`. FPGA/BGA (ulx3s, butterstick, cynthion,
  schoko, CPArti), DDR4/DDR5 test beds (antmicro), USB diff-pair boards
  (usb-sniffer, free-dap), QFN/QFP carriers (tinytapeout, caravel, system76,
  nitrokey), simple keyboards (crkbd, sofle, lily58). Boards live in
  `boards_unrouted_set2/`; results kept separate in `results_set2/`.
- **Set 3** — 15 boards listed in `manifest_set3.json`, fetched by
  `fetch_set3.py` to `$STRESS_DIR/sources/github_set3/`. Spans L2→L8 and tiny→huge:
  USB3 (daisho, 392 fp / L8), ECP5+DDR3 (orangecrab L6), iCE40/ECP5 FPGA (upduino,
  fomu, keks, eis), STM32WLE LoRaWAN (lora_v3), ESP32 wearable (watchy), FT2232H
  debug tool (tigard), Allwinner SBC (a20_can), RF LNA (lna3030), Olimex modules
  (esp_prog, olimex_lora, mod_bme280), and an all-through-hole passive tap
  (throwing_star). Deliberately heavy on **old KiCad formats** (v4 / v20171130):
  the text parser can't read pre-v6 directly, so the pcbnew round-trip in
  `prep_set3.sh` upgrades them first — extra coverage of that rescue path. Boards
  live in `boards_unrouted_set3/`; results in `results_set3/`.
- **Set 4** — listed in `manifest_set4.json`, fetched by `fetch_set4.py` to
  `$STRESS_DIR/sources/github_set4/`. Starts as a single board:
  `Rahul9-spb/FPGA-1` (`PCB/FPGA_SDRAM.kicad_pcb`) — an FPGA + SDRAM design
  (BGA/QFN/SOIC/SOT, 4-layer, KiCad 10; 126 footprints / 305 nets). Append more
  entries to the manifest (and a `MAP` line in `prep_set4.sh`) to grow it. Boards
  live in `boards_unrouted_set4/`; results in `results_set4/`.

Prepare set 2 / set 3 / set 4 (KiCad Python; loads each board once):

```bash
bash prep_set2.sh    # -> boards_set2/ (normalized routed + .kicad_pro)
                     #    boards_unrouted_set2/ (stripped, to route)

python3 fetch_set3.py && bash prep_set3.sh   # set 3: fetch + normalize + strip
python3 fetch_set4.py && bash prep_set4.sh   # set 4: fetch + normalize + strip
```

`prep_set2.py <src> <routed_dst> <stripped_dst>` normalizes the routed reference
and strips routing in a single load. The `lpddr4_testbed` replacement is a SET-1
board: prep it the same way into `boards_set1/` + `boards_unrouted_set1/`. ulx3s has no net
classes anywhere, so derive its params via `measure_routing.py` on its routed
reference.

## Parser workarounds baked into strip_routing.py

These mask real kicad_parser issues found during corpus preparation; the
corpus works around them so routing results aren't confounded:

1. `power`-type copper layers are dropped from `copper_layers`
   (kicad_parser.py only accepts `signal`) — boards commonly type their
   plane layers `power`.
2. Edge.Cuts regexes cross-match through other `gr_*` elements (lazy
   `.*?(layer "Edge.Cuts")`), corrupting outline and bounds when silk/fab
   board graphics precede edge lines in the file.
3. KiCad 6/7 files storing the reference as `(fp_text reference ...)` parse
   with all footprints collapsed onto one dict key (normalization to
   KiCad 10 format avoids it).
4. Mixed net-reference styles: `bga_fanout`/`qfn_fanout` write numeric
   `(net N)` refs into KiCad 10 boards (no `net_id_to_name` passed to
   `add_tracks_and_vias_to_pcb`), and `extract_segments()`/`extract_vias()`
   use an either/or regex fallback that silently drops ALL name-style
   elements when any numeric ones exist — blinding every downstream tool.
   Workaround: `fix_mixed_net_refs.py` is run on every fanout output
   (see RUNBOOK rule 5).
5. Board-edge obstacle memory blowup: `_add_polygon_edge_obstacles`
   (obstacle_map.py) allocates O(grid_cells × outline_vertices) float64
   broadcast arrays — a 432-vertex keyboard outline on a 222×90 mm board
   wants ~7 GB and OOMs the machine (route.py and
   repair_planes.py affected; route_planes.py has a frugal
   board-edge path and is fine). Workaround: strip_routing.py
   Douglas-Peucker-simplifies regenerated outlines to ≤0.025 mm tolerance,
   keeping vertex counts low. Fix candidate: chunk the broadcast, or port
   route_planes.py's implementation.
