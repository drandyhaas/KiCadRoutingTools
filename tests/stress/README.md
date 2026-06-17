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
python3 fetch_boards.py                      # -> $STRESS_DIR/sources/github/

# 2. Normalize to current KiCad format via pcbnew round-trip
#    (uses KiCad's bundled Python; rescues old KiCad 4-7 format boards)
KIPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
$KIPY normalize_boards.py                    # -> $STRESS_DIR/boards/

# 3. Strip routing -> unrouted test corpus
#    Removes tracks/vias/pour zones (keeps rule areas), regenerates Edge.Cuts
#    as plain chained lines, drops non-copper board graphics, and retypes all
#    copper layers as 'signal'. The last three steps work around known
#    kicad_parser limitations (see "Parser workarounds" below).
#    NOTE: run one board per process (pcbnew segfaults on multi-board runs):
for f in $STRESS_DIR/boards/*.kicad_pcb; do
  $KIPY strip_routing.py "$(basename ${f%.kicad_pcb})"
done                                         # -> $STRESS_DIR/boards_unrouted/

# 4. Sanity-check the corpus parses with the repo parser
python3 validate_boards.py
```

## Running the stress test

Drive the whole corpus with the queue manager:

```bash
bash run_queue.sh [concurrency=4] [model=sonnet]   # from tests/stress/
bash stress_status.sh                              # monitor: DONE/RUNNING/TODO + free slots
```

`run_queue.sh` keeps N headless `claude -p` workers in flight until every board
has a results JSON, deriving all state from disk (safe to stop and restart — it
skips finished boards and won't double-launch running ones). Each worker
(`run_board.sh <board> <set> [model]`) routes one board per `RUNBOOK.md`
(analysis -> fanout -> diff pairs -> signal routing -> power planes -> plane
repair -> DRC/connectivity/orphan verification -> compare-to-original), writing
`$STRESS_DIR/results[_set2]/<board>.json` (schema in the runbook) plus a
`FINDINGS.md`. The full mechanism, fresh-machine prereqs (including the one-time
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
  20-min/command cap and ~45-min board budget (RUNBOOK rule 12). The queue
  manager and `stress_status.sh` track liveness from disk (results JSON +
  run-dir activity), so dropped/stale notifications can't mislead them.

## Deterministic replay (redo without an LLM) — issue #132

The LLM agent routes each board once, with judgment. That run is slow (tens of
minutes, mostly API latency), can die mid-pipeline on a transient API error, and
picks parameters afresh each time — so two runs of the same board aren't
identical, which makes it impossible to cleanly A/B an engine change.

To fix this, each board-mutating tool (`route.py`, `route_diff.py`,
`route_planes.py`, `route_disconnected_planes.py`, `bga_fanout.py`) **self-records**
its invocation (fully quoted argv + cwd) to a manifest via
`redo_record.record_invocation()` at the top of `main()`. Recording is gated on
the `REDO_MANIFEST` env var (a no-op when unset); `run_board.sh` sets it to
`<run-dir>/redo_commands.sh` for every board run. Self-recording is reliable even
when a command isn't routed through `run_limited.sh` — which the agent does
inconsistently — so the manifest is a complete, replayable transcript of the run.

`redo_stress_test.py` replays a manifest verbatim — no LLM, no API calls,
seconds-to-minutes instead of tens of minutes, and immune to API outages. The
router is deterministic, so a replay reproduces the recorded board exactly (only
the freshly-assigned track/via UUIDs differ; geometry and nets are identical).

```bash
# Replay a recorded run in place (re-runs the exact recorded sequence):
python3 tests/stress/redo_stress_test.py <run-dir>/redo_commands.sh

# A/B an engine change: replay the SAME manifest into two fresh dirs with
# --remap (rewrites the original run-dir path prefix so absolute intermediate
# paths land in the new dir; the source board's own absolute path still resolves):
python3 tests/stress/redo_stress_test.py <run-dir>/redo_commands.sh \
    --remap /…/runs/<board>:/tmp/redo_baseline      # with your change reverted
python3 tests/stress/redo_stress_test.py <run-dir>/redo_commands.sh \
    --remap /…/runs/<board>:/tmp/redo_change         # with your change applied
# then diff the two final boards (ignore uuid lines) / re-grade DRC + connectivity.
```

Flags: `--skip-checks` (omit the non-mutating `check_*` commands for speed),
`--dry-run` (print the plan), `--continue-on-error` (push past a failing command
instead of stopping — the recorded sequence already contains the agent's retries,
so a recorded failure is normal and is followed by its successful retry).

Recording is opt-in: set `REDO_MANIFEST=<path>` to capture a manual run (only the
board-mutating tools record; re-run `check_*` separately for grading), or leave it
unset / `=/dev/null` to disable.

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

`compare_to_original.py --ours <our_final.kicad_pcb> --orig <boards/<board>.kicad_pcb>
--json` contrasts our routing with the human-routed original (via count, total
copper length, distinct track widths, layer balance, nets-with-copper) and emits
SUGGESTIONS for what to change in our routing or approach. The original is ground
truth for a manufacturable board; large gaps are router-improvement findings.
Wired into RUNBOOK step 11b; output goes into the results JSON `comparison` /
`suggestions` fields.

## Two 15-board sets

Both corpora are exactly **15 boards**:

- **Set 1** — the original 15 (curated in `fetch_boards.py` / `normalize_boards.py`).
  `spirit_cm5` (6-layer) was dropped and replaced by `lpddr4_testbed`
  (antmicro/lpddr4-testbed, 6-layer BGA LPDDR4). Boards live in
  `boards_unrouted/`; results in `results/`; run-1 baseline in `results_baseline/`.
- **Set 2** — 15 newer boards listed in `manifest_set2.json`, downloaded to
  `$STRESS_DIR/sources/github_set2/`. FPGA/BGA (ulx3s, butterstick, cynthion,
  schoko, CPArti), DDR4/DDR5 test beds (antmicro), USB diff-pair boards
  (usb-sniffer, free-dap), QFN/QFP carriers (tinytapeout, caravel, system76,
  nitrokey), simple keyboards (crkbd, sofle, lily58). Boards live in
  `boards_unrouted_set2/`; results kept separate in `results_set2/`.

Prepare set 2 (KiCad Python; loads each board once):

```bash
bash prep_set2.sh    # -> boards_set2/ (normalized routed + .kicad_pro)
                     #    boards_unrouted_set2/ (stripped, to route)
```

`prep_set2.py <src> <routed_dst> <stripped_dst>` normalizes the routed reference
and strips routing in a single load. The `lpddr4_testbed` replacement is a SET-1
board: prep it the same way into `boards/` + `boards_unrouted/`. ulx3s has no net
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
   route_disconnected_planes.py affected; route_planes.py has a frugal
   board-edge path and is fine). Workaround: strip_routing.py
   Douglas-Peucker-simplifies regenerated outlines to ≤0.025 mm tolerance,
   keeping vertex counts low. Fix candidate: chunk the broadcast, or port
   route_planes.py's implementation.
