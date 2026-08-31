# Router Stress-Test Runbook (per board)

You are stress-testing the KiCadRoutingTools autorouter on one real-world
open-source board. Follow the plan-pcb-routing skill methodology
non-interactively and record everything.

## Orchestration — drive the whole corpus with run_queue.sh

The whole set-1 + set-2 corpus is driven by ONE queue manager:

```bash
bash tests/stress/run_queue.sh [max_concurrency=8] [model=sonnet]
```

It keeps N headless `claude -p` board workers in flight until every board has a
results JSON, deriving ALL state from disk — so it's safe to Ctrl-C and restart
(it skips finished boards and won't double-launch running ones). Each worker is
`tests/stress/run_board.sh <board> <set> [model]`: a non-interactive agent that
follows THIS runbook, writes the results JSON + `FINDINGS.md` into the run dir,
captures `transcript.jsonl` and derives `agent_narrative.md` (a compact routing
decision trail, via `extract_narrative.py`), and drops a `.worker_done` marker
(`ok`/`NORESULT`). This replaces the older
manual approach (a parent launching one background `Agent` per board and
refilling on notifications) — the queue manager removes the drop-/stale-prone
notification stream entirely.

**Prereqs on a fresh machine:**
- Build the corpus first (README → "Pipeline" and "Two 15-board sets").
- Workers run `claude -p --dangerously-skip-permissions` (or, with
  `STRESS_AI_BACKEND=opencode`, `opencode run --auto` with any
  `provider/model` opencode supports, #503), which the harness
  blocks by default. Authorize it once: add a Bash allow-rule for
  `bash tests/stress/run_board.sh:*` and `bash tests/stress/run_queue.sh:*`
  to your local `.claude/settings.local.json` (gitignored, so it is NOT
  inherited from a checkout — each user opts in themselves), or approve when
  prompted.

**Monitoring:** `bash tests/stress/stress_status.sh` — prints DONE/RUNNING/TODO
across all 30 boards plus free slots. (For detail: `QUEUE_STATUS.txt` is the
manager heartbeat; `runs_set<N>/<board>/worker.log` holds the wrapper markers +
stderr, the per-tool `*.log` files update live, and after the run
`agent_narrative.md` is the readable routing decision trail derived from
`transcript.jsonl`.)

**State signals** (what the queue and `stress_status.sh` use):
- DONE: results JSON exists.
- RUNNING: a process matches the run-dir path, OR the run dir was touched <3 h
  ago (covers the gap between a worker's commands AND a long single signal-route
  step on a big board that writes no intermediate files). Naive checks mislead — pgrep
  on tool names is noisy and case-sensitive (KiCad's interpreter is `Python`,
  wrapped by run_limited.sh), and run-dir mtimes go quiet during a long route —
  so don't rely on those alone.
- LOST: no results JSON AND not running AND run dir idle ~15+ min → safe to relaunch.

### Manual fallback (driving by hand, no queue script)

If you must drive without `run_queue.sh` (e.g. orchestrating from a chat
session), launch one background board worker/agent at a time and keep **10 in
flight** (workers are LLM-latency-bound, not CPU-bound — see the concurrency
note in `run_queue.sh`), refilling off `stress_status.sh` — NOT the notification
stream, which drops and duplicates. Interleave heavy and light boards so several
route steps don't coincide (heavy = dense 4-layer / many pads / BGA fanout);
`run_limited.sh`'s per-job RAM cap is the backstop. Derive
state from disk, never from tracked agent IDs (they are lost on context
summarization).

### Clean restart (re-run the whole corpus from scratch)

Don't delete prior results — archive them. Move `results_set1/`, `results_set2/`,
`runs_set1/`, `runs_set2/` to `*_archive_<timestamp>/`, then recreate them as
empty dirs. The originals (`boards_set1/`, `boards_set2/`) and the stripped
inputs (`boards_unrouted_set1/`, `boards_unrouted_set2/`) are never touched. Also
sweep stray top-level `runs_*T0*` temp files. After that, the status script
reports 0/30 DONE and the recipe above drives the rest.

## Paths

Two corpora share this harness; pick the SET's paths and stay consistent
within a board. `<SET>` below is `_set<N>` (e.g. `_set1` for set 1, `_set2` for set 2).


**`<TOOLS_REPO>` below means the tools-repo directory whose ABSOLUTE path your
prompt already gives you** (the same path in "Run tools as: `python3 -u -X utf8
<TOOLS_REPO>/<tool>.py`" and in `--add-dir`). Substitute that absolute path
yourself wherever you see `<TOOLS_REPO>`; never type the literal string
`<TOOLS_REPO>` into a shell, and do not expect it to be set as an environment
variable.

- Tools repo (READ-ONLY — never write/modify anything here):
  `<TOOLS_REPO>`
- Skill to follow:
  `<TOOLS_REPO>/.claude/skills/plan-pcb-routing/SKILL.md`
- Input board: `~/Documents/kicad_stress_test/boards_unrouted<SET>/<BOARD>.kicad_pcb`
- Your working dir (create it): `~/Documents/kicad_stress_test/runs<SET>/<BOARD>/`
  ALL outputs, intermediates, and logs go here (NOT /tmp — parallel runs collide).
- Original (compare + ground-truth DRC): `~/Documents/kicad_stress_test/boards<SET>/<BOARD>.kicad_pcb`
- Final results JSON: `~/Documents/kicad_stress_test/results<SET>/<BOARD>.json`

  (`<SET>` is always `_set<N>`, e.g. set 1 → `boards_unrouted_set1/`, `runs_set1/`,
   `boards_set1/`, `results_set1/`; set 2 → `boards_unrouted_set2/`, `runs_set2/`,
   `boards_set2/`, `results_set2/`)

## Run artifacts: final snapshot + routing movie (#482)

At the end of each board's run the harness renders, into the run dir, via the
fast geometry renderer (`tests/stress/render_run.py` → `route_render` +
`animate_route`; no KiCad / kicad-cli / browser, ~2 s/board — this replaced the
old kicad-cli + headless-Chrome `board_layer_images` path):

- `<final-board>.png` — combined all-copper snapshot of the final board.
- `<final-board>_<layer>.png` — one snapshot per copper layer (plane vs signal
  at a glance; plane pours render natively).
- `<run-dir>/routing.mp4` — a movie of the WHOLE run: each chain board's copper
  delta is revealed in chain order (so fanout, diff pairs, planes, signal, and
  repair all appear), with fine per-copper rip/restore animation spliced in for
  any step that recorded a trace. New copper flashes white, reroutes/restores
  green, rips flash red. Written as **H.264 `.mp4`** (≈10-50× smaller than GIF,
  plays everywhere) when `imageio-ffmpeg` is installed, else falls back to
  `routing.gif`.

Both the live worker (`run_board.sh`) and the no-LLM replay
(`redo_stress_test.py`) produce these automatically.

**By hand, for any run dir** (same movie, same code — `render_run.py` now calls
`make_movie.py`, which is also what the GUI's "Routing Movie..." button runs):

```bash
python3 py_router/make_movie.py runs_set1/myboard              # -> runs_set1/myboard/routing.mp4
python3 py_router/make_movie.py step1.kicad_pcb step2.kicad_pcb -o out.mp4
```

**For these to be created correctly:**
- The movie discovers the chain boards automatically: it prefers a `stepN` in
  the filename (`step6_route` or `board_step6_route`, sub-steps `step2a/2b`
  ordered correctly); when a chain names outputs semantically instead
  (`diff_groupA`, `planes`, `final_board`), it falls back to **write-time
  order**. An explicit `final*` board is used as the final/substrate, else the
  last in order. (mp4 needs `pip install imageio imageio-ffmpeg`.)
- **Route tracing is ON by default** (`KICAD_ROUTE_TRACE=1`, exported by
  `run_board.sh`, set by `redo_stress_test.py`). Each `route.py`, `route_diff.py`,
  `route_planes.py`, and `repair_planes.py` step then drops a sibling
  `<output>_routetrace.json` recording every segment/via committed, ripped, and
  restored — which the movie splices in for fine animation. Set
  `KICAD_ROUTE_TRACE=0` in the environment to skip tracing (leaner/faster runs;
  the movie falls back to a coarse per-step reveal from the step boards alone).
  Tracing is read-only over routing and never changes the routed result.

## Building a board set (source → validate → prep)

How the `boards_setN/` (routed reference) and `boards_unrouted_setN/` (stripped
input) corpora are created from open-hardware GitHub repos. All tooling lives in
`tests/stress/`; board files live under `~/Documents/kicad_stress_test/` (NOT the repo).

**Pipeline:**
1. **Source + validate candidates.** Find `.kicad_pcb` files in open-hardware repos
   (enumerate a repo's tree: `gh api "repos/OWNER/REPO/git/trees/BRANCH?recursive=1"
   --jq '.tree[].path | select(endswith(".kicad_pcb"))'`; plus `gh search repos`).
   `curl -sL --fail RAWURL -o <slug>.kicad_pcb`, then gate each with
   **`validate_candidate.py <file>`** → one JSON line. Metrics come from **pcbnew**
   (KiCad's C++ loader), NOT the Python parser: pcbnew loads a 62MB/800fp/8L board
   in ~2s where the parser needs 30–100s+ (and hung unbounded on the biggest), and
   "loads in pcbnew" is what prep needs anyway. Reports copper layers / footprints /
   **off-board footprints** / routable_nets / a difficulty **tier**. Keep only
   `pass:true` (2–8 layers, ≥10 footprints, ≥20 routable nets, has an outline with
   ≤max(2,10%) footprints outside it). NB: a full **DRC** run is the WRONG check for
   a *source* board — unrouted, it reports thousands of unconnected-net violations;
   validation is a structural load. Common rejects: KiCad v4/v5 format, git-LFS
   pointer files (tiny), Eagle/Altium repos (no `.kicad_pcb`), off-board footprints.
2. **Curate** a balanced mix (easy/medium/hard) with variety (distinct
   designers/chip families), no duplicates with existing sets. Avoid 8-layer /
   600+-footprint monsters (too slow to route). **Verify each has a board outline**
   (`board_bounds` non-None) — some boards define no Edge.Cuts and are unroutable;
   swap them out.
3. **Manifest.** Each set has `manifest_setN.json` (list of `{repo, path, file,
   raw_url, github_url, layers_est, footprints, tier, packages, short_name, note}`).
   `fetch_setN.py` re-downloads all sources from it into `sources/github_setN/`.
4. **Prep** (needs KiCad's bundled python): `bash prep_setN.sh` runs
   `prep_set2.py <src> <routed_dst> <stripped_dst>` once per board (pcbnew segfaults
   if you batch several in one process). It normalizes the routed board →
   `boards_setN/<name>.kicad_pcb`, strips tracks/vias/zones + rebuilds Edge.Cuts →
   `boards_unrouted_setN/<name>.kicad_pcb`, and copies the sibling `.kicad_pro`.
   Every board needs a `.kicad_pro` (netclass); download it alongside the `.pcb`
   or generate a default.
5. **`assemble_corpus.py`** wires steps 2–4 together from a `curation.json`
   (`{name, set, slug, src, repo, raw_url, ...}`): copies sources into
   `sources/github_setN/`, best-effort fetches each sibling `.kicad_pro`, writes
   `manifest_setN.json`, and regenerates `prep_setN.sh` with its MAP. Idempotent
   (set4's base is only its original non-`short_name` entry).

**Validate the finished set** before routing: every unrouted board must be fully
stripped (0 segments/vias), have `board_bounds`, ≥10 footprints, ≥20 routable nets,
and a `.kicad_pro`. The `swig ... memory leak` lines from pcbnew during prep are
harmless.

## Rules

1. Invoke all tools as `python3 -u -X utf8 <TOOLS_REPO>/<tool>.py ...`
   from your working dir. Tee every command's output to a log file in your run dir.
   **`-u` is mandatory, not cosmetic (#599):** without it stdout is fully
   buffered, so a step killed part-way — by a command timeout, the memory
   watchdog, or a crash — leaves an EMPTY log and the one artifact that would
   explain the kill dies with it (`usmu_smu`, sets-21-27 wave).
   MEMORY CAP (mandatory): prefix EVERY routing/fanout/plane/check command with
   the watchdog wrapper, e.g.
   `bash <TOOLS_REPO>/tests/stress/run_limited.sh python3 -u -X utf8 .../route.py ... 2>&1 | tee step.log`
   It kills the job at ~4 GB RSS (exit 137, `MEMORY_LIMIT_EXCEEDED` on stderr).
   Separately, the board-mutating tools self-record their invocations to
   `<run-dir>/redo_commands.sh` (run_board.sh sets `REDO_MANIFEST`) so the whole
   run can later be replayed deterministically with no LLM via `redo_stress_test.py`
   (issue #132; see `tests/stress/README.md`). Nothing extra to do for recording.
   Up to 4 boards run concurrently — in practice most jobs sit well under the
   4 GB cap most of the time, so 4-in-flight is fine on an 8 GB machine; the
   per-job watchdog still backstops any board that spikes. Keep an eye on RAM.
   If a step is killed by the cap, that is an important finding: record it in
   `issues` (with the step and board), then try ONE cheaper variant (e.g.
   a coarser `--grid-step`, no retry round, or fewer nets); if that also
   blows the cap, mark the step as OOM and move on.
1a. PROJECT FILE TRAVELS WITH THE BOARD (#295): every board-mutating tool
   writes/updates a sibling `.kicad_pro` carrying the routed DRC floors. For a
   board that never had one, `python3 py_router/fix_kicad_drc_settings.py <board>` seeds
   and fills a correct project file.
1a'. NEVER `cp`/`mv`/alias BOARD FILES MID-CHAIN (zynq `final_board` lesson):
   only the recorded tools may create a `.kicad_pcb` in the run dir. A hand
   copy is invisible to `redo_commands.sh`, so it SEVERS the replay's
   file-dependency chain -- the pruned replay then silently seeds the copied
   board from the ORIGINAL run and grades stale copper as if it were current
   code (redo_stress_test now warns "CHAIN HOLE" when this happened). If you
   want a canonical final name, make the LAST TOOL STEP write it as its
   `--output` (e.g. `... final_board.kicad_pcb`); record the final board's
   name in the results JSON. Every tool step's input must be a previous tool
   step's output (or the original seed input), chained by name.
1a''. PROJECT CUSTODY: if you stage/rename the pristine board into the run
   dir at all (e.g. `board0.kicad_pcb` -- prefer NOT to, per 1a'), the
   sibling `.kicad_pro` MUST be copied to the SAME STEM (`board0.kicad_pro`).
   A pro left under the original board name is invisible to every tool step,
   so the design's real DRC rules (min_copper_edge_clearance above all)
   silently vanish for the whole chain and a later step invents defaults --
   the openstint lesson (#338/#404: design 0.3 edge rule lost, 0.5 recorded,
   phantom kicad floods). Best practice: route directly FROM
   `boards_unrouted_setN/<board>.kicad_pcb` as the first step's input; its
   sibling pro then travels automatically (`fix_project_for_output` copies
   input-sibling projects at every step, and the replay seeder now carries
   `<board>.kicad_pro` for legacy stem-mismatched manifests).
1b. PARSER-PARITY VALIDATION (per board, start AND end): run
   `python3 <TOOLS_REPO>/validate_pcb_data.py <board>`
   on the input board before any routing step, and again on the final board.
   It diffs the pcbnew-built PCBData (the GUI's model) against the text parse
   (the CLI's model) — the headless twin of the GUI's "Validate PCB Data"
   button; it re-execs into KiCad's bundled python by itself. A FAIL means one
   parser mis-models the board (custom pads, arc tracks, zones, bounds, ...):
   record the diff lines in the results JSON `issues` array and FINDINGS.md
   (it is a parser bug to file, not a board defect), then continue the run.
2. Follow SKILL.md's analysis steps (board stats, layers, fanout candidates,
   diff pairs via `list_nets.py <board> --diff-pairs --power`, power strategy,
   plan generation) but DO NOT invoke other skills and DO NOT ask the user
   anything — use the skill's inline name-pattern heuristics and your judgment.
   DESIGN RULES: also run `list_nets.py <board> --design-rules` and use the
   flags it prints. It now reports TWO tiers (#111/#115): the net-class defaults
   AND the DRC-enforced Board Constraints (`design_settings.rules`), combined
   with the JLCPCB fab floor into a "manufacturing floor". The router does NOT
   read any of this; its generic 0.25mm default is often wider than the board's
   own rule, which boxes pads in and fails nets with "no rippable blockers".
   - CLEARANCE: route at the Default NETCLASS clearance; a fine-pitch escape may
     drop toward the manufacturing floor (never below). Route non-Default-class
     nets separately with that class's clearance.
   - VIA: use the printed WORKING via (`--via-size`/`--via-drill` from the
     manufacturing floor = the board's `min_via_diameter`/`min_through_hole`,
     floored at the JLC min), NOT the net-class `via_diameter`. The net-class via
     is only a drawing default (a max), far too big for fine-pitch escape — using
     it everywhere was #115 (butterstick 0.8 vs the original's 0.45; lily58/crkbd
     QFN escapes need ~0.45-0.6, unroutable at 0.8).
   - FINE-PITCH ESCAPE VIA (4+ layer): for sub-~0.5mm-pitch BGA/QFN parts whose
     balls the standard via can't dog-bone/via-in-pad, pass the smaller
     `fine-pitch escape via` that `--design-rules` prints (e.g. 0.30/0.15, JLC
     advanced) as `--via-size`/`--via-drill` to THAT part's bga_fanout/route_diff
     (the route step's in-run finalize taps fine-pitch plane balls at the
     ROUTE step's via, so if it reports them unconnected, re-run that step
     with the smaller via). Keep the standard via for general route.py and
     the bulk route_planes pour (#99/#122).
   - PLANE FINALIZE (there is NO repair step since #562): every `route.py`
     run ends with an in-run plane finalize -- the same repair engine (pad
     taps + region joins), the plane-copper cleanup, and a KiCad-oracle
     completion verify, at the route step's own parameters, with stubborn
     nets joining that run's final reconciliation. So the old Step 5
     (repair_planes), Step 5c (reconnect the nets it ripped) and
     Step 5d (final plane verify) are all GONE from recorded chains: just
     make sure a route step runs LAST, and its finalize does all three.
     `KICAD_PLANE_FINALIZE=0` is the kill switch. The standalone
     `repair_planes.py` (engine now `repair_planes`, alias kept)
     is only for repairing a board routed OUTSIDE this chain.
     The GUI plan executor enforces the same rule -- it appends a final
     `route` step when copper-modifying steps follow the last plane step
     (ai_plan._append_final_plane_verify), and it SKIPS any legacy
     `repair_planes` step as a no-op.
   - TRACK WIDTH: the net-class `track_width` is a MINIMUM (keep it for the signal
     baseline); real boards widen power/high-current nets to many distinct widths
     (2-4mm buses) — widen those explicitly via `--power-nets`.
   A fine-pitch component's fanout NOTE still overrides locally for its nets.
   NOTE: qfn_fanout.py accepts only `--width`/`--extension` (NOT
   `--clearance`/`--track-width`); pass the width there, clearance to route.py.
3. Skip GND return vias / impedance / length matching unless the board
   obviously needs them (DDR memory). Keep `--add-gnd-vias` OFF to keep runs
   comparable. Skip schematic sync. Skip teardrops.
4. Plane strategy per skill: GND (+ main power rail if 4+ layers) as planes.
   2-layer boards: GND plane on B.Cu. 4+ layer: planes on inner layers.
   Exclude plane nets from FANOUT with `"!GND"`-style patterns (match the
   board's actual net names, e.g. "!/GND", "!AGND" — check the power listing
   first); that exclusion is also what marks those balls for plane-drop vias.
   Do NOT exclude them from the ROUTE step (#562): the route step takes
   `--nets "*"` INCLUDING the plane nets — their pads weld into the pour via
   pour-launch and the run's in-run plane finalize taps whatever the fill
   cannot reach. Pass every poured net in `--power-nets` with a width, which
   is where the finalize's welds and taps get their size.
   NET-COVERAGE INVARIANT (mandatory, plan-pcb-routing Step 5b): the route
   step's `!X` exclusion set must equal the Step-2b impedance nets, and every
   poured net must appear in `--power-nets`. A net excluded from routing and
   not poured silently gets ZERO copper and the run "completes" with it fully
   unrouted (this dropped ottercast_audio's GNDA 0/23); since #562 the twin
   failure is a poured net MISSING from the route step, which strands every
   one of its pads because nothing welds them to the pour. Secondary grounds (AGND/GNDA/
   DGND tied to GND through one 0Ω/ferrite — find the tie in the power listing)
   get their OWN pour region (Voronoi-share an inner layer is fine), NOT merged
   into GND and NOT left out. COVERAGE GATE at the end: `check_connected.py`'s
   "Unrouted net with N pads" list must be empty except for justified single-pad/
   NC nets — any multi-pad net there is a coverage defect to fix, not a stat to report.
5. Fanout: only for BGA/PGA/QFN/QFP components per the skill's depth rule.
   Through-hole connectors/DIPs don't need fanout.
   FIX VERIFICATION (issues #79/#80, fixed): fanout tools now write name-style
   net refs on KiCad 10 boards and the parser merges mixed styles. After each
   fanout step, still run
   `python3 <TOOLS_REPO>/tests/stress/fix_mixed_net_refs.py <fanout_output.kicad_pcb>`
   — it should report "rewrote 0 numeric net refs". If it reports >0, that is
   a REGRESSION: record it prominently in issues with the count.
   Heed the fanout tool's fine-pitch NOTE (issue #97 warning): use the
   suggested --grid-step/--clearance/--track-width for that component's nets.
   ESCAPE LAYERS (BGA/PGA): bga_fanout.py defaults to `--layers F.Cu B.Cu`
   only. On 4+ layer boards you MUST pass the board's inner copper layers too,
   e.g. `--layers F.Cu In1.Cu In2.Cu B.Cu`, or deep balls can't escape and are
   silently dropped (only the ~2 outer layers' worth of nets fan out — this
   capped ottercast_audio at ~23%). qfn_fanout.py is perimeter-only and
   doesn't need this.
   ESCAPE COMPLETENESS (issue #122): bga_fanout.py ends with
   `JSON_SUMMARY: {"requested","escaped","failed","unescaped_nets",...}`.
   ALWAYS parse it. If `failed > 0`, balls were DROPPED (removed from output;
   they resurface later as signal "no rippable blockers"). Retry the fanout with
   `--clearance` at the manufacturing floor (from --design-rules, e.g. 0.1) —
   this is the common cause regardless of pitch: an 0.8 mm-pitch BGA drops balls
   at --clearance 0.2 (a 0.2 mm track won't fit the ~0.45 mm inter-ball gap) but
   escapes all of them at 0.1. If still short, add the fine-pitch escape via /
   smaller --track-width. If a DENSE, fully-populated array still drops balls at
   the floor (channel router over-subscribes the between-row channels, e.g. ulx3s
   22x22 drops ~20), re-run with `--escape-method underpad` + small via/track
   (e.g. --via-size 0.35 --track-width 0.12 --clearance 0.1): it routes each ball
   UNDER the pad field on inner layers and escapes what channel can't (-> 0). It
   routes diff pairs single-ended and skips power/plane nets (plane them first).
   Do not start signal routing while balls are dropped.
   DECOUPLING-CAP OPTIMIZE (issue #130): ONCE after ALL BGA/PGA fanouts have
   completed (escaped == requested) and BEFORE signal routing -- not after each
   one: the pass is board-global, and per-BGA runs compound cap displacement
   (each run re-seeds at the moved position) and change what later fanouts
   route around (cap pads are escape obstacles). Run
   `python3 py_placer/place_fanout_clearance.py <fanned>.kicad_pcb <out>.kicad_pcb
   --clearance <floor>` (same clearance as the fanout). A foreign-net fanout via
   landing under a decoupling cap is a real PAD-VIA at the floor; this nudges
   those caps clear and pulls each pad toward its nearest same-net ball (so a
   later power/GND via shares the via). It reads each via's real size from the
   board, only moves 2-pad caps near a BGA, never overlaps caps, and is a no-op
   when nothing collides. It prints `resolved R/V initial violations; K
   unresolved`, with `(F freed by via-nudge)` when the #313 last resort moved
   a via to free a boxed cap; `resolved` is graded at the END of the pass and
   credits both mechanisms (#746). Unresolved caps are still grazing foreign
   copper (via, track or pad) and need a manual nudge; a `Re-grazed by this
   pass's own connector copper:` line names the ones that were clean before
   the nudge, i.e. copper this step drew rather than copper the board arrived
   with. Feed `<out>` into the next step; verify with
   `check_drc.py <out> -c <floor>` (PAD-VIA drops).
6. Diff pairs: if `--diff-pairs` reports pairs, route them with route_diff.py
   AFTER fanout and BEFORE signal routing (gap from --design-rules; use
   --no-gnd-vias). CRITICAL: a pair whose pads are on a BGA/PGA being fanned
   out MUST be escaped by bga_fanout itself — pass `--diff-pairs "<patterns>"
   --diff-pair-gap <gap>` to bga_fanout so P and N escape together on one
   layer. If you skip this (e.g. exclude the pair nets from fanout), the balls
   never escape and route_diff fails to launch from the deep balls ("no valid
   position at any setback"); route_diff then only connects the escaped stubs.
   Pairs NOT on an array package (e.g. between connectors) don't need fanout.
   If pair detection looks like a false positive (e.g. random net names that
   happen to end in P/N), note it as a finding and skip those.
7. DRC AT THE MANUFACTURING FLOOR (#111): grade DRC with the `check_drc.py`
   flags `--design-rules` prints (`--clearance <floor> --hole-to-hole-clearance
   <floor>`), NOT the net-class clearance and NOT a hardcoded 0.25. The floor is
   the JLC fab spacing minimum (the board's `min_clearance` is an unreliable
   edit-floor — sometimes 0, sometimes a stale large value — so it is not used).
   Fine-pitch escapes are routed down to this floor, so checking at it stops them
   reading as violations (the dominant set-1/set-2 DRC source) while still
   flagging anything genuinely sub-manufacturable.
   GRADE AT THE HOLE-TO-HOLE YOU ROUTED AT. The drill hole-to-hole minimum is
   only met if the via placers were *given* it: pass `--hole-to-hole-clearance
   <floor>` to route.py / route_planes.py / repair_planes.py, then
   grade at that SAME value. Grading at a hole-to-hole the routing never enforced
   invents phantom VIA-VIA-SAME-NET / VIA-DRILL-HOLE violations — the routed vias
   were never asked to meet it (a board routed without the flag shows 0 at its
   routed setting but dozens when graded at a forced 0.25). This is the
   hole-to-hole case of the general rule (grade at the value you routed at, never
   a stricter one): route AND grade at the floor so the two match and the result
   is genuinely manufacturable.
   AUTO-GRADE: `check_drc.py` now reads the routed clearance from the output
   `.kicad_pro` (the routing steps record the smallest clearance any step actually
   used — incl. auto-stepped fine-pitch taps — as `min_clearance_used` in
   JSON_SUMMARY and write it to the project DRC floor), so a bare `check_drc.py
   <out>` grades at the clearance you routed at. Keep passing the explicit
   `--clearance <floor> --hole-to-hole-clearance <floor>` for controlled A/B
   replays where you want a fixed, reproducible grading value regardless of the
   per-board project.
   BASELINE: before routing, run check_drc.py on the unrouted input at the same
   floor and record the count — real boards have pre-existing pad-to-pad
   proximity that is not the router's fault. Report post-route DRC as total AND delta.
   GROUND-TRUTH BASELINE (do this too): run `check_drc.py
   ~/Documents/kicad_stress_test/boards_set<N>/<board>.kicad_pcb --clearance <floor>
   --hole-to-hole-clearance <floor>` on the ORIGINAL human-routed board (SAME
   floor flags) and record that count as `drc.original_routed_violations`. That —
   not 0 — is the achievable floor under our DRC model (the originals carry a few
   clearance-independent hole-to-hole/pad violations: e.g. megadesk 1, piantor 6).
   Judge our routing's DRC against the ORIGINAL's count at the same floor, not against 0.
   KICAD-CLI CROSS-CHECK (MANDATORY on the final board, #316): run
   `python3 tests/stress/kicad_drc_compare.py <final>.kicad_pcb` -- it stages a
   copy with the netclass clearance equalized to the routed clearance (raw
   kicad-cli grades at the DESIGN netclass and storms phantoms), runs
   `kicad-cli pcb drc` on the copper classes, and diffs against check_drc.
   Record the kicad count and any KICAD-ONLY items in the results JSON
   (`drc.kicad_violations`, `drc.kicad_only`); KICAD-ONLY shorting_items are a
   red-alert finding (check_drc false negative -- the #324 offset-pad class
   shipped real shorts on boards check_drc graded clean). Two caveats: a
   kicad-cli "0" does NOT clear an *overlap/short* finding (KiCad 10
   net-unifies touching copper on load -- verified minimal repro, #260/#264;
   check_drc stays authoritative for touching-copper overlaps), and
   copper_edge_clearance on edge-connector fingers is design intent, not a
   routing defect. The no-LLM replay grader (ab_replay_grade.py) now records
   the same fields automatically.
   CONNECTION_WIDTH (#406): the cross-check also grades KiCad's min-copper-web
   class, SEPARATELY from the copper classes (check_drc has no counterpart --
   the artifact lives in KiCad's own float-borderline web measurement, so web
   items never count as KICAD-ONLY). KiCad ships the checker OFF
   (min_connection 0, warning severity); the staged copy turns it on at the
   author's min_connection when set, else the project's min_track_width (the
   post-route ledger floors it at the smallest object on the board). Recorded
   as `kicad_connection_width` (None = not graded: no recorded floor) +
   `connection_width_min`; ab_replay_grade compares the count per board (connw
   column) and gates the A/B verdict on its delta.
8. OOM REGRESSION CHECK (issue #81, fixed): the obstacle-map polygon pass is
   now chunked; DEFAULT grids should stay well under the 4 GB cap on every
   board. Use the default --grid-step unless component pitch demands finer.
   A MEMORY_LIMIT_EXCEEDED kill at the DEFAULT grid is a REGRESSION — record
   the command and RSS prominently. EXCEPTION: a board-global route at a fine
   grid (e.g. --grid-step 0.05) on a physically large 4+ layer board is the
   KNOWN issue #109, not a #81 regression — note it against #109 and, if you
   need a fine grid there, scope it per-component/region instead of board-wide.
   BGA-zone check (issue #82, fixed): keyswitches/diodes/thermal-via arrays
   should no longer be detected as BGA zones. Try WITHOUT --no-bga-zone
   first; if non-array footprints still get zones (or array parts lose
   them), record it as a regression with the detection printout.
9. Routing params: start with skill defaults. For boards with fine-pitch
   (<0.65mm) components, consider `--grid-step 0.05` and/or smaller clearance
   (record what you chose and why). For dense/2-layer boards use the skill's
   difficult-board params (`--max-ripup 5 --no-bga-zone`). Do NOT pass
   `--max-iterations`: #529 dynamic iterations is default ON and self-extends
   a progressing search to a 1e7 ceiling, so a fixed budget only caps it.
   `--max-ripup` above 5 is measured WORSE (each extra rip level risks a
   victim whose corridor is taken while it is out).
10. One retry round allowed: if routing fails some nets, re-run per the skill's
   "Diagnose and Retry" table (use the same output->input chaining) with
   **`--nets '*'`, NOT a hand-listed set of the failed net names**. route.py
   skips nets that are already fully connected, so a wildcard retry attempts
   exactly the nets still broken -- same work, same result, and it stays correct
   for any router. A hand-listed retry freezes THIS run's failure identity into
   the manifest: every future replay hands the baseline a rescue fitted to its
   own failures while any engine change, failing a different net, gets its
   failure shipped and healthy nets retried (RUNBOOK rule 5 -- 25% of the corpus
   is already contaminated this way). Name nets only when the naming IS the
   experiment (the skill's failed-first split), and expect that board to be
   unusable for A/B. Record both attempts. If the failures are CONGESTION (rippable
   churn, many fails clustered in one channel, or "boxed in by static obstacles"
   at fine pitch), route signals at the FAB FLOOR (skill: "Route signals at the
   FAB floor by default"). KEY POINTS: (a) thinner is monotonically better on
   dense boards — more nets complete AND faster (ottercast: 0.127mm=122 nets/2.7s
   vs 0.2mm=103 nets/6.5s), so route thin from the start, no widen-back sweep.
   (b) The fab floor is the fab's PHYSICAL track minimum (JLC 0.0889mm/3.5mil;
   0.127mm/5mil = safe zero-cost default), NOT the board's min_track_width and NOT
   the "manufacturing floor" track that --design-rules prints (it clamps track to
   max(board_min, JLC) so it can read 0.2 — that clamp is the bug; the via and
   clearance floors it prints are fine). Going BELOW the board's min_track_width is
   intended — the human ottercast board routes 0.127mm under its own 0.2mm
   constraint. (c) Re-run the WHOLE signal step thin (not just failed nets — their
   blockers are the already-routed wider tracks), keep power/impedance nets wide,
   add a finer --grid-step for fine-pitch escapes; if still congested step the
   width down further toward 0.0889.
10a. RIP AUTHORITY IS A LAST RESORT, NOT A RETRY DEFAULT (#600). `--rip-existing-nets`
   and `--force-reroute` are permission to DESTROY already-routed copper, and a rip
   whose restore is refused leaves that net broken. In the sets-21-27 wave this was
   the single largest source of lost connectivity — larger than routing failure
   itself (7 of 99 boards; `bms_sensor` turned a 3-pad problem into a 20-pad one,
   `spartan6_4layer` lost 20 nets all of their copper). **Scoping `--nets` does NOT
   protect you** — `ftdi_debug_toolkit` regressed from a retry naming three nets. It
   is the rip PERMISSION, not the route scope. Order of preference: (1) re-run the
   whole signal step THINNER per rule 10 — destroys nothing; (2) a PLAIN retry of the
   failed nets — the in-run #103 escalation already grants itself targeted authority
   over the exact blockers the log named, so you usually need no flag at all;
   (3) `--rip-existing-nets <the named blockers>`; (4) `'*'` only as a last resort,
   and never with `--force-reroute` over a large net list — that combination is the
   `spartan6_4layer` shape. The engine now backstops this: a run that ends net-worse
   prints `IMPROVEMENT GATE … REVERTED` and restores the input board. **If you see
   that, the step did not fail to run — it ran and was REJECTED.** Do not re-run it
   with more authority; change the approach or accept the open nets and report them.
   Assert on the `JSON_IMPROVEMENT_GATE:` line (`lost`/`gained`/`verdict`) rather
   than reading prose, and record a REVERTED step in `issues`.
11. Verification (always, on the final board):
    - `check_drc.py <final> --clearance <floor> --hole-to-hole-clearance <floor> 2>&1 | tee drc.log`
      (manufacturing floor from `--design-rules`, per step 7; note the flags used)
    - `check_connected.py <final> 2>&1 | tee connectivity.log`
    - `check_orphan_stubs.py <final> 2>&1 | tee orphans.log`
11b. COMPARE-TO-ORIGINAL (always, final step): run
    `python3 <TOOLS_REPO>/tests/stress/compare_to_original.py
     --ours <final> --orig ~/Documents/kicad_stress_test/boards_set<N>/<board>.kicad_pcb
     --json 2>&1 | tee compare.log`
    It contrasts OUR routing with the human-routed original (vias, total copper
    length, track-width strategy, layer balance, nets-with-copper) and prints
    SUGGESTIONS for what to change in our routing/approach. Fold its
    `JSON_COMPARISON` blob into the results JSON `comparison` field and copy its
    suggestion lines verbatim into the `suggestions` field. The original is the
    ground truth for a manufacturable board; treat large via/length/width/
    layer-balance gaps as router-improvement findings, not just board facts.
12. Budget: ~3.5 h wall-clock for the whole board, and a HARD 3-hour cap
    per command: if a single tool invocation passes 3 h — even with its log
    still growing — kill it, record the elapsed time + command as a runtime
    finding, and continue with the previous step's output. Aggressive params
    (deep --max-ripup at fine grids) can grind
    for hours; a board that's still making progress is fine, but one that
    cycles 1M-iteration A* exhaustions with no net newly connected (issue #211:
    ulx3s) is wedged, not slow. NEVER end your turn while
    a routing command is still running — you will be terminated and the run
    orphaned. Run commands in the FOREGROUND, and **pass an EXPLICIT timeout on
    every routing/fanout/plane command: `timeout: 600000` (10 min, the maximum).
    This is required, not an upper bound you may ignore (#599).** The Bash
    tool's DEFAULT timeout is 120000 ms — two minutes — while a route step
    routinely takes 3-20x that (`faderbank_16nx` 316 s, `wisweep_driver` 264 s,
    `crazyflie_fpga_deck` 189 s). Omitting the timeout killed at least one
    attempt on 21 of the 99 boards in the sets-21-27 wave; the kill takes the
    launching shell with it, so `run_limited.sh` reports it as
    `BACKGROUNDED_STEP_ORPHANED` and it reads like a hang or a rule-12
    violation when it was neither. The wrapper now prints the elapsed time and
    a "the caller's timeout is too short" hint when it dies far short of the
    3-hour cap — believe it, and re-run with the timeout rather than retrying
    the same way or blaming the router. If a
    command exceeds the 10-min foreground cap, keep waiting in foreground:
    repeatedly run `until ! pgrep -f "<unique-cmd-fragment>" >/dev/null; do
    sleep 10; done` (each up to 10 min) until the process exits, then read its
    log and continue. Big/dense boards (FPGA/USB3-class: daisho, large BGAs)
    can legitimately spend 30-90+ min in a single signal-route step — that is
    slow progress, NOT a hang. Only kill a command once it shows no log growth
    AND no output-file size change for >45 min, and record it as a hang.
    **This is ENFORCED, not advisory:** `run_limited.sh` carries an orphan guard.
    It cannot see the `&` (that lives in your shell), but when the launching shell
    exits it detects the re-parent to init, writes `ORPHANED_STEP.txt` into the run
    dir, prints `BACKGROUNDED_STEP_ORPHANED`, and KILLS the step with exit 138 —
    its output would never be graded, and a killed process never runs
    `redo_record`'s atexit hook, so the replay record is lost too (that is how
    ottercast_audio, abn6502 and pcie_test_edge were lost on 2026-08-06). Poll in
    the foreground as above instead. Escape hatch for a deliberate detached run:
    `STRESS_ALLOW_ORPHAN=1` (warns once, does not kill); grace period
    `ORPHAN_GRACE` seconds, default 60.
13. If a tool crashes (traceback), capture the full traceback in the JSON
    issues list. A crash is a valuable finding, not a failure of your task —
    continue with remaining steps if possible.
14. The JSON_SUMMARY line printed by route.py/route_diff.py has structured
    routed/failed counts — parse it for the results JSON.
15. TIMING (for run-to-run comparison): at your VERY FIRST action capture
    `T0=$(date +%s)`. Time EVERY routing/fanout/plane/check command — wrap as
    `s=$(date +%s); <cmd> ...; echo "step_wall=$(($(date +%s)-s))s"` (works with
    the run_limited.sh wrapper and background+poll too: record start before
    launch, end at DONE) — and put each elapsed value in the matching
    `steps[].wall_s`. Just before writing the JSON, compute
    `AGENT=$(($(date +%s)-T0))`. Then fill: `timing.agent_wall_clock_s`=AGENT
    (TOTAL wall-clock, INCLUDING model thinking + driving between commands),
    `timing.tool_exec_s`=sum of all `steps[].wall_s`,
    `timing.thinking_driving_s`=AGENT-tool_exec_s, and set
    `wall_clock_total_s`=AGENT. These let us compare both raw tool cost and
    end-to-end agent cost across runs.
16. GRADE YOUR ACTUAL FINAL BOARD. `drc.final_violations` (and `connectivity`)
    MUST come from `check_drc.py` + `check_connected.py` run on the EXACT
    `.kicad_pcb` produced by your LAST board-mutating step (including any `fix`/
    rip-up/reconnect retry). Never report a grade taken from an earlier step — a
    late step (esp. `route.py --rip-existing-nets`, #284) can corrupt a clean board
    (artix_dc_scm: `step5d`=0 → `step5e_fix`=519 shipped as "0"). This is a stress
    test of the FULL CHAIN: report the honest final and let us SEE the DRC errors —
    do NOT revert/cherry-pick a cleaner earlier board to lower the count. A step
    that INCREASES DRC is a valuable finding: keep it as the final, report the true
    number, and record the regressing step + command in `issues`. (The harness
    independently re-grades your final via `grade_final.py` → `authoritative_grade.json`
    and marks `MISGRADE` in `.worker_done` when your self-report disagrees.)

## Results JSON schema

```json
{
  "board": "<BOARD>",
  "layers": 2,
  "routable_nets": 0,
  "plan": {"fanout_components": [], "diff_pairs": [], "plane_nets": [], "plane_layers": [], "notes": ""},
  "steps": [{"name": "", "cmd": "", "wall_s": 0, "outcome": ""}],
  "timing": {"agent_wall_clock_s": 0, "tool_exec_s": 0, "thinking_driving_s": 0},
  "routing": {
    "nets_attempted": 0, "nets_routed": 0, "nets_failed": 0,
    "completion_pct": 0.0,
    "multipoint_pads_connected": 0, "multipoint_pads_total": 0,
    "vias": 0, "route_time_s": 0.0,
    "failed_nets": [], "retry_improved": false
  },
  "diff_pair_routing": {"pairs_attempted": 0, "pairs_routed": 0, "polarity_swaps": 0},
  "planes": {"nets": [], "unconnected_pads": 0, "isolated_regions": 0, "repair_outcome": ""},
  "drc": {"baseline_violations": 0, "original_routed_violations": 0, "design_clearance": 0.0, "final_violations": 0, "delta": 0, "by_type": {}},
  "connectivity": {"fully_connected": false, "detail": ""},
  "orphan_stubs": 0,
  "comparison": {"design_clearance": 0.0, "ours": {}, "original": {}},
  "suggestions": [],
  "wall_clock_total_s": 0,
  "issues": ["crash/hang/bogus-output/parser findings, each with 1-3 sentence detail"]
}
```

## Final report (your last message)

Return a compact summary: completion %, DRC delta (vs the ORIGINAL routed
board's count at the design clearance), connectivity verdict, the
compare-to-original highlights (vias/length/width/layer-balance vs original),
timing (agent_wall_clock_s + tool_exec_s), plus the `issues` and `suggestions`
lists verbatim. No file dumps.

## Replaying & A/B (no LLM)

Each board's `runs_set*/<board>/redo_commands.sh` records every board-mutating
command (fully-quoted argv + `# cwd=`), so a run replays deterministically with no
LLM. Manifests reference tools by **absolute repo path**, so a replay always runs
whatever is checked out — this is how you A/B an engine change. DRC is always
graded at each board's own routed `--clearance` (parsed from the manifest); never
grade stricter or you manufacture phantom grazes.

**When the CLI drops a flag, migrate the corpus — do not add a compatibility
shim.** Manifests are replayed verbatim, so a removed flag kills the chain in
argparse. `python3 tests/stress/migrate_manifests.py [ROOT] [--apply]` rewrites
them in place (dry-run by default, idempotent, surgical: it excises only the
flag and leaves every other byte alone, because the corpus is NOT under version
control). It is scoped per script — the #562 pass touched only `route_planes.py`
lines (54 of 459 chains, 63 `--rip-blocker-nets`) and deliberately left
`repair_planes.py` lines alone, since that engine still has those
flags. Extend its table when a future flag goes.

- **One board:** `python3 tests/stress/redo_stress_test.py
  runs_set1/<board>/redo_commands.sh --workdir <fresh-dir> --continue-on-error`.
  `--workdir` runs every command in the fresh dir (relative outputs chain there;
  the absolute source board still resolves) — prefer it over bare `--remap` so a
  recorded `# cwd=` can't overwrite the original run. `--skip-checks` drops the
  `check_*` steps (grade the final yourself); `--verbatim` replays superseded
  retries too (default prunes to the file-dependency chain).

- **Whole set, graded (full chain):** `python3 tests/stress/ab_replay_grade.py
  --set ~/Documents/kicad_stress_test/runs_set1 --out <wavedir> --label new
  [--jobs 4]`. Replays every board in parallel, grades each final for DRC + conn,
  writes `summary.json` (a JSON list of per-board dicts). Boards within a wave run
  in parallel; **waves must be sequential** (they share git state). Compare two
  waves: `ab_replay_grade.py --compare OLD/summary.json NEW/summary.json`.
  `--regrade <wavedir>` re-grades finals without re-routing.
  - **Baseline already exists:** `runs_setN/summary.json` is a graded wave from the
    last stress run (same schema), so to A/B current HEAD vs the last run you only
    run ONE new wave and `--compare` it against `runs_setN/summary.json` — no need
    to check out old code.

- **Whole corpus, diff-pair stages only:** `python3 tests/stress/redo_diff_stage.py
  [boards...] [--jobs 4 --stagger 8] [--out-dir ~/Documents/diff2]`. Auto-detects
  boards whose manifest has a `route_diff.py` step, replays only fanout +
  `route_diff` (truncated through the last non-help `route_diff` line), and flags
  any board with deferred/failed pairs, DRC violations, or no output — copying
  flagged boards' `kicad_*` to `--out-dir`. Much faster than the full chain; use it
  when the change only touches fanout / diff-pair routing.

- **Whole set, from a mid-chain step (reuse a prior wave's upstream boards):**
  `python3 tests/stress/partial_replay_from_planes.py --set runs_setN --seed
  <prior_wave>/setN --out <fresh_wave>/setN [--only b1,b2] [--from-script route_planes.py]`.
  Cuts each board's manifest over at the first `--from-script` command (default the
  first `route_planes.py`), stages the boards that step reads from upstream out of
  `--seed` (with their `.kicad_pro` DRC floors), and replays only the tail via
  `redo_stress_test`. Use it when a change only touches the later stages (plane
  routing, reconnect, grading) so the expensive fanout/diff/`route.py` signal
  routing is skipped. Grade the finals yourself (`ab_replay_grade.py --regrade` or
  `kicad_drc_compare.py`).

Rule of thumb: full-chain regressions → `ab_replay_grade.py`; diff-pair
regressions → `redo_diff_stage.py`; plane/reconnect/grading-only changes →
`partial_replay_from_planes.py` (reuses a prior wave's upstream boards).

## Corpus-scale A/B and bisect on the cloud (no LLM, ~$1/arm)

`ab_replay_grade.py` and `ab_wave_driver.py` run locally and grade what YOUR
machine can chew through. These two run the same replays on rented cores, keep
the routed boards, and are what you want for "did this change help across 150
boards" and "which commit broke connectivity".

- **`cloud_replay_sets.py`** — replay whole sets on Modal, KEEP the finished
  `.kicad_pcb` (with its `.kicad_pro`/`.kicad_dru` siblings), and A/B against a
  baseline. Six stages, pick with `--only`: `plan` (prices it, spends nothing) /
  `upload` / `run` / `harvest` / `baseline` / `compare`.

  **The image carries KiCad by default (since 2026-08-23).** `--with-kicad` is
  the default and builds on `kicad/kicad:10.0.0`, so the **oracle legs actually
  run**. `--no-kicad` builds `debian_slim` instead, and there every oracle leg
  is **DEAD, not degraded** -- `oracle_reconnect` returns `available=False` the
  moment `find_kicad_cli()` is None -- so a change acting through the finalize
  audit, the plane/oracle recheck or #589 measures as exactly zero on such a
  wave. Prefer the default unless you are deliberately reproducing an old one.

  A KiCad wave suffixes its label `-kc`, because the results volume RESUMES by
  arm name and arm = label + sha: two waves at the same commit differing only by
  the image would otherwise share rows, which is precisely the
  "the baseline was not the baseline" failure `arm_name()` exists to prevent.

  Note the crate is built IN the image. When `rust_router/Cargo.toml` is ahead
  of the latest release tag (i.e. a crate bump whose binaries are not published
  yet) `build_router.py` skips the prebuilt and compiles from source -- rustup
  is installed for exactly this, at the cost of a ~10 min cold build that Modal
  then caches.

  ```bash
  # where does HEAD stand vs the recorded runs, sets 10-19?
  python3 tests/stress/cloud_replay_sets.py --sets set10-set19
  # one arm with a knob changed (rides the ARM SPEC -- containers do NOT
  # inherit your shell's env)
  python3 tests/stress/cloud_replay_sets.py --sets set10-set19 --label smoothoff \
      --env KICAD_SMOOTH_ROUTE=0
  # or a routing_defaults constant, patched in the container's own repo copy
  ... --label hw19 --defaults HEURISTIC_WEIGHT=1.9
  ```

- **`corpus_bisect.sh`** — score ONE engine commit across the corpus, for
  bisecting a regression: `bash tests/stress/corpus_bisect.sh <sha> <tag>`.
  5-6 points bracket a 38-commit range for under $10.

- **`cloud_arms_to_sweep.py`** — score a knob screened as SEPARATE
  `cloud_replay_sets.py` arms. Each arm lands in its own wave dir, and the
  wave-dir readers (`ab_wave_report.py`, `--compare`) do a two-wave roll-up
  with no chain pairing, no rescue-clean cell and no `--hard` split — i.e.
  without rules 3, 5 and the congestion dilution below. Merge the arms into
  one sweep json and score them with the tool that applies all of it:

  ```bash
  # one control arm + one knob arm, launched separately, at the SAME commit
  python3 tests/stress/cloud_replay_sets.py --sets set1-set5 --label dirs250
  python3 tests/stress/cloud_replay_sets.py --sets set1-set5 --label dirs5 \
      --defaults DIRECTION_PREFERENCE_COST=5 --no-baseline
  python3 tests/stress/cloud_arms_to_sweep.py \
      ~/Documents/kicad_stress_test/cloud_dirs250_<sha> \
      ~/Documents/kicad_stress_test/cloud_dirs5_<sha> --out sweep_dirs.json
  python3 tests/stress/modal_sweep/rank_arms.py sweep_dirs.json --drop-rescue-clean
  ```

  It merges each wave's REGRADED grading with the `_raw` provenance the local
  regrade drops (`arm`, `steps`, `rescue_steps`, `patched_defaults`) — feed
  rank_arms the regraded rows alone and you silently disable its arm
  identification, its chain-identity guard and its rescue cell at once. Rows
  the regrade could not re-score are dropped rather than paired against
  locally-graded rows, per rule 2 (waves banked before 2026-08-23, or launched
  `--no-kicad`, carry no `drc_real` at all).

  **Launch the arms at ONE commit and do not commit in between.** The image is
  `git archive HEAD`, so a commit landing between two launches makes the arms
  differ by more than the knob — the arm name records the sha it was launched
  at, so check that both wave dirs carry the same one.

### Rules that make these trustworthy

1. **The baseline is the RECORDED RUNS, re-graded — not an archived `ab_*` wave.**
   The corpus gets re-recorded: after the #562 pours-first reshape every one of
   sets 10-19's 150 boards produces a different final output than the 2026-07-28
   wave did, so diffing against it mixes a different PLAN in with the engine
   delta. `--baseline recorded` (the default) compares like with like; preflight
   refuses a wave whose chains disagree.
2. **Grade both sides on the same terms.** Comparing a row graded one way
   against a baseline graded another measures the GRADER, not the engine: it
   once reported "DRC +40 worse" when the truth was "-37 better". Harvest
   re-grades the kept boards locally by default (`--no-local-regrade` opts out).

   **Keep that default even though the cloud now HAS KiCad.** Since 2026-08-23
   `--with-kicad` is the default (`kicad/kicad:10.0.0`), so the containers do
   have `kicad-cli` and their `drc_real` is genuine rather than a raw-DRC
   fallback. The rule survives anyway, because its reason changed rather than
   disappeared: the BASELINE is the recorded runs graded on YOUR machine, so the
   arm has to be graded there too. Same-terms is the invariant; which grader is
   incidental. (A wave built `--no-kicad` is a different matter -- see the
   image note under `cloud_replay_sets.py` -- and its rows carry no `drc_real`
   at all.)
3. **Compare arms only on boards that replayed an IDENTICAL chain** — same step
   count and same final board. A short chain grades artificially WELL, because
   nets its missing steps never attempted are not counted as incomplete.

   **Do NOT pair on `nets_total` as reported.** It is not a property of the
   board: check_connected's "Checking N routed nets" counts only nets that ended
   up with COPPER, so a net an arm fails entirely drops out of that arm's total
   and reappears under "Unrouted nets". The same board therefore reports a
   different total per arm (butterstick: 310/314/316/316 on an identical 16-step
   chain — all 317 once the unrouted are added back), and pairing on it discards
   exactly the boards WITH unrouted nets, i.e. the congested ones. On the #590
   sets 1-10 wave that dropped 40 of 103 boards carrying ~85% of all the
   incompleteness and turned a -48 result into -8. `ab_replay_grade._completion`
   now reports the corrected census; `rank_arms.gradeable_nets` reconstructs it
   for rows banked earlier. Known residual: a one-pad net that picks up plane
   copper counts as routed but never appears among the unrouted (which grades
   >=2-pad nets), leaving a rare +-1 that only a census emitted by
   check_connected itself can fix.
4. **Score connectivity on `nets_incomplete` ALONE.** It already counts unrouted
   PLUS connectivity-issue nets. `nets_incomplete + conn` (which the sweep's
   screened-stage gate used to score) counts every connectivity-issue net twice,
   pricing "failed to connect a net" at double "lost the net's copper entirely".
   Grading on `conn` alone is wrong from the other side: a net that loses its
   copper LEAVES the conn bucket for the unrouted one, so conn can fall while the
   board got worse. `rank_arms.py <sweep.json>` applies all of this — paired
   verdict per arm, W/L, DRC reported beside it rather than folded in.
5. **A recorded RESCUE step biases the board against any change — 25% of the
   corpus has one.** Chains often end with `route.py ... --nets '/CM4
   GPIO/GPIO22' '/CM4 GPIO/SD_CMD'`: the nets that failed *in the run being
   recorded*, retried at a tighter clearance or width. The baseline replays that
   run deterministically, so the rescue lands exactly on its failures and the
   board finishes clean; an arm that routes differently fails a DIFFERENT net,
   which the frozen list never retries, so its failure ships while healthy nets
   get retried. None of that measures routing — in production the retry is
   authored AFTER seeing what failed; only in replay is it pinned to one arm's
   failure set.

   The bias bites hardest where the rescue leaves the baseline nearly clean:
   no headroom to win, every displaced net a loss. Measured on both #590 waves,
   that cell punished EVERY arm — sets 11-20: +2..+8 per arm over 22 boards
   holding 2 baseline failures; sets 1-10: +3..+6 over 16 boards holding 4.
   Congested rescue boards still discriminate (they keep showing arm-ordered
   differences), so only the clean ones are unmeasurable. Removing that cell
   alone moved the sets 11-20 winner from -2.5% (p=0.15) to -5.2% (p=0.046).

   `ab_replay_grade` records `rescue_steps` per board; `rank_arms.py` reports
   the cell and drops it with `--drop-rescue-clean`. Report it either way —
   silently dropping boards is how a knob talks itself into a default.
6. **A two-board result is not a default change.** Per-board run-to-run spread is
   +-2..3 nets (the same config measured 7 and 5 on consecutive runs), so single
   boards cannot resolve anything smaller. Two defaults were shipped and reverted
   on this exact mistake.
7. **Arm names carry the source commit**, so resuming re-uses banked rows only
   within one commit; the launch-time name is recorded in `<out>/arm.txt` because
   HEAD moves between stages when another session commits.
8. **A manifest records COMMANDS, not the recording shell's environment.** A
   `KICAD_*` export the driving agent set as a workaround replays as unset --
   the recorded timing/outcome can then be unreproducible at ANY commit.
   core64_logic (#625) "replayed in 4 min historically": the original run only
   terminated because its agent exported `KICAD_DYNAMIC_ITERATIONS=0` mid-run;
   every replay ran the shipped default and burned the 3 h cap. Before trusting
   a recorded run as a baseline, grep its `transcript.jsonl` for `KICAD_`
   exports (the timing sidecar cannot tell you).

## Multi-set waves & release sign-off

`ab_replay_grade.py` grades **one set**. A release decision (should this become a
tag?) spans the whole corpus and more than one baseline, which is what these two
add. Both keep `ab_replay_grade`'s grading semantics and `summary.json` schema, so
they interoperate with `--compare` and `--regrade`.

- **`ab_wave_driver.py`** — replays many sets under ONE global queue, so `--jobs`
  boards stay in flight **across set boundaries** (`ab_replay_grade --set` drains
  each set's tail to idle before the next starts — over 11 sets that dominates).
  It calls `ab_replay_grade.do_board` per board, so grading, the per-board
  `--clearance`, and the #405 baseline subtraction are unchanged.

  ```bash
  # candidate wave, sets 1-11, 5 boards in flight at all times
  nohup caffeinate -s python3 -u tests/stress/ab_wave_driver.py wave \
      --out ~/Documents/kicad_stress_test/ab_main_0728a --label head --jobs 5 \
      --cost-baseline ~/Documents/kicad_stress_test/ab_main_0726a \
      > ~/wave.log 2>&1 &

  # re-grade an existing wave in place with today's grader (no re-routing)
  python3 tests/stress/ab_wave_driver.py regrade \
      --out ~/Documents/kicad_stress_test/ab_main_0726a --label base --jobs 5
  ```

  Scheduling: **`--order name` is the default** — plain corpus order, by set then
  board name (numeric sets sort 9 before 10). Prefer it: the Nth board of a wave
  is the same board every run, two waves' logs interleave comparably, and a wave
  killed part way through covers a predictable prefix. Order never affects
  grading (routing is deterministic and each board is independent), so this costs
  only wall-clock.

  `--order lpt` is the opt-in alternative — longest-processing-time-first from
  `--cost-baseline`, so the corpus's multi-hour board starts at t=0 instead of
  stranding idle cores at the end. Shorter makespan, but run order becomes
  data-dependent (it shifts whenever the baseline changes), so waves stop being
  comparable board-for-board. Use it only when wall-clock genuinely matters.

  **Memory admission is off by default** (`--heavy-slots 0`), so the chosen order
  is honoured strictly. Set `--heavy-slots 1` on a small-RAM box to cap how many
  boards over `--heavy-mb` run at once (~11 corpus boards exceed 2.5 GB and one
  peaks at 6.6 GB; several together on an 8 GB box swap and get workers
  OOM-killed, surfacing as NORESULT rows hours in). When enabled a blocked heavy
  board does **not** hold a worker slot — the scheduler starts the next eligible
  board — so `--jobs` stay in flight, but that *is* a reordering.

- **`ab_wave_report.py`** — rolls a candidate wave up against one or more
  baseline arms, all sets at once, ranking the per-board regressions.

  ```bash
  python3 tests/stress/ab_wave_report.py \
      --new  ~/Documents/kicad_stress_test/ab_main_0728a \
      --base 0726a=~/Documents/kicad_stress_test/ab_main_0726a \
      --base dp250=~/Documents/kicad_stress_test/ab_dp250
  ```

  Grades on `drc_real` / `nets_incomplete` / `kicad_connection_width` /
  `diff_pairs_coupled` (negative is better except diff-pairs) — never raw `drc`
  (counts pre-existing input copper) and never `conn` alone (a net that loses its
  copper entirely moves from the conn bucket to the unrouted bucket, so `conn`
  can drop while the board got worse). A baseline covering only some sets (e.g.
  `ab_dp250` = sets 6-11) reports on the sets it has. Boards whose **chain broke**
  are listed separately and are a release blocker — they can never show up as a
  DRC delta, because a broken chain has no final board to grade.

  `diff_pairs_coupled` measures COUPLED TRUNKS, not member-pad connectivity
  (#602): a pair whose terminals were peeled to the single-ended follow-up is
  counted as coupled by design. To assert that a diff-pair stage left no open
  member pads, gate on **`diff_pairs_member_incomplete`** (route_diff's own
  member audit, `member_incomplete_pairs` in `JSON_SUMMARY`) — not on the
  coupled count, and not by grepping the `MEMBER AUDIT` lines out of the log.

### Running a wave that lasts hours

- **Detach it**: `nohup … &`, and verify it reparented to init
  (`ps -eo pid,ppid,etime,command | grep ab_wave_driver`). A foreground wave dies
  with the terminal or the agent session.
- **`caffeinate -s`** around the whole driver — a mid-wave sleep suspends every
  worker and corrupts the timing columns.
- **Attach a monitor** that greps for *both* progress and every failure shape;
  one that matches only the happy path is silent through a crashloop:
  `tail -f wave.log | grep -E --line-buffered
  "chain=BROKEN|NORESULT|ALL DONE|Traceback|Killed|MemoryError|PROGRESS"`.
- **Freeze the working tree for the whole wave.** Manifests bake tool paths
  absolutely, so a replay runs whatever is checked out *right now*; editing a
  routing module mid-wave means early boards ran different code than late ones,
  silently. Adding new files is safe. Two waves must therefore run sequentially.
- **Regrade the baseline — never diff against stored numbers.** Every grader here
  is under active development, so an old `summary.json` is a snapshot of code that
  no longer exists; re-grading an old wave's own copper has moved rows in both
  directions and once turned a real −24 into an apparent −72. `regrade` rewrites
  `summary.json` in place, so `cp summary.json summary_orig.json` first. If a
  regrade reproduces well under ~100 % of the stored rows, that table was never a
  usable baseline.
- **Wave dirs are write-once** (`ab_<what>_MMDD` + same-day `a`/`b`/`c`): re-running
  into an existing dir reads back the sibling `.kicad_pro` DRC floor and silently
  changes the routing — it looks like non-determinism but isn't.

## Choosing a subject BEFORE you stage it (read this first)

Four perturbed-corpus runs have posted a recovery near zero, and only one of
them (run 8, the corner-only slot model) was the placer's fault. Run 7 was a
wrong basin, run 9 was three tools failing to terminate, run 14 was a dose
clipped to 0.100 mm. **Three of the four were the measurement rig, and each one
cost an hour of chain time to discover.** Most of that is avoidable, because the
questions are cheap to ask up front and nobody was asking them.

### 0. You need UNROUTED candidates, and the corpus is nearly empty

`boards_unrouted_set1/` currently holds exactly one board. Qualify against
unrouted twins, not against `boards_set1/`: a routed board is refused by
`placement_driver --stage P0`, and its copper encodes the original poses (run 14
measured 301 of 569 pads sitting within 5 um of their own track endpoint, which
`fence_audit` cannot see because it compares poses and never opens copper).

```bash
python3 -X utf8 tests/stress/strip_copper_only.py \
    $STRESS/boards_set1/<name>.kicad_pcb $STRESS/boards_unrouted_set1/<name>.kicad_pcb
```

**Do NOT use `strip_routing.py` or `prep_set2.py` for this.** They are corpus
normalizers and they rewrite `Edge.Cuts`, so the subject would no longer share an
outline with the human reference it is graded against. `strip_copper_only.py`
drops exactly four form types (top-level `segment`, `arc`, `via`, and
`filled_polygon` inside zones) and leaves poses, pads, zones, stackup and outline
untouched; verified on castor_pollux, 14241 segments and 358 vias to zero with an
identical pose digest and identical bounds.

### 1. Qualify the board (seconds, not an hour)

```bash
python3 -X utf8 tests/stress/qualify_subject.py \
    $STRESS/boards_unrouted_set1/*.kicad_pcb --draws 8
```

It perturbs to a temp dir, grades copper-free, and prints aggregates only (rates
and medians, never the kind, block, seed or direction), so it is safe to run on a
board you then intend to stage blind. Three verdicts:

| verdict | meaning |
|---|---|
| `REJECT` | the rig cannot damage this board. Draws clip to nothing, so `recovery` and `home /N` will read near-perfect whatever the run does. |
| `WEAK` | usable, but a coin flip decides whether the run gets a subject. Redraw and never assume the dose landed. |
| `GOOD` | the dose lands nearly every time AND the copper-free gates fire. Only this is a subject. |

Run 14's board scores **WEAK** on this test (6 of 8 draws land; applied dose
ranges 0.100 to 57.2 mm, and the 0.100 is the draw the run actually got). Had
anyone run it, run 14 would have picked a different board or expected the
redraw. `stage_blind` now redraws by itself, but that only rescues an unlucky
draw; it cannot rescue an unsuitable board.

### 1b. Choose damage kinds the fence can adjudicate

A qualified board can still stage an **undecidable** run: on a grid-homed
board, `swap` and `translate` recovery is byte-identical to the truth board
BY CONSTRUCTION — every displaced part's home pose is a grid point any honest
search also lands on, so a perfect result and a truth-file LEAK produce the
same bytes and the fence cannot tell them apart (run 18's undecidable LEAK
verdict). Before staging, pick from `KINDS` (`placement/perturb.py:56` —
`translate`, `wrong_side`, `swap`, `scatter`, `pile`) with the fence in mind:
on grid-homed boards prefer `pile`/`scatter`, whose recovered poses carry no
byte-identity shortcut, or pre-declare the secondary tell (which independent
measurement will separate an honest recovery from a leak) BEFORE the damage
is drawn. A tell declared after the result is an accusation, not a fence.

### 2. Run a positive control first

The series has no control arm, so every null is ambiguous between "the placer
failed" and "the rig failed". **tigard is the known positive**: run 3 delivered
recovery +0.133 with 26 of 51 parts home. Re-run it whenever the rig changes.
If it reproduces, a null elsewhere means something. If it does not, you have
found the next rig bug for the price of a board you already understand.

### 3. Confirm the damage actually threatens ROUTABILITY

`qualify_subject.py` stops at the copper-free gates because routing is the
expensive half. The question it cannot answer is the one that decides whether a
placement run can succeed at all:

```bash
# the original must route ...
python3 -X utf8 py_router/route.py <control>.kicad_pcb /tmp/ctl.kicad_pcb --nets "*"
python3 -X utf8 py_router/check_connected.py /tmp/ctl.kicad_pcb
# ... and the damaged one must NOT
python3 -X utf8 py_router/route.py <staged>.kicad_pcb  /tmp/dmg.kicad_pcb --nets "*"
python3 -X utf8 py_router/check_connected.py /tmp/dmg.kicad_pcb
```

**A placement-focused subject is one where a material dose makes the board
unroutable and recovery makes it routable again.** That is falsifiable, it is
what the tool is actually for, and it is the doctrine's own metric: lead on
`blocking`, keep `recovery` as a diagnostic. On run 14 the damaged board routed
to `blocking 0` unaided, which means the placement half had nothing to prove
even before the dose was found to be 0.1 mm.

### 4. Do not pay for a full route per trial

Placement science needs many trials; a full chain costs about an hour and most
of that is routing that tells you nothing new. Grade trials with the copper-free
battery plus `tests/test_placement_probe.py`, which scopes the route CAUSALLY
(the nets `net_affinity` flagged plus the declared corridor nets, fixed from the
OFF board) rather than by which parts moved. Scoping by moved parts is circular:
a term that moves nothing scores a perfect null. Run the full chain once, at the
end, on the arm you intend to keep.

### 5. Report shape

Lead with `blocking`. Report `recovery` and `home /N` as diagnostics, and state
the applied dose next to them every time, because a recovery number without the
dose that produced it is uninterpretable. `collateral_pad_rms` is the one
recovery figure that signals a real defect: it means parts nothing had damaged
were moved (run 9: 0.000 to 1.171 mm; run 10: 0.000 to 3.670 mm).

## Staging a perturbed subject (#411 recovery rig)

A recovery experiment measures how close a repaired placement lands to the
original, and that number means nothing if the tools could have read the
original. So the staging has exactly one rule:

**Ground truth never enters the work dir. Stage it into a SIBLING `_truth/`
from the start, and audit by CONTENT before the run.**

```
wk/<run>/<subject>/         <- THE WORK DIR. every tool runs here.
    board.kicad_pcb         <- the damaged board, the only input
wk/<run>/_truth/<subject>/  <- fenced, OUTSIDE the work dir. nothing reads it.
    control.kicad_pcb       <- the human placement, pose for pose
    board.perturb.json      <- the record: it embeds `original_poses`
```

`placement.perturb.perturb(..., control_out=...)` puts both there in one call —
the record follows the control — and prints where each landed:

```python
P.perturb(src, 'wk/run12/tigard/board.kicad_pcb',
          kind=kind, dose_mm=dose, seed=seed,
          control_out='wk/run12/_truth/tigard/control.kicad_pcb')
```

**Omitting `control_out` is the unsafe default and is kept only for
compatibility.** It writes `<out>.control.kicad_pcb` beside the damaged board —
the human placement, inside the directory the run then works in, where any glob
or `--before` can reach it. `perturb()` prints a WARNING when it does this;
`tests/stress/perturb_batch.py` shows the fenced form (`_truth/` a sibling of
the dose cells).

Audit before the run and again after, by content, never by name:

```bash
python3 -X utf8 tests/stress/fence_audit.py \
    --control wk/run12/_truth/tigard/control.kicad_pcb \
    --workdir wk/run12/tigard --mode create        # exit 4 == a leak
# ... the run ...
python3 -X utf8 tests/stress/fence_audit.py \
    --control wk/run12/_truth/tigard/control.kicad_pcb \
    --workdir wk/run12/tigard --mode audit
```

`--mode create` writes `.fence-manifest.json`, which is what later tells a
*recovered* board (produced by the run, reaching truth — the experiment
succeeding) apart from a *leaked* one (present at creation). There is no
name-based exemption for `*.control.kicad_pcb`: a control inside the work dir is
a leak whatever it is called, because the next carrier will have a different
name.

**And none for `*.perturb.json` either — `DEFAULT_ALLOW` is empty.** The audit
now opens `.json` files and reads `original_poses` out of them, so a
perturbation record is caught by content like anything else. That exemption
existed while the scan walked only `.kicad_pcb`, i.e. while it exempted a file
the tool could not open; making the scan live turned it into a working blind
spot. This is the *default* path, not a corner case — `perturb()` without
`control_out` writes the record beside the damaged board, inside the fence, and
that record embeds the human placement pose-for-pose. Stage with `control_out=`
pointing at a **sibling** `_truth/` (never a child of the work dir — the audit
recurses into it) and neither file ever enters. The record test reads bytes, so
re-saving it as UTF-16 does not evade it.

### Staging blind, in one call

`tests/stress/stage_blind.py` does the whole staging above and draws the
perturbation itself, so the operator never learns the kind, dose, block or
seed:

```bash
python3 -X utf8 tests/stress/stage_blind.py \
    kicad_files/tigard.kicad_pcb wk/run12/tigard wk/run12/_truth/tigard
```

It also SANITISES the project it carries into the work dir. The project must
travel or the board grades at the stock netclass (#441), but KiCad writes
`meta.filename` into it and `pcbnew.last_paths` holds the author's own
directories, so a verbatim copy puts the source board's NAME inside the fence.
The declaration of what it withheld is written to `_truth/draw.json` under
`staged_project`, not into the work dir, because naming the withheld strings
inside the fence would be the leak itself.

### Auditing that every pose came from the engine

`fence_audit` answers "did the answer key get in". It cannot answer "did a
human place this by hand", because a hand-placed board is not the control's
placement either. That is a separate question with a separate instrument:

```bash
python3 -X utf8 tests/stress/provenance_audit.py --workdir wk/run12/tigard
# 0 CLEAN   every moved pose traces to a registered lever, and is where that
#           lever put it
# 4 VIOLATION a moved pose has no lever, or drifted after the lever wrote it
# 5 UNPROVEN  nothing was measured (no regime, or no board)
```

Arm it by staging the work dir with `placement.provenance.start_regime`; the
CLIs in `LEVER_REGISTRY` then record every pose they write to
`.pose-provenance.jsonl`, and an undeclared write RAISES instead of landing.

### Watching a long run

`tests/stress/run_watch.py` has two modes, both of which emit one stdout line
per event so they can be armed once and left alone:

```bash
python3 -X utf8 tests/stress/run_watch.py bugs   --workdir wk/run12/tigard
python3 -X utf8 tests/stress/run_watch.py cheats --workdir wk/run12/tigard \
    --truthdir wk/run12/_truth/tigard --done wk/run12/tigard/DONE
```

`bugs` reports new problems as they appear and runs until you stop it.
`cheats` reports the ways the run could report success without earning it (a
scope narrowed to the failing nets, a grader floor overridden, a waiver spent)
and ends when the `DONE` marker appears, running `fence_audit` and
`provenance_audit` as it goes. Neither budgets on a clock.

`cheats` reads a tool's argv from its `CMD:` banner line. The two skill
drivers install no banner, so wrap timed invocations in
`tests/stress/tee_cmd.py`, which tees the output and appends one
`cmd_timing.jsonl` row per invocation carrying the argv, the exit code and the
elapsed time:

```bash
python3 -X utf8 tests/stress/tee_cmd.py --workdir wk/run12/tigard \
    route4 -- python3 -X utf8 py_router/route.py in.kicad_pcb out.kicad_pcb
```

Wait on `logs/<label>.done`, which appears exactly when the child exits and
holds its exit code. Nothing else is a completion signal.
