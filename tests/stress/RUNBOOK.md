# Router Stress-Test Runbook (per board)

You are stress-testing the KiCadRoutingTools autorouter on one real-world
open-source board. Follow the plan-pcb-routing skill methodology
non-interactively and record everything.

## Orchestration — drive the whole corpus with run_queue.sh

The whole set-1 + set-2 corpus is driven by ONE queue manager:

```bash
bash tests/stress/run_queue.sh [concurrency=4] [model=sonnet]
```

It keeps N headless `claude -p` board workers in flight until every board has a
results JSON, deriving ALL state from disk — so it's safe to Ctrl-C and restart
(it skips finished boards and won't double-launch running ones). Each worker is
`tests/stress/run_board.sh <board> <set> [model]`: a non-interactive agent that
follows THIS runbook, writes the results JSON + `FINDINGS.md` into the run dir,
and drops a `.worker_done` marker (`ok`/`NORESULT`). This replaces the older
manual approach (a parent launching one background `Agent` per board and
refilling on notifications) — the queue manager removes the drop-/stale-prone
notification stream entirely.

**Prereqs on a fresh machine:**
- Build the corpus first (README → "Pipeline" and "Two 15-board sets").
- Workers run `claude -p --dangerously-skip-permissions`, which the harness
  blocks by default. Authorize it once: add a Bash allow-rule for
  `bash tests/stress/run_board.sh:*` and `bash tests/stress/run_queue.sh:*`
  to your local `.claude/settings.local.json` (gitignored, so it is NOT
  inherited from a checkout — each user opts in themselves), or approve when
  prompted.

**Monitoring:** `bash tests/stress/stress_status.sh` — prints DONE/RUNNING/TODO
across all 30 boards plus free slots. (For detail: `QUEUE_STATUS.txt` is the
manager heartbeat; `runs[_set2]/<board>/worker.log` is a worker's log — note
`claude -p` buffers its own stdout to the end, but the per-tool `*.log` files in
the run dir update live.)

**State signals** (what the queue and `stress_status.sh` use):
- DONE: results JSON exists.
- RUNNING: a process matches the run-dir path, OR the run dir was touched <15 min
  ago (covers the gap between a worker's commands). Naive checks mislead — pgrep
  on tool names is noisy and case-sensitive (KiCad's interpreter is `Python`,
  wrapped by run_limited.sh), and run-dir mtimes go quiet during a long route —
  so don't rely on those alone.
- LOST: no results JSON AND not running AND run dir idle ~15+ min → safe to relaunch.

### Manual fallback (driving by hand, no queue script)

If you must drive without `run_queue.sh` (e.g. orchestrating from a chat
session), launch one background board worker/agent at a time and keep **4 in
flight**, refilling off `stress_status.sh` — NOT the notification stream, which
drops and duplicates. Order the queue ~2 heavy + 2 light to stay under the
4 GB-per-job RAM cap (heavy = dense 4-layer / many pads / BGA fanout). Derive
state from disk, never from tracked agent IDs (they are lost on context
summarization).

### Clean restart (re-run the whole corpus from scratch)

Don't delete prior results — archive them. Move `results/`, `results_set2/`,
`runs/`, `runs_set2/` to `*_archive_<timestamp>/`, then recreate the four as
empty dirs. The originals (`boards/`, `boards_set2/`) and the stripped
inputs (`boards_unrouted/`, `boards_unrouted_set2/`) are never touched. Also
sweep stray top-level `runs_*T0*` temp files. After that, the status script
reports 0/30 DONE and the recipe above drives the rest.

## Paths

Two corpora share this harness; pick the SET's paths and stay consistent
within a board. `<SET>` below is empty for set 1 and `_set2` for set 2.


- Tools repo (READ-ONLY — never write/modify anything here):
  `/Users/andy/Documents/KiCadRoutingTools`
- Skill to follow:
  `/Users/andy/Documents/KiCadRoutingTools/.claude/skills/plan-pcb-routing/SKILL.md`
- Input board: `~/Documents/kicad_stress_test/boards_unrouted<SET>/<BOARD>.kicad_pcb`
- Your working dir (create it): `~/Documents/kicad_stress_test/runs<SET>/<BOARD>/`
  ALL outputs, intermediates, and logs go here (NOT /tmp — parallel runs collide).
- Original (compare + ground-truth DRC): `~/Documents/kicad_stress_test/boards<SET>/<BOARD>.kicad_pcb`
- Final results JSON: `~/Documents/kicad_stress_test/results<SET>/<BOARD>.json`

  (set 1 → `boards_unrouted/`, `runs/`, `boards/`, `results/`;
   set 2 → `boards_unrouted_set2/`, `runs_set2/`, `boards_set2/`, `results_set2/`)

## Rules

1. Invoke all tools as `python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/<tool>.py ...`
   from your working dir. Tee every command's output to a log file in your run dir.
   MEMORY CAP (mandatory): prefix EVERY routing/fanout/plane/check command with
   the watchdog wrapper, e.g.
   `bash /Users/andy/Documents/KiCadRoutingTools/tests/stress/run_limited.sh python3 -X utf8 .../route.py ... 2>&1 | tee step.log`
   It kills the job at ~4 GB RSS (exit 137, `MEMORY_LIMIT_EXCEEDED` on stderr).
   Up to 4 boards run concurrently — in practice most jobs sit well under the
   4 GB cap most of the time, so 4-in-flight is fine on an 8 GB machine; the
   per-job watchdog still backstops any board that spikes. Keep an eye on RAM.
   If a step is killed by the cap, that is an important finding: record it in
   `issues` (with the step and board), then try ONE cheaper variant (e.g.
   default `--max-iterations`, no retry round, or fewer nets); if that also
   blows the cap, mark the step as OOM and move on.
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
     AND to route_disconnected_planes (its per-pad repair connects the fine-pitch
     plane balls). Keep the standard via for general route.py and the bulk
     route_planes pour (#99/#122).
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
   Always exclude plane nets from fanout/routing with `"!GND"`-style patterns
   (match the board's actual net names, e.g. "!/GND", "!AGND" — check the
   power listing first).
5. Fanout: only for BGA/PGA/QFN/QFP components per the skill's depth rule.
   Through-hole connectors/DIPs don't need fanout.
   FIX VERIFICATION (issues #79/#80, fixed): fanout tools now write name-style
   net refs on KiCad 10 boards and the parser merges mixed styles. After each
   fanout step, still run
   `python3 /Users/andy/Documents/KiCadRoutingTools/tests/stress/fix_mixed_net_refs.py <fanout_output.kicad_pcb>`
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
   BASELINE: before routing, run check_drc.py on the unrouted input at the same
   floor and record the count — real boards have pre-existing pad-to-pad
   proximity that is not the router's fault. Report post-route DRC as total AND delta.
   GROUND-TRUTH BASELINE (do this too): run `check_drc.py
   ~/Documents/kicad_stress_test/boards/<board>.kicad_pcb --clearance <floor>
   --hole-to-hole-clearance <floor>` on the ORIGINAL human-routed board (SAME
   floor flags) and record that count as `drc.original_routed_violations`. That —
   not 0 — is the achievable floor under our DRC model (the originals carry a few
   clearance-independent hole-to-hole/pad violations: e.g. megadesk 1, piantor 6).
   Judge our routing's DRC against the ORIGINAL's count at the same floor, not against 0.
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
   difficult-board params (`--max-ripup 10 --max-iterations 1000000
   --no-bga-zone`).
10. One retry round allowed: if routing fails some nets, re-run the failed nets
   per the skill's "Diagnose and Retry" table (use the same output->input
   chaining). Record both attempts.
11. Verification (always, on the final board):
    - `check_drc.py <final> --clearance <floor> --hole-to-hole-clearance <floor> 2>&1 | tee drc.log`
      (manufacturing floor from `--design-rules`, per step 7; note the flags used)
    - `check_connected.py <final> 2>&1 | tee connectivity.log`
    - `check_orphan_stubs.py <final> 2>&1 | tee orphans.log`
11b. COMPARE-TO-ORIGINAL (always, final step): run
    `python3 /Users/andy/Documents/KiCadRoutingTools/tests/stress/compare_to_original.py
     --ours <final> --orig ~/Documents/kicad_stress_test/boards/<board>.kicad_pcb
     --json 2>&1 | tee compare.log`
    It contrasts OUR routing with the human-routed original (vias, total copper
    length, track-width strategy, layer balance, nets-with-copper) and prints
    SUGGESTIONS for what to change in our routing/approach. Fold its
    `JSON_COMPARISON` blob into the results JSON `comparison` field and copy its
    suggestion lines verbatim into the `suggestions` field. The original is the
    ground truth for a manufacturable board; treat large via/length/width/
    layer-balance gaps as router-improvement findings, not just board facts.
12. Budget: ~45 min wall-clock for the whole board, and a HARD 20-minute cap
    per command: if a single tool invocation passes 20 min — even with its log
    still growing — kill it, record the elapsed time + command as a runtime
    finding, and continue with the previous step's output. Aggressive params
    (--max-iterations 1000000 with --max-ripup 10 at fine grids) can grind
    for hours; that is a finding, not progress. NEVER end your turn while
    a routing command is still running — you will be terminated and the run
    orphaned. Run commands in the FOREGROUND (timeout up to 600000 ms). If a
    command exceeds the 10-min foreground cap, keep waiting in foreground:
    repeatedly run `until ! pgrep -f "<unique-cmd-fragment>" >/dev/null; do
    sleep 10; done` (each up to 10 min) until the process exits, then read its
    log and continue. If a command shows no log growth for >15 min, kill it
    and record it as a hang.
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
