# Router Stress-Test Runbook (per board)

You are stress-testing the KiCadRoutingTools autorouter on one real-world
open-source board. Follow the plan-pcb-routing skill methodology
non-interactively and record everything.

## Paths

- Tools repo (READ-ONLY — never write/modify anything here):
  `/Users/andy/Documents/KiCadRoutingTools`
- Skill to follow:
  `/Users/andy/Documents/KiCadRoutingTools/.claude/skills/plan-pcb-routing/SKILL.md`
- Input board: `~/Documents/kicad_stress_test/boards_unrouted/<BOARD>.kicad_pcb`
- Your working dir (create it): `~/Documents/kicad_stress_test/runs/<BOARD>/`
  ALL outputs, intermediates, and logs go here (NOT /tmp — parallel runs collide).
- Final results JSON: `~/Documents/kicad_stress_test/results/<BOARD>.json`

## Rules

1. Invoke all tools as `python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/<tool>.py ...`
   from your working dir. Tee every command's output to a log file in your run dir.
   MEMORY CAP (mandatory): prefix EVERY routing/fanout/plane/check command with
   the watchdog wrapper, e.g.
   `bash ~/Documents/kicad_stress_test/scripts/run_limited.sh python3 -X utf8 .../route.py ... 2>&1 | tee step.log`
   It kills the job at ~1 GB RSS (exit 137, `MEMORY_LIMIT_EXCEEDED` on stderr).
   If a step is killed by the cap, that is an important finding: record it in
   `issues` (with the step and board), then try ONE cheaper variant (e.g.
   default `--max-iterations`, no retry round, or fewer nets); if that also
   blows the cap, mark the step as OOM and move on.
2. Follow SKILL.md's analysis steps (board stats, layers, fanout candidates,
   diff pairs via `list_nets.py <board> --diff-pairs --power`, power strategy,
   plan generation) but DO NOT invoke other skills and DO NOT ask the user
   anything — use the skill's inline name-pattern heuristics and your judgment.
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
   KNOWN BUG WORKAROUND (mandatory): immediately after EVERY bga_fanout.py or
   qfn_fanout.py step, run
   `python3 ~/Documents/kicad_stress_test/scripts/fix_mixed_net_refs.py <fanout_output.kicad_pcb>`
   before any further tool touches the file. The fanout tools write
   KiCad-9-style numeric net refs into KiCad 10 boards and the parser then
   silently drops all name-style segments (known finding — do not re-report,
   but DO note in issues if the normalizer reports rewrites, with the count).
6. Diff pairs: if `--diff-pairs` reports pairs, route them with route_diff.py
   AFTER fanout and BEFORE signal routing (gap 0.15 default; use --no-gnd-vias).
   If pair detection looks like a false positive (e.g. random net names that
   happen to end in P/N), note it as a finding and skip those.
7. BASELINE: before routing, run check_drc.py on the unrouted input and record
   the violation count — real boards have pre-existing pad-to-pad proximity
   that is not the router's fault. Report post-route DRC as total AND delta.
8. KNOWN OOM (mandatory check): obstacle-map memory scales with
   grid_cells x outline_vertices (known finding - do not re-report, but DO
   record any MEMORY_LIMIT_EXCEEDED kill with its command). Before routing,
   estimate: cells = board_area_mm2 / grid_step^2, bytes = cells x
   (outline+cutout vertices) x 8. If the estimate exceeds ~0.8 GB, start
   with `--grid-step 0.2` (fine for coarse-pitch boards) and note it in the
   results JSON.
9. Routing params: start with skill defaults. For boards with fine-pitch
   (<0.65mm) components, consider `--grid-step 0.05` and/or smaller clearance
   (record what you chose and why). For dense/2-layer boards use the skill's
   difficult-board params (`--max-ripup 10 --max-iterations 1000000
   --no-bga-zone`).
10. One retry round allowed: if routing fails some nets, re-run the failed nets
   per the skill's "Diagnose and Retry" table (use the same output->input
   chaining). Record both attempts.
11. Verification (always, on the final board):
    - `check_drc.py <final> 2>&1 | tee drc.log` (default clearance; note flags)
    - `check_connected.py <final> 2>&1 | tee connectivity.log`
    - `check_orphan_stubs.py <final> 2>&1 | tee orphans.log`
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

## Results JSON schema

```json
{
  "board": "<BOARD>",
  "layers": 2,
  "routable_nets": 0,
  "plan": {"fanout_components": [], "diff_pairs": [], "plane_nets": [], "plane_layers": [], "notes": ""},
  "steps": [{"name": "", "cmd": "", "wall_s": 0, "outcome": ""}],
  "routing": {
    "nets_attempted": 0, "nets_routed": 0, "nets_failed": 0,
    "completion_pct": 0.0,
    "multipoint_pads_connected": 0, "multipoint_pads_total": 0,
    "vias": 0, "route_time_s": 0.0,
    "failed_nets": [], "retry_improved": false
  },
  "diff_pair_routing": {"pairs_attempted": 0, "pairs_routed": 0, "polarity_swaps": 0},
  "planes": {"nets": [], "unconnected_pads": 0, "isolated_regions": 0, "repair_outcome": ""},
  "drc": {"baseline_violations": 0, "final_violations": 0, "delta": 0, "by_type": {}},
  "connectivity": {"fully_connected": false, "detail": ""},
  "orphan_stubs": 0,
  "wall_clock_total_s": 0,
  "issues": ["crash/hang/bogus-output/parser findings, each with 1-3 sentence detail"]
}
```

## Final report (your last message)

Return a compact summary: completion %, DRC delta, connectivity verdict,
plus the `issues` list verbatim. No file dumps.
