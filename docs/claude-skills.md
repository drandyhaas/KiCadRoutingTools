# Claude Code Skills

The repository ships seven [Claude Code](https://claude.ai/claude-code) skills in `.claude/skills/`. They combine the project's deterministic Python tooling (`kicad_parser`, `list_nets.py`, `analyze_power_paths.py`, the checkers) with AI judgment where it's actually needed — reading datasheets, classifying components, planning workflows, and diagnosing failures. Outputs are formatted as ready-to-use CLI arguments so they feed straight back into the routing tools.

## Using the Skills

Skills are discovered from `.claude/skills/` in the directory Claude Code is launched from, so run it from the repository checkout, and pass board files by absolute path if they live elsewhere:

```bash
cd /path/to/KiCadRoutingTools
claude
> /plan-pcb-routing /absolute/path/to/my_board.kicad_pcb
```

Skills that look up datasheets (`/analyze-power-nets`, `/find-high-speed-nets`, `/identify-diff-pairs`, `/recommend-stackup`) use WebSearch and can take a few minutes on boards with many unique ICs.

## The Skills

| Skill | Purpose | Main output |
|-------|---------|-------------|
| [`/plan-pcb-routing`](#plan-pcb-routing) | Full board analysis and step-by-step routing plan | Ordered routing commands, executed on approval |
| [`/analyze-power-nets`](#analyze-power-nets) | Identify power nets and track widths via datasheets | `--power-nets` / `--power-nets-widths` arguments |
| [`/find-high-speed-nets`](#find-high-speed-nets) | Classify nets by speed tier via datasheets | `--gnd-via-distance` recommendation |
| [`/identify-diff-pairs`](#identify-diff-pairs) | Find diff pairs by pin function, not just naming | Per-interface `route_diff.py` commands |
| [`/recommend-stackup`](#recommend-stackup) | Review/recommend the board stackup | Stackup table + `--impedance`/plane-layer arguments |
| [`/diagnose-routing-failures`](#diagnose-routing-failures) | Root-cause failed routes from logs + board | Targeted retry command for the failed nets |
| [`/review-routed-board`](#review-routed-board) | Post-route QA and sign-off | Pass/fail report with next actions |

### /plan-pcb-routing

The orchestrator. Analyzes the board structure, identifies components needing fanout (BGA/QFN/QFP/PGA, with actual pad-depth analysis for hollow-center packages), detects differential pairs and DDR length-matching groups, categorizes power/ground nets, and assesses signal speeds. Produces a numbered plan — planes first, then fanout, diff pairs, signals, GND return vias, plane repair, verification — with each command explained, then runs it on approval. Defers to the other six skills at the appropriate points rather than duplicating their logic.

Note: guide corridors (`User.1` polylines) and keepout zones (`User.2` polygons) are **user-drawn** — the skill suggests in words where they would help, but never draws the geometry itself.

### /analyze-power-nets

Identifies power nets and recommends track widths. Auto-classifies obvious components (R/C/L as shunts), looks up datasheets for the rest, assigns each a role (power source, current sink, pass-through, shunt), traces current paths from sinks to sources, and emits ready-to-paste `--power-nets` / `--power-nets-widths` configurations. See [Power Net Analysis](power-nets.md) for the full documentation of the underlying analysis.

### /find-high-speed-nets

Classifies nets into speed tiers (ultra-high / high / medium / low) by looking up component datasheets for max interface frequencies and rise times, tracing signals through series passives. Recommends `--gnd-via-distance` values for GND return via placement (see the [planes documentation](route-plane.md)). `/plan-pcb-routing` includes a lightweight name-pattern-only version of this analysis; run this skill first for datasheet-accurate numbers.

### /identify-diff-pairs

Finds differential pairs by **pin function** rather than net naming: checks pad `pinfunction` metadata, looks up datasheet pinouts for high-speed ICs, and traces through series passives (AC coupling caps, termination resistors). Catches pairs whose net names don't follow P/N conventions and flags name-based false positives. Recommends per-interface parameters — differential impedance (USB 90Ω, LVDS 100Ω, PCIe 85Ω, …), `--diff-pair-gap`, and whether `--diff-pair-intra-match` matters — and outputs `route_diff.py` commands grouped by interface.

### /recommend-stackup

Reviews the board's stackup and flags untouched KiCad defaults — which silently skew `--impedance` width calculations and `--time-matching` delays. Gathers impedance targets from the board's interfaces, looks up the user's fab's standard stackups, and recommends layer roles and dielectric thicknesses, validated with the project's own IPC-2141 formulas in `impedance.py` so the recommendation matches what the router will compute. Outputs the resulting routing arguments; never modifies the board file.

### /diagnose-routing-failures

Root-causes failed routes after a `route.py` / `route_diff.py` run. Parses the `JSON_SUMMARY`, failed-net histories, and blocking reports from the captured logs, correlates failures spatially with board regions and components, classifies the failure mode (BGA exclusion zone, corridor congestion, layer conflicts, budget exhaustion, grid-vs-pitch, genuine capacity), and outputs a targeted retry command for just the failed nets — most-targeted fix first, global parameter increases last.

### /review-routed-board

Post-route QA on any routed board (from this tool, by hand, or another router). Runs the DRC/connectivity/orphan-stub checkers (noting that zone copper needs KiCad's own `kicad-cli pcb drc`), verifies length/time-match groups landed within tolerance, checks GND return via coverage on high-speed nets, and reviews diff-pair gaps and polarity. Produces a compact pass/fail sign-off report ending with concrete next actions.

## How They Fit Together

```
Before routing:
  /recommend-stackup        - fix the stackup before impedance work
  /identify-diff-pairs      - confirm pairs, get gap/impedance
  /analyze-power-nets       - power net widths / plane decisions
  /find-high-speed-nets     - GND return via distances
        |
        v
  /plan-pcb-routing         - builds and runs the routing plan
        |                     (consumes all of the above)
        v
  routing runs (route.py, route_diff.py, route_planes.py)
        |
        +-- failures? --> /diagnose-routing-failures --> targeted retry
        |
        v
  /review-routed-board      - final sign-off
```

Each skill also works standalone — you don't need the full plan flow to ask one question of one board.
