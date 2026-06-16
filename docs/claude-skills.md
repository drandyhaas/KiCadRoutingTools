# Claude Code Skills

The repository ships eight [Claude Code](https://claude.ai/claude-code) skills in `.claude/skills/`. They combine the project's deterministic Python tooling (`kicad_parser`, `list_nets.py`, `analyze_power_paths.py`, the checkers) with AI judgment where it's actually needed — reading datasheets, classifying components, planning workflows, and diagnosing failures. Outputs are formatted as ready-to-use CLI arguments so they feed straight back into the routing tools.

## Using the Skills

Skills are discovered from `.claude/skills/` in the directory Claude Code is launched from, so run it from the repository checkout, and pass board files by absolute path if they live elsewhere:

```bash
cd /path/to/KiCadRoutingTools
claude
> /plan-pcb-routing /absolute/path/to/my_board.kicad_pcb
```

Skills that look up datasheets (`/analyze-power-nets`, `/find-high-speed-nets`, `/identify-diff-pairs`, `/recommend-stackup`, `/recommend-plane-mappings`) use WebSearch and can take a few minutes on boards with many unique ICs.

## Plugin GUI Integration

<p align="center">
  <img src="claude_tab.png" alt="Claude tab: planned steps, controls, and live transcript" width="700">
</p>

All eight skills are reachable from the plugin's routing dialog (the GUI spawns
`claude` headless with the working directory set to the plugin root, streams a live
transcript, and parses a machine-readable `RESULT=` last line back into controls):

| Where | Button | Skill | What it fills / shows |
|-------|--------|-------|------------------------|
| Claude tab | **Plan Routing** | `/plan-pcb-routing` | Fills parameters across the tabs and loads a checkable step list; **Run Selected Steps** executes them in-process |
| Claude tab | **Review Routed Board** | `/review-routed-board` | QA report in the transcript with a PASS/FAIL verdict |
| Claude tab | **Diagnose Routing Failures** | `/diagnose-routing-failures` | Root-cause report from the board + the Log tab content |
| Basic tab (Layers) | **Check Stackup (Claude)** | `/recommend-stackup` | Stackup report; recommended layer count logged |
| Basic tab (Options) | **Ask Claude** (Power Nets) | `/analyze-power-nets` | Fills the Power Nets / Power Widths fields |
| Differential tab | **Ask Claude** | `/identify-diff-pairs` | Checks confirmed pairs, unchecks name-matching false positives; unconventional pairs reported in the log |
| Planes tab | **Ask Claude** (assignments) | `/recommend-plane-mappings` | Fills the net → layer assignment list (replace/merge prompt) |
| Planes tab | **Ask Claude** (GND vias) | `/find-high-speed-nets` | Fills the GND via Max Distance field |

The Claude tab's **Model** and **Effort** dropdowns apply to every button above and
persist with the other dialog settings. Each run shows a startup header (Claude Code
version, model, discovered skills) so you can confirm what actually ran.

## The Skills

| Skill | Purpose | Main output |
|-------|---------|-------------|
| [`/plan-pcb-routing`](#plan-pcb-routing) | Full board analysis and step-by-step routing plan | Ordered routing commands, executed on approval |
| [`/analyze-power-nets`](#analyze-power-nets) | Identify power nets and track widths via datasheets | `--power-nets` / `--power-nets-widths` arguments |
| [`/find-high-speed-nets`](#find-high-speed-nets) | Classify nets by speed tier via datasheets | `--gnd-via-distance` recommendation |
| [`/identify-diff-pairs`](#identify-diff-pairs) | Find diff pairs by pin function, not just naming | Per-interface `route_diff.py` commands |
| [`/recommend-stackup`](#recommend-stackup) | Review/recommend the board stackup | Stackup table + `--impedance`/plane-layer arguments |
| [`/recommend-plane-mappings`](#recommend-plane-mappings) | Recommend net → plane-layer assignments with SI rationale | Assignment list for the Planes tab / `route_planes.py` arguments |
| [`/diagnose-routing-failures`](#diagnose-routing-failures) | Root-cause failed routes from logs + board | Targeted retry command for the failed nets |
| [`/review-routed-board`](#review-routed-board) | Post-route QA and sign-off | Pass/fail report with next actions |
| [`/stress-test-router`](#stress-test-router) | Batch-test the router on real-world open-source boards | Completion/DRC summary table + GitHub issues for findings |

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

### /recommend-plane-mappings

Recommends which nets deserve copper planes and on which layers. Reads the stackup and existing zones, identifies plane-worthy nets by pad count and datasheet current estimates (the `/analyze-power-nets` approach), and assigns layers with signal-integrity rationale: GND adjacent to the primary signal layers for return paths, power planes paired against GND for interplane capacitance, and split layers for multiple low-current rails (flagging the seams). On 2-layer boards it recommends pours and points at `/recommend-stackup` when the signal content justifies four layers. Ends with a machine-readable `RESULT=GND:In1.Cu;VCC:In2.Cu` line that maps 1:1 onto the Planes tab's assignment list.

### /diagnose-routing-failures

Root-causes failed routes after a `route.py` / `route_diff.py` run. Parses the `JSON_SUMMARY`, failed-net histories, and blocking reports from the captured logs, correlates failures spatially with board regions and components, classifies the failure mode (BGA exclusion zone, corridor congestion, layer conflicts, budget exhaustion, grid-vs-pitch, genuine capacity), and outputs a targeted retry command for just the failed nets — most-targeted fix first, global parameter increases last.

### /review-routed-board

Post-route QA on any routed board (from this tool, by hand, or another router). Runs the DRC/connectivity/orphan-stub checkers (noting that zone copper needs KiCad's own `kicad-cli pcb drc`), verifies length/time-match groups landed within tolerance, checks GND return via coverage on high-speed nets, and reviews diff-pair gaps and polarity. Produces a compact pass/fail sign-off report ending with concrete next actions.

### /stress-test-router

A development/QA skill rather than a board-design skill: batch-tests the router against real-world open-source KiCad boards of varying complexity (simple keyboards up to 6-layer SoC carriers). Drives the `tests/stress/` harness — downloads the corpus from upstream GitHub projects (boards are never checked into the repo), normalizes via a `pcbnew` round-trip, strips all routing, then routes each board end-to-end with the `/plan-pcb-routing` workflow under strict resource limits (~4 GB per job, 4 boards concurrent via the `run_queue.sh` manager). Aggregates per-board completion rates, DRC deltas vs the unrouted baseline, connectivity verdicts, and crash/hang findings into a summary table, deduplicates findings against existing GitHub issues, and files new issues after user approval. See `tests/README.md` and `tests/stress/README.md` for the harness internals.

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
