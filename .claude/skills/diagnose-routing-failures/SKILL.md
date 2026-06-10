---
name: diagnose-routing-failures
description: Root-causes failed routes from a routing run's logs and the board file. Parses the JSON summary, failed-net histories, and blocking reports, correlates failures spatially with board regions and components, classifies the failure mode, and outputs a ready-to-run retry command. Use after route.py/route_diff.py runs report failed nets.
---

# Diagnose Routing Failures

When this skill is invoked with a board file and routing log file(s), root-cause the failures and produce a targeted retry command. Routing runs should always be captured with `2>&1 | tee /tmp/<step>.txt` so the logs are available.

## Step 1: Parse the Structured Outputs

### JSON summary

```bash
grep "JSON_SUMMARY" /tmp/route_output.txt | sed 's/.*JSON_SUMMARY: //' | python3 -m json.tool
```

Key fields:
- `failed_single`: failed single-ended net names
- `failed_multipoint`: nets with unconnected pads, including pad coordinates
- `multipoint_pads_connected` vs `multipoint_pads_total`: connection success rate

### Failed net histories

```bash
grep -A 30 "FAILED NET HISTORIES" /tmp/route_output.txt
```

Each failed net's history records its route attempts, which nets ripped it up (`ripped_by`, with escalation level N), and reroute outcomes. A net that was ripped and never successfully rerouted points at the net that ripped it as much as at the failed net itself.

### Blocking reports

Look for blocking analysis lines:

```bash
grep -B2 -A8 -i "blocked by\|no rippable blockers\|Route stuck" /tmp/route_output.txt
```

These name the previously-routed nets occupying the failed route's frontier and where the search got stuck (position + layer).

## Step 2: Correlate Failures Spatially

Load the board and map failures to regions and components:

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('board.kicad_pcb')

# For each failed net: where are its pads, and what is nearby?
for net in pcb.nets.values():
    if net.name in failed_names:
        for pad in net.pads:
            print(net.name, pad.component_ref, pad.global_x, pad.global_y, pad.layers)
```

Check whether failures cluster:
- Near one component (especially a BGA/PGA — its auto-detected exclusion zone may be walling off paths)
- In one board region (a congested corridor every failed net needs to cross)
- On one layer (layer costs or direction preference starving that layer)

Report the spatial pattern explicitly, e.g. "all 6 failures have an endpoint within 3mm of U9's southeast corner."

## Step 3: Classify and Recommend

Work through the failure modes most-targeted-first. Do not jump straight to global parameter increases — they are the last resort, not the first.

| Evidence | Failure mode | Fix |
|----------|--------------|-----|
| "no rippable blockers found" near a BGA/PGA | Blocked by the BGA exclusion zone or pre-existing copper | `--no-bga-zone <ref>` for that component |
| Stuck positions cluster in one corridor; blockers are routed nets that themselves had few alternatives | Corridor congestion | Suggest the user **draw a guide corridor**: a polyline on `User.1` through a less congested region, then re-route the failed nets with `--guide-corridor`. Describe in words where the corridor should go (e.g. "south of J3, between the mounting hole and C14"). **Do not draw the geometry yourself** — guide drawing is the user's decision |
| Failed nets have source and target stubs on conflicting layers; rip-up histories show repeated mutual ripping | Same-layer crossing conflicts | `--mps-layer-swap`, or revisit `--layer-costs` |
| "Re-route FAILED: no path found" for ripped nets | Retry budget exhausted | Increase `--max-iterations` (e.g. 1000000) |
| Many multipoint pads failed on the same fine-pitch component | Grid too coarse for the pad geometry | `--grid-step 0.05`, and check `--track-width`/`--clearance` against the pad pitch |
| Failures spread across the board, blockers vary | Genuine capacity problem | Escalate in order: `--max-ripup 10` → reduce `--clearance`/`--track-width` toward fab minimums → add routing layers |

## Step 4: Output the Retry Command

Produce one command that retries **only the failed nets** with the targeted fixes, and explain each parameter change in one line:

```bash
python3 -X utf8 route.py board_routed.kicad_pcb board_retry.kicad_pcb \
    --nets "SDA" "Net-(U1-Pad8)" \
    --no-bga-zone U9 \
    --max-ripup 10 \
    2>&1 | tee /tmp/route_retry.txt
```

Routing only the failed nets preserves the successful routes (already in the input file) and is much faster than a full re-run. After the retry, re-check with `check_connected.py`, and if nets still fail, repeat the diagnosis on the new log — the failure mode often changes after the first fix.
