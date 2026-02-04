---
name: plan-pcb-routing
description: Analyzes a KiCad PCB file and creates a comprehensive routing plan. Examines components for fanout needs (BGA/QFN/QFP/PGA), identifies differential pairs, categorizes power/ground nets, and presents a step-by-step routing workflow with explanations.
---

# Plan PCB Routing

When this skill is invoked with a KiCad PCB file, perform a comprehensive analysis and present a routing plan to the user.

## Step 1: Load and Analyze PCB Structure

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('path/to/file.kicad_pcb')

# Basic stats
print(f'Total nets: {len(pcb.nets)}')
print(f'Total footprints: {len(pcb.footprints)}')
print(f'Existing segments: {len(pcb.segments)}')
print(f'Existing vias: {len(pcb.vias)}')
```

Report to user:
- Number of nets, components, existing routing
- Whether this is a fresh board or partially routed

## Step 2: Identify Copper Layers

Check the KiCad file directly for layer definitions:

```bash
grep -E "^\s+\([0-9]+ \".*\.Cu\"" path/to/file.kicad_pcb
```

Report to user:
- Available copper layers (F.Cu, B.Cu, In1.Cu, In2.Cu, etc.)
- Whether it's a 2-layer, 4-layer, or multi-layer board

## Step 3: Check for Components Needing Fanout

Identify BGA, QFN, QFP, PGA, and other array packages that benefit from escape routing:

```python
for ref, fp in pcb.footprints.items():
    name_upper = fp.footprint_name.upper()
    pad_count = len(fp.pads)

    # Check for array packages
    needs_fanout = any([
        'BGA' in name_upper,
        'PGA' in name_upper,
        'QFN' in name_upper,
        'QFP' in name_upper,
        'LQFP' in name_upper,
        'TQFP' in name_upper,
    ])

    # Also flag high pin-count components
    if pad_count > 40:
        needs_fanout = True

    if needs_fanout:
        # Analyze pad arrangement
        xs = sorted(set(round(p.local_x, 2) for p in fp.pads))
        ys = sorted(set(round(p.local_y, 2) for p in fp.pads))
        grid_cols, grid_rows = len(xs), len(ys)

        # Check SMD vs through-hole
        smd_count = sum(1 for p in fp.pads if p.drill == 0)
        th_count = sum(1 for p in fp.pads if p.drill > 0)
```

### Fanout Tool Selection

| Package Type | Tool | Notes |
|--------------|------|-------|
| BGA (SMD grid) | `bga_fanout.py` | Escape routing for ball grid arrays |
| PGA (through-hole grid) | `bga_fanout.py` | Same tool works for PGA |
| QFN/QFP (perimeter SMD) | `qfn_fanout.py` | Stub routing for quad flat packages |
| DIP/SOIC (through-hole/SMD rows) | None needed | Standard routing handles these |

### When to Use Fanout for BGA/PGA

**Rule: Use fanout for any BGA/PGA with more than 2 pins depth from outside to center.**

**Important:** Calculate ACTUAL depth by counting pads from the edge toward center, not grid size.
Many PGA/BGA packages (especially FPGAs/CPLDs) have hollow centers with only perimeter pins populated.

To calculate actual depth:
```python
# Check middle column from top edge toward center
mid_col = xs[len(xs)//2]
depth = 0
for y in ys:  # ys sorted from edge
    if (mid_col, y) in pad_positions:
        depth += 1
    else:
        break  # Stop at first empty position
```

Examples:
- 13×13 grid, fully populated → depth = 7 → **USE FANOUT**
- 13×13 grid, hollow center (3 rows populated) → depth = 3 → **USE FANOUT**
- 10×10 grid, hollow center (2 rows populated) → depth = 2 → fanout optional
- 4×4 grid, fully populated → depth = 2 → fanout optional

Inner pins beyond depth 2 cannot escape without fanout routing through channels between outer pins.

Report to user:
- List of components that may need fanout
- Package type, pad count, and grid depth for each
- Recommended fanout tool

## Step 4: Check for Differential Pairs and Power Nets

Use `list_nets.py` to detect differential pairs and power/ground nets:

```bash
python3 list_nets.py path/to/file.kicad_pcb --diff-pairs --power
```

This will output:
- Differential pairs detected (P/N naming conventions)
- Ground nets with pad counts
- Power nets with pad counts

If differential pairs are found:
- List each P/N pair
- Note that `route_diff.py` should be used for these
- Explain that diff pairs maintain consistent spacing and length matching

### Check for DDR/High-Speed Memory Signals

Look for DDR signal patterns in the net list that may need length matching:
- Data signals: DQ0-DQ63
- Strobes: DQS, DQM, DM
- Clocks: CLK, CK

If DDR signals detected:
- Note that `--length-match-group auto` should be used
- DQ0-7 + DQS0 form byte lane 0, DQ8-15 + DQS1 form byte lane 1, etc.

Report to user:
- List of detected differential pairs (or "none found")
- Whether `route_diff.py` is needed
- Whether DDR/length-matching is needed

## Step 5: Review Power and Ground Net Strategy

From the `list_nets.py --power` output, analyze:

### Power Net Routing Strategies

| Net Type | Strategy | Rationale |
|----------|----------|-----------|
| GND (many pads) | `route_planes.py` on bottom/inner layer | Low impedance return path |
| VCC/Power (many pads) | Wide traces (0.5mm+) or plane | Current carrying capacity |
| VCC/Power (few pads) | Wide traces with `--power-nets` | Simpler than plane |

Report to user:
- Identified GND nets and pad counts
- Identified power nets and pad counts
- Recommended strategy (plane vs wide traces)

## Step 6: Generate Routing Plan

Based on the analysis, generate a step-by-step plan. The general order is:

### Routing Order Rationale

1. **Power Planes First** - Create GND and VCC planes together, handles most-connected nets
2. **Fanout** (if needed) - Escape routing before signal routing, exclude plane nets
3. **Differential Pairs** - Route before single-ended to get best paths (if present)
4. **Signal Routing** - All remaining nets
5. **Plane Repair** - Reconnect any broken plane regions
6. **Verification** - DRC and connectivity checks

### Example Plan Output Format

Present the plan to the user as a numbered list with explanations:

```
## Routing Plan for board.kicad_pcb

### Board Summary
- 2-layer board (F.Cu, B.Cu)
- 174 nets, 25 components
- Unrouted (0 existing traces)

### Components Requiring Special Handling
- **U9 (PGA120)**: 120-pin grid array - use bga_fanout.py for signals only

### Differential Pairs
- None detected

### Power/Ground Nets
- **GND**: 42 pads - use plane on B.Cu
- **VCC**: 23 pads - use plane on F.Cu (or wide traces if planes not desired)

---

## Step-by-Step Routing Commands

### Step 1: Create Power Planes (GND and VCC)
Creates power planes in a single call. Each net is paired with its corresponding
layer (GND→B.Cu, VCC→F.Cu). Through-hole PGA/BGA pads automatically connect to
planes on their layer; SMD pads get vias routed to the plane.

python -X utf8 route_planes.py board.kicad_pcb board_step1.kicad_pcb \
    --nets GND VCC \
    --plane-layers B.Cu F.Cu \
    2>&1 | tee /tmp/step1_planes.txt

### Step 2: Fanout U9 (PGA120) - All Non-Plane Nets
Generates escape routing for ALL nets on the component EXCEPT those handled
by power planes. This ensures every signal net gets fanned out, avoiding
`--no-bga-zone` workarounds during routing.

**Important:** Use `"*" "!GND" "!VCC"` to fan out all nets except the power
plane nets. Do NOT use `"/*"` alone, as it misses nets with non-hierarchical
names like `Net-(U9-Pad1)` which would then require `--no-bga-zone` to route.

python -X utf8 bga_fanout.py board_step1.kicad_pcb \
    --component U9 \
    --nets "*" "!GND" "!VCC" \
    --output board_step2.kicad_pcb \
    2>&1 | tee /tmp/step2_fanout.txt

### Step 3: Route All Signal Nets
Routes all remaining unrouted nets. For boards with BGA/PGA components,
use `--no-bga-zone` to allow the router to find alternative paths through
the dense pin area (even when fanout was done, some paths may require this).
Use `--max-ripup 10 --max-iterations 1000000` for difficult 2-layer boards.

python -X utf8 route.py board_step2.kicad_pcb board_step3.kicad_pcb \
    --nets "*" \
    --no-bga-zone \
    --max-ripup 10 \
    --max-iterations 1000000 \
    2>&1 | tee /tmp/step3_routing.txt

### Step 4: Repair Disconnected Plane Regions
Signal traces may have cut through planes. This step reconnects any
isolated copper islands.

python -X utf8 route_disconnected_planes.py board_step3.kicad_pcb board_step4.kicad_pcb \
    2>&1 | tee /tmp/step4_plane_repair.txt

### Step 5: Verify Results
Check for DRC violations, unrouted nets, and orphan stubs.
Save outputs to /tmp for analysis if issues are found.

python -X utf8 check_drc.py board_step4.kicad_pcb --clearance 0.25 2>&1 | tee /tmp/step5_drc.txt
python -X utf8 check_connected.py board_step4.kicad_pcb 2>&1 | tee /tmp/step5_connectivity.txt
python -X utf8 check_orphan_stubs.py board_step4.kicad_pcb 2>&1 | tee /tmp/step5_orphans.txt
```

### Alternative: VCC as Wide Traces (No Plane)

If you prefer not to use a VCC plane, route VCC with wide traces instead:

```
### Step 2 (Alternative): Fanout U9 Including VCC
python -X utf8 bga_fanout.py board_step1.kicad_pcb \
    --component U9 \
    --nets "/*" VCC \
    --output board_step2.kicad_pcb

### Step 3 (Alternative): Route VCC with Wide Traces
python -X utf8 route.py board_step2.kicad_pcb board_step3.kicad_pcb \
    --nets VCC \
    --track-width 0.5
```

Or if VCC wasn't fanned out, use `--no-bga-zone` to allow router access:
```
python -X utf8 route.py board_step2.kicad_pcb board_step3.kicad_pcb \
    --nets VCC \
    --track-width 0.5 \
    --no-bga-zone U9
```

## Step 7: Check for High-Speed Signal Requirements

### Length Matching (DDR, high-speed buses)

For DDR memory or other length-matched buses, detect signals that need matching:

```python
# Common DDR signal patterns
ddr_patterns = ['DQ', 'DQS', 'DQM', 'DM', 'CLK', 'CK', 'CAS', 'RAS', 'WE', 'CS', 'ODT', 'CKE']
ddr_nets = [n.name for n in pcb.nets.values()
            if n.name and any(p in n.name.upper() for p in ddr_patterns)]
```

If DDR or length-matched signals detected, add to the plan:
- `--length-match-group auto` for automatic DDR byte lane grouping
- `--length-match-tolerance 0.1` for acceptable variance (mm)
- `--time-matching` if routes span different layers (accounts for dielectric)

### Impedance-Controlled Routing

For high-speed signals with impedance requirements:
- `--impedance 50` for 50Ω single-ended (calculates width per layer from stackup)
- `--impedance 100` with `route_diff.py` for 100Ω differential

### Bus Detection

For parallel data/address buses with clustered endpoints:
- `--bus` enables automatic bus detection and parallel routing
- Routes are attracted to neighbors, creating clean parallel traces

## Step 8: Handle Special Cases

### 2-Layer Board with Dense Components

On 2-layer boards, BGA/PGA fanout may fail for some inner pins due to
insufficient routing channels. Options:
- Accept partial fanout; router will complete remaining connections
- Skip fanout entirely; direct routing often works for through-hole PGA

**Important:** If you skip fanout for a BGA/PGA component but still need to connect its
internal pads, use `--no-bga-zone <component>` to disable the automatic exclusion zone
and allow the router to enter the dense pin area:

```bash
python3 route.py board.kicad_pcb \
    --nets "*" \
    --no-bga-zone U9 \
    --output board_routed.kicad_pcb
```

Without this flag, the router auto-detects BGA/PGA zones and avoids them, which would
leave internal pads unconnected if they weren't fanned out.

### Multi-Layer Boards (4+ layers)

- Use inner layers for planes (In1.Cu for GND, In2.Cu for VCC)
- More fanout options available
- Use `--layer-costs` to prefer certain layers:
  ```bash
  --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 5.0 5.0 1.0
  ```
  Higher cost = layer is avoided (used only when necessary)

### Differential Pairs Present

Insert diff pair routing after fanout but before single-ended signals:

```bash
python3 route_diff.py board.kicad_pcb \
    --nets "*LVDS*" "*USB*" \
    --diff-pair-gap 0.15 \
    --output board_diff.kicad_pcb
```

Key options:
- `--diff-pair-gap 0.1` - Gap between P and N traces (mm)
- `--no-gnd-vias` - Disable automatic GND via placement near signal vias
- `--diff-pair-intra-match` - Match P/N lengths within each pair
- `--swappable-nets "*rx*"` - Allow target swap optimization for memory lanes

### QFN/QFP Components (Perimeter Pads)

Use `qfn_fanout.py` instead of `bga_fanout.py`:

```bash
python3 qfn_fanout.py board.kicad_pcb \
    --component U1 \
    --output board_qfn.kicad_pcb
```

Creates two-segment stubs (straight + 45° fan) for each pad.

### Power Net Width Options

Instead of routing power separately, use `--power-nets` with signal routing:

```bash
python3 route.py board.kicad_pcb \
    --nets "*" \
    --power-nets "GND" "VCC" "+3.3V" \
    --power-nets-widths 0.5 0.4 0.4 \
    --output board_routed.kicad_pcb
```

First matching pattern determines width. Useful when not using planes.

### GND Return Vias for Signal Integrity

After creating planes, add GND vias near signal vias:

```bash
python3 route_planes.py board.kicad_pcb \
    --nets GND --plane-layers B.Cu \
    --add-gnd-vias --gnd-via-distance 2.0 \
    --output board_gnd.kicad_pcb
```

### Target Swap Optimization (Memory Routing)

For swappable signals (e.g., memory data lanes where any DQ can connect to any):

```bash
python3 route.py board.kicad_pcb \
    --nets "*DQ*" \
    --swappable-nets "*DQ*" \
    --output board_routed.kicad_pcb
```

Uses Hungarian algorithm to find optimal assignments minimizing crossings.

### Schematic Synchronization After Swaps

When routing performs polarity swaps (P↔N) or target swaps, the schematic can get
out of sync with the PCB. Use `--schematic-dir` to automatically update:

```bash
python3 route_diff.py board.kicad_pcb \
    --nets "*LVDS*" \
    --swappable-nets "*LVDS*" \
    --schematic-dir /path/to/kicad/project \
    --output board_routed.kicad_pcb
```

This updates the `.kicad_sch` files with any pad swaps made during routing.

**Important:** After routing with swaps, ask the user:
> "The router performed X polarity swaps and Y target swaps. Would you like to
> update the schematic to match? If so, provide the path to your KiCad project
> directory and I'll re-run with `--schematic-dir`."

Schematic sync is **disabled by default** to avoid unexpected changes. Only enable
when the user confirms they want schematic updates.

### Net Ordering Strategies

| Strategy | Flag | Best For |
|----------|------|----------|
| MPS (default) | `--ordering mps` | General routing, minimizes crossings |
| Inside-Out | `--ordering inside_out` | BGA escape routing |
| Original | `--ordering original` | Manual control |

### Useful Utility Scripts

| Script | Purpose |
|--------|---------|
| `list_nets.py U1` | List all nets connected to a component |
| `list_nets.py U1 --pads` | Show pad-to-net assignments |
| `check_orphan_stubs.py` | Find traces ending without connection |

### Debug and Visualization Options

When routing fails or behaves unexpectedly:

```bash
# Verbose output with diagnostic info
python3 route.py board.kicad_pcb --nets "*" --verbose --output board_debug.kicad_pcb

# Debug geometry on User layers (visible in KiCad)
python3 route.py board.kicad_pcb --nets "*" --debug-lines --output board_debug.kicad_pcb

# Real-time visualization (requires pygame-ce)
python3 route.py board.kicad_pcb --nets "*" --visualize --output board_debug.kicad_pcb

# A* search statistics
python3 route.py board.kicad_pcb --nets "*" --stats --output board_debug.kicad_pcb
```

### Post-Routing Enhancements

```bash
# Add teardrop settings to all pads (improves manufacturability)
python3 route.py board.kicad_pcb --nets "*" --add-teardrops --output board_routed.kicad_pcb
```

### Advanced Routing Parameters

For difficult boards, consider tuning these parameters:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `--max-ripup 3` | 3 | Max blocking nets to rip up and retry |
| `--max-iterations 200000` | 200000 | A* iteration limit per route |
| `--heuristic-weight 1.9` | 1.9 | >1 = faster but may miss tight routes, 1.0 = optimal |
| `--via-cost 50` | 50 | Higher = fewer vias, longer paths; lower (10-25) for BGA escape |
| `--grid-step 0.1` | 0.1 | Smaller = finer routing but slower; 0.05 for fine-pitch |

Manufacturing constraints (set to match your fab's requirements):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--clearance 0.25` | 0.25 | Track-to-track clearance (mm) |
| `--board-edge-clearance 0.5` | 0 | Min distance from board edge (mm) |
| `--hole-to-hole-clearance 0.2` | 0.2 | Min drill-to-drill spacing (mm) |

### Proximity Penalties

For dense boards, use proximity penalties to spread out routes:

```bash
python3 route.py board.kicad_pcb --nets "*" \
    --stub-proximity-radius 2.0 --stub-proximity-cost 0.2 \
    --bga-proximity-radius 7.0 --bga-proximity-cost 0.2 \
    --track-proximity-distance 2.0 --track-proximity-cost 0.1 \
    --output board_routed.kicad_pcb
```

## Important Notes

1. **Always check for GND connections** - If a component has GND pads but GND isn't being fanned out, the plane vias will handle it
2. **Fanout ALL non-plane nets** - Use `--nets "*" "!GND" "!VCC"` to fan out all nets except those handled by planes. Do NOT use `"/*"` alone as it misses nets with non-hierarchical names like `Net-(U9-Pad1)`. Unconnected nets are automatically filtered out.
3. **Order matters** - Planes first, then fanout, then diff pairs, then signals, then repair
4. **Verify at the end** - Always run DRC, connectivity, and orphan stub checks
5. **Consider the analyze-power-nets skill** - For complex boards where power net identification isn't obvious, use that skill first to analyze component datasheets
6. **Stub layer switching is on by default** - The router automatically moves stubs to eliminate vias when beneficial; disable with `--no-stub-layer-swap`
7. **Default layer costs** - 2-layer boards default to F.Cu=1.0, B.Cu=3.0 to prefer top layer; 4+ layer boards use 1.0 for all
8. **Schematic sync is disabled by default** - After routing with swaps, offer to re-run with `--schematic-dir` if the user wants to update their schematic
9. **Rip-up and reroute is automatic** - When a route fails, the router automatically rips up blocking nets and retries (up to `--max-ripup` blockers)
10. **Component shortcut** - Use `--component U1` to route all signal nets on a component (auto-excludes GND/VCC/unconnected)
11. **Use --no-bga-zone for difficult boards** - Even when fanout is complete, use `--no-bga-zone` during routing to allow the router to find alternative paths through the dense pin area. This is especially important for 2-layer boards where routing channels are limited.
12. **Windows UTF-8 encoding** - On Windows, use `python -X utf8` to avoid Unicode encoding errors when scripts print special characters (like Ω for resistance). Example: `python -X utf8 route_planes.py ...`
13. **BGA/PGA power pins and planes** - When using power planes, BGA/PGA power pins (GND, VCC) connect most efficiently via direct vias to the plane rather than fanout routing. Create planes first, then fanout only signal nets. Through-hole PGA pads automatically connect to planes on that layer; SMD BGA pads need vias placed by `route_planes.py`. This approach:
    - Reduces routing congestion (power pins don't consume escape channels)
    - Provides lower impedance power connections
14. **Aggressive parameters for 2-layer BGA/PGA boards** - Use `--max-ripup 10 --max-iterations 1000000` from the start for boards with dense components. These parameters help resolve routing conflicts that would otherwise fail.

## Presenting the Plan

After generating the plan:
1. Show the board summary
2. Explain any special components found
3. List differential pairs if detected
4. Highlight any length-matching or impedance requirements
5. Present each step with the command AND a brief explanation of why
6. Ask the user if they want to proceed or modify the plan
7. Offer to run the commands if approved

## After Routing Completes

### Capture Logs for Analysis

Always capture command output to `/tmp` files for later analysis:

```bash
python -X utf8 route.py input.kicad_pcb output.kicad_pcb --nets "*" 2>&1 | tee /tmp/route_output.txt
python -X utf8 route_planes.py input.kicad_pcb output.kicad_pcb --nets GND --plane-layers B.Cu 2>&1 | tee /tmp/planes_output.txt
python -X utf8 check_connected.py output.kicad_pcb 2>&1 | tee /tmp/connectivity.txt
python -X utf8 check_drc.py output.kicad_pcb 2>&1 | tee /tmp/drc.txt
```

### Parse Logs for Failure Analysis

After routing, parse the log files to understand failures:

```bash
# Check routing summary (last 20 lines usually have the summary)
tail -20 /tmp/route_output.txt

# Look for failed nets
grep -i "failed\|FAILED" /tmp/route_output.txt

# Check JSON summary for detailed failure info
grep "JSON_SUMMARY" /tmp/route_output.txt | sed 's/JSON_SUMMARY: //' | python -m json.tool

# Find specific failure reasons
grep -A5 "FAILED NET HISTORIES" /tmp/route_output.txt
```

The JSON_SUMMARY line contains structured data including:
- `failed_single`: List of failed single-ended net names
- `failed_multipoint`: List of nets with unconnected pads (includes pad coordinates)
- `multipoint_pads_connected` vs `multipoint_pads_total`: Connection success rate

### Diagnose and Retry

After running routing commands:
1. Report how many nets were routed successfully
2. **If routes failed**, parse the logs to understand why, then retry with appropriate parameters:

| Failure Pattern | Likely Cause | Solution |
|-----------------|--------------|----------|
| "no rippable blockers found" | Route blocked by non-rippable obstacle | Use `--no-bga-zone` |
| "Re-route FAILED: no path found" | Ripped net couldn't find new path | Increase `--max-iterations` |
| Many multipoint pads failed on same component | Congested area | Use `--max-ripup 10` or higher |
| Routes near BGA boundary failing | BGA exclusion zone too aggressive | Use `--no-bga-zone` |

```bash
python -X utf8 route.py board_prev.kicad_pcb board_routed.kicad_pcb \
    --nets "*" \
    --no-bga-zone \
    --max-ripup 10 \
    --max-iterations 1000000 \
    2>&1 | tee /tmp/route_retry.txt
```

   Key parameters for difficult boards (especially 2-layer with BGA/PGA):
   - `--no-bga-zone` - **Critical**: Allows router to enter BGA area for alternative paths
   - `--max-ripup 10` (default 3) - More rip-up attempts to resolve conflicts
   - `--max-iterations 1000000` (default 200000) - 5x more search iterations
   - `--stub-proximity-radius 10 --stub-proximity-cost 3.0` - Spread out fanout stubs (optional, for aesthetics)

3. **If swaps occurred** (polarity or target swaps):
   - Tell the user how many swaps were made
   - Ask if they want to sync the schematic
   - If yes, ask for the KiCad project directory path
   - Re-run the routing command with `--schematic-dir` added
4. Run verification (DRC and connectivity checks)
5. Summarize the final state of the board
6. **Offer to clean up intermediate files**:
   - List the intermediate `.kicad_pcb` files created (e.g., `board_step1.kicad_pcb`, `board_step2.kicad_pcb`, etc.)
   - Ask if the user wants to delete them, keeping only the final output
   - If yes, delete the intermediate files

Example cleanup prompt:
> "Routing complete. The following intermediate files were created:
> - board_step1.kicad_pcb (after GND/VCC planes)
> - board_step2.kicad_pcb (after fanout)
> - board_step3.kicad_pcb (after signal routing)
>
> The final routed board is: board_step4.kicad_pcb
>
> Would you like me to delete the intermediate files?"
