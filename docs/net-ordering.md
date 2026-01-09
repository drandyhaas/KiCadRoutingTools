# Net Ordering Strategies

This document describes the net ordering strategies available in the KiCad Grid Router.

## Overview

The order in which nets are routed significantly affects success rate. Nets routed early have more freedom, while later nets must work around existing routes.

Three ordering strategies are available:

| Strategy | Flag | Best For |
|----------|------|----------|
| MPS | `--ordering mps` | General routing, crossing conflicts |
| Inside-Out | `--ordering inside_out` | BGA escape routing |
| Original | `--ordering original` | Manual control |

## MPS (Maximum Planar Subset) Algorithm

**Default strategy.** Minimizes crossing conflicts between net paths.

[View interactive MPS visualization](https://htmlpreview.github.io/?https://github.com/drandyhaas/KiCadRoutingTools/blob/main/docs/MPS.html)

### How It Works

1. **Identify chips**: Detect source and target chips from pad positions

2. **Boundary position computation**: "Unroll" each chip's rectangular boundary into a linear [0,1] position, starting from the "far side" (side facing away from the other chip)

3. **Detect crossings**: Two nets cross if their source order is inverted relative to their target order:
   ```
   Source chip:  Net A at position 0.3, Net B at position 0.5  (A < B)
   Target chip:  Net A at position 0.6, Net B at position 0.4  (A > B)
   Orders differ → crossing detected
   ```

4. **Build conflict graph**: Nets that cross share an edge in the conflict graph

5. **Greedy ordering**: Repeatedly select the net with fewest active conflicts. Within same conflict count, select shorter routes first (based on routing-aware distance)

6. **Routing-aware distance**: Instead of straight-line distance, the router computes a BGA-aware distance that routes around the target BGA. This measures from the target stub free end, around the BGA corner, to the source chip center - reflecting actual routing difficulty.

This chip boundary approach accurately detects crossings that respect the physical constraint that routes cannot go through BGA chips.

### Algorithm Detail

```python
def compute_mps_net_ordering(pcb_data, net_ids):
    # Step 1: Get routing endpoints for each net
    net_endpoints = {}
    for net_id in net_ids:
        endpoints = get_net_routing_endpoints(pcb_data, net_id)
        if len(endpoints) >= 2:
            net_endpoints[net_id] = endpoints[:2]

    # Step 2: Identify chips from pad positions
    chips = build_chip_list(pcb_data)  # List of ChipBoundary objects

    # Step 3: Compute boundary positions for each net
    net_boundary_info = {}
    for net_id, endpoints in net_endpoints.items():
        src_chip = identify_chip_for_point(endpoints[0], chips)
        tgt_chip = identify_chip_for_point(endpoints[1], chips)

        if src_chip and tgt_chip and src_chip != tgt_chip:
            # Determine far sides based on chip-to-chip direction
            src_far, tgt_far = compute_far_side(src_chip, tgt_chip)

            # Compute boundary positions with opposite traversal directions
            # Source: clockwise, Target: counter-clockwise (chips "face each other")
            src_pos = compute_boundary_position(src_chip, endpoints[0], src_far, clockwise=True)
            tgt_pos = compute_boundary_position(tgt_chip, endpoints[1], tgt_far, clockwise=False)
            net_boundary_info[net_id] = (src_pos, tgt_pos, src_chip, tgt_chip)

    # Step 4: Detect crossing conflicts
    def nets_cross(net_a, net_b):
        info_a = net_boundary_info.get(net_a)
        info_b = net_boundary_info.get(net_b)
        if not info_a or not info_b:
            return False
        # Only compare nets with same chip pair
        if (info_a[2], info_a[3]) != (info_b[2], info_b[3]):
            return False
        # Crossing = source order inverted relative to target order
        return (info_a[0] < info_b[0]) != (info_a[1] < info_b[1])

    # Build conflict graph
    conflicts = {net_id: set() for net_id in net_list}
    for i, net_a in enumerate(net_list):
        for net_b in net_list[i+1:]:
            if nets_cross(net_a, net_b):
                conflicts[net_a].add(net_b)
                conflicts[net_b].add(net_a)

    # Step 5: Greedy ordering
    ordered = []
    remaining = set(net_list)

    while remaining:
        # Find net with minimum active conflicts
        best_net = min(remaining, key=lambda n: len(conflicts[n] & remaining))
        ordered.append(best_net)
        remaining.remove(best_net)

        # Remove conflicting nets from this round
        for loser in conflicts[best_net] & remaining:
            remaining.remove(loser)

    return ordered
```

### Example

```
Before MPS ordering:
  Net A: crosses B, C
  Net B: crosses A, D
  Net C: crosses A
  Net D: crosses B

After MPS ordering:
  Round 1: Select C (1 conflict), D (1 conflict) - no mutual conflict
  Round 2: Select A (crosses already-routed C)
  Round 3: Select B (crosses already-routed A, D)

Order: [C, D, A, B]
```

### Usage

```bash
python route.py input.kicad_pcb output.kicad_pcb "Net-*" --ordering mps
```

### Segment Intersection Method

For boards without BGA chips (or when no nets being routed are on BGAs), the MPS algorithm uses a **segment intersection method** instead of boundary ordering:

1. **MST segments**: For each net, compute a Minimum Spanning Tree (MST) between all pad locations. This approximates the actual routing path better than using a centroid
2. **Segment intersection**: Two nets cross if any of their MST segments intersect (using CCW orientation test)
3. **Auto-detection**: When no net endpoints are inside BGA exclusion zones, segment intersection is automatically enabled

This method is more accurate for non-BGA boards where nets route directly between pads without needing to "escape" through chip boundaries.

```bash
# Force segment intersection method
python route.py input.kicad_pcb output.kicad_pcb "Net-*" --ordering mps --mps-segment-intersection

# Auto-detection (default): uses segment intersection when no nets on BGAs
python route.py input.kicad_pcb output.kicad_pcb "Net-*" --ordering mps
```

### Reverse Round Order

By default, MPS routes the least-conflicting groups first (fewest active conflicts). The `--mps-reverse-rounds` flag reverses this, routing the most-conflicting groups first:

```bash
python route.py input.kicad_pcb output.kicad_pcb "Net-*" --ordering mps --mps-reverse-rounds
```

This can sometimes improve results when:
- The most-constrained routes need maximum freedom
- Early routing of difficult nets allows easier nets to route around them
- You want to fail fast on impossible routes

### MPS Layer Swap

When MPS detects crossing conflicts (nets placed in Round 2+), the `--mps-layer-swap` flag enables automatic layer swaps to eliminate same-layer crossings:

```bash
python route.py input.kicad_pcb output.kicad_pcb "Net-*" --ordering mps --mps-layer-swap
```

#### How It Works

1. **Detect conflicts**: After initial MPS ordering, identify Round 2+ units that conflict with Round 1 units on shared layers

2. **Try Round 2 unit swap**: Attempt to move the Round 2 unit's stubs (source, target, or both) to a different layer where there's no overlap with the Round 1 unit

3. **Try Round 1 unit swap**: If the Round 2 unit can't be moved (all alternative layers have existing routes), try moving the Round 1 conflicting unit instead

4. **Validate swaps**: Each swap candidate is validated using the same validation as upfront layer swaps (no overlaps with existing routes, setback clearance)

5. **Re-run MPS**: After successful swaps, MPS ordering is re-run to verify the conflict is resolved

#### Example

```
Before MPS layer swap:
  MPS Round 1: 4 units (DQS0_A, DQS0_B, DQS1_B, CK_A)
  MPS Round 2: 1 unit (DQS1_A) - crosses DQS0_A on In2.Cu
  1 crossing conflict

After MPS layer swap:
  DQS0_A moved from In2.Cu → In1.Cu (both source and target)

  MPS Round 1: 5 units (all nets!)
  0 crossing conflicts
```

#### When to Use

- **Dense BGA routing**: When multiple diff pairs must cross between chips
- **Same-layer conflicts**: When nets end up on the same layer due to fanout assignment
- **Multi-round MPS results**: If MPS ordering shows more than 1 round, layer swaps may help

#### Limitations

- Requires available alternative layers (swaps fail if all layers have existing routes)
- Adds vias when swapping layers (2 vias per stub moved)
- Only helps when `--crossing-layer-check` is enabled (default) - routes on different layers don't count as crossing

### Routing-Aware Distance Calculation

When ordering routes within each MPS round, shorter routes are prioritized. The distance is calculated using a BGA-aware algorithm:

1. **Target stub free end**: The actual endpoint of the target stub (not the centroid)
2. **Source chip center**: The center of the component containing the source pads
3. **BGA avoidance**: The path cannot go through the target BGA - it must go around

```
Target BGA:
┌─────────────────┐
│                 │
│    [Target]     │   Path goes around corner
│       ↓         │   to avoid crossing BGA
│       *─────────┼──→ corner → source chip center
└─────────────────┘
```

The algorithm:
1. Find which BGA zone contains (or is nearest to) the target stub
2. Determine which edge of the BGA the stub is on (top, bottom, left, right)
3. Compute two candidate paths around the two corners of that edge
4. Return the shorter path distance

This produces more accurate ordering than straight-line distance because it reflects the actual routing constraint that routes cannot pass through BGA areas.

## Inside-Out Ordering

Routes BGA nets from center outward, improving escape routing success.

### How It Works

1. Identify nets with pads inside BGA exclusion zones
2. Calculate each net's minimum distance from any BGA pad to the BGA center
3. Sort by distance (closest to center first)
4. Non-BGA nets keep original order and are routed last

### Algorithm

```python
def inside_out_ordering(net_ids, pcb_data, bga_zones):
    bga_nets = []
    non_bga_nets = []

    for net_name, net_id in net_ids:
        pads = pcb_data.pads_by_net.get(net_id, [])
        has_bga_pad = any(pad_in_zone(pad, bga_zones) for pad in pads)

        if has_bga_pad:
            dist = min_distance_to_center(net_id, bga_zones)
            bga_nets.append((net_name, net_id, dist))
        else:
            non_bga_nets.append((net_name, net_id))

    # Sort BGA nets by distance (inside-out)
    bga_nets.sort(key=lambda x: x[2])

    return [(name, nid) for name, nid, _ in bga_nets] + non_bga_nets
```

### Why Inside-Out Works

Inner BGA pads have limited escape paths. By routing them first:
- They can use the most direct escape routes
- Outer pads have more alternative paths available
- Reduces routing failures due to blocked escape channels

```
BGA Pad Array:
    . . . . . . .
    . o o o o o .    o = inner pads (route first)
    . o * * * o .    * = center pads (route first)
    . o * * * o .
    . o o o o o .
    . . . . . . .    . = outer pads (route last)
```

### Usage

```bash
python route.py input.kicad_pcb output.kicad_pcb "Net-*" --ordering inside_out
```

## Original Ordering

Keeps nets in the order they were specified on the command line.

### When to Use

- You've manually determined the optimal order
- Testing specific routing sequences
- Debugging routing failures

### Usage

```bash
python route.py input.kicad_pcb output.kicad_pcb "Net-A" "Net-B" "Net-C" --ordering original
```

## Strategy Comparison

| Scenario | Recommended Strategy |
|----------|---------------------|
| General routing | MPS |
| BGA escape | Inside-Out |
| Long parallel routes | MPS |
| Manual control | Original |
| Dense routing with many crossings | MPS |
| Simple 2-point routes | Any |

## Combining with Direction Order

Net ordering (which net routes first) combines with direction order (which endpoint is source vs target):

```bash
# MPS ordering + random direction per net
python route.py input.kicad_pcb output.kicad_pcb "Net-*" \
    --ordering mps \
    --direction random
```

Direction options:
- `forward`: Always route from stub group 1 to stub group 2
- `backwards`: Always route from stub group 2 to stub group 1
- `random`: Randomly choose direction per net (helps avoid systematic bias)

## Performance Notes

- MPS preprocessing adds ~100-500ms for 100+ nets
- Inside-Out sorting is O(n log n) - negligible overhead
- Better ordering can reduce total routing time by avoiding failed attempts
- MPS typically achieves higher success rates on complex boards
