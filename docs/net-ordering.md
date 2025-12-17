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

### How It Works

1. **Project endpoints to circle**: Each net's source and target are projected onto a circular boundary around the routing region

2. **Detect crossings**: Two nets "cross" if their endpoints alternate on the circle:
   ```
   Circle: ... A1 ... B1 ... A2 ... B2 ...
                    ^ these nets cross
   ```

3. **Build conflict graph**: Nets that cross share an edge in the conflict graph

4. **Greedy ordering**: Repeatedly select the net with fewest active conflicts, add to result, remove its conflicting neighbors from consideration

### Algorithm Detail

```python
def compute_mps_net_ordering(pcb_data, net_ids, center=None):
    # Step 1: Get routing endpoints for each net
    net_endpoints = {}
    for net_id in net_ids:
        endpoints = get_net_routing_endpoints(pcb_data, net_id)
        if len(endpoints) >= 2:
            net_endpoints[net_id] = endpoints[:2]

    # Step 2: Compute center (centroid of all endpoints)
    if center is None:
        center = compute_centroid(all_endpoints)

    # Step 3: Compute angular position for each endpoint
    net_angles = {}
    for net_id, endpoints in net_endpoints.items():
        a1 = angle_from_center(endpoints[0])
        a2 = angle_from_center(endpoints[1])
        net_angles[net_id] = (min(a1, a2), max(a1, a2))

    # Step 4: Detect crossing conflicts
    def nets_cross(net_a, net_b):
        a1, a2 = net_angles[net_a]
        b1, b2 = net_angles[net_b]
        # Check if intervals interleave
        return (a1 < b1 < a2 < b2) or (b1 < a1 < b2 < a2)

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
