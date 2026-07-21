---
name: review-routed-board
description: Post-route QA on a KiCad board. Runs the DRC, connectivity, and orphan-stub checkers, verifies length/time-match groups landed within tolerance, checks GND return via coverage on high-speed nets, and reviews differential pairs. Produces a pass/fail sign-off report with concrete next actions. Works on boards routed by this tool, by hand, or by other tools.
---

# Review Routed Board

When this skill is invoked with a board file, run a full post-route review and present a sign-off report.

## Step 1: Mechanical Checks

Run all three checkers, capturing output:

```bash
python3 -X utf8 check_drc.py board.kicad_pcb 2>&1 | tee /tmp/review_drc.txt
python3 -X utf8 check_connected.py board.kicad_pcb 2>&1 | tee /tmp/review_connectivity.txt
python3 -X utf8 check_orphan_stubs.py board.kicad_pcb 2>&1 | tee /tmp/review_orphans.txt
```

`check_drc.py` auto-grades at the clearance the routing steps wrote into the sibling
`.kicad_pro` (the smallest clearance any step actually used, including auto-stepped
fine-pitch taps), so the bare invocation above already grades at the true routed
floor. Pass `--clearance <value>` only to override (e.g. to grade a hand-routed
board with no routed-floor `.kicad_pro`).

**Important caveat to include in the report:** `check_drc.py` does not check zone copper, minimum trace width, or netclass compliance. If the board has copper zones/planes, recommend the zone-aware check:

```bash
kicad-cli pcb drc board.kicad_pcb --refill-zones --format json -o /tmp/drc.json
```

**Ignore silk and dangling violations** when reading that report — they are not
routing/connectivity defects and are excluded by the harness graders
(`kicad_drc_compare.py`, `kicad_oracle.py`) for exactly this reason. Filter them
out before counting:

```bash
python3 -c "import json;v=json.load(open('/tmp/drc.json'))['violations'];\
drop={'via_dangling','track_dangling','silk_overlap','silk_over_copper',\
'silk_edge_clearance','silk_mask_clearance'};\
c=[x for x in v if x['type'] not in drop];\
print(f'{len(c)} copper/connectivity violations ({len(v)-len(c)} silk/dangling ignored)')"
```

The cross-check is one-directional (#260): kicad-cli can refute a borderline
check_drc *near-miss* (a sub-clearance gap), but a kicad-cli "0" does NOT clear
an *overlap/short* finding — KiCad 10 net-unifies touching copper on load, so
overlapping foreign-net tracks/pads report nothing at any severity. For
touching-copper overlaps, `check_drc.py` is the authoritative one.

## Step 2: Length/Time Match Verification

If the board has length-matched groups (DDR byte lanes, matched buses — detect DQ/DQS patterns or ask the user for the groups and tolerance):

```python
from kicad_parser import parse_kicad_pcb
from net_queries import calculate_route_length

pcb = parse_kicad_pcb('board.kicad_pcb')
for group_name, net_names in groups.items():
    lengths = {}
    for net in pcb.nets.values():
        if net.name in net_names:
            lengths[net.name] = calculate_route_length(pcb, net.net_id)
    spread = max(lengths.values()) - min(lengths.values())
    # PASS if spread <= tolerance (default 0.1 mm); report worst offender otherwise
```

For time-matched groups use `calculate_route_propagation_time_ps()` from `impedance.py` instead, and compare against the ps tolerance. Lengths include via barrels from the stackup, matching KiCad's measurement.

Report per group: the spread, the tolerance, and the worst offender net.

## Step 3: GND Return Via Coverage

For high-speed nets (use `/find-high-speed-nets` results if available, else the name-pattern tiers from `/plan-pcb-routing`), check each signal via has a GND via nearby:

```python
gnd_ids = {n.net_id for n in pcb.nets.values() if n.name and 'GND' in n.name.upper()}
gnd_vias = [v for v in pcb.vias if v.net_id in gnd_ids]
# Through-hole GND pads also count as return paths
for via in pcb.vias:
    if via.net_id in high_speed_net_ids:
        nearest = min(((v.x-via.x)**2 + (v.y-via.y)**2)**0.5 for v in gnd_vias)
        # Flag if nearest > recommended distance for the net's speed tier
        # (ultra-high: 2.0mm, high: 3.0mm, medium: 5.0mm)
```

## Step 4: Differential Pair Review

For each differential pair (from `list_nets.py --diff-pairs`):

- **Gap consistency**: sample parallel P/N segment pairs on the same layer and confirm the spacing matches the design gap.
- **Intra-pair skew**: compare P and N lengths; flag pairs differing by more than ~0.5mm if intra-pair matching was expected.
- **Polarity swaps**: if the routing logs (or the user) indicate pad swaps were applied, remind that the schematic may be out of sync and `--schematic-dir` can update it.

## Step 5: The Sign-Off Report

Present a compact report — one pass/fail line per category, details only for failures, ending with next actions:

```
## Board Review: board.kicad_pcb

| Check | Result |
|-------|--------|
| DRC (check_drc.py)        | PASS (0 violations) |
| Zone DRC (kicad-cli)      | NOT RUN - board has 2 zones, recommend running |
| Connectivity              | FAIL - 2 nets with disconnected pads |
| Orphan stubs              | PASS |
| Length match (byte_lane_0)| PASS (spread 0.06mm, tol 0.1mm) |
| GND return vias           | WARN - 3 high-tier signal vias lack GND via within 3mm |
| Diff pairs                | PASS (4 pairs, gap consistent) |

### Failures
- SDA: pad (151.25, 109.06) on F.Cu [U1] disconnected ...

### Next actions
1. Run /diagnose-routing-failures with the routing logs for the 2 disconnected nets
2. Re-run route_planes.py --add-gnd-vias for the 3 uncovered signal vias
```

When connectivity or routing failures are found, recommend `/diagnose-routing-failures` as the follow-up rather than diagnosing inline here.
