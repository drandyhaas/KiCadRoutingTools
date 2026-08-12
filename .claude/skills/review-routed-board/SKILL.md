---
name: review-routed-board
description: Post-route QA on a KiCad board. Runs the DRC, connectivity, and orphan-stub checkers, verifies length/time-match groups landed within tolerance, checks GND return via coverage on high-speed nets, and reviews differential pairs. Produces a pass/fail sign-off report with concrete next actions. Works on boards routed by this tool, by hand, or by other tools.
---

# Review Routed Board

When this skill is invoked with a board file, run a full post-route review and present a sign-off report.

## Step 1: Mechanical Checks

Run all four checkers, capturing output:

```bash
python3 -X utf8 py_router/check_drc.py board.kicad_pcb 2>&1 | tee /tmp/review_drc.txt
python3 -X utf8 py_router/check_connected.py board.kicad_pcb 2>&1 | tee /tmp/review_connectivity.txt
python3 -X utf8 py_tools/check_orphan_stubs.py board.kicad_pcb 2>&1 | tee /tmp/review_orphans.txt
python3 -X utf8 py_router/check_weird.py board.kicad_pcb 2>&1 | tee /tmp/review_weird.txt
```

`check_weird.py` is here because the other three cannot see its classes.
`check_orphan_stubs` iterates SEGMENT endpoints and treats a via as an anchor,
so it structurally cannot report a bad via; `check_weird` owns `dangling-via`
(same-net copper on only one of the layers the barrel spans -- KiCad's
`via_dangling`), `unsupported-via`, `stacked-copper` and `orphan-island`.
Measured on run 11's final board: `check_orphan_stubs` none, `check_weird`
**3 dangling vias**, each independently confirmed.

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

Why `via_dangling` in particular, measured on run 11's final board: KiCad
reported **66** of them, all severity `warning`, while only **3** vias on the
board are genuinely supported on one layer. The other ~63 are plane-tap vias on
the two poured nets whose zone copper KiCad has not filled — the pour is real in
the file, but a via's second-layer connection is the zone, and an unfilled zone
connects nothing. Drop the flag from the *count*, and get the true number from
`check_weird.py`'s `dangling-via`, which credits the zone polygon directly. Do
NOT chase parity with KiCad's figure here; you would be chasing a fill artifact.

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
from net_queries import net_copper_lengths, pin_pair_path_length

pcb = parse_kicad_pcb('board.kicad_pcb')
name_to_id = {n.name: nid for nid, n in pcb.nets.items()}
for group_name, net_names in groups.items():
    ids = [name_to_id[n] for n in net_names if n in name_to_id]
    by_id = net_copper_lengths(pcb, ids)          # one pass over the copper
    lengths = {n: by_id[name_to_id[n]] for n in net_names if n in name_to_id}
    spread = max(lengths.values()) - min(lengths.values())
    # PASS if spread <= tolerance (default 0.1 mm); report worst offender otherwise
```

`net_copper_lengths` (and `net_copper_length` for one net) measures **total net
copper**. That is the right number only for a clean point-to-point net. On a
**multipoint or fly-by net, or any net with a stub**, total copper is the sum of
every branch and matches no signal path — measure the driver→receiver path
instead, and match on that:

```python
pads = pcb.pads_by_net[name_to_id['DQ0']]
path_mm = pin_pair_path_length(pcb, name_to_id['DQ0'], pads[0], pads[1])
# None = those two pads are not joined by TRACK copper (plane-only or broken)
```

Report both when they differ by more than the tolerance: a group that passes on
total copper can fail on path length. Pads joined only through a zone return
`None` — say so rather than reporting a length.

For time-matched groups use `calculate_route_propagation_time_ps()` from `impedance.py` instead, and compare against the ps tolerance. Lengths include via barrels from the stackup, matching KiCad's measurement.

Report per group: the spread, the tolerance, and the worst offender net.

## Step 2b: Meander Arm-Spacing Audit (#501)

Serpentine arms are **same-net copper, so no DRC checker will ever flag them** —
tightly packed arms couple to themselves, split odd/even-mode velocity, and make
the meander add less *delay* than *length*. A group can report "PASS, spread
0.06mm" and still be worse-matched than an untuned board. For every net in a
length/time-matched group (and any net with visible serpentines), audit the
arm-to-arm spacing directly:

```python
from kicad_parser import parse_kicad_pcb
from geometry_utils import segment_to_segment_distance

pcb = parse_kicad_pcb('board.kicad_pcb')
for nid in matched_net_ids:
    segs = [s for s in pcb.segments if s.net_id == nid]
    worst = None
    for i, a in enumerate(segs):
        for b in segs[i + 1:]:
            if a.layer != b.layer:
                continue
            d = segment_to_segment_distance(
                a.start_x, a.start_y, a.end_x, a.end_y,
                b.start_x, b.start_y, b.end_x, b.end_y)
            # touching/chained segments (d ~ 0) are the route itself, not arms
            gap = d - (a.width + b.width) / 2
            if d > 1e-6 and (worst is None or gap < worst):
                worst = gap
    # PASS: worst edge-to-edge gap >= 1x width (= the default 2W pitch).
    # WARN below 1W; FAIL below ~0.5W (arms tighter than the router's own default).
```

Judge the gap in **track widths**: the router's default arm pitch is
`--meander-spacing` = 2.0 × width centre-to-centre (1W edge gap); SI practice
prefers 3W–4W pitch for long serpentines on fast edges. Report the worst
arm gap per net in mm and in multiples of that net's width, and — when the board
was routed by this tool below the recommendation — the
`--meander-spacing 3` rerun that would spread the arms.

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

## Step 3b: Impedance & Return-Path Audit (#486)

A controlled-impedance trace whose reference plane has a **gap under its path**
gets the ideal-plane impedance anyway — the classic return-path discontinuity.
It is not a DRC violation and ships silent, so check it explicitly:

```bash
# every routed signal net (plane nets are skipped automatically)
python3 py_tools/check_impedance.py board.kicad_pcb --verbose

# narrow to the controlled-impedance nets when you know them
python3 py_tools/check_impedance.py board.kicad_pcb --nets "RF*" "/DDR*"

# if the route step declared a coplanar gap, AUDIT that promise
python3 py_tools/check_impedance.py board.kicad_pcb --coplanar-gap 0.2
```

Boards routed by this tool need no `--coplanar-gap`: `--impedance` steps record
each net's declaration (ohms + coplanar gap) in the sibling `.kicad_pro`
(#521), and `check_impedance.py` auto-reads them — every net audits against its
OWN recorded promise (an "Auto-read N net impedance declaration(s)" line
confirms it). Pass the flag only for boards without records.

Reports per net: length over a reference-plane **void**, plane **split**
crossings (the return current cannot follow the trace across either), the
measured coplanar side-gap distribution, and the implied Z0 error vs. what the
route call assumed. Exits 1 when anything is found.

How to read it:

- **Length over void on a controlled-impedance net is the finding that matters.**
  Report it with the net, the layer, and the offending location; recommend a
  reroute onto intact plane, or a stitching via pair straddling the gap where
  the crossing is unavoidable.
- **A "reference layer carries NO filled pour" note** is a stackup/flow problem,
  not a per-net defect — it means nothing was poured on that layer at all. Say
  so once rather than listing every net.
- **Void on a plain low-speed net is usually noise.** Only escalate for nets that
  are genuinely impedance-controlled or high-speed.
- **If `--coplanar-gap` was declared**, the audit's "NO ground beside" and "gap
  off-target" lengths are the real result: that copper was routed at a width
  assuming a ground that is not there. Some off-target length near via antipads
  and pads is normal; a net that is mostly off-target is a genuine defect.

## Step 4: Differential Pair Review

For each differential pair (from `list_nets.py --diff-pairs`):

- **Gap consistency**: sample parallel P/N segment pairs on the same layer and confirm the spacing matches the design gap.
- **Intra-pair skew**: compare P and N lengths; flag pairs differing by more than ~0.5mm if intra-pair matching was expected.
- **Polarity swaps**: if the routing logs (or the user) indicate pad swaps were applied, remind that the schematic may be out of sync and `--schematic-dir` can update it.

## Step 5: The Sign-Off Report

**Lead with the score, so this review and `/plan-pcb-routing` agree on what
"done" means.** One command produces it, and it is the same number Step 9 of the
routing skill loops on:

```bash
python3 -X utf8 .claude/skills/plan-pcb-routing/scripts/board_score.py \
    board.kicad_pcb --intent wk/floorplan.json \
    --min-track-width <spec> --min-via-diameter <spec> --min-via-drill <spec>
```

`blocking == 0` is the only state in which this report may say the board is
ready. Anything else is **not done** — hand it back to `/plan-pcb-routing`
Step 9 rather than signing off with caveats. Two traps this closes:

- **`ungraded` is not `passed`.** Components with no intent, no impedance nets
  and no length groups were *unexamined*; list them as such.
- **The size floors default to the FAB minimum, not the spec.** Pass the spec's
  numbers, or copper that meets the fab and violates the board's own tighter
  requirement signs off clean.

Then present a compact report — one pass/fail line per category, details only for failures, ending with next actions:

```
## Board Review: board.kicad_pcb

BLOCKING=0  (unrouted=0 broken=0 drc=0 undersized=0 floorplan=0)
UNGRADED: impedance, length          <- unexamined, NOT passed

| Check | Result |
|-------|--------|
| DRC (check_drc.py)        | PASS (0 violations) |
| Zone DRC (kicad-cli)      | NOT RUN - board has 2 zones, recommend running |
| Connectivity              | FAIL - 2 nets with disconnected pads |
| Orphan stubs              | PASS |
| Length match (byte_lane_0)| PASS (spread 0.06mm, tol 0.1mm) |
| Meander arm spacing       | PASS (worst gap 1.0W on DQ3) |
| GND return vias           | WARN - 3 high-tier signal vias lack GND via within 3mm |
| Diff pairs                | PASS (4 pairs, gap consistent) |

### Failures
- SDA: pad (151.25, 109.06) on F.Cu [U1] disconnected ...

### Next actions
1. Run /diagnose-routing-failures with the routing logs for the 2 disconnected nets
2. Re-run route_planes.py --add-gnd-vias for the 3 uncovered signal vias
```

When connectivity or routing failures are found, recommend `/diagnose-routing-failures` as the follow-up rather than diagnosing inline here.
