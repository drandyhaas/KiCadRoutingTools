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
`via_dangling`), `unsupported-via`, `stacked-copper`, `orphan-island` and
`kicad-dangling` (a joint stub lying on one other track, which KiCad reports as
`track_dangling` though both its ends touch copper).
Measured on run 11's final board: `check_orphan_stubs` none, `check_weird`
**3 dangling vias**, each independently confirmed.

`check_drc.py` auto-grades at the clearance the routing steps wrote into the sibling
`.kicad_pro` (the smallest clearance any step actually used, including auto-stepped
fine-pitch taps), so the bare invocation above already grades at the true routed
floor. Like KiCad, it floors that class at Board Setup `min_clearance` (#1210):
an unrouted or hand-routed board whose project declares a minimum above its
Default class grades at the minimum. Pass `--clearance <value>` only to override
(e.g. to grade a hand-routed board with no routed-floor `.kicad_pro`).

When the board was routed from an input you have, add `--baseline <the input
board>`. `VIA-IN-PASTE` rows are vias whose barrel sits in a solder-paste
opening of their own net without IPC-4761 Type VII (filled AND capped); solder
wicks into such a barrel. KiCad has no such check, so check_drc is the only
instrument that reports them. On a KiCad 10-format board the router stamps
Type VII onto every via it adds in a pad or paste opening, so these are almost
always vias the input already had: `--baseline` accepts those (a via the input
had under solder, unprotected) as `inherited-via-in-paste` and the console line
counts them. A file older than KiCad 10 (version < 20250000) cannot carry
the tokens at all, so there every such via `--baseline` does not inherit is
accepted `undeclarable-via-in-paste` and counted on the same line: Type VII
belongs on the fab drawing. Report whatever still fires as a fab defect: the
via needs filled+capped on the fab drawing, or it must move out of the opening.
Without `--baseline`, every pre-existing one reads as a violation
(one corpus board carries 136). The same flag grades a graze of footprint graphic copper that a
part MOVE created (`graphic-board-edge`); `graphic-off-board` (copper past the
outline) is reported either way. `KEEPOUT` rows are tracks or vias inside a
keep-out rule area that forbids them (an antenna clearance, a mounting hole's
screw-head ring); kicad-cli 10.0.0 did not report these when measured, so
check_drc is the only instrument. `--baseline` accepts copper the input already
had there as `inherited-keepout`; report what still fires as a routing defect.

**Important caveat to include in the report:** `check_drc.py` does not grade zone (pour) fill copper. It does grade per-netclass clearance and `.kicad_dru` rules, and track width / via size against the FAB floor for the layer count unless given `--min-track-width` / `--min-via-diameter` / `--min-via-drill` (pass the spec's numbers when they are tighter). If the board has copper zones/planes, recommend the zone-aware check:

```bash
KCLI=$(python3 py_tools/kicad_cli_path.py)
"$KCLI" pcb drc board.kicad_pcb --refill-zones --format json -o /tmp/drc.json
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

A `shorting_items` between `<no net>` and a net, on a part's own graphic copper
("Polygon [<no net>] of U2"), is not a short when that net is the part's own
pad's: a SOT-89 tab its pad's net was routed onto. `check_drc.py` lists each one
under WARNINGS as `footprint own copper` (#995). Report them as KiCad errors the
user will see, not as shorts. A `<no net>` item with any OTHER net is a real
short.

Two pads of ONE footprint touching on different nets are a check_drc `pad-pad`
violation (#1306), KiCad's `shorting_items` (`clearance` at actual 0 when one
pad has no net). Some libraries ship them (a Kelvin shunt with no net-tie
group, a shield tab or thermal-via pad with no net): pass `--baseline <input
board>` and a contact the input's footprint already had is accepted
`inherited-own-pad-contact`. A contact that is NOT inherited means a tool turned
or rewrote the footprint's pads.

The cross-check is one-directional (#260): kicad-cli can refute a borderline
check_drc *near-miss* (a sub-clearance gap), but a kicad-cli "0" does NOT clear
an *overlap/short* finding — KiCad 10 net-unifies touching copper on load, so
overlapping foreign-net tracks/pads report nothing at any severity. For
touching-copper overlaps, `check_drc.py` is the authoritative one.

## Step 2: Length/Time Match Verification

If the board has length-matched groups (DDR byte lanes, matched buses — detect DQ/DQS patterns or ask the user for the groups and tolerance):

```python
import sys
sys.path.insert(0, "py_router")
from kicad_parser import parse_kicad_pcb
from net_queries import net_match_length

pcb = parse_kicad_pcb('board.kicad_pcb')
name_to_id = {n.name: nid for nid, n in pcb.nets.items()}
for group_name, net_names in groups.items():
    lengths = {n: net_match_length(pcb, name_to_id[n])    # total net copper
               for n in net_names if n in name_to_id}
    unjoined = [n for n, mm in lengths.items() if mm is None]
    if unjoined:
        # UNMEASURED, not a pass or a fail: these nets' pads are not joined by
        # track copper (unrouted, fanout stubs only, broken, or plane-only).
        print(f"{group_name}: unmeasured, not joined: {unjoined}")
        continue
    spread = max(lengths.values()) - min(lengths.values())
    # PASS if spread <= tolerance (default 0.1 mm); report worst offender otherwise
```

A **`.kicad_dru` length window** (`(constraint length (min ..) (max ..))`) is
an absolute bound, not a spread: `check_drc.py` grades it (Step 1's run) and
lists each net outside it as `LENGTH-OUT-OF-RANGE`, KiCad's
`length_out_of_range`, plus any rule it could not model as `NOT GRADED`
(#1313). Report those rows here too; a group can pass its spread while every
member misses the window.

`net_match_length` is the measurement `board_score.py --length-groups` grades
with, so the two cannot disagree. **Never grade raw `net_copper_lengths`
spreads**: a net with no copper reads 0.0 mm and one with only fanout stubs
reads the stub length, so two unrouted nets "pass" at spread 0.0 and a
stub-only byte lane "fails" (#1246). `None` means not routed yet; report it as
unmeasured, which `unrouted`/`broken` already count.

The default mode measures **total net copper**. That is the right number only
for a clean point-to-point net. On a **multipoint or fly-by net, or any net
with a stub**, total copper is the sum of every branch and matches no signal
path — measure the driver→receiver path instead, and match on that:

```python
path_mm = net_match_length(pcb, name_to_id['DQ0'], mode='pin_pair')
# None = the net's first two pads are not joined by TRACK copper
# (plane-only or broken; a via centred in a pad joins it through its
# barrel, #1314); pin_pair_path_length(pcb, nid, pad_a, pad_b)
# measures any other pad pair
```

Report both when they differ by more than the tolerance: a group that passes on
total copper can fail on path length. Pads joined only through a zone return
`None` — say so rather than reporting a length.

For time-matched groups use `calculate_route_propagation_time_ps()` from `impedance.py` instead, plus each pad's `die_delay` (`net_pad_to_die_delay`, ps), and compare against the ps tolerance. Lengths include via barrels from the stackup and, in `net_match_length`, every pad's pad-to-die length (`(die_length ...)`, #1248) -- the terms KiCad's length/skew DRC adds. A board that declares pad-to-die is graded wrong by raw copper: KiCad sees the difference as skew.

Report per group: the spread, the tolerance, and the worst offender net.

## Step 2b: Meander Arm-Spacing Audit (#501)

Serpentine arms are **same-net copper, so no DRC checker will ever flag them** —
tightly packed arms couple to themselves, split odd/even-mode velocity, and make
the meander add less *delay* than *length*. A group can report "PASS, spread
0.06mm" and still be worse-matched than an untuned board. For every net in a
length/time-matched group (and any net with visible serpentines), audit the
arm-to-arm spacing directly:

```python
import sys
sys.path.insert(0, "py_router")
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
from kicad_parser import pad_is_plated_through
gnd_ids = {n.net_id for n in pcb.nets.values() if n.name and 'GND' in n.name.upper()}
# GND vias, and plated through-hole GND pads, are return paths
gnd_pts = [(v.x, v.y) for v in pcb.vias if v.net_id in gnd_ids]
gnd_pts += [(p.global_x, p.global_y) for fp in pcb.footprints.values()
            for p in fp.pads if p.net_id in gnd_ids and pad_is_plated_through(p)]
for via in pcb.vias:
    if via.net_id in high_speed_net_ids:
        nearest = min((((x-via.x)**2 + (y-via.y)**2)**0.5 for x, y in gnd_pts),
                      default=float('inf'))
        # Flag if nearest > recommended distance for the net's speed tier
        # (ultra-high: the 3 x (via_size + clearance) floor, 2.25mm at the
        # default 0.5 via / 0.25 clearance; high: 3.0mm; medium: 5.0mm)
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
- **A void inside the antipad of the net's own via (or its P/N partner's) is a
  layer change, not a slot.** check_impedance counts those as
  `own_via_antipad_runs`, never as crossings. Whether a GND via sits beside the
  transition is a separate return-path question.
- **If `--coplanar-gap` was declared**, the audit's "NO ground beside" and "gap
  off-target" lengths are the real result: that copper was routed at a width
  assuming a ground that is not there. Some off-target length near via antipads
  and pads is normal; a net that is mostly off-target is a genuine defect.

## Step 4: Differential Pair Review

Read each pair off its **copper**, never off the router's word for it (#1303).
`board_score.py` (Step 5's run) carries a `diff_pairs` component that measures
every pair KiCad recognises as KiCad's DRC does: coupled and uncoupled length
(`net_queries.diff_pair_coupling`), the layers each member uses, and the P/N
skew (`net_queries.diff_pair_skew`). Its console line names the largest skews
and any pair with **NO COUPLED COPPER** -- P and N never run side by side,
which is what a pair whose members took separate layers looks like (imx6ulz
DQS0: P on In2.Cu, N on B.Cu, 0 mm coupled, 4.5 mm skew, while route_diff
called it coupled). In Python:

```python
from net_queries import diff_pair_copper
rec = diff_pair_copper(pcb, p_id, n_id)   # coupled_mm, uncoupled_mm, skew_mm, layers_p/_n
```

- **Coupling**: a pair the plan routed coupled must show coupled copper. In a
  route_diff log, `JSON_SUMMARY.uncoupled_diff_pairs` (outcome `uncoupled`)
  lists pairs routed whose shipped copper is not coupled (a coupled run under
  one pitch or under 10% of the pair's length, `pair_reports[].copper.
  coupled_share`) -- route_diff no longer
  counts them routed, and protects one only while its members still run
  alongside within 1 mm and both connect (`pair_reports[].protected`, #1361),
  so the next step may reroute the rest.
- **Skew / uncoupled length against the spec**: a `.kicad_dru` `skew` rule
  (`(within_diff_pairs)` for P against N) or `diff_pair_uncoupled` rule is
  graded by `check_drc.py` (`SKEW-OUT-OF-RANGE`, `DIFF-PAIR-UNCOUPLED`, KiCad's
  `skew_out_of_range` / `diff_pair_uncoupled_length_too_long`) and counted in
  board_score's `diff_pairs`. With no such rule the component is UNGRADED --
  report the measured skews, and flag pairs differing by more than ~0.5mm if
  intra-pair matching was expected. An intra-pair status in a route_diff log is
  read off the shipped copper too (`intra_pair_matching[].delta_mm`; the
  matcher's own number is `matcher_delta_mm`).
- **Polarity swaps**: if the routing logs (or the user) indicate pad swaps were applied, remind that the schematic may be out of sync and `--schematic-dir` can update it.

## Step 5: The Sign-Off Report

**Lead with the score, so this review and `/plan-pcb-routing` agree on what
"done" means.** One command produces it, and it is the same number Step 9 of the
routing skill loops on:

```bash
python3 -X utf8 py_tools/board_score.py \
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

`blocking == 0` is necessary, not sufficient: `/plan-pcb-routing`'s DONE also
needs `check_complete.py` (repo root) to say DONE. It takes the same spec
flags, runs `board_score` plus the instruments `board_score` has no component
for (orphan stubs, weird copper, ...), and fails closed: exit 0 DONE, 4
INCOMPLETE, 5 UNSOUND (the board grades clean only against floors the chain
rewrote; pass `--authored-from <the board the chain started from>` so it can
tell).

```bash
python3 -X utf8 check_complete.py board.kicad_pcb --authored-from <original> \
    --min-track-width <spec> --min-via-diameter <spec> --min-via-drill <spec>
```

Then present a compact report — one pass/fail line per category, details only for failures, ending with next actions:

```
## Board Review: board.kicad_pcb

BLOCKING=0  (unrouted=0 broken=0 drc=0 undersized=0 floorplan=0 assembly=0 impedance=0 length=0 net_widths=0 diff_pairs=0)  vias=214 copper_mm=1893.4
UNGRADED (not scored, not passed): impedance, length

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
2. Re-run route_planes.py --add-gnd-vias for the 3 uncovered signal vias, then
   a closing route.py whose --nets covers the poured nets (the re-pour leaves
   its welds unverified until route.py's plane finalize runs)
```

When connectivity or routing failures are found, recommend `/diagnose-routing-failures` as the follow-up rather than diagnosing inline here. If they trace to part positions (pad copper off the outline, unreachable pads, `check_assembly` not buildable), recommend `/plan-pcb-placement full` instead of a router retry.
