---
name: plan-pcb-placement-and-routing
description: Plans a KiCad PCB end to end when ONE run must both place and route it. Sequences the placement skill and the routing skill, and owns the rules that exist only when the two meet: that placement invalidates every downstream routed board, that a routing failure must be classified before it is retried, and that a placement-shaped failure re-enters placement rather than burning retries in the router. Use when the board needs both; use the individual skills when it needs one.
---

# Plan PCB Placement and Routing

Use this when a single run must do both. It does not restate either half: it
sequences `/plan-pcb-placement` and `/plan-pcb-routing` and owns the four rules
that only exist when they meet.

**The goal of the whole run is a board that ROUTES — parts arranged so they
work together, ending at zero `unrouted` and zero `broken`.** On the perturbed
corpus, `recovery` and `home /N` measure distance to the ORIGINAL pose and are
diagnostics, not the score; a run can post a negative recovery while taking the
board from NOT BUILDABLE to buildable (measured, run 10). Lead every report
with `blocking`, and use `collateral_pad_rms` as the one recovery figure that
signals a real defect — it means parts nothing had damaged were moved.

**The handoff gate that matters is off-board pad copper, not total
`blocking`.** A part whose pad copper lies outside the outline makes its nets
unroutable, so it converts one-for-one into `unrouted` and `broken` — measured,
run 10: 11 such parts caused all 13 unrouted nets. Total `blocking` is the
wrong gate here in the other direction too: on a copper-free board it is
dominated by `unrouted`, which is the routing half's own job and not a
placement residue at all.

**Feed each gate the document it was written for**, and check that what you
passed actually carries the keys it reads. Different tools in this chain use
the field name `blocking` for different quantities, so a gate handed the wrong
report can no-op on the checks whose keys are absent and still refuse on the
one key that collides — with a message naming the wrong cause. When a
blanket-waiver flag exists, remember it waives the checks that were working as
well as the one that misfired.

<non_negotiable>
1. Placement runs FIRST and ONCE. Every routed board downstream of a placement
   change is stale -- re-run the chain from the placed board, never patch
   around it.
2. A routing failure is CLASSIFIED before it is retried: parameter-shaped,
   placement-shaped, or floorplan-shaped. Retrying a placement-shaped failure
   with router parameters is the most expensive mistake available here.
3. The placement half's acceptance gates still apply to a placement produced
   mid-chain. A board that reaches routing with a blocking assembly pair, a
   locked-part contact, or a rigid inconsistency is a board that will fail
   routing for reasons routing cannot fix.
4. Copper is not evidence about placement. A route that completed does not
   ratify the placement it ran on; a route that failed does not condemn it.
   Classify first.
</non_negotiable>

## How to run this skill

Ask the driver for one stage at a time. That is not a style preference here: the
two halves have DIFFERENT accept rules -- placement accepts a lap when the named
finding it aimed at is gone, routing accepts an iteration when `blocking`
strictly decreased -- so an executor holding both skills at once carries two
contradictory definitions of "better". The driver never emits both.

```bash
D=.claude/skills/plan-pcb-placement-and-routing/scripts/loop_driver.py
python3 -X utf8 $D --list
python3 -X utf8 $D --stage L1 --board board.kicad_pcb --ledger wk/ledger.jsonl
```

Its guards are the three this skill exists to enforce:

| stage | refuses without | because |
|---|---|---|
| `L2` route | a placement close-out | routing cannot start on a placement nobody proved, and a board with a blocking pair fails for a reason routing cannot fix |
| `L3` classify | a routing score | a retry without a classification is a guess |
| `L4` re-enter | a measured `--shape` | the three shapes re-enter at three different points, and the cost of guessing is asymmetric |
| `L5` close out | a `check_complete` close-out that **agrees** with `converge` | nothing refused to FINISH, so a run reached the terminal artifact having never entered routing's own V1–V5 loop, and shipped a power-to-signal short. Only the *contradiction* refuses: `DONE-EXHAUSTED` against `INCOMPLETE`/`UNSOUND` |

**Both inner halves go to a teammate. Always, at every board size.** You do not
decide it and you cannot forget it — the driver reads the board, delegates, and
prints the size it measured as context.

**This used to be a CONTEXT decision on two size thresholds, and it is now a
CORRECTNESS one.** The thresholds are gone. Run 14 is why: 191 pad-bearing
parts against a 200 cut and 150 nets against 300, so both halves ran inline —
and the routing half, holding far more context than the orchestrator,
classified its own failure in its V2 stage and acted on it. The classification
never travelled up, `L3` and `L4` never fired, and a full re-run of the routing
chain happened that the outer loop never authorised and never saw.

That is not a threshold set wrong. **An inline inner loop can silently do the
outer loop's job**, because it always knows more than the parent does, and no
number separates the boards where it will from the boards where it will not. A
teammate cannot: the only thing crossing the boundary is a document the parent
has to read.

`--no-delegate` is the escape hatch and is load-bearing — the self-test, the
parity gates and headless CI need one process. `--delegate` is the explicit
form of the default and changes nothing. A board that cannot be read still
delegates; the size was never the decision, so failing to measure it changes
only what can be said about it.

Spawn the teammate with an agent type that **has the Agent tool** — `claude` or
`general-purpose`, never `Explore` or `Plan`, whose definitions exclude it. Each
half dispatches its own verification subagents at its close-out, and a half that
cannot spawn cannot verify itself. (The older wording here said a subagent
cannot spawn a subagent. That is false in this harness and has been retired; the
constraint is the agent *type*.)

State crosses the boundary on DISK, in the converge ledger, never in a head.
That is what makes a re-entry able to say what was already tried.

## The sequence

1. **Place.** Follow `/plan-pcb-placement` to its close-out. It ends with a
   board whose copper-free `check_drc` and `check_assembly` are clean, or with
   the residue NAMED and measured as unfixable.
2. **Freeze what the placement decided.** Lock the refs whose poses are
   decisions (mechanically fixed parts, anything a spec pins). A later step
   that moves them silently undoes the placement work.
3. **Route.** Follow `/plan-pcb-routing` from Step 1 on the placed board. Its
   Step 0 gate will pass, because you just did that work.
4. **On a routing failure, classify before retrying** (the routing skill's
   convergence section owns the classifier):

   | the diagnosis says | re-enter at |
   |---|---|
   | parameters (grid, rip-up depth, layer costs, width) | the failing ROUTING step |
   | congestion at a fine-pitch part's escape face | placement -- the face has no lanes, and no router setting adds any |
   | a part sits where its net cannot reach | placement |
   | a spec clause a different arrangement would satisfy | placement, with the clause stated as intent |

5. **Re-entering placement restarts the chain.** Go to 1, and discard every
   routed board produced from the old placement.

## When the loop is the tool

For congestion-shaped failures the repo has a loop that alternates the two
halves under one budget:

```bash
python3 -X utf8 py_placer/place_route_loop.py board.kicad_pcb out.kicad_pcb \
    --rounds <N> --target-nets "<the nets the failure named>" \
    --accept-cmd "<the command that tells better from worse>"
```

Both flags matter: without `--target-nets` the loop moves parts unrelated to
the failure, and without `--accept-cmd` its comparator cannot see the thing you
are trying to fix (a length, a width, a clause). An ACCEPTED round is not a
verdict -- grade the output with the same battery either skill would use.

## Eyes at the boundaries (run-23)

Numbers gate legality; nothing gates LOOKING, and run 23 shipped a board a
human rejected at a glance — 15 courtyard interpenetrations and four mid-board
connectors — while every key read clean. Its orchestrator viewed ONE image in
4.7 hours, after the failure. So LOOK at each boundary (the placement close,
the hand-off, after the first route lap, the final close), and look
BLIND-FIRST: build the sheet with `render_placement.py --review-sheet
<PATH>`, VIEW it, write your observations — connectors versus edges with
distances, density pockets versus empty regions, anything wrong that no key
names — **before reading any checklist key**, then write a reconciliation
paragraph dispositioning each observation against a named number. Ordering is
the whole mechanism: a reviewer who reads the keys first has a closed question,
and run 23's did read past "overlap 26.30mm²" printed in the image banner
because the keys had already said clean. Observations must carry distances or
mm² the keys alone cannot produce; that is what separates a review from
theater.

## What a run DELIVERS

Four artifacts, every time, in the work dir. A run that produces the board alone
is not finished — the other three are how anyone else can tell whether the board
is good, and they are the first thing to get skipped under time pressure.

1. **The board** — the final `.kicad_pcb` WITH its sibling `.kicad_pro` (the DRC
   floor rides in the project; a board without it is ungradeable, #441). State
   its sha256 and which chain step produced it.
2. **The movie** — `python3 -X utf8 make_movie.py <work-dir>` over the chain
   boards. `place_route_loop` makes one by default; a hand-driven chain does
   NOT, so build it explicitly. `KICAD_ROUTE_TRACE=1` (the default) gives the
   fine per-copper rip/restore animation.
3. **The report** — `REPORT.md`, and it compares on TWO axes or it is not a
   report:
   - **against the human**, when a human-routed reference exists:
     `compare_to_original.py --ours <final> --orig <reference> --json` (vias,
     copper length, width spread, layer balance). The human layout is one
     solution, not the only one — it is a benchmark to approach, never a pose
     to match.
   - **against prior runs of the same board**, by re-deriving their numbers
     with TODAY's graders. Never diff against numbers stored in an old report:
     the graders here drift within days, and a re-grade has moved rows in both
     directions.
   Lead with `blocking`; quality (vias, copper_mm, segments) is the tie-break
   once blocking is 0.
4. **The journal** — numbered entries, written as you go, each carrying the
   measurement behind it and the command that produced it. Batch-writing it
   afterwards is detectable and has been detected.

## Blind subjects, and the fence

When the subject is a PERTURBED board (the #411 recovery rig), truth never
enters the work dir:

```
wk/<run>/<subject>/     the damaged board -- the only board you may open
wk/<run>/_truth/        control + record -- unreadable until the board is frozen
```

`placement.perturb.perturb(..., control_out=...)` puts the control and the
record (it embeds `original_poses`) there in one call. Audit by CONTENT before
starting and again at the end — `tests/stress/fence_audit.py --mode create` /
`--mode audit`; exit 4 is a leak, and a control inside the work dir is a leak
whatever it is named. Full recipe: `tests/stress/RUNBOOK.md` → "Staging a
perturbed subject".

If a previous run already opened this subject's truth, say so up front and
report `recovery` / `home /N` as diagnostics. They are not blind any more, and
presenting them as a score would be a claim the fence no longer supports.

<agent_identity>
You run a board end to end. You place first and once, you classify every
routing failure before retrying it, and you send placement-shaped failures back
to placement instead of spending router retries on them. You finish with four
artifacts — board, movie, report, journal — not one.
</agent_identity>

<!-- Moved here from plan-pcb-routing/SKILL.md: the convergence loop scores a
     PLACED-and-ROUTED board and iterates over both, so it belongs in the skill
     that sequences the two. plan-pcb-routing is routing-only again. -->

## Step 9: converge — score the board, pick a lever, repeat

**Full procedure, worked example and ledger schema:
[`references/convergence.md`](references/convergence.md).** The summary below is
the part you must not get wrong.

**A chain that ran is not a board that is done.** The failure this step exists to
prevent is concrete: a board went out at **39 of 44 nets connected, 762 DRC
errors, and 141 of 141 vias below its own spec**, and every tool in the chain
reported success. Nothing looped back, because nothing had measured the board.

#### 9.1 — Score it. The router's opinion is not evidence.

```bash
python3 -X utf8 .claude/skills/plan-pcb-placement-and-routing/scripts/board_score.py \
    board.kicad_pcb --intent floorplan.json \
    --min-track-width 0.15 --min-via-diameter 0.6 --min-via-drill 0.3 \
    --net-min-widths wk/net_min_widths.json \
    --impedance-nets '<every net with a reference-plane clause>' \
    --length-groups '<every length-matched group>' \
    --json wk/score_iter3.json
```

**Every one of those flags is what makes its clause reach `blocking`. A component
with no flag reports `ungraded`, which is not a pass.** The pattern is identical
each time, and it is how a HARD clause ships unmeasured:

| flag | without it | measured worth on one board |
|---|---|---|
| `--net-min-widths` | `undersized` sees only BOARD-WIDE floors, so a clause naming ONE net — a 0.8 mm pair, a 0.4 mm rail — is invisible | `net_widths` 5, while `undersized` read 0 |
| `--impedance-nets` | the component returns *"no --impedance-nets given"* and a plane-continuity clause is never checked at all | `impedance` 10 — 68 reference crossings, 63 segments over void |
| `--length-groups` | length matching is ungraded | — |

Same board, same copper: **`blocking` 12 without those flags, 27 with them.** A
run that reports 12 has not found a better board; it has looked at less of it.

**Glob-list flags take SPACE-separated patterns** (`--impedance-nets 'USB_*'
'QSPI_*'`); comma-joined tokens are also split, and a token matching NOTHING
routes the impedance component to `unknown` → **exit 4**, never a silent pass
(run 5 scored several iterations with a comma no-op before this was fixed —
the score was vacuously clean). **Assert non-vacuity on the FIRST scoring run**
of every chain: check `impedance.nets_analyzed` equals the number of nets you
named, exactly as you assert `ran == true`. A vacuity discovered at iteration
9 invalidates every earlier score.

Also **read `net_widths.patterns_matching_no_routed_net`.** A width clause on a
net with NO copper never appears in `net_widths` — the component only walks nets
that HAVE segments — so an unrouted net's width requirement lands in that list
and nowhere else.

**And `blocking == 0` is not the whole gate when the repo ships its own spec
checker.** Some clauses are not expressible to `board_score` at all: an absolute
maximum length, a symmetry match between two *series chains* through a resistor,
a via ban per leg. If the repo has a `check_spec.py` (or equivalent), run it
**beside** `board_score.py` every iteration, treat a HARD failure as blocking even
when `blocking` reads 0, and wire it into `place_route_loop --accept-cmd` so the
inner loop stops accepting rounds that break it.

**Produce:** the command above, every iteration, on the board you just wrote.
**Read:** `blocking`, `blocking_by`, `ungraded`, `unknown`, `quality`.
**Decide:** `blocking == 0` → go to 9.4. Otherwise pick the lever by **9.1a**,
NOT by the largest `blocking_by` entry.

##### 9.1a — CONNECTIVITY FIRST. The largest number is not the lever.

The obvious rule — *"the biggest `blocking_by` entry names the lever"* — is wrong,
and it wrecked a run. On a board with 5 nets carrying **no copper at all**, the
biggest entry was `drc: 18`, of which **16 were grading artifacts**. The loop spent
eleven iterations on clearances while five nets sat dead, and the board could not
have booted.

**Work the components in this fixed order, regardless of size:**

| order | component | why it outranks the rest |
|---|---|---|
| 1 | `unrouted` | a net with no copper is a dead wire. Nothing else matters while one exists. **Run `converge.py where BOARD --nets <names>` before touching a parameter** — it names the gap endpoints and the foreign copper walling them in, per layer, nearest-first (9.1b-ii). **And READ the focus panels** — image read-case 3: `render_placement --summary-json wk/routeN.json --focus` classifies pocket-vs-scattered in one look, BEFORE the first lever. Guessing from the score is how eleven iterations went to clearances while five nets sat dead |
| 2 | `broken` | a net in N pieces is N−1 dead wires. **Read `components.broken.nets`, not the count** — see below; the count alone is not a work list and a loop driven on it does not move |
| 3 | `net_widths`, `undersized` | real copper, wrong size — fixable by re-routing what is already there |
| 4 | `floorplan` | placement or intent |
| 5 | `drc` | **last, and only after auditing it — see below** |

`unrouted` and `broken` are the **ratsnest**: they count connections the board is
supposed to have and does not. They are never artifacts, never a grading choice,
and they map one-to-one onto whether the thing works. Drive the loop on them.

##### 9.1a-ii — `broken` needs a WORK LIST, and one tool per class

`unrouted` is actionable from its names alone: the net has no copper, so route
it. **`broken` is not.** Before you can act you need three more things — which
net, how many pieces (a 5-way split and a 2-way split are different jobs), and
*where* the stranded pads are. Measured failure mode: a run drove `unrouted` to
0 using `net_widths`' per-net detail as its model, then left `broken` at **14
across two iterations**, because `blocking_by.broken: 14` is a number with
nothing behind it.

`board_score.py` emits the list under `components.broken.nets`:

```jsonc
"GND":      {"components": 5, "joins_needed": 4, "handler": "repair_planes",
             "stranded_pads": [{"x":137.72,"y":66.05,"layer":"F.Cu","ref":"SW1"}, ...]},
"VCC1V1":   {"components": 4, "joins_needed": 3, "handler": "route",
             "stranded_pads": [{"ref":"U1"}, ...]},
"FLASH_CS": {"components": 2, "joins_needed": 1, "handler": "route",
             "stranded_pads": [{"ref":"R1"}]}
```

`joins_needed` sums exactly to `blocking_by.broken`, so the list is complete and
you can see what each entry is worth. **Sort by it** — above, GND alone is 4 of
the 14, and seven single-join nets are worth 1 each.

**`handler` names the step, and it is a FACT off the board, not a guess:** it is
`repair_planes` when the net has a zone (see `broken.poured_nets`,
read from the board's own `(zone (net "…"))` blocks) and `route` otherwise.
`route.py` cannot tap a pour, so a stranded plane pad handed to it is work that
cannot succeed. Measured: `broken` sat at **14 across two iterations** of
`route.py` calls and fell to **11 in one** `repair_planes` call once
the plane net was separated out.

**`poured_nets` is that handler decision and NOTHING else — it is not a list of
plane nets, and it is not a safe `--ignore-nets` population.** It means "this net
has at least one zone", which on a board that pours signal nets includes them:
measured on neo6502, its 61 nets covered **332 of 545 pads (72%)**, all of
`/A0`–`/A15` among them. A run read it as "the planes, ignore those" and removed
most of the board from its own render. The field publishes this sentence itself,
as `broken.poured_nets_meaning` — read it there rather than inferring from the
name. Every net name `board_score` publishes is now checked against the board
(`net_name_audit`); a non-zero `unknown_count` is a bug in the instrument, not a
finding about the board.

**One tool per class — they are not interchangeable, and using `route.py` on all
of them is why the count does not move:**

| the break | the tool |
|---|---|
| a **plane net** (GND, any poured rail): stranded pads that cannot reach the pour | `repair_planes --rip-blocker-nets`. `route.py` will not tap a pour |
| a **multipoint** signal/power net: some MST edges landed, one did not | `route.py --nets <that net>` — and read **`failed_multipoint`**, which is where its failure is reported |
| a break whose stranded pad sits on a **DNF / do-not-fit** part | **not a defect.** Chasing it never converges. Say so once, with the ref, and exclude it from the target set |
| a break at a fine-pitch pad with no room for a tap via | smaller `--via-size`/`--via-drill`, then finer `--grid-step` — the Step 5 ladder |

The `ref` on each stranded pad is what tells these apart, which is why it is in
the list. A break on `[R1]` where R1 is unpopulated and a break on `[U1]` are the
same number and completely different work.

##### 9.1b — Audit `drc` before you believe it

(And when two DRC instruments disagree, audit the PASSING one first — confirm
it implements the clause in code, not just in a docstring; see the
cross-instrument rule in the verifier section. Reconciling them is a
`--kind systemic` ledger entry.)

`check_drc` grades the whole board at **one clearance**. A board with more than one
net class therefore reports violations that are purely a grading choice. The
signature is unmistakable — **many violations, all the same net pair, all the same
overlap**:

```
SEGMENT-SEGMENT  USB_DM <-> USB_DP   Overlap: 0.010mm     x15
PAD-SEGMENT      USB_DP  <-> USB_DM   Overlap: 0.010mm     x1
```

0.010 mm is exactly `0.16 − 0.15`: the Default class grading a pair whose own class
permits 0.15. Re-grade at the tighter class and they vanish:

```bash
check_drc.py board.kicad_pcb            # 18 violations
check_drc.py board.kicad_pcb -c 0.15    #  2 violations
```

**Before letting `drc` drive anything:** group the violations by (type, net pair,
overlap). Any group that is large, uniform, and sits within a µm of a
class-clearance difference is an artifact of the grading scalar, not a defect.
Quote both numbers in the report and say which classes each applies to. Never pick
the flattering one silently.

##### 9.1b-ii — Tools that already answer this, which nothing in a chain calls

A whole convergence went by hand-rolling worse versions of four of these. Before
writing a script to answer a question, check whether one of them already does.

| you want | run | why it beats the obvious thing |
|---|---|---|
| where is the gap, and what is walling it in | `net_forensics.py --nets N --radius 1.0` | per net: the connected ISLANDS, the exact unclosed gap endpoints, and an inventory of the foreign copper around each gap — **named, per layer, nearest-first**. Better than a ratsnest, which tells you two pads are unjoined and nothing about why |
| the honest unconnected count | `kicad_unconnected.py board --items` | KiCad's own DRC, and it **refills the zones itself** — which is 9.1c's whole problem, already solved. Exit 4 = items remain, 3 = no oracle (NOT clean) |
| WHERE the DRC violations sit, as a picture | `check_drc.py board --render wk/drc/` | one cropped panel per spatial cluster, red rings at each violation, count/types/rect in the caption — image read-case 7. The panel shows WHERE; the violation records say how much |
| the endgame work list, join by join | `kicad_unconnected.py board --pairs-json wk/pairs.json`, or `converge.py where BOARD --oracle` | each remaining join as an exact net + pad↔copper endpoint pair (x/y/layer/kind) — the JOIN SPEC for a scoped route, no re-deriving from prose. `where --oracle` prints the pairs then runs forensics on exactly those nets |
| what kind of failure is this | `converge.py where` / the router's own hint | the hint names the flag and the nets (9.3b); it diagnoses better than the score does |
| where should this part go, facing which way | `converge.py poses BOARD --ref R` | ranks legal (x, y, rotation) poses by placement cost in **milliseconds**, with a per-component breakdown, and `--route` pays for tier 3 on only the top few |
| will this hand join fit, BEFORE committing it | `check_join.py BOARD NET x,y,layer ... via:x,y` | stages the candidate polyline+vias onto a copy of the board and diffs the REAL check_drc engine (netclasses, `.kicad_dru`, rotated pads, edge, hole-to-hole), plus missing-via and same-net-stack checks DRC omits. Exit 0 clean / 1 violations. Rung 8's condition 3 |
| is this even the engine I pinned | `route.py --capabilities` / `krt_capabilities.py --require` | a chain can otherwise run green against a clone missing the module it depends on. **Spelling is `module:--flag`, WITH the dashes.** And ground-truth a PLANE-step flag with `--help`: `--require` scans imports one level to catch shared registrars, and both plane scripts import `route.py` — so they used to inherit its whole vocabulary and answer OK for flags argparse rejects with exit 2 (fixed, but the lesson stands: a capability gate is evidence, not proof) |
| step back to iteration N | `converge.py step-back --iteration N` | byte-exact, because the board is addressed by content instead of by a path three iterations overwrote |
| re-run what iteration N did | `converge.py replay --iteration N` | replays the recorded argv. If it refuses, the ledger recorded prose instead of a command — fix the ledger, not the memory |

**Trust order when instruments disagree on connectivity: the KiCad oracle
(`kicad_unconnected`) > `net_forensics` islands > `board_score` components >
route.py's own JSON_SUMMARY tallies.** The router's multipoint model reported
25/27 pads connected on a net the oracle showed in **7 islands** (run 6,
VCC3V3). Router tallies pick the next lever; **only the oracle accepts an
iteration.** When a tally and the oracle diverge, that divergence is itself a
`--kind systemic` finding — file it, don't average it. (route.py now runs the
oracle itself at end of run — `oracle_check`/`oracle_open` in JSON_SUMMARY —
and a `fragmented_nets` key names pad-connected nets whose copper is several
KiCad islands. `oracle_open` feeds its own reconciliation; `fragmented_nets` is
DISCLOSURE ONLY -- it names the splits, nothing re-queues them, so a fragmented
net there is YOUR next action (route it by name). `oracle_check: unavailable`
means the run had no kicad-cli and you are on in-process grading alone.)

**The divergence is a MODE, not an event.** After the model's success channel
is caught lying ONCE on a board (`failed_single`/routed tallies vs oracle
opens), demote it for the remainder of that board's endgame: the oracle's
`--pairs-json` work list is the only open-set, every accept runs the oracle,
and after every FAILED call diff the board itself (per-net segment/via
counts, duplicate vias at identical coords) before trusting "no change" — a
failed rip-restore can write fragments while reporting success. Run 7 held
the trust order per event but kept consuming the model's tallies lap after
lap, and paid one oracle run of latency for each of three defect classes.
(The engine now counts routed-but-OPEN nets in `open_single`, grades
terminal restores before claiming them — `terminal_restores` — and dedups
stacked vias with a `stacked_copper` disclosure; a summary carrying those
keys has already had this class of lie audited once, which narrows the gap
but does not repeal the trust order.)

**Ratchet floors measured by a refill-jittery instrument (kicad-cli DRC) flap
at exact equality.** (1) A single exit-4 at floor==count is a RE-MEASURE
event, not a regression — re-run the checker once before reverting anything.
(2) Register or lower a floor only to a count observed twice. (3) Name
at-equality rules in the promote note as jitter-exposed, and keep the flap
log.

##### 9.1c — The authoritative ratsnest needs the zones FILLED

`route_planes` writes a zone **outline** with no `filled_polygon`. Until something
fills it, every KiCad-side check reads the pour as empty. Measured on one board,
same file, fill the only difference:

```
unfilled -> kicad-cli pcb drc:  48 unconnected items
filled   -> kicad-cli pcb drc:  15 unconnected items   == what check_connected says
```

So: **fill before you grade, and then the two agree.** If `check_connected` and
`kicad-cli` disagree by a lot on a board with a pour, the fill is the first
suspect — not the checker.

**Use `kicad_unconnected.py`, which refills for you** — it exists precisely for
this, and a hand-rolled fill has a trap the tool does not:

```bash
python3 -X utf8 py_tools/kicad_unconnected.py board.kicad_pcb --items
```

For the ENDGAME — the last few opens after the all-nets route — add `--pairs-json` (or run
`converge.py where BOARD --oracle`): it writes each remaining join as an exact
net + endpoint pair, which is the work list a scoped single-net call consumes.
Run 5 spent its endgame re-deriving those endpoints from the `--items` prose,
one join at a time.

If you must fill in place (to hand a filled board to something else), note that
`pcbnew.LoadBoard(...).Save()` **rewrites the sibling `.kicad_pro` and deletes
every non-Default net class**, leaving the netclass patterns orphaned. A board has
shipped that way. Restore the project afterwards and assert the classes are back
rather than trusting a success message.

**And re-assert the net classes afterwards.** `pcbnew.LoadBoard(...).Save()`
rewrites the sibling `.kicad_pro` and **deletes every non-Default net class**,
leaving the `netclass_patterns` orphaned. A board has shipped that way. Restore the
project after the fill, and assert the classes exist rather than trusting a
success message.

Three rules about that number:

- **`blocking` must reach 0 before a board is deliverable.** It is
  `unrouted + broken + drc + undersized + floorplan + impedance + length`.
  `quality` (vias, copper length) is a **tie-break only**, compared once
  `blocking` is 0 — otherwise a router buys off a disconnected net with a lower
  via count.
- **Pass the spec's size floors when the spec is tighter than the fab.**
  `check_drc` defaults to the fab minimum for the layer count. That is why 141
  vias at 0.25 mm graded clean against a 0.6 mm spec. If the spec gives numbers,
  pass them.
- **`ungraded` is not `passed`.** A component with no `--intent`, no
  `--impedance-nets`, no `--length-groups` is *unexamined*. Say so in the report;
  never let it read as clean.

**`place_route_loop`'s own `ACCEPTED` / `REJECTED` is NOT a quality verdict.**
`better()` (`place_route_loop.py:358`) compares `failures` and `iterations`, both
from route.py's own `JSON_SUMMARY`; it never runs a checker. Treat it as a cheap
pre-filter and **re-score with `board_score.py` before believing it.**

**It is also spec-blind, and `--accept-cmd` is the fix.** `better()` compares
failures then iterations; nothing in a route summary tells it a net exceeded a
maximum length, took a via where none is allowed, came out under a required
width, or drifted a decap past a proximity limit. On a board with a real spec
those are what decide whether a placement improved, so the loop will accept a
round that broke one and print ACCEPTED. Pass
`--accept-cmd 'CMD'` and the loop asks your judge instead:
`CMD <placed> <routed> <route.json>` printing one line `SCORE=<float>`, lower
better; a non-zero exit or a missing SCORE rejects the round.

#### 9.2 — Budget: 100 iterations per board, and they are cheap if you spend them right

**The budget is 100 per board.** Not 20 — 20 was set when every iteration meant a
full chain re-run, and that assumption is wrong (9.3a). A scoped retry takes
seconds, so a hundred of them is an afternoon, not a week.

**Count three kinds separately, and say which you are spending:**

| kind | what it does | example |
|---|---|---|
| **completion** | changes the copper: routes a net, heals a separation, fixes a width | `route.py --nets QSPI_SD1 ... --rip-existing-nets ...` |
| **placement** | moves footprints: a quench, a repair, a reconstruction — connects nothing, tunes no instrument | `place_seed --repair`, `place_reconstruct`, a 0c quench, a loop round |
| **systemic** | changes how the chain routes, measures or grades — no net gets connected by it | pinning the fab floor, restoring net classes, filling zones, fixing a checker |

(`placement` exists because two runs had to file placement repairs as
`systemic` for want of a kind, and `status`'s systemic-share warning cried
wolf about runs that were spending their budget on the board.)

Systemic iterations are necessary and they are not progress. A run once spent
**nine of eleven** on them, moved `blocking` every time, and finished with five
nets carrying no copper. **If three consecutive iterations are systemic, stop and
ask what is actually unconnected** — you are tuning the instrument, not the board.

Record `"kind": "completion" | "placement" | "systemic"` in every ledger entry.
The final report states all three counts.

```bash
python3 -X utf8 py_router/route.py board.kicad_pcb --list-groups --group-by auto
```

**The per-group budget needs groups that are separately convergeable — not just
groups that exist.** Test each candidate against all three:

1. its parts occupy a **distinct region** (not interleaved with other blocks),
2. its nets are mostly **internal** (`--list-groups` prints touching/internal),
3. routing it can **succeed or fail on its own**, without the others' copper.

Fail any of them and it is a *label*, not a convergence unit: take the per-board
budget and say so. A board of functional modules sharing one congested centre is
the common case — iterating per module there routes a fraction and reports
success on that fraction, which is the same defect the `route.py --group` rule
warns about.

**`kicad` groups exist on 0 of 27 boards *in this repo's corpus*** — that figure
is about KRT's own test boards, not about boards in general. A generated board
(e.g. Zener `.zen`) carries one `kicad:` group **per module**, so the naive
reading of "groups exist → per-group" authorised **8 × 20 = 160 iterations** on a
42-part board whose modules all fight over the same 21 mm of width. Take the
per-board budget there.

**Do not invent groups to iterate over** — a `sheet` block of 16–83 parts moved
on no board tried, so iterating per sheet-block burns the budget on a lever that
does not move.

#### 9.3 — Cheapest lever first, and revert what did not help

##### 9.3a — RE-ENTER AT THE FAILING STEP. Do not re-run the chain.

The single most expensive mistake available here. A full chain run is 3–5 minutes;
re-routing three nets from the board that failed them is **seconds**. The ledger
already records `parent_sha` per iteration precisely so you can go back to it
(`converge.py step-back` checks it out byte-exact).

```bash
# NOT: bash chain.sh          (re-seeds, re-places, re-routes everything)
# THIS:
route.py wk/r4.kicad_pcb wk/i15.kicad_pcb --nets QSPI_SD1 ...
```

Re-run the chain only when a **placement** changed (which invalidates every routed
board downstream) or when you are producing the final artifact. Everything else is
a scoped retry on the board that already failed.

##### 9.3b — READ THE ROUTER'S HINT. It names the flag and the nets.

When `route.py` fails a net it prints the fix, and it is usually right:

```
ROUTE FAILED - no rippable blockers found
  Hint: the blocking copper belongs to pre-existing net(s) 'QSPI_SD2' 'QSPI_SS'
  'VCC3V3' (committed by an earlier run/step), which this run is not allowed to
  rip. Retry with --rip-existing-nets 'QSPI_SD2' 'QSPI_SS' 'VCC3V3' ...
```
```
  Hint: the start/target pads are boxed in by static obstacles ... try
  --grid-step 0.025 --clearance 0.15 --track-width 0.15
```

On one board these two hints, applied, took `unrouted` from 5 to 0. The router
diagnoses better than the score does — the score said `drc`, the router said
"rip these four nets", and the router was right.

##### 9.3b-ii — Carry `--fab-overrides` on EVERY retry when the spec floor is tighter

A scoped retry is a fresh `route.py` call, and it resolves its floor from the fab
tier unless told otherwise. Two things then happen quietly: the **per-net rescue
re-routes a failed net AT the tier floor**, and the `standard`→`advanced` tier
escalation is allowed (the default `--fab-tier auto` / `--escalation fab`), which
is what puts sub-spec vias on a board that asked for big ones. Both report the net
routed, and both are counted in the run's `design_rules` summary; `--fab-tier
standard` or `--escalation board` forbid them outright, `--strict-sizes` makes
them a non-zero exit.

So every route call in the loop — not only the first one — carries
`--fab-overrides <the spec file>` when the spec is tighter than the tier.
Measured, one such file took a board's `undersized` from **169 to 0**. Check
`min_clearance_used` in the `JSON_SUMMARY` afterwards: it is the only place a
floor that was silently loosened shows up.

A width clause rides on `--track-width` (or the board's Default netclass, which
is where `route.py` reads it from when the flag is absent). **There is no hard
per-net width floor any more**: `--track-width-floor` was removed in 53a5a16e
along with the two engine behaviours it drove, so the rescue path re-necks at
the class-aware `min(nominal, fab_track, netclass)` with no floor guard.
Passing the flag is an argparse error — exit 2, mid-chain, with the lap lost.

##### 9.3c — Ripping blocking nets IS a sanctioned lever

`--rip-existing-nets` rips named nets, re-routes them in the same run, and reports
honestly if one cannot be. It is often the **only** way past copper an earlier step
committed. Use it — with four rules, each of which cost a wasted iteration to
learn:

1. **Scope the rip.** Start with the set the hint names, then bisect if you want a
   minimal one. Do not reach for `'*'` — and subtract **large multipoint rails**
   from the hint set before using it: rip a rail as collateral and every one of
   its pads opens at once (run 5: one collateral rail rip opened **19 pads** and
   cost the iteration). Rip leaf/2-pad nets by exact name; a rail that truly
   blocks gets its own deliberate, single-net call.
2. **A ripped net returns at the CALLING command's parameters, not the ones it was
   originally routed with.** Ripping a 0.8 mm USB net from a plain signal call
   brings it back at 0.16 mm and silently destroys the spec geometry. **Whenever
   the rip set contains a width-bearing net, pass its `--power-nets` /
   `--power-nets-widths` (or `--impedance`) in the same call.** And the rule
   does not extend to dru rules — a net routed under a staged/lifted dru
   cannot be re-made by any call that reads the full sibling dru (see Step 5's
   dru-has-no-pin bullet).
3. **One net per call.** Routing two nets together let the second rip the first —
   reported as `1/2 routed` twice running, a different net each time. Sequential
   single-net calls connected both.
4. **A glob does not override a lock.** `--rip-existing-nets 'QSPI_*'` silently
   skips a locked or protected net (#521) while the router keeps asking for that
   exact rip. Name it EXACTLY (the exact-name override now reaches the in-run
   ladders too, not just the pre-run filters), and if it is KiCad-locked,
   nothing overrides that — unlock it or route around it. To protect a
   SINGLE net YOU verified (not just matcher-produced ones), there is no flag
   — `--protect-nets` was removed in 53a5a16e. Protection is recorded by the
   step that routes a matched group or a diff pair, it persists in the
   `.kicad_pro` under `kicad_routing_tools.protected_nets`, and every later
   step's rip machinery honors it. **A GROUP routed together is NOT protected
   on its own pass**, and must not be: the
   protection binds that same call's in-run ladder, so the bus can no longer
   rip/reorder itself (measured, run 6: 6/7 with `protected_skipped` on the
   group's own QSPI pass; 7/7 without). Protect the group on the NEXT
   committing step instead; the `.kicad_pro` record carries it from there.
5. **Tap passes over routed copper are a one-way door.** With the pour-first
   order (#424) the Step 1c pour ambushes nobody — its taps land on an empty
   board and every later route sees them from the start. The door is the
   LATE tap passes: the plane FINALIZE (`--add-gnd-vias`/`--stitch-*`), the
   repair, and any re-pour over a routed board. Never name a
   geometry-constrained net in a rip set after those have run: the corridors
   it used are gone (run 5 measured the static frontier at **10,903/10,939
   cells** around the wrapped nets — the rip could only put the copper back
   where it was, minus luck); their rips are for freeing a blocked pad,
   never for improving a route. **The door also closes on UNROUTED pads**:
   a tap carpet consumes every open pad's escape channel and is not
   rippable copper (run 6: 5 bare pads at a late pour, never recovered) —
   and no gate stops you: the exit-3 refusal and its `--allow-bare-pads`
   override were removed in 5832e4eb (the empty-board 1c pour was the
   exempt case). Connect every pad first, or accept losing it. (Cross-ref:
   the Step 5 ordering block says the same from the other side.)

For plane-net pads that cannot reach their pour, the equivalent is
`repair_planes --rip-blocker-nets` (out-of-chain only; it leaves the ripped
nets unrouted for a following `route.py` pass — never re-route them in-step,
#141. In-chain, route.py's own finalize does the whole rip-and-reconnect). Budget for it: on a
dense board it can run **20× longer** than the plain repair, so start it early
rather than discovering the cost at the end.

##### 9.3d — Classify the blocker, then pick

Never spend a full-chain iteration on something a parameter fixes. Classify the
top blocker on the exact keys, not on impressions:

| evidence | verdict | where to go |
|---|---|---|
| failures cluster into ≤2 pockets (`--focus` panels), their refs share one block, `blockers` non-empty | **floorplan** | back to **Step 0e** — re-zone. A 3 mm nudge cannot move a block 80 mm |
| failures scattered, `blockers` non-empty, every failing ref is a ≤40-pin passive | **placement detail** | back to **Step 0c**, `place_route_loop` with the caps above |
| `blockers` empty; the log says boxed in by static obstacles | **parameters** | stay here — grid, ripup budget, width. Placement is not the lever |
| 2-layer board, heavy F.Cu skew, via count far above a hand layout | **parameters** | layer-cost rebalance, below |
| `oob_count` or `overlap_area` rose after the last placement | **the placement is illegal** | discard it; do not route it |
| `check_floorplan` exits 4 with `zone_containment` | **intent violated** | fix the placement to match, or say why the intent changed. Do not quietly rewrite the intent to match the board |
| a whole net has no copper while `pad_pairs_connected` looks healthy | **coverage bug** | the Step 5b ledger — not a placement problem at all |
| `undersized` non-zero | **parameters** | re-route at the spec's width/via. Placement is not the lever |
| a **maximum-length clause fails** and the net's own geometry pass ran at the default `--heuristic-weight` | **parameters — rung 1, seconds** | 1.9 is inadmissible; it returns a path up to ~1.9× optimal. Re-run **that pass**, on **its own input board**, at `--heuristic-weight 1.0` with a finer `--grid-step` (the #529 dynamic budget self-extends; do not pass `--max-iterations`), then re-measure routed:straight-line. Measured: 44.50 mm → 7.73 mm against a 7.71 mm direct. **Do not go to placement before this.** See Step 2c |
| `--heuristic-weight 1.0` **on the net's own FIRST pass**, on a board carrying only what must precede it, did not change the length | **placement** | now the router genuinely had no shorter path. Signature: routed length far above the straight-line pad distance *and stable under an admissible search*. Go to `place_route_loop` — see the warning below, it needs BOTH `--target-nets` and `--accept-cmd` to see this at all. **A null measured on a SATURATED board proves nothing** — one run tested 1.0 at iteration 4, after fanout, USB and every signal were committed, got a byte-identical board, and recorded "no shorter path exists at this placement". Re-tested on the first pass that lays the net's copper, the same flag was worth 5.8× |
| `unrouted` names a plane net | **the pour step** | it was excluded and never poured — Step 1c (or the Step 3 finalize / Step 5 repair), not placement |
| the log names **pre-existing nets** it is "not allowed to rip" | **rip lever** | 9.3c — `--rip-existing-nets` with the set it named |
| a net fails on ONE layer at every grid and rip set, and routes instantly with a second layer | **the single-layer constraint is the blocker** | not a router failure. Report it against the requirement that imposed the layer restriction, with both measurements |
| `drc` is large, uniform, one net pair, one overlap value | **grading artifact** | 9.1b — re-grade at the right class. Not a lever at all |
| `broken` is mostly plane-net pads | **the pour could not reach them** | `repair_planes --rip-blocker-nets` (out-of-chain utility; in-chain, route.py's own finalize does this) |
| `check_connected` and `kicad-cli` disagree badly | **the zones are unfilled** | 9.1c — fill, then re-read. Do not "average" them |
| a **symmetry/match clause fails SHORT** (one leg under-length, not over) | **routing lever first** | `--length-match-group` on the pair's own pass meanders the short leg up; only if the group cannot meander (no room) is it placement — then the lever is the **free terminal's position** (the series R/C in the chain), not the ICs |
| a **multi-net rip-return rotates its victim** (each order strands a different net) | **stop rotating orders** | run each candidate order as a FULL chain lineage and compare their `blocking` scores in the ledger — order A's board vs order B's board, not order A's tail vs order B's head. Fifteen ordering experiments in run 5 re-learned this. **Quote each lineage's `min_clearance_used` in the compare**: branch boards inherit their branch-point's `.kicad_pro` floor (run 6 had three floors live at once — 0.1508/0.1532/0.1556), and a floor delta is a confound unless the LOSING side held the looser floor |
| the **same victim set recurs under every order** at every grid | **capacity, not order** | the lane ledger (`check_floorplan --health`) will show the deficit; that is stop condition 3 with the ledger as the measurement, not another ordering lap |
| you are about to write **"this pad cannot be routed"** | **unproven until measured** | `check_reachability.py --pad REF.NUM`. PASSABLE means it is a ROUTER finding and placement is the wrong lever; CAGED means geometry. 9 of 14 such claims across four runs were later refuted — see the impossibility-claim rule |
| one part carries most of a critical net while its BLOCK sits elsewhere | **floorplan, at PART granularity** | `health_net_affinity_offenders` names it and prints the `converge.py poses --ref` line. Block displacement averages this away, so a quiet block metric is not evidence of absence |
| the **bulk pass keeps stranding fine-pitch RAIL pads** under mps | **ordering, before placement** | re-run the bulk with `--ordering original`, rails FIRST (netlist order puts power nets before GPIOs). Order cannot change how many strand — but it chooses WHICH, and a stranded leaf GPIO can still be re-routed before the plane FINALIZE/repair taps land, while a stranded trace-fed rail pad tends to stay lost once they do (rule 5's one-way door; poured rails are already connected from Step 1c and out of this fight). Spend the strandings on the recoverable class. Measured (run 6, signals-first era): mps stranded 5 QFN rail pads; rails-first closed them and moved the fails to leaf nets |

**Accept an iteration only if `blocking` strictly decreased**, or `blocking` is
unchanged and `quality` improved. Otherwise **revert to the parent board** and
take the next lever. An iteration that made it worse is not a starting point.

**One exception, and state it when you use it:** an iteration that reduces
`unrouted` or `broken` while raising a lower-ranked component may be accepted even
if `blocking` is level, because 9.1a ranks connectivity above the rest. Say so in
the ledger with both numbers. A dead net is worse than a wide trace, and the scalar
does not know that.

**A THIRD EXCEPTION, for a MANDATORY CHAIN STEP that manufactures `broken` by
construction.** A fanout converts nets that had NO copper into nets whose copper
is in fragments — that is what an escape stub *is* — so it moves work from
`unrouted` into `broken` and `blocking` rises. The step is not a candidate
iteration to be accepted or reverted; it is a step the chain requires, and the
pass that closes those fragments comes later. Measured on one board: the U1
fanout took `blocking` 297 → 371 while `unrouted` fell 144 → 83 (61 nets gained
copper) and `broken` rose 11 → 140; the bulk signal route then took `blocking`
to 222 and `broken` to 65.

Neither of the two exceptions above covers it, and reaching for them is the
mistake: exception 1 is scoped to "`blocking` is **level**" and here it rose 74,
and exception 2's commensurability probes (`ungraded`,
`patterns_matching_no_routed_net`, `nets_analyzed`) are **identical** across the
two rows, because nothing new became measurable — the same nets are graded, they
simply moved between components. By the letter of the accept rule that lap
should have been reverted, which would have deleted the fanout.

So: **name the step as a mandatory chain step, record the component-level
movement (not the scalar), and say which later step closes the fragments.** Do
not dress it up as 9.1a rank — 9.1a is a LEVER-SELECTION rule, not an accept
rule, and citing it here is a category error that reads as compliance.

**A SECOND EXCEPTION, and it is the one you will get backwards: `blocking` can
RISE because the iteration made more of the board MEASURABLE.** 9.1's rule that
"a run reporting 12 has not found a better board, it has looked at less of it"
does not stop applying once you are inside the loop — it governs comparing two
*iterations* exactly as it governs comparing two flag sets. A net with **no
copper** is invisible to `net_widths` (its clause lands in
`patterns_matching_no_routed_net`), invisible to `check_impedance` (it has no
segments to walk, so it is not in `nets_analyzed`), and invisible to any
per-segment geometry check. Route it and every one of those violations *appears*,
having been there all along.

Measured across one pair of iterations:

| | i1 | i2 |
|---|---|---|
| `blocking` | **34** | 41 |
| width clauses with no copper to measure | 2 nets | **0** |
| `impedance.nets_analyzed` | 7 | **9** |
| HARD clauses failing (repo `check_spec`) | 8 | **7** |

i2 shipped. **Before comparing two `blocking` values, compare what each one
looked at** — `patterns_matching_no_routed_net`, `nets_analyzed`, and `ungraded`.
If they differ, the scores are not commensurable and the scalar is the wrong
arbiter; fall back to the components that ran in both, and to the repo's own spec
checker if it has one. Assert `nets_analyzed` equals the number of nets you named
in `--impedance-nets` every iteration, exactly as you assert `ran == true`.

**Watch for whack-a-mole.** Ripping to route net A can leave net B unrouted, and
the tally still reads "1 failed" — a *different* net. Compare the failing net
**names** between iterations, never just the counts. If they alternate, route them
one per call with the other explicitly out of the rip set (9.3c rule 3). If the
victim keeps rotating across MORE than two nets as you permute the order, stop
permuting: score each order as its own full-chain lineage and let the ledger
pick (9.3d's rip-return row) — and if the same victim SET survives every order,
it is capacity (the lane ledger has the number), not ordering.

**The tooling-vs-placement discriminator (#118), for a converged board with
few failures left:** ask whether a competent human could hand-route the
remaining nets on THIS placement. If yes, the gap is tooling — file it as a
systemic finding and take rung 8, which exists for exactly this. If no human
could either, it is placement or capacity — the lane ledger has the number,
and the finding goes to the next run's Step 0, not to more router laps.
Run 7's west fan answered "no human could" (~25 exact-clearance candidates,
every one within 0.1 mm of committed constrained copper), which is what made
it a capacity finding rather than an engine complaint.

**After ANY placement change every downstream routed board is stale** — re-run
the chain from the placed board. Never keep a routed artifact from before it.

#### 9.4 — Write the ledger, every iteration, before the next one starts

`wk/ledger.jsonl` in the work dir. It is what makes the run resumable, lets the
final report name which stop condition fired, and gives the film its frames.

**Write it with `converge.py record`, and read `converge.py status` back every
iteration.** The verbs that make a ledger worth keeping — `step-back` (byte-exact,
because the board is stored by content hash), `replay` (re-runs the recorded argv),
`status` (the systemic/completion split) and `make_film.py --from-ledger` — all
read append-only **JSONL** through `board_store.Ledger`. A hand-written single JSON
document is readable by a person and by nothing else, so every one of them is
unreachable from it.

```bash
python3 -X utf8 py_placer/converge.py record --ledger wk/ledger.jsonl \
    --board wk/iter03.kicad_pcb --kind completion \
    --lever 'rip lever: --rip-existing-nets QSPI_SD2 + --grid-step 0.025' \
    --score-file wk/score_iter03.json \
    --argv python3 -X utf8 py_router/route.py wk/iter02.kicad_pcb wk/iter03.kicad_pcb --nets QSPI_SD1 ...

python3 -X utf8 py_placer/converge.py status --ledger wk/ledger.jsonl      # EVERY iteration
```

`status` is the alarm for 9.2's failure mode: it splits the budget into completion
vs systemic and warns when at least half went to the instrument. Nothing else in
the loop says that out loud, and the run that needed to hear it did not.

**Name the panels you READ in the `--lever` text** (`... [read: focus3/panel1,
drc3/cluster1]`). The image mandates are auditable only through the ledger: run
5's breach — a produced-but-never-opened delta render — was invisible precisely
because nothing recorded reads. An iteration whose score had `unrouted`/`broken`
> 0 or a failed `check_drc`, with no `[read: ...]` in its entry, skipped
read-case 3 or 7. **Record NON-triggers the same way**: when a mandate's
trigger is checked and absent, say so in the entry (`[checked: 0 B.Cu parts ->
no --per-side]`) — an unrecorded non-trigger is indistinguishable from a
skipped mandate to any later audit (run 6's watcher had to grep the board to
tell them apart). **A pose decision is read-case 5 even when the arithmetic
is decisive**: a rot-0-vs-rot-180 call made on `components.inversions` alone,
with no side-by-side ratsnest reads in the ledger, is a mandate skipped —
run 7 decided the U3 pose twice that way; the number was right, and the
breach is still a breach the audit had to flag.

**What `record` actually writes is this, and only this:**

```jsonc
{"iteration": 3, "kind": "completion",       // or "systemic" -- see 9.2
 "parent_sha": "...", "result_sha": "...",   // content hashes, not paths
 "lever": "rip lever: --rip-existing-nets QSPI_SD2 QSPI_SS + --grid-step 0.025",
 "lever_argv": ["python3", "-X", "utf8", "route.py", "..."],
 "score": {"blocking": 12, "blocking_by": {"unrouted": 1, "drc": 11}},
 "accepted": true}
```

**There is no `--unrouted-nets`, no `--parent`, no `--verdicts` flag** — those
fields do not exist, and a reader who assumes they are captured will keep no
record of the one thing 9.3d says decides an iteration. Until they do exist,
**put the failing net NAMES in `--lever`**, which is free text and is what
`status` and the film both display:

```bash
--lever 'rip lever: --rip-existing-nets QSPI_SD2 + --grid-step 0.025.
         unrouted BY NAME: QSPI_SD0, QSPI_SD1. Fixed USB_DP_R/USB_DM_R;
         newly broke QSPI_SS, GPIO2 -- whack-a-mole, net a traded for net b.'
```

**Record the failing nets by NAME, not by count.** Counts hide whack-a-mole
completely: measured on one run, `unrouted` read **4 → 4** across an iteration
that had in fact fixed four nets and broken five different ones. The scalar said
"no progress"; the names said the iteration was churning. **This applies to
PROBE records too, and to the BASELINE side of every comparison**: run 7's
adoption entry recorded `full_probe_failures: 41` — a count — and the 41
names the ADOPTED board failed appeared nowhere, so the routing phase's first
whack-a-mole comparison had nothing to diff against. A probe verdict enters
the ledger as names (`score.failed_nets`, or the names in the lever text),
for the candidate AND the baseline row it beat. `record` now nags exactly
this: a score carrying `failures` without `failed_nets` draws a NOTE.

**`parent_sha` is the board this iteration actually came from** — `record`
derives it from the last accepted entry, not from iteration N−1. When you need a
path for `render_placement --before`, resolve it out of the store by that sha
rather than guessing; using N−1 renders a delta that never existed. **Never
reuse an output path across iterations**: a ledger that says
`wk/placed.kicad_pcb` when three iterations wrote that name is unauditable, and
one that named a *rejected* board as the parent of everything downstream got
shipped.

**Take the argv from the tool's own `CMD:` line, not from memory.** Every
routing tool now self-echoes `CMD: <the exact invocation>` as its first stdout
line and `EXIT=<rc>` as its last (`route.py`, `route_diff.py`,
`route_planes.py`, `repair_planes.py`, and the checkers). That line
comes from `sys.orig_argv`, so it carries interpreter flags like `-X utf8`
verbatim and is REPLAYABLE truth rather than a reconstruction. Until run 12 the
three signal/diff/plane routers did not have it, which made "paste the tool's
own `CMD:` line into `converge record --argv`" unsatisfiable for a routing lap
— run 11 hand-wrote replay scripts instead, which is exactly the placeholder-
argv failure the next paragraph exists to stop.

**The argv must be real, and the close-out must name its stop condition —
`record` now enforces both.** An `--argv` whose first token is neither an
existing file nor on PATH is refused (exit 2, nothing written): run 7's
endgame recorded `["python3","-X","utf8","dummy"]`, which `replay` can never
run — a placeholder argv turns the ledger back into prose. The run-closing
entry takes `--final --stop-condition '<which of 9.5 fired>'`; `--final`
without a stop condition is refused the same way. And **before quoting a
headline in the lever text, diff it against the SAME entry's score payload**:
run 7's final entry said "SWD closed, 5 opens" while its own score listed
SWDIO among 6 unrouted — the prose shipped into the report and the correction
cost a commit. The score is the record; the lever text is a caption of it.

**Log the systemic/completion split in the final report**: *"41 iterations: 9
systemic, 32 completion"* is a fact about how the budget was spent, and a run that
cannot state it was not keeping a ledger.

#### 9.4b — Boundary verification: BLOCKING, at every accepted iteration and at close

The ledger records what the operator SAYS happened; nothing above checks it
until the close-out audit, and by then the errors have compounded. Run 4's
audit found exactly this: 6 of 8 entries batch-written after the fact (the
per-step timestamps said so), and one `[read:]` tag claiming a pixel read
that never happened. Both were honest-looking entries a contemporaneous
check would have bounced in seconds.

**The rule: after every ACCEPTED iteration's ledger entry, and at close-out,
an independent subagent verifies the entry against its artifacts BEFORE the
next step may start. A FAIL blocks; remediate (fix the entry, re-read the
artifact, or re-run the step) and re-verify.**

What the boundary verifier receives — and it must be ONLY this, never the
raw board (it verifies the RECORD, not the routing):

- the ledger entry (the JSONL line just written),
- the score JSON it attaches (`--score` payload),
- the render JSON(s) the entry's `[read: ...]` tags name,
- the operator's one-paragraph claim of what the iteration did.

What it checks, each with the artifact that decides:

1. **board_sha binding** — the score payload's `board_sha` matches the
   entry's `result_sha`; a stale attachment (run 4 had two, deliberate but
   warned) must be lever-explained in the entry itself.
2. **`[read:]` truthfulness** — every panel the entry claims was read exists
   in the named render JSON, and every checklist value the claim quotes
   matches that JSON. An entry whose claim quotes numbers appearing in no
   attached artifact FAILS (run 4's e4: the tag implied a read that lived
   only in the journal).
3. **Contemporaneity** — the entry's timestamp is AFTER its artifacts'
   mtimes and BEFORE any later step's artifacts. Batch-written history
   shows up as a cluster of entries stamped within seconds; that is a FAIL
   on every entry in the cluster except the last (run-2's defect, recurred
   run 4; "record between steps or disclose in the entry, no third option").
4. **Claims-vs-artifacts** — every number in the operator's claim traces to
   a field in the entry, the score, or a named render JSON. "Fixed 4 nets"
   with no names in the lever text is a FAIL (the whack-a-mole rule above).
5. **Assembly-clean** (run 6; placement-phase and fix-loop boundaries) —
   the verifier additionally receives the fresh `check_assembly` JSON and
   the render JSON, and FAILS unless `blocking == 0`, the checklist's
   `b_body_overlap_pairs` is empty, and every NEW-vs-baseline advisory
   pair is either fixed or dispositioned in the entry. A claim of
   "placement done" with no attached `check_assembly` JSON is itself a
   FAIL — the run-5 lesson is that the phase ended on belief.

Reply format is the watcher's: one line,
`VERDICT=PASS` or `VERDICT=FAIL:check=<1-5>;finding=<one line>;evidence=<path#pointer>`.

Cadence discipline: REJECTED iterations do not get a boundary verification
(their entries record a road not taken; the close-out audit samples them),
and the verifier is bounded to the slices above — handing it the whole work
dir invites it to re-litigate routing decisions, which is the convergence
loop's job, not the record-keeper's. The close-out boundary verification
additionally walks the WHOLE ledger for checks 3 and 4 (monotone t-stamps
end to end; the final entry's stop condition quoted against its score).

#### 9.5 — Stop conditions. Only these four. Say which one fired, every time.

1. **`blocking == 0`, the repo's own spec checker passes, and every verifier lens
   passes** → done. All three are required. `board_score` exits **0** at
   `blocking == 0` even on a board with ten HARD clauses violated, because the
   clauses a repo checker measures are not `board_score` components — so
   exit-code-driven automation stops there unless you gate on the checker too.
   See "Verify with independent subagents".
2. **Budget exhausted** — you have actually written **100** ledger entries for this
   board. Report the best-scoring board **and the remaining blockers itemised with
   measurements**. Do not present it as finished.
3. **Five consecutive iterations with `unrouted` and `broken` both unchanged,
   after trying the rip lever, a finer grid, and a layer change on the failing
   nets** → floorplan-limited or spec-limited. Say which, with the number. (Five,
   and on the connectivity components — three iterations of `drc` not moving means
   nothing when the real blocker is a dead net.)
4. **A blocker is geometrically unsatisfiable** → stop and report it as a
   **finding about the requirement**, with the measurements that prove it. Worked
   example: a 2.4 mm clearance requirement written as a netclass also applies
   pad-to-pad, and on that connector the measured pad gaps were 0.500 mm (VBUS)
   and 1.300 mm (GND) — 23/44 nets with it, 38/44 without. That is unsatisfiable
   *as written*, and it took one measurement to prove. **Do not silently relax
   it, and do not grind iterations against it.**

   **When the unsatisfiable clause is a dru rule the router hard-enforces**
   (the .kicad_dru track channel or a layer rule), stop-4 scopes to the REGION, not the
   run: (1) route the pass WITH the rule first and measure the failure —
   decide on the measurement, not in advance; (2) then stage a sibling dru
   for that pass with ONLY the unsatisfiable rule lifted — never a bare
   no-dru copy, which silently drops every OTHER rule's protection for that
   pass (run 6's watcher caught exactly this before it shipped); (3) grade
   the residue against the registered floor and report the clause as a
   requirement finding. The rest of the chain keeps the full dru.

##### These are NOT stop conditions

Stopping for any of these is a process failure, not an outcome. If one of them is
pulling at you, write the next ledger entry instead:

- **"This is taking a long time."** Wall-clock is not a stop condition. A run once
  stopped at 11 of 20, labelled it "budget exhausted", and recorded in its own
  ledger that the levers were *not* exhausted. Eleven is not twenty.
- **"The score stopped moving."** Check *which* component. `blocking` level while
  `unrouted` falls is progress (9.3d's exception).
- **"The remaining work is hard."** Hard is what the budget is for. A net that
  needs a scoped rip, a finer grid and a layer change is three cheap iterations,
  not a wall.
- **"I have written up the findings."** The report is not the deliverable while
  nets are unrouted. Finish the board, then write.
- **"The last lever failed."** Revert and take the next one. The ladder has more
  rungs than you have tried: rip set → grid → layer → via cost → width → order →
  placement → hand-authored micro-copper.

**Rung 8, hand-authored micro-copper, exists — with FIVE hard conditions.**

1. **Only after every mechanical rung is exhausted** (dru lifts, nc-map fixes,
   scoped grids, rip sets — run 7 entered rung 8 only after all four were
   demonstrably spent, and that part it got right).
2. **Round-cap arithmetic off the ACTUAL copper widths** (a track's reach is
   `endpoint + width/2`; run 5 authored 0.427 and 0.442 mm candidates that
   both failed the arithmetic before 0.465 mm passed).
3. **A join verifier BEFORE the first segment, graded against ALL copper.**
   Reach is the easy half; the shorts live in the other half — third-party
   segments, vias, pads, pour. Run every candidate polyline+via through
   `check_join.py BOARD NET x,y,layer ... via:x,y` — it stages the candidate
   onto a copy of the board and diffs the REAL check_drc engine (netclasses,
   `.kicad_dru` layers, rotated pads, board edge, hole-to-hole), plus the
   join-specific checks DRC omits: a layer change with no `via:` and same-net
   via stacks. (No check_join in the pinned engine? A scratch-board
   `check_drc.py` run per candidate is the zero-build version.) Measured,
   run 7: pad-edge-only arithmetic at 0.1575+ margins authored **42 shorts**;
   the verifier was built mid-recovery instead of first.
4. **Re-gate EVERY edit** — drc + connectivity + score on the edited board,
   recorded in the ledger like any other lever, revert on regression. Per
   EDIT, not per phase: run 7 gated a whole hand-copper session on
   connectivity alone with DRC deferred, and the deferred DRC was where the
   42 shorts surfaced. A hand segment that skips the gate is not a fix, it is
   an unmeasured edit.
5. **Lock it, and commit it FIRST.** Stamp every hand-authored segment and via
   `(locked yes)` at authoring time — locked copper's net is never
   rip-eligible (#521), which is exactly the semantics a hand join needs: no
   later `--rip-existing-nets` glob, in-run ladder, or plane-repair
   `--rip-blocker-nets` picker may treat the one corridor you proved as free
   space (measured, run 7: the plane repair picked three hand-routed GPIOs as
   tap blockers and ripped them; unlock deliberately if a rebuild must move
   them). And hand joins are the most constrained copper on the board — one
   proven corridor, zero router flexibility — so constrained-first applies
   across the hand/router boundary: commit them BEFORE the flexible nets'
   final routing and make the router route around them (strip and re-route
   the flexible set last if needed). Never author a hand join into a fabric
   of committed copper the router could have placed elsewhere (measured,
   run 7: fixed-joins-first closed at 0 shorts where the reverse order had
   authored 42).

**Rung 8 has TWO exits: placed-and-gated copper, or an exhaustive NO-JOIN —
and the second is a result, not a failure, but ONLY when it ships all four
parts:** (1) the sweep envelope (width, clearance, path families, step)
recorded in the ledger like any lever; (2) ONE scoped route at the router's
own hinted finest grid (0.025 at ≤0.4 mm pitch) to cover paths outside the
enumerated families; (3) a `--view` crop of the region, read and ledgered;
(4) the verification-mode disclosure — a fanned-out hostile rebuild, or the
single-agent statement that the sweep itself is the rebuild. A NO-JOIN
missing any part is a HYPOTHESIS, and every report that quotes it must say
so (run 7's west-fan capacity claim shipped with the envelope, the
finest-grid route and the crop all missing — it is a hypothesis pending the
next run, by this rule). A complete sweep IS the hostile rebuild the watcher
pattern demands; do not follow it with more router laps at the grid that
already failed (run 6 ran two rotation laps after its sweep proved the
fabric sealed).

**A worked case of stopping wrongly, because it is the most expensive mistake in
this document.** One run reported four unrouted nets, wrote up "stop condition 3"
and named the next lever *in the same write-up without trying it*. Budget spent:
**4 of 100**. Resuming cost four scoped single-net calls of a few seconds each
and took `unrouted` **4 → 0**. Every blocker it had reported dissolved:

| reported as | actually was |
|---|---|
| "boxed in by its QFN neighbours" | needed the rip set the **router** named, which was wider than the one `net_forensics`' 1 mm radius showed |
| "walled by VREG_AVDD/VCC3V3" | the analyst's own `--track-width 0.4`, on a net with no width clause |
| "needs a fanout; that is what run 5 should try" | routed on one layer at `--grid-step 0.025`, no fanout |

Two rules fall out. **The working grid at a 0.4 mm-pitch part is 0.025, not
0.05** — "0.05, or 0.025 for sub-0.4 mm pitch" reads as excluding a part that is
*at* 0.4 mm, and it should not. And **the router's hint beats the forensics
wall**: forensics reports what is inside a radius, the router reports what its
whole obstacle map says is decisive. When they disagree, take the router's set.

**Before invoking condition 2 or 3, answer in writing:** how many nets are
unrouted, what is the router's own hint for each, and which of the 9.3c rip rules
has not been tried on them? If any of those is unanswered, the loop is not done.
**The answers are already printed:** each failing net's `Hint:` line is an
untried rung until you run it or refute it — quote it; and any
`protected_skipped: {net: user}` line means YOUR OWN protection is the named
blocker — exercise the exact-name override or write why not (run 6's final
watcher found both classes sitting unread in the logs while the stop was
claimed). And an endgame burst is still iterations: **one ledger entry per
board-state, journal per phase** — a stop-3 claim rising out of a one-entry
endgame cannot exhibit its five unchanged iterations.

Ending on 2, 3 or 4 is a legitimate outcome. Ending on any of them **while
calling the board finished is not**, and ending on none of them is not an ending.
