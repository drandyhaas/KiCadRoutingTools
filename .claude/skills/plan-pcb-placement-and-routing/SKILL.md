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
