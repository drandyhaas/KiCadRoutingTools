---
name: stress-test-router
description: Stress-tests the router against real-world open-source KiCad boards (downloaded, normalized, stripped of routing), measuring routing completion rates and DRC violations per board. Aggregates results and files GitHub issues for new router/parser findings after user approval. Use to regression-test the router at scale or to hunt for robustness issues.
---

# Stress-Test Router on Real-World Boards

Run the `tests/stress/` harness end-to-end: prepare the board corpus, route
every board following the plan-pcb-routing skill workflow, aggregate
completion/DRC statistics, and turn novel findings into GitHub issues.

All corpus artifacts live OUTSIDE the repo in `$STRESS_DIR`
(default `~/Documents/kicad_stress_test`). Never commit boards to the repo.

## Step 1: Prepare the corpus (skip parts that already exist)

Check `$STRESS_DIR/boards_unrouted/*.kicad_pcb` first — if the corpus exists
and parses (run `tests/stress/validate_boards.py`), skip to Step 2.

```bash
cd tests/stress
python3 fetch_boards.py            # downloads from GitHub (needs `gh` auth)

KIPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
$KIPY normalize_boards.py          # pcbnew round-trip -> current format
for f in "$STRESS_DIR"/boards/*.kicad_pcb; do   # ONE board per process
  $KIPY strip_routing.py "$(basename "${f%.kicad_pcb}")"
done
python3 validate_boards.py         # all boards must parse with sane stats
```

Platform note: on Linux/Windows find the KiCad-bundled python equivalent, or
any python with a working `pcbnew` module of KiCad 9+.

To extend the corpus, add `(owner/repo, note)` entries to `REPOS` in
`fetch_boards.py` and a fragment->name mapping in `normalize_boards.py`.
Only KiCad 6+ sources survive; older ones are rescued by the pcbnew
round-trip.

## Step 2: Run boards (queue manager)

Drive the whole corpus with the queue manager — it keeps headless `claude -p`
board workers in flight until every board has a results JSON, deriving all state
from disk (safe to stop and restart):

```bash
bash tests/stress/run_queue.sh [concurrency=4] [model=sonnet]
bash tests/stress/stress_status.sh        # monitor: DONE/RUNNING/TODO + free slots
```

Each worker (`run_board.sh <board> <set> [model]`) routes one board per
`RUNBOOK.md` and writes `$STRESS_DIR/results[_set2]/<board>.json` plus a
`FINDINGS.md`. The headless workers run `claude -p --dangerously-skip-permissions`,
which the harness blocks by default — authorize it once with a Bash allow-rule
for `bash tests/stress/run_board.sh:*` / `bash tests/stress/run_queue.sh:*` in
`.claude/settings.local.json` (gitignored, so a checkout never inherits it), or
approve when prompted.

Hard operational limits (baked into the scripts; violating these has crashed the
machine before):

- Concurrency is **4** (the manager default) — most jobs stay well under the
  4 GB per-job cap; lower the arg if you see swapping.
- **Every tool command runs through `tests/stress/run_limited.sh`** (~4 GB RSS
  watchdog). An OOM kill is a finding, not noise.
- On 4+ layer boards, BGA/PGA fanout must pass the inner copper layers to
  `bga_fanout.py` (`--layers F.Cu In1.Cu In2.Cu B.Cu`); its default is the two
  outer layers only, which silently caps deep-ball escape (RUNBOOK rule 5).
  `route_diff.py` has the same F.Cu/B.Cu default and the same trap: pass the
  full copper-layer list or pairs are silently stranded (issue #116,
  butterstick 8/40 -> 22/40).
- Track liveness from disk (results JSON + run-dir activity) via
  `stress_status.sh` — NOT the notification stream, which drops and duplicates.

### Manual fallback (no queue script)

To drive by hand instead, spawn one general-purpose subagent per board without a
fresh results JSON, keeping ~4 in flight and refilling off `stress_status.sh`:

> Read tests/stress/RUNBOOK.md (in the tools repo — the single source of truth)
> and execute it for BOARD=<name> (<one-line complexity hint>). Follow the runbook
> exactly: analyze per the plan-pcb-routing skill, route with the repo's tools,
> verify, and write $STRESS_DIR/results[_set2]/<name>.json. Never modify the
> tools repo.

A subagent must never end its turn while a routing process is still running (the
run gets orphaned — RUNBOOK rule 12).

## Step 3: Aggregate

When all boards have results JSONs, build a summary table sorted by
completion rate: board, layers, routable nets, completion %, multipoint pads
connected/total, DRC baseline/final/delta, connectivity verdict, orphan
stubs, wall time, issue count. Flag:

- completion < 100% — which nets, what failure mode
- DRC delta > 0 — violation types introduced by the router
- crashes / hangs / OOM kills — always report, with tracebacks
- per-board `issues` lists — deduplicate into distinct findings

## Step 4: File GitHub issues (with approval)

For each distinct finding:

1. Search for an existing issue first
   (`gh issue list --search "<keywords>" --state all`), then act by **state**:
   - **Open, same root cause** → don't re-file; add a comment with the new
     evidence (affected boards + numbers + run date).
   - **Closed and the bug RECURRED** → **reopen it** (`gh issue reopen <n>`) and
     add an updated comment with the fresh evidence. A refound closed issue means
     the fix regressed or was insufficient — reopen, never file a duplicate.
   - **Closed but only an *adjacent* new bug** (the original fix still holds) →
     file a NEW issue (or comment on the closed one) and say so explicitly.
   - **No match** → draft a new issue (step 2).
2. Draft: title, affected boards, reproduction command (exact tool
   invocation against the corpus board), observed vs expected, relevant log
   excerpt, and severity (router-correctness > parser-robustness >
   route-quality > workflow-friction).
3. **Present all drafts to the user and get approval BEFORE creating any
   issue.** File only approved ones with `gh issue create`, and **always apply
   a label** (`--label`): `bug` for correctness/robustness/DRC defects,
   `enhancement` for new features or route-quality improvements (`documentation`
   / `question` when apt). When commenting on or reopening an existing issue,
   add a label too if it has none.

Known findings already on record (do not re-file — search/comment, or reopen if
closed-and-refound, per the state rule above).
Now FIXED (if refound, REOPEN with a repro/evidence comment — do NOT file a new
one): power-type copper
layers dropped (#76), Edge.Cuts regex cross-match (#77), KiCad 6/7
fp_text-reference collapse (#78), oval/slot drills read as SMD (#106),
multipoint `route_multipoint_main` UnboundLocalError on free-end-less nets.
Still OPEN (add evidence, don't duplicate): multipoint orphan dead-end stubs
(#84), router success-vs-connectivity mismatch (#8), fine-pitch pads boxed in
by sub-clearance copper / misleading "no rippable blockers found" (#95), no
incremental output so a killed run loses work (#100), thermal-via exposed-pad
falsely reported disconnected (#108), board-global fine-grid OOM on large 4+
layer boards (#109).

## Reporting

End with: the summary table, the list of new issues filed (numbers/links),
duplicates skipped, and any corpus-preparation problems. Keep per-board
detail in the results JSONs, not the chat.
