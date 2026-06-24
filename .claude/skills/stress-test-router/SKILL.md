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
bash tests/stress/run_queue.sh [concurrency=<#cores>] [model=sonnet]
bash tests/stress/stress_status.sh        # monitor: DONE/RUNNING/TODO + free slots
```

`run_queue.sh` auto-launches `tests/stress/queue_watchdog.sh`, which caps per-board
attempts (default 3, `QUEUE_MAX_LAUNCH`) and stubs a FAILED results JSON for any
board whose worker keeps dying without writing one — otherwise the queue relaunches
it forever and never terminates. The watchdog also runs standalone next to a
manually-driven queue.

Each worker (`run_board.sh <board> <set> [model]`) routes one board per
`RUNBOOK.md` and writes `$STRESS_DIR/results[_set2]/<board>.json` plus a
`FINDINGS.md`. It also captures the agent transcript (`transcript.jsonl`) and
auto-derives `agent_narrative.md` — a compact routing decision trail (the agent's
narration paired with the actual route/diff/plane/fanout commands) via
`tests/stress/extract_narrative.py`. The headless workers run `claude -p --dangerously-skip-permissions`,
which the harness blocks by default — authorize it once with a Bash allow-rule
for `bash tests/stress/run_board.sh:*` / `bash tests/stress/run_queue.sh:*` in
`.claude/settings.local.json` (gitignored, so a checkout never inherits it), or
approve when prompted.

Hard operational limits (baked into the scripts; violating these has crashed the
machine before):

- Concurrency defaults to the **core count**. A board worker is mostly *thinking*
  (LLM latency) and only intermittently in a heavy python route step, so ~Ncore
  boards keep the cores busy without Ncore heavy processes at once. The real guard
  is per-step memory, not the worker count — `run_limited.sh` kills any single step
  that exceeds ~4 GB. Lower the arg only if you actually see swapping.
- **Every tool command runs through `tests/stress/run_limited.sh`** (~4 GB RSS
  watchdog). An OOM kill is a finding, not noise.
- On 4+ layer boards, BGA/PGA fanout must pass the inner copper layers to
  `bga_fanout.py` (`--layers F.Cu In1.Cu In2.Cu B.Cu`); its default is the two
  outer layers only, which silently caps deep-ball escape (RUNBOOK rule 5).
  `route_diff.py` has the same F.Cu/B.Cu default and the same trap: pass the
  full copper-layer list or pairs are silently stranded (issue #116,
  butterstick 8/40 -> 22/40).
- **Check the fanout `JSON_SUMMARY` and retry on dropped balls (issue #122).**
  `bga_fanout.py` ends with `JSON_SUMMARY: {"requested","escaped","failed",
  "unescaped_nets",...}`. Dropped balls are removed from the output and resurface
  later as signal-route "no rippable blockers" failures, dominating the shortfall.
  If `failed > 0`, **re-run the fanout with `--clearance` at the manufacturing
  floor** (the design-rules step prints it, e.g. 0.1) — this is the common cause,
  not pitch: even an 0.8 mm-pitch BGA drops balls at `--clearance 0.2` (a 0.2 mm
  track won't fit the ~0.45 mm inter-ball gap) but escapes all of them at 0.1.
  If still short, add the fine-pitch escape via / smaller `--track-width`. If a
  **dense, fully-populated array** still drops balls at the floor (the channel
  router over-subscribes the between-row channels — e.g. ulx3s 22×22 drops ~20),
  re-run with **`--escape-method underpad`** and a small via/track (e.g.
  `--via-size 0.35 --track-width 0.12 --clearance 0.1`): it routes each ball under
  the pad field on inner layers and escapes what `channel` can't (→ 0). It routes
  diff pairs single-ended and skips power/plane nets (plane them first), so reach
  for it specifically when `channel` floors out on a dense array. Don't start
  signal routing with balls still dropped.
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

When driving by hand (the subagent path doesn't write `transcript.jsonl`), generate
the narrative afterward from the sub-agent's transcript:

```bash
python3 tests/stress/extract_narrative.py \
  ~/.claude/projects/<project-slug>/subagents/agent-<id>.jsonl \
  -o "$STRESS_DIR/runs[_setN]/<board>/agent_narrative.md" --board "<board> (set N)"
```

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
