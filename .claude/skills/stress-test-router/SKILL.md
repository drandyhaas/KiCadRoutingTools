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
The corpus scripts read `STRESS_DIR`; the queue scripts (`run_queue.sh`,
`run_board.sh`, `queue_watchdog.sh`, `stress_status.sh`) read `STRESS_ROOT`.
To move the corpus, set both.

The corpus is split into sets. For set N: sources in `sources/github_setN/`,
the routed originals in `boards_setN/`, the stripped inputs in
`boards_unrouted_setN/`, worker run dirs in `runs_setN/<board>/`, results in
`results_setN/<board>.json` (set 1 uses the `_set1` suffix too).

## Step 1: Prepare the corpus (skip parts that already exist)

Check `$STRESS_DIR/boards_unrouted_set*/*.kicad_pcb` first — if the sets you
want exist, skip to Step 2. Set 1 is built by its own scripts:

```bash
cd tests/stress
: "${STRESS_DIR:=$HOME/Documents/kicad_stress_test}"
python3 fetch_boards.py            # -> sources/github_set1/ (needs `gh` auth)

KIPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
$KIPY normalize_boards.py          # pcbnew round-trip -> boards_set1/
for f in "$STRESS_DIR"/boards_set1/*.kicad_pcb; do   # ONE board per process
  $KIPY strip_routing.py "$(basename "${f%.kicad_pcb}")"
done                               # -> boards_unrouted_set1/
python3 validate_boards.py         # the set-1 SOURCES parse, with complexity stats
```

Every other set has a manifest and two generated scripts:
`python3 fetch_setN.py && bash prep_setN.sh` (from `manifest_setN.json`;
`prep_setN.sh` runs KiCad's python one board per process).

Platform note: on Linux/Windows find the KiCad-bundled python equivalent, or
any python with a working `pcbnew` module of KiCad 9+.

To extend the corpus, build a NEW set (RUNBOOK → "Building a board set"):
gate each candidate `.kicad_pcb` with `validate_candidate.py` (it rejects
pre-KiCad-6 files, which have no `.kicad_pro`), list the curated ones in
`$STRESS_DIR/sources/candidates/curation.json`, and run `assemble_corpus.py`,
which writes `manifest_setN.json`, `fetch_setN.py` and `prep_setN.sh`.

## Step 2: Run boards (queue manager)

Drive the whole corpus with the queue manager — it keeps headless `claude -p`
board workers in flight until every board has a results JSON, deriving all state
from disk (safe to stop and restart):

```bash
bash tests/stress/run_queue.sh [max_concurrency=8] [model=sonnet]
bash tests/stress/stress_status.sh        # monitor: DONE/RUNNING/TODO + admit/hold
```

`sonnet` is the alias for the latest Sonnet: pass an alias, never a pinned
model ID. Workers run at `STRESS_AI_EFFORT=high` by default
(`low|medium|high|xhigh|max`). `QUEUE_SETS="11 12"` limits the queue to those
sets (default: every `boards_unrouted_set*/` on disk). The heartbeat is
`$STRESS_ROOT/QUEUE_STATUS.txt`.

`run_queue.sh` auto-launches `tests/stress/queue_watchdog.sh`, which caps per-board
attempts (default 3, `QUEUE_MAX_LAUNCH`) and stubs a FAILED results JSON for any
board whose worker keeps dying without writing one — otherwise the queue relaunches
it forever and never terminates. The watchdog also runs standalone next to a
manually-driven queue.

Each worker (`run_board.sh <board> <set> [model]`) routes one board per
`RUNBOOK.md` (note RUNBOOK rule 1a': never `cp`/`mv`/alias board files in the
run dir -- only tool `--output`s may create them, or the redo manifest's
replay chain is silently severed) and writes `results_set<N>/<board>.json` plus a
`FINDINGS.md`. It also captures the agent transcript (`transcript.jsonl`) and
auto-derives `agent_narrative.md` — a compact routing decision trail (the agent's
narration paired with the actual route/diff/plane/fanout commands) via
`tests/stress/extract_narrative.py`. The tools record every board-mutating
command into the run dir's `redo_commands.sh` (`REDO_MANIFEST`), and
`git_version.txt` names the checkout. After the agent exits, the worker grades
the final board itself (`grade_final.py` → `authoritative_grade.json`, with
`misgrade` set when the agent's self-reported DRC disagrees), renders PNGs of
it, checks parser parity (`py_tools/validate_pcb_data.py`), converts the
manifest to a GUI plan (`<board>_plan.json`), and writes `.worker_done`
(`ok`, with `MISGRADE` when flagged, or `NORESULT`). The headless workers run `claude -p --dangerously-skip-permissions`,
which the harness blocks by default — authorize it once with a Bash allow-rule
for `bash tests/stress/run_board.sh:*` / `bash tests/stress/run_queue.sh:*` in
`.claude/settings.local.json` (gitignored, so a checkout never inherits it), or
approve when prompted.

Hard operational limits (baked into the scripts; violating these has crashed the
machine before):

- **Concurrency is LOAD-BASED; the arg is a hard ceiling (default 8).** A fixed
  worker count cannot know whether the heavy python3 route steps happen to be
  coinciding — a board worker is mostly *thinking* (LLM latency), so the same N
  workers sit at load 3 or load 20 depending on what each is doing at that
  instant. A new board launches only while the 1-minute load average is under
  `QUEUE_LOAD_MAX` (default **ncore-2**, leaving two cores for other work), and
  at most **one launch per minute** (`QUEUE_LAUNCH_INTERVAL`). Four things that
  gate gets right, each a way to get it wrong:
    - **Load, not swap or memory pressure.** A busy Mac's *normal* state is
      pressure level 2 ("warn") with GBs of swap in use — macOS compressing idle
      `claude` processes, not thrashing. Gating on either wedges the queue
      forever. Only pressure level **4 (critical)** blocks.
    - **A floor** (`QUEUE_CONC_MIN`, default 2) launches regardless of load, so
      other work on the machine can't stall the wave indefinitely.
    - **Rate-limit, because load lags.** The 1-min average is an EWMA: a launch
      doesn't show up in it for up to a minute. Measured at target 8: three
      consecutive admissions each read load 7.4–7.5 (all under the bar) and load
      then settled at 9.3 and hit 11.3. Poll faster than the interval or the
      limit doesn't govern the cadence.
    - **Don't target half the machine.** `ncore/2` collapsed the tail to the
      floor: one heavy route step alone holds load near 4.
  The other guard is per-step memory, not the worker count — `run_limited.sh`
  kills any single step that exceeds **12 GB**.
- **Every tool command runs through `tests/stress/run_limited.sh`** (**12 GB** RSS
  watchdog — raised from 4 GB in #422; a legitimate fine-grid run on a big sparse
  board can still peak several GB, so 4 GB killed real work). An OOM kill is a
  finding, not noise.
  **Size a machine from 12 GB, not 4** — this is the figure people provision
  from, and 4 GB under-provisions by 3x.
- **Give every routing command an explicit long timeout.** The runbook tells the
  worker to block for up to the 3-hour per-command cap, but a driving harness's
  *default* is often ~120 s — two orders of magnitude smaller. In the sets-21-27
  wave that killed at least one route attempt on **21 of 99 boards** (steps that
  legitimately take 141-316 s), and the orphan guard then reaped the reparented
  child. Worse, the killed step's log is usually **empty** (stdout fully buffered,
  nothing flushed before SIGTERM), so it looks like a hang rather than a timeout —
  pass `python3 -u` if you want a diagnosable partial log. See #599.
  `run_board.sh` therefore exports `BASH_MAX_TIMEOUT_MS` (default 10800000, the
  3-hour cap) and tells the worker to pass it on every routing command: under
  the stock 10-minute cap a call that reaches it is moved to the background and
  stopped, which kills a long route step even when the worker waits as told.
  `run_limited.sh` names the caller's timeout when it is the likely killer.
- On 4+ layer boards, BGA/PGA fanout must pass the inner copper layers to
  `bga_fanout.py` (`--layers F.Cu In1.Cu In2.Cu B.Cu`); its default is the two
  outer layers only, which silently caps deep-ball escape (RUNBOOK rule 5).
  `route_diff.py` has the same F.Cu/B.Cu default and the same trap: pass the
  full copper-layer list or pairs are silently stranded (issue #116,
  measured on an 8-layer corpus board: 8/40 -> 22/40).
- **Check the fanout `JSON_SUMMARY` and retry on dropped balls (issue #122).**
  `bga_fanout.py` ends with `JSON_SUMMARY: {"requested","escaped","failed",
  "unescaped_nets",...}`. Dropped balls are removed from the output and resurface
  later as signal-route "no rippable blockers" failures, dominating the shortfall
  (route.py names them in `JSON_SUMMARY.fanout_dropped`, #652).
  If `failed > 0`, **re-run the fanout with `--clearance` at the manufacturing
  floor** (the design-rules step prints it, e.g. 0.1) — this is the common cause,
  not pitch: even an 0.8 mm-pitch BGA drops balls at `--clearance 0.2` (a 0.2 mm
  track won't fit the ~0.45 mm inter-ball gap) but escapes all of them at 0.1.
  If still short, add the fine-pitch escape via / smaller `--track-width`. If a
  **dense, fully-populated array** still drops balls at the floor (the channel
  router over-subscribes the between-row channels — e.g. a fully-populated 22×22
  array drops ~20; the default `--escape-method auto` already retries such a
  drop with `underpad` and keeps whichever escapes more, #288),
  re-run with **`--escape-method dogbone`** and a small via/track (e.g.
  `--via-size 0.35 --track-width 0.12 --clearance 0.1`): each ball vias in a free
  inter-ball gap (the dominant method on human-routed BGAs) and falls back
  per-ball to via-in-pad, so it escapes everything `underpad` can (→ 0) while
  keeping the inner-layer streets clear of barrels. Both dogbone and underpad
  escape diff pairs coupled when given `--diff-pairs` (#182; a pair with no room
  for a coupled corridor falls back to single-ended), and skip plane nets as
  escapes, dropping a plane via for each plane ball instead (#424). Don't start
  signal routing with balls still dropped.
- Track liveness from disk (results JSON + run-dir activity) via
  `stress_status.sh` — NOT the notification stream, which drops and duplicates.

### Manual fallback (no queue script)

To drive by hand instead, spawn one general-purpose subagent per board without a
fresh results JSON, admitting by load the way `run_queue.sh` does (one at a time,
while `stress_status.sh`'s admission line says `admit`):

> Read tests/stress/RUNBOOK.md (in the tools repo — the single source of truth)
> and execute it for BOARD=<name> (<one-line complexity hint>). Follow the runbook
> exactly: analyze per the plan-pcb-routing skill, route with the repo's tools,
> verify, and write $STRESS_DIR/results_set<N>/<name>.json. Never modify the
> tools repo.

A subagent must never end its turn while a routing process is still running (the
run gets orphaned — RUNBOOK rule 12).

Nothing on this path sets `REDO_MANIFEST`, which `run_board.sh` exports, and the
tools record their commands only when it is set. Add to the prompt that every
board-mutating command runs with `REDO_MANIFEST=<absolute run dir>/redo_commands.sh`
as a command prefix (an `export` does not survive between Bash calls), or the
board leaves no replayable manifest.

When driving by hand (the subagent path doesn't write `transcript.jsonl`), generate
the narrative afterward from the sub-agent's transcript:

```bash
python3 tests/stress/extract_narrative.py \
  ~/.claude/projects/<project-slug>/subagents/agent-<id>.jsonl \
  -o "$STRESS_DIR/runs_setN/<board>/agent_narrative.md" --board "<board> (set N)"
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
- a `MISGRADE` in a run dir's `.worker_done` — the agent's self-reported DRC
  disagrees with the harness's own grade of the final board; report the
  numbers from that run dir's `authoritative_grade.json`, and the misgrade

A board whose worker never wrote results gets a stub FAILED results JSON from
the watchdog after `QUEUE_MAX_LAUNCH` attempts: report it as a failure, not as
routed.

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

Known findings already on record, all now CLOSED (if refound, REOPEN with a
repro/evidence comment, per the state rule above — do NOT file a new one):
power-type copper layers dropped (#76), Edge.Cuts regex cross-match (#77),
KiCad 6/7 fp_text-reference collapse (#78), oval/slot drills read as SMD
(#106), multipoint orphan dead-end stubs (#84), router success-vs-connectivity
mismatch (#8), fine-pitch pads boxed in by sub-clearance copper / misleading
"no rippable blockers found" (#95), no incremental output so a killed run loses
work (#100), thermal-via exposed-pad falsely reported disconnected (#108),
board-global fine-grid OOM on large 4+ layer boards (#109). Fixed without an
issue: multipoint `route_multipoint_main` UnboundLocalError on free-end-less
nets. The search in step 1 is the authority on what is open today.

## Regression-testing an engine change: replay, don't re-run

An LLM wave is for hunting new failures. To judge whether an engine change
helped, replay the recorded chains instead: every run dir's `redo_commands.sh`
replays the board with no LLM, deterministically, against whatever is checked
out. The recipes are in `tests/stress/RUNBOOK.md` → "Replaying & A/B (no LLM)"
and "Corpus-scale A/B and bisect on the cloud":

```bash
# one board, into a fresh dir
python3 tests/stress/redo_stress_test.py runs_set1/<board>/redo_commands.sh \
    --workdir <fresh-dir> --continue-on-error
# one set, replayed and graded; then compare two waves
python3 tests/stress/ab_replay_grade.py --set ~/Documents/kicad_stress_test/runs_set3 \
    --out <wave-dir> --label new
python3 tests/stress/ab_replay_grade.py --compare <old>/summary.json <new>/summary.json
# whole sets on rented cores (Modal), keeping the routed boards
python3 tests/stress/cloud_replay_sets.py --sets set10-set19
```

`redo_diff_stage.py` replays only the fanout + diff-pair stages,
`pernet_via_ab.py` replays route steps without `--via-size` (per-net vias), and
`corpus_bisect.sh <sha> <tag>` scores one commit across the corpus for
bisecting. The rules that make a result trustworthy: the baseline is the
recorded runs re-graded, not an archived wave; grade both sides on the same
terms, at each board's routed clearance; compare only boards that replayed an
identical chain; and a two-board result is not a default change (per-board
spread is ±2..3 nets).

## Reporting

End with: the summary table, the list of new issues filed (numbers/links),
duplicates skipped, and any corpus-preparation problems. Keep per-board
detail in the results JSONs, not the chat.
