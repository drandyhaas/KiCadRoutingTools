#!/bin/bash
# Headless per-board stress worker.
#
# Runs a non-interactive agent (`claude -p` by default, or `opencode run`
# with STRESS_AI_BACKEND=opencode, #503) that follows tests/stress/RUNBOOK.md
# to route ONE board, then writes the results JSON (RUNBOOK schema) and a
# concise FINDINGS.md into the board's run dir. Decouples board execution from
# the harness notification stream — the queue manager just watches files.
#
# Also captures the agent transcript (transcript.jsonl) and derives a compact
# routing decision trail (agent_narrative.md) via extract_narrative.py —
# both backends' JSON event schemas are supported there.
#
# Usage: run_board.sh <board> <set:1|2|3...> [model]
#   model: claude backend takes tier aliases (default: sonnet); opencode takes
#          provider/model (e.g. anthropic/claude-sonnet-4-5; default: the
#          user's configured opencode default model).
set -u
BOARD="${1:?board}"; SET="${2:?set}"; MODEL="${3:-}"
BACKEND="${STRESS_AI_BACKEND:-claude}"
if [ "$BACKEND" = "claude" ] && [ -z "$MODEL" ]; then MODEL=sonnet; fi
# Repo root is derived from this script's own location (tests/stress/run_board.sh),
# so the script is portable; override with STRESS_REPO if needed.
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${STRESS_ROOT:-$HOME/Documents/kicad_stress_test}"
REPO="${STRESS_REPO:-$(cd "$SELF/../.." && pwd)}"
SFX="_set$SET"   # set 1 lives in *_set1 dirs too (uniform suffix)
RUNDIR="$ROOT/runs${SFX}/$BOARD"
RESULT="$ROOT/results${SFX}/$BOARD.json"
mkdir -p "$RUNDIR"
rm -f "$RUNDIR/.worker_done"

# Provenance: record which checkout of the router produced this board's results,
# so a run dir self-documents the code under test (git failures are non-fatal).
{
  echo "describe:  $(git -C "$REPO" describe --tags --always --dirty 2>/dev/null)"
  echo "commit:    $(git -C "$REPO" rev-parse HEAD 2>/dev/null)"
  echo "branch:    $(git -C "$REPO" rev-parse --abbrev-ref HEAD 2>/dev/null)"
  echo "subject:   $(git -C "$REPO" log -1 --pretty=%s 2>/dev/null)"
  echo "captured:  $(date -Iseconds 2>/dev/null || date)"
} > "$RUNDIR/git_version.txt"

# Record every board-mutating tool invocation to a replay manifest (issue #132).
# The tools self-record when REDO_MANIFEST is set, so capture is reliable even if
# the agent doesn't route a command through run_limited.sh. Start each run fresh.
export REDO_MANIFEST="$RUNDIR/redo_commands.sh"
# The run dir is the recorder's authority (redo_record._resolve_manifest): agents
# re-export REDO_MANIFEST as "$PWD/redo_commands.sh" and also cd into the repo to
# import kicad_parser, which would otherwise record into the CHECKOUT.
export REDO_RUNDIR="$RUNDIR"
# ...but NEVER by deleting the previous attempt's record. A relaunch (watchdog
# retry after a worker routed the board yet died before writing its results JSON)
# used to `rm -f` a COMPLETE manifest and leave the routed step*.kicad_pcb behind,
# so the new agent either did nothing (board ships with NO manifest) or resumed
# mid-chain (manifest starts at a step*.kicad_pcb that no step in it creates).
# Either way the board reported success with a silently unreplayable chain --
# measured at 8 damaged manifests over 9 relaunches in the 2026-08-06 sets-11-25
# wave. Archive the whole prior attempt instead: the record survives, AND moving
# the intermediates out forces the relaunched agent to route from the input.
if [ -s "$REDO_MANIFEST" ] || ls "$RUNDIR"/step*.kicad_pcb >/dev/null 2>&1; then
  n=1; while [ -e "$RUNDIR/attempt_$n" ]; do n=$((n+1)); done
  mkdir -p "$RUNDIR/attempt_$n"
  mv "$REDO_MANIFEST" "$RUNDIR/attempt_$n/" 2>/dev/null
  # timings is append-only across attempts: COPY (keep the running log intact).
  cp -p "$RUNDIR/redo_commands.timings.jsonl" "$RUNDIR/attempt_$n/" 2>/dev/null
  mv "$RUNDIR"/step*.kicad_pcb "$RUNDIR"/step*.kicad_pro "$RUNDIR/attempt_$n/" 2>/dev/null
  echo "[run_board] archived prior attempt -> attempt_$n" >> "$RUNDIR/worker.log"
fi
rm -f "$REDO_MANIFEST"

PROMPT="Stress-test the KiCadRoutingTools autorouter on ONE board: **$BOARD** (set $SET).

FIRST read $REPO/tests/stress/RUNBOOK.md and obey EVERY rule exactly. Do NOT ask
anything (you are non-interactive) — use the skill's inline heuristics and your judgment.

You ARE the one and only worker for THIS board. Any run_board.sh or headless
agent process ('claude -p' / 'opencode run') you notice for '$BOARD' (or
touching $RUNDIR) is YOUR OWN launcher and yourself — NOT a competing/duplicate worker. Do NOT check whether the board is
'already being routed', do NOT wait for or defer to 'another worker', and do NOT
skip routing to avoid 'collisions'. Start routing immediately and produce the
results JSON yourself; if you don't, the board is recorded as a failure.

Paths for THIS board:
- Input board: $ROOT/boards_unrouted${SFX}/$BOARD.kicad_pcb
- Working dir (use it; it exists): $RUNDIR/  — ALL logs/intermediates here
- Original (compare + ground-truth DRC): $ROOT/boards${SFX}/$BOARD.kicad_pcb
- Results JSON you MUST write (RUNBOOK schema): $RESULT

Rules that matter most:
- Prefix EVERY routing/fanout/plane/check command with: bash $REPO/tests/stress/run_limited.sh
- Run tools as: python3 -u -X utf8 $REPO/py_router/<tool>.py ...  ('-u' is
  REQUIRED: without it a step killed part-way leaves an EMPTY log, #599. #522 moved the
  engine + CLIs into py_router/; a bare \$REPO/<tool>.py does not exist. The few
  repo-root scripts -- build_router.py, install_plugin.py -- stay at \$REPO/.)
- REDO_MANIFEST is ALREADY exported for you (the replay manifest). Do NOT re-export
  or override it -- especially not as \"\$PWD/redo_commands.sh\": you also cd into
  the tools repo to import kicad_parser, and from there \$PWD is the CHECKOUT, so
  your steps get recorded into the repo instead of this board's manifest.
- Stay in your working dir and invoke tools by ABSOLUTE path. Do not cd into
  $REPO; write every output/log under $RUNDIR, never into the tools checkout.
- Use the flags that 'list_nets.py <board> --design-rules' prints (working via,
  manufacturing-floor clearance, DRC floor). Grade DRC at that floor.
- On a 4+ layer board, pass ALL inner copper layers to bga_fanout AND route_diff
  (e.g. --layers F.Cu In1.Cu In2.Cu B.Cu) so deep balls escape.
- GND (+ main power rail on 4+ layers) as planes; exclude plane nets from routing.
- Keep fine (sub-Default) clearance LOCAL to fine-pitch escapes, never board-wide.
- Run EVERY command in the FOREGROUND and BLOCK until it returns — even if a
  single route step takes an hour (hard 3-hour/command cap, ~3.5-hour board
  budget). PASS AN EXPLICIT timeout: 600000 (10 min, the max) ON EVERY routing/
  fanout/plane command — the DEFAULT is only 120000 ms (2 min) and route steps
  routinely take 3-20x that, so omitting it kills your own work and looks like a
  hang (#599: it cost 21 of 99 boards an attempt). If run_limited.sh prints
  'the CALLER's command timeout is too short', believe it: re-run WITH the
  timeout instead of retrying identically or recording a router bug. NEVER
  background a command (no trailing '&', no run_in_background, no
  'I'll wait for the background task to finish'): this is a headless run with NO
  notifications and NO scheduled wakeups, so if you background a step and pause,
  the board is recorded as a FAILURE. Do not stop until the results JSON is written.
- GRADE YOUR ACTUAL FINAL BOARD: the drc.final_violations (and connectivity) you
  report MUST come from running check_drc.py + check_connected.py on the EXACT
  .kicad_pcb produced by your LAST board-mutating step (including any 'fix'/rip-up/
  reconnect retry). Never report a grade taken from an earlier step. This is a
  stress test of the FULL CHAIN: we want the honest final result and to SEE the DRC
  errors — do NOT revert to, cherry-pick, or ship a cleaner earlier board to lower
  the count. If a step INCREASES DRC (e.g. a route.py --rip-existing-nets fix, #284),
  that is a valuable finding: keep it as your final, report the true (higher) number,
  and record the regressing step + command in the issues list.

When fully done:
  1. Write the results JSON to $RESULT (full RUNBOOK schema).
  2. Write a concise $RUNDIR/FINDINGS.md with: completion %, DRC final-vs-original
     delta, connectivity verdict, compare-to-original highlights, and bulleted
     issues + suggestions.
  3. Print the single line: BOARD_DONE $BOARD"

# APPEND, never truncate: a relaunch used to overwrite the previous attempt's
# worker.log, so relaunch history was unrecoverable and the manifest-clobber bug
# above was invisible in the logs (a retried board showed exactly one start line).
# Note `board=` appears on BOTH the start and the exit line -- count launches with
# `grep -c 'start='`, not `grep -c 'board='`.
{
  echo "[run_board] board=$BOARD set=$SET backend=$BACKEND model=${MODEL:-(backend default)} start=$(date)"
  echo "[run_board] result=$RESULT"
} >> "$RUNDIR/worker.log"

# Record a per-copper route trace for each route.py / route_diff.py step the
# agent runs (#482), so render_run.py can build a whole-run movie afterward.
# Default-on; set KICAD_ROUTE_TRACE=0 in the outer env to skip.
export KICAD_ROUTE_TRACE="${KICAD_ROUTE_TRACE:-1}"

# Capture the agent transcript as JSONL (stream-json) so we can derive a routing
# narrative; worker.log keeps the wrapper markers + stderr.
( cd "$RUNDIR" && claude -p "$PROMPT" \
    --model "$MODEL" \
    --dangerously-skip-permissions \
    --output-format stream-json --verbose \
    --add-dir "$ROOT" --add-dir "$REPO" ) > "$RUNDIR/transcript.jsonl" 2>> "$RUNDIR/worker.log"
rc=$?

echo "[run_board] board=$BOARD exit=$rc end=$(date)" >> "$RUNDIR/worker.log"

# Routing decision trail from the transcript (non-fatal — never fails the run).
python3 "$REPO/tests/stress/extract_narrative.py" "$RUNDIR/transcript.jsonl" \
    -o "$RUNDIR/agent_narrative.md" --board "$BOARD (set $SET)" >> "$RUNDIR/worker.log" 2>&1 || true

# Harness backstop: independently grade the FINAL board (DRC at the routed
# clearance + connectivity) and flag a MISGRADE if it disagrees with the agent's
# self-reported drc.final_violations. Catches a stale/pre-mutation self-grade
# (e.g. a `--rip-existing-nets '*'` fix step that corrupts a clean final). Non-fatal.
MIS=""
if [ -f "$RESULT" ]; then
  python3 "$REPO/tests/stress/grade_final.py" "$RUNDIR" "$RESULT" >> "$RUNDIR/worker.log" 2>&1 || true
  MIS=$(python3 -c "import json; print('MISGRADE' if json.load(open('$RUNDIR/authoritative_grade.json')).get('misgrade') else '')" 2>/dev/null)

  # Final-board snapshot (combined + per-layer PNGs, #424 layer-role study) AND
  # the whole-run routing movie (#482), via the fast geometry renderer
  # (render_run.py -> route_render + animate_route), replacing the old kicad-cli
  # + headless-Chrome board_layer_images path. Drops <board>.png,
  # <board>_<layer>.png, and <rundir>/routing.gif. This MIRRORS the no-LLM
  # replay path (redo_stress_test.py). Non-fatal; needs no KiCad/browser.
  FB=$(python3 -c "import json,os,sys
d=json.load(open('$RUNDIR/authoritative_grade.json'))
fb=d.get('final_board') or ''
if fb and not os.path.isabs(fb) and not os.path.exists(fb): fb=os.path.join('$RUNDIR', fb)
print(fb if fb and os.path.exists(fb) else '')" 2>/dev/null)
  if [ -n "$FB" ]; then
    python3 "$REPO/tests/stress/render_run.py" "$RUNDIR" "$FB" >> "$RUNDIR/worker.log" 2>&1 || true

    # Parser-parity validation of the FINAL board (headless twin of the GUI's
    # "Validate PCB Data" button): the pcbnew-built PCBData and the text parse
    # must describe the same board, else the CLI and GUI route different worlds.
    # Mirrors redo_stress_test.py; non-fatal -- a FAIL is a parser bug to file,
    # not a routing failure. validate_pcb_data.py self-execs into KiCad's python.
    if python3 "$REPO/validate_pcb_data.py" "$FB" > "$RUNDIR/validate_pcb_data.txt" 2>&1; then
      :
    else
      echo "[run_board] PARSER-PARITY FAIL on $(basename "$FB") -- see validate_pcb_data.txt" >> "$RUNDIR/worker.log"
    fi
  fi
fi

# GUI-loadable plan JSON from the recorded manifest (non-fatal). Lets this board's
# routing chain be replayed through the Claude tab's Load... button with no LLM run
# (same file-dependency pruning as redo_stress_test / the Claude-tab plan executor).
if [ -f "$REDO_MANIFEST" ]; then
  python3 "$REPO/tests/stress/manifest_to_plan.py" "$REDO_MANIFEST" \
      "$RUNDIR/${BOARD}_plan.json" >> "$RUNDIR/worker.log" 2>&1 || true
fi

if [ -f "$RESULT" ]; then echo "ok rc=$rc $MIS"   > "$RUNDIR/.worker_done";
else                       echo "NORESULT rc=$rc" > "$RUNDIR/.worker_done"; fi
exit $rc
