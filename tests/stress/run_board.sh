#!/bin/bash
# Headless per-board stress worker.
#
# Runs a non-interactive `claude -p` agent that follows tests/stress/RUNBOOK.md
# to route ONE board, then writes the results JSON (RUNBOOK schema) and a
# concise FINDINGS.md into the board's run dir. Decouples board execution from
# the harness notification stream — the queue manager just watches files.
#
# Also captures the agent transcript (transcript.jsonl) and derives a compact
# routing decision trail (agent_narrative.md) via extract_narrative.py.
#
# Usage: run_board.sh <board> <set:1|2|3...> [model]
set -u
BOARD="${1:?board}"; SET="${2:?set}"; MODEL="${3:-sonnet}"
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
rm -f "$REDO_MANIFEST"

PROMPT="Stress-test the KiCadRoutingTools autorouter on ONE board: **$BOARD** (set $SET).

FIRST read $REPO/tests/stress/RUNBOOK.md and obey EVERY rule exactly. Do NOT ask
anything (you are non-interactive) — use the skill's inline heuristics and your judgment.

Paths for THIS board:
- Input board: $ROOT/boards_unrouted${SFX}/$BOARD.kicad_pcb
- Working dir (use it; it exists): $RUNDIR/  — ALL logs/intermediates here
- Original (compare + ground-truth DRC): $ROOT/boards${SFX}/$BOARD.kicad_pcb
- Results JSON you MUST write (RUNBOOK schema): $RESULT

Rules that matter most:
- Prefix EVERY routing/fanout/plane/check command with: bash $REPO/tests/stress/run_limited.sh
- Run tools as: python3 -X utf8 $REPO/<tool>.py ...
- Use the flags that 'list_nets.py <board> --design-rules' prints (working via,
  manufacturing-floor clearance, DRC floor). Grade DRC at that floor.
- On a 4+ layer board, pass ALL inner copper layers to bga_fanout AND route_diff
  (e.g. --layers F.Cu In1.Cu In2.Cu B.Cu) so deep balls escape.
- GND (+ main power rail on 4+ layers) as planes; exclude plane nets from routing.
- Keep fine (sub-Default) clearance LOCAL to fine-pitch escapes, never board-wide.
- Run commands in the FOREGROUND. Hard 3-hour/command cap; ~3.5-hour board budget.

When fully done:
  1. Write the results JSON to $RESULT (full RUNBOOK schema).
  2. Write a concise $RUNDIR/FINDINGS.md with: completion %, DRC final-vs-original
     delta, connectivity verdict, compare-to-original highlights, and bulleted
     issues + suggestions.
  3. Print the single line: BOARD_DONE $BOARD"

{
  echo "[run_board] board=$BOARD set=$SET model=$MODEL start=$(date)"
  echo "[run_board] result=$RESULT"
} > "$RUNDIR/worker.log"

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

if [ -f "$RESULT" ]; then echo "ok rc=$rc"       > "$RUNDIR/.worker_done";
else                       echo "NORESULT rc=$rc" > "$RUNDIR/.worker_done"; fi
exit $rc
