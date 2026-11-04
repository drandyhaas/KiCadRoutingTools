#!/bin/bash
# Headless per-board PLACEMENT stress worker (#1486): the AI pass.
#
# Piles EVERY part of the corpus board's unrouted input off the board (the
# first recorded step), then runs a non-interactive agent (`claude -p`) that
# places it with /plan-pcb-placement in full mode (place, then route). The
# agent is given the pile only: it never sees the human placement. Every placement step records itself into
# <run>/placement_manifest.json (placement.run_record) and every routing step
# into <run>/redo_commands.sh (redo_record), so the run can be replayed with no
# AI (tests/stress/placement_replay.py).
#
# Usage: run_place_board.sh <board> <set> [model]
#   model: a tier alias (default: sonnet).  effort: STRESS_AI_EFFORT (default high).
set -u
BOARD="${1:?board}"; SET="${2:?set}"; MODEL="${3:-sonnet}"
EFFORT="${STRESS_AI_EFFORT:-high}"
export BASH_MAX_TIMEOUT_MS="${BASH_MAX_TIMEOUT_MS:-10800000}"
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${STRESS_ROOT:-$HOME/Documents/kicad_stress_test}"
REPO="${STRESS_REPO:-$(cd "$SELF/../.." && pwd)}"
SFX="_set$SET"
INPUT="$ROOT/boards_unrouted${SFX}/$BOARD.kicad_pcb"
RUNDIR="$ROOT/runs_place${SFX}/$BOARD"
RESULT="$RUNDIR/place_result.json"
[ -f "$INPUT" ] || { echo "run_place_board: no corpus input $INPUT" >&2; exit 2; }
mkdir -p "$RUNDIR"
rm -f "$RUNDIR/.worker_done"

{
  echo "describe:  $(git -C "$REPO" describe --tags --always --dirty 2>/dev/null)"
  echo "commit:    $(git -C "$REPO" rev-parse HEAD 2>/dev/null)"
  echo "captured:  $(date -Iseconds 2>/dev/null || date)"
} > "$RUNDIR/git_version.txt"

export PLACEMENT_MANIFEST="$RUNDIR/placement_manifest.json"
export REDO_MANIFEST="$RUNDIR/redo_commands.sh"
export REDO_RUNDIR="$RUNDIR"
# a relaunch archives the prior attempt rather than deleting its records
if [ -s "$PLACEMENT_MANIFEST" ] || [ -s "$REDO_MANIFEST" ]; then
  n=1; while [ -e "$RUNDIR/attempt_$n" ]; do n=$((n+1)); done
  mkdir -p "$RUNDIR/attempt_$n"
  mv "$PLACEMENT_MANIFEST" "$REDO_MANIFEST" "$RESULT" "$RUNDIR/attempt_$n/" 2>/dev/null
  mv "$RUNDIR"/wk "$RUNDIR/attempt_$n/" 2>/dev/null
  echo "[run_place_board] archived prior attempt -> attempt_$n" >> "$RUNDIR/worker.log"
fi

PILE="$RUNDIR/pile.kicad_pcb"
python3 -u -X utf8 "$REPO/.claude/skills/plan-pcb-placement/scripts/make_unplaced.py" \
    "$INPUT" "$PILE" >> "$RUNDIR/worker.log" 2>&1 || {
  echo "run_place_board: could not pile $INPUT" >&2; exit 2; }

PROMPT="Place and route ONE board with the plan-pcb-placement skill, in FULL mode:
**$BOARD** (corpus set $SET). This is the AI pass of the placement stress test
(#1486); you are non-interactive, so nobody answers questions.

Follow the plan-pcb-placement skill in FULL mode (place, then route): read
$REPO/.claude/skills/plan-pcb-placement/SKILL.md and the references it names,
and follow them exactly, on: $PILE
(every part is piled off the board: place it from scratch)
Working dir (it exists; put wk/ and every output here): $RUNDIR

- The skill's questions to the user (EMC rows, enclosure, regions, layout
  notes) have no answer here: nobody was there to ask. Use the skill's own
  defaults and the board's datasheets, and say so in REPORT.md.
- PLACEMENT_MANIFEST and REDO_MANIFEST are ALREADY exported: every placement
  and routing step records itself. Do NOT re-export or override them, and run
  every tool by its ABSOLUTE path from $REPO, never by cd'ing into it.
- Run every command in the FOREGROUND with an explicit timeout of
  $BASH_MAX_TIMEOUT_MS ms; never background a command.
- Do not read any other copy of this board (the corpus directories under
  $ROOT): the placement is judged against the human one, so it must not
  inform it.

When fully done:
  1. Write $RUNDIR/REPORT.md (the skill's report).
  2. Write $RESULT as JSON: {\"board\": \"$BOARD\", \"set\": \"$SET\",
     \"placed_board\": <the kept seed's placed board, absolute path>,
     \"kept_seed\": <its seed number>, \"final_board\": <the routed board,
     absolute path>, \"groups\": <groups.json path>, \"intent\": <intent path>}.
  3. Print the single line: BOARD_DONE $BOARD"

{
  echo "[run_place_board] board=$BOARD set=$SET model=$MODEL effort=$EFFORT start=$(date)"
} >> "$RUNDIR/worker.log"

( cd "$RUNDIR" && claude -p "$PROMPT" \
    --model "$MODEL" \
    --effort "$EFFORT" \
    --dangerously-skip-permissions \
    --output-format stream-json --verbose \
    --add-dir "$RUNDIR" --add-dir "$REPO" ) > "$RUNDIR/transcript.jsonl" 2>> "$RUNDIR/worker.log"
rc=$?
echo "[run_place_board] board=$BOARD exit=$rc end=$(date)" >> "$RUNDIR/worker.log"
if [ -f "$RESULT" ] && [ -s "$PLACEMENT_MANIFEST" ]; then
  echo ok > "$RUNDIR/.worker_done"
else
  echo NORESULT > "$RUNDIR/.worker_done"
fi
exit $rc
