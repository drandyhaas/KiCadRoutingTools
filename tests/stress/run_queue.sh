#!/bin/bash
# Stress-test queue manager — keeps N headless board workers in flight until the
# whole corpus (set 1 + set 2) has a results JSON. Pure file/process polling, no
# notification stream. Safe to (re)start anytime: state is derived from disk, so
# it skips finished boards and won't double-launch ones already running (incl.
# harness Agent runs detected via the run-dir).
#
# Usage: run_queue.sh [concurrency] [model]   (defaults: 4 sonnet)
# Watch:  tail -f ~/Documents/kicad_stress_test/QUEUE_STATUS.txt
set -u
CONC="${1:-4}"; MODEL="${2:-sonnet}"
# Repo root is derived from this script's own location (tests/stress/run_queue.sh),
# so the script is portable; override with STRESS_REPO if needed.
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${STRESS_ROOT:-$HOME/Documents/kicad_stress_test}"
REPO="${STRESS_REPO:-$(cd "$SELF/../.." && pwd)}"
STATUS="$ROOT/QUEUE_STATUS.txt"

boards1=$(ls "$ROOT"/boards_unrouted/*.kicad_pcb 2>/dev/null      | xargs -n1 basename | sed 's/\.kicad_pcb$//')
boards2=$(ls "$ROOT"/boards_unrouted_set2/*.kicad_pcb 2>/dev/null | xargs -n1 basename | sed 's/\.kicad_pcb$//')
total=$(( $(echo "$boards1" | grep -c .) + $(echo "$boards2" | grep -c .) ))

rundir(){ if [ "$2" = 1 ]; then echo "$ROOT/runs/$1"; else echo "$ROOT/runs_set2/$1"; fi; }
resfile(){ if [ "$2" = 1 ]; then echo "$ROOT/results/$1.json"; else echo "$ROOT/results_set2/$1.json"; fi; }
is_done(){ [ -f "$(resfile "$1" "$2")" ]; }
is_running(){  # our worker, any tool writing the run dir, or run dir touched <15min
  pgrep -f "run_board.sh $1 $2" >/dev/null 2>&1 && return 0
  local d; d=$(rundir "$1" "$2")
  pgrep -f "$d" >/dev/null 2>&1 && return 0
  [ -d "$d" ] && [ -n "$(find "$d" -mmin -15 2>/dev/null | head -1)" ] && return 0
  return 1
}

log(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$STATUS"; }
: > "$STATUS"
log "queue start: concurrency=$CONC model=$MODEL total=$total"

while true; do
  done_n=0; run_n=0; run_l=""; todo=""
  for pair in $(for b in $boards1; do echo "$b:1"; done; for b in $boards2; do echo "$b:2"; done); do
    b="${pair%%:*}"; s="${pair##*:}"
    if   is_done "$b" "$s";    then done_n=$((done_n+1))
    elif is_running "$b" "$s"; then run_n=$((run_n+1)); run_l="$run_l $b"
    else todo="$todo $b:$s"; fi
  done
  log "DONE=$done_n/$total RUNNING=$run_n [$run_l ] TODO=$(echo $todo | wc -w | tr -d ' ')"

  if [ "$done_n" -ge "$total" ]; then
    log "ALL $total BOARDS DONE"
    rm -f "$STATUS"   # transient heartbeat; results JSONs + FINDINGS.md are the record
    break
  fi

  free=$(( CONC - run_n ))
  for item in $todo; do
    [ "$free" -le 0 ] && break
    b="${item%%:*}"; s="${item##*:}"
    is_running "$b" "$s" && continue
    nohup bash "$REPO/tests/stress/run_board.sh" "$b" "$s" "$MODEL" >/dev/null 2>&1 &
    log "  launched $b (set $s) pid=$!"
    free=$(( free - 1 ))
    sleep 3   # let the worker register before the next is_running sweep
  done
  sleep 45
done
