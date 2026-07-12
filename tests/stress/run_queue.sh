#!/bin/bash
# Stress-test queue manager — keeps N headless board workers in flight until the
# whole corpus (set 1 + set 2 + set 3 ...) has a results JSON. Pure file/process polling, no
# notification stream. Safe to (re)start anytime: state is derived from disk, so
# it skips finished boards and won't double-launch ones already running (incl.
# harness Agent runs detected via the run-dir).
#
# Usage: run_queue.sh [concurrency] [model]   (defaults: <#cores> sonnet)
# Watch:  tail -f ~/Documents/kicad_stress_test/QUEUE_STATUS.txt
#
# Concurrency defaults to the core count: a board worker is mostly *thinking*
# (LLM latency) and only intermittently in a heavy python route step, so ~Ncore
# boards keep the machine busy without Ncore heavy processes at once. run_limited.sh
# caps each tool step at ~4 GB and kills it (a finding) if several heavy steps do
# coincide, so memory can't run away. Lower the arg if you still see swapping.
set -u
NCORE="$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)"
CONC="${1:-$NCORE}"; MODEL="${2:-sonnet}"
# Repo root is derived from this script's own location (tests/stress/run_queue.sh),
# so the script is portable; override with STRESS_REPO if needed.
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${STRESS_ROOT:-$HOME/Documents/kicad_stress_test}"
REPO="${STRESS_REPO:-$(cd "$SELF/../.." && pwd)}"
STATUS="$ROOT/QUEUE_STATUS.txt"

# Build "board:set" pairs across every set N (boards_unrouted_setN/) discovered
# on disk, so new sets need no edit here (set 1 = boards_unrouted_set1).
pairs=""
for d in "$ROOT"/boards_unrouted_set*; do
  [ -d "$d" ] || continue
  s="${d##*_set}"
  for f in "$d"/*.kicad_pcb; do
    [ -e "$f" ] || continue
    b=$(basename "$f" .kicad_pcb); pairs="$pairs $b:$s"
  done
done
total=$(echo $pairs | wc -w | tr -d ' ')

setdir(){ echo "${2}_set$1"; }
rundir(){ echo "$ROOT/$(setdir "$2" runs)/$1"; }
resfile(){ echo "$ROOT/$(setdir "$2" results)/$1.json"; }
is_done(){ [ -f "$(resfile "$1" "$2")" ]; }
is_running(){  # our worker, any tool writing the run dir, or run dir touched <180min
  pgrep -f "run_board.sh $1 $2" >/dev/null 2>&1 && return 0
  local d; d=$(rundir "$1" "$2")
  pgrep -f "$d" >/dev/null 2>&1 && return 0
  # 180 min, not 15: a big board (FPGA/USB3-class) can spend up to the 3-hour
  # per-command cap in one signal-route step writing no intermediate files; a
  # shorter window marks it idle and double-launches it, starving both copies
  # (issue #148 daisho; cap raised to 3 h for issue #211 ulx3s).
  [ -d "$d" ] && [ -n "$(find "$d" -mmin -180 2>/dev/null | head -1)" ] && return 0
  return 1
}

log(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$STATUS"; }
: > "$STATUS"
log "queue start: concurrency=$CONC model=$MODEL total=$total"

# Bounded-retry watchdog: run_board.sh writes no results JSON on failure, so a
# board whose worker can never finish would be relaunched forever. The watchdog
# stubs a FAILED result after QUEUE_MAX_LAUNCH (default 3) attempts so the queue
# terminates. It self-exits when every board is accounted for; the trap kills it
# if the queue exits first.
bash "$SELF/queue_watchdog.sh" >/dev/null 2>&1 &
WATCHDOG_PID=$!
trap 'kill "$WATCHDOG_PID" 2>/dev/null' EXIT

while true; do
  done_n=0; run_n=0; run_l=""; todo=""
  for pair in $pairs; do
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
