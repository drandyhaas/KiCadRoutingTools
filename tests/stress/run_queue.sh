#!/bin/bash
# Stress-test queue manager — keeps N headless board workers in flight until the
# whole corpus (set 1 + set 2 + set 3 ...) has a results JSON. Pure file/process polling, no
# notification stream. Safe to (re)start anytime: state is derived from disk, so
# it skips finished boards and won't double-launch ones already running (incl.
# harness Agent runs detected via the run-dir).
#
# Usage: run_queue.sh [max_concurrency] [model]   (defaults: 8, backend default)
#   Backend: STRESS_AI_BACKEND=claude (default) | opencode (#503) — inherited by
#   every run_board.sh worker. Model defaults: claude -> sonnet; opencode -> the
#   user's configured opencode default (or pass provider/model explicitly).
# Watch:  tail -f ~/Documents/kicad_stress_test/QUEUE_STATUS.txt
#
# Concurrency is LOAD-BASED, with the first arg as a hard CEILING (default 8).
# A fixed worker count cannot know whether the heavy python route steps happen to
# be coinciding: a board worker is mostly *thinking* (LLM latency), so N workers
# might sit at load 3 or at load 20 depending on what they are each doing at that
# instant. The old fixed default of 10 drove sustained load averages north of 20
# on an 8-core box, which starves the route steps themselves and pushed three
# routing integration tests past a 900 s timeout that all passed uncontended.
#
# So a new board launches only when the 1-minute load average is below
# QUEUE_LOAD_MAX (default = HALF the core count, leaving room for other work).
# Notes on the gate:
#
#   * LOAD, not swap or memory-pressure level. This box sits at
#     kern.memorystatus_vm_pressure_level 2 ("warn") with ~6.5 GB of swap in use
#     as its NORMAL idle state -- that is macOS compressing idle `claude`
#     processes, not thrashing. Gating on either would wedge the queue forever.
#     Only pressure level 4 (CRITICAL) blocks a launch.
#   * A FLOOR (QUEUE_CONC_MIN, default 2) always launches, whatever the load.
#     Without it, load from anything else on the machine -- a test suite, a
#     build, another wave -- would stall the queue indefinitely with no way out.
#   * Above the floor it launches at most ONE board per 45 s pass. The 1-minute
#     load average is a lagging, exponentially-weighted signal: launching a burst
#     and re-reading it immediately just overshoots, which is the failure this
#     gate exists to prevent. Cold start is therefore fast to the floor and
#     gentle above it.
#   * Every skipped pass LOGS its reason, so a throttled queue reads as
#     throttled rather than as hung.
#
# run_limited.sh still caps each tool step and kills it (a finding) if several
# heavy steps coincide, so memory can't run away.
set -u
CONC="${1:-8}"; MODEL="${2:-}"   # CONC = hard ceiling; empty MODEL -> per-backend default
NCORE=$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)
# Target NCORE-2 by default: leave two cores for other work (a test suite, a
# build, an interactive session) but do not give away half the machine --
# ncore/2 collapsed the tail to the floor, because a SINGLE heavy route step
# holds load near 4 on its own. Clamped to >= 1 for small boxes.
LOAD_MAX="${QUEUE_LOAD_MAX:-$(( NCORE - 2 > 0 ? NCORE - 2 : 1 ))}"
# One launch per minute, whatever else is true. The 1-minute load average is a
# LAGGING, exponentially-weighted signal: a launch does not show up in it for
# up to a minute, so admitting on consecutive passes reads a load that predates
# the previous launch and overshoots. Measured: at LOAD_MAX=8 three consecutive
# admissions each saw load 7.4-7.5 (all legitimately under the bar) and load then
# settled at 9.3 and reached 11.3. Rate-limiting is what makes the gate a
# controller rather than a burst.
LAUNCH_INTERVAL="${QUEUE_LAUNCH_INTERVAL:-60}"
LAST_LAUNCH=0
CONC_MIN="${QUEUE_CONC_MIN:-2}"        # always allow this many, whatever the load

# 1-minute load average, portable (macOS sysctl prints "{ 7.58 8.93 11.46 }").
load1(){
  if [ -r /proc/loadavg ]; then awk '{print $1}' /proc/loadavg
  else sysctl -n vm.loadavg 2>/dev/null | awk '{print $2}'; fi
}
# ONLY hard-critical (4). Level 2 is this box's normal idle state -- see above.
mem_critical(){
  local lvl; lvl=$(sysctl -n kern.memorystatus_vm_pressure_level 2>/dev/null || echo 1)
  case "$lvl" in ''|*[!0-9]*) return 1 ;; esac
  [ "$lvl" -ge 4 ]
}
# Why we may NOT launch right now (empty = go ahead).
launch_block_reason(){
  local run_n="$1" load
  [ "$run_n" -ge "$CONC" ] && { echo "at ceiling $CONC"; return; }
  local since=$(( $(date +%s) - LAST_LAUNCH ))
  [ "$since" -lt "$LAUNCH_INTERVAL" ] \
    && { echo "rate limit (${since}s since last launch, need ${LAUNCH_INTERVAL}s)"; return; }
  [ "$run_n" -lt "$CONC_MIN" ] && return          # floor: always allow (load-wise)
  mem_critical && { echo "memory pressure CRITICAL"; return; }
  load=$(load1)
  awk -v l="$load" -v m="$LOAD_MAX" 'BEGIN{exit !(l+0 >= m+0)}' \
    && echo "load $load >= $LOAD_MAX"
  return 0
}
# Repo root is derived from this script's own location (tests/stress/run_queue.sh),
# so the script is portable; override with STRESS_REPO if needed.
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${STRESS_ROOT:-$HOME/Documents/kicad_stress_test}"
REPO="${STRESS_REPO:-$(cd "$SELF/../.." && pwd)}"
STATUS="$ROOT/QUEUE_STATUS.txt"

# Build "board:set" pairs across every set N (boards_unrouted_setN/) discovered
# on disk, so new sets need no edit here (set 1 = boards_unrouted_set1).
# Optional scope: QUEUE_SETS="11 12 .. 25" (space-separated) restricts the run to
# those sets; empty/unset = the whole corpus (the default, backward-compatible).
QUEUE_SETS="${QUEUE_SETS:-}"
in_scope(){ [ -z "$QUEUE_SETS" ] && return 0; for x in $QUEUE_SETS; do [ "$x" = "$1" ] && return 0; done; return 1; }
pairs=""
for d in "$ROOT"/boards_unrouted_set*; do
  [ -d "$d" ] || continue
  s="${d##*_set}"
  in_scope "$s" || continue
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
  # A worker that wrote .worker_done has EXITED (ok OR NORESULT). run_board.sh
  # rm's this at start (line ~24), so its PRESENCE means "no live worker here".
  # Without this, a NORESULT worker whose orphaned background route step kept
  # touching the run dir false-positives the mtime window below and freezes the
  # slot for the full 180 min while the watchdog (needs 3 launches) can't yet
  # stub it -- a ~2 h per-stall deadlock. Checked AFTER the live-worker pgrep so
  # a fresh relaunch (which removes the file first thing) can't be double-run.
  [ -f "$d/.worker_done" ] && return 1
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
log "queue start: ceiling=$CONC load_max=$LOAD_MAX floor=$CONC_MIN cores=$NCORE backend=${STRESS_AI_BACKEND:-claude} model=${MODEL:-(backend default)} total=$total"

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

  # LOAD-BASED admission. Below the floor we fill straight to CONC_MIN (fast cold
  # start); above it we admit at most ONE board per pass and then wait out the
  # 45 s sleep, so the 1-minute load average -- a lagging, EWMA signal -- has time
  # to reflect the launch instead of being overshot by a burst.
  launched_this_pass=0
  for item in $todo; do
    reason=$(launch_block_reason "$run_n")
    if [ -n "$reason" ]; then
      [ "$launched_this_pass" -eq 0 ] && log "  holding: $reason (running=$run_n, load=$(load1))"
      break
    fi
    b="${item%%:*}"; s="${item##*:}"
    is_running "$b" "$s" && continue
    nohup bash "$REPO/tests/stress/run_board.sh" "$b" "$s" "$MODEL" >/dev/null 2>&1 &
    log "  launched $b (set $s) pid=$! [running=$((run_n+1))/$CONC load=$(load1)]"
    LAST_LAUNCH=$(date +%s)
    run_n=$(( run_n + 1 ))
    launched_this_pass=$(( launched_this_pass + 1 ))
    sleep 3   # let the worker register before the next is_running sweep
  done
  # Poll faster than LAUNCH_INTERVAL so the rate limit sets the cadence rather
  # than the sleep: at a 45 s poll a 60 s limit would actually admit every 90 s.
  sleep 30
done
