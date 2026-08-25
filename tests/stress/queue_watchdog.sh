#!/bin/bash
# Bounded-retry watchdog for run_queue.sh.
#
# run_board.sh does NOT write a results JSON when its agent fails (crash, timeout,
# OOM) -- the agent writes it. run_queue.sh treats "no results JSON" as TODO and
# relaunches forever, so a board whose worker can never finish makes the queue
# (and any orchestrator waiting on it) loop indefinitely, burning tokens.
#
# This watchdog caps attempts: when a board has been launched >= MAXLAUNCH times
# (counted from QUEUE_STATUS.txt), has no results JSON, and has no worker running,
# it writes a stub FAILED results JSON so the queue records the failure and moves
# on. run_queue.sh launches this automatically; it can also be run standalone
# alongside a manually-started queue.
#
# Usage: queue_watchdog.sh [max_launch]      (default: $QUEUE_MAX_LAUNCH or 3)
# Watch: tail -f ~/Documents/kicad_stress_test/QUEUE_WATCHDOG.log
set -u
ROOT="${STRESS_ROOT:-$HOME/Documents/kicad_stress_test}"
STATUS="$ROOT/QUEUE_STATUS.txt"          # run_queue.sh's heartbeat (has "launched <b> (set <s>)")
WLOG="$ROOT/QUEUE_WATCHDOG.log"
MAXLAUNCH="${1:-${QUEUE_MAX_LAUNCH:-3}}"

# Same board:set discovery as run_queue.sh, so new sets need no edit here.
pairs=""
for d in "$ROOT"/boards_unrouted_set*; do
  [ -d "$d" ] || continue
  s="${d##*_set}"
  for f in "$d"/*.kicad_pcb; do
    [ -e "$f" ] || continue
    b=$(basename "$f" .kicad_pcb); pairs="$pairs $b:$s"
  done
done

resfile(){ echo "$ROOT/results_set$2/$1.json"; }
launches(){ grep -c "launched $1 (set $2)" "$STATUS" 2>/dev/null || echo 0; }
is_running(){ pgrep -f "run_board.sh $1 $2" >/dev/null 2>&1; }

echo "watchdog start $(date) maxlaunch=$MAXLAUNCH pairs=$(echo $pairs | wc -w | tr -d ' ')" > "$WLOG"
while true; do
  done_n=0; total=0
  for p in $pairs; do
    total=$((total+1)); b="${p%%:*}"; s="${p##*:}"
    if [ -f "$(resfile "$b" "$s")" ]; then done_n=$((done_n+1)); continue; fi
    n=$(launches "$b" "$s")
    if [ "$n" -ge "$MAXLAUNCH" ] && ! is_running "$b" "$s"; then
      printf '{"board":"%s","set":%s,"status":"FAILED_AFTER_RETRIES","chain_complete":false,"stub":true,"launches":%s,"note":"queue_watchdog stub: worker produced no results JSON after %s attempts"}\n' \
        "$b" "$s" "$n" "$n" > "$(resfile "$b" "$s")"
      echo "$(date) STUBBED $b (set $s) after $n launches" >> "$WLOG"
      done_n=$((done_n+1))
    fi
  done
  echo "$(date) accounted=$done_n/$total" >> "$WLOG"
  [ "$done_n" -ge "$total" ] && { echo "$(date) all boards accounted for; exit" >> "$WLOG"; break; }
  sleep 120
done
