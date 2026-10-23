#!/bin/bash
# Orchestration status for the stress-test corpus (set 1 + set 2 + set 3 ...).
#
# Classifies every board from DISK so a context-summarized parent can resume
# cold (see RUNBOOK.md "Orchestration" and its "Manual fallback"):
#   DONE    = results JSON exists
#   RUNNING = run_queue.sh's is_running: its worker process, else (no
#             .worker_done marker) a process matches the run-dir path, OR the
#             run dir was touched in the last 180 min (covers the pgrep
#             blind-spot while an agent is between commands, AND a long single
#             signal-route step on a big board that writes no intermediate
#             files, up to the 3-hour per-command cap; matches RUNBOOK threshold)
#   TODO    = everything else (safe to launch)
# and then whether run_queue.sh would admit another board now (its load-based
# rule; there is no fixed slot count).
#
# Board lists are derived from boards_unrouted*/ so this never goes stale when
# the corpus changes. Override the data root with $1 or $STRESS_ROOT.
ROOT="${1:-${STRESS_ROOT:-$HOME/Documents/kicad_stress_test}}"

# Pairs across every set N (boards_unrouted_setN/; set 1 = boards_unrouted_set1).
PAIRS=""
for d in "$ROOT"/boards_unrouted_set*; do
  [ -d "$d" ] || continue
  s="${d##*_set}"
  for f in "$d"/*.kicad_pcb; do
    [ -e "$f" ] || continue
    PAIRS="$PAIRS $(basename "$f" .kicad_pcb):$s"
  done
done
total=$(echo $PAIRS | wc -w | tr -d ' ')

done_n=0; run_n=0; todo_n=0
done_l=""; run_l=""; todo_l=""
check() { # $1 board, $2 set(1|2|3...)
  local b=$1 s=$2 resdir rundir
  resdir="$ROOT/results_set$s"; rundir="runs_set$s/$b"
  if [ -f "$resdir/$b.json" ]; then done_n=$((done_n+1)); done_l="$done_l $b"; return; fi
  if pgrep -f "run_board.sh $b $s" >/dev/null 2>&1; then run_n=$((run_n+1)); run_l="$run_l ${b}[s$s]"; return; fi
  # .worker_done = the worker has EXITED (ok or NORESULT); run_queue.sh reads it
  # the same way, so an orphaned step still touching the dir is not RUNNING.
  if [ -f "$ROOT/$rundir/.worker_done" ]; then todo_n=$((todo_n+1)); todo_l="$todo_l ${b}[s$s]"; return; fi
  if pgrep -f "$rundir" >/dev/null 2>&1; then run_n=$((run_n+1)); run_l="$run_l ${b}[s$s]"; return; fi
  if [ -d "$ROOT/$rundir" ] && [ -n "$(find "$ROOT/$rundir" -mmin -180 2>/dev/null | head -1)" ]; then
    run_n=$((run_n+1)); run_l="$run_l ${b}[s$s,idle?]"; return; fi
  todo_n=$((todo_n+1)); todo_l="$todo_l ${b}[s$s]"
}
for pair in $PAIRS; do check "${pair%%:*}" "${pair##*:}"; done
echo "DONE ($done_n/$total):$done_l"
echo "RUNNING ($run_n):$run_l"
echo "TODO ($todo_n):$todo_l"

# Admission, by run_queue.sh's own rule (see its concurrency note): a board
# launches while running < the ceiling, at most one per QUEUE_LAUNCH_INTERVAL;
# below the floor whatever the load, else only while memory pressure is not
# CRITICAL and the 1-minute load average is under load_max. The numbers are the
# live queue's (its "queue start:" line) when QUEUE_STATUS.txt has one, else
# run_queue.sh's defaults.
NCORE=$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)
CONC=8
LOAD_MAX="${QUEUE_LOAD_MAX:-$(( NCORE - 2 > 0 ? NCORE - 2 : 1 ))}"
CONC_MIN="${QUEUE_CONC_MIN:-2}"
src="run_queue.sh defaults"
start=$(grep -m1 'queue start:' "$ROOT/QUEUE_STATUS.txt" 2>/dev/null)
if [ -n "$start" ]; then
  v=$(echo "$start" | sed -n 's/.* ceiling=\([0-9]*\).*/\1/p');    [ -n "$v" ] && CONC=$v
  v=$(echo "$start" | sed -n 's/.* load_max=\([0-9.]*\).*/\1/p');  [ -n "$v" ] && LOAD_MAX=$v
  v=$(echo "$start" | sed -n 's/.* floor=\([0-9]*\).*/\1/p');      [ -n "$v" ] && CONC_MIN=$v
  src="live queue"
fi
# Same probes as run_queue.sh's load1 / mem_critical.
if [ -r /proc/loadavg ]; then load=$(awk '{print $1}' /proc/loadavg)
else load=$(sysctl -n vm.loadavg 2>/dev/null | awk '{print $2}'); fi
lvl=$(sysctl -n kern.memorystatus_vm_pressure_level 2>/dev/null || echo 1)
case "$lvl" in ''|*[!0-9]*) lvl=1 ;; esac
if   [ "$run_n" -ge "$CONC" ];      then adm="hold: at ceiling $CONC"
elif [ "$run_n" -lt "$CONC_MIN" ];  then adm="admit: below the floor $CONC_MIN, whatever the load"
elif [ "$lvl" -ge 4 ];              then adm="hold: memory pressure CRITICAL"
elif awk -v l="$load" -v m="$LOAD_MAX" 'BEGIN{exit !(l+0 >= m+0)}'
                                    then adm="hold: load $load >= $LOAD_MAX"
else                                     adm="admit: load $load < $LOAD_MAX"; fi
echo "--- admission ($src: ceiling $CONC, floor $CONC_MIN, load_max $LOAD_MAX):" \
     "running $run_n -> $adm (at most one launch per ${QUEUE_LAUNCH_INTERVAL:-60} s)"
