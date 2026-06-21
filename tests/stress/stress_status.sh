#!/bin/bash
# Orchestration status for the stress-test corpus (set 1 + set 2 + set 3 ...).
#
# Classifies every board from DISK so a context-summarized parent can resume
# cold (see RUNBOOK.md "Driving the run"):
#   DONE    = results JSON exists
#   RUNNING = a process matches the run-dir path, OR the run dir was touched in
#             the last 45 min (covers the pgrep blind-spot while an agent is
#             between commands, AND a long single signal-route step on a big
#             board that writes no intermediate files; matches RUNBOOK threshold)
#   TODO    = everything else (safe to launch)
#
# Board lists are derived from boards_unrouted*/ so this never goes stale when
# the corpus changes. Override the data root with $1 or $STRESS_ROOT.
ROOT="${1:-${STRESS_ROOT:-$HOME/Documents/kicad_stress_test}}"

# Pairs across set 1 (boards_unrouted/) and every set N (boards_unrouted_setN/).
PAIRS=""
for d in "$ROOT"/boards_unrouted "$ROOT"/boards_unrouted_set*; do
  [ -d "$d" ] || continue
  case "$d" in *_set*) s="${d##*_set}";; *) s=1;; esac
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
  if [ "$s" = "1" ]; then resdir="$ROOT/results"; rundir="runs/$b"; else resdir="$ROOT/results_set$s"; rundir="runs_set$s/$b"; fi
  if [ -f "$resdir/$b.json" ]; then done_n=$((done_n+1)); done_l="$done_l $b"; return; fi
  if pgrep -f "$rundir" >/dev/null 2>&1; then run_n=$((run_n+1)); run_l="$run_l ${b}[s$s]"; return; fi
  if [ -d "$ROOT/$rundir" ] && [ -n "$(find "$ROOT/$rundir" -mmin -45 2>/dev/null | head -1)" ]; then
    run_n=$((run_n+1)); run_l="$run_l ${b}[s$s,idle?]"; return; fi
  todo_n=$((todo_n+1)); todo_l="$todo_l ${b}[s$s]"
}
for pair in $PAIRS; do check "${pair%%:*}" "${pair##*:}"; done
echo "DONE ($done_n/$total):$done_l"
echo "RUNNING ($run_n):$run_l"
echo "TODO ($todo_n):$todo_l"
echo "--- free slots (4 - running): $((4 - run_n))"
