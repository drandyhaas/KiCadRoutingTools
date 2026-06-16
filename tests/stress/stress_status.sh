#!/bin/bash
# Orchestration status for the stress-test corpus (set 1 + set 2).
#
# Classifies every board from DISK so a context-summarized parent can resume
# cold (see RUNBOOK.md "Driving the run"):
#   DONE    = results JSON exists
#   RUNNING = a process matches the run-dir path, OR the run dir was touched in
#             the last 15 min (covers the pgrep blind-spot while an agent is
#             between commands; matches RUNBOOK's 15-min-idle LOST threshold)
#   TODO    = everything else (safe to launch)
#
# Board lists are derived from boards_unrouted*/ so this never goes stale when
# the corpus changes. Override the data root with $1 or $STRESS_ROOT.
ROOT="${1:-${STRESS_ROOT:-$HOME/Documents/kicad_stress_test}}"

SET1=$(ls "$ROOT"/boards_unrouted/*.kicad_pcb 2>/dev/null | xargs -n1 basename 2>/dev/null | sed 's/\.kicad_pcb$//')
SET2=$(ls "$ROOT"/boards_unrouted_set2/*.kicad_pcb 2>/dev/null | xargs -n1 basename 2>/dev/null | sed 's/\.kicad_pcb$//')
total=$(( $(echo "$SET1" | grep -c .) + $(echo "$SET2" | grep -c .) ))

done_n=0; run_n=0; todo_n=0
done_l=""; run_l=""; todo_l=""
check() { # $1 board, $2 set(1|2)
  local b=$1 s=$2 resdir rundir
  if [ "$s" = "1" ]; then resdir="$ROOT/results"; rundir="runs/$b"; else resdir="$ROOT/results_set2"; rundir="runs_set2/$b"; fi
  if [ -f "$resdir/$b.json" ]; then done_n=$((done_n+1)); done_l="$done_l $b"; return; fi
  if pgrep -f "$rundir" >/dev/null 2>&1; then run_n=$((run_n+1)); run_l="$run_l ${b}[s$s]"; return; fi
  if [ -d "$ROOT/$rundir" ] && [ -n "$(find "$ROOT/$rundir" -mmin -15 2>/dev/null | head -1)" ]; then
    run_n=$((run_n+1)); run_l="$run_l ${b}[s$s,idle?]"; return; fi
  todo_n=$((todo_n+1)); todo_l="$todo_l ${b}[s$s]"
}
for b in $SET1; do check "$b" 1; done
for b in $SET2; do check "$b" 2; done
echo "DONE ($done_n/$total):$done_l"
echo "RUNNING ($run_n):$run_l"
echo "TODO ($todo_n):$todo_l"
echo "--- free slots (4 - running): $((4 - run_n))"
