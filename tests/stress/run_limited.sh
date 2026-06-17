#!/bin/bash
# Run a command with a ~4 GB RSS watchdog (process + direct children).
# Usage: run_limited.sh <cmd> [args...]   (override with LIMIT_KB=<kb>)
# Exits 137 with MEMORY_LIMIT_EXCEEDED on stderr if the limit is breached.
#
# Command recording (issue #132): because the RUNBOOK routes EVERY routing /
# fanout / plane / check command through this wrapper, this is the single choke
# point that can record the exact command sequence a board run executed. Each
# invocation is appended (safely quoted) to a manifest so a later, no-LLM
# redo_stress_test.py can replay the run deterministically. Set REDO_MANIFEST to
# a path to override the default ($PWD/redo_commands.sh), or to /dev/null to
# disable. Recording is best-effort and never affects the wrapped command.
LIMIT_KB=${LIMIT_KB:-4194304}

MANIFEST="${REDO_MANIFEST:-$PWD/redo_commands.sh}"
if [ "$MANIFEST" != /dev/null ]; then
  {
    if [ ! -s "$MANIFEST" ]; then
      printf '#!/bin/bash\n# Auto-recorded stress-test command manifest (issue #132).\n'
      printf '# Replay with redo_stress_test.py. cwd at record time: %s\n' "$PWD"
      printf 'set -e\n'
    fi
    # Record the cwd (commands reference intermediate files by relative name) and
    # the fully-quoted argv on the next line.
    printf '# cwd=%q\n' "$PWD"
    printf '%q ' "$@"
    printf '\n'
  } >> "$MANIFEST" 2>/dev/null || true
fi

"$@" &
PID=$!
trap 'kill -9 $PID 2>/dev/null' INT TERM
while kill -0 $PID 2>/dev/null; do
  RSS=$(ps -o rss= -p $PID 2>/dev/null | awk '{print $1+0}')
  CH=$(pgrep -P $PID 2>/dev/null | while read c; do ps -o rss= -p "$c" 2>/dev/null; done | awk '{s+=$1} END{print s+0}')
  TOT=$(( ${RSS:-0} + ${CH:-0} ))
  if [ "$TOT" -gt "$LIMIT_KB" ]; then
    echo "MEMORY_LIMIT_EXCEEDED: ${TOT}KB rss > ${LIMIT_KB}KB cap - killing job" >&2
    pgrep -P $PID 2>/dev/null | xargs -I{} kill -9 {} 2>/dev/null
    kill -9 $PID 2>/dev/null
    exit 137
  fi
  sleep 2
done
wait $PID
