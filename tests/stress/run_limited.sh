#!/bin/bash
# Run a command with a ~4 GB RSS watchdog (process + direct children).
# Usage: run_limited.sh <cmd> [args...]   (override with LIMIT_KB=<kb>)
# Exits 137 with MEMORY_LIMIT_EXCEEDED on stderr if the limit is breached.
#
# Command recording for the redo harness (issue #132) is done by the tools
# themselves (redo_record.record_invocation, called from each board-mutating
# CLI's main()), gated on the REDO_MANIFEST env var. That captures the run
# reliably even when a command is NOT routed through this wrapper, which the LLM
# agent does inconsistently. This wrapper therefore only enforces the memory cap.
LIMIT_KB=${LIMIT_KB:-4194304}
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
