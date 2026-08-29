#!/bin/bash
# The fanout regressions that must still pass after escape_dir_hints:
# with no hints the engine is required to be bit-identical, so any
# failure here is a real break, not a tolerance.
cd "$(dirname "$0")/../tests"
for t in "$@"; do
  printf '%-46s ' "$t"
  if python3 "$t" > "/tmp/ft_$(basename "$t").log" 2>&1; then
    echo PASS
  else
    echo FAIL
    tail -6 "/tmp/ft_$(basename "$t").log" | sed 's/^/    /'
  fi
done
