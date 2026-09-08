#!/bin/bash
# mem_chain.sh TAG K... -- chain_k.sh under the memory sampler
# (mem_watch.py, 1 s), report at the end. BASE / DEST as chain_k.sh.
cd "$(dirname "$0")"
OUT="tmp/${1}_mem.txt"
rm -f "$OUT" "$OUT.stop"
python3 mem_watch.py "$OUT" 1 &
W=$!
bash chain_k.sh "$@"
touch "$OUT.stop"; wait $W
echo "=== memory (peak per process) ==="
python3 mem_report.py "$OUT"
