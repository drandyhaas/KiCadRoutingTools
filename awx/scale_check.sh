#!/bin/bash
# Bus detection + certified side choice + LIS refinement across the
# coherent ladder, to see whether the mechanism holds as K grows.
cd "$(dirname "$0")"
for K in "$@"; do
  echo "================ K=$K"
  python3 probe_lis_headroom.py "$K" 2>&1 \
    | grep -E "REFUSED|bus\[|bus of|BEST|headroom|LIS:"
done
echo "=== scale check done"
