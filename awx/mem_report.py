#!/usr/bin/env python3
"""mem_report.py SAMPLES -- per chain process: seconds seen, peak RSS,
peak footprint (mem + cmprs), from mem_watch.py's samples."""
import sys
from collections import defaultdict
rows = defaultdict(list)
for line in open(sys.argv[1]):
    p = line.split()
    if len(p) != 6:
        continue
    rows[(int(p[1]), p[5])].append((p[0], float(p[2]), float(p[3]), float(p[4])))
# `mem` is top's physical footprint, which already counts the compressed
# pages; `cmprs` is how much of it was compressed at the sample (a peak
# read under memory pressure is only trustworthy in the mem column)
print(f'{"pid":>6} {"name":<18} {"first":>8} {"last":>8} {"n":>4} {"peak rss":>9} {"peak mem":>9} {"peak cmprs":>10}')
for (pid, name), r in sorted(rows.items(), key=lambda kv: kv[1][0][0]):
    print(f'{pid:>6} {name:<18} {r[0][0]:>8} {r[-1][0]:>8} {len(r):>4} '
          f'{max(x[1] for x in r):>7.0f}MB {max(x[2] for x in r):>7.0f}MB {max(x[3] for x in r):>8.0f}MB')
