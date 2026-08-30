#!/usr/bin/env python3
"""Top of a cProfile dump: by cumulative time (where the seconds are)
and by own time (what to speed up). usage: prof_top.py FILE.prof [N]"""
import pstats
import sys

p = pstats.Stats(sys.argv[1])
n = int(sys.argv[2]) if len(sys.argv) > 2 else 25
p.strip_dirs()
print('=== by cumulative')
p.sort_stats('cumulative').print_stats(n)
print('=== by own time')
p.sort_stats('tottime').print_stats(n)
