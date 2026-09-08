#!/usr/bin/env python3
"""mem_watch.py OUT [interval] -- every `interval` seconds, one line per
process of the chain (fanout_from_plan / braid / check_drc / grade_k /
kicad-cli ...): time pid rss_MB mem_MB cmprs_MB name.  `rss` is ps's
resident size; `mem` and `cmprs` are top's, and mem + cmprs is the
footprint that survives memory pressure (macOS compresses pages OUT of
RSS, so a peak read under load is low without the cmprs column).
Stops when OUT + '.stop' exists."""
import os, re, subprocess, sys, time

out = sys.argv[1]
iv = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
pat = re.compile(r'(fanout_from_plan|braid|check_drc|grade_k|coherent_nets'
                 r'|flow_frame|check_connected|kicad-cli|src_fanout|bga_fanout)')


def size(s):
    s = s.rstrip('+-')
    m = re.match(r'([\d.]+)([KMGB]?)', s)
    if not m:
        return 0.0
    v = float(m.group(1))
    return v * {'B': 1 / 1048576, 'K': 1 / 1024, 'M': 1, 'G': 1024,
                '': 1 / 1048576}[m.group(2)]


with open(out, 'a') as f:
    while not os.path.exists(out + '.stop'):
        ps = subprocess.run(['ps', '-axo', 'pid=,rss=,command='],
                            capture_output=True, text=True).stdout
        procs = {}
        for line in ps.splitlines():
            parts = line.split(None, 2)
            if len(parts) < 3:
                continue
            pid, rss, cmd = parts
            if int(pid) == os.getpid() or 'mem_w' 'atch' in cmd:
                continue
            m = pat.search(cmd)
            if m:
                procs[int(pid)] = (int(rss) / 1024, m.group(1))
        cm = {}
        if procs:
            tp = subprocess.run(['top', '-l', '1', '-stats', 'pid,mem,cmprs',
                                 '-n', '40', '-o', 'mem'],
                                capture_output=True, text=True).stdout
            for line in tp.splitlines():
                parts = line.split()
                if len(parts) == 3 and parts[0].isdigit():
                    cm[int(parts[0])] = (size(parts[1]), size(parts[2]))
        ts = time.strftime('%H:%M:%S')
        for pid, (rss, name) in sorted(procs.items()):
            mem, cmp_ = cm.get(pid, (rss, 0.0))
            f.write(f'{ts} {pid} {rss:.0f} {mem:.0f} {cmp_:.0f} {name}\n')
        f.flush()
        time.sleep(iv)
