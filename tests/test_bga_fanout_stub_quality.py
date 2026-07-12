#!/usr/bin/env python3
"""Stub-quality regression guard for bga_fanout (issue #242 follow-up).

Runs a real BGA fanout (glasgow_revC U30 - BGA-121, 0.8mm pitch, 13 diff pairs,
4 copper layers) and measures four stub-quality metrics, asserting each is **no
worse than the baseline** captured when this test was written. This locks in the
#242 fixes (no escape-stub crossings, no loop-backs, coupled diff pairs) and the
healthy grid-landing / layer-balance behaviour, so future changes can't quietly
regress them.

The four checks:
  1. Grid landing   - stub ends land on the routing grid (off-grid count bounded).
  2. Loop-backs     - every stub runs monotonically toward its escape edge.
  3. Pair coupling  - diff-pair P/N stub ends that converge to ~pair spacing.
  4. Layer balance  - stubs spread across all requested layers, evenly enough.

Plus: no diff-pair P/N SEGMENT-CROSSINGs in the output (the core #242 defect).

Baselines are deliberately a hair looser than the measured values so the test
guards against regressions without being brittle to incidental shifts. bga_fanout
geometry is deterministic, so the measured values themselves are stable.

Run:
    python3 tests/test_bga_fanout_stub_quality.py
"""
import math
import os
import subprocess
import sys
import tempfile
from collections import defaultdict, Counter

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb
from net_queries import extract_diff_pair_base

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
COMP = 'U30'
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
GRID = 0.1
TRACK_W, GAP = 0.1, 0.1
PAIR_SPACING = TRACK_W + GAP            # 0.2 mm
COUPLED_MAX = 0.30                      # stub-end dist <= this == "coupled"

# --- Baselines (measured at write time; thresholds = "no worse than") ---------
ESCAPED_MIN = 90        # measured 90
OFFGRID_MAX = 1         # measured 1 (a single edge-boundary exit w/o end jog)
LOOPBACKS_MAX = 0       # measured 0  (HARD: #242)
CROSSINGS_MAX = 0       # measured 0  (HARD: #242)
COUPLED_MIN = 7         # measured 7 of 13 pairs converge to <=0.30 mm
LAYER_MIN_EACH = 5      # measured min per requested layer = 11; keep margin


def stub_terminals(pcb, comp):
    """Per fanned net: (stub_end_xy, stub_layer, ordered_polyline)."""
    fp = pcb.footprints[comp]
    pad_of_net = {}
    for p in fp.pads:
        if p.net_id:
            pad_of_net.setdefault(p.net_id, (p.global_x, p.global_y))
    out = {}
    for net_id, (px, py) in pad_of_net.items():
        segs = [s for s in pcb.segments if s.net_id == net_id]
        if not segs:
            continue
        adj = defaultdict(list)
        deg = defaultdict(int)
        lay = defaultdict(list)
        for s in segs:
            a = (round(s.start_x, 4), round(s.start_y, 4))
            b = (round(s.end_x, 4), round(s.end_y, 4))
            if a == b:
                continue
            adj[a].append(b); adj[b].append(a)
            deg[a] += 1; deg[b] += 1
            lay[a].append(s.layer); lay[b].append(s.layer)
        if not deg:
            continue
        start = min(deg, key=lambda n: (n[0] - px) ** 2 + (n[1] - py) ** 2)
        path = [start]; seen = {start}; prev = None; cur = start
        while True:
            nxts = [n for n in adj[cur] if n != prev and n not in seen]
            if not nxts:
                break
            path.append(nxts[0]); seen.add(nxts[0]); prev, cur = cur, nxts[0]
        se = path[-1]
        out[net_id] = (se, Counter(lay[se]).most_common(1)[0][0], path)
    return out


def on_grid(v):
    return abs(v / GRID - round(v / GRID)) < 1e-6


def check_loopbacks(terminals):
    """Return list of net_ids whose stub doubles back toward the BGA."""
    BACK_TOL = 0.01
    bad = []
    for net_id, (se, _lay, path) in terminals.items():
        if len(path) < 2:
            continue
        dx, dy = path[-1][0] - path[0][0], path[-1][1] - path[0][1]
        axis, dirn = (0, 1 if dx >= 0 else -1) if abs(dx) >= abs(dy) else (1, 1 if dy >= 0 else -1)
        worst = 0.0
        for i in range(1, len(path)):
            worst = min(worst, (path[i][axis] - path[i - 1][axis]) * dirn)
        if worst < -BACK_TOL:
            bad.append(net_id)
    return bad


def count_segment_crossings(pcb):
    """Count diff-pair P/N escape-stub crossings (the core #242 defect)."""
    def cross(p1, p2, p3, p4):
        def o(a, b, c):
            return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
        for a in (p1, p2):
            for b in (p3, p4):
                if abs(a[0]-b[0]) < 1e-9 and abs(a[1]-b[1]) < 1e-9:
                    return False
        d1, d2, d3, d4 = o(p3,p4,p1), o(p3,p4,p2), o(p1,p2,p3), o(p1,p2,p4)
        return ((d1>1e-9 and d2<-1e-9) or (d1<-1e-9 and d2>1e-9)) and \
               ((d3>1e-9 and d4<-1e-9) or (d3<-1e-9 and d4>1e-9))
    # group segments by diff-pair base + layer
    base_of_net = {}
    for nid, net in pcb.nets.items():
        r = extract_diff_pair_base(net.name or '')
        if r:
            base_of_net[nid] = r[0]
    n = 0
    by_base = defaultdict(list)
    for s in pcb.segments:
        if s.net_id in base_of_net:
            by_base[(base_of_net[s.net_id], s.layer)].append(s)
    for (_base, _lay), segs in by_base.items():
        for i, s1 in enumerate(segs):
            for s2 in segs[i+1:]:
                if s1.net_id == s2.net_id:
                    continue
                if cross((s1.start_x, s1.start_y), (s1.end_x, s1.end_y),
                         (s2.start_x, s2.start_y), (s2.end_x, s2.end_y)):
                    n += 1
    return n


def main():
    out = os.path.join(tempfile.mkdtemp(), 'glasgow_fan.kicad_pcb')
    cmd = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'bga_fanout.py'),
           BOARD, '--output', out, '--component', COMP, '--layers', *LAYERS,
           '--nets', '*', '!GND', '!+3V3', '!+1V2',
           '--diff-pairs', '*_P', '*_N',
           '--track-width', str(TRACK_W), '--clearance', '0.1',
           '--via-size', '0.45', '--via-drill', '0.2', '--diff-pair-gap', str(GAP),
           # This test grades the CHANNEL engine's stub quality (grid snap,
           # layer spread, coupling); the default 'auto' would switch this
           # board to the under-pad engine (channel drops 7 balls, #288).
           '--escape-method', 'channel']
    res = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    summary = next((l for l in res.stdout.splitlines() if 'JSON_SUMMARY' in l), '')
    if res.returncode != 0 or not os.path.exists(out):
        print("FAIL: bga_fanout did not produce output")
        print(res.stdout[-2000:]); print(res.stderr[-2000:])
        return 1

    import json
    js = json.loads(summary.split('JSON_SUMMARY:', 1)[1])
    escaped = js['escaped']

    pcb = parse_kicad_pcb(out)
    terminals = stub_terminals(pcb, COMP)

    # 1) grid landing
    off_grid = [nid for nid, (se, _l, _p) in terminals.items()
                if not (on_grid(se[0]) and on_grid(se[1]))]
    # 2) loop-backs
    loopbacks = check_loopbacks(terminals)
    # 3) pair coupling
    pairs = defaultdict(dict)
    for nid in terminals:
        r = extract_diff_pair_base(pcb.nets[nid].name or '')
        if r:
            pairs[(r[0], r[2])]['P' if r[1] else 'N'] = nid
    dists = []
    for d in pairs.values():
        if 'P' in d and 'N' in d:
            a, b = terminals[d['P']][0], terminals[d['N']][0]
            dists.append(math.hypot(a[0]-b[0], a[1]-b[1]))
    coupled = sum(1 for x in dists if x <= COUPLED_MAX)
    # 4) layer balance
    layc = Counter(terminals[nid][1] for nid in terminals)
    per_layer = {L: layc.get(L, 0) for L in LAYERS}
    # cross-check
    crossings = count_segment_crossings(pcb)

    print(f"  escaped={escaped}  stub_ends={len(terminals)}")
    print(f"  1) off-grid stub ends : {len(off_grid)}  (max {OFFGRID_MAX})")
    print(f"  2) loop-backs         : {len(loopbacks)}  (max {LOOPBACKS_MAX})")
    print(f"  3) coupled pairs<= {COUPLED_MAX}: {coupled}/{len(dists)}  (min {COUPLED_MIN})")
    print(f"  4) per-layer stubs    : {per_layer}  (min each {LAYER_MIN_EACH})")
    print(f"  +) P/N seg-crossings  : {crossings}  (max {CROSSINGS_MAX})")

    fails = []
    if escaped < ESCAPED_MIN:
        fails.append(f"escaped {escaped} < {ESCAPED_MIN}")
    if len(off_grid) > OFFGRID_MAX:
        fails.append(f"off-grid {len(off_grid)} > {OFFGRID_MAX}: "
                     + ", ".join(pcb.nets[n].name for n in off_grid))
    if len(loopbacks) > LOOPBACKS_MAX:
        fails.append("loop-backs: " + ", ".join(pcb.nets[n].name for n in loopbacks))
    if crossings > CROSSINGS_MAX:
        fails.append(f"P/N segment-crossings {crossings} > {CROSSINGS_MAX}")
    if coupled < COUPLED_MIN:
        fails.append(f"coupled pairs {coupled} < {COUPLED_MIN}")
    for L in LAYERS:
        if per_layer[L] < LAYER_MIN_EACH:
            fails.append(f"layer {L} only {per_layer[L]} stubs (< {LAYER_MIN_EACH})")

    if fails:
        print("FAIL:")
        for f in fails:
            print("   - " + f)
        return 1
    print("PASS: bga_fanout stub quality no worse than baseline "
          "(grid, loop-backs, coupling, layer balance, crossings)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
