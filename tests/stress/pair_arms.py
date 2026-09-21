#!/usr/bin/env python3
"""Pair two cloud arms board by board from their _raw rows, the way the v0.22.1
validation was scored: a board counts when its chain completed in BOTH arms
with the same final board; totals, direction, and every non-zero delta.

    python3 tests/stress/pair_arms.py <baseline wave dir> <new wave dir>

rank_arms.py needs a control arm inside ONE sweep and cannot score a single-arm
wave; this reads the harvested rows directly. Reproduces the recorded
v0.22.0 -> v0.22.1 numbers (149 boards, real DRC 74 -> 40, 15/10/124)."""
import glob, json, os, sys
def rows(d):
    out = {}
    for f in glob.glob(os.path.join(d, '_raw', '*', 'set*__*.json')):
        r = json.load(open(f)); out[(r['set'], r['board'])] = r
    return out
base, new = rows(sys.argv[1]), rows(sys.argv[2])
keys = sorted(set(base) & set(new))
comp, skipped = [], []
for k in keys:
    b, n = base[k], new[k]
    if not (b.get('chain_complete') and n.get('chain_complete')):
        skipped.append((k, 'chain incomplete', b.get('chain_complete'), n.get('chain_complete'))); continue
    if b.get('final') != n.get('final'):
        skipped.append((k, 'final differs', b.get('final'), n.get('final'))); continue
    comp.append((k, b, n))
def tot(field, which): return sum((r[which].get(field) or 0) for r in comp)
print(f"boards in both arms: {len(keys)}; comparable: {len(comp)}; skipped: {len(skipped)}")
for k, why, a, b in skipped: print(f"  skip {k[0]}/{k[1]}: {why} ({a} vs {b})")
print()
print(f"{'':28s} {'base':>6s} {'new':>6s} {'delta':>6s}")
for field in ('drc_real', 'nets_incomplete', 'kicad_drc'):
    bo = sum((b.get(field) or 0) for _, b, n in comp); nw = sum((n.get(field) or 0) for _, b, n in comp)
    print(f"{field:28s} {bo:6d} {nw:6d} {nw-bo:+6d}")
better = worse = tied = 0
deltas = []
for k, b, n in comp:
    d = ((n.get('drc_real') or 0) - (b.get('drc_real') or 0), (n.get('nets_incomplete') or 0) - (b.get('nets_incomplete') or 0))
    s = d[0] + d[1]
    if s < 0: better += 1
    elif s > 0: worse += 1
    else: tied += 1
    if d != (0, 0): deltas.append((s, k, b, n, d))
print(f"\ndirection (drc_real + nets_incomplete): {better} better / {worse} worse / {tied} tied")
print("\nper-board deltas (new - base), worst first:")
for s, k, b, n, d in sorted(deltas, key=lambda t: -t[0]):
    print(f"  {k[0]:>5s}/{k[1]:32s} drc {b.get('drc_real'):>3} -> {n.get('drc_real'):>3} ({d[0]:+d})   nets {b.get('nets_incomplete'):>3} -> {n.get('nets_incomplete'):>3} ({d[1]:+d})")
cb = sum(b.get('cpu_seconds') or 0 for _, b, n in comp); cn = sum(n.get('cpu_seconds') or 0 for _, b, n in comp)
print(f"\ncpu_seconds total: base {cb:.0f}  new {cn:.0f}  ({(cn/cb-1)*100:+.1f}%)" if cb else "")
