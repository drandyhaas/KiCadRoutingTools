#!/usr/bin/env python3
"""Repair malformed Edge.Cuts outlines on corpus boards (#450 board-prep).

Two defect classes seen on real corpus boards, both flagged by kicad-cli as
``invalid_outline`` (which then spams every wave's DRC report):

- **pinci**: 199x "segment has null or very small length: 0 nm" --
  zero-length Edge.Cuts primitives (drawing slop, duplicated points).
  Fix: delete them; they contribute no geometry.
- **bus_pirate5**: 10x "not a closed shape" -- the outline chain has
  degree-1 endpoints: a few 8um drafting slops plus genuinely missing
  straight pieces (up to ~9mm) along otherwise-straight edges.
  Fix: bridge each odd endpoint to its nearest odd partner with a straight
  gr_line. Existing geometry is never moved or reshaped.

Usage:
    python3 tests/stress/fix_outline_gaps.py BOARD.kicad_pcb [--max-bridge 12]
        [--dry-run]

Writes in place, keeping a one-time BOARD.kicad_pcb.origoutline backup.
Verify afterward with:
    kicad-cli pcb drc BOARD.kicad_pcb --format json --severity-error ...
"""
import argparse
import math
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

_NUM = r'(-?[\d.]+)'


def _edge_blocks(src):
    """(global_blocks, fp_blocks): (span, kind, start_xy, end_xy) for every
    Edge.Cuts line/arc primitive. gr_* coordinates are board-global (usable
    for degree/bridging); fp_* coordinates are FOOTPRINT-LOCAL (pinci draws
    its whole outline with footprint fp_lines), so those participate only in
    the zero-length cull, never in gap pairing."""
    from kicad_parser import find_matching_paren
    out, fp_out = [], []
    for m in re.finditer(r'\((gr|fp)_(line|arc)\b', src):
        end = find_matching_paren(src, m.start())
        block = src[m.start():end]
        if '"Edge.Cuts"' not in block:
            continue
        s = re.search(r'\(start\s+' + _NUM + r'\s+' + _NUM + r'\)', block)
        e = re.search(r'\(end\s+' + _NUM + r'\s+' + _NUM + r'\)', block)
        if not (s and e):
            continue
        rec = ((m.start(), end), m.group(2),
               (float(s.group(1)), float(s.group(2))),
               (float(e.group(1)), float(e.group(2))))
        (out if m.group(1) == 'gr' else fp_out).append(rec)
    return out, fp_out


def fix_board(path, max_bridge=12.0, dry_run=False, drop_crossing_rings=False):
    src = open(path, encoding='utf-8').read()
    blocks, fp_blocks = _edge_blocks(src)

    # 1. zero-length primitives (the pinci class; includes footprint-embedded
    # fp_line/fp_arc -- pinci carries 323 degenerate ones inside footprints)
    dead = [(span, s) for span, kind, s, e in blocks + fp_blocks
            if math.hypot(e[0] - s[0], e[1] - s[1]) < 1e-6]
    live = [(kind, s, e) for span, kind, s, e in blocks
            if math.hypot(e[0] - s[0], e[1] - s[1]) >= 1e-6]

    # 2. odd-degree endpoints among the live chain (the bus_pirate5 class)
    from collections import defaultdict
    deg = defaultdict(int)
    for kind, s, e in live:
        deg[(round(s[0], 4), round(s[1], 4))] += 1
        deg[(round(e[0], 4), round(e[1], 4))] += 1
    odd = [p for p, d in deg.items() if d % 2 == 1]

    # nearest-neighbor pairing (greedy, shortest gaps first)
    bridges = []
    pool = list(odd)
    pairs = []
    while len(pool) >= 2:
        best = None
        for i in range(len(pool)):
            for j in range(i + 1, len(pool)):
                d = math.hypot(pool[i][0] - pool[j][0], pool[i][1] - pool[j][1])
                if best is None or d < best[0]:
                    best = (d, i, j)
        d, i, j = best
        a, b = pool[i], pool[j]
        pool = [p for k, p in enumerate(pool) if k not in (i, j)]
        if d > max_bridge:
            print(f"  WARN: odd endpoints {a} / {b} are {d:.3f}mm apart "
                  f"(> --max-bridge {max_bridge}); not bridged")
            continue
        pairs.append((a, b, d))
        bridges.append(
            '\t(gr_line\n'
            f'\t\t(start {a[0]} {a[1]})\n'
            f'\t\t(end {b[0]} {b[1]})\n'
            '\t\t(stroke\n\t\t\t(width 0.05)\n\t\t\t(type default)\n\t\t)\n'
            '\t\t(layer "Edge.Cuts")\n'
            f'\t\t(uuid "0f1c0000-450b-4f1c-9{len(bridges):03d}-{abs(hash((a, b))) % 10**12:012d}")\n'
            '\t)\n')

    # 3. open fragments: a connected component that STILL has odd-degree
    #    endpoints after bridging can never close (bus_pirate5: two stray rays
    #    of an unfinished panel frame crossing the real outline -> kicad
    #    "self-intersecting"). Delete the whole fragment.
    bridged_deg = dict(deg)
    for a, b, _d in pairs:
        bridged_deg[a] += 1
        bridged_deg[b] += 1
    # union-find components over live gr_ primitives (endpoint coincidence)
    parent = {}

    def _find(x):
        while parent.get(x, x) != x:
            parent[x] = parent.get(parent[x], parent[x])
            x = parent[x]
        return x

    def _union(x, y):
        parent.setdefault(x, x)
        parent.setdefault(y, y)
        rx, ry = _find(x), _find(y)
        if rx != ry:
            parent[rx] = ry
    live_spans = [(span, s, e) for span, kind, s, e in blocks
                  if math.hypot(e[0] - s[0], e[1] - s[1]) >= 1e-6]
    for span, s, e in live_spans:
        _union((round(s[0], 4), round(s[1], 4)), (round(e[0], 4), round(e[1], 4)))
    for a, b, _d in pairs:
        _union(a, b)
    open_roots = {_find(p) for p, d in bridged_deg.items() if d % 2 == 1}
    frag = [(span, s) for span, s, e in live_spans
            if _find((round(s[0], 4), round(s[1], 4))) in open_roots]

    # 4. CROSSING rings: two closed components whose primitives properly
    #    intersect (bus_pirate5: an unfinished panel frame ring overlapping
    #    the real outline -> kicad "self-intersecting"). Keep the larger-bbox
    #    ring, delete the smaller. Arcs are treated as their chords -- fine
    #    for the line-line crossings this targets.
    from collections import defaultdict as _dd
    comp_spans = _dd(list)
    comp_bbox = {}
    for span, s, e in live_spans:
        root = _find((round(s[0], 4), round(s[1], 4)))
        if root in open_roots:
            continue  # already deleted as an open fragment
        comp_spans[root].append((span, s, e))
        bb = comp_bbox.get(root)
        xs = (s[0], e[0]); ys = (s[1], e[1])
        if bb is None:
            comp_bbox[root] = [min(xs), min(ys), max(xs), max(ys)]
        else:
            bb[0] = min(bb[0], *xs); bb[1] = min(bb[1], *ys)
            bb[2] = max(bb[2], *xs); bb[3] = max(bb[3], *ys)

    def _cross(p1, p2, p3, p4):
        def o(a, b, c):
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        d1, d2 = o(p3, p4, p1), o(p3, p4, p2)
        d3, d4 = o(p1, p2, p3), o(p1, p2, p4)
        return d1 * d2 < 0 and d3 * d4 < 0

    roots = list(comp_spans) if drop_crossing_rings else []
    crossing_drop = set()
    for i in range(len(roots)):
        for j in range(i + 1, len(roots)):
            ra, rb = roots[i], roots[j]
            ba, bb2 = comp_bbox[ra], comp_bbox[rb]
            if ba[2] < bb2[0] or bb2[2] < ba[0] or ba[3] < bb2[1] or bb2[3] < ba[1]:
                continue  # disjoint bboxes cannot cross
            if any(_cross(s1, e1, s2, e2)
                   for _sp1, s1, e1 in comp_spans[ra]
                   for _sp2, s2, e2 in comp_spans[rb]):
                area = lambda b: (b[2] - b[0]) * (b[3] - b[1])
                loser = ra if area(ba) < area(bb2) else rb
                crossing_drop.add(loser)
    for root in crossing_drop:
        print(f"  dropping ring crossing a larger ring "
              f"(bbox {['%.1f' % v for v in comp_bbox[root]]}, "
              f"{len(comp_spans[root])} primitive(s))")
        frag.extend((span, s) for span, s, e in comp_spans[root])
    if frag:
        dead.extend(frag)

    print(f"{os.path.basename(path)}: {len(blocks)} Edge.Cuts primitive(s); "
          f"removing {len(dead) - len(frag)} zero-length + {len(frag)} "
          f"open-fragment, bridging {len(pairs)} gap(s)"
          + (f" ({', '.join(f'{d*1000:.0f}um' if d < 1 else f'{d:.2f}mm' for _, _, d in pairs)})"
             if pairs else ""))
    if dry_run or (not dead and not bridges):
        return bool(dead or bridges)

    # apply: delete dead blocks back-to-front, then append bridges
    for (a, b), _ in sorted(dead, key=lambda x: -x[0][0]):
        # also swallow leading whitespace/newline before the block
        lead = a
        while lead > 0 and src[lead - 1] in ' \t':
            lead -= 1
        if lead > 0 and src[lead - 1] == '\n':
            lead -= 1
        src = src[:lead] + src[b:]
    if bridges:
        idx = src.rstrip().rfind(')')
        src = src.rstrip()[:idx] + ''.join(bridges) + src.rstrip()[idx:] + '\n'

    backup = path + '.origoutline'
    if not os.path.exists(backup):
        os.rename(path, backup)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(src)
    print(f"  wrote {path} (backup: {os.path.basename(backup)})")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('boards', nargs='+')
    ap.add_argument('--max-bridge', type=float, default=12.0,
                    help='max gap (mm) to close with a straight bridge')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--drop-crossing-rings', action='store_true',
                    help='EXPERIMENTAL: delete the smaller ring of a crossing '
                         'pair (boards with several overlapping outline drafts '
                         'can lose the real one -- verify with kicad-cli after)')
    args = ap.parse_args()
    for b in args.boards:
        fix_board(b, max_bridge=args.max_bridge, dry_run=args.dry_run,
                  drop_crossing_rings=args.drop_crossing_rings)


if __name__ == '__main__':
    main()
