"""conflict_groups(menu) -- the whole route's move conflicts (select_moves._conflict, strict, stack: what
pages_first._conflicts(menu, strict=True, stack=True) lists pair by pair) stated COMPACTLY, for a CP-SAT model:

  cliques     sets of moves that all conflict pairwise -- one AtMostOne each
  bicliques   (A, B): every move of A conflicts with every move of B -- one auxiliary each (y_A + y_B + the moves in
              both <= 1), their conflicts inside A or inside B stated elsewhere
  pairs       the few pairs neither carries

The relation's five parts:
  lane      two spans in one lane (axis, layer, coordinates within the strict tolerance) that meet: an interval
            graph, whose maximal cliques are the spans live at each start (a sweep) -- raw ends, the relation's own
  crossing  a row run and a column run on one layer that cross within the tolerance (SEL_XING 1: only when one of
            the two moves climbs or runs a street): per crossing point, the row runs through it against the climbing
            column runs, and the climbing row runs against the column runs -- bicliques
  site      two moves with one via site -- a clique
  reach     a move's via within a lane's reach of another move's run (any layer): per site and lane, the site's moves
            against the runs they reach -- bicliques, grouped by exactly which of the site's moves each run reaches
  exit      two exits on one layer closer than the stacking pitch: per face and layer, an interval graph again

Every part applies the relation's own test to its members, so the union states exactly its pairs. zynq U1's bus with
every move escape_moves has (4848 moves, climbs across the array): 3,098,663 pairs either way, none missing, none
extra; pages_first._conflicts listed them in 129 s."""
import collections

import select_moves as sm

TOL = 0.16          # select_moves._conflict's default `tol`


def _interval_cliques(items):
    """items [(a, b, member)]: members whose half-open intervals [a, b + TOUCH) meet, as maximal cliques"""
    ev = []
    for a, b, mbr in items:
        ev.append((a, 1, mbr))
        ev.append((b + sm._TOUCH, 0, mbr))
    ev.sort(key=lambda e: (e[0], e[1]))           # ends before starts at one position (half-open)
    live, out, grown = set(), [], False
    for _pos, kind, mbr in ev:
        if kind == 1:
            live.add(mbr)
            grown = True
        else:
            if grown and len(live) > 1:
                out.append(frozenset(live))
            grown = False
            live.discard(mbr)
    return out


def _windows(vals, tol):
    """sorted distinct values -> the maximal runs of them less than `tol` from end to end: the relation's `same lane`
    (|c1 - c2| < tol) is an interval graph on the coordinates, and these are its maximal cliques. On an array's
    lattice every run is one gap's few wandering values; a STREET lane (a via in an empty band, a stacking pitch from
    the next) chains with its neighbours into runs that overlap, and each pair within `tol` is in one of them."""
    vs = sorted(set(vals))
    out, last = [], -1
    j = 0
    for i in range(len(vs)):
        j = max(j, i)
        while j + 1 < len(vs) and vs[j + 1] - vs[i] < tol:
            j += 1
        if j > last:
            out.append(vs[i:j + 1])
            last = j
    return out


def conflict_groups(menu, stack=True, stack_pitch=None, via_r=None, reach_extra=None, xing=None, tags=None):
    """(cliques [frozenset of (ball, i)], bicliques [(frozenset, frozenset)], pairs [((ball, i), (ball, i))]).
    At the whole route's own sizes by default; a caller at REAL sizes gives the stacking pitch (track + clearance +
    the hug allowance) and, per move, its via's radius (via_r(move)) plus what a run needs beside it (reach_extra:
    clearance + half a track), so a site reaches a lane as far as ITS via does. `xing` the crossing rule
    (select_moves.SEL_XING by default -- 1, a crossing counts when one of the two climbs; 2, every crossing, what a
    plan that lays every ball of an array needs, where two plain escapes out of the interior do cross). A caller that
    means a rule passes it. `tags` (a dict): filled with the parts that stated each clique and biclique -- a set of
    ('lane', layer), ('crossing', layer), ('exit', layer), ('site',), ('reach',) -- for a caller that states a part
    otherwise (joint_escape: a run on no one layer yet, its lane, crossing and exit conflicts K layers' capacity)."""
    xing = sm.SEL_XING if xing is None else xing
    moves = [(k, i, m) for k in sorted(menu) for i, m in enumerate(menu[k])]
    cliques, bicliques, pairs = set(), set(), set()

    def tag(g, kind):
        if tags is not None:
            tags.setdefault(g, set()).add(kind)

    def bi(a, b, kind):
        a, b = frozenset(a), frozenset(b)
        if a and b and (len(a | b) > 1):
            g = (a, b) if sorted(a) <= sorted(b) else (b, a)
            bicliques.add(g)
            tag(g, kind)

    # the spans: (axis, layer, coord, a, b, member, climbs)
    spans = []
    for k, i, m in moves:
        climbs = bool(getattr(m, 'climb', 0) or getattr(m, 'street', 0))
        for key, a, b in sm._lane_spans(m):
            spans.append((key[0], key[2], key[1], a, b, (k, i), climbs))

    # lane: per axis and layer, each window of coordinates within the tolerance, the spans on it by their extent
    by_axis_layer = collections.defaultdict(lambda: collections.defaultdict(list))
    for s in spans:
        by_axis_layer[(s[0], s[1])][s[2]].append(s)
    for al, at in by_axis_layer.items():
        for w in _windows(list(at), TOL):
            ss = [s for c in w for s in at[c]]
            for g in _interval_cliques([(s[3], s[4], s[5]) for s in ss]):
                cliques.add(g)
                tag(g, ('lane', al[1]))

    # crossing: per layer, per row coordinate and column coordinate (the relation's own test, pair by coordinate)
    if xing:
        by_coord = collections.defaultdict(list)          # (axis, layer, coord) -> spans
        for s in spans:
            by_coord[(s[0], s[1], s[2])].append(s)
        for L in {s[1] for s in spans}:
            rows = sorted(c for (ax, ly, c) in by_coord if ax == 'row' and ly == L)
            cols = sorted(c for (ax, ly, c) in by_coord if ax == 'col' and ly == L)
            for y in rows:
                rss = by_coord[('row', L, y)]
                for x in cols:
                    R = [s for s in rss if s[3] - TOL < x < s[4] + TOL]
                    if not R:
                        continue
                    C = [s for s in by_coord[('col', L, x)] if s[3] - TOL < y < s[4] + TOL]
                    if not C:
                        continue
                    if xing >= 2:
                        bi({s[5] for s in R}, {s[5] for s in C}, ('crossing', L))
                    else:
                        bi({s[5] for s in R}, {s[5] for s in C if s[6]}, ('crossing', L))
                        bi({s[5] for s in R if s[6]}, {s[5] for s in C}, ('crossing', L))

    # site
    by_site = collections.defaultdict(list)
    for k, i, m in moves:
        sk = sm._site_key(m)
        if sk is not None:
            by_site[sk].append((k, i, m))
    for sk, mem in by_site.items():
        if len(mem) > 1:
            g = frozenset((k, i) for k, i, _m in mem)
            cliques.add(g)
            tag(g, ('site',))

    # reach: select_moves._site_in_lane of each site move against each run, grouped by exactly which of the site's
    # moves a run reaches (the site's moves stand within a micron of each other, so nearly always all or none)
    if via_r is None or reach_extra is None:
        def reach(m):
            return sm._VIA_REACH
    else:
        def reach(m):
            return via_r(m) + reach_extra
    R_max = max((reach(m) for mem in by_site.values() for _k, _i, m in mem), default=sm._VIA_REACH)
    span_c = int(R_max * 10) + 2                          # 0.1 mm cells either side that can hold a reached lane
    coord_index = collections.defaultdict(list)          # (axis, 0.1 mm cell) -> lane coordinates
    by_coord = collections.defaultdict(list)
    for s in spans:
        by_coord[(s[0], s[2])].append(s)
    for (ax, c) in by_coord:
        coord_index[(ax, int(c * 10))].append(c)
    for sk, mem in by_site.items():
        for axis in ('row', 'col'):
            perp = sk[1] if axis == 'row' else sk[0]
            cand = set()
            for dc in range(-span_c, span_c + 1):
                cand.update(coord_index.get((axis, int(perp * 10) + dc), ()))
            for c in cand:
                reached = collections.defaultdict(set)   # frozenset of the site's moves -> the runs' moves
                for s in by_coord[(axis, c)]:
                    who = []
                    for k, i, m in mem:
                        sx, sy = m.site
                        p_, al = (sy, sx) if axis == 'row' else (sx, sy)
                        R_ = reach(m)
                        if abs(p_ - c) < R_ and s[3] - R_ < al < s[4] + R_:
                            who.append((k, i))
                    if who:
                        reached[frozenset(who)].add(s[5])
                for who, runs in reached.items():
                    bi(who, runs, ('reach',))

    # exit: per layer and face, points along the face (the stacking pitch, or the exit tolerance box)
    by_face = collections.defaultdict(list)
    for k, i, m in moves:
        ax = 1 if m.direction in ('left', 'right') else 0
        by_face[(m.layer, m.direction) if stack else (m.direction,)].append((m.exit_pt[ax], (k, i)))
    r = max(sm._STACK_PITCH if stack_pitch is None else stack_pitch, sm._EXIT_TOL) if stack else sm._EXIT_TOL
    for fk, pts in by_face.items():
        for g in _interval_cliques([(v - r / 2, v + r / 2 - sm._TOUCH, mbr) for v, mbr in pts]):
            cliques.add(g)
            tag(g, ('exit', fk[0] if stack else None))
    return cliques, bicliques, pairs


def expand(cliques, bicliques, pairs):
    """every pair the three state, one ball's own moves left out (at most one per ball)"""
    out = set(pairs)
    for g in cliques:
        mem = sorted(g)
        for x in range(len(mem)):
            for y in range(x + 1, len(mem)):
                if mem[x][0] != mem[y][0]:
                    out.add((mem[x], mem[y]))
    for a, b in bicliques:
        for p in a:
            for q in b:
                if p[0] != q[0]:
                    out.add((p, q) if p < q else (q, p))
    return out
