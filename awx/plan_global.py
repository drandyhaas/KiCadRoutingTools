#!/usr/bin/env python3
"""#622 global plan model, increment 2: LEDGER + MENU-v2 + SOLVER.

ledger  PLAN.json BOARD.kicad_pcb   -- via arithmetic vs routed truth
verify  CONTRACT.json               -- do these asks fit the lattice?
menus   NET[,NET..]                 -- priced escape options (v2 paths)
solve   [--out-prefix P]            -- joint U1-exit x DU1-berth x ride
                                       optimization; emits P_contract
                                       .json + P_pages.json + report

The solver owns all three via markets on ONE ledger:
  vias = fanout path vias (U1 lattice)  + berth path vias (DU1 lattice)
       + end mismatches (exit layer vs ride, berth layer vs ride)
       + 2 x vertex cover of same-ride crossing pairs (one dive covers
         all a net's conflicts -- calibrated 0831: control comb model
         44 vs braid 48)
       + dive-capacity penalty (swap-region via columns; CAP_DIVES)
Crossings are chord PARITY between boundary exit points (a pair that
crosses twice is homotopic to uncrossed). Menus come from Dijkstra
over the gap lattice, so far travel is priced by the cells it
reserves, never forbidden a priori.
"""
import argparse
import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
from plan_lattice import Lattice          # noqa: E402

K28 = ('SDQ15,SDQ14,SDQ13,SDQ11,SDQ0,SDQM0,SDQ12,SDQ8,SDQ10,SDQ9,'
       'SDQM1,SRAS,SCAS,SA7,SA9,SODT0,SODT1,SA13,SWE,SCKE1,SCKE0,'
       'SA8,SA6,SA4,SA1,SDQ7,SBA1,SA0').split(',')


def nets_for_k(k):
    import subprocess
    out = subprocess.run(
        [sys.executable, os.path.join(HERE, 'coherent_nets.py'),
         str(k)], capture_output=True, text=True).stdout.strip()
    return out.split(',')
CAP_DIVES = 10      # via columns the swap region held (measured OK)
PEN_DIVE_OVER = 4
# the repo's own via/length exchange rate: the router prices a via
# at 75 cost units on the 0.1 mm grid = 7.5 mm of track. One via
# of ledger cost therefore equals VIA_MM of ride length.
VIA_MM = 7.5


# ---------------------------------------------------------------- ledger
def ledger(plan_path, board_path, emit_pages=None):
    plan = json.load(open(plan_path))
    pcb = parse_kicad_pcb(board_path)
    nm = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()}
    actual = collections.Counter(nm.get(v.net_id, '?') for v in pcb.vias)
    corrs = collections.defaultdict(list)
    for nmn, d in plan.items():
        corrs[d['corridor']].append(nmn)

    def crossings(nms):
        li = {n: plan[n]['launch_idx'] for n in nms}
        tr = {n: plan[n]['target_rank'] for n in nms}
        x = collections.defaultdict(set)
        for i, a in enumerate(nms):
            for b in nms[i + 1:]:
                if (li[a] - li[b]) * (tr[a] - tr[b]) < 0:
                    x[a].add(b)
                    x[b].add(a)
        return x

    print('ledger-OPT (model chooses rides):')
    tot_a = sum(actual.get(n, 0) for n in plan)
    tot_opt = 0
    all_pages = {}
    for ci, nms in sorted(corrs.items()):
        x = crossings(nms)
        f = {}
        for n in nms:
            d = plan[n]
            f[n] = {L: (d['tooth_layer'] != L) + (d['dest_layer'] != L)
                    for L in ('F.Cu', 'B.Cu')}
        ride = {n: plan[n]['dest_layer'] for n in nms}

        def cost():
            c = sum(f[n][ride[n]] for n in nms)
            conf = {n: {b for b in x[n] if ride[b] == ride[n]}
                    for n in nms}
            return c + 2 * greedy_cover(conf)

        cur = cost()
        improved = True
        while improved:
            improved = False
            for n in nms:
                old = ride[n]
                ride[n] = 'B.Cu' if old == 'F.Cu' else 'F.Cu'
                c2 = cost()
                if c2 < cur:
                    cur, improved = c2, True
                else:
                    ride[n] = old
        intr = sum((plan[n]['tooth_layer'] == 'B.Cu')
                   + (plan[n]['dest_layer'] == 'B.Cu') for n in nms)
        print(f'  corridor {ci}: intrinsic {intr} + order {cur} '
              f'= {intr + cur}')
        tot_opt += intr + cur
        # pages in THIS plan's frame: rides two-colored on the real
        # inversion graph; the cover members ride as swimmers
        conf = {n: {b for b in x[n] if ride[b] == ride[n]}
                for n in nms}
        divers = set()
        greedy_cover(conf, divers)
        for n in nms:
            all_pages[n] = None if n in divers else ride[n]
    all_divers = {n for n, v in all_pages.items() if v is None}
    all_rides = {}
    for ci, nms in corrs.items():
        for n in nms:
            all_rides[n] = all_pages.get(n) or plan[n]['dest_layer']
    extra = inter_corridor_cover(plan, all_rides, all_divers)
    if extra:
        print(f'  inter-corridor dives: +{2 * len(extra)} '
              f'({sorted(extra)})')
        tot_opt += 2 * len(extra)
        for n in extra:
            all_pages[n] = None
    print(f'  MODEL-OPT TOTAL: {tot_opt}   (actual {tot_a})')
    if emit_pages:
        json.dump(all_pages, open(emit_pages, 'w'), indent=1,
                  sort_keys=True)
        n_sw = sum(1 for v in all_pages.values() if v is None)
        print(f'  wrote {emit_pages} ({len(all_pages) - n_sw} paged, '
              f'{n_sw} swimmers)')


def opt_rides(plan):
    """The ledger's ride optimizer as a callable: per-corridor local
    search over ride layers (field = end mismatches, order = 2 x
    greedy vertex cover of same-ride crossings). Returns (rides,
    divers, per_net_model_vias) -- the diagnosis the improve loop
    turns into targeted pages edits."""
    corrs = collections.defaultdict(list)
    for nmn, d in plan.items():
        corrs[d['corridor']].append(nmn)
    rides, divers, model = {}, set(), {}
    for ci, nms in sorted(corrs.items()):
        li = {n: plan[n]['launch_idx'] for n in nms}
        tr = {n: plan[n]['target_rank'] for n in nms}
        x = collections.defaultdict(set)
        for i, a in enumerate(nms):
            for b in nms[i + 1:]:
                if (li[a] - li[b]) * (tr[a] - tr[b]) < 0:
                    x[a].add(b)
                    x[b].add(a)
        f = {n: {L: (plan[n]['tooth_layer'] != L)
                 + (plan[n]['dest_layer'] != L)
                 for L in ('F.Cu', 'B.Cu')} for n in nms}
        ride = {n: plan[n]['dest_layer'] for n in nms}

        def cost():
            conf = {n: {b for b in x[n] if ride[b] == ride[n]}
                    for n in nms}
            return (sum(f[n][ride[n]] for n in nms)
                    + 2 * greedy_cover(conf))

        cur = cost()
        improved = True
        while improved:
            improved = False
            for n in nms:
                old = ride[n]
                ride[n] = 'B.Cu' if old == 'F.Cu' else 'F.Cu'
                c2 = cost()
                if c2 < cur:
                    cur, improved = c2, True
                else:
                    ride[n] = old
        conf = {n: {b for b in x[n] if ride[b] == ride[n]}
                for n in nms}
        dv = set()
        greedy_cover(conf, dv)
        divers |= dv
        for n in nms:
            model[n] = (f[n][ride[n]]
                        + (plan[n]['tooth_layer'] == 'B.Cu')
                        + (plan[n]['dest_layer'] == 'B.Cu')
                        + (2 if n in dv else 0))
        rides.update({n: ride[n] for n in nms})
    extra = inter_corridor_cover(plan, rides, divers)
    for n in extra:
        model[n] = model.get(n, 0) + 2
    divers |= extra
    return rides, divers, model


def inter_corridor_cover(plan, rides, divers):
    """The dives the per-corridor model cannot see: same-ride chords
    (tooth -> berth straight lines) that cross BETWEEN corridors.
    Measured on the K28 record: singleton-corridor SA0/SA4/SODT1 each
    pay a real 2-via dive under the main ribbon (band-free re-lays
    found nothing cheaper), and the per-corridor ledger priced them 0
    -- promising 36 where ~42 is the physical floor. A per-corridor
    diver's dive covers its inter-corridor crossings for free; the
    rest are covered greedily. Plans without 'berth' (older dumps)
    contribute nothing."""
    xg = collections.defaultdict(set)
    names = sorted(plan)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if plan[a].get('corridor') == plan[b].get('corridor'):
                continue
            if rides.get(a) != rides.get(b):
                continue
            pa = (plan[a].get('tooth'), plan[a].get('berth'))
            pb = (plan[b].get('tooth'), plan[b].get('berth'))
            if None in pa or None in pb:
                continue
            if seg_x(pa[0], pa[1], pb[0], pb[1]):
                xg[a].add(b)
                xg[b].add(a)
    for d_ in divers:
        for b in list(xg.get(d_, ())):
            xg[b].discard(d_)
        xg.pop(d_, None)
    xg = {n: v for n, v in xg.items() if v}
    extra = set()
    greedy_cover(xg, extra)
    return extra


def greedy_cover(conf, members=None):
    live = {n: set(v) for n, v in conf.items() if v}
    cover = 0
    while live:
        w = max(live, key=lambda n: len(live[n]))
        if not live[w]:
            break
        cover += 1
        if members is not None:
            members.add(w)
        for b in list(live[w]):
            live[b].discard(w)
        del live[w]
        live = {n: v for n, v in live.items() if v}
    return cover


# ------------------------------------------------- chords / crossings
def seg_x(p1, p2, p3, p4):
    def o(a, b, c):
        v = ((b[0] - a[0]) * (c[1] - a[1])
             - (b[1] - a[1]) * (c[0] - a[0]))
        return 0 if abs(v) < 1e-12 else (1 if v > 0 else -1)
    return (o(p1, p2, p3) * o(p1, p2, p4) < 0
            and o(p3, p4, p1) * o(p3, p4, p2) < 0)


def seg_hits_rect(p1, p2, rect):
    x0, y0, x1, y1 = rect
    if max(p1[0], p2[0]) < x0 or min(p1[0], p2[0]) > x1 \
            or max(p1[1], p2[1]) < y0 or min(p1[1], p2[1]) > y1:
        return False
    for a, b in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
                 ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
        if seg_x(p1, p2, a, b):
            return True
    return x0 < p1[0] < x1 and y0 < p1[1] < y1


def chord(u1_pt, du1_pt, rects):
    """Polyline u1->du1 detouring around array corners. Tries the
    straight chord, then every 1-corner, then every 2-corner detour
    (a far-face exit must round TWO corners; the old single-corner
    fallback silently returned the straight-through-the-array chord
    and under-priced wraps by ~2x). Shortest clearing polyline wins."""
    def clear(poly):
        return not any(seg_hits_rect(poly[i], poly[i + 1], r)
                       for i in range(len(poly) - 1) for r in rects)

    def length(poly):
        return sum(abs(poly[i + 1][0] - poly[i][0])
                   + abs(poly[i + 1][1] - poly[i][1])
                   for i in range(len(poly) - 1))

    if clear([u1_pt, du1_pt]):
        return [u1_pt, du1_pt]
    m = 0.5
    corners = []
    for x0, y0, x1, y1 in rects:
        corners += [(x1 + m, y1 + m), (x1 + m, y0 - m),
                    (x0 - m, y1 + m), (x0 - m, y0 - m)]
    best = None
    for c in corners:
        p = [u1_pt, c, du1_pt]
        if clear(p) and (best is None or length(p) < length(best)):
            best = p
    if best is None:
        for c1 in corners:
            for c2 in corners:
                if c1 == c2:
                    continue
                p = [u1_pt, c1, c2, du1_pt]
                if clear(p) and (best is None
                                 or length(p) < length(best)):
                    best = p
    return best if best is not None else [u1_pt, du1_pt]


def poly_cross(pa, pb):
    n = 0
    for i in range(len(pa) - 1):
        for j in range(len(pb) - 1):
            if seg_x(pa[i], pa[i + 1], pb[j], pb[j + 1]):
                n += 1
    return n % 2 == 1


# ----------------------------------------------------------- verify
def verify(contract_path, base_path, ref='U1'):
    pcb = parse_kicad_pcb(base_path)
    lat = Lattice(pcb, ref)
    ask = json.load(open(contract_path))
    lat.reserve_board(pcb, skip_nets=set(ask))
    print(f'{len(lat.used)} cells pre-reserved by foreign copper')
    S2F = {'up': 'up', 'down': 'down', 'left': 'left', 'right': 'right'}
    items = sorted(ask.items(), key=lambda kv: -abs(
        kv[1]['coord'] - (lat.hl[len(lat.hl) // 2]
                          if kv[1]['side'] in ('left', 'right')
                          else lat.vl[len(lat.vl) // 2])))
    ok = 0
    for n, d in items:
        face = S2F[d['side']]
        dist, par = lat.dijkstra(n)
        best = None
        for node in lat.rim_nodes(face):
            if abs(lat.node_coord(face, node) - d['coord']) > 0.36:
                continue
            nd = (node[0], node[1], d['layer'])
            if nd in dist and (best is None or dist[nd] < dist[best]):
                best = nd
        if best is not None:
            lat.commit(n, par, best)
            v, c = dist[best] // lat.VIA_W, dist[best] % lat.VIA_W
            print(f'  OK      {n:7s} {face}/{d["layer"][0]} '
                  f'@{d["coord"]:.2f}  {v} via, {c} cells')
            ok += 1
        else:
            print(f'  BLOCKED {n:7s} {face}/{d["layer"][0]} '
                  f'@{d["coord"]:.2f}')
    print(f'\nverify v2: {ok}/{len(items)} asks fit')


def menus_cmd(nets, base_path, ref='U1'):
    pcb = parse_kicad_pcb(base_path)
    lat = Lattice(pcb, ref)
    lat.reserve_board(pcb, skip_nets=set(nets))
    for n in nets:
        r, c = lat.ball[n]
        m, _ = lat.menu(n)
        print(f'\n{n} ball (row {r}, col {c}): '
              f'{len(m)} priced exits; cheapest per face:')
        for face in ('right', 'down', 'left', 'up'):
            fm = [e for e in m if e['face'] == face]
            if not fm:
                print(f'  {face:5s}: none reachable')
                continue
            e = min(fm, key=lambda e: (e['vias'], e['cells']))
            print(f'  {face:5s}: {e["layer"][0]} @{e["coord"]:.2f}  '
                  f'{e["vias"]} via, {e["cells"]} cells')


# ------------------------------------------------------------- solve
def berth_ends(board_path, nets, du):
    """Actual berth stub free ends near the dest array: per net the
    degree-1 endpoint of its copper inside the dest window, with the
    layer it ends on. The braid+berth's OWN choice, read off copper."""
    p = parse_kicad_pcb(board_path)
    byname = {n.name.split('/')[-1]: i for i, n in p.nets.items()}
    x0, x1 = du.vl[0] - 2.5, du.vl[-1] + 2.5
    y0, y1 = du.hl[0] - 2.5, du.hl[-1] + 2.5
    out = {}
    for nm in nets:
        nid = byname.get(nm)
        segs = [s for s in p.segments if s.net_id == nid
                and x0 < (s.start_x + s.end_x) / 2 < x1
                and y0 < (s.start_y + s.end_y) / 2 < y1]
        deg = {}
        for s in segs:
            for pt in ((round(s.start_x, 3), round(s.start_y, 3)),
                       (round(s.end_x, 3), round(s.end_y, 3))):
                deg[pt] = deg.get(pt, 0) + 1
        free = [pt for pt, c in deg.items() if c == 1]
        if not free:
            continue
        # the end FARTHEST from the array centre is the stub end
        cx = (du.vl[0] + du.vl[-1]) / 2
        cy = (du.hl[0] + du.hl[-1]) / 2
        pt = max(free, key=lambda q: (q[0] - cx) ** 2
                 + (q[1] - cy) ** 2)
        lay = next(s.layer for s in segs
                   if pt in ((round(s.start_x, 3), round(s.start_y, 3)),
                             (round(s.end_x, 3), round(s.end_y, 3))))
        d = {'up': abs(pt[1] - du.hl[0]), 'down': abs(pt[1] - du.hl[-1]),
             'left': abs(pt[0] - du.vl[0]),
             'right': abs(pt[0] - du.vl[-1])}
        face = min(d, key=d.get)
        coord = pt[0] if face in ('up', 'down') else pt[1]
        out[nm] = {'face': face, 'coord': round(coord, 3),
                   'layer': lay}
    return out


def solve(base_path, out_prefix, nets=None, src='U1', dst='DU1',
          sweeps=8, shortlist=24, pin_du=None,
          anchor=None, anchor_w=0.6, len_w=1.0 / VIA_MM,
          frame='chord'):
    FACES = ('up', 'down', 'left', 'right')
    nets = list(nets or K28)
    pcb = parse_kicad_pcb(base_path)
    u1 = Lattice(pcb, src)
    du = Lattice(pcb, dst)
    # a net without a ball in BOTH arrays is outside this two-array
    # solve (K51's SZQ ends at a calibration resistor, not the DRAM);
    # it keeps its board pose and the chain routes it as ever
    scoped = [n for n in nets if n in u1.ball and n in du.ball]
    if len(scoped) != len(nets):
        print(f'  out of scope (no ball in {src} and {dst}): '
              f'{sorted(set(nets) - set(scoped))}')
    nets = scoped
    u1.reserve_board(pcb, skip_nets=set(nets))
    du.reserve_board(pcb, skip_nets=set(nets))
    pinned_du = (berth_ends(pin_du, nets, du) if pin_du else {})
    anchor_pts = {}
    if anchor:
        _ac = json.load(open(anchor))
        for n, b in _ac.items():
            nodes = u1.rim_nodes(b['side'])
            node = min(nodes, key=lambda nd: abs(
                u1.node_coord(b['side'], nd) - b['coord']))
            anchor_pts[n] = u1.exit_pt(b['side'], node)
    if pin_du:
        print(f'pinned {len(pinned_du)} berths from {pin_du}')
    # Rides may not hug the array boundary: the strip just outside it
    # belongs to the exit stubs (the LINES outside-margin disease --
    # a lane riding it collides with every tooth). Chords therefore
    # clear a MARGIN-padded rect, and each chord starts at its spur
    # pushed past that margin, so an exit near a corner pays its real
    # wrap instead of skimming the boundary for free.
    MARGIN = 0.8
    rects = [(u1.vl[0] - MARGIN, u1.hl[0] - MARGIN,
              u1.vl[-1] + MARGIN, u1.hl[-1] + MARGIN),
             (du.vl[0] - MARGIN, du.hl[0] - MARGIN,
              du.vl[-1] + MARGIN, du.hl[-1] + MARGIN)]
    SPUR = MARGIN + 0.1

    def _push(lat, face, node):
        x, y = lat.exit_pt(face, node)
        if face == 'up':
            return (x, y - SPUR)
        if face == 'down':
            return (x, y + SPUR)
        if face == 'left':
            return (x - SPUR, y)
        return (x + SPUR, y)
    st = {}   # net -> dict(u1=..., du=..., ride=...)

    def pick_cheapest(lat, n, faces):
        m, par = lat.menu(n, faces)
        if not m:
            return None, None
        e = min(m, key=lambda e: (e['vias'], e['cells']))
        return e, par

    # ---- init: most-constrained first by ACTUAL U1 menu size,
    # recomputed as commitments accumulate (geometric depth seated
    # the walled flank corner last and starved SA8/SA9/SDQ7)
    pending = set(nets) & set(u1.ball)

    def du_pin_entry(n):
        b = pinned_du[n]
        nodes = du.rim_nodes(b['face'])
        node = min(nodes, key=lambda nd: abs(
            du.node_coord(b['face'], nd) - b['coord']))
        return {'face': b['face'], 'node': node, 'layer': b['layer'],
                'coord': b['coord'], 'vias': 0, 'cells': 0,
                'nd': None, 'pinned': True}

    def seat(n):
        eu, pu = pick_cheapest(u1, n, ('up', 'down', 'left',
                                       'right'))
        if eu is None:
            return False
        if n in pinned_du:
            ed, pd = du_pin_entry(n), None
        else:
            ed, pd = pick_cheapest(du, n, FACES)
            if ed is None:
                return False
        u1.commit(n, pu, eu['nd'])
        if pd is not None:
            du.commit(n, pd, ed['nd'])
        st[n] = {'u1': eu, 'du': ed, 'ride': ed['layer']}
        return True

    def seat_evict(n, depth=3):
        """Seat n; on refusal, probe which seated net's copper is the
        wall (ignore one at a time), evict it, seat n, re-seat the
        evictee -- the fanout's own rip/negotiate move, lattice-grade."""
        if seat(n):
            return True
        if depth == 0:
            return False
        for m in sorted(st, key=lambda q: abs(
                u1.ball[q][0] - u1.ball[n][0])
                + abs(u1.ball[q][1] - u1.ball[n][1])):
            u1.ignore = {m}
            mm, _ = u1.menu(n)
            u1.ignore = set()
            if not mm:
                continue
            u1.rip(m)
            du.rip(m)
            del st[m]
            if seat(n) and seat_evict(m, depth - 1):
                print(f'  seated {n} by evicting/re-seating {m}')
                return True
            # roll forward best-effort: n stays if it seated
            if m not in st:
                seat_evict(m, depth - 1)
            return n in st
        return False

    while pending:
        sizes = {}
        for n in pending:
            m, _ = u1.menu(n)
            sizes[n] = len(m)
        n = min(pending, key=lambda q: (sizes[q], q))
        pending.discard(n)
        if sizes[n] == 0 or not seat(n):
            if not seat_evict(n):
                print(f'  PENDING {n} (no exits yet)')
    missing = sorted(set(nets) - set(st))
    PEN_MISSING = 50
    # nets the (deliberately conservative) lattice cannot seat fall
    # back to the pose their EXISTING stub on the base board already
    # holds -- achievability proven by the copper that exists, read
    # off the board itself (berth_ends pointed at the SOURCE lattice;
    # no bench-specific file). U1 pinned; berth + ride stay free.
    if missing:
        bench = {n: {'side': b['face'], 'coord': b['coord'],
                     'layer': b['layer']}
                 for n, b in berth_ends(base_path, missing,
                                        u1).items()}
        for n in list(missing):
            if n not in bench or n not in du.ball:
                continue
            b = bench[n]
            face = b['side']
            nodes = u1.rim_nodes(face)
            node = min(nodes, key=lambda nd: abs(
                u1.node_coord(face, nd) - b['coord']))
            ed, pd = pick_cheapest(du, n, FACES)
            if ed is None:
                continue
            du.commit(n, pd, ed['nd'])
            st[n] = {'u1': {'face': face, 'node': node,
                            'layer': b['layer'], 'coord': b['coord'],
                            'vias': 1 if b['layer'] == 'B.Cu' else 0,
                            'cells': 0, 'nd': None, 'pinned': True},
                     'du': ed, 'ride': ed['layer']}
            print(f'  {n}: PINNED to bench pose '
                  f'{face}/{b["layer"][0]}@{b["coord"]:.2f}')
            missing.remove(n)

    def pts(n):
        return (_push(u1, st[n]['u1']['face'], st[n]['u1']['node']),
                _push(du, st[n]['du']['face'], st[n]['du']['node']))

    def conflicts(fr):
        """The crossing structure the cover prices, in one of two
        frames. 'chord': 2-D chord-polyline parity between EVERY pair
        -- counts crossings the braid's corridor projection dissolves
        (a side exit joining at the right rank crosses nobody), which
        is why chord predictions run ~1.6x the delivered board at
        K32/K47. 'braid': the braid's own unit (corridor_groups' rule
        -- every net berthing on one face is one corridor): in-group
        conflict = the pair's order INVERTS between launch and berth
        on the face's transverse axis (what the weave must actually
        undo), cross-group conflict = the chords cross (a REAL forced
        dive -- K28's singleton-corridor SA0/SA4 pay 2 vias under the
        main ribbon and the per-corridor ledger priced them 0)."""
        chords = {n: chord(*pts(n), rects) for n in st}
        x = collections.defaultdict(set)
        ns = sorted(st)
        axis = {'up': 0, 'down': 0, 'left': 1, 'right': 1}
        face = {n: st[n]['du']['face'] for n in ns}
        for i, a in enumerate(ns):
            for b in ns[i + 1:]:
                if fr == 'braid' and face[a] == face[b]:
                    ax = axis.get(face[a], 0)
                    la, lb = pts(a)[0][ax], pts(b)[0][ax]
                    ca, cb = st[a]['du']['coord'], st[b]['du']['coord']
                    hit = (la != lb and ca != cb
                           and (la < lb) != (ca < cb))
                else:
                    hit = poly_cross(chords[a], chords[b])
                if hit:
                    x[a].add(b)
                    x[b].add(a)
        return chords, x

    def total(fr=None):
        chords, x = conflicts(fr or frame)
        ns = sorted(st)
        c = 0
        for n in ns:
            s = st[n]
            c += s['u1']['vias'] + s['du']['vias']
            c += (s['u1']['layer'] != s['ride'])
            c += (s['du']['layer'] != s['ride'])
        conf = {n: {b for b in x[n] if st[b]['ride'] == st[n]['ride']}
                for n in ns}
        cov = greedy_cover(conf)
        c += 2 * cov + PEN_DIVE_OVER * max(0, cov - CAP_DIVES)
        c += PEN_MISSING * len(set(nets) - set(st))
        for n in st:
            pl = chords[n]
            c += len_w * sum(
                abs(pl[i + 1][0] - pl[i][0])
                + abs(pl[i + 1][1] - pl[i][1])
                for i in range(len(pl) - 1))
        if anchor_pts:
            for n in st:
                ap_ = anchor_pts.get(n)
                if ap_ is not None:
                    q = u1.exit_pt(st[n]['u1']['face'],
                                   st[n]['u1']['node'])
                    c += anchor_w * (abs(q[0] - ap_[0])
                                     + abs(q[1] - ap_[1]))
        return c, cov, x

    cur, cov, _ = total()
    print(f'init: predicted {cur} vias (cover {cov})')

    for sw in range(sweeps):
        improved = False
        # retry pending nets first -- other nets' moves may have
        # opened their corner
        for n in sorted(set(nets) - set(st)):
            if n in u1.ball and seat(n):
                print(f'  seated {n} on retry')
                improved = True
                cur, _, _ = total()
        # ride flips (free moves first)
        for n in sorted(st):
            old = st[n]['ride']
            st[n]['ride'] = 'B.Cu' if old == 'F.Cu' else 'F.Cu'
            c2, _, _ = total()
            if c2 < cur:
                cur, improved = c2, True
            else:
                st[n]['ride'] = old
        # exit re-picks, worst contributor first
        for lat, key, faces in ((u1, 'u1', ('up', 'down', 'left',
                                            'right')),
                                (du, 'du', FACES)):
            for n in sorted(st):
                keep = st[n][key]
                if keep.get('pinned'):
                    continue
                saved = lat.paths.get(n)
                saved_exit = keep['nd']
                lat.rip(n)
                m, par = lat.menu(n, faces)
                if not m:
                    # nothing reachable with n ripped: put the old
                    # path back verbatim and move on
                    if saved is not None:
                        lat.restore(n, saved[0], saved[1], saved_exit)
                    st[n][key] = keep
                    continue
                m.sort(key=lambda e: (e['vias'], e['cells']))
                best = (cur, None)
                for e in m[:shortlist]:
                    st[n][key] = e
                    c2, _, _ = total()
                    if c2 < best[0]:
                        best = (c2, e)
                e = best[1] if best[1] is not None else next(
                    (e for e in m if e['nd'] == keep['nd']), keep)
                st[n][key] = e
                nd = e['nd']
                if nd not in par:      # keep unreachable: re-pick best
                    e2 = min(m, key=lambda q: (q['vias'], q['cells']))
                    st[n][key] = e2
                    nd = e2['nd']
                lat.commit(n, par, nd)
                c2, _, _ = total()
                if c2 < cur:
                    cur, improved = c2, True
        print(f'sweep {sw}: predicted {cur}')
        if not improved:
            break

    cur, cov, x = total()
    # who dives: recompute the cover MEMBERS -- they are emitted as
    # swimmers (pages=None); forcing every net onto a page left the
    # braid no weave relief and refused 10 lanes (measured)
    conf = {n: {b for b in x[n] if st[b]['ride'] == st[n]['ride']}
            for n in st}
    divers = set()
    greedy_cover(conf, divers)
    print(f'\nSOLVED: predicted {cur} vias (cover {cov}: '
          f'{sorted(divers)}, cap {CAP_DIVES})')
    # disclose the OTHER frame's prediction on the same final state,
    # so every run carries the calibration pair
    other = 'braid' if frame == 'chord' else 'chord'
    oc, ocov, _ox = total(other)
    print(f'  [{other}-frame prediction on this state: {oc:.1f} vias '
          f'(cover {ocov})]')
    contract = {}
    pages = {}
    dua = {}
    for n in sorted(st):
        s = st[n]
        contract[n] = {'side': s['u1']['face'],
                       'coord': round(s['u1']['coord'], 3),
                       'layer': s['u1']['layer']}
        pages[n] = None if n in divers else s['ride']
        dua[n] = {'side': s['du']['face'],
                  'coord': round(s['du']['coord'], 3),
                  'layer': s['du']['layer']}
        print(f"  {n:7s} u1 {s['u1']['face']:5s}/"
              f"{s['u1']['layer'][0]}@{s['u1']['coord']:.2f}"
              f" ({s['u1']['vias']}v)  du {s['du']['face']:5s}/"
              f"{s['du']['layer'][0]}@{s['du']['coord']:.2f}"
              f" ({s['du']['vias']}v)  ride {s['ride'][0]}"
              f"  xng {len(x[n])}")
    json.dump(contract, open(out_prefix + '_contract.json', 'w'),
              indent=1, sort_keys=True)
    json.dump(pages, open(out_prefix + '_pages.json', 'w'), indent=1,
              sort_keys=True)
    json.dump(dua, open(out_prefix + '_du1_ask.json', 'w'), indent=1,
              sort_keys=True)
    print(f'wrote {out_prefix}_contract.json / _pages.json / '
          f'_du1_ask.json')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['ledger', 'verify', 'menus',
                                    'solve'])
    ap.add_argument('args', nargs='*')
    ap.add_argument('--base', default='fb_t2q_base.kicad_pcb')
    ap.add_argument('--out-prefix', default='solved_k28')
    ap.add_argument('--emit-pages', default=None)
    ap.add_argument('--k', type=int, default=28)
    ap.add_argument('--pin-du', default=None,
                    help='board whose DU1 berth stubs pin the dest '
                         'side (the braid+berth own choice)')
    ap.add_argument('--len-w', type=float,
                    default=1.0 / VIA_MM)
    ap.add_argument('--anchor', default=None,
                    help='contract json: charge anchor-w per mm of '
                         'U1 exit movement away from it (damping)')
    ap.add_argument('--anchor-w', type=float, default=0.6)
    ap.add_argument('--frame', choices=('chord', 'braid'),
                    default='chord',
                    help='crossing frame the solver optimizes: chord '
                         '(2-D parity, every pair) or braid (the '
                         "corridor's own 1-D order in-group, chords "
                         'across groups); both predictions printed')
    a = ap.parse_args()
    if a.cmd == 'ledger':
        ledger(a.args[0], a.args[1], emit_pages=a.emit_pages)
    elif a.cmd == 'verify':
        verify(a.args[0], a.base)
    elif a.cmd == 'menus':
        menus_cmd(a.args[0].split(','), a.base)
    else:
        solve(a.base, a.out_prefix, nets=nets_for_k(a.k),
              pin_du=a.pin_du,
              anchor=a.anchor, anchor_w=a.anchor_w, len_w=a.len_w,
              frame=a.frame)
