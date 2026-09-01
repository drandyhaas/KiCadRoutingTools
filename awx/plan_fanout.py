#!/usr/bin/env python3
"""Drive generate_bga_fanout with the HUMAN-derived plan (#622
increment 2): per-pad LAYER + SIDE + SLOT hints extracted from the
human original's own U1 fanout, engine re-fans from bare balls, and
the grade measures obedience per net: layer, side, slot hit, and the
per-(side,layer) exit ORDER vs the human's (inversions).

usage: plan_fanout.py K --out OUT.kicad_pcb [--no-slots] [--no-hints]
       [--method underpad] [--base fb_t2q_base.kicad_pcb] [--ref U1]
"""
import argparse
import math
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402
import bga_fanout as bf  # noqa: E402

HUMAN = os.path.expanduser('~/Downloads/bus/00_human_original.kicad_pcb')
SLOT_W = 0.40

ap = argparse.ArgumentParser()
ap.add_argument('K', type=int)
ap.add_argument('--out', required=True)
ap.add_argument('--base', default='fb_t2q_base.kicad_pcb')
ap.add_argument('--human', default=HUMAN)
ap.add_argument('--ref', default='U1')
ap.add_argument('--method', default='underpad')
ap.add_argument('--no-slots', action='store_true')
ap.add_argument('--no-hints', action='store_true')
ap.add_argument('--no-rescue', action='store_true',
                help='skip the A* rescue pass (engine output only)')
ap.add_argument('--contract-json', default=None,
                help='load the contract (net -> side/coord/layer) '
                'from a PLAN file instead of extracting the human''s')
ap.add_argument('--dump-contract', default=None,
                help='write the extracted contract as JSON and exit')
a = ap.parse_args()

names = subprocess.run(
    [sys.executable, os.path.join(HERE, 'coherent_nets.py'), str(a.K)],
    capture_output=True, text=True).stdout.strip().split(',')[:a.K]

base = parse_kicad_pcb(a.base)
bby = {n.name.split('/')[-1]: (i, n) for i, n in base.nets.items()}
id2short = {i: n.name.split('/')[-1] for i, n in base.nets.items()}
fp = base.footprints[a.ref]
xs = [p.global_x for p in fp.pads]
ys = [p.global_y for p in fp.pads]
BB = (min(xs), min(ys), max(xs), max(ys))
W = (BB[0] - 0.25, BB[1] - 0.25, BB[2] + 0.25, BB[3] + 0.25)
RW = (BB[0] - 1.6, BB[1] - 1.6, BB[2] + 1.6, BB[3] + 1.6)


def walk_cross(pcb, nm):
    """(layer, crossing point) where nm's copper, walked from its U1
    ball, first crosses the window boundary; None if never."""
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    if nm not in byname:
        return None
    nid = byname[nm][0]
    fp2 = pcb.footprints[a.ref]
    ball = next((p for p in fp2.pads if p.net_id == nid), None)
    if ball is None:
        return None
    b = (ball.global_x, ball.global_y)
    segs = [s for s in pcb.segments if s.net_id == nid]
    vias = [v for v in pcb.vias if v.net_id == nid]
    adj = {}
    for s in segs:
        p = (round(s.start_x, 3), round(s.start_y, 3))
        q = (round(s.end_x, 3), round(s.end_y, 3))
        adj.setdefault(p, []).append((s, q))
        adj.setdefault(q, []).append((s, p))
    start = min(adj, key=lambda p: math.hypot(p[0] - b[0], p[1] - b[1]),
                default=None)
    if start is None or math.hypot(start[0] - b[0],
                                   start[1] - b[1]) > 0.6:
        return None
    memb = {}
    for v in vias:
        r = v.size / 2 + 0.05
        for p in adj:
            if math.hypot(p[0] - v.x, p[1] - v.y) <= r:
                memb.setdefault(p, []).append(v)
    seen, seenseg, queue = {start}, set(), [start]
    while queue:
        p = queue.pop(0)
        for v in memb.get(p, ()):
            for q in adj:
                if q not in seen and math.hypot(
                        q[0] - v.x, q[1] - v.y) <= v.size / 2 + 0.05:
                    seen.add(q)
                    queue.append(q)
        for s, q in adj.get(p, ()):
            if id(s) in seenseg:
                continue
            seenseg.add(id(s))
            inside = (W[0] <= q[0] <= W[2] and W[1] <= q[1] <= W[3])
            if inside:
                if q not in seen:
                    seen.add(q)
                    queue.append(q)
            else:
                t_best = None
                for lo, hi, ax in ((W[0], W[2], 0), (W[1], W[3], 1)):
                    for edge in (lo, hi):
                        dq = q[ax] - p[ax]
                        if abs(dq) < 1e-12:
                            continue
                        tt = (edge - p[ax]) / dq
                        if 0.0 <= tt <= 1.0:
                            if t_best is None or tt < t_best:
                                t_best = tt
                if t_best is None:
                    return (s.layer, q)
                pt = (p[0] + t_best * (q[0] - p[0]),
                      p[1] + t_best * (q[1] - p[1]))
                return (s.layer, pt)
    return None


def side_coord(pt):
    d = {'up': abs(pt[1] - W[1]), 'right': abs(pt[0] - W[2]),
         'down': abs(pt[1] - W[3]), 'left': abs(pt[0] - W[0])}
    side = min(d, key=d.get)
    coord = pt[1] if side in ('left', 'right') else pt[0]
    return side, coord


# ---- the contract: from --contract-json (a plan's), else the human
if a.contract_json:
    import json
    raw = json.load(open(a.contract_json))
    contract = {nm: {'layer': d['layer'], 'side': d['side'],
                     'coord': float(d['coord'])}
                for nm, d in raw.items() if nm in bby}
    LABEL = os.path.basename(a.contract_json)
else:
    hum = parse_kicad_pcb(a.human)
    contract = {}
    for nm in names:
        c = walk_cross(hum, nm)
        if c is None:
            print(f'{nm}: no HUMAN crossing -- skipped')
            continue
        side, coord = side_coord(c[1])
        contract[nm] = {'layer': c[0], 'side': side, 'coord': coord}
    LABEL = 'HUMAN'

# slots per (side, layer): the human's own F and B stubs interleave
# in the same span (measured: right face, deep-B y 59.2-61.9 inside
# DQ-F y 60.4-65.5), so cross-layer overlap is physical and allowed;
# order is enforced WITHIN a layer on a side.
groups = {}
for nm, c in contract.items():
    groups.setdefault((c['side'], c['layer']), []).append(nm)
for (side, lay), members in sorted(groups.items()):
    members.sort(key=lambda n: contract[n]['coord'])
    pos = [contract[n]['coord'] for n in members]
    for i in range(1, len(pos)):
        pos[i] = max(pos[i], pos[i - 1] + SLOT_W)
    for i in range(len(pos) - 2, -1, -1):
        pos[i] = min(pos[i], pos[i + 1] - SLOT_W)
    for n, c0 in zip(members, pos):
        contract[n]['slot'] = (c0 - SLOT_W / 2, c0 + SLOT_W / 2)
        contract[n]['rank'] = members.index(n)
print(f'contract: {len(contract)} nets from {LABEL}; groups: '
      + ', '.join(f'{s}/{l[0]}:{len(m)}'
                  for (s, l), m in sorted(groups.items())))
if a.dump_contract:
    import json
    with open(a.dump_contract, 'w') as f:
        json.dump({nm: {'side': c['side'], 'coord': c['coord'],
                        'layer': c['layer'], 'rank': c.get('rank'),
                        'slot': list(c.get('slot', ()))}
                   for nm, c in contract.items()}, f, indent=1,
                  sort_keys=True)
    print(f'wrote contract: {a.dump_contract}')
    sys.exit(0)

# ---- hints keyed by ball
dir_hints, layer_hints, slot_hints = {}, {}, {}
for nm, c in contract.items():
    nid = bby[nm][0]
    ball = next(p for p in fp.pads if p.net_id == nid)
    k = (round(ball.global_x, 3), round(ball.global_y, 3))
    dir_hints[k] = c['side']
    layer_hints[k] = c['layer']
    slot_hints[k] = (c['side'], c['slot'][0], c['slot'][1])

# ---- rip the comb, re-fan
strip_set = set()
for s in base.segments:
    if RW[0] <= s.start_x <= RW[2] and RW[1] <= s.start_y <= RW[3]:
        strip_set.add(id2short[s.net_id])
# only rip what we will re-lay: a net in the window but out of the
# contract (K51's SZQ -- no ball in the dest array) keeps its pose,
# else it ships with ZERO copper and the chain has no stub to braid
strip_set &= set(contract)


def in_rip(x, y):
    return RW[0] <= x <= RW[2] and RW[1] <= y <= RW[3]


def strip_region(txt, token, nid, full_name, collect=None):
    out, i = [], 0
    while True:
        j = txt.find('(' + token, i)
        if j < 0:
            out.append(txt[i:])
            break
        k, depth = j, 0
        while True:
            ch = txt[k]
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        block = txt[j:k + 1]
        m = re.search(r'\(net (\d+)\)', block)
        m2 = re.search(r'\(net "([^"]+)"\)', block)
        hit = (m and int(m.group(1)) == nid) or \
              (m2 and m2.group(1) == full_name)
        pts = re.findall(r'\((?:start|end|at) ([-\d.]+) ([-\d.]+)',
                         block)
        if hit and pts and all(in_rip(float(x), float(y))
                               for x, y in pts):
            if collect is not None:
                collect.append(block)
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


txt = open(a.base).read()
ripped_blocks = {}
for nm in sorted(strip_set):
    nid, net = bby[nm]
    blocks = ripped_blocks.setdefault(nm, [])
    for token in ('segment', 'via'):
        txt = strip_region(txt, token, nid, net.name, collect=blocks)
stripped = a.out + '.bare.tmp'
open(stripped, 'w').write(txt)
pcb = parse_kicad_pcb(stripped)

kw = {}
if not a.no_hints:
    kw = dict(escape_dir_hints=dir_hints,
              escape_layer_hints=layer_hints)
    if not a.no_slots:
        kw['escape_slot_hints'] = slot_hints
tracks, vias_add, vias_rm, failed = bf.generate_bga_fanout(
    pcb.footprints[a.ref], pcb, net_filter=list(contract),
    layers=['F.Cu', 'B.Cu'], track_width=0.1, clearance=0.1,
    via_size=0.45, via_drill=0.25, exit_margin=0.5,
    escape_method=a.method, plane_drop='off', **kw)

if tracks:
    add_tracks_and_vias_to_pcb(
        stripped, a.out, tracks, vias_add, vias_rm,
        net_id_to_name={i: n.name for i, n in pcb.nets.items()})
else:
    shutil.copy(stripped, a.out)
os.remove(stripped)
pro = os.path.splitext(a.base)[0] + '.kicad_pro'
if os.path.exists(pro):
    shutil.copy(pro, os.path.splitext(a.out)[0] + '.kicad_pro')
print(f'\nwrote {a.out}; engine failed: {failed}')

# ---- grade
def grade(board_path, tag):
    ours = parse_kicad_pcb(board_path)
    n_lay = n_side = n_slot = n_none = 0
    ach = {}
    for nm in sorted(contract):
        c0 = contract[nm]
        c = walk_cross(ours, nm)
        if c is None:
            print(f'  {nm}: NO CROSSING')
            n_none += 1
            continue
        side, coord = side_coord(c[1])
        ach[nm] = {'layer': c[0], 'side': side, 'coord': coord}
        ok_l = c[0] == c0['layer']
        ok_s = side == c0['side']
        ok_sl = (ok_s and 'slot' in c0
                 and c0['slot'][0] - 1e-6 <= coord
                 <= c0['slot'][1] + 1e-6)
        n_lay += ok_l
        n_side += ok_s
        n_slot += ok_sl
        if not (ok_l and ok_s and ok_sl):
            want = (f"{c0['side']}/{c0['layer'][0]} "
                    f"[{c0['slot'][0]:.2f},{c0['slot'][1]:.2f}]"
                    if 'slot' in c0 else f"{c0['side']}/{c0['layer'][0]}")
            print(f'  {nm}: got {side}/{c[0][0]} @{coord:.2f}, '
                  f'want {want}')
    inv_tot = 0
    for (side, lay), members in sorted(groups.items()):
        ms = [n for n in members if n in ach
              and ach[n]['side'] == side and ach[n]['layer'] == lay]
        order = sorted(ms, key=lambda n: ach[n]['coord'])
        ranks = [contract[n]['rank'] for n in order]
        inv = sum(1 for i in range(len(ranks))
                  for j in range(i + 1, len(ranks))
                  if ranks[i] > ranks[j])
        inv_tot += inv
        if inv:
            print(f'  ORDER {side}/{lay[0]}: {inv} inversion(s) '
                  f'({" ".join(order)})')
    n = len(contract)
    print(f'\nPLAN OBEDIENCE vs {LABEL} [{tag}]: layer {n_lay}/{n}, '
          f'side {n_side}/{n}, slot {n_slot}/{n}, order inversions '
          f'{inv_tot}, {n_none} no-crossing (method {a.method})')
    return ach


def drc_pairs(board_path):
    out2 = subprocess.run(
        ['python3', os.path.join(HERE, '..', 'py_router',
                                 'check_drc.py'), board_path,
         '--clearance', '0.1', '--clearance-margin', '0.1'],
        capture_output=True, text=True).stdout
    pairs = set()
    for l in out2.splitlines():
        if '<->' in l and '/' in l:
            for half in l.split('<->'):
                nm2 = half.strip().split('/')[-1].split(' ')[0]
                if nm2 in contract:
                    pairs.add(nm2)
    return pairs


ach = grade(a.out, 'engine')

if not a.no_rescue:
    bad = set(drc_pairs(a.out))
    for nm in contract:
        c0 = contract[nm]
        c1 = ach.get(nm)
        if (c1 is None or c1['layer'] != c0['layer']
                or c1['side'] != c0['side']
                or not (c0['slot'][0] - 1e-6 <= c1['coord']
                        <= c0['slot'][1] + 1e-6)):
            bad.add(nm)
    if bad:
        # THE A* RESCUE + NEGOTIATION (#622 wes design): rip the
        # miss/collision set and re-lay each net with a banded,
        # containment-limited ride by the real router against REAL
        # copper. Custody: every rip is snapshotted and RESTORED on
        # refusal. When a single net cannot thread its now-obedient
        # neighbours (measured: SA1's fenced pocket), NEGOTIATION
        # widens the rip to the slot-neighbour group and re-lays the
        # group jointly, short rides first -- plan_r2 measured that
        # the joint form reaches what no single re-lay can. A failed
        # group attempt rolls back COMPLETELY.
        print(f'\nA* RESCUE: {len(bad)} net(s): '
              + ','.join(sorted(bad)))
        import numpy as np
        import braid as te
        from kicad_parser import Segment as _Seg, Via as _Via
        pcb2 = parse_kicad_pcb(a.out)
        cfg2 = te.cn.make_config(pcb2, 0.1, 0.1, te.VIA_SIZE,
                                 te.VIA_DRILL, grid_step=0.025)
        pad_r2 = (te.VIA_SIZE - 0.1) / 2
        rev2 = [0]
        _o2 = {}

        def obs2(nid_, L_):
            k_ = (nid_, L_, rev2[0])
            if k_ not in _o2:
                _o2[k_] = te.build_obstacles(pcb2, nid_, set(), L_)
            return _o2[k_]

        def gate_rect(c0):
            side = c0['side']
            lo, hi = c0['slot']
            if side == 'right':
                return (W[2] - 0.25, lo, W[2] + 0.35, hi)
            if side == 'left':
                return (W[0] - 0.35, lo, W[0] + 0.25, hi)
            if side == 'up':
                return (lo, W[1] - 0.35, hi, W[1] + 0.25)
            return (lo, W[3] - 0.25, hi, W[3] + 0.35)

        def mk_band2(lay, rect):
            def band(xs2, ys2, lname):
                if lname != lay:
                    return np.zeros((len(xs2), len(ys2)), dtype=bool)
                m = np.outer((xs2 >= W[0]) & (xs2 <= W[2]),
                             (ys2 >= W[1]) & (ys2 <= W[3]))
                m |= np.outer((xs2 >= rect[0]) & (xs2 <= rect[2]),
                              (ys2 >= rect[1]) & (ys2 <= rect[3]))
                return m
            return band

        def _ride_len(n):
            ball_ = next(p_ for p_ in fp.pads
                         if p_.net_id == bby[n][0])
            c_ = contract[n]
            cen_ = (c_['slot'][0] + c_['slot'][1]) / 2
            if c_['side'] in ('left', 'right'):
                tgt = (W[0] if c_['side'] == 'left' else W[2], cen_)
            else:
                tgt = (cen_, W[1] if c_['side'] == 'up' else W[3])
            return math.hypot(tgt[0] - ball_.global_x,
                              tgt[1] - ball_.global_y)

        # per-net custody: what is ON pcb2 for a net right now is
        # either its engine copper or copper this rescue laid
        new_objs = {}     # nm -> (seg objs, via objs) laid by rescue
        new_dicts = {}    # nm -> (track dicts, via dicts) for write
        used_sites = []

        def rip(nm):
            """Remove nm's current window copper. Snapshot carries
            the write dicts too, so restoring rescue-laid copper
            restores its write-list entries as well."""
            nid = bby[nm][0]
            if nm in new_objs:
                rs, rv = new_objs.pop(nm)
                kind = ('new', new_dicts.pop(nm))
            else:
                rs = [s_ for s_ in pcb2.segments
                      if s_.net_id == nid
                      and in_rip(s_.start_x, s_.start_y)
                      and in_rip(s_.end_x, s_.end_y)]
                rv = [v_ for v_ in pcb2.vias
                      if v_.net_id == nid and in_rip(v_.x, v_.y)]
                kind = ('engine', None)
            for o_ in rs:
                pcb2.segments.remove(o_)
            for o_ in rv:
                pcb2.vias.remove(o_)
            rev2[0] += 1
            return (kind, rs, rv)

        def unrip(nm, snap):
            (k, nd), rs, rv = snap
            pcb2.segments.extend(rs)
            pcb2.vias.extend(rv)
            if k == 'new':
                new_objs[nm] = (rs, rv)
                new_dicts[nm] = nd
            rev2[0] += 1

        def attempt(nm):
            """Lay nm to its slot against pcb2's current copper.
            Appends to pcb2 + custody dicts on success."""
            c0 = contract[nm]
            nid = bby[nm][0]
            ball = next(p_ for p_ in fp.pads if p_.net_id == nid)
            b = (ball.global_x, ball.global_y)
            lay = c0['layer']
            side = c0['side']
            lo, hi = c0['slot']
            cen = (lo + hi) / 2
            if side in ('left', 'right'):
                ept = ((W[0] - 0.15) if side == 'left'
                       else (W[2] + 0.15), cen)
            else:
                ept = (cen, (W[1] - 0.15) if side == 'up'
                       else (W[3] + 0.15))
            band = mk_band2(lay, gate_rect(c0))
            got = None
            if lay == 'F.Cu':
                for mg in (0.5, 1.0, 2.0, 4.0, 6.0):
                    res = te.cn.connect(pcb2, nid, b, lay, ept, lay,
                                        cfg2, band=band, margin=mg)
                    if res is not None:
                        got = ([], [], res)
                        break
            else:
                hp = 0.325
                sites = []
                for ddx, ddy in ((hp, -hp), (-hp, -hp), (hp, hp),
                                 (-hp, hp), (3 * hp, -hp),
                                 (-3 * hp, -hp), (3 * hp, hp),
                                 (-3 * hp, hp)):
                    cand = (b[0] + ddx, b[1] + ddy)
                    if any(math.hypot(cand[0] - u[0],
                                      cand[1] - u[1]) < 0.5
                           for u in used_sites):
                        continue
                    if any(obs2(nid, L_).point_violation(
                            cand, pad=pad_r2) is not None
                           for L_ in ('F.Cu', 'B.Cu')):
                        continue
                    if not obs2(nid, 'F.Cu').seg_clear(b, cand):
                        continue
                    sites.append(cand)
                for site in sites:
                    pcb2.vias.append(_Via(site[0], site[1],
                                          te.VIA_SIZE, te.VIA_DRILL,
                                          ['F.Cu', 'B.Cu'], nid))
                    rev2[0] += 1
                    res = None
                    for mg in (0.5, 1.0, 2.0, 4.0, 6.0):
                        res = te.cn.connect(pcb2, nid, site, lay, ept,
                                            lay, cfg2, band=band,
                                            margin=mg)
                        if res is not None:
                            break
                    pcb2.vias.pop()
                    rev2[0] += 1
                    if res is not None:
                        got = ([{'start': b, 'end': site,
                                 'width': 0.1, 'layer': 'F.Cu',
                                 'net_id': nid}],
                               [{'x': site[0], 'y': site[1],
                                 'size': te.VIA_SIZE,
                                 'drill': te.VIA_DRILL,
                                 'layers': ['F.Cu', 'B.Cu'],
                                 'net_id': nid}], res)
                        used_sites.append(site)
                        break
            if got is None:
                return False
            leg, vp, res = got
            segs_o, vias_o = res
            objs_s, objs_v = [], []
            tds, vds = [], []
            for t_ in leg:
                o_ = _Seg(t_['start'][0], t_['start'][1],
                          t_['end'][0], t_['end'][1], t_['width'],
                          t_['layer'], nid)
                pcb2.segments.append(o_)
                objs_s.append(o_)
                tds.append(t_)
            for v_ in vp:
                o_ = _Via(v_['x'], v_['y'], v_['size'], v_['drill'],
                          ['F.Cu', 'B.Cu'], nid)
                pcb2.vias.append(o_)
                objs_v.append(o_)
                vds.append(v_)
            pcb2.segments.extend(segs_o)
            pcb2.vias.extend(vias_o)
            objs_s.extend(segs_o)
            objs_v.extend(vias_o)
            rev2[0] += 1
            for s_ in segs_o:
                tds.append({'start': (s_.start_x, s_.start_y),
                            'end': (s_.end_x, s_.end_y),
                            'width': s_.width, 'layer': s_.layer,
                            'net_id': nid})
            for v_ in vias_o:
                vds.append({'x': v_.x, 'y': v_.y, 'size': v_.size,
                            'drill': v_.drill,
                            'layers': ['F.Cu', 'B.Cu'],
                            'net_id': nid})
            new_objs[nm] = (objs_s, objs_v)
            new_dicts[nm] = (tds, vds)
            return True

        # pass 1: individual, short rides first
        fails = []
        for nm in sorted(bad, key=_ride_len):
            snap = rip(nm)
            if attempt(nm):
                c0 = contract[nm]
                print(f'  rescue {nm}: laid to {c0["side"]}/'
                      f'{c0["layer"][0]} [{c0["slot"][0]:.2f},'
                      f'{c0["slot"][1]:.2f}]')
            else:
                unrip(nm, snap)
                fails.append(nm)
                print(f'  rescue {nm}: refused solo -- copper '
                      'restored; negotiating')

        # pass 2: NEGOTIATION -- widen the rip to the slot-neighbour
        # group and re-lay jointly; total rollback on failure
        for nm in list(fails):
            if nm not in fails:
                continue
            c0 = contract[nm]
            cen0 = (c0['slot'][0] + c0['slot'][1]) / 2
            done = False
            for radius in (1.5, 1e9):
                grp = [o for o in contract
                       if contract[o]['side'] == c0['side']
                       and contract[o]['layer'] == c0['layer']
                       and 'slot' in contract[o]
                       and abs((contract[o]['slot'][0]
                                + contract[o]['slot'][1]) / 2
                               - cen0) <= radius]
                grp = sorted(set(grp) | {nm}, key=_ride_len)
                if len(grp) < 2:
                    continue
                print(f'  negotiate {nm}: group '
                      + ','.join(grp)
                      + (f' (radius {radius})' if radius < 1e8
                         else ' (whole side/layer)'))
                snaps = [(g, rip(g)) for g in grp]
                ok = True
                laid_grp = []
                for g in grp:
                    if attempt(g):
                        laid_grp.append(g)
                    else:
                        ok = False
                        break
                if ok:
                    for g in grp:
                        if g in fails:
                            fails.remove(g)
                    print(f'  negotiate {nm}: group of {len(grp)} '
                          'laid jointly')
                    done = True
                    break
                # rollback: strip what this attempt laid, restore all
                for g in laid_grp:
                    kind_, rs_, rv_ = rip(g)
                for g, snap in reversed(snaps):
                    unrip(g, snap)
                print(f'  negotiate {nm}: group attempt failed '
                      '-- rolled back')
            if not done:
                print(f'  negotiate {nm}: EXHAUSTED -- engine '
                      'near-miss stands')

        relaid = sorted(new_dicts)
        if relaid:
            txt2 = open(a.out).read()
            for nm in relaid:
                nid, net = bby[nm]
                for token in ('segment', 'via'):
                    txt2 = strip_region(txt2, token, nid, net.name)
            tmp2 = a.out + '.rsc.tmp'
            open(tmp2, 'w').write(txt2)
            r_tracks = [d for nm in relaid
                        for d in new_dicts[nm][0]]
            r_vias = [d for nm in relaid
                      for d in new_dicts[nm][1]]
            add_tracks_and_vias_to_pcb(
                tmp2, a.out, r_tracks, r_vias, [],
                net_id_to_name={i: n.name
                                for i, n in pcb2.nets.items()})
            os.remove(tmp2)
        ach = grade(a.out, 'engine + A* rescue')
        left = drc_pairs(a.out)
        print('DRC after rescue: '
              + (('pairs: ' + ','.join(sorted(left)))
                 if left else 'clean among contract nets'))

# ---- a ripped net that NOTHING re-laid (engine refused, rescue
# exhausted -- K51's SDQ6) gets its bench copper BACK: only rip what
# you re-lay, the same rule the strip scope follows. Shipped bare, the
# net has no stub end for any downstream consumer to walk.
pcb_f = parse_kicad_pcb(a.out)
by_f = {n.name.split('/')[-1]: i for i, n in pcb_f.nets.items()}
bare = [nm for nm in sorted(ripped_blocks)
        if ripped_blocks[nm]
        and not any(s.net_id == by_f[nm] for s in pcb_f.segments)]
if bare:
    out_txt = open(a.out).read()
    j = out_txt.rfind(')')
    ins = ''.join('\t' + b + '\n' for nm in bare
                  for b in ripped_blocks[nm])
    open(a.out, 'w').write(out_txt[:j] + ins + out_txt[j:])
    print('RESTORED bench copper for net(s) nothing re-laid: '
          + ','.join(bare))
    ach = grade(a.out, 'engine + A* rescue + bench restore')
