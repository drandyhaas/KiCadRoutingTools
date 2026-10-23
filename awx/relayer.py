"""relayer.py -- a lane's via ends moved onto the layers its plan gives them (whole_route, more routing layers than two).

A VIA END (whole_solve) is an end whose stub carries a via inside its array -- a dog-bone's, a via in its pad -- and
runs its last stretch, from that via out to the array's edge, on whichever routing layer the solve chose for the lane
there. The board's stub was laid on the layer the fanout chose; this moves that stretch onto the solve's, as it lies:
the stub's segments from its end back to the via, on the stub's layer, stripped and laid again on the other -- the
same path, no engine call. A stub whose run reaches no via is left as it is, and named. A via left with its net's
copper on one layer -- a run moved onto the layer of the neck or pad it leaves (F.Cu) -- serves nothing, and goes.

  python3 relayer.py BOARD OUT NET:X,Y:FROM:TO ...      (a move: the net, its stub's end, the layer it is on, the one)

Writes OUT with the board's siblings (its project, the fanout's plan sidecar) beside it."""
import contextlib
import io
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

TOL = 1e-3                  # a stub's end, a segment's end and a via's centre: one point within this (mm)
SLACK = 0.01                # (clashes) under the rule by more than this: the fanout's own grid lays a hair under it


def run_to_via(pcb, nid, end_pt, layer, with_via=False):
    """the segments of net `nid` on `layer` from `end_pt` back along the stub to the first of its vias -- a chain
    through their shared ends, each step one segment on -- or None where the chain forks or ends short of a via
    (`with_via`: (the segments, the via), or None)"""
    segs = [s for s in pcb.segments if s.net_id == nid and s.layer == layer]
    # (a via reached anywhere in its copper: a via in a pad is clamped off the stub's line -- the zynq's U5, 25 um)
    vias = [v for v in pcb.vias if v.net_id == nid]
    near = lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]) < TOL
    out, cur, used = [], (float(end_pt[0]), float(end_pt[1])), set()
    for _ in range(len(segs) + 1):
        at = [v for v in vias if math.hypot(cur[0] - v.x, cur[1] - v.y) < v.size / 2]
        if at:
            return ((out, at[0]) if out else None) if with_via else (out or None)
        nxt = [s for s in segs if id(s) not in used
               and (near((s.start_x, s.start_y), cur) or near((s.end_x, s.end_y), cur))]
        if len(nxt) != 1:
            return None
        s = nxt[0]
        used.add(id(s))
        out.append(s)
        cur = (s.end_x, s.end_y) if near((s.start_x, s.start_y), cur) else (s.start_x, s.start_y)
    return None


def clashes(pcb, runs, layers, clear):
    """what a via end's run would stand beside on another layer. `runs` {key: (its nets' ids, its segments)} -- each
    end's runs to its vias (run_to_via), a pair's both legs. Returns (ban {key: the layers whose copper -- another net's
    segment outside every run, or its pad there -- comes within the rule, `clear` past both half widths, of the run},
    sep [(key, key): two runs within the rule of each other, so on different layers wherever they end]). Each run
    segment is held off every net's copper but its own -- a pair's partner leg's too: its ball on the surface, where
    the run moved was the relayer's, shorted the zynq LVDS bus's TX_D5_P onto TX_D5_N's ball, U1.Y14"""
    import numpy as np
    from pack import seg_seg_dist
    moving = {id(s) for _nids, segs in runs.values() for s in segs}
    arr = lambda segs: (np.array([(s.start_x, s.start_y) for s in segs], float).reshape(-1, 2),
                        np.array([(s.end_x, s.end_y) for s in segs], float).reshape(-1, 2),
                        np.array([s.width / 2 for s in segs], float))
    R = {k: arr(segs) for k, (_n, segs) in runs.items()}
    RN = {k: np.array([s.net_id for s in segs]) for k, (_n, segs) in runs.items()}
    ban, sep = {}, []
    for L in layers:
        fixed = [s for s in pcb.segments if s.layer == L and id(s) not in moving]
        # (a pad on the layer: its rectangle's four sides, its copper inside them -- a run's end inside one is in it)
        pads = [p for fp in pcb.footprints.values() for p in fp.pads
                if (L in p.layers or '*.Cu' in p.layers) and p.pad_type != 'np_thru_hole']
        for k, (nids, _segs) in runs.items():
            a0, a1, ah = R[k]
            an = RN[k]
            oth = [s for s in fixed if not (len(nids) == 1 and s.net_id in nids)]
            b0, b1, bh = arr(oth)
            foreign = an[:, None] != np.array([s.net_id for s in oth])[None, :]
            hit = bool(len(oth)) and bool(((seg_seg_dist(a0, a1, b0, b1) < ah[:, None] + bh[None, :] + clear - SLACK)
                                           & foreign).any())
            for p in pads:
                if hit:
                    break
                own = an == p.net_id
                if own.all():
                    continue
                hx, hy = p.size_x / 2, p.size_y / 2
                c = [(p.global_x - hx, p.global_y - hy), (p.global_x + hx, p.global_y - hy),
                     (p.global_x + hx, p.global_y + hy), (p.global_x - hx, p.global_y + hy)]
                e0, e1 = np.array(c, float), np.array(c[1:] + c[:1], float)
                inside = ((np.abs(a0[:, 0] - p.global_x) <= hx) & (np.abs(a0[:, 1] - p.global_y) <= hy) & ~own).any()
                hit = bool(inside or ((seg_seg_dist(a0, a1, e0, e1) < ah[:, None] + clear - SLACK).any(axis=1)
                                      & ~own).any())
            if hit:
                ban.setdefault(k, set()).add(L)
    ks = sorted(runs)
    for i, k1 in enumerate(ks):
        for k2 in ks[i + 1:]:
            if runs[k1][0] & runs[k2][0]:
                continue
            (a0, a1, ah), (b0, b1, bh) = R[k1], R[k2]
            if (seg_seg_dist(a0, a1, b0, b1) < ah[:, None] + bh[None, :] + clear - SLACK).any():
                sep.append((k1, k2))
    return ban, sep


def relayer(board, out, moves, log=print):
    """`moves` [(net, its stub's end, the layer the stub is on, the layer to move it to[, group])] on `board`, written to
    `out` with its siblings. A GROUP (a pair end's legs, one key) moves whole or not at all: one leg moved alone leaves
    the pair on two layers there, which no coupled route launches from. Returns (the nets moved, the nets left -- no run
    to a via, or a leg of a group one of whose legs has none)"""
    from kicad_parser import parse_kicad_pcb
    from kicad_writer import remove_segments_from_content, add_tracks_and_vias_to_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    byname = {n.name.split('/')[-1]: i for i, n in pcb.nets.items() if n.name}
    strip, add, moved, left, at_via = [], [], [], [], []

    def got_of(m_):
        # the run to move, and its via -- or, an end standing ON its via (no run: the lane leaves the barrel on any
        # layer), no run and that via: the end's layer moves all the same (whole_solve's via end)
        nid = byname.get(m_[0])
        if nid is None:
            return None
        g_ = run_to_via(pcb, nid, m_[1], m_[2], with_via=True)
        if g_:
            return g_
        on = [v for v in pcb.vias if v.net_id == nid and math.hypot(v.x - m_[1][0], v.y - m_[1][1]) < v.size / 2]
        return ([], on[0]) if on else None
    gots = [got_of(m_) for m_ in moves]
    grp = lambda m_: m_[4] if len(m_) > 4 else None
    bad = {grp(m_) for m_, g in zip(moves, gots) if not g and grp(m_) is not None}
    for m_, got in zip(moves, gots):
        nm, pt, L0, L1 = m_[:4]
        nid = byname.get(nm)
        if not got or (grp(m_) is not None and grp(m_) in bad):
            left.append(nm)
            continue
        run, via = got
        at_via.append((nid, via, L1))
        strip += run
        add += [dict(start=(s.start_x, s.start_y), end=(s.end_x, s.end_y), width=s.width, layer=L1, net_id=s.net_id)
                for s in run]
        moved.append(nm)
    n2n = {i: n.name for i, n in pcb.nets.items()}
    content, n_cut = remove_segments_from_content(open(board, encoding='utf-8').read(), strip, n2n)
    if n_cut != len(strip):
        raise SystemExit(f'relayer: {n_cut} of {len(strip)} stub segments found in {board} to move')
    # (a via whose net's copper there -- the segments ending in it, the moved run among them, and a pad it stands in
    # -- is all on one layer joins nothing: the run moved onto its neck's layer)
    gone_ = {id(s) for s in strip}
    drop = []
    for nid, via, L1 in at_via:
        r_ = via.size / 2 + TOL
        touch = lambda x, y: math.hypot(x - via.x, y - via.y) < r_
        lays = {L1}
        lays |= {s.layer for s in pcb.segments if s.net_id == nid and id(s) not in gone_
                 and (touch(s.start_x, s.start_y) or touch(s.end_x, s.end_y))}
        for p in pcb.nets[nid].pads:
            if abs(via.x - p.global_x) <= p.size_x / 2 and abs(via.y - p.global_y) <= p.size_y / 2:
                lays |= ({'*'} if '*.Cu' in p.layers else {L for L in p.layers if L.endswith('.Cu')})
        if len(lays) == 1 and all(v is not via for v in drop):
            drop.append(via)
    if drop:
        from kicad_writer import remove_vias_from_content
        content, n_v = remove_vias_from_content(content, drop, n2n)
        if n_v != len(drop):
            raise SystemExit(f'relayer: {n_v} of {len(drop)} vias found in {board} to drop')
    tmp = out + '.relayer.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(content)
    with contextlib.redirect_stdout(io.StringIO()):
        add_tracks_and_vias_to_pcb(tmp, out, add, [], [], net_id_to_name=n2n)
    os.remove(tmp)
    stem_in, stem_out = board[:-len('.kicad_pcb')], out[:-len('.kicad_pcb')]
    from copy_board import copy_siblings
    copy_siblings(board, out)           # (the project and the .kicad_dru's per-layer rules with it)
    if os.path.isfile(stem_in + '.plan.json'):
        # the plan sidecar with each moved end's layer the one it now stands on: braid.setup reads a planned net's end
        # layers from it, not off the copper -- copied as it was, the loop held the stubs to the layers they had left,
        # and its first re-solve had no plan
        plan = json.load(open(stem_in + '.plan.json'))
        mv = set(moved)
        for nm, pt, _L0, L1 in (m_[:4] for m_ in moves):
            ends = plan.get('ends', {}).get(nm)
            if nm not in mv or not ends:
                continue
            for k_, key in ((0, 'tooth_layer'), (1, 'dest_layer')):
                if math.hypot(ends[k_][0] - pt[0], ends[k_][1] - pt[1]) < TOL and key in plan:
                    plan[key][nm] = L1
        with open(stem_out + '.plan.json', 'w', encoding='utf-8') as f:
            json.dump(plan, f, indent=1, sort_keys=True)
    log(f'relayer: {len(moved)} via end(s) moved onto the plan\'s layers'
        + (f', {len(drop)} via(s) left joining nothing dropped' if drop else '')
        + (f'; left as laid (no run to a via): {", ".join(left)}' if left else ''))
    return moved, left


def main():
    if len(sys.argv) < 3:
        print(__doc__.strip())
        sys.exit(2)
    moves = []
    for a in sys.argv[3:]:
        nm, xy, L0, L1 = a.split(':')
        x, y = (float(v) for v in xy.split(','))
        moves.append((nm, (x, y), L0, L1))
    relayer(sys.argv[1], sys.argv[2], moves)


if __name__ == '__main__':
    main()
