#!/usr/bin/env python3
"""pack_board.py BOARD.kicad_pcb [--out OUT] -- pack a braided board again on
its own. Reads the braid's sidecar (BOARD.pack.json: each lane's copper
as the pack receives it, the corridors' target orders and planned
lines, every lane's tooth) and the board, runs pack.pack_corridor per
corridor exactly as braid.write_out does, and writes OUT.kicad_pcb (+
.kicad_pro) with the routed nets' copper replaced. Seconds where the
braid took a minute: the way to iterate on the pack.

    BRAID_PACK_DEBUG=1 python3 pack_board.py tmp/pk0_k41.kicad_pcb --out tmp/x_k41
"""

KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door
import argparse
import json
import os
import shutil
import sys
import time
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)

from kicad_parser import parse_kicad_pcb, Segment  # noqa: E402
import braid as br  # noqa: E402
import pack as pk  # noqa: E402
import rules as _rules  # noqa: E402  ONE source for every design rule


def _h2h_of(board):
    """The board's own hole-to-hole clearance, as the braid reads it
    (list_nets.board_constraint), else the router's default."""
    from list_nets import board_constraint
    import connect as cn
    v = board_constraint(board, 'min_hole_to_hole')
    d = float(cn.GridRouteConfig().hole_to_hole_clearance or 0.0)
    return float(v) if v and float(v) > d else d


def _relay(ctx, pcb, byname, by_nm, nm, refused, cfg, board, _sr, log):
    """THE COUPLED RE-LAY (2026-09-19, Andy: "try the relay at the end,
    before packing, fast and only improving"): a splice the trim refused
    because another lane of the run stands between the stub and the
    backtrack (K44 DQ13: A10 inside its hairpin) is tried again with that
    lane LIFTED, then the lifted lane is routed anew between its own two
    tips by the production router on the board as it now stands. Kept
    only when the pair's copper is shorter, no lifted lane gained a via,
    and the scoped DRC over the nets involved names nothing it did not
    name before; else every piece goes back. Returns the mm saved."""
    import math
    import time as _t
    import connect as cn

    def k3(x, y):
        return (round(x, 3), round(y, 3))

    def L_of(segs):
        return sum(math.hypot(s_.end_x - s_.start_x, s_.end_y - s_.start_y) for s_ in segs)
    log(f'  relay {nm}: {len(refused)} refused splice(s): '
        + ', '.join(f'depth {r[2]:.1f}/{r[1]:.2f} mm blocked by {sorted(r[3])}' for r in sorted(refused, key=lambda r: -r[0])[:4]))
    for saving, d, s_at, blockers in sorted(refused, key=lambda r: -r[0]):
        if not blockers or len(blockers) > 2 or any(b not in by_nm or b == nm for b in blockers):
            continue
        t0 = _t.time()
        nets = [nm] + sorted(blockers)
        seg0, via0 = list(pcb.segments), list(pcb.vias)
        snap = {b: (list(by_nm[b].out_segs[b]), list(by_nm[b].out_vias[b])) for b in blockers}
        lane0 = (list(by_nm[nm].out_segs[nm]), list(by_nm[nm].out_vias[nm]))
        trims0 = dict(ctx.src_trims)
        v_before = set(_sr.drc_pairs(board, nets=nets, pcb_data=pcb))
        L_before = L_of(lane0[0]) + sum(L_of(snap[b][0]) for b in blockers)

        def restore():
            pcb.segments[:] = seg0
            pcb.vias[:] = via0
            by_nm[nm].out_segs[nm][:] = lane0[0]
            by_nm[nm].out_vias[nm][:] = lane0[1]
            for b in blockers:
                by_nm[b].out_segs[b][:] = snap[b][0]
                by_nm[b].out_vias[b][:] = snap[b][1]
            ctx.src_trims.clear()
            ctx.src_trims.update(trims0)
        drop = {id(x) for b in blockers for x in snap[b][0]} | {id(x) for b in blockers for x in snap[b][1]}
        pcb.segments[:] = [x for x in pcb.segments if id(x) not in drop]
        pcb.vias[:] = [x for x in pcb.vias if id(x) not in drop]
        ctx.src_trim_refused.pop(nm, None)
        got = br.note_source_joint(ctx, nm, by_nm[nm].out_segs[nm], by_nm[nm].out_vias[nm], board, log)
        if not got:
            restore()
            continue
        ok, why = True, ''
        for b in blockers:
            nid_b = byname[b][0]
            ta, tb = ctx.ends[b]
            def layer_at(pt):
                for s_ in snap[b][0]:
                    if k3(s_.start_x, s_.start_y) == k3(*pt) or k3(s_.end_x, s_.end_y) == k3(*pt):
                        return s_.layer
                return None
            la, lb = layer_at(ta), layer_at(tb)
            if la is None or lb is None:
                ok, why = False, f'{b}: tip layer unknown'
                break
            wpts = [(s_.start_x, s_.start_y) for s_ in snap[b][0]] + [(s_.end_x, s_.end_y) for s_ in snap[b][0]]
            res = cn.connect(pcb, nid_b, tuple(ta), la, tuple(tb), lb, cfg, margin=1.5, window_pts=wpts)
            if res is None:
                ok, why = False, f'{b}: no route between its tips'
                break
            segs_b, vias_b = res
            if len(vias_b) > len(snap[b][1]):
                ok, why = False, f'{b}: {len(vias_b)} vias, had {len(snap[b][1])}'
                break
            pcb.segments.extend(segs_b)
            pcb.vias.extend(vias_b)
            by_nm[b].out_segs[b][:] = segs_b
            by_nm[b].out_vias[b][:] = vias_b
        if ok:
            L_after = L_of(by_nm[nm].out_segs[nm]) + sum(L_of(by_nm[b].out_segs[b]) for b in blockers)
            v_after = set(_sr.drc_pairs(board, nets=nets, pcb_data=pcb))
            new_v = v_after - v_before
            if new_v:
                ok, why = False, f'{len(new_v)} new violation(s): {sorted(new_v)[0][:80]}'
            elif L_after >= L_before - 0.2:
                ok, why = False, f'pair {L_before:.1f} -> {L_after:.1f} mm, no gain'
        if not ok:
            restore()
            log(f'  relay {nm} with {sorted(blockers)} lifted: not kept ({why}; {_t.time() - t0:.1f} s)')
            continue
        log(f'  relay {nm}: {sorted(blockers)} lifted, spliced -{got:.1f} mm, re-laid '
            + ', '.join(f'{b} {L_of(snap[b][0]):.1f} -> {L_of(by_nm[b].out_segs[b]):.1f} mm '
                        f'({len(snap[b][1])} -> {len(by_nm[b].out_vias[b])} vias)' for b in blockers)
            + f'; pair {L_before:.1f} -> {L_after:.1f} mm, DRC clean -- KEPT ({_t.time() - t0:.1f} s)')
        return L_before - L_after
    return 0.0


def pack_whole(a):
    """Every lane of the run packed against the board as it stands, twice.
    Lanes come from the routed/fanout pair (the fanout's copper stays);
    each lane's two ends are the degree-1 vertices of its own copper
    (the tooth the one nearer --src), because the trims may have split a
    stub and the fanout board's tips need not exist on this board."""
    import math
    import replan
    # TAUT by default here (PK_FOLLOW=1 restores the follow): measured on
    # K18, four passes with the follow grew the lanes 392.6 -> 394.8 mm and
    # kept the wave (each lane copying its neighbour's jog); taut they went
    # 392.6 -> 386 mm and bend where the obstacle is
    os.environ.setdefault('PK_FOLLOW', '0')
    _r = _rules.install_defaults()
    stem = os.path.splitext(a.board)[0]
    out = a.out or stem + '_packed'
    t0 = time.time()
    pcb = parse_kicad_pcb(a.board)
    pcb_f = parse_kicad_pcb(a.fanout)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    names = [n for n in a.nets.split(',') if n in byname]
    if not names:
        print('whole-board pack: --nets named no net on the board')
        return 2
    kids = {byname[nm][0] for nm in names}
    import source_realize as _sr
    v0 = set(_sr.drc_pairs(a.board, nets=names, pcb_data=pcb))   # the board as it came, before any edit
    lanes = replan.lane_items(pcb, pcb_f, names, byname)

    def k3(x, y):
        return (round(x, 3), round(y, 3))
    ends = {}
    corridors = []
    cfg = SimpleNamespace(layers=list(pcb.board_info.copper_layers), board_edge_clearance=0.0,
                          hole_to_hole_clearance=_h2h_of(a.board))
    ctx = SimpleNamespace(pcb=pcb, byname=byname, kids=kids, cfg=cfg, ends=ends,
                          base_segments=list(pcb.segments), base_vias=list(pcb.vias),
                          dest_chain={}, trim_spans={})
    n_skip = 0
    for nm in names:
        lane_segs, lane_vias = lanes[nm]
        if not lane_segs:
            n_skip += 1
            continue
        nid, net = byname[nm]
        allsegs = [s_ for s_ in pcb.segments if s_.net_id == nid]
        allvias = [v for v in pcb.vias if v.net_id == nid]
        # a run that meets its via INSIDE the annulus rather than at the
        # centre (22 um off, a 12 um crumb) reads as a break: BRIDGE such
        # an end to the via centre with a link segment. Not a snap of the
        # end itself -- moving one end of a 2.1 mm segment by 0.106 mm
        # tilted it into a foreign via's clearance (K32 DQ8 vs DQ14).
        # ...and a link piece is not free either: the emitter's octilinear
        # build refuses a run that starts with a 20 um stub at an odd angle
        # and falls back to the string's raw chords (K44: 1205 -> 2176
        # segments, 1134 -> 1168 mm). So a TINY gap is snapped (a 22 um
        # move of a 0.5 mm piece tilts nothing) and only a large one is
        # bridged.
        SNAP = float(os.environ.get('PK_VIA_SNAP', '0.03') or 0)
        linked = set()
        for v in allvias:
            r_v = v.size / 2
            for s_ in list(allsegs):
                for end in ('start', 'end'):
                    ex, ey = getattr(s_, end + '_x'), getattr(s_, end + '_y')
                    d = math.hypot(ex - v.x, ey - v.y)
                    if d <= 0.001 or d > r_v:
                        continue
                    if d <= SNAP:
                        setattr(s_, end + '_x', v.x)
                        setattr(s_, end + '_y', v.y)
                        continue
                    key_ = (k3(ex, ey), k3(v.x, v.y))
                    if key_ not in linked:
                        linked.add(key_)
                        link = Segment(ex, ey, v.x, v.y, s_.width, s_.layer, nid)
                        allsegs.append(link)
                        pcb.segments.append(link)
        crumbs = {id(s_) for s_ in allsegs if math.hypot(s_.end_x - s_.start_x, s_.end_y - s_.start_y) < 1e-3}
        if crumbs:
            pcb.segments = [s_ for s_ in pcb.segments if id(s_) not in crumbs]
            allsegs = [s_ for s_ in allsegs if id(s_) not in crumbs]
        # THE LANE = everything outside the two STUB CHAINS. An evolved
        # world's fanout board is DERIVED from a routed one and keeps lane
        # fragments as the berth's copper, so "routed minus fanout" cut
        # WE and A3 (K44) into pieces with fanout-matched gaps between.
        # The stub chain is the copper reachable from a pad through
        # fanout-matched segments only; its tips are the lane's ends.
        lane_ids = {id(s_) for s_ in lane_segs}
        adj = {}
        for s_ in allsegs:
            a_, b_ = k3(s_.start_x, s_.start_y), k3(s_.end_x, s_.end_y)
            adj.setdefault(a_, []).append((b_, s_))
            adj.setdefault(b_, []).append((a_, s_))
        seeds = [q for q in adj if any(math.hypot(q[0] - p_.global_x, q[1] - p_.global_y)
                                        <= max(p_.size_x, p_.size_y) / 2 + 0.001 for p_ in net.pads)]
        stub, seen_v, stack = set(), set(seeds), list(seeds)
        while stack:
            q = stack.pop()
            for q2, s_ in adj.get(q, ()):
                if id(s_) in lane_ids or id(s_) in stub:
                    continue
                stub.add(id(s_))
                if q2 not in seen_v:
                    seen_v.add(q2)
                    stack.append(q2)
        segs = [s_ for s_ in allsegs if id(s_) not in stub]
        # SPURS: a short dead-end piece hanging off a branch vertex (a
        # 35 um crumb at K44 A0) leaves the packer's chaining with one
        # segment over, and "unchained" keeps the whole lane as laid (112
        # of 44 lanes' passes at K44). Dead copper anyway: dropped.
        for _rep in range(3):
            dg = {}
            for s_ in segs:
                for q in (k3(s_.start_x, s_.start_y), k3(s_.end_x, s_.end_y)):
                    dg[q] = dg.get(q, 0) + 1
            spurs = [s_ for s_ in segs
                     if math.hypot(s_.end_x - s_.start_x, s_.end_y - s_.start_y) < 0.2
                     and sorted((dg[k3(s_.start_x, s_.start_y)], dg[k3(s_.end_x, s_.end_y)])) [0] == 1
                     and sorted((dg[k3(s_.start_x, s_.start_y)], dg[k3(s_.end_x, s_.end_y)]))[1] >= 3]
            if not spurs:
                break
            drop = {id(x) for x in spurs}
            segs = [s_ for s_ in segs if id(s_) not in drop]
            pcb.segments = [s_ for s_ in pcb.segments if id(s_) not in drop]
        # the ends as the EXACT coordinates of the segment end, not the
        # rounded key: the packer chains on four decimals, and a fanout
        # tip at 80.0703 never matched its 80.070 key (112 lanes
        # "unchained" at K44 once the links carried such ends)
        deg, exact = {}, {}
        for s_ in segs:
            for x_, y_ in ((s_.start_x, s_.start_y), (s_.end_x, s_.end_y)):
                q = k3(x_, y_)
                deg[q] = deg.get(q, 0) + 1
                exact.setdefault(q, (x_, y_))
        tips = [exact[q] for q, d in deg.items() if d == 1]
        if len(tips) != 2:
            n_skip += 1
            print(f'  {nm}: lane has {len(tips)} free end(s), not 2 -- kept as laid')
            continue
        # LOOPS: a leftover of the chaining whose BOTH ends lie on the
        # chain (a 35 um duplicate diagonal at K44 A0, both ends on the
        # path) is copper the chain already provides; a leftover with an
        # end off the chain is a real branch and the lane is left alone
        pts_, _l, _o, left_ = pk.chain_segs(segs, tips[0])
        if left_:
            onchain = {(round(x_, 4), round(y_, 4)) for x_, y_ in pts_}
            loops = [l for l in left_ if (round(l.start_x, 4), round(l.start_y, 4)) in onchain
                     and (round(l.end_x, 4), round(l.end_y, 4)) in onchain]
            if len(loops) == len(left_):
                drop = {id(x) for x in loops}
                segs = [s_ for s_ in segs if id(s_) not in drop]
                pcb.segments = [s_ for s_ in pcb.segments if id(s_) not in drop]
            else:
                n_skip += 1
                print(f'  {nm}: {len(left_) - len(loops)} branch piece(s) off the lane -- kept as laid')
                continue
        if nm in os.environ.get('PK_WHOLE_DUMP', '').split(','):
            print(f'  DUMP {nm}: {len(allsegs)} segs, {len(stub)} in the stub chains, {len(segs)} lane; tips {tips}')
            pts_, lays_, objs_, left_ = pk.chain_segs(segs, tips[0])
            print(f'    chain from tips[0]: {len(pts_)} points, {len(left_)} left')
            for s_ in left_[:6]:
                print(f'      left {s_.layer} ({s_.start_x:.4f},{s_.start_y:.4f})->({s_.end_x:.4f},{s_.end_y:.4f})')
            if pts_:
                print(f'    chain end {pts_[-1]}')
        fan_via_xy = {k3(v.x, v.y) for v in pcb_f.vias if v.net_id == nid}
        vias = [v for v in allvias if k3(v.x, v.y) not in fan_via_xy]
        lanes[nm] = (segs, vias)
        src_pads = [p_ for p_ in byname[nm][1].pads if p_.component_ref == a.src]
        if not src_pads:
            n_skip += 1
            continue
        sp = src_pads[0]
        tips.sort(key=lambda q: math.hypot(q[0] - sp.global_x, q[1] - sp.global_y))
        if nm in os.environ.get('PK_WHOLE_DUMP', '').split(','):
            print(f'  DUMP {nm}: src pad {sp.component_ref}.{sp.pad_number} at ({sp.global_x:.2f},{sp.global_y:.2f}); tips by distance {[(round(q[0],2), round(q[1],2), round(math.hypot(q[0]-sp.global_x, q[1]-sp.global_y),1)) for q in tips]}')
        ends[nm] = (tips[0], tips[1])
        corridors.append(SimpleNamespace(members=[nm], target=[nm], lane_xy={nm: [tips[0], tips[1]]},
                                         out_segs={nm: list(segs)}, out_vias={nm: list(vias)}, ctx=ctx))
    log = print
    # THE SOURCE TRIM ON THE FINISHED BOARD (2026-09-19, Andy: "a long
    # stub going south whose long backtrack is not removed"): the same
    # splice as the braid's write-time note_source_joint, run here over
    # every lane with its stub chain walked from the tip -- a board the
    # evolution assembled from probes may carry a backtrack no braid saw
    # whole. Vias never change; the scoped DRC decides each splice.
    ctx.src_chain, ctx.src_trims, ctx.tooth_layer, ctx.src_trim_refused = {}, {}, {}, {}
    _k4 = lambda x, y: (round(x, 4), round(y, 4))          # noqa: E731

    def walk_chains():
        """Every lane's stub chain from its tooth tip toward the pad, on
        the board AS IT STANDS -- walked again each round, so a stub a
        splice has cut is seen cut (else the next round re-splices the
        tail that is gone and books the saving twice)."""
        ctx.src_chain.clear()
        for c in corridors:
            nm = c.members[0]
            nid, net = byname[nm]
            tip0 = ends[nm][0]
            lane_ids = {id(x) for x in c.out_segs[nm]}
            stubs = [s_ for s_ in pcb.segments if s_.net_id == nid and id(s_) not in lane_ids]
            best_ = None
            for s_ in stubs:
                for ex, ey in ((s_.start_x, s_.start_y), (s_.end_x, s_.end_y)):
                    d_ = math.hypot(ex - tip0[0], ey - tip0[1])
                    if d_ <= 0.15 and (best_ is None or d_ < best_[0]):
                        best_ = (d_, (ex, ey), s_)
            if best_ is None:
                continue
            vias_n = [(v.x, v.y, v.size / 2) for v in pcb.vias if v.net_id == nid]
            pads_n = [(p_.global_x, p_.global_y, max(p_.size_x, p_.size_y) / 2) for p_ in net.pads]

            def _stop(pt, _v=vias_n, _p=pads_n):
                return any(math.hypot(pt[0] - ax, pt[1] - ay) <= max(0.02, ar) for ax, ay, ar in _v + _p)
            ctx.src_chain[nm] = br._walk_stub(stubs, _k4(*best_[1]), best_[2].layer, _stop, _k4, max_hops=5000)
    _mm = 0.0
    by_nm0 = {c.members[0]: c for c in corridors}
    rcfg = None
    if a.relay:
        import connect as cn
        from list_nets import board_constraint
        kw = {}
        h2h_ = _h2h_of(a.board)
        if h2h_:
            kw['hole_to_hole_clearance'] = h2h_
        edge_ = board_constraint(a.board, 'min_copper_edge_clearance')
        if edge_ and float(edge_) > br.CLEAR:
            kw['board_edge_clearance'] = float(edge_)
        rcfg = cn.make_config(pcb, br.TRACK, br.CLEAR, br.VIA_SIZE, br.VIA_DRILL, grid_step=0.025, **kw)
    n_relay = 0
    # ROUNDS: a splice refused because another lane's backtrack stood in
    # the way (K44 DQ13, walled by A10's own hairpin one row over) is
    # clear once that lane's own trim has run -- which, in ladder order,
    # may be later in the same round. Every round re-asks every lane
    # until a round changes nothing.
    for _round in range(4):
        changed = 0
        walk_chains()
        for c in corridors:
            nm = c.members[0]
            ctx.src_trim_refused.pop(nm, None)
            got = br.note_source_joint(ctx, nm, c.out_segs[nm], c.out_vias[nm], a.board, log)
            if not got and rcfg is not None and ctx.src_trim_refused.get(nm):
                got = _relay(ctx, pcb, byname, by_nm0, nm, ctx.src_trim_refused[nm], rcfg, a.board, _sr, log)
                if got:
                    n_relay += 1
            if got:
                changed += 1
            _mm += got
        if not changed:
            break
        log(f'  trim round {_round + 1}: {changed} lane(s) spliced')
    if ctx.src_trims:
        log(f'source stub trim (finished board): {len(ctx.src_trims)} lane(s) spliced, -{_mm:.1f} mm '
            f'({", ".join(sorted(ctx.src_trims))}){f", {n_relay} by a coupled re-lay" if n_relay else ""}')
    L0 = sum(pk.seg_len(c.out_segs[nm]) for c in corridors for nm in c.members)
    log(f'pack_board (whole board): {len(corridors)} lane(s) of {len(names)} from the pair, {n_skip} kept as laid, '
        f'{L0:.1f} mm  (read {time.time() - t0:.1f} s)')
    orig = {c.members[0]: (list(c.out_segs[c.members[0]]), list(c.out_vias[c.members[0]])) for c in corridors}
    for p_ in range(a.passes):
        for c in corridors:
            pk.pack_corridor(c, log)
        L1 = sum(pk.seg_len(c.out_segs[nm]) for c in corridors for nm in c.members)
        log(f'  pass {p_ + 1}: lanes {L0:.1f} -> {L1:.1f} mm')
    # THE GRADE IS THE GATE: the emitter validates piece by piece and
    # still let a re-emitted end into a via's clearance (K32 DQ8 against
    # DQ14's via, 0.073 mm). Every violation the pack ADDED names its
    # nets; those lanes go back to the copper they came with, until the
    # scoped DRC is what it was before the pack.
    import re as _re
    by_nm = {c.members[0]: c for c in corridors}
    for _round in range(6):
        new = [ln for ln in _sr.drc_pairs(a.board, nets=names, pcb_data=pcb) if ln not in v0]
        if not new:
            break
        culprits = set()
        for ln in new:
            for tok in br.drc_line_nets(ln):
                if tok in by_nm and tok in orig:
                    culprits.add(tok)
        if not culprits:
            log(f'  pack gate: {len(new)} new violation(s) name no packed lane -- {new[0][:100]}')
            break
        for nm in sorted(culprits):
            c = by_nm[nm]
            cur_s = {id(x) for x in c.out_segs.get(nm, [])}
            cur_v = {id(x) for x in c.out_vias.get(nm, [])}
            pcb.segments = [x for x in pcb.segments if id(x) not in cur_s] + orig[nm][0]
            pcb.vias = [x for x in pcb.vias if id(x) not in cur_v] + orig[nm][1]
            c.out_segs[nm], c.out_vias[nm] = list(orig[nm][0]), list(orig[nm][1])
            del orig[nm]
        log(f'  pack gate: {len(new)} new violation(s) -> {sorted(culprits)} back to the copper they came with')
    L2 = sum(pk.seg_len(c.out_segs[nm]) for c in corridors for nm in c.members)
    log(f'  lanes {L0:.1f} -> {L2:.1f} mm after the gate')
    txt = open(a.board, encoding='utf-8').read()
    kid_names = {pcb.nets[i].name for i in kids if i in pcb.nets}
    txt = br.strip_net_segments(txt, kids, kid_names)
    txt = br.strip_net_items(txt, 'via', kids, kid_names)
    add = []
    seen = set()
    for s_ in pcb.segments:
        if s_.net_id not in kids:
            continue
        if abs(s_.end_x - s_.start_x) < 1e-3 and abs(s_.end_y - s_.start_y) < 1e-3:
            continue
        k = (round(s_.start_x, 4), round(s_.start_y, 4), round(s_.end_x, 4), round(s_.end_y, 4), s_.layer)
        if k in seen or (k[2], k[3], k[0], k[1], k[4]) in seen:
            continue
        seen.add(k)
        add.append(f'  (segment (start {s_.start_x:.4f} {s_.start_y:.4f}) (end {s_.end_x:.4f} {s_.end_y:.4f}) '
                   f'(width {s_.width}) (layer "{s_.layer}") (net {s_.net_id}))\n')
    for v in pcb.vias:
        if v.net_id in kids:
            add.append(f'  (via (at {v.x:.4f} {v.y:.4f}) (size {v.size}) (drill {v.drill}) '
                       f'(layers "F.Cu" "B.Cu") (net {v.net_id}))\n')
    i = txt.rstrip().rfind(')')
    txt = txt[:i] + ''.join(add) + txt[i:]
    with open(out + '.kicad_pcb', 'w', encoding='utf-8') as f:
        f.write(txt)
    pro = stem + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, out + '.kicad_pro')
    log(f'wrote {out}.kicad_pcb: {len(add)} items  ({time.time() - t0:.1f} s)')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('--out', default=None, help='output stem (default: <board stem>_packed)')
    ap.add_argument('--sidecar', default=None, help='the pack sidecar (default: <board stem>.pack.json)')
    ap.add_argument('--fanout', default=None,
                    help='WHOLE-BOARD mode (2026-09-19): derive every lane of --nets as the copper the '
                         'board carries beyond this fanout board (replan.lane_items), one corridor per '
                         'lane, and pack them all -- for a board whose sidecar is a probe braid\'s (an '
                         'evolved world: the K44 record\'s sidecar named ONE lane of 44)')
    ap.add_argument('--nets', default='', help='whole-board mode: the run\'s nets, comma-separated')
    ap.add_argument('--src', default='U1', help='whole-board mode: the source array (the tooth end of each lane)')
    ap.add_argument('--relay', type=int, default=1,
                    help='whole-board mode: 1 (default) = a splice the trim refused because another lane of the '
                         'run stands in its way is tried again with that lane lifted and routed anew between its '
                         'own tips; kept only when the pair is shorter, no via is added and the scoped DRC adds '
                         'nothing. 0 = off')
    ap.add_argument('--passes', type=int, default=2,
                    help='whole-board mode: how many times every lane is packed (each pass sees the room the last left)')
    a = ap.parse_args()
    if a.fanout:
        return pack_whole(a)
    # THE DESIGN CONSTANTS (rules.py). pack.py reads braid's constants
    # through the MODULE (br.CLEAR, br.TRACK, br.VIA_SIZE) at call time, so
    # installing them here is all it takes.
    _r = _rules.install_defaults()
    print(f'rules: clearance {br.SPEC_CLEARANCE} (hug {br.CLEAR}), '
          f'track {br.TRACK}, via {br.VIA_SIZE}/{br.VIA_DRILL}'
          f'  [{_r.source}]')
    stem = os.path.splitext(a.board)[0]
    out = a.out or stem + '_packed'
    side_path = a.sidecar or stem + '.pack.json'
    if not os.path.exists(side_path):
        print(f'no sidecar {side_path}: run the braid on this board first (it writes one)')
        return 2
    side = json.load(open(side_path))
    t0 = time.time()
    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    names = list(side['lanes'])
    kids = {byname[nm][0] for nm in names}
    # the lanes' copper: the sidecar's pieces matched to the board's own
    # segment and via OBJECTS (the pack tells copper apart by identity)
    def key(x, y):
        return (round(x, 3), round(y, 3))
    seg_index = {}
    for s in pcb.segments:
        if s.net_id in kids:
            seg_index.setdefault((key(s.start_x, s.start_y), key(s.end_x, s.end_y), s.layer), []).append(s)
            seg_index.setdefault((key(s.end_x, s.end_y), key(s.start_x, s.start_y), s.layer), []).append(s)
    via_index = {}
    for v in pcb.vias:
        if v.net_id in kids:
            via_index.setdefault(key(v.x, v.y), []).append(v)
    out_segs, out_vias = {}, {}
    missing = 0
    for nm in names:
        segs = []
        for sx, sy, ex, ey, L, w in side['lanes'][nm]['segs']:
            hit = seg_index.get((key(sx, sy), key(ex, ey), L))
            if hit:
                segs.append(hit[0])
            else:
                missing += 1
        vias = []
        for x, y in side['lanes'][nm]['vias']:
            hit = via_index.get(key(x, y))
            if hit:
                vias.append(hit[0])
            else:
                missing += 1
        out_segs[nm], out_vias[nm] = segs, vias
    if missing:
        print(f'WARNING: {missing} sidecar piece(s) not found on the board (a board the braid did not write?)')
    cfg = SimpleNamespace(layers=list(side['layers']), board_edge_clearance=side['board_edge_clearance'],
                          hole_to_hole_clearance=_h2h_of(a.board))
    # the destination stub chain per lane, tip-side first, matched to the
    # board's segments so a join at a vertex can drop the bypassed stub
    dest_chain = {}
    for nm, ch in (side.get('dest_chain') or {}).items():
        lst = []
        for sx, sy, ex, ey, L in ch:
            hit = seg_index.get((key(sx, sy), key(ex, ey), L))
            lst.append((hit[0] if hit else None, (round(sx, 3), round(sy, 3)), (round(ex, 3), round(ey, 3))))
        dest_chain[nm] = lst
    ctx = SimpleNamespace(pcb=pcb, byname=byname, kids=kids, cfg=cfg,
                          ends={nm: (tuple(e[0]), tuple(e[1])) for nm, e in side['ends'].items()},
                          base_segments=list(pcb.segments), base_vias=list(pcb.vias),
                          dest_chain=dest_chain, trim_spans={})
    corridors = []
    for cd in side['corridors']:
        c = SimpleNamespace(members=list(cd['members']), target=list(cd['target']),
                            lane_xy={nm: [tuple(p) for p in poly] for nm, poly in cd['lane_xy'].items()},
                            out_segs={nm: out_segs[nm] for nm in cd['members'] if nm in out_segs},
                            out_vias={nm: out_vias[nm] for nm in cd['members'] if nm in out_vias},
                            ctx=ctx)
        corridors.append(c)
    log = print
    log(f'pack_board: {os.path.basename(a.board)}, {len(names)} lanes in {len(corridors)} corridor(s), '
        f'{sum(len(v) for v in out_segs.values())} lane pieces  (read {time.time() - t0:.1f} s)')
    for c in corridors:
        pk.pack_corridor(c, log)
    # the stub trim, as braid.write_out does it: the stub tip-side of the
    # vertex the packed lane ends at is bypassed copper
    n_trim = 0
    for c in corridors:
        for nm in c.members:
            if c.out_segs.get(nm):
                br.note_joint(ctx, nm, c.out_segs[nm])
                n_trim += len(ctx.trim_spans.get(nm, []))
    if n_trim:
        log(f'berth stub trim: {n_trim} bypassed segment(s) removed')
    # write: the routed nets' copper replaced, every other line of the file as it was
    txt = open(a.board, encoding='utf-8').read()
    kid_names = {pcb.nets[i].name for i in kids if i in pcb.nets}
    txt = br.strip_net_segments(txt, kids, kid_names)
    txt = br.strip_net_items(txt, 'via', kids, kid_names)
    add = []
    seen = set()
    for s in pcb.segments:
        if s.net_id not in kids:
            continue
        if abs(s.end_x - s.start_x) < 1e-3 and abs(s.end_y - s.start_y) < 1e-3:
            continue
        k = (round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4), round(s.end_y, 4), s.layer)
        if k in seen or (k[2], k[3], k[0], k[1], k[4]) in seen:
            continue
        seen.add(k)
        add.append(f'  (segment (start {s.start_x:.4f} {s.start_y:.4f}) (end {s.end_x:.4f} {s.end_y:.4f}) '
                   f'(width {s.width}) (layer "{s.layer}") (net {s.net_id}))\n')
    for v in pcb.vias:
        if v.net_id not in kids:
            continue
        layers = v.layers if getattr(v, 'layers', None) else ['F.Cu', 'B.Cu']
        add.append(f'  (via (at {v.x:.4f} {v.y:.4f}) (size {v.size}) (drill {v.drill}) '
                   f'(layers "{layers[0]}" "{layers[-1]}") (net {v.net_id}))\n')
    k = txt.rstrip().rfind(')')
    with open(out + '.kicad_pcb', 'w') as f:
        f.write(txt[:k] + ''.join(add) + txt[k:])
    pro = stem + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, out + '.kicad_pro')
    log(f'wrote {out}.kicad_pcb: {len(add)} items  ({time.time() - t0:.1f} s)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
