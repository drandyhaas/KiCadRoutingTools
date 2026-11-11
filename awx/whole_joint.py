#!/usr/bin/env python3
"""whole_joint.py DIR KEY -- the COUPLED solve of several buses off one shared array (route_bus JOINT_DIR): a round's
first plans (KEY the round) or a loop's re-solves (KEY ROUND.LOOP.I: every re-solve of the loops of the buses, in step).

Each bus's whole route solves its round's first plan alone (whole_solve), writes it and every variable of its model
(SOLVE.vals.json) and asks for the coupled solve (DIR/rROUND/BUS.req.json: the settings it solved under, its solve's
path, its flow frame's turn). Here every bus's model is built again into ONE model -- the very model it solved alone,
every variable started from its own plan -- and the buses are COUPLED where their lanes cross: two lanes of two buses
whose reference paths (their taut paths, turned back into the board's frame) cross stand on different layers there, each
one's changes a via's room off the place -- or the crossing is BROKEN, at a price above a net over two vias (W_CROSS).
Two lanes of two buses that come within a lane's pitch of each other are coupled alike, at each one's nearest point: on
one layer there, they would crowd -- a conflict the two buses settle together, as a crossing. The plans the buses found
alone are the coupled solve's warm start, laid out whole (each broken coupling of theirs priced), and the search mends
them -- where they break none, there is nothing to mend: no search, each bus's own plan the coupled one (its
BUS.joint.json 'own'). Each bus's plan is written over its solve.json (its own kept beside it, .solo.json), and DIR/rROUND/BUS.joint.json
says it is there. Couplings, and the broken ones, on stdout."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door

import contextlib
import io
import json
import math
import os
import shutil
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import awx_settings  # noqa: E402

W_CROSS = int(awx_settings.get('JOINT_W_CROSS') or '20')    # a broken coupling, in vias
# ...and one of a VIA-END RUN (whole_route.with_end_runs): a run inside the array moves onto its lane's end layer and
# nothing after moves it, so a broken one ships a short -- a loop's coupled re-solve broke two at 20 vias each, an LVDS
# bus's run across two of a memory bus's on one layer, and the merge had both crossings. Priced past any plan: at 200
# the same re-solve still broke both (its search stopped at 894 vias-worth); at this, none, for 546.6 against 534
W_RUN_CROSS = int(awx_settings.get('JOINT_W_RUN_CROSS') or '100000')
BATCHES = int(awx_settings.get('JOINT_BATCHES') or '100')   # the search's interleaved batches at most
# ROOM: a lane's change of layer within this (mm along its lane) of a place another bus's lane meets it is CRAMPED --
# allowed (a via's room off the place is the rule), priced, so that a change takes open run where its lane has some.
# The places are read off the buses' reference paths, and the laid-out lanes pass a few mm off them: a change put a
# via's room past its lane's last meeting with another bus stood, laid out, among that bus's lanes
ROOM = float(awx_settings.get('JOINT_ROOM') or '4')
W_ROOM = float(awx_settings.get('JOINT_W_ROOM') or '1')     # a cramped change, in vias
WORKERS = 4


def turn(p, q, cx, cy):
    """board point p in the flow frame of q quarter turns about (cx, cy) (flow_frame's turn)"""
    import flow_frame as ff
    d = ff._q(p[0] - cx, p[1] - cy, q)
    return (cx + d[0], cy + d[1])


def crossings(pa, pb):
    """the points where the pieces pa and pb ([(a, b)] segments) cross"""
    out = []
    for a0, a1 in pa:
        for b0, b1 in pb:
            if max(a0[0], a1[0]) < min(b0[0], b1[0]) or max(b0[0], b1[0]) < min(a0[0], a1[0]) or \
                    max(a0[1], a1[1]) < min(b0[1], b1[1]) or max(b0[1], b1[1]) < min(a0[1], a1[1]):
                continue
            r = (a1[0] - a0[0], a1[1] - a0[1])
            s = (b1[0] - b0[0], b1[1] - b0[1])
            den = r[0] * s[1] - r[1] * s[0]
            if abs(den) < 1e-12:
                continue
            t = ((b0[0] - a0[0]) * s[1] - (b0[1] - a0[1]) * s[0]) / den
            u = ((b0[0] - a0[0]) * r[1] - (b0[1] - a0[1]) * r[0]) / den
            if 0 <= t <= 1 and 0 <= u <= 1:
                out.append((a0[0] + t * r[0], a0[1] + t * r[1]))
    return out


def near_points(pa, pb, near):
    """[(xa, xb, d)]: where the pieces pa and pb ([(a, b)] segments) cross (xa = xb, d 0), and where a segment of each
    comes within `near` of the other's without crossing -- each one's nearest point and their distance, one pair a pair
    of segments"""
    out = [(x, x, 0.0) for x in crossings(pa, pb)]
    if near <= 0:
        return out
    def seg_pt(c, u, v):
        ux, uy = v[0] - u[0], v[1] - u[1]
        L2 = ux * ux + uy * uy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((c[0] - u[0]) * ux + (c[1] - u[1]) * uy) / L2))
        q = (u[0] + t * ux, u[1] + t * uy)
        return math.hypot(c[0] - q[0], c[1] - q[1]), q
    hit = {(round(x[0], 3), round(x[1], 3)) for x, _x, _d in out}
    for a0, a1 in pa:
        for b0, b1 in pb:
            if max(a0[0], a1[0]) + near < min(b0[0], b1[0]) or max(b0[0], b1[0]) + near < min(a0[0], a1[0]) or \
                    max(a0[1], a1[1]) + near < min(b0[1], b1[1]) or max(b0[1], b1[1]) + near < min(a0[1], a1[1]):
                continue
            if crossings([(a0, a1)], [(b0, b1)]):
                continue
            best = min([(seg_pt(a0, b0, b1)[0], a0, seg_pt(a0, b0, b1)[1]),
                        (seg_pt(a1, b0, b1)[0], a1, seg_pt(a1, b0, b1)[1]),
                        (seg_pt(b0, a0, a1)[0], seg_pt(b0, a0, a1)[1], b0),
                        (seg_pt(b1, a0, a1)[0], seg_pt(b1, a0, a1)[1], b1)], key=lambda t_: t_[0])
            if best[0] < near:
                k_ = (round(best[1][0], 3), round(best[1][1], 3))
                if k_ not in hit:
                    hit.add(k_)
                    out.append((best[1], best[2], best[0]))
    return out


def main():
    d, rnd = sys.argv[1], sys.argv[2]
    rd = os.path.join(d, f'r{rnd}')
    reqs = {}
    for f in sorted(os.listdir(rd)):
        if f.endswith('.req.json'):
            r = json.load(open(os.path.join(rd, f)))
            reqs[r['bus']] = r
    t0 = time.time()
    print(f'whole_joint: round {rnd}, buses {", ".join(sorted(reqs))}')
    from ortools.sat.python import cp_model
    import cp_seed
    m = cp_model.CpModel()
    parts, ws = {}, None
    files = lambda env, var: [x for x in (env.get(var) or '').split(',') if x]   # noqa: E731
    for bus, r in sorted(reqs.items()):
        env = r['env']
        vals = json.load(open(r['solve'][:-len('.json')] + '.vals.json'))
        with awx_settings.given(env), contextlib.redirect_stdout(io.StringIO()):
            import whole_ctx
            import whole_solve as ws
            ctx, _cs = whole_ctx.plan()
            p = ws.solve(ctx, env['DEST'], files(env, 'CUTS'), files(env, 'HIST'), None,
                         soft_cuts=files(env, 'SOFT_CUTS'), joint=types.SimpleNamespace(m=m))
        n0, n1 = p.nv
        if n1 - n0 != len(vals):
            print(f'  {bus}: its model here has {n1 - n0} variables, its own solve {len(vals)} -- NOT the same model: '
                  f'the bus keeps its own plan')
            continue
        for i, v in zip(range(n0, n1), vals):
            m.AddHint(m.GetIntVarFromProtoIndex(i), v)
        p.bus, p.req, p.vals = bus, r, vals
        p.q, p.cx, p.cy = r['turn']
        # (a bus whose pairs read chirality -1 is planned on its board TURNED OVER -- braid.setup: y mirrored about a
        # lattice line, F.Cu and B.Cu swapped, the mirror ctx.M its own inverse -- so its points are mirrored back
        # before the turn, and its layers' F and B swapped where two buses' layers are compared)
        p.flip = getattr(ctx, 'chi', 1) < 0 and getattr(ctx, 'M', None) is not None
        p.mir = ctx.M if p.flip else (lambda x, y: (x, y))
        # each lane's way in the board's frame, as pieces: its own plan laid out (the request's lanes, published in
        # the board's frame), else its taut path turned back
        p.board_segs, p.run_segs = {}, {}
        for row_ in ((r.get('lanes') or {}).get('pieces') or []):
            x0_, y0_, x1_, y1_, _L, n_ = row_[:6]
            if n_ not in p.M:
                continue
            if len(row_) > 6 and row_[6] in ('end0', 'end1'):
                # (a VIA-END RUN inside an array, whole_route.with_end_runs: the relayer moves it onto the lane's layer
                # at that end, so it is coupled on that layer, its lane's changes free there -- the run is a track)
                p.run_segs.setdefault(n_, []).append(((x0_, y0_), (x1_, y1_)))
            else:
                p.board_segs.setdefault(n_, []).append(((x0_, y0_), (x1_, y1_)))
        if not p.board_segs:
            for n_ in p.M:
                pb_ = [turn(p.mir(*map(float, pt[:2])), -p.q, p.cx, p.cy) for pt in ctx.paths.get(n_) or []]
                p.board_segs[n_] = list(zip(pb_, pb_[1:]))
        from bga_fanout.flip_frame import other_layer
        p.lix = [p.LAYN.index(other_layer(L_)) if p.flip else i_ for i_, L_ in enumerate(p.LAYN)]
        parts[bus] = p
        print(f'  {bus}: {len(p.M)} lanes, {n1 - n0} variables, its own plan the start'
              + (' (planned turned over: chirality -1)' if p.flip else ''))
    # the couplings: two lanes of two buses whose paths cross -- their layers there differ, each change a via's room
    # off the place, or the coupling is broken
    W_V = ws.W_V
    # (a near miss: two lanes closer than a lane's pitch -- the braid's track and clearance, as each bus plans them)
    import braid as te
    NEAR = te.TRACK + te.CLEAR - 1e-3
    # ...and two lanes closer than a VIA's reach -- a via of either there stands on the other's track whatever their
    # layers: coupled for their changes alone, each kept a via's room off the place (the joint loop's audits found a
    # lane run through another bus's via at 0.25 mm, beyond the pitch: no coupling had held the via off it)
    NEAR_V = te.VIA_SIZE / 2 + te.TRACK / 2 + te.CLEAR - 1e-3
    brk, cpl, cost, lits = [], [], [], []
    brk_run = []                # (the broken couplings of a run: W_RUN_CROSS)
    nvia = 0
    G = ws.G

    def bits_at(p, n, u, k_):
        """lane n's changes before u (monotone), each a via's room off u -- unless the coupling is broken (`k_`)"""
        cs_, act = p.chg[n]
        U, h = p.Q(u), p.QU(p.VNEED)
        out = []
        for x_, a_ in zip(cs_, act):
            bb = m.NewBoolVar('')
            m.Add(x_ <= U).OnlyEnforceIf(bb)
            m.Add(x_ >= U).OnlyEnforceIf([bb.Not(), a_])
            m.Add(x_ + h <= U).OnlyEnforceIf([bb, k_.Not()])
            m.Add(x_ - h >= U).OnlyEnforceIf([bb.Not(), a_, k_.Not()])
            m.AddImplication(bb, a_)
            out.append(bb)
        return out
    met = {}                               # (bus, lane) -> where along it (u) another bus's lane meets it
    free = m.NewBoolVar('')                # (a run's own lane: its changes held off nothing -- bits_at's `k_` set)
    m.Add(free == 1)

    def ways(p):
        """(lane, its pieces, a via-end run's?) of bus part `p`: its laid-out lanes, then its via-end runs"""
        return [(n_, r_, False) for n_, r_ in p.board_segs.items()] + [(n_, r_, True) for n_, r_ in p.run_segs.items()]
    buses = sorted(parts)
    for i, A in enumerate(buses):
        for B in buses[i + 1:]:
            pa, pb = parts[A], parts[B]
            for a, ra, run_a in ways(pa):
                for b, rb, run_b in ways(pb):
                    for Xa, Xb, d_ in near_points(ra, rb, max(NEAR, NEAR_V)):
                        if d_ >= NEAR and run_a and run_b:
                            continue                # (two runs within a via's reach: neither places a via there)
                        X = Xa
                        ua = pa.u_of(a, pa.mir(*turn(Xa, pa.q, pa.cx, pa.cy)))
                        ub = pb.u_of(b, pb.mir(*turn(Xb, pb.q, pb.cx, pb.cy)))
                        ua = min(max(ua, pa.entry[a]), pa.end[a])
                        ub = min(max(ub, pb.entry[b]), pb.end[b])
                        k_ = m.NewBoolVar('')
                        if not run_a:
                            met.setdefault((A, a), []).append(ua)
                        if not run_b:
                            met.setdefault((B, b), []).append(ub)
                        # (a run before its lane's entry, or past its end, reads the lane's layer there: its first or
                        # its last)
                        ba_ = bits_at(pa, a, ua, free if run_a else k_)
                        bb_ = bits_at(pb, b, ub, free if run_b else k_)
                        if d_ >= NEAR:
                            # (within a via's reach alone: the changes kept off, the layers free)
                            nvia += 1
                            brk.append(k_)
                            if run_a or run_b:
                                brk_run.append(k_)
                            lits.append(None)
                            cpl.append((A, a, round(ua, 3), B, b, round(ub, 3), [round(X[0], 3), round(X[1], 3)]))
                            continue
                        La = pa.layer_lits(a, ba_)
                        Lb = pb.layer_lits(b, bb_)
                        for l_ in range(len(pa.LAYN)):          # (each physical layer: each bus's index of it)
                            m.AddBoolOr([La[pa.lix[l_]].Not(), Lb[pb.lix[l_]].Not(), k_])
                        brk.append(k_)
                        if run_a or run_b:
                            brk_run.append(k_)
                        lits.append((pa, La, pb, Lb))
                        cpl.append((A, a, round(ua, 3), B, b, round(ub, 3), [round(X[0], 3), round(X[1], 3)]))
    print(f'  couplings: {len(cpl)} crossings and near misses (within {NEAR:.3f} mm) of two buses\' lanes, '
          f'{nvia} of them within a via\'s reach ({NEAR_V:.3f} mm)')
    # ROOM: each lane's stretches among another bus's lanes (its meetings with them, ROOM either side, joined where
    # they overlap), and each of its changes inside one priced
    cramped = []
    if ROOM > 0:
        for (bus_, n_), us_ in sorted(met.items()):
            p_ = parts[bus_]
            pad_ = p_.QU(p_.VNEED) + p_.QU(ROOM)
            spans = []
            for U_ in sorted(p_.Q(u_) for u_ in us_):
                if spans and U_ - pad_ <= spans[-1][1]:
                    spans[-1][1] = U_ + pad_
                else:
                    spans.append([U_ - pad_, U_ + pad_])
            cs_, act = p_.chg[n_]
            for x_, a_ in zip(cs_, act):
                for lo_, hi_ in spans:
                    bf, af, inn = m.NewBoolVar(''), m.NewBoolVar(''), m.NewBoolVar('')
                    m.Add(x_ <= lo_).OnlyEnforceIf(bf)
                    m.Add(x_ >= hi_).OnlyEnforceIf(af)
                    m.AddBoolOr([bf, af, inn, a_.Not()])
                    cramped.append(inn)
        print(f'  room: a change within {ROOM:g} mm of a place another bus\'s lane meets its lane priced '
              f'{W_ROOM:g} via(s): {len(cramped)} (change, stretch) pairs over {len(met)} lanes')
    OBJ = sum(p.OBJ for p in parts.values()) + W_CROSS * W_V * sum(brk) + \
        (W_RUN_CROSS - W_CROSS) * W_V * sum(brk_run) + int(round(W_ROOM * W_V)) * sum(cramped)
    m.Minimize(OBJ)
    # the buses' own plans laid out whole: every hinted variable held, the couplings' literals completed
    sh = cp_seed.solver()
    sh.parameters.num_workers = 1
    sh.parameters.fix_variables_to_their_hinted_value = True
    sh.parameters.max_deterministic_time = 60.0
    st_h = sh.Solve(m)
    ok = (cp_model.OPTIMAL, cp_model.FEASIBLE)
    if st_h in ok:
        n_b0 = sum(sh.Value(k_) for k_ in brk)
        print(f'  the buses\' own plans side by side: [{sh.StatusName(st_h)}] {sh.ObjectiveValue() / W_V:.1f} '
              f'vias-worth, {n_b0} of {len(brk)} couplings broken')
        n_c0 = sum(sh.Value(c_) for c_ in cramped)
        if cramped:
            print(f'  room: {n_c0} cramped change(s) in the buses\' own plans')
        vals_h = [sh.Value(m.GetIntVarFromProtoIndex(i_)) for i_ in range(len(m.Proto().variables))]
        m.ClearHints()
        for i_, v_ in enumerate(vals_h):
            m.AddHint(m.GetIntVarFromProtoIndex(i_), v_)
    else:
        print(f'  the buses\' own plans side by side: [{sh.StatusName(st_h)}] -- the search from their hints only')
    if st_h in ok and n_b0 == 0 and n_c0 == 0:
        # (the buses' own plans breaking NO coupling: nothing for the coupled solve to settle, and its search would only
        # search each bus's own model again, just solved by that bus -- four such searches, 198 to 264 s each, ended
        # where they began or 2 vias-worth lower, one of them with no couplings at all. Each bus keeps its own plan, as
        # the coupled plan: `own`, nothing written over its solve, and no coupling settled to hold through its loop)
        print(f'  coupled search: none -- the buses\' own plans break no coupling and cramp no change: each bus\'s own '
              'is the coupled plan',
              flush=True)
        json.dump({'round': rnd, 'couplings': len(cpl), 'secs': round(time.time() - t0, 1), 'all': cpl,
                   'broken_alone': [], 'broken': [], 'own': True}, open(os.path.join(rd, 'joint.json'), 'w'), indent=1)
        for bus in sorted(parts):
            json.dump({'bus': bus, 'joint': True, 'own': True, 'cuts': None},
                      open(os.path.join(rd, f'{bus}.joint.json'), 'w'))
        return
    s_ = cp_seed.solver()
    s_.parameters.num_workers = WORKERS
    s_.parameters.subsolvers.extend(ws.FALLBACK)
    s_.parameters.interleave_search = True
    s_.parameters.max_num_deterministic_batches = BATCHES
    s_.parameters.share_glue_clauses = False
    s_.parameters.share_binary_clauses = False
    st = s_.Solve(m)
    secs = time.time() - t0
    if st not in ok:
        print(f'  coupled search: [{s_.StatusName(st)}] {secs:.0f}s -- no plan: every bus keeps its own')
        sv = sh if st_h in ok else None
    else:
        sv = s_
        print(f'  coupled search: [{s_.StatusName(st)}] {secs:.0f}s, {s_.ObjectiveValue() / W_V:.1f} vias-worth '
              f'(bound {s_.BestObjectiveBound() / W_V:.1f}), {sum(s_.Value(k_) for k_ in brk)} of {len(brk)} '
              f'couplings broken')
    if cramped and sv is not None:
        print(f'  room: {sum(sv.Value(c_) for c_ in cramped)} cramped change(s) in the coupled plan')
    rep = {'round': rnd, 'couplings': len(cpl), 'secs': round(secs, 1), 'all': cpl,
           'broken_alone': [c for c, k_ in zip(cpl, brk) if st_h in ok and sh.Value(k_)]}
    for c in rep['broken_alone']:
        print(f'    broken by their own plans: {c[0]}:{c[1]} u {c[2]} x {c[3]}:{c[4]} u {c[5]} at {c[6]}')
    if sv is not None:
        rep['broken'] = [c for c, k_ in zip(cpl, brk) if sv.Value(k_)]
        for c in rep['broken'][:40]:
            print(f'    broken: {c[0]}:{c[1]} u {c[2]} x {c[3]}:{c[4]} u {c[5]} at {c[6]}')
    # each coupling HELD through the round's loop: every lane kept off the layer the other's plan has there, in a box
    # round the crossing -- a firm layer cut of its own bus's (route_bus's loop: SEED_CUTS), in its own frame and
    # layers; the loop's re-solves are each bus's alone, and one took a lane back onto the other's layer (its own model
    # dearer by a via end's run on the joint layer, and its audit, of its own geometry, finding nothing there)
    jc = {bus: [] for bus in parts}
    if sv is not None:
        R_ = float(awx_settings.get('JOINT_CUT_R') or '0.35')
        def phys_of(sol, p_, L_):
            return next((l_ for l_ in range(len(p_.LAYN)) if sol.Value(L_[p_.lix[l_]])), None)
        for (A, a, ua, B, b, ub, X), lt_, k_ in zip(cpl, lits, brk):
            if lt_ is None or sv.Value(k_):
                continue                # (a coupling of vias alone holds no layer: nothing to cut)
            pa, La, pb, Lb = lt_
            phys = {'a': phys_of(sv, pa, La), 'b': phys_of(sv, pb, Lb)}
            # (only the couplings the coupled solve SETTLED: broken in the buses' own plans, or a lane's layer there
            # moved off its own plan's -- elsewhere each bus's own re-solve has no reason to leave the layers it
            # chose, and a cut at every crossing held a bus of 11 lanes to 181 of them, its loop out of room)
            if st_h in ok and not sh.Value(k_) and phys['a'] == phys_of(sh, pa, La) and \
                    phys['b'] == phys_of(sh, pb, Lb):
                continue
            for p_, n_, other, ph in ((pa, a, f'{B}:{b}', phys['b']), (pb, b, f'{A}:{a}', phys['a'])):
                if ph is None:
                    continue
                c_ = p_.mir(*turn(X, p_.q, p_.cx, p_.cy))
                jc[p_.bus].append({'lane': n_, 'island': f'bus {other}', 'layer': 0,
                                   'box': [c_[0] - R_, c_[1] - R_, c_[0] + R_, c_[1] + R_],
                                   'blocked': [p_.lix[ph]], 'firm': True})
    for bus, p in sorted(parts.items()):
        out = p.req['solve']
        J = None
        if sv is not None:
            st_ = st if sv is s_ else st_h
            with awx_settings.given(p.req['env']):
                # (proved where the coupled search proved its whole objective: an unproved plan is kept as the round's
                # first solve keeps one, SOLVE_UNPROVED)
                J = p.finish(sv, st_, st_ == cp_model.OPTIMAL, secs)
        if J is not None:
            if not os.path.isfile(out[:-len('.json')] + '.solo.json'):
                shutil.copyfile(out, out[:-len('.json')] + '.solo.json')
            J['joint'] = {'round': rnd, 'with': sorted(set(parts) - {bus}),
                          'broken': [c for c in rep.get('broken', []) if bus in (c[0], c[3])]}
            json.dump(J, open(out, 'w'), indent=0)
        cf_ = None
        if J is not None and jc.get(bus):
            cf_ = os.path.join(rd, f'{bus}.cuts.json')
            json.dump({'cuts': [], 'vcuts': [], 'lcuts': jc[bus]}, open(cf_, 'w'), indent=0)
        json.dump({'bus': bus, 'joint': J is not None, 'cuts': cf_}, open(os.path.join(rd, f'{bus}.joint.json'), 'w'))
    json.dump(rep, open(os.path.join(rd, 'joint.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
