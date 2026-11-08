#!/usr/bin/env python3
"""frame_render.py WORK ROUND_DIR OUT.png X0,Y0,X1,Y1 --sizes CLR,TRK,FTRK,VIA,DRILL -- a bus's recorded round drawn
with the whole route's FRAME on it, as the solve builds it (whole_frame.build on the round's bench): the trunk spine
(grey), the arrival line H0 (black), each ring's spine (dashed) with its start and its handoff line at Hk, each ring
lane's handoff point (a dot, labelled), a GROUP's outline and front (purple); and with `--plan` a plan's lanes (those
`--lanes` names in colour and labelled at their start, the rest dark grey). The bench's copper drawn beneath: pads
(the bus's parts' tan), tracks by layer, vias. Frame and plan points are in the plan's frame, mirrored back to the
board where the round's chirality is -1.

What it is for: a contorted planned path checked against its frame first -- a ring stacked off the far side of its
part, a handoff out of order, a group's front where it should not be. WORK is the bus's work dir (route_bus's
BUS.bus); ROUND_DIR a round in it (run/rN); `--bench` another bench than its fo.kicad_pcb (a loop round's benchN)."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import argparse
import collections
import contextlib
import io
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))


def _font(size, bold=False):
    from PIL import ImageFont
    for f_ in (('/System/Library/Fonts/Supplemental/Arial Bold.ttf' if bold else
                '/System/Library/Fonts/Supplemental/Arial.ttf'), 'DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(f_, size)
        except OSError:
            pass
    return None


def main():
    import replay_stage
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    replay_stage.add_round_args(ap)
    ap.add_argument('out', help='the PNG to write')
    ap.add_argument('view', metavar='X0,Y0,X1,Y1', help='the board window drawn, mm')
    ap.add_argument('--plan', default=None, help='a plan to draw: a solve, a geometry or a polish (its lanes\' xy)')
    ap.add_argument('--lanes', default='', help='the lanes drawn in colour, comma separated')
    ap.add_argument('--bench', default=None, help="the bench (default ROUND_DIR/fo.kicad_pcb)")
    ap.add_argument('--passages', default=None, metavar='LAYER[,LAYER..]|all',
                    help="the PASSAGES (whole_passages.survey): each column's free runs on the layer (board names) "
                         "drawn across the frames, by how full the plan's lanes and vias make them -- grey empty, "
                         "green and orange filling, red over -- one picture per layer, OUT with _LAYER added. The "
                         "lanes stand where --plan laid them (a geometry), else at their references with --solve's "
                         "(or --plan's) layers; a via with no room is a red cross")
    ap.add_argument('--solve', default=None, help="the solve --plan's geometry was laid from (its layers and changes)")
    ap.add_argument('--cuts', default=None, metavar='FILE[,FILE..]',
                    help="the loop's cut files in round order (loop/cN.json: round N's new cuts), drawn labelled rN: "
                         "an island cut its island's box, a via cut a cross at the lane's change, a layer cut its box")
    ap.add_argument('--step', type=float, default=0.25, help="the passages' columns apart along each frame, mm")
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    plan = os.path.abspath(a.plan) if a.plan else None
    names = [n for n in a.lanes.split(',') if n]
    x0, y0, x1, y1 = map(float, a.view.split(','))
    env = replay_stage.round_env(a.work, a.round_dir, a.sizes, a.dest, a.layers, a.set)
    if a.bench:
        env['BENCH'] = os.path.abspath(a.bench)
    bench = env['BENCH']
    os.chdir(HERE)
    os.environ.update(env)
    from PIL import Image, ImageDraw
    from kicad_parser import parse_kicad_pcb
    import awx_settings
    with awx_settings.given({**awx_settings.environ(), **env}):
        import whole_ctx
        import whole_frame as wf
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            ctx, _cs = whole_ctx.plan()
            F = wf.build(ctx, env['DEST'])
        SV = None
        if a.passages or a.cuts:
            import route_layers
            import whole_passages as wp
            J_ = json.load(open(plan)) if plan else None
            # (a geometry's or a polish's lanes carry their pieces; a solve's plan its layers)
            G_ = J_ if J_ and any((v_ or {}).get('pieces') for v_ in (J_.get('lanes') or {}).values()) else None
            S_ = json.load(open(os.path.abspath(a.solve))) if a.solve else (J_ if J_ and G_ is None else None)
            LAYN = route_layers.layers()
            with contextlib.redirect_stdout(io.StringIO()):
                SV = wp.survey(ctx, F, LAYN, S_, G_, a.step)
    side = json.load(open(os.path.splitext(bench)[0] + '.plan.json'))
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(bench)
    CY = None
    if int(side.get('chi', 1)) < 0:
        from bga_fanout.flip_frame import mirror_axis
        with contextlib.redirect_stdout(io.StringIO()):
            CY = mirror_axis(pcb)
    swap = (lambda L: {'F.Cu': 'B.Cu', 'B.Cu': 'F.Cu'}.get(L, L)) if CY is not None else (lambda L: L)
    passl = []
    if a.passages:
        passl = list(LAYN) if a.passages == 'all' else [swap(L) for L in a.passages.split(',')]   # (frame names)
    RCOL = [(200, 0, 160), (0, 150, 170), (150, 90, 20), (90, 90, 220), (40, 140, 40)]

    def render(out, PL=None):
        W, TOP = 2200, 120
        s = W / (x1 - x0)
        H = int((y1 - y0) * s) + TOP
        im = Image.new('RGB', (W, H), (252, 252, 248))
        d = ImageDraw.Draw(im)
        B = lambda x, y: ((x - x0) * s, (y - y0) * s + TOP)                                     # a board point
        M = lambda p: B(float(p[0]), (2 * CY - float(p[1])) if CY is not None else float(p[1]))  # a frame or plan point
        f22, f30 = _font(22), _font(30, bold=True)
        LC = {'F.Cu': (225, 175, 175), 'B.Cu': (175, 185, 225), 'In1.Cu': (190, 215, 190), 'In2.Cu': (170, 210, 170),
              'In3.Cu': (205, 195, 160), 'In4.Cu': (200, 180, 210), 'In5.Cu': (170, 205, 210), 'In6.Cu': (210, 200, 180)}
        parts = set([F.src] + list(F.dests)) if hasattr(F, 'src') else set()
        for ref, fp in pcb.footprints.items():
            for p in fp.pads:
                if not any(L.endswith('.Cu') for L in p.layers):
                    continue
                # (a tilted pad drawn turned: its own size about its centre by its rect_rotation, as check_drc)
                t_ = math.radians(getattr(p, 'rect_rotation', 0.0) or 0.0)
                c_, s_ = math.cos(t_), math.sin(t_)
                hx_, hy_ = p.size_x / 2, p.size_y / 2
                poly_ = [B(p.global_x + lx * c_ - ly * s_, p.global_y + lx * s_ + ly * c_)
                         for lx, ly in ((-hx_, -hy_), (hx_, -hy_), (hx_, hy_), (-hx_, hy_))]
                if max(q[0] for q in poly_) < 0 or min(q[0] for q in poly_) > W or max(q[1] for q in poly_) < TOP \
                        or min(q[1] for q in poly_) > H:
                    continue
                on_ = PL is None or '*.Cu' in p.layers or (p.drill or 0) > 0 or swap(PL) in p.layers
                d.polygon(poly_, fill=((232, 215, 165) if ref in parts else (205, 210, 228)) if on_ else
                          ((246, 240, 225) if ref in parts else (236, 238, 244)))
        for sg in pcb.segments:
            d.line([B(sg.start_x, sg.start_y), B(sg.end_x, sg.end_y)], fill=LC.get(sg.layer, (200, 200, 200)),
                   width=max(1, int(sg.width * s)))
        for v in pcb.vias:
            q = B(v.x, v.y)
            r = v.size / 2 * s
            d.ellipse([q[0] - r, q[1] - r, q[0] + r, q[1] + r], outline=(110, 110, 110), width=2)

        def dashed(pts, col, w=4, step=22):
            for i in range(len(pts) - 1):
                p_, q_ = pts[i], pts[i + 1]
                n_ = max(1, int(math.hypot(q_[0] - p_[0], q_[1] - p_[1]) / step))
                for k in range(0, n_, 2):
                    d.line([(p_[0] + (q_[0] - p_[0]) * k / n_, p_[1] + (q_[1] - p_[1]) * k / n_),
                            (p_[0] + (q_[0] - p_[0]) * (k + 1) / n_, p_[1] + (q_[1] - p_[1]) * (k + 1) / n_)],
                           fill=col, width=w)

        # the trunk spine and the arrival line
        L_ = getattr(F.spine, 'L', 0) or getattr(F.spine, 'len', 0) or (F.H0 + 15)
        L_ = float(L_() if callable(L_) else L_)
        d.line([M(F.spine.xy(u_ / 4.0, 0.0)) for u_ in range(0, int(L_ * 4) + 1)], fill=(140, 140, 140), width=3)
        d.line([M(F.spine.xy(F.H0, -9.0)), M(F.spine.xy(F.H0, 9.0))], fill=(0, 0, 0), width=2)
        q = M(F.spine.xy(F.H0, 9.0))
        d.text((q[0] + 6, q[1]), 'arrival H0', fill=(0, 0, 0), font=f22)
        # a group's outline and front
        G = getattr(F, 'group', None)
        if G is not None:
            pts = [M(G.pt(i_ * G.L / 200.0)) for i_ in range(201)]
            fr_ = [M(G.pt(G.f0 + i_ * ((G.f1 - G.f0) % G.L) / 60.0)) for i_ in range(61)]
            d.line(pts, fill=(130, 50, 170), width=3)
            d.line(fr_, fill=(130, 50, 170), width=9)
            d.text((fr_[0][0] + 10, fr_[0][1] - 30), 'group front', fill=(130, 50, 170), font=f22)
        # the rings: spine, start, handoff line and each ring lane's handoff point
        RC = {'N': (40, 110, 200), 'S': (220, 120, 20)}
        for k, rg in (getattr(F, 'rings', None) or {}).items():
            col = RC.get(k, (90, 90, 90))
            pts = [M(p) for p in rg.pts]
            if len(pts) > 1:
                dashed(pts, col)
                d.ellipse([pts[0][0] - 12, pts[0][1] - 12, pts[0][0] + 12, pts[0][1] + 12], outline=col, width=4)
                d.text((pts[0][0] + 14, pts[0][1] - 26), f'ring {k} start', fill=col, font=f22)
            hk = F.Hk.get(k)
            if hk is not None:
                d.line([M(F.spine.xy(hk, -9.0)), M(F.spine.xy(hk, 9.0))], fill=col, width=1)
                for n, oh in F.o_h.items():
                    if F.cls.get(n) == k:
                        qq = M(F.spine.xy(hk, oh))
                        d.ellipse([qq[0] - 7, qq[1] - 7, qq[0] + 7, qq[1] + 7], fill=col)
                        d.text((qq[0] + 9, qq[1] - 10), n, fill=col, font=f22)
        # a plan's lanes
        COL = [(220, 30, 30), (30, 80, 220), (20, 150, 50), (240, 130, 0), (140, 40, 200), (0, 170, 190)]
        if plan:
            J = json.load(open(plan))
            if 'lanes' not in J and SV is not None:
                # (a solve's plan has no xy: its lanes drawn along their references, each run in its layer's colour --
                # dark on the passages' layer)
                for nm, R_ in SV.REF.items():
                    if len(R_) < 2:
                        continue
                    ch_ = sorted(J['changes'].get(nm, []))
                    for i_ in range(len(R_) - 1):
                        Lr_ = J['layers'][nm][sum(1 for c_ in ch_ if c_ <= R_[i_, 0])]
                        on_ = PL is None or Lr_ == PL
                        c_ = (COL[names.index(nm) % len(COL)] if nm in names else (60, 60, 60)) if on_ else (205, 205, 205)
                        d.line([M(R_[i_, 1:]), M(R_[i_ + 1, 1:])], fill=c_, width=(5 if nm in names else 3) if on_ else 1)
                    for cu_ in ch_:
                        q_ = M(wp.ref_at(R_, cu_))
                        d.ellipse([q_[0] - 7, q_[1] - 7, q_[0] + 7, q_[1] + 7], outline=(60, 60, 60), width=2)
                for i, nm in enumerate(names):
                    if nm in SV.REF and len(SV.REF[nm]):
                        q = M(SV.REF[nm][0, 1:])
                        d.text((q[0] + 8, q[1] - 28), nm + ' s', fill=COL[i % len(COL)], font=f22)
                J = {'lanes': {}}
            for nm, Ln in J['lanes'].items():
                xy = Ln.get('xy') or []
                if PL is not None and Ln.get('pieces'):
                    # (a layer's picture: the lanes ON it dark, the rest faint)
                    for (ax_, ay_, bx_, by_, Lp_) in Ln['pieces']:
                        d.line([M((ax_, ay_)), M((bx_, by_))], fill=(60, 60, 60) if Lp_ == PL else (200, 200, 200),
                               width=3 if Lp_ == PL else 1)
                    continue
                if len(xy) > 1 and nm not in names:
                    d.line([M(p) for p in xy], fill=(95, 95, 95), width=2)
            for i, nm in enumerate(names):
                xy = (J['lanes'].get(nm) or {}).get('xy') or []
                if PL is not None and (J['lanes'].get(nm) or {}).get('pieces'):
                    c = COL[i % len(COL)]
                    for (ax_, ay_, bx_, by_, Lp_) in J['lanes'][nm]['pieces']:
                        d.line([M((ax_, ay_)), M((bx_, by_))], fill=c, width=6 if Lp_ == PL else 2)
                    q = M(xy[0])
                    d.text((q[0] + 8, q[1] - 28), nm + ' s', fill=c, font=f22)
                    continue
                if len(xy) > 1:
                    c = COL[i % len(COL)]
                    d.line([M(p) for p in xy], fill=c, width=6)
                    q = M(xy[0])
                    d.text((q[0] + 8, q[1] - 28), nm + ' s', fill=c, font=f22)
        d.rectangle([0, 0, W, TOP - 8], fill=(252, 252, 248))
        d.text((12, 10), 'the frame (trunk grey, arrival black, rings dashed + starts, handoffs dots'
               + (', group purple' if G is not None else '') + ')' + (f'; plan {os.path.basename(plan)}' if plan else ''),
               fill=(0, 0, 0), font=f30)
        d.text((12, 55), 'lanes: ' + (', '.join(names) or 'none named')
               + ('   [frame mirrored back: chirality -1]' if CY is not None else ''), fill=(60, 60, 60), font=f22)

        # (the passages under the lanes would be hidden: drawn after them, thin, the lanes still read)
        if PL is not None:
            groups = []
            for r_ in SV.records:
                fk_, s__, L__, run_, ls_, need_, room_, pa_, pb_, mid_, vs_ = r_
                if L__ != PL:
                    continue
                if not ls_ and not vs_:
                    d.line([M(pa_), M(pb_)], fill=(215, 215, 215), width=1)
                    continue
                f_ = need_ / room_
                c_ = (220, 30, 30) if f_ > 1 + 1e-9 else (240, 150, 30) if f_ > 0.6 else (90, 180, 90)
                d.line([M(pa_), M(pb_)], fill=c_, width=5 if f_ > 1 + 1e-9 else 2)
                if f_ > 1 + 1e-9:
                    key_ = (fk_, run_[2], run_[3], tuple(ls_), tuple(vs_))
                    if groups and groups[-1][0] == key_ and s__ - groups[-1][1] <= a.step + 1e-6:
                        groups[-1][1] = s__
                    else:
                        groups.append([key_, s__, mid_, need_, room_])
            f18 = _font(18, bold=True)
            for key_, _s, mid_, need_, room_ in groups:
                q_ = M(mid_)
                txt_ = (f'{key_[1][:10]}/{key_[2][:10]} {need_ / SV.C.PMIN:.1f}>{room_ / SV.C.PMIN:.1f}: '
                        + ' '.join(n.replace('RAM_', '') for n in key_[3])
                        + (' v:' + ' '.join(n.replace('RAM_', '') for n in key_[4]) if key_[4] else ''))
                d.text((q_[0] + 10, q_[1] - 10), txt_, fill=(200, 20, 20), font=f18)
        if SV is not None:
            for n_, _cu, vx_, vy_, why_ in SV.vias:
                q_ = M((vx_, vy_))
                d.line([(q_[0] - 11, q_[1] - 11), (q_[0] + 11, q_[1] + 11)], fill=(220, 0, 0), width=4)
                d.line([(q_[0] - 11, q_[1] + 11), (q_[0] + 11, q_[1] - 11)], fill=(220, 0, 0), width=4)
                d.text((q_[0] + 12, q_[1] + 4), n_.replace('RAM_', '') + ' no via room', fill=(220, 0, 0),
                       font=_font(16))
        # each round's cuts: where the loop held a lane off an island, moved a change, took a layer away
        if a.cuts:
            f20b = _font(20, bold=True)
            for ri_, fn_ in enumerate(a.cuts.split(',')):
                cj_ = json.load(open(os.path.abspath(fn_)))
                rc_ = RCOL[ri_ % len(RCOL)]
                tag_ = f'r{ri_ + 1}'
                isl_ = collections.defaultdict(list)
                for c_ in cj_.get('cuts', []):
                    isl_[c_['island']].append(c_['lane'].replace('RAM_', ''))
                for lab_, ls_ in isl_.items():
                    bxs_ = [b_ for (l_, _ls), b_ in SV.C.islands.items() if l_ == lab_]
                    for b_ in bxs_:
                        p0_, p1_ = M((b_[0], b_[1])), M((b_[2], b_[3]))
                        d.rectangle([min(p0_[0], p1_[0]) - 3, min(p0_[1], p1_[1]) - 3, max(p0_[0], p1_[0]) + 3,
                                     max(p0_[1], p1_[1]) + 3], outline=rc_, width=3)
                    if bxs_:
                        b_ = bxs_[0]
                        p0_, p1_ = M((b_[0], b_[1])), M((b_[2], b_[3]))
                        tx_ = min(max(min(p0_[0], p1_[0]), 10), W - 600)
                        d.text((tx_, max(p0_[1], p1_[1]) + 8 + 22 * ri_),
                               f'{tag_} held off island {lab_[:16]}: ' + ' '.join(sorted(set(ls_))), fill=rc_, font=f20b)
                for c_ in cj_.get('vcuts', []):
                    n_ = c_['lane']
                    if n_ not in SV.REF:
                        continue
                    q_ = M(wp.ref_at(SV.REF[n_], float(c_['u'])))
                    d.ellipse([q_[0] - 9, q_[1] - 9, q_[0] + 9, q_[1] + 9], outline=rc_, width=4)
                    d.text((q_[0] + 10, q_[1] - 20), f'{tag_} via {n_.replace("RAM_", "")}', fill=rc_, font=_font(16, bold=True))
                lcb_ = collections.defaultdict(list)
                for c_ in cj_.get('lcuts', []):
                    ln_ = ','.join(swap(LAYN[i_]) for i_ in c_.get('blocked', []) if i_ < len(LAYN))
                    lcb_[(tuple(round(v_, 2) for v_ in c_['box']), c_['island'], ln_)].append(c_['lane'].replace('RAM_', ''))
                for (b_, lab_, ln_), ls_ in lcb_.items():
                    p0_, p1_ = M((b_[0], b_[1])), M((b_[2], b_[3]))
                    d.rectangle([min(p0_[0], p1_[0]), min(p0_[1], p1_[1]), max(p0_[0], p1_[0]), max(p0_[1], p1_[1])],
                                outline=rc_, width=5)
                    d.text((max(p0_[0], p1_[0]) + 6, min(p0_[1], p1_[1]) + 22 * ri_),
                           f'{tag_} off {ln_} at {lab_[:14]}: ' + ' '.join(sorted(set(ls_))), fill=rc_, font=f20b)
                for c_ in cj_.get('ccuts', []):
                    ls_ = [n_ for n_ in c_.get('lanes', []) if n_ in SV.REF]
                    if not ls_:
                        continue
                    q_ = M(wp.ref_at(SV.REF[ls_[0]], (float(c_['u_lo']) + float(c_['u_hi'])) / 2))
                    d.text((q_[0], q_[1]), f'{tag_} room {swap(c_["layer"])}: at most {c_["cap"]} of '
                           + ' '.join(n_.replace('RAM_', '') for n_ in ls_), fill=rc_, font=f20b)
        d.rectangle([0, 0, W, TOP - 8], fill=(252, 252, 248))
        d.text((12, 10), 'the frame (trunk grey, arrival black, rings dashed + starts, handoffs dots'
               + (', group purple' if G is not None else '') + ')' + (f'; plan {os.path.basename(plan)}' if plan else '')
               + (f'; PASSAGES {swap(PL)}' if PL is not None else ''), fill=(0, 0, 0), font=f30)
        d.text((12, 55), 'lanes: ' + (', '.join(names) or 'none named')
               + ('   [frame mirrored back: chirality -1]' if CY is not None else '')
               + ('   runs: grey empty, green < 60% full, orange, RED OVER (lanes and vias, each via its room on '
                  'every layer)' if PL is not None else '')
               + (f'   cuts: ' + ', '.join(f'r{i_ + 1} {os.path.basename(f_)}' for i_, f_ in enumerate(a.cuts.split(',')))
                  if a.cuts else ''), fill=(60, 60, 60), font=f22)
        im.save(out)
        print('frame_render: wrote', out, im.size, 'H0', round(F.H0, 2), 'Hk', {k: round(v, 2) for k, v in F.Hk.items()},
              'rings', list((getattr(F, 'rings', None) or {}).keys()), 'group', G is not None)
    if passl:
        for PL in passl:
            render(os.path.splitext(out)[0] + '_' + swap(PL).replace('.', '') + '.png', PL)
    else:
        render(out)


if __name__ == '__main__':
    main()
