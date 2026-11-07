#!/usr/bin/env python3
"""snap_render.py WORK ROUND_DIR SNAP.json OUT.png X0,Y0,X1,Y1 --sizes CLR,TRK,FTRK,VIA,DRILL -- a snap's LAID lanes at
track level (whole_snap.py's output, pairs or all): each pair's two legs (its centreline pieces half its pitch either
side, by layer), its dives' two barrels where both pair routers stand them (pairs.dive_barrels at pairs.dive_offset),
its crossover's legs and barrels exactly as laid (`cross`), a single's track; the lanes the snap could not lay
crossed out where they stopped; with `--plan` the plan's centrelines dashed (`--lanes` in colour); and the bench's
copper beneath, all in the plan's frame (mirrored where the round's chirality is -1).

What it is for: a lane the snap failed seen against what was laid before it -- a crossover's barrels on the next
pair's planned line, a dive with no room beside it. Sizes as the run's (route_bus's), so the pitch and the barrels
are the router's own."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import argparse
import contextlib
import io
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))


def main():
    import replay_stage
    from frame_render import _font
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    replay_stage.add_round_args(ap)
    ap.add_argument('snap', help="a snap's output (whole_snap.py OUT, e.g. loop/pairsN.json)")
    ap.add_argument('out', help='the PNG to write')
    ap.add_argument('view', metavar='X0,Y0,X1,Y1', help='the window drawn, mm, in the plan\'s frame')
    ap.add_argument('--plan', default=None, help='the plan the snap laid (its centrelines dashed)')
    ap.add_argument('--lanes', default='', help='the lanes drawn in colour, comma separated')
    ap.add_argument('--bench', default=None, help="the bench (default ROUND_DIR/fo.kicad_pcb)")
    a = ap.parse_args()
    snapf, out = os.path.abspath(a.snap), os.path.abspath(a.out)
    planf = os.path.abspath(a.plan) if a.plan else None
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
        import pairs as _pairs
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            ctx, _cs = whole_ctx.plan()
        cfg = ctx.cfg
        TW, VS = cfg.track_width, cfg.via_size
        HALF = _pairs.pitch(TW) / 2
        OFF = _pairs.dive_offset(cfg, HALF)
        side = json.load(open(os.path.splitext(bench)[0] + '.plan.json'))
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(bench)
        CY = None
        if int(side.get('chi', 1)) < 0:
            from bga_fanout.flip_frame import mirror_axis
            with contextlib.redirect_stdout(io.StringIO()):
                CY = mirror_axis(pcb)
        my = (lambda y: 2 * CY - y) if CY is not None else (lambda y: y)
        W, TOP = 2200, 110
        s = W / (x1 - x0)
        H = int((y1 - y0) * s) + TOP
        im = Image.new('RGB', (W, H), (252, 252, 248))
        d = ImageDraw.Draw(im, 'RGBA')
        P = lambda x, y: ((x - x0) * s, TOP + (y - y0) * s)
        LC = {'F.Cu': (200, 60, 60), 'B.Cu': (50, 90, 210), 'In1.Cu': (200, 150, 0), 'In2.Cu': (40, 160, 70),
              'In3.Cu': (150, 70, 190), 'In4.Cu': (0, 160, 170), 'In5.Cu': (210, 110, 30), 'In6.Cu': (120, 120, 120)}
        F_, Fs = _font(26), _font(20)
        inside = lambda x, y, m=1.0: x0 - m < x < x1 + m and y0 - m < y < y1 + m
        # the bench's copper, faint
        for fp in pcb.footprints.values():
            for pd in fp.pads:
                if not any(L.endswith('.Cu') for L in pd.layers):
                    continue
                x, y = pd.global_x, my(pd.global_y)
                if inside(x, y):
                    ya, yb = sorted((y - pd.size_y / 2, y + pd.size_y / 2))
                    d.rectangle([P(x - pd.size_x / 2, ya), P(x + pd.size_x / 2, yb)], fill=(205, 205, 225, 255))
        for sg in pcb.segments:
            if inside(sg.start_x, my(sg.start_y)) or inside(sg.end_x, my(sg.end_y)):
                d.line([P(sg.start_x, my(sg.start_y)), P(sg.end_x, my(sg.end_y))],
                       fill=LC.get(sg.layer, (150, 150, 150)) + (90,), width=max(1, int(sg.width * s)))
        for v in pcb.vias:
            if inside(v.x, my(v.y)):
                cx, cy = P(v.x, my(v.y))
                r = v.size / 2 * s
                d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(90, 90, 90, 160), width=3)
        # the plan's centrelines, dashed
        pal = [(220, 0, 0), (0, 70, 230), (0, 150, 0), (230, 120, 0), (150, 0, 170)]
        col = {n: pal[i % len(pal)] for i, n in enumerate(names)}
        if planf:
            for n, L in json.load(open(planf))['lanes'].items():
                pts = [P(*p_[-2:]) for p_ in L.get('xy') or []]
                c = col.get(n, (60, 60, 60))
                for p_, q_ in zip(pts, pts[1:]):
                    k = max(1, int(math.hypot(q_[0] - p_[0], q_[1] - p_[1]) / 14))
                    for i in range(0, k, 2):
                        d.line([(p_[0] + (q_[0] - p_[0]) * i / k, p_[1] + (q_[1] - p_[1]) * i / k),
                                (p_[0] + (q_[0] - p_[0]) * (i + 1) / k, p_[1] + (q_[1] - p_[1]) * (i + 1) / k)],
                               fill=c + (255,), width=3)
        # the snap's laid lanes
        snap = json.load(open(snapf))
        prs = getattr(ctx, 'pairs', {}) or {}
        tw = max(2, int(TW * s))

        def barrel(x, y, colour, width):
            cx, cy = P(x, y)
            r = VS / 2 * s
            d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=colour, width=width)
        for n, L in snap['lanes'].items():
            pair = n in prs
            cr = L.get('cross') or {}
            xs = cr.get('poses') or []
            cl = [((ax, ay), (bx, by), lay) for ax, ay, bx, by, lay in L['pieces']]
            for (ax, ay), (bx, by), lay in cl:
                if len(xs) == 2:
                    (p0x, p0y), (p1x, p1y) = xs
                    btw = lambda x, y: (min(p0x, p1x) - 1e-6 <= x <= max(p0x, p1x) + 1e-6
                                        and min(p0y, p1y) - 1e-6 <= y <= max(p0y, p1y) + 1e-6)
                    if btw(ax, ay) and btw(bx, by):
                        continue          # (the crossover's own legs, drawn exactly below)
                c = LC.get(lay, (120, 120, 120)) + (210,)
                ln = math.hypot(bx - ax, by - ay)
                if not pair or ln < 1e-9:
                    d.line([P(ax, ay), P(bx, by)], fill=c, width=tw)
                    continue
                nx, ny = -(by - ay) / ln * HALF, (bx - ax) / ln * HALF
                for sg_ in (1, -1):
                    d.line([P(ax + sg_ * nx, ay + sg_ * ny), P(bx + sg_ * nx, by + sg_ * ny)], fill=c, width=tw)
            for v in L.get('vias') or []:
                if cr.get('at') and math.hypot(v[0] - cr['at'][0], v[1] - cr['at'][1]) < 0.05:
                    continue              # (the crossover: its barrels drawn exactly below)
                for bx_, by_ in (_pairs.dive_barrels((v[0], v[1]), cl, OFF) if pair else [(v[0], v[1])]):
                    barrel(bx_, by_, (0, 0, 0, 255), 4)
            if cr:
                for _k, legs in (cr.get('legs') or {}).items():
                    for pts, lay in legs:
                        d.line([P(*p_) for p_ in pts], fill=LC.get(lay, (120, 120, 120)) + (255,), width=tw)
                for vx, vy, _k in cr.get('vias') or []:
                    barrel(vx, vy, (220, 0, 0, 255), 6)
            if L['pieces'] and inside(L['pieces'][0][0], L['pieces'][0][1], 0):
                d.text(P(L['pieces'][0][0], L['pieces'][0][1]), n, fill=col.get(n, (0, 0, 0)), font=Fs)
        for f_ in snap.get('conflicts') or []:
            cx, cy = P(*f_['xy'][:2])
            d.line([cx - 18, cy - 18, cx + 18, cy + 18], fill=(255, 0, 0), width=5)
            d.line([cx - 18, cy + 18, cx + 18, cy - 18], fill=(255, 0, 0), width=5)
        d.rectangle([0, 0, W, TOP - 8], fill=(252, 252, 248))
        d.text((12, 8), f'{os.path.basename(snapf)}: laid lanes (pair legs by layer, dive barrels black, crossover '
               f'barrels red), plan dashed', fill=(0, 0, 0), font=F_)
        failed = [c_['lanes'][0] for c_ in snap.get('conflicts') or []]
        d.text((12, 44), 'laid: ' + ', '.join(snap['lanes']) + (('   NOT LAID: ' + ', '.join(failed)) if failed else ''),
               fill=(40, 40, 40), font=Fs)
        x_ = 12
        for L_, c in LC.items():
            d.rectangle([x_, 78, x_ + 30, 92], fill=c)
            d.text((x_ + 36, 74), L_, fill=(0, 0, 0), font=Fs)
            x_ += 150
        im.save(out)
    print('snap_render: wrote', out, im.size)


if __name__ == '__main__':
    main()
