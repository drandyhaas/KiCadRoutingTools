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
    side = json.load(open(os.path.splitext(bench)[0] + '.plan.json'))
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(bench)
    CY = None
    if int(side.get('chi', 1)) < 0:
        from bga_fanout.flip_frame import mirror_axis
        with contextlib.redirect_stdout(io.StringIO()):
            CY = mirror_axis(pcb)
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
            d.polygon(poly_, fill=(232, 215, 165) if ref in parts else (205, 210, 228))
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
        for nm, Ln in J['lanes'].items():
            xy = Ln.get('xy') or []
            if len(xy) > 1 and nm not in names:
                d.line([M(p) for p in xy], fill=(95, 95, 95), width=2)
        for i, nm in enumerate(names):
            xy = (J['lanes'].get(nm) or {}).get('xy') or []
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
    im.save(out)
    print('frame_render: wrote', out, im.size, 'H0', round(F.H0, 2), 'Hk', {k: round(v, 2) for k, v in F.Hk.items()},
          'rings', list((getattr(F, 'rings', None) or {}).keys()), 'group', G is not None)


if __name__ == '__main__':
    main()
