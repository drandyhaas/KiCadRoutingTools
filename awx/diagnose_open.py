#!/usr/bin/env python3
"""diagnose_open.py -- why each open net of a braided board is open, in
the router's own terms, with a picture per net.

For every open net (the braid's refusals dump next to the board, else
every run net without a lane) it asks negotiate.Negotiator.classify:

  MISSED   routes now, wider window / bigger budget
  WALLED   fails with EVERY other lane removed: boxed by static copper
           (the fanout's own stubs, pads) -- names the copper at the
           stuck cell; a fanout matter, not a braid one
  BLOCKED  routes through other lanes when they are priced instead of
           blocked -- names the lanes, ranked by how much of the path
           they carry (the minimal set a rip-up must move)

With --negotiate the blocker-directed rip-up runs (top-k victims, the
victims re-laid, a strict improvement kept, PathFinder-style history),
and --apply OUT writes the negotiated board (sibling .kicad_pro
carried) for grade_k.

Renders (--png-dir): one PNG per open net, cropped round its chord --
the board's copper as it stands, the discovered path in magenta, the
blockers' lanes in orange, the tooth (green ring) and berth (cyan
ring) -- and an overview of the whole run with every open chord.

usage: diagnose_open.py FO.kicad_pcb BRAIDED.kicad_pcb K
       [--dest DU1] [--nets A,B] [--negotiate] [--apply OUT]
       [--png-dir DIR] [--victims 4] [--rounds 3]
"""
import argparse
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402
import negotiate as ng  # noqa: E402
import surgical as sg  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('fo')
ap.add_argument('braided')
ap.add_argument('k')
ap.add_argument('--dest', default='DU1')
ap.add_argument('--nets', default='', help='only these open nets')
ap.add_argument('--negotiate', action='store_true')
ap.add_argument('--apply', default='', help='write the negotiated board')
ap.add_argument('--png-dir', default='')
ap.add_argument('--victims', type=int, default=4)
ap.add_argument('--rounds', type=int, default=3)
ap.add_argument('--size', type=int, default=1400)
ap.add_argument('--reberth', action='store_true', help='let the negotiation move BERTHS (relay) as well as lanes')
a = ap.parse_args()

names = sg.k_nets(a.k).split(',')
ctx, groups = te.setup(a.fo, names, a.dest, lambda *x: None)
pcb_fo = ctx.pcb
pcb_b = parse_kicad_pcb(a.braided)
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb_b.nets.items()}
nids = {byname[nm][0] for nm in names}
fo_c, cur_c, rest = ng.split_copper(pcb_fo, pcb_b, nids)
fo = {nm: fo_c[byname[nm][0]] for nm in names}
state = {nm: cur_c[byname[nm][0]] for nm in names}
orig = dict(state)
ends = {nm: (ctx.ends[nm][0], ctx.tooth_layer[nm],
             ctx.ends[nm][1], ctx.dest_layer[nm]) for nm in names}
N = ng.Negotiator(pcb_b, ctx.cfg, byname, ends, fo, state, rest,
                  window_pts={nm: [ctx.ends[nm][0], ctx.ends[nm][1]]
                              for nm in names},
                  b_alts=ctx.dest_alts, chains=ctx.dest_chain,
                  board_path=a.fo, dst=a.dest, reberth=a.reberth)

rp = a.braided.rsplit('.', 1)[0] + '_refusals.json'
if os.path.exists(rp):
    open_ = sorted(json.load(open(rp)))
else:
    open_ = sorted(nm for nm in names if not N.routed(nm))
if a.nets:
    open_ = [n for n in open_ if n in a.nets.split(',')]
print(f'{os.path.basename(a.braided)}: {N.n_routed()}/{len(names)} routed, '
      f'{N.n_vias()} vias; open {open_}')

# ---- classify --------------------------------------------------------
stories = {}
for nm in open_:
    kind, path, blk = N.classify(nm)
    stories[nm] = (kind, path, blk)
    detail = ''
    if kind == 'WALLED':
        detail = f'boxed by {dict(blk) if blk else "static copper (unnamed)"}'
    elif kind == 'BLOCKED':
        detail = (f'{len(path[1])}-via path through '
                  + ', '.join(f'{o}x{c}' for o, c in blk.most_common()))
    elif kind == 'MISSED':
        detail = f'routes now ({len(path[1])} via)'
    (ax, ay), al, (bx, by), bl = ends[nm]
    print(f'  {nm:7s} {kind:8s} {detail}   tooth ({ax:.2f},{ay:.2f}){al[0]}'
          f' berth ({bx:.2f},{by:.2f}){bl[0]}')


# ---- render ----------------------------------------------------------
def render(png_dir):
    from route_render import BoardRenderer, load_font
    os.makedirs(png_dir, exist_ok=True)
    ORANGE, MAGENTA, GREEN, CYAN = ((255, 140, 0), (255, 0, 255),
                                    (0, 220, 0), (0, 220, 255))

    def ring(d, r, p, col, rad_mm=0.45, w=3):
        x, y = r.tf.pt(*p)
        rr = r.tf.length(rad_mm)
        d.ellipse([x - rr, y - rr, x + rr, y + rr], outline=col, width=w)

    def text(d, r, p, s, col):
        x, y = r.tf.pt(*p)
        d.text((x + 6, y - 6), s, fill=col, font=load_font(26))

    stem = os.path.basename(a.braided).rsplit('.', 1)[0]
    for nm in open_:
        kind, path, blk = stories[nm]
        (ax, ay), al, (bx, by), bl = ends[nm]
        xs, ys = [ax, bx], [ay, by]
        if path:
            xs += [v for s in path[0] for v in (s.start_x, s.end_x)]
            ys += [v for s in path[0] for v in (s.start_y, s.end_y)]
        pad = 1.5
        view = (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)
        r = BoardRenderer(pcb_b, size=a.size, supersample=2, layer_alpha=150,
                          view=view)
        hs, hv = [], []
        for om in (blk or {}):
            if om in state:
                s_, v_ = N.lane(om)
                hs += s_
                hv += v_

        def ov(d, r, nm=nm, path=path, blk=blk, kind=kind):
            if path:
                r._draw_segments(d, path[0], color=MAGENTA)
                r._draw_vias(d, path[1], color=MAGENTA)
            ring(d, r, (ax, ay), GREEN)
            ring(d, r, (bx, by), CYAN)
            text(d, r, (ax, ay), f'{nm} tooth {al[0]}', GREEN)
            text(d, r, (bx, by), f'{nm} berth {bl[0]}', CYAN)
            for om, c in (blk or {}).items():
                s_, v_ = N.lane(om) if om in state else ([], [])
                if s_:
                    m = s_[len(s_) // 2]
                    text(d, r, ((m.start_x + m.end_x) / 2,
                                (m.start_y + m.end_y) / 2), f'{om} x{c}', ORANGE)
        lab = f'{nm}: {kind}' + (f' by {", ".join(blk)}' if blk else '')
        img = r.frame(highlight_segments=hs, highlight_vias=hv,
                      highlight_color=ORANGE, label=lab, overlays=[ov])
        img.save(os.path.join(png_dir, f'{stem}_{nm}.png'))
    # overview: every open chord
    r = BoardRenderer(pcb_b, size=a.size, supersample=2, layer_alpha=150)

    def ov_all(d, r):
        for nm in open_:
            (ax, ay), al, (bx, by), bl = ends[nm]
            kind = stories[nm][0]
            col = {'WALLED': (255, 60, 60), 'BLOCKED': MAGENTA,
                   'MISSED': GREEN}.get(kind, (200, 200, 200))
            x0, y0 = r.tf.pt(ax, ay)
            x1, y1 = r.tf.pt(bx, by)
            d.line([x0, y0, x1, y1], fill=col, width=3)
            ring(d, r, (ax, ay), GREEN, 0.3, 2)
            ring(d, r, (bx, by), CYAN, 0.3, 2)
            text(d, r, ((ax + bx) / 2, (ay + by) / 2), nm, col)
    img = r.frame(label=f'{stem}: {len(open_)} open  (red WALLED, '
                  'magenta BLOCKED, green MISSED)', overlays=[ov_all])
    img.save(os.path.join(png_dir, f'{stem}_overview.png'))
    print(f'renders -> {png_dir}')


if a.png_dir:
    render(a.png_dir)

# ---- negotiate -------------------------------------------------------
if a.negotiate and open_:
    still = N.run(open_, rounds=a.rounds, max_victims=a.victims)
    print(f'NEGOTIATED: {N.n_routed()}/{len(names)} routed, {N.n_vias()} vias'
          f'; still open {sorted(still)}')
    if a.apply:
        from kicad_writer import add_tracks_and_vias_to_pcb
        changed = [nm for nm in names if N.state[nm] is not orig[nm]
                   or nm in N.reberthed]
        txt = open(a.braided, encoding='utf-8').read()
        for nm in changed:
            nid, net = byname[nm]
            m = sg._matcher(nid, net.name)
            txt = sg._walk_strip(txt, 'segment', m)
            txt = sg._walk_strip(txt, 'via', m)
        tmp = a.apply + '.stripped.kicad_pcb'
        open(tmp, 'w', encoding='utf-8').write(txt)
        tracks, vias = [], []
        for nm in changed:
            nid = byname[nm][0]
            for s in N.state[nm][0]:
                tracks.append(dict(start=(s.start_x, s.start_y),
                                   end=(s.end_x, s.end_y), width=s.width,
                                   layer=s.layer, net_id=nid))
            for v in N.state[nm][1]:
                vias.append(dict(x=v.x, y=v.y, size=v.size, drill=v.drill,
                                 layers=list(v.layers), net_id=nid))
        add_tracks_and_vias_to_pcb(tmp, a.apply, tracks, vias,
                                   net_id_to_name={i: n.name for i, n
                                                   in pcb_b.nets.items()})
        os.remove(tmp)
        for ext in ('.kicad_pro', '.kicad_prl'):
            src = a.braided.rsplit('.', 1)[0] + ext
            if os.path.exists(src):
                shutil.copy(src, a.apply.rsplit('.', 1)[0] + ext)
        print(f'wrote {a.apply} ({len(changed)} nets re-laid: {changed})')
        r = subprocess.run([sys.executable, os.path.join(HERE, 'grade_k.py'),
                            a.apply, ','.join(names)], capture_output=True,
                           text=True)
        print(r.stdout.strip())
