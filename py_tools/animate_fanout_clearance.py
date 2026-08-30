"""
Animate the fanout-clearance cap repair (place_fanout_clearance.py).

Runs repair_fanout_clearance on a post-fanout board and records every accepted
cap move via the engine's `on_move` hook, then renders an animated GIF of the
caps gliding from their seed placement to their final, via-clearing positions.
The view is framed to the BGA ball field (not the whole board).

Usage:
  python animate_fanout_clearance.py fanned.kicad_pcb [out.gif] [options]

Pipeline: bga_fanout.py -> animate_fanout_clearance.py (visualizes the same
repair that place_fanout_clearance.py performs).

Requires only Pillow: rendering goes through route_render.BoardRenderer and
encoding through animate_route.save_movie (#431), so this file no longer
carries its own transform, GIF writer or font handling. Both are already used by
this repo's tooling. No matplotlib / ffmpeg needed.
"""
import _path  # noqa: F401  (#522: makes ../py_router importable)


import argparse
import os
import sys

from kicad_parser import parse_kicad_pcb
import routing_defaults as defaults
from bga_fanout.constants import DEFAULT_VIA_SIZE
from placement.fanout_clearance import repair_fanout_clearance


# ---------------------------------------------------------------------------
# Recording: snapshot cap geometry on every on_move callback.
# ---------------------------------------------------------------------------

class _Recorder:
    """Collects static board geometry (once) and a keyframe per cap move."""

    def __init__(self):
        self.static = None          # filled on first callback
        self.frames = []            # list of {ref: {'court':rect,'pads':[...]}}

    def __call__(self, st):
        if self.static is None:
            self.static = {
                'bga': list(st.bga_bboxes),
                'clearance': st.clearance,
                # (x, y, net, keepout_radius) -- keepout includes clearance
                'vias': [(vx, vy, vnet, ko) for (vx, vy, vnet, ko) in st.vias],
                # same-net attraction targets (BGA balls), flattened with net id
                'balls': [(x, y, net) for net, pts in st.attract.items()
                          for (x, y) in pts],
            }
        frame = {}
        for ref, cap in st.caps.items():
            frame[ref] = {
                'court': cap.rect(),
                'pads': list(cap.pad_rects()),
                'seed_court': cap.rect(cap.seed_x, cap.seed_y, cap.seed_rot),
            }
        self.frames.append(frame)


# ---------------------------------------------------------------------------
# Rendering helpers
# ---------------------------------------------------------------------------

# Shared with render_placement / the movie camera (#431). Moved to
# movie_camera.py rather than copied: these are pure functions and the versions
# there are byte-for-byte these, verified over net ids -2..399 and t in [0,1],
# so docs/fanout-cap-placement.gif is unchanged. movie_camera imports no PIL and
# no pygame at module scope precisely so this import stays cheap here.
from movie_camera import (lerp_rect as _lerp_rect, net_color as _net_color,
                          smoothstep as _smoothstep)


def _view_bounds(recorder, pad_mm):
    """World bbox: BGA field + every cap position seen across all frames."""
    xs0, ys0, xs1, ys1 = [], [], [], []
    for bb in recorder.static['bga']:
        xs0.append(bb[0]); ys0.append(bb[1]); xs1.append(bb[2]); ys1.append(bb[3])
    for frame in recorder.frames:
        for d in frame.values():
            for rect in (d['court'], d['seed_court']):
                xs0.append(rect[0]); ys0.append(rect[1])
                xs1.append(rect[2]); ys1.append(rect[3])
    for (vx, vy, _net, ko) in recorder.static['vias']:
        xs0.append(vx - ko); ys0.append(vy - ko)
        xs1.append(vx + ko); ys1.append(vy + ko)
    return (min(xs0) - pad_mm, min(ys0) - pad_mm,
            max(xs1) + pad_mm, max(ys1) + pad_mm)


def render_gif(recorder, out_path, size=900, sub_frames=14,
               fps=30, hold_start=8, hold_end=24, pad_mm=0.8, pcb=None):
    """Render the recorded cap moves.

    Ported off pygame onto the shared stack (#431): `route_render.BoardRenderer`
    supplies the substrate and the mm->px transform, and
    `animate_route.save_movie` does the encoding. That deletes three
    duplications this file used to carry -- a hand-rolled world->screen mapping,
    a hand-rolled GIF writer, and its own font handling -- and buys two things
    for free: the REAL board underneath the BGA field (outline, cutouts, zones),
    which the pygame version never drew, and `.mp4` output when the caller asks
    for one.

    `pcb` is optional so a caller with only a recorder still works: without it
    there is no substrate, just the overlay on a plain background, which is
    exactly what the pygame version produced.
    """
    from PIL import Image, ImageDraw

    import animate_route as _ar
    from route_render import BoardRenderer, load_font

    view = _view_bounds(recorder, pad_mm)
    st = recorder.static

    if pcb is not None:
        # show_pads=False: the caps MOVE during a tween, so board pads drawn at
        # their final poses would ghost against the animated ones.
        r = BoardRenderer(pcb, size=size, supersample=1, show_pads=False,
                          view=view)
    else:
        r = _PlainCanvas(size, view)

    def overlay_for(interp):
        def _draw(d, rr):
            tf = rr.tf

            def px(rect):
                x0, y0 = tf.pt(rect[0], rect[1])
                x1, y1 = tf.pt(rect[2], rect[3])
                return [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]

            for bb in st['bga']:                       # BGA outline
                d.rectangle(px(bb), outline=(70, 78, 96), width=2)
            for (x, y, net) in st['balls']:            # same-net balls
                sx, sy = tf.pt(x, y)
                c = _net_color(net)
                d.ellipse([sx - 2, sy - 2, sx + 2, sy + 2],
                          fill=(c[0] // 3, c[1] // 3, c[2] // 3))
            for (vx, vy, vnet, ko) in st['vias']:      # keepout ring + via
                sx, sy = tf.pt(vx, vy)
                c = _net_color(vnet)
                kr = tf.length(ko)
                d.ellipse([sx - kr, sy - kr, sx + kr, sy + kr],
                          outline=(c[0] // 3, c[1] // 3, c[2] // 3), width=1)
                vr = max(2.0, tf.length(ko - st['clearance']))
                d.ellipse([sx - vr, sy - vr, sx + vr, sy + vr], fill=c)
            font = load_font(max(9, size // 90))
            for ref, cap in interp.items():
                d.rectangle(px(cap['seed_court']), outline=(52, 52, 60), width=1)
                d.rectangle(px(cap['court']), outline=(150, 150, 165), width=1)
                cx = cy = 0.0
                for (x0, y0, x1, y1, net) in cap['pads']:
                    c = _net_color(net)
                    d.rectangle(px((x0, y0, x1, y1)), fill=c, outline=c)
                    cx, cy = tf.pt((x0 + x1) / 2, (y0 + y1) / 2)
                d.text((cx, cy), ref, fill=(235, 235, 245), font=font,
                       anchor='mm')
        return _draw

    def interp_caps(fa, fb, t):
        out = {}
        for ref in fb:
            a = fa.get(ref, fb[ref])
            b = fb[ref]
            out[ref] = {
                'seed_court': b['seed_court'],
                'court': _lerp_rect(a['court'], b['court'], t),
                'pads': [tuple(list(_lerp_rect(pa, pb, t)) + [pb[4]])
                         for pa, pb in zip(a['pads'], b['pads'])],
            }
        return out

    images = []

    def grab(interp, label):
        images.append(r.frame(overlays=[overlay_for(interp)], label=label))

    frames = recorder.frames
    n = len(frames)
    for _ in range(hold_start):
        grab(frames[0], f'seed placement   (0/{n - 1} moves)')
    for i in range(1, n):
        for s in range(sub_frames):
            t = _smoothstep((s + 1) / sub_frames)
            grab(interp_caps(frames[i - 1], frames[i], t), f'move {i}/{n - 1}')
    for _ in range(hold_end):
        grab(frames[-1], f'final placement  ({n - 1}/{n - 1} moves)')

    # Extension picks the format, with the .mp4 -> .gif fallback, the PNG dump
    # and the even-dimension crop all inherited.
    _ar.save_movie(images, out_path, fps=fps, end_hold=0.0)
    return out_path


class _PlainCanvas:
    """The no-board fallback: a bare background plus the same Transform, so the
    overlay code is identical whether or not a board was supplied."""

    def __init__(self, size, view):
        from PIL import Image, ImageDraw
        from route_render import Transform, load_font
        self.W = self.H = size
        self.ss = 1
        self._bg = (18, 20, 26)
        self.tf = Transform(view, size, size, 40)
        self._Image, self._Draw, self._font = Image, ImageDraw, load_font

    def frame(self, overlays=None, label=None, **_kw):
        img = self._Image.new('RGB', (self.W, self.H), self._bg)
        d = self._Draw.Draw(img)
        for fn in (overlays or ()):
            fn(d, self)
        if label:
            f = self._font(max(12, self.H // 55))
            d.rectangle([3, 3, 12 + 8 * len(label), 24], fill=(0, 0, 0))
            d.text((6, 6), label, fill=(240, 240, 240), font=f)
        return img


def main():
    p = argparse.ArgumentParser(
        description="Animate the fanout-clearance cap repair as a GIF.")
    p.add_argument("input_file", help="Post-fanout KiCad PCB file")
    p.add_argument("output_file", nargs="?", help="Output GIF "
                   "(default: input_capmove.gif)")
    # #768: None = "resolve it from the board", the same contract
    # --board-edge-clearance has had since #733. This front writes no
    # project, but it must PRICE identically to place_fanout_clearance.py
    # or the GIF shows a repair the tool does not perform.
    p.add_argument("--clearance", type=float, default=None,
                   help="Copper clearance CEILING in mm; omitted = the "
                        "board's own Default net class (see #768).")
    p.add_argument("--grid-step", type=float, default=defaults.GRID_STEP)
    # None, not 0.55 (#733): the engine resolves it, so the GIF frames the
    # same usable box the run it visualises actually used. A private copy
    # here would be a third notion of the margin -- the defect #733 closed.
    p.add_argument("--board-edge-clearance", type=float, default=None)
    p.add_argument("--capture-radius", type=float, default=2.0)
    p.add_argument("--default-via-size", type=float, default=DEFAULT_VIA_SIZE)
    p.add_argument("--near-margin", type=float, default=1.0)
    p.add_argument("--step", type=float, default=0.2)
    p.add_argument("--max-displacement", type=float, default=2.0)
    p.add_argument("--max-displacement-cap", type=float, default=3.0)
    p.add_argument("--displacement-growth", type=float, default=1.5)
    p.add_argument("--no-rotate", action="store_true")
    p.add_argument("--cap-prefix", default="C,R")
    p.add_argument("--lock", nargs="+", default=None, metavar="REF")
    p.add_argument("--max-passes", type=int, default=30)
    # animation controls
    p.add_argument("--size", type=int, default=900, help="GIF size in px")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--sub-frames", type=int, default=14,
                   help="Interpolated frames per cap move (smoothness)")
    args = p.parse_args()

    if args.output_file is None:
        base, _ = os.path.splitext(args.input_file)
        args.output_file = base + '_capmove.gif'

    print(f"Loading {args.input_file}...")
    pcb_data = parse_kicad_pcb(args.input_file)

    rec = _Recorder()
    repair_fanout_clearance(
        pcb_data, pcb_file=args.input_file,
        clearance=args.clearance, netclass_ceiling=args.clearance,
        grid_step=args.grid_step,
        board_edge_clearance=args.board_edge_clearance,
        near_margin=args.near_margin, capture_radius=args.capture_radius,
        default_via_size=args.default_via_size, step=args.step,
        max_displacement=args.max_displacement,
        max_displacement_cap=args.max_displacement_cap,
        displacement_growth=args.displacement_growth,
        allow_rotations=not args.no_rotate, cap_prefix=args.cap_prefix,
        lock_refs=args.lock, max_passes=args.max_passes,
        on_move=rec,
    )

    if not rec.frames or len(rec.frames) < 2:
        print("No cap moves were recorded -- nothing to animate.")
        return 1
    render_gif(rec, args.output_file, size=args.size, fps=args.fps,
               sub_frames=args.sub_frames, pcb=pcb_data)
    return 0


if __name__ == "__main__":
    sys.exit(main())
