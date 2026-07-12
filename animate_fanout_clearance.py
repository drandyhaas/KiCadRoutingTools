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

Requires pygame (rendering) and Pillow (GIF encoding) -- both already used by
this repo's tooling. No matplotlib / ffmpeg needed.
"""

import argparse
import colorsys
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

def _net_color(net_id):
    """Stable, well-separated color per net id (gray for unconnected/0)."""
    if not net_id or net_id <= 0:
        return (130, 130, 130)
    h = ((net_id * 0.61803398875) % 1.0)
    r, g, b = colorsys.hsv_to_rgb(h, 0.65, 1.0)
    return (int(r * 255), int(g * 255), int(b * 255))


def _smoothstep(t):
    return t * t * (3.0 - 2.0 * t)


def _lerp_rect(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(4))


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
               fps=30, hold_start=8, hold_end=24, pad_mm=0.8):
    import pygame
    from PIL import Image

    os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
    pygame.init()
    try:
        font = pygame.font.SysFont('Menlo,DejaVuSansMono,monospace', 13)
        small = pygame.font.SysFont('Menlo,DejaVuSansMono,monospace', 10)
    except Exception:
        font = small = pygame.font.Font(None, 14)

    wx0, wy0, wx1, wy1 = _view_bounds(recorder, pad_mm)
    ww, wh = wx1 - wx0, wy1 - wy0
    margin = 40
    scale = min((size - 2 * margin) / ww, (size - 2 * margin) / wh)
    # center the content
    off_x = (size - ww * scale) / 2.0
    off_y = (size - wh * scale) / 2.0
    # KiCad Y increases downward, same as screen -> no vertical flip.
    def w2s(x, y):
        return (off_x + (x - wx0) * scale, off_y + (y - wy0) * scale)

    def rect_px(r):
        x0, y0 = w2s(r[0], r[1])
        x1, y1 = w2s(r[2], r[3])
        return pygame.Rect(x0, y0, max(1, x1 - x0), max(1, y1 - y0))

    BG = (18, 20, 26)
    st = recorder.static

    def draw_frame(surf, interp, label):
        surf.fill(BG)
        # BGA outline
        for bb in st['bga']:
            pygame.draw.rect(surf, (70, 78, 96), rect_px(bb), 2)
        # same-net balls (dim dots)
        for (x, y, net) in st['balls']:
            sx, sy = w2s(x, y)
            c = _net_color(net)
            pygame.draw.circle(surf, (c[0] // 3, c[1] // 3, c[2] // 3),
                               (int(sx), int(sy)), 2)
        # fanout vias: keepout ring + solid via disk
        for (vx, vy, vnet, ko) in st['vias']:
            sx, sy = w2s(vx, vy)
            c = _net_color(vnet)
            pygame.draw.circle(surf, (c[0] // 3, c[1] // 3, c[2] // 3),
                               (int(sx), int(sy)), int(ko * scale), 1)
            vr = max(2, (ko - st['clearance']) * scale)
            pygame.draw.circle(surf, c, (int(sx), int(sy)), int(vr))
        # caps: seed ghost, current courtyard, current pads, label
        for ref, d in interp.items():
            seed = d['seed_court']
            pygame.draw.rect(surf, (52, 52, 60), rect_px(seed), 1)
            court = d['court']
            pygame.draw.rect(surf, (150, 150, 165), rect_px(court), 1)
            cx = cy = 0.0
            for (x0, y0, x1, y1, net) in d['pads']:
                c = _net_color(net)
                pr = rect_px((x0, y0, x1, y1))
                shade = pygame.Surface((pr.w, pr.h), pygame.SRCALPHA)
                shade.fill((c[0], c[1], c[2], 210))
                surf.blit(shade, pr)
                pygame.draw.rect(surf, c, pr, 1)
                cx, cy = w2s((x0 + x1) / 2, (y0 + y1) / 2)
            lbl = small.render(ref, True, (235, 235, 245))
            surf.blit(lbl, (cx - lbl.get_width() / 2, cy - lbl.get_height() / 2))
        # HUD
        txt = font.render(label, True, (210, 215, 230))
        surf.blit(txt, (12, 10))

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

    surf = pygame.Surface((size, size))
    images = []

    def grab(interp, label):
        draw_frame(surf, interp, label)
        raw = pygame.image.tobytes(surf, 'RGB')
        images.append(Image.frombytes('RGB', (size, size), raw))

    frames = recorder.frames
    n = len(frames)
    for _ in range(hold_start):
        grab(frames[0], f'seed placement   (0/{n - 1} moves)')
    for i in range(1, n):
        for s in range(sub_frames):
            t = _smoothstep((s + 1) / sub_frames)
            grab(interp_caps(frames[i - 1], frames[i], t),
                 f'move {i}/{n - 1}')
    for _ in range(hold_end):
        grab(frames[-1], f'final placement  ({n - 1}/{n - 1} moves)')

    pygame.quit()
    dur = int(1000 / fps)
    images[0].save(out_path, save_all=True, append_images=images[1:],
                   duration=dur, loop=0, optimize=True)
    print(f"Wrote {out_path}  ({len(images)} frames, ~{len(images) / fps:.1f}s "
          f"at {fps} fps)")


def main():
    p = argparse.ArgumentParser(
        description="Animate the fanout-clearance cap repair as a GIF.")
    p.add_argument("input_file", help="Post-fanout KiCad PCB file")
    p.add_argument("output_file", nargs="?", help="Output GIF "
                   "(default: input_capmove.gif)")
    p.add_argument("--clearance", type=float, default=defaults.CLEARANCE)
    p.add_argument("--grid-step", type=float, default=defaults.GRID_STEP)
    p.add_argument("--board-edge-clearance", type=float, default=0.55)
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
        clearance=args.clearance, grid_step=args.grid_step,
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
               sub_frames=args.sub_frames)
    return 0


if __name__ == "__main__":
    sys.exit(main())
