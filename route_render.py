#!/usr/bin/env python3
"""Fast, dependency-light board renderer that draws tracks/vias/pads directly
from their geometry -- no KiCad, no SVG, no headless-browser rasterization.

The existing renderer (``tests/stress/board_image.py``) shells out to
``kicad-cli`` to export an SVG and then to headless Chrome to rasterize it:
two heavyweight subprocesses per image, far too slow to animate a route
frame-by-frame. This module instead rasterizes the parsed geometry with
Pillow, so a single frame is milliseconds and a whole routing movie is
feasible (issue #482).

It is built around :class:`BoardRenderer`, which computes the world->pixel
transform once and renders the *static substrate* (board outline, zones,
pads) a single time. Each :meth:`BoardRenderer.frame` call then composites a
chosen set of segments and vias on top of a copy of that substrate -- which
is exactly what an animator needs to draw the cumulative copper state at
each step, with optional highlight colors for the tracks/vias added, ripped,
or restored on that frame.

CLI (static image of a whole board):
    python3 route_render.py BOARD.kicad_pcb [-o OUT.png] [--size 1600]
        [--supersample 2] [--no-pads] [--no-zones] [--layers F.Cu,B.Cu]

Library:
    from kicad_parser import parse_kicad_pcb
    from route_render import BoardRenderer
    r = BoardRenderer(parse_kicad_pcb(path))
    r.render().save('board.png')                 # full board
    r.frame(segments=my_subset, vias=[]).save(..) # partial state (animation)
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------------------
# Colors
# ---------------------------------------------------------------------------
# A distinct color per copper layer, assigned in board stack order so any
# board (2, 4, 6, ... layers) renders sensibly. F.Cu warm/red, B.Cu cool/blue,
# inners spread across the spectrum -- roughly the KiCad convention.
_LAYER_PALETTE: List[Tuple[int, int, int]] = [
    (208, 64, 58),    # 0  F.Cu   red
    (70, 130, 210),   # 1  B.Cu   blue  (kept as the *last* layer below)
    (96, 190, 96),    # 2  In1    green
    (214, 190, 78),   # 3  In2    yellow
    (196, 110, 206),  # 4  In3    magenta
    (94, 200, 200),   # 5  In4    cyan
    (224, 150, 70),   # 6  In5    orange
    (150, 150, 224),  # 7  In6    periwinkle
    (170, 210, 90),   # 8  In7    lime
    (210, 120, 150),  # 9  In8    pink
]
_BG = (14, 16, 18)             # frame background (outside the board)
_BOARD_FILL = (26, 34, 28)     # soldermask-ish dark green board body
_EDGE = (225, 225, 210)        # Edge.Cuts stroke
_PAD = (192, 168, 96)          # exposed-pad gold
_PAD_HOLE = (10, 10, 10)       # drilled hole
_VIA = (176, 176, 184)         # via annulus
_VIA_HOLE = (10, 10, 10)
_HILITE = (255, 60, 60)        # default highlight (e.g. a rip)


def layer_palette(copper_layers: Sequence[str]) -> Dict[str, Tuple[int, int, int]]:
    """Map each copper layer name -> a color. B.Cu always gets the blue slot
    (index 1) so front/back read consistently; inner layers fill the rest."""
    pal: Dict[str, Tuple[int, int, int]] = {}
    inner_idx = 2
    for name in copper_layers:
        if name == 'F.Cu':
            pal[name] = _LAYER_PALETTE[0]
        elif name == 'B.Cu':
            pal[name] = _LAYER_PALETTE[1]
        else:
            pal[name] = _LAYER_PALETTE[inner_idx % len(_LAYER_PALETTE)]
            inner_idx += 1
    return pal


# ---------------------------------------------------------------------------
# World (mm) -> pixel transform
# ---------------------------------------------------------------------------
class Transform:
    """Uniform-scale mapping from board mm to image pixels.

    KiCad PCB coordinates and PIL image coordinates are BOTH y-down, so no
    vertical flip is needed -- the board renders in the same orientation KiCad
    shows it. A single ``scale`` (px per mm) preserves aspect ratio; the board
    is centered inside ``(width, height)`` with ``margin_px`` of padding.
    """

    def __init__(self, bounds: Tuple[float, float, float, float],
                 width: int, height: int, margin_px: float):
        min_x, min_y, max_x, max_y = bounds
        bw = max(max_x - min_x, 1e-6)
        bh = max(max_y - min_y, 1e-6)
        self.scale = min((width - 2 * margin_px) / bw,
                         (height - 2 * margin_px) / bh)
        # center the board in the canvas
        self.off_x = margin_px + (width - 2 * margin_px - bw * self.scale) / 2
        self.off_y = margin_px + (height - 2 * margin_px - bh * self.scale) / 2
        self.min_x, self.min_y = min_x, min_y

    def pt(self, x: float, y: float) -> Tuple[float, float]:
        return (self.off_x + (x - self.min_x) * self.scale,
                self.off_y + (y - self.min_y) * self.scale)

    def length(self, mm: float) -> float:
        return mm * self.scale


def _geometry_bounds(pcb) -> Tuple[float, float, float, float]:
    """Fallback bounds from all drawable geometry when the file has no
    Edge.Cuts outline (board_bounds is None)."""
    xs: List[float] = []
    ys: List[float] = []
    for s in pcb.segments:
        xs += [s.start_x, s.end_x]
        ys += [s.start_y, s.end_y]
    for v in pcb.vias:
        xs += [v.x - v.size / 2, v.x + v.size / 2]
        ys += [v.y - v.size / 2, v.y + v.size / 2]
    for fp in pcb.footprints.values():
        for p in fp.pads:
            r = max(p.size_x, p.size_y) / 2
            xs += [p.global_x - r, p.global_x + r]
            ys += [p.global_y - r, p.global_y + r]
    if not xs:
        return (0.0, 0.0, 1.0, 1.0)
    return (min(xs), min(ys), max(xs), max(ys))


def _rot(cx: float, cy: float, dx: float, dy: float, ang_deg: float) -> Tuple[float, float]:
    """Rotate (dx,dy) about origin by ang_deg (CCW numerically; in the y-down
    image frame this reproduces KiCad's visual orientation), offset to (cx,cy)."""
    a = math.radians(ang_deg)
    ca, sa = math.cos(a), math.sin(a)
    return (cx + dx * ca - dy * sa, cy + dx * sa + dy * ca)


class BoardRenderer:
    """Render a parsed board to PIL images, geometry-first.

    Build once (computes transform + the static substrate), then call
    :meth:`render` for the whole board or :meth:`frame` for an arbitrary
    subset of segments/vias (the animation hook).

    ``supersample`` renders at N x resolution and downsamples with LANCZOS for
    anti-aliasing; use 1 for maximum speed (many frames), 2 for crisp stills.
    """

    def __init__(self, pcb, size: int = 1600, supersample: int = 2,
                 margin_frac: float = 0.03, show_pads: bool = True,
                 show_zones: bool = True, layers: Optional[Sequence[str]] = None,
                 bg: Tuple[int, int, int] = _BG):
        self.pcb = pcb
        self.ss = max(1, int(supersample))
        self.copper_layers = list(layers) if layers else list(pcb.board_info.copper_layers)
        self.palette = layer_palette(self.copper_layers)
        self._layer_set = set(self.copper_layers)
        self.bg = bg

        bounds = pcb.board_info.board_bounds or _geometry_bounds(pcb)
        # Aspect-fit the output box to the board so the image isn't mostly empty.
        min_x, min_y, max_x, max_y = bounds
        bw, bh = max(max_x - min_x, 1e-6), max(max_y - min_y, 1e-6)
        if bw >= bh:
            self.W, self.H = size, max(1, int(round(size * bh / bw)))
        else:
            self.W, self.H = max(1, int(round(size * bw / bh))), size
        Wp, Hp = self.W * self.ss, self.H * self.ss
        self.tf = Transform(bounds, Wp, Hp, margin_frac * size * self.ss)

        self._base = Image.new('RGB', (Wp, Hp), bg)
        d = ImageDraw.Draw(self._base)
        self._draw_outline(d)
        if show_zones:
            self._draw_zones(d)
        if show_pads:
            self._draw_pads(d)

    # -- substrate -------------------------------------------------------
    def _draw_outline(self, d: ImageDraw.ImageDraw) -> None:
        bi = self.pcb.board_info
        outlines = bi.board_outlines or ([bi.board_outline] if bi.board_outline else [])
        cutouts = getattr(bi, 'board_cutouts', None) or []
        if outlines:
            for poly in outlines:
                if len(poly) >= 3:
                    d.polygon([self.tf.pt(x, y) for x, y in poly], fill=_BOARD_FILL)
            for poly in cutouts:
                if len(poly) >= 3:
                    d.polygon([self.tf.pt(x, y) for x, y in poly], fill=self.bg)
            ew = max(1, int(round(self.tf.length(0.15))))
            for poly in outlines:
                if len(poly) >= 2:
                    pts = [self.tf.pt(x, y) for x, y in poly]
                    d.line(pts + [pts[0]], fill=_EDGE, width=ew, joint='curve')
        elif bi.board_bounds:
            (x0, y0, x1, y1) = bi.board_bounds
            d.rectangle([self.tf.pt(x0, y0), self.tf.pt(x1, y1)],
                        fill=_BOARD_FILL, outline=_EDGE,
                        width=max(1, int(round(self.tf.length(0.15)))))

    def _draw_zones(self, d: ImageDraw.ImageDraw) -> None:
        # Plane pours drawn dim, under the tracks, tinted by layer. The stored
        # polygon is the zone OUTLINE (not the computed fill) -- a good-enough
        # substrate hint for a debug view.
        for z in getattr(self.pcb, 'zones', []) or []:
            if z.layer not in self._layer_set or len(z.polygon) < 3:
                continue
            base = self.palette.get(z.layer, (120, 120, 120))
            dim = tuple(int(_BOARD_FILL[i] * 0.55 + base[i] * 0.45) for i in range(3))
            d.polygon([self.tf.pt(x, y) for x, y in z.polygon], fill=dim)

    def _draw_pads(self, d: ImageDraw.ImageDraw) -> None:
        for fp in self.pcb.footprints.values():
            for p in fp.pads:
                self._draw_pad(d, p)

    def _draw_pad(self, d: ImageDraw.ImageDraw, p) -> None:
        # Custom copper outline(s) take precedence (comb/finger pads, #188).
        if getattr(p, 'polygons', None):
            for poly in p.polygons:
                if len(poly) >= 3:
                    d.polygon([self.tf.pt(x, y) for x, y in poly], fill=_PAD)
            return
        cx, cy = self.tf.pt(p.global_x, p.global_y)
        sx, sy = self.tf.length(p.size_x), self.tf.length(p.size_y)
        shape = (p.shape or 'rect').lower()
        rot = p.rect_rotation or 0.0
        if shape in ('circle', 'oval') and abs(sx - sy) < 0.5:
            r = max(sx, sy) / 2
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=_PAD)
        elif shape == 'oval':
            self._capsule(d, cx, cy, sx, sy, rot, _PAD)
        else:  # rect, roundrect, custom-without-polys, trapezoid...
            self._rrect(d, cx, cy, sx, sy, rot,
                        getattr(p, 'roundrect_rratio', 0.0), _PAD)
        # drilled hole for through-hole pads
        if getattr(p, 'drill', 0.0) and p.pad_type != 'connect':
            hx, hy = self.tf.pt(p.hole_x if p.hole_x is not None else p.global_x,
                                p.hole_y if p.hole_y is not None else p.global_y)
            hr = self.tf.length(p.drill) / 2
            if hr >= 0.5:
                d.ellipse([hx - hr, hy - hr, hx + hr, hy + hr], fill=_PAD_HOLE)

    def _rrect(self, d, cx, cy, sx, sy, rot, rratio, fill) -> None:
        hw, hh = sx / 2, sy / 2
        if abs(rot) < 0.05:
            box = [cx - hw, cy - hh, cx + hw, cy + hh]
            r = max(0.0, min(rratio, 0.5)) * min(sx, sy)
            if r >= 1.0:
                d.rounded_rectangle(box, radius=r, fill=fill)
            else:
                d.rectangle(box, fill=fill)
        else:  # rotated rect -> polygon
            corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
            d.polygon([_rot(cx, cy, dx, dy, rot) for dx, dy in corners], fill=fill)

    def _capsule(self, d, cx, cy, sx, sy, rot, fill) -> None:
        # stadium: central rect + two end semicircles, along the long axis
        if sx >= sy:
            r = sy / 2
            half = sx / 2 - r
            ends = [(-half, 0.0), (half, 0.0)]
            self._rrect(d, cx, cy, 2 * half if half > 0 else 0.1, sy, rot, 0.0, fill)
        else:
            r = sx / 2
            half = sy / 2 - r
            ends = [(0.0, -half), (0.0, half)]
            self._rrect(d, cx, cy, sx, 2 * half if half > 0 else 0.1, rot, 0.0, fill)
        for dx, dy in ends:
            ex, ey = _rot(cx, cy, dx, dy, rot)
            d.ellipse([ex - r, ey - r, ex + r, ey + r], fill=fill)

    # -- copper (per-frame) ---------------------------------------------
    def _draw_segments(self, d: ImageDraw.ImageDraw, segments: Iterable,
                       color: Optional[Tuple[int, int, int]] = None) -> None:
        for s in segments:
            if s.layer not in self._layer_set:
                continue
            c = color or self.palette.get(s.layer, (200, 200, 200))
            x0, y0 = self.tf.pt(s.start_x, s.start_y)
            x1, y1 = self.tf.pt(s.end_x, s.end_y)
            w = max(1, int(round(self.tf.length(s.width))))
            d.line([x0, y0, x1, y1], fill=c, width=w, joint='curve')
            if w >= 3:  # round caps so corners of a polyline look continuous
                r = w / 2
                d.ellipse([x0 - r, y0 - r, x0 + r, y0 + r], fill=c)
                d.ellipse([x1 - r, y1 - r, x1 + r, y1 + r], fill=c)

    def _draw_vias(self, d: ImageDraw.ImageDraw, vias: Iterable,
                   color: Optional[Tuple[int, int, int]] = None) -> None:
        for v in vias:
            cx, cy = self.tf.pt(v.x, v.y)
            r = max(1.0, self.tf.length(v.size) / 2)
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=color or _VIA)
            hr = self.tf.length(v.drill) / 2
            if hr >= 0.5:
                d.ellipse([cx - hr, cy - hr, cx + hr, cy + hr], fill=_VIA_HOLE)

    # -- public ----------------------------------------------------------
    def frame(self, segments: Optional[Iterable] = None,
              vias: Optional[Iterable] = None,
              highlight_segments: Optional[Iterable] = None,
              highlight_vias: Optional[Iterable] = None,
              highlight_color: Tuple[int, int, int] = _HILITE,
              label: Optional[str] = None) -> Image.Image:
        """Composite the given copper onto the static substrate and return an
        RGB image at output resolution.

        ``segments``/``vias`` default to the whole board. ``highlight_*`` draw
        on top in ``highlight_color`` (e.g. tracks/vias added, ripped, or
        restored on this animation frame). ``label`` is stamped top-left.
        """
        segs = self.pcb.segments if segments is None else segments
        vs = self.pcb.vias if vias is None else vias
        img = self._base.copy()
        d = ImageDraw.Draw(img)
        self._draw_segments(d, segs)
        self._draw_vias(d, vs)
        if highlight_segments:
            self._draw_segments(d, highlight_segments, color=highlight_color)
        if highlight_vias:
            self._draw_vias(d, highlight_vias, color=highlight_color)
        if self.ss > 1:
            img = img.resize((self.W, self.H), Image.LANCZOS)
        if label:
            self._label(img, label)
        return img

    def render(self) -> Image.Image:
        """Whole-board image (all segments + vias)."""
        return self.frame()

    def _label(self, img: Image.Image, text: str) -> None:
        d = ImageDraw.Draw(img)
        try:
            font = ImageFont.load_default(size=max(12, self.H // 55))
        except Exception:
            font = ImageFont.load_default()
        pad = 6
        try:
            bb = d.textbbox((0, 0), text, font=font)
            tw, th = bb[2] - bb[0], bb[3] - bb[1]
        except Exception:
            tw, th = 8 * len(text), 12
        d.rectangle([pad - 3, pad - 3, pad + tw + 3, pad + th + 5], fill=(0, 0, 0))
        d.text((pad, pad), text, fill=(240, 240, 240), font=font)


def render_board_file(board_path: str, out_png: Optional[str] = None,
                      size: int = 1600, supersample: int = 2,
                      show_pads: bool = True, show_zones: bool = True,
                      layers: Optional[Sequence[str]] = None,
                      quiet: bool = False) -> Optional[str]:
    """Parse ``board_path`` and write a PNG. Returns the path written."""
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(board_path)
    r = BoardRenderer(pcb, size=size, supersample=supersample,
                      show_pads=show_pads, show_zones=show_zones, layers=layers)
    if out_png is None:
        out_png = os.path.splitext(board_path)[0] + '.png'
    r.render().save(out_png)
    if not quiet:
        n_layers = len(r.copper_layers)
        print(f"route_render: wrote {out_png} "
              f"({len(pcb.segments)} segs, {len(pcb.vias)} vias, {n_layers}L, "
              f"{r.W}x{r.H})")
    return out_png


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board')
    ap.add_argument('-o', '--output', default=None,
                    help='output PNG (default: alongside the board)')
    ap.add_argument('--size', type=int, default=1600,
                    help='longest image dimension in px (default 1600)')
    ap.add_argument('--supersample', type=int, default=2,
                    help='anti-alias factor; 1 = fastest, 2 = crisp (default 2)')
    ap.add_argument('--layers', default=None,
                    help='comma-separated copper layers to draw (default: all)')
    ap.add_argument('--no-pads', action='store_true')
    ap.add_argument('--no-zones', action='store_true')
    args = ap.parse_args()
    layers = args.layers.split(',') if args.layers else None
    out = render_board_file(args.board, args.output, size=args.size,
                            supersample=args.supersample,
                            show_pads=not args.no_pads,
                            show_zones=not args.no_zones, layers=layers)
    return 0 if out else 1


if __name__ == '__main__':
    sys.exit(main())
