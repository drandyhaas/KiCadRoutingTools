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

Copper layers are composited with per-layer transparency (``layer_alpha``),
so overlapping layers blend at crossings instead of the top one hiding the
rest; pass ``--layer-alpha 255`` for the opaque look.

CLI (static image of a whole board):
    python3 route_render.py BOARD.kicad_pcb [-o OUT.png] [--size 1600]
        [--supersample 2] [--layer-alpha 150] [--no-pads] [--no-zones]
        [--layers F.Cu,B.Cu]

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


_FONTS: Dict[int, "ImageFont.ImageFont"] = {}


def load_font(px: int):
    """A default PIL font at ``px``, memoized.

    Pillow only grew ``load_default(size=)`` in 9.2; older builds raise, hence
    the fallback. Shared so the caption strip and per-part labels don't each
    copy the try/except -- and so they can't drift apart.
    """
    px = max(6, int(px))
    f = _FONTS.get(px)
    if f is None:
        try:
            f = ImageFont.load_default(size=px)
        except Exception:
            f = ImageFont.load_default()
        _FONTS[px] = f
    return f


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
                 bg: Tuple[int, int, int] = _BG, layer_alpha: int = 150,
                 dynamic_zones: bool = False,
                 view: Optional[Tuple[float, float, float, float]] = None):
        self.pcb = pcb
        # dynamic_zones: keep plane pours OUT of the static base so the animator
        # can reveal each plane's fill per frame (via frame(zone_net_ids=...)).
        self.dynamic_zones = dynamic_zones
        self.ss = max(1, int(supersample))
        self.copper_layers = list(layers) if layers else list(pcb.board_info.copper_layers)
        self.palette = layer_palette(self.copper_layers)
        self._layer_set = set(self.copper_layers)
        self.bg = bg
        # Per-layer copper opacity (0-255). <255 composites each layer as a
        # translucent overlay so overlapping layers blend at crossings ("layers
        # add on each other"); 255 = opaque (fast path, last layer wins).
        self.layer_alpha = max(1, min(255, int(layer_alpha)))

        # The BOARD bounds fix the canvas size; the VIEW only aims the transform
        # inside it. Keeping W/H off the board is what lets a camera change the
        # view mid-movie: every frame stays the same size, which _write_mp4 and
        # the Pillow GIF fallback both silently require (a size change mid-stream
        # raises, is caught, and degrades the whole movie to GIF).
        self.bounds = pcb.board_info.board_bounds or _geometry_bounds(pcb)
        # Aspect-fit the output box to the board so the image isn't mostly empty.
        min_x, min_y, max_x, max_y = self.bounds
        bw, bh = max(max_x - min_x, 1e-6), max(max_y - min_y, 1e-6)
        if bw >= bh:
            self.W, self.H = size, max(1, int(round(size * bh / bw)))
        else:
            self.W, self.H = max(1, int(round(size * bw / bh))), size
        self._margin_px = margin_frac * size * self.ss

        self._show_pads = show_pads
        self._show_zones = show_zones
        self.set_view(view)

    def set_view(self, view: Optional[Tuple[float, float, float, float]] = None) -> None:
        """Aim the renderer at a world rect ``(min_x, min_y, max_x, max_y)``;
        ``None`` = the whole board. This is the crop/zoom/pan seam -- a viewport
        IS a pan and a zoom, and every world->pixel conversion already goes
        through ``self.tf``, so nothing else has to know.

        ``W``/``H`` never change, so frames rendered at different views encode
        into one movie. A view whose aspect differs from the canvas letterboxes
        automatically, because ``Transform`` min()-fits and centers.
        """
        self._view = view
        Wp, Hp = self.W * self.ss, self.H * self.ss
        self.tf = Transform(view or self.bounds, Wp, Hp, self._margin_px)
        self._build_base()

    def _build_base(self) -> None:
        Wp, Hp = self.W * self.ss, self.H * self.ss
        self._base = Image.new('RGB', (Wp, Hp), self.bg)
        d = ImageDraw.Draw(self._base)
        self._draw_outline(d)
        if self._show_zones and not self.dynamic_zones:
            self._draw_zones(d)
        # With dynamic_zones the pours are drawn per-frame, so pads must be too
        # (drawn AFTER the pour so they read on top of it, as in the static base).
        if self._show_pads and not self.dynamic_zones:
            self.draw_pads(d)

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

    def _draw_zones(self, d: ImageDraw.ImageDraw, net_ids=None) -> None:
        # Plane pours drawn dim, under the tracks, tinted by layer. The stored
        # polygon is the zone OUTLINE (not the computed fill) -- a good-enough
        # substrate hint for a debug view. ``net_ids`` (a set) restricts to those
        # nets' zones, for animating a plane "filling in" per frame.
        for z in getattr(self.pcb, 'zones', []) or []:
            if z.layer not in self._layer_set or len(z.polygon) < 3:
                continue
            if net_ids is not None and z.net_id not in net_ids:
                continue
            base = self.palette.get(z.layer, (120, 120, 120))
            dim = tuple(int(_BOARD_FILL[i] * 0.55 + base[i] * 0.45) for i in range(3))
            d.polygon([self.tf.pt(x, y) for x, y in z.polygon], fill=dim)

    def zone_net_ids(self):
        """Net ids that have at least one drawable plane zone (for the animator
        to reveal each plane's fill when its taps first appear)."""
        return {z.net_id for z in (getattr(self.pcb, 'zones', []) or [])
                if z.layer in self._layer_set and len(z.polygon) >= 3}

    def draw_pads(self, d: ImageDraw.ImageDraw, pads: Optional[Iterable] = None,
                  fill_for=None) -> None:
        """Draw pads (default: every pad on the board).

        ``fill_for(pad) -> color`` overrides the fill per pad, which is how a
        caller distinguishes through-hole / front-SMD / back-SMD without
        re-implementing pad rasterization -- custom copper polygons (#188),
        capsules, roundrect ``rratio``, ``rect_rotation`` and offset drills all
        live here and are easy to get subtly wrong a second time.
        """
        if pads is None:
            pads = (p for fp in self.pcb.footprints.values() for p in fp.pads)
        for p in pads:
            self._draw_pad(d, p, fill=fill_for(p) if fill_for else None)

    def _draw_pads(self, d: ImageDraw.ImageDraw) -> None:
        self.draw_pads(d)

    def _draw_pad(self, d: ImageDraw.ImageDraw, p, fill=None) -> None:
        fill = fill or _PAD
        # Custom copper outline(s) take precedence (comb/finger pads, #188).
        if getattr(p, 'polygons', None):
            for poly in p.polygons:
                if len(poly) >= 3:
                    d.polygon([self.tf.pt(x, y) for x, y in poly], fill=fill)
            return
        cx, cy = self.tf.pt(p.global_x, p.global_y)
        sx, sy = self.tf.length(p.size_x), self.tf.length(p.size_y)
        shape = (p.shape or 'rect').lower()
        rot = p.rect_rotation or 0.0
        if shape in ('circle', 'oval') and abs(sx - sy) < 0.5:
            r = max(sx, sy) / 2
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=fill)
        elif shape == 'oval':
            self._capsule(d, cx, cy, sx, sy, rot, fill)
        else:  # rect, roundrect, custom-without-polys, trapezoid...
            self._rrect(d, cx, cy, sx, sy, rot,
                        getattr(p, 'roundrect_rratio', 0.0), fill)
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
    def _group_by_layer(self, segments: Iterable) -> Dict[str, List]:
        out: Dict[str, List] = {}
        for s in segments:
            if s.layer in self._layer_set:
                out.setdefault(s.layer, []).append(s)
        return out

    def _draw_segments(self, d: ImageDraw.ImageDraw, segments: Iterable,
                       color=None) -> None:
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
              label: Optional[str] = None, zone_net_ids=None,
              overlays: Optional[Sequence] = None) -> Image.Image:
        """Composite the given copper onto the static substrate and return an
        RGB image at output resolution.

        ``segments``/``vias`` default to the whole board. ``highlight_*`` draw
        on top in ``highlight_color`` (e.g. tracks/vias added, ripped, or
        restored on this animation frame). ``label`` is stamped top-left.
        ``zone_net_ids`` (used with dynamic_zones) draws just those nets' plane
        pours under the copper, so a plane "fills in" on the frame its taps land.
        ``overlays`` is a sequence of ``fn(draw, renderer)`` callables drawn at
        SUPERSAMPLED resolution, above the copper and below the label.
        """
        segs = self.pcb.segments if segments is None else segments
        vs = self.pcb.vias if vias is None else vias
        img = self._base.copy()
        if self.dynamic_zones:
            dz = ImageDraw.Draw(img)
            if zone_net_ids:
                self._draw_zones(dz, net_ids=zone_net_ids)
            if self._show_pads:
                self._draw_pads(dz)   # pads on top of the (dynamic) pour
        by_layer = self._group_by_layer(segs)
        # Draw layers in reverse stack order (B.Cu first ... F.Cu last, so the
        # top copper reads as "nearest the viewer"). With layer_alpha < 255 each
        # layer is a translucent RGBA overlay composited onto the accumulator,
        # so overlapping layers blend at crossings; == 255 is the opaque path.
        if self.layer_alpha >= 255:
            d = ImageDraw.Draw(img)
            for layer in reversed(self.copper_layers):
                self._draw_segments(d, by_layer.get(layer, ()),
                                    color=self.palette.get(layer))
        else:
            acc = img.convert('RGBA')
            for layer in reversed(self.copper_layers):
                lsegs = by_layer.get(layer)
                if not lsegs:
                    continue
                overlay = Image.new('RGBA', acc.size, (0, 0, 0, 0))
                self._draw_segments(ImageDraw.Draw(overlay), lsegs,
                                    color=self.palette[layer] + (self.layer_alpha,))
                acc = Image.alpha_composite(acc, overlay)
            img = acc.convert('RGB')
        # Vias and highlights are drawn opaque on top so they stay unambiguous.
        d = ImageDraw.Draw(img)
        self._draw_vias(d, vs)
        if highlight_segments:
            self._draw_segments(d, highlight_segments, color=highlight_color)
        if highlight_vias:
            self._draw_vias(d, highlight_vias, color=highlight_color)
        # Caller-supplied drawing, BEFORE the downsample so it antialiases like
        # everything else (an overlay drawn after frame() returns would alias
        # every diagonal at the default supersample=2) and UNDER the label HUD.
        # Each overlay is `fn(draw, renderer)` and must work through
        # `renderer.tf` -- this is what keeps placement vocabulary (courtyards,
        # ghosts, arrows, airwires) out of this copper renderer entirely.
        for fn in (overlays or ()):
            fn(ImageDraw.Draw(img), self)
        if self.ss > 1:
            img = img.resize((self.W, self.H), Image.LANCZOS)
        if label:
            self._label(img, label)
        return img

    def render(self) -> Image.Image:
        """Whole-board image (all segments + vias)."""
        return self.frame()

    def _label(self, img: Image.Image, text: str) -> None:
        """Stamp the caption top-left, WRAPPING rather than clipping.

        This measured the text width and then never used it, so PIL clipped at
        the image edge and the overflow was simply gone. Measured on a 217-part
        board: the caption built 156 chars and ~117 fit, so `hole-conflict
        0.60mm` and `oob 7` -- a fab blocker and the off-board count, i.e. two
        of the four questions the checklist exists to answer -- were absent
        from the picture while the strip looked complete because it ended at a
        plausible-looking field. A verdict strip that silently drops its last
        fields is worse than no strip: it reads as the whole story.
        """
        d = ImageDraw.Draw(img)
        font = load_font(max(12, self.H // 55))
        pad = 6
        avail = max(80, self.W - 2 * pad - 6)

        def _w(s):
            try:
                bb = d.textbbox((0, 0), s, font=font)
                return bb[2] - bb[0]
            except Exception:
                return 8 * len(s)

        # Break on the caption's own field separator so a wrap never lands
        # mid-number; fall back to words, then to the raw string.
        parts = [p.strip() for p in text.split('|')] if '|' in text \
            else text.split(' ')
        joiner = '  |  ' if '|' in text else ' '
        lines, cur = [], ''
        for p in parts:
            cand = (cur + joiner + p) if cur else p
            if cur and _w(cand) > avail:
                lines.append(cur)
                cur = p
            else:
                cur = cand
        if cur:
            lines.append(cur)
        if not lines:
            return
        try:
            lh = (d.textbbox((0, 0), 'Ag', font=font)[3]
                  - d.textbbox((0, 0), 'Ag', font=font)[1]) + 4
        except Exception:
            lh = 16
        box_w = max(_w(ln) for ln in lines)
        d.rectangle([pad - 3, pad - 3, pad + box_w + 3,
                     pad + lh * len(lines) + 5], fill=(0, 0, 0))
        for i, ln in enumerate(lines):
            d.text((pad, pad + i * lh), ln, fill=(240, 240, 240), font=font)


def render_board_file(board_path: str, out_png: Optional[str] = None,
                      size: int = 1600, supersample: int = 2,
                      show_pads: bool = True, show_zones: bool = True,
                      layers: Optional[Sequence[str]] = None,
                      layer_alpha: int = 150,
                      view: Optional[Tuple[float, float, float, float]] = None,
                      label: Optional[str] = None,
                      refs: Optional[bool] = None,
                      ruler: Optional[bool] = None,
                      quiet: bool = False) -> Optional[str]:
    """Parse ``board_path`` and write a PNG. Returns the path written.

    ``view`` crops to a world rect (board mm) -- the question-scoped-crop seam
    (set_view) exposed to callers; a cropped frame is auto-labeled with its
    rect so the picture says where on the board it is. ``refs``/``ruler``
    (None = auto: ON for a cropped view, OFF whole-board) draw reference
    designators at footprint origins and a mm coordinate ruler, so what the
    crop shows is matchable against the JSON that cites refs/coordinates."""
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(board_path)
    r = BoardRenderer(pcb, size=size, supersample=supersample,
                      show_pads=show_pads, show_zones=show_zones, layers=layers,
                      layer_alpha=layer_alpha, view=view)
    if out_png is None:
        out_png = os.path.splitext(board_path)[0] + '.png'
    if label is None and view is not None:
        label = (f"view ({view[0]:g},{view[1]:g})-({view[2]:g},{view[3]:g})mm")
    overlays = []
    if (view is not None if refs is None else refs):
        overlays.append(ref_label_overlay(pcb))
    if (view is not None if ruler is None else ruler):
        overlays.append(mm_ruler_overlay())
    r.frame(label=label, overlays=overlays or None).save(out_png)
    if not quiet:
        n_layers = len(r.copper_layers)
        print(f"route_render: wrote {out_png} "
              f"({len(pcb.segments)} segs, {len(pcb.vias)} vias, {n_layers}L, "
              f"{r.W}x{r.H})")
    return out_png


def ref_label_overlay(pcb, min_mm_px: float = 8.0):
    """Overlay: reference designators anchored at each footprint's ORIGIN
    (a small cross marks the exact (x, y) the JSON quotes, the label sits
    beside it) -- so what the picture shows is matchable against score/DRC/
    forensics records by name. Scale-gated: below ``min_mm_px`` px per mm
    (a whole-board view) the labels would blanket the copper, so nothing is
    drawn; at crop scales they are legible."""
    def fn(d, r):
        px_per_mm = r.tf.length(1.0)
        if px_per_mm < min_mm_px:
            return
        Wp, Hp = r.W * r.ss, r.H * r.ss
        font = load_font(max(11, int(Hp / 55)))
        cross = max(3.0, r.tf.length(0.15))
        for ref, fp in sorted(pcb.footprints.items()):
            x, y = r.tf.pt(fp.x, fp.y)
            if not (-50 <= x <= Wp + 50 and -50 <= y <= Hp + 50):
                continue
            d.line([x - cross, y, x + cross, y], fill=(255, 235, 130), width=2)
            d.line([x, y - cross, x, y + cross], fill=(255, 235, 130), width=2)
            try:
                bb = d.textbbox((0, 0), ref, font=font)
                tw, th = bb[2] - bb[0], bb[3] - bb[1]
            except Exception:
                tw, th = 7 * len(ref), 12
            tx, ty = x + cross + 2, y - th - 2
            d.rectangle([tx - 2, ty - 2, tx + tw + 2, ty + th + 3],
                        fill=(0, 0, 0, 180) if d.mode == 'RGBA' else (0, 0, 0))
            d.text((tx, ty), ref, fill=(255, 235, 130), font=font)
    return fn


def mm_ruler_overlay():
    """Overlay: mm tick marks + coordinates along the top and left edges, so
    anything seen in a crop can be located in BOARD coordinates and matched
    to the JSON that cites them. Tick step auto-picks from the visible width
    (0.5/1/2/5/10/20 mm, aiming for <=14 ticks)."""
    def fn(d, r):
        tf = r.tf
        Wp, Hp = r.W * r.ss, r.H * r.ss
        # Invert the transform over the canvas to get the visible world rect.
        wx0 = (0 - tf.off_x) / tf.scale + tf.min_x
        wx1 = (Wp - tf.off_x) / tf.scale + tf.min_x
        wy0 = (0 - tf.off_y) / tf.scale + tf.min_y
        wy1 = (Hp - tf.off_y) / tf.scale + tf.min_y
        span = max(wx1 - wx0, wy1 - wy0, 1e-6)
        step = next((s for s in (0.5, 1, 2, 5, 10, 20, 50) if span / s <= 14),
                    100.0)
        font = load_font(max(10, int(Hp / 70)))
        tick = max(6, int(Hp / 90))
        col = (200, 200, 200)
        import math as _m
        gx = _m.ceil(wx0 / step) * step
        while gx <= wx1:
            px, _ = tf.pt(gx, wy0)
            d.line([px, 0, px, tick], fill=col, width=2)
            d.text((px + 3, 2), f"{gx:g}", fill=col, font=font)
            gx += step
        gy = _m.ceil(wy0 / step) * step
        while gy <= wy1:
            _, py = tf.pt(wx0, gy)
            d.line([0, py, tick, py], fill=col, width=2)
            d.text((3, py + 2), f"{gy:g}", fill=col, font=font)
            gy += step
    return fn


def parse_view(text: Optional[str]) -> Optional[Tuple[float, float, float, float]]:
    """'X0,Y0,X1,Y1' (board mm) -> a view rect, or None. Shared by every CLI
    that exposes the crop seam, so the spelling cannot drift."""
    if not text:
        return None
    parts = [p for p in text.replace(',', ' ').split() if p]
    if len(parts) != 4:
        raise ValueError(f"--view wants X0,Y0,X1,Y1 in board mm, got {text!r}")
    x0, y0, x1, y1 = (float(p) for p in parts)
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


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
    ap.add_argument('--layer-alpha', type=int, default=150,
                    help='per-layer copper opacity 1-255; <255 blends overlapping '
                         'layers at crossings, 255 = opaque (default 150)')
    ap.add_argument('--no-pads', action='store_true')
    ap.add_argument('--no-zones', action='store_true')
    ap.add_argument('--view', default=None, metavar='X0,Y0,X1,Y1',
                    help='crop to this world rect in board mm (question-scoped '
                         'zoom; the frame is labeled with the rect). Example: '
                         '--view 117,73,121,79')
    ap.add_argument('--refs', default=None, action=argparse.BooleanOptionalAction,
                    help='draw reference designators at footprint origins (a '
                         'cross marks the exact JSON coordinate). Default: on '
                         'for a --view crop, off whole-board')
    ap.add_argument('--ruler', default=None, action=argparse.BooleanOptionalAction,
                    help='mm coordinate ticks along the top/left edges, so the '
                         'picture is matchable to JSON coordinates. Default: '
                         'on for a --view crop, off whole-board')
    args = ap.parse_args()
    layers = args.layers.split(',') if args.layers else None
    out = render_board_file(args.board, args.output, size=args.size,
                            supersample=args.supersample,
                            show_pads=not args.no_pads,
                            show_zones=not args.no_zones, layers=layers,
                            layer_alpha=args.layer_alpha,
                            view=parse_view(args.view),
                            refs=args.refs, ruler=args.ruler)
    return 0 if out else 1


if __name__ == '__main__':
    sys.exit(main())
