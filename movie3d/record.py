#!/usr/bin/env python3
"""Record a place_simple run for the 3D placement movie (prototype).

    python3 -X utf8 movie3d/record.py BOARD OUT.kicad_pcb --work DIR \\
        --groups groups.json --intent intent.json --seed 0

BOARD and OUT come first; every argument but this tool's own is
place_simple.py's.

The story is the 2D movie's own: `py_placer/placement_story.py` (shared with
`place_movie.py`) hooks place_simple's run -- read-only, so the placement and
OUT are place_simple's for the same seed -- and decides what happens when:
the piles, the fixed parts flying to their places, the scatter, the anneal's
schedule and captions, the best placement, placed. `Recorder` is its 3D
renderer: each shot becomes a timeline row, and the 3D beats are its
overrides -- parts off the board lie on a table below it, a part travelling
to its place arcs up and over, the pile takes off part by part while the
camera closes in, and each anneal snapshot plays as a short move with a hop.

Writes two JSON files into --work, which the page (page/movie3d.mjs) draws:

  scene.json     the board (outline, cutouts, thickness), every part in its
                 OWN frame (pads, courtyard, a body box with a height by part
                 kind), its side and group colour, the nets the airwires join,
                 the key and the colours; and, from models.py, the parts'
                 KiCad models (a GLB) and the poses they were exported at;
  timeline.json  one entry per movie frame: every part's pose [x, y, rot,
                 height of its mounting face], the parts that moved, the
                 caption, the box the camera frames and the camera's azimuth
                 and elevation (and, in a flow movie, the copper's tick).
"""
#: #937 registry: a prototype in its own directory, in no door; it writes
#: OUT (place_simple's placed board). Read by krt_registry.py by AST.
KRT_TOOL = {'scope': [], 'kind': 'actor'}

import argparse
import json
import math
import os
import random
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
import _path  # noqa: E402,F401  (py_router and py_tools on sys.path, as for every py_placer script)
from placement_story import Story, _ease, _lerp, install  # noqa: E402

FRAMES = 120        # anneal frames per run (place_movie's default): the budget the tweens share
TWEEN = 3           # frames each anneal step takes: FRAMES / TWEEN snapshots, each played as a move
HOP = 1.2           # mm a part hops at most on an anneal step (a shorter step hops less)
SPEEDUP = 4.0       # the anneal plays this much faster at its end than at its start
FPS = 12.0
FLY = 8             # frames a fixed part takes to fly to its place (at most)
GLIDE = 20          # frames the pile takes to reach the places it is seeded at
ZOOM_FROM = 0.3     # how far into the glide the camera starts closing in on the board
HOLD_S = 0.6        # seconds a step's frame holds
END_S = 3.0         # seconds on the placed board (the camera keeps orbiting)
TABLE = -3.0        # the table under the board (y of its top); the board stands above it
#: The board is DRAWN this thick at most: at its real 1.6 mm the slab read as
#: a block next to 0402 parts (Andy, on the first stills).
DRAWN_THICKNESS = 0.8
SWEEP = 20.0        # the camera orbits this far either side of its base azimuth over the movie
ELEV = 46.0         # camera elevation, deg
TALL = 0.9          # mm: a body this tall or taller can hide the parts behind it
#: The board's opacity, all film: see-through, so the back's parts, their
#: airwires and the back's copper show through it.
BOARD_ALPHA = 0.55
RISE_S = 0.6        # seconds the board takes to rise off the table before it turns over (and to settle)
TURN_S = 1.6        # seconds each half turn takes
BOTTOM_S = 2.0      # seconds the board holds with its bottom up
SETTLE_S = 1.0      # seconds on the board's top again, at the end

#: Body heights (mm) by footprint name, then by reference prefix. A stand-in:
#: the board file carries no heights the parser reads.
NAME_HEIGHTS = (('usb', 3.2), ('pinheader', 2.5), ('pin_header', 2.5), ('b3u', 1.6),
                ('sot-23', 1.1), ('sot23', 1.1), ('qfn', 0.85), ('dfn', 0.85),
                ('soic', 1.6), ('tssop', 1.1), ('qfp', 1.4), ('bga', 1.2),
                ('crystal_smd_2016', 0.55), ('crystal_smd_3225', 0.8), ('hc49', 3.5))
PREFIX_HEIGHTS = {'R': 0.35, 'C': 0.5, 'L': 0.8, 'FB': 0.5, 'D': 0.45, 'LED': 0.45,
                  'Q': 1.0, 'U': 1.0, 'Y': 0.8, 'X': 0.8, 'SW': 2.0, 'S': 2.0, 'J': 3.0,
                  'P': 3.0, 'BT': 3.0, 'F': 1.0}
#: Parts drawn by their courtyard (mechanical bodies), not by their pad field.
COURT_BODY = ('J', 'P', 'SW', 'S', 'BT')


def _prefix(ref):
    m = re.match(r'[A-Za-z]+', ref)
    return m.group(0).upper() if m else ''


def body_height(ref, name):
    n = (name or '').lower()
    for key, h in NAME_HEIGHTS:
        if key in n:
            return h
    return PREFIX_HEIGHTS.get(_prefix(ref), 1.0)


def _not_assembled(fp):
    """No body to draw: no 3D model AND nothing placed there (a test pad,
    a solder jumper, a printed antenna, a wire's solder hole: board-only,
    out of the position files or out of the BOM) -- stage3d's rule (#1098)
    -- or a DNP part, model or
    not (#1350): the GLB is exported with `--no-dnp`, so it never gets its
    model, and the box it got instead drew a part the board will not carry
    (a board with 23 DNP 0402s had them drawn as boxes, reported as model
    fallbacks)."""
    if getattr(fp, 'dnp', False):
        return True
    return (not getattr(fp, 'has_model', False)
            and set(getattr(fp, 'attrs', ()) or ()) & {'board_only', 'exclude_from_pos_files',
                                                       'exclude_from_bom'})


def part_scene(ref, fp, part, colour, fixed, board_area=0.0):
    """One part in its own frame (its pose (0, 0, 0)): pads, courtyard, body.
    A footprint with no 3D model whose pad field covers half the board or
    more gets no body: it is the board's own pin frame (a module-format
    board's pin rows or castellations), not a part on the board, and a box
    over it would hide the board."""
    from placement.legality import pads_at_pose
    posed = pads_at_pose(fp, (0.0, 0.0, 0.0))
    rot = math.radians(fp.rotation or 0.0)

    def to_local(x, y):
        dx, dy = x - fp.x, y - fp.y
        return (dx * math.cos(rot) - dy * math.sin(rot), dx * math.sin(rot) + dy * math.cos(rot))
    pads = []
    for q, src in zip(posed, fp.pads):
        layers = [str(x) for x in (src.layers or ())]
        drill = float(getattr(src, 'drill', 0.0) or 0.0)
        if getattr(src, 'pad_type', '') == 'np_thru_hole':
            face = 'N'                      # no copper: its hole only
        elif drill > 0:
            face = 'T'
        elif any(x.startswith('B.') for x in layers) and not any(
                x.startswith('F.') or x.startswith('*') for x in layers):
            face = 'B'
        else:
            face = 'F'
        hx = hy = None
        if drill > 0 and getattr(src, 'hole_x', None) is not None:
            hx, hy = to_local(src.hole_x, src.hole_y)
        row = [round(q.global_x, 4), round(q.global_y, 4), round(float(q.size_x), 4),
               round(float(q.size_y), 4), round(float(q.rect_rotation or 0.0), 3),
               (q.shape or 'rect').lower(), face, round(drill, 4),
               None if hx is None else round(hx, 4), None if hy is None else round(hy, 4)]
        # a custom pad's copper is its polygons (a printed antenna's meander),
        # which can lie to one side of its anchor: draw them, and box it by them
        polys = [[[round(c, 4) for c in to_local(x, y)] for x, y in ring]
                 for ring in (getattr(src, 'polygons', None) or []) if len(ring) >= 3]
        if row[5] == 'custom' and polys:
            xs = [x for ring in polys for x, _y in ring]
            ys = [y for ring in polys for _x, y in ring]
            row[:5] = [round((min(xs) + max(xs)) / 2, 4), round((min(ys) + max(ys)) / 2, 4),
                       round(max(xs) - min(xs), 4), round(max(ys) - min(ys), 4), 0.0]
            row.append(polys)
        pads.append(row)
    if part is not None:
        court = part.rect(0.0, 0.0, 0.0)
    elif pads:
        court = (min(p[0] - p[2] / 2 for p in pads), min(p[1] - p[3] / 2 for p in pads),
                 max(p[0] + p[2] / 2 for p in pads), max(p[1] + p[3] / 2 for p in pads))
    else:
        court = (-0.5, -0.5, 0.5, 0.5)
    court = [round(v, 4) for v in court]
    body = None
    copper = [p for p in pads if p[6] != 'N']
    frame = (copper and board_area > 0 and not getattr(fp, 'has_model', False)
             and (max(p[0] + p[2] / 2 for p in copper) - min(p[0] - p[2] / 2 for p in copper))
             * (max(p[1] + p[3] / 2 for p in copper) - min(p[1] - p[3] / 2 for p in copper))
             >= 0.5 * board_area)
    if not _not_assembled(fp) and copper and not frame:
        h = body_height(ref, fp.footprint_name)
        if _prefix(ref) in COURT_BODY:
            x0, y0, x1, y1 = court
            ix = iy = 0.25
        else:   # the body sits INSIDE the pad field, as a package does (stage3d's rule)
            x0 = min(p[0] - p[2] / 2 for p in copper)
            x1 = max(p[0] + p[2] / 2 for p in copper)
            y0 = min(p[1] - p[3] / 2 for p in copper)
            y1 = max(p[1] + p[3] / 2 for p in copper)
            ix, iy = min(0.15 * (x1 - x0), 0.4), min(0.15 * (y1 - y0), 0.4)
        body = [round(x0 + ix, 4), round(y0 + iy, 4), round(x1 - ix, 4), round(y1 - iy, 4), h]
    side = 'B' if (fp.layer or '').startswith('B') else 'F'
    return {'ref': ref, 'side': side, 'pads': pads, 'court': court, 'body': body,
            'color': list(colour) if colour else None, 'fixed': fixed,
            'dnp': bool(getattr(fp, 'dnp', False))}


class Recorder(Story):
    """The placement story, recorded as timeline rows for the 3D page. The
    Story (placement_story.py) decides what happens when; this supplies the
    renderer's primitives and plays the 3D beats its own way."""

    def __init__(self, a):
        super().__init__(a)
        self.frames = []
        self.scene = None
        self.view = None
        self.extra = {}         # fields every row carries from now on (a flow movie's copper)

    # -- the renderer the story draws through ---------------------------------
    def setup(self, pcb, board, args):
        from shapely.geometry import Point, Polygon
        from placement.grouping import is_ground, load_groups
        from render_placement import group_palette
        import render_theme
        st = self.state
        # the board the parts' models are exported from: the one placed (the
        # input, or place_simple's copy with the declared faces written), its
        # model paths resolved against the ORIGINAL board's directory
        with open(board, encoding='utf-8') as f:
            self.model_src = (f.read(), os.path.dirname(os.path.abspath(args.board)),
                              os.path.abspath(args.board))
        self.hold = max(1, int(round(HOLD_S * self.a.fps)))
        th = render_theme.theme('dark', strict=False)
        # each group with a movable member its own colour, as place_movie
        groups = [g for g in load_groups(args.groups)['groups']
                  if any(r in self.movable for r in g['refs'])] if args.groups else []
        colours = group_palette(len(groups))
        colour, key = {}, []
        for g, rgb in zip(groups, colours):
            role = g.get('role')
            key.append({'rgb': list(rgb), 'mark': 'solid',
                        'text': g['name'] + (f" ({role})" if role and role != g['name'] else '')})
            for ref in g['refs']:
                if ref in self.movable:
                    colour[ref] = rgb
        if any(r not in colour for r in self.movable):
            key.append({'rgb': list(th.rgb('place_court_front')), 'mark': 'solid', 'text': 'in no group'})
            for r in self.movable:
                colour.setdefault(r, th.rgb('place_court_front'))
        fixed_rgb = (120, 118, 108)
        if len(self.movable) < len(st.parts):
            key.append({'rgb': list(fixed_rgb), 'mark': 'solid', 'text': 'fixed'})
        # every part place_simple models, and any other footprint with pads
        # (a pad-less logo it does not model is left out)
        self.refs = sorted(r for r in pcb.footprints if not r.startswith('#')
                           and (r in st.parts or pcb.footprints[r].pads))
        self.idx = {r: i for i, r in enumerate(self.refs)}
        bb = pcb.board_info.board_bounds
        area = (bb[2] - bb[0]) * (bb[3] - bb[1]) if bb else 0.0
        parts = [part_scene(r, pcb.footprints[r], st.parts.get(r), colour.get(r, fixed_rgb),
                            r not in self.movable, area) for r in self.refs]
        self.side = {p['ref']: p['side'] for p in parts}
        self.bodyh = {p['ref']: (p['body'][4] if p['body'] else 0.0) for p in parts}
        self.court = {p['ref']: p['court'] for p in parts}
        self.fixed = {p['ref'] for p in parts if p['fixed']}
        # the airwires' nets: every net but ground, joining two parts or more
        nets = {}
        for i, ref in enumerate(self.refs):
            for k, p in enumerate(pcb.footprints[ref].pads):
                nm = pcb.nets[p.net_id].name if p.net_id in pcb.nets else ''
                if p.net_id and not is_ground(nm):
                    nets.setdefault(p.net_id, []).append([i, k])
        nets = [m for _n, m in sorted(nets.items()) if len({i for i, _k in m}) > 1]
        bi = pcb.board_info
        bx0, by0, bx1, by1 = bi.board_bounds
        outline = [[float(x), float(y)] for x, y in (bi.board_outline or [])]
        if len(outline) < 3:
            outline = [[bx0, by0], [bx1, by0], [bx1, by1], [bx0, by1]]
        cutouts = [[[float(x), float(y)] for x, y in ring]
                   for ring in (getattr(bi, 'board_cutouts', None) or []) if len(ring) >= 3]
        self.board_poly = Polygon(outline).buffer(0)
        self._Point = Point
        st_total = sum(float(getattr(s, 'thickness', 0) or 0) for s in (bi.stackup or []))
        real = st_total if 0.2 <= st_total <= 10 else 1.6
        self.d = min(real, DRAWN_THICKNESS)
        self.scene = {
            'thickness': round(self.d, 4), 'real_thickness': round(real, 4), 'table': TABLE, 'bounds': [bx0, by0, bx1, by1],
            'outline': outline, 'cutouts': cutouts, 'parts': parts, 'nets': nets, 'key': key,
            'glb': None, 'board_alpha': BOARD_ALPHA,
            # the poses the models are exported at: the page re-poses each by
            # F(now) F(export)^-1
            'export_poses': {r: [pcb.footprints[r].x, pcb.footprints[r].y,
                                 pcb.footprints[r].rotation or 0.0, self.side[r]] for r in self.refs},
            'colors': {'ground': list(th.rgb('ground')), 'board': [24, 74, 48],
                       'edge': list(th.rgb('board_edge')), 'pad': list(th.rgb('pad')),
                       'hole': list(th.rgb('pad_hole')), 'air': [150, 162, 185],
                       'table': [34, 36, 40], 'moved': [255, 255, 255],
                       'band': list(th.rgb('chrome_band')), 'text': list(th.rgb('chrome_text')),
                       'best': list(th.rgb('status_best'))}}
        diag = math.hypot(bx1 - bx0, by1 - by0)
        self.arc = min(15.0, max(2.5, 0.15 * diag))

    @property
    def frame_size(self):
        return self.a.width, self.a.height

    def view_of(self, box, wide=False):
        """[x0, y0, x1, y1, lowest, highest]: the board's box (1 mm round it),
        or with the piles in view, down to the table."""
        top = self.d + 2.5
        if wide:
            return (box[0], box[1], box[2], box[3], TABLE, top)
        return (box[0] - 1.0, box[1] - 1.0, box[2] + 1.0, box[3] + 1.0, 0.0, top)

    def set_view(self, view):
        self.view = view

    def _pile_scale(self, u):
        # the camera looks across the board's long side: fit either way round
        W, H = self.frame_size
        return max(min(W / (u[2] - u[0]), H / (u[3] - u[1])), min(W / (u[3] - u[1]), H / (u[2] - u[0])))

    def _wide_extent(self):
        # the board and the piles as drawn (not the parts where the file
        # left them: the movable pile is drawn moved up to the board)
        return [self.bounds] + list(self._rects_at({**self.pile, **self.fixed_pile}).values())

    # -- the 3D beats -----------------------------------------------------------
    def intro_times(self):
        return 2 * self.hold

    def fly_frames(self, n):
        return max(3, min(FLY, 48 // max(1, n)))

    def fly_shot(self, label, ref, frm, to, t, poses):
        self.shot(label, moved=[ref], poses=poses, lifts={ref: self._arc(ref, frm, to, t)})

    def after_fly(self, poses):
        self.shot("fixed parts in place", times=self.hold, poses=poses)

    # -- heights ----------------------------------------------------------------
    def base(self, ref, x, y):
        """Height of the part's mounting face: the board's face when the part
        stands on the board, else the table (a back-side part lies body down)."""
        on = self.board_poly.contains(self._Point(x, y))
        if self.side.get(ref) == 'B':
            return 0.0 if on else TABLE + self.bodyh.get(ref, 0.0)
        return self.d if on else TABLE

    def _arc(self, ref, a, b, t):
        """Height on a hop from pose `a` to pose `b`, at `t` in [0, 1]."""
        h0, h1 = self.base(ref, a[0], a[1]), self.base(ref, b[0], b[1])
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        return h0 + (h1 - h0) * t + min(self.arc, 0.5 * dist + 0.5) * math.sin(math.pi * t)

    # -- frames -----------------------------------------------------------------
    def shot(self, label, moved=(), times=1, given=None, poses=None, key=True, lifts=None, **extra):
        """One frame of the state as it stands; `poses` draws those parts at
        another pose (a glide), `lifts` at another height (a hop). `given`
        and `key` are the 2D movie's: a declared face or pose is drawn from
        the first frame here, and the key is always on."""
        poses, lifts = poses or {}, lifts or {}
        st = self.state
        row = []
        for r in self.refs:
            if r in poses:
                x, y, q = poses[r]
            elif r in st.parts:
                p = st.parts[r]
                x, y, q = p.x, p.y, p.rot
            else:
                fp = self.pcb.footprints[r]
                x, y, q = fp.x, fp.y, fp.rotation or 0.0
            h = lifts[r] if r in lifts else self.base(r, x, y)
            row.append([round(x, 4), round(y, 4), round(q, 3), round(h, 3)])
        rec = {'p': row, 'label': label, 'view': [round(v, 3) for v in self.view],
               'moved': sorted(self.idx[r] for r in set(moved) if r in self.idx),
               **self.extra, **extra}
        self.frames.extend([rec] * times)

    def copper_shot(self, label, times=1, **extra):
        """A frame of a flow movie's routing half (route3d.py): the placed
        board, its copper at the current tick, no airwires."""
        self.shot(label, times=times, poses=getattr(self, 'route_poses', None), tick=self.cop.tick,
                  air=False, **extra)

    def turn_over(self, shot=None):
        """The film's end: the board rises off the table, turns over about
        the screen's horizontal, holds with its bottom up, turns on round to
        its top and settles. `shot` makes each frame (a flow movie's is
        copper_shot). A row's `flip` is [the turn, deg; how far the board
        has risen, 0-1]; finish() adds the axis (the camera's azimuth as the
        turn starts) and the page does the geometry -- how high the board
        must rise to clear the table, and a view that holds the whole turn."""
        shot = shot or self.shot
        last = self.frames[-1]['label'] if self.frames else ''
        n = lambda s: max(1, int(round(s * self.a.fps)))  # noqa: E731
        rise, turn = n(RISE_S), n(TURN_S)
        for i in range(1, rise + 1):
            shot("turning the board over", flip=[0.0, round(_ease(i / rise), 4)])
        for i in range(1, turn + 1):
            shot("turning the board over", flip=[round(180.0 * _ease(i / turn), 3), 1.0])
        shot("the bottom of the board", times=n(BOTTOM_S), flip=[180.0, 1.0])
        for i in range(1, turn + 1):
            shot("back to the top", flip=[round(180.0 + 180.0 * _ease(i / turn), 3), 1.0])
        for i in range(1, rise + 1):
            shot("back to the top", flip=[0.0, round(1.0 - _ease(i / rise), 4)])
        shot(last, times=n(SETTLE_S))

    # -- the run's hooks, the 3D way ---------------------------------------------
    def on_scatter(self, seed, clustered=False, anchored=False):
        self.run += 1
        if self.run > 1:
            self.shot(f"run {self.run}, a fresh start (the last left a part with no legal "
                      f"pose): scatter (seed {seed})", moved=self.movable, times=self.hold)
            return
        label = (f"scatter: each group's parts round the place its wiring prefers (seed {seed})"
                 if anchored else
                 f"scatter: each group's parts round one random place (seed {seed})" if clustered
                 else f"scatter: each movable part to a random place (seed {seed})")
        seeded = {r: (p.x, p.y, p.rot) for r, p in self.state.parts.items() if r in self.movable}
        # the pile takes off part by part: each starts a little after the last
        rng = random.Random(1)
        start = {r: rng.uniform(0.0, 0.45) for r in sorted(seeded)}
        span = 1.0 - 0.45
        for i in range(1, GLIDE + 1):
            poses, lifts = {}, {}
            for r in seeded:
                t = _ease(min(1.0, max(0.0, (i / GLIDE - start[r]) / span)))
                poses[r] = _lerp(self.pile[r], seeded[r], t)
                lifts[r] = self._arc(r, self.pile[r], seeded[r], t)
            if self.wide:       # ... the camera closing in on the board as they land
                self.view = _lerp(self.wide, self.home,
                                  _ease(min(1.0, max(0.0, (i / GLIDE - ZOOM_FROM) / (1 - ZOOM_FROM)))))
            self.shot(label, moved=self.movable, poses=poses, lifts=lifts)
        self.view = self.home
        self.shot(label, times=self.hold)

    def anneal_snapshots(self):
        return self.a.frames // max(1, self.a.tween)

    def on_anneal(self, moves):
        # the frame budget is shared: --frames / --tween snapshots, each
        # played as a move of --tween frames
        self.tween = max(1, self.a.tween)
        super().on_anneal(moves)
        self.prev = self._poses()
        if getattr(self, 'anneal_from', None) is None:
            self.anneal_from = len(self.frames)

    def snapshot(self, label, moved, annealer):
        self.play(label, moved, 1 if annealer.shoving else self.tween)

    def _poses(self):
        return {r: (p.x, p.y, p.rot) for r, p in self.state.parts.items() if r in self.movable}

    def play(self, label, moved, frames):
        """The parts that moved since the last snapshot, played from where
        they were to where they are over `frames` frames, each with a small
        hop (a turn takes the short way round)."""
        cur = self._poses()
        went = [r for r in cur if cur[r] != self.prev.get(r, cur[r])]
        for j in range(1, frames + 1):
            t = _ease(j / frames)
            poses, lifts = {}, {}
            for r in went:
                a, b = self.prev[r], cur[r]
                turn = (b[2] - a[2] + 180.0) % 360.0 - 180.0
                poses[r] = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, (a[2] + turn * t) % 360.0)
                dist = math.hypot(b[0] - a[0], b[1] - a[1])
                h0, h1 = self.base(r, a[0], a[1]), self.base(r, b[0], b[1])
                lifts[r] = h0 + (h1 - h0) * t + min(HOP, 0.3 + 0.25 * dist) * math.sin(math.pi * t)
            self.shot(label, moved=moved, poses=poses, lifts=lifts)
        self.prev = cur

    def on_annealed(self, res):
        legal = res.get('reseated') or []
        label = (f"best placement, cost {res['best']:.0f}"
                 + (f" | to the nearest legal pose: {', '.join(legal)}" if legal else ''))
        # the anneal ends on the best placement it saw: the parts go back to it
        back = [r for r, q in self._poses().items() if q != self.prev.get(r, q)]
        self.play(label, set(back) | set(legal), 2 * self.tween)
        self.shot(label, moved=legal, times=self.hold)

    def _hidden(self, base, step=4):
        """How much the tall parts hide of the parts that move, over the
        film, with the camera orbiting SWEEP either side of `base`: for every
        sampled frame, each part whose centre lies in the ground a tall body
        hides from the camera (up to its height / tan(elevation) behind it)
        costs that body's height, double when the part is moving. A fixed
        part is drawn see-through, so it hides at half. Only the frames from
        the anneal on count: before it the parts lie piled, hiding each other
        from every side."""
        n = len(self.frames)
        cot = 1.0 / math.tan(math.radians(ELEV))
        tall = [i for i, r in enumerate(self.refs)
                if self.bodyh[r] >= TALL and self.side[r] == 'F']
        half = {i: 0.5 * max(c[2] - c[0], c[3] - c[1])
                for i, c in enumerate(self.court[r] for r in self.refs)}
        cost = 0.0
        for k in range(getattr(self, 'anneal_from', None) or 0, n, step):
            f = self.frames[k]
            if 'flip' in f:         # the board turning over: the camera looks up at whatever faces it
                continue
            az = math.radians(base + SWEEP * (2.0 * k / max(1, n - 1) - 1.0))
            ux, uy = math.sin(az), math.cos(az)        # board plane, toward the camera
            p = f['p']
            moving = set(f['moved'])
            for t in tall:
                tx, ty, ht = p[t][0], p[t][1], self.bodyh[self.refs[t]]
                w = ht / 2.0 if self.refs[t] in self.fixed else ht
                for i, q in enumerate(p):
                    if i == t:
                        continue
                    vx, vy = q[0] - tx, q[1] - ty
                    behind = -(vx * ux + vy * uy)
                    side = abs(vx * uy - vy * ux)
                    if 0.0 < behind < ht * cot + half[t] and side < half[t] + half[i]:
                        cost += w * (2.0 if i in moving else 1.0)
        return cost

    def _lay_grid(self, grid, az):
        """Lay the judged seeds' boards out side by side along the camera's
        own screen axes at azimuth `az`, in the arrangement (columns x rows)
        that fills the frame best, each with its label in front. Fills each
        cell's `off` (its board's offset from the live board's real place,
        which is the filmed seed's cell) and `label_at` / `label_w`; returns
        the views, each as four ground corners: the filmed seed's cell, the
        whole grid, the best's cell."""
        el = math.radians(ELEV)
        a = math.radians(az)
        u = (math.sin(a), math.cos(a))          # board plane, toward the camera
        r = (math.cos(a), -math.sin(a))         # board plane, screen right
        dvec = (-u[0], -u[1])                   # board plane, away (screen up)
        bx0, by0, bx1, by1 = self.pcb.board_info.board_bounds
        corners = [(bx0, by0), (bx1, by0), (bx1, by1), (bx0, by1)]
        dot = lambda p, v: p[0] * v[0] + p[1] * v[1]  # noqa: E731
        wr = max(dot(c, r) for c in corners) - min(dot(c, r) for c in corners)
        wd = max(dot(c, dvec) for c in corners) - min(dot(c, dvec) for c in corners)
        label_w = 0.95 * wr
        label_h = label_w / 8
        pitch_r = 1.3 * wr
        pitch_d = 1.25 * wd + 1.8 * label_h / math.sin(el)
        cells = grid['cells']
        n = len(cells)
        W, H = self.frame_size
        best = None
        for cols in range(1, n + 1):
            rows = math.ceil(n / cols)
            fit = min(W / (cols * pitch_r), H / (rows * pitch_d * math.sin(el) + label_h))
            if best is None or fit > best[0]:
                best = (fit, cols, rows)
        _fit, cols, rows = best
        centre = {}
        for k in range(n):
            col, row = k % cols, k // cols
            a_r = (col - (cols - 1) / 2) * pitch_r
            a_d = ((rows - 1) / 2 - row) * pitch_d
            centre[k] = (a_r * r[0] + a_d * dvec[0], a_r * r[1] + a_d * dvec[1])
        f0 = centre[grid['filmed']]
        cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
        pts = []
        for k, cell in enumerate(cells):
            ox, oy = centre[k][0] - f0[0], centre[k][1] - f0[1]
            cell['off'] = [round(ox, 4), round(oy, 4)]
            front = wd / 2 + 0.9 * label_h / math.sin(el)
            lx, ly = cx + ox + u[0] * front, cy + oy + u[1] * front
            cell['label_at'] = [round(lx, 4), round(ly, 4)]
            cell['label_w'] = round(label_w, 4)
            pts += [(x + ox, y + oy) for x, y in corners]
            pts += [(lx + r[0] * label_w / 2, ly + r[1] * label_w / 2),
                    (lx - r[0] * label_w / 2, ly - r[1] * label_w / 2)]
        grid['cols'], grid['rows'] = cols, rows
        home = self.home

        def quad(corners):
            # the corners in one order round the centre, starting far left on
            # screen, so two quads' corners pair up for a zoom between them
            mr = sum(dot(c, r) for c in corners) / 4
            md = sum(dot(c, dvec) for c in corners) / 4
            return sorted(corners, key=lambda c: (math.atan2(dot(c, dvec) - md, dot(c, r) - mr)
                                                  - math.radians(135)) % (2 * math.pi))

        def cell_quad(k):
            ox, oy = cells[k]['off']
            x0, y0, x1, y1 = home[0] + ox, home[1] + oy, home[2] + ox, home[3] + oy
            return quad([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
        # the grid's own rectangle, along the screen's axes (no empty corners)
        lo_r, hi_r = min(dot(p, r) for p in pts) - 1.0, max(dot(p, r) for p in pts) + 1.0
        lo_d, hi_d = min(dot(p, dvec) for p in pts) - 1.0, max(dot(p, dvec) for p in pts) + 1.0
        g = quad([(a_r * r[0] + a_d * dvec[0], a_r * r[1] + a_d * dvec[1])
                  for a_r, a_d in ((lo_r, lo_d), (hi_r, lo_d), (hi_r, hi_d), (lo_r, hi_d))])
        return {'filmed': cell_quad(grid['filmed']), 'grid': g, 'best': cell_quad(grid['best'])}

    def finish(self):
        """The timeline: each frame with the camera's azimuth and
        elevation. The camera orbits SWEEP either side of a base azimuth,
        chosen so that the tall parts hide the least of the parts that move;
        a view across the board's long side wins a near tie."""
        n = len(self.frames)
        bx0, by0, bx1, by1 = self.pcb.board_info.board_bounds
        long_x = (bx1 - bx0) >= (by1 - by0)
        scores = []
        for base in range(0, 360, 10):
            # 0: the camera south of the board; across the long side is
            # 90/270 for a tall board, 0/180 for a wide one
            across = abs(math.cos(math.radians(base))) if long_x else abs(math.sin(math.radians(base)))
            scores.append((self._hidden(base) * (1.0 + 0.25 * (1.0 - across)) + 0.5 * (1.0 - across),
                           base))
        self.base_az = min(scores)[1]
        az = [round(self.base_az + SWEEP * (2.0 * i / max(1, n - 1) - 1.0), 4) for i in range(n)]
        grid = self.scene.get('grid')
        if grid:
            at = next(i for i, f in enumerate(self.frames) if 'gv' in f)
            boxes = self._lay_grid(grid, az[at])
        # the board turns over about the screen's horizontal as the turn
        # starts, held for the whole turn while the camera orbits on
        turn_az = next((az[i] for i, f in enumerate(self.frames) if 'flip' in f), None)
        out = []
        for i, f in enumerate(self.frames):
            g = dict(f)
            g['az'] = az[i]
            g['el'] = ELEV
            if 'flip' in g:
                g['flip'] = list(g['flip']) + [turn_az]
            if 'gv' in g:       # a grid frame's view: the zoom out to the seeds, or back in
                kind, t = g.pop('gv')
                a, b = ((boxes['filmed'], boxes['grid']) if kind == 'out' else
                        (boxes['grid'], boxes['best']))
                # the four ground corners to fit, each moving to its fellow
                pts = [_lerp(p, q, t) for p, q in zip(a, b)]
                g['pts'] = [[round(x, 3), round(y, 3)] for x, y in pts]
                g['view'] = [round(min(x for x, _ in pts), 3), round(min(y for _, y in pts), 3),
                             round(max(x for x, _ in pts), 3), round(max(y for _, y in pts), 3),
                             0.0, round(self.home[5], 3)]
            out.append(g)
        return {'fps': self.a.fps, 'width': self.a.width, 'height': self.a.height,
                'refs': self.refs, 'base_az': self.base_az, 'frames': out}


def attach_models(a, rec):
    """The parts' KiCad models (models.py), named in the scene; a part
    without one keeps its box. Prints what fell back."""
    sc = rec.scene
    sc['tint'] = max(0.0, min(1.0, a.tint))
    if a.no_models:
        print('record: models off (--no-models): every part is a box')
        return
    import models
    t0 = time.time()
    info, why = models.export(rec.model_src[0], rec.model_src[1], rec.refs, cli=a.kicad_cli)
    if info is None:
        print(f'record: no part models ({why}); every part is a box')
        return
    matched = set(info['matched'])
    for p in sc['parts']:
        p['model'] = p['ref'] in matched
    # a DNP part is not populated: counted apart, never as a fallback (#1350)
    dnp = sorted(p['ref'] for p in sc['parts'] if p.get('dnp'))
    fitted = [p for p in sc['parts'] if not p.get('dnp')]
    boxed = sorted(p['ref'] for p in fitted if not p['model'] and p['body'])
    bare = sorted(p['ref'] for p in fitted if not p['model'] and not p['body'])
    sc['glb'] = {'path': info['path'], 'matched': sorted(matched), 'found': info['found']}
    rec.model_report = {'models': len(matched), 'parts': len(fitted), 'boxes': boxed,
                        'bare': bare, 'dnp': dnp, 'cached': info['cached']}
    print(f"record: {len(matched)} of {len(fitted)} populated parts drawn from their KiCad "
          f"model ({'cached' if info['cached'] else f'exported in {time.time() - t0:.1f} s'}"
          + (f"; {info['note']}" if info['note'] else '') + "); "
          + (f"{len(boxed)} fell back to a box ({', '.join(boxed)})" if boxed else 'none fell back to a box')
          + (f"; {len(bare)} have no model and nothing assembled ({', '.join(bare)})" if bare else '')
          + (f"; drawn from models found on the web: {', '.join(info['found'])}"
             if info['found'] else '')
          + (f"; {len(dnp)} DNP part(s) not populated, so drawn as pads: "
             f"{', '.join(dnp[:8])}{'...' if len(dnp) > 8 else ''}" if dnp else ''))
    # a model the skill already looked for and recorded as not found is not asked for again
    import find_models
    idx = find_models.load_index()
    sought = [m for m in info['missing']
              if (idx.get(find_models._key_of(m)) or {}).get('status') != 'none']
    if sought:
        print(f"record: {len(sought)} model(s) the installed library lacks; the make-3d-movie "
              f"skill looks for them on the web (python3 movie3d/find_models.py missing "
              f"{rec.model_src[2]})")


def own_args(ap, work_required=True):
    ap.add_argument('--work', required=work_required,
                    help='directory for scene.json and timeline.json' + ('' if work_required else
                         " (default: OUT's name + _movie3d)"))
    ap.add_argument('--frames', type=int, default=FRAMES,
                    help=f'anneal frames, tweens included (default {FRAMES})')
    ap.add_argument('--tween', type=int, default=TWEEN,
                    help=f'frames per anneal step: --frames / --tween snapshots, each played '
                         f'as a move (default {TWEEN}; 1 = cuts, as the 2D movie)')
    ap.add_argument('--speedup', type=float, default=SPEEDUP,
                    help=f'how much faster the anneal plays at its end (default {SPEEDUP:g})')
    ap.add_argument('--fps', type=float, default=FPS, help=f'frames per second (default {FPS:g})')
    ap.add_argument('--no-models', action='store_true',
                    help="draw every part as a box (default: each part's KiCad 3D model, "
                         'exported by kicad-cli and cached in movie3d/cache/)')
    ap.add_argument('--kicad-cli', help='the kicad-cli to export the models with (default: '
                                        '$KICAD_CLI, else the installed KiCad)')
    ap.add_argument('--tint', type=float, default=0.4,
                    help="how far each model's colours are tinted toward its group's colour, "
                         '0-1 (default 0.4)')
    ap.add_argument('--width', type=int, default=1000, help='frame width, px (default 1000)')
    ap.add_argument('--height', type=int, default=560, help='frame height, px (default 560)')


def place(a, rest):
    """Run place_simple (`rest`: BOARD OUT and its options) with a Recorder
    hooked in. Returns (place_simple's exit code, the recorder)."""
    rec = Recorder(a)
    install(rec)
    import place_simple
    os.makedirs(a.work, exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(rest[1])), exist_ok=True)
    return place_simple.main(rest), rec


def write(a, rec):
    """The parts' models attached, scene.json and timeline.json written into
    a.work. Returns (scene path, timeline path, frame count)."""
    attach_models(a, rec)
    sp, tp = os.path.join(a.work, 'scene.json'), os.path.join(a.work, 'timeline.json')
    tl = rec.finish()           # first: it lays a flow's seed grid out into the scene
    with open(sp, 'w', encoding='utf-8') as f:
        json.dump(rec.scene, f)
    with open(tp, 'w', encoding='utf-8') as f:
        json.dump(tl, f)
    return sp, tp, len(rec.frames)


def record(a, rest):
    """The placement movie's recording: place, the placed board held,
    written. Returns (place_simple's exit code, write()'s paths or None)."""
    rc, rec = place(a, rest)
    if rec.scene is None:
        return rc, None
    rec.placed(rc, max(1, int(round(END_S * a.fps))))
    rec.turn_over()
    return rc, write(a, rec)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0], allow_abbrev=False)
    own_args(ap)
    a, rest = ap.parse_known_args(argv)
    if len(rest) < 2 or rest[0].startswith('-') or rest[1].startswith('-'):
        ap.error('BOARD and OUT come first')
    rc, got = record(a, rest)
    if got:
        print(f"record: {got[2]} frames -> {got[0]}, {got[1]}")
    return rc


if __name__ == '__main__':
    sys.exit(main())
