"""A camera over a placement + routing chain (#431).

The user's ask: an overview establishing shot, a zoom to the block being worked
on, a pan when the work moves elsewhere on the board, and -- explicitly -- the
next moves only play *after* the pan completes.

That last clause is the whole design, turned into a structural rule rather than
a timing hope:

    A frame either MOVES THE CAMERA or CHANGES THE BOARD. Never both.

Every transition therefore happens over a frozen board, and every camera move is
followed by a settle BEAT before any action plays. Without the beat a hard cut
from motion into action reads as though the pan never finished.

Shot planning is a PURE FUNCTION over `Action` records -- no PIL, no pcbnew, no
board -- so the entire camera is testable as data. Keep it that way: this module
must not import PIL at module scope, because `animate_fanout_clearance` (a
pygame tool) imports the easing helpers from here.

Two things this deliberately does NOT do:

* Pretend the back side looks like the front. Going to B does what a person
  does with a real board: it FLIPS, 180 degrees about the vertical axis, and
  every frame after that is mirrored -- because you are now looking at the
  other face. Reading a back-side placement off an un-mirrored X-ray means
  mentally reversing every x coordinate, which is exactly the error a render is
  supposed to remove.
* Twitch. Consecutive loop rounds nudge the same parts, so without hysteresis
  the camera vibrates for a hundred frames saying nothing.
"""
from __future__ import annotations

import math
import os
from typing import List, NamedTuple, Optional, Sequence, Tuple

Rect = Tuple[float, float, float, float]

# Floor on the frames a PART MOVE gets. A placement beat is the one thing in a
# routing film that is not copper appearing, and at a handful of frames it reads
# as a jump cut rather than as a part travelling -- you cannot see WHICH part
# went WHERE, which is the only reason the beat is in the film. Applies to the
# glide and to the runtime budget alike: the budget may cut a pan to nothing,
# and it may not cut a move below this. `tween=0` stays an explicit "no glide,
# cut straight there" and is left alone.
MIN_MOVE_FRAMES = 10

# --- easing (shared; animate_fanout_clearance imports these) -----------------


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def lerp_rect(a: Rect, b: Rect, t: float) -> Rect:
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(4))  # type: ignore


def net_color(net_id: int) -> Tuple[int, int, int]:
    """Stable, well-separated colour per net id (grey for unconnected/0).

    Byte-for-byte the implementation from `animate_fanout_clearance`, which is
    the incumbent and has a committed golden GIF (docs/fanout-cap-placement.gif)
    -- so the shared version must reproduce ITS output, not an improved one.
    """
    import colorsys
    if not net_id or net_id <= 0:
        return (130, 130, 130)
    h = ((net_id * 0.61803398875) % 1.0)
    r, g, b = colorsys.hsv_to_rgb(h, 0.65, 1.0)
    return (int(r * 255), int(g * 255), int(b * 255))


# --- rect helpers ------------------------------------------------------------

def rect_center(r: Rect) -> Tuple[float, float]:
    return ((r[0] + r[2]) / 2, (r[1] + r[3]) / 2)


def rect_size(r: Rect) -> Tuple[float, float]:
    return (r[2] - r[0], r[3] - r[1])


def expand(r: Rect, frac: float) -> Rect:
    w, h = rect_size(r)
    dx, dy = w * frac, h * frac
    return (r[0] - dx, r[1] - dy, r[2] + dx, r[3] + dy)


def union(a: Optional[Rect], b: Optional[Rect]) -> Optional[Rect]:
    if a is None:
        return b
    if b is None:
        return a
    return (min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3]))


def aspect_fit(r: Rect, aspect: float) -> Rect:
    """Grow `r` (never shrink) to exactly `aspect` = width/height.

    Letterboxing done in WORLD space. `Transform` would letterbox anyway, but
    doing it here means `views_for()` returns the rect actually framed, so the
    tests can assert what the viewer sees.
    """
    w, h = rect_size(r)
    w, h = max(w, 1e-6), max(h, 1e-6)
    cx, cy = rect_center(r)
    if w / h < aspect:
        w = h * aspect
    else:
        h = w / aspect
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def clamp_min_size(r: Rect, min_size: float) -> Rect:
    w, h = rect_size(r)
    cx, cy = rect_center(r)
    w, h = max(w, min_size), max(h, min_size)
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def iou(a: Rect, b: Rect) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    if inter <= 0:
        return 0.0
    aw, ah = rect_size(a)
    bw, bh = rect_size(b)
    return inter / (aw * ah + bw * bh - inter)


# --- the shot list -----------------------------------------------------------

class Action(NamedTuple):
    """Something that changes the board: a placement round, or a routing step."""
    kind: str                 # 'place' | 'route'
    label: str
    focus: Optional[Rect]     # where it happens; None = the whole board
    side: Optional[str]       # 'F' | 'B' | None
    frames: int = 8           # content frames the producer will emit


class Shot(NamedTuple):
    kind: str                 # 'establish'|'transit'|'flip'|'beat'|'action'|'outro'
    frames: int
    view: Rect                # start view (== end view for non-transit shots)
    view_to: Optional[Rect] = None
    path: Optional[List[Rect]] = None
    label: str = ''
    side: Optional[str] = None
    action: Optional[Action] = None


class CameraOpts(NamedTuple):
    aspect: float = 16 / 9
    establish: int = 12
    outro: int = 10
    beat: int = 2
    pan: int = 8
    travel: int = 14
    zoom: int = 10
    flip: int = 6
    # Hysteresis. Below this overlap (and outside the scale band) the camera
    # moves; above it, it stays put. NOT optional -- consecutive rounds nudge
    # the same parts, and without it the camera vibrates.
    hold_iou: float = 0.70
    scale_band: Tuple[float, float] = (0.7, 1.4)
    # Beyond this fraction of the board diagonal a straight pan is an
    # unreadable smear, so the camera pulls back, crosses, and dives in.
    travel_frac: float = 0.35
    # How far in the camera may ever go, as a multiple of the whole-board view.
    # Expressed as MAGNIFICATION rather than an absolute size because "1/12 of
    # the diagonal" means a gentle zoom on a 300-part board and a 4x dive into
    # featureless copper on a 13-part one -- you lose the context that makes the
    # shot readable. 3x keeps the board recognisable at every zoom level.
    max_zoom: float = 3.0
    min_view_mm: float = 8.0


def _min_view(overview: Rect, o: CameraOpts) -> float:
    w, h = rect_size(overview)
    return max(o.min_view_mm, math.hypot(w, h) / max(1.0, o.max_zoom))


def plan_shots(actions: Sequence[Action], overview: Rect,
               o: Optional[CameraOpts] = None) -> List[Shot]:
    """The shot list for a chain. Pure: no PIL, no board, no IO."""
    o = o or CameraOpts()
    overview = aspect_fit(overview, o.aspect)
    mv = _min_view(overview, o)
    shots: List[Shot] = [Shot('establish', o.establish, overview,
                              label='overview')]
    cur, cur_side = overview, None

    for act in actions:
        tgt = overview if act.focus is None else \
            aspect_fit(clamp_min_size(expand(act.focus, 0.15), mv), o.aspect)

        # 1. A side change is not a pan: the two views occupy the same XY. It
        #    gets its own shot, alone, BEFORE any movement, so only one thing
        #    happens at a time.
        if act.side is not None and act.side != cur_side:
            # Adopting a side for the first time is not a flip -- there is
            # nothing to flip FROM, and a transition shot there just delays the
            # opening for no information.
            if cur_side is not None:
                shots.append(Shot('flip', o.flip, cur,
                                  label=f"-> {act.side} side", side=act.side))
            cur_side = act.side

        # 2. Move, unless we are already close enough (hysteresis).
        moved = False
        if not _close_enough(cur, tgt, o):
            cw, ch = rect_size(cur)
            tw, th = rect_size(tgt)
            d = math.dist(rect_center(cur), rect_center(tgt))
            bw, bh = rect_size(overview)
            if d > o.travel_frac * math.hypot(bw, bh):
                # dolly out -> cross -> dolly in. A pulled-back waypoint keeps
                # the viewer oriented and is what makes a long move readable.
                way = aspect_fit(union(cur, tgt), o.aspect)
                way = _shrink_to(way, overview)
                # When the camera is ALREADY pulled back (the opening move out
                # of the overview), the waypoint coincides with the start and
                # the first leg is a no-op that eats half the shot's frames.
                path = [cur, tgt] if _same_rect(way, cur) else [cur, way, tgt]
                shots.append(Shot('transit', o.travel, cur, tgt,
                                  path=path, label='travel', side=cur_side))
            else:
                kind_frames = o.zoom if abs((tw * th) - (cw * ch)) > 1e-9 else o.pan
                shots.append(Shot('transit', kind_frames, cur, tgt,
                                  path=[cur, tgt], label='pan', side=cur_side))
            cur = tgt
            moved = True

        # 3. Settle. THE clause: the moves play only once the camera has
        #    arrived, and a hard cut from motion into action does not read that
        #    way to a viewer.
        if moved:
            shots.append(Shot('beat', o.beat, cur, label='', side=cur_side))

        shots.append(Shot('action', act.frames, cur, label=act.label,
                          side=cur_side, action=act))

    if cur != overview:
        shots.append(Shot('transit', o.outro, cur, overview,
                          path=[cur, overview], label='overview',
                          side=cur_side))
    return shots


def _close_enough(cur: Rect, tgt: Rect, o: CameraOpts) -> bool:
    cw, ch = rect_size(cur)
    tw, th = rect_size(tgt)
    scale = (tw * th) / max(cw * ch, 1e-9)
    return (iou(cur, tgt) >= o.hold_iou
            and o.scale_band[0] <= math.sqrt(scale) <= o.scale_band[1])


def _same_rect(a: Rect, b: Rect, tol: float = 1e-6) -> bool:
    return all(abs(a[i] - b[i]) <= tol for i in range(4))


def _shrink_to(way: Rect, overview: Rect) -> Rect:
    """A travel waypoint never pulls back further than the whole board."""
    ww, wh = rect_size(way)
    ow, oh = rect_size(overview)
    return overview if (ww >= ow and wh >= oh) else way


def views_for(shot: Shot) -> List[Rect]:
    """One view per frame. Sampled by CUMULATIVE ARC LENGTH along the path --
    a per-leg lerp stalls at the waypoint of a 2-leg travel."""
    n = max(1, shot.frames)
    if shot.kind != 'transit' or not shot.path or len(shot.path) < 2:
        return [shot.view] * n
    path = shot.path
    segs = [math.dist(rect_center(path[i]), rect_center(path[i + 1]))
            + abs(rect_size(path[i])[0] - rect_size(path[i + 1])[0])
            for i in range(len(path) - 1)]
    total = sum(segs) or 1e-9
    cum = [0.0]
    for s in segs:
        cum.append(cum[-1] + s)
    out = []
    for k in range(n):
        u = smoothstep(k / (n - 1)) if n > 1 else 1.0
        dist = u * total
        i = 0
        while i < len(segs) - 1 and dist > cum[i + 1]:
            i += 1
        local = (dist - cum[i]) / (segs[i] or 1e-9)
        out.append(lerp_rect(path[i], path[i + 1], max(0.0, min(1.0, local))))
    out[-1] = path[-1]      # land exactly on target: no jump on arrival
    return out


def apply_budget(shots: Sequence[Shot], seconds: float, fps: float
                 ) -> List[Shot]:
    """Scale camera shots to fit a runtime budget.

    Content frames (`action`) are cut LAST and only if scaling everything else
    was not enough -- a movie that races past the moves to preserve a pan is
    backwards.
    """
    if not seconds or seconds <= 0:
        return list(shots)
    budget = int(seconds * fps)
    total = sum(s.frames for s in shots)
    if total <= budget:
        return list(shots)
    floors = {'transit': 3, 'beat': 1, 'establish': 4, 'outro': 3, 'flip': 2}
    out = list(shots)
    content = sum(s.frames for s in out if s.kind == 'action')
    camera = total - content
    room = max(0, budget - content)
    k = (room / camera) if camera else 1.0
    out = [s._replace(frames=max(floors.get(s.kind, 1), int(s.frames * k)))
           if s.kind != 'action' else s for s in out]
    if sum(s.frames for s in out) > budget:
        over = sum(s.frames for s in out) - budget
        # Only now touch the content, proportionally. A ROUTING beat may go to
        # 2; a PART MOVE may not go below MIN_MOVE_FRAMES -- squeezing the
        # travel out of a placement round to save a pan is the trade this
        # function exists to refuse.
        acts = [i for i, s in enumerate(out) if s.kind == 'action']
        for i in acts:
            floor = (MIN_MOVE_FRAMES if (out[i].action and
                                         out[i].action.kind == 'place') else 2)
            share = max(floor, out[i].frames - max(1, over // max(1, len(acts))))
            out[i] = out[i]._replace(frames=share)
    return out


def total_frames(shots: Sequence[Shot]) -> int:
    return sum(s.frames for s in shots)


def describe(shots: Sequence[Shot], fps: float = 6.0) -> str:
    n = total_frames(shots)
    bits = [f"{len(shots)} shot(s), {n} frames, {n / max(fps, 1e-9):.1f}s"]
    for s in shots:
        bits.append(f"  {s.kind:9s} {s.frames:3d}f  {s.label}")
    return "\n".join(bits)


# ---------------------------------------------------------------------------
# Stage: drives the camera and part motion through the EXISTING Movie
# ---------------------------------------------------------------------------
class Stage:
    """Camera + moving parts for a placement chain, as a driver over `Movie`.

    `Movie` is not modified at all. This holds the real `BoardRenderer` and
    mutates exactly two things between frames:

      * ``r.pcb``  -> the board state of that step. With ``dynamic_zones=True``
        (which ``build_boards`` already passes) ``frame()`` draws zones and pads
        per frame from ``self.pcb``, so re-pointing it animates the FOOTPRINTS
        with no new drawing code. Verified: the same call with
        ``dynamic_zones=False`` produces an identical image, because pads are
        baked into ``_base`` there.
      * ``r.set_view(rect)`` -> the camera.

    With ``stage=None`` in ``build_boards`` every hook below is skipped, so the
    routing movie is bit-for-bit unchanged. That is the point: this adds a
    driver, not a second pipeline.
    """

    def __init__(self, rounds, work_dir=None, *, opts=None, fps=6.0,
                 budget=60.0, moving_parts=True, tween=8, quiet=False):
        self.rounds = list(rounds)          # [dict] from loop_round{N}.json
        self.work_dir = work_dir
        self.opts = opts
        self.fps = fps
        self.budget = budget
        self.moving_parts = moving_parts
        # 0 = no glide: cut straight to the new placement. The camera work is
        # what carries the story; the glide is decoration, and on a long run it
        # is most of the runtime. Anything else is clamped to >=2, since a
        # 1-frame "glide" is just a cut with extra steps.
        self.tween_frames = (0 if int(tween) <= 0
                             else max(MIN_MOVE_FRAMES, int(tween)))
        self.quiet = quiet
        self.movie = None
        self.r = None
        self._mirror = False      # True once flipped to the back
        self._mark = 0            # frames emitted before the last step
        self.layers = None
        self.shots = []
        self._queue = []
        self._by_board = {}
        self._log = []                      # (shot_kind, first_frame, last_frame)

    # -- wiring ---------------------------------------------------------
    def attach(self, movie, renderer, layers):
        self.movie = movie
        self.r = renderer
        self.layers = layers
        self._overview = renderer.bounds
        # #1036: a chain that opens on an unplaced PILE has parts outside the
        # outline (run 32's glasgow pile reaches 13 mm below the board). The
        # overview must hold where the parts come FROM, or the glide starts
        # off-frame; `synth_rounds` records that as each round's `extent`.
        for rd in self.rounds:
            ext = rd.get('extent')
            if ext and len(ext) == 4:
                b = self._overview
                self._overview = (min(b[0], ext[0]), min(b[1], ext[1]),
                                  max(b[2], ext[2]), max(b[3], ext[3]))
        for rd in self.rounds:
            if rd.get('board'):
                self._by_board[self._key(rd['board'])] = rd
        self._plan()
        # THE OPENING FRAME holds the pile (#1036 verifier): `build_boards`
        # snapshots "input" right after this, and at the board's own bounds
        # an off-board pile was clipped -- then the establishing shot jumped
        # out to the overview three frames later.
        if tuple(self._overview) != tuple(renderer.bounds):
            self._aim(self._overview)

    def _aim(self, view):
        """Point the renderer at `view`, accounting for the flip.

        The camera plans in un-mirrored world space, but once we are looking at
        the BACK every frame is mirrored on the way out. Handing the transform
        the raw rect therefore lands the zoom on the MIRROR IMAGE of the target
        -- the parts you asked to see end up on the far side of the frame, or
        off it entirely once you are zoomed in. Mirror the rect about the
        board's vertical centre line first, so that after the frame is flipped
        the target is where the camera said it would be.
        """
        if view is not None and self._mirror:
            b = self._overview
            cx2 = b[0] + b[2]
            view = (cx2 - view[2], view[1], cx2 - view[0], view[3])
        self.r.set_view(view)

    def _snap(self, label):
        """Snapshot, mirrored when we are looking at the back.

        Applied at the FRAME level rather than inside the renderer so that
        everything -- camera frames, part tweens, and the copper the routing
        steps reveal through Movie's own path -- flips together. A movie where
        the board is mirrored but the tracks are not would be worse than no
        flip at all.
        """
        if not self._mirror:
            self.movie.snapshot(label)
            return
        # Render WITHOUT the caption, mirror the board, then stamp the caption
        # upright. Mirroring a frame that already carries its label reverses the
        # text and throws it into the opposite corner -- it reads as a rendering
        # fault, not as "you are looking at the back".
        from PIL import ImageOps
        img = self.r.frame(segments=list(self.movie.live_s.values()),
                           vias=list(self.movie.live_v.values()),
                           zone_net_ids=self.movie.revealed_zones)
        img = ImageOps.mirror(img)
        if label:
            self.r._label(img, label)
        self.movie.frames.append(img)

    def _mirror_new_frames(self, label=''):
        """Mirror frames appended by code that does not go through _snap --
        i.e. the routing steps, which Movie renders and captions itself."""
        if not self._mirror:
            self._mark = len(self.movie.frames)
            return
        from PIL import ImageDraw, ImageOps
        for i in range(self._mark, len(self.movie.frames)):
            fr = ImageOps.mirror(self.movie.frames[i])
            # Their caption was baked in before we saw the frame, so it mirrored
            # with the board. Clear the whole top strip -- the caption bar's own
            # territory -- and stamp it again upright.
            d = ImageDraw.Draw(fr)
            d.rectangle([0, 0, fr.size[0], max(18, fr.size[1] // 26)],
                        fill=tuple(self.r.bg))
            if label:
                self.r._label(fr, label)
            self.movie.frames[i] = fr
        self._mark = len(self.movie.frames)

    def _emit(self, kind, views, label, side=None):
        """One frame per view, at a FROZEN board. A frame either moves the
        camera or changes the board -- never both."""
        start = len(self.movie.frames)
        if kind == 'flip':
            self._emit_flip(views, label, side)
        else:
            for v in views:
                self._aim(v)
                self._snap(label)
        self._mark = len(self.movie.frames)
        self._log.append((kind, start, len(self.movie.frames)))

    def _plan(self):
        """Plan the WHOLE chain once, then consume shots per step.

        Planning per step would restart from the overview every time -- an
        establishing shot before each round and never a transit, because the
        camera would always already be where a fresh plan puts it. Hysteresis
        and travel only mean anything across a sequence.
        """
        from kicad_parser import parse_kicad_pcb
        acts = []
        for rd in self.rounds:
            if not rd.get('accepted') or not rd.get('board'):
                continue
            pcb = None
            moved = rd.get('moved') or []
            # A SYNTHESISED round (`synth_rounds`) records an ABSOLUTE board
            # path and there is no work dir: `make_movie` on a board list
            # passes ''. Requiring a work dir here meant every hand-driven
            # chain planned its placement shots with no box, so the camera
            # never zoomed on the parts that moved (#1036).
            if moved and (self.work_dir or os.path.isabs(rd['board'])):
                try:
                    pcb = parse_kicad_pcb(os.path.join(self.work_dir or '',
                                                       rd['board']))
                except Exception:
                    pcb = None
            # The shot frames where the parts come FROM as well as where they
            # land (#1036): focused on the destination alone, the camera
            # zoomed onto the board before the glide and cut the off-board
            # pile out of the frame, so parts glided in from outside it.
            _focus = (_moved_bbox(pcb, moved) if (pcb and moved) else None)
            _ext = rd.get('extent')
            if _focus and _ext and len(_ext) == 4:
                _focus = _union_box(_focus, tuple(_ext))
            acts.append(Action('place', f"round {rd['round']}",
                               _focus,
                               _moved_side(pcb, moved) if pcb else None,
                               frames=self.tween_frames))
        self.shots = plan_shots(acts, self._overview, self.opts)
        if self.budget:
            self.shots = apply_budget(self.shots, self.budget, self.fps)
        self._queue = list(self.shots)

    def _drain_until_action(self, label):
        """Emit every queued camera shot up to the next action, then stop.

        This is the sequencing contract: the content frames are emitted by the
        caller AFTER this returns, so a move never plays over a moving camera.
        """
        while self._queue:
            s = self._queue[0]
            if s.kind == 'action':
                self._queue.pop(0)
                return s
            self._queue.pop(0)
            self._emit(s.kind, views_for(s), s.label or label, side=s.side)
        return None

    def _emit_flip(self, views, label, side):
        """Turn the board over: a 180-degree flip about the VERTICAL axis.

        This is what a person does with a real board, and it is why the frames
        after it are mirrored -- you are looking at the other face. The
        animation is the honest one: the board narrows to an edge as it rotates
        past 90 degrees, then opens out again already reversed, so the moment
        the handedness changes is visible rather than implied.

        Rendering the far face means dropping the near copper layer, so the
        traces you see belong to the side you are looking at.
        """
        from PIL import Image, ImageOps
        from route_render import BoardRenderer

        to_back = (side == 'B')
        want = f'{side}.Cu'
        layers = [ln for ln in self.r.copper_layers
                  if ln == want or ln not in ('F.Cu', 'B.Cu')]
        far = None
        if want in self.r.copper_layers and layers:
            # theme= is NOT optional here. This is the SECOND BoardRenderer
            # in the system and no test covers it:
            # tests/test_431_placement_movie.py:92-121 pins the renderer count
            # on the NO-STAGE path, and this is the stage path. A second
            # renderer that does not inherit the theme is exactly the drift
            # render_theme exists to prevent (#1011).
            far = BoardRenderer(self.r.pcb, size=self.r.W, supersample=self.r.ss,
                                layers=layers, dynamic_zones=self.r.dynamic_zones,
                                view=getattr(self.r, '_view', None),
                                theme=getattr(self.r, 'theme', None))

        segs = list(self.movie.live_s.values())
        vias = list(self.movie.live_v.values())
        zones = self.movie.revealed_zones
        # No label on the faces: the board rotates, the HUD does not. Stamping
        # it before the flip transform squashes and reverses the text with the
        # board, which reads as a rendering fault rather than a flip.
        near_img = self.r.frame(segments=segs, vias=vias, zone_net_ids=zones)
        far_img = (far.frame(segments=segs, vias=vias, zone_net_ids=zones)
                   if far is not None else near_img)
        if self._mirror:                      # we were already looking at B
            near_img = ImageOps.mirror(near_img)
        # The far face, once turned toward us, reads mirrored relative to the
        # near one.
        far_img = far_img if self._mirror else ImageOps.mirror(far_img)

        W, H = near_img.size
        bg = tuple(self.r.bg)
        n = max(2, len(views))
        for i in range(n):
            t = (i + 1) / n
            ang = math.pi * t
            k = abs(math.cos(ang))            # 1 -> 0 -> 1: edge-on at halfway
            face = near_img if t < 0.5 else far_img
            w = max(1, int(round(W * k)))
            frame = Image.new('RGB', (W, H), bg)
            frame.paste(face.resize((w, H), Image.BILINEAR), ((W - w) // 2, 0))
            self.r._label(frame, label)       # upright, after the rotation
            self.movie.frames.append(frame)
        # From here on we are looking at the other face.
        self._mirror = to_back
        self._mark = len(self.movie.frames)

    # -- the build_boards hooks -----------------------------------------
    def handles(self, board):
        """True when `enter_step` will intercept this board's step."""
        return (self.moving_parts
                and self._key(board) in self._by_board)

    def _key(self, board):
        """A board's identity for `_by_board`: its RESOLVED ABSOLUTE path
        (#1036 review). Keyed by basename, two chain boards with one name in
        different directories were the same round -- `synth_rounds` records
        absolute paths, a loop sidecar a name relative to the work dir, and
        both resolve here to one spelling."""
        return os.path.normcase(os.path.abspath(
            os.path.join(self.work_dir or '', str(board))))

    def _arrive(self):
        """Call `on_arrive` once, right before the frame the parts LAND on.

        `build_boards` sets it so the lower box's inventory reads the SOURCE
        board through the glide and the destination only from the landing
        frame (#1036) -- it read the destination's "272 of 272 placed" over
        parts still in the pile.
        """
        fn, self.on_arrive = getattr(self, 'on_arrive', None), None
        if fn is not None:
            try:
                fn()
            except Exception:                                  # noqa: BLE001
                pass

    def enter_step(self, label, board, pcb, seg_rows, via_rows):
        """True = this stage handled the step itself.

        A placement round is intercepted: the loop re-routes from scratch each
        round, so routed(N) -> placed(N+1) legitimately loses ALL copper.
        Letting `reveal_delta` handle that calls `remove()`, which flashes red
        and labels it "(rip)" -- a lie. Clear silently, then tween the parts.
        """
        rd = self._by_board.get(self._key(board))
        if rd is None or not self.moving_parts:
            self._settle(label)
            return False
        moved = rd.get('moved') or []
        # The camera shots queued before this action are shot on the board
        # as it WAS (#1036): `build_boards` has already re-pointed `r.pcb` at
        # this step's board, so without the swap the establishing shot showed
        # the finished placement and the parts then jumped back to glide.
        prev = getattr(self, 'prev_pcb', None)
        if prev is not None and prev is not pcb:
            cur, self.r.pcb = self.r.pcb, prev
            try:
                self._drain_until_action(label)
            finally:
                self.r.pcb = cur
        else:
            self._drain_until_action(label)

        # silent clear: the copper of the PREVIOUS round is genuinely gone.
        # That is a fact about the LOOP -- it re-routes from scratch every
        # round -- not about placement. A chain that merely moved some parts
        # keeps the copper it has, so trueup to THIS board instead of to
        # nothing; for a loop round the board carries none and the two are the
        # same call.
        #
        # A SYNTHESISED round keeps the copper it has and does NOT snap to
        # this board's (#1036 review): a board that both moves parts and lays
        # copper -- place_fanout_clearance's vias, a routed step after moves
        # -- lost that copper's trace to a silent trueup. The glide plays
        # first, then `False` hands the step back to `build_boards`, whose
        # normal path reveals its copper (trace or chunks).
        synth = bool(rd.get('synth'))
        if not synth:
            self.movie.reconcile_to([], [], f"{label}: re-placing")
        if moved:
            self._tween(pcb, moved, label)
        else:
            self._arrive()
            self._snap(label)
        self._arrive()          # no-op unless the tween never reached one
        self._mark = len(self.movie.frames)
        return not synth

    def exit_step(self, label):
        # Routing steps render through Movie's own path, not _snap, so their
        # frames have to be flipped here or the board would be mirrored while
        # the tracks landing on it were not.
        self._mirror_new_frames(label)

    def _settle(self, label, n=None):
        """Bring the camera home to the BOARD before a copper step (#1036).

        The placement shots aim at the pile-inclusive overview (`extent`), which
        is right while parts are off the board and wrong once they have
        landed: every routing frame after a glide was drawn at that overview,
        so on run 32 the board filled ~600x370 of its 980x594 box. A short
        glide to the board's own bounds, then the renderer's default view --
        so a routing frame fills its box exactly as a film without a camera
        does. A no-op when the camera is already home.
        """
        from movie_camera import lerp_rect, smoothstep
        cur = getattr(self.r, '_view', None)
        home = self.r.bounds
        if cur is None or tuple(cur) == tuple(home):
            return
        n = n if n is not None else max(2, (self.opts.outro if self.opts
                                            else 10) // 2)
        start = len(self.movie.frames)
        for k in range(n):
            t = smoothstep((k + 1) / n)
            self._aim(lerp_rect(cur, home, t))
            self._snap(label)
        self.r.set_view(None)
        self._log.append(('settle', start, len(self.movie.frames)))

    def outro(self):
        """The closing move, onto the BOARD -- the parts have landed, so the
        pile-inclusive overview would only shrink the finished board."""
        from movie_camera import lerp_rect, smoothstep
        home = self.r.bounds
        cur = getattr(self.r, '_view', None) or home
        n = (self.opts.outro if self.opts else 10)
        start = len(self.movie.frames)
        for k in range(n):
            t = smoothstep((k + 1) / n)
            self._aim(lerp_rect(cur, home, t))
            self._snap("overview")
        self._log.append(('outro', start, len(self.movie.frames)))
        self.r.set_view(None)

    # -- part motion ----------------------------------------------------
    def _tween(self, pcb, moved, label):
        """Glide the moved parts from their source pose to the parsed one.

        Built from the DESTINATION parse, offsetting backwards, always
        recomputed from a cached home pose -- never accumulated, so there is no
        float drift -- and t=1 on the last frame reproduces the parsed board
        EXACTLY, so the hand-off to the copper reveal is seamless.

        Rotation SNAPS rather than tweens: tweening it means re-deriving
        global_x/y from local_x/y plus rect_rotation plus the >=90 deg
        size_x/size_y swap the parser already resolved. Quench rotations are
        90-degree multiples and rare.
        """
        from movie_camera import smoothstep
        # Keyed by REFERENCE: Footprint is unhashable (no __hash__), so it
        # cannot be a dict key.
        home = {}
        deltas = []
        for m in moved:
            ref = m['reference']
            fp = pcb.footprints.get(ref)
            if fp is None:
                continue
            home[ref] = (fp.x, fp.y,
                         [(p, p.global_x, p.global_y) for p in fp.pads],
                         [(p, [list(pt) for pt in (p.polygons or [])])
                          for p in fp.pads if getattr(p, 'polygons', None)])
            deltas.append((ref, fp, m['from'][0] - m['to'][0],
                           m['from'][1] - m['to'][1]))
        if not deltas:
            self._arrive()
            self._snap(label)
            return
        n = self.tween_frames
        start = len(self.movie.frames)
        if n == 0:
            # No glide: one BEFORE frame at the source poses, then one AFTER at
            # the parsed board. The delta still reads -- it is a cut, not a
            # missing beat -- and a long run stops spending most of its runtime
            # on decoration.
            for ref, fp, dx, dy in deltas:
                _offset_to(fp, home[ref], dx, dy)
            self._snap(f"{label}  before ({len(deltas)} part(s))")
            for ref, fp, _dx, _dy in deltas:
                _offset_to(fp, home[ref], 0.0, 0.0)
            self._arrive()
            self._snap(f"{label}  moved {len(deltas)} part(s)")
        else:
            # #1020: the GHOST and the ARROW, through the overlay seam, so
            # they cost no frame geometry. Without them a glide reads as the
            # board assembling itself rather than as these parts moving from
            # there to here -- the viewer sees where a part ARRIVED and never
            # where it came from, which is the question a placement film
            # exists to answer.
            import place_motion
            for i in range(n):
                t = smoothstep((i + 1) / n)
                for ref, fp, dx, dy in deltas:
                    _offset_to(fp, home[ref], dx * (1 - t), dy * (1 - t))
                try:
                    self.movie.overlay = place_motion.ghost_overlay(
                        place_motion.items_from_deltas(deltas, home),
                        getattr(self.r, 'theme', None), t=t)
                except Exception:                              # noqa: BLE001
                    self.movie.overlay = None
                if i == n - 1:
                    self._arrive()      # t = 1: the parts are where they land
                self._snap(f"{label}  moving {len(deltas)} part(s)")
            self.movie.overlay = None
            for ref, fp, _dx, _dy in deltas:      # exact restore
                _offset_to(fp, home[ref], 0.0, 0.0)
        self._log.append(('action', start, len(self.movie.frames)))

    # -- introspection for tests ----------------------------------------
    def frame_log(self):
        return list(self._log)


def _offset_to(fp, home, dx, dy):
    hx, hy, pads, polys = home
    fp.x, fp.y = hx + dx, hy + dy
    for p, gx, gy in pads:
        p.global_x, p.global_y = gx + dx, gy + dy
    for p, orig in polys:                     # custom pad copper is ABSOLUTE
        p.polygons = [[(x + dx, y + dy) for x, y in poly] for poly in orig]


def _moved_bbox(pcb, moved):
    xs, ys = [], []
    for m in moved:
        fp = pcb.footprints.get(m['reference'])
        if fp is None:
            continue
        for p in fp.pads:
            r = max(p.size_x, p.size_y) / 2
            xs += [p.global_x - r, p.global_x + r]
            ys += [p.global_y - r, p.global_y + r]
    if not xs:
        return None
    return (min(xs), min(ys), max(xs), max(ys))


def _union_box(a, b):
    if not a:
        return b
    if not b:
        return a
    return (min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3]))


def _moved_side(pcb, moved):
    """Majority side of the parts that MOVED -- a block can straddle both
    (ulx3s sheet:58d686d9 is 9 back / 11 front), so the side is a property of
    this round's work, not of the block."""
    f = b = 0
    for m in moved:
        fp = pcb.footprints.get(m['reference'])
        if fp is None:
            continue
        if (fp.layer or '').startswith('B'):
            b += 1
        else:
            f += 1
    if not (f or b):
        return None
    return 'B' if b > f else 'F'


def _move_floor_mm():
    try:
        import env_knobs
        return float(getattr(env_knobs, 'MOVIE_MOVE_MIN_MM', 0.5))
    except Exception:                                          # noqa: BLE001
        return 0.5


def synth_rounds(boards, min_mm=None):
    """Round records for a chain that has NO loop_round*.json sidecars.

    Footprint motion is already animated -- but only through a Stage, and a
    Stage was only ever built from `place_route_loop`'s sidecars. Every other
    chain (a hand-driven one, a plan run, anything a person assembled step by
    step) therefore got `stage=None`, and `build_boards` animates COPPER deltas.
    A placement step changes no copper, so it rendered as a single frame: the
    parts jumped, or more often the step vanished from the film entirely.

    The records a Stage needs are derivable from the boards themselves -- the
    poses are right there in both files -- so a missing sidecar is no reason to
    drop the motion. Diff consecutive boards and hand back the same shape
    `load_round_sidecars` returns.

    Every synthesised round is `accepted`: this is a sequence someone chose to
    put in a film, not a search tree with rejects in it. A caller that wants to
    show rejected attempts should say so by passing them and labelling them,
    not by having them inferred here.
    """
    from kicad_parser import parse_kicad_pcb
    out, prev = [], None
    for i, b in enumerate(boards):
        try:
            pcb = parse_kicad_pcb(b)
        except Exception:
            continue
        moved = []
        if prev is not None:
            # PAIRING (#1036 review). By uuid when the uuid names ONE block
            # on BOTH boards; else by key -- except a duplicate reference's
            # `~N` ordinal key when the number of blocks sharing that
            # reference changed, because the ordinals are file order and then
            # name different parts. A uuid two blocks share (a footprint
            # copy-pasted in a text editor, as kicad_files/cap_chain does)
            # identifies neither: pairing on it matched C1 to C2 and reported
            # a board compared with ITSELF as moving parts.
            def _uuid_counts(p):
                c = {}
                for f in p.footprints.values():
                    u = getattr(f, 'uuid', '')
                    if u:
                        c[u] = c.get(u, 0) + 1
                return c
            _uprev, _ucur = _uuid_counts(prev), _uuid_counts(pcb)
            by_uuid = {f.uuid: f for f in prev.footprints.values()
                       if getattr(f, 'uuid', '')
                       and _uprev.get(f.uuid) == 1
                       and _ucur.get(f.uuid) == 1}

            def _base_counts(p):
                c = {}
                for k in p.footprints:
                    b = k.split('~', 1)[0]
                    c[b] = c.get(b, 0) + 1
                return c
            _nprev, _ncur = _base_counts(prev), _base_counts(pcb)
            for ref, fp in pcb.footprints.items():
                old = (by_uuid.get(fp.uuid)
                       if getattr(fp, 'uuid', '') else None)
                if old is None:
                    base = ref.split('~', 1)[0]
                    if (_nprev.get(base) != _ncur.get(base)
                            and _ncur.get(base, 0) > 1):
                        continue
                    old = prev.footprints.get(ref)
                if old is None:
                    continue
                a_ = (round(old.x, 4), round(old.y, 4), round(old.rotation or 0.0, 3))
                b_ = (round(fp.x, 4), round(fp.y, 4), round(fp.rotation or 0.0, 3))
                # THE DISPLACEMENT FLOOR (#1036 review): a 0.05 mm nudge used
                # to switch a whole film to the placement camera. Below
                # `min_mm` a translation is drift, not a move; a rotation is
                # always a move.
                _floor = _move_floor_mm() if min_mm is None else min_mm
                _dist = ((a_[0] - b_[0]) ** 2 + (a_[1] - b_[1]) ** 2) ** 0.5
                if a_ != b_ and (a_[2] != b_[2] or _dist >= _floor):
                    # ROTATION is part of the pose. A part that turns 180 in
                    # place moves no origin at all, and a position-only diff
                    # shows nothing -- which is exactly how a rotation that
                    # relocated a pin to the far side of a package went
                    # unnoticed in a real run.
                    moved.append({'reference': ref, 'from': list(a_), 'to': list(b_)})
        if moved:
            # ONLY a board whose parts moved is a placement beat. Emitting a
            # record for every board would make `enter_step` intercept the
            # routing steps too -- and it clears the copper on the way in.
            refs = {m['reference'] for m in moved}
            # Where the moved parts START and END (#1036): the camera's
            # overview has to hold both, or a glide out of an off-board pile
            # begins off-frame.
            ext = _union_box(_moved_bbox(prev, moved),
                             _moved_bbox(pcb, moved))
            out.append({'schema': 1, 'round': i, 'board': os.path.abspath(b),
                        'accepted': True, 'screened': False, 'synth': True,
                        'moved': sorted(moved, key=lambda m: m['reference']),
                        'extent': list(ext) if ext else None,
                        'n_moved': len(refs)})
        prev = pcb
    return out


def load_round_sidecars(work_dir, accepted_only=False):
    """`loop_round{N}.json` records, ordered by round.

    Keyed on the SIDECARS rather than a loop_round*.kicad_pcb glob: --work-dir
    defaults to the output board's directory, which may hold unrelated boards,
    and mtime ordering would animate a REJECTED round as though it were kept.

    Every round is returned; the callers that want the accepted spine filter on
    `accepted` themselves (`Stage._plan`, `make_movie.placement_chain`), and
    `make_film` wants the dropped ones precisely because they are the search.
    `accepted_only=True` asks for the spine directly.
    """
    import glob
    import json
    out = []
    for p in sorted(glob.glob(os.path.join(work_dir, 'loop_round*.json'))):
        try:
            with open(p, encoding='utf-8') as f:
                doc = json.load(f)
        except Exception:
            continue
        if doc.get('schema') == 1 and 'round' in doc:
            if accepted_only and not doc.get('accepted'):
                continue
            out.append(doc)
    out.sort(key=lambda d: d['round'])
    return out
