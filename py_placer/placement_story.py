"""The placement movie's story: place_simple's run, shot by shot.

Shared by the 2D movie (`place_movie.py`, which draws each shot with
route_render) and the 3D movie (`movie3d/`, which records each shot as a
timeline row its three.js page draws). `install(story)` hooks a Story into
place_simple's run: the hooks only read the state and pass every call
through, so the placement is the one place_simple makes.

The Story decides what happens when -- the piles, the fixed parts flying to
their places, the faces and poses, the scatter, the edge seats, the anneal's
schedule and captions, the best placement, placed -- and draws nothing. A
renderer subclasses it and supplies:

  setup(pcb, board, args)   its own state, once, before the first shot;
  shot(label, moved=(), times=1, given=None, poses=None, key=True)
                            one frame of the state as it stands: `poses`
                            draws those parts at another pose (a glide),
                            `given` those parts from a footprint of their own
                            (the board as given), `key` the key band;
  bounds, frame_size        the board's box as it frames it, and (W, H) px;
  view_of(box, wide=False), set_view(view)
                            the view that shows `box` (`wide`: the piles in
                            it too), and moving to it.

Where a renderer plays a beat its own way it overrides that beat:
`intro_times`, `fly_frames`, `fly_shot`, `after_fly`, `on_scatter`,
`anneal_snapshots`, `snapshot`, `on_annealed`, `_pile_scale`,
`_wide_extent`. The defaults are the 2D movie's.
"""
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)

import math
import os

FRAMES = 120        # anneal frames per run
SPEEDUP = 4.0       # the anneal plays this much faster at its end than at its start
GLIDE = 10          # frames the pile takes to reach the places it is seeded at
PILE_GAP = 2.0      # mm off the board's edge the pile is drawn, at most
FLY = 3             # frames each fixed part takes to fly from its pile to its place
STEP_HOLD = 0.5     # seconds a step's frame holds


def _ease(t):
    return t * t * (3 - 2 * t)


def _lerp(a, b, t):
    return tuple(x + (y - x) * t for x, y in zip(a, b))


def _union(rects):
    rects = list(rects)
    return (min(q[0] for q in rects), min(q[1] for q in rects),
            max(q[2] for q in rects), max(q[3] for q in rects))


def _moved(a, b):
    return (abs(a.x - b.x) > 1e-6 or abs(a.y - b.y) > 1e-6
            or abs(((a.rotation or 0) - (b.rotation or 0) + 180) % 360 - 180) > 1e-6
            or a.layer != b.layer)


class Story:
    """The placement run as shots. `start` is called with place_simple's
    built state; the anneal's hooks call `on_*`."""

    def __init__(self, args):
        self.a = args
        self.run = 0
        self.wide = None        # the view with the pile in it, when it is off the board

    # -- what a renderer supplies -------------------------------------------
    def setup(self, pcb, board, args):
        pass

    def shot(self, label, moved=(), times=1, given=None, poses=None, key=True):
        raise NotImplementedError

    @property
    def bounds(self):
        return self.pcb.board_info.board_bounds

    @property
    def frame_size(self):
        raise NotImplementedError

    def view_of(self, box, wide=False):
        return box

    def set_view(self, view):
        pass

    # -- the beats a renderer may play its own way --------------------------
    def intro_times(self):
        return self.hold

    def fly_frames(self, n):
        return FLY

    def fly_shot(self, label, ref, frm, to, t, poses):
        self.shot(label, moved=[ref], given=self.given, poses=poses)

    def after_fly(self, poses):
        pass

    def anneal_snapshots(self):
        return self.a.frames

    def snapshot(self, label, moved, annealer):
        self.shot(label, moved=moved)

    def _pile_scale(self, u):
        W, H = self.frame_size
        return min(W / (u[2] - u[0]), H / (u[3] - u[1]))

    def _wide_extent(self):
        """What the view with the piles shows: the board, both piles, and
        every part where the board puts it."""
        return ([self.bounds] + list(self._rects_at({**self.pile, **self.fixed_pile}).values())
                + [p.rect() for p in self.state.parts.values()])

    # -- setup ----------------------------------------------------------------
    def start(self, pcb, board, args, built):
        from kicad_parser import parse_kicad_pcb
        self.state, self.model, self.movable, _notes = built
        self.pcb, self.args = pcb, args
        self.hold = max(1, int(round(STEP_HOLD * self.a.fps)))
        self.setup(pcb, board, args)
        # the piles: the movable parts where they lie before the scatter, and
        # the fixed parts in a pile of their own, each drawn PILE_GAP mm off an
        # edge of the board (parking places, not poses): the fixed parts fly to
        # their places first, one at a time, then the movable pile glides in
        st = self.state
        bx0, by0, bx1, by1 = self.bounds
        self.pile = {r: (p.x, p.y, p.rot) for r, p in st.parts.items() if r in self.movable}
        rects = [st.parts[r].rect() for r in self.pile]
        dx = dy = 0.0
        mside = None
        if rects:
            px0, py0 = min(q[0] for q in rects), min(q[1] for q in rects)
            px1, py1 = max(q[2] for q in rects), max(q[3] for q in rects)
            gaps = {'below': py0 - by1, 'above': by0 - py1, 'right': px0 - bx1, 'left': bx0 - px1}
            side, gap = max(gaps.items(), key=lambda kv: kv[1])
            if gap > PILE_GAP:
                move, mside = gap - PILE_GAP, side
                dx = {'right': -move, 'left': move}.get(side, 0.0)
                dy = {'below': -move, 'above': move}.get(side, 0.0)
                self.pile = {r: (x + dx, y + dy, q) for r, (x, y, q) in self.pile.items()}
        # the faces and poses the intent declares were written first: the
        # board as given shows those parts where they were
        self.given, declared = {}, []
        if os.path.abspath(board) != os.path.abspath(args.board):
            given = parse_kicad_pcb(args.board)
            declared = sorted(r for r in pcb.footprints
                              if r in given.footprints and _moved(given.footprints[r], pcb.footprints[r]))
            self.given = {r: given.footprints[r] for r in declared}
        fixed = sorted(r for r in st.parts if r not in self.movable)
        self.fixed_pile = self._fixed_pile([r for r in fixed if r not in self.given], mside)
        # ... and both in view, with the board
        x0, y0, x1, y1 = _union(self._wide_extent())
        self.home = self.view_of(self.bounds)
        if min(x0 - bx0, y0 - by0, bx1 - x1, by1 - y1) < -2.0:
            self.wide = self.view_of((x0 - 1.0, y0 - 1.0, x1 + 1.0, y1 + 1.0), wide=True)
        self.set_view(self.wide or self.home)
        poses = {**self.pile, **self.fixed_pile}
        self.shot(f"input: {len(st.parts)} parts, {len(self.movable)} movable, {len(fixed)} fixed",
                  times=self.intro_times(), given=self.given, poses=poses)
        # the fixed parts to their places, biggest first
        size = {r: (lambda q: (q[2] - q[0]) * (q[3] - q[1]))(st.parts[r].rect()) for r in self.fixed_pile}
        order = sorted(self.fixed_pile, key=lambda r: (-size[r], r))
        fly = self.fly_frames(len(order))
        for k, ref in enumerate(order, 1):
            home = (st.parts[ref].x, st.parts[ref].y, st.parts[ref].rot)
            for i in range(1, fly + 1):
                t = _ease(i / fly)
                poses[ref] = _lerp(self.fixed_pile[ref], home, t)
                self.fly_shot(f"fixed parts: {ref} to its place ({k}/{len(order)})", ref,
                              self.fixed_pile[ref], home, t, poses)
            del poses[ref]
        self.after_fly(poses)
        if declared:
            self.shot(f"faces and poses: {', '.join(declared)} at their declared face or pose",
                      moved=declared, times=self.hold, poses=self.pile)

    def _rects_at(self, poses):
        """{ref: courtyard rect} of each part at its pose in `poses` (its turn
        unchanged: the rect moves with it)."""
        out = {}
        for r, (x, y, _q) in poses.items():
            p = self.state.parts[r]
            a, b, c, d = p.rect()
            out[r] = (a + x - p.x, b + y - p.y, c + x - p.x, d + y - p.y)
        return out

    def _fixed_pile(self, refs, taken):
        """The fixed parts piled PILE_GAP mm off the board's edge -- the edge,
        other than `taken` (the movable pile's), that leaves the board biggest
        in the frame -- each a little off the pile's centre."""
        import random
        if not refs:
            return {}
        st = self.state
        bx0, by0, bx1, by1 = self.bounds
        rs = {r: st.parts[r].rect() for r in refs}
        mw = max(q[2] - q[0] for q in rs.values())
        mh = max(q[3] - q[1] for q in rs.values())
        cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
        anchors = {'above': (cx, by0 - PILE_GAP - mh / 2), 'below': (cx, by1 + PILE_GAP + mh / 2),
                   'left': (bx0 - PILE_GAP - mw / 2, cy), 'right': (bx1 + PILE_GAP + mw / 2, cy)}
        rng = random.Random(0)
        jit = {r: (rng.uniform(-1, 1), rng.uniform(-1, 1)) for r in sorted(refs)}
        reach = min(1.0, 0.15 * max(mw, mh))
        best = None
        for side, (ax, ay) in anchors.items():
            if side == taken:
                continue
            piled = {}
            for r, q in rs.items():
                p = st.parts[r]
                tx, ty = ax + jit[r][0] * reach, ay + jit[r][1] * reach
                piled[r] = (p.x + tx - (q[0] + q[2]) / 2, p.y + ty - (q[1] + q[3]) / 2, p.rot)
            u = _union([self.bounds] + list(self._rects_at({**self.pile, **piled}).values()))
            scale = self._pile_scale(u)
            if best is None or scale > best[0]:
                best = (scale, piled)
        return best[1]

    # -- the run's hooks ------------------------------------------------------
    def on_scatter(self, seed, clustered=False, anchored=False):
        self.run += 1
        if self.run > 1:
            self.shot(f"run {self.run}, a fresh start (the last left a part with no legal "
                      f"pose): scatter (seed {seed})", moved=self.movable, times=self.hold)
            return
        # the pile glides onto the board, each part to the place it is seeded at
        label = (f"scatter: each group's parts round the place its wiring prefers (seed {seed})"
                 if anchored else
                 f"scatter: each group's parts round one random place (seed {seed})" if clustered
                 else f"scatter: each movable part to a random place (seed {seed})")
        seeded = {r: (p.x, p.y, p.rot) for r, p in self.state.parts.items() if r in self.movable}
        for i in range(1, GLIDE + 1):
            t = _ease(i / GLIDE)
            self.shot(label, moved=self.movable,
                      poses={r: _lerp(self.pile[r], seeded[r], t) for r in seeded})
        if self.wide:           # ... and the view closes in on the board
            for i in range(1, 7):
                self.set_view(_lerp(self.wide, self.home, _ease(i / 6)))
                self.shot(label)
            self.set_view(self.home)
        self.shot(label, times=self.hold // 2)

    def on_anneal(self, moves):
        # frame i of F is taken at move N g(i/F): g' grows linearly to
        # SPEEDUP times its start, so the play speeds up that much
        f = max(1, self.anneal_snapshots())
        sp = max(1.0, self.a.speedup)
        g = lambda u: (u + (sp - 1) / 2 * u * u) / ((sp + 1) / 2)  # noqa: E731
        self.marks = [max(1, math.ceil(moves * g(i / f))) for i in range(1, f + 1)]
        self.n, self.k, self.mi, self.T = moves, 0, 0, math.inf
        self.since, self.note = set(), ''

    def on_seats(self, a):
        if a.seats:
            self.shot(f"edge seats: {', '.join(sorted(a.seats))} held to their declared edges",
                      moved=a.seats, times=self.hold)

    def on_total(self, cost):
        self.cost = cost

    def on_overlap(self, ov):
        self.ov = ov

    def on_trial(self, a, new, T, res):
        kept, dc, do = res
        if T > self.T * 1.5:          # the second start of the half-turn trial
            self.k, self.mi, self.note = 0, 0, ' (long edge parts turned half round)'
        self.T = T
        self.k += 1
        if kept:
            self.cost += dc
            self.ov = max(0.0, self.ov + do)
            self.since |= set(new)
        if a.shoving:
            due = self.k % 5 == 0
        else:
            due = self.mi < len(self.marks) and self.k >= self.marks[self.mi]
            while self.mi < len(self.marks) and self.k >= self.marks[self.mi]:
                self.mi += 1
        if due:
            what = 'repair: shoving the overlaps apart' if a.shoving else \
                f"anneal{self.note}: move {self.k}/{self.n}"
            self.snapshot(f"{what} | T {T:.3g} | cost {self.cost:.0f} | overlap {self.ov:.2f} mm2",
                          self.since, a)
            self.since = set()

    def on_annealed(self, res):
        legal = res.get('reseated') or []
        self.shot(f"best placement, cost {res['best']:.0f}"
                  + (f" | to the nearest legal pose: {', '.join(legal)}" if legal else ''),
                  moved=legal, times=self.hold)

    def placed(self, rc, times, handoff=False):
        """The placed board; with `handoff`, the view then opens to the
        whole frame, the key gone, as the routing frames show the board."""
        cost = self.model.total()['cost']
        label = f"placed: cost {cost:.0f}" + (' | parts left illegal (see the log)' if rc == 4 else '')
        self.shot(label, times=times)
        if handoff and self.home != tuple(self.bounds):
            for i in range(1, 5):
                self.set_view(_lerp(self.home, self.bounds, _ease(i / 4)))
                self.shot(label, key=False)


def install(story):
    """Hook the story into place_simple's run. The hooks only read the state
    and pass every call through, so the placement is the one place_simple
    makes."""
    import place_simple
    from placement import simple_anneal
    from placement.simple_anneal import Annealer

    build = place_simple.build

    def _build(pcb, board, doc, base, args, *k, **kw):
        out = build(pcb, board, doc, base, args, *k, **kw)
        model = out[1]
        total = model.total

        def _total(*a, **kk):
            t = total(*a, **kk)
            story.on_total(t['cost'])
            return t
        model.total = _total
        story.start(pcb, board, args, out)
        return out
    place_simple.build = _build

    scatter, anneal = simple_anneal.scatter, simple_anneal.anneal

    def _scatter(model, movable, seed=0, clustered=False, anchored=False):
        scatter(model, movable, seed=seed, clustered=clustered, anchored=anchored)
        story.on_scatter(seed, clustered, anchored)
    simple_anneal.scatter = _scatter

    def _anneal(model, movable, **kw):
        story.on_anneal(kw.get('moves', 11000))
        res = anneal(model, movable, **kw)
        story.on_annealed(res)
        return res
    simple_anneal.anneal = _anneal

    start_t, trial, overlap = Annealer.start_temperature, Annealer.trial, Annealer.overlap

    def _start_t(self, *a, **kw):
        story.on_seats(self)
        return start_t(self, *a, **kw)

    def _trial(self, new, T):
        res = trial(self, new, T)
        story.on_trial(self, new, T, res)
        return res

    def _overlap(self, refs=None):
        ov = overlap(self, refs)
        if refs is None:
            story.on_overlap(ov)
        return ov
    Annealer.start_temperature, Annealer.trial, Annealer.overlap = _start_t, _trial, _overlap
