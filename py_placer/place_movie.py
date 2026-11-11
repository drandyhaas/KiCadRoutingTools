#!/usr/bin/env python3
"""Make a movie of a placement: place_simple's run, step by step, and with
--route or --route-chain the placement skill's whole flow.

    python3 -X utf8 py_placer/place_movie.py board.kicad_pcb placed.kicad_pcb \\
        --groups groups.json --intent intent.json [--seed 0] [--movie placed.mp4]
    python3 -X utf8 py_placer/place_movie.py board.kicad_pcb placed.kicad_pcb \\
        --groups groups.json --intent intent.json --route ch32v003_usb:6 [--scattered 6 --clustered 6 --judge 6]
    python3 -X utf8 py_placer/place_movie.py board.kicad_pcb placed.kicad_pcb \\
        --groups groups.json --intent intent.json --route-chain wk/<run>/judge_chain.txt

BOARD and OUT come first; every other argument but this tool's own is
place_simple.py's (see `place_simple.py --help`). The placement is exactly
place_simple's for the same seed and start: --seed S is filmed from the start
seed S has in a run of --scattered/--clustered/--anchored (place_simple's
default seeds when none is given), whose counts become the --clustered-seeds
and --anchored-seeds lists that run numbers its seeds by, and a `place_movie:`
line names it. Lists given instead are kept; beside a count that starts the
seed otherwise, they are refused (exit 2).

Each placement frame is the board with every part's pads and courtyard
(movable parts tinted by group, fixed parts dim), the airwires the objective
prices (ground left out: it is poured), the parts that moved since the last
frame outlined in white, and a caption, in the routing movie's style
(`route_movie.py`). The steps:

  1. input: the board's outline, the movable parts in their pile and the
     fixed parts in a pile of their own, each off an edge, in view; then the
     fixed parts fly to their places, one at a time, biggest first;
  2. faces and poses: the parts the intent puts on the other face or at a
     declared pose (when there are any);
  3. scatter: the pile glides onto the board, each part to the random place
     it is seeded at, and the view closes in on the board;
  4. edge seats: the parts declared on an edge, held to it (when any);
  5. anneal: the moves, with the move count, temperature, cost and overlap,
     playing faster as it goes (--speedup times faster by its end); a long
     edge part's half-turn trial and a fresh start after a run that left a
     part illegal are captioned where they happen;
  6. best: the best placement the anneal saw, each part moved to its nearest
     legal pose where it needed one;
  7. placed.

With --route BOARD:SET (a stress-corpus board whose recorded chain routes
this one: `tests/stress/placement_ab/route_placed.py`), it runs the placement
skill's flow first and films it after: the skill's seeds are placed
(--scattered seeds started scattered, then --clustered started clustered, then
the opt-in --anchored started anchored, as place_simple's default seeds), the
--judge legal seeds place_simple names (`judge_order`: buildable and floorplan-clean first,
then the objective's cheapest) are fast-routed (the chain
with `--quick 2`) and scored by quick_score, the best (`quick_score.judge_key`)
is routed in full.

With --route-chain FILE instead (the chain a placement run routes its seeds
with, `judge_chain.txt`: one command per line, its boards named by {in} {out}
{dir} {seed} {run}, in judge_seeds.py --chain's format), the same flow routes
each seed with that chain the way judge_seeds.py routes it, by its own
`judge_seed`: the fast routes with every route.py / route_diff.py step at
`--quick 2`, the best seed's full route with `--quick` taken off, each scored
by quick_score with place_simple's --intent and BOARD as the baseline, and the
best picked by the same `quick_score.judge_key`. The routes land where
judge_seeds puts them, in --work as its run folder (`s<S>_route/`,
`s<S>_fast.score.json`, `s<S>_full_route/`, `s<S>_full.score.json`), and the
chain's commands run from the current folder, as judge_seeds runs them (the
repo root): the movie routes and picks as the run's judge does. --route and
--route-chain are exclusive.

The movie films the best seed's placement -- the one routed in full -- and
goes on:

  8. its fast route, played as the routing movie plays a run;
  9. a zoom out to the judged seeds' fast routes side by side, a box round
     the best, and a zoom into it;
 10. its fast route cleared, and its full route played.

OUT is then the best seed's placement; the work (placed seeds, routes,
scores) is in --work. The movie is an .mp4 through `ffmpeg` when it is on
PATH, else a GIF beside it. Exit codes are place_simple's (2 also when a
--route step fails, a --route-chain file cannot route a seed, or no route
finishes).
"""
#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement'], 'kind': 'actor'}

import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

# the story (what happens when: piles, scatter, anneal, captions) is
# placement_story's, shared with the 3D movie (movie3d/); this file draws it
from placement_story import (FRAMES, SPEEDUP, STEP_HOLD, Story, _ease,  # noqa: E402,F401
                             _lerp, install)

ZOOM = 12           # frames of a zoom between the board and the judged seeds
LABEL_PX = 16       # the line under each judged seed's picture
END_HOLD = 1.0      # seconds on the last frame


class Film(Story):
    """Draws the placement story (`placement_story.Story`) as it stands (or
    at the poses a shot overrides) and feeds the frames to `movie`."""

    def __init__(self, args, movie):
        super().__init__(args)
        self.movie = movie

    # -- the renderer the story draws through ---------------------------------
    def setup(self, pcb, board, args):
        from route_render import BoardRenderer
        from placement.grouping import is_ground, load_groups
        self.r = BoardRenderer(pcb, size=self.a.size, supersample=1, theme=self.a.theme,
                               show_pads=False, show_zones=False)
        th = self.r.theme
        body = th.rgb('board_body')
        mix = lambda c, k: tuple(int(body[i] * (1 - k) + c[i] * k) for i in range(3))  # noqa: E731
        self.c = {'front': th.rgb('place_court_front'), 'back': th.rgb('place_court_back'),
                  'fixed': th.rgb('place_locked'), 'air': th.rgb('place_airwire'),
                  'moved': th.rgb('event_new'), 'pad': th.rgb('pad'),
                  'pad_back': mix(th.rgb('pad'), 0.55)}
        # each group its own colour (outline full, fill toward the board), named in the key
        # (a group of fixed parts only shows no colour, so it takes none)
        groups = [g for g in load_groups(args.groups)['groups']
                  if any(r in self.movable for r in g['refs'])]
        from render_placement import group_palette
        colours = group_palette(len(groups))
        self.line, self.fill, self.key = {}, {}, []
        for g, rgb in zip(groups, colours):
            role = g.get('role')
            self.key.append((rgb, 'solid', g['name'] + (f" ({role})" if role and role != g['name'] else '')))
            for ref in g['refs']:
                if ref in self.movable:
                    self.line[ref], self.fill[ref] = rgb, mix(rgb, 0.55)
        if any(r not in self.line for r in self.movable):
            self.key.append((self.c['front'], 'line', 'in no group'))
        if len(self.movable) < len(self.state.parts):
            self.key.append((self.c['fixed'], 'hatch', 'fixed'))
        self.key.append((self.c['moved'], 'ring', 'just moved'))
        self._key_layout()
        # the airwires' nets: every net but ground, over each part's pads
        self.nets = {}
        for ref, fp in pcb.footprints.items():
            for i, p in enumerate(fp.pads):
                nm = pcb.nets[p.net_id].name if p.net_id in pcb.nets else ''
                if p.net_id and not is_ground(nm):
                    self.nets.setdefault(p.net_id, []).append((ref, i))
        self.nets = {n: m for n, m in self.nets.items() if len({r for r, _ in m}) > 1}
        self._posed = {}

    @property
    def bounds(self):
        return self.r.bounds

    @property
    def frame_size(self):
        return self.r.W, self.r.H

    def view_of(self, box, wide=False):
        return self._fit(box)

    def set_view(self, view):
        self.r.set_view(view)

    # -- the key -------------------------------------------------------------
    def _key_layout(self):
        """The key as a band along the frame's bottom: its entries in evenly
        spaced columns across the whole width, in as few rows as fit."""
        from PIL import Image, ImageDraw
        from route_render import load_font
        W, H = self.r.W, self.r.H
        self.kfont = load_font(max(11, H // 75))
        d = ImageDraw.Draw(Image.new('RGB', (1, 1)))
        self.ksw = max(9, H // 90)
        widths = [self.ksw + 6 + d.textlength(t, font=self.kfont) for _, _, t in self.key]
        cols = max(1, min(len(widths), int((W - 16) // (max(widths) + 14))))
        rows = math.ceil(len(widths) / cols)
        self.kcols = math.ceil(len(widths) / rows)      # no emptier last row than needed
        self.klh = self.ksw + 7
        self.band = rows * self.klh + 8

    def _fit(self, view):
        """`view` grown downward until what it shows clears the key's band."""
        r = self.r
        x0, y0, x1, y1 = view
        floor = r.H - self.band - 2

        def clears(e):
            r.set_view((x0, y0, x1, y1 + e))
            return r.tf.pt(x0, y1)[1] <= floor
        if clears(0.0):
            return view
        lo, hi = 0.0, y1 - y0
        for _ in range(20):
            mid = (lo + hi) / 2
            lo, hi = (lo, mid) if clears(mid) else (mid, hi)
        return (x0, y0, x1, y1 + hi)

    def _draw_key(self, d, rr):
        from render_chrome import draw_mark
        y0 = rr.H - self.band
        d.rectangle([0, y0, rr.W, rr.H], fill=rr.theme.rgb('chrome_band'))
        slot = (rr.W - 16) / self.kcols
        for i, (rgb, mark, text) in enumerate(self.key):
            x = int(8 + (i % self.kcols) * slot)
            y = int(y0 + 5 + (i // self.kcols) * self.klh)
            draw_mark(d, [x, y, x + self.ksw, y + self.ksw], rgb, mark)
            d.text((x + self.ksw + 6, y - 2), text, fill=rr.theme.rgb('chrome_text'), font=self.kfont)

    # -- drawing --------------------------------------------------------------
    def pads(self, ref, pose=None):
        from placement.legality import pads_at_pose
        p = self.state.parts.get(ref)
        fp = self.pcb.footprints[ref]
        if p is None:
            return fp.pads
        key = pose or (p.x, p.y, p.rot)
        got = self._posed.get(ref)
        if got is None or got[0] != key:
            got = self._posed[ref] = (key, pads_at_pose(fp, key))
        return got[1]

    def airwires(self, given, poses):
        lines = []
        for members in self.nets.values():
            pts = []
            for ref, i in members:
                pads = given[ref].pads if ref in given else self.pads(ref, poses.get(ref))
                pts.append((pads[i].global_x, pads[i].global_y))
            lines += _mst(pts)
        return lines

    def shot(self, label, moved=(), times=1, given=None, poses=None, key=True):
        """One frame of the state as it stands. `poses` draws those parts at
        another pose (the glide); `given` draws those parts from a footprint
        of their own (the board as given)."""
        given, poses = given or {}, poses or {}
        r, c, st = self.r, self.c, self.state
        moved = set(moved)

        def rect(ref, p):
            x0, y0, x1, y1 = p.rect()
            if ref in poses:
                dx, dy = poses[ref][0] - p.x, poses[ref][1] - p.y
                x0, y0, x1, y1 = x0 + dx, y0 + dy, x1 + dx, y1 + dy
            (ax, ay), (bx, by) = r.tf.pt(x0, y0), r.tf.pt(x1, y1)
            return [min(ax, bx), min(ay, by), max(ax, bx), max(ay, by)]

        def draw(d, rr):
            w = max(1, int(round(rr.tf.length(0.08))))
            for ref, p in sorted(st.parts.items()):
                if ref in given:
                    continue
                if ref not in self.movable:
                    d.rectangle(rect(ref, p), outline=c['fixed'], width=w)
                else:
                    d.rectangle(rect(ref, p), fill=self.fill.get(ref),
                                outline=self.line.get(ref, c['back'] if p.side == 'B' else c['front']),
                                width=w)
            for x0, y0, x1, y1 in self.airwires(given, poses):
                d.line([rr.tf.pt(x0, y0), rr.tf.pt(x1, y1)], fill=c['air'], width=1)
            for ref, fp in self.pcb.footprints.items():
                fpg = given.get(ref)
                pads = fpg.pads if fpg is not None else self.pads(ref, poses.get(ref))
                back = (fpg or fp).layer == 'B.Cu'
                for pad in pads:
                    rr._draw_pad(d, pad, fill=c['pad_back'] if back else c['pad'])
            for ref in moved & set(st.parts):
                d.rectangle(rect(ref, st.parts[ref]), outline=c['moved'], width=2 * w)
            if key:
                self._draw_key(d, rr)
        self.movie.add(r.frame(segments=[], vias=[], label=label, overlays=[draw]), times)


def _mst(pts):
    """Prim's tree over `pts`, as (x0, y0, x1, y1) lines."""
    if len(pts) < 2:
        return []
    inf = float('inf')
    best = {i: (inf, 0) for i in range(1, len(pts))}
    at, out = 0, []
    while best:
        ax, ay = pts[at]
        for i in best:
            dd = abs(pts[i][0] - ax) + abs(pts[i][1] - ay)
            if dd < best[i][0]:
                best[i] = (dd, at)
        at = min(best, key=lambda i: best[i][0])
        j = best.pop(at)[1]
        out.append((pts[j][0], pts[j][1], pts[at][0], pts[at][1]))
    return out


# -- the seeds and their starts ------------------------------------------------

def _with_defaults(given):
    """The start counts `given` (None where not given), place_simple's
    defaults filled in."""
    from place_simple import ANCHORED_SEEDS, CLUSTERED_SEEDS, SCATTERED_SEEDS
    return tuple(d if v is None else v for v, d in zip(
        given, (SCATTERED_SEEDS, CLUSTERED_SEEDS, ANCHORED_SEEDS)))


def start_counts(a):
    """(--scattered, --clustered, --anchored) as given, None where not, with
    `a`'s own counts set to place_simple's defaults where not given."""
    given = (a.scattered, a.clustered, a.anchored)
    a.scattered, a.clustered, a.anchored = _with_defaults(given)
    return given


def start_opts(scattered, clustered, anchored):
    """The --clustered-seeds and --anchored-seeds options of a run of these
    start counts (place_simple's `start_lists`)."""
    from place_simple import start_lists
    return [x for f, v in zip(('--clustered-seeds', '--anchored-seeds'),
                              start_lists(scattered, clustered, anchored)) if v for x in (f, v)]


def seed_starts(tool, given, opts, out):
    """place_simple's options for the film of one seed without --route: the
    start counts (`given`, as start_counts returns them; place_simple's
    defaults where None) become the --clustered-seeds and --anchored-seeds
    lists a run of those counts numbers its seeds by (`start_lists`), so
    `--seed S` films seed S of that run -- of a default run when no count is
    given -- from the start it had there. Lists already in `opts` are kept;
    beside a count, they must give the seed the start the count does. Several
    seeds (--seeds, or an OUT carrying {seed} and no --seed) are place_simple's
    own business, the counts passed on. Returns (opts, the line naming the seed
    and its start), or (None, why it is refused)."""
    from place_simple import _seed_list, start_lists
    counts = [(f, str(v)) for f, v in zip(('--scattered', '--clustered', '--anchored'), given)
              if v is not None]
    named = [' '.join(c) for c in counts]
    p = argparse.ArgumentParser(prog=tool, add_help=False, allow_abbrev=False)
    p.add_argument('--seed', type=int)
    for f in ('--seeds', '--clustered-seeds', '--anchored-seeds'):
        p.add_argument(f)
    k, _ = p.parse_known_args(opts)
    if k.seeds or (k.seed is None and '{seed}' in out):
        return opts + [x for c in counts for x in c], None
    sc, cl, an = _with_defaults(given)
    n, seed = sc + cl + an, (0 if k.seed is None else k.seed)
    if min(sc, cl, an) < 0 or n < 1:
        return None, f'{tool}: --scattered, --clustered and --anchored of 0 or more, at least one seed'

    def start(clustered, anchored):
        return ('anchored' if seed in _seed_list(anchored or '') else
                'clustered' if seed in _seed_list(clustered or '') else 'scattered')
    counted = start_lists(sc, cl, an)
    run = ' '.join(named) if named else f'a default run ({sc} scattered, {cl} clustered, {an} anchored)'
    if named and seed >= n:
        return None, (f'{tool}: seed {seed} is not one of the {n} seeds {run} numbers (0-{n - 1}); '
                      f'without a count, --clustered-seeds and --anchored-seeds give its start')
    if k.clustered_seeds or k.anchored_seeds:
        try:
            listed = start(k.clustered_seeds, k.anchored_seeds)
        except ValueError:
            return None, (f'{tool}: --clustered-seeds {k.clustered_seeds!r} or --anchored-seeds '
                          f'{k.anchored_seeds!r} is not a list like 0-3 or 0,1,2,3')
        lists = ' '.join(f'{f} {v}' for f, v in (('--clustered-seeds', k.clustered_seeds),
                                                ('--anchored-seeds', k.anchored_seeds)) if v)
        if named and listed != start(*counted):
            return None, (f'{tool}: seed {seed} starts {listed} by {lists} but {start(*counted)} '
                          f'by {run}; give the lists or the counts, not both')
        return opts, f"{tool}: filming seed {seed} from {_a(listed)} start ({lists})"
    add = start_opts(sc, cl, an)
    if seed >= n:
        return opts + add, (f"{tool}: filming seed {seed} from a scattered start (past the {n} "
                            f"seeds of {run})")
    return opts + add, (f"{tool}: filming seed {seed} from {_a(start(*counted))} start, as {run} "
                        f"starts it ({' '.join(add) or 'no clustered or anchored seed'})")


def _a(start):
    return ('an ' if start[0] in 'aeiou' else 'a ') + start


# -- the flow (--route) ---------------------------------------------------------

def _strip(opts, *flags):
    """`opts` without each of `flags` and its value."""
    out, skip = [], False
    for x in opts:
        if skip:
            skip = False
        elif x in flags:
            skip = True
        elif not any(x.startswith(f + '=') for f in flags):
            out.append(x)
    return out


def _score_key(sc):
    """The seed judge's own ranking (`quick_score.judge_key`)."""
    from quick_score import judge_key
    return judge_key(sc)


def _route(placed, name, set_, out, quick):
    cmd = [sys.executable, '-X', 'utf8', os.path.join(REPO, 'tests', 'stress', 'placement_ab',
                                                       'route_placed.py'),
           placed, name, set_, out] + (['--quick', '2'] if quick else [])
    r = subprocess.run(cmd, capture_output=True, text=True,
                       env=dict(os.environ, KICAD_ROUTE_TRACE='1'))
    print('  ' + ((r.stdout.strip().splitlines() or ['(no output)'])[-1]))
    sc = os.path.join(out, 'score.json')
    return json.load(open(sc)) if r.returncode == 0 and os.path.isfile(sc) else None


def _done(d):
    """The score of a route route_placed.py already finished in folder `d`."""
    p = os.path.join(d, 'score.json')
    return json.load(open(p)) if os.path.isfile(p) else None


def route_dir(work, s, full=False, chain=None):
    """The folder in `work` a route of seed `s` lands in: route_placed.py's
    `fast_s<S>` / `full_s<S>` (--route), or with a `chain` file judge_seeds'
    `s<S>_route` / `s<S>_full_route` (--route-chain)."""
    if chain:
        return os.path.join(work, f's{s}_full_route' if full else f's{s}_route')
    return os.path.join(work, f"{'full' if full else 'fast'}_s{s}")


def chain_steps(path):
    """(the commands of a --route-chain file, None), read and checked by
    judge_seeds' own `read_chain` / `chain_problems`, or (None, why not)."""
    from judge_seeds import chain_problems, read_chain
    try:
        steps = read_chain(path)
    except OSError as ex:
        return None, f'cannot read --route-chain {path}: {ex}'
    bad = chain_problems(steps)
    return (None, 'the chain cannot route a seed: ' + '; '.join(bad)) if bad else (steps, None)


def _chain_router(steps, board, opts, work):
    """route(seed, full) -> score or None, for --route-chain: judge_seeds'
    own `judge_seed` routes the seed by the chain in `work` (its run folder)
    -- route.py / route_diff.py steps at --quick 2, or with `full` at the full
    level -- and scores it with quick_score, with the intent place_simple is
    given and the board the seeds were placed from as the baseline, as
    judge_seeds grades them. Each step runs traced (KICAD_ROUTE_TRACE=1, the
    step's own env), so the film can play its router events. A seed whose
    score is already there (`s<S>_fast.score.json`, `s<S>_full.score.json`)
    is reused."""
    import threading
    from judge_seeds import judge_seed
    p = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    p.add_argument('--intent')
    intent = p.parse_known_args(opts)[0].intent
    baseline = os.path.abspath(board)          # what place_simple writes to judge.json
    traced = ['KICAD_ROUTE_TRACE=1 ' + s for s in steps]
    lock = threading.Lock()

    def route(s, full):
        sc = os.path.join(work, f"s{s}_{'full' if full else 'fast'}.score.json")
        if os.path.isfile(sc) and os.path.isdir(route_dir(work, s, full, chain=True)):
            return json.load(open(sc))
        return judge_seed(work, s, traced, intent, lock, baseline, full=full).get('score')
    return route


def run_flow(a, board, out, opts, work):
    """Place every seed, fast-route the judged ones, route the best in full,
    reusing what `work` already holds (a placed seed, a finished route), so a
    second run only films. The routes are route_placed.py's (--route
    NAME:SET) or the --route-chain file's, run by judge_seeds. Returns
    (judged, scores, best, full score) or None."""
    chain = getattr(a, 'route_chain', None)
    if chain:
        steps, why = chain_steps(chain)
        if steps is None:
            print(f'place_movie: {why}. Nothing was placed or routed.', file=sys.stderr)
            return None
        route = _chain_router(steps, board, opts, work)
    else:
        name, _, set_ = a.route.partition(':')

        def route(s, full):
            d = route_dir(work, s, full)
            return _done(d) or _route(os.path.join(work, f's{s}.kicad_pcb'), name, set_, d, not full)
    if a.fresh:
        shutil.rmtree(work, ignore_errors=True)
    os.makedirs(work, exist_ok=True)
    seeds = list(range(a.scattered + a.clustered + getattr(a, 'anchored', 0)))
    todo = [s for s in seeds if not (os.path.isfile(os.path.join(work, f's{s}.json'))
                                     and os.path.isfile(os.path.join(work, f's{s}.kicad_pcb')))]
    if todo:
        print(f"place_movie: placing seeds {','.join(map(str, todo))} ({a.jobs} at a time)")
        rc = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(HERE, 'place_simple.py'),
                             board, os.path.join(work, 's{seed}.kicad_pcb'), *opts,
                             '--seeds', ','.join(map(str, todo)), '--jobs', str(a.jobs),
                             '--json', os.path.join(work, 's{seed}.json')]).returncode
        if rc == 2:
            return None
    reps = {s: json.load(open(os.path.join(work, f's{s}.json'))) for s in seeds
            if os.path.isfile(os.path.join(work, f's{s}.json'))}
    from place_simple import judge_order
    judged = judge_order(reps)[:a.judge]
    if not judged:
        print("place_movie: no seed placed every part legally; nothing to route", file=sys.stderr)
        return None
    print(f"place_movie: fast-routing the {len(judged)} legal seeds the judge names: {judged}")
    with ThreadPoolExecutor(max_workers=max(1, a.route_jobs or a.jobs)) as pool:
        got = list(pool.map(lambda s: route(s, False), judged))
    scores = {s: sc for s, sc in zip(judged, got) if sc}
    if not scores:
        print("place_movie: no fast route finished", file=sys.stderr)
        return None
    best = min(scores, key=lambda s: _score_key(scores[s]))
    print(f"place_movie: the best fast route is seed {best}; routing it in full")
    full = route(best, True)
    if full is None:
        return None
    return judged, scores, best, full


def _chain(d):
    """(the board the chain started from, its boards in order) of a route
    in folder `d` (`route_dir`): route_placed.py's (in/, out/,
    manifest/redo_commands.sh) or judge_seeds' (in.kicad_pcb, each step's
    board and the route's own redo_commands.sh, side by side)."""
    from route_movie import run_boards
    if os.path.isdir(os.path.join(d, 'in')):
        first = [os.path.join(d, 'in', f) for f in os.listdir(os.path.join(d, 'in'))
                 if f.endswith('.kicad_pcb')][0]
        run, manifest = os.path.join(d, 'out'), os.path.join(d, 'manifest', 'redo_commands.sh')
    else:
        first, run, manifest = os.path.join(d, 'in.kicad_pcb'), d, os.path.join(d, 'redo_commands.sh')
    boards = [b for b in run_boards(run, manifest)
              if os.path.basename(b) != os.path.basename(first)]
    return first, boards


def routed_where(work, best, chain=None):
    """Where the best seed's full route's boards are, for the last line."""
    d = route_dir(work, best, True, chain)
    return d if chain else os.path.join(d, 'out')


def _score_text(sc):
    q = sc.get('quality') or {}
    mm = q.get('copper_mm')
    return f"blocking {sc.get('blocking')} | {q.get('vias')} vias" + (f" | {mm:.0f} mm" if mm else '')


def _grid_dims(n, W, H):
    """(cols, rows) giving the n cells the biggest board-shaped picture."""
    best = None
    for cols in range(1, n + 1):
        rows = math.ceil(n / cols)
        k = min(W / cols / W, (H / rows - LABEL_PX) / H)
        if best is None or k > best[0]:
            best = (k, cols, rows)
    return best[1], best[2]


def film_routes(a, movie, work, judged, scores, best, full, filmed):
    """The routing half of the movie, played with route_movie's own player:
    the filmed seed's fast route, the judged seeds side by side, the best
    one boxed and zoomed into, its fast route cleared and its full route."""
    from PIL import Image, ImageDraw
    from kicad_parser import parse_kicad_pcb
    from route_movie import ZONE_TINT, _board_copper, play_chain, shoot
    from route_render import BoardRenderer, load_font
    kw = dict(size=a.size, theme=a.theme)
    rkw = dict(kw, zone_tint=ZONE_TINT)
    hold = max(1, int(round(STEP_HOLD * a.fps)))
    chain = getattr(a, 'route_chain', None)
    # 8. the filmed seed's fast route
    first, boards = _chain(route_dir(work, filmed, chain=chain))
    r, live_s, live_v = play_chain(movie, boards, first, prefix='fast route | ',
                                   first_label=f'placed, seed {filmed}', **kw)
    shoot(movie, r, live_s, live_v, f"fast route, seed {filmed}: {_score_text(scores[filmed])}",
          times=hold)
    # 9. the judged seeds side by side
    W, H = movie.size
    cols, rows = _grid_dims(len(judged), W, H)
    cw, ch = W // cols, H // rows
    th = r.theme
    full_img = {}
    G = Image.new('RGB', (W, H), th.rgb('ground'))
    d = ImageDraw.Draw(G)
    font = load_font(LABEL_PX - 4)
    cells = {}
    for i, s in enumerate(sorted(scores)):
        _f, bs = _chain(route_dir(work, s, chain=chain))
        rr = BoardRenderer(parse_kicad_pcb(bs[-1]), supersample=1, **rkw)
        img = rr.render()
        if img.size != (W, H):
            img = img.resize((W, H))
        full_img[s] = img
        k = min(cw / W, (ch - LABEL_PX) / H)
        w, h = int(W * k), int(H * k)
        x = (i % cols) * cw + (cw - w) // 2
        y = (i // cols) * ch + (ch - LABEL_PX - h) // 2
        cells[s] = (x, y, x + w, y + h)
        G.paste(img.resize((w, h), Image.LANCZOS), (x, y))
        sc = scores[s]
        d.text((x, y + h + 1), f"s{s}  blk {sc.get('blocking')}  "
               f"{(sc.get('quality') or {}).get('vias')} v", fill=th.rgb('chrome_text'), font=font)

    def zoom(img, cell, t, label):
        """The grid seen through a camera at t (0: `cell` fills the frame, 1:
        the whole grid), with `img` drawn sharp over its cell."""
        cam = _lerp(cell, (0, 0, W, H), t)
        sx, sy = W / (cam[2] - cam[0]), H / (cam[3] - cam[1])
        f = G.crop(tuple(int(round(v)) for v in cam)).resize((W, H), Image.BILINEAR)
        box = ((cell[0] - cam[0]) * sx, (cell[1] - cam[1]) * sy,
               (cell[2] - cam[0]) * sx, (cell[3] - cam[1]) * sy)
        bw, bh = int(round(box[2] - box[0])), int(round(box[3] - box[1]))
        if bw > 0 and bh > 0:
            f.paste(img.resize((bw, bh), Image.BILINEAR), (int(round(box[0])), int(round(box[1]))))
        r._label(f, label)
        movie.add(f)
    title = f"the {len(scores)} cheapest seeds by the objective, fast-routed"
    for i in range(1, ZOOM + 1):
        zoom(full_img[filmed], cells[filmed], _ease(i / ZOOM), title)
    grid = G.copy()
    r._label(grid, title)
    movie.add(grid, hold)
    x0, y0, x1, y1 = cells[best]
    pick = f"the best: seed {best} ({_score_text(scores[best])})"
    boxed = G.copy()
    ImageDraw.Draw(boxed).rectangle([x0 - 3, y0 - 3, x1 + 3, y1 + LABEL_PX + 2],
                                    outline=th.rgb('status_best'), width=4)
    G = boxed
    for blink in range(3):
        f = (boxed if blink % 2 == 0 else grid).copy()
        r._label(f, pick)
        movie.add(f, hold // 2)
    f = boxed.copy()
    r._label(f, pick)
    movie.add(f, hold)
    for i in range(ZOOM - 1, -1, -1):
        zoom(full_img[best], cells[best], _ease(i / ZOOM), pick)
    # 10. its fast route cleared, net by net, then its full route
    _f, bs = _chain(route_dir(work, best, chain=chain))
    pcb = parse_kicad_pcb(bs[-1])
    r = BoardRenderer(pcb, supersample=1, **rkw)
    live_s, live_v = _board_copper(pcb)
    nets = sorted({o.net_id for o in list(live_s.values()) + list(live_v.values())})
    step = max(1, math.ceil(len(nets) / 6))
    for j in range(0, len(nets), step):
        part = set(nets[j:j + step])
        gone = [o for o in live_s.values() if o.net_id in part]
        shoot(movie, r, live_s, live_v, f"clearing the fast route of seed {best}", rips=gone)
        live_s = {k: o for k, o in live_s.items() if o.net_id not in part}
        live_v = {k: o for k, o in live_v.items() if o.net_id not in part}
    first, boards = _chain(route_dir(work, best, True, chain))
    r, live_s, live_v = play_chain(movie, boards, first, prefix='full route | ',
                                   first_label=f'seed {best}, cleared', **kw)
    shoot(movie, r, live_s, live_v, f"full route, seed {best}: {_score_text(full)}", times=hold)


def main(argv=None):
    # each line reaches a pipe or a log when it is printed, not at exit (#1829)
    import cli_banner
    cli_banner.line_buffer_streams()
    from route_movie import FPS, SIZE, MovieWriter
    from place_simple import ANCHORED_SEEDS, CLUSTERED_SEEDS, JUDGED_SEEDS, SCATTERED_SEEDS
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n')[0], allow_abbrev=False,   # --seed is place_simple's
        epilog="BOARD and OUT come first; every other argument is place_simple.py's "
               "(--groups, --intent, --seed and the weights).")
    ap.add_argument('--movie', help="the movie to write (default: OUT's name, .mp4); "
                                    ".gif writes a GIF")
    ap.add_argument('--frames', type=int, default=FRAMES,
                    help=f'frames of anneal per run (default {FRAMES})')
    ap.add_argument('--speedup', type=float, default=SPEEDUP,
                    help=f'how much faster the anneal plays at its end than its start '
                         f'(default {SPEEDUP:g})')
    ap.add_argument('--size', type=int, default=SIZE, help=f'longest frame side, px (default {SIZE})')
    ap.add_argument('--fps', type=float, default=FPS, help=f'frames per second (default {FPS:g})')
    ap.add_argument('--theme', default='dark', help='render theme: dark (default) or light')
    how = ap.add_mutually_exclusive_group()
    how.add_argument('--route', metavar='BOARD:SET',
                     help='run and film the placement flow, routing with this stress-corpus '
                          "board's recorded chain (e.g. ch32v003_usb:6)")
    how.add_argument('--route-chain', metavar='FILE',
                     help="run and film the placement flow, routing with this chain file "
                          "(judge_seeds.py --chain's format: the placement run's "
                          "judge_chain.txt), run as judge_seeds runs it, from the current "
                          "folder (the repo root)")
    ap.add_argument('--scattered', type=int,
                    help=f'the seeds started scattered (default {SCATTERED_SEEDS}): in the flow, '
                         f'the seeds placed; without, the run whose --seed is filmed, from the '
                         f'start that run gives it')
    ap.add_argument('--clustered', type=int,
                    help=f'the seeds started clustered, numbered after the scattered ones '
                         f'(default {CLUSTERED_SEEDS})')
    ap.add_argument('--anchored', type=int,
                    help=f'the seeds started anchored, numbered after the clustered ones '
                         f'(default {ANCHORED_SEEDS}: opt-in)')
    ap.add_argument('--judge', type=int, default=JUDGED_SEEDS,
                    help=f'in the flow, how many of the cheapest legal seeds are fast-routed '
                         f'(default {JUDGED_SEEDS})')
    ap.add_argument('--jobs', type=int, default=4,
                    help='in the flow, placements and fast routes run at once (default 4)')
    ap.add_argument('--route-jobs', type=int, default=0,
                    help='in the flow, fast routes run at once, when it should differ from '
                         '--jobs (a big board\'s route can take 5 GB: 1 runs them one at a time)')
    ap.add_argument('--fresh', action='store_true',
                    help='in the flow, empty --work first: place and route every seed again '
                         '(otherwise a placed seed or finished route there is reused, so a '
                         'second run only films)')
    ap.add_argument('--work', help="in the flow, the work directory (default: OUT's name + _work)")
    a, rest = ap.parse_known_args(argv)
    if len(rest) < 2 or rest[0].startswith('-') or rest[1].startswith('-'):
        ap.error('BOARD and OUT come first')
    board, out, opts = rest[0], rest[1], rest[2:]
    movie_path = a.movie or os.path.splitext(out)[0] + '.mp4'
    given = start_counts(a)
    if not (a.route or a.route_chain):
        opts, line = seed_starts('place_movie', given, opts, out)
        if opts is None:
            print(line, file=sys.stderr)
            return 2
        if line:
            print(line)
        movie = MovieWriter(movie_path, a.fps)
        film = Film(a, movie)
        install(film)
        import place_simple
        rc = place_simple.main([board, out, *opts])
        if hasattr(film, 'state'):
            film.placed(rc, 1)
        path = movie.close(hold=END_HOLD)
        print(f"place_movie: {path} ({movie.n} frames, {movie.n / a.fps:.0f} s)")
        return rc
    if a.route and ':' not in a.route:
        ap.error('--route BOARD:SET, e.g. ch32v003_usb:6')
    work = os.path.abspath(a.work or os.path.splitext(out)[0] + '_work')
    opts = _strip(opts, '--seed', '--json', '--seeds', '--jobs', '--clustered-seeds',
                  '--anchored-seeds')
    if min(a.scattered, a.clustered, a.anchored) < 0 or a.scattered + a.clustered + a.anchored < 1 \
            or a.judge < 1:
        ap.error('--scattered, --clustered and --anchored of 0 or more, at least one seed, '
                 '--judge 1 or more')
    # the round's clustered and anchored seeds, for the placing and the filming
    opts += start_opts(a.scattered, a.clustered, a.anchored)
    got = run_flow(a, board, out, opts, work)
    if got is None:
        return 2
    judged, scores, best, full = got
    filmed = best
    # film: the chosen seed's placement again (the same placement), then its routes
    movie = MovieWriter(movie_path, a.fps)
    film = Film(a, movie)
    install(film)
    import place_simple
    rc = place_simple.main([board, os.path.join(work, f'film_s{filmed}.kicad_pcb'), *opts,
                            '--seed', str(filmed)])
    film.placed(rc, film.hold, handoff=True)
    film_routes(a, movie, work, judged, scores, best, full, filmed)
    path = movie.close(hold=END_HOLD)
    subprocess.run([sys.executable, os.path.join(REPO, 'py_router', 'copy_board.py'),
                    os.path.join(work, f's{best}.kicad_pcb'), out], check=False, capture_output=True)
    print(f"place_movie: {path} ({movie.n} frames, {movie.n / a.fps:.0f} s); "
          f"{out} = seed {best}; full route in {routed_where(work, best, a.route_chain)}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
