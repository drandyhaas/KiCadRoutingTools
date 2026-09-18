#!/usr/bin/env python3
"""evolve_movie.py -- film an `evolve.py` run: parallel worlds, side by side,
with the copper that CHANGED lit up (#622, 2026-09-18).

`evolve.py` keeps a POPULATION of routed worlds and, every generation, runs
three operators on them side by side (descend / jump / cross), then selects
elitistically.  Nothing about that is visible in the ledger's numbers: which
lanes a descent actually moved, how far a jump threw the bus, whether the
crossover's child is its parents' copper recombined or something new.  This
renders the run as a movie from the ledger alone.

WHAT A FRAME IS
    A frame is one canvas holding the whole generation at once:

      row 0   the population ENTERING the generation, in rank order
      row 1   each one's DESCENT, directly under its parent -- so a descent's
              lineage is read off the geometry and needs no arrow
      row 2   the JUMP and CROSS worlds, with arrows drawn to their parent(s)

    plus a title bar (generation, phase, record so far), a lineage ribbon
    (every world as generation x via-count, parent->child edges -- the search's
    own trajectory, growing as the film advances) and a legend.

    Time runs in scenes: SEEDS, then per generation ROLL-CALL -> WORK ->
    SELECT.  In WORK every cell advances through its own steps SIMULTANEOUSLY,
    which is the point: the worlds are parallel, so the film is parallel.

WHAT A STEP IS
    A descent's `d.out` is a transcript: `=== round N: verdict off BOARD`,
    then one `    NET: FILE is the board now (open [...], drc N, vias M)` per
    STANDING probe, then `round N: KEPT`.  That gives the ordered chain of
    boards a descent really passed through -- round 0 (which IS the parent's
    board), each standing probe, each round's shipped board, the final board.
    Steps whose copper diff is empty are dropped, so every rendered step shows
    a real change.  A jump/cross world has no such chain (it is one re-plan),
    so it gets two steps -- its parent's board, then the landed board -- which
    is its whole lineage change in one cut.

HOW "CHANGED" IS COMPUTED
    Copper is reduced to a MULTISET of canonical keys and differenced:
    segments as (layer, net short name, width, endpoints) and vias as
    (x, y, size, drill, net), every number in integer NANOMETRES and the two
    endpoints stored in sorted order, so a segment re-emitted end-for-end is
    not a spurious change.  added = C(new) - C(old), removed = C(old) - C(new)
    as Counters, not sets, so duplicated copper differences exactly.  Added
    copper is drawn hot on top; removed copper is ghosted for the same beat
    and fades with it.

    This is checkable, and `--verify` checks it: for each step it prints the
    two boards' segment/via totals, the added/removed counts, the nets the
    diff touched, and the net the RUN's own log said stood.  The log's net
    must appear in the diff's nets.

NOTHING HERE NAMES A BOARD, A NET OR A REFERENCE.  The nets come from the
world's plan sidecar, the camera from their copper, the lineage from the
ledger's `origin` strings, the grades from the ledger.

usage: evolve_movie.py TAG K [--out movie.mp4] [--view X0,Y0,X1,Y1] [--fps N]
                             [--cell WxH] [--size WxH] [--gens N] [--gif]
                             [--frames-dir DIR] [--verify] [--supersample N]
                             [--hold S] [--roll S] [--select S] [--self-test]
  reads  tmp/TAG/evolve_kK.json  (tolerates a ledger still being written)
  writes tmp/movie/TAG_kK.mp4 + the frames beside it

Three things it tolerates, because all three were met in real ledgers:
  - a ledger still being WRITTEN: it is filmed up to its last COMPLETE
    generation, and generation directories on disk the ledger has not recorded
    are named and skipped (their worlds have no grade to caption);
  - a descent evolve.py DROPPED: the slot still gets a cell, captioned with the
    parent's grade and "no gain: parent kept", because an operator that
    produced nothing is part of the generation;
  - a run directory RENAMED after the fact: the ledger bakes absolute stems, so
    a dead stem is re-rooted onto the ledger's own directory by its `g<N>/...`
    tail, and the count is reported.

The default CAMERA is the bounding box of the run nets' copper -- every net the
worlds' plan sidecars name, unioned -- padded.  It deliberately does not crop:
on this bench one net leaves the corridor, and a default that hid real copper
would be worse than one that shows a quiet band.  `--view` is the crop.

`--self-test` (which also runs on every invocation, in milliseconds) asserts
the origin grammar and the copper diff.  Neither has a natural failure signal:
a mis-parsed origin draws an arrow to the wrong parent and a mis-keyed diff
highlights the wrong copper, and both still produce a plausible-looking movie.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import subprocess
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)

from PIL import Image, ImageDraw                                # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from route_render import BoardRenderer, load_font               # noqa: E402

# --------------------------------------------------------------------------
# palette -- one colour language for the whole film
# --------------------------------------------------------------------------
BG = (10, 11, 13)
PANEL = (20, 23, 28)
PANEL_EDGE = (44, 50, 58)
TEXT = (228, 232, 238)
DIM = (138, 146, 158)
FAINT = (78, 84, 94)

# operator colours: every caption, border, arrow and lineage edge of a kind
# uses its colour, so the kind is readable without reading the word.
KIND_COLOUR = {
    'seed':    (150, 162, 176),
    'descend': (86, 206, 130),
    'jump':    (242, 162, 58),
    'cross':   (190, 130, 236),
}
BEST = (255, 214, 88)
KEPT = (86, 206, 130)
DROPPED = (206, 78, 92)

ADDED = (255, 248, 150)        # copper present now and not before
REMOVED = (255, 70, 120)       # copper present before and not now (ghost)


def kind_colour(kind):
    return KIND_COLOUR.get(kind, DIM)


# --------------------------------------------------------------------------
# the ledger, and the lineage encoded in `origin`
# --------------------------------------------------------------------------
_ORIGIN = re.compile(r'^(\w+)\s*<(.*)>$', re.S)


def parse_origin(origin):
    """`origin` -> (kind, [parent names], detail).

    evolve.py writes four shapes and this reads all four:
        seed <spec>
        descend<NAME>                 descend<NAME: <that world's own origin>>
        jump<NAME; bans [...]; seed N>
        cross<A x B; n from B>
    The detail is the human-readable remainder (bans, split, seed spec).
    """
    origin = (origin or '').strip()
    if origin.startswith('seed'):
        return 'seed', [], origin[4:].strip()
    m = _ORIGIN.match(origin)
    if not m:
        return 'seed', [], origin
    kind, body = m.group(1), m.group(2)
    if kind == 'descend':
        # `descend<NAME>` or `descend<NAME: <inner origin>>`; the parent is the
        # leading name either way.
        name = body.split(':', 1)[0].strip()
        rest = body.split(':', 1)[1].strip() if ':' in body else ''
        return 'descend', [name], rest
    if kind == 'jump':
        head, _, rest = body.partition(';')
        return 'jump', [head.strip()], rest.strip()
    if kind == 'cross':
        head, _, rest = body.partition(';')
        parts = [p.strip() for p in head.split(' x ') if p.strip()]
        return 'cross', parts, rest.strip()
    return kind, [], body


def grade_tuple(g):
    """(open count, drc, vias) from a ledger grade, tolerating a missing one."""
    if not g:
        return (99, 99, 0)
    return (len(g[0]), g[1], g[2])


def fmt_grade(g):
    if not g:
        return 'no grade'
    o, d, v = grade_tuple(g)
    s = f'{v} vias'
    if o:
        s += f'  {o} open'
    if d:
        s += f'  {d} drc'
    return s


def better(a, b):
    """evolve.py's own ordering: opens, then DRC-free, then vias."""
    if a is None:
        return False
    if b is None:
        return True
    ka = (len(a[0]), a[1] != 0, a[2])
    kb = (len(b[0]), b[1] != 0, b[2])
    return ka < kb


class Ledger:
    """`tmp/TAG/evolve_kK.json`, plus everything derivable from it."""

    def __init__(self, path, max_gens=None):
        with open(path, encoding='utf-8') as f:
            self.raw = json.load(f)
        self.path = path
        self.root = os.path.dirname(os.path.abspath(path))
        self.tag = self.raw.get('tag', os.path.basename(self.root))
        self.K = self.raw.get('K')
        self.pop0 = [dict(w) for w in self.raw.get('pop0', [])]
        gens = [dict(g) for g in self.raw.get('gens', [])]
        if max_gens is not None:
            gens = gens[:max_gens]
        self.gens = gens
        # A ledger bakes ABSOLUTE stems, so a run directory that was renamed or
        # moved after the fact records paths that no longer exist (measured:
        # a K51 ledger whose every stem pointed at a directory since renamed).
        # The ledger's OWN location is the truth, so a dead stem is re-rooted
        # onto it by its `g<N>/...` tail before anything downstream sees it.
        self._rerooted = 0
        for w in self.pop0:
            self._reroot(w)
        for g in self.gens:
            for w in list(g.get('pop', [])) + list(g.get('new', [])) + [g.get('best')]:
                if w:
                    self._reroot(w)
        self.by_name = {}
        for w in self.pop0:
            self._ingest(w, 0)
        for g in self.gens:
            for w in g.get('new', []):
                self._ingest(dict(w), g['gen'])
            for w in g.get('pop', []):
                self._ingest(dict(w), None)

    def _reroot(self, w):
        """Point a world's `stem` at the ledger's own directory when the path
        it recorded is gone.  Keeps the `g<N>/...` tail, which is the part that
        identifies the world; anything above it was only where the run sat."""
        stem = w.get('stem') or ''
        if not stem or os.path.exists(stem + '.kicad_pcb'):
            return
        parts = stem.replace('\\', '/').split('/')
        for i, p in enumerate(parts):
            if re.match(r'^g\d+$', p):
                cand = os.path.join(self.root, *parts[i:])
                if os.path.exists(cand + '.kicad_pcb'):
                    w['stem'] = cand
                    self._rerooted += 1
                return

    def _ingest(self, w, born):
        old = self.by_name.get(w['name'])
        if old is None:
            w = dict(w)
            w['born'] = born if born is not None else 0
            w['kind'], w['parents'], w['detail'] = parse_origin(w.get('origin'))
            self.by_name[w['name']] = w
        elif born is not None and old.get('born') is None:
            old['born'] = born

    def entering_pop(self, gen):
        """The population generation `gen` descends: pop0 for gen 1, else the
        population the previous generation recorded AFTER selection -- which is
        exactly the list evolve.py slices as `pop[:POP]` to build its jobs, so
        job `w<i>` is entering_pop(gen)[i]."""
        if gen <= 1:
            return list(self.pop0)
        for g in self.gens:
            if g['gen'] == gen - 1:
                return list(g.get('pop', []))
        return []

    def gen(self, n):
        for g in self.gens:
            if g['gen'] == n:
                return g
        return None

    def unrecorded_gen_dirs(self):
        """Generation directories on disk the ledger has not recorded (a run
        still in flight).  Named, not filmed: their worlds have no grade."""
        seen = {g['gen'] for g in self.gens}
        out = []
        for d in sorted(glob.glob(os.path.join(self.root, 'g*'))):
            m = re.match(r'^g(\d+)$', os.path.basename(d))
            if m and int(m.group(1)) > 0 and int(m.group(1)) not in seen:
                out.append(os.path.basename(d))
        return out


# --------------------------------------------------------------------------
# the boards a job passed through
# --------------------------------------------------------------------------
# `    <NET>: <file> is the board now (open [...], drc N, vias M)`
_STOOD = re.compile(r'^\s+(\S+):\s+(\S+\.kicad_pcb) is the board now '
                    r'\(open \[(.*?)\], drc (\d+), vias (\d+)\)')
_ROUND = re.compile(r'^=== round (\d+):')
_KEPT = re.compile(r'^\s+round (\d+): KEPT')


class Step:
    """One board state in a cell's timeline."""

    def __init__(self, board, label, note='', grade=None):
        self.board = board
        self.label = label          # short verb for the caption
        self.note = note            # what moved, filled in after the diff
        self.grade = grade          # (opens, drc, vias) when the log gave one
        self.added_s = []
        self.added_v = []
        self.removed_s = []
        self.removed_v = []
        self.nets = []
        self.log_net = None         # the net the run's own log named


def descent_steps(job_dir, prefix, log_path=None):
    """The board chain a `replan.py` descent passed through, read off its
    `d.out` transcript; falls back to the round boards, then the final board.

    Returns [Step]; missing files are skipped, so a run whose intermediates
    were cleaned still yields at least its final board.
    """
    steps = []
    log = log_path or os.path.join(job_dir, 'd.out')
    seen = set()

    def add(path, label, note='', grade=None, log_net=None):
        if not path or path in seen or not os.path.exists(path):
            return
        seen.add(path)
        st = Step(path, label, note, grade)
        st.log_net = log_net
        steps.append(st)

    if os.path.exists(log):
        with open(log, encoding='utf-8', errors='replace') as f:
            rnd = 0
            for line in f:
                m = _ROUND.match(line)
                if m:
                    rnd = int(m.group(1))
                    continue
                m = _STOOD.match(line)
                if m:
                    net, fn, opens, drc, vias = m.groups()
                    opens = [o.strip().strip("'\"") for o in opens.split(',') if o.strip()]
                    add(os.path.join(job_dir, fn), f'round {rnd}', '',
                        [opens, int(drc), int(vias)], log_net=net)
                    continue
                m = _KEPT.match(line)
                if m:
                    add(f'{prefix}_r{m.group(1)}.kicad_pcb', f'round {m.group(1)} ships')
    # round 0 IS the parent's board: put it first whatever the log said
    start = f'{prefix}_r0.kicad_pcb'
    if os.path.exists(start) and start not in seen:
        seen.add(start)
        steps.insert(0, Step(start, 'start'))
    if not steps:
        for p in sorted(glob.glob(f'{prefix}_r*.kicad_pcb'),
                        key=lambda p: _round_no(p)):
            if re.search(r'_r\d+\.kicad_pcb$', p):
                add(p, 'round ' + str(_round_no(p)))
    add(f'{prefix}.kicad_pcb', 'ships')
    return steps


def _round_no(path):
    m = re.search(r'_r(\d+)\.kicad_pcb$', path)
    return int(m.group(1)) if m else 0


# --------------------------------------------------------------------------
# copper diff
# --------------------------------------------------------------------------
def _nm(v):
    return int(round(float(v) * 1e6))


def short(name):
    return (name or '').split('/')[-1]


def copper_keys(pcb):
    """Canonical multisets of a board's copper, plus the objects behind each key.

    A key is integer nanometres with the two endpoints in sorted order, so the
    same track re-emitted end-for-end is the same key; the net enters by SHORT
    NAME rather than id, so a renumbered file still differences correctly.
    Counters (not sets) so duplicated copper differences exactly.
    """
    n2n = {i: short(n.name) for i, n in pcb.nets.items()}
    cs, cv = Counter(), Counter()
    obj_s, obj_v = {}, {}
    for s in pcb.segments:
        a = (_nm(s.start_x), _nm(s.start_y))
        b = (_nm(s.end_x), _nm(s.end_y))
        k = (s.layer, n2n.get(s.net_id, ''), _nm(s.width), min(a, b), max(a, b))
        cs[k] += 1
        obj_s.setdefault(k, s)
    for v in pcb.vias:
        k = (_nm(v.x), _nm(v.y), _nm(v.size), _nm(getattr(v, 'drill', 0) or 0),
             n2n.get(v.net_id, ''))
        cv[k] += 1
        obj_v.setdefault(k, v)
    return cs, cv, obj_s, obj_v


def diff_copper(prev, cur):
    """(added segs, added vias, removed segs, removed vias, nets touched)."""
    if prev is None:
        return [], [], [], [], []
    ps, pv, pos, pov = prev
    cs, cv, cos, cov = cur
    add_s, rem_s = cs - ps, ps - cs
    add_v, rem_v = cv - pv, pv - cv
    nets = sorted({k[1] for k in add_s} | {k[1] for k in rem_s}
                  | {k[4] for k in add_v} | {k[4] for k in rem_v} - {''})
    return ([cos[k] for k in add_s], [cov[k] for k in add_v],
            [pos[k] for k in rem_s], [pov[k] for k in rem_v],
            [n for n in nets if n])


# --------------------------------------------------------------------------
# the camera
# --------------------------------------------------------------------------
def run_net_names(stem):
    """The nets this run routes, as the world's plan sidecar names them."""
    for cand in (stem + '_fo.plan.json', stem + '.plan.json'):
        if os.path.exists(cand):
            try:
                with open(cand, encoding='utf-8') as f:
                    return set(json.load(f).get('ends') or {})
            except Exception:                                   # noqa: BLE001
                pass
    return set()


def derive_view(pcb, names, pad=0.04):
    """Aim the camera at the copper of the run's own nets, padded.

    Derived, never named: the net list comes from the plan sidecar, the box
    from those nets' copper on the board.  Falls back to all netted copper,
    then to the board outline, so a board with no sidecar still films.
    """
    ids = {i for i, n in pcb.nets.items() if short(n.name) in names} if names else None
    for sel in (ids, None):
        xs, ys = [], []
        for s in pcb.segments:
            if sel is not None and s.net_id not in sel:
                continue
            if sel is None and not s.net_id:
                continue
            xs += [s.start_x, s.end_x]
            ys += [s.start_y, s.end_y]
        for v in pcb.vias:
            if sel is not None and v.net_id not in sel:
                continue
            if sel is None and not v.net_id:
                continue
            xs += [v.x - v.size / 2, v.x + v.size / 2]
            ys += [v.y - v.size / 2, v.y + v.size / 2]
        if xs:
            x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
            mx, my = (x1 - x0) * pad, (y1 - y0) * pad
            return (x0 - mx, y0 - my, x1 + mx, y1 + my)
    return pcb.board_info.board_bounds or (0.0, 0.0, 1.0, 1.0)


# --------------------------------------------------------------------------
# cell rendering
# --------------------------------------------------------------------------
def _paint(d, r, segs, vias, colour, scale=1.0, hollow_vias=False):
    """Draw copper in one flat colour through the renderer's transform.

    Geometry mirrors `BoardRenderer._draw_segments` / `_draw_vias` (round caps
    once a track is >= 3 px so a polyline's corners stay continuous), because a
    highlight that does not sit exactly on the copper it marks is a lie about
    which track moved.
    """
    for s in segs:
        x0, y0 = r.tf.pt(s.start_x, s.start_y)
        x1, y1 = r.tf.pt(s.end_x, s.end_y)
        w = max(1, int(round(r.tf.length(s.width) * scale)))
        d.line([x0, y0, x1, y1], fill=colour, width=w, joint='curve')
        if w >= 3:
            rr = w / 2
            d.ellipse([x0 - rr, y0 - rr, x0 + rr, y0 + rr], fill=colour)
            d.ellipse([x1 - rr, y1 - rr, x1 + rr, y1 + rr], fill=colour)
    for v in vias:
        cx, cy = r.tf.pt(v.x, v.y)
        rad = max(1.5, r.tf.length(v.size) / 2 * (1.0 if not hollow_vias else 1.25))
        if hollow_vias:
            d.ellipse([cx - rad, cy - rad, cx + rad, cy + rad],
                      outline=colour, width=max(1, int(round(rad / 2))))
        else:
            d.ellipse([cx - rad, cy - rad, cx + rad, cy + rad], fill=colour)


def _change_overlay(step):
    """An `overlays` callable painting the change: REMOVED copper first, then
    ADDED on top.

    Both go through `overlays` -- the renderer's documented seam for caller
    vocabulary, which its own docstring names "ghosts" -- rather than through
    `highlight_segments`, because the renderer draws highlights BEFORE overlays
    and the z-order has to be the other way round: what is there NOW must win
    over what is gone, or a re-route's new lane hides under the ghost of the
    old one.

    The ghost is drawn WIDER than the copper it marks, not narrower: a re-laid
    lane usually lands close to the path it replaced, so an exact-width ghost
    under an exact-width highlight is invisible exactly when it matters most.
    Widened, it survives as a fringe around the new lane -- "copper used to be
    here".  That halo is a MAGNIFIER for a change too small to see, so it is
    spent only where it is needed: a descent moves one or two lanes and wants
    it, a jump or crossover replaces the whole bus, where a fat ghost floods
    the cell and buries the copper that replaced it.  Scaled by the size of
    the change, not fixed.
    """
    def fn(d, r):
        halo = 2.2 if len(step.removed_s) <= 80 else 1.0
        _paint(d, r, step.removed_s, step.removed_v, REMOVED,
               scale=halo, hollow_vias=True)
        _paint(d, r, step.added_s, step.added_v, ADDED)
    return fn


class Camera:
    """One shared BoardRenderer for every world.

    Every world in a run is the SAME board with different copper -- same
    outline, same footprints, same pads -- so the expensive static substrate is
    built once and each cell is just a copper composite on top of it.  W/H are
    set to the cell box before `set_view`, which is the renderer's own seam for
    aiming the camera, so cells come out exactly the size the layout asked for
    instead of letterboxed inside a board-shaped canvas.
    """

    def __init__(self, ref_pcb, view, cell_w, cell_h, supersample=2, layer_alpha=205):
        self.r = BoardRenderer(ref_pcb, size=max(cell_w, cell_h),
                               supersample=supersample, margin_frac=0.012,
                               layer_alpha=layer_alpha)
        self.r.W, self.r.H = int(cell_w), int(cell_h)
        self.r.set_view(view)
        self.size = (int(cell_w), int(cell_h))
        self._cache = {}
        self.renders = 0

    def clear(self):
        self._cache.clear()

    def image(self, step, hot):
        """The cell image for a step, with (hot) or without (cool) its change
        highlight.  Cached: a step is rendered at most twice per generation."""
        key = (step.board, bool(hot))
        img = self._cache.get(key)
        if img is not None:
            return img
        pcb = _pcb(step.board)
        if hot and (step.added_s or step.added_v or step.removed_s or step.removed_v):
            img = self.r.frame(segments=pcb.segments, vias=pcb.vias,
                               overlays=[_change_overlay(step)])
        else:
            img = self.r.frame(segments=pcb.segments, vias=pcb.vias)
        self.renders += 1
        self._cache[key] = img
        return img


_PCB_CACHE = {}


def _pcb(path, limit=48):
    pcb = _PCB_CACHE.get(path)
    if pcb is None:
        pcb = parse_kicad_pcb(path)
        if len(_PCB_CACHE) >= limit:
            _PCB_CACHE.pop(next(iter(_PCB_CACHE)))
        _PCB_CACHE[path] = pcb
    return pcb


# --------------------------------------------------------------------------
# a cell: one world's slot on the stage
# --------------------------------------------------------------------------
class Cell:
    def __init__(self, row, col, kind, title, parents=(), world=None,
                 steps=(), detail='', parent_world=None):
        self.row, self.col = row, col
        self.kind = kind
        self.title = title
        self.parents = list(parents)
        self.world = world              # ledger world, or None (produced nothing)
        self.steps = list(steps)
        self.detail = detail
        self.parent_world = parent_world
        self.status = ''                # set at selection
        self.rect = None                # filled by the stage layout
        self.gworld = None              # global identity (see Registry)
        self.gparents = []

    @property
    def grade(self):
        """The grade to caption with.  A descent evolve.py DROPPED has no
        ledger world, but it is not ungraded -- it ships its parent's board, so
        the parent's grade is the honest number to show."""
        if self.gworld is not None and self.gworld.grade:
            return self.gworld.grade
        if self.world:
            return self.world.get('grade')
        for st in reversed(self.steps):
            if st.grade:
                return st.grade
        return self.parent_world.get('grade') if self.parent_world else None


def build_jobs(led, gen, verify=False):
    """The cells of one generation, derived from the ledger + the job dirs.

    A world's `stem` names the directory its job ran in, which is how a ledger
    world is attributed to a slot.  A descent slot with no ledger world still
    gets a cell -- evolve.py drops a descent that did not improve, and "this
    world tried and kept its parent" is part of the story.
    """
    gdir = os.path.join(led.root, f'g{gen}')
    rec = led.gen(gen) or {}
    new = [dict(w) for w in rec.get('new', [])]
    by_dir = {}
    for w in new:
        by_dir[os.path.dirname(os.path.abspath(w['stem']))] = w
    entering = led.entering_pop(gen)
    cells = []

    # row 0: the population entering the generation (static reference copies)
    for i, w in enumerate(entering):
        ww = led.by_name.get(w['name'], w)
        board = w['stem'] + '.kicad_pcb'
        steps = [Step(board, 'standing')] if os.path.exists(board) else []
        cells.append(Cell(0, i, ww.get('kind', 'seed'), w['name'], world=w,
                          steps=steps, detail=ww.get('detail', '')))

    # row 1: one descent per entering world, in the same column as its parent
    for i, w in enumerate(entering):
        d = os.path.join(gdir, f'w{i}')
        if not os.path.isdir(d):
            continue
        prefix = os.path.join(d, f'd_rp_k{led.K}')
        steps = descent_steps(d, prefix)
        landed = by_dir.get(os.path.abspath(d))
        cells.append(Cell(1, i, 'descend',
                          landed['name'] if landed else f'descend {w["name"]}',
                          parents=[w['name']], world=landed, steps=steps,
                          detail='' if landed else 'no gain: parent kept',
                          parent_world=w))

    # row 2: the far operators -- a jump/cross cell shows its parent's board
    # first, so the cut to the landed board IS its whole lineage change
    col = 0
    for d in sorted(glob.glob(os.path.join(gdir, 'j*')) +
                    glob.glob(os.path.join(gdir, 'x*'))):
        if not os.path.isdir(d):
            continue
        landed = by_dir.get(os.path.abspath(d))
        kind = 'jump' if os.path.basename(d).startswith('j') else 'cross'
        stem = os.path.join(d, 'jw' if kind == 'jump' else 'xw')
        board = stem + '.kicad_pcb'
        if landed:
            parents = landed.get('parents') or parse_origin(landed['origin'])[1]
            detail = landed.get('detail') or parse_origin(landed['origin'])[2]
        else:
            parents, detail = [], 'produced no world'
        steps = []
        pw = led.by_name.get(parents[0]) if parents else None
        if pw and os.path.exists(pw['stem'] + '.kicad_pcb'):
            steps.append(Step(pw['stem'] + '.kicad_pcb', f'from {pw["name"]}'))
        if os.path.exists(board):
            steps.append(Step(board, 'lands'))
        cells.append(Cell(2, col, kind, landed['name'] if landed else kind.upper(),
                          parents=parents, world=landed, steps=steps,
                          detail=detail, parent_world=pw))
        col += 1

    # the post-landing descents evolve.py runs on the fresh worlds before
    # selection: `n<i>` is the i-th non-descend world in `new`, in that order.
    fresh = [w for w in new if not str(w.get('origin', '')).startswith('descend')]
    for i, w in enumerate(fresh):
        d = os.path.join(gdir, f'n{i}')
        if not os.path.isdir(d):
            continue
        prefix = os.path.join(d, f'd_rp_k{led.K}')
        steps = descent_steps(d, prefix)
        if not steps:
            continue
        landed = by_dir.get(os.path.abspath(d))
        host = next((c for c in cells if c.world and c.world['name'] == w['name']), None)
        if host is not None:
            host.steps.extend(steps[1:] if len(steps) > 1 else steps)
            if landed:
                host.world = landed
                host.title = landed['name']
        else:
            cells.append(Cell(2, col, 'descend',
                              landed['name'] if landed else f'descend {w["name"]}',
                              parents=[w['name']], world=landed, steps=steps))
            col += 1
    return cells


def resolve_diffs(cells, verify=False, out=sys.stdout):
    """Fill every step's added/removed copper, and drop steps that changed
    nothing (a re-emitted board, or a round that shipped what a probe already
    laid) so every rendered step is a real change."""
    for c in cells:
        prev = None
        keep = []
        for st in c.steps:
            try:
                cur = copper_keys(_pcb(st.board))
            except Exception as e:                              # noqa: BLE001
                print(f'  [skip] {os.path.basename(st.board)}: {e}', file=out)
                continue
            a_s, a_v, r_s, r_v, nets = diff_copper(prev, cur)
            if prev is not None and not (a_s or a_v or r_s or r_v):
                continue                    # nothing moved: not a step
            st.added_s, st.added_v, st.removed_s, st.removed_v = a_s, a_v, r_s, r_v
            st.nets = nets
            if prev is not None:
                st.note = (f'{", ".join(nets[:3])}{"..." if len(nets) > 3 else ""} '
                           f'+{len(a_s)}/-{len(r_s)} seg')
                if a_v or r_v:
                    st.note += f'  +{len(a_v)}/-{len(r_v)} via'
            if verify and prev is not None:
                ok = (st.log_net in nets) if st.log_net else None
                print(f'  {c.title:>10s} {os.path.basename(st.board):<44s} '
                      f'segs {sum(prev[0].values())}->{sum(cur[0].values())} '
                      f'vias {sum(prev[1].values())}->{sum(cur[1].values())}  '
                      f'+{len(a_s)}s/-{len(r_s)}s +{len(a_v)}v/-{len(r_v)}v  '
                      f'nets {nets}'
                      + (f'  log said {st.log_net}: '
                         f'{"IN DIFF" if ok else "NOT IN DIFF"}' if st.log_net else ''),
                      file=out)
            prev = cur
            keep.append(st)
        c.steps = keep


# --------------------------------------------------------------------------
# SEVERAL runs as ONE evolution: identity by copper, order by time
# --------------------------------------------------------------------------
# A day's search is not one ledger.  A run gets stopped, fixed and RESEEDED
# from the previous run's best worlds, and standalone `replan.py` descents are
# run beside it on whichever world looks most promising.  Filmed per ledger,
# each of those looks like a fresh start from nowhere; filmed together they are
# one continuous evolution, which is what they are.
#
# Two derivations make that possible and neither needs a name:
#   IDENTITY  a world IS its routed copper.  A seed copied forward with
#             copy_board has byte-identical copper, so `dedupe_boards
#             .fingerprint` matches it back to the world it continues -- no new
#             node, and the lineage edge comes from the world it already was.
#             The same match deduplicates a descent that merely reproduces an
#             earlier board (evolve.py's own dedupe, surfacing across runs).
#   ORDER     a chapter is placed by WHEN ITS WORK BEGAN -- the earliest mtime
#             among its own artifacts -- not by the order the tags were typed
#             and not by the ledger's write time, which is when the run
#             FINISHED.  Runs overlap in a day; only start times interleave
#             them correctly.
_FP_CACHE = {}


def fingerprint_of(board):
    """`dedupe_boards.fingerprint`, memoised (it parses the whole board)."""
    fp = _FP_CACHE.get(board)
    if fp is None:
        import dedupe_boards
        fp = dedupe_boards.fingerprint(board)
        _FP_CACHE[board] = fp
    return fp


_START = re.compile(r'^\s+start:\s+open \[(.*?)\], drc (\d+), vias (\d+)')


class GlobalWorld:
    """One world in the stitched timeline, whatever run produced it."""

    def __init__(self, gid, label, run, grade, kind, parents, born, origin='', detail='', stem=''):
        self.gid, self.label, self.run = gid, label, run
        self.grade, self.kind, self.parents = grade, kind, list(parents)
        self.born, self.origin, self.detail, self.stem = born, origin, detail, stem


class Registry:
    """Copper fingerprint -> global world.  The whole stitch rests on this."""

    def __init__(self):
        self.by_fp = {}
        self.worlds = {}
        self.matches = 0                # seeds that continued an earlier world
        self.fresh = 0                  # seeds nothing earlier produced

    def lookup(self, board):
        if not os.path.exists(board):
            return None
        try:
            return self.worlds.get(self.by_fp.get(fingerprint_of(board)))
        except Exception:                                       # noqa: BLE001
            return None

    def add_or_match(self, board, gid, label, run, grade, kind, parents, born,
                     origin='', detail='', stem=''):
        """Register a world, or return the one whose copper this already is.

        Returns (world, is_new).  A caller that gets is_new False must NOT draw
        a second node for it: the board is a world the film already has.
        """
        fp = None
        if os.path.exists(board):
            try:
                fp = fingerprint_of(board)
            except Exception:                                   # noqa: BLE001
                fp = None
        if fp is not None and fp in self.by_fp:
            return self.worlds[self.by_fp[fp]], False
        w = GlobalWorld(gid, label, run, grade, kind, parents, born, origin, detail, stem)
        self.worlds[gid] = w
        if fp is not None:
            self.by_fp[fp] = gid
        return w, True


class LedgerSource:
    """One `evolve.py` ledger; each of its generations becomes a chapter."""

    def __init__(self, tag, path, K):
        self.tag, self.K = tag, K
        self.led = Ledger(path)
        self.alias = {}                 # run-local world name -> global world
        self.fresh = set()              # local names that entered as NEW worlds

    def chapters(self):
        for g in self.led.gens:
            d = os.path.join(self.led.root, f'g{g["gen"]}')
            yield {'kind': 'gen', 'src': self, 'gen': g['gen'], 't0': _began(d),
                   'label': self.tag, 'title': f'{self.tag}  generation {g["gen"]}'}


class DescentSource:
    """A standalone `replan.py` output directory as one chapter.

    Discovered, not declared: the round-0 board names the stem prefix, and the
    transcript is whichever `*.out` in the directory carries the round grammar
    -- so `replan.out`, `d.out` or any other spelling reads the same.
    """

    def __init__(self, path, K):
        self.dir = os.path.abspath(path)
        self.K = K
        self.name = os.path.basename(self.dir.rstrip('/'))
        r0 = sorted(glob.glob(os.path.join(self.dir, f'*_rp_k{K}_r0.kicad_pcb')))
        self.prefix = r0[0][:-len('_r0.kicad_pcb')] if r0 else None
        self.log = None
        for cand in sorted(glob.glob(os.path.join(self.dir, '*.out'))):
            try:
                with open(cand, encoding='utf-8', errors='replace') as f:
                    head = f.read(200000)
                if '=== round' in head:
                    self.log = cand
                    break
            except OSError:
                continue
        self.start_grade = None
        if self.log:
            with open(self.log, encoding='utf-8', errors='replace') as f:
                for line in f:
                    m = _START.match(line)
                    if m:
                        opens = [o.strip().strip("'\"") for o in m.group(1).split(',') if o.strip()]
                        self.start_grade = [opens, int(m.group(2)), int(m.group(3))]
                        break
                    if line.startswith('=== round'):
                        break

    def ok(self):
        return bool(self.prefix) and os.path.exists(self.prefix + '_r0.kicad_pcb')

    def chapters(self):
        if not self.ok():
            return
        yield {'kind': 'descent', 'src': self, 'gen': 0,
               't0': os.path.getmtime(self.prefix + '_r0.kicad_pcb'),
               'label': self.name, 'title': f'{self.name}  standalone descent'}

    def steps(self):
        """The chain, read with the SAME transcript parser a population
        descent uses -- `replan.py` writes one grammar, not two."""
        return descent_steps(os.path.dirname(self.prefix), self.prefix, self.log)


class Chapter:
    """One generation on screen, from either kind of source."""

    def __init__(self, idx, kind, title, label, cells, entering, kept, best, note=''):
        self.idx, self.kind, self.title, self.label = idx, kind, title, label
        self.cells, self.entering, self.kept = cells, entering, kept
        self.best, self.note = best, note


def register_chain(reg, cell, parent_w, chapter, run, gid_prefix, label,
                   final_world=None):
    """Every board a descent PASSED THROUGH that improved on the one before it
    is a world, chained parent to child.

    Two reasons this is not just the endpoint.  A descent's intermediate boards
    are real -- they were on disk, they were graded, and later runs seeded from
    them (one crashed mid-round on the day this was built and the board it had
    reached by then became the seed of three later runs, so read only from the
    ledger it is invisible and every run after it starts from nowhere).  And
    the RECORD is a function of time, not of chapters: a descent that walks
    98 -> 96 -> 95 set two records inside one chapter, and collapsing it to its
    endpoint silently deletes one of them from the film's own headline.

    Birth times are fractional inside the chapter -- (chapter-1, chapter] -- so
    the ladder orders correctly against everything else in the day.
    """
    prev = parent_w
    made = []
    graded = [(i, st) for i, st in enumerate(cell.steps) if st.grade]
    for k, (i, st) in enumerate(graded):
        is_last = (k == len(graded) - 1)
        if not better(st.grade, prev.grade if prev else None):
            continue
        born = (chapter - 1) + (k + 1) / max(1, len(graded))
        if is_last and final_world is not None:
            # The ledger already named this board.  Hang it off the ladder ONLY
            # if this chapter is where it first appeared: a world's birth is the
            # first time its copper existed, and a later run that walks the same
            # deterministic descent again is a DEDUPE HIT, not a re-birth.
            # (Measured: one run's recorded world is byte-identical to a
            # standalone descent's third round two chapters earlier, and moving
            # its birth forward dragged two records out of the record line.)
            if final_world.born >= chapter:
                final_world.parents = [prev.gid] if prev else final_world.parents
                final_world.born = min(final_world.born, chapter)
                prev = final_world
            made.append(final_world)
            continue
        w, _new = reg.add_or_match(
            st.board, f'{gid_prefix}:{i}', label, run, st.grade, 'descend',
            [prev.gid] if prev else [], born,
            detail=(st.note or 'a board the descent passed through'),
            stem=st.board[:-len('.kicad_pcb')])
        prev = w
        made.append(w)
    return made


def build_timeline(sources, K, out=sys.stdout, verify=False):
    """Every chapter, in time order, with one global identity per board."""
    descs = []
    for s in sources:
        descs.extend(s.chapters())
    descs.sort(key=lambda c: c['t0'])
    reg = Registry()
    chapters = []
    for i, cd in enumerate(descs, 1):
        src = cd['src']
        if cd['kind'] == 'gen':
            ch = _gen_chapter(reg, src, cd, i, out, verify)
        else:
            ch = _descent_chapter(reg, src, cd, i, out, verify)
        if ch is not None:
            chapters.append(ch)
    return chapters, reg


def _alias(reg, src, w, chapter, entering=False):
    """The global world a run-local ledger world names, registering it the
    first time.  A seed whose copper matches an earlier world IS that world."""
    if w['name'] in src.alias:
        return src.alias[w['name']]
    kind, parents, detail = parse_origin(w.get('origin'))
    # a world ENTERING a chapter was produced before that chapter's work, so it
    # is born one step earlier -- otherwise the seeds share a birthday with the
    # first generation's offspring and the record line has nothing to start from
    born = max(0, chapter - 1) if entering else chapter
    gw, is_new = reg.add_or_match(
        w['stem'] + '.kicad_pcb', f'{src.tag}:{w["name"]}', w['name'], src.tag,
        w.get('grade'), kind,
        [src.alias[p].gid for p in parents if p in src.alias],
        born, origin=w.get('origin', ''), detail=detail, stem=w['stem'])
    if is_new:
        reg.fresh += 1
        src.fresh.add(w['name'])
    else:
        reg.matches += 1
    src.alias[w['name']] = gw
    return gw


def _gen_chapter(reg, src, cd, idx, out, verify):
    gen = cd['gen']
    cells = build_jobs(src.led, gen)
    resolve_diffs(cells, verify=verify, out=out)
    entering = [_alias(reg, src, w, idx, entering=True) for w in src.led.entering_pop(gen)]
    rec = src.led.gen(gen) or {}
    # every world the generation RECORDED, in the ledger's own order
    for w in rec.get('new', []):
        _alias(reg, src, w, idx)
    for c in cells:
        pw = None
        if c.parents:
            pw = src.alias.get(c.parents[0])
        if c.world is not None:
            c.gworld = _alias(reg, src, c.world, idx)
            if c.row > 0 and len(c.steps) > 1:
                register_chain(reg, c, pw, idx, src.tag,
                               f'{src.tag}:g{gen}r{c.row}c{c.col}', c.gworld.label,
                               final_world=c.gworld)
        elif c.row > 0:
            made = register_chain(reg, c, pw, idx, src.tag,
                                  f'{src.tag}:g{gen}r{c.row}c{c.col}',
                                  f'g{gen}w{c.col}')
            if made:
                c.gworld = made[-1]
                c.gworld.detail = 'reached, not recorded (run stopped)'
                c.detail = c.gworld.detail
                c.title = c.gworld.label
        c.gparents = [src.alias[p].gid for p in c.parents if p in src.alias]
        if c.row == 0 and c.gworld is not None:
            # row 0 is what ENTERS: a world whose copper an earlier chapter
            # already produced is the same world continuing, so it is captioned
            # with the identity it continues, not with this run's local name
            c.kind = c.gworld.kind
            c.title = c.gworld.label
            local = c.world.get('name') if c.world else None
            if c.gworld.kind == 'seed' and local in src.fresh:
                # a SEED whose copper nothing filmed produced: it enters the day
                # from outside.  A world this run made in an earlier chapter is
                # also "fresh" to the registry but is not a new entry -- it is
                # the run's own previous work continuing.
                c.detail = f'new entry: {parse_origin(c.world.get("origin"))[2]}'
            elif c.gworld.run != src.tag:
                c.detail = f'continues from {c.gworld.run}'
            else:
                c.detail = 'continues'
    kept = {src.alias[w['name']].gid for w in (rec.get('pop') or [])
            if w['name'] in src.alias}
    best = src.alias.get((rec.get('best') or {}).get('name'))
    return Chapter(idx, 'gen', cd['title'], cd['label'], cells,
                   [w.gid for w in entering], kept, best,
                   note='elitist on (open, drc, vias), deduplicated by copper')


def _descent_chapter(reg, src, cd, idx, out, verify):
    steps = src.steps()
    if not steps:
        print(f'  note: {src.name}: no round boards -- not filmed', file=out)
        return None
    parent_board = steps[0].board
    pw = reg.lookup(parent_board)
    if pw is None:
        # nothing filmed so far produced this board: it enters as its own root
        pw, _ = reg.add_or_match(
            parent_board, f'{src.name}:from', 'from', src.name,
            src.start_grade, 'seed', [], max(0, idx - 1),
            detail='imported: no filmed world has this copper',
            stem=parent_board[:-len('.kicad_pcb')])
        reg.fresh += 1
    else:
        reg.matches += 1
    p_cell = Cell(0, 0, pw.kind, pw.label, world=None, steps=[Step(parent_board, 'standing')],
                  detail=pw.detail or pw.origin)
    p_cell.gworld = pw
    d_cell = Cell(1, 0, 'descend', src.name, parents=[pw.label], steps=steps,
                  parent_world={'grade': pw.grade})
    resolve_diffs([p_cell, d_cell], verify=verify, out=out)
    made = register_chain(reg, d_cell, pw, idx, src.name,
                          f'{src.name}:step', src.name)
    res = made[-1] if made else None
    if res is not None:
        res.detail = 'standalone replan descent'
    d_cell.gworld = res
    d_cell.gparents = [pw.gid]
    p_cell.gparents = []
    if res is not None:
        d_cell.title = f'{src.name} -> {res.grade[2]}v'
    return Chapter(idx, 'descent', cd['title'], cd['label'], [p_cell, d_cell],
                   [pw.gid], {res.gid} if res else set(), res,
                   note='a standalone replan.py descent, folded in at its place in the day')


def _began(path):
    """When the work in this directory STARTED: the earliest mtime among its
    own entries (its own mtime is when it last changed, i.e. when it ended)."""
    try:
        kids = [os.path.join(path, k) for k in os.listdir(path)]
        ts = [os.path.getmtime(k) for k in kids] or [os.path.getmtime(path)]
        return min(ts)
    except OSError:
        return float('inf')


# --------------------------------------------------------------------------
# the stage: slot geometry
# --------------------------------------------------------------------------
class Stage:
    """Where each cell is drawn.

    Rows are SEMANTIC (population / descent / far operators), so a descent sits
    directly under its parent and that lineage is geometry rather than an arrow.

    The cell size comes from the HEIGHT budget -- the number of rows is fixed by
    the algorithm, the number of columns is not -- and the stage then asks for
    exactly the width those columns need.  That is why the canvas fits the run:
    a two-world generation would otherwise sit in a void sized for a four-world
    one.  `max_w` caps it, shrinking the cells instead of overflowing.
    """

    CAP_H = 52
    GAP = 16

    def __init__(self, rows, cols, top, avail_h, view, max_w, cell=None):
        vw, vh = max(view[2] - view[0], 1e-6), max(view[3] - view[1], 1e-6)
        aspect = vw / vh
        if cell:
            bw, bh = float(cell[0]), float(cell[1])
        else:
            bh = (avail_h - self.GAP * (rows + 1)) / rows - self.CAP_H
            bw = bh * aspect
            need = cols * bw + self.GAP * (cols + 1)
            if need > max_w:                    # width-bound: shrink to fit
                bw = (max_w - self.GAP * (cols + 1)) / cols
                bh = bw / aspect
        self.cell = (max(40, int(bw)), max(30, int(bh)))
        self.pitch = (self.cell[0] + self.GAP, self.cell[1] + self.CAP_H + self.GAP)
        self.width = cols * self.pitch[0] + self.GAP
        self.height = rows * self.pitch[1] + self.GAP
        self.rows, self.cols, self.top = rows, cols, top
        self.x0 = 0

    def centre_in(self, canvas_w):
        self.x0 = max(0, (canvas_w - self.width) // 2)

    def rect(self, row, col):
        x = self.x0 + self.GAP + col * self.pitch[0]
        y = self.top + self.GAP + row * self.pitch[1]
        return (x, y, x + self.cell[0], y + self.cell[1] + self.CAP_H)

    def board_origin(self, rect):
        """Top-left of the board image inside a slot (caption below it)."""
        return (int(rect[0]), int(rect[1]))


# --------------------------------------------------------------------------
# the lineage ribbon: every world as (generation, via count)
# --------------------------------------------------------------------------
class Ribbon:
    """The search's own trajectory, over the WHOLE stitched day.

    x is the chapter a world was born in, y its via count (lower is higher,
    because lower is better); edges run parent -> child in the child's operator
    colour, and they cross run boundaries because identity is copper, not which
    ledger recorded it.  A world with open nets is drawn hollow: it is not
    admissible however few vias it has.

    The gold step-line is the RECORD -- the best admissible via count reached by
    the end of each chapter -- labelled at every drop.  That line is the day's
    actual result, and it is the one thing a viewer should be able to read off
    this film without knowing anything else about it.
    """

    def __init__(self, reg, chapters, box):
        self.reg, self.box = reg, box
        self.n = max([c.idx for c in chapters] + [1])
        self.chapters = chapters
        vs = [w.grade[2] for w in reg.worlds.values() if w.grade]
        self.vmin, self.vmax = (min(vs), max(vs)) if vs else (0, 1)
        if self.vmax - self.vmin < 1:
            self.vmax = self.vmin + 1
        x0, y0, x1, y1 = box
        self.plot = (x0 + 54, y0 + 24, x1 - 14, y1 - 22)
        self.nodes = {}
        per = {}
        for w in sorted(reg.worlds.values(), key=lambda w: (w.born, w.gid)):
            if not w.grade:
                continue
            i = per.get(round(w.born, 3), 0)
            per[round(w.born, 3)] = i + 1
            self.nodes[w.gid] = (self._x(w.born) + (i % 3 - 1) * 7, self._y(w.grade[2]), w)
        # The record is a function of TIME, sampled at every instant a world was
        # born -- not once per chapter.  A descent that walks 98 -> 96 -> 95 set
        # two records inside one chapter, and a per-chapter sample keeps only
        # the last of them.
        times = sorted({w.born for w in reg.worlds.values() if w.grade})
        self.record = []
        best = None
        for t in times:
            for w in reg.worlds.values():
                if w.born <= t and w.grade and not w.grade[0]:
                    if best is None or w.grade[2] < best:
                        best = w.grade[2]
            if best is not None:
                self.record.append((t, best))

    def _x(self, chapter):
        px0, _, px1, _ = self.plot
        return px0 + (px1 - px0) * (chapter / max(1, self.n))

    def _y(self, vias):
        _, py0, _, py1 = self.plot
        f = (vias - self.vmin) / max(1e-9, self.vmax - self.vmin)
        return py0 + f * (py1 - py0)

    def draw(self, d, upto, phase, best_gid=None, pop_gids=()):
        x0, y0, x1, y1 = self.box
        px0, py0, px1, py1 = self.plot
        d.rectangle([x0, y0, x1, y1], fill=PANEL, outline=PANEL_EDGE)
        f = load_font(12)
        fs = load_font(11)
        cap = ('lineage -- chapter (x) vs vias (y, lower is better);'
               ' hollow = open nets;  gold = the record')
        d.text((px1 - d.textlength(cap, font=f), y0 + 5), cap, fill=DIM, font=f)
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            yy = py0 + frac * (py1 - py0)
            val = int(round(self.vmin + frac * (self.vmax - self.vmin)))
            d.line([px0, yy, px1, yy], fill=(32, 36, 42))
            d.text((x0 + 10, yy - 6), f'{val}v', fill=FAINT, font=fs)
        for c in self.chapters:
            d.text((self._x(c.idx) - 10, y1 - 16), str(c.idx), fill=FAINT, font=fs)

        def vis(w):
            horizon = upto if phase in ('work', 'select') else upto - 1
            return w.born <= horizon + 1e-9

        for gid, (cx, cy, w) in self.nodes.items():
            if not vis(w):
                continue
            for pg in w.parents:
                pn = self.nodes.get(pg)
                if pn and vis(pn[2]):
                    d.line([pn[0], pn[1], cx, cy], fill=kind_colour(w.kind), width=1)
        # the record step-line, drawn over the edges and under the nodes
        horizon = upto if phase in ('work', 'select') else upto - 1
        pts, last, shown = [], None, set()
        for t, r in self.record:
            if t > horizon + 1e-9:
                break
            if last is not None and r != last:
                pts += [(self._x(t), self._y(last))]
            pts += [(self._x(t), self._y(r))]
            if r not in shown:
                shown.add(r)
                d.text((self._x(t) - 8, self._y(r) - 17), f'{r}', fill=BEST, font=fs)
            last = r
        if len(pts) > 1:
            d.line([p for xy in pts for p in xy], fill=BEST, width=2)

        for gid, (cx, cy, w) in self.nodes.items():
            if not vis(w):
                continue
            col = kind_colour(w.kind)
            r = 5 if gid == best_gid else 4
            if w.grade[0]:
                d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=col, width=2)
            else:
                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)
            if gid in pop_gids:
                d.ellipse([cx - r - 3, cy - r - 3, cx + r + 3, cy + r + 3],
                          outline=KEPT, width=1)
            if gid == best_gid:
                d.ellipse([cx - r - 5, cy - r - 5, cx + r + 5, cy + r + 5],
                          outline=BEST, width=2)


# --------------------------------------------------------------------------
# frame composition
# --------------------------------------------------------------------------
def _arrow(d, a, b, colour, dashed=False, width=2):
    ax, ay = a
    bx, by = b
    if dashed:
        n = max(2, int(math.hypot(bx - ax, by - ay) / 9))
        for i in range(n):
            if i % 2:
                continue
            t0, t1 = i / n, (i + 1) / n
            d.line([ax + (bx - ax) * t0, ay + (by - ay) * t0,
                    ax + (bx - ax) * t1, ay + (by - ay) * t1], fill=colour, width=width)
    else:
        d.line([ax, ay, bx, by], fill=colour, width=width)
    ang = math.atan2(by - ay, bx - ax)
    for s in (+1, -1):
        d.line([bx, by, bx - 11 * math.cos(ang + s * 0.42),
                by - 11 * math.sin(ang + s * 0.42)], fill=colour, width=width)


def _text_fit(d, xy, text, font, fill, max_w):
    """Draw text, ellipsised to fit -- a caption that silently overflows its
    cell is worse than one that says it was cut."""
    if d.textlength(text, font=font) <= max_w:
        d.text(xy, text, fill=fill, font=font)
        return
    while text and d.textlength(text + '...', font=font) > max_w:
        text = text[:-1]
    d.text(xy, text + '...', fill=fill, font=font)


class Film:
    """Composes frames and streams them to disk.

    Frames are written as they are made rather than accumulated: this canvas is
    ~5 MB per frame as RGB, so a few hundred frames would be a gigabyte of RAM
    on a machine that is also routing.  `animate_route.save_movie` is the
    repo's shared encoder but it materialises every frame, so it is the
    FALLBACK here and ffmpeg over the PNG sequence is the primary path.
    """

    def __init__(self, tagline, K, cam, stage, ribbon, canvas, frames_dir, out=sys.stdout):
        self.tagline, self.K = tagline, K
        self.cam, self.stage, self.ribbon = cam, stage, ribbon
        self.W, self.H = canvas
        self.dir = frames_dir
        self.n = 0
        self.out = out
        os.makedirs(frames_dir, exist_ok=True)

    # -- one frame -------------------------------------------------------
    def compose(self, cells, step_of, hot_of, gen, phase, title, subtitle,
                best, pop_names, note='', dim=0.40):
        """One frame.  `dim` is how far a world DROPPED by selection has faded
        (1.0 = still full brightness), so the selection scene can ramp it and
        the losers visibly leave the population rather than blinking out."""
        img = Image.new('RGB', (self.W, self.H), BG)
        d = ImageDraw.Draw(img)
        f_title = load_font(26)
        f_sub = load_font(14)
        f_cap = load_font(14)
        f_small = load_font(12)

        # title bar
        d.rectangle([0, 0, self.W, 76], fill=PANEL)
        d.line([0, 76, self.W, 76], fill=PANEL_EDGE)
        d.text((18, 12), title, fill=TEXT, font=f_title)
        _text_fit(d, (18, 46), subtitle, f_sub, DIM, self.W - 520)
        if best is not None:
            rec = f'record  {fmt_grade(best.grade)}   ({best.label})'
            w = d.textlength(rec, font=f_sub)
            d.text((self.W - w - 18, 46), rec, fill=BEST, font=f_sub)
            lab = f'{self.tagline}   K={self.K}'
            w2 = d.textlength(lab, font=f_title)
            d.text((self.W - w2 - 18, 12), lab, fill=DIM, font=f_title)

        # lineage arrows for the far operators, drawn BEFORE the cells so they
        # pass behind the boards instead of across them (a descent needs none:
        # it is drawn directly under its parent, which is the relation)
        bw_, bh_ = self.stage.cell
        for c in cells:
            if c.kind not in ('jump', 'cross') or c.rect is None:
                continue
            for j, pname in enumerate(c.parents):
                p = next((k for k in cells if k.row == 0 and k.title == pname), None)
                if p is None or p.rect is None:
                    continue
                _arrow(d, (p.rect[0] + bw_ / 2, p.rect[1] + bh_),
                       (c.rect[0] + bw_ / 2, c.rect[1] - 3),
                       kind_colour(c.kind), dashed=(j > 0))

        # cells
        for c in cells:
            if c.rect is None:
                continue
            bx, by = self.stage.board_origin(c.rect)
            bw, bh = self.stage.cell
            si = step_of(c)
            dimmed = c.status == 'dropped'
            if si is None:
                # a slot this chapter WILL fill reads as waiting, not as a hole:
                # the roll-call then shows the shape of the generation to come
                d.rectangle([bx, by, bx + bw, by + bh], fill=(16, 18, 21),
                            outline=(kind_colour(c.kind) if c.steps else PANEL_EDGE))
                lab = (c.kind if c.steps else 'no board')
                _text_fit(d, (bx + 10, by + bh / 2 - 8), lab, f_cap,
                          FAINT, bw - 20)
            else:
                st = c.steps[si]
                cell = self.cam.image(st, hot_of(c))
                if dimmed and dim < 0.999:
                    cell = cell.point(lambda p, k=dim: int(p * k))
                img.paste(cell, (bx, by))
                is_best = best is not None and c.gworld is not None and c.gworld.gid == best.gid
                col = BEST if is_best else (KEPT if c.status == 'kept' else
                                            (DROPPED if dimmed else kind_colour(c.kind)))
                d.rectangle([bx - 1, by - 1, bx + bw, by + bh], outline=col,
                            width=3 if (c.status or is_best) else 1)
                # step pips: how far through its own timeline this cell is
                if len(c.steps) > 1:
                    for k in range(len(c.steps)):
                        px = bx + 8 + k * 11
                        d.ellipse([px, by + bh - 12, px + 6, by + bh - 6],
                                  fill=ADDED if k <= si else (60, 64, 70))

            # caption
            cy = by + bh + 5
            tcol = FAINT if dimmed else TEXT
            head = c.title
            if best is not None and c.gworld is not None and c.gworld.gid == best.gid:
                head = '* ' + head
            waiting = si is None and bool(c.steps)
            _text_fit(d, (bx, cy), head, f_cap, FAINT if waiting else tcol, bw)
            if not waiting:      # no spoilers: an unrun slot has no result yet
                _text_fit(d, (bx, cy + 16), fmt_grade(c.grade), f_small,
                          FAINT if dimmed else DIM, bw)
            line3 = ''
            if si is not None and c.steps:
                st = c.steps[si]
                # a cell with no ledger world says WHY (the operator produced
                # nothing) rather than narrating the board it kept
                line3 = st.note or (c.detail if (c.world is None or c.row == 0)
                                    else st.label)
            if not line3:
                line3 = c.detail or c.kind
            _text_fit(d, (bx, cy + 32), line3, f_small,
                      FAINT if dimmed else kind_colour(c.kind), bw)

        # ribbon + legend
        self.ribbon.draw(d, gen, phase, best_gid=best.gid if best is not None else None,
                         pop_gids=pop_names)
        y = self.H - 30
        d.rectangle([0, y - 4, self.W, self.H], fill=PANEL)
        x = 18
        for kind in ('seed', 'descend', 'jump', 'cross'):
            d.rectangle([x, y + 3, x + 11, y + 14], fill=kind_colour(kind))
            d.text((x + 16, y + 2), kind, fill=DIM, font=f_small)
            x += 22 + d.textlength(kind, font=f_small) + 16
        for lab, col in (('copper added', ADDED), ('copper removed', REMOVED),
                         ('kept', KEPT), ('dropped', DROPPED), ('best', BEST)):
            d.rectangle([x, y + 3, x + 11, y + 14], fill=col)
            d.text((x + 16, y + 2), lab, fill=DIM, font=f_small)
            x += 22 + d.textlength(lab, font=f_small) + 16
        if note:
            _text_fit(d, (x + 10, y + 2), note, f_small, DIM, self.W - x - 20)
        return img

    def emit(self, img, n=1):
        for _ in range(max(1, int(n))):
            img.save(os.path.join(self.dir, f'f{self.n:06d}.png'))
            self.n += 1


# --------------------------------------------------------------------------
# the film script
# --------------------------------------------------------------------------
def open_sources(args, out=sys.stdout):
    """Every source named on the command line that can actually be filmed.

    A run with no ledger yet is NAMED and skipped rather than guessed at: its
    worlds have no grade, so there is nothing to caption or rank them by.  That
    is the in-flight case, and it stays a refusal.
    """
    srcs = []
    for tag in args.runs:
        lpath = os.path.join(HERE, 'tmp', tag, f'evolve_k{args.K}.json')
        if not os.path.exists(lpath):
            where = os.path.join(HERE, 'tmp', tag)
            print(f'  note: {tag}: no ledger at tmp/{tag}/evolve_k{args.K}.json'
                  + (' (run in flight)' if os.path.isdir(where) else ' (no such run)')
                  + ' -- not filmed', file=out)
            continue
        srcs.append(LedgerSource(tag, lpath, args.K))
    for d in args.descents:
        path = d if os.path.isabs(d) else os.path.join(HERE, d)
        src = DescentSource(path, args.K)
        if not src.ok():
            print(f'  note: {src.name}: no *_rp_k{args.K}_r0 board -- not filmed', file=out)
            continue
        srcs.append(src)
    return srcs


def run(args, out=sys.stdout):
    srcs = open_sources(args, out=out)
    if not srcs:
        raise SystemExit('evolve_movie: nothing to film')
    print(f'evolve_movie: K{args.K}; {len(srcs)} source(s): '
          + ', '.join(getattr(s, "tag", None) or s.name for s in srcs), file=out)
    for s in srcs:
        if isinstance(s, LedgerSource) and s.led._rerooted:
            print(f'  note: {s.tag}: {s.led._rerooted} recorded stem(s) re-rooted onto '
                  f'{os.path.relpath(s.led.root, HERE)}/', file=out)
        if isinstance(s, LedgerSource):
            for d_ in s.led.unrecorded_gen_dirs():
                print(f'  note: {s.tag}/{d_}/ on disk but not in the ledger '
                      f'(generation in flight) -- not filmed', file=out)

    chapters, reg = build_timeline(srcs, args.K, out=out, verify=args.verify)
    if not chapters:
        raise SystemExit('evolve_movie: no chapters')
    if args.gens:
        chapters = chapters[:args.gens]
    print(f'  {len(chapters)} chapter(s), {len(reg.worlds)} distinct world(s) by copper; '
          f'{reg.matches} seed/result(s) matched an earlier world, {reg.fresh} entered new',
          file=out)
    for c in chapters:
        print(f'    ch{c.idx:<2d} {c.title:<34s} '
              f'{len(c.cells)} cell(s)'
              + (f'  best {c.best.label} {fmt_grade(c.best.grade)}' if c.best else ''), file=out)

    # -- camera: one reference board, and the run nets unioned over every world
    ref = None
    for c in chapters:
        for cell in c.cells:
            if cell.steps:
                ref = cell.steps[0].board
                break
        if ref:
            break
    ref_pcb = _pcb(ref)
    names = set()
    for w in reg.worlds.values():
        if w.stem:
            names |= run_net_names(w.stem)
    view = args.view or derive_view(ref_pcb, names)
    print(f'  {len(names)} run net(s); view {", ".join(f"{v:.2f}" for v in view)}', file=out)

    # -- ONE stage geometry for the whole film, sized for the busiest chapter
    widest = max((sum(1 for c in ch.cells if c.row == r)
                  for ch in chapters for r in {c.row for c in ch.cells}), default=1)
    rowset = sorted({c.row for ch in chapters for c in ch.cells}) or [0]
    row_ix = {r: i for i, r in enumerate(rowset)}
    max_w, max_h = args.canvas or (1760, 1080)
    TITLE_H, LEGEND_H, MIN_W = 78, 34, 1220
    ribbon_h = 172 if len(chapters) <= 3 else 240
    stage = Stage(len(rowset), widest, TITLE_H,
                  max_h - TITLE_H - ribbon_h - LEGEND_H, view, max_w, cell=args.cell)
    W = max(MIN_W, stage.width)
    H = TITLE_H + stage.height + ribbon_h + LEGEND_H
    W += W & 1
    H += H & 1
    stage.centre_in(W)
    # each chapter's own columns are centred inside the stage, so a one-column
    # standalone descent does not sit in a hole sized for a four-world
    # generation -- while WITHIN a chapter the columns stay aligned, which is
    # what puts a descent directly under its parent
    for ch in chapters:
        used_c = max((c.col for c in ch.cells), default=0) + 1
        used_r = len({c.row for c in ch.cells})
        dx = ((widest - used_c) * stage.pitch[0]) // 2
        dy = ((len(rowset) - used_r) * stage.pitch[1]) // 2
        for c in ch.cells:
            r = stage.rect(row_ix[c.row], c.col)
            c.rect = (r[0] + dx, r[1] + dy, r[2] + dx, r[3] + dy)

    cam = Camera(ref_pcb, view, stage.cell[0], stage.cell[1], args.supersample)
    ribbon = Ribbon(reg, chapters, (0, H - ribbon_h - LEGEND_H, W, H - LEGEND_H))
    tagline = '+'.join(getattr(s, 'tag', None) or s.name for s in srcs)
    if len(tagline) > 38:
        tagline = f'{len(srcs)} runs'
    film = Film(tagline, args.K, cam, stage, ribbon, (W, H), args.frames_dir, out=out)
    print(f'  canvas {W}x{H}, cell {stage.cell[0]}x{stage.cell[1]}, '
          f'{len(rowset)} row(s) x {widest} column(s)', file=out)

    n_roll = max(1, int(args.roll * args.fps))
    n_step = max(1, int(args.hold * args.fps))
    n_hot = max(1, int(n_step * 0.6))
    n_sel = max(1, int(args.select * args.fps))
    best = None

    spans = []
    for ch in chapters:
        f0 = film.n
        for c in ch.cells:
            c.status = ''
        ent = [reg.worlds[g] for g in ch.entering if g in reg.worlds]
        sub = ' | '.join(f'{w.label} {fmt_grade(w.grade)}' for w in ent)
        title = f'{ch.title}'
        head = f'chapter {ch.idx}/{len(chapters)}'

        # roll-call: what enters this chapter
        img = film.compose(ch.cells,
                           lambda c: 0 if (c.row == 0 and c.steps) else None,
                           lambda c: False,
                           ch.idx, 'roll', title,
                           f'{head} -- entering: {sub}', best, set(ch.entering),
                           note=('the parent this descent starts from'
                                 if ch.kind == 'descent' else
                                 'each world descends; jumps and crossovers are created'))
        film.emit(img, n_roll)

        # work: every cell advances through its own steps AT THE SAME TIME
        T = max((len(c.steps) for c in ch.cells), default=1)
        for t in range(T):
            def step_of(c, t=t):
                return min(t, len(c.steps) - 1) if c.steps else None

            def age(c, t=t):
                return t - (len(c.steps) - 1) if c.steps else 0
            for fr in range(n_step):
                def hot_of(c, fr=fr, t=t):
                    return age(c) <= 0 and fr < n_hot and t > 0
                alpha = 1.0 - (fr / max(1, n_hot)) if fr < n_hot else 0.0
                note = f'step {t + 1}/{T} -- bright = copper added, pink = copper removed'
                base = film.compose(ch.cells, step_of, lambda c: False, ch.idx, 'work',
                                    title, f'{head} -- {sub}', best, set(ch.entering),
                                    note=note)
                if alpha > 0.01 and any(hot_of(c) for c in ch.cells):
                    hot = film.compose(ch.cells, step_of, hot_of, ch.idx, 'work',
                                       title, f'{head} -- {sub}', best, set(ch.entering),
                                       note=note)
                    base = Image.blend(base, hot, alpha)
                film.emit(base)

        # selection (a standalone descent "keeps" its result the same way)
        for c in ch.cells:
            c.status = 'kept' if (c.gworld is not None and c.gworld.gid in ch.kept) else 'dropped'
        if ch.best is not None and better(ch.best.grade, best.grade if best else None):
            best = ch.best
        for w in reg.worlds.values():
            if w.born <= ch.idx and better(w.grade, best.grade if best else None):
                best = w
        survivors = ' | '.join(f'{reg.worlds[g].label} {fmt_grade(reg.worlds[g].grade)}'
                               for g in ch.kept if g in reg.worlds)
        for i in range(n_sel):
            k = min(1.0, i / max(1.0, n_sel * 0.45))
            film.emit(film.compose(
                ch.cells, lambda c: len(c.steps) - 1 if c.steps else None,
                lambda c: False, ch.idx, 'select', title,
                f'{head} -- keeps: {survivors or "(nothing new)"}', best, ch.kept,
                note=ch.note, dim=1.0 - 0.62 * k))
        spans.append((ch, f0, film.n - 1))
        cam.clear()

    for ch, f0, f1 in spans:
        print(f'    ch{ch.idx:<2d} frames {f0}-{f1:<5d} {ch.title}', file=out)
    chain = []
    for _t, r in ribbon.record:
        if not chain or r != chain[-1]:
            chain.append(r)
    print(f'  record chain: {" -> ".join(str(r) for r in chain)}', file=out)
    print(f'  {film.n} frame(s), {cam.renders} board render(s) -> {args.frames_dir}',
          file=out)
    return film.n, (W, H)


# --------------------------------------------------------------------------
# encoding
# --------------------------------------------------------------------------
def encode(frames_dir, out_path, fps, n_frames, gif=False, log=sys.stdout):
    """PNG sequence -> mp4.  ffmpeg over the files is the primary path because
    it streams; the repo's shared `animate_route.save_movie` is the fallback
    (it materialises every frame, which is why it is not first here)."""
    pat = os.path.join(frames_dir, 'f%06d.png')
    ff = None
    for cand in ('/opt/homebrew/bin/ffmpeg', 'ffmpeg'):
        try:
            subprocess.run([cand, '-version'], capture_output=True, check=True)
            ff = cand
            break
        except Exception:                                       # noqa: BLE001
            continue
    ok = False
    if ff:
        cmd = [ff, '-y', '-loglevel', 'error', '-framerate', str(fps), '-i', pat,
               '-c:v', 'libx264', '-preset', 'slow', '-crf', '20',
               '-pix_fmt', 'yuv420p', out_path]
        r = subprocess.run(cmd, capture_output=True, text=True)
        ok = r.returncode == 0 and os.path.exists(out_path)
        if not ok:
            print(f'evolve_movie: ffmpeg failed: {r.stderr.strip()[:300]}', file=log)
    if not ok:
        try:
            import animate_route
            frames = [Image.open(os.path.join(frames_dir, f'f{i:06d}.png')).convert('RGB')
                      for i in range(n_frames)]
            animate_route.save_movie(frames, out_path, fps, end_hold=1.0)
            ok = True
        except Exception as e:                                  # noqa: BLE001
            print(f'evolve_movie: no encoder ({e})', file=log)
    if ok:
        sz = os.path.getsize(out_path) / 1e6 if os.path.exists(out_path) else 0
        print(f'evolve_movie: wrote {out_path} ({n_frames} frames @ {fps}fps, '
              f'{sz:.1f} MB)', file=log)
    if gif:
        gpath = os.path.splitext(out_path)[0] + '.gif'
        stride = max(1, n_frames // 260)
        frames = []
        for i in range(0, n_frames, stride):
            im = Image.open(os.path.join(frames_dir, f'f{i:06d}.png')).convert('RGB')
            frames.append(im.resize((im.width // 2, im.height // 2), Image.LANCZOS))
        if frames:
            frames[0].save(gpath, save_all=True, append_images=frames[1:],
                           duration=int(1000 * stride / fps), loop=0, optimize=True)
            print(f'evolve_movie: wrote {gpath} ({len(frames)} frames, '
                  f'{os.path.getsize(gpath) / 1e6:.1f} MB)', file=log)
    return ok


# --------------------------------------------------------------------------
def self_test():
    """Check the two derivations the whole film rests on, in milliseconds.

    Neither has a natural failure signal: a mis-parsed `origin` draws an arrow
    to the wrong parent and a mis-keyed diff highlights the wrong copper, and
    both produce a perfectly plausible-looking movie.  So they are asserted
    rather than eyeballed.
    """
    cases = [
        ('seed smoke15/r0/c0', 'seed', [], 'smoke15/r0/c0'),
        ('descend<s1>', 'descend', ['s1'], ''),
        ('descend<g1j9: jump<s0; bans [1]; seed 7>>', 'descend', ['g1j9'], None),
        ('jump<s0; bans [1, 2]; seed 797927>', 'jump', ['s0'], 'bans [1, 2]; seed 797927'),
        ('cross<s1 x s2; 23 from B>', 'cross', ['s1', 's2'], '23 from B'),
    ]
    for origin, kind, parents, detail in cases:
        k, p, d = parse_origin(origin)
        assert (k, p) == (kind, parents), f'parse_origin({origin!r}) -> {(k, p)}'
        assert detail is None or d == detail, f'parse_origin({origin!r}) detail {d!r}'

    class S:                                    # a minimal segment/via stand-in
        def __init__(self, x0, y0, x1, y1, layer='F.Cu', net=1, w=0.1):
            self.start_x, self.start_y, self.end_x, self.end_y = x0, y0, x1, y1
            self.layer, self.net_id, self.width = layer, net, w

    class V:
        def __init__(self, x, y, net=1):
            self.x, self.y, self.size, self.drill, self.net_id = x, y, 0.25, 0.15, net

    class Net:
        def __init__(self, n):
            self.name = n

    class P:
        def __init__(self, segs, vias):
            self.segments, self.vias = segs, vias
            self.nets = {1: Net('/BUS/A'), 2: Net('/BUS/B')}

    a = copper_keys(P([S(0, 0, 1, 0), S(1, 0, 2, 0)], [V(1, 0)]))
    # the SAME copper with one track written end-for-end and the list reordered
    b = copper_keys(P([S(2, 0, 1, 0), S(0, 0, 1, 0)], [V(1, 0)]))
    assert diff_copper(a, b)[:4] == ([], [], [], []), 'endpoint order is not a change'
    # one track moved: exactly one added and one removed, and the net is named
    c = copper_keys(P([S(0, 0, 1, 0), S(1, 0, 2, 1)], [V(1, 0)]))
    a_s, a_v, r_s, r_v, nets = diff_copper(a, c)
    assert (len(a_s), len(r_s), len(a_v), len(r_v)) == (1, 1, 0, 0), 'moved track'
    assert nets == ['A'], f'changed nets {nets}'
    # duplicated copper differences by MULTIPLICITY, which a set would miss
    dup = copper_keys(P([S(0, 0, 1, 0), S(0, 0, 1, 0), S(1, 0, 2, 0)], [V(1, 0)]))
    assert len(diff_copper(a, dup)[0]) == 1, 'a duplicate track is an addition'
    assert len(diff_copper(dup, a)[2]) == 1, 'losing a duplicate is a removal'
    # a via that moved, and a net rename that is not a geometry change
    d2 = copper_keys(P([S(0, 0, 1, 0), S(1, 0, 2, 0)], [V(1, 1)]))
    assert len(diff_copper(a, d2)[1]) == 1 and len(diff_copper(a, d2)[3]) == 1, 'via move'
    print('evolve_movie: self-test ok (origin grammar, copper diff)')


def _wh(s):
    m = re.match(r'^(\d+)x(\d+)$', s or '')
    if not m:
        raise argparse.ArgumentTypeError('expected WxH, e.g. 420x260')
    return (int(m.group(1)), int(m.group(2)))


def _view(s):
    p = [float(v) for v in (s or '').split(',')]
    if len(p) != 4:
        raise argparse.ArgumentTypeError('expected X0,Y0,X1,Y1 in mm')
    return (min(p[0], p[2]), min(p[1], p[3]), max(p[0], p[2]), max(p[1], p[3]))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Film an evolve.py run from its ledger.',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('tag', nargs='?', help='run tag: reads tmp/TAG/evolve_kK.json '
                    '(omit when --runs is given)')
    ap.add_argument('K', nargs='?', type=int, help='the K the run used')
    ap.add_argument('--runs', default='', help='TAG1,TAG2,... in time order: film several '
                    'ledgers as ONE evolution (a run reseeded from the last one continues it; '
                    'worlds are matched by copper fingerprint, chapters ordered by when their '
                    'work began)')
    ap.add_argument('--descents', default='', help='DIR[,DIR...]: standalone replan.py output '
                    'directories, folded in as descent chapters of the world they started from')
    ap.add_argument('--name', default=None, help='basename for a stitched movie '
                    '(default: the joined tags)')
    ap.add_argument('--out', default=None, help='movie path (default tmp/movie/TAG_kK.mp4)')
    ap.add_argument('--view', type=_view, default=None,
                    help='camera rect X0,Y0,X1,Y1 in mm (default: the run nets\' copper)')
    ap.add_argument('--fps', type=int, default=12)
    ap.add_argument('--cell', type=_wh, default=None,
                    help='board size per world, WxH px (canvas grows to fit)')
    ap.add_argument('--size', dest='canvas', type=_wh, default=None,
                    help='canvas size WxH px (default 1760x1080)')
    ap.add_argument('--gens', type=int, default=None, help='film only the first N generations')
    ap.add_argument('--gif', action='store_true', help='also write a half-size GIF')
    ap.add_argument('--frames-dir', default=None)
    ap.add_argument('--supersample', type=int, default=2)
    ap.add_argument('--hold', type=float, default=0.65, help='seconds per descent step')
    ap.add_argument('--roll', type=float, default=1.7, help='seconds per roll-call card')
    ap.add_argument('--select', type=float, default=2.2, help='seconds per selection card')
    ap.add_argument('--verify', action='store_true',
                    help='print the copper bookkeeping behind every highlight')
    ap.add_argument('--no-encode', action='store_true', help='frames only')
    ap.add_argument('--self-test', action='store_true',
                    help='check the origin grammar and the copper diff, then exit')
    args = ap.parse_args(argv)
    if args.self_test:
        self_test()
        return 0
    self_test()                 # cheap, and it runs on EVERY invocation
    args.runs = [t for t in args.runs.split(',') if t.strip()]
    args.descents = [d for d in args.descents.split(',') if d.strip()]
    # `--runs A,B 51` names its sources by flag, so the ONE positional left is K
    if args.K is None and args.tag is not None and (args.runs or args.descents):
        try:
            args.K, args.tag = int(args.tag), None
        except ValueError:
            pass
    if args.tag and not args.runs:
        args.runs = [args.tag]          # the single-run form is one source
    if args.K is None:
        ap.error('K is required (or pass --self-test alone)')
    if not args.runs and not args.descents:
        ap.error('give a TAG, --runs, or --descents')

    stitched = len(args.runs) + len(args.descents) > 1
    base = args.name or (f'k{args.K}_day' if stitched
                         else f'{(args.runs or args.descents)[0].strip("/").split("/")[-1]}_k{args.K}')
    outdir = os.path.join(HERE, 'tmp', 'movie')
    os.makedirs(outdir, exist_ok=True)
    args.out = args.out or os.path.join(outdir, f'{base}.mp4')
    args.frames_dir = args.frames_dir or os.path.join(outdir, f'frames_{base}')
    for old in glob.glob(os.path.join(args.frames_dir, 'f*.png')):
        os.remove(old)
    n, wh = run(args)
    if n and not args.no_encode:
        encode(args.frames_dir, args.out, args.fps, n, gif=args.gif)
    return 0


if __name__ == '__main__':
    sys.exit(main())
