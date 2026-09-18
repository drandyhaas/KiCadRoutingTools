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


def descent_steps(job_dir, prefix):
    """The board chain a `replan.py` descent passed through, read off its
    `d.out` transcript; falls back to the round boards, then the final board.

    Returns [Step]; missing files are skipped, so a run whose intermediates
    were cleaned still yields at least its final board.
    """
    steps = []
    log = os.path.join(job_dir, 'd.out')
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

    @property
    def grade(self):
        """The grade to caption with.  A descent evolve.py DROPPED has no
        ledger world, but it is not ungraded -- it ships its parent's board, so
        the parent's grade is the honest number to show."""
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
    """The search's own trajectory.  x is the generation a world was born in,
    y its via count (lower is higher on the plot, because lower is better);
    edges run parent -> child in the child's operator colour.  A world with
    open nets is drawn hollow -- it is not admissible however few vias it has.
    """

    def __init__(self, led, box):
        self.led, self.box = led, box
        self.nodes = {}
        vs = []
        for w in led.by_name.values():
            g = w.get('grade')
            if g:
                vs.append(g[2])
        self.vmin, self.vmax = (min(vs), max(vs)) if vs else (0, 1)
        if self.vmax - self.vmin < 1:
            self.vmax = self.vmin + 1
        self.gmax = max([w.get('born', 0) for w in led.by_name.values()] + [1])
        x0, y0, x1, y1 = box
        per = {}
        for w in sorted(led.by_name.values(), key=lambda w: (w.get('born', 0), w['name'])):
            g = w.get('grade')
            if not g:
                continue
            b = w.get('born', 0)
            i = per.get(b, 0)
            per[b] = i + 1
            cx = x0 + 58 + (x1 - x0 - 96) * (b / max(1, self.gmax))
            cx += (i % 3 - 1) * 9                    # spread a crowded column
            frac = (g[2] - self.vmin) / (self.vmax - self.vmin)
            cy = (y0 + 22) + frac * ((y1 - 26) - (y0 + 22))
            self.nodes[w['name']] = (cx, cy, w)

    def draw(self, d, upto_gen, upto_phase, best_name=None, pop_names=()):
        x0, y0, x1, y1 = self.box
        d.rectangle([x0, y0, x1, y1], fill=PANEL, outline=PANEL_EDGE)
        f = load_font(12)
        fs = load_font(11)
        cap = ('lineage -- generation (x) vs vias (y, lower is better);'
               ' hollow = has open nets')
        # right-aligned: the interesting worlds cluster at the TOP-LEFT of this
        # plot (few vias, early), which is where a left-aligned caption sat
        d.text((x1 - 12 - d.textlength(cap, font=f), y0 + 5), cap, fill=DIM, font=f)
        # axis ticks, kept clear of the caption so neither is unreadable
        for frac in (0.0, 0.5, 1.0):
            yy = (y0 + 22) + frac * ((y1 - 26) - (y0 + 22))
            val = int(round(self.vmin + frac * (self.vmax - self.vmin)))
            d.line([x0 + 44, yy, x1 - 12, yy], fill=(34, 38, 44))
            d.text((x0 + 10, yy - 6), f'{val}v', fill=FAINT, font=fs)
        for g in range(0, self.gmax + 1):
            cx = x0 + 58 + (x1 - x0 - 96) * (g / max(1, self.gmax))
            d.text((cx - 8, y1 - 18), f'g{g}' if g else 'seed', fill=FAINT, font=fs)

        def visible(w):
            b = w.get('born', 0)
            if b < upto_gen:
                return True
            if b == upto_gen:
                return upto_phase in ('work', 'select')
            return False

        for name, (cx, cy, w) in self.nodes.items():
            if not visible(w):
                continue
            for p in w.get('parents') or []:
                pn = self.nodes.get(p)
                if not pn or not visible(pn[2]):
                    continue
                d.line([pn[0], pn[1], cx, cy], fill=kind_colour(w.get('kind')), width=1)
        for name, (cx, cy, w) in self.nodes.items():
            if not visible(w):
                continue
            g = w.get('grade')
            opens = len(g[0]) if g else 0
            col = kind_colour(w.get('kind'))
            r = 5 if name == best_name else 4
            if opens:
                d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=col, width=2)
            else:
                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)
            if name in pop_names:
                d.ellipse([cx - r - 3, cy - r - 3, cx + r + 3, cy + r + 3],
                          outline=KEPT, width=1)
            if name == best_name:
                d.ellipse([cx - r - 5, cy - r - 5, cx + r + 5, cy + r + 5],
                          outline=BEST, width=2)
                d.text((cx + 9, cy - 7), name, fill=BEST, font=fs)


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

    def __init__(self, led, cam, stage, ribbon, canvas, frames_dir, out=sys.stdout):
        self.led, self.cam, self.stage, self.ribbon = led, cam, stage, ribbon
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
        if best:
            rec = f'record  {fmt_grade(best.get("grade"))}   ({best["name"]})'
            w = d.textlength(rec, font=f_sub)
            d.text((self.W - w - 18, 46), rec, fill=BEST, font=f_sub)
            lab = f'{self.led.tag}   K={self.led.K}'
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
            if si is None or not c.steps:
                d.rectangle([bx, by, bx + bw, by + bh], fill=(18, 20, 23),
                            outline=PANEL_EDGE)
                _text_fit(d, (bx + 10, by + bh / 2 - 8), 'no board', f_cap, FAINT, bw - 20)
            else:
                st = c.steps[si]
                cell = self.cam.image(st, hot_of(c))
                if dimmed and dim < 0.999:
                    cell = cell.point(lambda p, k=dim: int(p * k))
                img.paste(cell, (bx, by))
                col = BEST if (best and c.world and c.world['name'] == best['name']) \
                    else (KEPT if c.status == 'kept' else
                          (DROPPED if dimmed else kind_colour(c.kind)))
                d.rectangle([bx - 1, by - 1, bx + bw, by + bh], outline=col,
                            width=3 if c.status or (best and c.world and
                                                    c.world['name'] == best['name']) else 1)
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
            if best and c.world and c.world['name'] == best['name']:
                head = '* ' + head
            _text_fit(d, (bx, cy), head, f_cap, tcol, bw)
            _text_fit(d, (bx, cy + 16), fmt_grade(c.grade), f_small,
                      FAINT if dimmed else DIM, bw)
            line3 = ''
            if si is not None and c.steps:
                st = c.steps[si]
                # a cell with no ledger world says WHY (the operator produced
                # nothing) rather than narrating the board it kept
                line3 = st.note or (c.detail if c.world is None else st.label)
            if not line3:
                line3 = c.detail or c.kind
            _text_fit(d, (bx, cy + 32), line3, f_small,
                      FAINT if dimmed else kind_colour(c.kind), bw)

        # ribbon + legend
        self.ribbon.draw(d, gen, phase, best_name=best['name'] if best else None,
                         pop_names=pop_names)
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
def run(led, args, out=sys.stdout):
    # -- the reference board fixes the camera and the substrate
    ref_world = (led.pop0 or [w for g in led.gens for w in g.get('new', [])])[0]
    ref_board = ref_world['stem'] + '.kicad_pcb'
    if not os.path.exists(ref_board):
        raise SystemExit(f'evolve_movie: reference board missing: {ref_board}')
    ref_pcb = _pcb(ref_board)
    # the run's nets, unioned over EVERY world's sidecar: a seed imported from
    # a narrower recorded run names fewer nets than the bus has, and reading
    # one world's sidecar would then aim the camera by an accident of which
    # world the ledger happens to list first
    names = set()
    for w in led.by_name.values():
        names |= run_net_names(w['stem'])
    view = args.view or derive_view(ref_pcb, names)
    print(f'evolve_movie: {led.tag} K{led.K}; {len(led.gens)} generation(s) recorded, '
          f'{len(led.by_name)} world(s); {len(names)} run net(s); '
          f'view {", ".join(f"{v:.2f}" for v in view)}', file=out)
    if led._rerooted:
        print(f'  note: {led._rerooted} recorded stem(s) pointed at a path that no longer '
              f'exists and were re-rooted onto {os.path.relpath(led.root, HERE)}/', file=out)
    for d_ in led.unrecorded_gen_dirs():
        print(f'  note: {d_}/ exists on disk but the ledger has not recorded it '
              f'(generation in flight) -- not filmed', file=out)

    # -- build every generation's cells first: the layout must be stable
    gens = [g['gen'] for g in led.gens]
    all_cells = {}
    for gen in gens:
        cells = build_jobs(led, gen)
        if args.verify:
            print(f'\n-- verify generation {gen} --------------------------------', file=out)
        resolve_diffs(cells, verify=args.verify, out=out)
        all_cells[gen] = cells
    seed_cells = []
    for i, w in enumerate(led.pop0):
        b = w['stem'] + '.kicad_pcb'
        seed_cells.append(Cell(0, i, 'seed', w['name'], world=w,
                               steps=[Step(b, 'seed')] if os.path.exists(b) else [],
                               detail=parse_origin(w.get('origin'))[2]))
    resolve_diffs(seed_cells)
    all_cells[0] = seed_cells

    # -- ONE stage geometry for the whole film, sized for the busiest
    # generation, so no cell moves or resizes between scenes
    widest = max((sum(1 for c in cs if c.row == r) for cs in all_cells.values()
                  for r in {c.row for c in cs}), default=1)
    rowset = sorted({c.row for cs in all_cells.values() for c in cs}) or [0]
    row_ix = {r: i for i, r in enumerate(rowset)}
    max_w, max_h = args.canvas or (1760, 1080)
    TITLE_H, RIBBON_H, LEGEND_H, MIN_W = 78, 172, 34, 1220
    stage = Stage(len(rowset), widest, TITLE_H,
                  max_h - TITLE_H - RIBBON_H - LEGEND_H, view, max_w,
                  cell=args.cell)
    W = max(MIN_W, stage.width)
    H = TITLE_H + stage.height + RIBBON_H + LEGEND_H
    W += W & 1                  # even dimensions: yuv420p requires them
    H += H & 1
    stage.centre_in(W)
    for cs in all_cells.values():
        for c in cs:
            c.rect = stage.rect(row_ix[c.row], c.col)
    cam = Camera(ref_pcb, view, stage.cell[0], stage.cell[1], args.supersample)
    ribbon = Ribbon(led, (0, H - RIBBON_H - LEGEND_H, W, H - LEGEND_H))
    film = Film(led, cam, stage, ribbon, (W, H), args.frames_dir, out=out)
    print(f'  canvas {W}x{H}, cell {stage.cell[0]}x{stage.cell[1]}, '
          f'{len(rowset)} row(s) x {widest} column(s)', file=out)

    n_roll = max(1, int(args.roll * args.fps))
    n_step = max(1, int(args.hold * args.fps))
    n_hot = max(1, int(n_step * 0.6))
    n_sel = max(1, int(args.select * args.fps))
    best = None

    # ---- scene: the seeds
    for w in led.pop0:
        if better(w.get('grade'), best.get('grade') if best else None):
            best = w
    pop_names = {w['name'] for w in led.pop0}
    for c in seed_cells:
        c.status = 'kept'
    sub = ' | '.join(f'{w["name"]} {fmt_grade(w["grade"])}' for w in led.pop0)
    img = film.compose(seed_cells, lambda c: 0 if c.steps else None,
                       lambda c: False, 0, 'select', 'seed population',
                       sub, best, pop_names,
                       note='recorded runs imported as worlds')
    film.emit(img, n_roll)

    # ---- per generation
    for gen in gens:
        cells = all_cells[gen]
        rec = led.gen(gen)
        entering = led.entering_pop(gen)
        pop_names = {w['name'] for w in entering}
        for c in cells:
            c.status = ''
        sub = ' | '.join(f'{w["name"]} {fmt_grade(w["grade"])}' for w in entering)

        # roll-call: the population that enters, offspring slots still empty
        img = film.compose([c for c in cells if c.row == 0],
                           lambda c: 0 if c.steps else None, lambda c: False,
                           gen, 'roll', f'generation {gen}',
                           f'population entering: {sub}', best, pop_names,
                           note='each world descends; jumps and crossovers are created')
        film.emit(img, n_roll)

        # work: every cell advances through its own steps AT THE SAME TIME
        T = max((len(c.steps) for c in cells), default=1)
        for t in range(T):
            def step_of(c, t=t):
                return min(t, len(c.steps) - 1) if c.steps else None

            def age(c, t=t):
                return t - (len(c.steps) - 1) if c.steps else 0
            for f in range(n_step):
                def hot_of(c, f=f, t=t):
                    return age(c) <= 0 and f < n_hot and t > 0
                alpha = 1.0 - (f / max(1, n_hot)) if f < n_hot else 0.0
                # the hot image fades into the cool one over the beat
                base = film.compose(cells, step_of, lambda c: False, gen, 'work',
                                    f'generation {gen}', f'population: {sub}',
                                    best, pop_names,
                                    note=f'step {t + 1}/{T} -- bright = copper added, '
                                         f'pink = copper removed')
                if alpha > 0.01 and any(hot_of(c) for c in cells):
                    hot = film.compose(cells, step_of, hot_of, gen, 'work',
                                       f'generation {gen}', f'population: {sub}',
                                       best, pop_names,
                                       note=f'step {t + 1}/{T} -- bright = copper added, '
                                            f'pink = copper removed')
                    base = Image.blend(base, hot, alpha)
                film.emit(base)

        # selection
        kept = {w['name'] for w in (rec.get('pop') or [])}
        for c in cells:
            c.status = 'kept' if (c.world and c.world['name'] in kept) else 'dropped'
        newbest = rec.get('best')
        if newbest and better(newbest.get('grade'), best.get('grade') if best else None):
            best = led.by_name.get(newbest['name'], newbest)
        survivors = ' | '.join(f'{w["name"]} {fmt_grade(w["grade"])}'
                               for w in (rec.get('pop') or []))
        # the dropped worlds FADE rather than blink out, over the first part of
        # the card, so it is visible which copies the generation discarded
        for i in range(n_sel):
            k = min(1.0, i / max(1.0, n_sel * 0.45))
            film.emit(film.compose(
                cells, lambda c: len(c.steps) - 1 if c.steps else None,
                lambda c: False, gen, 'select', f'generation {gen}',
                f'selection keeps: {survivors}', best, kept,
                note='elitist on (open, drc, vias), deduplicated by copper',
                dim=1.0 - 0.62 * k))
        cam.clear()

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
    ap.add_argument('tag', nargs='?', help='run tag: reads tmp/TAG/evolve_kK.json')
    ap.add_argument('K', nargs='?', type=int, help='the K the run used')
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
    if not args.tag or args.K is None:
        ap.error('TAG and K are required (or pass --self-test alone)')

    lpath = os.path.join(HERE, 'tmp', args.tag, f'evolve_k{args.K}.json')
    if not os.path.exists(lpath):
        raise SystemExit(f'evolve_movie: no ledger at {lpath}'
                         + (' (the run has not written one yet)'
                            if os.path.isdir(os.path.dirname(lpath)) else ''))
    led = Ledger(lpath, max_gens=args.gens)
    outdir = os.path.join(HERE, 'tmp', 'movie')
    os.makedirs(outdir, exist_ok=True)
    args.out = args.out or os.path.join(outdir, f'{args.tag}_k{args.K}.mp4')
    args.frames_dir = args.frames_dir or os.path.join(outdir, f'frames_{args.tag}_k{args.K}')
    for old in glob.glob(os.path.join(args.frames_dir, 'f*.png')):
        os.remove(old)
    n, wh = run(led, args)
    if n and not args.no_encode:
        encode(args.frames_dir, args.out, args.fps, n, gif=args.gif)
    return 0


if __name__ == '__main__':
    sys.exit(main())
