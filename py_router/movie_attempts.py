#!/usr/bin/env python3
"""The search behind a film, read off disk (#946, #1021).

Routing and placement are not one shot. `place_route_loop` tries a round, routes
it, keeps it or throws it away, widens the nudge cap and tries again; `converge`
does the same thing one lap at a time with a ledger; and `awx/evolve_movie`
already films exactly this for the bus-routing work -- a population of worlds, a
node per world, and a gold staircase tracking the best admissible result over
time. This module reads those records into one `Track` of `Attempt`s, and
shares the record rule (`best_so_far`) with `awx`.

It drew the ATTEMPTS BAND too, a verdict graph under the board. That band is
retired with every film layout but stage3d, whose one band -- the benchmark
band (`movie_benchmark`) -- folds the verdict and the placement laps into one
curve; what stays here is the readers, the shared helpers `movie_benchmark`
and `movie_placement` import (`_blocking_value`, `_row_t`,
`ledger_time_domain`), and `band_height`, which sizes the stage3d band.

**THE Y-AXIS IS THE RUN'S OWN ACCEPT RULE, CHOSEN ONCE.** `place_route_loop`'s
ranker is

    def better(a, b):
        \"\"\"Is metrics a better than b? Failures first, then iterations.\"\"\"

so `failures` is the axis, not `vias` -- `vias` is the literal analogue of what
`evolve_movie` plots and it is in every sidecar, but the loop annotates it
report-only. A y-axis that is not the accept rule draws a staircase pointing one
way while the accepted/rejected rings on the same frame point the other: a
verdict drawn beside numbers that refute it. On the `awx` side `vias` IS
correct, because it is the last term of that ranker's key. For a converge ledger
it is `score.blocking`, which `_score_key` and the plateau verdict already rank
on. When a run used `--accept-cmd`, `accept_score` is the accept rule and
`failures` is not.

So the metric is decided **once over the whole attempt list** and named in the
axis label. **NEVER MIXED**: a film whose axis changes meaning halfway is worse
than no film.

**ON A PLACEMENT RUN THE AXIS IS STILL THE ROUTED RESULT**, and that surprises
people, so it is worth saying plainly. `place_route_loop` is a place-AND-route
loop: a round moves parts, routes the board, and is kept or thrown away on
`better()`, whose leading term is `failures` -- copper, from the route summary
(`failed_single + open_single + the multipoint pad deficit`). So the y-axis of
a placement film reads "how much is still unrouted after moving the parts".
That IS the run's own accept rule; a placement score would not be.

The sidecar also carries `ratsnest_crossings`, `ratsnest_hpwl` and
`ratsnest_length`, which are placement-only proxies, and this deliberately does
NOT plot them. `_ratsnest_screen` uses them to decide whether a candidate is
worth paying a routing run for -- it is a SCREEN, not the judge, and the loop's
own comment marks them report-only. Plotting a screen where the verdict belongs
is the "staircase pointing one way beside rings pointing the other" failure in
its exact form, and CLAUDE.md has the measurement behind it: on a run-16 board
crossings were *anti-correlated* with correctness.

A placement tool that does not route -- `place_optimize`, `place_seed`,
`place_portfolio` -- writes no `loop_round*.json` at all, so there is no band
and nothing is invented. That is the degradation arm, not a gap.

**A SCREENED ROUND STILL GETS A NODE.** Its sidecar has `board: None`,
`routed: None`, `metrics: {}` -- written that way precisely so a consumer
"cannot tell 'screened' from 'crashed'". It is read as an UNGRADED attempt --
present and countable, and visibly not a score, which is the whole point.
Likewise a converge row with `score: null` is **not**
`score 0` and is not dropped: `board_score` returns `blocking = None` when a
component could not run, and dropping such rows is a measured bug -- the
placement half once read *plateau* from two accepted laps while five accepted
laps were invisible for having a null score.

**NOTHING IS EVER SYNTHESISED.** `movie_camera.synth_rounds` forbids exactly
this extension in its own docstring, and inferring a search that did not happen
is the worst thing a reader of the search could do. No attempts on disk means
no track.
"""
from __future__ import annotations

import glob
import json
import os
import re
import sys
from typing import List, NamedTuple, Optional, Sequence, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

#: Band height as a fraction of the frame height, and its floor. The floor
#: matters: the existing movie tests render at `size=200` and `size=160`, where
#: 12% of the height is ~20 px and an unfloored band would be a smear.
BAND_FRAC = 0.16
BAND_MIN_PX = 64

#: And a CEILING, because the floor above has no opinion about the frame it is
#: floored in. Measured on a long thin board (560x86): a band took **74% of
#: the frame**, and 52% at 124 px -- a time series about the run dwarfing the
#: film it annotates. Above this share the frame is simply too short to carry
#: a band, and `band_height` returns 0. Refusing is the honest arm: a 64 px
#: band on an 86 px frame is not a smaller band, it is a different picture.
BAND_MAX_FRAC = 0.34

class Attempt(NamedTuple):
    """One thing that was tried.

    `score` is the leading term of the run's OWN accept rule, or `None` when
    the attempt produced no gradeable result -- a screened round, a converge
    row whose `board_score` could not run. `None` is not zero and is never
    coerced to one.

    `admissible` is "this result had nothing blocking left", which is what
    makes a node solid rather than hollow. It is False for an ungraded attempt,
    because an attempt that could not be graded is not known to be admissible.
    """
    index: int
    label: str
    kind: str
    parent: Optional[int]
    accepted: bool
    screened: bool
    score: Optional[float]
    admissible: bool
    board: Optional[str]
    #: RUN TIME, epoch seconds, when the adapter's record carries it (the
    #: converge ledger's `t`). The band draws x as run time only when EVERY
    #: attempt has it (#1042).
    t: Optional[float] = None


class Track(NamedTuple):
    """An attempt list plus the one thing the axis means.

    `gate_record` is the distinction that decides whether the staircase says
    anything at all, and it is not cosmetic:

    * when the axis IS the blocking term (`failures`, `blocking`), being low IS
      being admissible, so gating the record on admissibility as well would
      collapse it -- measured on a nine-round fixture the whole staircase
      became a single point at the one round that reached zero;
    * when the axis is a QUALITY term (`vias`, an `--accept-cmd` scalar),
      admissibility is an independent condition and MUST gate it. That is
      `evolve_movie.Ribbon`'s own rule: "A world with open nets is drawn
      hollow: it is not admissible however few vias it has."

    So the gate travels with the metric, decided by the adapter that chose it,
    rather than being a constant in the drawer.
    """
    attempts: Tuple[Attempt, ...]
    metric: str                 # the axis label, e.g. 'failures (lower better)'
    source: str                 # 'loop' | 'converge' | 'evolve'
    note: str                   # 'N of M attempts dropped'
    gate_record: bool = False   # see above
    #: `(t0, t1)`, the RUN's time span -- every ledger row's `t`, placement
    #: laps included, so the placement panels (#1042) share this x domain
    #: with the band even though their laps are not plotted on it.
    x_domain: Optional[Tuple[float, float]] = None


def _note(rows: Sequence[Attempt]) -> str:
    dropped = sum(1 for a in rows if not a.accepted)
    ungraded = sum(1 for a in rows if a.score is None)
    out = '%d of %d attempts dropped' % (dropped, len(rows))
    if ungraded:
        out += '; %d ungraded' % ungraded
    return out


# ---------------------------------------------------------------------------
# adapters -- one record type, three producers
# ---------------------------------------------------------------------------
def attempts_from_loop_dir(work_dir: str) -> Optional[Track]:
    """`loop_round{N}.json` sidecars -> a Track.

    Reads EVERY round, not the accepted spine: `load_round_sidecars`' own
    docstring says the dropped ones are the search. Uses that loader when it
    imports, so this module and the camera cannot disagree about what a sidecar
    is; falls back to the same glob when it does not.

    The metric is chosen once. A run with `--accept-cmd` ranks on
    `metrics['accept_score']` and NOT on `failures`, so a single attempt
    carrying that key switches the whole axis -- and the label says so.
    """
    docs = []
    try:
        import movie_camera
        docs = movie_camera.load_round_sidecars(work_dir)
    except Exception:                                          # noqa: BLE001
        for p in sorted(glob.glob(os.path.join(work_dir, 'loop_round*.json'))):
            try:
                with open(p, encoding='utf-8') as f:
                    doc = json.load(f)
            except Exception:                                  # noqa: BLE001
                continue
            if doc.get('schema') == 1 and 'round' in doc:
                docs.append(doc)
        docs.sort(key=lambda d: d['round'])
    if not docs:
        return None
    use_accept = any('accept_score' in (d.get('metrics') or {}) for d in docs)
    key = 'accept_score' if use_accept else 'failures'
    label = ('accept score (lower better)' if use_accept
             else 'failures (lower better)')
    # `parent` names a BOARD, and the band's x-axis is the round number, so the
    # edge is resolved back to the round that produced that board. A parent
    # naming a board no round wrote (round 0's input) has no edge, which is
    # correct -- it is the root.
    by_board = {}
    for d in docs:
        for k in ('routed', 'board'):
            if d.get(k):
                by_board.setdefault(os.path.basename(d[k]), d['round'])
    rows = []
    for d in docs:
        met = d.get('metrics') or {}
        sc = met.get(key)
        rows.append(Attempt(
            index=int(d['round']),
            label='round %d' % int(d['round']),
            kind='round',
            parent=by_board.get(os.path.basename(d.get('parent') or '')),
            accepted=bool(d.get('accepted')),
            screened=bool(d.get('screened')),
            score=None if sc is None else float(sc),
            # `failures` is the loop's own blocking term; 0 means nothing left
            # to fix. With --accept-cmd the accept score is an arbitrary scalar
            # and admissibility is not defined by it, so only `failures`
            # answers this -- read it directly whichever key drives the axis.
            admissible=(met.get('failures') == 0),
            board=d.get('routed') or d.get('board')))
    # gate_record: only when the axis is an arbitrary --accept-cmd scalar
    # rather than the loop's own blocking term.
    return Track(tuple(rows), label, 'loop', _note(rows),
                 gate_record=use_accept)


def _is_placement_row(e) -> bool:
    return str(e.get('kind') or '') == 'placement'


#: `blocking` as a count the axis can plot, or None (drawn ungraded): THE
#: rule `converge` ranks and refuses by, imported rather than mirrored
#: (#1088). A count is an int or a finite float >= 0, within the float range,
#: and never a bool -- a per-term dict raised `float(b)` inside
#: `make_film.main()`, and `false` plotted at 0.0 as admissible (#1077).
from ledger_score import blocking_value as _blocking_value  # noqa: E402


def _graded(e) -> bool:
    sc = e.get('score') if isinstance(e.get('score'), dict) else None
    return bool(sc) and _blocking_value(sc.get('blocking')) is not None


def attempts_from_converge_ledger(path: str,
                                  drop_placement=None) -> Optional[Track]:
    """A converge JSONL ledger -> a Track, ranked on `score.blocking`.

    `_score_key`'s own comment is the rule this follows: "`blocking == None` is
    NOT zero -- it means a component that was asked for could not answer". So a
    null-scored row keeps its node and is drawn ungraded, never plotted at the
    bottom of the axis as though it were perfect. So does a row whose
    `blocking` is not a count (`_blocking_value`, #1077), and the note counts
    those.

    `drop_placement` decides whether `kind == placement` laps are on this
    axis. None (the default) drops them only when the ledger ALSO holds a
    graded routing row: then the axis is the routed verdict, and a copper-free
    placement lap's `blocking` (every net unrouted) says nothing on it -- those
    laps belong to the placement panels (#1042). A PLACEMENT-ONLY ledger (the
    placement skill's `make_film --from-ledger` film) has no routed verdict to
    protect, and its laps are the whole search, so they stay on the axis.
    `discover` passes True when loop rounds will supply the routing half.
    """
    rows_in = []
    try:
        with open(path, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    doc = json.loads(line)
                except ValueError:
                    continue      # a torn last line never loses the rest
                # A LINE IS ONLY A ROW IF IT IS AN OBJECT. A scalar or a list
                # parses fine and then raises `AttributeError` on `.get` --
                # inside `make_film.main()`, which calls this adapter
                # UNGUARDED, so one stray line took the film down.
                if isinstance(doc, dict):
                    rows_in.append(doc)
    except OSError:
        return None
    if not rows_in:
        return None
    def _row_index(e, i):
        # `record` writes an int. Anything else (null, a string) fell into an
        # unguarded `int(...)` and took the film down (#1077); the row's
        # position is the honest fallback, as it already was for an absent key.
        it = e.get('iteration', i)
        return it if isinstance(it, int) and not isinstance(it, bool) else i

    by_sha = {}
    for i, e in enumerate(rows_in):
        if e.get('result_sha') and isinstance(e['result_sha'], str):
            by_sha.setdefault(e['result_sha'], _row_index(e, i))
    if drop_placement is None:
        drop_placement = any(not _is_placement_row(e) and _graded(e)
                             for e in rows_in)
    rows = []
    # LINEAGE. A row's parent is the row that produced its `parent_sha`. A row
    # that names none (or names a board no row produced) is drawn from the
    # LAST ACCEPTED row before it -- the loop's own rule for what a lap starts
    # from -- and COUNTED, because under parallel lineages that guess can be
    # wrong: `record` does not yet take a parent explicitly (#1034). The first
    # row is the root either way.
    fallback = 0
    last_acc = None
    n_place = 0
    n_bad = 0
    for i, e in enumerate(rows_in):
        sc = e.get('score') if isinstance(e.get('score'), dict) else None
        b = sc.get('blocking') if sc else None
        not_a_count = b is not None and _blocking_value(b) is None
        b = _blocking_value(b)
        idx = _row_index(e, i)
        if drop_placement and _is_placement_row(e):
            # OFF THE VERDICT AXIS (#1042) when there IS a routed verdict. A
            # placement lap scores the COPPER-FREE board, where `blocking` is
            # every net unrouted: run 32's accepted placement rows read
            # 267 -> 251 -> 239 ... on this axis while the laps moved
            # floorplan errors 41 -> 11. They belong to the placement panels
            # (`movie_placement`), in their own currency; here they are
            # counted and said, never plotted.
            n_place += 1
            if e.get('accepted'):
                last_acc = idx
            continue
        _psha = e.get('parent_sha')
        parent = by_sha.get(_psha) if isinstance(_psha, str) else None
        if parent is None and i > 0 and last_acc is not None:
            parent = last_acc
            fallback += 1
        if not_a_count:
            n_bad += 1
        rows.append(Attempt(
            index=idx,
            label=str(e.get('lever') or e.get('kind') or 'lap')[:40],
            kind=str(e.get('kind') or 'completion'),
            parent=parent,
            accepted=bool(e.get('accepted')),
            screened=False,
            score=None if b is None else float(b),
            admissible=(b == 0),
            board=e.get('result_sha'),
            t=_row_t(e)))
        if e.get('accepted'):
            last_acc = idx
    if not rows:
        return None
    note = _note(rows)
    if n_place:
        note += ('; %d placement lap(s) off this axis (copper-free, see the '
                 'placement panels)' % n_place)
    if n_bad:
        # No ';' inside the clause (see below).
        note += ('; %d with a blocking that is not a count (drawn ungraded, '
                 'converge.blocking_value)' % n_bad)
    if fallback:
        # No ';' inside the clause: the note is a '; '-separated list, and
        # `join_tracks` carries clauses over by splitting on it.
        note += ('; %d parent(s) by last-accepted (no parent_sha, until '
                 'record takes a parent explicitly, #1034)' % fallback)
    return Track(tuple(rows), 'blocking (lower better)', 'converge',
                 note, gate_record=False, x_domain=ledger_time_domain(rows_in))


def _row_t(row) -> Optional[float]:
    """A ledger row's run time (`t`, epoch seconds), or None."""
    try:
        v = row.get('t')
        return None if v is None else float(v)
    except (TypeError, ValueError, AttributeError):
        return None


def ledger_time_domain(rows) -> Optional[Tuple[float, float]]:
    """`(t0, t1)` over EVERY row that carries a time, or None when fewer
    than two do or they span nothing. The placement panels' run clock
    (#1042)."""
    ts = [t for t in (_row_t(r) for r in rows) if t is not None]
    if len(ts) < 2 or max(ts) - min(ts) <= 0:
        return None
    return (min(ts), max(ts))


def attempts_from_evolve_ledger(path: str) -> Optional[Track]:
    """`awx/tmp/TAG/evolve_kK.json` -> a Track, ranked on the via count.

    `vias` is the right axis HERE and the wrong one on the loop side, for the
    one reason that decides it: it is the last term of `awx`'s own ranker key
    `(len(open), drc != 0, vias)`, and it is not a term of
    `place_route_loop.better` at all. A world with open nets is not admissible
    however few vias it has -- which is the hollow node.

    Read directly rather than through `awx.evolve_movie.Ledger`: `awx/` is
    research-local and is not importable from `py_router/`, and this adapter
    needs four fields of a JSON document, not a film.
    """
    try:
        with open(path, encoding='utf-8') as f:
            raw = json.load(f)
    except (OSError, ValueError):
        return None
    worlds, seen = [], set()

    def take(w, born):
        if not isinstance(w, dict) or not w.get('name'):
            return
        if w['name'] in seen:
            return
        seen.add(w['name'])
        worlds.append((born, w))

    # `born` FOLLOWS `awx.Ledger._ingest`: a world is born in the generation
    # that made it NEW, and a world seen only in a `pop` list was carried over
    # rather than created, so it keeps born 0 -- `_ingest` passes `born=None`
    # for exactly those. Giving a pop-only world the generation it was carried
    # INTO put it at the wrong x, and the two readers of one ledger then
    # disagreed about the same world.
    for w in raw.get('pop0') or ():
        take(w, 0)
    for g in raw.get('gens') or ():
        for w in g.get('new') or ():
            take(w, int(g.get('gen', 0)))
    for g in raw.get('gens') or ():
        for w in g.get('pop') or ():
            take(w, 0)
    if not worlds:
        return None
    # The population a generation KEPT is its `pop`, so a world named there is
    # one the search held on to.
    kept = set()
    for g in raw.get('gens') or ():
        for w in g.get('pop') or ():
            if isinstance(w, dict) and w.get('name'):
                kept.add(w['name'])
    for w in raw.get('pop0') or ():
        if isinstance(w, dict) and w.get('name'):
            kept.add(w['name'])
    idx = {w['name']: i for i, (_b, w) in enumerate(worlds)}
    rows = []
    for i, (born, w) in enumerate(worlds):
        grade = w.get('grade')
        vias = None
        ok = False
        if isinstance(grade, (list, tuple)) and len(grade) >= 3:
            opens = grade[0]
            vias = float(grade[2])
            ok = not opens
        kind, parents = _parse_origin(w.get('origin'))
        rows.append(Attempt(
            index=int(born), label=str(w.get('name') or ''), kind=kind,
            parent=idx.get(parents[0]) if parents else None,
            accepted=w.get('name') in kept, screened=False,
            score=vias, admissible=ok, board=w.get('stem')))
    # vias is a QUALITY term, so admissibility gates the record: a world
    # with open nets is not admissible however few vias it has.
    return Track(tuple(rows), 'vias (lower better)', 'evolve',
                 _note(rows), gate_record=True)


_ORIGIN = re.compile(r'^\s*(\w+)\s*<?\s*([^;>]*)')


def _parse_origin(origin) -> Tuple[str, List[str]]:
    """`descend<s1>` / `cross<s1 x s2; ...>` -> (kind, parents).

    A deliberately small reading of `evolve_movie.parse_origin`: this module
    needs the kind and the first parent, and re-implementing the whole grammar
    here would be the second copy that #1021 exists to avoid. The full grammar
    stays in `awx/`, which is the only place that needs its detail field.
    """
    if not origin:
        return 'seed', []
    m = _ORIGIN.match(str(origin))
    if not m:
        return 'seed', []
    kind = m.group(1)
    rest = (m.group(2) or '').strip()
    parents = [p for p in re.split(r'\s+x\s+|\s+', rest) if p and p != 'x']
    return kind, parents[:2]


def discover(hint: str) -> Optional[Track]:
    """The attempts for a board or directory, or `None`.

    `None` is a real answer and the common one: a plain routing chain has no
    search behind it. Nothing is inferred from the boards themselves.
    """
    if not hint:
        return None
    d = hint if os.path.isdir(hint) else os.path.dirname(os.path.abspath(hint))
    if not d:
        return None
    loop = attempts_from_loop_dir(d)
    led, led_path = None, None
    for name in ('ledger.jsonl', 'converge.jsonl'):
        p = os.path.join(d, name)
        if os.path.isfile(p):
            # loop rounds are a routing half: then the ledger's placement laps
            # are off the joined axis whatever else the ledger holds
            led = attempts_from_converge_ledger(
                p, drop_placement=True if loop else None)
            if led:
                led_path = p
                break
    if loop and led:
        return join_tracks(led, loop, first=_which_first(d, led_path))
    return loop or led


def _which_first(d, ledger_path):
    """'ledger' or 'loop': which half of a place-and-route run STARTED
    first, read off the ledger's own `t` and the sidecars' write times."""
    t_led = None
    try:
        with open(ledger_path, encoding='utf-8') as f:
            for line in f:
                try:
                    doc = json.loads(line)
                except ValueError:
                    continue
                if isinstance(doc, dict) and doc.get('t') is not None:
                    t_led = float(doc['t'])
                    break
    except (OSError, ValueError, TypeError):
        t_led = None
    side = glob.glob(os.path.join(d, 'loop_round*.json'))
    t_loop = min((os.path.getmtime(p) for p in side), default=None)
    if t_led is None or t_loop is None:
        return 'ledger'
    return 'ledger' if t_led <= t_loop else 'loop'


def join_tracks(a: Track, b: Track, first='ledger') -> Track:
    """ONE attempts graph for a place-and-route run (#946/C4).

    A combined run leaves two records of its search: the converge ledger
    (placement laps and routing laps, `kind` telling them apart) and, when
    `place_route_loop` ran, its `loop_round*.json` sidecars. Drawn apart they
    are two films; joined, the x-axis is LAPS ACROSS BOTH HALVES -- the
    first half keeps its indices, the second is shifted past it -- every
    attempt is a point (accepted, rejected or ungraded), and the gold record
    runs through both, the way `evolve_movie.Ribbon` draws a search.

    The two axes are both a run's BLOCKING term (the ledger's
    `score.blocking`, the loop's `failures`), which is why they may share one
    y-axis; the label says it is both. A loop ranked on an `--accept-cmd`
    scalar is a QUALITY axis and is not joined -- the ledger half is drawn
    alone, and the note says the loop half was left out.
    """
    led, loop = (a, b) if a.source == 'converge' else (b, a)
    if loop.gate_record:
        return led._replace(note=led.note + '; loop rounds not joined '
                            '(ranked on --accept-cmd, not a blocking term)')
    halves = (led, loop) if first == 'ledger' else (loop, led)
    rows, base = [], 0
    for h in halves:
        if not h.attempts:
            continue
        lo = min(x.index for x in h.attempts)
        shift = base - lo
        for x in h.attempts:
            rows.append(x._replace(
                index=x.index + shift,
                parent=None if x.parent is None else x.parent + shift))
        base = max(r.index for r in rows) + 1
    # Where the halves MEET, the second half's root descends from the first
    # half's last accepted attempt: the loop starts from the board the
    # ledger's laps kept, or the other way round.
    first_n = len(halves[0].attempts)
    if first_n and len(rows) > first_n and rows[first_n].parent is None:
        acc = [r.index for r in rows[:first_n] if r.accepted]
        if acc:
            rows[first_n] = rows[first_n]._replace(parent=acc[-1])
    note = '%s + %s: %s' % (halves[0].source, halves[1].source, _note(rows))
    # ONLY the lineage clause is carried over. Taking everything after the
    # first '; ' also copied the half's own "N ungraded" clause, which the
    # joined `_note(rows)` above already states for both halves together.
    extra = [c for h in halves for c in h.note.split('; ')
             if 'last-accepted' in c]
    if extra:
        note += '; ' + '; '.join(extra)
    return Track(tuple(rows), 'blocking / failures (lower better)',
                 'converge+loop', note, gate_record=False)


# ---------------------------------------------------------------------------
# the band's height, and the record rule
# ---------------------------------------------------------------------------
def band_height(width: int, height: int) -> int:
    """The stage3d band's height (the benchmark band's, via
    `animate_route.build_boards(attempts_band=True)`), decided ONCE over the
    whole film and constant for every frame -- the invariant `save_movie`
    cannot take a violation of."""
    try:
        import frame_layout
        even = frame_layout.even
    except Exception:                                          # noqa: BLE001
        def even(v):
            return int(v) - (int(v) % 2)
    want = max(BAND_MIN_PX, even(int(height * BAND_FRAC)))
    if want > height * BAND_MAX_FRAC:
        return 0          # too short to carry one: no band
    return want


def best_so_far(rows: Sequence[Attempt], *, require_accepted=True,
                require_admissible=False):
    """`[(index, best), ...]` -- the running record, ONE ENTRY PER DISTINCT
    INDEX, holding the best eligible score among every attempt at or before it.

    **Per INDEX, not per row, and the phase-11 verifier measured why.** The
    first version emitted one entry per attempt, which is the same thing on the
    routing side (each round has its own index) and is NOT the same thing on
    the `awx` side, where a whole generation shares one `born`. Loading both
    `Ribbon` classes side by side on one four-world registry: the original gave
    `[(0, 98), (1, 91)]` and the per-row version gave
    `[(0, 98), (1, 96), (1, 93), (1, 91)]` -- and `Ribbon.draw` labels each new
    record once, so the film gained **two extra gold numbers for values that
    were never the record at the end of any instant**. Four of five
    registry-shaped cases diverged. "The algorithm is shared, the policy is
    each caller's own" was false until this was per-index.

    Sampled at every instant rather than once per phase, for the reason
    `evolve_movie.Ribbon` records in its own comment: "A descent that walks
    98 -> 96 -> 95 set two records inside one chapter, and a per-chapter sample
    keeps only the last of them."

    **The two predicates are arguments, not constants, because the two films
    genuinely disagree and both are right.**

    * `require_accepted` -- on this side a rejected round cannot set a record:
      the loop rejects exactly what `better()` says is not better, so an
      accepted round IS the running best and the staircase is the loop's own
      `best` read back. That is also the literal answer to what the band is
      for: attempts, re-attempts, and taking the best ones.
    * `require_admissible` -- on the `awx` side the axis is `vias`, a QUALITY
      term, and "a world with open nets is not admissible however few vias it
      has". When the axis IS the blocking term (`failures`, `blocking`) this
      must be OFF, or the staircase collapses: measured on a nine-round
      fixture it became a single point at the one round that reached zero.

    `awx/evolve_movie.Ribbon` calls this with `require_accepted=False`, so its
    record line is what it always was -- which the test now checks against a
    re-implementation of the ORIGINAL's own per-instant loop, not against a
    third algorithm.

    A non-finite score (a `-Infinity` that reached a sidecar) is ignored rather
    than allowed to win every comparison forever.
    """
    def ok(a):
        return (a.score is not None
                and a.score == a.score           # not NaN
                and a.score not in (float('inf'), float('-inf'))
                and (a.accepted or not require_accepted)
                and (a.admissible or not require_admissible))

    srt = sorted(rows, key=lambda a: a.index)
    out, best, k = [], None, 0
    for i in sorted({a.index for a in rows}):
        while k < len(srt) and srt[k].index <= i:
            a = srt[k]
            k += 1
            if ok(a) and (best is None or a.score < best):
                best = a.score
        if best is not None:
            out.append((i, best))
    return out
