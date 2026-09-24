#!/usr/bin/env python3
"""The attempts band: every round a point, the record a step-line (#946, #1021).

Routing and placement are not one shot. `place_route_loop` tries a round, routes
it, keeps it or throws it away, widens the nudge cap and tries again; `converge`
does the same thing one lap at a time with a ledger; and `awx/evolve_movie`
already films exactly this for the bus-routing work -- a population of worlds, a
node per world, and a gold staircase tracking the best admissible result over
time. **That ribbon is the thing this module shares.** It was written once for
`awx/`, and the main side has the same data and throws it away.

Because it does throw it away, and the throwing-away is documented:
`make_movie.placement_chain` skips every non-accepted round ("rejected/screened
rounds are on disk, not the story"), while two docstrings in `movie_camera` say
the opposite about the same files --

    "`make_film` wants the dropped ones precisely because they are the search"
    "A caller that wants to show rejected attempts should say so by passing them
     and labelling them, not by having them inferred here."

-- and `write_round_sidecar` carries a `parent` field that exists *only* because
a rejected round sits between two accepted ones in both name and mtime order.
The search is on disk in full. Nothing drew it.

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
"cannot tell 'screened' from 'crashed'". It draws as a tick on the axis rail --
present and countable, and visibly not a score, which is the whole point.
Likewise a converge row with `score: null` is **not**
`score 0` and is not dropped: `board_score` returns `blocking = None` when a
component could not run, and dropping such rows is a measured bug -- the
placement half once read *plateau* from two accepted laps while five accepted
laps were invisible for having a null score.

**NOTHING IS EVER SYNTHESISED.** `movie_camera.synth_rounds` forbids exactly
this extension in its own docstring, and inferring a search that did not happen
is the worst thing this panel could do. No attempts on disk means no band.

**DEGRADATION IS A FIRST-CLASS CASE AND IT REFUSES BY NAME.** A plain routing
chain has exactly one attempt, and a scatter of one point under a flat staircase
is noise. `attach` then returns the frame list **completely untouched** -- the
same list object, the same `Image` objects -- with a report that says why. That
is `compose_two_panel`'s stated degradation contract, and its rule that no OFF
state may read like success. The middle case -- two or more rounds, all accepted
-- **does** draw: it is a real monotone staircase with no forks, and the note
says `0 of N attempts dropped` rather than letting the panel silently look like
a line chart.

**WHY A BAND AND NOT A FOURTH CONTENT OF THE LOWER BOX** (#1020). The box's
contents are each a statement about *this frame's board*; the attempts graph is
the only element whose subject is the whole run. Making it a phase would put it
on screen exactly when it is least needed (the bookends) and off when it is most
needed (during an attempt). `evolve_movie` draws its ribbon in every frame of
every scene, unconditionally, and that is right. A band also composes: it is a
time series, so it wants width and little height, and it adds a constant to
every frame exactly as `cmd_timing.add_clock_band` does -- which is the
precedent for growing a legacy frame, too.

Composition order is **board -> attempts -> clock -> iso**, so the band sits
adjacent to the board it annotates and the iso panel still stacks last.

**ONE ORDERING CONSTRAINT, AND IT IS LOAD-BEARING.** `make_film.build_film`
must call `attach()` **before** its badge loop. `_badge` draws a border on the
frame it is given; attach the band afterwards and the border encloses only the
board, which is precisely the trap `movie_panels.py:40-44` documents.
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
#: floored in. Measured on a long thin board (`legacy`, 560x86): the band took
#: **74% of the frame**, and 52% at 124 px -- a time series about the run
#: dwarfing the film it annotates. Above this share the frame is simply too
#: short to carry a band, and `band_height` returns 0 so `attach` declines and
#: says why. Refusing is the honest arm: a 64 px band on an 86 px frame is not
#: a smaller band, it is a different picture.
BAND_MAX_FRAC = 0.34

#: The band's Y axis (#946 review): 'broken' (the working range gets the plot,
#: outliers a thin log strip under a break mark), or the two it replaced,
#: 'symlog' and 'linear', kept so a test can show it tells them apart.
AXIS_MODE = 'broken'
#: The working range is every graded attempt up to this percentile.
WORK_PCTL = 0.90
#: The axis breaks only when the worst attempt is this many times the working
#: range's top; otherwise the whole range is one linear scale.
BREAK_RATIO = 2.0
#: The outlier strip's share of the plot height.
STRIP_FRAC = 0.18

#: An attempt whose `kind` is not one of these draws in `op_seed`'s grey. The
#: names are the vocabulary the two producers already use: `place_route_loop`
#: rounds are descents, `converge` rows carry a `kind`, and `evolve` carries an
#: origin grammar.
KIND_ROLE = {'seed': 'op_seed', 'descend': 'op_descend', 'jump': 'op_jump',
             'cross': 'op_cross', 'completion': 'op_descend',
             'placement': 'op_jump', 'systemic': 'op_cross',
             'round': 'op_descend'}


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


def attempts_from_converge_ledger(path: str) -> Optional[Track]:
    """A converge JSONL ledger -> a Track, ranked on `score.blocking`.

    `_score_key`'s own comment is the rule this follows: "`blocking == None` is
    NOT zero -- it means a component that was asked for could not answer". So a
    null-scored row keeps its node and is drawn ungraded, never plotted at the
    bottom of the axis as though it were perfect.
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
    by_sha = {}
    for i, e in enumerate(rows_in):
        if e.get('result_sha'):
            by_sha.setdefault(e['result_sha'], e.get('iteration', i))
    rows = []
    # LINEAGE. A row's parent is the row that produced its `parent_sha`. A row
    # that names none (or names a board no row produced) is drawn from the
    # LAST ACCEPTED row before it -- the loop's own rule for what a lap starts
    # from -- and COUNTED, because under parallel lineages that guess can be
    # wrong: `record` does not yet take a parent explicitly (#1034). The first
    # row is the root either way.
    fallback = 0
    last_acc = None
    for i, e in enumerate(rows_in):
        sc = e.get('score') if isinstance(e.get('score'), dict) else None
        b = sc.get('blocking') if sc else None
        idx = int(e.get('iteration', i))
        parent = by_sha.get(e.get('parent_sha'))
        if parent is None and i > 0 and last_acc is not None:
            parent = last_acc
            fallback += 1
        rows.append(Attempt(
            index=idx,
            label=str(e.get('lever') or e.get('kind') or 'lap')[:40],
            kind=str(e.get('kind') or 'completion'),
            parent=parent,
            accepted=bool(e.get('accepted')),
            screened=False,
            score=None if b is None else float(b),
            admissible=(b == 0),
            board=e.get('result_sha')))
        if e.get('accepted'):
            last_acc = idx
    note = _note(rows)
    if fallback:
        # No ';' inside the clause: the note is a '; '-separated list, and
        # `join_tracks` carries clauses over by splitting on it.
        note += ('; %d parent(s) by last-accepted (no parent_sha, until '
                 'record takes a parent explicitly, #1034)' % fallback)
    return Track(tuple(rows), 'blocking (lower better)', 'converge',
                 note, gate_record=False)


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
            led = attempts_from_converge_ledger(p)
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
# the graph
# ---------------------------------------------------------------------------
def band_height(width: int, height: int) -> int:
    """The band's height, decided ONCE over the whole film and constant for
    every frame -- the invariant `save_movie` cannot take a violation of."""
    try:
        import frame_layout
        even = frame_layout.even
    except Exception:                                          # noqa: BLE001
        def even(v):
            return int(v) - (int(v) % 2)
    want = max(BAND_MIN_PX, even(int(height * BAND_FRAC)))
    if want > height * BAND_MAX_FRAC:
        return 0          # too short to carry one; `attach` declines and says so
    return want


def _plot(box, cap_h=14, label_h=12):
    """The plotting rectangle inside the band.

    `cap_h` is the caption's own line, and `label_h` is the headroom the record
    labels need: they are drawn ABOVE their step, so a plot that starts at the
    caption puts the first record value on top of the axis title. Reserving it
    here rather than clamping each label keeps the y-scale honest -- a clamped
    label sits at a height that is not its value.
    """
    return (box.x + 46, box.y + cap_h + label_h,
            box.x + box.w - 12, box.y + box.h - 12)


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


def draw_track(d, box, track: Optional[Track], *, upto=None, theme=None,
               debug=None):
    """The ribbon: x is when an attempt was born, y is the accept rule's
    leading term with LOWER HIGHER on screen.

    A node is hollow while the attempt still has something blocking and filled
    once it is admissible; its colour is the operator that made it; a kept
    attempt is ringed; and a gold staircase tracks the best admissible result
    over time, labelling each new record once.

    `upto` is the visibility horizon, so the graph grows with the film.

    **Returns True when it drew and False when it declined**, because a caller
    cannot be asked to tell those apart by looking: the body is wrapped in a
    bare `except`, so a decline and a crash and a success all return the same
    `None`. `attach` reports the difference in its status line.

    Never raises: a band is an artifact, and taking a routing run down for a
    font metric is the trade this repo refuses (`movie_panels._finite`).

    **A reserved band is never left blank** (#1036 review): when the
    `AXIS_MODE` axis fails to draw, the plain linear axis is drawn instead
    (the box is repainted first, so nothing half-drawn survives).
    """
    ok = _draw_track(d, box, track, upto=upto, theme=theme, debug=debug,
                     _mode=AXIS_MODE)
    if not ok and AXIS_MODE != 'linear':
        ok = _draw_track(d, box, track, upto=upto, theme=theme, debug=debug,
                         _mode='linear')
    return ok


def _draw_track(d, box, track, *, upto=None, theme=None, debug=None,
                _mode='broken'):
    """`draw_track`'s body for one axis mode; False when it could not draw."""
    if box is None or box.h <= 0 or track is None or not track.attempts:
        return False
    try:
        import render_theme
        from route_render import load_font
        # RESOLVED, not assumed to be a Theme: `make_movie(theme=...)` carries
        # the NAME the CLI was given and hands it straight on, so a bare
        # `theme or DARK` would put a `str` here and every `.rgb()` would raise
        # into the swallow below -- a blank band that looks like "no attempts".
        th = render_theme.theme(theme, strict=False)
        rows = track.attempts
        d.rectangle([box.x, box.y, box.x + box.w - 1, box.y + box.h - 1],
                    fill=th.rgb('chrome_panel'),
                    outline=th.rgb('chrome_panel_edge'))
        f = load_font(max(9, min(15, int(box.h * 0.16))))
        fs = load_font(max(8, min(13, int(box.h * 0.13))))
        px0, py0, px1, py1 = _plot(box, f.size + 6, fs.size + 4)
        if px1 <= px0 or py1 <= py0:
            return False
        graded = [a.score for a in rows if a.score is not None
                  and a.score == a.score
                  and a.score not in (float('inf'), float('-inf'))]
        vmin = min(graded) if graded else 0.0
        vmax = max(graded) if graded else 1.0
        if vmax - vmin < 1e-9:
            vmax = vmin + 1.0
        # THE Y AXIS (#946 review). A search spends most of its laps in a
        # narrow WORKING RANGE and a few attempts far outside it: run 32 opens
        # at blocking 12703 (the unplaced pile) and spends ~200 laps between
        # 19 and 43. A linear axis puts all of those laps on one pixel row; the
        # symlog axis this used first still gave them the top ~7% of the band,
        # so the record's drops 41 -> 38 -> 33 -> 32 -> 30 were invisible.
        #
        # So the axis is BROKEN, the way the owner's evolve_movie Ribbon makes
        # progress readable: the working range -- every graded attempt up to
        # the WORK_PCTL percentile, padded -- gets the main plot, and the
        # outliers above it are compressed (log) into a thin strip at the
        # bottom, under a visible break mark. `AXIS_MODE` keeps the two
        # rejected axes callable, because the test proves it can tell them
        # apart.
        import math
        srt = sorted(graded)
        hi_w = (srt[max(0, int(math.ceil(WORK_PCTL * len(srt))) - 1)]
                if srt else vmax)
        # OFFSET-BASED, so a negative score cannot break it (#1036 review):
        # the break needs a positive GAP above the working range, measured
        # against the working range's own span -- not a ratio of raw values,
        # which misfires once hi_w < 0 -- and the strip maps the distance
        # ABOVE the working range, log1p(v - w_hi), which is defined for any
        # sign. The old log10(1 + v) raised on a negative range and the
        # except below left a reserved band blank.
        _wspan = hi_w - vmin
        _gap = vmax - hi_w
        broken = (_mode == 'broken' and _wspan > 0 and _gap > 0
                  and _gap > (BREAK_RATIO - 1.0) * _wspan)
        symlog = (_mode == 'symlog' and vmin >= 0
                  and vmax > 50.0 * (vmin + 1.0))
        ph = py1 - py0
        if broken:
            _r = _wspan
            w_lo, w_hi = vmin - 0.06 * _r, hi_w + 0.10 * _r
            main_h = ph * (1.0 - STRIP_FRAC)
            s0 = py0 + main_h + max(4.0, ph * 0.05)
            _top = math.log1p(max(0.0, vmax - w_hi))

            def Y(v):
                if v <= w_hi:
                    return py0 + main_h * ((v - w_lo) / (w_hi - w_lo))
                t = math.log1p(v - w_hi) / max(1e-9, _top)
                return s0 + (py1 - s0) * t
        else:
            def _fy(v):
                return math.log10(1.0 + max(0.0, v)) if symlog else v
            fmin, fmax = _fy(vmin), _fy(vmax)

            def Y(v):
                return py0 + ph * ((_fy(v) - fmin) / (fmax - fmin))
        xs = [a.index for a in rows]
        x0v, x1v = min(xs), max(xs)
        span = max(1, x1v - x0v)
        horizon = x1v if upto is None else upto

        def X(i):
            return px0 + (px1 - px0) * ((i - x0v) / float(span))

        _tspan = (hi_w - vmin) if broken else (vmax - vmin)
        _decimals = (0 if _tspan >= 10 else 1 if _tspan >= 1
                     else 2 if _tspan >= 0.1 else 3)

        # the axis, and the one thing it means
        def _tick(v):
            # COMPACT, because the tick column is ~38 px wide: run 32's
            # 12703 was drawn as '1270' with its last digit under the plot.
            av = abs(v)
            if av >= 10000:
                return '%.0fk' % (v / 1000.0)
            if av >= 1000:
                return '%.1fk' % (v / 1000.0)
            # PRECISION FROM THE SPAN (#1036 review): rounding to integers
            # on the broken and symlog axes labelled a 0.12..0.9 axis 0/1/1.
            # The caption's "[axis broken above X]" uses this same function.
            return '%g' % round(v, _decimals)

        # EVERY label drawn in the band is registered here, ticks and caption
        # included, so a record label is placed against all of them -- not
        # only against other record labels (#1036 review: '12703' still sat
        # on the axis tick beside it).
        taken = []

        def _bbox(xy, txt, font, anchor=None):
            try:
                return d.textbbox(xy, txt, font=font, anchor=anchor)
            except Exception:                                  # noqa: BLE001
                w = d.textlength(txt, font=font)
                return (xy[0], xy[1], xy[0] + w, xy[1] + font.size)

        if broken:
            ticks = [vmin, (vmin + hi_w) / 2.0, hi_w, vmax]
        else:
            ticks = [vmin, (vmin + vmax) / 2.0, vmax]
            if symlog:
                ticks = [10.0 ** (fmin + k * (fmax - fmin)) - 1.0
                         for k in (0.0, 0.5, 1.0)]
        last_y = None
        for v in ticks:
            yy = Y(v)
            d.line([px0, yy, px1, yy], fill=th.rgb('chrome_rule'))
            if last_y is not None and abs(yy - last_y) < fs.size + 2:
                continue
            _t = _tick(v)
            d.text((box.x + 8, yy - 6), _t,
                   fill=th.rgb('chrome_text_faint'), font=fs)
            taken.append(_bbox((box.x + 8, yy - 6), _t, fs))
            last_y = yy
        if broken:
            # THE BREAK MARK: two short slashes across the axis in the gap,
            # so nobody reads the strip as a continuation of the scale.
            gy = (py0 + main_h + s0) / 2.0
            for gx in (px0, px1):
                for dy in (-3, 3):
                    d.line([gx - 5, gy + dy + 3, gx + 5, gy + dy - 3],
                           fill=th.rgb('chrome_text_dim'), width=2)
        if debug is not None:
            debug['plot'] = (px0, py0, px1, py1)
            debug['mode'] = ('broken' if broken else
                             'symlog' if symlog else 'linear')
            debug['work_hi'] = hi_w
            debug['ys'] = [(v, Y(v)) for v in graded]
        # The caption is the axis's meaning plus the disclosure, and it is
        # DROPPED rather than ellipsised or overprinted when the band is too
        # narrow to hold it beside the plot -- same rule as the layer strip's
        # count. A half-sentence about what the axis means is worse than none:
        # 'failures (lower bet...' invites the reader to guess the rest.
        metric = track.metric + (
            '  [axis broken above %s]' % _tick(hi_w) if broken
            else '  [log scale]' if symlog else '')
        cap = '%s  -  %s' % (metric, track.note)
        if d.textlength(cap, font=f) <= (px1 - px0) * 0.92:
            d.text((px1, box.y + 3), cap, fill=th.rgb('chrome_text_dim'),
                   font=f, anchor='ra')
            taken.append(_bbox((px1, box.y + 3), cap, f, 'ra'))
        elif d.textlength(metric, font=f) <= (px1 - px0) * 0.92:
            d.text((px1, box.y + 3), metric,
                   fill=th.rgb('chrome_text_dim'), font=f, anchor='ra')
            taken.append(_bbox((px1, box.y + 3), metric, f, 'ra'))

        vis = [a for a in rows if a.index <= horizon]
        pos = {a.index: (X(a.index), Y(a.score) if a.score is not None else py1)
               for a in rows}
        # edges first, under everything
        for a in vis:
            if a.parent is None or a.parent not in pos:
                continue
            if a.parent > horizon:
                continue
            ax, ay = pos[a.parent]
            bx, by = pos[a.index]
            d.line([ax, ay, bx, by],
                   fill=th.rgb(KIND_ROLE.get(a.kind, 'op_seed')), width=1)
        # the record staircase, over the edges and under the nodes
        pts, last, shown, labels = [], None, set(), []
        for i, r in best_so_far(rows,
                                require_admissible=track.gate_record):
            if i > horizon:
                break
            if last is not None and r != last:
                pts += [(X(i), Y(last))]
            pts += [(X(i), Y(r))]
            if r not in shown:
                shown.add(r)
                labels.append((X(i) - 6, Y(r) - fs.size - 4,
                               '%g' % round(r, 2)))
            last = r
        if len(pts) > 1:
            d.line([p for xy in pts for p in xy], fill=th.rgb('status_best'),
                   width=2)
        # the nodes
        for a in vis:
            cx, cy = pos[a.index]
            col = th.rgb(KIND_ROLE.get(a.kind, 'op_seed'))
            r = 4
            if a.score is None:
                # AN UNGRADED ATTEMPT IS NOT A ZERO. It is drawn as a tick on
                # the rail at the bottom of the axis -- present, countable, and
                # visibly not a score.
                d.line([cx, py1 - 5, cx, py1 + 3],
                       fill=th.rgb('status_dropped'), width=2)
                continue
            if a.admissible:
                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)
            else:
                d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=col,
                          width=2)
            if a.accepted:
                d.ellipse([cx - r - 3, cy - r - 3, cx + r + 3, cy + r + 3],
                          outline=th.rgb('status_kept'), width=1)

        # THE RECORD LABELS, LAST, placed against EVERYTHING already in the
        # band: the ticks and the caption (registered as they were drawn),
        # every visible node with its kept ring, and every ungraded tick on
        # the rail. Run 32's '12703' sat on the rejected-attempt ticks beside
        # it and its first drops (267, 251, 239 ...) printed over their own
        # nodes -- the old rule avoided other record labels only. Newest
        # first, so the record the film is currently about always wins; each
        # label tries above, right and below its step; one with no free spot
        # is not drawn. A backing plate keeps the gold line from striking
        # through the digits.
        obstacles = list(taken)
        for a in vis:
            cx, cy = pos[a.index]
            if a.score is None:
                obstacles.append((cx - 2, py1 - 6, cx + 2, py1 + 4))
            else:
                obstacles.append((cx - 8, cy - 8, cx + 8, cy + 8))

        def _hits(rect):
            return any(not (rect[2] < o[0] or o[2] < rect[0]
                            or rect[3] < o[1] or o[3] < rect[1])
                       for o in obstacles)

        placed = []
        plate = th.rgb('chrome_panel')
        for lx, ly, txt in reversed(labels):
            h = fs.size + 4
            for cx, cy in ((lx, ly - 6), (lx + 14, ly - 6),
                           (lx + 14, ly + h + 10), (lx, ly + h + 10)):
                rect = _bbox((cx, cy), txt, fs)
                inside = (rect[0] >= box.x and rect[2] <= box.x + box.w
                          and rect[1] >= box.y and rect[3] <= box.y + box.h)
                if inside and not _hits(rect):
                    obstacles.append(rect)
                    placed.append((txt, rect))
                    d.rectangle([rect[0] - 1, rect[1] - 1, rect[2] + 1,
                                 rect[3] + 1], fill=plate)
                    d.text((cx, cy), txt, fill=th.rgb('status_best'),
                           font=fs)
                    break
        if debug is not None:
            debug['labels'] = placed
            debug['obstacles'] = obstacles[:len(obstacles) - len(placed)]
            debug['wanted'] = [t for _x, _y, t in labels]
        return True
    except Exception:                                          # noqa: BLE001
        return False  # a band is never worth failing a render over


def attach(frames, track: Optional[Track], *, theme=None, marks=None,
           box=None):
    """Draw the graph into a band on every frame.

    ``box`` (#946/C4), a `frame_layout.Box`, is the band the LAYOUT reserved
    (`FrameGeometry.track`): the graph is drawn INTO it and the frame keeps
    the size the layout planned, so every ratio preset stays the size it
    declares. Without a box every frame GROWS by a constant band, as it
    always did -- the path for a caller that planned no band.

    Returns `(frames, report)`. **When there is nothing to draw the frame list
    comes back COMPLETELY UNTOUCHED** -- the same list object holding the same
    `Image` objects -- and the report says why in words. No OFF state may read
    like success.

    `marks` is `build_boards`' per-step `(label, board, first, last)` list; when
    given, the visibility horizon follows the STEPS rather than a linear ramp,
    so a step that took 40 frames does not advance the graph 40 attempts.
    """
    report = {'drawn': False, 'why': '', 'attempts': 0, 'source': '',
              'band_px': 0}
    if not frames:
        report['why'] = 'no frames'
        return frames, report
    if track is None or not track.attempts:
        report['why'] = ('no loop_round*.json sidecars and no converge '
                         'ledger: this chain is one attempt')
        return frames, report
    report['attempts'] = len(track.attempts)
    report['source'] = track.source
    if len(track.attempts) < 2:
        report['why'] = ('one attempt on disk (%s): a single point under a '
                         'flat staircase is noise' % track.source)
        return frames, report
    try:
        from PIL import Image, ImageDraw
        import frame_layout
    except Exception as exc:                                   # noqa: BLE001
        report['why'] = 'no PIL (%s)' % exc
        return frames, report
    import frame_spool
    sizes = frame_spool.frame_sizes(frames)
    if len(sizes) != 1:
        # A caller that hands us a mixed list has a defect of its own, and
        # pasting onto the first frame's size would CROP the others silently --
        # the same class of quiet distortion this whole subsystem exists to
        # refuse. Say so and decline.
        report['why'] = ('the frames are not one size (%s); the band declines '
                         'rather than cropping them' % sorted(sizes)[:3])
        return frames, report
    W, H = frames[0].size
    if box is not None and box.w > 0 and box.h > 0:
        bh = int(box.h)
    else:
        box = None
        bh = band_height(W, H)
    if not bh:
        report['why'] = ('the frame is %dx%d; a legible band would be over '
                         '%.0f%% of it, so there is no room for one'
                         % (W, H, BAND_MAX_FRAC * 100))
        return frames, report
    idx = [a.index for a in track.attempts]
    lo, hi = min(idx), max(idx)
    n = max(1, len(frames) - 1)
    # The horizon per frame, decided before any frame is rebuilt.
    horizons = None
    if marks:
        try:
            ends = sorted({int(m[3]) for m in marks if len(m) >= 4})
            if ends:
                horizons = []
                for i in range(len(frames)):
                    k = sum(1 for e in ends if e < i)
                    horizons.append(lo + (hi - lo) * (k / float(len(ends))))
        except Exception:                                      # noqa: BLE001
            horizons = None
    try:
        import render_theme
        th = render_theme.theme(theme, strict=False)
    except Exception:                                          # noqa: BLE001
        th = None
    bg = th.rgb('ground') if th is not None else (14, 16, 18)
    into = box is not None
    if not into:
        box = frame_layout.Box(0, H, W, bh)
    # `draw_track` RETURNS whether it drew: a band shorter than its own plot
    # rectangle declines, and reporting `drawn=True` over a blank strip is an
    # OFF state reading like success -- which is the one thing this module's
    # degradation contract forbids. The answer depends only on the band's
    # size, so it is asked ONCE, on a scratch band, before any frame is
    # touched -- which is also what lets a spool apply the band lazily while
    # the encoder streams (#1036).
    _probe = Image.new('RGB', (box.w, bh), bg)
    drew = bool(draw_track(ImageDraw.Draw(_probe),
                           frame_layout.Box(0, 0, box.w, bh), track, upto=hi,
                           theme=th))
    if drew and into:
        def _into(i, f):
            up = horizons[i] if horizons else (lo + (hi - lo) * (i / float(n)))
            d = ImageDraw.Draw(f)
            d.rectangle([box.x, box.y, box.x + box.w - 1, box.y + box.h - 1],
                        fill=bg)
            draw_track(d, box, track, upto=up, theme=th)
            return f
        frames = frame_spool.transform(frames, _into, out_size=(W, H))
    elif drew:
        def _band(i, f):
            canvas = Image.new('RGB', (W, H + bh), bg)
            canvas.paste(f, (0, 0))
            up = horizons[i] if horizons else (lo + (hi - lo) * (i / float(n)))
            draw_track(ImageDraw.Draw(canvas), box, track, upto=up, theme=th)
            return canvas
        frames = frame_spool.transform(frames, _band, out_size=(W, H + bh))
    if not drew:
        report.update(drawn=False, band_px=bh,
                      why='the band is %d px, too short for its own plot; '
                          'nothing was drawn in it' % bh)
        return frames, report
    report.update(drawn=True, band_px=bh, reserved=into,
                  why='%d attempts from %s (%s)%s'
                      % (len(track.attempts), track.source, track.note,
                         ', in the layout-reserved band' if into else ''))
    return frames, report


def status_line(report) -> str:
    """One line saying whether the band ran, was skipped, or failed -- the
    same channel `movie_panels.iso_status_line` gives the iso panel, and for
    the same reason: a feature with no dialog control needs a way to say what
    it did."""
    if not report:
        return 'attempts band: not asked for'
    if report.get('drawn'):
        return ('attempts band: %s (%d px)'
                % (report.get('why', ''), report.get('band_px', 0)))
    return 'attempts band: not drawn -- %s' % (report.get('why') or 'unknown')
