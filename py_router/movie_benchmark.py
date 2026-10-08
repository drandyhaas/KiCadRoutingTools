#!/usr/bin/env python3
"""ONE benchmark band for the stage3d film (#1081): is it working yet, and is
it still getting better?

The verdict band plots `blocking`, and placement is three more panels. A viewer
asking "is this board working yet, and is it still improving?" had to read
four graphs and still found no finish line. This band is one curve on one
x axis, in two regimes split by ONE horizontal line:

  * **above the line: not working yet.** `blocking` (log scale) falling toward
    the line -- placement laps and routing laps alike, one curve, because a
    placement lap's `board_score` already reports the `blocking` a routed
    result would still carry. Placement detail rides on its record labels.
  * **the line is DONE**, `ledger_score.row_done`: blocking 0, nothing unknown
    or ungraded, no lens FAILed, a score about THIS board. The band is WORKING
    while the run's latest ACCEPTED lap is done: the ground turns green
    (`status_kept`, a role the theme already measures) from the first such lap
    -- a `WORKING @ t` chip -- and turns back, marked, if an accepted lap
    later falls above the line. A rejected lap never makes the board working:
    the phase-4 verifier found three real ledgers whose "crossing" was a lap
    the run itself had thrown away.
  * **below the line: better.** Records are the running best over accepted
    laps on the run's own lexicographic order -- working before not, then
    `blocking`, then `(vias, copper_mm, segments)` -- never a weighted sum. y
    is the via count as a percentage of a reference; a lap that ties on vias
    but wins on copper is still a record step, labelled with the term that
    decided it (`copper -3.2 mm`).

**The human benchmark is OPTIONAL** (`--benchmark-board`). With one whose via
count is measured, y is a percentage of it, a dashed line marks 100 %, and the
first record strictly better than it on the FULL key -- when the benchmark is
itself a working board -- earns a GOLD marker (`status_best`). Without one,
100 % is the first working board, there is no line and no gold, and the band's
caption says "no benchmark board" so the absence reads as a fact.

`--final` rows and `--exhausted` declarations are not laps
(`converge._is_lap`); the last final row's verdict is named in the caption.
Degradation is never silent: `attach` returns the frames untouched and says
why.
"""
from __future__ import annotations

import glob
import hashlib
import json
import math
import os
import shutil
import sys
from typing import NamedTuple, Optional, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import ledger_score as LS                                       # noqa: E402

#: The share of the plot height above the DONE line (the blocking regime).
ABOVE_FRAC = 0.52
#: The blocking axis tops out at this multiple of its 90th percentile, so one
#: 12 703-blocking pile at lap 0 cannot squeeze every later lap into 6 px
#: (run 32, measured); a lap above it is drawn at the top, marked.
OUTLIER_X = 2.0
#: Left margin for the axis names.
AXIS_W = 64


class Point(NamedTuple):
    index: int                      # the lap's ordinal on this band
    iteration: Optional[int]        # the ledger's own number, for labels
    t: Optional[float]
    kind: str
    accepted: bool
    blocking: Optional[float]       # None = ungraded (drawn as a tick)
    done: Optional[bool]
    key: tuple
    evidence: str                   # 'verified' | 'measured' | ''
    detail: str                     # placement detail for a record label
    #: components the score names `ungraded`: a working board carries them
    #: as UNEXAMINED, never as passed (`ledger_score.unexamined`)
    unexamined: Tuple[str, ...] = ()


class Benchmark(NamedTuple):
    name: str
    key: tuple
    blocking: Optional[float]       # None = not graded
    why: str                        # how it was graded, or why not


class BenchTrack(NamedTuple):
    points: Tuple[Point, ...]
    source: str
    domain: Optional[Tuple[float, float]]
    benchmark: Optional[Benchmark]
    note: str
    final: str = ''                 # the last --final row's verdict


# ---------------------------------------------------------------------------
# adapters
# ---------------------------------------------------------------------------
def _placement_detail(score) -> str:
    """The placement terms a placement record's label carries."""
    by = score.get('blocking_by') if isinstance(score, dict) else None
    if not isinstance(by, dict):
        return ''
    parts = []
    for k, v in sorted(by.items(), key=lambda kv: str(kv[0])):
        if isinstance(v, (int, float)) and not isinstance(v, bool) and v:
            parts.append('%s %g' % (k, v))
    return ', '.join(parts[:2])


def _final_note(row) -> str:
    stop = row.get('stop_condition') or ''
    fails = sorted(k for k, v in LS.lens_verdicts(row).items() if v == 'FAIL')
    done = LS.row_done(dict(row, kind='completion'))
    word = stop or ('DONE' if done else 'not done')
    if len(word) > 40:                  # a stop_condition can be a paragraph
        word = word[:37] + '...'
    # `done_evidence` is about a --final row, and laps never are one, so this
    # is the one place 'verified' can be said (the final review: the chip's
    # tag could only ever read 'measured')
    tag = ', verified' if (done and LS.done_evidence(row) == 'verified') \
        else ''
    return 'final: %s%s%s' % (word, (' (%s FAIL)' % ', '.join(fails))
                              if fails else '', tag)


def from_converge_ledger(path) -> Optional[BenchTrack]:
    """A converge ledger (JSONL) -> a BenchTrack: every placement and
    routing LAP on one curve. Non-object lines, non-lap kinds, `--final` and
    `--exhausted` rows are not laps; a lap with no countable `blocking` is
    kept, ungraded. Laps are numbered by their order in the file -- the
    ledger's `iteration` can repeat or be missing, and a horizon counted on
    it would reveal the wrong laps."""
    from movie_attempts import _blocking_value, _row_t
    rows = []
    try:
        with open(path, encoding='utf-8') as f:
            for line in f:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if isinstance(r, dict):
                    rows.append(r)
    except OSError:
        return None
    pts = []
    final = ''
    for r in rows:
        if r.get('final'):
            final = _final_note(r)
        if not LS.is_lap(r):
            continue
        kind = str(r.get('kind') or '').lower()
        sc = r.get('score') if isinstance(r.get('score'), dict) else {}
        b = _blocking_value(sc.get('blocking')) if sc else None
        it = r.get('iteration')
        done = LS.row_done(r)
        pts.append(Point(
            index=len(pts),
            iteration=it if isinstance(it, int) and not isinstance(it, bool)
            else None,
            t=_row_t(r), kind=kind, accepted=bool(r.get('accepted')),
            blocking=None if b is None else b, done=done,
            key=LS.quality_key(sc),
            evidence=LS.done_evidence(r) if done else '',
            detail=_placement_detail(sc) if kind == 'placement' else '',
            unexamined=tuple(LS.unexamined(r))))
    if not pts:
        return None
    ts = [p.t for p in pts]
    dom = None
    if all(t is not None for t in ts) and max(ts) > min(ts):
        dom = (min(ts), max(ts))
    ung = sum(1 for p in pts if p.blocking is None)
    note = '%d laps' % len(pts) + (', %d ungraded' % ung if ung else '')
    return BenchTrack(tuple(pts), 'converge', dom, None, note, final)


def from_loop_dir(work_dir) -> Optional[BenchTrack]:
    """`loop_round*.json` sidecars -> a BenchTrack: the loop's `failures` is
    its blocking term, and `metrics.vias` its only quality term (copper and
    segments are not recorded, so they rank as unmeasured, +inf)."""
    from movie_attempts import _blocking_value
    docs = []
    for p in sorted(glob.glob(os.path.join(work_dir, 'loop_round*.json'))):
        try:
            with open(p, encoding='utf-8') as f:
                d = json.load(f)
        except (OSError, ValueError):
            continue
        if isinstance(d, dict) and d.get('schema') == 1 and 'round' in d:
            docs.append(d)
    if not docs:
        return None
    docs.sort(key=lambda d: d['round'])
    pts = []
    for d in docs:
        met = d.get('metrics') or {}
        b = _blocking_value(met.get('failures'))
        done = None if b is None else (b == 0)
        pts.append(Point(index=len(pts), iteration=int(d['round']), t=None,
                         kind='completion', accepted=bool(d.get('accepted')),
                         blocking=b, done=done,
                         key=LS.quality_key({'quality': {
                             'vias': met.get('vias')}}),
                         evidence='measured' if done else '', detail=''))
    return BenchTrack(tuple(pts), 'loop', None, None, '%d rounds' % len(pts))


def discover(hint) -> Optional[BenchTrack]:
    """The converge ledger beside `hint` if there is one, else its loop
    sidecars, else None."""
    if not hint:
        return None
    d = hint if os.path.isdir(hint) else os.path.dirname(os.path.abspath(hint))
    for name in ('ledger.jsonl', 'converge.jsonl'):
        p = os.path.join(d, name)
        if os.path.isfile(p):
            tr = from_converge_ledger(p)
            if tr:
                return tr
    return from_loop_dir(d)


def _sha256(path) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def grade_benchmark(board, score_json=None, timeout=900) -> Benchmark:
    """The human board's key and blocking. Its quality is read in process
    (`board_score.quality`); its blocking from `score_json` when given (a
    `board_score --json` document, which must name THIS board by its
    `board_sha`), else by running `board_score` once -- never guessed. A
    benchmark that cannot be graded still gets its line; it cannot earn
    gold, and `why` says so."""
    name = os.path.splitext(os.path.basename(board))[0]
    root = os.path.dirname(_HERE)
    tools = os.path.join(root, 'py_tools')
    if tools not in sys.path:
        sys.path.insert(0, tools)
    try:
        import board_score
        q = board_score.quality(board)
    except Exception as exc:                                   # noqa: BLE001
        q = {'error': str(exc)}
    key = LS.quality_key({'quality': q})
    doc = None
    why = ''
    if score_json:
        try:
            with open(score_json, encoding='utf-8') as f:
                doc = json.load(f)
            why = 'graded by %s' % os.path.basename(score_json)
        except (OSError, ValueError) as exc:
            why = 'could not read %s (%s)' % (score_json, exc)
    else:
        import subprocess
        import tempfile
        tmp = tempfile.mkdtemp(prefix='bench_')
        out = os.path.join(tmp, 'score.json')
        try:
            subprocess.run([sys.executable,
                            os.path.join(tools, 'board_score.py'),
                            board, '--json', out, '-q'],
                           capture_output=True, timeout=timeout, cwd=root)
            with open(out, encoding='utf-8') as f:
                doc = json.load(f)
            why = 'graded by board_score'
        except Exception as exc:                               # noqa: BLE001
            msg = str(exc).splitlines()[0][:80] if str(exc) else ''
            why = 'board_score could not grade it (%s)' % (
                msg or type(exc).__name__)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    b = None
    if isinstance(doc, dict):
        sha = doc.get('board_sha')
        if not sha:
            why += ('; but that score names no board_sha, so which board it '
                    'graded is unknown')
        elif os.path.isfile(board) and sha != _sha256(board):
            why += '; but that score is about ANOTHER board (board_sha)'
        else:
            from movie_attempts import _blocking_value
            b = _blocking_value(doc.get('blocking'))
            if b is None:
                why += '; its blocking is not a count'
    return Benchmark(name, key, b, why)


def with_benchmark(track, bench) -> Optional[BenchTrack]:
    return track._replace(benchmark=bench) if track is not None else None


# ---------------------------------------------------------------------------
# the record, decided once
# ---------------------------------------------------------------------------
class Plan(NamedTuple):
    order: Tuple[Point, ...]            # by x
    done_at: Optional[int]              # position of the first working lap
    spans: Tuple[Tuple[int, Optional[int]], ...]   # working [start, end)
    records: Tuple[Tuple[int, str], ...]   # (position, label)
    ref_vias: Optional[float]           # 100 %
    bench_line: bool                    # draw the benchmark's 100 % line
    gold_at: Optional[int]              # position of the first strict beat
    ties_at: Optional[int]              # ...or of a match, when none beat
    bmax: float                         # the blocking axis' top


def rank(p) -> tuple:
    """The run's own order: a working lap before any other, then fewer
    `blocking`, then the lexicographic quality key."""
    if p.done:
        return (0, 0, p.key)
    return (1, p.blocking, ())


def plan(track) -> Plan:
    """Where the board works, which laps are records, and against what.

    Only ACCEPTED graded laps move the story: they are the run's own spine,
    so they decide both the records and whether the board is working now.
    """
    order = tuple(sorted(track.points,
                         key=lambda p: ((p.t if track.domain else 0.0),
                                        p.index)))
    spans = []
    records = []
    best = None
    working = False
    for i, p in enumerate(order):
        if not p.accepted or p.blocking is None:
            continue
        now = bool(p.done)
        if now and not working:
            spans.append([i, None])
        elif working and not now:
            spans[-1][1] = i
        working = now
        r = rank(p)
        if best is None or r < best[0]:
            lab = ''
            if p.done and best is not None and best[1].done:
                lab = LS.term_label(LS.deciding_term(best[1].key, p.key))
            elif not p.done and p.detail:
                lab = p.detail
            records.append((i, lab))
            best = (r, p)
    done_at = spans[0][0] if spans else None
    bench = track.benchmark
    ref = None
    line = False
    if (bench is not None and LS.plottable(bench.key[0])
            and bench.key[0] > 0):
        ref, line = float(bench.key[0]), True
    elif done_at is not None and LS.plottable(order[done_at].key[0]):
        ref = max(float(order[done_at].key[0]), 1.0)
    gold = ties = None
    if (bench is not None and bench.blocking == 0
            and all(LS.plottable(v) for v in bench.key)):
        for i, _lab in records:
            p = order[i]
            if not p.done:
                continue
            if p.key < bench.key and gold is None:
                gold = i
            elif p.key == bench.key and ties is None:
                ties = i
    bs = sorted(float(p.blocking) for p in order
                if p.blocking is not None and not p.done
                and LS.plottable(p.blocking))
    bmax = 1.0
    if bs:
        p90 = bs[int(0.9 * (len(bs) - 1))]
        bmax = max(1.0, min(bs[-1], OUTLIER_X * max(p90, 1.0)))
    return Plan(order, done_at, tuple((a, b) for a, b in spans),
                tuple(records), ref, line, gold,
                None if gold is not None else ties, bmax)


# ---------------------------------------------------------------------------
# drawing
# ---------------------------------------------------------------------------
def _fmt_t(sec) -> str:
    sec = int(max(0, sec))
    return '%d:%02d:%02d' % (sec // 3600, (sec // 60) % 60, sec % 60)


def _blend(a, b, k):
    return tuple(int(round(x * (1 - k) + y * k)) for x, y in zip(a, b))


def _overlaps(a, b):
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def draw_band(d, box, track, *, upto=None, theme=None, debug=None) -> bool:
    """Draw the band into `box` on draw `d`, showing laps up to `upto` (an
    index horizon, like the verdict band). Returns whether it drew."""
    try:
        import render_theme
        th = render_theme.theme(theme, strict=False)
    except Exception:                                          # noqa: BLE001
        th = None

    def rgb(role, fallback):
        try:
            return th.rgb(role) if th is not None else fallback
        except Exception:                                      # noqa: BLE001
            return fallback
    ground = rgb('ground', (14, 16, 18))
    ink = rgb('chrome_text', (200, 204, 196))
    dim = rgb('chrome_text_dim', (120, 126, 118))
    ok = rgb('status_kept', (86, 206, 130))
    gold = rgb('status_best', (255, 214, 88))
    tried = rgb('status_tried', (200, 130, 60))
    dropped = rgb('status_dropped', (110, 110, 110))
    from route_render import load_font
    cap_h = max(12, min(16, box.h // 9))
    font = load_font(cap_h - 2)
    small = load_font(max(9, cap_h - 4))
    x0, y0 = box.x + AXIS_W, box.y + cap_h + 8
    x1, y1 = box.x + box.w - 10, box.y + box.h - (cap_h + 2)
    if x1 - x0 < 60 or y1 - y0 < 40:
        return False
    pl = plan(track)
    order = pl.order
    if not order:
        return False
    line_y = int(y0 + (y1 - y0) * ABOVE_FRAC)
    if track.domain:
        t0, t1 = track.domain
        xs = [x0 + (x1 - x0) * (p.t - t0) / (t1 - t0) for p in order]
    else:
        n = max(1, len(order) - 1)
        xs = [x0 + (x1 - x0) * i / n for i in range(len(order))]
    horizon = upto if upto is not None else max(p.index for p in order)
    shown = [i for i, p in enumerate(order) if p.index <= horizon]
    show = set(shown)
    lb = math.log1p(pl.bmax)

    def y_above(b):
        b = min(float(b), pl.bmax)
        return line_y - 4 - (line_y - 4 - y0) * math.log1p(max(b, 0.5)) / lb

    pcts = []
    if pl.ref_vias:
        for p in order:
            if p.done and LS.plottable(p.key[0]):
                pcts.append(100.0 * float(p.key[0]) / pl.ref_vias)
        if pl.bench_line:
            pcts.append(100.0)
    lo_p = min(pcts + [100.0]) if pcts else 90.0
    hi_p = max(pcts + [100.0]) if pcts else 100.0
    if hi_p - lo_p < 10:
        lo_p = hi_p - 10

    def y_below(pct):
        return line_y + 5 + (y1 - line_y - 5) * (hi_p - pct) / (hi_p - lo_p)

    def ypos(p):
        if p.blocking is None:
            return None
        if p.done:
            if pl.ref_vias and LS.plottable(p.key[0]):
                return y_below(100.0 * float(p.key[0]) / pl.ref_vias)
            return line_y + 5
        return y_above(p.blocking)
    ys = {i: ypos(order[i]) for i in shown}

    # ground, then the WORKING spans the horizon has reached
    d.rectangle([box.x, box.y, box.x + box.w - 1, box.y + box.h - 1],
                fill=ground)
    tint = _blend(ground, ok, 0.12)
    live_spans = []
    for a, b in pl.spans:
        if a not in show:
            continue
        end = b if (b is not None and b in show) else None
        d.rectangle([int(xs[a]), y0, int(xs[end]) if end is not None
                     else x1, y1], fill=tint)
        live_spans.append((a, end))
    # the DONE line and its name
    d.line([(x0, line_y), (x1, line_y)], fill=ok, width=2)
    occupied = []

    def put(x, y, text, f, fill, tries=((0, 0),)):
        """Draw `text` at the first offset that overlaps nothing drawn yet;
        text shifted past either end is pulled back inside the plot.
        False = dropped."""
        for dx, dy in tries:
            bb = d.textbbox((x + dx, y + dy), text, font=f)
            if bb[2] > x1:
                shift = bb[2] - x1
                bb = (bb[0] - shift, bb[1], bb[2] - shift, bb[3])
            # a left-shifted try at a lap on the axis ran off the band: two
            # placement laps 2.6 s apart both sit at x0 (esp_prog, run 35)
            if bb[0] < x0:
                shift = x0 - bb[0]
                bb = (bb[0] + shift, bb[1], bb[2] + shift, bb[3])
                if bb[2] > x1:                  # wider than the plot
                    continue
            rect = (bb[0] - 2, bb[1] - 1, bb[2] + 2, bb[3] + 1)
            if rect[1] < box.y or rect[3] > box.y + box.h:
                continue
            if any(_overlaps(rect, o) for o in occupied):
                continue
            d.text((bb[0], bb[1]), text, font=f, fill=fill, anchor='lt')
            occupied.append(rect)
            return True
        return False
    # the caption and the axes' own words go down FIRST, as obstacles: drawn
    # last, a record label placed before them was overwritten by the caption
    # and the axis name (esp_prog, run 35)

    def _fixed(xy, text, fill, anchor='la'):
        d.text(xy, text, fill=fill, font=small, anchor=anchor)
        bb = d.textbbox(xy, text, font=small, anchor=anchor)
        occupied.append((bb[0] - 2, bb[1] - 1, bb[2] + 2, bb[3] + 1))
    # caption
    bench = track.benchmark
    if bench is None:
        tail = 'no benchmark board (100 % = the first working board)'
    elif not pl.bench_line:
        tail = ('benchmark %s: its via count is unmeasured, so 100 %% = the '
                'first working board' % bench.name)
    elif bench.blocking == 0:
        tail = 'benchmark %s' % bench.name
    else:
        tail = 'benchmark %s is not a working board, so no gold' % bench.name
    parts = ['records ranked on (vias, copper, segments)', tail,
             track.note] + ([track.final] if track.final else [])
    cap = '  |  '.join(parts)
    while len(parts) > 2 and d.textlength(cap, font=small) > box.w - 12:
        parts.pop(2)
        cap = '  |  '.join(parts)
    _fixed((box.x + 6, box.y + 2), cap, dim)
    _fixed((box.x + 6, y0), 'blocking', ink)
    _fixed((box.x + 6, line_y + 16), 'vias %', ink)
    # the axes' own numbers: the top of the blocking scale, the 100 % mark,
    # and the run clock (or lap numbers) at each end
    _fixed((x0 - 4, y0 + 16), '%g' % round(pl.bmax, 1), dim, 'ra')
    if pl.ref_vias and lo_p <= 100.0 <= hi_p:
        _fixed((x0 - 4, int(y_below(100.0))), '100 %', dim, 'rm')
    if track.domain:
        left, right = _fmt_t(0), _fmt_t(track.domain[1] - track.domain[0])
    else:
        first, last = order[0], order[-1]
        left = 'lap %d' % (first.iteration if first.iteration is not None
                           else first.index)
        right = 'lap %d' % (last.iteration if last.iteration is not None
                            else last.index)
    _fixed((x0, y1 + 1), left, dim, 'la')
    _fixed((x1, y1 + 1), right, dim, 'ra')
    for i in shown:                       # nodes are obstacles for text
        if ys[i] is not None:
            occupied.append((xs[i] - 6, ys[i] - 6, xs[i] + 6, ys[i] + 6))
    cap_line = 'WORKING: blocking 0, nothing unknown, no lens FAIL'
    put(x0 + 4, line_y - 16, cap_line, small, ok,
        tries=((0, 0), (0, 20)))
    # the benchmark's 100 % line
    if bench is not None and pl.bench_line:
        yb = int(y_below(100.0))
        for xx in range(int(x0), int(x1), 10):
            d.line([(xx, yb), (min(xx + 5, x1), yb)], fill=gold, width=1)
        put(x1, yb + 2, 'benchmark %s = 100 %%' % bench.name, small, gold,
            tries=((0, 0), (0, -16)))
        # the dashed line is an obstacle too: a record label struck through
        # by it read as crossed out (the phase-7 verification)
        occupied.append((x0, yb - 1, x1, yb + 1))
    # the curve over the accepted spine
    prev = None
    for i in shown:
        p = order[i]
        if ys[i] is None or not p.accepted:
            continue
        if prev is not None:
            col = ok if (p.done and order[prev].done) else tried
            d.line([(xs[prev], ys[prev]), (xs[i], ys[i])], fill=col, width=3)
        prev = i
    # the record staircase
    rec = [(i, lab) for i, lab in pl.records if i in show]
    for (a, _la), (b, _lb) in zip(rec, rec[1:]):
        d.line([(xs[a], ys[a]), (xs[b], ys[a]), (xs[b], ys[b])], fill=gold,
               width=2)
    # nodes
    r = 4
    for i in shown:
        p = order[i]
        y = ys[i]
        if y is None:
            d.line([(xs[i], y1), (xs[i], y1 - 7)], fill=dropped, width=2)
            continue
        col = ok if p.done else tried
        box_ = [xs[i] - r, y - r, xs[i] + r, y + r]
        if p.accepted:
            d.ellipse(box_, fill=col)
        else:
            d.ellipse(box_, outline=col, width=2)
        if (not p.done and p.blocking is not None
                and LS.plottable(p.blocking) and p.blocking > pl.bmax):
            d.polygon([(xs[i], y - 9), (xs[i] - 4, y - 5),
                       (xs[i] + 4, y - 5)], fill=col)   # off the scale
    # the crossing chip, clear of the data
    chip = None
    if live_spans and pl.done_at in show:
        p = order[pl.done_at]
        xc = xs[pl.done_at]
        d.ellipse([xc - 7, line_y - 7, xc + 7, line_y + 7], outline=ok,
                  width=3)
        when = (_fmt_t(p.t - track.domain[0]) if track.domain
                else 'lap %d' % (p.iteration if p.iteration is not None
                                 else p.index))
        tags = [p.evidence] if p.evidence == 'measured' else []
        if p.unexamined:
            tags.append('%d unexamined' % len(p.unexamined))
        chip = 'WORKING @ %s%s' % (when, (' (%s)' % ', '.join(tags))
                                   if tags else '')
        tw = int(d.textlength(chip, font=font)) + 12
        ch = cap_h + 4
        for cx, cy in ((xc + 12, line_y - ch - 22), (xc - tw - 12,
                                                     line_y - ch - 22),
                       (xc + 12, y0), (xc - tw - 12, y0)):
            cx = max(x0, min(int(cx), x1 - tw))
            rect = (cx, cy, cx + tw, cy + ch)
            if not any(_overlaps(rect, o) for o in occupied):
                break
        d.rectangle(rect, fill=ok)
        d.text((cx + 6, cy + 2), chip, fill=ground, font=font)
        occupied.append(rect)
    # a regression: the board STOPPED working, and says when
    for a, b in live_spans:
        if b is None:
            continue
        p = order[b]
        when = (_fmt_t(p.t - track.domain[0]) if track.domain
                else 'lap %d' % (p.iteration if p.iteration is not None
                                 else p.index))
        put(xs[b] + 6, line_y - 16, 'not working @ %s' % when, small, tried,
            tries=((0, 0), (0, -16), (-120, -16)))
    # gold: the first record that beats the (working) benchmark
    marker = None
    gold_xy = None
    for pos, word in ((pl.gold_at, 'beats'), (pl.ties_at, 'matches')):
        if pos is None or pos not in show:
            continue
        p = order[pos]
        s = 7
        # inside the plot: at the corner the diamond was clipped
        y = min(max(ys[pos], y0 + s), y1 - s)
        gx = min(max(xs[pos], x0 + s), x1 - s)
        d.polygon([(gx, y - s), (gx + s, y), (gx, y + s), (gx - s, y)],
                  fill=gold)
        gold_xy = (gx, y)
        when = (_fmt_t(p.t - track.domain[0]) if track.domain
                else 'lap %d' % (p.iteration if p.iteration is not None
                                 else p.index))
        marker = '%s benchmark @ %s' % (word, when)
        put(gx + 10, y - 18, marker, small, gold,
            tries=((0, 0), (0, 12), (-170, -18), (-170, 12), (-170, -32),
                   (10, -32), (-85, -34)))
        break
    # record labels: the deciding term below the line, placement detail
    # above; a label that would overlap anything is dropped, never stacked
    labels = []
    for i, lab in rec:
        if not lab or ys[i] is None:
            continue
        col = ok if order[i].done else tried
        if put(xs[i] + 7, ys[i] + 4, lab, small, col,
               tries=((0, 0), (0, -18), (-60, 6), (-90, -18), (8, -32),
                      (-110, -32), (-110, 6))):
            labels.append((i, lab))
    if debug is not None:
        debug.update(line_y=line_y, xs=xs, shown=shown, gold_xy=gold_xy,
                     crossed=bool(live_spans), chip=chip, marker=marker,
                     labels=labels, records=[i for i, _l in rec],
                     caption=cap, plot=(x0, y0, x1, y1), done_at=pl.done_at,
                     spans=live_spans, ypos=ys,
                     text_rects=[o for o in occupied
                                 if (o[2] - o[0]) != 12 or (o[3] - o[1]) != 12])
    return True


def attach(frames, track, *, box, theme=None, marks=None):
    """Draw the band into `box` (the layout's reserved band) on every frame,
    revealing laps as the film's steps pass (the lap's time against the
    film's clock). Returns `(frames, report)`; with nothing to draw the frames
    come back untouched and the report says why."""
    report = {'drawn': False, 'why': '', 'laps': 0}
    if not frames:
        report['why'] = 'no frames'
        return frames, report
    if box is None or box.w <= 0 or box.h <= 0:
        report['why'] = 'the layout reserved no band'
        return frames, report
    if track is None or len(track.points) < 2:
        report['why'] = ('no converge ledger or loop rounds with two laps: '
                         'nothing to rank')
        return frames, report
    from PIL import Image, ImageDraw
    import frame_spool
    import frame_layout
    idx = [p.index for p in track.points]
    lo, hi = min(idx), max(idx)
    n = max(1, len(frames) - 1)
    horizons = None
    if marks:
        ends = sorted({int(m[3]) for m in marks if len(m) >= 4})
        if ends:
            # a step counts once its LAST frame is drawn (`last` is
            # exclusive), so the final frame shows every lap -- counting only
            # steps that ended BEFORE a frame left the last lap off the film
            horizons = [lo + (hi - lo) * (sum(1 for e in ends if e <= i + 1)
                                          / float(len(ends)))
                        for i in range(len(frames))]
    probe = Image.new('RGB', (box.w, box.h))
    if not draw_band(ImageDraw.Draw(probe), frame_layout.Box(0, 0, box.w,
                                                             box.h),
                     track, theme=theme):
        report['why'] = 'the band is %dx%d, too small for its plot' % (
            box.w, box.h)
        return frames, report
    W, H = frames[0].size

    def _into(i, f):
        up = horizons[i] if horizons else lo + (hi - lo) * (i / float(n))
        draw_band(ImageDraw.Draw(f), box, track, upto=up, theme=theme)
        return f
    frames = frame_spool.transform(frames, _into, out_size=(W, H),
                                   optional='benchmark band')
    pl = plan(track)
    work = ('working from lap %d' % pl.order[pl.done_at].index
            if pl.done_at is not None else 'never working')
    report.update(drawn=True, laps=len(track.points),
                  why='%s (%s); %s; %s'
                      % (track.source, track.note, work,
                         ('benchmark %s' % track.benchmark.name)
                         if track.benchmark else 'no benchmark board'))
    return frames, report


def status_line(report) -> str:
    if not report:
        return 'benchmark band: not asked for'
    if report.get('drawn'):
        return 'benchmark band: %s' % report.get('why', '')
    return 'benchmark band: not drawn -- %s' % (report.get('why') or 'unknown')
