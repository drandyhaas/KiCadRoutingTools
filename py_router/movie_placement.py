#!/usr/bin/env python3
"""The placement progress panels, in placement currency (#1042).

`movie_attempts` keeps the routed VERDICT on its axis and deliberately keeps
placement proxies off it ("ON A PLACEMENT RUN THE AXIS IS STILL THE ROUTED
RESULT", #1021): a copper-free placement lap scores `blocking` ~250 on that
axis because every net is unrouted, while the thing the lap was doing moved
elsewhere. So placement gets its OWN panels, beside the verdict band and never
on its axis:

  1. **Legality** (log y): pads off the outline (parts), pad-conflict pairs,
     overlap mm² -- with the floor the KiCad-locked parts set, in the legend.
  2. **Arrangement** (a SCREEN, not the verdict): airwire crossings and hpwl
     as step lines, each on its own axis, with dashed benchmark lines when a
     benchmark board is given.
  3. **Intent**: floorplan errors per beat, from ONE instrument per line --
     `check_floorplan --intent` on every beat when an intent is given, else
     the ledger's own `board_score` value, labelled as such.

Plus **downstream-defect flags**: a ledger row `kind == classification`,
`shape == placement` flags the placement beat it sent the run back to.

**One point per placement BOARD, never per frame.** A glide's frames are
pixel interpolation, not evaluated placements. Every number is measured on the
film's own boards, IN PROCESS, by the named instruments' own functions
(`render_placement.PlacementModel` / `legality_findings` -- the numbers
`render_placement --json-out` writes -- and `check_floorplan.main`), cached per
board sha. Never a subprocess of `sys.executable`: inside KiCad that is the
pcbnew binary, and a child built that way hangs
(`kicad_routing_plugin/deps_check.py`).

**Cheap checks first.** A chain pays for a measurement only when it has at
least two copper-free boards AND a part moved between them (poses parsed, no
instrument run). **No placement, no panel**; nothing is synthesised.

**x is RUN TIME** when the ledger carries `t` -- the same domain the verdict
band draws (`movie_attempts.ledger_time_domain`), so a re-entry sits where it
happened. Without a clock it is the board order, and the header says so.

**Readable or not drawn.** Each plot is at least `PLOT_MIN_PX` tall and every
title, legend item and footer line renders WHOLE. A box too narrow for three
panels keeps fewer, in `PRIORITY` order, and the header names what it
dropped; a frame too small for one is declined (`plan_band`), with the reason
on the status line.
"""
from __future__ import annotations

import contextlib
import hashlib
import io
import json
import math
import os
import shutil
import sys
import tempfile
from typing import Dict, NamedTuple, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

#: The instrument every legality/arrangement number comes from, named in the
#: footers because the same word means different counts elsewhere:
#: `render_placement`'s `crossings` (3750 airwire crossings on run 32's
#: placed_v3) is not `board_score`'s `pin_order_crossings` (1211 part pairs).
#: The numbers are the ones its `--json-out` writes; they are computed here by
#: its own functions, in process.
INSTRUMENT = 'render_placement --json-out'
FP_INSTRUMENT = 'check_floorplan --intent'
FP_LEDGER = 'ledger board_score (no --intent)'

#: The smallest plot a panel may draw: a 13 px plot is a stripe, not a chart.
PLOT_MIN_PX = 48
#: The band may take at most this share of the frame when it carries the
#: placement panels; past it they are declined, and the status line says so.
BAND_MAX_FRAC = 0.48
#: Side by side, placement takes the FIRST of these shares of the band's
#: width that holds all three panels whole (the verdict keeps >= 42%).
SIDE_FRACS = (0.46, 0.52, 0.58)
SIDE_FRAC = SIDE_FRACS[0]
#: Which panels survive a narrow box, in order.
PRIORITY = ('intent', 'legality', 'arrangement')
#: Left-to-right order of the panels that are drawn.
ORDER = ('legality', 'arrangement', 'intent')
TITLES = {'legality': 'LEGALITY (log y)', 'arrangement': 'ARRANGEMENT (screen)',
          'intent': 'INTENT'}
#: The SCREEN statement: a footer line of its own, always drawn whole.
SCREEN_NOTE = 'not the verdict -- see band'
#: Series colours: every pair a reader must tell apart is a different role
#: (off-outline vs conflict pairs, conflict pairs vs crossings), and the
#: frame marker is not the intent series' gold.
ROLE = {'off-outline parts': 'status_dropped',
        'conflict pairs': 'defect_conflict',
        'overlap mm2': 'op_cross',
        'airwire crossings': 'place_net_pick',
        'hpwl mm': 'pad_back',
        'floorplan errors': 'status_best',
        'flag': 'defect_net_fail',
        'marker': 'chrome_text_dim',
        'floor': 'chrome_text_dim'}


class Beat(NamedTuple):
    """One placement board of the film, measured."""
    board: str
    label: str
    first: int                  # the frame it is shown from (its landing)
    off_outline: Optional[int]  # parts with pad copper off the outline
    conflict_pairs: Optional[int]
    overlap_mm2: Optional[float]
    crossings: Optional[int]
    hpwl: Optional[float]
    locked_pairs: Optional[int]  # metrics.locked_contact_pairs: the floor
    floorplan: Optional[int]
    floorplan_source: str        # FP_INSTRUMENT | 'ledger row N' | ''
    ledger_index: Optional[int]
    t: Optional[float] = None    # run time (ledger `t`), epoch seconds
    unmeasured: str = ''         # why the legality numbers are missing


class Flag(NamedTuple):
    beat: int                   # index into beats
    text: str
    row: int


class PlacementTrack(NamedTuple):
    beats: tuple
    benchmark: Optional[dict]   # measured metrics of the benchmark board
    benchmark_name: str
    flags: tuple
    notes: tuple
    x_domain: Optional[tuple] = None   # (t0, t1): x is run time
    floorplan_source: str = ''  # the ONE instrument the intent line reads


class Fit(NamedTuple):
    names: tuple                # the panels drawn, left to right
    fs: int                     # font px
    pw: int                     # panel width
    need_h: int                 # the box height they need
    dropped: tuple              # panels that did not fit, by name


class BandPlan(NamedTuple):
    band_h: int
    mode: str                   # 'side' | 'stacked' | 'placement' | 'declined'
    verdict_h: int
    place_h: int
    why: str


def _sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _tools_path():
    """py_tools and py_placer importable -- APPENDED, so a basename clash
    still resolves to py_router, as `py_tools/_path.py` arranges it."""
    for sub in ('py_tools', 'py_placer'):
        p = os.path.join(_ROOT, sub)
        if p not in sys.path:
            sys.path.append(p)


_CACHE: Dict[str, dict] = {}


def measure_board(path, cache=None):
    """`render_placement`'s legality and arrangement numbers for one board,
    IN PROCESS -- the same `PlacementModel` and `legality_findings` its
    `--json-out` reads, at the same defaults (board-first floors, no ignored
    nets). Keyed by the board's sha so a run measures a board once.

    Always a dict. `{'unmeasured': why}` when the instrument could not answer
    -- no parts, an unreadable board, a model with no placement state -- so a
    failure is DISCLOSED on the panel, never plotted as zero."""
    cache = _CACHE if cache is None else cache
    try:
        key = _sha(path)
    except OSError as exc:
        return {'unmeasured': 'unreadable (%s)' % type(exc).__name__}
    if key in cache:
        return cache[key]
    try:
        _tools_path()
        sink = io.StringIO()
        with contextlib.redirect_stdout(sink), contextlib.redirect_stderr(sink):
            import render_placement as RP
            from kicad_parser import parse_kicad_pcb
            pcb = parse_kicad_pcb(path)
            if not getattr(pcb, 'footprints', None):
                res = {'unmeasured': 'no parts on the board'}
            else:
                model = RP.PlacementModel(
                    pcb, path, exact=True,
                    quench_kwargs={'clearance': None, 'ignore_net_ids': None})
                if model.state is None or not model.metrics:
                    res = {'unmeasured': 'render_placement built no '
                                         'placement state'}
                else:
                    fnd = RP.legality_findings(model)
                    m = model.metrics
                    off = fnd.get('oob_refs_pad_copper')
                    res = {'off_outline': (len(off) if isinstance(off, list)
                                           else None),
                           'conflict_pairs': m.get('pad_conflict_pairs'),
                           'overlap_mm2': m.get('overlap_area'),
                           'crossings': m.get('crossings'),
                           'hpwl': m.get('hpwl'),
                           'locked_pairs': m.get('locked_contact_pairs')}
    except Exception as exc:                                   # noqa: BLE001
        res = {'unmeasured': 'render_placement raised %s'
               % type(exc).__name__}
    cache[key] = res
    return res


def check_floorplan_errors(path, intent):
    """`(errors, why)`: ERROR-severity violations from `check_floorplan
    --intent`, the count `board_score.score_floorplan` sums, run IN PROCESS
    through `check_floorplan.main`. `(None, why)` when it could not grade."""
    if not intent or not os.path.isfile(intent):
        return None, 'no intent file'
    tmp = tempfile.mkdtemp(prefix='krt_fp_')
    out_json = os.path.join(tmp, 'f.json')
    rc = None
    try:
        _tools_path()
        sink = io.StringIO()
        with contextlib.redirect_stdout(sink), contextlib.redirect_stderr(sink):
            import check_floorplan as CF
            try:
                rc = CF.main([path, '--intent', intent, '--json', out_json,
                              '--exit-zero', '-q', '--allow-unplaced'])
            except SystemExit as exc:
                rc = exc.code
        with open(out_json, encoding='utf-8') as f:
            d = json.load(f)
    except Exception as exc:                                   # noqa: BLE001
        return None, ('check_floorplan exited %r without a grade' % (rc,)
                      if isinstance(exc, OSError)
                      else 'check_floorplan raised %s' % type(exc).__name__)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    viols = d.get('violations') or []
    return (sum(1 for v in viols if (v.get('severity') or 'error') == 'error'),
            '')


def read_ledger(path):
    rows = []
    if not path or not os.path.isfile(path):
        return rows
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                doc = json.loads(line)
            except ValueError:
                continue
            if isinstance(doc, dict):
                rows.append(doc)
    return rows


def _row_fp(r):
    bb = (r.get('score') or {}).get('blocking_by') \
        if isinstance(r.get('score'), dict) else None
    return (bb or {}).get('floorplan')


def _row_t(r):
    try:
        return None if r.get('t') is None else float(r['t'])
    except (TypeError, ValueError):
        return None


def _ledger_match(sha, rows):
    """`(floorplan, row_index, t)` for a board. The floorplan (and its row)
    comes from the row that PRODUCED the board, else a row that started FROM
    it (a later lock re-saves a board with identical poses, so the producing
    row can carry another sha); the time is the first row naming it -- its
    producer when there is one. Nones when no row names it: the pile."""
    seq = ([(i, r) for i, r in enumerate(rows) if r.get('result_sha') == sha]
           + [(i, r) for i, r in enumerate(rows)
              if r.get('parent_sha') == sha])
    if not seq:
        return None, None, None
    fp_row = next(((i, r) for i, r in seq if _row_fp(r) is not None), None)
    i0, r0 = seq[0]
    if fp_row is not None:
        i, r = fp_row
        return _row_fp(r), int(r.get('iteration', i)), _row_t(r0)
    return None, int(r0.get('iteration', i0)), _row_t(r0)


def is_placement_board(path, prev_path=None, moved=None):
    """A board is a placement beat when it carries NO COPPER -- the boards of
    the placement half, the ones #1042's table measures. A routed board whose
    parts moved (a clearance nudge) is a routing step: its legality is read
    off copper these panels do not model, and the verdict band covers it."""
    import re
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            txt = f.read()
    except OSError:
        return False
    return not re.search(r'^(?:\t| {2})\((?:segment|arc|via)[\s)]', txt,
                         re.MULTILINE)


def _poses_moved(boards):
    """True when some part moved between consecutive boards: poses PARSED,
    no instrument run -- the cheap test that decides whether anything is
    measured at all."""
    if len(boards) < 2:
        return False
    try:
        import movie_camera
        return any(r['moved'] for r in movie_camera.synth_rounds(boards))
    except Exception:                                          # noqa: BLE001
        return False


def _headline(lever):
    """A flag's words: the lever's first sentence, cut at its first ':' --
    run 32's 'ONE pocket', 'Focus panels', 'GLOBAL capacity, not per-net'.
    The ledger holds the evidence; the flag names the defect."""
    text = str(lever or 'placement-shaped').split('. ')[0]
    text = text.split(' [read:')[0].strip()
    head = text.split(':')[0].strip()
    if len(head) >= 3:
        text = head
    words = text.split()
    return ' '.join(words[:5]) + (' ...' if len(words) > 5 else '')


def build_track(steps, marks, *, ledger=None, benchmark=None, intent=None,
                cache=None, quiet=False):
    """The placement track for a chain, or `(None, why)`.

    `steps` are the chain's `(label, board, ...)`; `marks` are build_boards'
    `(label, board, first, last)`. The cheap gates run FIRST: fewer than two
    copper-free boards, or no part moved between them, returns before any
    instrument is paid for.
    """
    if not steps:
        return None, 'no boards'
    cands = []
    for st in steps:
        b = st[1]
        nb = os.path.normcase(os.path.abspath(b))
        # a make_film 'revert' step re-shows a board already measured, and a
        # board repeated back to back is one placement, not two
        if len(st) > 3 and st[3] == 'revert':
            continue
        if cands and os.path.normcase(os.path.abspath(cands[-1][1])) == nb:
            continue
        if not is_placement_board(b):
            continue
        cands.append((str(st[0]), b))
    if len(cands) < 2:
        return None, ('%d copper-free board(s) in this chain: no placement '
                      'to show' % len(cands))
    if not _poses_moved([b for _l, b in cands]):
        return None, ('no part moved between the chain\'s placement boards: '
                      'no placement to show')

    rows = read_ledger(ledger)
    firsts = {}
    for mk in marks or ():
        firsts.setdefault(os.path.normcase(os.path.abspath(mk[1])), mk[2])
    notes = []
    beats = []
    for label, b in cands:
        nb = os.path.normcase(os.path.abspath(b))
        m = measure_board(b, cache)
        um = m.get('unmeasured', '')
        if um:
            notes.append('%s unmeasured: %s' % (label, um))
        try:
            sha = _sha(b)
        except OSError:
            sha = None
        lfp, li, lt = (_ledger_match(sha, rows) if sha
                       else (None, None, None))
        # ONE INSTRUMENT PER LINE: check_floorplan on every beat when an
        # intent is given; the ledger's value only when none is.
        if intent:
            fpv, why = check_floorplan_errors(b, intent)
            src = FP_INSTRUMENT if fpv is not None else ''
            if fpv is None:
                notes.append('%s floorplan unmeasured: %s' % (label, why))
        elif lfp is not None:
            fpv, src = lfp, 'ledger row %d' % li
        else:
            fpv, src = None, ''
            if rows:
                notes.append('%s: no ledger row scores its floorplan' % label)
        beats.append(Beat(b, label, firsts.get(nb, 0), m.get('off_outline'),
                          m.get('conflict_pairs'), m.get('overlap_mm2'),
                          m.get('crossings'), m.get('hpwl'),
                          m.get('locked_pairs'), fpv, src, li, lt, um))
    if not intent and not rows:
        notes.append('no --intent and no ledger: intent unmeasured')

    # RUN TIME: the ledger's own `t`, over the same domain the verdict band
    # draws. A board no row names (the pile: the run's INPUT) sits at the
    # start; a later unnamed board holds the previous beat's time.
    dom = None
    try:
        import movie_attempts
        dom = movie_attempts.ledger_time_domain(rows)
    except Exception:                                          # noqa: BLE001
        dom = None
    if dom is not None and any(bt.t is not None for bt in beats):
        fixed, last, n_start = [], None, 0
        for bt in beats:
            t = bt.t
            if t is None:
                n_start += last is None
                t = dom[0] if last is None else last
            t = min(max(t, dom[0]), dom[1])
            fixed.append(bt._replace(t=t))
            last = t
        beats = fixed
        if n_start:
            notes.append('%d board(s) the ledger does not name drawn at the '
                         'run start' % n_start)
    else:
        dom = None
        beats = [bt._replace(t=None) for bt in beats]

    bench = None
    if benchmark:
        bench = measure_board(benchmark, cache)
        if bench.get('unmeasured'):
            notes.append('benchmark unmeasured: %s' % bench['unmeasured'])
            bench = None
    flags = []
    for r in rows:
        if r.get('kind') != 'classification' or r.get('shape') != 'placement':
            continue
        it = int(r.get('iteration', 0))
        tgt = next((i for i, bt in enumerate(beats)
                    if bt.ledger_index is not None and bt.ledger_index > it),
                   None)
        text = _headline(r.get('lever'))
        if tgt is None:
            tgt = len(beats) - 1
            text = text + ' (after this film)'
        flags.append(Flag(tgt, text, it))
    src = ((FP_INSTRUMENT if intent else FP_LEDGER)
           if any(bt.floorplan is not None for bt in beats) else '')
    return (PlacementTrack(tuple(beats), bench,
                           os.path.splitext(os.path.basename(benchmark))[0]
                           if benchmark else '', tuple(flags), tuple(notes),
                           dom, src),
            '%d placement board(s)' % len(beats))


def beat_at(track, frame):
    """The index of the beat frame `frame` shows (the latest beat LANDED at
    or before it), or None before the first -- and then nothing is drawn."""
    cur = None
    for i, bt in enumerate(track.beats):
        if bt.first <= frame:
            cur = i
    return cur


# ---------------------------------------------------------------------------
# layout: shared by the band planner and the drawer, so what is reserved is
# exactly what is drawn
# ---------------------------------------------------------------------------
def _fmt(v):
    if v is None:
        return '--'
    av = abs(v)
    if av >= 10000:
        return '%.0fk' % (v / 1000.0)
    if av >= 1000:
        return '%.1fk' % (v / 1000.0)
    if av >= 10 or float(v).is_integer():
        return '%d' % round(v)
    return '%.2f' % v


def _nice_ceil(v):
    """The smallest 1/2/2.5/5 x 10^k at or above `v`: a ROUND tick."""
    if v is None or v <= 0:
        return 1.0
    k = 10.0 ** math.floor(math.log10(v))
    for m in (1.0, 2.0, 2.5, 5.0, 10.0):
        if m * k >= v - 1e-9:
            return m * k
    return 10.0 * k


def _font_px(frame_h):
    import render_chrome
    return max(9, min(12, render_chrome.type_px('small', frame_h)))


def _measure_draw():
    from PIL import Image, ImageDraw
    return ImageDraw.Draw(Image.new('RGB', (4, 4)))


def _floor(track):
    lb = track.beats[-1]
    if (lb.conflict_pairs is not None and lb.locked_pairs
            and lb.conflict_pairs <= lb.locked_pairs):
        return lb.conflict_pairs
    return None


def _specs(track):
    """Per panel: title, legend items `(role, text, kind)`, footer lines."""
    floor = _floor(track)
    leg_l = [(ROLE['off-outline parts'], 'off-outline parts', 'box'),
             (ROLE['conflict pairs'], 'conflict pairs', 'box'),
             (ROLE['overlap mm2'], 'overlap mm2', 'box')]
    if floor is not None:
        leg_l.append((ROLE['floor'], 'floor %s = locked parts' % _fmt(floor),
                      'dash'))
    leg_a = [(ROLE['airwire crossings'], 'airwire crossings', 'box'),
             (ROLE['hpwl mm'], 'hpwl mm', 'box')]
    if track.benchmark:
        # the benchmark's lines are dashed in EACH series' own colour; the
        # legend names the board and the dash, not a colour of its own
        leg_a.append((ROLE['floor'], '%s (dashed)'
                      % (track.benchmark_name or 'benchmark'), 'dash'))
    leg_i = [(ROLE['floorplan errors'], 'floorplan errors', 'box')]
    for k, fl in enumerate(track.flags):
        leg_i.append((ROLE['flag'], '%d: %s' % (k + 1, fl.text), 'flag'))
    return {
        'legality': (TITLES['legality'], leg_l, [INSTRUMENT]),
        'arrangement': (TITLES['arrangement'], leg_a,
                        [INSTRUMENT, SCREEN_NOTE]),
        'intent': (TITLES['intent'], leg_i,
                   [track.floorplan_source or 'unmeasured: no --intent']),
    }


def _wrap_words(d, text, fs, width):
    """`text` as lines no wider than `width`, broken at WORDS only."""
    lines, cur = [], ''
    for w in text.split(' '):
        cand = (cur + ' ' + w) if cur else w
        if cur and d.textlength(cand, font=fs) > width:
            lines.append(cur)
            cur = w
        else:
            cur = cand
    if cur:
        lines.append(cur)
    return lines


def _legend_lines(d, items, fs, width):
    """Legend items laid out WHOLE: `[(x_off, line, item, text_lines)]` and
    the line count. Items share a line while they fit; an item too wide for
    the panel (a flag's headline) starts its own line and WRAPS at words
    beside its swatch -- never cut, never ellipsised."""
    sw = max(6, fs.size - 4)
    out, x, line = [], 0, 0
    for it in items:
        wid = sw + 3 + d.textlength(it[1], font=fs)
        if wid <= width:
            if x > 0 and x + wid > width:
                x, line = 0, line + 1
            out.append((x, line, it, [it[1]]))
            x += wid + 10
            continue
        if x > 0:
            line += 1
        tl = _wrap_words(d, it[1], fs, width - sw - 3)
        out.append((0, line, it, tl))
        line += len(tl) - 1
        x = width           # the next item starts a new line
    return out, (line + 1 if items else 0)


def _min_width(d, spec, fs):
    """The narrowest panel that renders every title, footer line and legend
    WORD whole (legend items wrap at words; see `_legend_lines`)."""
    title, leg, foot = spec
    sw = max(6, fs.size - 4)
    w = max([d.textlength(title, font=fs)]
            + [sw + 3 + d.textlength(word, font=fs)
               for _r, t, _k in leg for word in t.split(' ')]
            + [d.textlength(t, font=fs) for t in foot])
    return int(math.ceil(w)) + 6


def _gutter(d, fs):
    return int(math.ceil(d.textlength('9.5k', font=fs))) + 6


def _chrome_h(d, spec, fs, width, flag_levels=0):
    """The panel's height outside its plot -- exactly what `_frame` lays
    out: title, legend lines, the flag lane, gaps, footer lines."""
    lh = fs.size + 3
    _l, n = _legend_lines(d, spec[1], fs, width - 4)
    return lh + n * lh + flag_levels * lh + 6 + len(spec[2]) * lh + 4


def _flag_levels(track):
    """How many lane rows the flags need: flags on one beat STACK."""
    per = {}
    for fl in track.flags:
        per[fl.beat] = per.get(fl.beat, 0) + 1
    return max(per.values()) if per else 0


def _header_lines(d, track, dropped, fs, width):
    """The header's lines: PLACEMENT, what x is, and which panels were
    dropped -- each segment WHOLE, the longest variant that fits, wrapped.
    The right of the first line is reserved for 'placement settled'."""
    if track.x_domain:
        hrs = (track.x_domain[1] - track.x_domain[0]) / 3600.0
        xs = ['x = run time, %.1f h' % hrs, 'x = run time']
    else:
        xs = ['x = board order (no run clock)', 'x = board order']
    variants = [xs]
    if dropped:
        names = '/'.join(n.upper() for n in dropped)
        variants.append(['%s dropped: too narrow' % names,
                         '%s dropped' % names])
    right = d.textlength('placement settled', font=fs) + 12
    segs = ['PLACEMENT']
    for vs in variants:
        segs.append(next((v for v in vs
                          if d.textlength(v, font=fs) <= width - right),
                         vs[-1]))
    lines, cur = [], ''
    for s in segs:
        cand = (cur + '   ' + s) if cur else s
        lim = width - (right if not lines else 0)
        if cur and d.textlength(cand, font=fs) > lim:
            lines.append(cur)
            cur = s
        else:
            cur = cand
    if cur:
        lines.append(cur)
    return lines


_FIT_CACHE: Dict[tuple, Optional[Fit]] = {}


def fit(track, width, frame_h, max_h=None, d=None):
    """The panels a box `width` wide can hold READABLY: the most panels (in
    PRIORITY order) whose every text renders whole, at the largest font
    from the type scale down to 9 px that keeps each plot >= PLOT_MIN_PX
    within `max_h`. None when not even one panel fits."""
    if track is None or not track.beats:
        return None
    specs = _specs(track)
    levels = _flag_levels(track)
    # keyed on what the layout READS, never on id(): a collected track's id
    # is reused by the next one
    key = (repr(sorted(specs.items())), levels, track.x_domain, int(width),
           int(frame_h), max_h)
    if key in _FIT_CACHE:
        return _FIT_CACHE[key]
    import render_chrome
    from route_render import load_font
    d = d or _measure_draw()
    g = render_chrome.gutter_px(width)
    fs0 = _font_px(frame_h)
    out = None
    for n in (3, 2, 1):
        names = tuple(p for p in ORDER if p in PRIORITY[:n])
        pw = (width - (n + 1) * g) // n
        for px in range(fs0, 8, -1):
            fs = load_font(px)
            if any(_min_width(d, specs[p], fs) > pw
                   or pw - 2 * _gutter(d, fs) < 40 for p in names):
                continue
            head = len(_header_lines(d, track, PRIORITY[n:], fs,
                                     width - 2 * g))
            need = (2 * g + head * (fs.size + 3) + 2
                    + max(_chrome_h(d, specs[p], fs, pw,
                                    levels if p == 'intent' else 0)
                          for p in names) + PLOT_MIN_PX)
            if max_h is None or need <= max_h:
                out = Fit(names, px, pw, need, PRIORITY[n:])
                break
        if out is not None:
            break
    if len(_FIT_CACHE) > 256:
        _FIT_CACHE.clear()
    _FIT_CACHE[key] = out
    return out


def plan_band(track, W, H, verdict):
    """How tall the band must be for this frame, and how it splits.

    Side by side (placement left, verdict right) when all three panels fit
    in one of `SIDE_FRACS` of the width; else stacked, placement on top. The
    band never exceeds `BAND_MAX_FRAC` of the frame: the verdict graph gives
    up height down to its own floor first, and past that the panels are
    DECLINED (`mode == 'declined'`, with why) rather than drawn unreadable."""
    import movie_attempts as MA
    cap = int(BAND_MAX_FRAC * H)
    vh = MA.band_height(W, H) if verdict else 0
    if verdict and not vh:
        verdict = False
    d = _measure_draw()
    if verdict:
        for frac in SIDE_FRACS:
            f = fit(track, int(W * frac), H, d=d)
            if f is not None and len(f.names) == 3:
                h = max(vh, f.need_h)
                if h <= cap:
                    return BandPlan(h + h % 2, 'side', h, h,
                                    '3 panels beside the verdict')
    f = fit(track, W, H, d=d)
    if f is None:
        return BandPlan(vh, 'declined', vh, 0,
                        'the frame is too narrow for one readable panel')
    if not verdict:
        if f.need_h > cap:
            return BandPlan(0, 'declined', 0, 0,
                            'panels need %d px, the band may take %d'
                            % (f.need_h, cap))
        return BandPlan(f.need_h + f.need_h % 2, 'placement', 0, f.need_h,
                        '%d panel(s)' % len(f.names))
    if vh + f.need_h > cap:
        vh = max(MA.BAND_MIN_PX, cap - f.need_h)
    if vh + f.need_h > cap:
        full = MA.band_height(W, H)
        return BandPlan(full, 'declined', full, 0,
                        'panels need %d px above a %d px verdict graph, the '
                        'band may take %d' % (f.need_h, vh, cap))
    h = vh + f.need_h
    return BandPlan(h + h % 2, 'stacked', vh, f.need_h,
                    '%d panel(s) above the verdict' % len(f.names))


def band_px(track, verdict):
    """The callable `build_boards(attempts_band=)` sizes the band with. Its
    `.plans` list keeps every plan it made, so the caller can say whether
    the panels were declined."""
    plans = []

    def _px(W, H):
        p = plan_band(track, W, H, verdict)
        plans.append(p)
        return p.band_h
    _px.plans = plans
    return _px


def split_band(box, both, track=None, frame_h=None):
    """`(placement_box, verdict_box)` inside the reserved band.

    With both, and a track to fit: SIDE by side when three panels fit in one
    of `SIDE_FRACS` of the width at this height (the same order `plan_band`
    tries), else STACKED with placement on top at exactly the height its
    panels need, and the verdict below. With no track (a caller that planned
    no panels) the old proportions. With placement only, it takes the band;
    with no box, `(None, None)`."""
    if box is None:
        return None, None
    if not both:
        return box, None
    if track is not None:
        import movie_attempts as MA
        fh = frame_h or box.h * 6
        d = _measure_draw()
        for frac in SIDE_FRACS:
            pw = int(box.w * frac)
            f = fit(track, pw, fh, max_h=box.h, d=d)
            if f is not None and len(f.names) == 3:
                return box._replace(w=pw), box._replace(x=box.x + pw,
                                                        w=box.w - pw)
        f = fit(track, box.w, fh, max_h=box.h - MA.BAND_MIN_PX, d=d)
        if f is None:
            return box, None
        return (box._replace(h=f.need_h),
                box._replace(y=box.y + f.need_h, h=box.h - f.need_h))
    if box.w >= 5 * box.h:
        pw = int(box.w * SIDE_FRAC)
        return box._replace(w=pw), box._replace(x=box.x + pw, w=box.w - pw)
    ph = int(box.h * 0.48)
    return box._replace(h=ph), box._replace(y=box.y + ph, h=box.h - ph)


# ---------------------------------------------------------------------------
# drawing
# ---------------------------------------------------------------------------
class _Texts(object):
    """Every text the panels draw, with its box and panel, so none overlaps
    another and each stays inside its panel (`debug['texts']`)."""

    def __init__(self, d):
        self.d, self.boxes = d, []

    def bbox(self, xy, txt, fs, anchor='la'):
        try:
            return tuple(self.d.textbbox(xy, txt, font=fs, anchor=anchor))
        except Exception:                                      # noqa: BLE001
            w = self.d.textlength(txt, font=fs)
            x = xy[0] - (w if anchor[0] == 'r' else 0)
            return (x, xy[1], x + w, xy[1] + fs.size)

    def free(self, rect, inside=None):
        if inside is not None and not (
                rect[0] >= inside.x and rect[1] >= inside.y
                and rect[2] <= inside.x + inside.w
                and rect[3] <= inside.y + inside.h):
            return False
        return not any(not (rect[2] <= o[0] or o[2] <= rect[0]
                            or rect[3] <= o[1] or o[3] <= rect[1])
                       for o, _t, _p in self.boxes)

    def put(self, xy, txt, fs, fill, anchor='la', panel='', must=True,
            inside=None):
        """Draw `txt`. A `must` text was given its room by the layout; an
        optional one (a tick, a value label) is skipped when it would
        overlap another or leave `inside`."""
        r = self.bbox(xy, txt, fs, anchor)
        if not must and not self.free(r, inside):
            return False
        self.d.text(xy, txt, fill=fill, font=fs, anchor=anchor)
        self.boxes.append((r, txt, panel))
        return True


def draw_panels(d, box, track, *, cur=None, theme=None, frame_h=720,
                routing=False, debug=None):
    """The placement panels in `box`: a header line, then each panel with
    its OWN y axis and unit; nothing is summed. `cur` is the beat on screen
    (None before the first has landed: chrome only, no point, no flag).
    Returns True when drawn.

    **Never half-drawn**: a failure repaints the box and says so in one
    line, the way `movie_attempts.draw_track` falls back."""
    if track is None or not track.beats or box is None or box.w < 60:
        return False
    try:
        return _draw_panels(d, box, track, cur, theme, frame_h, routing,
                            debug)
    except Exception as exc:                                   # noqa: BLE001
        try:
            import render_theme
            from route_render import load_font
            th = render_theme.theme(theme, strict=False)
            d.rectangle([box.x, box.y, box.x + box.w - 1, box.y + box.h - 1],
                        fill=th.rgb('chrome_panel'),
                        outline=th.rgb('chrome_panel_edge'))
            d.text((box.x + 6, box.y + 4), 'placement panels: not drawn (%s)'
                   % type(exc).__name__, fill=th.rgb('chrome_error'),
                   font=load_font(9))
        except Exception:                                      # noqa: BLE001
            pass
        if debug is not None:
            debug['error'] = repr(exc)
        return False


def _draw_panels(d, box, track, cur, theme, frame_h, routing, debug):
    import render_chrome
    import render_theme
    from route_render import load_font
    th = render_theme.theme(theme, strict=False)
    f = fit(track, box.w, frame_h, max_h=box.h, d=d)
    if f is None:
        if debug is not None:
            debug['declined'] = 'box %dx%d holds no readable panel' % (
                box.w, box.h)
        return False
    fs = load_font(f.fs)
    g = render_chrome.gutter_px(box.w)
    d.rectangle([box.x, box.y, box.x + box.w - 1, box.y + box.h - 1],
                fill=th.rgb('chrome_panel'), outline=th.rgb('chrome_panel_edge'))
    T = _Texts(d)
    lh = fs.size + 3
    hl = _header_lines(d, track, f.dropped, fs, box.w - 2 * g)
    for k, line in enumerate(hl):
        T.put((box.x + g, box.y + g + k * lh), line, fs,
              th.rgb('chrome_text_dim'), panel='header')
    if routing:
        T.put((box.x + box.w - g, box.y + g), 'placement settled', fs,
              th.rgb('chrome_text_faint'), anchor='ra', panel='header')
    top = box.y + g + len(hl) * lh + 2
    subs = {}
    for k, name in enumerate(f.names):
        subs[name] = box._replace(x=box.x + g + k * (f.pw + g), y=top,
                                  w=f.pw, h=box.y + box.h - g - top)
    specs = _specs(track)
    if debug is not None:
        debug.update({'panels': [subs[p] for p in f.names], 'names': f.names,
                      'dropped': f.dropped, 'header': hl, 'series': {},
                      'titles': [], 'legends': [], 'footers': [],
                      'plots': {}, 'font': f.fs, 'box': box,
                      'x_mode': 'time' if track.x_domain else 'index'})
    for name in f.names:
        sub = subs[name]
        plot, lane = _frame(d, T, sub, specs[name], th, fs, debug, name,
                            _flag_levels(track) if name == 'intent' else 0)
        if debug is not None:
            debug['plots'][name] = plot
        if plot[3] - plot[1] < PLOT_MIN_PX:
            raise ValueError('plot %s is %d px' % (name, plot[3] - plot[1]))
        xs = _beat_xs(track, plot[0], plot[2])
        _PANEL[name](d, T, sub, plot, lane, xs, track, cur, th, fs, debug)
        # an unmeasured beat is MARKED on the axis, never plotted as zero
        for i, bt in enumerate(track.beats):
            if (bt.unmeasured and name != 'intent' and cur is not None
                    and i <= cur):
                cx, cy = xs[i], plot[3] - 4
                d.line([cx - 3, cy - 3, cx + 3, cy + 3],
                       fill=th.rgb('chrome_error'), width=2)
                d.line([cx - 3, cy + 3, cx + 3, cy - 3],
                       fill=th.rgb('chrome_error'), width=2)
        if cur is not None:
            d.line([xs[cur], plot[1], xs[cur], plot[3]],
                   fill=th.rgb(ROLE['marker']), width=1)
    if debug is not None:
        debug['texts'] = list(T.boxes)
    return True


def _frame(d, T, sub, spec, th, fs, debug, name, flag_levels):
    """A panel's chrome: TITLE, the LEGEND (one swatch per series, each
    naming its unit, wrapped WHOLE), the flag lane, and the FOOTER lines
    (the instrument). Returns `(plot rectangle, flag lane (y0, y1))`."""
    title, leg, foot = spec
    lh = fs.size + 3
    T.put((sub.x + 2, sub.y), title, fs, th.rgb('chrome_text'), panel=name)
    y = sub.y + lh
    sw = max(6, fs.size - 4)
    placed, nlines = _legend_lines(d, leg, fs, sub.w - 4)
    for xo, line, (role, _text, kind), tlines in placed:
        x = sub.x + 2 + xo
        ly = y + line * lh
        cy = ly + fs.size // 2 + 1
        rgb = th.rgb(role)
        if kind == 'dash':
            for xx in range(int(x), int(x + sw), 4):
                d.line([xx, cy, xx + 2, cy], fill=rgb, width=2)
        elif kind == 'flag':
            d.polygon([(x, cy - 4), (x + sw, cy), (x, cy + 4)], fill=rgb)
        else:
            d.rectangle([x, cy - sw // 2, x + sw, cy + sw // 2], fill=rgb)
        for k, tl in enumerate(tlines):
            T.put((x + sw + 3, ly + k * lh), tl, fs,
                  th.rgb('chrome_text_dim'), panel=name)
    y += nlines * lh
    lane = (y, y + flag_levels * lh)
    y = lane[1] + 6
    fy = sub.y + sub.h - len(foot) * lh
    for k, line in enumerate(foot):
        T.put((sub.x + 2, fy + k * lh), line, fs,
              th.rgb('chrome_text_faint'), panel=name)
    gut = _gutter(d, fs)
    right = gut if name == 'arrangement' else 8
    if debug is not None:
        debug['titles'].append(title)
        debug['legends'].append([t for _r, t, _k in leg])
        debug['footers'].append(' '.join(foot))
    return (sub.x + gut, y, sub.x + sub.w - right, fy - 4), lane


#: Beats closer than this on a run-time axis are spread to it, so each board
#: keeps a point of its own: run 32's pile and placed_v2 are 1400 s apart on
#: a 31.9 h axis -- 0.2 px at panel width, one dot where there are two.
MIN_BEAT_PX = 8


def _beat_xs(track, x0, x1):
    """Each beat's x: its RUN TIME over the shared domain (spread to
    `MIN_BEAT_PX` where boards would share a pixel), else its order."""
    n = len(track.beats)
    if track.x_domain:
        t0, t1 = track.x_domain
        xs = [x0 + (x1 - x0) * ((bt.t - t0) / float(t1 - t0))
              for bt in track.beats]
        for i in range(1, n):
            xs[i] = max(xs[i], xs[i - 1] + MIN_BEAT_PX)
        for i in range(n - 1, -1, -1):
            xs[i] = min(xs[i], x1 - (n - 1 - i) * MIN_BEAT_PX)
        return xs
    if n <= 1:
        return [(x0 + x1) / 2.0]
    return [x0 + (x1 - x0) * i / float(n - 1) for i in range(n)]


def _series(d, pts, xs, Y, rgb, visible=None):
    """A STEP line, one dot per board. `visible` is the beat on screen;
    None draws NOTHING -- before the first beat lands, the future stays
    hidden."""
    if visible is None:
        return
    last = None
    for i, v in enumerate(pts):
        if v is None or i > visible:
            continue
        p = (xs[i], Y(v))
        if last is not None:
            d.line([last[0], last[1], p[0], last[1]], fill=rgb, width=2)
            d.line([p[0], last[1], p[0], p[1]], fill=rgb, width=2)
        d.ellipse([p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3], fill=rgb)
        last = p


def _legality(d, T, sub, plot, lane, xs, track, cur, th, fs, debug):
    x0, y0, x1, y1 = plot
    series = [('off-outline parts', [b.off_outline for b in track.beats]),
              ('conflict pairs', [b.conflict_pairs for b in track.beats]),
              ('overlap mm2', [b.overlap_mm2 for b in track.beats])]
    vals = [v for _n, s in series for v in s if v is not None]
    vmax = max(vals or [1.0])
    top = math.log10(1.0 + max(1.0, vmax))

    def Y(v):
        return y1 - (y1 - y0) * (math.log10(1.0 + max(0.0, v))
                                 / max(1e-9, top))
    for name, s in series:
        _series(d, s, xs, Y, th.rgb(ROLE[name]), visible=cur)
        if debug is not None:
            debug['series'][name] = list(s)
    # the axis labels are DATA: the largest value any beat has, and zero
    faint = th.rgb('chrome_text_faint')
    if vals:
        T.put((sub.x + 2, y0), _fmt(vmax), fs, faint, panel='legality',
              must=False, inside=sub)
    T.put((sub.x + 2, y1 - fs.size), '0', fs, faint, panel='legality',
          must=False, inside=sub)
    floor = _floor(track)
    if floor is not None:
        fy = Y(floor)
        for xx in range(int(x0), int(x1), 8):
            d.line([xx, fy, xx + 4, fy], fill=th.rgb(ROLE['floor']))
        if debug is not None:
            debug['floor'] = floor


def _arrangement(d, T, sub, plot, lane, xs, track, cur, th, fs, debug):
    x0, y0, x1, y1 = plot
    bench = track.benchmark or {}
    for name, key, side in (('crossings', 'crossings', 'l'),
                            ('hpwl mm', 'hpwl', 'r')):
        s = [getattr(b, key) for b in track.beats]
        vals = [v for v in s if v is not None]
        if bench.get(key) is not None:
            vals.append(bench[key])
        # EACH SERIES ITS OWN AXIS (left crossings, right hpwl mm), each
        # topped at a ROUND tick: the two units are never on one scale.
        lo, hi = 0.0, _nice_ceil(max(vals or [1.0]))
        if debug is not None:
            debug.setdefault('axes', {})[name] = (lo, hi, side)

        def Y(v, lo=lo, hi=hi):
            return y1 - (y1 - y0) * ((v - lo) / max(1e-9, hi - lo))
        rgb = th.rgb(ROLE['airwire crossings' if key == 'crossings'
                          else 'hpwl mm'])
        _series(d, s, xs, Y, rgb, visible=cur)
        tx = sub.x + 2 if side == 'l' else sub.x + sub.w - 2
        anc = 'la' if side == 'l' else 'ra'
        T.put((tx, y0), _fmt(hi), fs, rgb, anchor=anc, panel='arrangement',
              must=False, inside=sub)
        if bench.get(key) is not None:
            by = Y(bench[key])
            for xx in range(int(x0), int(x1), 9):
                d.line([xx, by, xx + 5, by], fill=rgb, width=1)
            # the benchmark's VALUE, in its own axis gutter, when free
            T.put((tx, by - fs.size // 2), _fmt(bench[key]), fs, rgb,
                  anchor=anc, panel='arrangement', must=False, inside=sub)
        if debug is not None:
            debug['series'][name] = list(s)
            debug.setdefault('benchmark', {})[name] = bench.get(key)


def _intent(d, T, sub, plot, lane, xs, track, cur, th, fs, debug):
    x0, y0, x1, y1 = plot
    s = [b.floorplan for b in track.beats]
    vals = [v for v in s if v is not None]
    faint = th.rgb('chrome_text_faint')
    if debug is not None:
        debug['series']['floorplan errors'] = list(s)
        debug['flags'] = []
    if not vals:
        # NO NUMBER is not zero, and not an axis either
        T.put(((x0 + x1) / 2.0, (y0 + y1) / 2.0 - fs.size / 2.0),
              'unmeasured', fs, th.rgb('chrome_text_dim'), anchor='ma',
              panel='intent')
    else:
        hi = _nice_ceil(max(vals))

        def Y(v):
            return y1 - (y1 - y0) * (v / hi)
        _series(d, s, xs, Y, th.rgb(ROLE['floorplan errors']), visible=cur)
        T.put((sub.x + 2, y0), _fmt(hi), fs, faint, panel='intent',
              must=False, inside=sub)
        T.put((sub.x + 2, y1 - fs.size), '0', fs, faint, panel='intent',
              must=False, inside=sub)
    if cur is None:
        return
    # THE FLAGS, in their own lane above the plot -- never on a series line
    # -- numbered as in the legend, flags on one beat STACKED.
    lh = fs.size + 3
    level = {}
    for k, fl in enumerate(track.flags):
        if fl.beat > cur:
            continue
        lv = level.get(fl.beat, 0)
        level[fl.beat] = lv + 1
        fx = xs[fl.beat]
        ty = lane[0] + lv * lh
        cy = ty + fs.size // 2 + 1
        d.polygon([(fx, cy - 4), (fx + 7, cy), (fx, cy + 4)],
                  fill=th.rgb(ROLE['flag']))
        if not T.put((fx + 9, ty), str(k + 1), fs, th.rgb(ROLE['flag']),
                     panel='intent', must=False, inside=sub):
            T.put((fx - 3, ty), str(k + 1), fs, th.rgb(ROLE['flag']),
                  anchor='ra', panel='intent', must=False, inside=sub)
        if debug is not None:
            debug['flags'].append((fl.beat, fl.text, lv))


_PANEL = {'legality': _legality, 'arrangement': _arrangement,
          'intent': _intent}


# ---------------------------------------------------------------------------
# composition
# ---------------------------------------------------------------------------
def with_firsts(track, marks, lands=None):
    """The track with each beat's frame read off build_boards: its LANDING
    frame when the step glided (`lands`, so a glide shows the source board's
    numbers until the parts land -- as the inventory does), else the step's
    first frame. The track is measured BEFORE the frame is planned, so the
    region can be reserved, and learns its frames after."""
    if track is None:
        return None
    firsts = {}
    for mk in marks or ():
        firsts.setdefault(os.path.normcase(os.path.abspath(mk[1])), mk[2])
    lands = lands or {}

    def _first(b):
        nb = os.path.normcase(os.path.abspath(b.board))
        return lands.get(nb, firsts.get(nb, b.first))
    return track._replace(beats=tuple(b._replace(first=_first(b))
                                      for b in track.beats))


def routing_from(track, marks):
    """The first frame of the first NON-placement step after the last beat,
    or None: from there on the panels say the placement is settled."""
    if track is None or not marks:
        return None
    beat_boards = {os.path.normcase(os.path.abspath(b.board))
                   for b in track.beats}
    last = max(b.first for b in track.beats)
    for mk in marks:
        if (mk[2] > last and os.path.normcase(os.path.abspath(mk[1]))
                not in beat_boards):
            return mk[2]
    return None


def compose(frames, box, track, marks, theme, frame_h):
    """Draw the panels into `box` on every frame, as a per-frame transform
    (streamed on a spool, in place on a list)."""
    import frame_spool
    rf = routing_from(track, marks)

    def _fn(i, f):
        from PIL import ImageDraw
        cur = beat_at(track, i)
        draw_panels(ImageDraw.Draw(f), box, track, cur=cur, theme=theme,
                    frame_h=frame_h, routing=(rf is not None and i >= rf))
        return f
    return frame_spool.transform(frames, _fn)


def status_line(track, why, plan=None):
    if track is None:
        return 'placement panels: not drawn -- %s' % why
    b = track.benchmark_name or 'no benchmark'
    x = 'x run time' if track.x_domain else 'x board order (no run clock)'
    p = (', %s' % plan.why) if plan is not None and plan.why else ''
    return ('placement panels: %d placement board(s), benchmark %s, %d '
            'flag(s), %s, intent from %s%s%s'
            % (len(track.beats), b, len(track.flags), x,
               track.floorplan_source or 'nothing (unmeasured)', p,
               ('; ' + '; '.join(track.notes)) if track.notes else ''))


def placed_anything(track, steps):
    """True when placement HAPPENED in this chain: at least two placement
    beats, and some part moved between them. `build_track` already refuses
    a chain that fails this -- before measuring -- so a track it returned
    passes; kept for callers holding a track from elsewhere."""
    if track is None or len(track.beats) < 2:
        return False
    return _poses_moved([b.board for b in track.beats])
