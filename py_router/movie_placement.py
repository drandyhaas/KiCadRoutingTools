#!/usr/bin/env python3
"""The placement progress panels, in placement currency (#1042).

`movie_attempts` keeps the routed VERDICT on its axis and deliberately keeps
placement proxies off it ("ON A PLACEMENT RUN THE AXIS IS STILL THE ROUTED
RESULT", #1021): a copper-free placement lap scores `blocking` ~250 on that
axis because every net is unrouted, while the thing the lap was doing moved
elsewhere. So placement gets its OWN panels, beside the verdict band and never
on its axis:

  1. **Legality** (log y): pads off the outline (parts), pad-conflict pairs,
     overlap mm² -- with the floor the KiCad-locked parts set, labelled.
  2. **Arrangement vs benchmark**, a SCREEN, not the verdict: airwire
     crossings and hpwl as step lines (each on its own axis), with dashed
     benchmark lines when a benchmark board is given.
  3. **Intent**: check_floorplan error count per beat.

Plus **downstream-defect flags**: a ledger row `kind == classification`,
`shape == placement` flags the placement beat it sent the run back to.

**One point per placement BOARD, never per frame.** A glide's frames are
pixel interpolation, not evaluated placements; a per-frame value would be
invented. Every number is measured on the film's own boards with the named
instrument (`render_placement --json-out`, `check_floorplan`), cached per board
sha within a run. **No placement boards, no panel**; nothing is synthesised.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
from typing import Dict, NamedTuple, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

#: The instrument every legality/arrangement number comes from, named in the
#: legends because the same word means different counts elsewhere:
#: `render_placement`'s `crossings` (3750 airwire crossings on run 32's
#: placed_v3) is not `board_score`'s `pin_order_crossings` (1211 part pairs).
INSTRUMENT = 'render_placement --json-out'


class Beat(NamedTuple):
    """One placement board of the film, measured."""
    board: str
    label: str
    first: int                  # first frame of its step
    off_outline: Optional[int]  # parts with pad copper off the outline
    conflict_pairs: Optional[int]
    overlap_mm2: Optional[float]
    crossings: Optional[int]
    hpwl: Optional[float]
    locked_pairs: Optional[int]  # metrics.locked_contact_pairs: the floor
    floorplan: Optional[int]
    floorplan_source: str        # 'ledger row N' | 'check_floorplan' | ''
    ledger_index: Optional[int]


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


def _sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


_CACHE: Dict[str, dict] = {}


def measure_board(path, cache=None):
    """`render_placement --json-out` on one board: the legality and
    arrangement numbers, keyed by the board's sha so a run measures a board
    once. None when the instrument could not answer."""
    cache = _CACHE if cache is None else cache
    try:
        key = _sha(path)
    except OSError:
        return None
    if key in cache:
        return cache[key]
    tmp = tempfile.mkdtemp(prefix='krt_place_')
    out_json = os.path.join(tmp, 'm.json')
    try:
        subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(_ROOT, 'py_tools', 'render_placement.py'),
                        path, '--json-out', out_json,
                        '-o', os.path.join(tmp, 'm.png'), '--quiet'],
                       capture_output=True, text=True, timeout=600,
                       env=dict(os.environ, KRT_NO_BANNER='1'))
        with open(out_json, encoding='utf-8') as f:
            d = json.load(f)
    except Exception:                                          # noqa: BLE001
        d = None
    finally:
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)
    if not d:
        cache[key] = None
        return None
    m = d.get('metrics') or {}
    off = ((d.get('checklist') or {}).get('a_off_outline') or {}).get(
        'pad_copper')
    res = {'off_outline': len(off) if isinstance(off, list) else None,
           'conflict_pairs': m.get('pad_conflict_pairs'),
           'overlap_mm2': m.get('overlap_area'),
           'crossings': m.get('crossings'),
           'hpwl': m.get('hpwl'),
           'locked_pairs': m.get('locked_contact_pairs')}
    cache[key] = res
    return res


def check_floorplan_errors(path, intent):
    """ERROR-severity violations from `check_floorplan --intent`, the count
    `board_score.score_floorplan` sums. None when it could not grade."""
    if not intent or not os.path.isfile(intent):
        return None
    tmp = tempfile.mkdtemp(prefix='krt_fp_')
    out_json = os.path.join(tmp, 'f.json')
    try:
        subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(_ROOT, 'py_tools', 'check_floorplan.py'),
                        path, '--intent', intent, '--json', out_json,
                        '--exit-zero', '-q', '--allow-unplaced'],
                       capture_output=True, text=True, timeout=600,
                       env=dict(os.environ, KRT_NO_BANNER='1'))
        with open(out_json, encoding='utf-8') as f:
            d = json.load(f)
    except Exception:                                          # noqa: BLE001
        return None
    finally:
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)
    viols = d.get('violations') or []
    return sum(1 for v in viols if (v.get('severity') or 'error') == 'error')


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


def _ledger_match(sha, rows):
    """`(floorplan, row_index)` for a board: the row that PRODUCED it, else a
    row that started FROM it (a later lock re-saves a board with identical
    poses, so the producing row can carry another sha). (None, None) when no
    row names it -- the pile has none."""
    def fp(r):
        bb = (r.get('score') or {}).get('blocking_by') \
            if isinstance(r.get('score'), dict) else None
        return (bb or {}).get('floorplan')
    for key in ('result_sha', 'parent_sha'):
        for i, r in enumerate(rows):
            if r.get(key) == sha and fp(r) is not None:
                return fp(r), int(r.get('iteration', i))
    idx = next((int(r.get('iteration', i)) for i, r in enumerate(rows)
                if sha in (r.get('result_sha'), r.get('parent_sha'))), None)
    return None, idx


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


def build_track(steps, marks, *, ledger=None, benchmark=None, intent=None,
                cache=None, quiet=False):
    """The placement track for a chain, or `(None, why)`.

    `steps` are the chain's `(label, board, ...)`; `marks` are build_boards'
    `(label, board, first, last)` -- every beat keeps the frame it starts on.
    """
    if not steps:
        return None, 'no boards'
    rows = read_ledger(ledger)
    beats = []
    firsts = {}
    for mk in marks or ():
        firsts.setdefault(os.path.normcase(os.path.abspath(mk[1])), mk[2])
    for k, st in enumerate(steps):
        b = st[1]
        nb = os.path.normcase(os.path.abspath(b))
        # a make_film 'revert' step re-shows a board already measured, and a
        # board repeated back to back is one placement, not two
        if len(st) > 3 and st[3] == 'revert':
            continue
        if beats and os.path.normcase(os.path.abspath(beats[-1].board)) == nb:
            continue
        if not is_placement_board(b):
            continue
        m = measure_board(b, cache)
        if m is None:
            continue
        sha = _sha(b)
        fpv, li = _ledger_match(sha, rows)
        src = ('ledger row %d' % li) if fpv is not None else ''
        if fpv is None and intent:
            fpv = check_floorplan_errors(b, intent)
            src = 'check_floorplan' if fpv is not None else ''
        beats.append(Beat(b, str(st[0]), firsts.get(nb, 0),
                          m['off_outline'], m['conflict_pairs'],
                          m['overlap_mm2'], m['crossings'], m['hpwl'],
                          m['locked_pairs'], fpv, src, li))
    if not beats:
        return None, 'no placement boards in this chain'
    bench = measure_board(benchmark, cache) if benchmark else None
    flags = []
    for r in rows:
        if r.get('kind') != 'classification' or r.get('shape') != 'placement':
            continue
        it = int(r.get('iteration', 0))
        tgt = next((i for i, bt in enumerate(beats)
                    if bt.ledger_index is not None and bt.ledger_index > it),
                   None)
        # the lever's FIRST sentence: the flag names the defect, the ledger
        # holds the evidence
        text = str(r.get('lever') or 'placement-shaped').split('. ')[0]
        text = text.split(' [read:')[0].strip()
        if tgt is None:
            tgt = len(beats) - 1
            text = text + ' (after this film)'
        flags.append(Flag(tgt, text, it))
    notes = []
    if not rows:
        notes.append('no ledger: intent from check_floorplan'
                     if intent else 'no ledger and no intent: intent unmeasured')
    return (PlacementTrack(tuple(beats), bench,
                           os.path.splitext(os.path.basename(benchmark))[0]
                           if benchmark else '', tuple(flags), tuple(notes)),
            '%d placement board(s)' % len(beats))


def beat_at(track, frame):
    """The index of the beat frame `frame` shows (the latest beat started at
    or before it), or None before the first."""
    cur = None
    for i, bt in enumerate(track.beats):
        if bt.first <= frame:
            cur = i
    return cur


# ---------------------------------------------------------------------------
# drawing
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


def draw_panels(d, box, track, *, cur=None, theme=None, frame_h=720,
                routing=False, debug=None):
    """The three panels in `box`, left to right. Each has its OWN y axis and
    unit; nothing is summed. `cur` is the beat on screen; during routing the
    marker sits on the last beat and says so. Returns True when drawn."""
    if track is None or not track.beats or box is None or box.w < 60:
        return False
    try:
        import render_chrome
        import render_theme
        from route_render import load_font
        th = render_theme.theme(theme, strict=False)
        g = render_chrome.gutter_px(box.w * 3)
        # the TYPE SCALE's small size, but never larger than a narrow panel
        # can hold: three panels across a 9:16 frame are ~250 px each
        _pw = (box.w - 4 * g) // 3
        fs = load_font(min(render_chrome.type_px('small', frame_h),
                           max(9, _pw // 24)))
        d.rectangle([box.x, box.y, box.x + box.w - 1, box.y + box.h - 1],
                    fill=th.rgb('chrome_panel'),
                    outline=th.rgb('chrome_panel_edge'))
        pw = (box.w - 4 * g) // 3
        subs = [box._replace(x=box.x + g + k * (pw + g), y=box.y + g,
                             w=pw, h=box.h - 2 * g) for k in range(3)]
        if debug is not None:
            debug['panels'] = subs
            debug['series'] = {}
            debug['titles'] = []
        _legality(d, subs[0], track, cur, th, fs, debug)
        _arrangement(d, subs[1], track, cur, th, fs, debug)
        _intent(d, subs[2], track, cur, th, fs, debug)
        if routing and track.beats:
            d.text((box.x + box.w - g, box.y + 2), 'placement settled',
                   fill=th.rgb('chrome_text_faint'), font=fs, anchor='ra')
        return True
    except Exception:                                          # noqa: BLE001
        return False


def _frame(d, sub, title, legend, footer, th, fs, debug):
    """A panel's chrome: its TITLE, a LEGEND line of coloured swatches (one
    per series, each naming its unit), and a FOOTER naming the instrument.
    Returns the plot rectangle between them. Everything shortens by whole
    words, never mid-word."""
    import render_chrome
    d.text((sub.x + 2, sub.y), render_chrome.fit_words(d, title, fs,
                                                         sub.w - 4),
           fill=th.rgb('chrome_text'), font=fs)
    ly = sub.y + fs.size + 3
    x = sub.x + 2
    sw = max(6, fs.size - 4)
    for rgb, text, dashed in legend:
        tw = d.textlength(text, font=fs)
        if x + sw + 4 + tw > sub.x + sub.w - 2 and x > sub.x + 2:
            # WRAP rather than drop: every series keeps its named unit
            x, ly = sub.x + 2, ly + fs.size + 2
        cy = ly + fs.size // 2
        if dashed:
            for xx in range(int(x), int(x + sw), 4):
                d.line([xx, cy, xx + 2, cy], fill=rgb, width=2)
        else:
            d.rectangle([x, cy - sw // 2, x + sw, cy + sw // 2], fill=rgb)
        d.text((x + sw + 3, ly), text, fill=th.rgb('chrome_text_dim'),
               font=fs)
        x += sw + 3 + tw + 10
    fy = sub.y + sub.h - fs.size - 1
    d.text((sub.x + 2, fy), render_chrome.fit_words(d, footer, fs,
                                                    sub.w - 4),
           fill=th.rgb('chrome_text_faint'), font=fs)
    if debug is not None:
        debug['titles'].append(title)
        debug.setdefault('legends', []).append([t for _r, t, _d in legend])
        debug.setdefault('footers', []).append(footer)
    top = ly + fs.size + 5
    return (sub.x + 30, top, sub.x + sub.w - 30, fy - 4)


def _xs(n, x0, x1):
    if n <= 1:
        return [(x0 + x1) / 2.0]
    return [x0 + (x1 - x0) * i / float(n - 1) for i in range(n)]


def _series(d, pts, xs, Y, rgb, dashed=False, visible=None):
    last = None
    for i, v in enumerate(pts):
        if v is None or (visible is not None and i > visible):
            continue
        p = (xs[i], Y(v))
        if last is not None:
            # a STEP line: hold the old value, then drop
            d.line([last[0], last[1], p[0], last[1]], fill=rgb, width=2)
            d.line([p[0], last[1], p[0], p[1]], fill=rgb, width=2)
        d.ellipse([p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3], fill=rgb)
        last = p


def _marker(d, x, y0, y1, th):
    d.line([x, y0, x, y1], fill=th.rgb('pad'), width=1)


def _legality(d, sub, track, cur, th, fs, debug):
    # THE FLOOR the KiCad-locked parts set: when the last beat's conflict
    # pairs are all locked-part contacts, no placement lap can go lower. It
    # is named in the LEGEND, where no series line can cross it.
    lb = track.beats[-1]
    floor = (lb.conflict_pairs if (lb.conflict_pairs is not None
                                   and lb.locked_pairs
                                   and lb.conflict_pairs <= lb.locked_pairs)
             else None)
    leg = [(th.rgb('defect_conflict'), 'off-outline parts', False),
           (th.rgb('op_jump'), 'conflict pairs', False),
           (th.rgb('op_cross'), 'overlap mm2', False)]
    if floor is not None:
        leg.append((th.rgb('chrome_text_dim'),
                    'floor %s = locked parts' % _fmt(floor), True))
    x0, y0, x1, y1 = _frame(d, sub, 'LEGALITY  (log y)', leg, INSTRUMENT,
                            th, fs, debug)
    series = [('off-outline parts', [b.off_outline for b in track.beats],
               'defect_conflict'),
              ('conflict pairs', [b.conflict_pairs for b in track.beats],
               'op_jump'),
              ('overlap mm2', [b.overlap_mm2 for b in track.beats],
               'op_cross')]
    vals = [v for _n, s, _r in series for v in s if v is not None]
    top = math.log10(1.0 + max(vals or [1.0]))

    def Y(v):
        return y1 - (y1 - y0) * (math.log10(1.0 + max(0.0, v))
                                 / max(1e-9, top))
    xs = _xs(len(track.beats), x0, x1)
    for name, s, role in series:
        _series(d, s, xs, Y, th.rgb(role), visible=cur)
        if debug is not None:
            debug['series'][name] = list(s)
    d.text((sub.x + 2, y0), _fmt(10 ** top - 1), fill=th.rgb(
        'chrome_text_faint'), font=fs)
    d.text((sub.x + 2, y1 - fs.size), '0', fill=th.rgb('chrome_text_faint'),
           font=fs)
    if floor is not None:
        fy = Y(floor)
        for xx in range(int(x0), int(x1), 8):
            d.line([xx, fy, xx + 4, fy], fill=th.rgb('chrome_text_dim'))
        if debug is not None:
            debug['floor'] = floor
    if cur is not None:
        _marker(d, xs[cur], y0, y1, th)


def _arrangement(d, sub, track, cur, th, fs, debug):
    _leg = [(th.rgb('op_jump'), 'airwire crossings', False),
            (th.rgb('op_descend'), 'hpwl mm', False)]
    if track.benchmark:
        _leg.append((th.rgb('chrome_text_dim'), track.benchmark_name or
                     'benchmark', True))
    x0, y0, x1, y1 = _frame(
        d, sub, 'ARRANGEMENT  SCREEN, not the verdict', _leg,
        INSTRUMENT + '  (the verdict is the band)', th, fs, debug)
    xs = _xs(len(track.beats), x0, x1)
    bench = track.benchmark or {}
    for k, (name, key, role, side) in enumerate((
            ('crossings', 'crossings', 'op_jump', 'l'),
            ('hpwl mm', 'hpwl', 'op_descend', 'r'))):
        s = [getattr(b, key) for b in track.beats]
        vals = [v for v in s if v is not None]
        if bench.get(key) is not None:
            vals.append(bench[key])
        # EACH SERIES ITS OWN AXIS (left crossings, right hpwl mm): the two
        # units are never put on one scale, and never summed.
        lo, hi = 0.0, max(vals or [1.0]) * 1.08
        if debug is not None:
            debug.setdefault('axes', {})[name] = (lo, hi, side)

        def Y(v, lo=lo, hi=hi):
            return y1 - (y1 - y0) * ((v - lo) / max(1e-9, hi - lo))
        rgb = th.rgb(role)
        _series(d, s, xs, Y, rgb, visible=cur)
        tx = sub.x + 2 if side == 'l' else sub.x + sub.w - 2
        anc = 'la' if side == 'l' else 'ra'
        d.text((tx, y0), _fmt(hi), fill=rgb, font=fs, anchor=anc)
        if bench.get(key) is not None:
            by = Y(bench[key])
            for xx in range(int(x0), int(x1), 9):
                d.line([xx, by, xx + 5, by], fill=rgb, width=1)
            # the VALUE on the line; the benchmark's name is in the legend
            lab = _fmt(bench[key])
            d.text((x0 + 2 if side == 'l' else x1 - 2, by + 2), lab,
                   fill=rgb, font=fs, anchor='la' if side == 'l' else 'ra')
        if debug is not None:
            debug['series'][name] = list(s)
            debug.setdefault('benchmark', {})[name] = bench.get(key)
    if cur is not None:
        _marker(d, xs[cur], y0, y1, th)


def _intent(d, sub, track, cur, th, fs, debug):
    x0, y0, x1, y1 = _frame(
        d, sub, 'INTENT', [(th.rgb('status_best'), 'floorplan errors',
                            False)],
        'check_floorplan (ledger score, else --intent)', th, fs, debug)
    s = [b.floorplan for b in track.beats]
    vals = [v for v in s if v is not None]
    hi = max(vals or [1]) * 1.1 or 1.0

    def Y(v):
        return y1 - (y1 - y0) * (v / hi)
    xs = _xs(len(track.beats), x0, x1)
    _series(d, s, xs, Y, th.rgb('status_best'), visible=cur)
    d.text((sub.x + 2, y0), _fmt(hi), fill=th.rgb('chrome_text_faint'),
           font=fs)
    if debug is not None:
        debug['series']['floorplan errors'] = list(s)
        debug['flags'] = []
    import render_chrome
    for fl in track.flags:
        if cur is not None and fl.beat > cur:
            continue
        fx = xs[fl.beat]
        d.polygon([(fx, y0), (fx + 7, y0 + 4), (fx, y0 + 8)],
                  fill=th.rgb('defect_net_fail'))
        txt = render_chrome.fit_words(d, fl.text, fs, max(40, x1 - fx - 10))
        d.text((fx + 9, y0), txt, fill=th.rgb('defect_net_fail'), font=fs)
        if debug is not None:
            debug['flags'].append((fl.beat, fl.text))
    if cur is not None:
        _marker(d, xs[cur], y0, y1, th)


# ---------------------------------------------------------------------------
# composition
# ---------------------------------------------------------------------------
def with_firsts(track, marks):
    """The track with each beat's first frame read off build_boards' marks
    (the track is measured BEFORE the frame is planned, so the region can be
    reserved, and learns its frames after)."""
    if track is None:
        return None
    firsts = {}
    for mk in marks or ():
        firsts.setdefault(os.path.normcase(os.path.abspath(mk[1])), mk[2])
    beats = tuple(b._replace(first=firsts.get(
        os.path.normcase(os.path.abspath(b.board)), b.first))
        for b in track.beats)
    return track._replace(beats=beats)


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


def split_band(box, both):
    """`(placement_box, verdict_box)` inside the reserved band.

    With both panels, a wide band puts placement on the LEFT and the verdict
    on the right; a band too narrow for three panels beside a graph (under
    ~5:1, the 9:16 frame) stacks them, placement on top. With placement
    only, it takes the band; with no placement, `(None, box)`.
    """
    if box is None:
        return None, None
    if not both:
        return box, None
    if box.w >= 5 * box.h:
        pw = int(box.w * 0.46)
        return box._replace(w=pw), box._replace(x=box.x + pw, w=box.w - pw)
    ph = int(box.h * 0.48)
    return box._replace(h=ph), box._replace(y=box.y + ph, h=box.h - ph)


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


def status_line(track, why):
    if track is None:
        return 'placement panels: not drawn -- %s' % why
    b = track.benchmark_name or 'no benchmark'
    return ('placement panels: %d placement board(s), benchmark %s, %d flag(s)'
            '%s' % (len(track.beats), b, len(track.flags),
                    ('; ' + '; '.join(track.notes)) if track.notes else ''))


def placed_anything(track, steps):
    """True when placement HAPPENED in this chain: at least two placement
    beats, and some part moved between the chain's boards. A routing film
    whose first snapshot happens to be copper-free is not a placement film."""
    if track is None or len(track.beats) < 2:
        return False
    try:
        import movie_camera
        return any(r['moved'] for r in
                   movie_camera.synth_rounds([b.board for b in track.beats]))
    except Exception:                                          # noqa: BLE001
        return False
