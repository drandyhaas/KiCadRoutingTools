#!/usr/bin/env python3
"""Stack a 3D isometric panel under the movie's X-ray panel (#887).

``movie_camera`` plans WHERE TO LOOK; this plans WHAT IS BESIDE IT. Same posture
as its neighbour: ``plan_iso_shots`` and ``board_for_frames`` are pure functions
over frame indices -- no PIL, no subprocess, no board reads -- so the cadence can
be tested to the frame before anything is drawn or rendered.

The composition runs on the FRAME LIST, between ``build_boards`` and
``save_movie``, exactly where ``py_tools/make_film.build_film`` badges and
splices. That placement is forced, not chosen:

* ``tests/test_431_placement_movie.py:92`` asserts exactly ONE ``BoardRenderer``
  is constructed on the no-stage path, so the panel may not build a second one;
* compositing any earlier would be deformed by ``Stage._snap``'s mirror and by
  ``_emit_flip``'s horizontal squash (``movie_camera.py:527-582``), both of which
  operate on whole frames and would squash the 3D view along with the board.

**THE INVARIANT.** Every frame handed to ``save_movie`` must be the same size,
and the reason is worse than an exception: **nothing raises.**

Measured on this repo's Pillow (12.1.1): saving a GIF whose frames differ in
size does NOT raise. Pillow writes a valid file in which every later frame has
been silently resized to the first frame's, so a mixed-size film comes out
looking almost right and quietly distorted. `_write_mp4`
(``animate_route.py:373-404``) does fail on a size change -- it prints
``mp4 encode failed (...); falling back to GIF``, so that half is audible -- and
the GIF it falls back to then absorbs the mismatch without a word.

(An earlier version of this docstring said the Pillow path "raises
``ValueError: images do not match`` uncaught". It does not, on any of three
orderings tried. The invariant is if anything MORE worth keeping for that: a
loud crash you would notice; a silently squashed frame you would not.)

So the decision to show two panels is taken ONCE, up front, on evidence that a
render actually works -- and after that the box height is fixed and a failed
render keeps its box with the reason written inside it, rather than dropping a
panel mid-list.

**Why this is not wired into ``py_tools/make_film.py``.**
``tests/test_film_composition.py:158-159`` probes pixel ``(0, f.height // 2)``
for the ``TRIED`` badge colour. Stacking a panel moves that probe point into the
iso box and turns that test red for a reason that has nothing to do with
badging. Anyone adding panels to the film needs to move that assertion first.
"""
from __future__ import annotations

import bisect
import collections
import math
import os
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

#: Panel background and caption strip, matching make_film._card_frame so the
#: film and the movie do not drift into two different dark greys.
_PANEL_BG = (14, 14, 18)
_STRIP_BG = (28, 28, 34)
_STRIP_FG = (228, 228, 236)


def _finite(value, default, name):
    """float(value) when it is finite, else `default` with a warning.

    A refusal would be worse here: the movie is an artifact, and aborting a
    routing run because a tuning knob was mistyped trades a cosmetic problem for
    a real one. Saying so on stderr is the middle.
    """
    try:
        v = float(value)
    except (TypeError, ValueError):
        v = None
    if v is None or not math.isfinite(v):
        print('movie_panels: %s=%r is not a finite number; using %g'
              % (name, value, default), file=sys.stderr)
        return default
    return v


class IsoOpts(object):
    """Tuning for the iso panel. Only ``make_movie.main()`` builds one.

    ``max_renders`` is THE cost cap, and it is a COUNT rather than a number of
    seconds on purpose: a count is deterministic, so the same chain composes the
    same movie on a fast machine and a slow one. (The per-render timeout in
    ``kicad_iso_render`` is a hang guard, a different thing.) At the default 24,
    a chain costs about 24 x 2.5 s over 4 workers, roughly 15 s, against a
    single-panel movie of about a second -- which is the trade this feature is,
    and why it is off by default.
    """

    __slots__ = ('max_renders', 'height_frac', 'yaw0_deg', 'sweep_deg',
                 'tilt_deg', 'quality', 'floor', 'perspective', 'zoom', 'jobs',
                 'timeout', 'cli', 'keep_dir')

    def __init__(self, max_renders=24, height_frac=0.62, yaw0_deg=45.0,
                 sweep_deg=60.0, tilt_deg=-45.0, quality='basic', floor=False,
                 perspective=False, zoom=None, jobs=None, timeout=120.0,
                 cli=None, keep_dir=None):
        self.max_renders = int(max_renders)
        # FINITE, checked here rather than trusted. argparse's `type=float`
        # happily accepts `nan` and `inf`, and make_movie's main() catches only
        # FileNotFoundError -- so `--iso-height-frac nan` came out of
        # panel_geometry as `ValueError: cannot convert float NaN to integer`
        # and took the whole movie down. That is precisely the "not crash" half
        # of this module's stated contract. 0.0, 0.01, -1.0 and 5.0 are all
        # legitimate and are handled downstream by the 48-px floor.
        self.height_frac = _finite(height_frac, 0.62, 'height_frac')
        self.yaw0_deg = _finite(yaw0_deg, 45.0, 'yaw0_deg')
        self.sweep_deg = _finite(sweep_deg, 60.0, 'sweep_deg')
        self.tilt_deg = _finite(tilt_deg, -45.0, 'tilt_deg')
        self.quality = quality
        self.floor = bool(floor)
        # Perspective is OFF by default even though the hand-typed recipe in
        # wk/run24/esp_prog/iso_render_cmd.txt uses it: orthographic keeps the
        # board's apparent size steady across the yaw sweep, where perspective
        # makes it breathe.
        self.perspective = bool(perspective)
        self.zoom = zoom
        self.jobs = jobs
        self.timeout = float(timeout)
        self.cli = cli
        self.keep_dir = keep_dir

    def __repr__(self):
        return ('IsoOpts(max_renders=%d, height_frac=%g, sweep_deg=%g, '
                'quality=%r)' % (self.max_renders, self.height_frac,
                                 self.sweep_deg, self.quality))


IsoShot = collections.namedtuple('IsoShot', 'board rotate first last')

#: A shot must be held for at least this many frames. The budget buys ROTATION
#: across the film, not a render per frame: at 6 fps three frames is half a
#: second, and paying ~2.5 s of kicad-cli for half a second of screen time is
#: the per-frame cadence this module was written to avoid.
MIN_SHOT_FRAMES = 3


# --------------------------------------------------------------------------
# planning -- pure
# --------------------------------------------------------------------------

def board_for_frames(marks, n_frames, final_board):
    """Which board's 3D view belongs beside each frame. Length ``n_frames``.

    ``marks`` is ``build_boards``' own ``(label, board, first, last)`` per step.
    Two edges matter and both are real frames, not theory:

    * frames BEFORE the first mark are ``build_boards``' own ``"input"``
      snapshot (``animate_route.py:309``), which no step owns -> the FIRST
      step's board, taken from ``marks[0]`` even when that mark is EMPTY.
      A step can contribute zero frames (measured: pairing two unrelated
      boards gave ``marks[0] == (label, board, 1, 1)``), and skipping it here
      opened the film on the second step's 3D view while the X-ray panel still
      showed the starting board -- two panels disagreeing about which board
      this is, which is the one thing a two-panel frame must never do;
    * frames from the last mark on are the final trueup -> ``final``, which is
      not always the last step's board.
    """
    if n_frames <= 0:
        return []
    all_marks = [m for m in (marks or []) if m]
    spans = [m for m in all_marks if m[3] > m[2]]
    if not all_marks:
        return [final_board] * n_frames
    owner = [None] * n_frames
    for _label, board, first, last in spans:
        for i in range(max(0, first), min(n_frames, last)):
            owner[i] = board
    head = all_marks[0][1]
    for i in range(min(n_frames, max(0, all_marks[0][2]))):
        owner[i] = head
    for i in range(n_frames):
        if owner[i] is None:
            owner[i] = final_board
    return owner


def _segments(owner):
    """Consecutive runs of the same board: ``[[board, first, last], ...]``."""
    segs = []
    for i, b in enumerate(owner):
        if segs and segs[-1][0] == b:
            segs[-1][2] = i + 1
        else:
            segs.append([b, i, i + 1])
    return segs


def plan_iso_shots(owner, opts):
    """``(shots, frame_to_shot)``. Pure: no kicad-cli, no PIL, no board reads.

    One shot is one render. The budget is spent in two regimes, and they are the
    SAME code path -- a list of cut points, one shot per cut:

    * more board-segments than budget: keep ``max_renders`` segments evenly
      spaced, always including the first and the last, and hold each across the
      frames until the next kept one;
    * fewer: every segment gets a cut, then the LEFTOVER budget is spent
      splitting the longest spans in half, repeatedly. Without this a two-board
      chain would render twice and sit motionless on an unspent budget of 22.

    Shot ``k`` of ``K`` is rendered at ``yaw0 + sweep * k/(K-1)`` -- one
    monotonic turn across the whole film. That sweep is what makes the bottom
    panel ANIMATED rather than a still: a different ``--rotate`` costs exactly
    the same render, so the motion is free. It has to be, because copper sits
    under soldermask and the 3D view shows no routing progress at all; what it
    shows is the parts moving and the board turning.
    """
    if not owner:
        return [], []
    budget = max(1, int(opts.max_renders))
    segs = _segments(owner)

    if len(segs) > budget:
        # Evenly spaced over the segments, endpoints pinned.
        if budget == 1:
            pick = [0]
        else:
            pick = sorted({int(round(j * (len(segs) - 1) / (budget - 1)))
                           for j in range(budget)})
        cuts = [segs[j][1] for j in pick]
    else:
        cuts = [s[1] for s in segs]
        # Split the longest current span until the budget is spent. Ties break
        # on the earlier span, so the plan is a pure function of `owner`.
        while len(cuts) < budget:
            bounds = cuts + [len(owner)]
            spans = [(bounds[i + 1] - bounds[i], i) for i in range(len(cuts))]
            best_len, best_i = max(spans, key=lambda t: (t[0], -t[1]))
            if best_len < 2 * MIN_SHOT_FRAMES:
                # Stop before a render buys less than a beat. Without this a
                # 3-frame movie spent 3 renders -- one per FRAME, which is the
                # very thing the per-step cadence exists to avoid, and it only
                # showed up on a tiny chain where nobody looks.
                break
            mid = bounds[best_i] + best_len // 2
            if mid in cuts:
                break
            cuts.append(mid)
            cuts.sort()

    cuts = sorted(set(cuts))
    if cuts[0] != 0:
        cuts[0] = 0
    k_total = len(cuts)
    shots = []
    for k, start in enumerate(cuts):
        end = cuts[k + 1] if k + 1 < k_total else len(owner)
        yaw = opts.yaw0_deg
        if k_total > 1:
            yaw += opts.sweep_deg * k / float(k_total - 1)
        shots.append(IsoShot(owner[start], (opts.tilt_deg, 0.0, yaw),
                             start, end))
    frame_to_shot = [bisect.bisect_right(cuts, i) - 1 for i in range(len(owner))]
    return shots, frame_to_shot


def panel_geometry(top_size, height_frac):
    """``(W, H_top, H_iso, H_total)``, with ``H_total`` forced EVEN.

    Even because ``_write_mp4`` crops every frame with ``a.shape[0] & ~1``
    (``animate_route.py:397``); on an odd height that crop would quietly shave a
    row off the caption strip.
    """
    W, H_top = top_size
    H_iso = max(48, int(round(H_top * float(height_frac))))
    total = H_top + H_iso
    if total % 2:
        H_iso += 1
        total += 1
    return W, H_top, H_iso, total


# --------------------------------------------------------------------------
# drawing
# --------------------------------------------------------------------------

def iso_panel(box_wh, png_path, caption, error=''):
    """``(panel, error)``: a foreign PNG letterboxed into an EXACT box, captioned.

    The second return value is what the caller must fold into its failure count.
    An earlier version returned only the image, so a panel that had "could not
    read the render" written across it still counted as a success.

    Same contract as ``make_film._card_frame`` (``py_tools/make_film.py:231``),
    and deliberately so -- fitting an image of untrusted size into a fixed frame
    is the same problem, and it has been solved once here already.

    Two differences. kicad-cli's PNG is RGBA with a transparent background, so it
    is pasted with itself as the mask onto the panel colour rather than being
    flattened onto white. And ``error`` replaces the picture with the reason
    while KEEPING THE BOX -- the whole point, since a shorter frame is the defect
    this module exists to avoid.
    """
    from PIL import Image, ImageDraw
    from route_render import load_font

    W, H = box_wh
    canvas = Image.new('RGB', (W, H), _PANEL_BG)
    strip = max(18, H // 10)
    d = ImageDraw.Draw(canvas)
    drawn_error = error or ''

    if error:
        font = load_font(max(11, strip // 2))
        msg = 'no 3D view: %s' % error
        _wrapped_text(d, font, msg, 10, max(8, H // 3), W - 20, (196, 128, 128))
    elif png_path and os.path.isfile(png_path):
        try:
            im = Image.open(png_path)
            aw = max(1, W - 16)
            ah = max(1, H - strip - 12)
            sc = min(aw / float(im.width), ah / float(im.height))
            new = (max(1, int(im.width * sc)), max(1, int(im.height * sc)))
            im = im.resize(new, Image.LANCZOS)
            x, y = (W - im.width) // 2, max(0, (H - strip - im.height) // 2)
            if im.mode in ('RGBA', 'LA'):
                canvas.paste(im.convert('RGB'), (x, y), im.split()[-1])
            else:
                canvas.paste(im.convert('RGB'), (x, y))
        except Exception as exc:                                # noqa: BLE001
            # DRAWN and REPORTED. Drawing it alone put "could not read the
            # render" into three panels under a status line saying
            # "3 render(s)" with nothing failed -- the composer knew and the
            # report did not, which is exactly the indistinguishable silence
            # the named states exist to end.
            drawn_error = 'could not read the render (%s)' % exc
            font = load_font(max(11, strip // 2))
            _wrapped_text(d, font, drawn_error,
                          10, max(8, H // 3), W - 20, (196, 128, 128))

    d.rectangle([0, H - strip, W, H], fill=_STRIP_BG)
    font = load_font(max(11, strip // 2))
    _clipped_text(d, font, caption or '', 8, H - strip + max(1, strip // 6),
                  W - 16, _STRIP_FG)
    return canvas, drawn_error


def _clipped_text(d, font, text, x, y, avail, fill):
    """Draw one line, ELLIDING rather than running off the edge.

    PIL clips silently at the image edge, and a caption that ends at a
    plausible-looking field reads as the whole story -- the same trap
    ``route_render._label`` documents.
    """
    txt = text
    while txt and _text_w(d, txt, font) > avail:
        txt = txt[:-2]
    if txt != text and len(txt) > 1:
        txt = txt[:-1] + '...'
    d.text((x, y), txt, fill=fill, font=font)


def _wrapped_text(d, font, text, x, y, avail, fill):
    """Draw ``text`` over as many lines as it needs, breaking on spaces."""
    words, lines, cur = text.split(' '), [], ''
    for w in words:
        cand = (cur + ' ' + w) if cur else w
        if cur and _text_w(d, cand, font) > avail:
            lines.append(cur)
            cur = w
        else:
            cur = cand
    if cur:
        lines.append(cur)
    lh = font.size + 4 if hasattr(font, 'size') else 16
    for i, ln in enumerate(lines):
        d.text((x, y + i * lh), ln, fill=fill, font=font)


def _text_w(d, s, font):
    try:
        return d.textlength(s, font=font)
    except Exception:                                           # noqa: BLE001
        return 8 * len(s)


def stack(top, bottom):
    """One image, ``top`` above ``bottom``. Widths must match."""
    from PIL import Image
    if top.width != bottom.width:
        raise ValueError('stack: widths differ (%d vs %d)'
                         % (top.width, bottom.width))
    out = Image.new('RGB', (top.width, top.height + bottom.height), _PANEL_BG)
    out.paste(top.convert('RGB') if top.mode != 'RGB' else top, (0, 0))
    out.paste(bottom, (0, top.height))
    return out


# --------------------------------------------------------------------------
# the decision, and the composition
# --------------------------------------------------------------------------

def _report(state, detail='', **kw):
    r = {'state': state, 'detail': detail, 'shots': [], 'failed': 0,
         'models': None, 'panel_wh': None, 'boards': 0}
    r.update(kw)
    return r


def compose_two_panel(frames, marks, final_board, opts=None, quiet=False):
    """``(frames, report)``. Stack the iso panel under every frame, IN PLACE.

    ``frames`` comes back as the SAME list object with each entry replaced by a
    taller composed image. The X-ray image is dropped as it is overwritten, so
    peak memory is about two frames rather than twice the movie -- a composed
    1000x1620 RGB frame is 4.9 MB, and a 300-frame movie held twice would be
    3 GB.

    When the panel cannot run, ``frames`` is returned COMPLETELY UNTOUCHED --
    same list, same Image objects -- and ``report['state']`` says why. That is
    the degradation contract the issue asks for: "not crash, not silently drop a
    panel, and not change the frame size".
    """
    import kicad_iso_render as kir

    opts = opts or IsoOpts()
    # Reachable only by a direct caller: make_movie returns before this when it
    # has no frames. Kept as a guard on the FUNCTION's contract rather than on
    # one caller's behaviour, and a test calls it here -- deleting it used to
    # survive the suite, which is the same "declared and read by nothing" shape
    # this PR is otherwise about.
    if not frames:
        return frames, _report('not_applicable', 'no frames')
    if opts.max_renders <= 0:
        # Echo the VALUE the caller actually passed. Hardcoding "0" here named a
        # number nobody typed when the value was negative.
        return frames, _report('disabled',
                               '--iso-max-renders %d' % opts.max_renders)

    owner = board_for_frames(marks, len(frames), final_board)
    if not any(owner):
        return frames, _report('not_applicable', 'no chain boards to render')

    cli, why = kir.resolve_cli(opts.cli)
    if not cli:
        return frames, _report('did_not_run', why)

    shots, frame_to_shot = plan_iso_shots(owner, opts)
    W, _H_top, H_iso, _total = panel_geometry(frames[0].size, opts.height_frac)
    req_w = int(round(W * kir._REQUEST_OVERSCAN))
    req_h = int(round(H_iso * kir._REQUEST_OVERSCAN))

    # INSIDE the guard, not above it. These two lines used to sit outside the
    # try, so a read-only or full %TEMP% (PermissionError) or a keep_dir naming
    # an existing FILE (FileExistsError) escaped as a traceback through
    # compose_two_panel and make_movie to the caller -- the one shape this
    # function promises never to take.
    try:
        tmp = opts.keep_dir or tempfile.mkdtemp(prefix='krt_iso_')
        if opts.keep_dir:
            os.makedirs(tmp, exist_ok=True)
    except OSError as exc:
        return frames, _report('error',
                               'could not make a directory for the renders (%s)'
                               % exc)
    render_kw = dict(width=req_w, height=req_h, quality=opts.quality,
                     floor=opts.floor, perspective=opts.perspective,
                     zoom=opts.zoom, timeout=opts.timeout)
    try:
        # THE PROBE. Render the LAST shot synchronously before committing to a
        # taller frame. It is both the availability test and a shot the plan
        # needs anyway, so nothing is wasted -- and the commitment is then made
        # on evidence that a render actually works, not on a binary existing.
        probe_k = len(shots) - 1
        probe_png = os.path.join(tmp, 'iso_%03d.png' % probe_k)
        got, err = kir.render_iso(shots[probe_k].board, probe_png, cli,
                                  rotate=shots[probe_k].rotate, **render_kw)
        if not got:
            return frames, _report('error', err)

        results = {probe_k: (got, '')}
        rest = [(k, s.board, os.path.join(tmp, 'iso_%03d.png' % k), s.rotate)
                for k, s in enumerate(shots) if k != probe_k]
        if rest:
            results.update(kir.render_many(rest, cli, workers=opts.jobs,
                                           **render_kw))

        # PER BOARD, not once for the whole film. The note used to be computed
        # from shots[0].board and pasted under every panel, so a frame showing
        # board B carried board A's model count -- a caption stating a fact
        # about something other than the picture above it. Memoized because a
        # chain revisits the same board many times and resolve_models re-reads
        # the file each call.
        notes = {}

        def _note_for(board):
            if board not in notes:
                m = kir.resolve_models(board, kir.model_dirs(cli, board))
                notes[board] = (m, kir.models_note(m))
            return notes[board]

        panels, errors = {}, {}
        for k, shot in enumerate(shots):
            png, err = results.get(k, (None, 'not rendered'))
            _m, note = _note_for(shot.board)
            cap = '%s  |  yaw %.0f deg  |  %s' % (
                os.path.splitext(os.path.basename(shot.board))[0],
                shot.rotate[2], note)
            # The panel reports back: a PNG that rendered but would not DECODE
            # is a failure the count must see, and it is only discoverable here.
            panels[k], drawn = iso_panel((W, H_iso), png, cap, error=err)
            errors[k] = err or drawn or ''
        failed = sum(1 for e in errors.values() if e)
        # The report's single models figure is the FILM'S OPENING board, and the
        # status line says so; the per-frame truth is in each caption.
        models = notes.get(shots[0].board, ({}, ''))[0]

        for i in range(len(frames)):
            frames[i] = stack(frames[i], panels[frame_to_shot[i]])

        rep = _report('ran', '', shots=[{'board': s.board, 'yaw': s.rotate[2],
                                         'first': s.first, 'last': s.last,
                                         'error': errors.get(k, '')}
                                        for k, s in enumerate(shots)],
                      failed=failed, models=models, panel_wh=(W, H_iso),
                      boards=len({s.board for s in shots}))
        return frames, rep
    finally:
        if not opts.keep_dir:
            import shutil
            shutil.rmtree(tmp, ignore_errors=True)


def iso_status_line(report):
    """One line saying which of the states this run was actually in.

    Modelled on ``check_connected.reconcile_status_line`` for the reason #654
    gives: "ran and agreed", "skipped -- no kicad-cli", "disabled" and "not
    applicable" were one indistinguishable silence until each was given words.
    None of the OFF states may read like success.
    """
    st = (report or {}).get('state', 'not_applicable')
    detail = (report or {}).get('detail') or ''
    if st == 'ran':
        n = len(report.get('shots') or ())
        wh = report.get('panel_wh') or (0, 0)
        line = ('movie: iso panel ON  -- %d render(s) over %d board(s), panel '
                '%dx%d' % (n, report.get('boards') or 0, wh[0], wh[1]))
        if report.get('failed'):
            line += ', %d FAILED (kept the box)' % report['failed']
        import kicad_iso_render as kir
        line += '  |  opening board: ' + kir.models_note(report.get('models'))
        return line
    tail = ' Single X-ray panel, full speed.'
    if st == 'did_not_run':
        return 'movie: iso panel OFF -- %s.%s' % (detail, tail)
    if st == 'disabled':
        return 'movie: iso panel OFF -- DISABLED (%s). Single X-ray panel.' % detail
    if st == 'error':
        return 'movie: iso panel OFF -- UNAVAILABLE (%s).%s' % (detail, tail)
    return 'movie: iso panel OFF -- not applicable (%s).' % (detail or 'nothing to render')
