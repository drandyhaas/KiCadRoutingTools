#!/usr/bin/env python3
"""make_film.py -- one film of the WHOLE search, not just the part that worked.

`make_movie.py` animates a chain of boards, and the convergence movie is fed
the ACCEPTED boards deliberately: a reverted iteration spliced into that
sequence animates a change that was undone, which reads as the router thrashing
when it was doing the opposite. That rule is right, and this does not change it.

But it leaves three things off the film, and they are most of what a run
actually consisted of:

  * **the first placements** -- the parts arriving on the board. A placement
    step changes no copper, and the routing movie animates copper deltas, so
    the step that decides everything downstream rendered as a single frame.
  * **the attempts that were not kept** -- on one run the accepted spine was 10
    boards out of 45 on disk. The 35 that were tried and dropped are where the
    search actually happened, and nothing could see them.
  * **the diagnostics** -- the delta renders, focus panels and score plots that
    were the INPUT to each decision. They sat beside the film as loose PNGs, so
    reading "why did it do that" meant opening files in another window.

This composes all three into one artifact. An attempt is shown and then
explicitly undone, badged and captioned as such, so it reads as a search that
considered something and moved on -- which is what happened -- rather than as
copper thrashing. Diagnostics are spliced in as cards where they were produced.

Everything is rendered in ONE pass so a single scale holds across the film;
per-segment renders restart from an empty board and re-reveal the copper, which
looks like the board redrawing itself between beats.

    # the whole search from a place_route_loop work dir, attempts included
    python3 make_film.py --from-loop-dir wk/ -o film.gif

    # a hand-driven chain, marking which boards were dead ends
    python3 make_film.py seed.kicad_pcb placed.kicad_pcb 'try_*.kicad_pcb' \\
        final.kicad_pcb --reject 'try_*' --cards-from renders/ -o film.gif

    # explicit, with captions
    python3 make_film.py seed.kicad_pcb \\
        'delta.png=what moved, and by how much' placed.kicad_pcb -o film.gif
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)
import argparse
import fnmatch
import glob
import json
import os
import sys

CARD_EXT = ('.png', '.jpg', '.jpeg', '.gif', '.bmp', '.webp')
BOARD_EXT = ('.kicad_pcb',)

DEFAULT_SIZE = 900
DEFAULT_FPS = 6.0
DEFAULT_HOLD = 1.6          # seconds a diagnostic card stays up


# ---------------------------------------------------------------------------
# Shots
# ---------------------------------------------------------------------------
def board_shot(path, label=None, accepted=True, revert_to=None):
    return {'kind': 'board', 'path': os.path.abspath(path),
            'label': label or os.path.splitext(os.path.basename(path))[0],
            'accepted': bool(accepted), 'revert_to': revert_to}


def card_shot(path, caption=None, hold=DEFAULT_HOLD):
    return {'kind': 'card', 'path': os.path.abspath(path),
            'caption': caption or os.path.splitext(os.path.basename(path))[0]
            .replace('_', ' '), 'hold': hold}


def shots_from_loop_dir(work_dir, include_rejected=True, cards=True):
    """Every round a `place_route_loop` run wrote -- kept AND dropped.

    `make_movie.placement_chain` reads the same sidecars and keeps only the
    accepted ones, because it is building the convergence movie. This is the
    other half of that file: the rounds that were tried and reverted, which are
    on disk and were, until now, invisible.
    """
    from movie_camera import load_round_sidecars
    rounds = load_round_sidecars(work_dir, accepted_only=False)
    shots, last_accepted = [], None
    for rd in rounds:
        b = rd.get('board') and os.path.join(work_dir, rd['board'])
        if not b or not os.path.exists(b):
            continue
        n = rd.get('round')
        acc = bool(rd.get('accepted'))
        if not acc and not include_rejected:
            continue
        moved = len(rd.get('moved') or [])
        what = f"{moved} part{'s' if moved != 1 else ''} moved" if moved else 'no move'
        if acc:
            shots.append(board_shot(b, f"round {n}  ({what})", True))
            r = rd.get('routed') and os.path.join(work_dir, rd['routed'])
            if r and os.path.exists(r):
                shots.append(board_shot(r, f"round {n} routed", True))
            last_accepted = r if (r and os.path.exists(r)) else b
        else:
            why = 'screened' if rd.get('screened') else 'rejected'
            shots.append(board_shot(b, f"round {n}  ({what}) -- {why}", False,
                                    revert_to=last_accepted))
    if cards:
        shots = _interleave_cards(shots, _cards_in(work_dir))
    return shots


def shots_from_ledger(ledger_path, store_root=None, work=None,
                      include_rejected=True):
    """The film of a `converge.py` run, straight off its ledger.

    The ledger already records what this needs -- parent_sha, result_sha,
    accepted, kind, lever_argv -- so the film is the ledger read out loud, and
    each beat's caption is the actual lever that produced it rather than a
    description of it.
    """
    from board_store import BoardStore, Ledger
    # CALL the ladder, do not mirror it: converge writes these rows and owns
    # what each field means. Function-scope only to keep this module importable
    # by anything that does not need the ledger reader; `import _path` at the
    # top has already put py_placer on sys.path.
    from converge import row_label as _row_label
    entries = Ledger(ledger_path).entries()
    # `boards`, not `store`: `converge.py record` is the only writer of a ledger
    # and it puts the content-addressed boards in `<ledger dir>/boards`
    # (converge.py:218). Defaulting to a different name here made
    # `--from-ledger` return ZERO shots against a real ledger, silently -- the
    # per-sha `store.has()` check just skips every entry.
    root = store_root or os.path.join(os.path.dirname(
        os.path.abspath(ledger_path)), 'boards')
    if not os.path.isdir(root):                 # tolerate the older name
        alt = os.path.join(os.path.dirname(os.path.abspath(ledger_path)), 'store')
        if os.path.isdir(alt):
            root = alt
    store = BoardStore(root)
    work = work or os.path.join(os.path.dirname(os.path.abspath(ledger_path)),
                                '_film')
    os.makedirs(work, exist_ok=True)
    shots, last_accepted = [], None
    for e in entries:
        sha = e.get('result_sha')
        if not sha or not store.has(sha):
            continue
        acc = bool(e.get('accepted'))
        if not acc and not include_rejected:
            continue
        dest = os.path.join(work, f"{e.get('iteration', len(shots))}_{sha[:8]}.kicad_pcb")
        try:
            store.get(sha, dest)
        except Exception:
            continue
        argv = e.get('lever_argv') or []
        # `or ''` used to end the ladder here, so a row whose whole content
        # is a human's reason -- an --exhausted declaration, which carries
        # `lever: null` by construction -- got a blank caption in the film
        # that is meant to be the ledger read out loud. converge.row_label
        # is the one ladder (lever -> exhausted.reason -> stop_condition).
        lever = (' '.join(str(a) for a in argv[:6]) if argv
                 else _row_label(e))
        head = f"i{e.get('iteration', '?')} {e.get('kind', 'completion')}"
        if acc:
            shots.append(board_shot(dest, f"{head}  {lever}".strip(), True))
            last_accepted = dest
        else:
            shots.append(board_shot(dest, f"{head}  {lever}  -- not kept".strip(),
                                    False, revert_to=last_accepted))
    return shots


def _cards_in(d):
    out = []
    for p in sorted(glob.glob(os.path.join(d, '*'))):
        # isfile: a DIRECTORY named like an image (render_placement's old
        # --per-side `-o x.png` layout) passed the extension filter and died
        # at Image.open with PermissionError (run-3, the film build).
        if os.path.isfile(p) and os.path.splitext(p)[1].lower() in CARD_EXT:
            out.append(p)
    return out


def _interleave_cards(shots, card_paths):
    """Put each diagnostic next to the beat it describes.

    A render is named after the board it was made from far more often than not
    (`round3_delta.png`, `r3_focus1.png`), so matching on the board stem puts it
    where it belongs. Anything that matches nothing goes up front, which is
    where an overview or a spec sheet wants to be anyway.
    """
    stems = {}
    for i, s in enumerate(shots):
        if s['kind'] == 'board':
            stems.setdefault(os.path.splitext(os.path.basename(s['path']))[0], i)
    placed, out = {}, []
    for c in card_paths:
        cstem = os.path.splitext(os.path.basename(c))[0].lower()
        hit = None
        for stem, i in stems.items():
            if stem.lower() in cstem:
                if hit is None or len(stem) > hit[1]:
                    hit = (i, len(stem))
        placed.setdefault(hit[0] if hit else -1, []).append(c)
    for c in placed.get(-1, []):
        out.append(card_shot(c))
    for i, s in enumerate(shots):
        out.append(s)
        for c in placed.get(i, []):
            out.append(card_shot(c))
    return out


def parse_positional(items, reject_globs):
    """`path`, `path=caption`, or a glob of either. Extension picks the kind."""
    shots, last_accepted = [], None
    for raw in items:
        spec, _, caption = raw.partition('=')
        matches = sorted(glob.glob(spec)) or [spec]
        for p in matches:
            ext = os.path.splitext(p)[1].lower()
            if ext in CARD_EXT and os.path.isdir(p):
                continue  # a directory named like an image is not a card
            if ext in CARD_EXT:
                shots.append(card_shot(p, caption or None))
            elif ext in BOARD_EXT:
                base = os.path.basename(p)
                rejected = any(fnmatch.fnmatch(base, g) or fnmatch.fnmatch(p, g)
                               for g in reject_globs)
                s = board_shot(p, caption or None, not rejected,
                               revert_to=None if not rejected else last_accepted)
                if not rejected:
                    last_accepted = s['path']
                shots.append(s)
            else:
                raise SystemExit(f"make_film: don't know what {p} is "
                                 f"(expected a board or an image)")
    return shots


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def _card_frame(size_wh, image_path, caption, theme=None):
    """A diagnostic held at frame size: letterboxed, captioned, not cropped.

    Cropping to fill would cut the legend off a delta render, and the legend is
    the half that carries the numbers.
    """
    from PIL import Image, ImageDraw
    from route_render import load_font
    W, H = size_wh
    # The ACTIVE theme (#946/C4): a card on a light film used to be a dark
    # slab, the one frame in the film that ignored `--theme`.
    import render_theme
    _TH = render_theme.theme(theme, strict=False)
    canvas = Image.new('RGB', (W, H), _TH.rgb('chrome_panel'))
    strip = max(22, H // 9)
    if image_path and os.path.exists(image_path):
        try:
            im = Image.open(image_path).convert('RGB')
            aw, ah = max(1, W - 16), max(1, H - strip - 16)
            sc = min(aw / im.width, ah / im.height)
            im = im.resize((max(1, int(im.width * sc)), max(1, int(im.height * sc))),
                           Image.LANCZOS)
            canvas.paste(im, ((W - im.width) // 2, (H - strip - im.height) // 2))
        except Exception as exc:
            print(f"make_film: could not read {image_path} ({exc})", file=sys.stderr)
    d = ImageDraw.Draw(canvas)
    d.rectangle([0, H - strip, W, H], fill=_TH.rgb('chrome_strip'))
    font = load_font(max(11, strip // 2))
    txt = caption or ''
    try:
        tw = d.textlength(txt, font=font)
    except Exception:
        tw = len(txt) * strip // 4
    d.text((max(6, (W - tw) // 2), H - strip + strip // 5), txt,
           fill=_TH.rgb('chrome_strip_text'), font=font)
    return canvas


def _badge(frame, text, rgb=None, theme=None):
    """Mark a frame as an attempt: a border and a tag, drawn in place.

    Without it a rejected beat is indistinguishable from a kept one, and a film
    that shows an undone change without saying so is worse than one that omits
    it -- which is exactly why the convergence movie omits it.
    """
    # `rgb=None` -> the theme's `status_tried`. #1012 moves it off red: a red
    # badge on a frame whose copper also flashes red is the same collision
    # #946 is about. #1011 keeps the value.
    import render_theme
    _TH = render_theme.theme(theme, strict=False)
    rgb = _TH.rgb('status_tried') if rgb is None else rgb
    from PIL import ImageDraw
    from route_render import load_font
    W, H = frame.size
    d = ImageDraw.Draw(frame)
    w = max(2, H // 160)
    for i in range(w):
        d.rectangle([i, i, W - 1 - i, H - 1 - i], outline=rgb)
    font = load_font(max(11, H // 44))
    try:
        tw = d.textlength(text, font=font)
    except Exception:
        tw = len(text) * H // 80
    pad = max(4, H // 100)
    x0, y0 = W - tw - 3 * pad, pad + w
    d.rectangle([x0, y0, W - pad - w, y0 + font.size + 2 * pad // 2], fill=rgb)
    d.text((x0 + pad, y0 + pad // 2), text, fill=(255, 255, 255), font=font)
    return frame


def build_film(shots, size=DEFAULT_SIZE, fps=DEFAULT_FPS, supersample=1,
               layer_alpha=None, rip_hold=2, chunks=6, camera='auto',
               camera_budget=0.0, tween=10, quiet=False, theme=None,
               attempts=None, attempts_from=None, layout=None, aspect=None,
               panels=None, iso_opts=None, spool=False, max_frames=None,
               placement=None):
    """Frames for the whole shot list. One render pass, one scale.

    `attempts` is a `movie_attempts.Track` -- the search behind this film. Left
    `None` it is DISCOVERED from `attempts_from`, else from the boards' own
    directory, because the sidecars that record the search sit next to the
    boards a film is made from. `None` after that is a real answer: a chain
    with no search behind it gets no band.

    `panels` is make_movie's (#946/C4): 'xray' or 'xray+iso'. With the iso
    view on, a layout that has a panel to split gives the 3D view a region of
    its own; legacy and inset stack it under the frame.

    `spool=True` (#1036; `main()` passes it) streams the film through
    `frame_spool.FrameSpool`s -- the board frames, then the assembled film
    with its cards -- so memory does not grow with the frame count, and a
    spool comes back instead of a list (it indexes, iterates and has a
    `len`; the caller closes it). The default stays a list for in-process
    callers that edit frames in place.

    `max_frames` is make_movie's frame budget, resolved by the same
    `make_movie.resolve_max_frames`: None = $KICAD_MOVIE_MAX_FRAMES, else
    2400; 0 = none.
    """
    import animate_route as a
    import render_theme
    _th = render_theme.theme(theme)
    boards = [s for s in shots if s['kind'] == 'board']
    if not boards:
        return []

    # A rejected attempt is followed by an explicit undo back to the last
    # accepted board, so the NEXT beat's delta is measured against what was
    # actually kept -- not against a board that was thrown away.
    steps, owner = [], []
    for s in shots:
        if s['kind'] != 'board':
            continue
        steps.append((s['label'], s['path'], None))
        owner.append(s)
        if not s['accepted'] and s.get('revert_to'):
            steps.append(('back to the kept board', s['revert_to'], None, 'revert'))
            owner.append({'kind': 'board', 'accepted': True, 'revert': True,
                          'label': 'reverted'})
    final = steps[-1][1]

    stage = None
    if str(camera).lower() not in ('off', '', 'none', '0'):
        try:
            from movie_camera import Stage, synth_rounds
            rounds = synth_rounds([st[1] for st in steps])
            if any(rd['moved'] for rd in rounds):
                stage = Stage(rounds, '', fps=fps, budget=camera_budget,
                              tween=tween, quiet=quiet)
        except Exception as exc:
            if not quiet:
                print(f"make_film: no camera ({exc})", file=sys.stderr)

    marks = []
    # `theme=` was accepted and DROPPED here until #1021: `--theme light`
    # reached build_film and never reached the renderer, so the flag was a
    # no-op on the film path. Same defect as make_movie's, found the same way.
    # #1018's layout reaches the PLACEMENT film too. It did not until now,
    # which is the wrong way round: this is the film that actually shows
    # placement, so it is the one whose lower box has a placement content to
    # hold and whose rail has laps to count. A film of a search rendered with
    # no rail and no box was the one place the design system could not be
    # seen doing its job.
    _geom = []
    # $KICAD_MOVIE_LAYOUT / $KICAD_MOVIE_ASPECT apply here too, through the
    # same resolver make_movie uses -- this passed None straight through, and
    # build_boards reads None as 'legacy', so both knobs were no-ops on films.
    import frame_layout
    layout, aspect = frame_layout.resolve_layout_aspect(layout, aspect)
    # #946/C4: the attempts are found BEFORE the frame is planned, so the band
    # is reserved inside a declared ratio rather than grown under it.
    try:
        import movie_attempts
        if attempts is None and attempts_from != '':
            attempts = movie_attempts.discover(
                attempts_from or os.path.dirname(os.path.abspath(final)))
    except Exception as exc:                                    # noqa: BLE001
        if not quiet:
            print(f"make_film: no attempts ({exc})", file=sys.stderr)
        attempts = None
    import make_movie as _mm
    want_iso = _mm._panels_wanted(panels, quiet)
    iso_box = False
    if want_iso:
        import movie_panels
        import copy as _copy
        # a COPY (#1036 review): the caller's IsoOpts is not ours to fill in
        iso_opts = (_copy.copy(iso_opts) if iso_opts is not None
                    else movie_panels.IsoOpts())
        # the theme resolved ONCE, above -- never the name again
        iso_opts.theme = _th
        if str(layout or 'legacy').lower() not in ('legacy', 'inset'):
            iso_box = movie_panels.preflight(steps[0][1], iso_opts) is None
    import frame_spool
    # #1036 review: BOTH spools are closed on every way out -- an exception
    # anywhere below, and the empty-film return. They leaked two krt_frames_*
    # directories when an append raised (a full disk), where make_movie left
    # none.
    sink = frame_spool.FrameSpool() if spool else None
    try:
        return _build_film_body(
            a, frame_spool, sink, steps, final, size, supersample,
            layer_alpha, rip_hold, chunks, stage, marks, _th, layout, aspect,
            _geom, attempts, iso_box, want_iso, iso_opts, owner, shots, fps,
            boards, quiet, max_frames, placement)
    except BaseException:
        if sink is not None:
            sink.close()
        raise


def _build_film_body(a, frame_spool, sink, steps, final, size, supersample,
                     layer_alpha, rip_hold, chunks, stage, marks, _th, layout,
                     aspect, _geom, attempts, iso_box, want_iso, iso_opts,
                     owner, shots, fps, boards, quiet, max_frames,
                     placement=None):
    import make_movie as _mm
    # the ONE resolver make_movie uses: an explicit budget, else
    # $KICAD_MOVIE_MAX_FRAMES, else the default -- never None, which
    # build_boards reads as "no budget"
    max_frames = _mm.resolve_max_frames(max_frames)
    if sink is not None:
        max_frames = _mm.spool_budget(sink, steps, size, max_frames,
                                      rip_hold, who='make_film')
    # #1042: the placement panels, measured before the frame is planned so
    # their region is reserved -- the same call make_movie makes.
    placement = placement or {}
    _verdict = bool(attempts is not None and len(attempts.attempts) >= 2)
    _ptrack, _pwhy = None, 'off (--no-placement-panel)'
    if not placement.get('off'):
        try:
            import movie_placement
            _ptrack, _pwhy = movie_placement.build_track(
                steps, [], ledger=placement.get('ledger'),
                benchmark=placement.get('benchmark'),
                intent=placement.get('intent'), quiet=quiet)
        except Exception as exc:                                # noqa: BLE001
            _ptrack, _pwhy = None, 'could not measure (%s)' % exc
    _pfn, _lands = None, {}
    if _ptrack is not None:
        import movie_placement
        _pfn = movie_placement.band_px(_ptrack, _verdict)
    frames = a.build_boards(steps, final, size, supersample, layer_alpha,
                            rip_hold, chunks, stage=stage, marks=marks,
                            frames_sink=sink, max_frames=max_frames,
                            theme=_th, layout=layout, aspect=aspect,
                            geom_out=_geom,
                            attempts_band=(_pfn if _pfn is not None
                                           else bool(_verdict)),
                            iso_panel=iso_box, lands_out=_lands)
    if not frames:
        if sink is not None:
            sink.close()
        return []
    _g0 = _geom[0] if _geom else None
    _vbox = _g0.track if _g0 is not None else None
    if _ptrack is not None:
        import movie_placement
        _plan = _pfn.plans[-1] if _pfn.plans else None
        if _plan is not None and _plan.mode == 'declined':
            _ptrack, _pwhy = None, 'declined: %s' % _plan.why
        elif _g0 is not None and _g0.track is not None:
            _pbox, _vbox = movie_placement.split_band(
                _g0.track, both=_verdict, track=_ptrack, frame_h=_g0.frame.h)
            _ptrack = movie_placement.with_firsts(_ptrack, marks, _lands)
            frames = movie_placement.compose(frames, _pbox, _ptrack, marks,
                                             _th, _g0.frame.h)
        else:
            _ptrack, _pwhy = None, 'no band could be reserved in this frame'
        # SAID whenever a placement was found, drawn or declined
        print('make_film: ' + movie_placement.status_line(_ptrack, _pwhy,
                                                          _plan),
              file=sys.stderr)
    if _ptrack is not None and _vbox is None:
        attempts = None           # the band is all placement: no verdict box

    # #1021. THE ATTEMPTS BAND, AND IT GOES HERE -- BEFORE THE BADGE LOOP.
    # `_badge` draws a border on the frame it is given; attach the band
    # afterwards and the border encloses only the board, which is exactly the
    # trap `movie_panels.py:40-44` documents. It is also before `size_wh` is
    # read, so the spliced cards are cut at the band-inclusive size and the
    # film keeps ONE frame size.
    #
    # Discovered from the boards' own directory when the caller named no
    # attempts: this film is usually made FROM a search, and the sidecars that
    # record it are sitting next to the boards. Nothing is ever inferred from
    # the boards themselves -- no sidecars means no band.
    try:
        import movie_attempts
        frames, _rep = movie_attempts.attach(
            frames, attempts, theme=_th, marks=marks, box=_vbox)
        if not quiet:
            print('make_film: ' + movie_attempts.status_line(_rep),
                  file=sys.stderr)
    except Exception as exc:                                    # noqa: BLE001
        if not quiet:
            print(f"make_film: no attempts band ({exc})", file=sys.stderr)

    # The iso view, after the band and BEFORE the badges and cards, for the
    # band's reason: a badge's border must enclose the whole composed frame,
    # and the cards are cut at the composed size.
    if want_iso:
        import movie_panels
        _box = (_g0.panel_split[0] if (iso_box and _g0 is not None
                                      and _g0.panel_split) else None)
        frames, _irep = movie_panels.compose_two_panel(
            frames, marks, final, iso_opts,
            **({'box': _box} if _box is not None else {}))
        print('make_film: ' + movie_panels.iso_status_line(_irep),
              file=sys.stderr)

    # Badge every frame that belongs to an attempt.
    by_step = {}
    for i, (label, board, first, last) in enumerate(marks):
        if i < len(owner):
            by_step[i] = (owner[i], first, last)
    n_att = 0
    badged = set()
    for i, (s, first, last) in by_step.items():
        if s.get('accepted'):
            continue
        n_att += 1
        badged.update(range(first, last))

    def _badge_fn(i, f):
        if i in badged:
            _badge(f, 'TRIED', theme=_th)
        return f
    frame_spool.transform(frames, _badge_fn)

    # Splice the cards in where they sit in the shot order. Walk the shot list
    # and the marks together: the Nth board shot is the Nth mark, and a card
    # goes in front of whatever board shot follows it.
    size_wh = frames[0].size
    hold_frames = lambda h: max(1, int(round(h * fps)))
    inserts = {}
    mark_i, pending = 0, []
    for s in shots:
        if s['kind'] == 'card':
            pending.append(s)
            continue
        # A card in front of the FIRST beat opens the film, so it goes before
        # frame 0 -- build_boards' own "input" snapshot, which is otherwise the
        # first thing on screen and explains nothing.
        at = 0 if mark_i == 0 else (marks[mark_i][2] if mark_i < len(marks)
                                    else len(frames))
        for c in pending:
            inserts.setdefault(at, []).extend(
                [_card_frame(size_wh, c['path'], c['caption'], theme=_th)]
                * hold_frames(c['hold']))
        pending = []
        # skip the auto-inserted revert step so the pairing stays aligned
        mark_i += 1
        if not s['accepted'] and s.get('revert_to'):
            mark_i += 1
    for c in pending:
        inserts.setdefault(len(frames), []).extend(
            [_card_frame(size_wh, c['path'], c['caption'], theme=_th)]
            * hold_frames(c['hold']))

    if frame_spool.is_spool(frames):
        out = frame_spool.FrameSpool()
        try:
            for i in range(len(frames)):
                for c in inserts.get(i, []):
                    out.append(c)
                out.append(frames[i])
            for c in inserts.get(len(frames), []):
                out.append(c)
        except BaseException:
            out.close()
            raise
        finally:
            frames.close()
    else:
        out = []
        for i, f in enumerate(frames):
            out.extend(inserts.get(i, []))
            out.append(f)
        out.extend(inserts.get(len(frames), []))

    if not quiet:
        n_cards = sum(1 for s in shots if s['kind'] == 'card')
        # "assembled", not "frames": this is the list handed to the writer, and
        # the GIF encoder collapses runs of identical frames (card holds, the
        # end hold), so the file that lands holds FEWER. Printing both as
        # "frames" invited exactly one reconciliation failure -- 377 here, 348
        # in the delivered GIF, and nothing said which was the film. The
        # `animate_route: wrote ...` line below counts the artifact itself; that
        # is the number to quote.
        print(f"make_film: {len(boards)} beats ({n_att} attempts), "
              f"{n_cards} cards, {len(out)} frames assembled "
              f"(the writer reports what the file actually holds)",
              file=sys.stderr)
    return out


def _iso_opts(a):
    import movie_panels
    return movie_panels.IsoOpts(require_models=not a.iso_allow_bare,
                                theme=a.theme)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="A shot is a board, an image, or either with '=caption'. "
               "Globs are expanded in place.")
    ap.add_argument('shots', nargs='*',
                    help="boards and diagnostic images, in film order")
    ap.add_argument('--from-loop-dir', metavar='DIR',
                    help="expand every round a place_route_loop run wrote, "
                         "kept AND dropped, from its loop_round*.json sidecars")
    ap.add_argument('--from-ledger', metavar='FILE',
                    help="expand a converge.py ledger; boards come out of the "
                         "content-addressed store")
    ap.add_argument('--store', help="board store root (default: <ledger dir>/store)")
    ap.add_argument('--reject', action='append', default=[], metavar='GLOB',
                    help="positional boards matching this were dead ends "
                         "(repeatable)")
    ap.add_argument('--accepted-only', action='store_true',
                    help="drop the attempts -- the convergence movie's cut")
    ap.add_argument('--cards-from', metavar='DIR', action='append', default=[],
                    help="splice every image in DIR next to the beat its name "
                         "matches (repeatable)")
    ap.add_argument('--no-cards', action='store_true',
                    help="boards only, even if a source dir has images in it")
    ap.add_argument('--hold', type=float, default=DEFAULT_HOLD, metavar='SEC',
                    help=f"how long a card stays up (default {DEFAULT_HOLD})")
    ap.add_argument('-o', '--out', default='film.gif')
    ap.add_argument('--size', type=int, default=DEFAULT_SIZE)
    ap.add_argument('--fps', type=float, default=DEFAULT_FPS)
    ap.add_argument('--supersample', type=int, default=1)
    ap.add_argument('--layer-alpha', type=int, default=None,
                    help="per-layer copper opacity 1-255. Default: the "
                         "theme's own measured alpha (dark 150, light 205)")
    ap.add_argument('--rip-hold', type=int, default=2)
    ap.add_argument('--chunks', type=int, default=6)
    ap.add_argument('--end-hold', type=float, default=1.5)
    ap.add_argument('--attempts-ledger', default=None, metavar='PATH',
                    help='the converge ledger for the attempts band and the '
                         'placement panels (#1042); --from-ledger also '
                         'supplies one')
    ap.add_argument('--benchmark-board', default=None, metavar='PATH',
                    help="a benchmark placement drawn DASHED on the "
                         "placement arrangement panel (a screen, not the "
                         "verdict)")
    ap.add_argument('--floorplan-intent', default=None, metavar='PATH',
                    help='grade placement boards the ledger does not name '
                         'with check_floorplan --intent')
    ap.add_argument('--no-placement-panel', action='store_true',
                    help='never draw the placement panels (#1042)')
    ap.add_argument('--max-frames', type=int, default=None, metavar='N',
                    help="frame budget, as make_movie's: a trace over its "
                         "share is revealed in --chunks batches, loudly. "
                         "Default: $KICAD_MOVIE_MAX_FRAMES or 2400; 0 = none")
    ap.add_argument('--camera', default='auto', choices=['off', 'auto'],
                    help="animate footprint motion (default: on -- the parts "
                         "arriving is most of what a film is for)")
    ap.add_argument('--camera-budget', type=float, default=0.0,
                    help="cap the camera runtime in seconds (0 = unlimited)")
    ap.add_argument('--tween', type=int, default=10,
                    help="frames per part move (0 snaps)")
    ap.add_argument('--png-dir', help="also dump every frame as a PNG")
    ap.add_argument('--shots-json', help="write the resolved shot list here")
    ap.add_argument('--theme', default=None, choices=('dark', 'light'), help="'dark' (default, or $KICAD_RENDER_THEME) or 'light'. A light ground is for a figure going into a light-background document; the file's ground cannot be changed afterwards.")
    ap.add_argument('--layout', default=None,
                    help="frame layout: 'legacy' (default, or "
                         "$KICAD_MOVIE_LAYOUT; today's frame), "
                         "'stacked', 'sidebar', 'inset', 'split' or 'auto'. "
                         "Anything but legacy reserves a rail and a lower "
                         "box, which is where the placement content lives")
    ap.add_argument('--aspect', default=None, metavar='W:H',
                    help="target frame aspect, or $KICAD_MOVIE_ASPECT; "
                         "'board' (default) keeps the board's own bounding box")
    ap.add_argument('--panels', default=None, choices=('xray', 'xray+iso'),
                    help="'xray' (default, or $KICAD_MOVIE_PANELS) or "
                         "'xray+iso': a kicad-cli 3D view, in the layout's "
                         "own panel when it has one to split (stacked, "
                         "sidebar, split), else under the frame. Same gate "
                         "as make_movie: a mostly-bare board gets none")
    ap.add_argument('--iso-allow-bare', action='store_true',
                    help='draw the 3D view even when its models do not '
                         'resolve (#1016)')
    ap.add_argument('--no-attempts', action='store_true',
                    help="drop the attempts band -- the boards alone")
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args(argv)

    shots = []
    if a.from_loop_dir:
        shots += shots_from_loop_dir(a.from_loop_dir,
                                     include_rejected=not a.accepted_only,
                                     cards=not a.no_cards)
    if a.from_ledger:
        shots += shots_from_ledger(a.from_ledger, a.store,
                                   include_rejected=not a.accepted_only)
    if a.shots:
        shots += parse_positional(a.shots, a.reject)
    for d in a.cards_from:
        shots = _interleave_cards(shots, _cards_in(d))
    if a.no_cards:
        shots = [s for s in shots if s['kind'] != 'card']
    for s in shots:
        if s['kind'] == 'card':
            s['hold'] = a.hold

    if not any(s['kind'] == 'board' for s in shots):
        print("make_film: no boards to film. Give board paths, --from-loop-dir "
              "or --from-ledger.", file=sys.stderr)
        return 2
    if a.shots_json:
        with open(a.shots_json, 'w', encoding='utf-8') as f:
            json.dump(shots, f, indent=2)

    # #1021: the attempts come from whichever source this film came from --
    # the loop dir's sidecars or the converge ledger -- and never from the
    # boards themselves. `--no-attempts` is the OFF arm, and it says so in the
    # status line rather than silently drawing nothing.
    attempts = None
    if not a.no_attempts:
        import movie_attempts
        if a.from_loop_dir:
            attempts = movie_attempts.attempts_from_loop_dir(a.from_loop_dir)
        elif a.from_ledger or a.attempts_ledger:
            attempts = movie_attempts.attempts_from_converge_ledger(
                a.attempts_ledger or a.from_ledger)
    frames = build_film(shots, theme=a.theme,
                        size=a.size, fps=a.fps, supersample=a.supersample,
                        layer_alpha=a.layer_alpha, rip_hold=a.rip_hold,
                        chunks=a.chunks, camera=a.camera,
                        camera_budget=a.camera_budget, tween=a.tween,
                        quiet=a.quiet, layout=a.layout, aspect=a.aspect,
                        panels=a.panels, spool=True,
                        max_frames=a.max_frames,
                        placement={'off': a.no_placement_panel,
                                   'ledger': (a.attempts_ledger
                                              or a.from_ledger),
                                   'benchmark': a.benchmark_board,
                                   'intent': a.floorplan_intent},
                        iso_opts=_iso_opts(a),
                        attempts=attempts,
                        attempts_from=('' if a.no_attempts else
                                       (a.from_loop_dir or
                                        (os.path.dirname(os.path.abspath(
                                            a.from_ledger))
                                         if a.from_ledger else None))))
    if not frames:
        print("make_film: nothing to animate", file=sys.stderr)
        return 1
    import animate_route as ar
    # theme=: the pad a mixed-size film is letterboxed with is the theme's
    # ground. Without it a light film's letterbox was the DARK ground.
    try:
        ar.save_movie(frames, a.out, a.fps, a.end_hold, png_dir=a.png_dir,
                      theme=a.theme)
    finally:
        if hasattr(frames, 'close'):
            frames.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
