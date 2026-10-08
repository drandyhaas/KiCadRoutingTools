#!/usr/bin/env python3
"""Make a routing movie (.mp4) from boards you already have (issue #506).

The stress harness renders one per run; this is the same movie, on demand, from
anything: a run/output directory, an explicit list of step boards, or a single
board that draws itself.

    python3 make_movie.py RUNDIR                 # whole chain -> RUNDIR/routing.mp4
    python3 make_movie.py step1.kicad_pcb step2.kicad_pcb step3.kicad_pcb
    python3 make_movie.py board.kicad_pcb        # one board reveals its copper
    python3 make_movie.py RUNDIR -o out.mp4 --size 1400 --fps 8 --png

New copper flashes white, reroutes/restores green, rips flash red on the frame
before they vanish. Steps that recorded a fine trace (``KICAD_ROUTE_TRACE=1``
leaves ``<board>_routetrace.json``) animate per segment/via; steps without one
reveal their board-to-board delta in chunks.

**Output format follows the extension**: ``.mp4`` (H.264, small, plays
everywhere) needs ``imageio`` + ``imageio-ffmpeg`` and falls back to a sibling
``.gif`` when they are missing; ``.gif`` is native Pillow, no dependency.

This module is the shared core: ``tests/stress/render_run.py`` (stress runs) and
the GUI's "Routing Movie..." button both call ``make_movie()`` rather than
re-implementing it, so all three produce the same movie.
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'combined'], 'kind': 'instrument'}

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# Defaults shared by the CLI, the stress renderer and the GUI button, so the
# movie looks the same wherever it is made from.
DEFAULT_SIZE = 1000
DEFAULT_FPS = 6.0
DEFAULT_SUPERSAMPLE = 1
#: None = the THEME's measured alpha (dark 150, light 205). A number here
#: overrode it for every film, so LIGHT's 205 -- measured by
#: `palette_audit` for a light ground -- was never used (#946/C4).
DEFAULT_LAYER_ALPHA = None
DEFAULT_RIP_HOLD = 2
DEFAULT_CHUNKS = 6
DEFAULT_END_HOLD = 1.5


def resolve_inputs(inputs):
    """(steps, final_board) for whatever was passed.

    * a single directory  -> the chain discovered in it (stepN order, else
      write-time order), exactly as a stress run's movie sees it
    * one or more boards  -> that sequence, in the order given (already the
      chain order for GUI snapshots / shell globs like ``step*.kicad_pcb``)
    """
    import animate_route as a
    if len(inputs) == 1 and os.path.isdir(inputs[0]):
        chain = placement_chain(inputs[0])
        if chain:
            return chain
        return a.discover_steps(inputs[0])
    boards = [os.path.abspath(b) for b in inputs]
    missing = [b for b in boards if not os.path.exists(b)]
    if missing:
        raise FileNotFoundError(missing[0])
    return a.steps_for_boards(boards), (boards[-1] if boards else None)


def placement_chain(work_dir):
    """(steps, final) for a place_route_loop work dir, or None.

    Keyed on the loop_round{N}.json SIDECARS, never a loop_round*.kicad_pcb
    glob: --work-dir defaults to the output board's directory, which may hold
    unrelated boards, and mtime order would animate a REJECTED round as though
    it had been kept. The sidecar's `parent` is what makes the set a chain --
    round N follows the last ACCEPTED board, not N-1.
    """
    try:
        from movie_camera import load_round_sidecars
    except Exception:
        return None
    rounds = load_round_sidecars(work_dir)
    if not rounds:
        return None
    steps = []
    for rd in rounds:
        if not rd.get('accepted') or not rd.get('board'):
            continue          # rejected/screened rounds are on disk, not the story
        b = os.path.join(work_dir, rd['board'])
        if os.path.exists(b):
            steps.append((f"round {rd['round']}", b, None))
        r = rd.get('routed') and os.path.join(work_dir, rd['routed'])
        if r and os.path.exists(r):
            steps.append((f"round {rd['round']} routed", r, None))
    if not steps:
        return None
    return steps, steps[-1][1]


#: The film's frame budget when neither the caller nor $KICAD_MOVIE_MAX_FRAMES
#: names one -- the env knob's own default (`env_knobs.MOVIE_MAX_FRAMES`).
DEFAULT_MAX_FRAMES = 2400


def resolve_max_frames(max_frames=None):
    """The film's frame budget, resolved ONE way for every front end.

    An explicit value wins (0 = no budget); else `$KICAD_MOVIE_MAX_FRAMES`;
    else `DEFAULT_MAX_FRAMES`. `make_movie` and `make_film` both call this,
    so the two cannot drift: `make_film` used to hand `None` straight to
    `build_boards`, which reads it as "no budget", so its documented default
    and the env knob were both no-ops on films.
    """
    if max_frames is not None:
        return max(0, int(max_frames))
    try:
        import env_knobs
        return max(0, int(getattr(env_knobs, 'MOVIE_MAX_FRAMES',
                                  DEFAULT_MAX_FRAMES)))
    except Exception:                                           # noqa: BLE001
        return DEFAULT_MAX_FRAMES


def spool_budget(spool, steps, size, max_frames, rip_hold, who='make_movie'):
    """The frame budget to render with, after checking the spool's DISK.

    #1036: the spool trades RAM for disk (~1.27 MB per 1400 px frame, so
    ~7.6 GB for 6000 frames). The film is estimated before it is drawn -- the
    budget when there is one, else the traces' own frame estimates -- and
    when the spool's disk cannot hold it that is said LOUDLY; an UNBUDGETED
    film then falls back to the default budget rather than filling the disk.
    Shared by `make_movie` and `make_film`.
    """
    try:
        import animate_route as _a
        import frame_spool as _fsp
        est = (max_frames if max_frames else
               sum(_a.trace_frame_estimate(_a.load_trace(s[2]), rip_hold)
                   for s in steps if len(s) > 2 and s[2]
                   and os.path.isfile(s[2])) + 50 * len(steps))
        px = int(size) * int(size) * 0.6          # ~a 16:10 frame at `size`
        fits, need, free = _fsp.disk_check(spool.dir, est, px)
        if not fits:
            print('%s: SPOOL DISK -- ~%d frames need ~%.1f GB, %s has %.1f GB '
                  'free' % (who, est, need / 1e9, spool.dir,
                            (free or 0) / 1e9), file=sys.stderr)
            if not max_frames:
                max_frames = resolve_max_frames(None) or DEFAULT_MAX_FRAMES
                print('%s: falling back to a %d-frame budget so the spool '
                      'fits' % (who, max_frames), file=sys.stderr)
    except Exception:                                           # noqa: BLE001
        pass
    return max_frames


def leading_copper_free(steps):
    """How many boards at the head of the chain carry no copper at all.

    Read off the file TEXT (a `(segment`, `(arc` or `(via` token), not a
    parse: it is asked on every film without a camera, and a parse of a large
    board costs seconds to answer a yes/no question.
    """
    import re
    # TOP-LEVEL tokens only (#1036 review): `(arc` also appears inside a
    # zone's or a gr_poly's `(pts ...)`. A board item sits one indent deep --
    # a tab (KiCad 8+) or two spaces (older) -- and those are deeper.
    copper = re.compile(r'^(?:\t| {2})\((?:segment|arc|via)[\s)]',
                        re.MULTILINE)
    n = 0
    for st in steps:
        try:
            with open(st[1], encoding='utf-8', errors='replace') as f:
                txt = f.read()
        except OSError:
            break
        if copper.search(txt):
            break
        n += 1
    return n


def default_output(inputs):
    """Where the movie lands when no -o is given: inside a run dir, else next to
    the last board."""
    if len(inputs) == 1 and os.path.isdir(inputs[0]):
        return os.path.join(os.path.abspath(inputs[0]), 'routing.mp4')
    return os.path.splitext(os.path.abspath(inputs[-1]))[0] + '_routing.mp4'


def make_movie(inputs, out=None, size=DEFAULT_SIZE, fps=DEFAULT_FPS,
               supersample=DEFAULT_SUPERSAMPLE, layer_alpha=DEFAULT_LAYER_ALPHA,
               rip_hold=DEFAULT_RIP_HOLD, chunks=DEFAULT_CHUNKS,
               end_hold=DEFAULT_END_HOLD, png_dir=None, quiet=False,
               camera=None, camera_budget=60.0, tween=10,
               timing=None, theme=None,
               aspect=None, attempts=None, max_frames=None,
               title=None, attempts_ledger=None, benchmark_board=None,
               floorplan_intent=None, placement_panel=None, board3d=None,
               benchmark_score=None):
    """Render the movie. ``inputs`` is a run dir (one entry) or a board sequence.

    Returns the path actually written -- which is a sibling ``.gif`` when an
    ``.mp4`` was asked for and imageio-ffmpeg is unavailable -- or None when
    there was nothing to animate.

    Frames are SPOOLED to disk as they are drawn (#1036,
    `frame_spool.FrameSpool`) and every post-pass -- the planned frame, the
    benchmark band, the run clock -- is a per-frame transform
    applied while the encoder streams, so an .mp4's memory does not grow
    with the frame count (a .gif's grows up to `animate_route.GIF_MAX_FRAMES`
    frames, which Pillow's writer holds). ``max_frames`` is the film's frame budget (None = $KICAD_MOVIE_MAX_FRAMES,
    default 2400; 0 = none): a route trace that does not fit its share falls
    back to the chunked reveal, loudly.
    """
    import frame_spool
    spool = frame_spool.FrameSpool()
    try:
        return _make_movie(
            inputs, out=out, size=size, fps=fps, supersample=supersample,
            layer_alpha=layer_alpha, rip_hold=rip_hold, chunks=chunks,
            end_hold=end_hold, png_dir=png_dir, quiet=quiet, camera=camera,
            camera_budget=camera_budget, tween=tween,
            timing=timing, theme=theme,
            aspect=aspect, attempts=attempts, max_frames=max_frames,
            title=title, attempts_ledger=attempts_ledger,
            benchmark_board=benchmark_board,
            floorplan_intent=floorplan_intent,
            placement_panel=placement_panel,
            board3d=board3d, benchmark_score=benchmark_score,
            spool=spool)
    finally:
        spool.close()


def _make_movie(inputs, out, size, fps, supersample, layer_alpha, rip_hold,
                chunks, end_hold, png_dir, quiet, camera, camera_budget, tween,
                timing, theme, aspect, attempts,
                max_frames, spool, title=None, attempts_ledger=None,
                benchmark_board=None, floorplan_intent=None,
                placement_panel=None, board3d=None, benchmark_score=None):
    import animate_route as a
    if isinstance(inputs, str):
        inputs = [inputs]
    if not inputs:
        return None
    steps, final = resolve_inputs(inputs)
    if not final:
        if not quiet:
            print(f"make_movie: no boards found in {inputs[0]}", file=sys.stderr)
        return None
    # THE THEME, RESOLVED ONCE (#1036 review). Passed down as a NAME, every
    # region resolved it again -- and an invalid $KICAD_RENDER_THEME warned
    # once per FRAME from the clock band. A name given in code still refuses (strict); the environment's
    # warns, here, once.
    import render_theme as _rt
    theme = (_rt.theme(theme) if theme is not None
             else _rt.default_theme())
    # #431: the camera is OPT-IN. camera=None falls back to the env knob, so
    # one variable turns it on for the GUI recorder, run_plan.py --movie and the
    # stress renderer at once -- and OFF is the default everywhere, because
    # every GUI movie is a routing movie and changing those is pure regression
    # risk for no user-visible win.
    stage = None
    # #1036: whether anyone SAID 'off'. The default is still off for a chain
    # whose boards only differ in copper -- every GUI movie -- but a chain
    # whose parts MOVE between boards is a placement film, and rendering it
    # without the camera silently dropped every copper-free placement board
    # (run 32: boards 01-07 of 22 contributed no frames). So an UNSTATED
    # camera switches to 'auto' exactly when the boards themselves show a pose
    # change; an explicit `--camera off` or $KICAD_MOVIE_CAMERA is obeyed.
    camera_explicit = (camera is not None
                       or bool(os.environ.get('KICAD_MOVIE_CAMERA')))
    if camera is None:
        try:
            import env_knobs
            camera = getattr(env_knobs, 'MOVIE_CAMERA', 'off')
        except Exception:
            camera = 'off'
    _synth = None
    _is_dir = len(inputs) == 1 and os.path.isdir(inputs[0])
    if (str(camera).lower() in ('off', '', 'none', '0')
            and not camera_explicit and len(steps) > 1):
        try:
            from movie_camera import synth_rounds
            _synth = synth_rounds([s[1] for s in steps])
        except Exception:                                       # noqa: BLE001
            _synth = None
        if _synth and any(rd['moved'] for rd in _synth):
            camera = 'auto'
            # PRINTED EVEN WHEN QUIET: the film changed shape because of what
            # was on disk, and the only way to learn that is this line.
            print('make_movie: %d of %d boards move parts -- camera auto, so '
                  'the placement glides in before the routing (#1036); pass '
                  '--camera off for a copper-only film'
                  % (len(_synth), len(steps)), file=sys.stderr)
    if str(camera).lower() not in ('off', '', 'none', '0'):
        rounds = []
        if len(inputs) == 1 and os.path.isdir(inputs[0]):
            try:
                from movie_camera import load_round_sidecars
                rounds = load_round_sidecars(inputs[0])
            except Exception:
                rounds = []
        work_dir = inputs[0] if (len(inputs) == 1 and os.path.isdir(inputs[0])) else ''
        if not rounds:
            # No sidecars: only place_route_loop writes them, so every chain a
            # person drove by hand landed here and got stage=None -- which
            # animates COPPER deltas only. A placement step changes no copper,
            # so the first placements, the very thing the camera exists to show,
            # rendered as ONE frame or vanished. The poses are in the boards;
            # diff them and the camera has everything it needs.
            try:
                from movie_camera import synth_rounds
                rounds = (_synth if _synth is not None
                          else synth_rounds([s[1] for s in steps]))
                if not any(rd['moved'] for rd in rounds):
                    rounds = []      # pure routing chain: nothing to tween
                elif not quiet:
                    n = sum(len(rd['moved']) for rd in rounds)
                    print(f"make_movie: no loop_round*.json sidecars; recovered "
                          f"{n} footprint moves from the boards themselves",
                          file=sys.stderr)
            except Exception as exc:
                if not quiet:
                    print(f"make_movie: could not read poses off the boards "
                          f"({exc}); rendering without a camera", file=sys.stderr)
                rounds = []
        if rounds:
            from movie_camera import Stage
            stage = Stage(rounds, work_dir, fps=fps,
                          budget=camera_budget, tween=tween, quiet=quiet)
    if stage is None and not _is_dir:
        # #1036, the other half: without a stage a board that carries no copper
        # draws nothing, so a chain that OPENS with placement boards opens on
        # the first routed one and never says why. Say so, and name the lever.
        _lead = leading_copper_free(steps)
        if _lead and _lead < len(steps):
            print('make_movie: %d leading copper-free board(s) skipped -- they '
                  'change no copper, so a film without the camera shows '
                  'nothing for them; use --camera auto (or make_film) to film '
                  'the placement' % _lead, file=sys.stderr)
    # #887: the run clock is PRESENCE-GATED, not a mode. cmd_timing.jsonl
    # exists only in a teed stress-run work dir, so every existing movie --
    # the GUI recorder's, place_route_loop's, render_run's, and every
    # board-sequence invocation outside such a run -- is untouched. That is a
    # far narrower trigger than the #431 camera's, which is why the camera is
    # opt-in and this is not.
    ledger = None
    # `timing is None` is AUTO, and testing it as a string was a real bug:
    # `str(None).lower()` is 'none', which the off-list contained, so the
    # DEFAULT disabled the clock and the feature never ran once. It shipped
    # green because every test constructed a RunClock directly rather than
    # going through make_movie -- the integration was the one path nothing
    # exercised. Compare the object, not its repr.
    _timing_off = (isinstance(timing, str)
                   and timing.strip().lower() in ('off', 'none', '0', ''))
    if not _timing_off:
        if isinstance(timing, str) and timing.strip():
            # An EXPLICIT ledger. A path that is not there is a typo, and
            # falling through to auto-discovery would stamp this movie with
            # ANOTHER RUN's clock -- silently, and plausibly, because the
            # numbers would look perfectly reasonable. There is no way for a
            # viewer to tell afterwards. Refuse instead; `main()` catches this
            # into an argparse error, and the caller who asked for a specific
            # file learns it was not found rather than getting a clock they
            # did not ask for.
            if not os.path.isfile(timing):
                raise FileNotFoundError(
                    'timing ledger: no such file: %s' % timing)
            ledger = timing
        else:
            try:
                import cmd_timing
                ledger = cmd_timing.find_ledger(inputs[0])
            except Exception:                                   # noqa: BLE001
                ledger = None
    marks = [] if ledger else None
    # #1018: resolved once inside build_boards; collected here so the status
    # line can say what frame ran and what it gave up.
    geom_out = []
    import frame_layout
    aspect = frame_layout.resolve_aspect(aspect)
    # The run directory's own name is the closest thing a multi-step chain has
    # to a board name, and it is what the rail's stable left should carry.
    # `title` (--title) wins; else the run directory; else `board_title`
    # derives one that is never a LATER board's name (#1036 review).
    _title = title or (os.path.basename(os.path.abspath(inputs[0]))
                       if len(inputs) == 1 and os.path.isdir(inputs[0])
                       else None)
    max_frames = resolve_max_frames(max_frames)
    max_frames = spool_budget(spool, steps, size, max_frames, rip_hold,
                              who='make_movie')
    # THE BANDS AND PANELS (#1087): one implementation for make_movie and
    # make_film, planned BEFORE the frame so their regions are reserved --
    # the benchmark band, or the placement panels.
    import film_passes
    _bands = film_passes.plan(
        steps, final, attempts=attempts,
        attempts_ledger=attempts_ledger,
        placement={'off': placement_panel is False,
                   'asked': placement_panel,
                   'ledger': attempts_ledger,
                   'benchmark': benchmark_board,
                   'benchmark_score': benchmark_score,
                   'intent': floorplan_intent},
        quiet=quiet, who='make_movie')
    _lands = {}
    if _bands.ptrack is not None and marks is None:
        marks = []
    _band = _bands.band
    frames = a.build_boards(steps, final, size, supersample, layer_alpha,
                            rip_hold, chunks, stage=stage, marks=marks,
                            theme=theme, aspect=aspect,
                            geom_out=geom_out, title=_title,
                            frames_sink=spool, max_frames=max_frames,
                            attempts_band=_band,
                            lands_out=_lands, board3d=board3d, fps=fps)
    if not frames:
        if not quiet:
            print("make_movie: no frames (nothing routed?)", file=sys.stderr)
        return None
    _geom0 = geom_out[0] if geom_out else None
    # #1021. THE BANDS, before the clock: composition order is board ->
    # bands -> clock, so a band sits adjacent to the board it annotates.
    # Status lines print EVEN WHEN QUIET (bare, as they always did).
    frames = film_passes.compose(frames, _bands, _geom0, marks, _lands, theme,
                                 quiet=quiet, who='')

    frame_meta = None
    if ledger:
        try:
            import cmd_timing
            clock = cmd_timing.clock_for(marks, ledger, len(frames))
            if clock is not None:
                # The band height comes from EVERY frame's text, once, before
                # any frame is rebuilt: frames carry different numbers of
                # wrapped lines, and a per-frame band would make the frames
                # different sizes -- the one thing save_movie cannot take.
                all_lines = [clock.lines(i) for i in range(len(frames))]
                _f0 = frames[0]
                band = cmd_timing.clock_band_height(
                    all_lines, _f0.width, _f0.height)
                # #1036: a per-frame transform, applied while the encoder
                # streams -- it needs each frame's LINES, never another
                # frame's pixels.
                import frame_spool
                frames = frame_spool.transform(
                    frames,
                    lambda i, f: cmd_timing.add_clock_band(
                        f, all_lines[i], band, theme=theme),
                    out_size=None, optional='run clock',
                    ground=theme.rgb('chrome_band'), grow_px=band)
                frame_meta = [clock.meta(i) for i in range(len(frames))]
                if not quiet:
                    unmapped = clock.unmapped()
                    print('make_movie: run clock from %s (%d of %d beats '
                          'mapped%s)'
                          % (os.path.basename(ledger), len(clock.resolved),
                             len(clock.anchors),
                             '' if not unmapped
                             else '; no instant for ' + ', '.join(unmapped[:3])),
                          file=sys.stderr)
        except Exception as exc:                                # noqa: BLE001
            # A clock is decoration; it may never take the movie down.
            if not quiet:
                print('make_movie: no run clock (%s)' % exc, file=sys.stderr)
    if geom_out and not quiet:
        import frame_layout
        print(frame_layout.frame_status_line(geom_out[0]), file=sys.stderr)
    out = out or default_output(inputs)
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    try:
        if not a.save_movie(frames, out, fps=fps, end_hold=end_hold,
                            png_dir=png_dir, frame_meta=frame_meta,
                            theme=theme):
            return None
    finally:
        # the 3D board's state frames, once the film is written (#1081)
        try:
            from stage3d import film as _s3f
            _s3f.cleanup()
        except Exception:                                      # noqa: BLE001
            pass
    # save_movie falls back .mp4 -> .gif when imageio-ffmpeg is missing; report
    # the file that actually exists so callers (and the GUI) point at it.
    if out.lower().endswith('.mp4') and not os.path.exists(out):
        out = os.path.splitext(out)[0] + '.gif'
    return out


def board_snapshot(board_path, out=None, size=1600, quiet=False):
    """Still PNG of a board (the movie's last frame, at full resolution)."""
    from route_render import render_board_file
    out = out or (os.path.splitext(os.path.abspath(board_path))[0] + '.png')
    return render_board_file(board_path, out, size=size, quiet=quiet)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('inputs', nargs='+',
                    help='a run/output DIRECTORY, or step boards in chain order, '
                         'or a single board')
    ap.add_argument('-o', '--output', default=None,
                    help='output path; the extension picks the format (.mp4 default, '
                         '.gif for a dependency-free animated GIF). '
                         'Default: <rundir>/routing.mp4 or <last-board>_routing.mp4')
    ap.add_argument('--size', type=int, default=DEFAULT_SIZE,
                    help=f'longest frame dimension in px (default {DEFAULT_SIZE})')
    ap.add_argument('--fps', type=float, default=DEFAULT_FPS,
                    help=f'frames per second (default {DEFAULT_FPS:g})')
    ap.add_argument('--supersample', type=int, default=DEFAULT_SUPERSAMPLE,
                    help='anti-aliasing factor (1 = fastest, 2 = crisp)')
    ap.add_argument('--layer-alpha', type=int, default=DEFAULT_LAYER_ALPHA,
                    help='per-layer copper opacity 1-255 (<255 blends '
                         'crossings). Default: the theme\'s own measured '
                         'alpha (dark 150, light 205)')
    ap.add_argument('--rip-hold', type=int, default=DEFAULT_RIP_HOLD,
                    help='frames to hold ripped copper red before it vanishes')
    ap.add_argument('--chunks', type=int, default=DEFAULT_CHUNKS,
                    help='reveal batches for a step with no fine trace')
    ap.add_argument('--end-hold', type=float, default=DEFAULT_END_HOLD,
                    help='seconds to hold the final frame')
    ap.add_argument('--max-frames', type=int, default=None, metavar='N',
                    help='frame budget for the whole film (#1036). A route '
                         'trace that does not fit its share is revealed in '
                         '--chunks batches instead, and the movie says so. '
                         'Default: $KICAD_MOVIE_MAX_FRAMES or 2400; 0 = none')
    ap.add_argument('--png-dir', default=None,
                    help='also dump the raw PNG frames here')
    ap.add_argument('--png', action='store_true',
                    help='also write a full-resolution still of the final board')
    ap.add_argument('--aspect', default=None, metavar='W:H',
                    help="target frame aspect, or $KICAD_MOVIE_ASPECT. "
                         "Default 16:9, the stage3d frame's own (the only "
                         "film layout: the board, a layer column and one "
                         "band). Outside 1:2..3:1 the frame is board-only, "
                         "and says so")
    ap.add_argument('--no-attempts', action='store_true',
                    help="drop the benchmark band (#1021, #1081). The band "
                         "is drawn when loop_round*.json sidecars or a "
                         "converge ledger sit next to the boards; a chain "
                         "with no search behind it gets the placement "
                         "panels instead, and says so.")
    ap.add_argument('--attempts-ledger', default=None, metavar='PATH',
                    help='the converge ledger to draw the benchmark band '
                         'from, instead of looking beside the boards; the '
                         'placement panels read their laps and scores from '
                         'it too (#1042)')
    ap.add_argument('--benchmark-board', default=None, metavar='PATH',
                    help="a benchmark board (the human's, or a previous "
                         "run): drawn DASHED on the placement arrangement "
                         "panel, and the 100%% "
                         "line of the benchmark band, gold once a WORKING "
                         "board beats it on (vias, copper, segments)")
    ap.add_argument('--benchmark-score', default=None, metavar='PATH',
                    help="the benchmark board's `board_score --json` "
                         "document (must name that board by board_sha); "
                         "without it board_score is run once to grade it")
    ap.add_argument('--board-3d', default=None, choices=('auto', '2d', 'blender'),
                    help="'auto' (default) draws the 3D board "
                         "when Node, playwright-core (npm ci in "
                         "py_router/stage3d) and a Chromium are present, "
                         "else the 2D X-ray and says why; '2d' always the "
                         "X-ray; 'blender' the hi-fi backend: the same "
                         "scene in Blender's Cycles on the CPU "
                         "($KICAD_STAGE3D_BLENDER, #1089)")
    ap.add_argument('--floorplan-intent', default=None, metavar='PATH',
                    help='the floorplan intent to grade placement boards the '
                         'ledger does not name (check_floorplan --intent)')
    ap.add_argument('--no-placement-panel', action='store_true',
                    help='never draw the placement panels (#1042)')
    ap.add_argument('--title', default=None,
                    help="the film's name on the rail's left (default: the "
                         "run directory, or the directory the chain's boards "
                         "share; never a later board's name)")
    ap.add_argument('--theme', default=None, type=str.lower, choices=('dark', 'light'), help="'light' (default, or $KICAD_RENDER_THEME) or 'dark' (KiCad's own canvas). The file's ground cannot be changed afterwards.")
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--camera', default=None,
                    choices=('off', 'auto'),
                    help="placement camera: overview -> zoom to the "
                         "round's parts -> pan when work moves -> then "
                         "play the moves (#431). Needs loop_round*.json "
                         "sidecars in the work dir. Default: off, or "
                         "$KICAD_MOVIE_CAMERA")
    ap.add_argument('--camera-budget', type=float, default=60.0,
                    metavar='SECONDS',
                    help='cap the camera runtime (0 = unlimited)')
    ap.add_argument('--tween', type=int, default=10,
                    help='frames per placement glide; 0 = no glide, cut '
                         'straight to the new placement (default: 10)')
    clock = ap.add_argument_group(
        'run clock (#887)',
        'A run wrapped in tests/stress/tee_cmd.py leaves a cmd_timing.jsonl. '
        'When one is found beside the chain the movie draws a run-clock '
        'overlay and writes the same numbers into --png-dir frames. Nothing '
        'to turn on: no ledger, no clock.')
    clock.add_argument('--no-timing', dest='timing', action='store_const',
                       const='off', default=None,
                       help='never draw the run clock, even with a ledger')
    clock.add_argument('--timing-ledger', dest='timing', metavar='PATH',
                       help='use THIS cmd_timing.jsonl instead of searching')
    args = ap.parse_args()

    # A named ledger that is not there is an argparse error, not a fallback.
    # `make_movie` refuses it too; this is the spelling that gives the CLI a
    # clean "error: --timing-ledger: no such file" and exit 2 rather than the
    # library's exception. `--no-timing` sets the same dest to the sentinel
    # 'off', so exempt it.
    if (args.timing and args.timing != 'off'
            and not os.path.isfile(args.timing)):
        ap.error('--timing-ledger: no such file: %s' % args.timing)

    try:
        out = make_movie(args.inputs, out=args.output, theme=args.theme,
                         aspect=args.aspect,
                         size=args.size, fps=args.fps,
                         supersample=args.supersample, layer_alpha=args.layer_alpha,
                         rip_hold=args.rip_hold, chunks=args.chunks,
                         end_hold=args.end_hold, png_dir=args.png_dir,
                         max_frames=args.max_frames, title=args.title,
                         attempts_ledger=args.attempts_ledger,
                         benchmark_board=args.benchmark_board,
                         benchmark_score=args.benchmark_score,
                         board3d=args.board_3d,
                         floorplan_intent=args.floorplan_intent,
                         placement_panel=(False if args.no_placement_panel
                                          else None),
                         quiet=args.quiet,
                       camera=args.camera,
                       camera_budget=args.camera_budget,
                       tween=args.tween,
                       attempts=(False if args.no_attempts else None),
                       timing=args.timing)
    except FileNotFoundError as e:
        print(f"make_movie: no such board: {e}", file=sys.stderr)
        return 1
    except ImportError as e:
        print(f"make_movie: missing dependency ({e}). Pillow is required; "
              f"for .mp4 also: pip install imageio imageio-ffmpeg", file=sys.stderr)
        return 1
    if not out:
        return 1
    if args.png:
        _steps, final = resolve_inputs(args.inputs)
        if final:
            still = board_snapshot(final, quiet=args.quiet)
            if still and not args.quiet:
                print(f"make_movie: wrote {still}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
