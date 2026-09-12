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
DEFAULT_LAYER_ALPHA = 150
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


def default_output(inputs):
    """Where the movie lands when no -o is given: inside a run dir, else next to
    the last board."""
    if len(inputs) == 1 and os.path.isdir(inputs[0]):
        return os.path.join(os.path.abspath(inputs[0]), 'routing.mp4')
    return os.path.splitext(os.path.abspath(inputs[-1]))[0] + '_routing.mp4'


#: What `panels` accepts. 'xray' is the single full-frame board view this movie
#: has always been; 'xray+iso' stacks a 3D isometric render under it (#887).
PANEL_SETS = ('xray', 'xray+iso')


def _panels_wanted(panels, quiet=False):
    """True when the iso panel was asked for. Handles the env knob and typos.

    Deliberately ASYMMETRIC, and both halves are audible:

    * an unknown value passed as the KWARG raises, naming the accepted set -- a
      typo in code is a bug, and silently rendering the wrong movie hides it;
    * an unknown value in the ENV KNOB warns and falls back to 'xray' -- a typo
      in a shell must not abort a routing run that happened to ask for a movie.

    This is stricter than the #431 camera, which turns ON for any unrecognised
    string (`make_movie.py:140`). Do not copy that here: 'xray' is the safe
    default and an unreadable value must not silently buy 20 seconds of
    kicad-cli.
    """
    if panels is None:
        try:
            import env_knobs
            raw = getattr(env_knobs, 'MOVIE_PANELS', 'xray')
        except Exception:                                       # noqa: BLE001
            raw = 'xray'
        val = str(raw or 'xray').strip().lower()
        if val not in PANEL_SETS:
            if not quiet:
                print("make_movie: KICAD_MOVIE_PANELS=%r is not one of %s; "
                      "using 'xray'" % (raw, ', '.join(PANEL_SETS)),
                      file=sys.stderr)
            val = 'xray'
        return val == 'xray+iso'
    val = str(panels).strip().lower()
    if val not in PANEL_SETS:
        raise ValueError('make_movie: panels=%r is not one of %s'
                         % (panels, ', '.join(PANEL_SETS)))
    return val == 'xray+iso'


def make_movie(inputs, out=None, size=DEFAULT_SIZE, fps=DEFAULT_FPS,
               supersample=DEFAULT_SUPERSAMPLE, layer_alpha=DEFAULT_LAYER_ALPHA,
               rip_hold=DEFAULT_RIP_HOLD, chunks=DEFAULT_CHUNKS,
               end_hold=DEFAULT_END_HOLD, png_dir=None, quiet=False,
               camera=None, camera_budget=60.0, tween=10,
               panels=None, iso_opts=None, timing=None):
    """Render the movie. ``inputs`` is a run dir (one entry) or a board sequence.

    Returns the path actually written -- which is a sibling ``.gif`` when an
    ``.mp4`` was asked for and imageio-ffmpeg is unavailable -- or None when
    there was nothing to animate.
    """
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
    # #431: the camera is OPT-IN. camera=None falls back to the env knob, so
    # one variable turns it on for the GUI recorder, run_plan.py --movie and the
    # stress renderer at once -- and OFF is the default everywhere, because
    # every GUI movie is a routing movie and changing those is pure regression
    # risk for no user-visible win.
    stage = None
    if camera is None:
        try:
            import env_knobs
            camera = getattr(env_knobs, 'MOVIE_CAMERA', 'off')
        except Exception:
            camera = 'off'
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
                rounds = synth_rounds([s[1] for s in steps])
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
    # #887: the second panel is OPT-IN, exactly like the camera above, and for
    # the same reason -- one variable turns it on for the GUI recorder,
    # run_plan.py --movie and the stress renderer at once. `marks` is asked for
    # ONLY when the panel is on, and `marks=None` is build_boards' own default,
    # so the fast path below is not merely equivalent to what it was: it is the
    # same call.
    # #887: the run clock is PRESENCE-GATED, not a mode. cmd_timing.jsonl
    # exists only in a teed stress-run work dir, so every existing movie --
    # the GUI recorder's, place_route_loop's, render_run's, and every
    # board-sequence invocation outside such a run -- is untouched. That is a
    # far narrower trigger than the #431 camera's, which is why the camera is
    # opt-in and this is not.
    want_iso = _panels_wanted(panels, quiet)
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
    marks = [] if (want_iso or ledger) else None
    frames = a.build_boards(steps, final, size, supersample, layer_alpha,
                            rip_hold, chunks, stage=stage, marks=marks)
    if not frames:
        if not quiet:
            print("make_movie: no frames (nothing routed?)", file=sys.stderr)
        return None
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
                band = cmd_timing.clock_band_height(
                    all_lines, frames[0].width, frames[0].height)
                # In place, so peak memory stays ~2 frames rather than 2x the
                # movie: each original is released as its replacement lands.
                for i, ln in enumerate(all_lines):
                    frames[i] = cmd_timing.add_clock_band(frames[i], ln, band)
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
    if want_iso:
        # Imported HERE, not at module scope: make_movie is imported in-process
        # by the GUI recorder, run_plan.py, place_route_loop.py and
        # render_run.py, and none of them should pay for a feature they did not
        # ask for. Bound through the module rather than `from ... import`, so a
        # test that monkeypatches movie_panels.compose_two_panel still bites.
        import movie_panels
        frames, report = movie_panels.compose_two_panel(
            frames, marks, final, iso_opts)
        # PRINTED EVEN WHEN QUIET. `quiet` silences the ordinary progress
        # chatter, but this line is the only channel that says whether the
        # panel ran, was skipped, or failed -- and the one front end the env
        # knob exists for, the GUI recorder, calls make_movie with quiet=True
        # (movie_recorder.py:160). Suppressing it there meant a user could turn
        # the panel on, pay 15 s of kicad-cli, and be told nothing at all.
        # The panel is opt-in, so this line only ever appears when it was asked
        # for.
        print(movie_panels.iso_status_line(report), file=sys.stderr)
    out = out or default_output(inputs)
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    if not a.save_movie(frames, out, fps=fps, end_hold=end_hold,
                        png_dir=png_dir, frame_meta=frame_meta):
        return None
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
                    help='per-layer copper opacity 1-255 (<255 blends crossings)')
    ap.add_argument('--rip-hold', type=int, default=DEFAULT_RIP_HOLD,
                    help='frames to hold ripped copper red before it vanishes')
    ap.add_argument('--chunks', type=int, default=DEFAULT_CHUNKS,
                    help='reveal batches for a step with no fine trace')
    ap.add_argument('--end-hold', type=float, default=DEFAULT_END_HOLD,
                    help='seconds to hold the final frame')
    ap.add_argument('--png-dir', default=None,
                    help='also dump the raw PNG frames here')
    ap.add_argument('--png', action='store_true',
                    help='also write a full-resolution still of the final board')
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
    iso = ap.add_argument_group(
        '3D isometric panel (#887)',
        'Stack a kicad-cli 3D render UNDER the X-ray board view. OFF by '
        'default: a render costs ~2-4 s against a whole movie of about a '
        'second, and this subsystem exists precisely because kicad-cli was '
        'dropped from it (#482). Every flag here is inert under --panels xray.')
    iso.add_argument('--panels', default=None, choices=PANEL_SETS,
                     help="'xray' (default, or $KICAD_MOVIE_PANELS) or "
                          "'xray+iso'")
    iso.add_argument('--iso-max-renders', type=int, default=24, metavar='N',
                     help='THE cost cap, as a COUNT of renders rather than a '
                          'number of seconds, so the same chain composes the '
                          'same movie on a fast machine and a slow one. '
                          '0 disables the panel even with --panels xray+iso. '
                          '(default: 24, about 20 s over 4 workers: 6 waves of 4)')
    iso.add_argument('--iso-height-frac', type=float, default=0.62,
                     metavar='F',
                     help='iso panel height as a fraction of the X-ray panel '
                          '(default: 0.62)')
    iso.add_argument('--iso-sweep', type=float, default=60.0, metavar='DEG',
                     help='total yaw travelled across the whole film -- what '
                          'makes the bottom panel animated rather than a '
                          'still, at no extra cost (default: 60)')
    iso.add_argument('--iso-quality', default='basic',
                     choices=('basic', 'high', 'user', 'job_settings'),
                     help='basic measured 1.4-2.7 s serial and 1.9-4.2 s '
                          'four at once; high measured 5.0-7.5 s on the '
                          'same four boards, about 3x (default: basic)')
    iso.add_argument('--iso-floor', action='store_true',
                     help='kicad-cli --floor: shadows and post-processing')
    iso.add_argument('--iso-perspective', action='store_true',
                     help='perspective instead of orthographic. Off by '
                          'default: orthographic keeps the board the same '
                          'apparent size across the yaw sweep')
    iso.add_argument('--iso-zoom', type=float, default=None)
    iso.add_argument('--iso-jobs', type=int, default=None, metavar='N',
                     help='parallel renders (default: min(4, cpu count)). The '
                          'movie is identical at any value. (At '
                          '--iso-quality high kicad-cli is not reproducible '
                          'against itself run to run, on any worker count; '
                          'basic is.)')
    iso.add_argument('--iso-timeout', type=float, default=120.0,
                     metavar='SECONDS',
                     help='HANG GUARD on ONE render -- not a budget, and it '
                          'trims no content. The cost cap is '
                          '--iso-max-renders (default: 120)')
    iso.add_argument('--kicad-cli', default=None, metavar='PATH',
                     help='explicit binary; else $KICAD_CLI, else the shared '
                          'resolver')
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

    # UNCONDITIONALLY. Gating this on `args.panels == 'xray+iso'` dropped every
    # --iso-* flag whenever the panel was turned on by KICAD_MOVIE_PANELS
    # instead of by the flag: the panel ran at full defaults and nine options
    # were discarded in silence. It looked correct in the obvious test, because
    # `--panels xray+iso --iso-quality high` does set both. The comparison
    # against the default 24 had the same shape -- typing the default
    # explicitly was indistinguishable from not typing it.
    #
    # Building it always costs nothing: IsoOpts is inert under `--panels xray`,
    # where compose_two_panel is never called at all.
    import movie_panels
    iso_opts = movie_panels.IsoOpts(
        max_renders=args.iso_max_renders,
        height_frac=args.iso_height_frac, sweep_deg=args.iso_sweep,
        quality=args.iso_quality, floor=args.iso_floor,
        perspective=args.iso_perspective, zoom=args.iso_zoom,
        jobs=args.iso_jobs, timeout=args.iso_timeout,
        cli=args.kicad_cli)

    try:
        out = make_movie(args.inputs, out=args.output, size=args.size, fps=args.fps,
                         supersample=args.supersample, layer_alpha=args.layer_alpha,
                         rip_hold=args.rip_hold, chunks=args.chunks,
                         end_hold=args.end_hold, png_dir=args.png_dir,
                         quiet=args.quiet,
                       camera=args.camera,
                       camera_budget=args.camera_budget,
                       tween=args.tween,
                       panels=args.panels, iso_opts=iso_opts,
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
