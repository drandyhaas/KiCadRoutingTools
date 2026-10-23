#!/usr/bin/env python3
"""Stress-run rendering: the final-board snapshot and the routing movie (#482).

Replaces the old kicad-cli + headless-Chrome renderer (board_image.py /
board_layer_images.py) with the fast geometry renderer (route_render). Drops,
next to the final board / in the run dir:

  * ``<board>.png``            -- combined final snapshot (all copper layers)
  * ``<board>_<layer>.png``    -- one snapshot per copper layer
  * ``<rundir>/routing.mp4``   -- the run's routing movie (py_router/route_movie.py):
                                  each step's copper in the order it was laid,
                                  router event by router event where the step
                                  recorded a ``*_routetrace.json``
                                  (KICAD_ROUTE_TRACE=1). Only where ffmpeg is
                                  on PATH.

Geometry rendering is ~100x faster than the SVG->PNG path and needs no KiCad
or browser, so it is feasible on every run.

Usage:
    python3 tests/stress/render_run.py RUNDIR [FINAL_BOARD]
      RUNDIR      stress run dir (holds stepN_*.kicad_pcb [+ *_routetrace.json])
      FINAL_BOARD final board to snapshot (default: the last step board)
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _REPO)
sys.path.insert(0, os.path.join(_REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(_REPO, 'py_placer'))  # #522
sys.path.insert(0, os.path.join(_REPO, 'py_tools'))  # #522


def final_board_of(run_dir):
    """The run dir's final board: an explicit ``final*`` board when there is
    one, else the chain's last board, in route_movie's chain order (the order
    redo_commands.sh writes them; without one, step-numbered order, else
    write-time order). None when there is no board at all."""
    from route_movie import run_boards
    boards = run_boards(run_dir)
    final = boards[-1] if boards else None
    for b in boards:                      # an explicit final_board wins
        if 'final' in os.path.basename(b).lower():
            final = b
    return final


def render_final_snapshot(board_path, size=1600, per_layer=True, quiet=False):
    """Combined + per-layer PNG snapshots of the final board. Returns paths."""
    from kicad_parser import parse_kicad_pcb
    from route_render import BoardRenderer, render_board_file
    board_path = os.path.abspath(board_path)
    base = os.path.splitext(board_path)[0]
    written = []
    combined = render_board_file(board_path, base + '.png', size=size, quiet=quiet)
    if combined:
        written.append(combined)
    if per_layer:
        pcb = parse_kicad_pcb(board_path)
        for layer in pcb.board_info.copper_layers:
            try:
                r = BoardRenderer(pcb, size=size, layers=[layer])
                out = f'{base}_{layer}.png'
                r.render().save(out)
                written.append(out)
            except Exception as e:
                if not quiet:
                    print(f"render_run: per-layer {layer} failed ({e})")
    return written


def render_run_movie(run_dir, out=None, quiet=False):
    """The run dir's routing movie (`route_movie.make_movie`). Returns its path,
    or None when the dir holds no board or the machine has no `ffmpeg`: a GIF
    keeps every frame in memory, over 2 GB for a big chain, so the harness
    films only where it can stream an .mp4 (`route_movie.py` makes the GIF
    by hand)."""
    import shutil
    if not shutil.which('ffmpeg'):
        if not quiet:
            print("render_run: no ffmpeg on PATH; no routing movie")
        return None
    from route_movie import make_movie
    return make_movie([run_dir], out=out, quiet=quiet)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('final_board', nargs='?', default=None)
    ap.add_argument('--size', type=int, default=1600, help='snapshot px (default 1600)')
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--no-movie', action='store_true',
                    help='snapshots only: skip RUNDIR/routing.mp4')
    args = ap.parse_args()

    final = args.final_board or final_board_of(args.run_dir)
    if not final or not os.path.exists(final):
        print(f"render_run: no board to snapshot in {args.run_dir}", file=sys.stderr)
        return 1
    try:
        paths = render_final_snapshot(final, size=args.size, quiet=args.quiet)
    except Exception as e:
        print(f"render_run: snapshot failed: {e}", file=sys.stderr)
        return 1
    if not args.no_movie:
        try:
            render_run_movie(args.run_dir, quiet=args.quiet)
        except Exception as e:
            print(f"render_run: movie failed: {e}", file=sys.stderr)
    return 0 if paths else 1


if __name__ == '__main__':
    sys.exit(main())
