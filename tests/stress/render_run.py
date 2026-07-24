#!/usr/bin/env python3
"""Stress-run rendering: final-board snapshot + whole-run routing movie (#482).

Replaces the old kicad-cli + headless-Chrome renderer (board_image.py /
board_layer_images.py) with the fast geometry renderer (route_render), and
adds an animated movie of the whole routing run (animate_route). Drops, next
to the final board / in the run dir:

  * ``<board>.png``            -- combined final snapshot (all copper layers)
  * ``<board>_<layer>.png``    -- one snapshot per copper layer
  * ``<rundir>/routing.gif``   -- whole-run movie: every stepN board's copper
                                  delta revealed in order, with fine rip/restore
                                  animation spliced in for steps that recorded a
                                  ``*_routetrace.json`` (KICAD_ROUTE_TRACE=1).

Geometry rendering is ~100x faster than the SVG->PNG path and needs no KiCad
or browser, so it is feasible to render a movie per board on every run.

Usage:
    python3 tests/stress/render_run.py RUNDIR [FINAL_BOARD]
      RUNDIR      stress run dir (holds stepN_*.kicad_pcb [+ *_routetrace.json])
      FINAL_BOARD final board to snapshot (default: the last step board)
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


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


def render_run_movie(run_dir, out_gif=None, size=1000, fps=6.0, quiet=False):
    """Whole-run movie from the run dir's step boards + traces. Returns path
    or None (e.g. no step boards found)."""
    import animate_route as a
    frames = a.build_run(run_dir, size=size, ss=1, alpha=150, rip_hold=2, chunks=6)
    if not frames:
        if not quiet:
            print(f"render_run: no stepN boards in {run_dir}; no movie")
        return None
    out = out_gif or os.path.join(run_dir, 'routing.gif')
    a.save_gif(frames, out, fps=fps, end_hold=1.5)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('final_board', nargs='?', default=None)
    ap.add_argument('--size', type=int, default=1600, help='snapshot px (default 1600)')
    ap.add_argument('--movie-size', type=int, default=1000)
    ap.add_argument('--fps', type=float, default=6.0)
    ap.add_argument('--no-movie', action='store_true')
    ap.add_argument('--no-snapshot', action='store_true')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    final = args.final_board
    if final is None:
        import animate_route as a
        _steps, final = a.discover_steps(args.run_dir)

    ok = False
    if final and os.path.exists(final) and not args.no_snapshot:
        try:
            paths = render_final_snapshot(final, size=args.size, quiet=args.quiet)
            ok = ok or bool(paths)
        except Exception as e:
            print(f"render_run: snapshot failed: {e}", file=sys.stderr)
    if not args.no_movie:
        try:
            mv = render_run_movie(args.run_dir, size=args.movie_size,
                                  fps=args.fps, quiet=args.quiet)
            ok = ok or bool(mv)
        except Exception as e:
            print(f"render_run: movie failed: {e}", file=sys.stderr)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
