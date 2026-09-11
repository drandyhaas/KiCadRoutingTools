#!/usr/bin/env python3
"""Run a GUI routing plan on a board from the command line, headless (#507).

Takes the plan JSON the AI tab saves (or that ``make_plan.py`` converts from
a recorded run) and executes it against a board through the REAL plugin dialog
and plan executor -- no window, no clicks, no AI:

    python3 run_plan.py board.kicad_pcb plan.json -o routed.kicad_pcb

    python3 run_plan.py board.kicad_pcb plan.json --list      # just show the steps
    python3 run_plan.py board.kicad_pcb plan.json --steps 1,3-5
    python3 run_plan.py board.kicad_pcb plan.json --movie routing.mp4

The input board is never modified: it is copied (with its ``.kicad_pro``, which
carries the DRC floor -- #441) to the output path, and the plan runs on that.

``--movie`` snapshots the board after every step and renders the whole run into
a movie with ``make_movie.py``.

Needs KiCad's bundled python (pcbnew + wx); this script re-execs into it
automatically, so plain ``python3 run_plan.py ...`` works.
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'actor'}

import argparse
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import headless_plan


def parse_step_selection(spec, count):
    """'1,3-5' -> [0, 2, 3, 4] (1-based inclusive ranges -> 0-based indices)."""
    if not spec:
        return list(range(count))
    out = []
    for part in spec.replace(' ', '').split(','):
        if not part:
            continue
        if '-' in part:
            a, b = part.split('-', 1)
            out.extend(range(int(a) - 1, int(b)))
        else:
            out.append(int(part) - 1)
    bad = [i + 1 for i in out if not 0 <= i < count]
    if bad:
        raise ValueError(f"step(s) out of range 1-{count}: "
                         f"{', '.join(str(b) for b in bad)}")
    seen, uniq = set(), []
    for i in out:
        if i not in seen:
            seen.add(i)
            uniq.append(i)
    return uniq


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board', help='input .kicad_pcb (never modified)')
    ap.add_argument('plan', help='plan JSON (AI tab Save..., or make_plan.py)')
    ap.add_argument('-o', '--output', default=None,
                    help='output board (default: <board>_routed.kicad_pcb)')
    ap.add_argument('--steps', default=None,
                    help="which steps to run, 1-based, e.g. '1,3-5' (default: all)")
    ap.add_argument('--list', action='store_true',
                    help='list the plan steps and exit (nothing is routed)')
    ap.add_argument('--snapshot-dir', default=None,
                    help='save the board after each step here (a movie/debug chain)')
    ap.add_argument('--movie', nargs='?', const='', default=None,
                    metavar='OUT.mp4',
                    help='also render a routing movie of the run (default name: '
                         '<output>_routing.mp4); implies per-step snapshots')
    ap.add_argument('--timeout', type=float, default=7200,
                    help='seconds before the run is abandoned (default 7200)')
    args = ap.parse_args()

    if not os.path.isfile(args.board):
        print(f"run_plan: no such board: {args.board}", file=sys.stderr)
        return 1
    if not os.path.isfile(args.plan):
        print(f"run_plan: no such plan: {args.plan}", file=sys.stderr)
        return 1

    # The plugin (and the plan parser) need pcbnew + wx: hop into KiCad's python.
    if not headless_plan.have_kicad_python():
        headless_plan.reexec_into_kicad(__file__)
        print("run_plan: no python with pcbnew + wx found (KiCad's bundled "
              "python). Run this with KiCad's interpreter.", file=sys.stderr)
        return 3

    steps, messages = headless_plan.load_plan(args.plan)
    for m in messages:
        print(f"plan: {m}")
    if not steps:
        print(f"run_plan: {args.plan} holds no usable steps", file=sys.stderr)
        return 1

    labels = headless_plan.step_labels(steps)
    try:
        indices = parse_step_selection(args.steps, len(steps))
    except ValueError as e:
        print(f"run_plan: {e}", file=sys.stderr)
        return 1

    if args.list:
        for i, label in enumerate(labels):
            # step_label already numbers the step; '-' marks one --steps skips.
            print(f"{' ' if i in indices else '-'} {label}")
        return 0

    out = args.output or (os.path.splitext(os.path.abspath(args.board))[0]
                          + '_routed.kicad_pcb')
    out = os.path.abspath(out)
    if os.path.abspath(args.board) == out:
        print("run_plan: --output must differ from the input board", file=sys.stderr)
        return 1
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)

    from copy_board import copy_board       # carries the .kicad_pro DRC floor
    copy_board(os.path.abspath(args.board), out)

    snapshot_dir = args.snapshot_dir
    temp_snapshots = None
    if args.movie is not None and not snapshot_dir:
        temp_snapshots = tempfile.mkdtemp(prefix='kicadrt_movie_')
        snapshot_dir = temp_snapshots

    print(f"run_plan: {len(indices)} of {len(steps)} step(s) on "
          f"{os.path.basename(out)}")

    def on_status(index, status):
        if status == 'running':
            print(f"  step {index + 1}/{len(steps)}  {labels[index]} ...",
                  flush=True)
        else:
            print(f"  step {index + 1}/{len(steps)}  {status}", flush=True)

    result = headless_plan.run_plan(out, steps, indices=indices,
                                    snapshot_dir=snapshot_dir,
                                    timeout=args.timeout, on_status=on_status)

    for line in result['log']:
        print(f"  log: {line}")
    print(f"run_plan: {result['completed']} step(s) completed in "
          f"{result['seconds']}s -> {out}")
    if result['aborted']:
        print(f"run_plan: stopped early: {result['aborted']}", file=sys.stderr)
    if result['timed_out']:
        print(f"run_plan: TIMED OUT after {args.timeout}s", file=sys.stderr)

    if args.movie is not None:
        movie_out = args.movie or (os.path.splitext(out)[0] + '_routing.mp4')
        try:
            from make_movie import make_movie
            # The snapshots are the step chain; the output board is the end state.
            written = make_movie(result['snapshots'] + [out], out=movie_out)
            if written:
                print(f"run_plan: movie -> {written}")
            if (written and movie_out.lower().endswith('.mp4')
                    and written.lower().endswith('.gif')):
                print("run_plan: (.mp4 encoding needs imageio + imageio-ffmpeg; "
                      "wrote a GIF instead)")
        except Exception as e:
            print(f"run_plan: movie failed ({e})", file=sys.stderr)

    if temp_snapshots:
        import shutil
        shutil.rmtree(temp_snapshots, ignore_errors=True)

    if result['timed_out'] or result['aborted']:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
