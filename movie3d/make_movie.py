#!/usr/bin/env python3
"""Make a 3D placement movie: place_simple's run, drawn on a 3D board, as an .mp4.

    python3 -X utf8 movie3d/make_movie.py BOARD OUT.kicad_pcb \\
        --groups groups.json --intent intent.json --seed 0 [--movie OUT_3d.mp4]
    python3 -X utf8 movie3d/make_movie.py BOARD OUT.kicad_pcb \\
        --groups groups.json --intent intent.json --route NAME:SET

BOARD and OUT come first; every argument but this tool's own is
place_simple.py's. Three steps, each printed with its time:

  1. record  (record.py)   place_simple runs in-process with read-only hooks
                           and writes scene.json + timeline.json to --work;
                           with --route BOARD:SET, place_movie's flow runs
                           first (its `run_flow`: the round's seeds placed,
                           the cheapest fast-routed, the best routed in full,
                           in --route-work, which a 2D flow run's work dir
                           can be: what is there is reused), the best seed's
                           placement is recorded, and its routes are played
                           on the 3D board (route3d.py) -- OUT is that seed;
  2. render  (render.mjs)  headless Chromium draws each frame with three.js
                           (page/movie3d.mjs) to --work/frames/f00000.png ...;
  3. encode  (ffmpeg)      the frames become the .mp4, and three stills are
                           copied beside it: <movie>_start.png (the input),
                           <movie>_anneal.png (mid-anneal), <movie>_final.png.

Needs `npm ci` in movie3d/ once (playwright-core + three, pinned), a Chromium
playwright-core can drive (its own cached headless shell, or --browser /
$MOVIE3D_CHROMIUM), and ffmpeg on PATH. Exit codes: place_simple's, or 2
when rendering or encoding fails.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import record  # noqa: E402

#: #937 registry: a prototype in its own directory, in no door; it writes
#: OUT (the placed board). Read by krt_registry.py by AST.
KRT_TOOL = {'scope': [], 'kind': 'actor'}


def flow(a, rest):
    """place_movie's flow (--route), filmed in 3D: run_flow places the
    round's seeds, fast-routes the judged ones and routes the best in full
    (reusing what --route-work holds); the best seed's placement is then
    recorded and its routes played on the 3D board. Returns what
    record.record does."""
    import place_movie
    from route3d import film_routes
    if ':' not in a.route:
        sys.exit('make_movie: --route NAME:SET, a stress-corpus board and its set number')
    board, out = rest[0], rest[1]
    work = os.path.abspath(a.route_work or os.path.splitext(os.path.abspath(out))[0] + '_work')
    opts = place_movie._strip(rest[2:], '--seed', '--json', '--seeds', '--jobs', '--clustered-seeds',
                              '--anchored-seeds')
    c0 = a.scattered + a.clustered
    n = c0 + a.anchored
    if min(a.scattered, a.clustered, a.anchored) < 0 or n < 1 or a.judge < 1:
        sys.exit('make_movie: --scattered, --clustered and --anchored of 0 or more, at least one seed, '
                 '--judge 1 or more')
    if a.clustered:
        opts += ['--clustered-seeds', f'{a.scattered}-{c0 - 1}']
    if a.anchored:
        opts += ['--anchored-seeds', f'{c0}-{n - 1}']
    fa = argparse.Namespace(route=a.route, fresh=a.fresh, scattered=a.scattered, clustered=a.clustered,
                            anchored=a.anchored, judge=a.judge, jobs=a.jobs, route_jobs=a.route_jobs)
    got = place_movie.run_flow(fa, board, out, opts, work)
    if got is None:
        return 2, None
    judged, scores, best, full = got
    # film a seed the judge did not pick: the cheapest by the objective, or
    # the next cheapest when that one is the best (judged runs cheapest first)
    filmed = next((s for s in judged if s in scores and s != best), best)
    print(f"make_movie: filming seed {filmed} (the objective's cheapest the judge did not pick); "
          f"the best is seed {best}")
    rc, rec = record.place(a, [board, os.path.join(work, f'film3d_s{filmed}.kicad_pcb'), *opts,
                               '--seed', str(filmed)])
    if rec.scene is None:
        return rc or 2, None
    rec.placed(rc, rec.hold)
    film_routes(rec, work, judged, scores, best, full, filmed, budget=a.route_frames)
    rec.turn_over(rec.copper_shot)
    subprocess.run([sys.executable, os.path.join(record.REPO, 'py_router', 'copy_board.py'),
                    os.path.join(work, f's{best}.kicad_pcb'), out], check=False, capture_output=True)
    print(f"make_movie: {out} = seed {best}; its full route in {os.path.join(work, f'full_s{best}', 'out')}")
    return rc, record.write(a, rec)


#: an installed Chrome that draws on the GPU through Metal (macOS)
GPU_CHROMES = ('/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
               '/Applications/Chromium.app/Contents/MacOS/Chromium')


def gpu_browser(a):
    """(browser executable or None, draw on the GPU?) for the render. The
    software renderer draws a 306-model frame in about 0.4 s; Metal on an M3,
    in 0.06 s. --gpu on with no installed Chrome falls back, and says so."""
    if a.gpu == 'off' or sys.platform != 'darwin':
        if a.gpu == 'on':
            print('make_movie: --gpu on needs macOS; drawing in software', file=sys.stderr)
        return a.browser, False
    exe = a.browser or next((p for p in GPU_CHROMES if os.path.isfile(p)), None)
    if exe is None:
        if a.gpu == 'on':
            print('make_movie: --gpu on found no installed Chrome; drawing in software',
                  file=sys.stderr)
        return a.browser, False
    return exe, True

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0], allow_abbrev=False,
                                 epilog="BOARD and OUT come first; every other argument is "
                                        "place_simple.py's (--groups, --intent, --seed, ...).")
    record.own_args(ap, work_required=False)
    from place_simple import ANCHORED_SEEDS, CLUSTERED_SEEDS, JUDGED_SEEDS, SCATTERED_SEEDS
    ap.add_argument('--route', metavar='BOARD:SET',
                    help="run and film place_movie's placement flow, routing with this "
                         "stress-corpus board's recorded chain (NAME:SET: the board and its set number)")
    ap.add_argument('--scattered', type=int, default=SCATTERED_SEEDS,
                    help=f'with --route, the seeds started scattered (default {SCATTERED_SEEDS})')
    ap.add_argument('--clustered', type=int, default=CLUSTERED_SEEDS,
                    help=f'with --route, the seeds started clustered (default {CLUSTERED_SEEDS})')
    ap.add_argument('--anchored', type=int, default=ANCHORED_SEEDS,
                    help=f'with --route, the seeds started anchored (default {ANCHORED_SEEDS}: opt-in)')
    ap.add_argument('--judge', type=int, default=JUDGED_SEEDS,
                    help=f'with --route, the cheapest legal seeds fast-routed (default {JUDGED_SEEDS})')
    ap.add_argument('--jobs', type=int, default=4, help='with --route, placements and routes at once (default 4)')
    ap.add_argument('--route-jobs', type=int, default=0, help='with --route, fast routes at once (default --jobs)')
    ap.add_argument('--fresh', action='store_true', help='with --route, empty --route-work first')
    ap.add_argument('--route-work', help="with --route, the flow's work dir (default: OUT's name + _work, "
                                         "place_movie's own default, so a 2D flow run's work is reused)")
    ap.add_argument('--route-frames', type=int, default=240,
                    help='with --route, past this many router events in a chain, several share a '
                         'frame (default 240; 0 = one frame each)')
    ap.add_argument('--movie', help="the movie to write (default: OUT's name + _3d.mp4)")
    ap.add_argument('--browser', default=os.environ.get('MOVIE3D_CHROMIUM'),
                    help='a Chromium executable (default: $MOVIE3D_CHROMIUM, else the headless '
                         'shell playwright-core finds in its own browser cache)')
    ap.add_argument('--gpu', choices=('auto', 'on', 'off'), default='auto',
                    help="draw the frames on the GPU, through an installed Chrome on Metal (macOS): "
                         "auto (default) when one is installed, off = the software renderer "
                         "(SwiftShader) in any case")
    ap.add_argument('--crf', type=int, default=20, help='x264 quality, lower is better (default 20)')
    a, rest = ap.parse_known_args(argv)
    if len(rest) < 2 or rest[0].startswith('-') or rest[1].startswith('-'):
        ap.error('BOARD and OUT come first')
    stem = os.path.splitext(os.path.abspath(rest[1]))[0]
    a.work = os.path.abspath(a.work or stem + '_movie3d')
    movie = os.path.abspath(a.movie or stem + '_3d.mp4')
    node = shutil.which('node')
    if not node:
        sys.exit('make_movie: no `node` on PATH')
    if not os.path.isdir(os.path.join(HERE, 'node_modules', 'playwright-core')):
        sys.exit(f'make_movie: run `npm ci` in {HERE} first')
    if not shutil.which('ffmpeg'):
        sys.exit('make_movie: no `ffmpeg` on PATH')

    # 1. record
    t0 = time.time()
    if a.route:
        rc, got = flow(a, rest)
    else:
        rc, got = record.record(a, rest)
    t_rec = time.time() - t0
    if not got:
        print('make_movie: place_simple did not get as far as a placement', file=sys.stderr)
        return rc or 2
    sp, tp, n = got
    print(f"make_movie: recorded {n} frames in {t_rec:.1f} s")

    # 2. render
    frames = os.path.join(a.work, 'frames')
    shutil.rmtree(frames, ignore_errors=True)
    with open(sp, encoding='utf-8') as f:
        glb = (json.load(f).get('glb') or {}).get('path')
    browser, gpu = gpu_browser(a)
    job = {'browser': browser, 'gpu': gpu, 'width': a.width, 'height': a.height, 'outDir': frames,
           'scene': sp, 'timeline': tp, 'glb': glb}
    jp = os.path.join(a.work, 'job.json')
    with open(jp, 'w', encoding='utf-8') as f:
        json.dump(job, f)
    t1 = time.time()
    proc = subprocess.Popen([node, os.path.join(HERE, 'render.mjs'), jp], cwd=HERE,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    done, err = None, None
    for line in proc.stdout:
        try:
            msg = json.loads(line)
        except ValueError:
            continue
        kind = msg.get('type')
        if kind == 'info':
            print(f"make_movie: rendering on {msg.get('renderer')} (Chromium {msg.get('browser')}, "
                  f"three r{msg.get('three')}); {msg.get('models', 0)} part models loaded"
                  + (f"; models found on the web that do not sit on their footprint, kept as "
                     f"boxes: {', '.join(msg['misfit'])}" if msg.get('misfit') else '')
                  + (f" -- MODELS FAILED: {msg['modelError']}" if msg.get('modelError') else ''))
        elif kind == 'progress':
            print(f"  {msg['done']}/{msg['of']} frames", flush=True)
        elif kind == 'done':
            done = msg
        elif kind == 'error':
            err = msg.get('why')
    proc.wait()
    t_ren = time.time() - t1
    if err or done is None or proc.returncode:
        tail = (proc.stderr.read() or '').strip().splitlines()[-3:]
        print(f"make_movie: the render failed: {err or ' | '.join(tail) or proc.returncode}",
              file=sys.stderr)
        return 2
    print(f"make_movie: rendered {done['frames']} frames in {t_ren:.1f} s "
          f"({done['ms_per_frame']:.0f} ms each)")

    # 3. encode, and the stills
    t2 = time.time()
    os.makedirs(os.path.dirname(movie), exist_ok=True)
    r = subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', f'{a.fps:g}',
                        '-i', os.path.join(frames, 'f%05d.png'), '-c:v', 'libx264',
                        '-pix_fmt', 'yuv420p', '-crf', str(a.crf), '-movflags', '+faststart', movie],
                       capture_output=True, text=True)
    if r.returncode or not os.path.isfile(movie):
        print(f"make_movie: ffmpeg failed: {r.stderr.strip()[:300]}", file=sys.stderr)
        return 2
    with open(tp, encoding='utf-8') as f:
        labels = [fr['label'] for fr in json.load(f)['frames']]
    anneal = [i for i, lab in enumerate(labels) if lab.startswith('anneal')]
    stills = {'start': 0, 'anneal': anneal[len(anneal) // 2] if anneal else len(labels) // 2}
    placed = [i for i, lab in enumerate(labels) if lab.startswith('placed: ')]
    if a.route and placed:
        stills['placed'] = placed[-1]
        fast = [i for i, lab in enumerate(labels) if lab.startswith('fast route, seed')]
        if fast:
            stills['fast_route'] = fast[0]
        seeds = [i for i, lab in enumerate(labels) if lab.startswith('the best: seed')]
        if seeds:
            stills['seeds'] = seeds[0]
    bottom = [i for i, lab in enumerate(labels) if lab == 'the bottom of the board']
    if bottom:
        stills['bottom'] = bottom[len(bottom) // 2]
    stills['final'] = len(labels) - 1
    base = os.path.splitext(movie)[0]
    for name, i in stills.items():
        shutil.copy(os.path.join(frames, f'f{i:05d}.png'), f'{base}_{name}.png')
    print(f"make_movie: {movie} ({n} frames, {n / a.fps:.1f} s, "
          f"{os.path.getsize(movie) / 1e6:.1f} MB) in {time.time() - t2:.1f} s; stills "
          + ', '.join(f'{base}_{k}.png' for k in stills))
    return rc


if __name__ == '__main__':
    sys.exit(main())
