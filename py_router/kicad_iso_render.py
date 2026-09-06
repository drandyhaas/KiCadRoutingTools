#!/usr/bin/env python3
"""``kicad-cli pcb render`` -- the 3D isometric view of a board (#887).

Everything in here shells out; nothing in here touches a pixel. That split is
deliberate: ``movie_panels`` owns the compositing and can be tested in full with
this module monkeypatched, so the planner, the frame map and the status line are
all green before a single subprocess runs.

    python3 py_router/kicad_iso_render.py board.kicad_pcb -o iso.png

Until now this recipe existed only as hand-typed text in one work dir
(``wk/run24/esp_prog/iso_render_cmd.txt``), with an absolute path to one
machine's KiCad. It resolves the binary through ``kicad_oracle.find_kicad_cli``
-- the repo's most complete resolver, and the ONE this must not fork.

**Read this before believing a render.** Three things were measured on KiCad
10.0.0 and each of them shapes the API:

* **The output size is not the requested size.** Asking 900x700 returned
  872x672; asking 640x480 returned 616x448 -- the same 616x448 for two very
  different boards and across an 8-step yaw sweep, so the delta is deterministic
  per REQUEST, but it is not the request. Every caller must letterbox into a box
  it chose itself. ``_REQUEST_OVERSCAN`` exists so that letterbox downscales
  (sharp) rather than upscales (blurry).
* **Cost is roughly board-independent, ~2-3.4 s.** tigard 2.4 s at 900x700;
  lvds 2.1 s, ulx3s (225 models) 2.0 s and glasgow_revC (224 models) 3.4 s at
  640x480. So a render per FRAME is out of the question and a render per chain
  STEP is affordable -- and eight of them run in 4.4 s over six workers rather
  than 11.9 s serially.
* **Component bodies are board-dependent, and their absence is silent.**
  ``kicad_files/tigard.kicad_pcb`` renders as a BARE BOARD -- pads, mask,
  silkscreen, no parts -- because its 84 ``(model ...)`` references are
  ``${KISYS3DMOD}/....wrl`` while KiCad 10 ships ``.step`` only, and passing
  ``-D KISYS3DMOD=<dir>`` does not fix it. ``kicad_files/lvds_converter_dualclk``
  renders with full bodies. kicad-cli says nothing either way, so
  ``resolve_models`` counts what is actually on disk and the caller captions the
  answer instead of shipping an empty green rectangle that reads as a bug.
"""
from __future__ import annotations

import os
import re
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

#: kicad-cli's own --help documents this triple as the isometric view.
ISO_ROTATE = (-45.0, 0.0, 45.0)

#: A HANG GUARD on one subprocess, not a budget. The distinction matters in this
#: repo: the cost cap for the iso panel is a COUNT of renders
#: (``movie_panels.IsoOpts.max_renders``), which is deterministic, and this is
#: only here so a wedged child cannot stall a movie forever. Same category as
#: ``kicad_oracle.ORACLE_DRC_TIMEOUT``.
ISO_RENDER_HANG_GUARD_S = 120.0

#: Ask for a little more than the box so the letterbox downscales. Renders cost
#: the same at either size (measured), so this is free sharpness.
_REQUEST_OVERSCAN = 1.15

#: Every path variable a board in this corpus actually uses. Measured over
#: kicad_files/: KISYS3DMOD (KiCad 5/6 era), KICAD6/7/8/9/10_3DMODEL_DIR,
#: KIPRJMOD (the board's own directory) and KICAD_USER_DIR.
_MODEL_DIR_VARS = ('KISYS3DMOD', 'KICAD6_3DMODEL_DIR', 'KICAD7_3DMODEL_DIR',
                   'KICAD8_3DMODEL_DIR', 'KICAD9_3DMODEL_DIR',
                   'KICAD10_3DMODEL_DIR')

_MODEL_RE = re.compile(r'\(model\s+"([^"]+)"')
_VAR_RE = re.compile(r'\$\{([A-Za-z0-9_]+)\}')
_UNRESOLVED = '\x00UNRESOLVED\x00'

#: Extensions KiCad will happily load for the same part, so a miss whose twin
#: exists under one of these is a STALE REFERENCE, not a missing install -- a
#: distinction worth drawing in the caption, because the two have different
#: fixes.
_MODEL_EXTS = ('.step', '.stp', '.wrl', '.wrz', '.igs', '.iges')


def resolve_cli(explicit=None):
    """``(path, reason)``. ``reason`` is ``''`` on success, else one sentence.

    ``explicit`` (a ``--kicad-cli`` argument) wins; then the shared resolver,
    which itself honours ``$KICAD_CLI`` before anything else.

    The shared resolver's own not-found warning is SUPPRESSED here, and that is
    not laziness: it says "every KiCad-oracle leg becomes a NO-OP ... check_drc's
    drc_real falls back to raw DRC", which is true and important for a DRC grade
    and simply false for a movie. Telling someone their DRC is compromised
    because they asked for a picture is worse than saying nothing. The reason
    string below is this module's own, and it names the same env var.
    """
    if explicit:
        if os.path.isfile(explicit):
            return explicit, ''
        return None, ('--kicad-cli %r is not a file' % explicit)
    try:
        from kicad_oracle import find_kicad_cli
    except Exception as exc:                                   # noqa: BLE001
        return None, ('could not load the kicad-cli resolver (%s)' % exc)
    try:
        cli = find_kicad_cli(warn=False)
    except TypeError:
        # An older kicad_oracle without the keyword: take the shout rather than
        # forking the resolver, which is the one thing this must not do.
        cli = find_kicad_cli()
    if cli:
        return cli, ''
    return None, ('kicad-cli not found (set $KICAD_CLI to it, or install KiCad)')


def kicad_share_dirs(cli_path):
    """Candidate ``.../3dmodels`` directories for the KiCad that owns ``cli_path``.

    Two layouts, because the two platforms differ and guessing one silently
    reports every model missing on the other:

    * Linux/Windows: ``<prefix>/bin/kicad-cli`` -> ``<prefix>/share/kicad/3dmodels``
    * macOS bundle:  ``.../Contents/MacOS/kicad-cli`` ->
      ``.../Contents/SharedSupport/3dmodels``
    """
    if not cli_path:
        return []
    bindir = os.path.dirname(os.path.abspath(cli_path))
    prefix = os.path.dirname(bindir)
    cands = [os.path.join(prefix, 'share', 'kicad', '3dmodels'),
             os.path.join(prefix, 'SharedSupport', '3dmodels'),
             os.path.join(prefix, '3dmodels')]
    return [c for c in cands if os.path.isdir(c)]


def model_dirs(cli_path=None, board_path=None):
    """What to substitute for each ``${...}`` in a board's model paths.

    An environment variable of the same name WINS -- that is what KiCad itself
    does, and it is how someone with a custom library tree makes this correct.
    Otherwise every versioned variable and the legacy ``KISYS3DMOD`` alias
    resolve to the install's own 3dmodels tree, and ``KIPRJMOD`` to the board's
    directory.

    ``AppData/kicad/<ver>/kicad_common.json`` is deliberately NOT consulted: on
    this machine its ``environment.vars`` is null, because KiCad supplies the
    versioned defaults internally rather than writing them out, so reading it
    would add a file dependency that answers nothing.
    """
    dirs = {}
    if board_path:
        dirs['KIPRJMOD'] = os.path.dirname(os.path.abspath(board_path))
    tree = (kicad_share_dirs(cli_path) or [None])[0]
    for var in _MODEL_DIR_VARS:
        env = os.environ.get(var)
        if env and os.path.isdir(env):
            dirs[var] = env
        elif tree:
            dirs[var] = tree
    user = os.environ.get('KICAD_USER_DIR')
    if user:
        dirs['KICAD_USER_DIR'] = user
    return dirs


def resolve_models(board_path, dirs=None):
    """How many of a board's 3D models actually exist on disk.

    Returns ``{'total', 'found', 'other_ext', 'unresolved_var', 'example'}``.

    Text-only and deliberately so: it regexes the RAW board rather than parsing
    it, so the count cannot move when ``kicad_parser`` changes, and it costs
    nothing on a 20 MB board.

    ``other_ext`` counts misses whose same-stem twin DOES exist under another
    extension KiCad accepts. That is the measured tigard case -- 84 references,
    0 found, 78 present as ``.step`` -- and it is a different problem from a
    missing install, so it gets a different number rather than being folded in.

    ``found > 0`` is the signal that a render will show component bodies, NOT
    ``found == total``: lvds_converter_dualclk resolves 10 of 15 and renders
    fully populated, because the 5 misses are project-local models for parts
    whose bodies barely matter.
    """
    out = {'total': 0, 'found': 0, 'other_ext': 0, 'unresolved_var': 0,
           'example': ''}
    try:
        txt = open(board_path, encoding='utf-8', errors='replace').read()
    except OSError:
        return out
    if dirs is None:
        dirs = model_dirs(board_path=board_path)
    for raw in _MODEL_RE.findall(txt):
        out['total'] += 1
        path = _VAR_RE.sub(lambda m: dirs.get(m.group(1), _UNRESOLVED), raw)
        if _UNRESOLVED in path:
            out['unresolved_var'] += 1
            if not out['example']:
                out['example'] = raw
            continue
        path = path.replace('\\', '/')
        if os.path.isfile(path):
            out['found'] += 1
            continue
        stem = os.path.splitext(path)[0]
        if any(os.path.isfile(stem + e) for e in _MODEL_EXTS):
            out['other_ext'] += 1
        if not out['example']:
            out['example'] = raw
    return out


def models_note(models):
    """One human clause for a caption. Never claims more than it counted."""
    if not models or not models.get('total'):
        return 'no 3D models referenced'
    t, f = models['total'], models['found']
    note = '3D models %d/%d' % (f, t)
    if f == 0:
        note += ' -- BARE BOARD, no component bodies'
        if models.get('other_ext'):
            note += ' (%d exist under another extension)' % models['other_ext']
        elif models.get('unresolved_var'):
            note += ' (%d paths use an undefined variable)' % models['unresolved_var']
    return note


def render_iso(board_path, out_png, cli, width, height, rotate=ISO_ROTATE,
               quality='basic', floor=False, perspective=False, zoom=None,
               timeout=ISO_RENDER_HANG_GUARD_S, define=None):
    """One ``kicad-cli pcb render``. ``(png_path, error)``; error ``''`` on success.

    The PNG keeps kicad-cli's default TRANSPARENT background on purpose: the
    caller alpha-composites it onto its own panel colour, whereas
    ``--background opaque`` would paste KiCad's light viewer background into a
    dark movie.

    argv is built as a LIST and never a string, and ``shell`` is never true:
    ``--rotate -45,0,45`` has to arrive as one argument, and on Windows a shell
    would be free to reinterpret it.
    """
    if not cli:
        return None, 'no kicad-cli'
    argv = [cli, 'pcb', 'render', '-o', out_png,
            '--width', str(int(round(width))),
            '--height', str(int(round(height))),
            '--rotate', '%g,%g,%g' % tuple(rotate),
            '--quality', quality]
    if floor:
        argv.append('--floor')
    if perspective:
        argv.append('--perspective')
    if zoom:
        argv += ['--zoom', '%g' % zoom]
    for k, v in sorted((define or {}).items()):
        argv += ['-D', '%s=%s' % (k, v)]
    argv.append(board_path)
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None, 'kicad-cli pcb render timed out after %gs' % timeout
    except OSError as exc:
        return None, 'could not run kicad-cli (%s)' % exc
    if r.returncode != 0 or not os.path.isfile(out_png):
        # The FIRST line, not the last 160 characters. A failing kicad-cli says
        # what went wrong up front and then prints usage, so slicing from the
        # END delivered the middle of a Windows path with the reason cut off:
        #   "exited 2: l\\Temp\\claude\\...\\scratchpad\\pcb': [Errno 2] ..."
        blob = (r.stderr or r.stdout or '').strip()
        head = blob.splitlines()[0].strip() if blob else ''
        return None, 'kicad-cli pcb render exited %s%s' % (
            r.returncode, (': ' + head[:160]) if head else '')
    return out_png, ''


def render_many(jobs, cli, workers=None, **kw):
    """``{key: (png|None, error)}`` for ``jobs = [(key, board, out_png, rotate)]``.

    Threads, not processes: the unit of work is a subprocess, so the GIL is
    released and there is nothing to pickle. Capped at 4 because kicad-cli is
    itself multithreaded and oversubscribing past that stopped buying anything
    (measured: 8 renders in 11.9 s serial, 4.4 s over six workers).

    Results are keyed, never appended in completion order, so the composed movie
    is byte-identical at any worker count. A test asserts exactly that, and also
    that more than one thread really ran -- otherwise the determinism claim
    could be satisfied by silently serialising.
    """
    from concurrent.futures import ThreadPoolExecutor
    out = {}
    if not jobs:
        return out
    n = workers if workers else min(4, os.cpu_count() or 1, len(jobs))
    n = max(1, int(n))

    def one(job):
        key, board, png, rotate = job
        return key, render_iso(board, png, cli, rotate=rotate, **kw)

    if n == 1:
        for job in jobs:
            k, v = one(job)
            out[k] = v
        return out
    with ThreadPoolExecutor(max_workers=n) as ex:
        for k, v in ex.map(one, jobs):
            out[k] = v
    return out


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board')
    ap.add_argument('-o', '--output', default=None,
                    help='PNG path (default: <board>_iso.png)')
    ap.add_argument('--width', type=int, default=1600)
    ap.add_argument('--height', type=int, default=900)
    ap.add_argument('--rotate', default='%g,%g,%g' % ISO_ROTATE,
                    help="X,Y,Z degrees (default the isometric %s)"
                         % (','.join('%g' % v for v in ISO_ROTATE)))
    ap.add_argument('--quality', default='basic',
                    choices=('basic', 'high', 'user', 'job_settings'),
                    help='basic ~2-3 s; high measured ~12.6 s (default: basic)')
    ap.add_argument('--floor', action='store_true', help='shadows')
    ap.add_argument('--perspective', action='store_true')
    ap.add_argument('--zoom', type=float, default=None)
    ap.add_argument('--kicad-cli', default=None)
    args = ap.parse_args(argv)

    cli, why = resolve_cli(args.kicad_cli)
    if not cli:
        print('render_iso: %s' % why, file=sys.stderr)
        return 2
    out = args.output or (os.path.splitext(os.path.abspath(args.board))[0]
                          + '_iso.png')
    try:
        rot = tuple(float(x) for x in args.rotate.split(','))
        if len(rot) != 3:
            raise ValueError
    except ValueError:
        print("render_iso: --rotate wants three numbers, 'X,Y,Z'", file=sys.stderr)
        return 2
    models = resolve_models(args.board, model_dirs(cli, args.board))
    png, err = render_iso(args.board, out, cli, args.width, args.height,
                          rotate=rot, quality=args.quality, floor=args.floor,
                          perspective=args.perspective, zoom=args.zoom)
    if not png:
        print('render_iso: %s' % err, file=sys.stderr)
        return 1
    print('render_iso: wrote %s (%s)' % (png, models_note(models)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
